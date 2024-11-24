#![no_std]
#![no_main]

use embassy_sync::blocking_mutex::raw::{CriticalSectionRawMutex, NoopRawMutex};
use embassy_sync::blocking_mutex::NoopMutex;
use embassy_sync::channel::{Channel, Receiver, Sender, TryReceiveError};
use embedded_dht_rs::dht22::Dht22;
use embedded_dht_rs::SensorReading;
use esp_backtrace as _;
use esp_hal::delay::Delay;
use esp_hal::i2c::master::{Config, I2c};
use esp_hal::Async;
use esp_hal::{gpio::Pull, prelude::*};
use esp_hal::gpio::{Level, OutputOpenDrain};
use esp_storage::FlashStorage;
use log::{debug, error, info};
use embedded_storage::{ReadStorage, Storage};

use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use sgp30::{Baseline, Humidity, Measurement, Sgp30, Sgp30Async};

extern crate alloc;

static HUMIDITY_CHANNEL: Channel<CriticalSectionRawMutex, Humidity, 2> = Channel::new();
static BASELINE_CHANNEL: Channel<CriticalSectionRawMutex, Baseline, 2> = Channel::new();

const SATURATION_PRESSURE_TABLE: [u16; 36] = [
    610, 650, 700, 750, 810, 870, 940, 1010, 1080, 1160,  // 0-9°C
    1250, 1340, 1440, 1550, 1670, 1790, 1920, 2060, 2210, 2370,  // 10-19°C
    2540, 2720, 2920, 3130, 3350, 3590, 3840, 4110, 4400, 4700,  // 20-29°C
    5030, 5370, 5740, 6130, 6540, 6980  // 30-35°C
];

#[main]
async fn main(spawner: Spawner) {
    let peripherals = esp_hal::init({
        let mut config = esp_hal::Config::default();
        config.cpu_clock = CpuClock::max();
        config
    });
    let delay = Delay::new();

    esp_alloc::heap_allocator!(72 * 1024);

    esp_println::logger::init_logger_from_env();

    let timer0 = esp_hal::timer::timg::TimerGroup::new(peripherals.TIMG1);
    esp_hal_embassy::init(timer0.timer0);

    info!("Embassy initialized!");

    let timer1 = esp_hal::timer::timg::TimerGroup::new(peripherals.TIMG0);
    let _init = esp_wifi::init(
        timer1.timer0,
        esp_hal::rng::Rng::new(peripherals.RNG),
        peripherals.RADIO_CLK,
    )
    .unwrap();

    let _ = spawner;
    let dht22_pin = OutputOpenDrain::new(peripherals.GPIO2, Level::High, Pull::None);
    let mut dht22 = Dht22::new(dht22_pin, delay);
    debug!("Entering loop");

    let config = Config::default();
    let i2c = peripherals.I2C0;
    let sda = peripherals.GPIO21;
    let scl = peripherals.GPIO22;
    let i2c = I2c::new(i2c, config)
        .with_scl(scl)
        .with_sda(sda)
        .into_async();

    let mut sgp30 = Sgp30Async::new(i2c, 0x58, embassy_time::Delay);
    sgp30.init().await.expect("Failed to initialize SGP30");

    // Restore old baseline from flash
    let mut flash = FlashStorage::default();
    let mut bytes = [0u8; 6];
    flash.read(0x90000, &mut bytes).expect("Failed to read from flash");
    let baseline = Baseline {
        co2eq: (u16::from(bytes[0]) << 8) | u16::from(bytes[1]),
        tvoc: (u16::from(bytes[3]) << 8) | u16::from(bytes[4]),
    };
    sgp30.set_baseline(&baseline).await.expect("Failed to set baseline");
    info!("Restored baseline: {:?}", baseline);

    spawner.spawn(read_sgp30(sgp30, HUMIDITY_CHANNEL.receiver(), BASELINE_CHANNEL.sender())).unwrap();
    spawner.spawn(persist_sgp30_baseline(BASELINE_CHANNEL.receiver())).unwrap();

    loop {
        Timer::after(Duration::from_secs(5)).await;
        info!("Hello world!");

        match dht22.read() {
            Ok(sensor_reading) => {
                info!("Temperature: {:?}°C, Humidity: {:?} %", sensor_reading.temperature, sensor_reading.humidity);
                let humidity = absolute_humidity(sensor_reading.temperature, sensor_reading.humidity);
                info!("Absolute humidity: {:?} g/m³", humidity);
                let humidity_u8 = (humidity.max(1.0) as u8).clamp(1, 255);
                HUMIDITY_CHANNEL.send(Humidity::new(humidity_u8, 0).unwrap()).await;
            },
            Err(e) => {
                error!("Failed to read DHT22: {:?}", e);
                // Optional: Add a small delay to let the sensor recover
                Timer::after(Duration::from_millis(500)).await;
            }
        }
    }
}

#[embassy_executor::task]
async fn read_sgp30(mut sgp30: Sgp30Async<I2c<'static, Async>, embassy_time::Delay>, receiver: Receiver<'static, CriticalSectionRawMutex, Humidity, 2>, sender: Sender<'static, CriticalSectionRawMutex, Baseline, 2>) {
    const INTERVAL: Duration = Duration::from_secs(1);
    const BASELINE_INTERVAL: Duration = Duration::from_secs(60);

    let mut last_baseline_reading = embassy_time::Instant::now();

    loop {
        let start = embassy_time::Instant::now();
        match receiver.try_receive() {
            Ok(humidity) => {
                sgp30.set_humidity(Some(&humidity)).await.expect("Failed to set humidity");
            }
            Err(TryReceiveError::Empty) => {
                debug!("Channel empty");
            }
        }
        let measurement: Measurement = sgp30.measure().await.unwrap();
        info!("CO₂eq parts per million: {}", measurement.co2eq_ppm);
        info!("TVOC parts per billion: {}", measurement.tvoc_ppb);
        
        if last_baseline_reading.elapsed() > BASELINE_INTERVAL {
            let baseline = sgp30.get_baseline().await.unwrap();
            sender.send(baseline).await;
            last_baseline_reading = embassy_time::Instant::now();
        }

        let elapsed = start.elapsed();
        if elapsed < INTERVAL {
            Timer::after(INTERVAL - elapsed).await;
        }
    }
}

#[embassy_executor::task]
async fn persist_sgp30_baseline(receiver: Receiver<'static, CriticalSectionRawMutex, Baseline, 2>) {
    let mut flash = FlashStorage::default();

    loop {
        let baseline = receiver.receive().await;
        info!("Saving Baseline: {:?}", baseline);

        let bytes = baseline_to_bytes(baseline);

        let co2eq_ppm = (u16::from(bytes[0]) << 8) | u16::from(bytes[1]);
        let tvoc_ppb = (u16::from(bytes[3]) << 8) | u16::from(bytes[4]);
        let check_baseline = Measurement {
            co2eq_ppm,
            tvoc_ppb,
        };
        debug!("Check baseline: {:?}", check_baseline);
        flash.write(0x90000, &bytes).expect("Failed to write to flash");
    }
}

fn baseline_to_bytes(baseline: Baseline) -> [u8; 6] {
    let mut bytes = [0u8; 6];
    bytes[0] = (baseline.co2eq >> 8) as u8;
    bytes[1] = baseline.co2eq as u8;
    bytes[2] = 0; // CRC byte (unused)
    bytes[3] = (baseline.tvoc >> 8) as u8;
    bytes[4] = baseline.tvoc as u8;
    bytes[5] = 0; // CRC byte (unused)
    bytes
}

fn absolute_humidity(temp_c: f32, rel_humidity: f32) -> f32 {
    // Clamp temperature to 0-35°C range
    let temp_idx = (temp_c.clamp(0.0, 35.0) as usize).min(35);
    
    // Get saturation pressure from lookup table (in 0.01 hPa)
    let saturation_pressure = SATURATION_PRESSURE_TABLE[temp_idx] as f32;
    
    // Calculate actual vapor pressure (hPa)
    let vapor_pressure = saturation_pressure * (rel_humidity / 100.0);
    
    // Calculate absolute humidity (g/m³)
    let abs_humidity = (vapor_pressure * 2.1674) / (temp_c + 273.15);
    
    debug!("temp_idx: {}, saturation_pressure: {}, vapor_pressure: {}, abs_humidity: {}", 
           temp_idx, saturation_pressure, vapor_pressure, abs_humidity);
    
    abs_humidity
}