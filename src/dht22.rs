use embassy_time::{Duration, Timer};
use esp_hal::gpio::{Flex, OutputOpenDrain, Pull};
use log::{debug, error};

pub async fn read(pin: &mut OutputOpenDrain<'_>) -> Reading {
    pin.set_low();
    Timer::after(Duration::from_millis(18)).await;
    pin.set_high();
    Timer::after_micros(48).await;
    wait_for_state(pin, true).await;
    wait_for_state(pin, false).await;

    let mut data = [0; 4];

    for b in data.iter_mut() {
        *b = read_byte(pin).await;
    }
    let checksum = read_byte(pin).await;

    let calculated_checksum = data.iter().fold(0u8, |sum, v| sum.wrapping_add(*v));
    debug!("Checksum: {}", checksum);
    debug!("Data: {:?}", data);
    debug!("Calculated checksum: {}", calculated_checksum);

    if calculated_checksum != checksum {
        error!("Checksum mismatch");
    }

    let [humidity, _, temp_signed, _] = data;
    let temperature = {
        let (signed, magnitude) = convert_signed(temp_signed);
        let temp_sign = if signed { -1 } else { 1 };
        temp_sign * magnitude as i8
    };

    Reading {
        temperature,
        humidity,
    }
}

async fn wait_for_state(pin: &mut OutputOpenDrain<'_>, state: bool) {
    while !match state {
        true => pin.is_high(),
        false => pin.is_low(),
    } {
        Timer::after_micros(1).await;
    }
}

async fn read_bit(pin: &mut OutputOpenDrain<'_>) -> bool {
    wait_for_state(pin, true).await;
    Timer::after_micros(30).await;
    let high = pin.is_high();
    wait_for_state(pin, false).await;
    high
}

async fn read_byte(pin: &mut OutputOpenDrain<'_>) -> u8 {
    let mut byte: u8 = 0;
    for i in 0..8 {
        let bit_mask = 1 << (7 - (i % 8));
        if read_bit(pin).await {
            byte |= bit_mask;
        }
    }
    byte
}

fn convert_signed(signed: u8) -> (bool, u8) {
    let sign = signed & 0x80 != 0;
    let magnitude = signed & 0x7F;
    (sign, magnitude)
}

#[derive(Debug)]
pub struct Reading {
    pub temperature: i8,
    pub humidity: u8,
}
