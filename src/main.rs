#![no_std]
#![no_main]

mod fmt;

use defmt::{warn};
use embassy_executor::{Executor, Spawner};
use embassy_stm32::i2c::{Config, I2c};
use embassy_stm32::time::Hertz;
use embassy_time::Timer;
use fmt::info;
use {defmt_rtt as _, panic_probe as _};

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_stm32::init(Default::default());
    info!("STM32C011F6 I2C scanner starting...");

    let mut config = Config::default();
    config.frequency = Hertz(100_000);

    let mut i2c = I2c::new_blocking(
        p.I2C1,
        p.PA9,   // SCL
        p.PA10,  // SDA
        config,
    );

    info!("Scanning I2C bus...");
    let mut found = false;

    for addr in 0x08u8..=0x77 {
        let data = [0x00u8];
        match i2c.blocking_write(addr, &data) {
            Ok(_) => {
                info!("Found device at address: 0x{:02X}", addr);
                found = true;
            }
            Err(e) => {
                warn!("Error at 0x{:02X}: {:?}", addr, e);
            }
        }
    }

    if !found {
        info!("  No devices found.");
    }

    info!("Scan complete.");
}
