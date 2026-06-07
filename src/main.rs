#![no_std]
#![no_main]

/* RTT Logging */
use defmt::{warn, info, error};

/* Embassy framework */
use embassy_executor::{Executor, Spawner};
// use embassy_stm32::gpio::Pull;
use embassy_stm32::i2c::{Config, I2c};
use embassy_stm32::time::Hertz;
// use embassy_time::Timer;

use embassy_time::Timer;
/* Embedded graphics */
use embedded_graphics;
use sh1106::mode::GraphicsMode;
use sh1106::{prelude::DisplaySize, Builder};

/* Exception handling */
use {defmt_rtt as _, panic_probe as _};

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_stm32::init(Default::default());
    
    let mut config = Config::default();
    config.frequency = Hertz(400_000);
    config.scl_pullup = true;  // Enable SCL pull-up
    config.sda_pullup = true;  // Enable SDA pull-up

    let i2c = I2c::new_blocking(
        p.I2C1,
        p.PA9,   // SCL
        p.PA10,  // SDA
        config,
    );
    
    // Create the display with SH1106
    let mut display: GraphicsMode<_> = Builder::new()
        .with_i2c_addr(0x3D)  // Default I2C address for SH1106 (verify yours)
        .with_size(DisplaySize::Display128x64)  // Adjust if needed
        .connect_i2c(i2c)
        .into();

    // Initialize the display
    match display.init() {
        Ok(_) => info!("Display initialized successfully!"),
        Err(_e) => {
            error!("Display init failed");
        }
    }

    match display.flush() {
        Ok(_) => info!("Display flushed successfully!"),
        Err(_e) => error!("Display flush failed")
    }

    loop {}
}
