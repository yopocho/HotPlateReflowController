#![no_std]
#![no_main]

use core::ops::Add;

/* RTT Logging */
use defmt::{warn, info, error};

/* Embassy framework */
use embassy_executor::{Executor, Spawner};
// use embassy_stm32::gpio::Pull;
use embassy_stm32::i2c::{Config as i2cConfig, I2c};
use embassy_stm32::spi::Phase::{CaptureOnFirstTransition, CaptureOnSecondTransition};
use embassy_stm32::spi::Polarity::IdleLow;
use embassy_stm32::spi::{Config as spiConfig, Spi, Mode};
use embassy_stm32::time::Hertz;
// use embassy_time::Timer;

/* Sensors */
use ina219::address::Address;
use ina219::SyncIna219;

/* Embedded graphics */
use embedded_graphics;
use sh1106::mode::GraphicsMode;
use sh1106::{prelude::DisplaySize, Builder};

/* Exception handling */
use {defmt_rtt as _, panic_probe as _};

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_stm32::init(Default::default());
    
    let mut spi_config = spiConfig::default();
    spi_config.nss_output_disable = false;
    spi_config.frequency = Hertz(1_000_000);
    spi_config.mode.phase = CaptureOnSecondTransition; // TODO: Possibly wrong value
    spi_config.mode.polarity = IdleLow; // TODO: Possibly wrong value

    let spi = Spi::new_blocking_rxonly(
        p.SPI1, 
        p.PA1, 
        p.PA6, 
        spi_config
    );

    let mut i2c_config = i2cConfig::default();
    i2c_config.frequency = Hertz(400_000);
    i2c_config.scl_pullup = true;  // Enable SCL pull-up
    i2c_config.sda_pullup = true;  // Enable SDA pull-up

    let i2c = I2c::new_blocking(
        p.I2C1,
        p.PA9,   // SCL
        p.PA10,  // SDA
        i2c_config,
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
