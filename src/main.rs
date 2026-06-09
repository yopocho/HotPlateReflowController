#![no_std]
#![no_main]

use core::ops::Add;

/* RTT Logging */
use defmt::{info, error};

/* Embassy framework */
use embassy_executor::{Executor, Spawner};
// use embassy_stm32::gpio::Pull;
use embassy_stm32::i2c::{Config as i2cConfig, I2c};
use embassy_stm32::spi::Phase::{CaptureOnFirstTransition, CaptureOnSecondTransition};
use embassy_stm32::spi::Polarity::IdleLow;
use embassy_stm32::spi::{Config as spiConfig, Spi, Mode};
use embassy_stm32::time::Hertz;
use embassy_stm32::pac;
// use embassy_time::Timer;

/* Embedded graphics */
use embedded_graphics::{
    mono_font::{ascii::FONT_6X10, MonoTextStyleBuilder},
    pixelcolor::BinaryColor,
    prelude::*,
    text::{Baseline, Text},
    primitives::{Rectangle, PrimitiveStyleBuilder},
};
use sh1106::{prelude::*, Builder};

/* Exception handling */
use {defmt_rtt as _, panic_probe as _};

/* Constants */
const WIDTH: u8 = 128;
const HEIGHT: u8 = 64;

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_stm32::init(Default::default());
    
    /* Remap PA9/PA10 I2C1 Alternate Functions to PA11/PA12 */
    unsafe {
        pac::RCC.apbenr2().modify(|w| w.set_syscfgen(true));
        pac::SYSCFG.cfgr1().modify(|w| {
            w.set_pa11_rmp(true);  // PA11 pin acts as PA9 (I2C1_SCL)
            w.set_pa12_rmp(true);  // PA12 pin acts as PA10 (I2C1_SDA)
        });
    }
    
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
    
    /* Create the display with SH1106 */
    let mut display: GraphicsMode<_> = Builder::new()
        .with_i2c_addr(0x3C)  // Default I2C address for SH1106 (verify yours)
        .with_size(DisplaySize::Display128x64)  // Adjust if needed
        .connect_i2c(i2c)
        .into();

    /* Build rectangle style */
    let rect_style = PrimitiveStyleBuilder::new()
        .stroke_width(0)
        .fill_color(BinaryColor::On)
        .build();
    
    /* Build text style */
    let text_style = MonoTextStyleBuilder::new()
        .font(&FONT_6X10)
        .text_color(BinaryColor::On)
        .build();

    let text_style_knockout = MonoTextStyleBuilder::new()
        .font(&FONT_6X10)
        .text_color(BinaryColor::Off)
        .build();

    /* Initialize the display */
    match display.init() {
        Ok(_) => info!("Display initialized successfully!"),
        Err(_e) => {
            error!("Display init failed");
        }
    }

    /* Write display */
    match display.flush() {
        Ok(_) => info!("Display flushed successfully!"),
        Err(_e) => error!("Display flush failed")
    }

    Text::with_baseline("Screen Test!", Point::zero(), text_style, Baseline::Top)
        .draw(&mut display)
        .unwrap();

    Rectangle::new(Point::new(0, HEIGHT as i32 - 14) , Size::new(WIDTH as u32, 14))
        .into_styled(rect_style)
        .draw(&mut display)
        .unwrap();

    Text::with_baseline("Mode:", Point::new(2, HEIGHT as i32 - 12), text_style_knockout, Baseline::Top)
        .draw(&mut display)
        .unwrap();


    /* Write updated display */
    match display.flush() {
        Ok(_) => info!("Display flushed successfully!"),
        Err(_e) => error!("Display flush failed")
    }

    loop {}
}
