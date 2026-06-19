#![no_std]
#![no_main]

/* RTT Logging */
use defmt::{debug, error, info, warn};

/* Embassy framework */
use embassy_executor::Spawner;
use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_stm32::i2c::{Config as i2cConfig, I2c, self};
use embassy_stm32::mode::{Blocking, Async};
use embassy_stm32::pac::syscfg::vals::{Pinmux2};
use embassy_stm32::spi::{Config as spiConfig, Spi, mode::Master, Phase::CaptureOnFirstTransition, Polarity::IdleLow};
use embassy_stm32::time::Hertz;
use embassy_stm32::pac;
use embassy_stm32::bind_interrupts;
use embassy_stm32::peripherals;
use embassy_stm32::dma::InterruptHandler as DmaInterruptHandler;
use embassy_time::Timer;
use embassy_embedded_hal::{shared_bus::asynch::i2c::I2cDevice};
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::mutex::Mutex;
use embedded_graphics::primitives::PrimitiveStyle;
use static_cell::StaticCell;

/* Embedded graphics */
use embedded_graphics::{
    pixelcolor::BinaryColor,
    prelude::*,
    text::{Baseline, Alignment, Text},
    primitives::{Rectangle, PrimitiveStyleBuilder},
};
use display_interface_i2c::I2CInterface;
use oled_async::{prelude::*, Builder, displays::sh1106};
use itoa;
use embedded_bitmap_fonts::{terminus::FONT_6x12, TextStyle};
use core::fmt::Write;
use heapless::String;

/* Exception handling */
use {defmt_rtt as _, panic_probe as _};

/* INA219 */
use ina219::{AsyncIna219, address::Address, configuration::{
        Configuration,
        BusVoltageRange,
        ShuntVoltageRange,
        Resolution,
        OperatingMode,
        MeasuredSignals,
        Reset
        }};

/* Declare mutex for i2c bus */
type I2c1Bus = Mutex<ThreadModeRawMutex, I2c<'static, Async, i2c::Master>>;
static I2C_BUS: StaticCell<I2c1Bus> = StaticCell::new();

/* Declare mutex for sharing thermocouple data */
static TEMPERATURE: Mutex<ThreadModeRawMutex, u32> = Mutex::new(0);

/* Constants */
const WIDTH: u8 = 128;
const HEIGHT: u8 = 64;
const SMALL_FONT: embedded_bitmap_fonts::BitmapFont<'_> = FONT_6x12;
const MEDIUM_FONT: embedded_bitmap_fonts::BitmapFont<'_> = FONT_6x12.pixel_double();
const LARGE_FONT: embedded_bitmap_fonts::BitmapFont<'_> = FONT_6x12.pixel_triple();
const RECT_STYLE: PrimitiveStyle<BinaryColor> = PrimitiveStyleBuilder::new().stroke_width(0).fill_color(BinaryColor::On).build();
const TEXT_STYLE_SMALL: TextStyle<'_> = TextStyle::new(&SMALL_FONT, BinaryColor::On);
const TEXT_STYLE_SMALL_KNOCKOUT: TextStyle<'_> = TextStyle::new(&SMALL_FONT, BinaryColor::Off);
const TEXT_STYLE_MEDIUM: TextStyle<'_> = TextStyle::new(&MEDIUM_FONT, BinaryColor::On);
const TEXT_STYLE_LARGE: TextStyle<'_> = TextStyle::new(&LARGE_FONT, BinaryColor::On);


#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_stm32::init(Default::default());
    
    unsafe {
        /* Remap PA9/PA10 I2C1 Alternate Functions to PA11/PA12 */
        pac::RCC.apbenr2().modify(|w| w.set_syscfgen(true));
        pac::SYSCFG.cfgr1().modify(|w| {
            w.set_pa11_rmp(true);  // PA11 pin acts as PA9 (I2C1_SCL)
            w.set_pa12_rmp(true);  // PA12 pin acts as PA10 (I2C1_SDA)
        });

        pac::SYSCFG.cfgr3().modify(|w| w.set_pinmux2(Pinmux2::from_bits(0b01)));
    }

    bind_interrupts!(struct Irqs {
        I2C1 => i2c::EventInterruptHandler<peripherals::I2C1>,
                i2c::ErrorInterruptHandler<peripherals::I2C1>;
        DMA1_CHANNEL1 => DmaInterruptHandler<peripherals::DMA1_CH1>;
        DMA1_CHANNEL2_3 => DmaInterruptHandler<peripherals::DMA1_CH2>;
    });

    let n_cs = Output::new(p.PA4, Level::High, Speed::High);
    let mut fan_enable = Output::new(p.PB3, Level::Low, Speed::High);

    let mut spi_config = spiConfig::default();
    spi_config.nss_output_disable = false; // Hardware NSS (not GPIO)
    spi_config.frequency = Hertz(1_000_000);
    spi_config.mode.phase = CaptureOnFirstTransition;
    spi_config.mode.polarity = IdleLow;

    let spi = Spi::new_blocking_rxonly( // TODO: Enough DMA available for async maybe?
        p.SPI1, 
        p.PA1, 
        p.PA6, 
        spi_config
    );

    /* Spawn tasks */
    spawner.spawn(read_thermocouple_task(spi, n_cs).unwrap());

    let mut i2c_config = i2cConfig::default();
    i2c_config.frequency = Hertz(400_000);
    i2c_config.scl_pullup = true;  // Enable SCL pull-up
    i2c_config.sda_pullup = true;  // Enable SDA pull-up

    let i2c = I2c::new(
        p.I2C1, 
        p.PA9, 
        p.PA10, 
        p.DMA1_CH1, 
        p.DMA1_CH2, 
        Irqs, 
        i2c_config);

    let i2c_bus = I2C_BUS.init(Mutex::new(i2c));

    spawner.spawn(display_measure_mode_task(i2c_bus).unwrap());
    spawner.spawn(read_transformer_ina219_task(i2c_bus).unwrap());
    spawner.spawn(read_fan_ina219_task(i2c_bus).unwrap());

    loop {
        Timer::after_millis(5000).await;
    }
} 

#[embassy_executor::task]
async fn read_thermocouple_task(mut spi_dev: Spi<'static, Blocking, Master>, mut nss: Output<'static>) {

    Timer::after_millis(100).await; // TODO: Wait for amp to setlle?

    let mut buf: [u8; 4] = [0; 4];
    let mut data: u32;
    let mut temperature: f32;
    
    loop {
        Timer::after_millis(100).await;

        /* Read 32 bits of MAX31855 register to buffer*/
        nss.set_low();
        match spi_dev.blocking_read(&mut buf) {
            Ok(_) => debug!("MAX31855 Data:{}", &buf),
            Err(e) => {
                error!("MAX31855 Read failed: {}", e);
            }
        }
        nss.set_high();

        /* Check data for faults */
        data = u32::from_be_bytes(buf);
        let error_data: u32 = data & 0x0001_0007; // get_bit(16) is general fault, get_bit(0..2) are specific faults
        if error_data > 0 {
            match error_data & 0x0000_0007 {

                1 => {
                    warn!("No thermocouple connected!");
                    continue;
                },
                2 => {
                    error!("Thermocouple shorted to GND!");
                    continue;
                },
                4 => {
                    error!("Thermocouple shorted to Vcc!");
                    continue;
                },
                _ => {
                    error!("Thermocouple issues!");
                    continue;
                }
            }   
        }

        let mut temp_data: u32 = data & 0xfffc_0000; // Mask for temperature bits (18..31) 14-bit signed value
        temp_data >>= 18;
        temperature = temp_data as f32 * 0.25;
        *TEMPERATURE.lock().await = temperature as u32 * 100;
        info!("Temperature: {=f32} C", temperature);
        

    }
}

#[embassy_executor::task]
async fn read_transformer_ina219_task(bus: &'static I2c1Bus) {
    let i2c_dev = I2cDevice::new(bus);

    // let mut ina_transformer = AsyncIna219::new_calibrated(i2c_dev, Address::from_byte(0x42).unwrap(), ina_calib);
    let mut ina_transformer = AsyncIna219::new(i2c_dev, Address::from_byte(0x42).unwrap()).await.unwrap();
    
    let ina_conf = Configuration {
        bus_voltage_range: BusVoltageRange::Fsr16v,
        bus_resolution: Resolution::Avg128,
        operating_mode: OperatingMode::Continous(MeasuredSignals::ShutAndBusVoltage),
        shunt_resolution: Resolution::Avg128,
        shunt_voltage_range: ShuntVoltageRange::Fsr40mv,
        reset: Reset::Reset,
    };
    
    ina_transformer.set_configuration(ina_conf).await.unwrap();

    let conversion_time = ina_conf.conversion_time_us().unwrap();

    loop {
        Timer::after_micros(conversion_time as u64).await;
        ina_transformer.next_measurement().await.expect("New reading ready!").unwrap();
        let _current_ma = ina_transformer.shunt_voltage().await.unwrap().shunt_voltage_uv() / 250;
        let _bus_voltage_v = ina_transformer.bus_voltage().await.unwrap().voltage_mv() as f32 / 1000 as f32;
        let _shunt_voltage_mv = ina_transformer.shunt_voltage().await.unwrap().shunt_voltage_mv();
        info!("INA219 Fan: Bus: {}V, Shunt: {}mV, Current: {}mA", _bus_voltage_v, _shunt_voltage_mv, _current_ma)
    }
}

#[embassy_executor::task]
async fn read_fan_ina219_task(bus: &'static I2c1Bus) {
    let i2c_dev = I2cDevice::new(bus);

    // let mut ina_transformer = AsyncIna219::new_calibrated(i2c_dev, Address::from_byte(0x42).unwrap(), ina_calib);
    let mut ina_fan = AsyncIna219::new(i2c_dev, Address::from_byte(0x40).unwrap()).await.unwrap();
    
    let ina_conf = Configuration {
        bus_voltage_range: BusVoltageRange::Fsr16v,
        bus_resolution: Resolution::Avg128,
        operating_mode: OperatingMode::Continous(MeasuredSignals::ShutAndBusVoltage),
        shunt_resolution: Resolution::Avg128,
        shunt_voltage_range: ShuntVoltageRange::Fsr40mv,
        reset: Reset::Reset,
    };
    
    ina_fan.set_configuration(ina_conf).await.unwrap();

    let conversion_time = ina_conf.conversion_time_us().unwrap();

    loop {
        Timer::after_micros(conversion_time as u64).await;
        ina_fan.next_measurement().await.expect("New reading ready!").unwrap();
        let _current_ma = ina_fan.shunt_voltage().await.unwrap().shunt_voltage_uv() / 250;
        let _bus_voltage_v = ina_fan.bus_voltage().await.unwrap().voltage_mv() as f32 / 1000 as f32;
        let _shunt_voltage_mv = ina_fan.shunt_voltage().await.unwrap().shunt_voltage_mv();
        info!("INA219 Fan: Bus: {}V, Shunt: {}mV, Current: {}mA", _bus_voltage_v, _shunt_voltage_mv, _current_ma)
    }
}

#[embassy_executor::task]
async fn display_measure_mode_task(bus: &'static I2c1Bus) {
    /* Create new i2c device from shared bus */
    let i2c_dev = I2cDevice::new(bus);

    /* Wrap i2c device in display interface */
    let display_interface = display_interface_i2c::I2CInterface::new(i2c_dev, 0x3C, 0x40);

    /* Create raw display handle */
    let display_raw = Builder::new(sh1106::Sh1106_128_64{})
        .with_rotation(DisplayRotation::Rotate0)
        .connect(display_interface);

    /* Connect display to handle */
    let mut display: GraphicsMode<_,_> = display_raw.into();

    /* Initialize display */
    display.init().await.unwrap();
    display.clear();
    display.flush().await.unwrap();

    /* Write updated display */
    display.flush().await.unwrap();

    /*  */
    let mut buffer = itoa::Buffer::new();
    let mut temperature: u32;
    
    loop {
        /* Clear display ready for new data */
        display.clear();

        /* Read the temperature mutex and format it into a string */
        temperature = *TEMPERATURE.lock().await / 100;
        let temperature_str = buffer.format(temperature);
        let mut temperature_str_concat: String<10> = String::new();
        write!(&mut temperature_str_concat, "{temperature_str}°C").unwrap();

        /* Display elements */
        Text::with_alignment(&temperature_str_concat, Point { x: (WIDTH as i32 + 10), y: (HEIGHT as i32 / 2 - 24) }, TEXT_STYLE_LARGE, Alignment::Right)
            .draw(&mut display)
            .unwrap();

        Rectangle::new(Point::new(0, HEIGHT as i32 - 12) , Size::new(WIDTH as u32, 12))
            .into_styled(RECT_STYLE)
            .draw(&mut display)
            .unwrap();

        Text::with_baseline("Mode: ", Point::new(2, HEIGHT as i32 - 12), TEXT_STYLE_SMALL_KNOCKOUT, Baseline::Top)
            .draw(&mut display)
            .unwrap();

        Text::with_alignment("Measure", Point { x: (WIDTH as i32 - 2), y: (HEIGHT as i32 - 12) }, TEXT_STYLE_SMALL_KNOCKOUT, Alignment::Right)
            .draw(&mut display)
            .unwrap();

        /* Flush to display */
        display.flush().await.unwrap();
        Timer::after_millis(10).await;
    }
}