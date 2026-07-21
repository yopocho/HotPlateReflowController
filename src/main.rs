#![no_std]
#![no_main]
#![allow(unused_unsafe)]

/** INCLUDES BEGIN **/
/* RTT Logging */
use defmt::{debug, error, info, warn};
/* Embassy framework */
use embassy_executor::Spawner;
use embassy_stm32::gpio::{Level, Output, Speed, Pull};
use embassy_stm32::interrupt::typelevel::{EXTI2_3, EXTI4_15};
use embassy_stm32::exti::ExtiInput;
use embassy_stm32::i2c::{Config as i2cConfig, I2c, self};
use embassy_stm32::mode::{Async, Blocking};
use embassy_stm32::pac::syscfg::vals::{Pinmux2};
use embassy_stm32::rcc::{Hsi, HsiSysDiv, HsiKerDiv};
use embassy_stm32::spi::{Config as spiConfig, Spi, mode::Master, Phase::CaptureOnFirstTransition, Polarity::IdleLow};
use embassy_stm32::time::Hertz;
use embassy_stm32::pac::{self};
use embassy_stm32::bind_interrupts;
use embassy_stm32::peripherals::{self};
use embassy_stm32::dma::InterruptHandler as DmaInterruptHandler;
use embassy_stm32::timer::Channel::{Ch2, Ch3};
use embassy_stm32::timer::simple_pwm::{PwmPin, SimplePwm};
use embassy_time::{Timer, Instant};
use embassy_embedded_hal::{shared_bus::asynch::i2c::I2cDevice};
use embassy_futures::select::{select3, Either3};
use embassy_sync::blocking_mutex::raw::{CriticalSectionRawMutex, ThreadModeRawMutex};
use embassy_sync::watch::Watch;
use embassy_sync::mutex::Mutex;
use embassy_sync::pubsub::{PubSubChannel};
use embassy_sync::channel::{Channel};
use embedded_hal::Pwm;
use static_cell::StaticCell;
/* Embedded graphics */
use embedded_graphics::{
    pixelcolor::BinaryColor,
    prelude::*,
    text::{
        Baseline, 
        Alignment, 
        Text
    },
    primitives::{
        Rectangle, 
        PrimitiveStyleBuilder, 
        PrimitiveStyle,
        Triangle,
        Line,
    },
};
use display_interface_i2c;
use oled_async::{
    prelude::*, 
    Builder, 
    displays::sh1106
};
use itoa;
use embedded_bitmap_fonts::{
    terminus::FONT_6x12, 
    TextStyle
};
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
/* FSM */
use statig::prelude::*;
/* PID */
use pid::{ControlOutput, Pid};
/** INCLUDES END **/


/** CONSTANTS BEGIN **/
/* Display dimensions */
const WIDTH: u8 = 128;
const HEIGHT: u8 = 64;
/* Embedded graphics styles */
const SMALL_FONT: embedded_bitmap_fonts::BitmapFont<'_> = FONT_6x12;
const MEDIUM_FONT: embedded_bitmap_fonts::BitmapFont<'_> = FONT_6x12.pixel_double();
const LARGE_FONT: embedded_bitmap_fonts::BitmapFont<'_> = FONT_6x12.pixel_triple();
const RECT_STYLE: PrimitiveStyle<BinaryColor> = PrimitiveStyleBuilder::new().stroke_width(0).fill_color(BinaryColor::On).build();
const TRI_STYLE: PrimitiveStyle<BinaryColor> = PrimitiveStyleBuilder::new().stroke_width(0).fill_color(BinaryColor::On).build();
const _TRI_KNOCKOUT_STYLE: PrimitiveStyle<BinaryColor> = PrimitiveStyleBuilder::new().stroke_width(0).fill_color(BinaryColor::Off).build();
const LINE_STYLE: PrimitiveStyle<BinaryColor> = PrimitiveStyleBuilder::new().stroke_width(1).stroke_color(BinaryColor::On).build();
const LINE_STYLE_KNOCKOUT: PrimitiveStyle<BinaryColor> = PrimitiveStyleBuilder::new().stroke_width(1).stroke_color(BinaryColor::Off).build();
const TEXT_STYLE_SMALL: TextStyle<'_> = TextStyle::new(&SMALL_FONT, BinaryColor::On);
const TEXT_STYLE_SMALL_KNOCKOUT: TextStyle<'_> = TextStyle::new(&SMALL_FONT, BinaryColor::Off);
const TEXT_STYLE_MEDIUM: TextStyle<'_> = TextStyle::new(&MEDIUM_FONT, BinaryColor::On);
const TEXT_STYLE_LARGE: TextStyle<'_> = TextStyle::new(&LARGE_FONT, BinaryColor::On);
/* Min/max hot plate temperatures */
const MAX_TEMP: u32 = 300;
const MIN_TEMP: u32 = 0;
/* Task timeout */
const TASK_TIMEOUT: embassy_time::Duration = embassy_time::Duration::from_millis(100);
/* PID Gain values */
const PID_KP: f32 = 1000.0;
const PID_KI: f32 = 1.5;
const _PID_KD: f32 = 100.0;
const PID_KI_LIMIT: f32 = 38000.0;
/* PID Temperature dead-zone */
const PID_TEMP_DEADZONE: f32 = 3.0; // °C
/** CONSTANTS END **/


/** EMBASSY_SYNC DECLARATIONS BEGIN **/
/* Declare mutex for i2c bus */
type I2c1Bus = Mutex<ThreadModeRawMutex, I2c<'static, Async, i2c::Master>>;
static I2C_BUS: StaticCell<I2c1Bus> = StaticCell::new();
/* Declare mutex for sharing thermocouple data */
static TEMPERATURE: Mutex<ThreadModeRawMutex, f32> = Mutex::new(0.0);
/* Declare mutex for static setpoint temperature */
static SETPOINT_TEMPERATURE: Mutex<ThreadModeRawMutex, u32> = Mutex::new(20);
/* Declare mutex for selected reflow profile */
static SELECTED_REFLOW_PROFILE: Mutex<ThreadModeRawMutex, ReflowProfiles> = Mutex::new(ReflowProfiles::NoProfileSelected);
/* Declare Pub/Sub-Channel for rotary encoder data */
static ROT_ENC_CHANNEL: PubSubChannel<ThreadModeRawMutex, RotaryEncoder, 1, 4, 1> = PubSubChannel::new();
/* Watch channel for FSM state */
static FSM_STATE: Watch<CriticalSectionRawMutex, State, 10> = Watch::new();
/* FSM Event Queue */
static EVENT_QUEUE: Channel<CriticalSectionRawMutex, Event, 10> = Channel::new();
/* Declare Mutex for currently selected UI element */
static SELECTEDUIELEMENT: Mutex<ThreadModeRawMutex, SelectedUIElement> = Mutex::new(SelectedUIElement::NoneSelected);
/** EMBASSY_SYNC DECLARATIONS END **/


/** STATIC IRQ PINS BEGIN **/
/* Static encoder GPIOs */
static ENCODER_A_INPUT: StaticCell<ExtiInput<Async>> = StaticCell::new();
static ENCODER_B_INPUT: StaticCell<ExtiInput<Async>> = StaticCell::new();
static ENCODER_BTN_INPUT: StaticCell<ExtiInput<Async>> = StaticCell::new();
static ZCD_DETECT: StaticCell<ExtiInput<Async>> = StaticCell::new();
/** STATIC IRQ PINS END **/


/* ENUMS BEGIN */
/* Enum containing possible rotational directions for enncoder */
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum RotaryEncoderDirection {
    CW,
    CCW,
    Stationary,
}
/* Default implementation for RotaryEncoderDirection */
impl Default for RotaryEncoderDirection {
    fn default() -> Self { RotaryEncoderDirection::Stationary }
}
/* PubSubChannel for rotary encoder position */
#[derive(Clone, Default)]
struct RotaryEncoder {
    position: u32,
    pressed: bool,
    direction: RotaryEncoderDirection,
}
/* FSM Events */
#[derive(Clone, Copy, PartialEq, Debug)]
pub enum Event {
    EncoderPressed,
    SetpointSelected,
    SetpointTemperatureRollerSelected,
    SetpointMenuSelected,
    SetpointTemperatureSet,
    SetpointTemperatureUnset,
    ReflowSelected,
    ReflowStartSelected,
    ReflowStopSelected,
    ReflowProfileSelectorSelected,
    ReflowMenuSelected,
    MenuSelected,
    MeasureSelected,
    EncoderPositionReading(u16),
    TemperatureReading(f32),
    FanCurrentReading(f32),
    TransformerCurrentReading(f32),
    ControlTick,
    DisplayTick,
    Error(ErrorType),
}
/* Types of possible errors */
#[derive(Clone, Copy, PartialEq, Debug)]
pub enum ErrorType {
    ThermocoupleShortGnd,
    ThermocoupleShortVcc,
    ThermocoupleIssue,
    Overcurrent,
    Overtemp,
    NoHeat,
    NoFan,
    NoDisplay,
    NoTransformer,
    NoInaFan,
    NoInaTransformer,
    NoMax,
    NoZCD,
    NoEncoder,
    NoThermocouple,
    NoErrors,
}
/* All selectable UI elements */
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum SelectedUIElement {
    MenuReflow,
    MenuSetpoint,
    MenuMeasure,
    SetpointTemperatureRollerInactive,
    SetpointTemperatureRollerActive,
    SetpointMenu,
    ReflowProfile1,
    ReflowProfile2,
    ReflowProfile3,
    ReflowProfile4,
    ReflowProfile5,
    ReflowProfile6,
    ReflowProfileMenu,
    ReflowProfileSelectorMenu,
    ReflowStart,
    ReflowStop,
    ReflowMenu,
    MeasureMenu,
    NoneSelected,
}
/* Available reflow profiles */
#[derive(Clone, Copy, PartialEq, Debug)]
pub enum ReflowProfiles {
    TS391SNL,
    GC10,
    NoProfileSelected,
}
// TODO: Create reflow profiles. Maybe add a struct for each type containing temp setpoints over time and 
// extrapolating ramp onto curve with points spaced 100ms apart
// struct TS391SNL {
//     name: Str = "TS391SNL",
//     max_temp: u32 = 249, //

// }
/** ENUMS END **/


/** FSM MACRO DEFINITION BEGIN **/
pub struct HPRC;
#[state_machine(
    initial = "State::menu()", 
    after_transition = "Self::after_transition",
    state(derive(Debug, Clone, PartialEq, Eq)),
)]
impl HPRC {
    #[superstate] // TODO: How do I set this up?
    async fn issue(event: &Event) -> Outcome<State> {
        match event {
            Event::Error(_) => Transition(State::error()),
            _ => Super,
        }
    }

    #[state(superstate = "issue")]
    async fn menu(event: &Event) -> Outcome<State> {
        match event {
            Event::ReflowSelected => {
                *SELECTEDUIELEMENT.lock().await = SelectedUIElement::ReflowProfile1;
                Transition(State::reflow_profile_selection())
            },
            Event::SetpointSelected => {
                *SELECTEDUIELEMENT.lock().await = SelectedUIElement::SetpointTemperatureRollerInactive;
                Transition(State::setpoint())
            },
            Event::MeasureSelected => {
                *SELECTEDUIELEMENT.lock().await = SelectedUIElement::MeasureMenu;
                Transition(State::measure())
            },
            _ => Super
        }
    }
    
    #[state(superstate = "issue")]
    async fn setpoint(event: &Event) -> Outcome<State> {
        match event {
            Event::SetpointMenuSelected => {
                *SELECTEDUIELEMENT.lock().await = SelectedUIElement::MenuReflow;
                Transition(State::menu())
            },
            Event::SetpointTemperatureRollerSelected => {
                *SELECTEDUIELEMENT.lock().await = SelectedUIElement::SetpointTemperatureRollerActive;
                Transition(State::setpoint_selecting())

            }
            _ => Super,
        }
    }
    
    #[state(superstate = "issue")]
    async fn setpoint_selecting(event: &Event) -> Outcome<State> {
        match event {
            Event::SetpointTemperatureSet => {
                *SELECTEDUIELEMENT.lock().await = SelectedUIElement::SetpointTemperatureRollerInactive;
                Transition(State::setpoint_running())
            }
            Event::SetpointTemperatureUnset => {
                *SELECTEDUIELEMENT.lock().await = SelectedUIElement::SetpointTemperatureRollerInactive;
                Transition(State::setpoint())
            }
            _ => Super,
        }
    }

    #[state(superstate = "issue")]
    async fn setpoint_running(event: &Event) -> Outcome<State> {
        match event {
            Event::SetpointMenuSelected => {
                *SELECTEDUIELEMENT.lock().await = SelectedUIElement::MenuReflow;
                Transition(State::menu())
            },
            Event::SetpointTemperatureRollerSelected => {
                *SELECTEDUIELEMENT.lock().await = SelectedUIElement::SetpointTemperatureRollerActive;
                Transition(State::setpoint_selecting())
            }
            _ => Super,
        }
    }

    #[state(superstate = "issue")]
    async fn reflow(event: &Event) -> Outcome<State> {
        match event {
            Event::ReflowStartSelected => {
                *SELECTEDUIELEMENT.lock().await = SelectedUIElement::ReflowStop;
                Transition(State::reflow_running())
            }
            Event::ReflowProfileSelectorSelected => {
                *SELECTEDUIELEMENT.lock().await = SelectedUIElement::ReflowProfile1;
                Transition(State::reflow_profile_selection())
            }
            Event::ReflowMenuSelected => {
                *SELECTEDUIELEMENT.lock().await = SelectedUIElement::MenuReflow;
                Transition(State::menu())
            }
            _ => Super,
        }
    }

    #[state(superstate = "issue")]
    async fn reflow_profile_selection(event: &Event) -> Outcome<State> {
        match event {
            Event::MenuSelected =>  {
                *SELECTEDUIELEMENT.lock().await = SelectedUIElement::MenuReflow;
                Transition(State::menu())
            },
            Event::ReflowSelected => {
                *SELECTEDUIELEMENT.lock().await = SelectedUIElement::ReflowStart;
                Transition(State::reflow())
            }
            _ => Super,
        }
    }

    #[state(superstate = "issue")]
    async fn reflow_running(event: &Event) -> Outcome<State> {
        match event {
            Event::ReflowStopSelected => {
                *SELECTEDUIELEMENT.lock().await = SelectedUIElement::ReflowStart;
                Transition(State::reflow())
            },
            _ => Super,
        }
    }

    #[state(superstate = "issue")]
    async fn measure(event: &Event) -> Outcome<State> {
        match event {
            Event::MenuSelected => {
                *SELECTEDUIELEMENT.lock().await = SelectedUIElement::MenuReflow;
                Transition(State::menu())
            }
            _ => Super,
        }
    }

    #[state(superstate = "issue")]
    async fn error(event: &Event) -> Outcome<State> {
        match event {
            Event::Error(ErrorType::NoErrors) => Transition(State::menu()),
            _ => Super,
        }
    }

    async fn after_transition(&mut self, _source: &State, target: &State, _context: &mut ()) {
        FSM_STATE.sender().send(target.clone());
        info!("State transition");
    }
}
/** FSM MACRO DEFINITION END **/

/** TASKS BEGIN **/
#[embassy_executor::main]
async fn main(spawner: Spawner) {

    /* Construct MCU config */
    let mut mcu_config = embassy_stm32::Config::default();
    /* Set HSI48 as SYSCLK */
    mcu_config.rcc.hsi = Some(Hsi {
        sys_div: HsiSysDiv::DIV1,
        ker_div: HsiKerDiv::DIV1,
    });
    mcu_config.rcc.sys = embassy_stm32::rcc::Sysclk::HSISYS;
    mcu_config.rcc.ahb_pre = embassy_stm32::rcc::AHBPrescaler::DIV1;
    mcu_config.rcc.apb1_pre = embassy_stm32::rcc::APBPrescaler::DIV1;

    /* Initialize MCU with config and return peripheral handle */
    let p = embassy_stm32::init(mcu_config);
    
    unsafe {
        /* Remap PA9/PA10 I2C1 Alternate Functions to PA11/PA12 */
        pac::RCC.apbenr2().modify(|w| w.set_syscfgen(true));
        pac::SYSCFG.cfgr1().modify(|w| {
            w.set_pa11_rmp(true);  // PA11 pin acts as PA9 (I2C1_SCL)
            w.set_pa12_rmp(true);  // PA12 pin acts as PA10 (I2C1_SDA)
        });
        pac::SYSCFG.cfgr3().modify(|w| w.set_pinmux2(Pinmux2::from_bits(0b01)));
        
        /* Set I2C1 mode to FM+ (1MHz) */
        pac::SYSCFG.cfgr1().modify(|w| {
            w.set_i2c1_fmp(true);
        });
    }

    info!("clocks = {}", embassy_stm32::rcc::clocks(&p.RCC));
    
    bind_interrupts!(struct Irqs {
        I2C1 => i2c::EventInterruptHandler<peripherals::I2C1>,
                i2c::ErrorInterruptHandler<peripherals::I2C1>;
        DMA1_CHANNEL1 => DmaInterruptHandler<peripherals::DMA1_CH1>;
        DMA1_CHANNEL2_3 => DmaInterruptHandler<peripherals::DMA1_CH2>;
    });

    bind_interrupts!(struct IrqsEncoder {
        EXTI4_15 => embassy_stm32::exti::InterruptHandler<EXTI4_15>;
    });

    bind_interrupts!(struct IrqsZcd {
        EXTI2_3 => embassy_stm32::exti::InterruptHandler<EXTI2_3>;
    });

    let n_cs = Output::new(p.PA4, Level::High, Speed::High);
    let fan_pin: PwmPin<'_, peripherals::TIM2, embassy_stm32::timer::Ch2> = PwmPin::new(p.PB3, embassy_stm32::gpio::OutputType::PushPull);
    let mut fan_pmw = SimplePwm::new(p.TIM2, None, Some(fan_pin), None, None, Hertz(1000), embassy_stm32::timer::low_level::CountingMode::EdgeAlignedUp);
    let _max_duty_fan_pmw = fan_pmw.get_max_duty(); // TODO: Create fan task
    fan_pmw.enable(Ch2); // TODO: Create fan task
    fan_pmw.set_duty(Ch2, 0); // TODO: Create fan task

    let triac_pin: PwmPin<'_, peripherals::TIM1, embassy_stm32::timer::Ch3> = PwmPin::new(p.PA2, embassy_stm32::gpio::OutputType::PushPull);
    let triac_pwm: SimplePwm<'_, peripherals::TIM1> = SimplePwm::new(p.TIM1, None, None, Some(triac_pin), None, Hertz(10), embassy_stm32::timer::low_level::CountingMode::EdgeAlignedUp);

    let mut spi_config = spiConfig::default();
    spi_config.nss_output_disable = false; // Hardware NSS (not GPIO)
    spi_config.frequency = Hertz(1_000_000);
    spi_config.mode.phase = CaptureOnFirstTransition;
    spi_config.mode.polarity = IdleLow;

    let spi = Spi::new_blocking_rxonly( //  TODO: Enough DMA available for async maybe?
        p.SPI1, 
        p.PA1, 
        p.PA6, 
        spi_config
    );
    
    let mut i2c_config = i2cConfig::default();
    i2c_config.frequency = Hertz(1_000_000);
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
    
    /* Bind encoder interrupts */
    let encoder_a = ENCODER_A_INPUT.init(ExtiInput::new(p.PA5, p.EXTI5, Pull::Up, IrqsEncoder));
    let encoder_b = ENCODER_B_INPUT.init(ExtiInput::new(p.PA7, p.EXTI7, Pull::Up, IrqsEncoder));
    let encoder_btn = ENCODER_BTN_INPUT.init(ExtiInput::new(p.PA8, p.EXTI8, Pull::Up, IrqsEncoder));

    /* Construct the encoder */
    RotaryEncoder::default();

    /* Bind ZCD interrupt */
    let _zcd_detector = ZCD_DETECT.init(ExtiInput::new(p.PA3, p.EXTI3, Pull::Up, IrqsZcd));

    /* Spawn tasks */
    spawner.spawn(read_transformer_ina219_task(i2c_bus).unwrap());
    spawner.spawn(read_fan_ina219_task(i2c_bus).unwrap());
    spawner.spawn(task_encoder(encoder_a, encoder_b, encoder_btn).unwrap());
    spawner.spawn(read_thermocouple_task(spi, n_cs).unwrap());
    // spawner.spawn(task_zcd_detector(zcd_detector).unwrap());
    spawner.spawn(display_task(i2c_bus).unwrap());
    spawner.spawn(pid_task(triac_pwm).unwrap());

    /* FSM Event Queue receiver */
    let mut event: Event;

    /* Initialize FSM */
    let mut machine = HPRC.state_machine();
    FSM_STATE.sender().send(machine.state().clone());
    
    loop {
        /* Handle FSM events */
        event = EVENT_QUEUE.receive().await;
        machine.handle(&event).await;
    }
} 

#[embassy_executor::task]
async fn pid_task(mut triac_pwm: SimplePwm<'static, peripherals::TIM1>) {
    /* Triac PWM output */
    let max_duty_triac_pwm = triac_pwm.get_max_duty();
    triac_pwm.set_duty(Ch3, max_duty_triac_pwm);
    triac_pwm.enable(Ch3);

    let triac_pmw_freq = triac_pwm.get_frequency();
    info!("triac_pwm freq: {}", &&triac_pmw_freq);
    
    /* Create PID controller with gains */
    // TODO: Hook PID target into both setpoint and reflow
    let mut pid: Pid<f32> = Pid::new(0.0, max_duty_triac_pwm as f32);
    pid.p(PID_KP, max_duty_triac_pwm as f32);
    pid.i(PID_KI,   PID_KI_LIMIT);
    // pid.d(PID_KD,  max_duty_triac_pwm as f32);
    let mut pid_output: ControlOutput<f32>;
    let mut triac_pwm_output: u16;

    /* Accurate time tracking */
    let mut expiration_time: Instant;

    /* FSM_STATE receiver */
    let mut fsm_state_rx = FSM_STATE.receiver().unwrap();
    
    /* Local vars */
    let mut power_percentage: u8;
    let mut fsm_state: State;
    let mut setpoint_target: u32;
    let mut temperature: f32;
    let mut error_abs: f32;

    loop {
        /* Get current time to reference task duration to */
        expiration_time = Instant::now() + TASK_TIMEOUT;

        /* Set PID target to setpoint temperature */
        setpoint_target = *SETPOINT_TEMPERATURE.lock().await;
        pid.setpoint(setpoint_target as f32);

        /* Receive FSM_STATE */
        fsm_state = fsm_state_rx.try_get().unwrap();

        /* Drive PID according to current state */
        match fsm_state {
            State::SetpointRunning {  } => {
                /* Retreive current temperature */
                temperature = *TEMPERATURE.lock().await;

                /* Calculate absolute error */
                error_abs = (pid.setpoint - temperature).abs();

                /* Stop I-term wind-up/down if inside deadzone */
                if error_abs < PID_TEMP_DEADZONE {
                    pid.i(0.0, PID_KI_LIMIT);
                }
                else {
                    pid.i(PID_KI, PID_KI_LIMIT);
                }

                /* Feed current temperature into PID */
                pid_output = pid.next_control_output(temperature );
                /* Shape output into clamped and inverted value for PWM output */
                triac_pwm_output = max_duty_triac_pwm as u16 - (pid_output.output.clamp(0.0, max_duty_triac_pwm as f32) as u16);
                power_percentage = 100 - (triac_pwm_output as f32 / max_duty_triac_pwm as f32 * 100.0) as u8;
                warn!("\x1B[32mP\x1B[0m: {} \x1B[32mI\x1B[0m: {} \x1B[32mD\x1B[0m: {} \x1B[32moutput\x1B[0m: {} \x1B[32mPWM\x1B[0m: {} \x1B[32mtemp\x1B[0m: {} \x1B[32mpwr\x1B[0m: {} \x1B[32mtarget\x1B[0m: {}", &pid_output.p, &pid_output.i, &pid_output.d, &pid_output.output, &triac_pwm_output, &temperature, &power_percentage, &setpoint_target);
            }
            _ => {
                triac_pwm_output = max_duty_triac_pwm as u16; 
            }
        }
        /* Set triac  */
        triac_pwm.set_duty(Ch3, triac_pwm_output as u32);
        
        /* Await until the loop has taken exactly PID_TIMEOUT(_CORE) (100ms) */
        Timer::at(expiration_time).await;
    }
}

#[embassy_executor::task]
async fn read_thermocouple_task(mut spi_dev: Spi<'static, Blocking, Master>, mut nss: Output<'static>) {
    /* Local vars */
    let mut buf: [u8; 4] = [0; 4];
    let mut data: u32;
    let mut temperature: f32;
    
    loop {
        /* 10Hz maximum read frequency */
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
        *TEMPERATURE.lock().await = temperature;
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
        shunt_voltage_range: ShuntVoltageRange::Fsr80mv,
        reset: Reset::Reset,
    };
    
    ina_transformer.set_configuration(ina_conf).await.unwrap();

    let conversion_time = ina_conf.conversion_time_us().unwrap();

    loop {
        Timer::after_micros(conversion_time as u64).await;
        ina_transformer.next_measurement().await.expect("New reading ready!").unwrap();
        let bus_voltage_v = ina_transformer.bus_voltage().await.unwrap().voltage_mv() as f32 / 1000 as f32;
        let shunt_voltage_mv = ina_transformer.shunt_voltage().await.unwrap().shunt_voltage_mv();
        let mut current = shunt_voltage_mv as f32 / 1.41414141 / 10 as f32;
        if current < 0 as f32 {current *= -1 as f32};
        info!("INA219 Transformer: Bus: {}V, Shunt: {}mV, Current: {}A", bus_voltage_v, shunt_voltage_mv, current)
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
        info!("INA219 Fan: Bus: {}V, Shunt: {}mV, Current: {}mA", _bus_voltage_v, _shunt_voltage_mv, _current_ma);
    }
}

#[embassy_executor::task]
async fn task_encoder(encoder_a: &'static mut ExtiInput<'static, Async>, encoder_b: &'static mut ExtiInput<'static, Async>, encoder_btn: &'static mut ExtiInput<'static, Async>) {

    /* Create a publisher for the channel */
    let publisher = ROT_ENC_CHANNEL.publisher().unwrap();

    /* FSM_STATE receiver */
    let mut fsm_state_rx = FSM_STATE.receiver().unwrap();
    let mut fsm_state: State;

    /* Local vars */
    let mut rot_enc_pos: u32 = 0;
    let mut pressed: bool;
    let mut direction: RotaryEncoderDirection;

    loop {
        /* Wait for changes on the encoder interrupt lines */
        match select3(
            encoder_a.wait_for_falling_edge(),
            encoder_b.wait_for_falling_edge(),
            encoder_btn.wait_for_any_edge(),
        ).await {
            Either3::First(_) => {
                if encoder_b.is_low() {
                    // CCW
                    rot_enc_pos = (rot_enc_pos + 359) % 360;
                    direction = RotaryEncoderDirection::CCW;
                    fsm_state = fsm_state_rx.try_get().unwrap();
                    match fsm_state {
                        State::SetpointSelecting {  } => { 
                            let setpoint_temp = *SETPOINT_TEMPERATURE.lock().await;
                            if setpoint_temp <= MIN_TEMP || setpoint_temp >= MAX_TEMP {continue}
                            *SETPOINT_TEMPERATURE.lock().await -= 1; 
                        },
                        _ => {  },
                    }
                } else {
                    // CW
                    rot_enc_pos = (rot_enc_pos + 1) % 360;
                    direction = RotaryEncoderDirection::CW;
                    fsm_state = fsm_state_rx.try_get().unwrap();
                    match fsm_state {
                        State::SetpointSelecting {  } => { 
                            let setpoint_temp = *SETPOINT_TEMPERATURE.lock().await;
                            if setpoint_temp <= MIN_TEMP || setpoint_temp >= MAX_TEMP {continue}
                            *SETPOINT_TEMPERATURE.lock().await += 1; 
                        },
                        _ => {  },
                    }
                }
                pressed = false;
            }
            Either3::Second(_) => {
                if encoder_a.is_low() {
                    // CW
                    rot_enc_pos = (rot_enc_pos + 1) % 360;
                    direction = RotaryEncoderDirection::CW;
                    fsm_state = fsm_state_rx.try_get().unwrap();
                    match fsm_state {
                        State::SetpointSelecting {  } => { 
                            let setpoint_temp = *SETPOINT_TEMPERATURE.lock().await;
                            if setpoint_temp <= MIN_TEMP || setpoint_temp >= MAX_TEMP {continue}
                            *SETPOINT_TEMPERATURE.lock().await += 1; 
                        },
                        _ => {  },
                    }
                } else {
                    // CCW
                    rot_enc_pos = (rot_enc_pos + 359) % 360;
                    direction = RotaryEncoderDirection::CCW;
                    fsm_state = fsm_state_rx.try_get().unwrap();
                    match fsm_state {
                        State::SetpointSelecting {  } => { 
                            let setpoint_temp = *SETPOINT_TEMPERATURE.lock().await;
                            if setpoint_temp <= MIN_TEMP || setpoint_temp >= MAX_TEMP {continue}
                            *SETPOINT_TEMPERATURE.lock().await -= 1; 
                        },
                        _ => {  },
                    }
                }
                pressed = false;
            }
            Either3::Third(_) => {
                if encoder_btn.is_high() { pressed = false; }
                else { pressed = true; }
                direction = RotaryEncoderDirection::Stationary;
                EVENT_QUEUE.send(Event::EncoderPressed).await;
            }
        }

        /* TODO: Might just change the rotary encoder to a mutex as this only fires on changes, 
        * which can be unreliable with unreliable hardware such as rotary encoder and button due
        * to bounce
        */
        publisher.publish_immediate(RotaryEncoder {
            position: rot_enc_pos,
            pressed: pressed,
            direction: direction,
        });
        info!("Encoder position: \x1B[32m{}\x1B[0m Button: \x1B[32m{}\x1B[0m", &rot_enc_pos, pressed);
    }
}

#[embassy_executor::task]
async fn task_zcd_detector(zcd_detector: &'static mut ExtiInput<'static, Async>) {
    loop {
        zcd_detector.wait_for_any_edge().await;
        info!("Zero Crossing Detected!");
    }
}

#[embassy_executor::task]
async fn display_task(bus: &'static I2c1Bus) {
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

    /* Buffers */
    let mut temperature_str_buffer = itoa::Buffer::new();
    let mut temperature: u32;

    /* Rotary Encoder subscriber */
    let mut rot_enc_subscriber = ROT_ENC_CHANNEL.subscriber().unwrap();
    let mut setpoint_target_temp: u32;
    let mut setpoint_target_temp_str_buffer = itoa::Buffer::new();
    let mut position: u32;
    let mut pressed: bool;
    let mut direction: RotaryEncoderDirection;

    /* FSM_STATE receiver */
    let mut fsm_state_rx = FSM_STATE.receiver().unwrap();
    let mut fsm_state: State;

    /* Selected UI Element initial value and local var  */
    *SELECTEDUIELEMENT.lock().await = SelectedUIElement::MenuReflow;
    let mut selected_element: SelectedUIElement;

    /* Local var for keeping selected reflow profile */
    let mut selected_reflow_profile: ReflowProfiles;

    loop {

        /* Task ticker */
        Timer::after_millis(10).await;

        /* Clear display ready for new data */
        display.clear();

        /* Retreive current state */
        fsm_state = fsm_state_rx.try_get().unwrap();

        /* Reset encoder vars awaiting next update */
        position = 0;
        direction = RotaryEncoderDirection::Stationary;
        pressed = false;

        /* Retreive encoder data */
        if let Some(msg) = rot_enc_subscriber.try_next_message_pure() {
            position = msg.position;
            direction = msg.direction;
            pressed = msg.pressed;
        }

        /* Retreive selected UI element */
        selected_element = *SELECTEDUIELEMENT.lock().await;

        /* Retreive selected reflow profile */
        selected_reflow_profile = *SELECTED_REFLOW_PROFILE.lock().await;

        match fsm_state {
            State::Menu {  } => {

                /* Send event if UI element has been pressed */
                if pressed {
                    match selected_element {
                        SelectedUIElement::MenuReflow => { 
                            EVENT_QUEUE.send(Event::ReflowSelected).await;
                            continue;
                        },
                        SelectedUIElement::MenuSetpoint => { 
                            EVENT_QUEUE.send(Event::SetpointSelected).await;
                            continue;
                        },
                        SelectedUIElement::MenuMeasure => { 
                            EVENT_QUEUE.send(Event::MeasureSelected).await;
                            continue;
                        },
                        _ => { panic!("Display_task, State::Menu, if pressed, match selected_element") },
                    }
                }

                /* Move cursor based on encoder direction */
                if direction != RotaryEncoderDirection::Stationary {
                    match selected_element {
                        SelectedUIElement::MenuReflow => {
                            if direction == RotaryEncoderDirection::CW {selected_element = SelectedUIElement::MenuSetpoint;}
                            else {selected_element = SelectedUIElement::MenuMeasure}
                        },

                        SelectedUIElement::MenuSetpoint => {
                            if direction == RotaryEncoderDirection::CW {selected_element = SelectedUIElement::MenuMeasure;}
                            else {selected_element = SelectedUIElement::MenuReflow}
                        },

                        SelectedUIElement::MenuMeasure => {
                            if direction == RotaryEncoderDirection::CW {selected_element = SelectedUIElement::MenuReflow;}
                            else {selected_element = SelectedUIElement::MenuSetpoint}
                        },

                        _ => { panic!("Display_task, match fsm_state = Menu, match selected_element") },
                    }
                }
                
                /* Display elements */
                Text::with_baseline("Reflow", Point::new(10, 2), TEXT_STYLE_SMALL, Baseline::Top)
                    .draw(&mut display)
                    .unwrap();

                Text::with_baseline("Setpoint", Point::new(10, 16), TEXT_STYLE_SMALL, Baseline::Top)
                    .draw(&mut display)
                    .unwrap();

                Text::with_baseline("Measure", Point::new(10, 30), TEXT_STYLE_SMALL, Baseline::Top)
                    .draw(&mut display)
                    .unwrap();

                Rectangle::new(Point::new(0, HEIGHT as i32 - 12) , Size::new(WIDTH as u32, 12))
                    .into_styled(RECT_STYLE)
                    .draw(&mut display)
                    .unwrap();

                Text::with_alignment("Menu", Point { x: (WIDTH as i32 - 2), y: (HEIGHT as i32 - 12) }, TEXT_STYLE_SMALL_KNOCKOUT, Alignment::Right)
                    .draw(&mut display)
                    .unwrap();

                /* Display cursor depending on which UI element is selected */
                match selected_element {
                    SelectedUIElement::MenuReflow => {
                        Line::new(Point { x: (2), y: (3) }, Point { x: (6), y: (7) })
                            .into_styled(LINE_STYLE)
                            .draw(&mut display)
                            .unwrap();
        
                        Line::new(Point { x: (2), y: (11) }, Point { x: (6), y: (7) })
                            .into_styled(LINE_STYLE)
                            .draw(&mut display)
                            .unwrap();
                    }

                    SelectedUIElement::MenuSetpoint => {
                        Line::new(Point { x: (2), y: (17) }, Point { x: (6), y: (21) })
                            .into_styled(LINE_STYLE)
                            .draw(&mut display)
                            .unwrap();
        
                        Line::new(Point { x: (2), y: (25) }, Point { x: (6), y: (21) })
                            .into_styled(LINE_STYLE)
                            .draw(&mut display)
                            .unwrap();
                    }

                    SelectedUIElement::MenuMeasure => {
                        Line::new(Point { x: (2), y: (31) }, Point { x: (6), y: (35) })
                            .into_styled(LINE_STYLE)
                            .draw(&mut display)
                            .unwrap();
        
                        Line::new(Point { x: (2), y: (39) }, Point { x: (6), y: (35) })
                            .into_styled(LINE_STYLE)
                            .draw(&mut display)
                            .unwrap();
                    }

                    _ => { panic!("Display_task, match selected_element, cursor draw") }
                }
            }

            State::Reflow {  } => {     

                /* Read the temperature mutex and format it into a string */
                temperature = *TEMPERATURE.lock().await as u32;
                let temperature_str = temperature_str_buffer.format(temperature);
                let mut temperature_str_concat: String<10> = String::new();
                write!(&mut temperature_str_concat, "{temperature_str}°C").unwrap();

                /* Send event if UI element has been pressed */
                if pressed {
                    match selected_element {
                        SelectedUIElement::ReflowStart => { 
                            EVENT_QUEUE.send(Event::ReflowStartSelected).await;
                            continue;
                        },
                        SelectedUIElement::ReflowProfileSelectorMenu => { 
                            EVENT_QUEUE.send(Event::ReflowProfileSelectorSelected).await;
                            continue;
                        },
                        SelectedUIElement::ReflowMenu => { 
                            EVENT_QUEUE.send(Event::ReflowMenuSelected).await;
                            continue;
                        },

                        _ => { panic!("Display_task, State::Reflow, if pressed, match selected_element") },
                    }
                }

                /* Move cursor based on encoder direction */
                if direction != RotaryEncoderDirection::Stationary {
                    match selected_element {
                        SelectedUIElement::ReflowStart => {
                            if direction == RotaryEncoderDirection::CW {selected_element = SelectedUIElement::ReflowProfileSelectorMenu;}
                            else {selected_element = SelectedUIElement::ReflowMenu}
                        },

                        SelectedUIElement::ReflowProfileSelectorMenu => {
                            if direction == RotaryEncoderDirection::CW {selected_element = SelectedUIElement::ReflowMenu;}
                            else {selected_element = SelectedUIElement::ReflowStart}
                        },

                        SelectedUIElement::ReflowMenu => {
                            if direction == RotaryEncoderDirection::CW {selected_element = SelectedUIElement::ReflowStart;}
                            else {selected_element = SelectedUIElement::ReflowProfileSelectorMenu}
                        },

                        _ => { panic!("Display_task, match fsm_state = reflow, match selected_element") },
                    }
                }

                /* Display elements */
                Text::with_baseline("Start", Point::new(10, 2), TEXT_STYLE_SMALL, Baseline::Top)
                    .draw(&mut display)
                    .unwrap();

                Text::with_baseline("Back", Point::new(10, 16), TEXT_STYLE_SMALL, Baseline::Top)
                    .draw(&mut display)
                    .unwrap();

                Line::new(Point { x: (47), y: (5) }, Point { x: (47), y: (HEIGHT as i32 - 17) })
                    .into_styled(LINE_STYLE)
                    .draw(&mut display)
                    .unwrap();

                Text::with_baseline("Duration:", Point::new(55, 2), TEXT_STYLE_SMALL, Baseline::Top)
                    .draw(&mut display)
                    .unwrap();

                /* TODO: Display actual duration of reflow profile, taken from profile struct */
                Text::with_baseline("10m30s", Point::new(55, 14), TEXT_STYLE_SMALL, Baseline::Top)
                    .draw(&mut display)
                    .unwrap();

                Text::with_baseline("Max Temp.:", Point::new(55, 26), TEXT_STYLE_SMALL, Baseline::Top)
                    .draw(&mut display)
                    .unwrap();

                /* TODO: Display actual max temp. of reflow profile, taken from profile struct */
                Text::with_baseline("250°C", Point::new(55, 38), TEXT_STYLE_SMALL, Baseline::Top)
                    .draw(&mut display)
                    .unwrap();

                Rectangle::new(Point::new(0, HEIGHT as i32 - 12) , Size::new(WIDTH as u32, 12))
                    .into_styled(RECT_STYLE)
                    .draw(&mut display)
                    .unwrap();

                Text::with_alignment("Reflow", Point { x: (WIDTH as i32 - 2), y: (HEIGHT as i32 - 12) }, TEXT_STYLE_SMALL_KNOCKOUT, Alignment::Right)
                    .draw(&mut display)
                    .unwrap();

                match selected_reflow_profile {
                    ReflowProfiles::TS391SNL => {
                        Text::with_alignment("(TS391SNL)", Point { x: (2), y: (HEIGHT as i32 - 12) }, TEXT_STYLE_SMALL_KNOCKOUT, Alignment::Left)
                            .draw(&mut display)
                            .unwrap();
                    }

                    ReflowProfiles::GC10 => {
                        Text::with_alignment("(GC10)", Point { x: (2), y: (HEIGHT as i32 - 12) }, TEXT_STYLE_SMALL_KNOCKOUT, Alignment::Left)
                            .draw(&mut display)
                            .unwrap();
                    }

                    _ => {
                        Text::with_alignment("(None)", Point { x: (2), y: (HEIGHT as i32 - 12) }, TEXT_STYLE_SMALL_KNOCKOUT, Alignment::Left)
                            .draw(&mut display)
                            .unwrap();
                    }
                }

                /* Display cursor depending on which UI element is selected */
                match selected_element {
                    SelectedUIElement::ReflowStart => {
                        Line::new(Point { x: (2), y: (3) }, Point { x: (6), y: (7) })
                            .into_styled(LINE_STYLE)
                            .draw(&mut display)
                            .unwrap();
        
                        Line::new(Point { x: (2), y: (11) }, Point { x: (6), y: (7) })
                            .into_styled(LINE_STYLE)
                            .draw(&mut display)
                            .unwrap();
                    }

                    SelectedUIElement::ReflowProfileSelectorMenu => {
                        Line::new(Point { x: (2), y: (17) }, Point { x: (6), y: (21) })
                            .into_styled(LINE_STYLE)
                            .draw(&mut display)
                            .unwrap();
        
                        Line::new(Point { x: (2), y: (25) }, Point { x: (6), y: (21) })
                            .into_styled(LINE_STYLE)
                            .draw(&mut display)
                            .unwrap();
                    }


                    SelectedUIElement::ReflowMenu => {
                        Line::new(Point { x: (WIDTH as i32 - 40), y: (HEIGHT as i32 - 10)}, Point { x: (WIDTH as i32 - 36), y: (HEIGHT as i32 - 6)})
                            .into_styled(LINE_STYLE_KNOCKOUT)
                            .draw(&mut display)
                            .unwrap();
        
                        Line::new(Point { x: (WIDTH as i32 - 40), y: (HEIGHT as i32 - 2)}, Point { x: (WIDTH as i32 - 36), y: (HEIGHT as i32 - 6)})
                            .into_styled(LINE_STYLE_KNOCKOUT)
                            .draw(&mut display)
                            .unwrap();
                    }

                    _ => { panic!("Display_task, state reflow, match selected_element, cursor draw") }
                }
            }

            State::ReflowProfileSelection {  } => {

                /* Send event if UI element has been pressed */
                if pressed {
                    match selected_element {
                        SelectedUIElement::ReflowProfile1 => { 
                            *SELECTED_REFLOW_PROFILE.lock().await = ReflowProfiles::TS391SNL;
                            EVENT_QUEUE.send(Event::ReflowSelected).await;
                            continue;
                        }

                        SelectedUIElement::ReflowProfile2 => {
                            *SELECTED_REFLOW_PROFILE.lock().await = ReflowProfiles::GC10;
                            EVENT_QUEUE.send(Event::ReflowSelected).await;
                            continue;
                        }

                        SelectedUIElement::ReflowProfileMenu => {
                            EVENT_QUEUE.send(Event::MenuSelected).await;
                            continue;
                        }
    
                        _ => { panic!("Display_task, state reflowprofileselection, match selected_element") }
                    }
                }

                /* Move cursor based on encoder direction */
                if direction != RotaryEncoderDirection::Stationary {
                    match selected_element {
                        SelectedUIElement::ReflowProfile1 => {
                            if direction == RotaryEncoderDirection::CW { selected_element = SelectedUIElement::ReflowProfile2; }
                            else { selected_element = SelectedUIElement::ReflowProfileMenu }
                        },

                        SelectedUIElement::ReflowProfile2 => {
                            if direction == RotaryEncoderDirection::CW { selected_element = SelectedUIElement::ReflowProfileMenu; }
                            else { selected_element = SelectedUIElement::ReflowProfile1; }
                        },

                        SelectedUIElement::ReflowProfileMenu => {
                            if direction == RotaryEncoderDirection::CW { selected_element = SelectedUIElement::ReflowProfile1; }
                            else { selected_element = SelectedUIElement::ReflowProfile2; }
                        }

                        _ => { panic!("Display_task, state reflow_profile_selection, match fsm_state = Menu, match selected_element") },
                    }
                }

                Text::with_baseline("TS319SNL (RoHS)", Point::new(10, 2), TEXT_STYLE_SMALL, Baseline::Top)
                    .draw(&mut display)
                    .unwrap();

                Text::with_baseline("GC10 (RoHS)", Point::new(10, 16), TEXT_STYLE_SMALL, Baseline::Top)
                    .draw(&mut display)
                    .unwrap();

                Rectangle::new(Point::new(0, HEIGHT as i32 - 12) , Size::new(WIDTH as u32, 12))
                    .into_styled(RECT_STYLE)
                    .draw(&mut display)
                    .unwrap();

                Text::with_alignment("Select Profile", Point { x: (WIDTH as i32 - 2), y: (HEIGHT as i32 - 12) }, TEXT_STYLE_SMALL_KNOCKOUT, Alignment::Right)
                    .draw(&mut display)
                    .unwrap();

                /* Display cursor depending on which UI element is selected */
                match selected_element {
                    SelectedUIElement::ReflowProfile1 => {
                        Line::new(Point { x: (2), y: (3) }, Point { x: (6), y: (7) })
                            .into_styled(LINE_STYLE)
                            .draw(&mut display)
                            .unwrap();
        
                        Line::new(Point { x: (2), y: (11) }, Point { x: (6), y: (7) })
                            .into_styled(LINE_STYLE)
                            .draw(&mut display)
                            .unwrap();
                    }

                    SelectedUIElement::ReflowProfile2 => {
                        Line::new(Point { x: (2), y: (17) }, Point { x: (6), y: (21) })
                            .into_styled(LINE_STYLE)
                            .draw(&mut display)
                            .unwrap();
        
                        Line::new(Point { x: (2), y: (25) }, Point { x: (6), y: (21) })
                            .into_styled(LINE_STYLE)
                            .draw(&mut display)
                            .unwrap();
                    }

                    SelectedUIElement::ReflowProfileMenu => {
                        Line::new(Point { x: (35), y: (HEIGHT as i32 - 10)}, Point { x: (39), y: (HEIGHT as i32 - 6)})
                            .into_styled(LINE_STYLE_KNOCKOUT)
                            .draw(&mut display)
                            .unwrap();
        
                        Line::new(Point { x: (35), y: (HEIGHT as i32 - 2)}, Point { x: (39), y: (HEIGHT as i32 - 6)})
                            .into_styled(LINE_STYLE_KNOCKOUT)
                            .draw(&mut display)
                            .unwrap();
                    }
                    _ => { panic!("Display_task, match selected_element, cursor draw") }
                }
            }

            State::ReflowRunning {  } => {

                /* Send event if UI element has been pressed */
                if pressed {
                    match selected_element {
                        SelectedUIElement::ReflowStop => { 
                            EVENT_QUEUE.send(Event::ReflowStopSelected).await;
                            continue;
                        }
    
                        _ => { panic!("Display_task, state reflow_running, match selected_element") }
                    }
                }

                Text::with_baseline("Stop", Point::new(10, 2), TEXT_STYLE_SMALL, Baseline::Top)
                    .draw(&mut display)
                    .unwrap();

                Rectangle::new(Point::new(0, HEIGHT as i32 - 12) , Size::new(WIDTH as u32, 12))
                    .into_styled(RECT_STYLE)
                    .draw(&mut display)
                    .unwrap();

                Text::with_alignment("Reflow", Point { x: (WIDTH as i32 - 2), y: (HEIGHT as i32 - 12) }, TEXT_STYLE_SMALL_KNOCKOUT, Alignment::Right)
                    .draw(&mut display)
                    .unwrap();

                match selected_reflow_profile {
                    ReflowProfiles::TS391SNL => {
                        Text::with_alignment("(TS391SNL)", Point { x: (2), y: (HEIGHT as i32 - 12) }, TEXT_STYLE_SMALL_KNOCKOUT, Alignment::Left)
                            .draw(&mut display)
                            .unwrap();
                    }

                    ReflowProfiles::GC10 => {
                        Text::with_alignment("(GC10)", Point { x: (2), y: (HEIGHT as i32 - 12) }, TEXT_STYLE_SMALL_KNOCKOUT, Alignment::Left)
                            .draw(&mut display)
                            .unwrap();
                    }

                    _ => {
                        Text::with_alignment("(None)", Point { x: (2), y: (HEIGHT as i32 - 12) }, TEXT_STYLE_SMALL_KNOCKOUT, Alignment::Left)
                            .draw(&mut display)
                            .unwrap();
                    }
                }

                /* Display cursor depending on which UI element is selected */
                match selected_element {
                    SelectedUIElement::ReflowStop => {
                        Line::new(Point { x: (2), y: (3) }, Point { x: (6), y: (7) })
                            .into_styled(LINE_STYLE)
                            .draw(&mut display)
                            .unwrap();
        
                        Line::new(Point { x: (2), y: (11) }, Point { x: (6), y: (7) })
                            .into_styled(LINE_STYLE)
                            .draw(&mut display)
                            .unwrap();
                    }
                    _ => { panic!("Display_task, state reflow_running, match selected_element, cursor draw") }
                }
            }

            State::Setpoint {  } | State::SetpointRunning {  } => {
                /* Read the temperature mutex and format it into a string */
                temperature = *TEMPERATURE.lock().await as u32;
                let temperature_str = temperature_str_buffer.format(temperature);
                let mut temperature_str_concat: String<10> = String::new();
                write!(&mut temperature_str_concat, "{temperature_str}°C").unwrap();
                
                /* Read the setpoint target temperature mutex and format it into a string */
                setpoint_target_temp = *SETPOINT_TEMPERATURE.lock().await;
                let setpoint_target_temp_str = setpoint_target_temp_str_buffer.format(setpoint_target_temp);
                let mut setpoint_target_temp_str_concat: String<10> = String::new();
                write!(&mut setpoint_target_temp_str_concat, "{setpoint_target_temp_str}°C").unwrap();

                /* Send event if UI element has been pressed */
                if pressed {
                    match selected_element {
                        SelectedUIElement::SetpointTemperatureRollerInactive => { 
                            EVENT_QUEUE.send(Event::SetpointTemperatureRollerSelected).await;
                            continue;
                        },
                        SelectedUIElement::SetpointMenu => { 
                            EVENT_QUEUE.send(Event::SetpointMenuSelected).await;
                            continue;
                        },
                        _ => { panic!("Display_task, state setpoint, if pressed, match selected_element") },
                    }
                }

                /* Move cursor based on encoder direction */
                if direction != RotaryEncoderDirection::Stationary {
                    match selected_element {
                        SelectedUIElement::SetpointTemperatureRollerInactive => {
                            if direction == RotaryEncoderDirection::CW {selected_element = SelectedUIElement::SetpointMenu;}
                            else {selected_element = SelectedUIElement::SetpointMenu}
                        },

                        SelectedUIElement::SetpointMenu => {
                            if direction == RotaryEncoderDirection::CW {selected_element = SelectedUIElement::SetpointTemperatureRollerInactive;}
                            else {selected_element = SelectedUIElement::SetpointTemperatureRollerInactive}
                        },

                        _ => { panic!("Display_task, state reflow, match selected_element") },
                    }
                }
                
                /* Display elements */
                Text::with_alignment("Target:", Point { x: (2), y: (12) }, TEXT_STYLE_SMALL, Alignment::Left)
                    .draw(&mut display)
                    .unwrap();

                Text::with_alignment(setpoint_target_temp_str, Point { x: (56), y: (2) }, TEXT_STYLE_MEDIUM, Alignment::Left)
                    .draw(&mut display)
                    .unwrap();

                Text::with_alignment("Current:", Point { x: (2), y: (36) }, TEXT_STYLE_SMALL, Alignment::Left)
                    .draw(&mut display)
                    .unwrap();

                Text::with_alignment(&temperature_str_concat, Point { x: (56), y: (26) }, TEXT_STYLE_MEDIUM, Alignment::Left)
                    .draw(&mut display)
                    .unwrap();

                Rectangle::new(Point::new(0, HEIGHT as i32 - 12) , Size::new(WIDTH as u32, 12))
                    .into_styled(RECT_STYLE)
                    .draw(&mut display)
                    .unwrap();

                Text::with_alignment("Setpoint", Point { x: (WIDTH as i32 - 2), y: (HEIGHT as i32 - 12) }, TEXT_STYLE_SMALL_KNOCKOUT, Alignment::Right)
                    .draw(&mut display)
                    .unwrap();

                /* Display cursor depending on which UI element is selected */
                match selected_element {
                    SelectedUIElement::SetpointTemperatureRollerInactive => {
                        Line::new(Point { x: (94), y: (14) }, Point { x: (98), y: (10) })
                            .into_styled(LINE_STYLE)
                            .draw(&mut display)
                            .unwrap();
        
                        Line::new(Point { x: (94), y: (14) }, Point { x: (98), y: (18) })
                            .into_styled(LINE_STYLE)
                            .draw(&mut display)
                            .unwrap();
                    }

                    SelectedUIElement::SetpointMenu => {
                        Line::new(Point { x: (WIDTH as i32 - 58), y: (HEIGHT as i32 - 10)}, Point { x: (WIDTH as i32 - 54), y: (HEIGHT as i32 - 6)})
                            .into_styled(LINE_STYLE_KNOCKOUT)
                            .draw(&mut display)
                            .unwrap();
        
                        Line::new(Point { x: (WIDTH as i32 - 58), y: (HEIGHT as i32 - 2)}, Point { x: (WIDTH as i32 - 54), y: (HEIGHT as i32 - 6)})
                            .into_styled(LINE_STYLE_KNOCKOUT)
                            .draw(&mut display)
                            .unwrap();
                    }
                    _ => { panic!("Display_task, state setpoint, match selected_element, cursor draw") }
                }
            }

            State::SetpointSelecting {  } => {
                /* Read the temperature mutex and format it into a string */
                temperature = *TEMPERATURE.lock().await as u32;
                let temperature_str = temperature_str_buffer.format(temperature);
                let mut temperature_str_concat: String<10> = String::new();
                write!(&mut temperature_str_concat, "{temperature_str}°C").unwrap();
                
                /* Read the setpoint target temperature mutex and format it into a string */
                setpoint_target_temp = *SETPOINT_TEMPERATURE.lock().await;
                let setpoint_target_temp_str = setpoint_target_temp_str_buffer.format(setpoint_target_temp);
                let mut setpoint_target_temp_str_concat: String<10> = String::new();
                write!(&mut setpoint_target_temp_str_concat, "{setpoint_target_temp_str}°C").unwrap();

                /* Send event if UI element has been pressed */
                if pressed {
                    match selected_element {
                        SelectedUIElement::SetpointTemperatureRollerActive => { 
                            let setpoint_temp = *SETPOINT_TEMPERATURE.lock().await;
                            if setpoint_temp > MIN_TEMP {
                                EVENT_QUEUE.send(Event::SetpointTemperatureSet).await;

                            }
                            else {
                                EVENT_QUEUE.send(Event::SetpointTemperatureUnset).await;
                            }
                            continue;
                        },
                        _ => { panic!("Display_task, state setpoint, if pressed, match selected_element") },
                    }
                }
                
                /* Display elements */
                Text::with_alignment("Target:", Point { x: (2), y: (12) }, TEXT_STYLE_SMALL, Alignment::Left)
                    .draw(&mut display)
                    .unwrap();

                Text::with_alignment(setpoint_target_temp_str, Point { x: (56), y: (2) }, TEXT_STYLE_MEDIUM, Alignment::Left)
                    .draw(&mut display)
                    .unwrap();

                Text::with_alignment("Current:", Point { x: (2), y: (36) }, TEXT_STYLE_SMALL, Alignment::Left)
                    .draw(&mut display)
                    .unwrap();

                Text::with_alignment(&temperature_str_concat, Point { x: (56), y: (26) }, TEXT_STYLE_MEDIUM, Alignment::Left)
                    .draw(&mut display)
                    .unwrap();

                Rectangle::new(Point::new(0, HEIGHT as i32 - 12) , Size::new(WIDTH as u32, 12))
                    .into_styled(RECT_STYLE)
                    .draw(&mut display)
                    .unwrap();

                Text::with_alignment("Setpoint", Point { x: (WIDTH as i32 - 2), y: (HEIGHT as i32 - 12) }, TEXT_STYLE_SMALL_KNOCKOUT, Alignment::Right)
                    .draw(&mut display)
                    .unwrap();

                /* Display cursor depending on which UI element is selected */
                match selected_element {
                    SelectedUIElement::SetpointTemperatureRollerActive => {
                        Triangle::new(Point { x: (94), y: (14)}, Point { x: (98), y: (10)}, Point { x: (98), y: (18) })
                            .into_styled(TRI_STYLE)
                            .draw(&mut display)
                            .unwrap();
                    }
                    _ => { panic!("Display_task, state setpoint_selecting, match selected_element, cursor draw") }
                }
            }

            State::Measure {  } => {

                /* Read the temperature mutex and format it into a string */
                temperature = *TEMPERATURE.lock().await as u32;
                let temperature_str = temperature_str_buffer.format(temperature);
                let mut temperature_str_concat: String<10> = String::new();
                write!(&mut temperature_str_concat, "{temperature_str}°C").unwrap();

                /* Send event if UI element has been pressed */
                if pressed {
                    match selected_element {
                        SelectedUIElement::MeasureMenu => { 
                            EVENT_QUEUE.send(Event::MenuSelected).await;
                            continue;
                        },
                        _ => { panic!("Display_task, state measure, if pressed, match selected_element") },
                    }
                }

                /* Display elements */
                Text::with_alignment(&temperature_str_concat, Point { x: (WIDTH as i32 + 10), y: (HEIGHT as i32 / 2 - 24) }, TEXT_STYLE_LARGE, Alignment::Right)
                    .draw(&mut display)
                    .unwrap();

                Rectangle::new(Point::new(0, HEIGHT as i32 - 12) , Size::new(WIDTH as u32, 12))
                    .into_styled(RECT_STYLE)
                    .draw(&mut display)
                    .unwrap();

                Text::with_alignment("Measure", Point { x: (WIDTH as i32 - 2), y: (HEIGHT as i32 - 12) }, TEXT_STYLE_SMALL_KNOCKOUT, Alignment::Right)
                    .draw(&mut display)
                    .unwrap();

                /* Display cursor depending on which UI element is selected */
                match selected_element {
                    SelectedUIElement::MeasureMenu => {
                        Line::new(Point { x: (WIDTH as i32 - 52), y: (HEIGHT as i32 - 10)}, Point { x: (WIDTH as i32 - 48), y: (HEIGHT as i32 - 6)})
                            .into_styled(LINE_STYLE_KNOCKOUT)
                            .draw(&mut display)
                            .unwrap();
        
                        Line::new(Point { x: (WIDTH as i32 - 52), y: (HEIGHT as i32 - 2)}, Point { x: (WIDTH as i32 - 48), y: (HEIGHT as i32 - 6)})
                            .into_styled(LINE_STYLE_KNOCKOUT)
                            .draw(&mut display)
                            .unwrap();
                    }
                    _ => { panic!("Display_task, state reflow_running, match selected_element, cursor draw") }
                }
            }

            State::Error {  } => {
                /* TODO: Add safe shutdown of all heating */
                warn!("Error State");
                Text::with_alignment("ERROR", Point { x: (20), y: (20) }, TEXT_STYLE_MEDIUM, Alignment::Right)
                    .draw(&mut display)
                    .unwrap();
            }
        }

        *SELECTEDUIELEMENT.lock().await = selected_element;

        /* Flush to display */
        display.flush().await.unwrap();
    }
}