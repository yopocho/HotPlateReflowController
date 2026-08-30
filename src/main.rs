#![no_std]
#![no_main]
#![allow(unused_unsafe)]

/** INCLUDES BEGIN **/
/* RTT Logging */
use defmt::{debug, error, info, warn};
/* Exception handling */
use {defmt_rtt as _, panic_probe as _};
/* Embassy framework */
use embassy_executor::Spawner;
use embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice;
use embassy_stm32::time::Hertz;
use embassy_stm32::bind_interrupts;
use embassy_stm32::exti::ExtiInput;
use embassy_stm32::peripherals::{self};
use embassy_stm32::dma::InterruptHandler;
use embassy_stm32::mode::Async;
use embassy_sync::watch::Watch;
use embassy_sync::mutex::Mutex;
use embassy_sync::pubsub::PubSubChannel;
use embassy_sync::channel::Channel;
use embedded_hal::Pwm;
use static_cell::StaticCell;
use embassy_stm32::gpio::{
    Level, 
    Output, 
    Speed, 
    Pull
};
use embassy_stm32::interrupt::typelevel::{
    /* ZCD currently unused but will be left uncommented here for reference */
    // EXTI2_3, 
    EXTI4_15
};
use embassy_stm32::i2c::{
    Config as i2cConfig, 
    I2c, 
    self
};
use embassy_stm32::pac::{
    self, 
    syscfg::vals::Pinmux2
};
use embassy_stm32::rcc::{
    Hsi, 
    HsiSysDiv, 
    HsiKerDiv
};
use embassy_stm32::spi::{
    Config as spiConfig, 
    Spi, 
    mode::Master, 
    Phase::CaptureOnFirstTransition, 
    Polarity::IdleLow
};
use embassy_stm32::timer::{
    simple_pwm::{
        PwmPin, 
        SimplePwm
    }, 
    Channel::{
        Ch2, 
        Ch3
    }
};
use embassy_time::{
    Timer, 
    Instant
};
use embassy_futures::select::{
    select3, 
    Either3
};
use embassy_sync::blocking_mutex::raw::{
    CriticalSectionRawMutex, 
    ThreadModeRawMutex
};
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
/* INA219 */
use ina219::{address::Address, configuration::{
        BusVoltageRange, Configuration, MeasuredSignals, OperatingMode, Reset, Resolution, ShuntVoltageRange
        }, measurements::{ Measurements}, AsyncIna219};
/* FSM */
use statig::prelude::*;
/* PID */
use pid::Pid;
/* Local */
mod reflow_profiles;
use crate::reflow_profiles::ReflowProfiles;
mod rotary_encoder;
use crate::rotary_encoder as encoder;
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
const PID_KI_LIMIT: f32 = 38000.0;
/* PID Temperature dead-zone */
const PID_TEMP_DEADZONE: f32 = 3.0; // °C
/* Setpoint start temperature */
const SETPOINT_START_TEMPERATURE: u32 = 20;
/** CONSTANTS END **/

/** STRUCTS BEGIN **/
pub struct ReflowParams {
    preheat_ramp: f32,
    soak_ramp: f32,
    reflow_ramp: f32,
    cool_ramp: f32,
}
/** STRUCTS END **/


/** EMBASSY_SYNC DECLARATIONS BEGIN **/
/* Declare mutex for i2c bus */
type I2c1Bus = Mutex<ThreadModeRawMutex, I2c<'static, Async, i2c::Master>>;
static I2C_BUS: StaticCell<I2c1Bus> = StaticCell::new();
/* Declare mutex for sharing thermocouple data */
static TEMPERATURE: Mutex<ThreadModeRawMutex, f32> = Mutex::new(0.0);
/* Declare mutex for sharing triac power percentage */
static TRIAC_PWR: Mutex<ThreadModeRawMutex, u8> = Mutex::new(0);
/* Declare mutex for static setpoint temperature */
static SETPOINT_TEMPERATURE: Mutex<ThreadModeRawMutex, u32> = Mutex::new(SETPOINT_START_TEMPERATURE);
/* Declare mutex for reflow target temperature */
static REFLOW_TARGET_TEMPERATURE: Mutex<ThreadModeRawMutex, u32> = Mutex::new(0);
/* Declare mutex for tracking start time of reflow */
static REFLOW_START_TIME: Mutex<ThreadModeRawMutex, Instant> = Mutex::new(Instant::MIN);
/* Declare mutex for tracking starting temp of hot plate during reflow */
static REFLOW_START_TEMP: Mutex<ThreadModeRawMutex, u32> = Mutex::new(0);
/* Declare mutex for tracking reflow time for current phase */
static REFLOW_PHASE_ELAPSED_STEPS: Mutex<ThreadModeRawMutex, u32> = Mutex::new(0);
/* Declare mutex for selected reflow profile */
static SELECTED_REFLOW_PROFILE: Mutex<ThreadModeRawMutex, ReflowProfiles> = Mutex::new(ReflowProfiles::NoProfile);
/* Declare mutex for holding parsed reflow profile parameters */
static REFLOW_PARAMETERS: Mutex<ThreadModeRawMutex, ReflowParams> = Mutex::new(ReflowParams { preheat_ramp: (0.0), soak_ramp: (0.0), reflow_ramp: (0.0), cool_ramp: (0.0) });
/* Declare Pub/Sub-Channel for rotary encoder data */
static ROT_ENC_CHANNEL: PubSubChannel<ThreadModeRawMutex, RotaryEncoder, 1, 4, 1> = PubSubChannel::new();
/* Watch channel for FSM state */
static FSM_STATE: Watch<CriticalSectionRawMutex, State, 10> = Watch::new();
/* FSM Event Queue */
static EVENT_QUEUE: Channel<CriticalSectionRawMutex, Event, 10> = Channel::new();
/* Declare Mutex for currently selected UI element */
static SELECTEDUIELEMENT: Mutex<ThreadModeRawMutex, SelectedUIElement> = Mutex::new(SelectedUIElement::NoneSelected);
/* Declare Mutex for holding the current error */
static CURRENT_ERROR: Mutex<ThreadModeRawMutex, ErrorType> = Mutex::new(ErrorType::NoErrors);
/** EMBASSY_SYNC DECLARATIONS END **/


/** STATIC IRQ PINS BEGIN **/
/* Static encoder GPIOs */
static ENCODER_A_INPUT: StaticCell<ExtiInput<Async>> = StaticCell::new();
static ENCODER_B_INPUT: StaticCell<ExtiInput<Async>> = StaticCell::new();
static ENCODER_BTN_INPUT: StaticCell<ExtiInput<Async>> = StaticCell::new();
/* ZCD currently unused but will be left uncommented here for reference */
// static _ZCD_DETECT: StaticCell<ExtiInput<Async>> = StaticCell::new();
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
    ReflowPhasePreheatDone,
    ReflowPhaseSoakDone,
    ReflowPhaseReflowDone,
    ReflowPhaseCoolDone,
    ReflowCompleteConfirmed,
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
    NoErrors = 0x00,
    // Recoverable
    ThermocoupleShortGnd = 0x01,
    ThermocoupleShortVcc = 0x02,
    ThermocoupleIssue = 0x03,
    NoThermocouple = 0x04,
    NoZCD = 0x05,
    Overtemp = 0x06,
    // Unrecoverable
    NoHeat = 0xF0,
    NoDisplay = 0xF1,
    NoTransformer = 0xF2,
    NoInaTransformer = 0xF3,
    NoFan = 0xF4,
    NoInaFan = 0xF5,
    NoMax = 0xF6,
    NoEncoder = 0xF7,
    Overcurrent = 0xF8,
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
    ReflowCompleteConfirmation,
    MeasureMenu,
    RecoverableErrorConfirmation,
    NoneSelected,
}
/** ENUMS END **/


/** FSM MACRO DEFINITION BEGIN **/
pub struct HPRC;
#[state_machine(
    initial = "State::menu()", 
    after_transition = "Self::after_transition",
    state(derive(Debug, Clone, PartialEq, Eq)),
)]
impl HPRC {
    #[superstate]
    async fn issue(event: &Event) -> Outcome<State> {
        match event {
            Event::Error(error) => {
                if (*error as usize) >= 0xF0 { // Unrecoverable error
                    *CURRENT_ERROR.lock().await = *error;
                    error!("Error: {:#04X}", *error as usize);
                    Transition(State::unrecoverable_error())
                }
                else if (*error as usize) > 0 && (*error as usize) < 0xF0 { // Recoverable error
                    *CURRENT_ERROR.lock().await = *error;
                    *SELECTEDUIELEMENT.lock().await = SelectedUIElement::RecoverableErrorConfirmation;
                    error!("Error: {:#04X}", *error as usize);
                    Transition(State::recoverable_error())

                }
                else { // NoErrors
                    *CURRENT_ERROR.lock().await = *error;
                    *SELECTEDUIELEMENT.lock().await = SelectedUIElement::MenuReflow;
                    error!("Error: {:#04X}", *error as usize);
                    Transition(State::menu())
                }
            },
            _ => Super,
        }
    }

    #[state(superstate = "issue")]
    async fn recoverable_error(event: &Event) -> Outcome<State> {
        match event {
            Event::Error(ErrorType::NoErrors) => {
                *SELECTEDUIELEMENT.lock().await = SelectedUIElement::MenuReflow;
                Transition(State::menu())
            },
            _ => Super,
        }
    }

    #[state(superstate = "issue")]
    async fn unrecoverable_error(event: &Event) -> Outcome<State> {
        /* Unrecoverable error, no state transition possible */
        Super
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
                *SETPOINT_TEMPERATURE.lock().await = SETPOINT_START_TEMPERATURE;
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
    async fn reflow(event: &Event) -> Outcome<State> {
        match event {
            Event::ReflowStartSelected => {
                let selected_reflow_profile = *SELECTED_REFLOW_PROFILE.lock().await;
                let temp_reflow_parameters: ReflowParams = ReflowParams { 
                    preheat_ramp: (selected_reflow_profile.profile().preheat_temp as f32 / selected_reflow_profile.profile().preheat_time as f32 / 10.0), 
                    soak_ramp: (selected_reflow_profile.profile().soak_temp as f32 / selected_reflow_profile.profile().soak_time as f32 / 10.0), 
                    reflow_ramp: (selected_reflow_profile.profile().reflow_temp as f32 / selected_reflow_profile.profile().reflow_time as f32 / 10.0), 
                    cool_ramp: (selected_reflow_profile.profile().cool_temp as f32 / selected_reflow_profile.profile().cool_time as f32 / 10.0) };
                *REFLOW_PARAMETERS.lock().await =  temp_reflow_parameters;
                *REFLOW_START_TIME.lock().await = Instant::now();
                *REFLOW_PHASE_ELAPSED_STEPS.lock().await = 0;
                *SELECTEDUIELEMENT.lock().await = SelectedUIElement::ReflowStop;
                Transition(State::reflow_phase_preheat())
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
    async fn reflow_phase_preheat(event: &Event) -> Outcome<State> {
        match event {
            Event::ReflowStopSelected => {
                *REFLOW_START_TIME.lock().await = Instant::MIN;
                *REFLOW_PHASE_ELAPSED_STEPS.lock().await = 0;
                *SELECTEDUIELEMENT.lock().await = SelectedUIElement::ReflowStart;
                Transition(State::reflow())
            },
            Event::ReflowPhasePreheatDone => {
                *REFLOW_PHASE_ELAPSED_STEPS.lock().await = 0;
                *SELECTEDUIELEMENT.lock().await = SelectedUIElement::ReflowStop;
                Transition(State::reflow_phase_soak())
            }
            _ => Super,
        }
    }

    #[state(superstate = "issue")]
    async fn reflow_phase_soak(event: &Event) -> Outcome<State> {
        match event {
            Event::ReflowStopSelected => {
                *REFLOW_START_TIME.lock().await = Instant::MIN;
                *REFLOW_PHASE_ELAPSED_STEPS.lock().await = 0;
                *SELECTEDUIELEMENT.lock().await = SelectedUIElement::ReflowStart;
                Transition(State::reflow())
            },
            Event::ReflowPhaseSoakDone => {
                *REFLOW_PHASE_ELAPSED_STEPS.lock().await = 0;
                *SELECTEDUIELEMENT.lock().await = SelectedUIElement::ReflowStop;
                Transition(State::reflow_phase_reflow())
            }
            _ => Super,
        }
    }

    #[state(superstate = "issue")]
    async fn reflow_phase_reflow(event: &Event) -> Outcome<State> {
        match event {
            Event::ReflowStopSelected => {
                *REFLOW_START_TIME.lock().await = Instant::MIN;
                *REFLOW_PHASE_ELAPSED_STEPS.lock().await = 0;
                *SELECTEDUIELEMENT.lock().await = SelectedUIElement::ReflowStart;
                Transition(State::reflow())
            },
            Event::ReflowPhaseReflowDone => {
                *REFLOW_PHASE_ELAPSED_STEPS.lock().await = 0;
                *SELECTEDUIELEMENT.lock().await = SelectedUIElement::ReflowStop;
                Transition(State::reflow_phase_cool())
            }
            _ => Super,
        }
    }

    #[state(superstate = "issue")]
    async fn reflow_phase_cool(event: &Event) -> Outcome<State> {
        match event {
            Event::ReflowStopSelected => {
                *REFLOW_PHASE_ELAPSED_STEPS.lock().await = 0;
                *REFLOW_START_TIME.lock().await = Instant::MIN;
                *SELECTEDUIELEMENT.lock().await = SelectedUIElement::ReflowStart;
                Transition(State::reflow())
            },
            Event::ReflowPhaseCoolDone => {
                *REFLOW_PHASE_ELAPSED_STEPS.lock().await = 0;
                *SELECTEDUIELEMENT.lock().await = SelectedUIElement::ReflowCompleteConfirmation;
                Transition(State::reflow_phase_completed())
            }
            _ => Super,
        }
    }

    #[state(superstate = "issue")]
    async fn reflow_phase_completed(event: &Event) -> Outcome<State> {
        match event {
            Event::ReflowCompleteConfirmed => {
                *REFLOW_PHASE_ELAPSED_STEPS.lock().await = 0;
                *REFLOW_START_TIME.lock().await = Instant::MIN;
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

    async fn after_transition(&mut self, _source: &State, target: &State, _context: &mut ()) {
        FSM_STATE.sender().send(target.clone());
        debug!("State transitioned");
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
    
    /* Bind interrupts to DMA channels for I2C and SPI */
    bind_interrupts!(struct Irqs {
        I2C1 => i2c::EventInterruptHandler<peripherals::I2C1>,
                i2c::ErrorInterruptHandler<peripherals::I2C1>;
        DMA1_CHANNEL1 => InterruptHandler<peripherals::DMA1_CH1>;
        DMA1_CHANNEL2_3 => InterruptHandler<peripherals::DMA1_CH2>, InterruptHandler<peripherals::DMA1_CH3>;
        DMAMUX1_DMA1_CH4_5 => InterruptHandler<peripherals::DMA1_CH4>, InterruptHandler<peripherals::DMA1_CH5>;
    });

    /* Bind interrupt for encoder button input */
    bind_interrupts!(struct IrqsEncoder {
        EXTI4_15 => embassy_stm32::exti::InterruptHandler<EXTI4_15>;
    });

    /* Bind interrupt for ZCD input */
    /* ZCD currently unused but will be left uncommented here for reference */
    // bind_interrupts!(struct IrqsZcd {
    //     EXTI2_3 => embassy_stm32::exti::InterruptHandler<EXTI2_3>;
    // });

    /* Create PWM output pin for fan control */
    let fan_pin: PwmPin<'_, peripherals::TIM2, embassy_stm32::timer::Ch2> = PwmPin::new(p.PB3, embassy_stm32::gpio::OutputType::PushPull);
    let mut fan_pmw = SimplePwm::new(p.TIM2, None, Some(fan_pin), None, None, Hertz(1000), embassy_stm32::timer::low_level::CountingMode::EdgeAlignedUp);
    fan_pmw.enable(Ch2);
    fan_pmw.set_duty(Ch2, 0);
    
    /* Create PWM output pin for triac control */
    let triac_pin: PwmPin<'_, peripherals::TIM1, embassy_stm32::timer::Ch3> = PwmPin::new(p.PA2, embassy_stm32::gpio::OutputType::PushPull);
    let triac_pwm: SimplePwm<'_, peripherals::TIM1> = SimplePwm::new(p.TIM1, None, None, Some(triac_pin), None, Hertz(10), embassy_stm32::timer::low_level::CountingMode::EdgeAlignedUp);
    
    /* Create SPI configuration */
    let mut spi_config = spiConfig::default();
    spi_config.nss_output_disable = false; // Hardware NSS (not GPIO)
    spi_config.frequency = Hertz(1_000_000);
    spi_config.mode.phase = CaptureOnFirstTransition;
    spi_config.mode.polarity = IdleLow;
    let n_cs = Output::new(p.PA4, Level::High, Speed::High);

    /* Construct async SPI device */
    let spi = Spi::new_rxonly(
        p.SPI1, 
        p.PA1, 
        p.PA6, 
        p.DMA1_CH4, 
        p.DMA1_CH5, 
        Irqs, 
        spi_config
    );
    
    /* Create I2C configuration */
    let mut i2c_config = i2cConfig::default();
    i2c_config.frequency = Hertz(1_000_000);
    i2c_config.scl_pullup = true;  // Enable SCL pull-up
    i2c_config.sda_pullup = true;  // Enable SDA pull-up
    
    /* Construct async I2C device */
    let i2c = I2c::new(
        p.I2C1, 
        p.PA9, 
        p.PA10, 
        p.DMA1_CH1, 
        p.DMA1_CH2, 
        Irqs, 
        i2c_config);
        
    /* Create shared bus with I2C device */
    let i2c_bus = I2C_BUS.init(Mutex::new(i2c));
    
    /* Bind encoder interrupts */
    let encoder_a = ENCODER_A_INPUT.init(ExtiInput::new(p.PA5, p.EXTI5, Pull::Up, IrqsEncoder));
    let encoder_b = ENCODER_B_INPUT.init(ExtiInput::new(p.PA7, p.EXTI7, Pull::Up, IrqsEncoder));
    let encoder_btn = ENCODER_BTN_INPUT.init(ExtiInput::new(p.PA8, p.EXTI8, Pull::Up, IrqsEncoder));

    /* Construct the encoder */
    RotaryEncoder::default();

    /* Bind ZCD interrupt */
    /* ZCD currently unused but will be left uncommented here for reference */
    // let _zcd_detector = ZCD_DETECT.init(ExtiInput::new(p.PA3, p.EXTI3, Pull::Up, IrqsZcd));

    /* Spawn tasks */
    spawner.spawn(read_transformer_ina219_task(i2c_bus).unwrap());
    spawner.spawn(read_fan_ina219_task(i2c_bus).unwrap());
    spawner.spawn(task_encoder(encoder_a, encoder_b, encoder_btn).unwrap());
    spawner.spawn(read_thermocouple_task(spi, n_cs).unwrap());
    spawner.spawn(display_task(i2c_bus).unwrap());
    spawner.spawn(pid_task(triac_pwm).unwrap());
    /* Unused task but will be left uncommented here for reference */
    // spawner.spawn(task_zcd_detector(zcd_detector).unwrap());

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

async fn pid_step(pid: &mut Pid<f32>, setpoint: f32, max_duty_triac_pwm: u32, temperature: f32) -> u32 {
    /* Set PID target to setpoint temperature */
    pid.setpoint(setpoint);

    /* Calculate absolute error */
    let error_abs = (pid.setpoint - temperature).abs();

    /* Stop I-term wind-up/down if inside deadzone */
    if error_abs < PID_TEMP_DEADZONE {
        pid.i(0.0, PID_KI_LIMIT);
    }
    else {
        pid.i(PID_KI, PID_KI_LIMIT);
    }

    /* Feed current temperature into PID */
    let pid_output = pid.next_control_output(temperature );
    /* Shape output into clamped and inverted value for PWM output */
    let triac_pwm_output = max_duty_triac_pwm - (pid_output.output.clamp(0.0, max_duty_triac_pwm as f32) as u32);
    let power_percentage = 100 - (triac_pwm_output as f32 / max_duty_triac_pwm as f32 * 100.0) as u8;
    warn!("\x1B[32mP\x1B[0m: {} \x1B[32mI\x1B[0m: {} \x1B[32mD\x1B[0m: {} \x1B[32moutput\x1B[0m: {} \x1B[32mPWM\x1B[0m: {} \x1B[32mtemp\x1B[0m: {} \x1B[32mpwr\x1B[0m: {} \x1B[32mtarget\x1B[0m: {}", &pid_output.p, &pid_output.i, &pid_output.d, &pid_output.output, &triac_pwm_output, &temperature, &power_percentage, &setpoint);
    return triac_pwm_output
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
    let mut pid: Pid<f32> = Pid::new(0.0, max_duty_triac_pwm as f32);
    pid.p(PID_KP, max_duty_triac_pwm as f32);
    pid.i(PID_KI,   PID_KI_LIMIT);
    let mut triac_pwm_output: u32;

    /* Accurate time tracking */
    let mut expiration_time: Instant;

    /* FSM_STATE receiver */
    let mut fsm_state_rx = FSM_STATE.receiver().unwrap();
    
    /* Local vars */
    let mut fsm_state: State;
    let mut setpoint_target: u32;
    let mut temperature: f32;
    let mut reflow_target: f32 = 0.0;

    loop {
        /* Get current time to reference task duration to */
        expiration_time = Instant::now() + TASK_TIMEOUT;
        
        /* Receive FSM_STATE */
        fsm_state = fsm_state_rx.try_get().unwrap();

        /* Retreive current temperature */
        temperature = *TEMPERATURE.lock().await;
        
        /* Drive PID according to current state */
        match fsm_state {
            State::SetpointRunning {  } => {
                /* Set PID target to setpoint temperature */
                setpoint_target = *SETPOINT_TEMPERATURE.lock().await;
                
                /* Compute next PID output */
                triac_pwm_output = pid_step(&mut pid, setpoint_target as f32, max_duty_triac_pwm, temperature).await;
            }
            State::ReflowPhasePreheat {  } => {
                /* Retreive phase temperature target*/
                let preheat_temp = SELECTED_REFLOW_PROFILE.lock().await.profile().preheat_temp as f32;

                /* Check if target has been reached */
                if temperature >= preheat_temp - PID_TEMP_DEADZONE {
                    EVENT_QUEUE.send(Event::ReflowPhasePreheatDone).await;
                    Timer::at(expiration_time).await;
                    continue;
                }

                /* Calculate reflow target if it hasn't reached the target  */
                if !(reflow_target >= preheat_temp) {
                    reflow_target = *REFLOW_PHASE_ELAPSED_STEPS.lock().await as f32 * REFLOW_PARAMETERS.lock().await.preheat_ramp; 
                    let ambient_temp = *REFLOW_START_TEMP.lock().await as f32;
                    if reflow_target < ambient_temp {
                        reflow_target = ambient_temp;
                    }
                }
                *REFLOW_TARGET_TEMPERATURE.lock().await = reflow_target as u32;

                /* Step the PID for new output value */
                triac_pwm_output = pid_step(&mut pid, reflow_target, max_duty_triac_pwm, temperature).await;
                *REFLOW_PHASE_ELAPSED_STEPS.lock().await += 1;
            }
            State::ReflowPhaseSoak {  } => {
                /* Retreive phase temperature target*/
                let soak_temp = SELECTED_REFLOW_PROFILE.lock().await.profile().soak_temp as f32;

                /* Check if target has been reached */
                if temperature >= soak_temp - PID_TEMP_DEADZONE {
                    EVENT_QUEUE.send(Event::ReflowPhaseSoakDone).await;
                    Timer::at(expiration_time).await;
                    continue;
                }

                /* Calculate reflow target if it hasn't reached the target  */
                if !(reflow_target >= soak_temp) {
                    reflow_target = SELECTED_REFLOW_PROFILE.lock().await.profile().preheat_temp as f32 + *REFLOW_PHASE_ELAPSED_STEPS.lock().await as f32 * REFLOW_PARAMETERS.lock().await.soak_ramp; 
                }
                *REFLOW_TARGET_TEMPERATURE.lock().await = reflow_target as u32;

                /* Step the PID for new output value */
                triac_pwm_output = pid_step(&mut pid, reflow_target, max_duty_triac_pwm, temperature).await;
                *REFLOW_PHASE_ELAPSED_STEPS.lock().await += 1;
            }
            State::ReflowPhaseReflow {  } => {
                /* Retreive phase temperature target*/
                let reflow_temp = SELECTED_REFLOW_PROFILE.lock().await.profile().reflow_temp as f32;

                /* Check if target has been reached */
                if temperature >= reflow_temp - PID_TEMP_DEADZONE {
                    EVENT_QUEUE.send(Event::ReflowPhaseReflowDone).await;
                    Timer::at(expiration_time).await;
                    continue;
                }

                /* Calculate reflow target if it hasn't reached the target  */
                if !(reflow_target >= reflow_temp) {
                    reflow_target = SELECTED_REFLOW_PROFILE.lock().await.profile().soak_temp as f32 + *REFLOW_PHASE_ELAPSED_STEPS.lock().await as f32 * REFLOW_PARAMETERS.lock().await.reflow_ramp; 
                }
                *REFLOW_TARGET_TEMPERATURE.lock().await = reflow_target as u32;

                /* Step the PID for new output value */
                triac_pwm_output = pid_step(&mut pid, reflow_target, max_duty_triac_pwm, temperature).await;
                *REFLOW_PHASE_ELAPSED_STEPS.lock().await += 1;
            }
            State::ReflowPhaseCool {  } => {
                /* Retreive phase temperature target*/
                let cool_temp = SELECTED_REFLOW_PROFILE.lock().await.profile().cool_temp;
                
                /* Turn triac off */
                triac_pwm_output = max_duty_triac_pwm; 
                *REFLOW_TARGET_TEMPERATURE.lock().await = cool_temp;

                /* Check if target has been reached */
                if temperature <= cool_temp as f32 {
                    EVENT_QUEUE.send(Event::ReflowPhaseCoolDone).await;
                    Timer::at(expiration_time).await;
                    continue;
                }

                /* Calculate reflow target steps for consistent UI, does not really impact cooling phase  */
                if !(reflow_target <= cool_temp as f32) {
                    reflow_target = SELECTED_REFLOW_PROFILE.lock().await.profile().max_temp as f32 - *REFLOW_PHASE_ELAPSED_STEPS.lock().await as f32 * REFLOW_PARAMETERS.lock().await.cool_ramp;
                }
                *REFLOW_TARGET_TEMPERATURE.lock().await = reflow_target as u32;
                *REFLOW_PHASE_ELAPSED_STEPS.lock().await += 1;

            }
            _ => {
                /* Disable heater in all other states */
                triac_pwm_output = max_duty_triac_pwm; 
            }
        }
        /* Set triac  */
        triac_pwm.set_duty(Ch3, triac_pwm_output);

        /* Calculate and share triac power percentage */
        *TRIAC_PWR.lock().await = ((max_duty_triac_pwm - triac_pwm_output) as f32 / max_duty_triac_pwm as f32 * 100.0) as u8;
        
        /* Await until the loop has taken exactly PID_TIMEOUT(_CORE) (100ms) */
        Timer::at(expiration_time).await;
    }
}

#[embassy_executor::task]
async fn read_thermocouple_task(mut spi_dev: Spi<'static, Async, Master>, mut nss: Output<'static>) {
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
                    EVENT_QUEUE.send(Event::Error(ErrorType::NoThermocouple)).await;
                    continue;
                },
                2 => {
                    error!("Thermocouple shorted to GND!");
                    EVENT_QUEUE.send(Event::Error(ErrorType::ThermocoupleShortGnd)).await;
                    continue;
                },
                4 => {
                    error!("Thermocouple shorted to Vcc!");
                    EVENT_QUEUE.send(Event::Error(ErrorType::ThermocoupleShortVcc)).await;
                    continue;
                },
                _ => {
                    error!("Thermocouple issues!");
                    EVENT_QUEUE.send(Event::Error(ErrorType::ThermocoupleIssue)).await;
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

    let mut ina_transformer = AsyncIna219::new(i2c_dev, Address::from_byte(0x42).unwrap()).await.unwrap();
    
    let ina_conf = Configuration {
        bus_voltage_range: BusVoltageRange::Fsr16v,
        bus_resolution: Resolution::Avg128,
        operating_mode: OperatingMode::Continous(MeasuredSignals::ShutAndBusVoltage),
        shunt_resolution: Resolution::Avg128,
        shunt_voltage_range: ShuntVoltageRange::Fsr80mv,
        reset: Reset::Reset,
    };
    
    match ina_transformer.set_configuration(ina_conf).await {
        Ok(_) => {},
        Err(e) => {
            match e {
                embassy_embedded_hal::shared_bus::I2cDeviceError::I2c(_) => {
                    error!("Transformer INA219 I2C Device Error!");
                    EVENT_QUEUE.send(Event::Error(ErrorType::NoInaTransformer)).await; 
                }
                _bus => {
                    error!("Transformer INA219 I2C Bus Error!");
                    EVENT_QUEUE.send(Event::Error(ErrorType::NoInaTransformer)).await;
                }
            }
        }
    }

    let conversion_time = ina_conf.conversion_time_us().unwrap();
    let mut measurement: Measurements<(), ()>;
    let mut bus_voltage_v: f32;
    let mut shunt_voltage_mv: i16;
    let mut current: f32;

    loop {
        Timer::after_micros(conversion_time as u64).await;
        match ina_transformer.next_measurement().await {
            Ok(option) => {
                match option {
                    Some(result) => {
                        measurement = result;
                        bus_voltage_v = measurement.bus_voltage.voltage_mv() as f32 / 1000 as f32;
                        shunt_voltage_mv = measurement.shunt_voltage.shunt_voltage_mv();
                        current = shunt_voltage_mv as f32 / 1.41414141 / 10 as f32;
                        if current < 0 as f32 { current *= -1 as f32 };
                        debug!("INA219 Transformer: Bus: {}V, Shunt: {}mV, Current: {}A", bus_voltage_v, shunt_voltage_mv, current)
                    },
                    None => {}
                }
            }
            Err(e) => {
                match e {
                    ina219::errors::MeasurementError::BusVoltageReadError(_) => { error!("Transformer INA219: Bus Voltage Read Error") },
                    ina219::errors::MeasurementError::I2cError(_) => { error!("Transformer INA219: I2C Error") },
                    ina219::errors::MeasurementError::MathOverflow(_) => { error!("Transformer INA219: Math Overflow Error") },
                    ina219::errors::MeasurementError::ShuntVoltageReadError(_) => { error!("Transformer INA219: Shunt Voltage Read Error") },
                }
                EVENT_QUEUE.send(Event::Error(ErrorType::NoInaTransformer)).await;
            }
        };
    }
}

#[embassy_executor::task]
async fn read_fan_ina219_task(bus: &'static I2c1Bus) {
    let i2c_dev = I2cDevice::new(bus);

    let mut ina_fan = AsyncIna219::new(i2c_dev, Address::from_byte(0x40).unwrap()).await.unwrap();
    
    let ina_conf = Configuration {
        bus_voltage_range: BusVoltageRange::Fsr16v,
        bus_resolution: Resolution::Avg128,
        operating_mode: OperatingMode::Continous(MeasuredSignals::ShutAndBusVoltage),
        shunt_resolution: Resolution::Avg128,
        shunt_voltage_range: ShuntVoltageRange::Fsr40mv,
        reset: Reset::Reset,
    };
    
    match ina_fan.set_configuration(ina_conf).await {
                Ok(_) => {},
        Err(e) => {
            match e {
                embassy_embedded_hal::shared_bus::I2cDeviceError::I2c(_) => {
                    error!("Fan INA219 I2C Device Error!");
                    EVENT_QUEUE.send(Event::Error(ErrorType::NoInaFan)).await; 
                }
                _bus => {
                    error!("Fan INA219 I2C Bus Error!");
                    EVENT_QUEUE.send(Event::Error(ErrorType::NoInaFan)).await;
                }
            }
        }
    }

    let conversion_time = ina_conf.conversion_time_us().unwrap();
    let mut measurement: Measurements<(), ()>;
    let mut bus_voltage_v: f32;
    let mut shunt_voltage_mv: i16;
    let mut current: f32;

    loop {
        Timer::after_micros(conversion_time as u64).await;
        match ina_fan.next_measurement().await {
            Ok(option) => {
                match option {
                    Some(result) => {
                        measurement = result;
                        bus_voltage_v = measurement.bus_voltage.voltage_mv() as f32 / 1000 as f32;
                        shunt_voltage_mv = measurement.shunt_voltage.shunt_voltage_mv();
                        current = shunt_voltage_mv as f32 / 1.41414141 / 10 as f32;
                        if current < 0 as f32 { current *= -1 as f32 };
                        debug!("INA219 Fan: Bus: {}V, Shunt: {}mV, Current: {}A", bus_voltage_v, shunt_voltage_mv, current)
                    },
                    None => {}
                }
            }
            Err(e) => {
                match e {
                    ina219::errors::MeasurementError::BusVoltageReadError(_) => { error!("Fan INA219: Bus Voltage Read Error") },
                    ina219::errors::MeasurementError::I2cError(_) => { error!("Fan INA219: I2C Error") },
                    ina219::errors::MeasurementError::MathOverflow(_) => { error!("Fan INA219: Math Overflow Error") },
                    ina219::errors::MeasurementError::ShuntVoltageReadError(_) => { error!("Fan INA219: Shunt Voltage Read Error") },
                }
                EVENT_QUEUE.send(Event::Error(ErrorType::NoInaFan)).await;
            }
        };
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
    let mut decoder = encoder::GrayDecoder::new();

    loop {
        /* Wait for changes on the encoder interrupt lines */
        match select3(
            encoder_a.wait_for_any_edge(),
            encoder_b.wait_for_any_edge(),
            encoder_btn.wait_for_any_edge(),
        ).await {
            Either3::First(_) | Either3::Second(_) => {

                if let Some(dir) =  decoder.update(encoder_a.is_high(), encoder_b.is_high()) {
                    match dir {
                        encoder::Direction::Clockwise => {
                            rot_enc_pos = (rot_enc_pos + 1) % 360;
                            direction = RotaryEncoderDirection::CW;
                            fsm_state = fsm_state_rx.try_get().unwrap();
                            match fsm_state {
                                State::SetpointSelecting {  } => { 
                                    let setpoint_temp = *SETPOINT_TEMPERATURE.lock().await;
                                    if setpoint_temp >= MAX_TEMP {continue}
                                    *SETPOINT_TEMPERATURE.lock().await += 1; 
                                },
                                _ => {  },
                            }
                        },
                        encoder::Direction::CounterClockwise => {
                            rot_enc_pos = (rot_enc_pos + 359) % 360;
                            direction = RotaryEncoderDirection::CCW;
                            fsm_state = fsm_state_rx.try_get().unwrap();
                            match fsm_state {
                                State::SetpointSelecting {  } => { 
                                    let setpoint_temp = *SETPOINT_TEMPERATURE.lock().await;
                                    if setpoint_temp <= MIN_TEMP {continue}
                                    *SETPOINT_TEMPERATURE.lock().await -= 1; 
                                },
                                _ => {  },
                            }
                        },
                    };
                }
                else {
                    direction = RotaryEncoderDirection::Stationary
                }
                pressed = false;
            }

            Either3::Third(_) => {
                if encoder_btn.is_high() { pressed = false; }
                else { 
                    pressed = true;
                }
                direction = RotaryEncoderDirection::Stationary;
                EVENT_QUEUE.send(Event::EncoderPressed).await;
            }
        }

        publisher.publish_immediate(RotaryEncoder {
            pressed: pressed,
            direction: direction,
        });

        debug!("Encoder position: \x1B[32m{}\x1B[0m Button: \x1B[32m{}\x1B[0m", &rot_enc_pos, pressed);

    }
}

#[embassy_executor::task]
async fn task_zcd_detector(zcd_detector: &'static mut ExtiInput<'static, Async>) {
    loop {
        zcd_detector.wait_for_any_edge().await;
        debug!("Zero Crossing Detected!");
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
    match display.init().await {
        Ok(_) => {},
        Err(_) => {
            error!("Display flush failed!");
            EVENT_QUEUE.send(Event::Error(ErrorType::NoDisplay)).await;
        }
    }
    display.clear();
    match display.flush().await {
        Ok(_) => {},
        Err(_) => {
            error!("Display flush failed!");
            EVENT_QUEUE.send(Event::Error(ErrorType::NoDisplay)).await;
        }
    }

    /* Buffers */
    let mut temperature_str_buffer = itoa::Buffer::new();
    let mut triac_pwr_temp_str_buffer = itoa::Buffer::new();
    let mut selected_reflow_profile_max_temp_buffer = itoa::Buffer::new();
    let mut selected_reflow_profile_duration_buffer = itoa::Buffer::new();
    let mut reflow_target_temp_str_buffer = itoa::Buffer::new();
    let mut temperature: u32;
    let mut triac_pwr: u8;
    let mut reflow_target_temp: u32;
    let mut current_error: ErrorType;


    /* Rotary Encoder subscriber */
    let mut rot_enc_subscriber = ROT_ENC_CHANNEL.subscriber().unwrap();
    let mut setpoint_target_temp: u32;
    let mut setpoint_target_temp_str_buffer = itoa::Buffer::new();
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
        direction = RotaryEncoderDirection::Stationary;
        pressed = false;

        /* Retreive encoder data */
        if let Some(msg) = rot_enc_subscriber.try_next_message_pure() {
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
                let selected_reflow_profile_max_temp_str: &str;
                let selected_reflow_profile_duration_str: &str;
                let mut selected_reflow_profile_max_temp_str_concat: String<10> = String::new();
                let mut selected_reflow_profile_duration_str_concat: String<10> = String::new();
                match selected_reflow_profile {
                    ReflowProfiles::TS391SNL => {
                        Text::with_alignment("(TS391SNL)", Point { x: (2), y: (HEIGHT as i32 - 12) }, TEXT_STYLE_SMALL_KNOCKOUT, Alignment::Left)
                            .draw(&mut display)
                            .unwrap();

                        selected_reflow_profile_max_temp_str = selected_reflow_profile_max_temp_buffer.format(ReflowProfiles::TS391SNL.profile().max_temp);
                        selected_reflow_profile_duration_str = selected_reflow_profile_duration_buffer.format(ReflowProfiles::TS391SNL.profile().total_duration);
                        write!(&mut selected_reflow_profile_max_temp_str_concat, "{selected_reflow_profile_max_temp_str}°C").unwrap();
                        write!(&mut selected_reflow_profile_duration_str_concat, "{selected_reflow_profile_duration_str} Sec.").unwrap();
                    }
    
                    ReflowProfiles::GC10 => {
                        Text::with_alignment("(GC10)", Point { x: (2), y: (HEIGHT as i32 - 12) }, TEXT_STYLE_SMALL_KNOCKOUT, Alignment::Left)
                            .draw(&mut display)
                            .unwrap();

                        selected_reflow_profile_max_temp_str = selected_reflow_profile_max_temp_buffer.format(ReflowProfiles::GC10.profile().max_temp);
                        selected_reflow_profile_duration_str = selected_reflow_profile_duration_buffer.format(ReflowProfiles::GC10.profile().total_duration);
                        write!(&mut selected_reflow_profile_max_temp_str_concat, "{selected_reflow_profile_max_temp_str}°C").unwrap();
                        write!(&mut selected_reflow_profile_duration_str_concat, "{selected_reflow_profile_duration_str} Sec.").unwrap();
                    }
    
                    _ => {
                        Text::with_alignment("(None)", Point { x: (2), y: (HEIGHT as i32 - 12) }, TEXT_STYLE_SMALL_KNOCKOUT, Alignment::Left)
                            .draw(&mut display)
                            .unwrap();

                        selected_reflow_profile_max_temp_str = selected_reflow_profile_max_temp_buffer.format(ReflowProfiles::NoProfile.profile().max_temp);
                        selected_reflow_profile_duration_str = selected_reflow_profile_duration_buffer.format(ReflowProfiles::NoProfile.profile().total_duration);
                        write!(&mut selected_reflow_profile_max_temp_str_concat, "{selected_reflow_profile_max_temp_str}°C").unwrap();
                        write!(&mut selected_reflow_profile_duration_str_concat, "{selected_reflow_profile_duration_str} Sec.").unwrap();
                    }
                }

                Text::with_baseline("Start", Point::new(10, 2), TEXT_STYLE_SMALL, Baseline::Top)
                    .draw(&mut display)
                    .unwrap();

                Text::with_baseline("Back", Point::new(10, 16), TEXT_STYLE_SMALL, Baseline::Top)
                    .draw(&mut display)
                    .unwrap();
                
                Line::new(Point { x: (45), y: (5) }, Point { x: (45), y: (HEIGHT as i32 - 17) })
                .into_styled(LINE_STYLE)
                    .draw(&mut display)
                    .unwrap();

                Text::with_baseline("Duration:", Point::new(52, 2), TEXT_STYLE_SMALL, Baseline::Top)
                    .draw(&mut display)
                    .unwrap();

                Text::with_baseline(&selected_reflow_profile_duration_str_concat, Point::new(52, 14), TEXT_STYLE_SMALL, Baseline::Top)
                    .draw(&mut display)
                    .unwrap();

                Text::with_baseline("Max Temp.:", Point::new(52, 26), TEXT_STYLE_SMALL, Baseline::Top)
                    .draw(&mut display)
                    .unwrap();

                Text::with_baseline(&selected_reflow_profile_max_temp_str_concat, Point::new(52, 38), TEXT_STYLE_SMALL, Baseline::Top)
                    .draw(&mut display)
                    .unwrap();

                Rectangle::new(Point::new(0, HEIGHT as i32 - 12) , Size::new(WIDTH as u32, 12))
                    .into_styled(RECT_STYLE)
                    .draw(&mut display)
                    .unwrap();

                Text::with_alignment(SELECTED_REFLOW_PROFILE.lock().await.profile().name, Point { x: (2), y: (HEIGHT as i32 - 12) }, TEXT_STYLE_SMALL_KNOCKOUT, Alignment::Left)
                    .draw(&mut display)
                    .unwrap();

                Text::with_alignment("Menu", Point { x: (WIDTH as i32 - 2), y: (HEIGHT as i32 - 12) }, TEXT_STYLE_SMALL_KNOCKOUT, Alignment::Right)
                    .draw(&mut display)
                    .unwrap();

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
                        Line::new(Point { x: (WIDTH as i32 - 33), y: (HEIGHT as i32 - 10)}, Point { x: (WIDTH as i32 - 29), y: (HEIGHT as i32 - 6)})
                            .into_styled(LINE_STYLE_KNOCKOUT)
                            .draw(&mut display)
                            .unwrap();
        
                        Line::new(Point { x: (WIDTH as i32 - 33), y: (HEIGHT as i32 - 2)}, Point { x: (WIDTH as i32 - 29), y: (HEIGHT as i32 - 6)})
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

                Text::with_alignment("Menu", Point { x: (WIDTH as i32 - 2), y: (HEIGHT as i32 - 12) }, TEXT_STYLE_SMALL_KNOCKOUT, Alignment::Right)
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
                        Line::new(Point { x: (WIDTH as i32 - 33), y: (HEIGHT as i32 - 10)}, Point { x: (WIDTH as i32 - 29), y: (HEIGHT as i32 - 6)})
                            .into_styled(LINE_STYLE_KNOCKOUT)
                            .draw(&mut display)
                            .unwrap();
        
                        Line::new(Point { x: (WIDTH as i32 - 33), y: (HEIGHT as i32 - 2)}, Point { x: (WIDTH as i32 - 29), y: (HEIGHT as i32 - 6)})
                            .into_styled(LINE_STYLE_KNOCKOUT)
                            .draw(&mut display)
                            .unwrap();
                    }
                    _ => { panic!("Display_task, match selected_element, cursor draw") }
                }
            }

            State::ReflowPhasePreheat {  } | State::ReflowPhaseSoak {  } | State::ReflowPhaseReflow {  } | State::ReflowPhaseCool {  } => {

                /* Read the temperature mutex and format it into a string */
                temperature = *TEMPERATURE.lock().await as u32;
                let temperature_str = temperature_str_buffer.format(temperature);
                let mut temperature_str_concat: String<10> = String::new();
                write!(&mut temperature_str_concat, "{temperature_str}°C").unwrap();
                
                /* Read the setpoint target temperature mutex and format it into a string */
                reflow_target_temp = *REFLOW_TARGET_TEMPERATURE.lock().await;
                let reflow_target_temp_str = reflow_target_temp_str_buffer.format(reflow_target_temp);
                let mut reflow_target_temp_str_concat: String<10> = String::new();
                write!(&mut reflow_target_temp_str_concat, "{reflow_target_temp_str}°C").unwrap();

                /* Read triac power mutex and format it into a string */
                triac_pwr = *TRIAC_PWR.lock().await;
                let triac_pwr_str = triac_pwr_temp_str_buffer.format(triac_pwr);
                let mut triac_pwr_str_concat: String<5> = String::new();
                write!(&mut triac_pwr_str_concat, "{triac_pwr_str}%").unwrap();

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

                Line::new(Point { x: (45), y: (5) }, Point { x: (45), y: (20) })
                .into_styled(LINE_STYLE)
                    .draw(&mut display)
                    .unwrap();

                Line::new(Point { x: (2), y: (20) }, Point { x: (45), y: (20) })
                .into_styled(LINE_STYLE)
                    .draw(&mut display)
                    .unwrap();

                Text::with_baseline("Target Temp.:", Point::new(52, 2), TEXT_STYLE_SMALL, Baseline::Top)
                    .draw(&mut display)
                    .unwrap();

                Text::with_baseline(&reflow_target_temp_str_concat, Point::new(52, 14), TEXT_STYLE_SMALL, Baseline::Top)
                    .draw(&mut display)
                    .unwrap();

                Text::with_baseline("Measured:", Point::new(52, 26), TEXT_STYLE_SMALL, Baseline::Top)
                    .draw(&mut display)
                    .unwrap();
                
                Text::with_baseline(&temperature_str_concat, Point::new(52, 38), TEXT_STYLE_SMALL, Baseline::Top)
                    .draw(&mut display)
                    .unwrap();

                Text::with_baseline("Power:", Point::new(2, 26), TEXT_STYLE_SMALL, Baseline::Top)
                    .draw(&mut display)
                    .unwrap();

                Text::with_baseline(&triac_pwr_str_concat, Point::new(2, 38), TEXT_STYLE_SMALL, Baseline::Top)
                    .draw(&mut display)
                    .unwrap();

                Text::with_baseline("Stop", Point::new(10, 2), TEXT_STYLE_SMALL, Baseline::Top)
                    .draw(&mut display)
                    .unwrap();

                Rectangle::new(Point::new(0, HEIGHT as i32 - 12) , Size::new(WIDTH as u32, 12))
                    .into_styled(RECT_STYLE)
                    .draw(&mut display)
                    .unwrap();

                
                match fsm_state {
                    State::ReflowPhasePreheat {  } | State::ReflowPhaseSoak {  } | State::ReflowPhaseReflow {  } => {
                        Text::with_alignment("Heating", Point { x: (WIDTH as i32 - 2), y: (HEIGHT as i32 - 12) }, TEXT_STYLE_SMALL_KNOCKOUT, Alignment::Right)
                            .draw(&mut display)
                            .unwrap();

                    }
                    State::ReflowPhaseCool {  } => {
                        Text::with_alignment("Cooling", Point { x: (WIDTH as i32 - 2), y: (HEIGHT as i32 - 12) }, TEXT_STYLE_SMALL_KNOCKOUT, Alignment::Right)
                            .draw(&mut display)
                            .unwrap();
                    }
                    _ => {
                        
                    }
                }

                Text::with_alignment(SELECTED_REFLOW_PROFILE.lock().await.profile().name, Point { x: (2), y: (HEIGHT as i32 - 12) }, TEXT_STYLE_SMALL_KNOCKOUT, Alignment::Left)
                    .draw(&mut display)
                    .unwrap();

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

            State::ReflowPhaseCompleted {  } => {
                /* Send event if UI element has been pressed */
                if pressed {
                    match selected_element {
                        SelectedUIElement::ReflowCompleteConfirmation => { 
                            EVENT_QUEUE.send(Event::ReflowCompleteConfirmed).await;
                            continue;
                        },
                        _ => { panic!("Display_task, State::ReflowPhaseCompleted, if pressed, match selected_element") },
                    }
                }
                
                /* Display elements */
                Text::with_alignment("Reflow complete!", Point { x: ( WIDTH / 2 ) as i32, y: ( 2 ) as i32 }, TEXT_STYLE_SMALL, Alignment::Center)
                    .draw(&mut display)
                    .unwrap();
                
                Text::with_alignment("OK", Point { x: ( WIDTH / 2 ) as i32, y: ( HEIGHT / 2 - 18) as i32 }, TEXT_STYLE_MEDIUM, Alignment::Center)
                .draw(&mut display)
                .unwrap();
            
                Rectangle::new(Point::new(0, HEIGHT as i32 - 12) , Size::new(WIDTH as u32, 12))
                    .into_styled(RECT_STYLE)
                    .draw(&mut display)
                    .unwrap();

                /* Display cursor depending on which UI element is selected */
                match selected_element {
                    SelectedUIElement::ReflowCompleteConfirmation => {
                        Line::new(Point { x: (WIDTH / 2 - 20) as i32, y: (HEIGHT / 2 - 3) as i32 }, Point { x: (WIDTH / 2 - 16) as i32, y: (HEIGHT / 2 - 7) as i32 })
                            .into_styled(LINE_STYLE)
                            .draw(&mut display)
                            .unwrap();
        
                        Line::new(Point { x: (WIDTH / 2 - 20) as i32, y: (HEIGHT / 2 - 11) as i32 }, Point { x: (WIDTH / 2 - 16) as i32, y: (HEIGHT / 2 - 7) as i32 })
                            .into_styled(LINE_STYLE)
                            .draw(&mut display)
                            .unwrap();
                    }

                    _ => { panic!("Display_task, match selected_element, cursor draw") }
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
                Text::with_alignment("Target:", Point { x: (2), y: (7) }, TEXT_STYLE_SMALL, Alignment::Left)
                    .draw(&mut display)
                    .unwrap();

                Text::with_alignment(&setpoint_target_temp_str_concat, Point { x: (WIDTH as i32 - 8), y: (2) }, TEXT_STYLE_MEDIUM, Alignment::Right)
                    .draw(&mut display)
                    .unwrap();

                Text::with_alignment("Temp.:", Point { x: (2), y: (31) }, TEXT_STYLE_SMALL, Alignment::Left)
                    .draw(&mut display)
                    .unwrap();

                Text::with_alignment(&temperature_str_concat, Point { x: (WIDTH as i32 - 8), y: (26) }, TEXT_STYLE_MEDIUM, Alignment::Right)
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
                    SelectedUIElement::SetpointTemperatureRollerInactive => {
                        Line::new(Point { x: (WIDTH as i32 - 16), y: (13) }, Point { x: (WIDTH as i32 - 12), y: (9) })
                            .into_styled(LINE_STYLE)
                            .draw(&mut display)
                            .unwrap();
        
                        Line::new(Point { x: (WIDTH as i32 - 16), y: (13) }, Point { x: (WIDTH as i32 - 12), y: (17) })
                            .into_styled(LINE_STYLE)
                            .draw(&mut display)
                            .unwrap();
                    }

                    SelectedUIElement::SetpointMenu => {
                        Line::new(Point { x: (WIDTH as i32 - 33), y: (HEIGHT as i32 - 10)}, Point { x: (WIDTH as i32 - 29), y: (HEIGHT as i32 - 6)})
                            .into_styled(LINE_STYLE_KNOCKOUT)
                            .draw(&mut display)
                            .unwrap();
        
                        Line::new(Point { x: (WIDTH as i32 - 33), y: (HEIGHT as i32 - 2)}, Point { x: (WIDTH as i32 - 29), y: (HEIGHT as i32 - 6)})
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
                Text::with_alignment("Target:", Point { x: (2), y: (7) }, TEXT_STYLE_SMALL, Alignment::Left)
                    .draw(&mut display)
                    .unwrap();

                Text::with_alignment(&setpoint_target_temp_str_concat, Point { x: (WIDTH as i32 - 8), y: (2) }, TEXT_STYLE_MEDIUM, Alignment::Right)
                    .draw(&mut display)
                    .unwrap();

                Text::with_alignment("Temp.:", Point { x: (2), y: (31) }, TEXT_STYLE_SMALL, Alignment::Left)
                    .draw(&mut display)
                    .unwrap();

                Text::with_alignment(&temperature_str_concat, Point { x: (WIDTH as i32 - 8), y: (26) }, TEXT_STYLE_MEDIUM, Alignment::Right)
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
                    SelectedUIElement::SetpointTemperatureRollerActive => {
                        Triangle::new(Point { x: (WIDTH as i32 - 16), y: (13)}, Point { x: (WIDTH as i32 - 12), y: (9)}, Point { x: (WIDTH as i32 - 12), y: (17) })
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

                Text::with_alignment("Menu", Point { x: (WIDTH as i32 - 2), y: (HEIGHT as i32 - 12) }, TEXT_STYLE_SMALL_KNOCKOUT, Alignment::Right)
                    .draw(&mut display)
                    .unwrap();

                /* Display cursor depending on which UI element is selected */
                match selected_element {
                    SelectedUIElement::MeasureMenu => {
                        Line::new(Point { x: (WIDTH as i32 - 33), y: (HEIGHT as i32 - 10)}, Point { x: (WIDTH as i32 - 29), y: (HEIGHT as i32 - 6)})
                            .into_styled(LINE_STYLE_KNOCKOUT)
                            .draw(&mut display)
                            .unwrap();
        
                        Line::new(Point { x: (WIDTH as i32 - 33), y: (HEIGHT as i32 - 2)}, Point { x: (WIDTH as i32 - 29), y: (HEIGHT as i32 - 6)})
                            .into_styled(LINE_STYLE_KNOCKOUT)
                            .draw(&mut display)
                            .unwrap();
                    }
                    _ => { panic!("Display_task, state reflow_running, match selected_element, cursor draw") }
                }
            }

            State::RecoverableError {  } => {
                /* Send event if UI element has been pressed */
                if pressed {
                    match selected_element {
                        SelectedUIElement::RecoverableErrorConfirmation => { 
                            EVENT_QUEUE.send(Event::Error(ErrorType::NoErrors)).await;
                            continue;
                        },
                        _ => { panic!("Display_task, state recoverable_error, if pressed, match selected_element") },
                    }
                }

                current_error = *CURRENT_ERROR.lock().await;
                let mut current_error_str: String<10> = String::new();
                write!(&mut current_error_str, "{:#04X}", current_error as u8).unwrap();

                Text::with_alignment("Recoverable Error", Point { x: (WIDTH as i32 / 2), y: ( 1 ) }, TEXT_STYLE_SMALL, Alignment::Center)
                    .draw(&mut display)
                    .unwrap();

                Text::with_alignment(&current_error_str, Point { x: (WIDTH as i32 / 2), y: ( 14 ) }, TEXT_STYLE_SMALL, Alignment::Center)
                    .draw(&mut display)
                    .unwrap();

                Text::with_alignment("OK", Point { x: (WIDTH as i32 / 2), y: ( 52 ) }, TEXT_STYLE_SMALL, Alignment::Center)
                    .draw(&mut display)
                    .unwrap();

                /* Display cursor depending on which UI element is selected */
                match selected_element {
                    SelectedUIElement::RecoverableErrorConfirmation => {
                        Line::new(Point { x: ( 73 ), y: ( 58 )}, Point { x: ( 77 ), y: ( 54 )})
                            .into_styled(LINE_STYLE)
                            .draw(&mut display)
                            .unwrap();
        
                        Line::new(Point { x: ( 73 ), y: ( 58 )}, Point { x: ( 77 ), y: ( 62 )})
                            .into_styled(LINE_STYLE)
                            .draw(&mut display)
                            .unwrap();
                    }
                    _ => { panic!("Display_task, state recoverable_error, match selected_element, cursor draw") }
                }

            }
            State::UnrecoverableError {  } => {
                current_error = *CURRENT_ERROR.lock().await;
                let mut current_error_str: String<10> = String::new();
                write!(&mut current_error_str, "{:#04X}", current_error as u8).unwrap();

                Text::with_alignment("UNRECOVERABLE ERROR", Point { x: (WIDTH as i32 / 2), y: ( 1 ) }, TEXT_STYLE_SMALL, Alignment::Center)
                    .draw(&mut display)
                    .unwrap();

                Text::with_alignment( &current_error_str , Point { x: (WIDTH as i32 / 2), y: ( 14 ) }, TEXT_STYLE_SMALL, Alignment::Center)
                    .draw(&mut display)
                    .unwrap();
            }
        }

        *SELECTEDUIELEMENT.lock().await = selected_element;

        /* Flush to display */
        match display.flush().await {
            Ok(_) => {},
            Err(_) => {
                error!("Display flush failed!");
                EVENT_QUEUE.send(Event::Error(ErrorType::NoDisplay)).await;
            }
        }
    }
}