#![no_std]
#![no_main]

use embassy_executor::{InterruptExecutor, Spawner};
use embassy_stm32::adc::Adc;
use embassy_stm32::Config;
use embassy_stm32::gpio::{Input, Level, Output, Pull, Speed};
use embassy_stm32::i2c::I2c;
use embassy_stm32::interrupt::{Priority, InterruptExt};
use embassy_stm32::interrupt;
use embassy_stm32::peripherals::*;
use embassy_stm32::time::Hertz;
use embassy_stm32::usart::Uart;
use embassy_stm32::wdg::IndependentWatchdog;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::pubsub::{PubSubChannel, Publisher, Subscriber};
use embassy_time::{Delay, Duration, Ticker, Timer};
use shared_types::IoBoardRole;
use static_cell::StaticCell;

use {defmt_rtt as _, panic_probe as _};

// --- Channel/type aliases expected by roles/* (kept so everything still compiles) ---

type OutputStateChannel =
    PubSubChannel::<CriticalSectionRawMutex, ([bool; 8], bool), 1, 3, 2>;
type OutputStateSubscriber =
    Subscriber::<'static, CriticalSectionRawMutex, ([bool; 8], bool), 1, 3, 2>;
type OutputStatePublisher =
    Publisher::<'static, CriticalSectionRawMutex, ([bool; 8], bool), 1, 3, 2>;
static OUTPUT_STATE: StaticCell<OutputStateChannel> = StaticCell::new();

#[allow(dead_code)]
enum InputMode {
    Disabled,
    Uart(embassy_stm32::usart::Config),
    I2c(Hertz, embassy_stm32::i2c::Config),
    Adc,
    Gpio(Pull),
}

#[allow(dead_code)]
enum DriveVoltage {
    Battery,
    ChargeBus,
    BoostConverter,
    Regulator5V,
}

// == J7
#[allow(dead_code)]
enum Input0 {
    Disabled,
}

// == J6
#[allow(dead_code)]
enum Input1 {
    Disabled,
    Uart(Uart<'static, USART3, DMA1_CH2, DMA1_CH3>),
    I2c(I2c<'static, I2C2, DMA1_CH4, DMA1_CH5>),
}

// == J5
#[allow(dead_code)]
enum Input2 {
    Disabled,
    Uart, // TODO
    Adc,  // TODO
}

// == J4
#[allow(dead_code)]
enum Input3 {
    Disabled,
    Uart, // TODO
    I2c(I2c<'static, I2C1, DMA1_CH6, DMA1_CH7>),
    Gpio(Input<'static, PB6>, Input<'static, PB7>),
}

// == J12
#[allow(dead_code)]
enum Input4 {
    Disabled,
    Adc, // TODO
}

// == J13
#[allow(dead_code)]
enum Input5 {
    Disabled,
    Adc, // TODO
}

/// Abstraction for the inputs and outputs. These are the same for all roles.
#[allow(dead_code)]
struct BoardIo {
    pub input0: Input0,
    pub input1: Input1,
    pub input2: Input2,
    pub input3: Input3,
    pub input4: Input4,
    pub input5: Input5,
    pub leds: (Output<'static, PB12>, Output<'static, PB13>, Output<'static, PB14>),
}

// Global allocator kept (unused unless you allocate).
#[global_allocator]
static ALLOCATOR: alloc_cortex_m::CortexMHeap = alloc_cortex_m::CortexMHeap::empty();

// High-prio executor kept because roles/* reference Irqs; we won’t actually start it.
static EXECUTOR_HIGH: InterruptExecutor = InterruptExecutor::new();

// Interrupt bindings kept so types like `Irqs` resolve in roles/*
embassy_stm32::bind_interrupts!(struct Irqs {
    CAN1_RX1 => embassy_stm32::can::Rx1InterruptHandler<CAN>;
    CAN1_SCE => embassy_stm32::can::SceInterruptHandler<CAN>;
    USB_LP_CAN1_RX0 => embassy_stm32::can::Rx0InterruptHandler<CAN>;
    USB_HP_CAN1_TX => embassy_stm32::can::TxInterruptHandler<CAN>;

    I2C1_EV => embassy_stm32::i2c::EventInterruptHandler<I2C1>;
    I2C1_ER => embassy_stm32::i2c::ErrorInterruptHandler<I2C1>;

    I2C2_EV => embassy_stm32::i2c::EventInterruptHandler<I2C2>;
    I2C2_ER => embassy_stm32::i2c::ErrorInterruptHandler<I2C2>;

    USART3 => embassy_stm32::usart::InterruptHandler<USART3>;
});

// --- Minimal main: init clocks + log once per second ---

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    // Same clock setup you had; HSE 8MHz -> SYSCLK 72MHz.
    let mut config = Config::default();
    config.rcc.hse = Some(Hertz::mhz(8));
    config.rcc.sys_ck = Some(Hertz::mhz(72));
    config.rcc.hclk = Some(Hertz::mhz(72));
    config.rcc.pclk1 = Some(Hertz::mhz(36));
    config.rcc.pclk2 = Some(Hertz::mhz(72));
    config.rcc.adcclk = Some(Hertz::mhz(14));

    // Bring up Embassy HAL; we don’t use peripherals further in this minimal demo.
    let _p = embassy_stm32::init(config);

    defmt::info!("Embassy is up. Logging once per second…");

    let mut ticker = Ticker::every(Duration::from_secs(1));
    loop {
        defmt::info!("tick");
        ticker.next().await;
    }
}

// Kept for completeness; not used by this minimal main.
#[interrupt]
unsafe fn SPI2() {
    EXECUTOR_HIGH.on_interrupt()
}
