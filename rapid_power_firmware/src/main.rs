#![no_std]
#![no_main]

mod config;

#[cfg(feature = "cypd")]
mod cypd3177;
#[cfg(feature = "bq76952")]
mod bq76952;
#[cfg(feature = "bq25756")]
mod bq25756;
#[cfg(feature = "usb_cdc")]
mod usb_driver;

#[cfg(feature = "can")]
mod can;
mod shell;
mod shared_state;

use embassy_executor::Spawner;
#[cfg(feature = "bq76952")]
use embassy_stm32::gpio::Flex;
use embassy_stm32::gpio::{Level, Output, Pull, Speed};
#[cfg(any(feature = "cypd", feature = "bq25756"))]
use embassy_stm32::exti::ExtiInput;
use embassy_stm32::i2c::{Config as I2cConfig, I2c, Master};
use embassy_stm32::mode::Async;
use embassy_stm32::peripherals::*;
use embassy_stm32::time::Hertz;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_time::{Duration, Timer};
use embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice;
use static_cell::StaticCell;
use {defmt_rtt as _, panic_probe as _};

#[cfg(feature = "usb_cdc")]
use embassy_stm32::usb::{self, Driver as OtgDriver};
#[cfg(feature = "usb_cdc")]
use embassy_usb::class::cdc_acm::{CdcAcmClass, State as CdcState};
#[cfg(feature = "usb_cdc")]
use embassy_usb::{Builder, Config as UsbConfig};

#[allow(dead_code)]
enum InputMode {
    Disabled,
    Uart(embassy_stm32::usart::Config),
    I2c(Hertz, I2cConfig),
    Adc,
    Gpio(Pull),
}

static EP_MEMORY:   StaticCell<[u8; 4096]>        = StaticCell::new();
static DEVICE_DESC: StaticCell<[u8; 256]>         = StaticCell::new();
static CONFIG_DESC: StaticCell<[u8; 256]>         = StaticCell::new();
static BOS_DESC:    StaticCell<[u8; 256]>         = StaticCell::new();
static CONTROL_BUF: StaticCell<[u8; 64]>          = StaticCell::new();
#[cfg(feature = "usb_cdc")]
static CDC_STATE:   StaticCell<CdcState<'static>>  = StaticCell::new();

static I2C_BUS: StaticCell<I2cBusMutex> = StaticCell::new();
#[cfg(feature = "bq76952")]
static BQ76952_DRV: StaticCell<Mutex<CriticalSectionRawMutex, Bq76952<'static>>> = StaticCell::new();
#[cfg(feature = "bq25756")]
static BQ25756_DRV: StaticCell<Mutex<CriticalSectionRawMutex, Bq25756<'static>>> = StaticCell::new();
#[cfg(feature = "cypd")]
pub static CYPD_DRV: StaticCell<Mutex<CriticalSectionRawMutex, Cypd<'static>>> = StaticCell::new();

#[global_allocator]
static ALLOCATOR: alloc_cortex_m::CortexMHeap = alloc_cortex_m::CortexMHeap::empty();

embassy_stm32::bind_interrupts!(struct Irqs {
    CAN1_TX  => embassy_stm32::can::TxInterruptHandler<CAN1>;
    CAN1_RX0 => embassy_stm32::can::Rx0InterruptHandler<CAN1>;
    CAN1_RX1 => embassy_stm32::can::Rx1InterruptHandler<CAN1>;
    CAN1_SCE => embassy_stm32::can::SceInterruptHandler<CAN1>;
    I2C1_EV  => embassy_stm32::i2c::EventInterruptHandler<I2C1>;
    I2C1_ER  => embassy_stm32::i2c::ErrorInterruptHandler<I2C1>;
    I2C2_EV  => embassy_stm32::i2c::EventInterruptHandler<I2C2>;
    I2C2_ER  => embassy_stm32::i2c::ErrorInterruptHandler<I2C2>;
    USART3   => embassy_stm32::usart::InterruptHandler<USART3>;
    OTG_FS   => embassy_stm32::usb::InterruptHandler<USB_OTG_FS>;
});

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    use embassy_stm32::rcc::{
        ADCPrescaler, AHBPrescaler, APBPrescaler, Hse, HseMode, Pll, PllMul, PllPreDiv, PllSource,
        Sysclk,
    };

    let mut config = embassy_stm32::Config::default();
    config.rcc.hse = Some(Hse { freq: Hertz::mhz(8), mode: HseMode::Oscillator });
    config.rcc.pll = Some(Pll { src: PllSource::HSE, prediv: PllPreDiv::DIV1, mul: PllMul::MUL9 });
    config.rcc.sys = Sysclk::PLL1_P;
    config.rcc.ahb_pre  = AHBPrescaler::DIV1;
    config.rcc.apb1_pre = APBPrescaler::DIV2;
    config.rcc.apb2_pre = APBPrescaler::DIV1;
    config.rcc.adc_pre  = ADCPrescaler::DIV6;

    let p = embassy_stm32::init(config);
    defmt::info!("Embassy is up.");

    // ---- BQ25756 CE pin (PC9) ----
    // Drive LOW to allow charging (CE is active-low on this design)
    let bq_ce = Output::new(p.PC9, Level::Low, Speed::VeryHigh);
    // keep it in scope (or move into a StaticCell) so it stays driven for the lifetime
    core::mem::forget(bq_ce);

    // USB (single build)
    let mut usb_cfg = UsbConfig::new(0xCAFE, 0xF00D);
    let mut otg_cfg = usb::Config::default();
    otg_cfg.vbus_detection = true;

    let usb_driver: OtgDriver<'static, USB_OTG_FS> = OtgDriver::new_fs(
        p.USB_OTG_FS,
        Irqs,
        p.PA12,
        p.PA11,
        EP_MEMORY.init([0; 4096]),
        otg_cfg,
    );

    usb_cfg.manufacturer = Some("TUDSAT e.V.");
    usb_cfg.product = Some("RAPID Power Module");
    usb_cfg.serial_number = Some("0001");
    usb_cfg.max_power = 100;
    usb_cfg.max_packet_size_0 = 64;

    let mut builder = Builder::new(
        usb_driver,
        usb_cfg,
        DEVICE_DESC.init([0; 256]),
        CONFIG_DESC.init([0; 256]),
        BOS_DESC.init([0; 256]),
        CONTROL_BUF.init([0; 64]),
    );

    #[cfg(feature = "usb_cdc")]
    let cdc = CdcAcmClass::new(&mut builder, CDC_STATE.init(CdcState::new()), 64);

    let usb = builder.build();

    // I2C bus + shared device handles
    let mut i2c_cfg = I2cConfig::default();
    i2c_cfg.frequency = Hertz(100_000);
    let raw_i2c = I2c::new(p.I2C1, p.PB6, p.PB7, Irqs, p.DMA1_CH6, p.DMA1_CH7, i2c_cfg);
    let bus_mutex: &'static I2cBusMutex = I2C_BUS.init(Mutex::new(raw_i2c));

    // BQ25756
    #[cfg(feature = "bq25756")]
    {
        let i2c_dev: I2cDev<'static> = I2cDevice::new(bus_mutex);
        let bq25756 = bq25756::Bq25756::new(i2c_dev);
        let bq25756_mutex = BQ25756_DRV.init(Mutex::new(bq25756));

        let bq_int   = ExtiInput::new(p.PC10, p.EXTI10, Pull::Up);
        let bq_stat1 = ExtiInput::new(p.PC7,  p.EXTI7,  Pull::Up);
        let bq_stat2 = ExtiInput::new(p.PC6,  p.EXTI6,  Pull::Up);
        let bq_pg    = ExtiInput::new(p.PB15, p.EXTI15, Pull::Up);

        defmt::info!("Spawning BQ25756 tasks");
        spawner
            .spawn(bq25756::bq25756_task(
                bq25756_mutex,
            ))
            .unwrap();

        // IRQ tasks
        spawner.spawn(bq25756::bq25756_int_task(bq_int,   bq25756_mutex)).unwrap();
        spawner.spawn(bq25756::bq25756_pg_task(bq_pg,     bq25756_mutex)).unwrap();
        spawner.spawn(bq25756::bq25756_stat1_task(bq_stat1, bq25756_mutex)).unwrap();
        spawner.spawn(bq25756::bq25756_stat2_task(bq_stat2, bq25756_mutex)).unwrap();
        defmt::info!("BQ25756 tasks spawned");

    }

    // --- BQ76952 FIRST ---
    #[cfg(feature = "bq76952")]
    {
        let ts2 = Flex::new(p.PA3);
        let i2c_dev: I2cDev<'static> = I2cDevice::new(bus_mutex);
        let bq = bq76952::Bq76952::new(i2c_dev, 0x08, 5);
        let bq_mutex = BQ76952_DRV.init(Mutex::new(bq));

        // Start BQ task (it will SET_CFGUPDATE etc.)
        spawner.spawn(bq76952::bq_readout_task(bq_mutex, ts2)).unwrap();

        // Immediately command the address switch -> 0x0B and wait a beat
        let bms_command_publisher = shared_state::BMS_COMMAND_CHANNEL.publisher().unwrap();
        bms_command_publisher.publish(shared_state::BmsCommand::SetI2cAddress(0x0B, None)).await;
        Timer::after(Duration::from_millis(10)).await;

        // (Optional but nice): give the BQ task time to log “successfully communicating at 0x0B”
        Timer::after(Duration::from_millis(10)).await;
    }

    // --- CYPD ONLY AFTER BQ MOVED AWAY FROM 0x08 ---
    #[cfg(feature = "cypd")]
    {
        let i2c_dev: I2cDev<'static> = I2cDevice::new(bus_mutex);
        let mut cypd_tmp = cypd3177::Cypd3177::new(i2c_dev);

        // Sanity probe w/ retries (see section 2)
        match cypd_tmp.probe_plausible().await {
            Ok(id) => {
                defmt::info!(
                    "CYPD detected: DEVICE_MODE=0x{:02X}, SILICON_ID=0x{:04X}",
                    id.device_mode, id.silicon_id
                );

                // If you want basic event IRQs, do it now (optional)
                #[cfg(feature = "cypd-write")]
                if let Err(_) = cypd_tmp.enable_basic_events().await {
                    defmt::warn!("CYPD: enabling EVENT_MASK failed");
                }

                // Put the live instance into the global mutex
                let cypd_mutex = CYPD_DRV.init(Mutex::new(cypd_tmp));

                // IRQ pin
                let hpi_int = ExtiInput::new(p.PA5, p.EXTI5, Pull::Up);

                // Spawn tasks ONLY if probe passed
                spawner.spawn(cypd3177::device::cypd_monitor_task(cypd_mutex)).unwrap();
                spawner.spawn(cypd3177::device::hpi_int_task(hpi_int, cypd_mutex)).unwrap();

                #[cfg(feature = "usb_cdc")]
                spawner.spawn(usb_driver::device::cdc_task(cdc)).unwrap();
            }
            Err(reason) => {
                defmt::warn!(
                    "CYPD not detected at 0x08: {:?}. Skipping CYPD tasks (no USB connected or address clash).",
                    reason
                );
                // IMPORTANT: do not spawn CYPD tasks; nothing will publish.
            }
        }
    }

    // Run USB
    spawner.spawn(usb_task(usb)).unwrap();

    // CAN
    #[cfg(feature = "can")]
    {
        let can = can::init(p.CAN1, p.PB8, p.PB9, Irqs);
        spawner.spawn(can::run(can)).unwrap();
    }
} // <-- CLOSES main

type I2cBus = I2c<'static, Async, Master>;
type I2cBusMutex = Mutex<CriticalSectionRawMutex, I2cBus>;
pub type I2cDev<'a> = I2cDevice<'a, CriticalSectionRawMutex, I2cBus>;

#[cfg(feature = "cypd")]
type Cypd<'a> = cypd3177::Cypd3177<I2cDev<'a>>;
#[cfg(feature = "bq76952")]
type Bq76952<'a> = bq76952::Bq76952<I2cDev<'a>>;
#[cfg(feature = "bq25756")]
type Bq25756<'a> = bq25756::Bq25756<I2cDev<'a>>;

use crate::shared_state::{
    BmsReadings, Bq25756Readings, CypdReadings,
};
use embassy_sync::pubsub::Publisher;
type BmsPublisher<'a> = Publisher<'a, CriticalSectionRawMutex, BmsReadings, 2, 3, 1>;
type Bq25756Publisher<'a> = Publisher<'a, CriticalSectionRawMutex, Bq25756Readings, 2, 3, 1>;
type CypdPublisher<'a> = Publisher<'a, CriticalSectionRawMutex, CypdReadings, 2, 3, 1>;

#[embassy_executor::task]
async fn usb_task(mut dev: embassy_usb::UsbDevice<'static, OtgDriver<'static, USB_OTG_FS>>) {
    defmt::info!("USB device: run()");
    dev.run().await;
}
