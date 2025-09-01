#![no_std]
#![no_main]

mod cypd3177;

use embassy_executor::Spawner;
use embassy_stm32::gpio::{Input, Output, Pull};
use embassy_stm32::exti::ExtiInput;
use embassy_stm32::i2c::{I2c, Config as I2cConfig, Master};
use embassy_stm32::mode::Async;
use embassy_stm32::peripherals::*;
use embassy_stm32::time::Hertz;
use embassy_stm32::usart::Config as UartConfig;
use embassy_time::{Duration, Ticker};
use embassy_usb::driver::EndpointError;
use embassy_sync::mutex::Mutex;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use static_cell::StaticCell;
use {defmt_rtt as _, panic_probe as _};

use embassy_stm32::usb::{self, Driver as OtgDriver};
use embassy_usb::{Builder, Config as UsbConfig};
use embassy_usb::class::cdc_acm::{CdcAcmClass, State as CdcState};

#[allow(dead_code)]
enum InputMode {
    Disabled,
    Uart(UartConfig),
    I2c(Hertz, I2cConfig),
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
#[allow(dead_code)]
struct BoardIo {
    pub leds: (Output<'static>, Output<'static>, Output<'static>),
}

// Global allocator kept (unused unless you allocate).
#[global_allocator]
static ALLOCATOR: alloc_cortex_m::CortexMHeap = alloc_cortex_m::CortexMHeap::empty();

// Interrupt bindings kept so types like `Irqs` resolve in roles/*
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
    OTG_FS => embassy_stm32::usb::InterruptHandler<USB_OTG_FS>;
});

// Endpoint RAM for Synopsys OTG FS (4 KiB)
static EP_MEMORY:    StaticCell<[u8; 4096]>          = StaticCell::new();
static DEVICE_DESC:  StaticCell<[u8; 256]>           = StaticCell::new();
static CONFIG_DESC:  StaticCell<[u8; 256]>           = StaticCell::new();
static BOS_DESC:     StaticCell<[u8; 256]>           = StaticCell::new();
static CONTROL_BUF:  StaticCell<[u8; 64]>            = StaticCell::new();
static CDC_STATE:    StaticCell<CdcState<'static>>   = StaticCell::new();

static CYPD_DRV: StaticCell<
    Mutex<CriticalSectionRawMutex, cypd3177::Cypd3177<I2c<'static, Async, Master>>>
> = StaticCell::new();
// Updated main function with corrected RCC configuration
#[embassy_executor::main]
async fn main(spawner: Spawner) {
    use embassy_stm32::time::Hertz;
    use embassy_stm32::rcc::{
        ADCPrescaler, AHBPrescaler, APBPrescaler, Hse, HseMode,
        Pll, PllMul, PllPreDiv, PllSource, Sysclk,
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

    defmt::info!("Embassy is up. Logging once per second...");

    let mut usb_cfg = UsbConfig::new(0xCAFE, 0xF00D);
    let mut otg_cfg = usb::Config::default();
    otg_cfg.vbus_detection = true; // PA9 is OTG_FS_VBUS on F105

    let ep_memory   = EP_MEMORY.init([0; 4096]);
    let device_desc = DEVICE_DESC.init([0; 256]);
    let config_desc = CONFIG_DESC.init([0; 256]);
    let bos_desc    = BOS_DESC.init([0; 256]);
    let control_buf = CONTROL_BUF.init([0; 64]);
    let cdc_state   = CDC_STATE.init(CdcState::new());

    let usb_driver: OtgDriver<'static, USB_OTG_FS> = OtgDriver::new_fs(
        p.USB_OTG_FS,
        Irqs,
        p.PA12,
        p.PA11,
        ep_memory,
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
        device_desc,
        config_desc,
        bos_desc,
        control_buf,
    );

    let cdc = CdcAcmClass::new(&mut builder, cdc_state, 64);
    let usb = builder.build();

    let mut i2c_cfg = I2cConfig::default();
    i2c_cfg.frequency = Hertz(400_000);
    let i2c1 = I2c::new(
        p.I2C1,
        p.PB6, p.PB7,
        Irqs,
        p.DMA1_CH6, // TX
        p.DMA1_CH7, // RX
        i2c_cfg,
    );
    let cypd = cypd3177::Cypd3177::new(i2c1);
    let cypd_mutex = CYPD_DRV.init(Mutex::new(cypd));

    let hpi_int = ExtiInput::new(p.PA5, p.EXTI5, Pull::Up);

    spawner.spawn(usb_task(usb)).unwrap();
    spawner.spawn(cdc_task(cdc, cypd_mutex)).unwrap();      // <— pass mutex
    spawner.spawn(cypd_monitor_task(cypd_mutex)).unwrap();  // <— new task
    spawner.spawn(hpi_int_task(hpi_int, cypd_mutex)).unwrap();

    let mut ticker = Ticker::every(Duration::from_secs(1));
    let mut count = 0;
    loop {
        defmt::info!("tick {}", count);
        ticker.next().await;
        count += 1;
    }
}

type UsbDrv = embassy_stm32::usb::Driver<'static, USB_OTG_FS>;
type I2cBus = I2c<'static, Async, Master>;
type Cypd   = cypd3177::Cypd3177<I2cBus>;
#[embassy_executor::task]
async fn usb_task(mut dev: embassy_usb::UsbDevice<'static, UsbDrv>) {
    defmt::info!("USB device: run()");
    dev.run().await;
}

#[embassy_executor::task]
async fn cdc_task(
    mut cdc: embassy_usb::class::cdc_acm::CdcAcmClass<'static, UsbDrv>,
    _cypd: &'static Mutex<CriticalSectionRawMutex, Cypd>,
) {
    use heapless::String;

    let mut inbuf = [0u8; 64];
    let _line = heapless::String::<128>::new();
    
    loop {
        cdc.wait_connection().await;
        let _ = cdc.write_packet(
            b"\r\nRAPID Power Module ready.\r\nType 'help' and press Enter.\r\n> "
        ).await;
        defmt::info!("CDC connected");

        loop {
            match cdc.read_packet(&mut inbuf).await {
                Ok(n) if n > 0 => {
                    // ---- DEBUG: log raw packet ----
                    defmt::info!("CDC RX pkt ({}B) ASCII:{:a}", n, &inbuf[..n]);
                    defmt::info!("CDC RX pkt ({}B) HEX  :{:x}", n, &inbuf[..n]);
                }
                Ok(_) => {}
                Err(EndpointError::Disabled) => {
                    defmt::info!("USB CDC disconnected");
                    break; // back to wait_connection()
                }
                Err(EndpointError::BufferOverflow) => {
                    defmt::warn!("USB CDC buffer overflow");
                    let _ = cdc.write_packet(b"\r\n[warn] USB CDC buffer overflow\r\n> ").await;
                }
            }
        }
    }
}

#[embassy_executor::task]
async fn cypd_monitor_task(cypd: &'static Mutex<CriticalSectionRawMutex, Cypd>) {
    use embassy_time::{Ticker, Duration};

    // One-shot probe (what used to be in main)
    {
        let mut dev = cypd.lock().await;

        match dev.device_mode().await {
            Ok(dm) => defmt::info!("CYPD device_mode=0x{:02X} (expect 0x95 active)", dm),
            Err(_) => defmt::warn!("CYPD device_mode read failed"),
        }
        match dev.silicon_id().await {
            Ok(id) => defmt::info!("CYPD silicon_id=0x{:04X} (expect 0x0420)", id),
            Err(_) => defmt::warn!("CYPD silicon_id read failed"),
        }
        match dev.pd_status().await {
            Ok(s) => defmt::info!(
                "PD: contract={} sink_tx_ok={} pe_snk_ready={}",
                s.contract, s.sink_tx_ok, s.pe_snk_ready
            ),
            Err(_) => defmt::warn!("PD status read failed"),
        }
        match dev.typec_status().await {
            Ok(s) => defmt::info!(
                "TypeC: connected={} polarity={} current_level={}",
                s.connected, if s.cc_polarity_cc2 { "CC2" } else { "CC1" }, s.current_level
            ),
            Err(_) => defmt::warn!("TypeC status read failed"),
        }
        match dev.bus_voltage_mv().await {
            Ok(mv) => defmt::info!("VBUS={} mV", mv),
            Err(_) => defmt::warn!("VBUS read failed"),
        }
    }

    // Optional: light periodic status so you see live updates
    let mut ticker = Ticker::every(Duration::from_secs(2));
    loop {
        ticker.next().await;
        let mut dev = cypd.lock().await;
        if let Ok(ps) = dev.pd_status().await {
            defmt::info!("PD: contract={} sink_tx_ok={} pe_snk_ready={}", ps.contract, ps.sink_tx_ok, ps.pe_snk_ready);
        }
        if let Ok(mv) = dev.bus_voltage_mv().await {
            defmt::info!("VBUS={} mV", mv);
        }
    }
}

#[embassy_executor::task]
async fn hpi_int_task(
    mut int_pin: ExtiInput<'static>,
    cypd: &'static Mutex<CriticalSectionRawMutex, Cypd>
) {
    use embassy_time::Timer;

    loop {
        int_pin.wait_for_falling_edge().await;   // EXTI-powered, no polling
        defmt::info!("HPI_INT asserted (low)");

        let mut dev = cypd.lock().await;

        // new public helper (see §3 below)
        if let Ok(b) = dev.interrupt_status().await {
            defmt::info!(
                "CYPD INT=0x{:02X} (dev_int={}, pd_port_int={})",
                b, (b & 1) != 0, (b & 2) != 0
            );

            #[cfg(feature = "cypd-write")]
            let _ = dev.clear_interrupts(b).await;   // write-1-to-clear
        }

        if let Ok(r) = dev.pd_response().await {
            let total_len: u32 = if r.len_hi == 0 { r.len_lo as u32 } else { r.len_hi as u32 };
            let meaning = match r.code {
                0x02 => "success",
                0x01 => "fail",
                0x03 => "busy",
                c    => if (c & 0x80) != 0 { "async-event" } else { "unknown" },
            };
            defmt::info!("PD_RESPONSE: code=0x{:02X} ({}) len={}", r.code, meaning, total_len);
        }

        // small debounce
        Timer::after_millis(3).await;

        // Optionally also wait for the line to deassert to avoid edge re-arm races:
        // int_pin.wait_for_rising_edge().await;
    }
}
