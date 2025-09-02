#![no_std]
#![no_main]

 #[cfg(feature = "cypd")]
 use crate::cypd3177::CypdError;
 #[cfg(feature = "cypd")]
 mod cypd3177;
mod bq76952;

use embassy_executor::Spawner;
use embassy_stm32::gpio::{Input, Output, Pull};
#[cfg(feature = "cypd")]
use embassy_stm32::exti::ExtiInput;
use embassy_stm32::i2c::{I2c, Config as I2cConfig, Master};
use embassy_stm32::mode::Async;
use embassy_stm32::peripherals::*;
use embassy_stm32::time::Hertz;
use embassy_stm32::usart::Config as UartConfig;
#[cfg(feature = "cypd")]
use embassy_time::{Duration, Ticker};
use embassy_usb::driver::EndpointError;
use embassy_sync::mutex::Mutex;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use static_cell::StaticCell;
use {defmt_rtt as _, panic_probe as _};

use embassy_stm32::usb::{self, Driver as OtgDriver};
use embassy_usb::{Builder, Config as UsbConfig};
use embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice;
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

static I2C_BUS: StaticCell<I2cBusMutex> = StaticCell::new();
static BQ_DRV: StaticCell<Mutex<CriticalSectionRawMutex, Bq>> = StaticCell::new();

 #[cfg(feature = "cypd")]
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

    defmt::info!("Embassy is up.");

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
    i2c_cfg.frequency = Hertz(100_000);
    let raw_i2c = I2c::new(
        p.I2C1,
        p.PB6, p.PB7,
        Irqs,
        p.DMA1_CH6, // TX
        p.DMA1_CH7, // RX
        i2c_cfg,
    );

    let bus_mutex: &'static I2cBusMutex = I2C_BUS.init(Mutex::new(raw_i2c));
    let i2c_cypd: I2cDev<'static>       = I2cDevice::new(bus_mutex);
    let i2c_bq:   I2cDev<'static>       = I2cDevice::new(bus_mutex);

    // ---- CYPD3177 (unchanged logic, just pass the I2cDevice) ----
    #[cfg(feature = "cypd")]
    {
        let cypd = cypd3177::Cypd3177::new(i2c_cypd);
        let cypd_mutex = CYPD_DRV.init(Mutex::new(cypd));

        #[cfg(feature = "cypd-write")]
        {
            let mut dev = cypd_mutex.lock().await;
            if dev.enable_basic_events().await.is_err() {
                defmt::warn!("CYPD: EVENT_MASK write failed");
            }
        }
        let hpi_int = ExtiInput::new(p.PA5, p.EXTI5, Pull::Up);
        spawner.spawn(cdc_task(cdc, cypd_mutex)).unwrap();
        spawner.spawn(cypd_monitor_task(cypd_mutex)).unwrap();
        spawner.spawn(hpi_int_task(hpi_int, cypd_mutex)).unwrap();
    }

    #[cfg(not(feature = "cypd"))]
    {
        // No CYPD: spawn CDC task without cypd arg
        spawner.spawn(cdc_task(cdc)).unwrap();
    }

        // ---- BQ76952 ----
    {
        // 7-bit default is 0x08 (8-bit write is 0x10). :contentReference[oaicite:13]{index=13}
        let bq = bq76952::Bq76952::new(i2c_bq, bq76952::I2C_ADDR_7BIT, /*Rsense mΩ*/ 1);
        let bq_mutex = BQ_DRV.init(Mutex::new(bq));

        // Spawn a short demo that (1) reads at default addr, (2) changes I²C addr in RAM,
        // (3) proves read at the new address succeeds, then starts periodic logging.
        spawner.spawn(bq_address_demo_and_monitor(bq_mutex)).unwrap();
    }

    spawner.spawn(usb_task(usb)).unwrap();

    // let mut ticker = Ticker::every(Duration::from_secs(1));
    // let mut count = 0;
    // loop {
    //     defmt::info!("tick {}", count);
    //     ticker.next().await;
    //     count += 1;
    // }
}

type UsbDrv = embassy_stm32::usb::Driver<'static, USB_OTG_FS>;
type I2cBus = I2c<'static, Async, Master>;
type I2cBusMutex = Mutex<CriticalSectionRawMutex, I2cBus>;
type I2cDev<'a> = I2cDevice<'a, CriticalSectionRawMutex, I2cBus>;

#[cfg(feature = "cypd")]
type Cypd   = cypd3177::Cypd3177<I2cBus>;
type Bq    = bq76952::Bq76952<I2cDev<'static>>;
#[embassy_executor::task]
async fn usb_task(mut dev: embassy_usb::UsbDevice<'static, UsbDrv>) {
    defmt::info!("USB device: run()");
    dev.run().await;
}

#[embassy_executor::task]
#[cfg(feature = "cypd")]
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

#[cfg(not(feature = "cypd"))]
#[embassy_executor::task]
async fn cdc_task(
    mut cdc: embassy_usb::class::cdc_acm::CdcAcmClass<'static, UsbDrv>,
) {
    // Reuse the same implementation but ignore CYPD-related behaviour.
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

#[cfg(feature = "cypd")]
#[embassy_executor::task]
async fn cypd_monitor_task(cypd: &'static Mutex<CriticalSectionRawMutex, Cypd>) {
    use embassy_time::{Ticker, Duration};

    {
        let mut dev = cypd.lock().await;

        if let Ok(dm) = dev.device_mode().await {
            defmt::info!("CYPD DEVICE_MODE=0x{:02X} (informational)", dm);
        } else {
            defmt::warn!("CYPD DEVICE_MODE read failed");
        }

        if let Ok(id) = dev.silicon_id().await {
            defmt::info!("CYPD SILICON_ID=0x{:04X}", id);
        } else {
            defmt::warn!("CYPD SILICON_ID read failed");
        }

        // These status regs can be zero until a partner is attached; that’s fine.
        if let Ok(s) = dev.typec_status().await {
            defmt::info!("TypeC: connected={} polarity={} current_level={}",
                s.connected, if s.cc_polarity_cc2 { "CC2" } else { "CC1" }, s.current_level);
        }
        if let Ok(s) = dev.pd_status().await {
            defmt::info!("PD: contract={} sink_tx_ok={} pe_snk_ready={}",
                s.contract, s.sink_tx_ok, s.pe_snk_ready);
        }
        if let Ok(mv) = dev.bus_voltage_mv().await {
            defmt::info!("VBUS={} mV", mv);
        }
    }
}

#[cfg(feature = "cypd")]
#[embassy_executor::task]
async fn hpi_int_task(
    mut int_pin: ExtiInput<'static>,
    cypd: &'static Mutex<CriticalSectionRawMutex, Cypd>,
) {
    use embassy_time::Timer;

    // Helper to pretty-print PD/Type-C snapshots
    async fn snapshot(dev: &mut Cypd) {
        match dev.pd_status().await {
            Ok(s) => defmt::info!(
                "PD_STATUS: contract={} sink_tx_ok={} pe_snk_ready={}",
                s.contract, s.sink_tx_ok, s.pe_snk_ready
            ),
            Err(_) => defmt::warn!("PD_STATUS read failed"),
        }
        match dev.typec_status().await {
            Ok(s) => defmt::info!(
                "TYPEC_STATUS: connected={} polarity={} current_level={}",
                s.connected, if s.cc_polarity_cc2 { "CC2" } else { "CC1" }, s.current_level
            ),
            Err(_) => defmt::warn!("TYPEC_STATUS read failed"),
        }
        if let Ok(mv) = dev.bus_voltage_mv().await {
            defmt::info!("VBUS={} mV", mv);
        }
        if let Ok(pdo) = dev.current_pdo_raw().await {
            defmt::info!("CURRENT_PDO raw: 0x{:08X}", pdo);
        }
        if let Ok(rdo) = dev.current_rdo_raw().await {
            defmt::info!("CURRENT_RDO raw: 0x{:08X}", rdo);
        }
        if let Ok(ev) = dev.event_status().await {
            defmt::info!("EVENT_STATUS=0x{:02X}", ev);
        }
    }

    loop {
        // Wait until the HPI line is asserted (active-low)
        int_pin.wait_for_falling_edge().await;
        defmt::info!("HPI_INT asserted (low)");

        // Drain all pending HPI causes while the line stays low.
        // The line should deassert only after we read/clear all pending causes.
        loop {
            // Do the HPI work in a short critical section (don’t hold the lock across awaits on the GPIO).
            {
                let mut dev = cypd.lock().await;

                // 0x0006 = INTERRUPT (bit0=device, bit1=pd_port); W1C.
                // The CYPD can NACK for a few ms right after attach/detach – retry briefly.
                let flags = {
                    let mut got: Option<u8> = None;
                    for attempt in 0..8 {
                        match dev.interrupt_status().await {
                            Ok(f) => { got = Some(f); break; }
                            Err(CypdError::I2c(_)) => {
                                defmt::warn!("INT@0x0006 NACK/busy (try {})", attempt);
                                Timer::after_millis(1 + attempt).await; // 1,2,3,... ms
                            }
                            Err(_) => {
                                defmt::warn!("INT@0x0006 read failed (non-I2C)");
                                break;
                            }
                        }
                    }
                    match got {
                        Some(f) => f,
                        None => {
                            // Bail out of this IRQ burst; we’ll re-arm after the line deasserts.
                            drop(dev);
                            int_pin.wait_for_rising_edge().await;
                            Timer::after_millis(1).await;
                            continue;
                        }
                    }
                };

                let dev_int     = (flags & 0x01) != 0;
                let pd_port_int = (flags & 0x02) != 0;
                defmt::info!("INTERRUPT=0x{:02X} dev_int={} pd_port_int={}", flags, dev_int, pd_port_int);

                snapshot(&mut dev).await;

                let mut clear_mask: u8 = 0;

                // PD Service Queue
                if pd_port_int {
                    // Service the PD queue (PD_RESPONSE @ 0x1400)
                    if let Ok(hdr) = dev.pd_response_header().await {
                        let code   = hdr[0];
                        let len_lo = hdr[1];
                        let len_hi = u16::from_le_bytes([hdr[2], hdr[3]]);
                        let total_len: u32 = if len_hi == 0 { len_lo as u32 } else { len_hi as u32 };
                        defmt::info!("PD_RESPONSE: code=0x{:02X} len={}", code, total_len);

                        if total_len > 4 {
                            let pay_len = core::cmp::min((total_len - 4) as usize, 24);
                            let mut pay = [0u8; 24];
                            let _ = dev.pd_response_payload(&mut pay[..pay_len]).await;
                            defmt::info!("PD_RESPONSE payload ({}B): {:x}", pay_len, &pay[..pay_len]);
                        }
                    }
                    clear_mask |= 0x02;  // W1C bit1 only if we serviced PD queue
                }

                // Dev Service Queue
                if dev_int {
                    // Service the device queue (DEV_RESPONSE @ 0x007E)
                    if let Ok(dr) = dev.dev_response_code().await {
                        defmt::info!("DEV_RESPONSE code=0x{:02X}", dr);
                    }
                    clear_mask |= 0x01;  // W1C bit0 only if we serviced DEV queue
                }
                let ev = dev.event_status().await.unwrap_or(0);
                if ev != 0 {
                    // (Optional) quick decode of the most useful bits:
                    if (ev & (1 << 0)) != 0 { defmt::info!("EVENT: Type-C device attached"); }
                    if (ev & (1 << 1)) != 0 { defmt::info!("EVENT: Type-C device disconnected"); }
                    if (ev & (1 << 2)) != 0 { defmt::info!("EVENT: PD contract completed"); }
                    if (ev & (1 << 29)) != 0 { defmt::warn!("EVENT: Unexpected voltage on VBUS"); }
                    if (ev & (1 << 30)) != 0 { defmt::warn!("EVENT: VBUS outside expected range"); }
                    // Clear only what we observed (W1C):
                    #[cfg(feature = "cypd-write")]
                    let _ = dev.clear_event_status(ev).await;
                }
                // Write-1-to-clear only what we handled.
                #[cfg(feature = "cypd-write")]
                if clear_mask != 0 {
                    let _ = dev.clear_interrupts(clear_mask).await; // W1C @ 0x0006
                }

            } // <- mutex guard dropped here before we await on GPIO again

            // If more causes are pending the line will still be low and we’ll loop again;
            // otherwise it will go high and we re-arm on the next falling edge.
            if int_pin.is_high() {
                break;
            }

            // Give the HPI a couple of ms to release the line after W1C (helps on slow I²C).
            Timer::after_millis(2).await;
        }

        // Re-arm cleanly: wait for the line to deassert so the next falling edge is distinct.
        int_pin.wait_for_rising_edge().await;
        Timer::after_millis(1).await;
    }
}

#[cfg(feature = "cypd")]
#[embassy_executor::task]
async fn wait_for_contract(cypd: &'static Mutex<CriticalSectionRawMutex, Cypd>) {
    use embassy_time::{Timer, Duration};
    loop {
        {
            let mut dev = cypd.lock().await;
            if let Ok(s) = dev.pd_status().await {
                if s.contract && s.pe_snk_ready {
                    if let Ok(mv) = dev.bus_voltage_mv().await {
                        if mv >= 4500 {
                            defmt::info!("PD contract ready, VBUS ~{} mV — FET_EN should be high (owned by BCR).", mv);
                            break;
                        }
                    }
                }
            }
        }
        Timer::after(Duration::from_millis(50)).await;
    }
}

#[cfg(feature = "cypd")]
#[embassy_executor::task]
async fn event_status_poll_task(
    cypd: &'static Mutex<CriticalSectionRawMutex, Cypd>,
) {
    use embassy_time::{Timer, Duration};

    loop {
        // Light polling (e.g., 50–100 ms) is fine when nothing’s happening.
        Timer::after(Duration::from_millis(100)).await;
        // If IRQ work is pending, let the IRQ task own the bus
        let irq_pending = {
            let mut dev = cypd.lock().await;
            dev.interrupt_status().await.unwrap_or(0) != 0
        };
        if irq_pending {
            // Back off a little; IRQ task will drain PD/DEV queues.
            Timer::after(Duration::from_millis(5)).await;
            continue;
        }

        // Read/clear sticky EVENT_STATUS (W1C) for informational events
        let to_clear = {
            let mut dev = cypd.lock().await;
            match dev.event_status().await {
                Ok(ev) if ev != 0 => {
                    defmt::info!(
                        "EVENT_STATUS=0x{:08X} (attached={} detached={} contract_done={})",
                        ev,
                        (ev & (1 << 0)) != 0,
                        (ev & (1 << 1)) != 0,
                        (ev & (1 << 2)) != 0,
                    );
                    ev
                }
                _ => 0
            }
        };

        if to_clear != 0 {
            let mut dev = cypd.lock().await;
            #[cfg(feature = "cypd-write")]
            let _ = dev.clear_event_status(to_clear).await; // your W1C helper
        }
    }
}


#[embassy_executor::task]
async fn bq_address_demo_and_monitor(
    bq_mutex: &'static Mutex<CriticalSectionRawMutex, Bq>,
) {
    use embassy_time::{Timer, Duration};
    use bq76952::{Thermistor};

    // Small async delay that implements embedded_hal_async::delay::DelayNs.
    struct Delay;
    impl embedded_hal_async::delay::DelayNs for Delay {
        async fn delay_ns(&mut self, ns: u32) { Timer::after(Duration::from_nanos(ns as u64)).await; }
        async fn delay_us(&mut self, us: u32) { Timer::after(Duration::from_micros(us as u64)).await; }
        async fn delay_ms(&mut self, ms: u32) { Timer::after(Duration::from_millis(ms as u64)).await; }
    }
    let mut d = Delay;

    // 1) Talk to default address (0x08) and read something simple (Battery Status).
    {
        let mut bq = bq_mutex.lock().await;
        match bq.get_battery_status().await {
            Ok(s) => defmt::info!("BQ@0x08 BatteryStatus: cfg_update={} sealed?={}",
                s.config_update_mode, (s.security_state == 0)), // rough indicator
            Err(_) => defmt::warn!("BQ@0x08 initial read failed"),
        }
    }

    // 2) Ephemeral address change demo: 0x08 -> 0x0B (arbitrary choice).
    {
        let mut bq = bq_mutex.lock().await;
        if let Err(e) = bq.set_i2c_address_ram(0x0B, &mut d).await {
            defmt::warn!("BQ: set_i2c_address_ram failed: {:?}", defmt::Debug2Format(&e));
            return;
        }
        defmt::info!("BQ: changed RAM I²C address to 0x0B and applied via SWAP_COMM_MODE.");
    }

    // 3) Proof: re-read from the **new** address using the same driver (it updated its addr).
    {
        let mut bq = bq_mutex.lock().await;
        match (bq.get_battery_status().await, bq.get_stack_voltage().await) {
            (Ok(s), Ok(v)) => {
                defmt::info!("BQ@0x0B OK: cfg_update={} vstack(raw)=0x{:04X}", s.config_update_mode, v);
            }
            _ => defmt::warn!("BQ@0x0B read failed"),
        }
    }

    // 4) Light periodic monitor (every 500 ms).
    loop {
        {
            let mut bq = bq_mutex.lock().await;
            if let Ok(v10mv) = bq.get_stack_voltage().await {
                defmt::info!("BQ: VSTACK raw=0x{:04X}", v10mv);
            }
            if let Ok(cur) = bq.get_current_cc2().await {
                defmt::info!("BQ: CC2 current = {} (user units)", cur);
            }
            if let Ok(t) = bq.get_internal_temp_c().await {
                defmt::info!("BQ: die temp ~{=f32}C", t);
            }
        }
        Timer::after(Duration::from_millis(500)).await;
    }
}
