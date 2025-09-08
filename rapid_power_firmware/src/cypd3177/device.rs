//! CYPD3177 USB-PD (EZ-PD BCR) async driver
//! HPI over I²C (7-bit addr 0x08). LSB-first 16-bit register addresses.

use embedded_hal_async::i2c::I2c;
use defmt::{self, trace, warn};
use embassy_stm32::exti::ExtiInput;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_time::{Duration, Timer};

use super::regs::*;
use super::types::*;
use crate::{Cypd, CypdPublisher};
use crate::cypd3177::CypdError;
use crate::shared_state::CypdReadings;

#[derive(Debug, Clone, Copy)]
pub struct CypdId {
    pub device_mode: u8,
    pub silicon_id: u16,
}

pub struct Cypd3177<I2C> {
    i2c: I2C,
    addr: u8,
}

impl<I2C> Cypd3177<I2C>
where
    I2C: I2c,
{
    /// Returns Ok(id) if DEVICE_MODE/SILICON_ID look sane; Err otherwise.
    pub async fn probe_plausible(&mut self) -> Result<CypdId, CypdError<I2C::Error>> {
        use embassy_time::Timer;

        // Try a few times; the BCR can NACK/return zeros briefly during its internal boot
        for attempt in 0..5 {
            let dm = self.device_mode().await.unwrap_or(0x00);
            let sid = self.silicon_id().await.unwrap_or(0x0000);

            // Heuristics: both must be non-zero and not all-ones.
            // (Docs often show DEVICE_MODE ~0x92 and SILICON_ID ~0x11B0, but don’t hardcode.)
            let looks_ok = dm != 0x00 && dm != 0xFF && sid != 0x0000 && sid != 0xFFFF;

            if looks_ok {
                return Ok(CypdId { device_mode: dm, silicon_id: sid });
            }

            // Short backoff; increase slightly on each try
            Timer::after_millis(2 + attempt).await;
        }

        Err(CypdError::Protocol("implausible CYPD ID (zeros or 0xFFFF)"))
    }
    pub fn new(i2c: I2C) -> Self {
        Self { i2c, addr: 0x08 }
    }
    pub fn with_address(i2c: I2C, addr: u8) -> Self {
        Self { i2c, addr }
    }
    pub fn release(self) -> I2C {
        self.i2c
    }

    // ===== Low-level HPI =====

    /// HPI requires LSB, MSB for the 16-bit register address.
    async fn wr(&mut self, reg: u16, data: &[u8]) -> Result<(), CypdError<I2C::Error>> {
        let mut buf = [0u8; 64];
        if data.len() + 2 > buf.len() {
            return Err(CypdError::Protocol("tx too long"));
        }
        buf[0] = reg as u8; // LSB
        buf[1] = (reg >> 8) as u8; // MSB
        buf[2..2 + data.len()].copy_from_slice(data);

        #[cfg(feature = "cypd-trace")]
        trace!("I2C WR @0x{:04X} <- {:x}", reg, &data);

        let res = self.i2c.write(self.addr, &buf[..2 + data.len()]).await;
        #[cfg(feature = "cypd-trace")]
        match &res {
            Ok(()) => trace!("I2C WR OK @0x{:04X} ({}B)", reg, data.len()),
            Err(_) => warn!("I2C WR ERR @0x{:04X}", reg),
        }
        res.map_err(CypdError::I2c)
    }

    async fn rd(&mut self, reg: u16, dst: &mut [u8]) -> Result<(), CypdError<I2C::Error>> {
        let reg_bytes = [reg as u8, (reg >> 8) as u8]; // LSB, MSB

        #[cfg(feature = "cypd-trace")]
        trace!("I2C RD @0x{:04X} -> {}B", reg, dst.len());

        let res = self.i2c.write_read(self.addr, &reg_bytes, dst).await;

        #[cfg(feature = "cypd-trace")]
        match &res {
            Ok(()) => trace!("I2C RD OK @0x{:04X}: {:x}", reg, &dst),
            Err(_) => warn!("I2C RD ERR @0x{:04X}", reg),
        }
        res.map_err(CypdError::I2c)
    }

    async fn rd_u8(&mut self, reg: u16) -> Result<u8, CypdError<I2C::Error>> {
        let mut b = [0u8; 1];
        self.rd(reg, &mut b).await?;
        Ok(b[0])
    }
    async fn rd_u16(&mut self, reg: u16) -> Result<u16, CypdError<I2C::Error>> {
        let mut b = [0u8; 2];
        self.rd(reg, &mut b).await?;
        Ok(u16::from_le_bytes(b))
    }
    async fn rd_u32(&mut self, reg: u16) -> Result<u32, CypdError<I2C::Error>> {
        let mut b = [0u8; 4];
        self.rd(reg, &mut b).await?;
        Ok(u32::from_le_bytes(b))
    }

    // ===== Typed registers (per HPI docs) =====

    /// DEVICE_MODE (0x0000) — BCR returns a fixed ID byte (doc shows 0x92).
    /// Treat it as informational; don't gate logic on a specific literal.
    pub async fn device_mode(&mut self) -> Result<u8, CypdError<I2C::Error>> {
        self.rd_u8(DEVICE_MODE).await
    }

    pub async fn silicon_id_raw(&mut self) -> Result<[u8; 2], CypdError<I2C::Error>> {
        let mut b = [0u8; 2];
        self.rd(SILICON_ID, &mut b).await?;
        Ok(b)
    }

    /// SILICON_ID (0x0002) — BCR doc shows 0x11B0. Just log for info.
    pub async fn silicon_id(&mut self) -> Result<u16, CypdError<I2C::Error>> {
        let raw = self.silicon_id_raw().await?;
        Ok(u16::from_be_bytes(raw))
    }

    /// BUS_VOLTAGE (0x100D): value * 100 = mV.
    pub async fn bus_voltage_mv(&mut self) -> Result<u16, CypdError<I2C::Error>> {
        let v = self.rd_u8(BUS_VOLTAGE).await?;
        Ok((v as u16) * 100)
    }

    /// TYPE_C_STATUS (0x100C)
    pub async fn typec_status(&mut self) -> Result<TypeCStatus, CypdError<I2C::Error>> {
        let raw = self.rd_u32(TYPE_C_STATUS).await?;
        Ok(TypeCStatus::from(raw))
    }

    /// PD_STATUS (0x1008)
    pub async fn pd_status(&mut self) -> Result<PdStatus, CypdError<I2C::Error>> {
        let raw = self.rd_u32(PD_STATUS).await?;
        Ok(PdStatus::from(raw))
    }

    /// DEV_RESPONSE (0x007E) — first byte is response code
    pub async fn dev_response_code(&mut self) -> Result<u8, CypdError<I2C::Error>> {
        self.rd_u8(DEV_RESPONSE).await
    }

    /// PD_RESPONSE (0x1400) — first byte is response code (e.g. 0x02 = success)
    pub async fn pd_response(&mut self) -> Result<PdResponse, CypdError<I2C::Error>> {
        let mut hdr = [0u8; 4];
        self.rd(PD_RESPONSE, &mut hdr).await?;
        Ok(PdResponse {
            code: hdr[0], // 0x02=success, 0x01=fail, 0x03=busy, bit7 async-event
            len_lo: hdr[1],
            len_hi: u16::from_le_bytes([hdr[2], hdr[3]]),
        })
    }

    pub async fn interrupt_status(&mut self) -> Result<u8, CypdError<I2C::Error>> {
        self.rd_u8(INTERRUPT).await
    }

    /// EVENT_STATUS (0x1044) — sticky event bits (datasheet-defined)
    pub async fn event_status(&mut self) -> Result<u32, CypdError<I2C::Error>> {
        self.rd_u32(EVENT_STATUS).await
    }

    pub async fn clear_event_status(&mut self, mask: u32) -> Result<(), CypdError<I2C::Error>> {
        self.wr(EVENT_STATUS, &mask.to_le_bytes()).await
    }

    pub async fn clear_interrupts(&mut self, mask: u8) -> Result<(), CypdError<I2C::Error>> {
        self.wr(INTERRUPT, &[mask]).await
    }

    /// CURRENT_PDO (0x1010) — raw 32-bit PDO the port partner selected for CYPD
    pub async fn current_pdo_raw(&mut self) -> Result<u32, CypdError<I2C::Error>> {
        self.rd_u32(CURRENT_PDO).await
    }

    /// CURRENT_RDO (0x1014) — raw 32-bit request data object
    pub async fn current_rdo_raw(&mut self) -> Result<u32, CypdError<I2C::Error>> {
        self.rd_u32(CURRENT_RDO).await
    }

    /// Read PD_RESPONSE header (0x1400..0x1403)
    pub async fn pd_response_header(&mut self) -> Result<[u8; 4], CypdError<I2C::Error>> {
        let mut hdr = [0u8; 4];
        self.rd(PD_RESPONSE, &mut hdr).await?;
        Ok(hdr)
    }

    /// Read up to `buf.len()` bytes of PD_RESPONSE payload starting at 0x1404
    pub async fn pd_response_payload(
        &mut self,
        buf: &mut [u8],
    ) -> Result<(), CypdError<I2C::Error>> {
        if buf.is_empty() {
            return Ok(());
        }
        self.rd(PD_RESPONSE_PAYLOAD, buf).await
    }

    /// Set a minimal, useful event mask: attach/detach/contract (bits 3,4,5) 0x38. 0x1024 is LE 32-bit.
    pub async fn enable_basic_events(&mut self) -> Result<(), CypdError<I2C::Error>> {
        self.wr(EVENT_MASK, &[0x38, 0x00, 0x00, 0x00]).await
    }

    /// Clear bits in INTERRUPT (0x0006, W1C) after servicing queues.
    pub async fn clear_interrupt_bits(&mut self, mask: u8) -> Result<(), CypdError<I2C::Error>> {
        self.wr(INTERRUPT, &[mask]).await
    }

    pub async fn reset_device(&mut self) -> Result<(), CypdError<I2C::Error>> {
        self.wr(RESET, &[0x52, 0x01]).await // 'R', device reset
    }

    pub async fn write_sink_pdos(&mut self, pdos: &[u32]) -> Result<(), CypdError<I2C::Error>> {
        if pdos.len() > 7 {
            return Err(CypdError::Protocol("up to 7 PDOs"));
        }

        // Wire order must be P K N S (0x50, 0x4B, 0x4E, 0x53)
        let mut buf = [0u8; 4 + 7 * 4];
        buf[0..4].copy_from_slice(&[0x50, 0x4B, 0x4E, 0x53]);

        for (i, p) in pdos.iter().enumerate() {
            buf[4 + i * 4..8 + i * 4].copy_from_slice(&p.to_le_bytes());
        }
        self.wr(WRITE_SINK_PDOS, &buf).await
    }

    pub async fn select_sink_pdo_mask(&mut self, mask: u8) -> Result<(), CypdError<I2C::Error>> {
        if mask == 0 {
            return Err(CypdError::Protocol("mask must be non-zero"));
        }
        self.wr(SELECT_SINK_PDO_MASK, &[mask]).await
    }
}

#[embassy_executor::task]
pub async fn cypd_monitor_task(
    cypd: &'static Mutex<CriticalSectionRawMutex, Cypd<'static>>,
) {
    let mut publisher = crate::shared_state::CYPD_CHANNEL.publisher().unwrap();
    use embassy_time::Ticker;

    // One-time info snapshot + gate the task if implausible
    {
        let mut dev = cypd.lock().await;

        let dm = dev.device_mode().await.unwrap_or(0x00);
        let sid = dev.silicon_id().await.unwrap_or(0x0000);

        let plausible = dm != 0x00 && dm != 0xFF && sid != 0x0000 && sid != 0xFFFF;

        if !plausible {
            defmt::warn!(
                "CYPD: implausible ID (DEVICE_MODE=0x{:02X}, SILICON_ID=0x{:04X}) — no device / no USB / bus issue. Task will idle.",
                dm, sid
            );
            // Don’t publish anything; just park this task forever
            loop { embassy_time::Timer::after_millis(500).await; }
        }

        defmt::info!("CYPD ok: DEVICE_MODE=0x{:02X}, SILICON_ID=0x{:04X}", dm, sid);

        if let Ok(s) = dev.typec_status().await {
            defmt::info!(
                "TypeC: connected={} polarity={} current_level={}",
                s.connected,
                if s.cc_polarity_cc2 { "CC2" } else { "CC1" },
                s.current_level
            );
        }
        if let Ok(s) = dev.pd_status().await {
            defmt::info!(
                "PD: contract={} sink_tx_ok={} pe_snk_ready={}",
                s.contract,
                s.sink_tx_ok,
                s.pe_snk_ready
            );
        }
        if let Ok(mv) = dev.bus_voltage_mv().await {
            defmt::info!("VBUS={} mV", mv);
        }
    }

    // Periodic publish
    let mut ticker = Ticker::every(Duration::from_secs(1));
    loop {
        let readings = {
            let mut dev = cypd.lock().await;
            let typec_status = dev.typec_status().await.unwrap_or_default();
            let pd_status = dev.pd_status().await.unwrap_or_default();
            let vbus_mv = dev.bus_voltage_mv().await.unwrap_or(0);

            CypdReadings {
                vbus_mv,
                typec_connected: typec_status.connected,
                typec_polarity_cc2: typec_status.cc_polarity_cc2,
                typec_current_level: typec_status.current_level as u8,
                pd_contract: pd_status.contract,
                pd_sink_tx_ok: pd_status.sink_tx_ok,
                pd_pe_snk_ready: pd_status.pe_snk_ready,
            }
        };

        publisher.publish(readings).await;
        defmt::info!("CYPD data published (chip present).");

        ticker.next().await;
    }
}

#[embassy_executor::task]
pub async fn hpi_int_task(
    mut int_pin: ExtiInput<'static>,
    cypd: &'static Mutex<CriticalSectionRawMutex, Cypd<'static>>,
) {
    // Helper to pretty-print PD/Type-C snapshots
    async fn snapshot(dev: &mut Cypd<'static>) {
        match dev.pd_status().await {
            Ok(s) => defmt::info!(
                "PD_STATUS: contract={} sink_tx_ok={} pe_snk_ready={}",
                s.contract,
                s.sink_tx_ok,
                s.pe_snk_ready
            ),
            Err(_) => defmt::warn!("PD_STATUS read failed"),
        }
        match dev.typec_status().await {
            Ok(s) => defmt::info!(
                "TYPEC_STATUS: connected={} polarity={} current_level={}",
                s.connected,
                if s.cc_polarity_cc2 { "CC2" } else { "CC1" },
                s.current_level
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
                            Ok(f) => {
                                got = Some(f);
                                break;
                            }
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

                let dev_int = (flags & 0x01) != 0;
                let pd_port_int = (flags & 0x02) != 0;
                defmt::info!(
                    "INTERRUPT=0x{:02X} dev_int={} pd_port_int={}",
                    flags,
                    dev_int,
                    pd_port_int
                );

                snapshot(&mut dev).await;

                let mut clear_mask: u8 = 0;

                // PD Service Queue
                if pd_port_int {
                    // Service the PD queue (PD_RESPONSE @ 0x1400)
                    if let Ok(hdr) = dev.pd_response_header().await {
                        let code = hdr[0];
                        let len_lo = hdr[1];
                        let len_hi = u16::from_le_bytes([hdr[2], hdr[3]]);
                        let total_len: u32 = if len_hi == 0 {
                            len_lo as u32
                        } else {
                            len_hi as u32
                        };
                        defmt::info!("PD_RESPONSE: code=0x{:02X} len={}", code, total_len);

                        if total_len > 4 {
                            let pay_len = core::cmp::min((total_len - 4) as usize, 24);
                            let mut pay = [0u8; 24];
                            let _ = dev.pd_response_payload(&mut pay[..pay_len]).await;
                            defmt::info!(
                                "PD_RESPONSE payload ({}B): {:x}",
                                pay_len,
                                &pay[..pay_len]
                            );
                        }
                    }
                    clear_mask |= 0x02; // W1C bit1 only if we serviced PD queue
                }

                // Dev Service Queue
                if dev_int {
                    // Service the device queue (DEV_RESPONSE @ 0x007E)
                    if let Ok(dr) = dev.dev_response_code().await {
                        defmt::info!("DEV_RESPONSE code=0x{:02X}", dr);
                    }
                    clear_mask |= 0x01; // W1C bit0 only if we serviced DEV queue
                }

                let ev = dev.event_status().await.unwrap_or(0);
                if ev != 0 {
                    // (Optional) quick decode of useful bits:
                    if (ev & (1 << 0)) != 0 {
                        defmt::info!("EVENT: Type-C device attached");
                    }
                    if (ev & (1 << 1)) != 0 {
                        defmt::info!("EVENT: Type-C device disconnected");
                    }
                    if (ev & (1 << 2)) != 0 {
                        defmt::info!("EVENT: PD contract completed");
                    }
                    if (ev & (1 << 29)) != 0 {
                        defmt::warn!("EVENT: Unexpected voltage on VBUS");
                    }
                    if (ev & (1 << 30)) != 0 {
                        defmt::warn!("EVENT: VBUS outside expected range");
                    }

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

#[embassy_executor::task]
pub async fn wait_for_contract(cypd: &'static Mutex<CriticalSectionRawMutex, Cypd<'static>>) {
    loop {
        {
            let mut dev = cypd.lock().await;
            if let Ok(s) = dev.pd_status().await {
                if s.contract && s.pe_snk_ready {
                    if let Ok(mv) = dev.bus_voltage_mv().await {
                        if mv >= 4500 {
                            defmt::info!(
                                "PD contract ready, VBUS ~{} mV — FET_EN should be high (owned by BCR).",
                                mv
                            );
                            break;
                        }
                    }
                }
            }
        }
        Timer::after(Duration::from_millis(50)).await;
    }
}

#[embassy_executor::task]
pub async fn event_status_poll_task(
    cypd: &'static Mutex<CriticalSectionRawMutex, Cypd<'static>>,
) {
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
                _ => 0,
            }
        };

        if to_clear != 0 {
            let mut dev = cypd.lock().await;
            let _ = dev.clear_event_status(to_clear).await; // your W1C helper
        }
    }
}
