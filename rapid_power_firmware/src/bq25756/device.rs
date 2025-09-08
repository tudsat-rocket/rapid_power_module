//! High-level driver API (async, owns I²C device so it can live in a Mutex).

use core::result::Result as CoreResult;
use embedded_hal_async::i2c::I2c;
use defmt;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_time::{Duration, Ticker, with_timeout};
use embassy_stm32::exti::ExtiInput;
use embassy_stm32::gpio::Pull;

use crate::bq25756::regs::*;
use crate::bq25756::types::*;

use crate::shared_state::{Bq25756Command, Bq25756Readings};

use crate::config;

pub type Result<E> = CoreResult<(), Error<E>>;

/// BQ25756 async driver over I²C.
pub struct Bq25756<I2C> {
    i2c: I2C,
    addr: u8,
}

const I2C_OP_TIMEOUT: Duration = Duration::from_millis(30);

#[inline]
fn u16_to_i16(x: u16) -> i16 { x as i16 }

async fn with_i2c_timeout<F, T, E>(fut: F) -> CoreResult<T, Error<E>>
where
    F: core::future::Future<Output = CoreResult<T, Error<E>>>
{
    match with_timeout(I2C_OP_TIMEOUT, fut).await {
        Ok(r) => r,
        Err(_) => Err(Error::Timeout), // add Error::I2cTimeout to your Error enum
    }
}

impl<I2C> Bq25756<I2C>
where
    I2C: I2c,
{
    /// One-shot bring-up for 3S Li-ion/LiPo @ 1A fast-charge.
    /// Assumes CE pin is held LOW by the MCU.
    pub async fn bringup_3s_1a_defaults(&mut self) -> Result<I2C::Error> {
        // 1) Watchdog off (host mode without periodic petting)
        self.set_watchdog_timer(crate::bq25756::types::WdtTimer::Disable).await?;

        // 2) Ensure not in Hi-Z
        self.enable_hiz(false).await?;

        // 3) DPMs: pick sane defaults for a 9–12 V source
        //    - If you *know* your source (e.g. fixed 9 V PD), set VIN_DPM ~ 9000 mV.
        //    - If 12 V, bump to ~11000 mV to leave margin.
        self.set_input_voltage_dpm_limit(5_000).await?;
        //    Limit adapter draw to 1A (adjust upward if you have system load headroom)
        self.set_input_current_dpm_limit(1_000).await?;

        // 4) Charge targets for a 3S pack
        //    - CV: 12.60 V (4.20 V/cell)
        //    - CC: 1.00 A
        self.set_charge_voltage_limit(12_600).await?;
        self.set_charge_current_limit(1_000).await?;

        // 5) Precharge / termination
        //    Typical starting points: precharge ≈ 10% of ICHG, termination ≈ 100 mA
        // self.set_precharge_current_limit(100).await?;
        // self.set_termination_current_limit(100).await?;

        // 6) Finally: enable charging
        self.enable_charger(true).await?;

        Ok(())
    }
    /// Create with default 7-bit I²C address (0x6B).
    pub fn new(i2c: I2C) -> Self { Self { i2c, addr: I2C_ADDR } }

    /// Create with explicit address.
    pub fn with_address(i2c: I2C, addr: u8) -> Self { Self { i2c, addr } }

    /// Consume and return the underlying I²C device.
    pub fn release(self) -> I2C { self.i2c }

    // ------------------ Core / Identity ------------------

    pub async fn whoami(&mut self) -> CoreResult<u8, Error<I2C::Error>> {
        let v = self.read1(REG_PART_INFO).await?;
        Ok((v & PART_NUM_MASK) >> 3)
    }

    /// Optional probe that validates the part number.
    pub async fn probe(&mut self) -> Result<I2C::Error> {
        match self.whoami().await? {
            0x02 => Ok(()),
            other => Err(Error::InvalidDevice(other)),
        }
    }

    // ------------------ Helpers ------------------

    /// Read single-byte register.
    async fn read1(&mut self, reg: u8) -> CoreResult<u8, Error<I2C::Error>> {
        with_i2c_timeout(async {
            let mut buf = [0u8; 1];
            self.i2c
                .write_read(self.addr, &[reg], &mut buf)
                .await
                .map_err(Error::I2c)?;
            Ok(buf[0])
        }).await
    }

    /// Read two-byte register.
    async fn read2(&mut self, reg: u8) -> CoreResult<u16, Error<I2C::Error>> {
        with_i2c_timeout(async {
            let mut buf = [0u8; 2];
            self.i2c
                .write_read(self.addr, &[reg], &mut buf)
                .await
                .map_err(Error::I2c)?;
            Ok(u16::from_le_bytes(buf))
        }).await
    }

    /// Write single-byte register.
    async fn write1(&mut self, reg: u8, val: u8) -> Result<I2C::Error> {
        with_i2c_timeout(async {
            self.i2c
                .write(self.addr, &[reg, val])
                .await
                .map_err(Error::I2c)
        }).await
    }

    /// Write two-byte register.
    async fn write2(&mut self, reg: u8, val: u16) -> Result<I2C::Error> {
        with_i2c_timeout(async {
            let bytes = val.to_le_bytes();
            self.i2c
                .write(self.addr, &[reg, bytes[0], bytes[1]])
                .await
                .map_err(Error::I2c)
        }).await
    }

    pub async fn int_unmask_all(&mut self) -> Result<I2C::Error> {
        self.write1(REG_CHARGER_MASK_1, 0x00).await?;
        self.write1(REG_CHARGER_MASK_2, 0x00).await?;
        self.write1(REG_FAULT_MASK,      0x00).await
    }

    /// Read all FLAG registers (ClearOnRead) then a STATUS snapshot.
    pub async fn read_int_cause_and_status(
        &mut self
    ) -> CoreResult<((u8,u8,u8),(u8,u8,u8,u8)), Error<I2C::Error>> {
        let f1 = self.read1(REG_CHARGER_FLAG_1).await?; // clear-on-read
        let f2 = self.read1(REG_CHARGER_FLAG_2).await?; // clear-on-read
        let ff = self.read1(REG_FAULT_FLAG).await?;     // clear-on-read
        let s1 = self.read1(REG_CHARGER_STATUS_1).await?;
        let s2 = self.read1(REG_CHARGER_STATUS_2).await?;
        let s3 = self.read1(REG_CHARGER_STATUS_3).await?;
        let fs = self.read1(REG_FAULT_STATUS).await?;
        Ok(((f1,f2,ff),(s1,s2,s3,fs)))
    }
    // ------------------ Charging Parameters ------------------

    /// Set charge voltage limit (Vreg).
    pub async fn set_charge_voltage_limit(&mut self, mv: u16) -> Result<I2C::Error> {
        let mv = mv.clamp(8192, 19200);
        self.write2(REG_CHARGE_VOLTAGE_LIMIT, mv).await
    }

    /// Read charge voltage limit (Vreg).
    pub async fn get_charge_voltage_limit(&mut self) -> CoreResult<u16, Error<I2C::Error>> {
        self.read2(REG_CHARGE_VOLTAGE_LIMIT).await
    }

    /// Set charge current limit (Ireg).
    pub async fn set_charge_current_limit(&mut self, ma: u16) -> Result<I2C::Error> {
        let ma = ma.clamp(0, 8000);
        self.write2(REG_CHARGE_CURRENT_LIMIT, ma).await
    }

    /// Read charge current limit (Ireg).
    pub async fn get_charge_current_limit(&mut self) -> CoreResult<u16, Error<I2C::Error>> {
        self.read2(REG_CHARGE_CURRENT_LIMIT).await
    }

    /// Set input current DPM limit (IIN_DPM).
    pub async fn set_input_current_dpm_limit(&mut self, ma: u16) -> Result<I2C::Error> {
        let ma = ma.clamp(0, 8000);
        self.write2(REG_INPUT_CURRENT_DPM_LIMIT, ma).await
    }

    /// Read input current DPM limit (IIN_DPM).
    pub async fn get_input_current_dpm_limit(&mut self) -> CoreResult<u16, Error<I2C::Error>> {
        self.read2(REG_INPUT_CURRENT_DPM_LIMIT).await
    }

    /// Set input voltage DPM limit (VIN_DPM).
    pub async fn set_input_voltage_dpm_limit(&mut self, mv: u16) -> Result<I2C::Error> {
        let mv = mv.clamp(3600, 22000);
        self.write2(REG_INPUT_VOLTAGE_DPM_LIMIT, mv).await
    }

    /// Read input voltage DPM limit (VIN_DPM).
    pub async fn get_input_voltage_dpm_limit(&mut self) -> CoreResult<u16, Error<I2C::Error>> {
        self.read2(REG_INPUT_VOLTAGE_DPM_LIMIT).await
    }

    // ------------------ JEITA and Thermal Controls ------------------

    /// Enable/disable NTC window (TS) monitoring.
    pub async fn set_ts_monitoring(&mut self, enable: bool) -> Result<I2C::Error> {
        let mut v = self.read1(REG_TS_BEHAVIOR_CTRL).await?;
        if enable { v |= EN_TS; } else { v &= !EN_TS; }
        self.write1(REG_TS_BEHAVIOR_CTRL, v).await
    }

    /// Enable/disable JEITA profile (cool/warm derates & CV offset).
    pub async fn set_jeita(&mut self, enable: bool) -> Result<I2C::Error> {
        let mut v = self.read1(REG_TS_BEHAVIOR_CTRL).await?;
        if enable { v |= EN_JEITA; } else { v &= !EN_JEITA; }
        self.write1(REG_TS_BEHAVIOR_CTRL, v).await
    }

    /// Disable all temperature-based charging behavior (TS off + JEITA off).
    pub async fn disable_all_temp_checks(&mut self) -> Result<I2C::Error> {
        // Order doesn’t matter, but keep both for clarity.
        self.set_jeita(false).await?;
        self.set_ts_monitoring(false).await
    }

    /// Read raw TS/JEITA behavior register.
    pub async fn get_ts_behavior(&mut self) -> CoreResult<u8, Error<I2C::Error>> {
        self.read1(REG_TS_BEHAVIOR_CTRL).await
    }

    // ------------------ Pre-charge and Termination ------------------

    pub async fn set_precharge_current_limit(&mut self, ma: u16) -> Result<I2C::Error> {
        let ma = ma.clamp(0, 1024);
        self.write2(REG_PRECHARGE_CURRENT_LIMIT, ma).await
    }

    pub async fn get_precharge_current_limit(&mut self) -> CoreResult<u16, Error<I2C::Error>> {
        self.read2(REG_PRECHARGE_CURRENT_LIMIT).await
    }

    pub async fn set_termination_current_limit(&mut self, ma: u16) -> Result<I2C::Error> {
        let ma = ma.clamp(0, 1024);
        self.write2(REG_TERMINATION_CURRENT_LIMIT, ma).await
    }

    pub async fn get_termination_current_limit(&mut self) -> CoreResult<u16, Error<I2C::Error>> {
        self.read2(REG_TERMINATION_CURRENT_LIMIT).await
    }

    // ------------------ Public Raw Reads for Debug / Snapshot ------------------

    /// Get raw TIMER_CONTROL register (read-only from app’s perspective).
    pub async fn get_timer_control(&mut self) -> CoreResult<u8, Error<I2C::Error>> {
        self.read1(REG_TIMER_CONTROL).await
    }

    /// Get raw CHARGER_CONTROL register (read-only from app’s perspective).
    pub async fn get_charger_control(&mut self) -> CoreResult<u8, Error<I2C::Error>> {
        self.read1(REG_CHARGER_CONTROL).await
    }

    // ------------------ Timer Controls ------------------

    pub async fn set_topoff_timer(&mut self, timer: TopOffTimer) -> Result<I2C::Error> {
        let mut val = self.read1(REG_TIMER_CONTROL).await?;
        val &= !TOPOFF_TMR_MASK;
        val |= (timer as u8) << 6;
        self.write1(REG_TIMER_CONTROL, val).await
    }

    pub async fn set_watchdog_timer(&mut self, timer: WdtTimer) -> Result<I2C::Error> {
        let mut val = self.read1(REG_TIMER_CONTROL).await?;
        val &= !WDT_MASK;
        val |= (timer as u8) << 4;
        self.write1(REG_TIMER_CONTROL, val).await
    }

    pub async fn set_charge_timer(&mut self, timer: ChgTmr) -> Result<I2C::Error> {
        let mut val = self.read1(REG_TIMER_CONTROL).await?;
        val &= !CHG_TMR_MASK;
        val |= (timer as u8) << 1;
        self.write1(REG_TIMER_CONTROL, val).await
    }

    pub async fn enable_2x_timer(&mut self, enable: bool) -> Result<I2C::Error> {
        let mut val = self.read1(REG_TIMER_CONTROL).await?;
        if enable { val |= EN_TMR2X; } else { val &= !EN_TMR2X; }
        self.write1(REG_TIMER_CONTROL, val).await
    }

    // ------------------ Misc Controls ------------------

    pub async fn enable_charger(&mut self, enable: bool) -> Result<I2C::Error> {
        let mut val = self.read1(REG_CHARGER_CONTROL).await?;
        if enable { val |= EN_CHG; } else { val &= !EN_CHG; }
        self.write1(REG_CHARGER_CONTROL, val).await
    }

    pub async fn enable_hiz(&mut self, enable: bool) -> Result<I2C::Error> {
        let mut val = self.read1(REG_CHARGER_CONTROL).await?;
        if enable { val |= EN_HIZ; } else { val &= !EN_HIZ; }
        self.write1(REG_CHARGER_CONTROL, val).await
    }

    pub async fn reset_watchdog_timer(&mut self) -> Result<I2C::Error> {
        let mut val = self.read1(REG_CHARGER_CONTROL).await?;
        val |= WD_RST;
        self.write1(REG_CHARGER_CONTROL, val).await
    }

    // --- ADC config
    pub async fn adc_enable(&mut self, enable: bool, continuous: bool, sample_bits_efr: u8, avg: bool)
        -> Result<I2C::Error>
    {
        use crate::bq25756::regs::*;
        let mut v = self.read1(REG_ADC_CONTROL).await?;
        if enable { v |= ADC_EN; } else { v &= !ADC_EN; }
        if continuous { v &= !ADC_RATE; } else { v |= ADC_RATE; }
        v = (v & !ADC_SAMPLE_MASK) | ((sample_bits_efr & 0x3) << 4);
        if avg { v |= ADC_AVG; } else { v &= !ADC_AVG; }
        self.write1(REG_ADC_CONTROL, v).await
    }

    pub async fn adc_set_channels(&mut self, vac: bool, vbat: bool, iac: bool, ibat: bool, ts: bool)
        -> Result<I2C::Error>
    {
        use crate::bq25756::regs::*;
        let mut v = self.read1(REG_ADC_CHANNEL_CTRL).await?;
        // Bits are *_DIS (1 = disable). Clear to enable.
        v = (v & !(IAC_ADC_DIS | IBAT_ADC_DIS | VAC_ADC_DIS | VBAT_ADC_DIS | TS_ADC_DIS))
            | (if !iac  { IAC_ADC_DIS } else { 0 })
            | (if !ibat { IBAT_ADC_DIS } else { 0 })
            | (if !vac  { VAC_ADC_DIS } else { 0 })
            | (if !vbat { VBAT_ADC_DIS } else { 0 })
            | (if !ts   { TS_ADC_DIS } else { 0 });
        // Keep VFB ADC disabled while charging (datasheet recommendation).
        v |= VFB_ADC_DIS;
        self.write1(REG_ADC_CHANNEL_CTRL, v).await
    }

    // --- ADC reads (raw; VBAT reports 2 mV/LSB so we scale that one)
    pub async fn read_vbat_mv(&mut self) -> CoreResult<u16, Error<I2C::Error>> {
        use crate::bq25756::regs::*;
        Ok(self.read2(REG_VBAT_ADC).await? * 2) // 2 mV/LSB
    }
    pub async fn read_vac_raw(&mut self)  -> CoreResult<u16, Error<I2C::Error>> {
        use crate::bq25756::regs::*; self.read2(REG_VAC_ADC).await
    }
    pub async fn read_vac_mv(&mut self)  -> CoreResult<u16, Error<I2C::Error>> {
        Ok(self.read2(REG_VAC_ADC).await? * 2) // 2 mV/LSB
    }
    pub async fn read_iac_raw(&mut self)  -> CoreResult<u16, Error<I2C::Error>> {
        use crate::bq25756::regs::*; self.read2(REG_IAC_ADC).await
    }
    pub async fn read_iac_ma(&mut self, rac_milliohm: u16)
        -> CoreResult<i32, Error<I2C::Error>>
    {
        let raw = u16_to_i16(self.read2(REG_IAC_ADC).await?) as i32;
        let lsb_ma_num = 4i32; // numerator of (4 / RmΩ)
        Ok(raw * lsb_ma_num / (rac_milliohm as i32))
    }
    pub async fn read_ibat_raw(&mut self) -> CoreResult<u16, Error<I2C::Error>> {
        use crate::bq25756::regs::*; self.read2(REG_IBAT_ADC).await
    }
    pub async fn read_ibat_ma(&mut self, rbat_milliohm: u16)
        -> CoreResult<i32, Error<I2C::Error>>
    {
        let raw = u16_to_i16(self.read2(REG_IBAT_ADC).await?) as i32;
        let lsb_ma_num = 10i32; // numerator of (10 / RmΩ)
        Ok(raw * lsb_ma_num / (rbat_milliohm as i32))
    }

    // Set TS thresholds (T1/T2/T3/T5)
    pub async fn ts_set_thresholds(&mut self, t1: TsT1, t2: TsT2, t3: TsT3, t5: TsT5)
        -> Result<I2C::Error>
    {
        let mut v = self.read1(REG_TS_THRESHOLD_CTRL).await?;
        v = (v & !(TS_T1_MASK | TS_T2_MASK | TS_T3_MASK | TS_T5_MASK))
            | ((t1 as u8) << 0)
            | ((t2 as u8) << 2)
            | ((t3 as u8) << 4)
            | ((t5 as u8) << 6);
        self.write1(REG_TS_THRESHOLD_CTRL, v).await
    }

    // Configure JEITA behavior (cool/warm regions)
    pub async fn ts_config_jeita(&mut self, enable: bool, isetc: JeitaIsetc, iseth_100pct: bool, vset: JeitaVset)
        -> Result<I2C::Error>
    {
        let mut v = self.read1(REG_TS_BEHAVIOR_CTRL).await?;
        v &= !(JEITA_VSET_MASK | JEITA_ISETC_MASK | EN_JEITA | JEITA_ISETH);
        v |= ((vset as u8) << 5) | ((isetc as u8) << 2);
        if iseth_100pct { v |= JEITA_ISETH; }
        if enable { v |= EN_JEITA; }
        self.write1(REG_TS_BEHAVIOR_CTRL, v).await
    }

    // Enable/disable TS monitoring entirely
    pub async fn ts_enable(&mut self, enable: bool) -> Result<I2C::Error> {
        let mut v = self.read1(REG_TS_BEHAVIOR_CTRL).await?;
        if enable { v |= EN_TS; } else { v &= !EN_TS; }
        self.write1(REG_TS_BEHAVIOR_CTRL, v).await
    }
}

// Alias used by the task
type Bq25756Dev<'a> = Bq25756<super::super::I2cDev<'a>>;

use embassy_sync::pubsub::Subscriber;

#[embassy_executor::task]
pub async fn bq25756_task(
    bq: &'static Mutex<CriticalSectionRawMutex, Bq25756Dev<'static>>,
) {
    let publisher = crate::shared_state::BQ25756_CHANNEL.publisher().unwrap();
    let mut command_subscriber: Subscriber<
        'static,
        CriticalSectionRawMutex,
        Bq25756Command,
        2,
        1,
        3,
    > = crate::shared_state::BQ25756_COMMAND_CHANNEL
        .subscriber()
        .unwrap();
    // -------- One-time init snapshot (read-only) --------
    let mut skip_init = false;
    {
        let mut dev = bq.lock().await;

        match with_i2c_timeout(dev.whoami()).await {
            Ok(0x02) => defmt::info!("BQ25756: detected (PART=0x02) at 0x{:02X}", dev.addr),
            Ok(other) => {
                defmt::warn!("BQ25756: unexpected PART=0x{:02X} @ 0x{:02X}. Skipping init.", other, dev.addr);
                // Skip init block entirely so we can still run the ticker loop
                skip_init = true;
                // jump ahead by enclosing the rest of the init in a `if false { ... }` or return from the block
            }
            Err(e) => {
                defmt::error!("BQ25756: unreachable at 0x{:02X}: {:?}", dev.addr, defmt::Debug2Format(&e));
                skip_init = true;
                // Same: skip init snapshot
            }
        }
        if !skip_init {
            let mut dev = bq.lock().await;
            if let Ok(v) = dev.get_charge_voltage_limit().await {
                defmt::info!("BQ25756:init VREG={} mV", v);
            }
            if let Ok(i) = dev.get_charge_current_limit().await {
                defmt::info!("BQ25756:init ICHG={} mA", i);
            }
            if let Ok(iin) = dev.get_input_current_dpm_limit().await {
                defmt::info!("BQ25756:init IIN_DPM={} mA", iin);
            }
            if let Ok(vin) = dev.get_input_voltage_dpm_limit().await {
                defmt::info!("BQ25756:init VIN_DPM={} mV", vin);
            }
            if let Ok(ipre) = dev.get_precharge_current_limit().await {
                defmt::info!("BQ25756:init IPRECHG={} mA", ipre);
            }
            if let Ok(iterm) = dev.get_termination_current_limit().await {
                defmt::info!("BQ25756:init ITERM={} mA", iterm);
            }

            if let Ok(timer_ctrl) = dev.get_timer_control().await {
                let topoff = (timer_ctrl & TOPOFF_TMR_MASK) >> 6;
                let wdt    = (timer_ctrl & WDT_MASK) >> 4;
                let chgtmr = (timer_ctrl & CHG_TMR_MASK) >> 1;
                let tmr2x  = (timer_ctrl & EN_TMR2X) != 0;
                defmt::info!(
                    "BQ25756:init TIMER_CTRL=0x{:02X} (TOPOFF={} WDT={} CHG_TMR={} 2x={})",
                    timer_ctrl, topoff, wdt, chgtmr, tmr2x
                );
            }
            if let Ok(chg_ctrl) = dev.get_charger_control().await {
                let en_chg = (chg_ctrl & EN_CHG) != 0;
                let hiz    = (chg_ctrl & EN_HIZ) != 0;
                defmt::info!(
                    "BQ25756:init CHARGER_CTRL=0x{:02X} (EN_CHG={} HIZ={})",
                    chg_ctrl, en_chg, hiz
                );
            }

            // === Conditional init: either full charge config or telemetry-only ===

            // Default: full charge configuration
            #[cfg(not(feature = "bq25756-mcu-control"))]
            {
                if let Err(e) = dev.bringup_3s_1a_defaults().await {
                    defmt::warn!("BQ25756: bring-up failed: {:?}", defmt::Debug2Format(&e));
                } else {
                    defmt::info!("BQ25756: bring-up sequence completed, verifying settings...");
                    let vreg = dev.get_charge_voltage_limit().await.unwrap_or(0);
                    let ichg = dev.get_charge_current_limit().await.unwrap_or(0);
                    let iin_dpm = dev.get_input_current_dpm_limit().await.unwrap_or(0);
                    let vin_dpm = dev.get_input_voltage_dpm_limit().await.unwrap_or(0);
                    defmt::info!(
                        "BQ25756: VREG={}mV, ICHG={}mA, IIN_DPM={}mA, VIN_DPM={}mV",
                        vreg,
                        ichg,
                        iin_dpm,
                        vin_dpm
                    );
                }
                dev.adc_enable(true, true, 0b10, false).await.ok();          // continuous, ~13-bit eff
                dev.adc_set_channels(true, true, true, true, false).await.ok(); // VAC,VBAT,IAC,IBAT on; TS off
            }

            dev.set_watchdog_timer(crate::bq25756::types::WdtTimer::S40).await.ok();

            // --- Policy: if no external thermistors, disable TS + JEITA to keep charging functional.
            #[cfg(feature = "bq25756-no-jeita")]
            let want_ts = false;
            #[cfg(not(feature = "bq25756-no-jeita"))]
            let want_ts = crate::config::EXTERNAL_THERMISTORS_PRESENT;

            let want_jeita = want_ts; // JEITA only meaningful if TS is monitored

            // Apply once at init
            dev.set_ts_monitoring(want_ts).await.ok();
            dev.set_jeita(want_jeita).await.ok();

            // MCU Control feature: Telemetry-only init
            #[cfg(feature = "bq25756-mcu-control")]
            {
                defmt::info!("BQ25756: MCU control mode: enabling ADC for telemetry.");
                // Telemetry only: turn on ADC in continuous mode and enable channels
                dev.adc_enable(true, true, 0b10, false).await.ok();      // 13-bit effective, single value
                dev.adc_set_channels(true, true, true, true, false).await.ok(); // VAC, VBAT, IAC, IBAT on; TS off
            }
            dev.int_unmask_all().await.ok();
        }
    }

    // -------- Periodic poll + command handling + publish --------
    let mut ticker = Ticker::every(Duration::from_millis(config::BQ25756_LOG_PERIOD_MS));
    loop {
        // Handle any pending commands first
        if let Some(command) = command_subscriber.try_next_message_pure() {
            let mut dev = bq.lock().await;
            match command {
                Bq25756Command::SetChargeVoltageLimit(mv) => {
                    dev.set_charge_voltage_limit(mv).await.ok();
                }
                Bq25756Command::SetChargeCurrentLimit(ma) => {
                    dev.set_charge_current_limit(ma).await.ok();
                }
                Bq25756Command::SetInputCurrentDpmLimit(ma) => {
                    dev.set_input_current_dpm_limit(ma).await.ok();
                }
                Bq25756Command::SetInputVoltageDpmLimit(mv) => {
                    dev.set_input_voltage_dpm_limit(mv).await.ok();
                }
                Bq25756Command::SetPrechargeCurrentLimit(ma) => {
                    dev.set_precharge_current_limit(ma).await.ok();
                }
                Bq25756Command::SetTerminationCurrentLimit(ma) => {
                    dev.set_termination_current_limit(ma).await.ok();
                }
                Bq25756Command::EnableCharger(enable) => {
                    dev.enable_charger(enable).await.ok();
                }
                Bq25756Command::EnableHiz(enable) => {
                    dev.enable_hiz(enable).await.ok();
                }
                Bq25756Command::Init3S1A => {
                    dev.bringup_3s_1a_defaults().await.ok();
                }
            }
        }

        // -------- Reconcile IC state with desired driver state (once per second) --------
        {
            let mut dev = bq.lock().await;

            // Desired policy (same logic as at init)
            #[cfg(feature = "bq25756-no-jeita")]
            let want_ts = false;
            #[cfg(not(feature = "bq25756-no-jeita"))]
            let want_ts = crate::config::EXTERNAL_THERMISTORS_PRESENT;
            let want_jeita = want_ts;

            // 1) Watchdog setting should be 40s
            if let Ok(timer_ctrl) = dev.get_timer_control().await {
                let wdt = (timer_ctrl & WDT_MASK) >> 4;
                // WdtTimer::S40 = 0b01
                if wdt != (crate::bq25756::types::WdtTimer::S40 as u8) {
                    defmt::warn!("BQ25756: WDT != 40s (was code {}), forcing 40s", wdt);
                    dev.set_watchdog_timer(crate::bq25756::types::WdtTimer::S40).await.ok();
                }
            }

            // 2) TS/JEITA policy
            if let Ok(tsb) = dev.get_ts_behavior().await {
                let ic_ts    = (tsb & EN_TS)    != 0;
                let ic_jeita = (tsb & EN_JEITA) != 0;
                if ic_ts != want_ts {
                    defmt::warn!("BQ25756: TS_EN mismatch (ic={}, want={}), fixing", ic_ts, want_ts);
                    dev.set_ts_monitoring(want_ts).await.ok();
                }
                if ic_jeita != want_jeita {
                    defmt::warn!("BQ25756: JEITA_EN mismatch (ic={}, want={}), fixing", ic_jeita, want_jeita);
                    dev.set_jeita(want_jeita).await.ok();
                }
            }

            // 3) Pet the watchdog each second while we’re alive
            dev.reset_watchdog_timer().await.ok();
        }

        // Snapshot readings and publish
        let readings = {
            let mut dev = bq.lock().await;
            Bq25756Readings {
                // Limits
                charge_voltage_limit_mv: dev.get_charge_voltage_limit().await.unwrap_or(0),
                charge_current_limit_ma: dev.get_charge_current_limit().await.unwrap_or(0),
                input_current_dpm_limit_ma: dev.get_input_current_dpm_limit().await.unwrap_or(0),
                input_voltage_dpm_limit_mv: dev.get_input_voltage_dpm_limit().await.unwrap_or(0),
                // Telemetry
                vbat_mv: dev.read_vbat_mv().await.unwrap_or(0),
                vac_raw: dev.read_vac_raw().await.unwrap_or(0),
                iac_raw: dev.read_iac_raw().await.unwrap_or(0),
                ibat_raw: dev.read_ibat_raw().await.unwrap_or(0),

                iac_ma: dev.read_iac_ma(5u16).await.unwrap_or(0),
                ibat_ma: dev.read_ibat_ma(5u16).await.unwrap_or(0),
                vac_mv: dev.read_vac_mv().await.unwrap_or(0u16),
            }

        };


        defmt::info!(
            "BQ25756: VAC={}mV (raw={})  VBAT={}mV  IAC={}mA  IBAT={}mA  [VIN_DPM={}mV IIN_DPM={}mA]",
            readings.vac_mv,
            readings.vac_raw,
            readings.vbat_mv,
            readings.iac_ma,
            readings.ibat_ma,
            readings.input_voltage_dpm_limit_mv,
            readings.input_current_dpm_limit_ma
        );

        if let Err(_e) = publisher.try_publish(readings) {
            defmt::debug!("BQ25756 drop (no subscriber)");
        }
        else {
            defmt::debug!("BQ25756 data published.");
        }

        ticker.next().await;
    }
}

#[embassy_executor::task]
pub async fn bq25756_int_task(
    mut int_pin: ExtiInput<'static>,
    bq: &'static Mutex<CriticalSectionRawMutex, Bq25756Dev<'static>>,
) {
    loop {
        int_pin.wait_for_falling_edge().await; // INT is active-low, open-drain
        let mut dev = bq.lock().await;
        if let Ok(((f1,f2,ff),(s1,s2,s3,fs))) = dev.read_int_cause_and_status().await {
            defmt::info!("BQ25756: INT flags f1=0x{:02X} f2=0x{:02X} ff=0x{:02X}  status s1=0x{:02X} s2=0x{:02X} s3=0x{:02X} fault=0x{:02X}",
                f1,f2,ff,s1,s2,s3,fs);
        }
    }
}

#[embassy_executor::task]
pub async fn bq25756_pg_task(
    mut pg_pin: ExtiInput<'static>,
    bq: &'static Mutex<CriticalSectionRawMutex, Bq25756Dev<'static>>,
) {
    loop {
        pg_pin.wait_for_any_edge().await;
        let mut dev = bq.lock().await;
        let s2 = dev.read1(REG_CHARGER_STATUS_2).await.unwrap_or(0);
        let fs = dev.read1(REG_FAULT_STATUS).await.unwrap_or(0);
        defmt::info!("BQ25756: PG edge -> STATUS2=0x{:02X} FAULT_STATUS=0x{:02X}", s2, fs);
    }
}

#[embassy_executor::task]
pub async fn bq25756_stat1_task(
    mut stat1_pin: ExtiInput<'static>,
    bq: &'static Mutex<CriticalSectionRawMutex, Bq25756Dev<'static>>,
) {
    loop {
        stat1_pin.wait_for_any_edge().await;
        let mut dev = bq.lock().await;
        let s1 = dev.read1(REG_CHARGER_STATUS_1).await.unwrap_or(0);
        let s2 = dev.read1(REG_CHARGER_STATUS_2).await.unwrap_or(0);
        let s3 = dev.read1(REG_CHARGER_STATUS_3).await.unwrap_or(0);
        defmt::info!("BQ25756: STAT1 edge -> S1=0x{:02X} S2=0x{:02X} S3=0x{:02X}", s1,s2,s3);
    }
}

#[embassy_executor::task]
pub async fn bq25756_stat2_task(
    mut stat2_pin: ExtiInput<'static>,
    bq: &'static Mutex<CriticalSectionRawMutex, Bq25756Dev<'static>>,
) {
    loop {
        stat2_pin.wait_for_any_edge().await;
        let mut dev = bq.lock().await;
        let s1 = dev.read1(REG_CHARGER_STATUS_1).await.unwrap_or(0);
        let s2 = dev.read1(REG_CHARGER_STATUS_2).await.unwrap_or(0);
        let s3 = dev.read1(REG_CHARGER_STATUS_3).await.unwrap_or(0);
        defmt::info!("BQ25756: STAT2 edge -> S1=0x{:02X} S2=0x{:02X} S3=0x{:02X}", s1,s2,s3);
    }
}
