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
    /// Assumes CE pin is held LOW by the MCU.
    pub async fn init_from_config(&mut self) -> Result<I2C::Error> {
        self.set_watchdog_timer(crate::bq25756::types::WdtTimer::Disable).await?;

        self.enable_hiz(false).await?;
        self.set_input_voltage_dpm_limit(crate::config::BQ25756_INPUT_VOLTAGE_MAX_MV).await?;
        self.set_input_current_dpm_limit(crate::config::BQ25756_INPUT_CURRENT_MAX_MA).await?;
        self.set_charge_voltage_limit(crate::config::BQ25756_CHG_VOLTAGE_MAX_MV).await?;
        self.set_charge_current_limit(crate::config::BQ25756_CHG_CURRENT_MAX_MA).await?;

        self.ts_enable(crate::config::EXTERNAL_THERMISTORS_PRESENT).await.ok();
        self.set_jeita(crate::config::EXTERNAL_THERMISTORS_PRESENT).await.ok();

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



    // ==== Helper math: CV (voltage) ====

    /// Vfb = Vbat * (Rbot / (Rtop + Rbot))
    #[inline]
    fn vbat_to_vfb_mv(vbat_mv: u32, r_top_ohm: u32, r_bot_ohm: u32) -> u16 {
        let num = (vbat_mv as u64) * (r_bot_ohm as u64);
        let den = (r_top_ohm as u64) + (r_bot_ohm as u64);
        (num / den) as u16
    }

    /// Inverse of vbat_to_vfb_mv.
    #[inline]
    fn vfb_to_vbat_mv(vfb_mv: u16, r_top_ohm: u32, r_bot_ohm: u32) -> u16 {
        let vfb = vfb_mv as u64;
        let sum = (r_top_ohm as u64) + (r_bot_ohm as u64);
        ((vfb * sum) / (r_bot_ohm as u64)) as u16
    }

    /// Encode a VFB set-point (mV) into VFB_REG[4:0] (1504..1566mV, 2mV/step).
    #[inline]
    fn vfb_mv_to_code(vfb_mv: u16) -> u16 {
        let v = vfb_mv.clamp(1504, 1566);
        ((v - 1504) / 2) as u16 & 0x1F
    }

    /// Decode VFB_REG[4:0] into VFB set-point (mV).
    #[inline]
    fn vfb_code_to_mv(code: u16) -> u16 {
        1504 + ((code & 0x1F) * 2)
    }

    // ==== Helper math: CC (charge current) ====

    /// ICHG_REG is 0.4..20.0 A with 50 mA/LSB (register-based path).
    #[inline]
    fn ichg_ma_to_reg(ma: u16) -> u16 {
        let clamped = ma.clamp(400, 20_000);
        (clamped / 50) as u16
    }
    #[inline]
    fn ichg_reg_to_ma(reg: u16) -> u16 {
        (reg as u16) * 50
    }

    // ------------------ Charging Parameters ------------------

    /// Set the VFB regulation target (mV at the FB pin). Valid: 1504..1566 mV (2 mV/step).
    pub async fn set_vfb_reg_mv(&mut self, vfb_mv: u16) -> Result<I2C::Error> {
        let code = Self::vfb_mv_to_code(vfb_mv);
        // read-modify-write 16-bit reg (we only touch bits [4:0])
        let mut reg = self.read2(REG_CHARGE_VOLTAGE_LIMIT).await?;
        reg = (reg & !0x001F) | code;
        self.write2(REG_CHARGE_VOLTAGE_LIMIT, reg).await
    }

    /// Set CV by desired battery voltage (mV), using R_TOP/R_BOT from config.
    pub async fn set_charge_voltage_limit(&mut self, vbat_target_mv: u16) -> Result<I2C::Error> {
        let rtop = crate::config::BQ25756_R_TOP_OHM;
        let rbot = crate::config::BQ25756_R_BOT_OHM;
        let vfb_mv = Self::vbat_to_vfb_mv(vbat_target_mv as u32, rtop, rbot);
        self.set_vfb_reg_mv(vfb_mv).await
    }

    /// Read CV limit as battery voltage (mV). (Decodes register -> VFB -> VBAT)
    pub async fn get_charge_voltage_limit(&mut self) -> CoreResult<u16, Error<I2C::Error>> {
        let reg = self.read2(REG_CHARGE_VOLTAGE_LIMIT).await?;
        let vfb_code = reg & 0x001F;
        let vfb_mv = Self::vfb_code_to_mv(vfb_code);
        let rtop = crate::config::BQ25756_R_TOP_OHM;
        let rbot = crate::config::BQ25756_R_BOT_OHM;
        Ok(Self::vfb_to_vbat_mv(vfb_mv, rtop, rbot))
    }

    /// Set charge current limit (mA). Encodes to 50 mA/LSB register.
    pub async fn set_charge_current_limit(&mut self, ma: u16) -> Result<I2C::Error> {
        let code = Self::ichg_ma_to_reg(ma);
        self.write2(REG_CHARGE_CURRENT_LIMIT, code).await
    }

    /// Read charge current limit (mA). (Decodes 50 mA/LSB register.)
    pub async fn get_charge_current_limit(&mut self) -> CoreResult<u16, Error<I2C::Error>> {
        let reg = self.read2(REG_CHARGE_CURRENT_LIMIT).await?;
        Ok(Self::ichg_reg_to_ma(reg))
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

    pub async fn get_charge_stat(&mut self) -> CoreResult<ChargeStat, Error<I2C::Error>> {
        let s1 = self.read1(REG_CHARGER_STATUS_1).await?;
        Ok(ChargeStat::from_code(s1))
    }

    // ------------------ JEITA and Thermal Controls ------------------

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
        self.ts_enable(false).await
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
    defmt::info!("BQ25756 task starting");

    // ---- Bring-up state ----
    let mut online = false;
    let mut backoff_ms: u64 = 250;


    // -------- Periodic poll + command handling + publish --------
    let mut ticker = Ticker::every(Duration::from_millis(config::BQ25756_LOG_PERIOD_MS));

    loop {
        defmt::debug!("BQ25756 loop entry");
        if !online {
            defmt::debug!("BQ25756: online false, probing...");
            // Try a quick probe
            {
                defmt::debug!("BQ25756: locking...");
                let mut dev = bq.lock().await;
                defmt::debug!("BQ25756: locked, probing...");
                match with_i2c_timeout(dev.whoami()).await {
                    Ok(0x02) => {
                        defmt::info!("BQ25756: online at 0x{:02X}", dev.addr);

                        // Configure once when we first come online
                        if let Err(e) = dev.init_from_config().await {
                            defmt::error!("BQ25756: init_from_config failed: {:?}", defmt::Debug2Format(&e));
                            // stay offline; try again after backoff
                        } else {
                            // Optional: a quick verify snapshot (non-fatal if a read fails)
                            if let Ok(v) = dev.get_charge_voltage_limit().await {
                                defmt::info!("BQ25756:init VREG={} mV", v);
                            }
                            if let Ok(i) = dev.get_charge_current_limit().await {
                                defmt::info!("BQ25756:init ICHG={} mA", i);
                            }
                            online = true;
                            backoff_ms = 250; // reset backoff
                        }
                    }
                    Ok(other) => {
                        defmt::warn!("BQ25756: unexpected PART=0x{:02X}; will retry", other);
                    }
                    Err(e) => {
                        defmt::debug!("BQ25756: probe failed: {:?}", defmt::Debug2Format(&e));
                    }
                }
            }

            if !online {
                embassy_time::Timer::after(Duration::from_millis(backoff_ms)).await;
                backoff_ms = (backoff_ms * 2).min(4000);
                continue; // retry probe; do not read/publish anything while offline
            }
        }
        // Handle any pending commands first
        if let Some(command) = command_subscriber.try_next_message_pure() {
            let mut dev = bq.lock().await;
            let res = match command {
                Bq25756Command::SetChargeVoltageLimit(mv)    => dev.set_charge_voltage_limit(mv).await,
                Bq25756Command::SetChargeCurrentLimit(ma)    => dev.set_charge_current_limit(ma).await,
                Bq25756Command::SetInputCurrentDpmLimit(ma)  => dev.set_input_current_dpm_limit(ma).await,
                Bq25756Command::SetInputVoltageDpmLimit(mv)  => dev.set_input_voltage_dpm_limit(mv).await,
                Bq25756Command::SetPrechargeCurrentLimit(ma) => dev.set_precharge_current_limit(ma).await,
                Bq25756Command::SetTerminationCurrentLimit(ma)=> dev.set_termination_current_limit(ma).await,
                Bq25756Command::EnableCharger(enable)        => dev.enable_charger(enable).await,
                Bq25756Command::EnableHiz(enable)            => dev.enable_hiz(enable).await,
                Bq25756Command::Init                         => dev.init_from_config().await,
            };
            if let Err(e) = res {
                defmt::warn!("BQ25756: command error -> going offline: {:?}", defmt::Debug2Format(&e));
                online = false;
                continue; // fall back to probe loop
            }
        }

        // Snapshot readings and publish
        let readings = {
            let mut dev = bq.lock().await;

            // If any of these fail, we treat the whole snapshot as failed and drop back offline.
            let vreg   = match dev.get_charge_voltage_limit().await { Ok(v)=>v, Err(e)=>{ defmt::warn!("BQ25756: get VREG failed: {:?}", defmt::Debug2Format(&e)); online=false; continue; } };
            let ichg   = match dev.get_charge_current_limit().await { Ok(v)=>v, Err(e)=>{ defmt::warn!("BQ25756: get ICHG failed: {:?}", defmt::Debug2Format(&e)); online=false; continue; } };
            let iin    = match dev.get_input_current_dpm_limit().await { Ok(v)=>v, Err(e)=>{ defmt::warn!("BQ25756: get IIN_DPM failed: {:?}", defmt::Debug2Format(&e)); online=false; continue; } };
            let vin    = match dev.get_input_voltage_dpm_limit().await { Ok(v)=>v, Err(e)=>{ defmt::warn!("BQ25756: get VIN_DPM failed: {:?}", defmt::Debug2Format(&e)); online=false; continue; } };
            let vbat   = match dev.read_vbat_mv().await { Ok(v)=>v, Err(e)=>{ defmt::warn!("BQ25756: read VBAT failed: {:?}", defmt::Debug2Format(&e)); online=false; continue; } };
            let vacraw = match dev.read_vac_raw().await { Ok(v)=>v, Err(e)=>{ defmt::warn!("BQ25756: read VAC raw failed: {:?}", defmt::Debug2Format(&e)); online=false; continue; } };
            let iacraw = match dev.read_iac_raw().await { Ok(v)=>v, Err(e)=>{ defmt::warn!("BQ25756: read IAC raw failed: {:?}", defmt::Debug2Format(&e)); online=false; continue; } };
            let ibatraw= match dev.read_ibat_raw().await { Ok(v)=>v, Err(e)=>{ defmt::warn!("BQ25756: read IBAT raw failed: {:?}", defmt::Debug2Format(&e)); online=false; continue; } };
            let iacma  = match dev.read_iac_ma(crate::config::BQ25756_R_INPUT_SENSE_MOHM).await { Ok(v)=>v, Err(e)=>{ defmt::warn!("BQ25756: calc IAC mA failed: {:?}", defmt::Debug2Format(&e)); online=false; continue; } };
            let ibatma = match dev.read_ibat_ma(crate::config::BQ25756_R_BATTERY_SENSE_MOHM).await { Ok(v)=>v, Err(e)=>{ defmt::warn!("BQ25756: calc IBAT mA failed: {:?}", defmt::Debug2Format(&e)); online=false; continue; } };
            let vacmv  = match dev.read_vac_mv().await { Ok(v)=>v, Err(e)=>{ defmt::warn!("BQ25756: read VAC mV failed: {:?}", defmt::Debug2Format(&e)); online=false; continue; } };

            Bq25756Readings {
                charge_voltage_limit_mv: vreg,
                charge_current_limit_ma: ichg,
                input_current_dpm_limit_ma: iin,
                input_voltage_dpm_limit_mv: vin,
                vbat_mv: vbat,
                vac_raw: vacraw,
                iac_raw: iacraw,
                ibat_raw: ibatraw,
                iac_ma: iacma,
                ibat_ma: ibatma,
                vac_mv: vacmv,
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
                f1,f2,ff,s1,s2,s3,fs
            );
            if let Ok(cs) = dev.get_charge_stat().await {
                defmt::info!("BQ25756: CHARGE_STAT -> {:?}", cs);
            }
        }
        dev.adc_enable(true, true, 0b10, false).await.ok();
        dev.adc_set_channels(true, true, true, true, false).await.ok();
        defmt::info!("BQ25756: INT edge -> ADC (re)enabled in continuous mode.");
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
        dev.adc_enable(true, true, 0b10, false).await.ok();
        dev.adc_set_channels(true, true, true, true, false).await.ok();
        defmt::info!("BQ25756: PG edge -> ADC (re)enabled in continuous mode.");
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
