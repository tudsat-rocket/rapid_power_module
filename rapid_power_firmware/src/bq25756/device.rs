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

use crate::bq25756::regs::{VAC_DPM_MASK, VAC_DPM_SHIFT, IAC_DPM_MASK, IAC_DPM_SHIFT};
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

        self.set_ilim_hiz_pin_enable(crate::config::BQ25756_EN_ILIM_HIZ_PIN).await?;
        self.set_ichg_pin_enable(crate::config::BQ25756_EN_ICHG_PIN).await?;

        self.adc_enable(true, true, 0b10, false).await.ok();
        self.adc_set_channels(true, true, true, true, crate::config::EXTERNAL_THERMISTORS_PRESENT).await.ok();

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

    // ------------------ 16b field helpers (generic) ------------------
    #[inline]
    fn set_field_16(orig: u16, mask: u16, shift: u8, code: u16) -> u16 {
        (orig & !mask) | ((code << shift) & mask)
    }
    #[inline]
    fn get_field_16(reg: u16, mask: u16, shift: u8) -> u16 {
        (reg & mask) >> shift
    }

    // ------------------ Quantization helpers ------------------
    /// Floor-encode to an integer step size (e.g., 50 mA/LSB, 20 mV/LSB).
    #[inline]
    fn enc_floor_step(val: u32, step_units: u32, min_units: u32, max_units: u32) -> u16 {
        let val = val.clamp(min_units, max_units);
        (val / step_units) as u16  // integer division floors by design
    }
    /// Decode from an integer step size.
    #[inline]
    fn dec_step(code: u16, step_units: u32) -> u16 {
        (code as u32 * step_units) as u16
    }

    /// Floor-encode to a rational LSB = NUM/DEN (units per LSB) using integer math.
    /// For IAC_DPM: LSB(mA) = 250 / RmΩ  -> code = floor(mA * RmΩ / 250)
    #[inline]
    fn enc_floor_ratio(val_units: u32, numerator_units: u32, denominator: u32,
                       min_units: u32, max_units: u32) -> u16 {
        let v = val_units.clamp(min_units, max_units);
        ((v as u64 * denominator as u64) / numerator_units as u64) as u16
    }
    #[inline]
    fn dec_ratio(code: u16, numerator_units: u32, denominator: u32) -> u16 {
        ((code as u32 * numerator_units) / denominator) as u16
    }

    // ----------- Shunt-aware DPM coding (per DS section 8.3.5.1.1) -----------
    // IAC_DPM code <-> mA. LSB(mA) = 250 / RmΩ.  (2 mΩ => 125 mA/LSB, 5 mΩ => 50 mA/LSB)
    #[inline]
    fn iac_dpm_code_from_ma(ma: u32, r_milliohm: u16) -> u16 {
        ((ma as u64 * r_milliohm as u64) / 250u64) as u16
    }
    #[inline]
    fn iac_dpm_ma_from_code(code: u16, r_milliohm: u16) -> u16 {
        ((code as u32 * 250u32) / r_milliohm as u32) as u16
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

    /// Encodable VBAT min/max given the resistor divider and VFB limits.
    /// VFB range: 1504..1566 mV (2 mV/LSB).
    #[inline]
    fn vbat_encodable_range_mv(r_top_ohm: u32, r_bot_ohm: u32) -> (u16, u16) {
        let vmin = Self::vfb_to_vbat_mv(1504, r_top_ohm, r_bot_ohm);
        let vmax = Self::vfb_to_vbat_mv(1566, r_top_ohm, r_bot_ohm);
        (vmin, vmax)
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

    /// Generic encode/decode helpers with an integer step (mA per LSB).
    #[inline] fn enc_step(ma: u16, step_ma: u16, min_ma: u16, max_ma: u16) -> u16 {
        let clamped = ma.clamp(min_ma, max_ma);
        clamped / step_ma
    }
    // ICHG_REG: 50 mA/LSB, 0.4–20 A
    #[inline] fn ichg_ma_to_reg(ma: u16) -> u16 { Self::enc_step(ma, 50, 400, 20_000) }
    #[inline] fn ichg_reg_to_ma(code: u16) -> u16 { Self::dec_step(code, 50) }

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
    /// Floors to the nearest representable value (never exceeds the request if within range).
    /// Warns if the request is outside the encodable range.
    pub async fn set_charge_voltage_limit(&mut self, vbat_req_mv: u16) -> Result<I2C::Error> {
        let rtop = crate::config::BQ25756_R_TOP_OHM;
        let rbot = crate::config::BQ25756_R_BOT_OHM;
        let (vmin, vmax) = Self::vbat_encodable_range_mv(rtop, rbot);

        // If below the minimum encodable VBAT, warn and clamp up to vmin (will exceed request).
        // If above the max encodable VBAT, warn and clamp down to vmax.
        let mut vbat_target_mv = vbat_req_mv;
        if vbat_req_mv < vmin {
            defmt::warn!(
                "BQ25756: requested VREG={} mV is below min encodable {}..{} mV; clamping to {} mV",
                vbat_req_mv, vmin, vmax, vmin
            );
            vbat_target_mv = vmin;
        } else if vbat_req_mv > vmax {
            defmt::warn!(
                "BQ25756: requested VREG={} mV is above max encodable {}..{} mV; clamping to {} mV",
                vbat_req_mv, vmin, vmax, vmax
            );
            vbat_target_mv = vmax;
        }

        // Convert to VFB (mV), then to code that floors in 2 mV steps.
        let vfb_mv      = Self::vbat_to_vfb_mv(vbat_target_mv as u32, rtop, rbot);
        let code        = Self::vfb_mv_to_code(vfb_mv);
        let vfb_prog_mv = Self::vfb_code_to_mv(code);
        let vbat_prog   = Self::vfb_to_vbat_mv(vfb_prog_mv, rtop, rbot);

        // Program (read-modify-write is done inside set_vfb_reg_mv)
        self.set_vfb_reg_mv(vfb_prog_mv).await?;

        // Optional: informative log of quantization.
        if vbat_prog != vbat_req_mv {
            defmt::info!(
                "BQ25756: VREG request {} mV -> programmed {} mV (VFB={} mV, code=0x{:02X})",
                vbat_req_mv, vbat_prog, vfb_prog_mv, code
            );
        }
        Ok(())
    }

    pub async fn set_ilim_hiz_pin_enable(&mut self, enable: bool) -> Result<I2C::Error> {
        let mut v = self.read1(REG_PIN_CONTROL).await?;
        if enable { v |= EN_ILIM_HIZ_PIN; } else { v &= !EN_ILIM_HIZ_PIN; }
        self.write1(REG_PIN_CONTROL, v).await
    }

    pub async fn set_ichg_pin_enable(&mut self, enable: bool) -> Result<I2C::Error> {
        let mut v = self.read1(REG_PIN_CONTROL).await?;
        if enable { v |= EN_ICHG_PIN; } else { v &= !EN_ICHG_PIN; }
        self.write1(REG_PIN_CONTROL, v).await
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

    /// Set charge current limit (mA).
    pub async fn set_charge_current_limit(&mut self, ma: u16) -> Result<I2C::Error> {
        let code = Self::ichg_ma_to_reg(ma);
        self.write2(REG_CHARGE_CURRENT_LIMIT, code).await
    }

    /// Read charge current limit (mA).
    pub async fn get_charge_current_limit(&mut self) -> CoreResult<u16, Error<I2C::Error>> {
        let reg = self.read2(REG_CHARGE_CURRENT_LIMIT).await?;
        Ok(Self::ichg_reg_to_ma(reg))
    }

    /// Set input current DPM limit (IIN_DPM).
    pub async fn set_input_current_dpm_limit(&mut self, ma: u16) -> Result<I2C::Error> {
        let r_milliohm = crate::config::BQ25756_R_INPUT_SENSE_MOHM as u32;
        let code = Self::enc_floor_ratio(ma as u32, 250, r_milliohm, 400, 20_000);
        let mut reg = self.read2(REG_INPUT_CURRENT_DPM_LIMIT).await?;
        reg = Self::set_field_16(reg, IAC_DPM_MASK, IAC_DPM_SHIFT, code);
        self.write2(REG_INPUT_CURRENT_DPM_LIMIT, reg).await
    }

    /// Read input current DPM limit (IIN_DPM).
    pub async fn get_input_current_dpm_limit(&mut self) -> CoreResult<u16, Error<I2C::Error>> {
        let r_milliohm = crate::config::BQ25756_R_INPUT_SENSE_MOHM as u32;
        let reg = self.read2(REG_INPUT_CURRENT_DPM_LIMIT).await?;
        let code = Self::get_field_16(reg, IAC_DPM_MASK, IAC_DPM_SHIFT);
        Ok(Self::dec_ratio(code, 250, r_milliohm)) 
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

    pub async fn dump_limits(&mut self) -> Result<I2C::Error> {
        use crate::bq25756::regs::*;

        // --- Control bits
        let chg_ctl = self.get_charger_control().await?;
        let hiz     = (chg_ctl & EN_HIZ) != 0;
        let en_chg  = (chg_ctl & EN_CHG) != 0;

        let pin_ctl = self.read1(REG_PIN_CONTROL).await?;
        let ilim_pin_en = (pin_ctl & EN_ILIM_HIZ_PIN) != 0;
        let ichg_pin_en = (pin_ctl & EN_ICHG_PIN) != 0;

        // --- Programmed limits (decoded to real units)
        let vreg = self.get_charge_voltage_limit().await?;
        let ichg = self.get_charge_current_limit().await?;
        let iin  = self.get_input_current_dpm_limit().await?;
        let vin  = self.get_input_voltage_dpm_limit().await?;

        // Quantization info for IIN_DPM
        let r_in_mohm = crate::config::BQ25756_R_INPUT_SENSE_MOHM as u32;
        let iin_lsb_ma = 250 / r_in_mohm.max(1); // LSB(mA) = 250 / RmΩ (e.g. 50 mA @ 5 mΩ)

        // --- Status: are DPM loops actually engaged?
        let s1 = self.read1(REG_CHARGER_STATUS_1).await.unwrap_or(0);
        let s2 = self.read1(REG_CHARGER_STATUS_2).await.unwrap_or(0);
        let s3 = self.read1(REG_CHARGER_STATUS_3).await.unwrap_or(0);

        let iindpm_active = (s1 & IINDPM_STAT) != 0; // from S1
        let vindpm_active = (s1 & VINDPM_STAT) != 0; // from S1
        let pg_ok         = (s2 & PG_STAT)     != 0; // from S2

        // --- Optional live readings (helps to reason about headroom)
        let vac_mv  = self.read_vac_mv().await.unwrap_or(0);
        let vbat_mv = self.read_vbat_mv().await.unwrap_or(0);
        let iac_ma  = self.read_iac_ma(crate::config::BQ25756_R_INPUT_SENSE_MOHM).await.unwrap_or(0);
        let ibat_ma = self.read_ibat_ma(crate::config::BQ25756_R_BATTERY_SENSE_MOHM).await.unwrap_or(0);

        defmt::info!("BQ25756 limits:");
        defmt::info!("  CHG_EN={}  HIZ={}  PIN_CTRL: ICHG_PIN_EN={} ILIM_HIZ_PIN_EN={}", en_chg, hiz, ichg_pin_en, ilim_pin_en);
        defmt::info!("  VREG={} mV  ICHG={} mA", vreg, ichg);
        defmt::info!("  VIN_DPM={} mV  IIN_DPM={} mA (LSB≈{} mA)", vin, iin, iin_lsb_ma);
        defmt::info!("  STATUS: S1=0x{:02X} S2=0x{:02X} S3=0x{:02X}  PG={}  IINDPM_ACTIVE={}  VINDPM_ACTIVE={}",
                    s1, s2, s3, pg_ok, iindpm_active, vindpm_active);
        defmt::info!("  Live: VAC={} mV  VBAT={} mV  IAC={} mA  IBAT={} mA", vac_mv, vbat_mv, iac_ma, ibat_ma);


        // Quick hint about who “wins” on input current
        if ichg_pin_en {
            defmt::info!("  Note: ICHG pin is ENABLED; effective fast-charge current = min(ICHG register, ICHG pin limit).");
        } else {
            defmt::info!("  Note: ICHG pin is DISABLED; fast-charge current controlled by ICHG register.");
        }
        if ilim_pin_en {
            defmt::info!("  Note: ILIM_HIZ pin is ENABLED; effective input current limit = min(ILIM_HIZ, IIN_DPM).");
        } else {
            defmt::info!("  Note: ILIM_HIZ pin is DISABLED; input current limited by IIN_DPM only (plus source capability).");
        }
        if hiz {
            defmt::info!("  Note: HIZ is set -> adapter draw is disabled; charging is suspended.");
        } else if iindpm_active || vindpm_active {
            defmt::info!("  Note: A DPM loop is ACTIVE -> the charger is throttling to protect the input.");
        }

        Ok(())
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
                        if let Err(e) = dev.dump_limits().await {
                            defmt::warn!("dump_limits failed: {:?}", defmt::Debug2Format(&e));
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
            let _ = dev.dump_limits().await;
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

            let reg06 = match dev.read2(REG_INPUT_CURRENT_DPM_LIMIT).await {
                Ok(v)=>v, Err(e)=>{ defmt::warn!("BQ25756: read REG06 failed: {:?}", defmt::Debug2Format(&e)); online=false; continue; }
            };
            let reg02 = match dev.read2(REG_CHARGE_CURRENT_LIMIT).await {
                Ok(v)=>v, Err(e)=>{ defmt::warn!("BQ25756: read REG02 failed: {:?}", defmt::Debug2Format(&e)); online=false; continue; }
            };
            // Extract the coded fields we program/use
            let iac_dpm_code = (reg06 & IAC_DPM_MASK) >> IAC_DPM_SHIFT; // bits 10:2
            let ichg_code    = reg02;                                   // 50 mA/LSB

            let ichg_from_code_ma: u32 = (ichg_code as u32) * 50;

            defmt::debug!(
                "BQ25756 RAW: REG06=0x{:04X} (IAC_DPM_CODE={})  REG02=0x{:04X} (ICHG_CODE={} -> {} mA)",
                reg06, iac_dpm_code, reg02, ichg_code, ichg_from_code_ma
            );

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
            "BQ25756: VAC={}mV (raw={})  VBAT={}mV  IAC={}mA  IBAT={}mA  [VIN_DPM={}mV IIN_DPM={}mA] VREG={}mV ICHG={}mA",
            readings.vac_mv,
            readings.vac_raw,
            readings.vbat_mv,
            readings.iac_ma,
            readings.ibat_ma,
            readings.input_voltage_dpm_limit_mv,
            readings.input_current_dpm_limit_ma,
            readings.charge_voltage_limit_mv,
            readings.charge_current_limit_ma
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
