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

/// Convert a little-endian u16 ADC result into a signed i16-like value.
///
/// The ADC registers are read as u16; some conversions use the raw bits
/// interpreted as signed values. This helper performs a direct cast.
#[inline]
fn u16_to_i16(x: u16) -> i16 { x as i16 }

/// Run the provided future with a short I2C operation timeout.
///
/// Inputs
/// - `fut`: an async future performing an I2C operation that returns
///   `CoreResult<T, Error<E>>`.
///
/// Outputs
/// - On success returns `Ok(T)`. On I2C errors returns `Err(Error::I2c(_))` as
///   produced by the inner future. On timeout returns `Err(Error::Timeout)`.
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

        self.ts_enable(crate::config::BQ25756_EXTERNAL_THERMISTORS_PRESENT).await.ok();
        self.set_jeita(crate::config::BQ25756_EXTERNAL_THERMISTORS_PRESENT).await.ok();

        self.set_ilim_hiz_pin_enable(crate::config::BQ25756_EN_ILIM_HIZ_PIN).await?;
        self.set_ichg_pin_enable(crate::config::BQ25756_EN_ICHG_PIN).await?;

        self.adc_enable(true, true, 0b10, false).await.ok();
        self.adc_set_channels(true, true, true, true, crate::config::BQ25756_EXTERNAL_THERMISTORS_PRESENT).await.ok();

        self.enable_charger(true).await?;
        Ok(())
    }
    /// Create a new driver instance using the default 7-bit I²C address.
    ///
    /// The driver takes ownership of the I2C peripheral instance.
    pub fn new(i2c: I2C) -> Self { Self { i2c, addr: I2C_ADDR } }

    /// Create a new driver instance using an explicit 7-bit I²C address.
    pub fn with_address(i2c: I2C, addr: u8) -> Self { Self { i2c, addr } }

    /// Consume the driver and return the owned I²C peripheral.
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

    /// Read a single byte from the given register address.
    ///
    /// Performs a write-read I2C transaction: writes the register address
    /// then reads one byte. Returns `Error::I2c` for underlying bus errors
    /// and `Error::Timeout` if the operation times out.
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

    /// Read a 16-bit little-endian register from the device.
    ///
    /// Performs a write-read I2C transaction and returns the LE u16 value.
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

    /// Write a single-byte value to the given register (write transaction).
    ///
    /// Returns `Ok(())` on success or `Err(Error::I2c(_))` on bus errors.
    async fn write1(&mut self, reg: u8, val: u8) -> Result<I2C::Error> {
        with_i2c_timeout(async {
            self.i2c
                .write(self.addr, &[reg, val])
                .await
                .map_err(Error::I2c)
        }).await
    }

    /// Write a 16-bit little-endian value to the given register.
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
    /// Set a bitfield inside a 16-bit register value.
    ///
    /// Clears `mask` in `orig` and inserts `code << shift` masked by `mask`.
    #[inline]
    fn set_field_16(orig: u16, mask: u16, shift: u8, code: u16) -> u16 {
        (orig & !mask) | ((code << shift) & mask)
    }
    /// Extract a bitfield from a 16-bit register value using `mask` and `shift`.
    #[inline]
    fn get_field_16(reg: u16, mask: u16, shift: u8) -> u16 {
        (reg & mask) >> shift
    }

    // ------------------ Quantization helpers ------------------
    /// Encode `val` into register units using an integer step and floor rounding.
    ///
    /// Clamps `val` into [min_units, max_units] then divides by `step_units`.
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

    /// Encode using a rational LSB = numerator/denominator with floor rounding.
    ///
    /// Clamps `val_units` and computes floor((val_units * denominator) / numerator_units).
    /// This is used for shunt-aware coding of IAC_DPM.
    #[inline]
    fn enc_floor_ratio(val_units: u32, numerator_units: u32, denominator: u32,
                       min_units: u32, max_units: u32) -> u16 {
        let v = val_units.clamp(min_units, max_units);
        ((v as u64 * denominator as u64) / numerator_units as u64) as u16
    }
    /// Decode a rational-coded register field back into real units.
    #[inline]
    fn dec_ratio(code: u16, numerator_units: u32, denominator: u32) -> u16 {
        ((code as u32 * numerator_units) / denominator) as u16
    }

    // ----------- Shunt-aware DPM coding (per DS section 8.3.5.1.1) -----------
    // IAC_DPM code <-> mA. LSB(mA) = 250 / RmΩ.  (2 mΩ => 125 mA/LSB, 5 mΩ => 50 mA/LSB)
    /// Convert an input current limit in mA to the IAC_DPM register code,
    /// taking the input sense resistor (mΩ) into account.
    #[inline]
    fn iac_dpm_code_from_ma(ma: u32, r_milliohm: u16) -> u16 {
        ((ma as u64 * r_milliohm as u64) / 250u64) as u16
    }
    /// Convert IAC_DPM register code back to mA using sense resistor value.
    #[inline]
    fn iac_dpm_ma_from_code(code: u16, r_milliohm: u16) -> u16 {
        ((code as u32 * 250u32) / r_milliohm as u32) as u16
    }

    pub async fn int_unmask_all(&mut self) -> Result<I2C::Error> {
        self.write1(REG_CHARGER_MASK_1, 0x00).await?;
        self.write1(REG_CHARGER_MASK_2, 0x00).await?;
        self.write1(REG_FAULT_MASK,      0x00).await
    }

    /// Read all interrupt FLAG registers (clear-on-read) and return a
    /// snapshot of flags and status registers.
    ///
    /// Returns a tuple: ((FLAG1, FLAG2, FAULT_FLAG), (S1, S2, S3, FAULT_STATUS)).
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
    /// Convert a battery voltage (mV) to the VFB pin voltage (mV) using a resistor divider.
    #[inline]
    fn vbat_to_vfb_mv(vbat_mv: u32, r_top_ohm: u32, r_bot_ohm: u32) -> u16 {
        let num = (vbat_mv as u64) * (r_bot_ohm as u64);
        let den = (r_top_ohm as u64) + (r_bot_ohm as u64);
        (num / den) as u16
    }

    /// Inverse of vbat_to_vfb_mv.
    /// Convert the VFB pin voltage (mV) back to the battery voltage (mV) for the given divider.
    #[inline]
    fn vfb_to_vbat_mv(vfb_mv: u16, r_top_ohm: u32, r_bot_ohm: u32) -> u16 {
        let vfb = vfb_mv as u64;
        let sum = (r_top_ohm as u64) + (r_bot_ohm as u64);
        ((vfb * sum) / (r_bot_ohm as u64)) as u16
    }

    /// Return the min/max battery voltages (mV) that can be encoded given the divider
    /// and the device's allowed VFB range (1504..1566 mV).
    #[inline]
    fn vbat_encodable_range_mv(r_top_ohm: u32, r_bot_ohm: u32) -> (u16, u16) {
        let vmin = Self::vfb_to_vbat_mv(1504, r_top_ohm, r_bot_ohm);
        let vmax = Self::vfb_to_vbat_mv(1566, r_top_ohm, r_bot_ohm);
        (vmin, vmax)
    }

    /// Encode a VFB voltage (mV) into the 5-bit register code (2 mV/LSB).
    #[inline]
    fn vfb_mv_to_code(vfb_mv: u16) -> u16 {
        let v = vfb_mv.clamp(1504, 1566);
        ((v - 1504) / 2) as u16 & 0x1F
    }

    /// Decode the 5-bit VFB register code into a VFB voltage (mV).
    #[inline]
    fn vfb_code_to_mv(code: u16) -> u16 {
        1504 + ((code & 0x1F) * 2)
    }

    /// Generic encode/decode helpers with an integer step (mA per LSB).
    /// Encode a value using an integer step size with clamping.
    #[inline] fn enc_step(ma: u16, step_ma: u16, min_ma: u16, max_ma: u16) -> u16 {
        let clamped = ma.clamp(min_ma, max_ma);
        clamped / step_ma
    }
    // ICHG_REG: 50 mA/LSB, 0.4–20 A
    #[inline] fn ichg_ma_to_reg(ma: u16) -> u16 { Self::enc_step(ma, 50, 400, 20_000) }
    #[inline] fn ichg_reg_to_ma(code: u16) -> u16 { Self::dec_step(code, 50) }

    // ------------------ Charging Parameters ------------------

    /// Set the VFB regulation target (mV at the FB pin). Valid: 1504..1566 mV (2 mV/step).
    /// Program the VFB register from a VFB voltage in mV.
    ///
    /// This performs a read-modify-write on the 16-bit charge voltage register and
    /// updates only the VFB bits [4:0].
    pub async fn set_vfb_reg_mv(&mut self, vfb_mv: u16) -> Result<I2C::Error> {
        let code = Self::vfb_mv_to_code(vfb_mv);
        // read-modify-write 16-bit reg (we only touch bits [4:0])
        let mut reg = self.read2(REG_CHARGE_VOLTAGE_LIMIT).await?;
        reg = (reg & !0x001F) | code;
        self.write2(REG_CHARGE_VOLTAGE_LIMIT, reg).await
    }

    /// Set the charge voltage regulation target (battery voltage in mV).
    ///
    /// Uses the configured resistor divider (R_TOP/R_BOT) to compute the required
    /// VFB code. The programmed voltage is floored to the nearest representable
    /// value; warnings are logged if the requested voltage is outside the
    /// encodable range.
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

    /// Enable or disable the ILIM/HIZ pin functionality.
    ///
    /// When enabled the external pin can override input current or HIZ behavior.
    pub async fn set_ilim_hiz_pin_enable(&mut self, enable: bool) -> Result<I2C::Error> {
        let mut v = self.read1(REG_PIN_CONTROL).await?;
        if enable { v |= EN_ILIM_HIZ_PIN; } else { v &= !EN_ILIM_HIZ_PIN; }
        self.write1(REG_PIN_CONTROL, v).await
    }

    /// Enable or disable the ICHG external pin which can limit charge current.
    pub async fn set_ichg_pin_enable(&mut self, enable: bool) -> Result<I2C::Error> {
        let mut v = self.read1(REG_PIN_CONTROL).await?;
        if enable { v |= EN_ICHG_PIN; } else { v &= !EN_ICHG_PIN; }
        self.write1(REG_PIN_CONTROL, v).await
    }

    /// Read the programmed charge voltage limit and return it as battery voltage (mV).
    ///
    /// Decodes the VFB register and converts via the resistor divider to VBAT mV.
    pub async fn get_charge_voltage_limit(&mut self) -> CoreResult<u16, Error<I2C::Error>> {
        let reg = self.read2(REG_CHARGE_VOLTAGE_LIMIT).await?;
        let vfb_code = reg & 0x001F;
        let vfb_mv = Self::vfb_code_to_mv(vfb_code);
        let rtop = crate::config::BQ25756_R_TOP_OHM;
        let rbot = crate::config::BQ25756_R_BOT_OHM;
        Ok(Self::vfb_to_vbat_mv(vfb_mv, rtop, rbot))
    }

    /// Set the fast-charge current limit in milliamperes.
    ///
    /// The device uses a 50 mA/LSB encoding; values are clamped to the valid range.
    pub async fn set_charge_current_limit(&mut self, ma: u16) -> Result<I2C::Error> {
        let code = Self::ichg_ma_to_reg(ma);
        self.write2(REG_CHARGE_CURRENT_LIMIT, code).await
    }

    /// Read the programmed fast-charge current limit and return it in milliamperes.
    pub async fn get_charge_current_limit(&mut self) -> CoreResult<u16, Error<I2C::Error>> {
        let reg = self.read2(REG_CHARGE_CURRENT_LIMIT).await?;
        Ok(Self::ichg_reg_to_ma(reg))
    }

    /// Set the input current DPM limit (mA).
    ///
    /// This encoding is shunt-aware and depends on the configured input sense resistor.
    pub async fn set_input_current_dpm_limit(&mut self, ma: u16) -> Result<I2C::Error> {
        let r_milliohm = crate::config::BQ25756_R_INPUT_SENSE_MOHM as u32;
        let code = Self::enc_floor_ratio(ma as u32, 250, r_milliohm, 400, 20_000);
        let mut reg = self.read2(REG_INPUT_CURRENT_DPM_LIMIT).await?;
        reg = Self::set_field_16(reg, IAC_DPM_MASK, IAC_DPM_SHIFT, code);
        self.write2(REG_INPUT_CURRENT_DPM_LIMIT, reg).await
    }

    /// Read the programmed input current DPM limit and return it in milliamperes.
    pub async fn get_input_current_dpm_limit(&mut self) -> CoreResult<u16, Error<I2C::Error>> {
        let r_milliohm = crate::config::BQ25756_R_INPUT_SENSE_MOHM as u32;
        let reg = self.read2(REG_INPUT_CURRENT_DPM_LIMIT).await?;
        let code = Self::get_field_16(reg, IAC_DPM_MASK, IAC_DPM_SHIFT);
        Ok(Self::dec_ratio(code, 250, r_milliohm)) 
    }

    /// Set the input voltage DPM limit in millivolts.
    ///
    /// Value is clamped to the device's allowed range before programming.
    pub async fn set_input_voltage_dpm_limit(&mut self, mv: u16) -> Result<I2C::Error> {
        let mv = mv.clamp(3600, 22000);
        self.write2(REG_INPUT_VOLTAGE_DPM_LIMIT, mv).await
        
    }

    /// Read the programmed input voltage DPM limit (mV).
    pub async fn get_input_voltage_dpm_limit(&mut self) -> CoreResult<u16, Error<I2C::Error>> {
        self.read2(REG_INPUT_VOLTAGE_DPM_LIMIT).await
    }

    /// Read the charger status register and decode its charging state.
    pub async fn get_charge_stat(&mut self) -> CoreResult<ChargeStat, Error<I2C::Error>> {
        let s1 = self.read1(REG_CHARGER_STATUS_1).await?;
        Ok(ChargeStat::from_code(s1))
    }

    /// Read and log a human-readable snapshot of configured limits and live readings.
    ///
    /// This routine is intended for debugging and informational logs; it reads
    /// multiple registers and ADC channels and converts values to engineering units.
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

    /// Enable or disable JEITA battery temperature profile behavior (cool/warm derates & CV offset).
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

    /// Read the raw TS/JEITA behavior control register byte.
    pub async fn get_ts_behavior(&mut self) -> CoreResult<u8, Error<I2C::Error>> {
        self.read1(REG_TS_BEHAVIOR_CTRL).await
    }

    // ------------------ TS / Thermistor ADC & math ------------------
    /// Read raw TS ADC (masked to the valid 10-bit field).
    pub async fn read_ts_raw(&mut self) -> CoreResult<u16, Error<I2C::Error>> {
        let raw16 = self.read2(REG_TS_ADC).await?;
        Ok(raw16 & TS_ADC_MASK)
    }

    /// Convert 10-bit TS code to percent of REGN (0.0 .. <100.0).
    #[inline]
    fn ts_raw_to_percent(raw10: u16) -> f32 {
        (raw10 as f32) * (100.0 / 1024.0)
    }


    // ------------------ Pre-charge and Termination ------------------

    /// Set the precharge current limit (mA). Value is clamped to allowed range.
    pub async fn set_precharge_current_limit(&mut self, ma: u16) -> Result<I2C::Error> {
        let ma = ma.clamp(0, 1024);
        self.write2(REG_PRECHARGE_CURRENT_LIMIT, ma).await
    }

    /// Read the programmed precharge current limit (mA).
    pub async fn get_precharge_current_limit(&mut self) -> CoreResult<u16, Error<I2C::Error>> {
        self.read2(REG_PRECHARGE_CURRENT_LIMIT).await
    }

    /// Set the termination current limit (mA). Value is clamped to allowed range.
    pub async fn set_termination_current_limit(&mut self, ma: u16) -> Result<I2C::Error> {
        let ma = ma.clamp(0, 1024);
        self.write2(REG_TERMINATION_CURRENT_LIMIT, ma).await
    }

    /// Read the programmed termination current limit (mA).
    pub async fn get_termination_current_limit(&mut self) -> CoreResult<u16, Error<I2C::Error>> {
        self.read2(REG_TERMINATION_CURRENT_LIMIT).await
    }

    // ------------------ Public Raw Reads for Debug / Snapshot ------------------

    /// Read the raw TIMER_CONTROL register byte.
    pub async fn get_timer_control(&mut self) -> CoreResult<u8, Error<I2C::Error>> {
        self.read1(REG_TIMER_CONTROL).await
    }

    /// Read the raw CHARGER_CONTROL register byte.
    pub async fn get_charger_control(&mut self) -> CoreResult<u8, Error<I2C::Error>> {
        self.read1(REG_CHARGER_CONTROL).await
    }

    // ------------------ Timer Controls ------------------

    /// Set the top-off timer selection.
    pub async fn set_topoff_timer(&mut self, timer: TopOffTimer) -> Result<I2C::Error> {
        let mut val = self.read1(REG_TIMER_CONTROL).await?;
        val &= !TOPOFF_TMR_MASK;
        val |= (timer as u8) << 6;
        self.write1(REG_TIMER_CONTROL, val).await
    }

    /// Set the watchdog timer selection.
    pub async fn set_watchdog_timer(&mut self, timer: WdtTimer) -> Result<I2C::Error> {
        let mut val = self.read1(REG_TIMER_CONTROL).await?;
        val &= !WDT_MASK;
        val |= (timer as u8) << 4;
        self.write1(REG_TIMER_CONTROL, val).await
    }

    /// Set the charge timer selection used by the charger state machine.
    pub async fn set_charge_timer(&mut self, timer: ChgTmr) -> Result<I2C::Error> {
        let mut val = self.read1(REG_TIMER_CONTROL).await?;
        val &= !CHG_TMR_MASK;
        val |= (timer as u8) << 1;
        self.write1(REG_TIMER_CONTROL, val).await
    }

    /// Enable or disable the 2x timer multiplier (double speed timers).
    pub async fn enable_2x_timer(&mut self, enable: bool) -> Result<I2C::Error> {
        let mut val = self.read1(REG_TIMER_CONTROL).await?;
        if enable { val |= EN_TMR2X; } else { val &= !EN_TMR2X; }
        self.write1(REG_TIMER_CONTROL, val).await
    }

    // ------------------ Misc Controls ------------------

    /// Enable or disable the charger engine (EN_CHG bit).
    pub async fn enable_charger(&mut self, enable: bool) -> Result<I2C::Error> {
        let mut val = self.read1(REG_CHARGER_CONTROL).await?;
        if enable { val |= EN_CHG; } else { val &= !EN_CHG; }
        self.write1(REG_CHARGER_CONTROL, val).await
    }

    /// Enable or disable HIZ mode (adapter draw disconnected).
    pub async fn enable_hiz(&mut self, enable: bool) -> Result<I2C::Error> {
        let mut val = self.read1(REG_CHARGER_CONTROL).await?;
        if enable { val |= EN_HIZ; } else { val &= !EN_HIZ; }
        self.write1(REG_CHARGER_CONTROL, val).await
    }

    /// Pulse the watchdog reset bit to kick the device watchdog.
    pub async fn reset_watchdog_timer(&mut self) -> Result<I2C::Error> {
        let mut val = self.read1(REG_CHARGER_CONTROL).await?;
        val |= WD_RST;
        self.write1(REG_CHARGER_CONTROL, val).await
    }

    // --- ADC config
    /// Configure ADC enable, conversion mode, sample bit depth and averaging.
    ///
    /// - `enable`: enable ADC subsystem
    /// - `continuous`: true for continuous conversions, false for one-shot
    /// - `sample_bits_efr`: value to program ADC sample resolution field (2 bits)
    /// - `avg`: enable averaging
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

    /// Enable/disable ADC channels. Note: bits in REG_ADC_CHANNEL_CTRL are "*_DIS"
    /// so `true` means enable and will clear the corresponding DIS bit.
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
    /// Read the VBAT ADC and return battery voltage in millivolts.
    pub async fn read_vbat_mv(&mut self) -> CoreResult<u16, Error<I2C::Error>> {
        use crate::bq25756::regs::*;
        Ok(self.read2(REG_VBAT_ADC).await? * 2) // 2 mV/LSB
    }
    /// Read the raw VAC ADC register (units are device LSBs).
    pub async fn read_vac_raw(&mut self)  -> CoreResult<u16, Error<I2C::Error>> {
        use crate::bq25756::regs::*; self.read2(REG_VAC_ADC).await
    }
    /// Read VAC and return millivolts (2 mV/LSB conversion applied).
    pub async fn read_vac_mv(&mut self)  -> CoreResult<u16, Error<I2C::Error>> {
        Ok(self.read2(REG_VAC_ADC).await? * 2) // 2 mV/LSB
    }
    /// Read the raw input AC current ADC register.
    pub async fn read_iac_raw(&mut self)  -> CoreResult<u16, Error<I2C::Error>> {
        use crate::bq25756::regs::*; self.read2(REG_IAC_ADC).await
    }
    /// Read the IAC ADC and convert to milliamperes given the sense resistor (mΩ).
    ///
    /// This uses the datasheet LSB expression and returns a signed value to handle
    /// bi-directional current readings.
    pub async fn read_iac_ma(&mut self, rac_milliohm: u16)
        -> CoreResult<i32, Error<I2C::Error>>
    {
        let raw = u16_to_i16(self.read2(REG_IAC_ADC).await?) as i32;
        let lsb_ma_num = 4i32; // numerator of (4 / RmΩ)
        Ok(raw * lsb_ma_num / (rac_milliohm as i32))
    }
    /// Read the raw IBAT ADC register (device LSBs).
    pub async fn read_ibat_raw(&mut self) -> CoreResult<u16, Error<I2C::Error>> {
        use crate::bq25756::regs::*; self.read2(REG_IBAT_ADC).await
    }
    /// Read IBAT ADC and convert to milliamperes given battery sense resistor (mΩ).
    pub async fn read_ibat_ma(&mut self, rbat_milliohm: u16)
        -> CoreResult<i32, Error<I2C::Error>>
    {
        let raw = u16_to_i16(self.read2(REG_IBAT_ADC).await?) as i32;
        let lsb_ma_num = 10i32; // numerator of (10 / RmΩ)
        Ok(raw * lsb_ma_num / (rbat_milliohm as i32))
    }

    // Set TS thresholds (T1/T2/T3/T5)
    /// Program thermal sense threshold selection fields (T1/T2/T3/T5).
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
    /// Configure JEITA temperature behavior and related current/voltage derates.
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
    /// Enable or disable the external temperature sensor (TS) monitoring.
    pub async fn ts_enable(&mut self, enable: bool) -> Result<I2C::Error> {
        let mut v = self.read1(REG_TS_BEHAVIOR_CTRL).await?;
        if enable { v |= EN_TS; } else { v &= !EN_TS; }
        self.write1(REG_TS_BEHAVIOR_CTRL, v).await
    }
}

// Alias used by the task: concrete driver type that wraps the project's I2C device helper.
/// Concrete driver type used by the async tasks (BQ device instance wrapped around I2cDev).
type Bq25756Dev<'a> = Bq25756<super::super::I2cDev<'a>>;

use embassy_sync::pubsub::Subscriber;

/// Background task that probes and periodically polls the BQ25756 device.
///
/// Responsibilities:
/// - Probe and initialize the device when it becomes reachable on I2C.
/// - Accept commands from `BQ25756_COMMAND_CHANNEL` and apply them to the device.
/// - Periodically sample readings and publish `Bq25756Readings`.
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
                        // Dump once at bring-up for a clear snapshot
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
            // After any successful command, dump current limits so we see the effect
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

            let ts_beh = dev.read1(REG_TS_BEHAVIOR_CTRL).await.unwrap_or(0);
            let en_ts    = (ts_beh & EN_TS) != 0;
            let en_jeita = (ts_beh & EN_JEITA) != 0;
            let jeita_vset  = (ts_beh & JEITA_VSET_MASK) >> 5; // 00 suspend, 01=94.3%, 10=97.6%, 11=100%
            let jeita_isetc = (ts_beh & JEITA_ISETC_MASK) >> 2; // 00=0%, 01=20%, 10=40%, 11=100%
            let jeita_iseth = (ts_beh & JEITA_ISETH) != 0;      // warm region current: 0=40%, 1=100%
            defmt::info!(
            "  TS: EN_TS={} EN_JEITA={}  VSET={} ISETC={} ISETH_100%={}",
            en_ts, en_jeita, jeita_vset, jeita_isetc, jeita_iseth
            );
            
            // --- Raw limit registers (input current & charge current)
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

        // // Optional periodic dump: when an adapter is present (VAC ~≥ 9 V / 20 V),
        // // this gives a live view of who is limiting (VIN_DPM / IIN_DPM / pins).
        // if readings.vac_mv > 4500 {
        //     let mut dev = bq.lock().await;
        //     if let Err(e) = dev.dump_limits().await {
        //         defmt::debug!("dump_limits (periodic) failed: {:?}", defmt::Debug2Format(&e));
        //     }
        // }
        if let Err(_e) = publisher.try_publish(readings) {
            defmt::debug!("BQ25756 drop (no subscriber)");
        }


        ticker.next().await;
    }
}

/// Interrupt handler task for the BQ25756 INT pin (open-drain, active-low).
///
/// Waits for a falling edge, reads and logs interrupt flags/status, and re-enables
/// ADC continuous conversions to refresh live metrics after an interrupt.
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

/// Handler task for the Power-Good (PG) pin. Logs PG transitions and refreshes ADC.
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

/// Handler task for STAT1 pin edges. Logs raw status registers when the pin changes.
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

/// Handler task for STAT2 pin edges. Logs raw status registers when the pin changes.
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
