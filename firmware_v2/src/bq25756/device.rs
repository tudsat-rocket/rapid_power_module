//! High-level driver API (async, owns I²C device so it can live in a Mutex).

use core::result::Result as CoreResult;
use embedded_hal_async::i2c::I2c;

use crate::bq25756::regs::*;
use crate::bq25756::types::*;

pub type Result<E> = CoreResult<(), Error<E>>;

/// BQ25756 async driver over I²C.
pub struct Bq25756<I2C> {
    i2c: I2C,
    addr: u8,
}

impl<I2C> Bq25756<I2C>
where
    I2C: I2c,
{
    /// Create with default 7-bit I²C address (0x6B).
    pub fn new(i2c: I2C) -> Self { Self { i2c, addr: I2C_ADDR } }

    /// Create with explicit address.
    pub fn with_address(i2c: I2C, addr: u8) -> Self { Self { i2c, addr } }

    /// Consume and return the underlying I²C device.
    pub fn release(self) -> I2C { self.i2c }

    // ------------------ Core / Identity ------------------

    /// Read WHOAMI (Part number). Expects 0b0010 (2) for BQ25756.
    /// NOTE: The register/mask constants are placeholders. Only use if verified.
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

    #[inline]
    fn clamp_u16(x: u16, lo: u16, hi: u16) -> u16 { core::cmp::min(hi, core::cmp::max(x, lo)) }

    /// Read single-byte register.
    async fn read1(&mut self, reg: u8) -> CoreResult<u8, Error<I2C::Error>> {
        let mut buf = [0u8; 1];
        self.i2c
            .write_read(self.addr, &[reg], &mut buf)
            .await
            .map_err(Error::I2c)?;
        Ok(buf[0])
    }

    /// Read two-byte register.
    async fn read2(&mut self, reg: u8) -> CoreResult<u16, Error<I2C::Error>> {
        let mut buf = [0u8; 2];
        self.i2c
            .write_read(self.addr, &[reg], &mut buf)
            .await
            .map_err(Error::I2c)?;
        Ok(u16::from_le_bytes(buf))
    }

    /// Write single-byte register.
    async fn write1(&mut self, reg: u8, val: u8) -> Result<I2C::Error> {
        self.i2c
            .write(self.addr, &[reg, val])
            .await
            .map_err(Error::I2c)
    }

    /// Write two-byte register.
    async fn write2(&mut self, reg: u8, val: u16) -> Result<I2C::Error> {
        let bytes = val.to_le_bytes();
        self.i2c
            .write(self.addr, &[reg, bytes[0], bytes[1]])
            .await
            .map_err(Error::I2c)
    }

    // ------------------ Charging Parameters ------------------

    /// Set charge voltage limit (Vreg).
    pub async fn set_charge_voltage_limit(&mut self, mv: u16) -> Result<I2C::Error> {
        let mv = Self::clamp_u16(mv, 8192, 19200);
        self.write2(REG_CHARGE_VOLTAGE_LIMIT, mv).await
    }

    /// Read charge voltage limit (Vreg).
    pub async fn get_charge_voltage_limit(&mut self) -> CoreResult<u16, Error<I2C::Error>> {
        self.read2(REG_CHARGE_VOLTAGE_LIMIT).await
    }

    /// Set charge current limit (Ireg).
    pub async fn set_charge_current_limit(&mut self, ma: u16) -> Result<I2C::Error> {
        let ma = Self::clamp_u16(ma, 0, 8000);
        self.write2(REG_CHARGE_CURRENT_LIMIT, ma).await
    }

    /// Read charge current limit (Ireg).
    pub async fn get_charge_current_limit(&mut self) -> CoreResult<u16, Error<I2C::Error>> {
        self.read2(REG_CHARGE_CURRENT_LIMIT).await
    }

    /// Set input current DPM limit (IIN_DPM).
    pub async fn set_input_current_dpm_limit(&mut self, ma: u16) -> Result<I2C::Error> {
        let ma = Self::clamp_u16(ma, 0, 8000);
        self.write2(REG_INPUT_CURRENT_DPM_LIMIT, ma).await
    }

    /// Read input current DPM limit (IIN_DPM).
    pub async fn get_input_current_dpm_limit(&mut self) -> CoreResult<u16, Error<I2C::Error>> {
        self.read2(REG_INPUT_CURRENT_DPM_LIMIT).await
    }

    /// Set input voltage DPM limit (VIN_DPM).
    pub async fn set_input_voltage_dpm_limit(&mut self, mv: u16) -> Result<I2C::Error> {
        let mv = Self::clamp_u16(mv, 3600, 22000);
        self.write2(REG_INPUT_VOLTAGE_DPM_LIMIT, mv).await
    }

    /// Read input voltage DPM limit (VIN_DPM).
    pub async fn get_input_voltage_dpm_limit(&mut self) -> CoreResult<u16, Error<I2C::Error>> {
        self.read2(REG_INPUT_VOLTAGE_DPM_LIMIT).await
    }

    // ------------------ Pre-charge and Termination ------------------

    pub async fn set_precharge_current_limit(&mut self, ma: u16) -> Result<I2C::Error> {
        let ma = Self::clamp_u16(ma, 0, 1024);
        self.write2(REG_PRECHARGE_CURRENT_LIMIT, ma).await
    }

    pub async fn get_precharge_current_limit(&mut self) -> CoreResult<u16, Error<I2C::Error>> {
        self.read2(REG_PRECHARGE_CURRENT_LIMIT).await
    }

    pub async fn set_termination_current_limit(&mut self, ma: u16) -> Result<I2C::Error> {
        let ma = Self::clamp_u16(ma, 0, 1024);
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
}
