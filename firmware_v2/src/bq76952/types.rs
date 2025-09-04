#![allow(non_camel_case_types)]

//! Contains type definitions, constants, and enums for the BQ76952 driver,
//! translated from the C++ library and cross-referenced with the
//! BQ76952 Technical Reference Manual (TRM).

/// 7-bit I²C address (default for BQ76952).
/// The original C library used 8-bit 0x10/0x11 (write/read). The 7-bit equivalent is 0x08.
pub const I2C_ADDR_7BIT: u8 = 0x08;

/// Direct command addresses used by this driver.
///
/// These are 1-byte addresses for reading/writing status, measurements, etc.
/// See TRM §12.1, Table 12-1 (p.91-93).
pub mod cmd {
    /// Write low byte of subcommand or data-memory address.
    pub const SUBCMD_LOW: u8 = 0x3E;
    /// Write high byte of subcommand or data-memory address.
    pub const SUBCMD_HI: u8 = 0x3F;
    /// Base address of the response buffer for subcommands.
    pub const RESP_START: u8 = 0x40;
    /// Response data length register (for data memory writes).
    pub const RESP_LEN: u8 = 0x61;
    /// Response checksum register (for data memory writes).
    pub const RESP_CHKSUM: u8 = 0x60;

    // --- Voltages ---
    /// Cell 1 Voltage. Subsequent cells are at `+2` intervals up to cell 16.
    pub const VCELL1: u8 = 0x14;
    /// Stack Voltage. (10mV units).
    pub const VSTACK: u8 = 0x34;

    // --- Currents ---
    /// CC2 Current (signed, user-configurable units, default mA).
    pub const CC2_CUR: u8 = 0x3A;

    // --- Status & Temperatures ---
    /// Battery Status register (16 bits). See `BatteryStatus` struct.
    pub const BATTERY_STATUS: u8 = 0x12;
    /// Latched Alarm Status register. Write 1 to clear bits.
    pub const ALARM_STATUS: u8 = 0x62;
    /// Raw (unlatched) Alarm Status register.
    pub const ALARM_RAW: u8 = 0x64;
    /// Internal die temperature (0.1 K units).
    pub const INT_TEMP: u8 = 0x68;
    /// FET Status register. See `get_temperature_status`, `is_charging`, `is_discharging`.
    pub const FET_STATUS: u8 = 0x7F;

    // --- Temperatures (Thermistors & FET pins) ---
    /// TS1 Temperature (0.1 K units).
    pub const TS1: u8 = 0x70;
    /// TS2 Temperature (0.1 K units).
    pub const TS2: u8 = 0x72;
    /// TS3 Temperature (0.1 K units).
    pub const TS3: u8 = 0x74;
}

/// Error type for the BQ76952 driver.
#[derive(Debug)]
pub enum Error<E> {
    /// I²C communication error.
    I2c(E),
    /// Invalid parameter.
    InvalidParameter,
    Timeout,
}

/// Available thermistors for temperature sensing.
/// See TRM §12.1.18–§12.1.20 (p.95-97).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Thermistor {
    /// Thermistor 1
    TS1,
    /// Thermistor 2
    TS2,
    /// Thermistor 3
    TS3,
}

/// FET identifiers for control operations.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Fet {
    /// Charging FET
    CHG,
    /// Discharging FET
    DCHG,
    /// Both FETs
    ALL,
}

/// Represents the state of a FET.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FetState {
    /// FET is turned off (disabled).
    Off,
    /// FET is turned on (enabled).
    On,
}

/// Short-circuit in discharge current protection thresholds.
/// See TRM §12.2.35 (p.137).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[allow(clippy::upper_case_acronyms)]
pub enum ScdThresh {
    /// 10 mV
    SCD_10,
    /// 20 mV
    SCD_20,
    /// 40 mV
    SCD_40,
    /// 60 mV
    SCD_60,
    /// 80 mV
    SCD_80,
    /// 100 mV
    SCD_100,
    /// 125 mV
    SCD_125,
    /// 150 mV
    SCD_150,
    /// 175 mV
    SCD_175,
    /// 200 mV
    SCD_200,
    /// 250 mV
    SCD_250,
    /// 300 mV
    SCD_300,
    /// 350 mV
    SCD_350,
    /// 400 mV
    SCD_400,
    /// 450 mV
    SCD_450,
    /// 500 mV
    SCD_500,
}

/// Bit-field representing protection status flags.
/// See TRM §12.1.3 (p.92).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Protection {
    pub sc_dchg: bool,
    pub oc2_dchg: bool,
    pub oc1_dchg: bool,
    pub oc_chg: bool,
    pub cell_ov: bool,
    pub cell_uv: bool,
}

/// Bit-field representing temperature protection flags.
/// See TRM §12.1.4 (p.92).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct TemperatureProtection {
    pub overtemp_fet: bool,
    pub overtemp_internal: bool,
    pub overtemp_dchg: bool,
    pub overtemp_chg: bool,
    pub undertemp_internal: bool,
    pub undertemp_dchg: bool,
    pub undertemp_chg: bool,
}

/// Bit-field representing the battery status register.
/// See TRM §12.1.2 (p.91).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct BatteryStatus {
    pub sleep_mode: bool,
    pub shutdown_pending: bool,
    pub permanent_fault: bool,
    pub safety_fault: bool,
    pub fuse_pin: bool,
    pub security_state: u8,
    pub wr_to_otp_blocked: bool,
    pub wr_to_otp_pending: bool,
    pub open_wire_check: bool,
    pub wd_was_triggered: bool,
    pub full_reset_occured: bool,
    pub sleep_en_allowed: bool,
    pub precharge_mode: bool,
    pub config_update_mode: bool,
}

/// Bit-field representing the FET status register.
/// See TRM §12.1.21 (p.97).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct FetStatus {
    pub chg_fet: bool,
    pub pchg_fet: bool,
    pub dsg_fet: bool,
    pub pdsg_fet: bool,
    pub dchg_pin: bool,
    pub ddsg_pin: bool,
    pub alert_pin: bool,
}

#[derive(Clone, Copy)]
pub enum ScdDelay {
    Us70,
    Us100,
    Us200,
    Us400,
}
#[derive(Clone, Copy)]
pub enum OcdDelay {
    Ms8,
    Ms20,
    Ms40,
    Ms80,
    Ms160,
    Ms320,
}
#[derive(Clone, Copy)]
pub enum OccDelay {
    Ms100,
    Ms200,
    Ms400,
    Ms800,
    S1,
    S2,
}

/// Bit-field representing the alarm status register.
/// See TRM §12.1.22 (p.98).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct AlarmStatus {
    pub cov: bool,
    pub cuv: bool,
    pub occ: bool,
    pub ocd1: bool,
    pub ocd2: bool,
    pub scd: bool,
    pub otf: bool,
    pub otd: bool,
    pub otc: bool,
    pub utf: bool,
    pub utd: bool,
    pub utc: bool,
}
