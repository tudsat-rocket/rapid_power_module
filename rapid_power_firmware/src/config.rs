//! Compile-time knobs for the power module firmware.
//! Edit these constants and rebuild.
use embassy_time::Duration;

/// If false, we skip external TSx reads and temperature-heavy logs.
pub const EXTERNAL_THERMISTORS_PRESENT: bool = false;

/// Internal die temperature into BMS readings.
pub const READ_INTERNAL_TEMP: bool = true;

/// Turn on autonomous balancing in CHARGE and RELAX states at bring-up.
pub const ENABLE_CB_CHG: bool = true;
pub const ENABLE_CB_RLX: bool = true;

pub const BQ76952_LOG_PERIOD_MS: u64 = 5_000; // log frequency BMS in ms
pub const BQ25756_LOG_PERIOD_MS: u64 = 5_000; // log frequency Charger in ms