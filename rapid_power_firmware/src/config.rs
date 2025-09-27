/// Internal die temperature into BMS readings.
pub const READ_INTERNAL_TEMP: bool = true;

/// Turn on autonomous balancing in CHARGE and RELAX states at bring-up.
pub const ENABLE_CB_CHG: bool = true;
pub const ENABLE_CB_RLX: bool = true;

pub const BQ76952_LOG_PERIOD_MS: u64 = 5_000; // log frequency BMS in ms


/// --- BQ25756 configuration ---
pub const BQ25756_LOG_PERIOD_MS: u64 = 2_000; // log frequency Charger in ms

pub const BQ25756_EN_ILIM_HIZ_PIN: bool = false;
pub const BQ25756_EN_ICHG_PIN: bool = false;

/// Divider on BQ25756 FB pin (VBAT -> R_TOP -> FB -> R_BOT -> FBG/PGND)
pub const BQ25756_R_TOP_OHM: u32 = 240_000;
pub const BQ25756_R_BOT_OHM: u32 = 33_000;

pub const BQ25756_R_BATTERY_SENSE_MOHM: u16 = 5;
pub const BQ25756_R_INPUT_SENSE_MOHM: u16 = 5;

pub const BQ25756_CHG_VOLTAGE_MAX_MV: u16 = 12_600; // Max charge voltage in mV
pub const BQ25756_CHG_CURRENT_MAX_MA: u16 = 3_000;   // Max charge current in mA

pub const BQ25756_INPUT_VOLTAGE_MAX_MV: u16 = 20_000; // Max input voltage in mV
pub const BQ25756_INPUT_CURRENT_MAX_MA: u16 = 3_000; // Max input current in mA

/// If false, we skip external TSx reads and temperature-heavy logs.
/// JEITA activated if present.
pub const BQ25756_EXTERNAL_THERMISTORS_PRESENT: bool = false;