//! src/shared_state.rs

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::pubsub::PubSubChannel;

/// The data structure that will be broadcast from the BMS task.
/// It must be `Copy` to be used in a PubSubChannel.
#[derive(Clone, Copy, Default, defmt::Format)]
pub struct BmsReadings {
    pub voltage_mv: u16,
    pub current_ma: i16,
    pub temperature_c: i8,
}

/// The PubSubChannel for broadcasting BMS data.
///
/// This channel has a buffer for 2 messages, can support up to 3 subscribers,
/// and only ever expects 1 publisher. The values can be tuned if needed.
pub static BMS_CHANNEL: PubSubChannel<CriticalSectionRawMutex, BmsReadings, 2, 3, 1> = PubSubChannel::new();

/// The data structure that will be broadcast from the BQ25756 charger task.
#[derive(Clone, Copy, Default, defmt::Format)]
pub struct Bq25756Readings {
    // Limits
    pub charge_voltage_limit_mv: u16,
    pub charge_current_limit_ma: u16,
    pub input_current_dpm_limit_ma: u16,
    pub input_voltage_dpm_limit_mv: u16,
    // Telemetry
    pub vbat_mv: u16,
    pub vac_raw: u16,
    pub iac_raw: u16,
    pub ibat_raw: u16,

    pub iac_ma: i32,
    pub ibat_ma: i32,
    pub vac_mv: u16,
}

/// The PubSubChannel for broadcasting BQ25756 data.
pub static BQ25756_CHANNEL: PubSubChannel<CriticalSectionRawMutex, Bq25756Readings, 2, 3, 1> = PubSubChannel::new();

/// The data structure that will be broadcast from the CYPD3177 USB-PD task.
#[derive(Clone, Copy, Default, defmt::Format)]
pub struct CypdReadings {
    pub vbus_mv: u16,
    pub typec_connected: bool,
    pub typec_polarity_cc2: bool,
    pub typec_current_level: u8,
    pub pd_contract: bool,
    pub pd_sink_tx_ok: bool,
    pub pd_pe_snk_ready: bool,
}

/// The PubSubChannel for broadcasting CYPD3177 data.
pub static CYPD_CHANNEL: PubSubChannel<CriticalSectionRawMutex, CypdReadings, 2, 3, 1> = PubSubChannel::new();

/// The commands that can be sent to the BQ25756 charger task.
#[derive(Clone, Copy, defmt::Format)]
pub enum Bq25756Command {
    SetChargeVoltageLimit(u16),
    SetChargeCurrentLimit(u16),
    SetInputCurrentDpmLimit(u16),
    SetInputVoltageDpmLimit(u16),
    SetPrechargeCurrentLimit(u16),
    SetTerminationCurrentLimit(u16),
    EnableCharger(bool),
    EnableHiz(bool),
    Init,
}

/// The PubSubChannel for sending commands to the BQ25756 task.
pub static BQ25756_COMMAND_CHANNEL: PubSubChannel<CriticalSectionRawMutex, Bq25756Command, 2, 1, 3> = PubSubChannel::new();

/// The commands that can be sent to the BQ76952 BMS task.
#[derive(Clone, Copy, defmt::Format)]
pub enum BmsCommand {
    ToggleFetEnable,
    SetBalancingTime(u8, u8),
    SetCellOvervoltageProtection(u16, u16),
    SetCellUndervoltageProtection(u16, u16),
    SetI2cAddress(u8, Option<(u16, u16)>),
}

/// The PubSubChannel for sending commands to the BQ76952 task.
pub static BMS_COMMAND_CHANNEL: PubSubChannel<CriticalSectionRawMutex, BmsCommand, 2, 1, 3> = PubSubChannel::new();
