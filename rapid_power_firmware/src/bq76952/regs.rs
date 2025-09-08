//! Register addresses and bit masks for the BQ76952.
#![allow(dead_code)]

/// Direct command addresses. (TRM §12.1)
pub mod cmd {
    // --- Subcommand / response window ---
    /// LOW byte write of subcommand or DF address
    pub const SUBCMD_LOW: u8 = 0x3E;
    /// HIGH byte write of subcommand or DF address
    pub const SUBCMD_HI:  u8 = 0x3F;
    /// Response / data buffer start
    pub const RESP_START: u8 = 0x40;
    /// Response checksum register (DF writes)
    pub const RESP_CHKSUM: u8 = 0x60;
    /// Response length register (DF writes)
    pub const RESP_LEN:   u8 = 0x61;

    // --- Status & temps ---
    /// Battery Status (16-bit)
    pub const BATTERY_STATUS: u8 = 0x12;
    /// Latched Alarm Status (write 1 to clear)
    pub const ALARM_STATUS:   u8 = 0x62;
    /// Raw Alarm Status (unlatched)
    pub const ALARM_RAW:      u8 = 0x64;
    /// Internal die temperature (0.1 K)
    pub const INT_TEMP:       u8 = 0x68;
    /// FET Status
    pub const FET_STATUS:     u8 = 0x7F;

    // --- Voltages ---
    /// Cell 1 voltage; next cells at +2 up to cell 16
    pub const VCELL1: u8 = 0x14;
    /// Stack voltage (10 mV units)
    pub const VSTACK: u8 = 0x34;

    // --- Currents ---
    /// CC2 current (signed; default mA units)
    pub const CC2_CUR: u8 = 0x3A;

    // --- Thermistors ---
    pub const TS1: u8 = 0x70;
    pub const TS2: u8 = 0x72;
    pub const TS3: u8 = 0x74;
}

/// Subcommands (write to 0x3E then 0x3F). (TRM §12.2, Table 12-2)
pub mod subcmd {
    pub const RESET:             u16 = 0x0012;
    pub const FET_ENABLE:        u16 = 0x0022;
    pub const DEEP_SLEEP:        u16 = 0x0020;
    pub const SHUTDOWN:          u16 = 0x0010;

    pub const CONFIG_UPDATE:     u16 = 0x0090;
    pub const EXIT_CONFIG_UPDATE:u16 = 0x0092;

    /// Apply comm settings (incl. I²C addr) from Data Memory
    pub const SWAP_COMM_MODE:    u16 = 0x29BC;

    pub const DASTATUS5: u16 = 0x0075;
    pub const DASTATUS6: u16 = 0x0076;
    pub const DASTATUS7: u16 = 0x0077;

    pub const CB_ACTIVE_CELLS: u16 = 0x0083;
}

/// Data Memory addresses. (TRM §12.2.1)
pub mod datamem {
    pub const CELL_BALANCING_TIME_BASE: u16 = 0x9352;

    // Protection settings
    pub const CUV_THRESHOLD: u16 = 0x9275;
    pub const CUV_DELAY:     u16 = 0x9276;
    pub const COV_THRESHOLD: u16 = 0x9278;
    pub const COV_DELAY:     u16 = 0x9279;
    pub const OCC_THRESHOLD: u16 = 0x9280;
    pub const OCC_DELAY:     u16 = 0x9281;
    pub const OCD1_THRESHOLD:u16 = 0x9282;
    pub const OCD1_DELAY:    u16 = 0x9283;
    pub const SCD_THRESHOLD: u16 = 0x9286;
    pub const SCD_DELAY:     u16 = 0x9287;

    pub const BAL_CFG_H1:    u16 = 0x9335;

    /// Settings:Configuration:I2C Address (expects 8-bit write address)
    pub const I2C_ADDRESS:   u16 = 0x923A;
}
