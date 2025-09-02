//! Register addresses and bit masks for the BQ76952.
#![allow(dead_code)]

/// Subcommand addresses (for writing to `0x3E`/`0x3F`).
/// See TRM §12.2, Table 12-2 (p.112-119).
pub mod subcmd {
    pub const RESET: u16 = 0x0012;
    pub const FET_ENABLE: u16 = 0x0022;
    pub const CONFIG_UPDATE: u16 = 0x0090;
    pub const EXIT_CONFIG_UPDATE: u16 = 0x0092;
    pub const DEEP_SLEEP: u16 = 0x0020;
    pub const SHUTDOWN: u16 = 0x0010;
}

/// Data memory addresses.
/// See TRM §12.2.1 (p.120).
pub mod datamem {
    /// Base address for cell balancing time registers.
    pub const CELL_BALANCING_TIME_BASE: u16 = 0x9352;

    // --- Protection settings ---
    pub const CUV_THRESHOLD: u16 = 0x9275;
    pub const CUV_DELAY: u16 = 0x9276;
    pub const COV_THRESHOLD: u16 = 0x9278;
    pub const COV_DELAY: u16 = 0x9279;
    pub const OCC_THRESHOLD: u16 = 0x9280;
    pub const OCC_DELAY: u16 = 0x9281;
    pub const OCD1_THRESHOLD: u16 = 0x9282;
    pub const OCD1_DELAY: u16 = 0x9283;
    pub const SCD_THRESHOLD: u16 = 0x9286;
    pub const SCD_DELAY: u16 = 0x9287;
    pub const I2C_ADDRESS: u16 = 0x923A;

    pub const CONFIG_UPDATE:       u16 = 0x0090;
    pub const EXIT_CONFIG_UPDATE:  u16 = 0x0092;
    pub const SWAP_COMM_MODE:      u16 = 0x29BC;
    pub const SWAP_TO_I2C:         u16 = 0x29E7;
}