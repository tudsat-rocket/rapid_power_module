//! Human-readable decoders for flags/status/faults (BQ25756).

use defmt::{info};
use super::regs::*;
use super::types::ChargeStat;

#[inline]
fn log_named_bits(tag: &str, val: u8, map: &[(&'static str, u8)]) {
    let mut any = false;
    for (name, mask) in map {
        if (val & *mask) != 0 {
            if !any { info!("{}:", tag); any = true; }
            info!("  - {}", *name);
        }
    }
    if !any {
        info!("{}: (none set, 0x{:02X})", tag, val);
    }
}

pub fn log_flag1(f1: u8) {
    // REG0x25 – bits are “FLAGs (clear-on-read)”
    const MAP: &[(&str, u8)] = &[
        ("ADC_DONE_FLAG", 1<<7),   // one-shot complete
        ("IAC_DPM_FLAG",  1<<6),
        ("VAC_DPM_FLAG",  1<<5),
        ("WD_FLAG",       1<<3),
        ("CV_TMR_FLAG",   1<<1),
        ("CHARGE_FLAG",   1<<0),
    ];
    info!("FLAG1 = 0x{:02X}", f1);
    log_named_bits("FLAG1 bits", f1, MAP);
}

pub fn log_flag2(f2: u8) {
    // REG0x26
    const MAP: &[(&str, u8)] = &[
        ("PG_FLAG",        1<<7),
        ("TS_FLAG",        1<<4),
        ("REVERSE_FLAG",   1<<3),
        ("FSW_SYNC_FLAG",  1<<1),
        ("MPPT_FLAG",      1<<0),
    ];
    info!("FLAG2 = 0x{:02X}", f2);
    log_named_bits("FLAG2 bits", f2, MAP);
}

pub fn log_fault_flag(ff: u8) {
    // REG0x27
    const MAP: &[(&str, u8)] = &[
        ("VAC_UV_FLAG",   1<<7),
        ("VAC_OV_FLAG",   1<<6),
        ("IBAT_OCP_FLAG", 1<<5),
        ("VBAT_OV_FLAG",  1<<4),
        ("TSHUT_FLAG",    1<<3),
        ("CHG_TMR_FLAG",  1<<2),
        ("DRV_OKZ_FLAG",  1<<1),
    ];
    info!("FAULT_FLAG = 0x{:02X}", ff);
    log_named_bits("FAULT_FLAG bits", ff, MAP);
}

pub fn log_status1(s1: u8) {
    info!("STATUS1 = 0x{:02X}", s1);
    let cs = ChargeStat::from_code(s1);
    info!("  - CHARGE_STAT: {:?}", cs);
    if (s1 & IINDPM_STAT) != 0 { info!("  - IINDPM_STAT: Input current limiting ACTIVE"); }
    if (s1 & VINDPM_STAT) != 0 { info!("  - VINDPM_STAT: Input voltage limiting ACTIVE"); }
}

pub fn log_status2(s2: u8) {
    info!("STATUS2 = 0x{:02X}", s2);
    if (s2 & PG_STAT) != 0 { info!("  - PG_STAT: Power Good = TRUE"); }
    else { info!("  - PG_STAT: Power Good = FALSE"); }
    // Add more STATUS2 bits here if you later expose masks in regs.rs
}

pub fn log_status3(s3: u8) {
    info!("STATUS3 = 0x{:02X}", s3);
    // FSW_SYNC_STAT is bits [5:4]
    match (s3 >> 4) & 0b11 {
        0b00 => info!("  - FSW_SYNC_STAT: Internal clock (no ext sync)"),
        0b01 => info!("  - FSW_SYNC_STAT: External clock valid"),
        0b10 => info!("  - FSW_SYNC_STAT: External clock fault/out-of-range"),
        _    => info!("  - FSW_SYNC_STAT: Reserved"),
    }
    if (s3 & (1<<3)) != 0 { info!("  - CV_TMR_STAT: CV timer expired"); }
    if (s3 & (1<<2)) != 0 { info!("  - REVERSE_STAT: Reverse Mode active"); }
}

pub fn log_fault_status(fs: u8) {
    // REG0x24 – latched status bits (not clear-on-read)
    const MAP: &[(&str, u8)] = &[
        ("VAC_UV_STAT (Input UVP)", 1<<7),
        ("VAC_OV_STAT (Input OVP)", 1<<6),
        ("IBAT_OCP_STAT",           1<<5),
        ("VBAT_OV_STAT",            1<<4),
        ("TSHUT_STAT",              1<<3),
        ("CHG_TMR_STAT",            1<<2),
        ("DRV_OKZ_STAT",            1<<1),
    ];
    info!("FAULT_STATUS = 0x{:02X}", fs);
    log_named_bits("FAULT_STATUS bits", fs, MAP);
}

/// Convenience: decode everything commonly read on the INT path.
pub fn log_int_snapshot(f1: u8, f2: u8, ff: u8, s1: u8, s2: u8, s3: u8, fs: u8) {
    log_flag1(f1);
    log_flag2(f2);
    log_fault_flag(ff);
    log_status1(s1);
    log_status2(s2);
    log_status3(s3);
    log_fault_status(fs);
}
