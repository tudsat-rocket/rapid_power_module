//! Register addresses and bit masks for the BQ25756.
//! NOTE: The WHOAMI/part-info addresses in the original snippet were placeholders.

pub const I2C_ADDR: u8 = 0x6B;

// 2-byte registers (unless stated otherwise)
pub const REG_CHARGE_VOLTAGE_LIMIT: u8 = 0x00; // VFB_REG
pub const REG_CHARGE_CURRENT_LIMIT: u8 = 0x02; // ICHG_REG
pub const REG_INPUT_CURRENT_DPM_LIMIT: u8 = 0x06; // IAC_DPM
pub const REG_INPUT_VOLTAGE_DPM_LIMIT: u8 = 0x08; // VAC_DPM
pub const REG_REVERSE_MODE_INPUT_CURRENT: u8 = 0x0A; // IAC_REV
pub const REG_REVERSE_MODE_INPUT_VOLTAGE: u8 = 0x0C; // VAC_REV
pub const REG_PRECHARGE_CURRENT_LIMIT: u8 = 0x10; // IPRECHG
pub const REG_TERMINATION_CURRENT_LIMIT: u8 = 0x12; // ITERM

// 1-byte bitfield registers
pub const REG_PRECHARGE_TERMINATION_CTRL: u8 = 0x14;
pub const   EN_TERM: u8 = 1 << 3;
pub const   EN_PRECHG: u8 = 1 << 0;
pub const   VBAT_LOWV_MASK: u8 = 0b0000_0110; // <<1

pub const REG_TIMER_CONTROL: u8 = 0x15;
pub const   TOPOFF_TMR_MASK: u8 = 0b1100_0000; // <<6
pub const   WDT_MASK: u8       = 0b0011_0000; // <<4
pub const   EN_CHG_TMR: u8    = 1 << 3;
pub const   CHG_TMR_MASK: u8  = 0b0000_0110; // <<1
pub const   EN_TMR2X: u8      = 1 << 0;

pub const REG_THREE_STAGE_CHG_CTRL: u8 = 0x16;
// Low nibble (3:0) is CV_TMR

pub const REG_CHARGER_CONTROL: u8 = 0x17;
pub const   VRECHG_MASK: u8   = 0b1100_0000; // <<6
pub const   WD_RST: u8        = 1 << 5;      // self-clearing pulse bit
pub const   EN_CHG_WDT_BEHAV: u8 = 1 << 3;   // EN_CHG bit behavior on WDT
pub const   EN_HIZ: u8        = 1 << 2;
pub const   EN_IBAT_LOAD: u8  = 1 << 1;      // Sinks SRN→GND (danger)
pub const   EN_CHG: u8        = 1 << 0;

pub const REG_PIN_CONTROL: u8 = 0x18;
pub const   EN_ICHG_PIN: u8   = 1 << 7;
pub const   EN_ILIM_HIZ_PIN: u8 = 1 << 6;
pub const   DIS_PG_PIN: u8    = 1 << 5;
pub const   DIS_STAT_PINS: u8 = 1 << 4;
// KEEP AS COMMENT – datasheet cross-check required.
pub const   CE_PIN_FUNC: u8   = 1 << 0; // Possible overlap with FORCE_STAT1 (verify against DS)
pub const   FORCE_STAT4: u8   = 1 << 3;
pub const   FORCE_STAT3: u8   = 1 << 2;
pub const   FORCE_STAT2: u8   = 1 << 1;
pub const   FORCE_STAT1: u8   = 1 << 0;

// WHOAMI placeholders (do not use in task)
pub const REG_PART_INFO: u8 = 0x00;
pub const PART_NUM_MASK: u8 = 0xF8;
