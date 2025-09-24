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

// --- Identity
pub const REG_PART_INFO: u8         = 0x3D;
pub const   PART_NUM_MASK: u8       = 0b0111_1000;

// --- ADC control & channels
pub const REG_ADC_CONTROL: u8       = 0x2B;
pub const   ADC_EN: u8              = 1 << 7;       // 1=enable
pub const   ADC_RATE: u8            = 1 << 6;       // 0=continuous, 1=one-shot
pub const   ADC_SAMPLE_MASK: u8     = 0b0011_0000;  // <<4
pub const   ADC_AVG: u8             = 1 << 3;
pub const   ADC_AVG_INIT: u8        = 1 << 2;

pub const REG_ADC_CHANNEL_CTRL: u8  = 0x2C;         // 1 = disable that channel
pub const   IAC_ADC_DIS: u8         = 1 << 7;
pub const   IBAT_ADC_DIS: u8        = 1 << 6;
pub const   VAC_ADC_DIS: u8         = 1 << 5;
pub const   VBAT_ADC_DIS: u8        = 1 << 4;
pub const   TS_ADC_DIS: u8          = 1 << 2;
pub const   VFB_ADC_DIS: u8         = 1 << 1;

// --- ADC result registers (u16, LE)
pub const REG_IAC_ADC: u8           = 0x2D;
pub const REG_IBAT_ADC: u8          = 0x2F;
pub const REG_VAC_ADC: u8           = 0x31;
pub const REG_VBAT_ADC: u8          = 0x33;

// --- Status / mask (optional reads)
pub const REG_CHARGER_STATUS_1: u8  = 0x21;
pub const REG_CHARGER_STATUS_2: u8  = 0x22;
pub const REG_CHARGER_STATUS_3: u8  = 0x23;
pub const REG_FAULT_STATUS: u8      = 0x24;
pub const REG_CHARGER_FLAG_1: u8    = 0x25;
pub const REG_CHARGER_FLAG_2: u8    = 0x26;
pub const REG_FAULT_FLAG: u8        = 0x27;
pub const REG_CHARGER_MASK_1: u8    = 0x28;
pub const REG_CHARGER_MASK_2: u8    = 0x29;
pub const REG_FAULT_MASK: u8        = 0x2A;

// CHARGER_STATUS_1 bits
pub const CHARGE_STAT_MASK: u8 = 0b0000_0111; // [2:0]

// --- TS / JEITA control
pub const REG_TS_THRESHOLD_CTRL: u8 = 0x1B; // REG0x1B
pub const   TS_T5_MASK: u8 = 0b1100_0000; // <<6  (50/55/60/65 °C)
pub const   TS_T3_MASK: u8 = 0b0011_0000; // <<4  (40/45/50/55 °C)
pub const   TS_T2_MASK: u8 = 0b0000_1100; // <<2  (5/10/15/20 °C)
pub const   TS_T1_MASK: u8 = 0b0000_0011; // <<0  (-10/-5/0/5 °C)

pub const REG_TS_BEHAVIOR_CTRL: u8 = 0x1C; // REG0x1C
pub const   JEITA_VSET_MASK: u8 = 0b0110_0000; // <<5  (Suspend/94.3%/97.6%/100%)
pub const   JEITA_ISETH: u8   = 1 << 4;        // 0=40% ICHG, 1=100% ICHG
pub const   JEITA_ISETC_MASK: u8 = 0b0000_1100; // <<2 (Suspend/20%/40%/100%)
pub const   EN_JEITA: u8      = 1 << 1;        // 0=disabled, 1=enabled
pub const   EN_TS: u8         = 1 << 0;        // 0=ignore TS, 1=monitor TS

