//! Public enums and driver error type.

use defmt;

#[derive(Copy, Clone, Debug, Eq, PartialEq, defmt::Format)]
pub enum Error<E> {
    I2c(E),
    /// WHOAMI (part number) mismatch.
    InvalidDevice(u8),
    /// Parameter outside allowed range.
    InvalidInput,
}

#[repr(u8)]
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum VbatLowV {
    Pct30   = 0,
    Pct55   = 1,
    Pct66_7 = 2,
    Pct71_4 = 3,
}

#[repr(u8)]
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum Vrechg {
    Pct93   = 0,
    Pct94_3 = 1,
    Pct95_2 = 2,
    Pct97_6 = 3,
}

#[repr(u8)]
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum TopOffTimer { Disable=0, Min15=1, Min30=2, Min45=3 }

#[repr(u8)]
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum WdtTimer { Disable=0, S40=1, S80=2, S160=3 }

#[repr(u8)]
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum ChgTmr { H5=0, H8=1, H12=2, H24=3 }

#[repr(u8)]
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum PAndOTimer { Disable=0, Ms500=1, S1=2, S10=3 }

#[repr(u8)]
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum FullSweepTimer { Min3=0, Min10=1, Min15=2, Min20=3 }
