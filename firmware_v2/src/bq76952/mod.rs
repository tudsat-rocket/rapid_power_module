pub mod bq76952;
pub mod regs;
pub mod types;

pub use bq76952::Bq76952;
pub use types::*;

pub trait Ts2PinControl {
    /// Put TS2 in a state that reads as logical "high" to the BQ (e.g., input + pull-up).
    fn prebias_high(&mut self);
    /// Drive TS2 low (e.g., output low / open-drain low).
    fn drive_low(&mut self);
    /// Release TS2 so it goes Hi-Z again (e.g., analog input or input no-pull).
    fn release(&mut self);
    /// Briefly bias TS2 low as input-pulldown (to guarantee a falling edge), then driver will Hi-Z again.
    fn bias_pulldown(&mut self);
}
