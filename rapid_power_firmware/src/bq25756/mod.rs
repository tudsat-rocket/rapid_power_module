#![allow(dead_code)]

pub mod regs;
pub mod types;
pub mod device;

pub use device::{Bq25756, bq25756_task, bq25756_int_task, bq25756_pg_task, bq25756_stat1_task, bq25756_stat2_task};
