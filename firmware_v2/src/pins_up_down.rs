#![no_std]
#![no_main]

use {defmt_rtt as _, panic_probe as _};
use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use embassy_stm32::time::Hertz;

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    use embassy_stm32::rcc::{Hse, HseMode, Pll, PllMul, PllPreDiv, PllSource, Sysclk, AHBPrescaler, APBPrescaler, ADCPrescaler};
    let mut cfg = embassy_stm32::Config::default();
    cfg.rcc.hse = Some(Hse { freq: Hertz::mhz(8), mode: HseMode::Oscillator });
    cfg.rcc.pll = Some(Pll { src: PllSource::HSE, prediv: PllPreDiv::DIV1, mul: PllMul::MUL9 });
    cfg.rcc.sys = Sysclk::PLL1_P;
    cfg.rcc.ahb_pre  = AHBPrescaler::DIV1;
    cfg.rcc.apb1_pre = APBPrescaler::DIV2;
    cfg.rcc.apb2_pre = APBPrescaler::DIV1;
    cfg.rcc.adc_pre  = ADCPrescaler::DIV6;

    let p = embassy_stm32::init(cfg);

    // --- pick ONE feature at build time ---
    #[cfg(feature = "ts2_hi_z")]
    {
        use embassy_stm32::gpio::Flex;
        let mut ts2 = Flex::new(p.PA3);
        ts2.set_as_analog(); // strongest Hi-Z
        defmt::info!("TS2: Hi-Z (analog input). Measure PA3.");
        loop { Timer::after(Duration::from_secs(1)).await; }
    }

    #[cfg(feature = "ts2_pullup")]
    {
        use embassy_stm32::gpio::{Flex, Pull};
        let mut ts2 = Flex::new(p.PA3);
        ts2.set_as_input(Pull::Up); // ~3.3 V bias
        defmt::info!("TS2: Input + Pull-Up (~3.3 V). Measure PA3.");
        loop { Timer::after(Duration::from_secs(1)).await; }
    }

    #[cfg(feature = "ts2_pulldown")]
    {
        use embassy_stm32::gpio::{OutputOpenDrain, Level, Speed};
        let mut ts2 = OutputOpenDrain::new(p.PA3, Level::Low, Speed::Low); // hard low
        defmt::info!("TS2: Open-drain LOW (0 V). Measure PA3.");
        loop { Timer::after(Duration::from_secs(1)).await; }
    }
}
