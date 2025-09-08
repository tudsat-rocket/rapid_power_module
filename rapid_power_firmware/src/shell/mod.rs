use core::fmt::Write;
use heapless::String;
use embassy_time::{Timer, Duration};
use embassy_usb::driver::EndpointError;

use crate::usb_driver::device::write_norm;
use crate::shared_state::*;

type Delay = Timer;

/// Handle device-level shell commands that interact with on-board IC drivers.
/// Commands are feature-gated; if a driver isn't present the command will inform the user.
pub async fn handle_shell(
    cdc: &mut embassy_usb::class::cdc_acm::CdcAcmClass<
        'static,
        embassy_stm32::usb::Driver<'static, embassy_stm32::peripherals::USB_OTG_FS>,
    >,
    line: &str,
) -> Result<(), EndpointError> {
    let line = line.trim();
    if line.is_empty() {
        return Ok(());
    }

    // Split into top-level command and args
    let (cmd, args) = match line.split_once(char::is_whitespace) {
        Some((c, rest)) => (c, rest.trim_start()),
        None => (line, ""),
    };

    match cmd {
        "bq76952" => handle_bq76952(cdc, args).await?,
        "bq25756" => handle_bq25756(cdc, args).await?,
        "cypd"    => handle_cypd(cdc, args).await?,
        _ => {
            let mut s: String<128> = String::new();
            let _ = write!(&mut s, "no shell command: {}\r\n", cmd);
            write_norm(cdc, &s).await?;
        }
    }

    Ok(())
}

#[cfg(feature = "bq76952")]
async fn handle_bq76952(
    cdc: &mut embassy_usb::class::cdc_acm::CdcAcmClass<
        'static,
        embassy_stm32::usb::Driver<'static, embassy_stm32::peripherals::USB_OTG_FS>,
    >,
    args: &str,
) -> Result<(), EndpointError> {
    // Helper: try to fetch one BMS reading, with a short wait loop
    async fn get_one_reading() -> Option<BmsReadings> {
        let mut sub = BMS_CHANNEL.subscriber().unwrap();
        // try a few short waits (device task publishes every ~5s)
        for _ in 0..10 {
            if let Some(m) = sub.try_next_message_pure() {
                return Some(m);
            }
            Timer::after(Duration::from_millis(100)).await;
        }
        None
    }

    match args.split_whitespace().next() {
        Some("vstack") => {
            if let Some(r) = get_one_reading().await {
                let mut s: String<64> = String::new();
                let _ = write!(&mut s, "VSTACK: {} mV\r\n", r.voltage_mv);
                write_norm(cdc, &s).await?;
            } else {
                write_norm(cdc, "BQ76952: no reading yet\r\n").await?;
            }
        }
        Some("fet_toggle") => {
            // Publish a command
            let pubr = BMS_COMMAND_CHANNEL.publisher().unwrap();
            pubr.publish(BmsCommand::ToggleFetEnable).await;
            write_norm(cdc, "BQ76952: ToggleFetEnable queued\r\n").await?;
        }
        Some("set_bal_time") => {
            let mut it = args.split_whitespace().skip(1);
            match (it.next().and_then(|s| s.parse::<u8>().ok()), it.next().and_then(|s| s.parse::<u8>().ok())) {
                (Some(cell), Some(sec)) => {
                    let pubr = BMS_COMMAND_CHANNEL.publisher().unwrap();
                    pubr.publish(BmsCommand::SetBalancingTime(cell, sec)).await;
                    write_norm(cdc, "BQ76952: SetBalancingTime queued\r\n").await?;
                }
                _ => write_norm(cdc, "usage: bq76952 set_bal_time <cell 1-16> <seconds>\r\n").await?,
            }
        }
        Some("help") | None => {
            write_norm(
                cdc,
                "bq76952 commands:\r\n  vstack\r\n  fet_toggle\r\n  set_bal_time <cell> <seconds>\r\n",
            ).await?;
        }
        _ => write_norm(cdc, "unknown bq76952 subcommand\r\n").await?,
    }
    Ok(())
}


#[cfg(not(feature = "bq76952"))]
async fn handle_bq76952(
    cdc: &mut embassy_usb::class::cdc_acm::CdcAcmClass<
        'static,
        embassy_stm32::usb::Driver<'static, embassy_stm32::peripherals::USB_OTG_FS>,
    >,
    _args: &str,
) -> Result<(), EndpointError> {
    write_norm(cdc, "BQ76952 driver not compiled in\r\n").await?;
    Ok(())
}

#[cfg(feature = "bq25756")]
async fn handle_bq25756(
    cdc: &mut embassy_usb::class::cdc_acm::CdcAcmClass<
        'static,
        embassy_stm32::usb::Driver<'static, embassy_stm32::peripherals::USB_OTG_FS>,
    >,
    args: &str,
) -> Result<(), EndpointError> {
    // one-shot subscriber to get snapshot
    async fn get_one() -> Option<Bq25756Readings> {
        let mut sub = BQ25756_CHANNEL.subscriber().unwrap();
        for _ in 0..10 {
            if let Some(m) = sub.try_next_message_pure() {
                return Some(m);
            }
            Timer::after(Duration::from_millis(100)).await;
        }
        None
    }

    match args.split_whitespace().next() {
        Some("vreg") => {
            if args.contains("set") {
                if let Some(mv) = args.split_whitespace().nth(2).and_then(|v| v.parse::<u16>().ok()) {
                    let pubr = BQ25756_COMMAND_CHANNEL.publisher().unwrap();
                    pubr.publish(Bq25756Command::SetChargeVoltageLimit(mv)).await;
                    write_norm(cdc, "BQ25756: VREG set queued\r\n").await?;
                } else {
                    write_norm(cdc, "usage: bq25756 vreg set <mv>\r\n").await?;
                }
            } else if let Some(r) = get_one().await {
                let mut s: String<64> = String::new();
                let _ = write!(&mut s, "VREG={} mV\r\n", r.charge_voltage_limit_mv);
                write_norm(cdc, &s).await?;
            } else {
                write_norm(cdc, "BQ25756: no reading yet\r\n").await?;
            }
        }
        Some("ichg") => {
            if args.contains("set") {
                if let Some(ma) = args.split_whitespace().nth(2).and_then(|v| v.parse::<u16>().ok()) {
                    let pubr = BQ25756_COMMAND_CHANNEL.publisher().unwrap();
                    pubr.publish(Bq25756Command::SetChargeCurrentLimit(ma)).await;
                    write_norm(cdc, "BQ25756: ICHG set queued\r\n").await?;
                } else {
                    write_norm(cdc, "usage: bq25756 ichg set <ma>\r\n").await?;
                }
            } else if let Some(r) = get_one().await {
                let mut s: String<64> = String::new();
                let _ = write!(&mut s, "ICHG={} mA\r\n", r.charge_current_limit_ma);
                write_norm(cdc, &s).await?;
            } else {
                write_norm(cdc, "BQ25756: no reading yet\r\n").await?;
            }
        }
        Some("help") | None => {
            write_norm(cdc, "bq25756 commands:\r\n  vreg [set <mv>]\r\n  ichg [set <ma>]\r\n").await?;
        }
        _ => write_norm(cdc, "unknown bq25756 subcommand\r\n").await?,
    }
    Ok(())
}


#[cfg(not(feature = "bq25756"))]
async fn handle_bq25756(
    cdc: &mut embassy_usb::class::cdc_acm::CdcAcmClass<
        'static,
        embassy_stm32::usb::Driver<'static, embassy_stm32::peripherals::USB_OTG_FS>,
    >,
    _args: &str,
) -> Result<(), EndpointError> {
    write_norm(cdc, "BQ25756 driver not compiled in\r\n").await?;
    Ok(())
}

#[cfg(feature = "cypd")]
#[cfg(feature = "cypd")]
async fn handle_cypd(
    cdc: &mut embassy_usb::class::cdc_acm::CdcAcmClass<
        'static,
        embassy_stm32::usb::Driver<'static, embassy_stm32::peripherals::USB_OTG_FS>,
    >,
    _args: &str,
) -> Result<(), EndpointError> {
    let mut sub = CYPD_CHANNEL.subscriber().unwrap();
    for _ in 0..10 {
        if let Some(r) = sub.try_next_message_pure() {
            let mut s: String<128> = String::new();
            let _ = write!(
                &mut s,
                "VBUS={} mV, connected={}, curr_level={}, contract={}\r\n",
                r.vbus_mv, r.typec_connected, r.typec_current_level, r.pd_contract
            );
            return write_norm(cdc, &s).await;
        }
        Timer::after(Duration::from_millis(100)).await;
    }
    write_norm(cdc, "CYPD: no reading yet\r\n").await
}


#[cfg(not(feature = "cypd"))]
async fn handle_cypd(
    cdc: &mut embassy_usb::class::cdc_acm::CdcAcmClass<
        'static,
        embassy_stm32::usb::Driver<'static, embassy_stm32::peripherals::USB_OTG_FS>,
    >,
    _args: &str,
) -> Result<(), EndpointError> {
    write_norm(cdc, "CYPD driver not compiled in\r\n").await?;
    Ok(())
}

pub fn fmt_bit_cells(mask: u16) -> impl core::fmt::Display {
    struct Cells(u16);
    impl core::fmt::Display for Cells {
        fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
            let mut wrote = false;
            for i in 0..16 {
                if (self.0 & (1 << i)) != 0 {
                    if wrote { write!(f, ",")?; }
                    write!(f, "{}", i + 1)?;
                    wrote = true;
                }
            }
            if !wrote { write!(f, "-")?; }
            Ok(())
        }
    }
    Cells(mask)
}
