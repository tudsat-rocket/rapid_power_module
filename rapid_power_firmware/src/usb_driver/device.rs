use core::fmt::Write as _;

use embassy_stm32::usb::Driver as OtgDriver;
use embassy_usb::driver::EndpointError;
use embassy_time::{Timer, Duration};
use defmt::Debug2Format;
use heapless::{String, Vec}; // no_std growable string

type UsbDrv = OtgDriver<'static, embassy_stm32::peripherals::USB_OTG_FS>;

#[embassy_executor::task]
pub async fn usb_task(mut dev: embassy_usb::UsbDevice<'static, UsbDrv>) {
    defmt::info!("USB device: run()");
    dev.run().await;
}

pub(crate) async fn write_norm(
    cdc: &mut embassy_usb::class::cdc_acm::CdcAcmClass<'static, UsbDrv>,
    s: &str,
) -> Result<(), EndpointError> {
    // Cap generously; grow if you print larger blobs.
    let mut out: Vec<u8, 512> = Vec::new();
    for &b in s.as_bytes() {
        if b == b'\n' {
            let _ = out.push(b'\r');
            let _ = out.push(b'\n');
        } else {
            let _ = out.push(b);
        }
    }
    write_all(cdc, &out).await
}

// Write in 64B chunks; send ZLP if len % 64 == 0 so transfer terminates cleanly.
async fn write_all(
    cdc: &mut embassy_usb::class::cdc_acm::CdcAcmClass<'static, UsbDrv>,
    buf: &[u8],
) -> Result<(), EndpointError> {
    let mut off = 0;
    while off < buf.len() {
        let end = core::cmp::min(off + 64, buf.len());
        cdc.write_packet(&buf[off..end]).await?;
        off = end;
    }
    if buf.len() % 64 == 0 {
        let _ = cdc.write_packet(&[]).await;
    }
    Ok(())
}

async fn write_str(
    cdc: &mut embassy_usb::class::cdc_acm::CdcAcmClass<'static, UsbDrv>,
    s: &str,
) -> Result<(), EndpointError> {
    write_all(cdc, s.as_bytes()).await
}

async fn prompt(
    cdc: &mut embassy_usb::class::cdc_acm::CdcAcmClass<'static, UsbDrv>,
) -> Result<(), EndpointError> {
    write_str(cdc, "> ").await
}

/* ---------------- Commands ---------------- */

async fn cmd_help(
    cdc: &mut embassy_usb::class::cdc_acm::CdcAcmClass<'static, UsbDrv>,
) -> Result<(), EndpointError> {
    write_norm(cdc,
"Commands:
  help           Show this help
  version        Show firmware version
  echo <text>    Echo text back
  ping           Respond with pong
").await
}

async fn cmd_version(
    cdc: &mut embassy_usb::class::cdc_acm::CdcAcmClass<'static, UsbDrv>,
) -> Result<(), EndpointError> {
    // env! is fine in no_std; it’s baked in at compile time.
    let ver = env!("CARGO_PKG_VERSION");
    let mut s: String<64> = String::new();
    let _ = write!(&mut s, "version {}\r\n", ver);
    write_norm(cdc, &s).await
}

async fn cmd_echo(
    cdc: &mut embassy_usb::class::cdc_acm::CdcAcmClass<'static, UsbDrv>,
    args: &str,
) -> Result<(), EndpointError> {
    let mut s: String<256> = String::new();
    let _ = write!(&mut s, "{}\r\n", args);
    write_norm(cdc, &s).await
}

async fn cmd_ping(
    cdc: &mut embassy_usb::class::cdc_acm::CdcAcmClass<'static, UsbDrv>,
) -> Result<(), EndpointError> {
    write_norm(cdc, "pong\r\n").await
}

/* -------------- Command dispatch -------------- */

async fn dispatch_command(
    cdc: &mut embassy_usb::class::cdc_acm::CdcAcmClass<'static, UsbDrv>,
    line: &str,
) -> Result<(), EndpointError> {
    // Trim \r\n and surrounding spaces
    let line = line.trim();
    if line.is_empty() {
        return Ok(());
    }

    // Split into command + args (first whitespace split)
    let (cmd, args) = match line.split_once(char::is_whitespace) {
        Some((c, rest)) => (c, rest.trim_start()),
        None => (line, ""),
    };

    match cmd {
        "help"    => cmd_help(cdc).await?,
        "version" => cmd_version(cdc).await?,
        "echo"    => cmd_echo(cdc, args).await?,
        "ping"    => cmd_ping(cdc).await?,
        _ => {
            // Let the device-level shell handle feature/IC-specific commands.
            // If it returns Ok, we're done; otherwise print the “unknown” line.
            if let Err(e) = crate::shell::handle_shell(cdc, line).await {
                let mut s: String<64> = String::new();
                let _ = write!(&mut s, "unknown command: {}\r\n", cmd);
                write_str(cdc, &s).await?;
            }
        }

    }
    Ok(())
}

/* -------------- CDC task (line-oriented) -------------- */

#[embassy_executor::task]
pub async fn cdc_task(
    mut cdc: embassy_usb::class::cdc_acm::CdcAcmClass<'static, UsbDrv>,
) {
    loop {
        // Wait until the host configures the interface
        cdc.wait_connection().await;

        // Wait for DTR so we know the terminal is actually “open”
        let mut last_dtr = cdc.dtr();
        defmt::info!("CDC connected (DTR={})", last_dtr);
        while !cdc.dtr() {
            Timer::after(Duration::from_millis(10)).await;
            let d = cdc.dtr();
            if d != last_dtr {
                defmt::info!("CDC DTR -> {}", d);
                last_dtr = d;
            }
        }

        // Greet and show prompt
        let _ = write_norm(&mut cdc, "\nRAPID Power Module ready.\nType 'help' and press Enter.\n").await;
        let _ = prompt(&mut cdc).await;

        // Line buffer and RX buffer
        let mut line: String<256> = String::new();
        let mut rx = [0u8; 64];

        // Main I/O loop
        loop {
            match cdc.read_packet(&mut rx).await {
                Ok(n) if n > 0 => {
                    for &b in &rx[..n] {
                        match b {
                            b'\r' => {
                                // Normalize CR or CRLF to CRLF on echo
                                let _ = write_all(&mut cdc, b"\r\n").await;
                                // Run command
                                let _ = dispatch_command(&mut cdc, &line).await;
                                line.clear();
                                let _ = prompt(&mut cdc).await;
                            }
                            b'\n' => {
                                // If we get lone LF, treat it as end-of-line too
                                let _ = write_all(&mut cdc, b"\r\n").await;
                                let _ = dispatch_command(&mut cdc, &line).await;
                                line.clear();
                                let _ = prompt(&mut cdc).await;
                            }
                            0x08 | 0x7f => {
                                // Optional: backspace handling
                                if !line.is_empty() {
                                    line.pop();
                                    // Erase on terminal: BS, space, BS
                                    let _ = write_all(&mut cdc, b"\x08 \x08").await;
                                }
                            }
                            _ => {
                                // Echo the char, and append if room
                                let _ = cdc.write_packet(&[b]).await;
                                // Don’t let extremely long input DOS the device
                                if line.len() < line.capacity() {
                                    // Only collect printable bytes; drop others
                                    line.push(b as char).ok();
                                }
                            }
                        }
                    }
                }
                Ok(_) => {} // zero-length; ignore
                Err(EndpointError::Disabled) => {
                    defmt::info!("USB CDC disconnected (read)");
                    break;
                }
                Err(e) => {
                    defmt::warn!("USB CDC read error: {}", Debug2Format(&e));
                }
            }
        }
    }
}
