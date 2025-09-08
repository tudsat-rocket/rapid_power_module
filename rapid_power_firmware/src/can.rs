use defmt;
use embassy_futures::select::select;

use embassy_stm32::can::{Can, Frame, Id, StandardId};
use embassy_stm32::can::frame::Header;

use embassy_stm32::peripherals::{CAN1, PB8, PB9};
use embassy_stm32::Peri;

use shared_types::can::{BatteryTelemetryMessage, CanBusMessage, CanBusMessageId};

pub type CanDriver = Can<'static>;

pub fn init(
    can: Peri<'static, CAN1>,
    rx_pin: Peri<'static, PB8>, // PB8 is RX for CAN1
    tx_pin: Peri<'static, PB9>, // PB9 is TX for CAN1
    irqs: crate::Irqs,
) -> CanDriver {
    // NOTE: signature for Can::new is (peri, rx, tx, irqs)
    let mut can = Can::new(can, rx_pin, tx_pin, irqs);

    {
        let mut cfg = can.modify_config();
        cfg.set_loopback(false)
            .set_silent(false)
            .set_automatic_retransmit(true);
    }

    can.set_bitrate(125_000);
    can
}

#[embassy_executor::task]
pub async fn run(mut can: CanDriver) {
    let (mut tx, mut rx) = can.split();

    let mut bms_sub = crate::shared_state::BMS_CHANNEL.subscriber().unwrap();
    let mut chg_sub = crate::shared_state::BQ25756_CHANNEL.subscriber().unwrap();

    loop {
        select(
            // ---- TX ----
            async {
                let bms = bms_sub.next_message_pure().await;

                // Use vbat_mv (not vbus_mv)
                let mut vcharge_mv: u16 = 0;
                if let Some(chg) = chg_sub.try_next_message_pure() {
                    vcharge_mv = chg.vbat_mv as u16;
                }

                let msg = BatteryTelemetryMessage {
                    voltage_battery: bms.voltage_mv as u16,
                    voltage_charge:  vcharge_mv,
                    current:         bms.current_ma as i32,
                    stat0:           false,
                    stat1:           false,
                };

                let (raw_id, data8) = msg.to_frame(CanBusMessageId::BatteryBoardInput(0));

                let sid = StandardId::new(raw_id).unwrap();
                let id: Id = Id::Standard(sid);
                let hdr = Header::new(id, 8, false); // len=u8, rtr=false
                let frame = Frame::new(hdr, &data8).unwrap(); // <- unwrap the Result

                let _ = tx.write(&frame).await;
            },

            // ---- RX ----
            async {
                if let Ok(envelope) = rx.read().await {
                    let id_u32 = match envelope.frame.header().id() {
                        Id::Standard(sid) => sid.as_raw() as u32,
                        Id::Extended(eid) => eid.as_raw(),
                    };

                    let data = envelope.frame.data();
                    let len = data.len();

                    if len == 8 {
                        let mut buf = [0u8; 8];
                        buf.copy_from_slice(data);

                        if let Ok(can_id) = CanBusMessageId::try_from(id_u32 as u16) {
                            if let CanBusMessageId::BatteryBoardInput(_idx) = can_id {
                                match BatteryTelemetryMessage::parse(buf) {
                                    Ok(Some(bt)) => {
                                        defmt::info!(
                                            "CAN rx BatteryTelemetry: vb={}mV vc={}mV i={}mA s0={} s1={}",
                                            bt.voltage_battery,
                                            bt.voltage_charge,
                                            bt.current,
                                            bt.stat0,
                                            bt.stat1
                                        );
                                    }
                                    Ok(None) => defmt::warn!("CAN rx BatteryTelemetry: CRC ok but payload None"),
                                    Err(())  => defmt::warn!("CAN rx BatteryTelemetry: CRC mismatch"),
                                }
                                return;
                            }
                        }
                    }

                    defmt::info!("CAN rx: id=0x{:x}, len={}", id_u32, len);
                }
            },
        )
        .await;
    }
}
