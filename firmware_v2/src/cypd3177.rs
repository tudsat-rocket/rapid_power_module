//! CYPD3177 USB-PD (EZ-PD BCR) async driver
//! HPI over I²C (7-bit addr 0x08). LSB-first 16-bit register addresses.

use embedded_hal_async::i2c::I2c;
use defmt::{trace, warn};

#[derive(Debug)]
pub enum CypdError<E> {
    I2c(E),
    /// HPI/PD response code indicating failure or busy.
    Response(u8),
    /// Protocol misuse or invalid argument on the caller side.
    Protocol(&'static str),
}
impl<E> From<E> for CypdError<E> {
    fn from(e: E) -> Self { CypdError::I2c(e) }
}

pub struct Cypd3177<I2C> {
    i2c: I2C,
    addr: u8,
}

impl<I2C> Cypd3177<I2C>
where
    I2C: I2c,
{
    pub fn new(i2c: I2C) -> Self { Self { i2c, addr: 0x08 } }
    pub fn with_address(i2c: I2C, addr: u8) -> Self { Self { i2c, addr } }
    pub fn release(self) -> I2C { self.i2c }

    // ===== Low-level HPI =====

    /// HPI requires LSB, MSB for the 16-bit register address.
    async fn wr(&mut self, reg: u16, data: &[u8]) -> Result<(), CypdError<I2C::Error>> {
        let mut buf = [0u8; 64];
        if data.len() + 2 > buf.len() { return Err(CypdError::Protocol("tx too long")); }
        buf[0] = reg as u8;            // LSB
        buf[1] = (reg >> 8) as u8;     // MSB
        buf[2..2 + data.len()].copy_from_slice(data);

        #[cfg(feature = "cypd-trace")]
        trace!("I2C WR @0x{:04X} <- {:x}", reg, &data);

        let res = self.i2c.write(self.addr, &buf[..2 + data.len()]).await;
        #[cfg(feature = "cypd-trace")]
        match &res {
            Ok(()) => trace!("I2C WR OK @0x{:04X} ({}B)", reg, data.len()),
            Err(_) => warn!("I2C WR ERR @0x{:04X}", reg),
        }
        res.map_err(CypdError::I2c)
    }

    async fn rd(&mut self, reg: u16, dst: &mut [u8]) -> Result<(), CypdError<I2C::Error>> {
        let reg_bytes = [reg as u8, (reg >> 8) as u8]; // LSB, MSB

        #[cfg(feature = "cypd-trace")]
        trace!("I2C RD @0x{:04X} -> {}B", reg, dst.len());

        let res = self.i2c.write_read(self.addr, &reg_bytes, dst).await;

        #[cfg(feature = "cypd-trace")]
        match &res {
            Ok(()) => trace!("I2C RD OK @0x{:04X}: {:x}", reg, &dst),
            Err(_) => warn!("I2C RD ERR @0x{:04X}", reg),
        }
        res.map_err(CypdError::I2c)
    }


    async fn rd_u8(&mut self, reg: u16) -> Result<u8, CypdError<I2C::Error>> {
        let mut b = [0u8; 1];
        self.rd(reg, &mut b).await?;
        Ok(b[0])
    }
    async fn rd_u16(&mut self, reg: u16) -> Result<u16, CypdError<I2C::Error>> {
        let mut b = [0u8; 2];
        self.rd(reg, &mut b).await?;
        Ok(u16::from_le_bytes(b))
    }
    async fn rd_u32(&mut self, reg: u16) -> Result<u32, CypdError<I2C::Error>> {
        let mut b = [0u8; 4];
        self.rd(reg, &mut b).await?;
        Ok(u32::from_le_bytes(b))
    }

    // ===== Typed registers (per HPI docs) =====

    /// DEVICE_MODE (0x0000) — BCR returns a fixed ID byte (doc shows 0x92).
    /// Treat it as informational; don't gate logic on a specific literal.  :contentReference[oaicite:1]{index=1}
    pub async fn device_mode(&mut self) -> Result<u8, CypdError<I2C::Error>> {
        self.rd_u8(0x0000).await
    }

    pub async fn silicon_id_raw(&mut self) -> Result<[u8; 2], CypdError<I2C::Error>> {
        let mut b = [0u8; 2];
        self.rd(0x0002, &mut b).await?;
        Ok(b)
    }

    /// SILICON_ID (0x0002) — BCR doc shows 0x11B0. Just log for info.  :contentReference[oaicite:2]{index=2}
    pub async fn silicon_id(&mut self) -> Result<u16, CypdError<I2C::Error>> {
        let raw = self.silicon_id_raw().await?;
        Ok(u16::from_be_bytes(raw))
    }

    /// BUS_VOLTAGE (0x100D): value * 100 = mV.
    pub async fn bus_voltage_mv(&mut self) -> Result<u16, CypdError<I2C::Error>> {
        let v = self.rd_u8(0x100D).await?;
        Ok((v as u16) * 100)
    }

    /// TYPE_C_STATUS (0x100C)
    pub async fn typec_status(&mut self) -> Result<TypeCStatus, CypdError<I2C::Error>> {
        let raw = self.rd_u32(0x100C).await?;
        Ok(TypeCStatus::from(raw))
    }

    /// PD_STATUS (0x1008)
    pub async fn pd_status(&mut self) -> Result<PdStatus, CypdError<I2C::Error>> {
        let raw = self.rd_u32(0x1008).await?;
        Ok(PdStatus::from(raw))
    }

    /// DEV_RESPONSE (0x007E) — first byte is response code
    pub async fn dev_response_code(&mut self) -> Result<u8, CypdError<I2C::Error>> {
        self.rd_u8(0x007E).await
    }

    /// PD_RESPONSE (0x1400) — first byte is response code (e.g. 0x02 = success)
    pub async fn pd_response(&mut self) -> Result<PdResponse, CypdError<I2C::Error>> {
        let mut hdr = [0u8; 4];
        self.rd(0x1400, &mut hdr).await?;
        Ok(PdResponse {
            code:   hdr[0],                          // 0x02=success, 0x01=fail, 0x03=busy, bit7 async-event
            len_lo: hdr[1],
            len_hi: u16::from_le_bytes([hdr[2], hdr[3]]),
        })
    }

    pub async fn interrupt_status(&mut self) -> Result<u8, CypdError<I2C::Error>> {
        // INTERRUPT register @ 0x0006
        self.rd_u8(0x0006).await
    }

    /// EVENT_STATUS (0x1044) — sticky event bits (datasheet-defined)
    pub async fn event_status(&mut self) -> Result<u32, CypdError<I2C::Error>> {
        self.rd_u32(0x1044).await
    }


    #[cfg(feature = "cypd-write")]
    pub async fn clear_event_status(&mut self, mask: u32) -> Result<(), CypdError<I2C::Error>> {
        // write-1-to-clear: little-endian 32-bit
        self.wr(0x1044, &mask.to_le_bytes()).await
    }

    #[cfg(feature = "cypd-write")]
    pub async fn clear_interrupts(&mut self, mask: u8) -> Result<(), CypdError<I2C::Error>> {
        // INTERRUPT (0x0006) W1C
        self.wr(0x0006, &[mask]).await
    }

    /// CURRENT_PDO (0x1010) — raw 32-bit PDO the port partner selected for CYPD
    pub async fn current_pdo_raw(&mut self) -> Result<u32, CypdError<I2C::Error>> {
        self.rd_u32(0x1010).await
    }

    /// CURRENT_RDO (0x1014) — raw 32-bit request data object
    pub async fn current_rdo_raw(&mut self) -> Result<u32, CypdError<I2C::Error>> {
        self.rd_u32(0x1014).await
    }

    /// Read PD_RESPONSE header (0x1400..0x1403)
    pub async fn pd_response_header(&mut self) -> Result<[u8; 4], CypdError<I2C::Error>> {
        let mut hdr = [0u8; 4];
        self.rd(0x1400, &mut hdr).await?;
        Ok(hdr)
    }

    /// Read up to `buf.len()` bytes of PD_RESPONSE payload starting at 0x1404
    pub async fn pd_response_payload(&mut self, buf: &mut [u8]) -> Result<(), CypdError<I2C::Error>> {
        if buf.is_empty() { return Ok(()); }
        self.rd(0x1404, buf).await
    }


    /// Set a minimal, useful event mask: attach/detach/contract (bits 3,4,5) 0x38. 0x1024 is LE 32-bit.  :contentReference[oaicite:4]{index=4}
    #[cfg(feature = "cypd-write")]
    pub async fn enable_basic_events(&mut self) -> Result<(), CypdError<I2C::Error>> {
        // EVENT_MASK @ 0x1024, 4 bytes, little-endian
        self.wr(0x1024, &[0x38, 0x00, 0x00, 0x00]).await
    }

    /// Clear bits in INTERRUPT (0x0006, W1C) after servicing queues.  :contentReference[oaicite:5]{index=5}
    #[cfg(feature = "cypd-write")]
    pub async fn clear_interrupt_bits(&mut self, mask: u8) -> Result<(), CypdError<I2C::Error>> {
        self.wr(0x0006, &[mask]).await
    }

    #[cfg(feature = "cypd-write")]
    pub async fn reset_device(&mut self) -> Result<(), CypdError<I2C::Error>> {
        self.wr(0x0008, &[0x52, 0x01]).await // 'R', device reset
    }

    #[cfg(feature = "cypd-write")]
    pub async fn write_sink_pdos(&mut self, pdos: &[u32]) -> Result<(), CypdError<I2C::Error>> {
        if pdos.len() > 7 { return Err(CypdError::Protocol("up to 7 PDOs")); }

        // Wire order must be P K N S (0x50, 0x4B, 0x4E, 0x53)
        let mut buf = [0u8; 4 + 7 * 4];
        buf[0..4].copy_from_slice(&[0x50, 0x4B, 0x4E, 0x53]);

        for (i, p) in pdos.iter().enumerate() {
            buf[4 + i*4 .. 8 + i*4].copy_from_slice(&p.to_le_bytes());
        }
        self.wr(0x1800, &buf).await
    }

    #[cfg(feature = "cypd-write")]
    pub async fn select_sink_pdo_mask(&mut self, mask: u8) -> Result<(), CypdError<I2C::Error>> {
        if mask == 0 { return Err(CypdError::Protocol("mask must be non-zero")); }
        self.wr(0x1005, &[mask]).await
    }
}

// ===== Status types =====

#[derive(Debug, Clone, Copy)]
pub struct TypeCStatus {
    pub connected: bool,
    pub cc_polarity_cc2: bool,
    pub attached_dev: u8,   // 0..7 per HPI
    pub current_level: u8,  // 0=900mA,1=1.5A,2=3A
}
impl From<u32> for TypeCStatus {
    fn from(v: u32) -> Self {
        let b0 = (v & 0xFF) as u8;
        Self {
            connected: (b0 & 0x01) != 0,
            cc_polarity_cc2: (b0 & 0x02) != 0,
            attached_dev: (b0 >> 2) & 0x07,
            current_level: (b0 >> 6) & 0x03,
        }
    }
}

#[derive(Debug, Clone, Copy)]
pub struct PdStatus {
    pub contract: bool,     // explicit contract exists
    pub sink_tx_ok: bool,   // true => SinkTxOk
    pub pe_snk_ready: bool,
}
impl From<u32> for PdStatus {
    fn from(v: u32) -> Self {
        Self {
            contract:     ((v >> 10) & 1) != 0,  // 0x0400
            sink_tx_ok:   ((v >> 14) & 1) != 0,  // 0x4000
            pe_snk_ready: ((v >> 15) & 1) != 0,  // 0x8000
        }
    }
}

#[derive(Debug, Clone, Copy)]
pub struct PdResponse {
    /// 0x02=success, 0x01=fail, 0x03=busy; bit7=1 => async event
    pub code: u8,
    /// if len_hi==0, total length = len_lo; otherwise = len_hi (little-endian 16-bit)
    pub len_lo: u8,
    pub len_hi: u16,
}