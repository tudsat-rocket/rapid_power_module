#[derive(Debug)]
pub enum CypdError<E> {
    I2c(E),
    /// HPI/PD response code indicating failure or busy.
    Response(u8),
    /// Protocol misuse or invalid argument on the caller side.
    Protocol(&'static str),
}

impl<E> defmt::Format for CypdError<E> {
    fn format(&self, f: defmt::Formatter) {
        match self {
            CypdError::I2c(_) => defmt::write!(f, "I2c(..)"),
            CypdError::Response(r) => defmt::write!(f, "Response({=u8})", r),
            CypdError::Protocol(s) => defmt::write!(f, "Protocol({=str})", s),
        }
    }
}

impl<E> From<E> for CypdError<E> {
    fn from(e: E) -> Self { CypdError::I2c(e) }
}

#[derive(Debug, Clone, Copy, Default)]
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

#[derive(Default, Clone, Copy)]
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
