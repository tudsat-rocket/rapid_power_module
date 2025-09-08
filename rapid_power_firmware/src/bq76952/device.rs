use embedded_hal_async::delay::DelayNs;
use embedded_hal_async::i2c::I2c;
use embassy_stm32::gpio::{Flex, Pull, Speed};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_time::{Duration, Timer};
use defmt;

use super::regs::{cmd, datamem, subcmd};
use super::types::*;
use crate::I2cDev;
use crate::shared_state::{BmsCommand, BmsReadings};
use crate::shell::fmt_bit_cells;
use crate::config;

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

impl Ts2PinControl for Flex<'static> {
    fn prebias_high(&mut self) {
        self.set_as_input(Pull::Up);
    }
    fn drive_low(&mut self) {
        self.set_as_output(Speed::Low);
        self.set_low();
    }
    fn release(&mut self) {
        self.set_as_analog();
    }
    fn bias_pulldown(&mut self) {
        self.set_as_input(Pull::Down);
    }
}

/// An async driver for the TI **BQ76952** battery monitor and protector.
pub struct Bq76952<I2C> {
    i2c: I2C,
    addr: u8, // 7-bit I²C address
    rsense_milliohms: u32,
    active_cells_mask: u16, // bitmask of cells physically present (1..=16)
}

impl<I2C, E> Bq76952<I2C>
where
    I2C: I2c<Error = E>,
    E: core::fmt::Debug,
{
    pub fn new(i2c: I2C, addr: u8, rsense_milliohms: u32) -> Self {
        Self { i2c, addr, rsense_milliohms, active_cells_mask: 0}
    }

    fn set_active_cells_mask(&mut self, m: u16) { self.active_cells_mask = m; }
    fn get_active_cells_mask(&self) -> u16 { self.active_cells_mask }

    pub async fn get_stack_voltage(&mut self) -> Result<u16, Error<E>> {
        self.read_direct_u16(cmd::VSTACK).await
    }

    pub async fn get_current_cc2(&mut self) -> Result<i16, Error<E>> {
        self.read_direct_i16(cmd::CC2_CUR).await
    }

    pub async fn get_internal_temp_c(&mut self) -> Result<f32, Error<E>> {
        let temp_0_1k = self.read_direct_u16(cmd::INT_TEMP).await?;
        Ok(Self::temp_0_1k_to_c(temp_0_1k))
    }

    pub async fn get_thermistor_temp_c(&mut self, thermistor: Thermistor) -> Result<f32, Error<E>> {
        let reg = match thermistor {
            Thermistor::TS1 => cmd::TS1,
            Thermistor::TS2 => cmd::TS2,
            Thermistor::TS3 => cmd::TS3,
        };
        let temp_0_1k = self.read_direct_u16(reg).await?;
        Ok(Self::temp_0_1k_to_c(temp_0_1k))
    }



    pub async fn get_cell_voltage(&mut self, cell_number: u8) -> Result<u16, Error<E>> {
        if !(1..=16).contains(&cell_number) {
            return Err(Error::InvalidParameter);
        }
        let reg = cmd::VCELL1 + (cell_number - 1) * 2;
        self.read_direct_u16(reg).await
    }

    pub async fn get_all_cell_voltages(&mut self) -> Result<[u16; 16], Error<E>> {
        let mut voltages = [0u16; 16];
        for i in 0..16 {
            voltages[i] = self.get_cell_voltage((i + 1) as u8).await?;
        }
        Ok(voltages)
    }

    pub async fn log_all_cell_voltages(&mut self) {
        match self.get_all_cell_voltages().await {
            Ok(v) => {
                let mask = self.get_active_cells_mask();

                let mut populated: usize = 0;
                let mut sum_cells_mv: u32 = 0;

                for i in 0..16 {
                    let bit = 1u16 << i;
                    if mask == 0 || (mask & bit) != 0 {
                        let mv = v[i];
                        let cell_idx: u8 = (i + 1) as u8;
                        defmt::info!("VCELL{=u8}: {=u16} mV", cell_idx, mv);
                        if mv > 0 { populated += 1; }
                        sum_cells_mv = sum_cells_mv.saturating_add(mv as u32);
                    }
                }

                defmt::info!(
                    "Cells populated (non-zero among configured): {=usize}, sum(cells)={=u32} mV",
                    populated, sum_cells_mv
                );

                if let Ok(vstack) = self.get_stack_voltage_mv().await {
                    defmt::info!(
                        "VSTACK={=u16} mV, delta(sum(cells)-VSTACK)={=i32} mV",
                        vstack, sum_cells_mv as i32 - vstack as i32
                    );
                }
            }
            Err(e) => defmt::warn!("Cell voltages read failed: {:?}", defmt::Debug2Format(&e)),
        }
    }

    async fn read_direct_u16(&mut self, command: u8) -> Result<u16, Error<E>> {
        let mut buffer = [0u8; 2];
        self.i2c.write_read(self.addr, &[command], &mut buffer).await.map_err(Error::I2c)?;
        defmt::debug!("Read direct u16 from command {=u8}: {=u16}", command, u16::from_le_bytes(buffer));
        Ok(u16::from_le_bytes(buffer))
    }

    async fn read_direct_i16(&mut self, command: u8) -> Result<i16, Error<E>> {
        let mut buffer = [0u8; 2];
        self.i2c.write_read(self.addr, &[command], &mut buffer).await.map_err(Error::I2c)?;
        defmt::debug!("Read direct i16 from command {=u8}: {=i16}", command, i16::from_le_bytes(buffer));
        Ok(i16::from_le_bytes(buffer))
    }

    fn le_u16(b: &[u8]) -> u16 { u16::from_le_bytes([b[0], b[1]]) }
    fn le_i16(b: &[u8]) -> i16 { i16::from_le_bytes([b[0], b[1]]) }
    fn le_u32(b: &[u8]) -> u32 { u32::from_le_bytes([b[0], b[1], b[2], b[3]]) }
    fn le_i32(b: &[u8]) -> i32 { i32::from_le_bytes([b[0], b[1], b[2], b[3]]) }

    // Single place for K(0.1) -> °C conversion used by all temp reads.
    #[inline]
    fn temp_0_1k_to_c(tenths_kelvin: u16) -> f32 {
        (tenths_kelvin as f32) * 0.1 - 273.15
    }

    async fn sub_command(&mut self, command: u16) -> Result<(), Error<E>> {
        let [lo, hi] = command.to_le_bytes();
        self.i2c.write(self.addr, &[cmd::SUBCMD_LOW, lo]).await.map_err(Error::I2c)?;
        self.i2c.write(self.addr, &[cmd::SUBCMD_HI, hi]).await.map_err(Error::I2c)?;
        Ok(())
    }

    pub async fn set_balancing_time<D: DelayNs>(
        &mut self,
        cell_number: u8,
        time_seconds: u8,
        delay: &mut D,
    ) -> Result<(), Error<E>> {
        if !(1..=16).contains(&cell_number) {
            return Err(Error::InvalidParameter);
        }
        let address = datamem::CELL_BALANCING_TIME_BASE + (cell_number - 1) as u16;
        self.write_data_memory_u8(address, time_seconds, delay).await
    }

    // Private helper: write a Data Memory block (0x3E/0x3F + 0x40.. + checksum/len).
    async fn write_df(&mut self, df_addr: u16, payload: &[u8]) -> Result<(), Error<E>> {
        assert!(!payload.is_empty() && payload.len() <= 32);

        let a = df_addr.to_le_bytes();

        // Set DF address at 0x3E/0x3F
        self.i2c.write(self.addr, &[cmd::SUBCMD_LOW, a[0]]).await.map_err(Error::I2c)?;
        self.i2c.write(self.addr, &[cmd::SUBCMD_HI,  a[1]]).await.map_err(Error::I2c)?;

        // Copy payload to 0x40.. (RESP_START)
        let mut buf = [0u8; 1 + 32];
        buf[0] = cmd::RESP_START;
        buf[1..1 + payload.len()].copy_from_slice(payload);
        self.i2c.write(self.addr, &buf[..1 + payload.len()]).await.map_err(Error::I2c)?;

        // Compute checksum (one’s complement of sum of {addr_lo, addr_hi, payload...})
        let mut sum = a[0].wrapping_add(a[1]);
        for &b in payload { sum = sum.wrapping_add(b); }
        let csum = 0xFFu8.wrapping_sub(sum);
        let len  = (payload.len() as u8).saturating_add(4);

        // Write checksum (0x60) and length (0x61 = payload_len + 4)
        self.i2c.write(self.addr, &[cmd::RESP_CHKSUM, csum]).await.map_err(Error::I2c)?;
        self.i2c.write(self.addr, &[cmd::RESP_LEN,    len]).await.map_err(Error::I2c)?;
        Ok(())
    }

    // Helper: map BatteryStatus.security_state (2 bits) to enum
    async fn get_security_state(&mut self) -> Result<SecurityState, Error<E>> {
        let s = self.get_battery_status().await?;
        Ok(match s.security_state {
            0 => SecurityState::NotInit,
            1 => SecurityState::FullAccess,
            2 => SecurityState::Unsealed,
            3 => SecurityState::Sealed,
            x => SecurityState::Unknown(x),
        })
    }

    pub async fn write_data_memory_u8<D: DelayNs>(
        &mut self,
        address: u16,
        data: u8,
        delay: &mut D,
    ) -> Result<(), Error<E>> {
        // Enter CONFIG_UPDATE (0x0090)
        self.sub_command(subcmd::CONFIG_UPDATE).await?;
        delay.delay_ms(2u32).await;

        // Do the write
        self.write_df(address, &[data]).await?;

        // Exit CONFIG_UPDATE (0x0092)
        self.sub_command(subcmd::EXIT_CONFIG_UPDATE).await
    }

    pub async fn toggle_fet_enable(&mut self) -> Result<(), Error<E>> {
        self.sub_command(subcmd::FET_ENABLE).await
    }

    pub async fn set_cell_overvoltage_protection<D: DelayNs>(
        &mut self,
        voltage_mv: u16,
        delay_ms: u16,
        delay: &mut D,
    ) -> Result<(), Error<E>> {
        let threshold = ((voltage_mv - 2000) as f32 / 50.6) as u8;
        let delay_val = (delay_ms as f32 / 3.3) as u8;
        self.write_data_memory_u8(datamem::COV_THRESHOLD, threshold, delay).await?;
        self.write_data_memory_u8(datamem::COV_DELAY, delay_val, delay).await
    }

    pub async fn set_cell_undervoltage_protection<D: DelayNs>(
        &mut self,
        voltage_mv: u16,
        delay_ms: u16,
        delay: &mut D,
    ) -> Result<(), Error<E>> {
        let threshold = ((voltage_mv - 500) as f32 / 50.6) as u8;
        let delay_val = (delay_ms as f32 / 3.3) as u8;
        self.write_data_memory_u8(datamem::CUV_THRESHOLD, threshold, delay).await?;
        self.write_data_memory_u8(datamem::CUV_DELAY, delay_val, delay).await
    }

    // Read N bytes from Data Memory starting at df_addr (0x3E/0x3F then 0x40..).
    async fn read_df<const N: usize>(&mut self, df_addr: u16) -> Result<[u8; N], Error<E>> {
        let [lo, hi] = df_addr.to_le_bytes();
        self.i2c.write(self.addr, &[cmd::SUBCMD_LOW, lo]).await.map_err(Error::I2c)?;
        self.i2c.write(self.addr, &[cmd::SUBCMD_HI,  hi]).await.map_err(Error::I2c)?;
        let mut out = [0u8; N];
        for i in 0..N {
            let mut b = [0u8; 1];
            self.i2c.write_read(self.addr, &[cmd::RESP_START + (i as u8)], &mut b).await.map_err(Error::I2c)?;
            out[i] = b[0];
        }
        Ok(out)
    }

    /// Issue a subcommand and read its response buffer from 0x40.. .
    async fn read_subcmd_response<const N: usize>(&mut self, sub: u16, delay_ms_after: u32)
        -> Result<(usize, [u8; N]), Error<E>>
    {
        // Write LOW then HIGH to 0x3E/0x3F
        let [lo, hi] = sub.to_le_bytes();
        self.i2c.write(self.addr, &[cmd::SUBCMD_LOW, lo]).await.map_err(Error::I2c)?;
        self.i2c.write(self.addr, &[cmd::SUBCMD_HI,  hi]).await.map_err(Error::I2c)?;

        // Wait a tad for the device to fill the response window
        embassy_time::Timer::after(Duration::from_millis(delay_ms_after as u64)).await;

        // Read length from 0x61
        let mut len_b = [0u8; 1];
        self.i2c.write_read(self.addr, &[cmd::RESP_LEN], &mut len_b).await.map_err(Error::I2c)?;
        let resp_len = len_b[0] as usize;

        // Clamp to our static buffer
        let to_read = core::cmp::min(resp_len, N);

        // Read the payload from 0x40.. (RESP_START)
        let mut buf = [0u8; N];
        // We read exactly `to_read` bytes; many subcmds return up to 32 bytes.
        // Device auto-increments the internal pointer when you keep reading the same "register".
        for i in 0..to_read {
            let mut b = [0u8; 1];
            let reg = cmd::RESP_START + (i as u8);
            self.i2c.write_read(self.addr, &[reg], &mut b).await.map_err(Error::I2c)?;
            buf[i] = b[0];
        }

        Ok((to_read, buf))
    }

    pub async fn read_dastatus5(&mut self) -> Result<[u8; 32], Error<E>> {
        let (_n, buf) = self.read_subcmd_response::<32>(subcmd::DASTATUS5, 2).await?;
        Ok(buf)
    }

        pub fn decode_dastatus5(&self, b: &[u8; 32]) -> super::types::Dastatus5Decoded {
        use super::types::Dastatus5Decoded;
        let vreg18_counts      = Self::le_u16(&b[0..2]);
        let vss_counts         = Self::le_u16(&b[2..4]);
        let max_cell_mv        = Self::le_u16(&b[4..6]);
        let min_cell_mv        = Self::le_u16(&b[6..8]);
        // Pack voltage is typically centivolts (10 mV). Convert to mV.
        let pack_mv            = (Self::le_u16(&b[8..10]) as u32) * 10;
        let avg_cell_temp_c    = Self::temp_0_1k_to_c(Self::le_u16(&b[10..12]));
        let fet_temp_c         = Self::temp_0_1k_to_c(Self::le_u16(&b[12..14]));
        let max_cell_temp_c    = Self::temp_0_1k_to_c(Self::le_u16(&b[14..16]));
        let min_cell_temp_c    = Self::temp_0_1k_to_c(Self::le_u16(&b[16..18]));
        let cc3_usera          = Self::le_i16(&b[20..22]);
        let cc1_usera          = Self::le_i16(&b[22..24]);
        let cc2_counts         = Self::le_i32(&b[24..28]);
        let cc3_counts         = Self::le_i32(&b[28..32]);

        Dastatus5Decoded {
            vreg18_counts, vss_counts, max_cell_mv, min_cell_mv, pack_mv,
            avg_cell_temp_c, fet_temp_c, max_cell_temp_c, min_cell_temp_c,
            cc3_usera, cc1_usera, cc2_counts, cc3_counts,
        }
    }

    pub async fn read_dastatus6(&mut self) -> Result<[u8; 32], Error<E>> {
        let (_n, buf) = self.read_subcmd_response::<32>(subcmd::DASTATUS6, 2).await?;
        Ok(buf)
    }

        pub fn decode_dastatus6(&self, b: &[u8; 32]) -> super::types::Dastatus6Decoded {
        use super::types::Dastatus6Decoded;
        let q_hi     = Self::le_u32(&b[0..4]);
        let q_lo     = Self::le_u32(&b[4..8]);
        let t_s      = Self::le_u32(&b[8..12]);
        let cfetoff  = Self::le_i32(&b[12..16]);
        let dfetoff  = Self::le_i32(&b[16..20]);
        let alert    = Self::le_i32(&b[20..24]);
        let ts1      = Self::le_i32(&b[24..28]);
        let ts2      = Self::le_i32(&b[28..32]);

        Dastatus6Decoded {
            q_accum_userah_hi: q_hi,
            q_accum_userah_lo: q_lo,
            time_accum_s: t_s,
            cfetoff_counts: cfetoff,
            dfetoff_counts: dfetoff,
            alert_counts: alert,
            ts1_counts: ts1,
            ts2_counts: ts2,
        }
    }

    pub async fn read_dastatus7(&mut self) -> Result<[u8; 32], Error<E>> {
        let (_n, buf) = self.read_subcmd_response::<32>(subcmd::DASTATUS7, 2).await?;
        Ok(buf)
    }


        pub fn decode_dastatus7(&self, b: &[u8; 32]) -> super::types::Dastatus7Decoded {
        use super::types::Dastatus7Decoded;
        let ts3  = Self::le_i32(&b[0..4]);
        let hdq  = Self::le_i32(&b[4..8]);
        let dchg = Self::le_i32(&b[8..12]);
        let ddsg = Self::le_i32(&b[12..16]);

        Dastatus7Decoded { ts3_counts: ts3, hdq_counts: hdq, dchg_counts: dchg, ddsg_counts: ddsg }
    }

    pub async fn get_stack_voltage_mv(&mut self) -> Result<u16, Error<E>> {
        // VSTACK at 0x34, 10 mV units -> mV
        let raw = self.read_direct_u16(cmd::VSTACK).await?;
        Ok(raw.saturating_mul(10))
    }

    pub async fn get_current_cc2_raw(&mut self) -> Result<i16, Error<E>> {
        // CC2 at 0x3A, signed two’s complement
        self.read_direct_i16(cmd::CC2_CUR).await
    }

    pub async fn get_battery_status(&mut self) -> Result<BatteryStatus, Error<E>> {
        defmt::debug!("Reading Battery Status");
        let status = self.read_direct_u16(cmd::BATTERY_STATUS).await?;
        defmt::debug!("Battery Status: {=u16}", status);
        Ok(BatteryStatus {
            sleep_mode: (status & (1 << 15)) != 0,
            shutdown_pending: (status & (1 << 13)) != 0,
            permanent_fault: (status & (1 << 12)) != 0,
            safety_fault: (status & (1 << 11)) != 0,
            fuse_pin: (status & (1 << 10)) != 0,
            security_state: ((status >> 8) & 0b11) as u8,
            wr_to_otp_blocked: (status & (1 << 7)) != 0,
            wr_to_otp_pending: (status & (1 << 6)) != 0,
            open_wire_check: (status & (1 << 5)) != 0,
            wd_was_triggered: (status & (1 << 4)) != 0,
            full_reset_occured: (status & (1 << 3)) != 0,
            sleep_en_allowed: (status & (1 << 2)) != 0,
            precharge_mode: (status & (1 << 1)) != 0,
            config_update_mode: (status & (1 << 0)) != 0,
        })
    }

    pub async fn get_alarm_status(&mut self) -> Result<AlarmStatus, Error<E>> {
        let status = self.read_direct_u16(cmd::ALARM_STATUS).await?;
        Ok(AlarmStatus {
            cov: (status & (1 << 11)) != 0,
            cuv: (status & (1 << 10)) != 0,
            occ: (status & (1 << 9)) != 0,
            ocd1: (status & (1 << 8)) != 0,
            ocd2: (status & (1 << 7)) != 0,
            scd: (status & (1 << 6)) != 0,
            otf: (status & (1 << 5)) != 0,
            otd: (status & (1 << 4)) != 0,
            otc: (status & (1 << 3)) != 0,
            utf: (status & (1 << 2)) != 0,
            utd: (status & (1 << 1)) != 0,
            utc: (status & (1 << 0)) != 0,
        })
    }

    pub async fn get_cb_active_mask(&mut self) -> Result<u16, Error<E>> {
        // Read 2-byte payload; little-endian bitmask, bit0 = Cell1, bit15 = Cell16
        let (_n, buf) = self.read_subcmd_response::<2>(subcmd::CB_ACTIVE_CELLS, 2).await?;
        Ok(u16::from_le_bytes([buf[0], buf[1]]))
    }



    /// Attempt to transition from SEALED to UNSEALED by sending two 16-bit words (little-endian)
    /// to 0x3E/0x3F back-to-back within 4 seconds.
    pub async fn try_unseal_with_keys<D: DelayNs>(
        &mut self,
        key1: u16,
        key2: u16,
        delay: &mut D,
    ) -> Result<(), Error<E>> {
        let [k1_lo, k1_hi] = key1.to_le_bytes();
        let [k2_lo, k2_hi] = key2.to_le_bytes();

        defmt::info!("UNSEAL: sending key1=0x{:04x}, key2=0x{:04x}", key1, key2);

        // Write key1 -> 0x3E (low), 0x3F (high)
        self.i2c.write(self.addr, &[cmd::SUBCMD_LOW, k1_lo]).await.map_err(Error::I2c)?;
        self.i2c.write(self.addr, &[cmd::SUBCMD_HI, k1_hi]).await.map_err(Error::I2c)?;
        delay.delay_ms(2).await;

        // Write key2 -> 0x3E (low), 0x3F (high) within 4s
        self.i2c.write(self.addr, &[cmd::SUBCMD_LOW, k2_lo]).await.map_err(Error::I2c)?;
        self.i2c.write(self.addr, &[cmd::SUBCMD_HI, k2_hi]).await.map_err(Error::I2c)?;
        delay.delay_ms(4).await;

        // Verify transition by reading SEC1:0
        match self.get_security_state().await {
            Ok(SecurityState::Unsealed) | Ok(SecurityState::FullAccess) => {
                defmt::info!("UNSEAL: success (state now UNSEALED/FULLACCESS)");
                Ok(())
            }
            Ok(s) => {
                defmt::warn!("UNSEAL: device remained in state {:?}", s);
                Err(Error::InvalidParameter)
            }
            Err(e) => Err(e),
        }
    }

    // Probe without changing persistent state.
    async fn probe_addr(&mut self, addr: u8) -> bool {
        let old = self.addr;
        self.addr = addr;
        let ok = self.get_battery_status().await.is_ok();
        self.addr = old;
        ok
    }

    /// Change I²C address (RAM) and immediately switch the host to the new address.
    ///
    /// `new_addr_7bit` is the *7-bit* address (e.g., 0x0B). TRM requires writing the *8-bit write address* into DM.
    /// If the device is SEALED, provide the two 16-bit UNSEAL words; if None, we attempt without unsealing.
    pub async fn set_i2c_address_ram_checked<D: DelayNs>(
        &mut self,
        new_addr_7bit: u8,
        unseal_keys: Option<(u16, u16)>,
        delay: &mut D,
    ) -> Result<(), Error<E>> {
        defmt::info!("I2C-ADDR: requested 7-bit=0x{:02x}", new_addr_7bit);

        if self.probe_addr(new_addr_7bit).await {
            defmt::info!("I2C-ADDR: device already at 0x{:02x}; switching driver only", new_addr_7bit);
            self.addr = new_addr_7bit;
            return Ok(());
        }
        // 0) Check/handle security state
        let sec = self.get_security_state().await?;
        defmt::info!("SEC state before: {:?}", sec);
        if matches!(sec, SecurityState::Sealed) {
            match unseal_keys {
                Some((k1, k2)) => {
                    defmt::info!("Device SEALED -> attempting UNSEAL");
                    self.try_unseal_with_keys(k1, k2, delay).await?;
                }
                None => {
                    defmt::warn!("Device SEALED and no UNSEAL keys provided; aborting.");
                    return Err(Error::InvalidParameter);
                }
            }
        }

        // 1) Enter CONFIG_UPDATE and verify bit set
        defmt::info!("SET_CFGUPDATE");
        self.sub_command(subcmd::CONFIG_UPDATE).await?;
        delay.delay_ms(2).await;

        let bs1 = self.get_battery_status().await?;
        if !bs1.config_update_mode {
            defmt::warn!("CONFIG_UPDATE bit not set; abort.");
            return Err(Error::Timeout);
        }
        defmt::info!("CONFIG_UPDATE confirmed.");

        // 2) Write DM: Settings:Configuration:I2C Address (0x923A) = 8-bit write address
        let write_addr_8bit = new_addr_7bit << 1;
        defmt::info!("Writing DM I2C Address (0x923A) = 0x{:02x}", write_addr_8bit);

        self.write_df(datamem::I2C_ADDRESS, &[write_addr_8bit]).await?;

        // Small settle; also a light poll pattern to avoid reading during busy window
        delay.delay_ms(2).await;

        // 3) Exit CONFIG_UPDATE and verify bit clear
        defmt::info!("EXIT_CFGUPDATE");
        self.sub_command(subcmd::EXIT_CONFIG_UPDATE).await?;
        delay.delay_ms(1).await;

        let bs2 = self.get_battery_status().await?;
        if bs2.config_update_mode {
            defmt::warn!("CONFIG_UPDATE bit still set; abort.");
            return Err(Error::Timeout);
        }
        defmt::info!("Exited CONFIG_UPDATE.");

        // 4) SWAP_COMM_MODE to apply new comm settings (incl. I²C Address)
        defmt::info!("SWAP_COMM_MODE to apply new DM comm settings");
        self.sub_command(subcmd::SWAP_COMM_MODE).await?;
        delay.delay_ms(2).await;

        // 5) Probe at new address: try a benign read
        let old_addr = self.addr;
        self.addr = new_addr_7bit;
        defmt::info!("Probing BQ at new addr 0x{:02x}", self.addr);

        // Try several times in case bus/host needs a beat
        let mut ok = false;
        for _ in 0..5 {
            if self.get_battery_status().await.is_ok() {
                ok = true;
                break;
            }
            delay.delay_ms(2).await;
        }

        if ok {
            defmt::info!("I2C-ADDR: successfully communicating at 0x{:02x}", self.addr);
            Ok(())
        } else {
            defmt::warn!("I2C-ADDR: probe at new address failed; reverting to 0x{:02x}", old_addr);
            self.addr = old_addr;
            Err(Error::Timeout)
        }
    }

    /// Set autonomous balancing modes. Only touches CB_CHG/CB_RLX bits, leaves others as-is.
    pub async fn set_autobalance_modes<D: DelayNs>(
        &mut self,
        enable_chg: bool,
        enable_rlx: bool,
        delay: &mut D,
    ) -> Result<(), Error<E>> {
        const CB_RLX: u8 = 1 << 2;
        const CB_CHG: u8 = 1 << 3;

        // Read-modify-write H1
        let h1 = self.read_df::<1>(datamem::BAL_CFG_H1).await?[0];
        let mut new_h1 = h1;
        new_h1 = if enable_chg { new_h1 | CB_CHG } else { new_h1 & !CB_CHG };
        new_h1 = if enable_rlx { new_h1 | CB_RLX } else { new_h1 & !CB_RLX };

        if new_h1 != h1 {
            // Enter/exit CONFIG_UPDATE already wrapped by write_data_memory_u8().
            self.write_data_memory_u8(datamem::BAL_CFG_H1, new_h1, delay).await?;
            defmt::info!("Balancing H1: 0x{:02x} -> 0x{:02x}", h1, new_h1);
        } else {
            defmt::info!("Balancing H1 unchanged: 0x{:02x}", h1);
        }
        Ok(())
    }

    /// Wake the BQ76952 from SHUTDOWN by manufacturing a high→low on TS2.
    pub async fn wake_via_ts2<PIN, D>(
        &mut self,
        ts2: &mut PIN,
        delay: &mut D,
    ) -> Result<(), Error<E>>
    where
        PIN: Ts2PinControl,
        D: DelayNs,
    {
        const TS2_PREBIAS_MS: u32 = 15;   // get node clearly above threshold
        const TS2_LOW_PULSE_MS: u32 = 120; // 100–150 ms; choose 120 ms
        const TS2_PDN_MS: u32 = 8;        // brief pulldown to force a clean falling edge
        const TS2_BOOT_WAIT_MS: u32 = 350; // 300–400 ms typical OTP/config load

        defmt::debug!("Waking BQ76952 via TS2 (pre-bias > threshold)");
        ts2.prebias_high();
        delay.delay_ms(TS2_PREBIAS_MS).await;

        defmt::debug!("TS2 -> LOW pulse");
        ts2.drive_low();
        delay.delay_ms(TS2_LOW_PULSE_MS).await;

        defmt::debug!("TS2 brief input pulldown ({} ms)", TS2_PDN_MS);
        ts2.bias_pulldown();
        delay.delay_ms(TS2_PDN_MS).await;

        defmt::debug!("TS2 final release (Hi-Z)");
        ts2.release();

        delay.delay_ms(TS2_BOOT_WAIT_MS).await;

        // Probe with retries (covers long OTP boot / CUDEP)
        for _ in 0..30 {
            if self.get_battery_status().await.is_ok() {
                defmt::debug!("BQ76952 is awake");
                return Ok(());
            }
            delay.delay_ms(50).await;
        }
        Err(Error::Timeout)
    }

    pub async fn ensure_awake_at_addr<D: DelayNs, PIN: super::device::Ts2PinControl>(
        &mut self,
        target_7bit: u8,
        ts2: &mut PIN,
        delay: &mut D,
    ) -> Result<(), Error<E>> {
        // Try at 0x0B then 0x08
        if self.probe_addr(target_7bit).await {
            self.addr = target_7bit;
        } else if self.probe_addr(0x08).await {
            self.addr = 0x08;
        } else {
            // Wake then probe again
            self.wake_via_ts2(ts2, delay).await.ok();
            Timer::after(Duration::from_millis(300)).await;
            if self.probe_addr(target_7bit).await {
                self.addr = target_7bit;
            } else if self.probe_addr(0x08).await {
                self.addr = 0x08;
            } else {
                defmt::warn!("BQ76952 not detected at 0x0B/0x08 after wake.");
                return Err(Error::Timeout);
            }
        }

        // If we're at 0x08, move to target_7bit now (we might have needed the wake first).
        if self.addr == 0x08 && target_7bit != 0x08 {
            defmt::info!("Switching RAM I²C address 0x08 -> 0x{:02x}", target_7bit);
            // pass None for unseal keys; will fail gracefully if sealed
            self.set_i2c_address_ram_checked(target_7bit, None, delay).await?;
        }

        // Enable autonomous balancing (charge + relax)
        self.set_autobalance_modes(config::ENABLE_CB_CHG, config::ENABLE_CB_RLX, delay).await?;
        Ok(())
    }
}

use embassy_sync::pubsub::Subscriber;

#[embassy_executor::task]
pub async fn bq_readout_task(
    bq_mutex: &'static Mutex<CriticalSectionRawMutex, Bq76952<I2cDev<'static>>>,
    mut ts2: Flex<'static>,
) {
    let publisher = crate::shared_state::BMS_CHANNEL.publisher().unwrap();
    let mut command_subscriber: Subscriber<'static, CriticalSectionRawMutex, BmsCommand, 2, 1, 3> =
        crate::shared_state::BMS_COMMAND_CHANNEL.subscriber().unwrap();
    // Small async Delay that implements embedded_hal_async::delay::DelayNs.
    struct Delay;
    impl embedded_hal_async::delay::DelayNs for Delay {
        async fn delay_ns(&mut self, ns: u32) { Timer::after(Duration::from_nanos(ns as u64)).await; }
        async fn delay_us(&mut self, us: u32) { Timer::after(Duration::from_micros(us as u64)).await; }
        async fn delay_ms(&mut self, ms: u32) { Timer::after(Duration::from_millis(ms as u64)).await; }
    }
    let mut d = Delay;

    {
        let mut bq = bq_mutex.lock().await;
        if let Err(e) = bq.ensure_awake_at_addr(0x0B, &mut ts2, &mut d).await {
            defmt::warn!("BQ76952 bring-up failed: {:?}", defmt::Debug2Format(&e));
        } else {
            defmt::info!("BQ76952 ready at 0x0B with CB_CHG/CB_RLX enabled");
        }
    }
    {
        let mut bq = bq_mutex.lock().await;

        // Heuristic: any VCELL > 50 mV counts as “present”.
        let mut mask: u16 = 0;
        if let Ok(v) = bq.get_all_cell_voltages().await {
            for i in 0..16 {
                if v[i] > 50 { mask |= 1u16 << i; }
            }
        }
        if mask == 0 {
            defmt::warn!("Active-cells autodetect found none; leaving mask=0 (will still log all).");
        } else {
            bq.set_active_cells_mask(mask);
            // Pretty print (e.g., 1,2,16).
            defmt::info!(
                "Active cells (autodetect): {}",
                defmt::Display2Format(&crate::shell::fmt_bit_cells(mask))
            );
        }
    }
    loop {
        // Handle incoming commands
        if let Some(command) = command_subscriber.try_next_message_pure() {
            let mut bq = bq_mutex.lock().await;
            match command {
                BmsCommand::ToggleFetEnable => { bq.toggle_fet_enable().await.ok(); }
                BmsCommand::SetBalancingTime(cell, time) => { bq.set_balancing_time(cell, time, &mut Delay).await.ok(); }
                BmsCommand::SetCellOvervoltageProtection(mv, delay_ms) => {
                    bq.set_cell_overvoltage_protection(mv, delay_ms, &mut Delay).await.ok();
                }
                BmsCommand::SetCellUndervoltageProtection(mv, delay_ms) => {
                    bq.set_cell_undervoltage_protection(mv, delay_ms, &mut Delay).await.ok();
                }
                BmsCommand::SetI2cAddress(new_addr, unseal_keys) => {
                    if let Err(e) = bq.set_i2c_address_ram_checked(new_addr, unseal_keys, &mut Delay).await {
                        defmt::warn!("BQ76952: set_i2c_address_ram_checked failed: {:?}", defmt::Debug2Format(&e));
                    }
                }
            }
        }

        let readings = {
            let mut bq = bq_mutex.lock().await;

            // ---------- Direct status ----------
            match bq.get_battery_status().await {
                Ok(s) => defmt::info!(
                    "BatteryStatus: sleep={} shutdown_pend={} perm_fault={} safety_fault={} cfg_update={} sealed?={}",
                    s.sleep_mode, s.shutdown_pending, s.permanent_fault, s.safety_fault,
                    s.config_update_mode, (s.security_state == 0)
                ),
                Err(_) => defmt::warn!("BatteryStatus read failed"),
            }

            // ---------- Cells ----------
            bq.log_all_cell_voltages().await;

            // ---------- Balancing ----------
            match bq.get_cb_active_mask().await {
                Ok(mask) => {
                    defmt::info!("Balancing active mask = 0x{:04x}", mask);
                    defmt::info!("Balancing cells: {}", defmt::Display2Format(&crate::shell::fmt_bit_cells(mask)));
                }
                Err(_) => defmt::warn!("CB_ACTIVE_CELLS read failed"),
            }

            // ---------- DASTATUS 5..7 (typed defmt only; no width/precision) ----------
            if let Ok(buf5) = bq.read_dastatus5().await {
                let d5 = bq.decode_dastatus5(&buf5);
                if crate::config::EXTERNAL_THERMISTORS_PRESENT || crate::config::READ_INTERNAL_TEMP {
                    defmt::info!(
                        "DA5: VREG18={=u16}cts VSS={=u16}cts MAX={=u16}mV MIN={=u16}mV PACK={=u32}mV \
                        T(avg/fet/max/min)={=f32}/{=f32}/{=f32}/{=f32}°C \
                        CC1/3(userA)={=i16}/{=i16} rawCC2={=i32} rawCC3={=i32}",
                        d5.vreg18_counts, d5.vss_counts, d5.max_cell_mv, d5.min_cell_mv, d5.pack_mv,
                        d5.avg_cell_temp_c, d5.fet_temp_c, d5.max_cell_temp_c, d5.min_cell_temp_c,
                        d5.cc1_usera, d5.cc3_usera, d5.cc2_counts, d5.cc3_counts
                    );
                } else {
                    defmt::info!(
                        "DA5: VREG18={=u16}cts VSS={=u16}cts MAX={=u16}mV MIN={=u16}mV PACK={=u32}mV \
                        (temps disabled) CC1/3(userA)={=i16}/{=i16} rawCC2={=i32} rawCC3={=i32}",
                        d5.vreg18_counts, d5.vss_counts, d5.max_cell_mv, d5.min_cell_mv, d5.pack_mv,
                        d5.cc1_usera, d5.cc3_usera, d5.cc2_counts, d5.cc3_counts
                    );
                }
            } else {
                defmt::warn!("DASTATUS5 read failed");
            }

            if let Ok(buf6) = bq.read_dastatus6().await {
                let d6 = bq.decode_dastatus6(&buf6);
                defmt::info!(
                    "DA6: Qacc_hi=0x{=u32:x} Qacc_lo=0x{=u32:x} t={=u32}s \
                    pins[CFET/DFET/ALERT/TS1/TS2]={=i32}/{=i32}/{=i32}/{=i32}/{=i32}",
                    d6.q_accum_userah_hi, d6.q_accum_userah_lo, d6.time_accum_s,
                    d6.cfetoff_counts, d6.dfetoff_counts, d6.alert_counts, d6.ts1_counts, d6.ts2_counts
                );
            } else {
                defmt::warn!("DASTATUS6 read failed");
            }

            if let Ok(buf7) = bq.read_dastatus7().await {
                let d7 = bq.decode_dastatus7(&buf7);
                defmt::info!(
                    "DA7: pins[TS3/HDQ/DCHG/DDSG]={=i32}/{=i32}/{=i32}/{=i32}",
                    d7.ts3_counts, d7.hdq_counts, d7.dchg_counts, d7.ddsg_counts
                );
            } else {
                defmt::warn!("DASTATUS7 read failed");
            }

            // ---------- Compact readings ----------
            let v_mv = bq.get_stack_voltage_mv().await.unwrap_or(0);
            if v_mv == 0 { defmt::warn!("VSTACK read failed"); }
            let t_c: i8 = if crate::config::READ_INTERNAL_TEMP {
                bq.get_internal_temp_c().await.unwrap_or(0.0) as i8
            } else { 0 };

            // No shunt installed -> publish 0 mA
            BmsReadings { voltage_mv: v_mv, current_ma: 0, temperature_c: t_c }
        };

        if let Err(_e) = publisher.try_publish(readings) {
            defmt::debug!("BMS drop (no subscriber)");
        }
        else {
            defmt::debug!("BMS data published.");
        }

        // (Optional) debug dump could include per-cell voltages / alarms on a slower cadence.

        Timer::after_secs(5).await;
    }
}
