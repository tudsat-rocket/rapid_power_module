use embedded_hal_async::delay::DelayNs;
use embedded_hal_async::{i2c::I2c};

use super::types::*;
use super::regs::{datamem, subcmd};
use super::Ts2PinControl;

/// An async driver for the TI **BQ76952** battery monitor and protector.
///
/// This driver provides a high-level API to configure, control, and read data from the BQ76952.
/// It is designed to be used with any I²C implementation that satisfies the `embedded-hal-async` traits.
///
/// The driver's architecture mirrors the device's command structure, distinguishing between:
/// - **Direct Commands**: For reading measurements and status registers.
/// - **Subcommands**: For control operations and fetching detailed data blocks.
/// - **Data Memory**: For persistent configuration, accessed via a special procedure involving
///   `CONFIG_UPDATE` mode.
///
/// # Example
/// See the `monitor` example in the workspace for a demonstration of how to initialize and use the driver.
pub struct Bq76952<I2C> {
    i2c: I2C,
    addr: u8, // 7-bit I²C address
    rsense_milliohms: u32,
}

impl<I2C, E> Bq76952<I2C>
where
    I2C: I2c<Error = E>,
{
    /// Creates a new driver instance.
    ///
    /// # Arguments
    /// * `i2c` - An I²C peripheral that implements the `embedded-hal-async` traits.
    /// * `addr` - The 7-bit I²C address of the device. The default is `0x08`.
    /// * `rsense_milliohms` - The value of the sense resistor in milliohms. This is used for current calculations.
    pub fn new(i2c: I2C, addr: u8, rsense_milliohms: u32) -> Self {
        Self {
            i2c,
            addr,
            rsense_milliohms,
        }
    }

    /// Consumes the driver and returns the underlying I²C peripheral.
    pub fn free(self) -> I2C {
        self.i2c
    }

    /// Initializes the driver. Currently a no-op, but included for API completeness.
    pub fn begin(&mut self) {}

    /// Sends the `RESET` subcommand (`0x0012`) to perform a full device reset.
    /// See TRM Subcommands Table (p.112-119).
    pub async fn reset<D: DelayNs>(&mut self, _delay: &mut D) -> Result<(), Error<E>> {
    self.sub_command(super::regs::subcmd::RESET).await
    }

    /// Reads the total voltage of the battery stack.
    ///
    /// The value is returned in units of 10 mV.
    /// See TRM §12.1.15 (p.95).
    pub async fn get_stack_voltage(&mut self) -> Result<u16, Error<E>> {
        self.read_direct_u16(cmd::VSTACK).await
    }

    /// Reads the current measured by the CC2 integrator.
    ///
    /// The value is returned in milliamps (mA) by default.
    /// See TRM §12.1.16 (p.95).
    pub async fn get_current_cc2(&mut self) -> Result<i16, Error<E>> {
        self.read_direct_i16(cmd::CC2_CUR).await
    }

    /// Reads the internal die temperature of the BQ76952.
    ///
    /// The value is returned in degrees Celsius.
    /// See TRM §12.1.17 (p.96).
    pub async fn get_internal_temp_c(&mut self) -> Result<f32, Error<E>> {
        let temp_0_1k = self.read_direct_u16(cmd::INT_TEMP).await?;
        Ok((temp_0_1k as f32 * 0.1) - 273.15)
    }

    /// Reads the temperature from one of the external thermistors.
    ///
    /// The value is returned in degrees Celsius.
    /// See TRM §12.1.18–§12.1.20 (p.95-97).
    pub async fn get_thermistor_temp_c(&mut self, thermistor: Thermistor) -> Result<f32, Error<E>> {
        let reg = match thermistor {
            Thermistor::TS1 => cmd::TS1,
            Thermistor::TS2 => cmd::TS2,
            Thermistor::TS3 => cmd::TS3,
        };
        let temp_0_1k = self.read_direct_u16(reg).await?;
        Ok((temp_0_1k as f32 * 0.1) - 273.15)
    }

    /// Reads the voltage of a single cell.
    ///
    /// # Arguments
    /// * `cell_number` - The cell number to read (1-16).
    ///
    /// The value is returned in millivolts (mV).
    /// See TRM §12.1.1 (p.91).
    pub async fn get_cell_voltage(&mut self, cell_number: u8) -> Result<u16, Error<E>> {
        if !(1..=16).contains(&cell_number) {
            return Err(Error::InvalidParameter);
        }
        let reg = cmd::VCELL1 + (cell_number - 1) * 2;
        self.read_direct_u16(reg).await
    }

    /// Reads the voltages of all 16 cells.
    ///
    /// The values are returned in millivolts (mV) in a 16-element array.
    pub async fn get_all_cell_voltages(&mut self) -> Result<[u16; 16], Error<E>> {
        let mut voltages = [0u16; 16];
        for i in 0..16 {
            voltages[i] = self.get_cell_voltage((i + 1) as u8).await?;
        }
        Ok(voltages)
    }

    /// Helper to read a 16-bit unsigned value from a direct command address.
    async fn read_direct_u16(&mut self, command: u8) -> Result<u16, Error<E>> {
        let mut buffer = [0u8; 2];
        self.i2c
            .write_read(self.addr, &[command], &mut buffer)
            .await
            .map_err(Error::I2c)?;
        defmt::debug!("Read direct u16 from command {=u8}: {=u16}", command, u16::from_le_bytes(buffer));
        Ok(u16::from_le_bytes(buffer))
    }

    /// Helper to read a 16-bit signed value from a direct command address.
    async fn read_direct_i16(&mut self, command: u8) -> Result<i16, Error<E>> {
        let mut buffer = [0u8; 2];
        self.i2c
            .write_read(self.addr, &[command], &mut buffer)
            .await
            .map_err(Error::I2c)?;
        defmt::debug!("Read direct i16 from command {=u8}: {=i16}", command, i16::from_le_bytes(buffer));
        Ok(i16::from_le_bytes(buffer))
    }

    /// Helper to send a 16-bit subcommand.
    async fn sub_command(&mut self, command: u16) -> Result<(), Error<E>> {
        let [lo, hi] = command.to_le_bytes();
        self.i2c
            .write(self.addr, &[cmd::SUBCMD_HI, lo])
            .await
            .map_err(Error::I2c)?;
        self.i2c
            .write(self.addr, &[cmd::SUBCMD_LOW, hi])
            .await
            .map_err(Error::I2c)?;
        Ok(())
    }

    /// Sets the balancing time for a single cell.
    ///
    /// # Arguments
    /// * `cell_number` - The cell to configure (1-16).
    /// * `time_seconds` - The balancing time in seconds.
    ///
    /// Note: The device must be in CONFIG_UPDATE mode to change this setting.
    pub async fn set_balancing_time<D: DelayNs>(
        &mut self,
        cell_number: u8,
        time_seconds: u8,
        delay: &mut D,
    ) -> Result<(), Error<E>> {
        if !(1..=16).contains(&cell_number) {
            return Err(Error::InvalidParameter);
        }
    let address = super::regs::datamem::CELL_BALANCING_TIME_BASE + (cell_number - 1) as u16;
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

    /// Helper to write a single byte to a data memory address.
    pub async fn write_data_memory_u8<D: DelayNs>(
        &mut self,
        address: u16,
        data: u8,
        delay: &mut D,
    ) -> Result<(), Error<E>> {
        // Enter CONFIG_UPDATE (0x0090)
    self.sub_command(super::regs::subcmd::CONFIG_UPDATE).await?;
        delay.delay_ms(2u32).await;

        // Do the write
        self.write_df(address, &[data]).await?;

        // Exit CONFIG_UPDATE (0x0092)
    self.sub_command(super::regs::subcmd::EXIT_CONFIG_UPDATE).await
    }

    /// Toggles the FET_EN bit in Manufacturing Status.
    pub async fn toggle_fet_enable(&mut self) -> Result<(), Error<E>> {
    self.sub_command(super::regs::subcmd::FET_ENABLE).await
    }

    /// Reads the status of the FETs.
    pub async fn get_fet_status(&mut self) -> Result<FetStatus, Error<E>> {
        let s = self.read_direct_u8(cmd::FET_STATUS).await?;
        Ok(FetStatus {
            chg_fet: (s & (1 << 0)) != 0,
            pchg_fet: (s & (1 << 1)) != 0,
            dsg_fet: (s & (1 << 2)) != 0,
            pdsg_fet: (s & (1 << 3)) != 0,
            dchg_pin: (s & (1 << 4)) != 0,
            ddsg_pin: (s & (1 << 5)) != 0,
            alert_pin: (s & (1 << 6)) != 0,
        })
    }

    /// Checks if the discharging FET is enabled.
    pub async fn is_discharging(&mut self) -> Result<bool, Error<E>> {
        Ok(self.get_fet_status().await?.dsg_fet)
    }

    /// Checks if the charging FET is enabled.
    pub async fn is_charging(&mut self) -> Result<bool, Error<E>> {
        Ok(self.get_fet_status().await?.chg_fet)
    }

    /// Helper to read a single byte from a direct command address.
    async fn read_direct_u8(&mut self, command: u8) -> Result<u8, Error<E>> {
        let mut buffer = [0u8; 1];
        self.i2c
            .write_read(self.addr, &[command], &mut buffer)
            .await
            .map_err(Error::I2c)?;
        Ok(buffer[0])
    }

    /// Sets the cell overvoltage protection threshold and delay.
    ///
    /// # Arguments
    /// * `voltage_mv` - The voltage threshold in millivolts.
    /// * `delay_ms` - The delay in milliseconds.
    pub async fn set_cell_overvoltage_protection<D: DelayNs>(
        &mut self,
        voltage_mv: u16,
        delay_ms: u16,
        delay: &mut D,
    ) -> Result<(), Error<E>> {
        let threshold = ((voltage_mv - 2000) as f32 / 50.6) as u8;
        let delay_val = (delay_ms as f32 / 3.3) as u8;
    self.write_data_memory_u8(super::regs::datamem::COV_THRESHOLD, threshold, delay)
            .await?;
    self.write_data_memory_u8(super::regs::datamem::COV_DELAY, delay_val, delay)
            .await
    }

    /// Sets the cell undervoltage protection threshold and delay.
    ///
    /// # Arguments
    /// * `voltage_mv` - The voltage threshold in millivolts.
    /// * `delay_ms` - The delay in milliseconds.
    pub async fn set_cell_undervoltage_protection<D: DelayNs>(
        &mut self,
        voltage_mv: u16,
        delay_ms: u16,
        delay: &mut D,
    ) -> Result<(), Error<E>> {
        let threshold = ((voltage_mv - 500) as f32 / 50.6) as u8;
        let delay_val = (delay_ms as f32 / 3.3) as u8;
    self.write_data_memory_u8(super::regs::datamem::CUV_THRESHOLD, threshold, delay)
            .await?;
    self.write_data_memory_u8(super::regs::datamem::CUV_DELAY, delay_val, delay)
            .await
    }

    /// Sets the short-circuit in discharge protection threshold and delay.
    ///
    /// # Arguments
    /// * `thresh_mv` - The threshold in millivolts.
    /// * `delay` - The delay.
    pub async fn set_scd<D: DelayNs>(
        &mut self,
        thresh_mv: u16,
        delay: ScdDelay,
        d: &mut D,
    ) -> Result<(), Error<E>> {
        let threshold_code = match thresh_mv {
            10 => 0,
            20 => 1,
            40 => 2,
            60 => 3,
            80 => 4,
            100 => 5,
            125 => 6,
            150 => 7,
            175 => 8,
            200 => 9,
            250 => 10,
            300 => 11,
            350 => 12,
            400 => 13,
            450 => 14,
            500 => 15,
            _ => return Err(Error::InvalidParameter),
        };
        let delay_code = match delay {
            ScdDelay::Us70 => 0,
            ScdDelay::Us100 => 1,
            ScdDelay::Us200 => 2,
            ScdDelay::Us400 => 3,
        };
        let value = (delay_code << 4) | threshold_code;
    self.write_data_memory_u8(super::regs::datamem::SCD_THRESHOLD, value, d)
            .await
    }

    pub async fn set_ocd1_ma<D: DelayNs>(
        &mut self,
        current_ma: u32,
        delay: OcdDelay,
        d: &mut D,
    ) -> Result<(), Error<E>> {
        let mv = ((current_ma as u64) * (self.rsense_milliohms as u64) / 1000) as u16;
        self.set_ocd1_mv(mv, delay, d).await
    }

    async fn set_ocd1_mv<D: DelayNs>(
        &mut self,
        thresh_mv: u16,
        delay: OcdDelay,
        d: &mut D,
    ) -> Result<(), Error<E>> {
        let threshold_code = (thresh_mv / 2) as u8;
        let delay_code = match delay {
            OcdDelay::Ms8 => 0,
            OcdDelay::Ms20 => 1,
            OcdDelay::Ms40 => 2,
            OcdDelay::Ms80 => 3,
            OcdDelay::Ms160 => 4,
            OcdDelay::Ms320 => 5,
        };
        let value = (delay_code << 4) | threshold_code;
    self.write_data_memory_u8(super::regs::datamem::OCD1_THRESHOLD, value, d)
            .await
    }

    pub async fn set_occ_ma<D: DelayNs>(
        &mut self,
        current_ma: u32,
        delay: OccDelay,
        d: &mut D,
    ) -> Result<(), Error<E>> {
        let mv = ((current_ma as u64) * (self.rsense_milliohms as u64) / 1000) as u16;
        self.set_occ_mv(mv, delay, d).await
    }

    async fn set_occ_mv<D: DelayNs>(
        &mut self,
        thresh_mv: u16,
        delay: OccDelay,
        d: &mut D,
    ) -> Result<(), Error<E>> {
        let threshold_code = (thresh_mv / 2) as u8;
        let delay_code = match delay {
            OccDelay::Ms100 => 0,
            OccDelay::Ms200 => 1,
            OccDelay::Ms400 => 2,
            OccDelay::Ms800 => 3,
            OccDelay::S1 => 4,
            OccDelay::S2 => 5,
        };
        let value = (delay_code << 4) | threshold_code;
    self.write_data_memory_u8(super::regs::datamem::OCC_THRESHOLD, value, d)
            .await
    }

    /// Reads the battery status register.
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

    /// Reads the alarm status register.
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
    pub async fn set_i2c_address_ram<D: DelayNs>(
        &mut self,
        new_addr_7bit: u8,
        delay: &mut D,
    ) -> Result<(), Error<E>> {
        // Enter CONFIG_UPDATE (0x0090). TRM suggests ~2 ms op time. :contentReference[oaicite:9]{index=9}
        self.sub_command(subcmd::CONFIG_UPDATE).await?;
        delay.delay_ms(2).await;

        // Write 0x923A (Settings:Configuration:I2C Address) = 8-bit write address
        let write_addr_8bit = new_addr_7bit << 1;
        self.write_df(datamem::I2C_ADDRESS, &[write_addr_8bit]).await?;

        // Exit CONFIG_UPDATE (0x0092). :contentReference[oaicite:10]{index=10}
        self.sub_command(subcmd::EXIT_CONFIG_UPDATE).await?;
        delay.delay_ms(1).await;

        // Apply the new comm settings immediately. :contentReference[oaicite:11]{index=11}
        self.sub_command(subcmd::SWAP_COMM_MODE).await?; 
        delay.delay_ms(2).await;

        // Now switch our **local** driver to use the new 7-bit address.
        self.addr = new_addr_7bit;
        Ok(())
    }
    /// Wake the BQ76952 from SHUTDOWN by manufacturing a high→low on TS2.
    /// Sequence: pre-bias high (~3.3 V) → pull low (~75 ms) → release (Hi-Z), then wait for boot.
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
        delay.delay_ms(TS2_PREBIAS_MS).await;          // short, just to charge the node above VWAKEONTS2

        defmt::debug!("TS2 -> LOW pulse");
        ts2.drive_low();
        delay.delay_ms(TS2_LOW_PULSE_MS).await;          // conservative pulse width

        defmt::debug!("TS2 brief input pulldown ({} ms)", TS2_PDN_MS);
        ts2.bias_pulldown();
        delay.delay_ms(TS2_PDN_MS).await;

        // Final Hi-Z so BQ owns the pin during boot
        defmt::debug!("TS2 final release (Hi-Z)");
        ts2.release();

        // Let OTP/config load
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
}
