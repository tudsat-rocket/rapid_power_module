# RAPID Power Module Firmware

This repository contains the firmware for the RAPID Power Module.

## Table of Contents

- [BQ76952 Battery Monitor](#bq76952-battery-monitor)
- [BQ25756 Charger](#bq25756-charger)
- [CYPD3177 USB-PD Controller](#cypd3177-usb-pd-controller)
- [Building and Flashing](#building-and-flashing)

## BQ76952 Battery Monitor

TODO: Add a general description of the BQ76952 driver.

### Safe I2C Address Change

The firmware implements a robust method to change the I2C address of the BQ76952 device. This is useful for systems with multiple devices on the same I2C bus.

The `set_i2c_address_ram_checked` function provides a safe way to change the address. It includes the following features:
- **Security State Handling**: The function checks the security state of the device and can unseal it if provided with the correct keys.
- **Verbose Logging**: Each step of the address change process is logged for easier debugging.
- **State Verification**: The function verifies each state transition to ensure the device is in the correct mode.
- **Fallback Mechanism**: If the address change fails, the driver reverts to the old address to maintain communication.

#### Usage

To change the I2C address, you can send a `BmsCommand::SetI2cAddress` command to the `bq_readout_task`. The command takes the new 7-bit I2C address and an `Option<(u16, u16)>` for the unseal keys.

```rust
// Example of changing the I2C address to 0x0B
let unseal_keys = Some((0x1234, 0x5678)); // Replace with actual keys if needed
bms_command_publisher.publish(shared_state::BmsCommand::SetI2cAddress(0x0B, unseal_keys)).await;
```

At startup, the firmware is configured to change the I2C address from the default `0x08` to `0x0B`.

## BQ25756 Charger

TODO: Add documentation for the BQ25756 charger.

## CYPD3177 USB-PD Controller

TODO: Add documentation for the CYPD3177 USB-PD controller.

## Building and Flashing

TODO: Add instructions on how to build and flash the firmware.






### KNOWN ERRORS

- Voltage on ST-Link Debug from Batteries
- Due to missing thermistor only about 23-25% of the charging current is used. Due to this the charging current needs to be increased fourfold to reach the desired current.