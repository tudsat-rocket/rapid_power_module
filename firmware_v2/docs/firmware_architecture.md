# Firmware Architecture

This document provides an overview of the firmware architecture for the Hyacinth Rapid Power Board.

## Core Framework

The firmware is built using the [Embassy](https://embassy.dev/) asynchronous embedded framework for Rust. This allows for robust, concurrent operation without the need for a traditional Real-Time Operating System (RTOS).

Key features of this approach:
- **Asynchronous Execution**: All I/O operations (like I2C communication) are non-blocking. This allows the CPU to perform other tasks while waiting for hardware, leading to better efficiency and responsiveness.
- **Static Guarantees**: Embassy leverages Rust's type system to provide compile-time checks for resource sharing and task scheduling, eliminating entire classes of bugs common in embedded systems.
- **Modern Tooling**: The project uses `defmt` for efficient and flexible logging.

## Project Structure

The project is organized as a Cargo workspace with the following members:

-   `drivers/bq76952/`: An `async` driver crate for the **TI BQ76952** Battery Management System (BMS).
-   `drivers/bq25756/`: An `async` driver crate for the **TI BQ25756` Buck-Boost Charger.
-   `apps/monitor/`: A binary crate that contains the main application logic. It demonstrates how to use the two drivers together on the target hardware.

This modular structure separates the hardware drivers from the application logic, making the code easier to maintain, test, and reuse.

## Application Logic (`monitor`)

The main application is located in `apps/monitor/src/main.rs`. Its responsibilities include:

1.  **Initialization**:
    -   Configuring the `embassy-stm32` HAL for the STM32F105RC microcontroller, including setting the system clock.
    -   Initializing the I2C1 peripheral on pins `PB6` (SCL) and `PB7` (SDA).
    -   Creating a shared I2C bus using an `embassy-sync` Mutex. This allows both the BMS and Charger drivers to safely share the single I2C bus.

2.  **Driver Setup**:
    -   Instantiating the `Bq76952` and `Bq25756` drivers.
    -   Calling initialization routines for both chips (`bms_init` and `charger_init`) to configure them with safe, default settings.

3.  **Main Loop**:
    -   The application enters an infinite async loop.
    -   Every 5 seconds, it reads key telemetry from both the BMS and the charger.
    -   The data is logged to the console using `defmt`.

## Monitored Values

The `monitor` application reads and logs the following values:

### From the BQ76952 (BMS):
-   **Stack Voltage**: The total voltage of the entire battery stack (in mV).
-   **Current**: The current flowing into or out of the battery (in mA).
-   **Internal Temperature**: The internal die temperature of the BMS chip (in °C).

### From the BQ25756 (Charger):
-   **Input Voltage**: The voltage at the charger's input (in mV).
-   **Charge Current**: The current being delivered to the battery (in mA).
-   **Charger Status**: The operational status of the charger (e.g., charging, idle, fault).

## USB Composite Device

The firmware includes a USB composite device that exposes:

1. **CDC-ACM (Virtual Serial Port)**: Used for logs, telemetry, and CLI.
2. **DFU Runtime Interface**: Allows the host to send a `DFU_DETACH` command, triggering the application to reset into the bootloader for firmware updates.

#### Implementation Details

- **CDC-ACM**: Implemented using `embassy_usb::class::cdc_acm`. This provides a standard serial-over-USB interface.
- **DFU Runtime**: Implemented using `embassy_usb_dfu::usb_dfu`. This adds a DFU interface to the USB composite device and handles the detach-reset-bootloader flow.
- **Composite Device Class**: The device class is set to Miscellaneous (0xEF/0x02/0x01), which is conventional for multi-interface USB devices.

#### USB Configuration

- **Vendor ID (VID)**: 0xCafe
- **Product ID (PID)**: 0x4010
- **Manufacturer**: Blue Networks
- **Product**: RapidPower
- **Serial Number**: RP-0001
- **Max Power**: 500 mA

#### Host-Side Notes

- **udev Rules** (Linux):
  Add the following to `/etc/udev/rules.d/99-rapidpower.rules` to avoid requiring `sudo` for accessing the USB device:
  ```
  SUBSYSTEM=="tty", ATTRS{idVendor}=="cafe", ATTRS{idProduct}=="4010", MODE="0666"
  SUBSYSTEM=="usb", ATTR{idVendor}=="cafe", ATTR{idProduct}=="4010", MODE="0666"
  ```
  Then reload the rules:
  ```
  sudo udevadm control --reload-rules && sudo udevadm trigger
  ```

- **DFU-Util**:
  - To trigger the bootloader:
    ```
    dfu-util -e
    ```
  - To flash firmware in DFU mode:
    ```
    dfu-util -a 0 -D rapid_power_fw.bin
    ```

#### Tasks

- **USB Device Runner**: The USB device must be polled continuously to keep it alive. This is handled by the `run_usb` task.
- **CDC Task**: A simple echo task is implemented for the CDC interface. Replace this with telemetry publishing or CLI handling as needed.

## Future Work

- **CAN Communication**: Add support for CAN communication to integrate with the vehicle's broader system.
- **Error Handling**: Improve error handling and recovery mechanisms for I2C communication.
- **Testing**: Add unit tests and hardware-in-the-loop (HIL) tests to ensure reliability.
