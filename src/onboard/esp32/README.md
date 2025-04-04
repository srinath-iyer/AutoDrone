# ESP32 PlatformIO Project

This directory contains the source code and configuration files for the ESP32 module of the AutoDrone project. The project is managed using [PlatformIO](https://platformio.org/), a professional tool for embedded development.

## Directory Structure

### `src/`
- **`main.c`**: Entry point for the ESP32 application.

### `include/`
- **`config.h`**: Configuration constants and macros for the project.

### `lib/`

`lib/` contains custom libraries and files.
- **`constants`**: Constant values
- **`motor_driver`**: Sending PWM signals to motors
- **`motor_mixing`**: 
- **`mpu6050`**:
- **`pid_controller`**:
- **`pose`**:
- **`safeties`**:
- **`sensor_fusion`**:


### `test/`
- **`test_main.c`**: Unit tests for the main application logic.

### `platformio.ini`
- PlatformIO configuration file for setting up the ESP32 environment.

## Getting Started with PlatformIO and this Project

### Drivers:
- If you haven't worked with microcontrollers before, you first need to download [this](https://www.silabs.com/developer-tools/usb-to-uart-bridge-vcp-drivers) driver. This will allow you to connect to an ESP32 and communicate over USB.

### PlatformIO Setup:


## License

This project is licensed under the MIT License. See the `LICENSE` file for details.  