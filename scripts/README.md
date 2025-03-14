These files are individual testing scripts used for quick validation.

# Table of Contents:
- [`esc_calibration.ino`](/esc_calibration.ino): 
    - Calibration code for [ESCs](https://www.rcelectricparts.com/classic-esc-user-guide.html). Run with ESP32/Arduino Uno R3 with a button wired up as well to control maximum and minimum PWM signals.
- [`test_motor_speeds.ino`](/test_motor_speeds.ino):
    - Once calibration of ESCs has been completed, test motor speed range with this script. Please note that you should have a good mount for the motor.
