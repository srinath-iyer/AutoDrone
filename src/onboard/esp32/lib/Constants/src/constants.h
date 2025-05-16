#ifndef CONSTANTS_C
#define CONSTANTS_C
// PID Constants:
#define THRUST_KP 1.0
#define THRUST_KI 1.0
#define THRUST_KD 1.0
#define YAW_KP 1.0
#define YAW_KI 1.0
#define YAW_KD 1.0
#define ROLL_KP 1.0
#define ROLL_KI 1.0
#define ROLL_KD 1.0
#define PITCH_KP 1.0
#define PITCH_KI 1.0
#define PITCH_KD 1.0
#define POSITION_ROLL_KP 1.0
#define POSITION_ROLL_KI 1.0
#define POSITION_ROLL_KD 1.0
#define POSITION_PITCH_KP 1.0
#define POSITION_PITCH_KI 1.0
#define POSITION_PITCH_KD 1.0

// Motor control constants:

#define MOTOR1_GPIO    4
#define MOTOR2_GPIO    5
#define MOTOR3_GPIO    18
#define MOTOR4_GPIO    19

// ESCs expect ~50 Hz, pulses of 1–2 ms out of 20 ms → duty = 5%–10%
#define PWM_FREQUENCY_HZ   50
#define MIN_PULSE_US     1000    // 1 ms
#define MAX_PULSE_US     2000    // 2 ms

// LEDC configuration
#define LEDC_TIMER              LEDC_TIMER_0
#define LEDC_MODE               LEDC_HIGH_SPEED_MODE
#define LEDC_DUTY_RES           LEDC_TIMER_13_BIT  // 13‑bit resolution (0–8191)
#define LEDC_MAX_DUTY           ((1 << 13) - 1)    // 8191

// Which LEDC channels to use for each motor
#define MOTOR1_CH               LEDC_CHANNEL_0
#define MOTOR2_CH               LEDC_CHANNEL_1
#define MOTOR3_CH               LEDC_CHANNEL_2
#define MOTOR4_CH               LEDC_CHANNEL_3

// Acceptable Error Tolerance:
#define X_ERROR_TOLERANCE 0.1f // 10cm
#define Y_ERROR_TOLERANCE 0.1f // 10cm
#define Z_ERROR_TOLERANCE 0.1f // 10cm
#define ROLL_ERROR_TOLERANCE 0.1f // 10 degrees
#define PITCH_ERROR_TOLERANCE 0.1f // 10 degrees
#define YAW_ERROR_TOLERANCE 0.1f // 10 degrees

// All Pinouts:

//ESP32 I2C Pins:

#define ESP32_I2C_MASTER_SLC 22
#define ESP32_I2C_MASTER_SDA 21
#define ESP32_I2C_MASTER_FREQ_HZ 100000
#define ESP32_I2C_MASTER_NUM I2C_NUM_0

// MPU6050 IMU Registers:
// https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Register-Map1.pdf
#define DEVICE_ADDRESS 0x68
#define PWR_MGMT_1_REG 0x6B
#define SMPLRT_DIV 0x19
#define ACCEL_XOUT_H 0x3B
#define ACCEL_XOUT_L 0x3C
#define ACCEL_YOUT_H 0x3D
#define ACCEL_YOUT_L 0x3E
#define ACCEL_ZOUT_H 0x3F
#define ACCEL_ZOUT_L 0x40
#define GYRO_XOUT_H 0x43
#define GYRO_XOUT_L 0x44
#define GYRO_YOUT_H 0x45
#define GYRO_YOUT_L 0x46
#define GYRO_ZOUT_H 0x47
#define GYRO_ZOUT_L 0x48
#define TEMP_OUT_H 0x41
#define TEMP_OUT_L 0x42
#define ACCEL_CONFIG 0x1C
#define GYRO_CONFIG 0x1B
#define DLPF_CONFIG 0x1A

// mpu6050 Sample Rate:
#define MPU6050_SAMPLE_RATE 4

// MPU Sensitivity:
#define ACCEL_SENSITIVITY_INPUT 3 // 0, 1, 2, 3
#define ACCEL_SENSITIVITY_OUTPUT (float)32768 // 16384, 8192, 16384, 32768

#define GYRO_SENSITIVITY_INPUT 1 // 0, 1, 2, 3
#define GYRO_SENSITIVITY_OUTPUT (float)65.5 // 131 65.5, 131, 262

// Motor PWM Pins:
#define FRONT_RIGHT_MOTOR_PIN GPIO_
#define FRONT_LEFT_MOTOR_PIN GPIO_
#define BACK_RIGHT_MOTOR_PIN GPIO_
#define BACK_LEFT_MOTOR_PIN GPIO_

// ESP32 to RPi Comms Pins:
#define UART_NUM UART_NUM_1
#define TXD_PIN (GPIO_NUM_17)
#define RXD_PIN (GPIO_NUM_16)
#define BUF_SIZE 1024

// Motor Mixing Algorithm:
// These constants are used in scaling different aspects of the motor mixing algorithm. This is because the drone, while symmetrical biaxially, it is not a perfect square.
// Source: https://www.iforce2d.net/mixercalc
#define K_ROLL 0.83f
#define K_PITCH 1.0f
#define MOTOR_MIXING_MAX 3.83f // This is the max possible value for the algorithm, and used to scale results down from -1 to 1.
// Miscellaneous Constants:
#define PI_FLOAT 3.1415927f
#endif // CONSTANTS_C