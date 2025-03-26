#ifndef CONSTANTS_C
#define CONSTANTS_C

#define Kp 1.0
#define Ki 1.0
#define Kd 1.0
#define PWM_MIN 1000
#define PWM_MAX 2000

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

// mpu6050 Sample Rate:
#define MPU6050_SAMPLE_RATE 4

// MPU Sensitivity:
#define ACCEL_SENSITIVITY_INPUT 0 // 1, 2, 3
#define ACCEL_SENSITIVITY_OUTPUT (float)16384 // 8192, 16384, 32768

#define GYRO_SENSITIVITY_INPUT 0 // 1, 2, 3
#define GYRO_SENSITIVITY_OUTPUT (float)131 // 65.5, 131, 262

// Motor PWM Pins:

// ESP32 to RPi Comms Pins:

#endif // CONSTANTS_C