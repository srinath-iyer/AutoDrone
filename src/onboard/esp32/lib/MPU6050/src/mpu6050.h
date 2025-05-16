#ifndef MPU6050_H
#define MPU6050_H

#include <stdio.h>
#include "driver/i2c.h"

typedef struct{
    float accel_x;
    float accel_y;
    float accel_z;
    float gyro_x;
    float gyro_y;
    float gyro_z;
    float temp;
} MPU6050;


// Call these following in main.c
void init_mpu6050_converted_data(MPU6050 *mpu6050); 
void init_mpu6050();

// Do not call the following in main.c:
void wake_up_mpu6050();
void i2c_master_init();
esp_err_t write_mpu6050(uint8_t reg, uint8_t data);
esp_err_t read_mpu6050(uint8_t reg, uint8_t *data); 
void set_sample_rate(uint8_t rate);
void set_accel_sensitivity(uint8_t sensitivity);
void set_gyro_sensitivity(uint8_t sensitivity);
void set_dplf_config(uint8_t config);

// Call these functions multiple times in main.c:
void update_mpu6050_measurements(MPU6050 *mpu6050);
void print_mpu6050_data(MPU6050 *mpu6050);
esp_err_t mpu6050_read_bytes(uint8_t reg_addr, uint8_t *data, size_t length);
void mpu6050_read_all(MPU6050 *mpu6050);

#endif