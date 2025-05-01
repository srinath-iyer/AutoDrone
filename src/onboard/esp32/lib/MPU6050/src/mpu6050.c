#include <stdio.h>
#include "constants.h"
#include "driver/i2c.h"
#include "mpu6050.h"
#include "driver/gpio.h"

void init_mpu6050_converted_data(MPU6050 *mpu6050) {
    mpu6050->accel_x = 0.0;
    mpu6050->accel_y = 0.0;
    mpu6050->accel_z = 0.0;
    mpu6050->gyro_x = 0.0;
    mpu6050->gyro_y = 0.0;
    mpu6050->gyro_z = 0.0;
    mpu6050->temp = 0.0;
}

void init_mpu6050() {
    gpio_reset_pin(2);
    gpio_set_direction(2, GPIO_MODE_OUTPUT);
    gpio_set_level(2, 1);
    vTaskDelay(pdMS_TO_TICKS(1000));  // Delay 500 ms
    gpio_set_level(2, 0);
    i2c_master_init();
    wake_up_mpu6050();
    set_sample_rate(MPU6050_SAMPLE_RATE);
}

void wake_up_mpu6050(){
    uint8_t data = 0x00;
    esp_err_t ret = write_mpu6050(PWR_MGMT_1_REG, data);
    if (ret == ESP_OK) {
        printf("MPU6050 woken up successfully.\n");
        gpio_set_level(2, 1);
        vTaskDelay(pdMS_TO_TICKS(100));  // Delay 500 ms
        gpio_set_level(2, 0);
    } else {
        printf("Failed to wake up MPU6050.\n");
    }
}

void i2c_master_init() {
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = ESP32_I2C_MASTER_SDA;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = ESP32_I2C_MASTER_SCL;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = ESP32_I2C_MASTER_FREQ_HZ;
    i2c_param_config(ESP32_I2C_MASTER_NUM, &conf);
    i2c_driver_install(ESP32_I2C_MASTER_NUM, conf.mode, 0, 0, 0);
}

esp_err_t write_mpu6050(uint8_t reg, uint8_t data) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (DEVICE_ADDRESS << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_write_byte(cmd, data, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(ESP32_I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

esp_err_t read_mpu6050(uint8_t reg, uint8_t *data) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (DEVICE_ADDRESS << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (DEVICE_ADDRESS << 1) | I2C_MASTER_READ, true);
    i2c_master_read_byte(cmd, data, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(ESP32_I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

void set_sample_rate(uint8_t rate) {
    rate = MPU6050_SAMPLE_RATE;
    esp_err_t ret = write_mpu6050(SMPLRT_DIV, rate);
    if (ret == ESP_OK) {
        printf("Sample rate set successfully.\n");
    } else {
        printf("Failed to set sample rate.\n");
    }
} 

void set_accel_sensitivity(uint8_t sensitivity) {
    sensitivity = ACCEL_SENSITIVITY_INPUT;
    esp_err_t ret = write_mpu6050(ACCEL_CONFIG, sensitivity);
    if (ret == ESP_OK) {
        printf("Acceleration sensitivity set successfully.\n");
    } else {
        printf("Failed to set Acceleration sensitivity.\n");
    }
}

void set_gyro_sensitivity(uint8_t sensitivity) {
    sensitivity = GYRO_SENSITIVITY_INPUT;
    esp_err_t ret = write_mpu6050(GYRO_CONFIG, sensitivity);
    if (ret == ESP_OK) {
        printf("Gyro sensitivity set successfully.\n");
    } else {
        printf("Failed to set Gyro sensitivity.\n");
    }
}

void update_mpu6050_measurements(MPU6050 *mpu6050) {
    uint8_t data_high, data_low;
    int16_t accel_x, accel_y, accel_z;
    int16_t gyro_x, gyro_y, gyro_z;
    int16_t temp;

    // Read Accelerometer X
    read_mpu6050(0x3B, &data_high);
    read_mpu6050(0x3C, &data_low);
    accel_x = (data_high << 8) | data_low;

    // Read Accelerometer Y
    read_mpu6050(0x3D, &data_high);
    read_mpu6050(0x3E, &data_low);
    accel_y = (data_high << 8) | data_low;

    // Read Accelerometer Z
    read_mpu6050(0x3F, &data_high);
    read_mpu6050(0x40, &data_low);
    accel_z = (data_high << 8) | data_low;

    // Read Temperature
    read_mpu6050(0x41, &data_high);
    read_mpu6050(0x42, &data_low);
    temp = (data_high << 8) | data_low;

    // Read Gyroscope X
    read_mpu6050(0x43, &data_high);
    read_mpu6050(0x44, &data_low);
    gyro_x = (data_high << 8) | data_low;

    // Read Gyroscope Y
    read_mpu6050(0x45, &data_high);
    read_mpu6050(0x46, &data_low);
    gyro_y = (data_high << 8) | data_low;

    // Read Gyroscope Z
    read_mpu6050(0x47, &data_high);
    read_mpu6050(0x48, &data_low);
    gyro_z = (data_high << 8) | data_low;

    mpu6050->accel_x = (float)accel_x / ACCEL_SENSITIVITY_OUTPUT * 9.81;
    mpu6050->accel_y = (float)accel_y / ACCEL_SENSITIVITY_OUTPUT * 9.81;
    mpu6050->accel_z = (float)accel_z / ACCEL_SENSITIVITY_OUTPUT * 9.81;
    mpu6050->gyro_x = (float)gyro_x / GYRO_SENSITIVITY_OUTPUT;
    mpu6050->gyro_y = (float)gyro_y / GYRO_SENSITIVITY_OUTPUT;
    mpu6050->gyro_z = (float)gyro_z / GYRO_SENSITIVITY_OUTPUT;
    mpu6050->temp = (float)temp / 340.0 + 36.53;

}

void print_mpu6050_data(MPU6050 *mpu6050) {
    printf("MPU6050 Data - Accel X: %f, Accel Y: %f, Accel Z: %f, Gyro X: %f, Gyro Y: %f, Gyro Z: %f, Temp: %f\n",
           mpu6050->accel_x, mpu6050->accel_y, mpu6050->accel_z, mpu6050->gyro_x, mpu6050->gyro_y, mpu6050->gyro_z, mpu6050->temp);
}

