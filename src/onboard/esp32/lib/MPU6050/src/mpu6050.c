#include <stdio.h>
#include "constants.h"
#include "driver/i2c.h"
#include "mpu6050.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_err.h"

#define I2C_MASTER_SCL_IO           22          // Set your SCL pin
#define I2C_MASTER_SDA_IO           21          // Set your SDA pin
#define I2C_MASTER_NUM              I2C_NUM_0   // I2C port number
#define I2C_MASTER_FREQ_HZ          400000      // I2C clock frequency
#define MPU6050_ADDR                0x68        // MPU6050 device address
#define MPU6050_PWR_MGMT_1_REG      0x6B
#define MPU6050_ACCEL_XOUT_H        0x3B


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
    vTaskDelay(pdMS_TO_TICKS(500));  // Delay 500 ms
    i2c_master_init();
    printf("i2c_master_init() done!\n");
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
        i2c_config_t conf = {
            .mode = I2C_MODE_MASTER,
            .sda_io_num = 21,
            .scl_io_num = 22,
            .sda_pullup_en = GPIO_PULLUP_ENABLE,
            .scl_pullup_en = GPIO_PULLUP_ENABLE,
            .master.clk_speed = 400000,
        };
        ESP_ERROR_CHECK(i2c_param_config(ESP32_I2C_MASTER_NUM, &conf));
        ESP_ERROR_CHECK(i2c_driver_install(ESP32_I2C_MASTER_NUM, conf.mode, 0, 0, 0));
    }
    

esp_err_t write_mpu6050(uint8_t reg_addr, uint8_t data) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (DEVICE_ADDRESS << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);
    i2c_master_write_byte(cmd, data, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(ESP32_I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);
    return ret;
}

// esp_err_t read_mpu6050(uint8_t reg_addr, uint8_t *data) {
//     size_t length = 6;
//     i2c_cmd_handle_t cmd = i2c_cmd_link_create();
//     i2c_master_start(cmd);
//     i2c_master_write_byte(cmd, (DEVICE_ADDRESS << 1) | I2C_MASTER_WRITE, true);
//     i2c_master_write_byte(cmd, reg_addr, true);
//     i2c_master_start(cmd);
//     i2c_master_write_byte(cmd, (ESP32_I2C_MASTER_NUM << 1) | I2C_MASTER_READ, true);
//     if (length > 1) {
//         i2c_master_read(cmd, data, length - 1, I2C_MASTER_ACK);
//     }
//     i2c_master_read_byte(cmd, data + length - 1, I2C_MASTER_NACK);
//     i2c_master_stop(cmd);
//     esp_err_t ret = i2c_master_cmd_begin(ESP32_I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(1000));
//     i2c_cmd_link_delete(cmd);
//     return ret;
// }

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
    printf("Raw MPU6050 Data - Accel X: %f, Accel Y: %f, Accel Z: %f, Gyro X: %f, Gyro Y: %f, Gyro Z: %f, Temp: %f\n",
           mpu6050->accel_x, mpu6050->accel_y, mpu6050->accel_z, mpu6050->gyro_x, mpu6050->gyro_y, mpu6050->gyro_z, mpu6050->temp);

}

esp_err_t mpu6050_read_bytes(uint8_t reg_addr, uint8_t *data, size_t length) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | I2C_MASTER_READ, true);
    if (length > 1) {
        i2c_master_read(cmd, data, length - 1, I2C_MASTER_ACK);
    }
    i2c_master_read_byte(cmd, data + length - 1, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);
    return ret;
}

void mpu6050_read_all(MPU6050 *mpu6050)
{
    uint8_t data[14];
    // Read ACCEL_XOUT_H through GYRO_ZOUT_L
    ESP_ERROR_CHECK(mpu6050_read_bytes(0x3B, data, sizeof(data)));

    // Helper to combine two bytes
    #define TO_INT16(h,l)  ((int16_t)(((uint16_t)(h) << 8) | (l)))

    // Raw accel
    int16_t raw_ax = TO_INT16(data[0], data[1]);
    int16_t raw_ay = TO_INT16(data[2], data[3]);
    int16_t raw_az = TO_INT16(data[4], data[5]);
    // Raw temp
    int16_t raw_temp = TO_INT16(data[6], data[7]);
    // Raw gyro
    int16_t raw_gx = TO_INT16(data[8], data[9]);
    int16_t raw_gy = TO_INT16(data[10], data[11]);
    int16_t raw_gz = TO_INT16(data[12], data[13]);

    // Convert to m/s²
    mpu6050->accel_x = (float) (raw_ax / ACCEL_SENSITIVITY_OUTPUT) * 9.81;
    mpu6050->accel_y = (raw_ay / ACCEL_SENSITIVITY_OUTPUT) * 9.81;
    mpu6050->accel_z = (raw_az / ACCEL_SENSITIVITY_OUTPUT) * 9.81;

    // Convert to °C (per datasheet: Temp = raw/340 + 3s6.53)
    mpu6050->temp    = (raw_temp / 340.0f) + 36.53f;

    // Convert to °/s
    mpu6050->gyro_x  = raw_gx / GYRO_SENSITIVITY_OUTPUT;
    mpu6050->gyro_y  = raw_gy / GYRO_SENSITIVITY_OUTPUT;
    mpu6050->gyro_z  = raw_gz / GYRO_SENSITIVITY_OUTPUT;

    #undef TO_INT16
}

void print_mpu6050_data(MPU6050 *mpu6050) {
    // printf("MPU6050 Data - Accel X: %f, Accel Y: %f, Accel Z: %f, Gyro X: %f, Gyro Y: %f, Gyro Z: %f, Temp: %f\n",
    //        mpu6050->accel_x, mpu6050->accel_y, mpu6050->accel_z, mpu6050->gyro_x, mpu6050->gyro_y, mpu6050->gyro_z, mpu6050->temp);

    printf("X: %f, Y: %f, Z: %f\n", mpu6050->accel_x, mpu6050->accel_y, mpu6050->accel_z);
}

