#include "bmp390.h"
#include <stdio.h>
#include "driver/i2c.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_timer.h"
#include "driver/gpio.h"
#include "constants.h"
#include "math.h"

void init_bmp390(BMP390 *bmp390, BMP390_Calib *calib) {
    bmp390->temperature = 0.0f;
    bmp390->pressure = 0.0f;
    bmp390->altitude = 0.0f;
    bmp390->timestamp = 0;
    bmp390->new_reading = false;
    
    vTaskDelay(pdMS_TO_TICKS(10)); // Wait for sensor power-up
    
    // Read and verify chip ID
    uint8_t chip_id = 0;
    bmp390_read_bytes(0x00, &chip_id, 1);
    if (chip_id == 0x60) {
        printf("BMP390_INIT:0x%02X\n", chip_id);
    } else {
        printf("BMP390_ERROR:0x%02X\n", chip_id);
        return;
    }
    
    wake_up_bmp390();
    bmp390_read_calib(calib);
}

void wake_up_bmp390(){
    // 1. Soft reset first
    printf("BMP390_RESET:starting\n");
    write_bmp390(0x7E, 0xB6); // CMD register: soft reset
    vTaskDelay(pdMS_TO_TICKS(10)); // Wait for reset
    
    // 2. Set oversampling: OSR_P = x8, OSR_T = x1
    write_bmp390(0x1C, 0x03); // OSR register: bits [2:0] = 011 (8x pressure), bits [5:3] = 000 (1x temp)
    
    // 3. Set output data rate: 50 Hz
    write_bmp390(0x1D, 0x04); // ODR register: 0x04 = 50 Hz
    
    // 4. Enable pressure and temperature, set to NORMAL mode
    write_bmp390(0x1B, 0x33); // PWR_CTRL: bits [5:4] = 11 (NORMAL), bit [1] = 1 (temp_en), bit [0] = 1 (press_en)
    
    vTaskDelay(pdMS_TO_TICKS(40)); // Wait for first measurement (40ms for forced mode)
    
    printf("BMP390_READY:1\n");
}

void write_bmp390(uint8_t reg_addr, uint8_t data){
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (BMP390_I2C_ADDRESS << 1) | I2C_MASTER_WRITE, true); // Assuming BMP390 I2C address is 0x76
    i2c_master_write_byte(cmd, reg_addr, true);
    i2c_master_write_byte(cmd, data, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(ESP32_I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(1000));
    if (ret != ESP_OK) {
        ESP_LOGE("BMP390", "I2C write failed: %s", esp_err_to_name(ret));
    }
    i2c_cmd_link_delete(cmd);
}

// Read multiple bytes from a register
esp_err_t bmp390_read_bytes(uint8_t reg_addr, uint8_t *data, size_t length) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (BMP390_I2C_ADDRESS << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);
    i2c_master_start(cmd); // Repeated START
    i2c_master_write_byte(cmd, (BMP390_I2C_ADDRESS << 1) | I2C_MASTER_READ, true);
    
    // Read all bytes except the last one with ACK
    if (length > 1) {
        i2c_master_read(cmd, data, length - 1, I2C_MASTER_ACK);
    }
    
    // Read the last byte with NACK
    i2c_master_read_byte(cmd, &data[length - 1], I2C_MASTER_NACK);
    
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(ESP32_I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(1000));
    if (ret != ESP_OK) {
        ESP_LOGE("BMP390", "I2C read bytes failed: %s", esp_err_to_name(ret));
    }
    i2c_cmd_link_delete(cmd);
    return ret;
}


void bmp390_read_calib(BMP390_Calib *calib) {
    uint8_t calib_data[21];
    esp_err_t ret = bmp390_read_bytes(0x31, calib_data, 21);
    
    if (ret != ESP_OK) {
        ESP_LOGE("BMP390", "Failed to read calibration data");
        return;
    }
    
    // Extract raw NVM calibration coefficients (little-endian, sign-extended where needed)
    uint16_t nvm_T1 = (uint16_t)(calib_data[0]) | ((uint16_t)(calib_data[1]) << 8);
    uint16_t nvm_T2 = (uint16_t)(calib_data[2]) | ((uint16_t)(calib_data[3]) << 8);
    int8_t nvm_T3 = (int8_t)calib_data[4];
    
    int16_t nvm_P1 = (int16_t)((uint16_t)(calib_data[5]) | ((uint16_t)(calib_data[6]) << 8));
    int16_t nvm_P2 = (int16_t)((uint16_t)(calib_data[7]) | ((uint16_t)(calib_data[8]) << 8));
    int8_t nvm_P3 = (int8_t)calib_data[9];
    int8_t nvm_P4 = (int8_t)calib_data[10];
    uint16_t nvm_P5 = (uint16_t)(calib_data[11]) | ((uint16_t)(calib_data[12]) << 8);
    uint16_t nvm_P6 = (uint16_t)(calib_data[13]) | ((uint16_t)(calib_data[14]) << 8);
    int8_t nvm_P7 = (int8_t)calib_data[15];
    int8_t nvm_P8 = (int8_t)calib_data[16];
    int16_t nvm_P9 = (int16_t)((uint16_t)(calib_data[17]) | ((uint16_t)(calib_data[18]) << 8));
    int8_t nvm_P10 = (int8_t)calib_data[19];
    int8_t nvm_P11 = (int8_t)calib_data[20];
    
    // Apply scaling factors from BMP390 datasheet section 8.4
    calib->par_t1 = (float)nvm_T1 * 256.0f; // 2^-8 = * 256
    calib->par_t2 = (float)nvm_T2 / 1073741824.0f; // 2^30
    calib->par_t3 = (float)nvm_T3 / 281474976710656.0f; // 2^48
    
    calib->par_p1 = ((float)nvm_P1 - 16384.0f) / 1048576.0f; // (x - 2^14) / 2^20
    calib->par_p2 = ((float)nvm_P2 - 16384.0f) / 536870912.0f; // (x - 2^14) / 2^29
    calib->par_p3 = (float)nvm_P3 / 4294967296.0f; // 2^32
    calib->par_p4 = (float)nvm_P4 / 137438953472.0f; // 2^37
    calib->par_p5 = (float)nvm_P5 * 8.0f; // 2^-3 = * 8
    calib->par_p6 = (float)nvm_P6 / 64.0f; // 2^6
    calib->par_p7 = (float)nvm_P7 / 256.0f; // 2^8
    calib->par_p8 = (float)nvm_P8 / 32768.0f; // 2^15
    calib->par_p9 = (float)nvm_P9 / 281474976710656.0f; // 2^48
    calib->par_p10 = (float)nvm_P10 / 281474976710656.0f; // 2^48
    calib->par_p11 = (float)nvm_P11 / 36893488147419103232.0f; // 2^65
    
    printf("BMP390_CALIB:loaded\n");
}

void bmp390_read_all(BMP390 *bmp390, BMP390_Calib *calib) {
    uint8_t data[6];
    esp_err_t ret = bmp390_read_bytes(0x04, data, 6);
    
    if (ret != ESP_OK) {
        ESP_LOGE("BMP390", "Failed to read sensor data");
        return;
    }
    
    // Extract 24-bit values (little-endian)
    uint32_t uncomp_press = (uint32_t)data[0] | ((uint32_t)data[1] << 8) | ((uint32_t)data[2] << 16);
    uint32_t uncomp_temp = (uint32_t)data[3] | ((uint32_t)data[4] << 8) | ((uint32_t)data[5] << 16);
    
    // Temperature compensation (BMP390 datasheet section 8.5)
    float partial_data1 = (float)uncomp_temp - calib->par_t1;
    float partial_data2 = partial_data1 * calib->par_t2;
    float t_lin = partial_data2 + (partial_data1 * partial_data1) * calib->par_t3;
    
    // Pressure compensation (BMP390 datasheet section 8.6)
    float partial_data3, partial_data4;
    float partial_out1, partial_out2;
    
    partial_data1 = calib->par_p6 * t_lin;
    partial_data2 = calib->par_p7 * (t_lin * t_lin);
    partial_data3 = calib->par_p8 * (t_lin * t_lin * t_lin);
    partial_out1 = calib->par_p5 + partial_data1 + partial_data2 + partial_data3;
    
    partial_data1 = calib->par_p2 * t_lin;
    partial_data2 = calib->par_p3 * (t_lin * t_lin);
    partial_data3 = calib->par_p4 * (t_lin * t_lin * t_lin);
    partial_out2 = (float)uncomp_press * (calib->par_p1 + partial_data1 + partial_data2 + partial_data3);
    
    partial_data1 = (float)uncomp_press * (float)uncomp_press;
    partial_data2 = calib->par_p9 + calib->par_p10 * t_lin;
    partial_data3 = partial_data1 * partial_data2;
    partial_data4 = partial_data3 + ((float)uncomp_press * (float)uncomp_press * (float)uncomp_press) * calib->par_p11;
    
    float comp_press = partial_out1 + partial_out2 + partial_data4;
    
    bmp390->temperature = t_lin;
    bmp390->pressure = comp_press / 100.0f; // Convert to hPa
    bmp390->altitude = 44330.0f * (1.0f - powf(bmp390->pressure / 1013.25f, 0.1903f));
    bmp390->timestamp = esp_timer_get_time();
    bmp390->new_reading = !bmp390->new_reading;
//     printf("BMP390_READING:%.2f,%.2f,%.2f\n",
//              bmp390->temperature, bmp390->pressure, bmp390->altitude);
 }