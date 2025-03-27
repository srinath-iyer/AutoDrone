// Standard Library Imports

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <mpu6050.h>

MPU6050 mpu6050;

void app_main(){
    init_drone();
    
    xTaskCreate(get_imu_data, "Get IMU Data", 2048, NULL, 8, NULL);
    xTaskCreate(sensor_fusion, "Sensor Fusion", 2048, NULL, 8, NULL);
    xTaskCreate(pid_controllers, "PID Controllers", 2048, NULL, 8, NULL);
    xTaskCreate(motor_control, "Motor Control", 2048, NULL, 10, NULL);
    xTaskCreate(check_safety, "Check Safety", 2048, NULL, 6, NULL);
    xTaskCreate(send_data_to_pi, "Send Data to Pi", 2048, NULL, 6, NULL);
}

void init_drone(){
    // MPU6050:
    init_mpu6050_converted_data(&mpu6050);
    init_mpu6050();
}

void get_imu_data(){
    TickType_t last_wake_time = xTaskGetTickCount();
    update_mpu6050_measurements(&mpu6050);
    // Log or print_mpu6050_data(&mpu6050);
    vTaskDelayUntil(&last_wake_time, 10 / portTICK_PERIOD_MS);
}

void sensor_fusion(){
    TickType_t last_wake_time = xTaskGetTickCount();
}

void pid_controllers(){
    TickType_t last_wake_time = xTaskGetTickCount();
}

void motor_control(){
    TickType_t last_wake_time = xTaskGetTickCount();
}

void check_safety(){
    TickType_t last_wake_time = xTaskGetTickCount();
}

void send_data_to_pi(){
    TickType_t last_wake_time = xTaskGetTickCount();
}