// Standard Library Imports

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <mpu6050.h>

void app_main(){
    printf("Hello World\n");
    MPU6050 mpu6050;
    init_mpu6050_converted_data(&mpu6050);
    init_mpu6050();
    while(1){
        update_mpu6050_measurements(&mpu6050);
        print_mpu6050_data(&mpu6050);
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

