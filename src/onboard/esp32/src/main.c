// Standard Library Imports

#include <stdio.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"

// Custom Imports
#include <mpu6050.h>
#include <pid_controller.h>
#include <pose.h>
#include <sensor_fusion.h>
#include "constants.h"
#include <safeties.h>


// Global Objects:
MPU6050 mpu6050;
PIDController thrust_pid, yaw_pid, roll_pid, pitch_pid, position_roll_pid, position_pitch_pid;
Pose pose; // Current pose of the drone.

// Forward Declarations:
void sensors_and_controls();
void check_safety();
void send_pose_to_pi();
void correct_pose();
void check_for_enable();
void init_drone();
float thrust_error, yaw_error, roll_error, pitch_error, position_roll_pid_error, position_pitch_pid_error; // Errors for PID controllers.
Pose setpoint; // Setpoint for the drone.


// State:
bool is_enabled = false;

void app_main(){
    while(!is_enabled){
        check_for_enable();
        vTaskDelay(1000 / portTICK_PERIOD_MS); // Wait for 1 second before checking again.
    }

    init_drone();
    xTaskCreate(sensors_and_controls, "Sensors and Controls", 2048, NULL, 10, NULL);
    xTaskCreate(check_safety, "Check Safety", 2048, NULL, 7, NULL);
    xTaskCreate(send_pose_to_pi, "Send Data to Pi", 2048, NULL, 9, NULL);
    xTaskCreate(correct_pose, "Correct Pose", 2048, NULL, 8, NULL);
    printf("Tasks created");
}

void init_drone(){
    // MPU6050:
    init_mpu6050_converted_data(&mpu6050);
    printf("MPU6050 date intialized!\n");
    init_mpu6050();

    // PID:
    init_pid_controller(&thrust_pid, THRUST_KP, THRUST_KI, THRUST_KD);
    init_pid_controller(&yaw_pid, YAW_KP, YAW_KI, YAW_KD);
    init_pid_controller(&roll_pid, ROLL_KP, ROLL_KI, ROLL_KD);
    init_pid_controller(&pitch_pid, PITCH_KP, PITCH_KI, PITCH_KD);
    init_pid_controller(&position_roll_pid, POSITION_ROLL_KP, POSITION_ROLL_KI, POSITION_ROLL_KD);
    init_pid_controller(&position_pitch_pid, POSITION_PITCH_KP, POSITION_PITCH_KI, POSITION_PITCH_KD);

    // Errors:
    thrust_error = 0.0;
    yaw_error = 0.0;
    roll_error = 0.0;
    pitch_error = 0.0;
    position_roll_pid_error = 0.0;
    position_pitch_pid_error = 0.0;

}

void sensors_and_controls(){
    printf("Sensors and Controls Task Started!\n");
    while(true){
        TickType_t last_wake_time = xTaskGetTickCount();
        mpu6050_read_all(&mpu6050);
        print_mpu6050_data(&mpu6050); // Print the data to the console for debugging.
        // Add sensor fusion and update pose function call here
        
        float cos_yaw = cosf(pose.yaw * PI_FLOAT / 180.0); // Convert yaw to radians for calculations.
        float sin_yaw = sinf(pose.yaw * PI_FLOAT / 180.0); // Convert yaw to radians for calculations.
        // PID loops:

        // For position PID, we first convert to local drone coordinates to remove yaw effects. Then we calculate the error.
        Pose local_pose_error = get_local_error_to_setpoint(&pose, &setpoint, cos_yaw, sin_yaw); 

        // Provide position errors, output is roll and pitch angles for the drone.
        setpoint.roll = calculate_pid(&position_roll_pid, local_pose_error.x);
        setpoint.pitch = calculate_pid(&position_pitch_pid, local_pose_error.y);
        
        float yaw_error = 0; // Needs to have some trig in it.
        float thrust_output = calculate_pid(&thrust_pid, setpoint.z-pose.z); // Thrust error is in the z direction.
        float yaw_output = calculate_pid(&yaw_pid, yaw_error); // Yaw error is thr angle between the 
        float roll_output = calculate_pid(&roll_pid, setpoint.roll-pose.roll);
        float pitch_error = calculate_pid(&pitch_pid, setpoint.pitch-pose.pitch);
        
        // Motor Mixing Algo:

        // Motor Control:
        
        // Log or print_mpu6050_data(&mpu6050);
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(20)); // Trying 50 Hz.
    }
}

void pid_controllers(){
    while(true){
    TickType_t last_wake_time = xTaskGetTickCount();
    vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(200));
    }
}

void motor_control(){
    while(true){
        TickType_t last_wake_time = xTaskGetTickCount();
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(200));
        }
}

void check_safety(){
    while(true){
        TickType_t last_wake_time = xTaskGetTickCount();
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(200));
        }
    }

void send_pose_to_pi(){
    while(true){
        TickType_t last_wake_time = xTaskGetTickCount();
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(200));
        }
}

void correct_pose(){
    // This function gets the pose from the RPi via the Camera, and then overrides the Pose from IMU Sensor Fusion.
    // This is done to create a more accurate pose while keeping the ESP32 lightweight.
    while(true){
        TickType_t last_wake_time = xTaskGetTickCount();
        vTaskDelayUntil(&last_wake_time, 2 / portTICK_PERIOD_MS);
        }
}

void check_for_enable(){
    // This function checks for the enable signal from the RPi, which allows all other tasks to be launched. Enable is called by the user running the enable.py script on the RPi.
    is_enabled = true; // This is a placeholder. In the future, this will be replaced with a check for the enable signal from the RPi. Basically just a High or Low signal on a GPIO pin.
}

// #include <stdio.h>
// #include <stdint.h>
// #include "freertos/FreeRTOS.h"
// #include "freertos/task.h"
// #include "driver/i2c.h"
// #include "esp_log.h"
// #include "esp_err.h"

// #define I2C_MASTER_SCL_IO           22          // Set your SCL pin
// #define I2C_MASTER_SDA_IO           21          // Set your SDA pin
// #define I2C_MASTER_NUM              I2C_NUM_0   // I2C port number
// #define I2C_MASTER_FREQ_HZ          400000      // I2C clock frequency
// #define MPU6050_ADDR                0x68        // MPU6050 device address
// #define MPU6050_PWR_MGMT_1_REG      0x6B
// #define MPU6050_ACCEL_XOUT_H        0x3B

// void i2c_master_init() {
//     i2c_config_t conf = {
//         .mode = I2C_MODE_MASTER,
//         .sda_io_num = I2C_MASTER_SDA_IO,
//         .scl_io_num = I2C_MASTER_SCL_IO,
//         .sda_pullup_en = GPIO_PULLUP_ENABLE,
//         .scl_pullup_en = GPIO_PULLUP_ENABLE,
//         .master.clk_speed = I2C_MASTER_FREQ_HZ,
//     };
//     ESP_ERROR_CHECK(i2c_param_config(I2C_MASTER_NUM, &conf));
//     ESP_ERROR_CHECK(i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0));
// }

// esp_err_t mpu6050_write_byte(uint8_t reg_addr, uint8_t data) {
//     i2c_cmd_handle_t cmd = i2c_cmd_link_create();
//     i2c_master_start(cmd);
//     i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | I2C_MASTER_WRITE, true);
//     i2c_master_write_byte(cmd, reg_addr, true);
//     i2c_master_write_byte(cmd, data, true);
//     i2c_master_stop(cmd);
//     esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(1000));
//     i2c_cmd_link_delete(cmd);
//     return ret;
// }

// esp_err_t mpu6050_read_bytes(uint8_t reg_addr, uint8_t *data, size_t length) {
//     i2c_cmd_handle_t cmd = i2c_cmd_link_create();
//     i2c_master_start(cmd);
//     i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | I2C_MASTER_WRITE, true);
//     i2c_master_write_byte(cmd, reg_addr, true);
//     i2c_master_start(cmd);
//     i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | I2C_MASTER_READ, true);
//     if (length > 1) {
//         i2c_master_read(cmd, data, length - 1, I2C_MASTER_ACK);
//     }
//     i2c_master_read_byte(cmd, data + length - 1, I2C_MASTER_NACK);
//     i2c_master_stop(cmd);
//     esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(1000));
//     i2c_cmd_link_delete(cmd);
//     return ret;
// }

// void mpu6050_init() {
//     ESP_ERROR_CHECK(mpu6050_write_byte(MPU6050_PWR_MGMT_1_REG, 0x00)); // Wake up
// }

// void mpu6050_read_accel(int16_t *accel_x, int16_t *accel_y, int16_t *accel_z) {
//     uint8_t data[6];
//     ESP_ERROR_CHECK(mpu6050_read_bytes(MPU6050_ACCEL_XOUT_H, data, 6));
//     *accel_x = (int16_t)(data[0] << 8 | data[1])/ ACCEL_SENSITIVITY_OUTPUT * 9.81;
//     *accel_y = (int16_t)(data[2] << 8 | data[3])/ ACCEL_SENSITIVITY_OUTPUT * 9.81;
//     *accel_z = (int16_t)(data[4] << 8 | data[5])/ ACCEL_SENSITIVITY_OUTPUT * 9.81;
// }

// void app_main() {
//     i2c_master_init();
//     mpu6050_init();
//     printf("MPU6050 Initialized!\n");

//     while (1) {
//         int16_t ax, ay, az;
//         mpu6050_read_accel(&ax, &ay, &az);
//         printf("Accel X: %d, Y: %d, Z: %d\n", ax, ay, az);
//         vTaskDelay(pdMS_TO_TICKS(500)); // 500 ms delay
//     }
// }
