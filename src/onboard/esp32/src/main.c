// // Standard Library Imports

// #include <stdio.h>
// #include <math.h>
// #include "freertos/FreeRTOS.h"
// #include "freertos/task.h"

// // Custom Imports
// #include <mpu6050.h>
// #include <pid_controller.h>
// #include <pose.h>
// #include <sensor_fusion.h>
// #include "constants.h"
// #include <safeties.h>


// // Global Objects:
// MPU6050 mpu6050;
// PIDController thrust_pid, yaw_pid, roll_pid, pitch_pid, position_roll_pid, position_pitch_pid;
// Pose pose; // Current pose of the drone.

// // Forward Declarations:
// void sensors_and_controls();
// void check_safety();
// void send_pose_to_pi();
// void correct_pose();
// void check_for_enable();
// void init_drone();
// float thrust_error, yaw_error, roll_error, pitch_error, position_roll_pid_error, position_pitch_pid_error; // Errors for PID controllers.
// Pose setpoint; // Setpoint for the drone.


// // State:
// bool is_enabled = false;

// void app_main(){
//     while(!is_enabled){
//         check_for_enable();
//         vTaskDelay(1000 / portTICK_PERIOD_MS); // Wait for 1 second before checking again.
//     }

//     init_drone();
//     xTaskCreate(sensors_and_controls, "Sensors and Controls", 2048, NULL, 10, NULL);
//     xTaskCreate(check_safety, "Check Safety", 2048, NULL, 7, NULL);
//     xTaskCreate(send_pose_to_pi, "Send Data to Pi", 2048, NULL, 9, NULL);
//     xTaskCreate(correct_pose, "Correct Pose", 2048, NULL, 8, NULL);
// }

// void init_drone(){
//     // MPU6050:
//     init_mpu6050_converted_data(&mpu6050);
//     init_mpu6050();

//     // PID:
//     init_pid_controller(&thrust_pid, THRUST_KP, THRUST_KI, THRUST_KD);
//     init_pid_controller(&yaw_pid, YAW_KP, YAW_KI, YAW_KD);
//     init_pid_controller(&roll_pid, ROLL_KP, ROLL_KI, ROLL_KD);
//     init_pid_controller(&pitch_pid, PITCH_KP, PITCH_KI, PITCH_KD);
//     init_pid_controller(&position_roll_pid, POSITION_ROLL_KP, POSITION_ROLL_KI, POSITION_ROLL_KD);
//     init_pid_controller(&position_pitch_pid, POSITION_PITCH_KP, POSITION_PITCH_KI, POSITION_PITCH_KD);

//     // Errors:
//     thrust_error = 0.0;
//     yaw_error = 0.0;
//     roll_error = 0.0;
//     pitch_error = 0.0;
//     position_roll_pid_error = 0.0;
//     position_pitch_pid_error = 0.0;
// }

// void sensors_and_controls(){
//     TickType_t last_wake_time = xTaskGetTickCount();
//     update_mpu6050_measurements(&mpu6050);
//     // Add sensor fusion and update pose function call here
    
//     float cos_yaw = cosf(pose.yaw * PI_FLOAT / 180.0); // Convert yaw to radians for calculations.
//     float sin_yaw = sinf(pose.yaw * PI_FLOAT / 180.0); // Convert yaw to radians for calculations.
//     // PID loops:

//     // For position PID, we first convert to local drone coordinates to remove yaw effects. Then we calculate the error.
//     Pose local_pose_error = get_local_error_to_setpoint(&pose, &setpoint, cos_yaw, sin_yaw); 

//     // Provide position errors, output is roll and pitch angles for the drone.
//     setpoint.roll = calculate_pid(&position_roll_pid, local_pose_error.x);
//     setpoint.pitch = calculate_pid(&position_pitch_pid, local_pose_error.y);
    
//     float yaw_error = 0; // Needs to have some trig in it.
//     float thrust_output = calculate_pid(&thrust_pid, setpoint.z-pose.z); // Thrust error is in the z direction.
//     float yaw_output = calculate_pid(&yaw_pid, yaw_error); // Yaw error is thr angle between the 
//     float roll_output = calculate_pid(&roll_pid, setpoint.roll-pose.roll);
//     float pitch_error = calculate_pid(&pitch_pid, setpoint.pitch-pose.pitch);
    
//     // Motor Mixing Algo:

//     // Motor Control:
    
//     // Log or print_mpu6050_data(&mpu6050);
//     vTaskDelayUntil(&last_wake_time, 2 / portTICK_PERIOD_MS); // Trying 500 Hz.
// }

// void pid_controllers(){
//     TickType_t last_wake_time = xTaskGetTickCount();
// }

// void motor_control(){
//     TickType_t last_wake_time = xTaskGetTickCount();
// }

// void check_safety(){
//     TickType_t last_wake_time = xTaskGetTickCount();
// }

// void send_pose_to_pi(){
//     TickType_t last_wake_time = xTaskGetTickCount();
// }

// void correct_pose(){
//     // This function gets the pose from the RPi via the Camera, and then overrides the Pose from IMU Sensor Fusion.
//     // This is done to create a more accurate pose while keeping the ESP32 lightweight.
//     TickType_t last_wake_time = xTaskGetTickCount();
// }

// void check_for_enable(){
//     // This function checks for the enable signal from the RPi, which allows all other tasks to be launched. Enable is called by the user running the enable.py script on the RPi.
//     is_enabled = true; // This is a placeholder. In the future, this will be replaced with a check for the enable signal from the RPi. Basically just a High or Low signal on a GPIO pin.
// }

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/mcpwm.h"
#include "esp_log.h"

#define TAG "MAIN"

// GPIOs for ESC signal lines
#define MOTOR1_GPIO 18
#define MOTOR2_GPIO 19
#define MOTOR3_GPIO 21
#define MOTOR4_GPIO 22

void init_motor_pwm()
{
    // Initialize GPIOs for MCPWM outputs
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, MOTOR1_GPIO);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, MOTOR2_GPIO);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM1A, MOTOR3_GPIO);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM1B, MOTOR4_GPIO);

    // Basic config for each PWM timer
    mcpwm_config_t pwm_config = {
        .frequency = 50,  // 50Hz for typical ESCs (20ms period)
        .cmpr_a = 0.0,
        .cmpr_b = 0.0,
        .counter_mode = MCPWM_UP_COUNTER,
        .duty_mode = MCPWM_DUTY_MODE_0, // Active high
    };

    // Init two timers (2 channels per timer)
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_1, &pwm_config);
}

// Helper to convert microseconds to % duty for 50Hz (20ms)
float us_to_duty_percent(uint32_t us)
{
    return ((float)us / 20000.0f) * 100.0f;
}

// Set PWM for a motor
void set_motor_us(mcpwm_unit_t unit, mcpwm_timer_t timer, mcpwm_operator_t op, uint32_t us)
{
    float duty = us_to_duty_percent(us);
    mcpwm_set_duty(unit, timer, op, duty);
    mcpwm_set_duty_type(unit, timer, op, MCPWM_DUTY_MODE_0);
    ESP_LOGI(TAG, "Motor [%d:%d:%d] set to %.2f%% (%.0f us)", unit, timer, op, duty, (float)us);
}

void app_main()
{
    printf("Hello from ESP32 (printf)\n");

    ESP_LOGI(TAG, "Initializing motors...");
    init_motor_pwm();

    // Let ESCs boot up (usually 1–2 seconds)
    vTaskDelay(pdMS_TO_TICKS(10000));

    // Spin up motors to 10% (e.g., ~1100us)
    uint32_t throttle_us = 1100;

    set_motor_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, throttle_us); // MOTOR1
    set_motor_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, throttle_us); // MOTOR2
    set_motor_us(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, throttle_us); // MOTOR3
    set_motor_us(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_B, throttle_us); // MOTOR4

    ESP_LOGI(TAG, "Motors spinning at ~10%% throttle.");
}
