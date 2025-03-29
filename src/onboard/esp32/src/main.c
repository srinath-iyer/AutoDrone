// Standard Library Imports

#include <stdio.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// Custom Imports
#include <mpu6050.h>
#include <pid_controller.h>
#include <pose.h>
#include <sensor_fusion.h>
#include "constants.h"
#include <safeties.h>
#

// Global Objects:
MPU6050 mpu6050;
PIDController thrust_pid, yaw_pid, roll_pid, pitch_pid, position_roll_pid, position_pitch_pid;
Pose pose; // Current pose of the drone.
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
}

void init_drone(){
    // MPU6050:
    init_mpu6050_converted_data(&mpu6050);
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
    TickType_t last_wake_time = xTaskGetTickCount();
    update_mpu6050_measurements(&mpu6050);
    // Add sensor fusion and update pose function call here
    
    float cos_yaw = cosf(pose.yaw * PI_FLOAT / 180.0); // Convert yaw to radians for calculations.
    float sin_yaw = sinf(pose.yaw * PI_FLOAT / 180.0); // Convert yaw to radians for calculations.
    // PID loops:

    // For position PID, we first convert to local drone coordinates to remove yaw effects. Then we calculate the error.
    Pose local_pose_error = get_local_error_to_setpoint(&pose, &setpoint, cos_yaw, sin_yaw); 

    // Provide position errors, output is roll and pitch angles for the drone.
    float position_roll_output = calculate_pid(&position_roll_pid, local_pose_error.x);
    float position_pitch_output = calculate_pid(&position_pitch_pid, local_pose_error.y);
    //
    float thrust_output = calculate_pid(&thrust_pid, thrust_error);
    float yaw_output = calculate_pid(&yaw_pid, yaw_error);
    float roll_output = calculate_pid(&roll_pid, roll_error);
    float pitch_error = calculate_pid(&pitch_pid, pitch_error);
    
    // Motor Mixing Algo:

    // Motor Control:
    
    // Log or print_mpu6050_data(&mpu6050);
    vTaskDelayUntil(&last_wake_time, 2 / portTICK_PERIOD_MS); // Trying 500 Hz.
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

void send_pose_to_pi(){
    TickType_t last_wake_time = xTaskGetTickCount();
}

void correct_pose(){
    // This function gets the pose from the RPi via the Camera, and then overrides the Pose from IMU Sensor Fusion.
    // This is done to create a more accurate pose while keeping the ESP32 lightweight.
    TickType_t last_wake_time = xTaskGetTickCount();
}

void check_for_enable(){
    // This function checks for the enable signal from the RPi, which allows all other tasks to be launched. Enable is called by the user running the enable.py script on the RPi.
    is_enabled = true; // This is a placeholder. In the future, this will be replaced with a check for the enable signal from the RPi. Basically just a High or Low signal on a GPIO pin.
}