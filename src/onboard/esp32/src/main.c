// #include "driver/uart.h"
// #include "esp_log.h"
// #include "string.h"
// #include "driver/gpio.h"

// #define UART_NUM UART_NUM_1
// #define TXD_PIN (GPIO_NUM_17)
// #define RXD_PIN (GPIO_NUM_16)
// #define BUF_SIZE 1024

// void app_main() {
//         // GPIO config structure
//     gpio_config_t io_conf = {
//         .pin_bit_mask = (1ULL << GPIO_NUM_2),  // Bit mask for GPIO
//         .mode = GPIO_MODE_OUTPUT,            // Output mode
//         .pull_up_en = GPIO_PULLUP_DISABLE,   // No pull-up
//         .pull_down_en = GPIO_PULLDOWN_DISABLE, // No pull-down
//         .intr_type = GPIO_INTR_DISABLE       // No interrupt
//     };

//     // Apply configuration
//     gpio_config(&io_conf);
//     const char *TAG = "UART_TEST";

//     const uart_config_t uart_config = {
//         .baud_rate = 115200,
//         .data_bits = UART_DATA_8_BITS,
//         .parity    = UART_PARITY_DISABLE,
//         .stop_bits = UART_STOP_BITS_1,
//         .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
//     };

//     uart_driver_install(UART_NUM, BUF_SIZE, 0, 0, NULL, 0);
//     uart_param_config(UART_NUM, &uart_config);
//     uart_set_pin(UART_NUM, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

//     while (1) {
//         char message[] = "Hello from ESP32\n";
//         uart_write_bytes(UART_NUM, message, strlen(message));

//         uint8_t data[BUF_SIZE];
//         int len = uart_read_bytes(UART_NUM, data, BUF_SIZE, 100 / portTICK_PERIOD_MS);
//         if (len > 0) {
//             gpio_set_level(GPIO_NUM_2, 1); // Set GPIO pin high
//             data[len] = '\0';
//             ESP_LOGI(TAG, "Received: %s", (char *)data);
//             vTaskDelay(1000 / portTICK_PERIOD_MS); // Delay for 1 second
//             gpio_set_level(GPIO_NUM_2, 0); // Set GPIO pin low
//         }

//         vTaskDelay(pdMS_TO_TICKS(2000));
//     }
// }

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
#include <comms.h>
#include <motor_driver.h>
#include <motor_mixing.h>


// Global Objects:
MPU6050 mpu6050;
Pose pose = {0};
Pose setpoint = {0};
PIDController thrust_pid = {0}, yaw_pid = {0}, roll_pid = {0}, pitch_pid = {0}, position_roll_pid = {0}, position_pitch_pid = {0};
float motor_mixing_results[4] = {0};
SemaphoreHandle_t uart_mutex = NULL;

// Forward Declarations:
void sensors_and_controls();
void send_pose_to_pi();
void correct_pose();
void check_for_enable();
void init_drone();
void motor_control();
float thrust_error, yaw_error, roll_error, pitch_error, position_roll_pid_error, position_pitch_pid_error; // Errors for PID controllers.


// State:
bool is_enabled = false;

void app_main(){
    while(!is_enabled){
        check_for_enable();
        vTaskDelay(1000 / portTICK_PERIOD_MS); // Wait for 1 second before checking again.
    }

    init_drone();
    xTaskCreatePinnedToCore(uart_event_task, "UART Task", 4096, NULL, 10, NULL, 0);
    xTaskCreate(sensors_and_controls, "Sensors and Controls", 2048, NULL, 10, NULL);
    xTaskCreate(send_pose_to_pi, "Send Data to Pi", 2048, NULL, 9, NULL);
    xTaskCreate(motor_control, "Motor Control", 2048, NULL, 10, NULL);
}
void init_drone(){
    //initialize motors
    init_motor();

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

    // Pose:
    init_pose(&setpoint, 0, 0, 0, 0, 0, 0, 0); // Setpoint is initialized to the origin.
    init_pose(&pose, 0, 0, 0, 0, 0, 0, 0); // Pose is initialized to the origin.

    // UART Comms:
    uart_init();

    // Errors:
    thrust_error = 0.0;
    yaw_error = 0.0;
    roll_error = 0.0;
    pitch_error = 0.0;
    position_roll_pid_error = 0.0;
    position_pitch_pid_error = 0.0;

    // Motor mixing:
    motor_mixing_results[0] = 0.0;
    motor_mixing_results[1] = 0.0;
    motor_mixing_results[2] = 0.0;
    motor_mixing_results[3] = 0.0;
    printf("drone initialized\n");
    send_to_pi("ESP32 says: drone initialized");
}

void sensors_and_controls(){ //this one
    while(1){
    TickType_t last_wake_time = xTaskGetTickCount();
    mpu6050_read_all(&mpu6050);
    // print_mpu6050_data(&mpu6050);
    // Add sensor fusion and update pose function call here
    
    float cos_yaw = cosf(pose.yaw * PI_FLOAT / 180.0); // Convert yaw to radians for calculations.
    float sin_yaw = sinf(pose.yaw * PI_FLOAT / 180.0); // Convert yaw to radians for calculations.
    // PID loops:

    // For position PID, we first convert to local drone coordinates to remove yaw effects. Then we calculate the error.
    Pose local_pose_error = get_local_error_to_setpoint(&pose, &setpoint, cos_yaw, sin_yaw); 

    // Provide position errors, output is roll and pitch angles for the drone.
    setpoint.roll = calculate_pid(&position_roll_pid, local_pose_error.x);
    setpoint.pitch = calculate_pid(&position_pitch_pid, local_pose_error.y);
    
    setpoint.yaw = atan2f(setpoint.y-pose.y, setpoint.x-pose.x); // Needs to have some trig in it.
    float yaw_error = setpoint.yaw - pose.yaw; // Yaw error is the difference between the setpoint and the current yaw.
    float thrust_output = calculate_pid(&thrust_pid, setpoint.z-pose.z); // Thrust error is in the z direction.
    float yaw_output = calculate_pid(&yaw_pid, yaw_error); // Yaw error is thr angle between the 
    float roll_output = calculate_pid(&roll_pid, setpoint.roll-pose.roll);
    float pitch_output = calculate_pid(&pitch_pid, setpoint.pitch-pose.pitch);

    //PIDs
    
    // Motor Mixing Algo:
    

    // Motor Control:
    // set_motor_speed(1, motor1_pwm);
    // set_motor_speed(2, motor2_pwm);
    // set_motor_speed(3, motor3_pwm);
    // set_motor_speed(4, motor4_pwm);
    
    // Log or print_mpu6050_data(&mpu6050);
    //send mpu data to rpi

    vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(4)); // Trying 50 Hz.
    }
}

void pid_controllers(){
    TickType_t last_wake_time = xTaskGetTickCount();
    while (1) {
        // Your code to send pose to Pi here
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(100)); // e.g., 10 Hz
    }
}

void motor_control(){ //this one

    TickType_t last_wake_time = xTaskGetTickCount();
    while (1) {
        // Your code to send pose to Pi here
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(100)); // e.g., 10 Hz
    }
}

void check_safety(){
     TickType_t last_wake_time = xTaskGetTickCount();
    while (1) {
        // Your code to send pose to Pi here
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(100)); // e.g., 10 Hz
    }
}

void send_pose_to_pi() {
    TickType_t last_wake_time = xTaskGetTickCount();
    while (1) {
        // Your code to send pose to Pi here
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(100)); // e.g., 10 Hz
    }
}

void check_for_enable(){
    // This function checks for the enable signal from the RPi, which allows all other tasks to be launched. Enable is called by the user running the enable.py script on the RPi.
    comms_work = true; // Reset comms_work to false.
    if(comms_work){
        is_enabled = true;
    }
}