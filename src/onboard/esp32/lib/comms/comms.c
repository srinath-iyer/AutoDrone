#include <stdio.h>
#include "driver/uart.h"
#include "freertos/queue.h"
#include "freertos/ringbuf.h"
#include <constants.h>
#include "freertos/task.h"
#include "esp_log.h"
#include <string.h>
#include <stdatomic.h>
#include "driver/gpio.h"
#include <motor_driver.h>
#include <pose.h>
#include <comms.h>
#include <pid_controller.h>

#define TAG "UART_COMM"
#define UART_EVENT_QUEUE_SIZE 10
#include <mpu6050.h>
QueueHandle_t uart_event_queue;
// QueueHandle_t command_queue; // Global command queue.
QueueHandle_t pose_queue; // Global pose queue.
RingbufHandle_t uart_ringbuf;
volatile atomic_bool emergency_stop;
volatile atomic_bool comms_work = false; // A global boolean var that is set to true when /e-stop is received, which then triggers PID if/then to kill motors.

void uart_init()
{
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };

    // Configure UART parameters
    uart_param_config(UART_NUM, &uart_config);

    uart_set_pin(UART_NUM, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);


    // Install driver with RX and TX buffers + event queue
    uart_driver_install(UART_NUM, BUF_SIZE, BUF_SIZE, UART_EVENT_QUEUE_SIZE, &uart_event_queue, 0);

    // Initialize global queues and e_stop flag:
    emergency_stop = false;
    comms_work = false;
    pose_queue = xQueueCreate(10, sizeof(Pose)); // Queue for pose data
    uart_mutex = xSemaphoreCreateMutex(); // Mutex for UART communication
    ESP_LOGI(TAG, "UART initialized");
}

static void handle_uart_data(int len, uint8_t *data)
{
    data[len] = '\0';  // Null-terminate
    ESP_LOGI(TAG, "Received: %s", (char *)data);
    char *str = (char *)data;
    printf(str);
    if (strncmp(str, "/e-stop", 7) == 0) {
        ESP_LOGW(TAG, "EMERGENCY STOP TRIGGERED!");
        emergency_stop = true;  // Set the emergency stop flag
        return;
    }

    else if (strncmp(str, "/motor-pwm/", 11) == 0){
        int motor_pwm;
        if (sscanf(str + 11, "%d", &motor_pwm) == 1) {
            ESP_LOGI(TAG, "Motor PWM Command -> PWM: %d", motor_pwm);
            xSemaphoreTake(uart_mutex, portMAX_DELAY); // Take the mutex
            // Here you would typically send the motor PWM command to the motor controller
            printf("Speed: %d\n", motor_pwm);
            set_motor_speed(1, motor_pwm);
            set_motor_speed(2, motor_pwm);
            set_motor_speed(3, motor_pwm);
            set_motor_speed(4, motor_pwm);
            xSemaphoreGive(uart_mutex); // Give the mutex
        } else {
            ESP_LOGW(TAG, "Malformed /motor-pwm/ command: %s", str);
        }
    }

    else if (strncmp(str, "S:", 2) == 0) {
        uint32_t t;
        float x, y, z, yaw, pitch, roll;
        if (sscanf(str + 2, "%lu,%f,%f,%f,%f,%f,%f", &t, &x, &y, &z, &yaw, &pitch, &roll) == 7) {
            ESP_LOGI(TAG, "State Update -> t: %lu, x: %.2f, y: %.2f, z: %.2f, yaw: %.2f, pitch: %.2f, roll: %.2f",
                        (unsigned long)t, x, y, z, yaw, pitch, roll);
            Pose new_pose;
            init_pose(&new_pose, t, x, y, z, 0, 0, 0, roll, pitch, yaw);
            xQueueSend(pose_queue, &new_pose, pdMS_TO_TICKS(2)); // Send the new pose/IMU reading to the pose queue
        }
        else {
            ESP_LOGW(TAG, "Malformed state message: %s", str);
        }
        return;
    }

    else if (strncmp(str, "/move/", 6) == 0) {
        float x, y, z;
        if (sscanf(str + 6, "%f,%f,%f", &x, &y, &z) == 3) {
            ESP_LOGI(TAG, "Move Command -> x: %.2f, y: %.2f, z: %.2f", x, y, z);
            xSemaphoreTake(uart_mutex, portMAX_DELAY); // Take the mutex
            init_pose(&setpoint, 0, x, y, z, 0, 0, 0, 0, 0, 0);
            //xQueueSend(command_queue, &new_setpoint, pdMS_TO_TICKS(2)); // Send the new setpoint to the command queue
            xSemaphoreGive(uart_mutex); // Give the mutex
        } else {
            ESP_LOGW(TAG, "Malformed /move/ command: %s", str);
        }
        return;
    }

    else if(strncmp(str, "/pid/thrust/", 12) == 0){
        float thrust_kp, thrust_ki, thrust_kd;
        if (sscanf(str + 12, "%f,%f,%f", &thrust_kp, &thrust_ki, &thrust_kd) == 3) {
            ESP_LOGI(TAG, "Thrust PID -> Kp: %.2f, Ki: %.2f, Kd: %.2f", thrust_kp, thrust_ki, thrust_kd);
            init_pid_controller(&thrust_pid, thrust_kp, thrust_ki, thrust_kd);
        }
    }

    else if(strncmp(str, "/pid/roll/", 11) == 0){
        float roll_kp, roll_ki, roll_kd;
        if (sscanf(str + 11, "%f,%f,%f", &roll_kp, &roll_ki, &roll_kd) == 3) {
            ESP_LOGI(TAG, "Roll PID -> Kp: %.2f, Ki: %.2f, Kd: %.2f", roll_kp, roll_ki, roll_kd);
            init_pid_controller(&roll_pid, roll_kp, roll_ki, roll_kd);
        }
    }

    else if(strncmp(str, "/pid/pitch/", 12) == 0){
        float pitch_kp, pitch_ki, pitch_kd;
        if (sscanf(str + 12, "%f,%f,%f", &pitch_kp, &pitch_ki, &pitch_kd) == 3) {
            ESP_LOGI(TAG, "Pitch PID -> Kp: %.2f, Ki: %.2f, Kd: %.2f", pitch_kp, pitch_ki, pitch_kd);
            init_pid_controller(&pitch_pid, pitch_kp, pitch_ki, pitch_kd);
        }
    }

    else if(strncmp(str, "/pid/yaw/", 10) == 0){
        float yaw_kp, yaw_ki, yaw_kd;
        if (sscanf(str + 10, "%f,%f,%f", &yaw_kp, &yaw_ki, &yaw_kd) == 3) {
            ESP_LOGI(TAG, "Yaw PID -> Kp: %.2f, Ki: %.2f, Kd: %.2f", yaw_kp, yaw_ki, yaw_kd);
            init_pid_controller(&yaw_pid, yaw_kp, yaw_ki, yaw_kd);
        }
    }

    else if (strncmp(str, "/test_signal", 11) == 0) {
        ESP_LOGI(TAG, "Test signal received");
        send_message_to_pi("/test_signal_received\n");
        comms_work = true;
    }

}

void uart_event_task(void *pvParameters)
{
    printf("uart task started");
    uart_event_t event;
    uint8_t data[128];

    while (1) {
        // Wait for UART event
        if (xQueueReceive(uart_event_queue, (void *)&event, portMAX_DELAY)) {
            switch (event.type) {
                case UART_DATA:
                    // Read data from buffer
                    int len = uart_read_bytes(UART_NUM, data, sizeof(data), 0);
                    if (len > 0) {
                        handle_uart_data(len, data);
                    }
                    break;

                case UART_FIFO_OVF:
                    ESP_LOGW(TAG, "HW FIFO overflow");
                    uart_flush_input(UART_NUM);
                    xQueueReset(uart_event_queue);
                    break;

                case UART_BUFFER_FULL:
                    ESP_LOGW(TAG, "Ring buffer full");
                    uart_flush_input(UART_NUM);
                    xQueueReset(uart_event_queue);
                    break;

                case UART_BREAK:
                case UART_PARITY_ERR:
                case UART_FRAME_ERR:
                    ESP_LOGW(TAG, "UART error type: %d", event.type);
                    break;

                default:
                    break;
            }
        }
    }
}

void send_pose_to_pi(Pose* pose, uint32_t timestamp) { // Timestamp uses esp_timer_get_time(). In the future, state will also include voltage.
    char state_message[128];
    sprintf(state_message, "S:%lu:%f,%f,%f,%f,%f,%f\n", (unsigned long)timestamp, pose->x, pose->y, pose->z, pose->roll, pose->pitch, pose->yaw);
    uart_write_bytes(UART_NUM, state_message, strlen(state_message));
    //printf("Sending pose to Raspberry Pi: ");
    //print_pose(pose);
}

void send_imu_to_pi(MPU6050* mpu6050, uint32_t timestamp) {
    char imu_message[128];
    sprintf(imu_message, "%lu,%f,%f,%f,%f,%f,%f\n", (unsigned long)timestamp, mpu6050->accel_x, mpu6050->accel_y, mpu6050->accel_z, mpu6050->gyro_x, mpu6050->gyro_y, mpu6050->gyro_z);
    uart_write_bytes(UART_NUM, imu_message, strlen(imu_message));
    //ESP_LOGI(TAG, "Sending IMU to Raspberry Pi: %s", imu_message);
}

void send_message_to_pi(char *message) {
    uart_write_bytes(UART_NUM, message, strlen(message));
    ESP_LOGI(TAG, "Sending to Raspberry Pi: %s", message);
}



