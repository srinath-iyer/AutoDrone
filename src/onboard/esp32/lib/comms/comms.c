#include <stdio.h>
#include "driver/uart.h"
#include "freertos/queue.h"
#include "freertos/ringbuf.h"
#include <constants.h>
#include "freertos/task.h"
#include "esp_log.h"
#include <string.h>
#include <stdatomic.h>

#include <pose.h>
#include <comms.h>

#define TAG "UART_COMM"
#define UART_EVENT_QUEUE_SIZE 10

QueueHandle_t uart_event_queue;
QueueHandle_t command_queue; // Global command queue.
QueueHandle_t pose_queue; // Global pose queue.
RingbufHandle_t uart_ringbuf;
volatile atomic_bool emergency_stop;

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

    // Set UART pins (using default pins if NULL)
    uart_set_pin(UART_NUM, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    // Install driver with RX and TX buffers + event queue
    uart_driver_install(UART_NUM, BUF_SIZE, BUF_SIZE, UART_EVENT_QUEUE_SIZE, &uart_event_queue, 0);

    // Initialize global queues and e_stop flag:
    emergency_stop = false;
    command_queue = xQueueCreate(10, sizeof(char *)); // Queue for commands
    pose_queue = xQueueCreate(10, sizeof(Pose)); // Queue for pose data

    ESP_LOGI(TAG, "UART initialized");
}

static void handle_uart_data(int len, uint8_t *data)
{
    data[len] = '\0';  // Null-terminate
    ESP_LOGI(TAG, "Received: %s", (char *)data);
    char *str = (char *)data;

    if (strncmp(str, "/e-stop", 7) == 0) {
        ESP_LOGW(TAG, "EMERGENCY STOP TRIGGERED!");
        emergency_stop = true;  // Set the emergency stop flag
        // TODO: Stop motors, trigger fail-safe, etc.
        return;
    }

    else if (strncmp(str, "S:", 2) == 0) {
        float t, x, y, z, yaw, pitch, roll;
        if (sscanf(str + 2, "%u,%f,%f,%f,%f,%f,%f", &t, &x, &y, &z, &yaw, &pitch, &roll) == 7) {
            ESP_LOGI(TAG, "State Update -> t: %u, x: %.2f, y: %.2f, z: %.2f, yaw: %.2f, pitch: %.2f, roll: %.2f",
                        t, x, y, z, yaw, pitch, roll);
            Pose new_pose;
            init_pose(&new_pose, t, x, y, z, roll, pitch, yaw);
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
            Pose new_setpoint;
            init_pose(&new_setpoint, 0, x, y, z, 0, 0, 0);
            xQueueSend(command_queue, &new_setpoint, pdMS_TO_TICKS(2)); // Send the new setpoint to the command queue
            
        } else {
            ESP_LOGW(TAG, "Malformed /move/ command: %s", str);
        }
        return;
    }

    else if (strncmp(str, "/test_signal", 11) == 0) {
        ESP_LOGI(TAG, "Test signal received");
        send_to_pi("/test_signal_received");
    }

}

void uart_event_task(void *pvParameters)
{
    uart_event_t event;
    uint8_t data[128];

    while (1) {
        // Wait for UART event
        if (xQueueReceive(uart_event_queue, (void *)&event, portMAX_DELAY)) {
            switch (event.type) {
                case UART_DATA:
                    // Read data from buffer
                    int len = uart_read_bytes(UART_NUM, data, event.size, portMAX_DELAY);
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

void send_state_to_pi(Pose pose, uint_32_t timestamp) { // Timestamp uses esp_timer_get_time(). In the future, state will also include voltage.
    char state_message[] = "S";
    sprintf(state_message, "S:%u:%f,%f,%f,%f,%f,%f", timestamp, pose.x, pose.y, pose.z, pose.roll, pose.pitch, pose.yaw);
    uart_write_bytes(UART_NUM, state_message, strlen(state_message));
    printf("Sending pose to Raspberry Pi: ");
    print_pose(pose);
}

void send_to_pi(char *message) {
    uart_write_bytes(UART_NUM, message, strlen(message));
    ESP_LOGI(TAG, "Sending to Raspberry Pi: %s", message);
}



