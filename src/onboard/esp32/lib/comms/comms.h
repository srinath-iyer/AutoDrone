#ifndef COMMS_H
#define COMMS_H

#include <pose.h>
#include <stdatomic.h>

extern volatile atomic_bool emergency_stop; // A global boolean var that is set to true when /e-stop is received, which then triggers PID if/then to kill motors.
extern QueueHandle_t pose_queue; // Queue for sending pose data to the main task. Used in sensor fusion, pose estimation, etc.
extern QueueHandle_t command_queue; // Queue for sending commands to the main task. Used in main.c to create Pose setpoints.

void uart_init();
static void handle_uart_data(int len, uint8_t *data);
void uart_event_task(void *pvParameters);
void send_state_to_pi(Pose *pose);
void send_to_pi(char *message);

#endif