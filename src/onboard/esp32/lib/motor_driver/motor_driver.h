#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#include <constants.h>

void init_motor(void);
void set_motor_speed(int motor, uint32_t pulse_us);
static uint32_t pulse_us_to_duty(uint32_t pulse_us);

#endif