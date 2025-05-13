#include <stdio.h>
#include "pid_controller.h"

#include <constants.h>

void init_pid_controller(PIDController* pid_controller, float KP, float KI, float KD){
    pid_controller->KP = KP;
    pid_controller->KI = KI;
    pid_controller->KD = KD;
    pid_controller->error = 0;
    pid_controller->integral = 0;
    pid_controller->derivative = 0;
    pid_controller->previous_error = 0;
    pid_controller->dt = 0;
    pid_controller->min_ouput = MIN_PULSE_US;
    pid_controller->max_output = MAX_PULSE_US;
}

float calculate_pid(PIDController* pid_controller, float error){
    pid_controller->error = error;
    pid_controller->integral += pid_controller->error * pid_controller->dt;
    pid_controller->derivative = (pid_controller->error - pid_controller->previous_error) / pid_controller->dt;
    pid_controller->previous_error = pid_controller->error;

    float output = pid_controller->KP * pid_controller->error + pid_controller->KI * pid_controller->integral + pid_controller->KD * pid_controller->derivative;
    if (output > pid_controller->max_output){
        output = pid_controller->max_output;
    } else if (output < pid_controller->min_ouput){
        output = pid_controller->min_ouput;
    }

    return output;
}