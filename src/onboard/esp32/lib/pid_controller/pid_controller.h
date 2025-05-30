#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

typedef struct{
    float KP;
    float KI;
    float KD;
    float error;
    float integral;
    float derivative;
    float previous_error;
    float dt;
    float min_ouput;
    float max_output;

} PIDController;

void init_pid_controller(PIDController* pid_controller, float KP, float KI, float KD);
float calculate_pid(PIDController* pid_controller, float error);

#endif

