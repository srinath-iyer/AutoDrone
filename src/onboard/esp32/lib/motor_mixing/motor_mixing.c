#include "motor_mixing.h"
#include "constants.h"

void motor_mixer(float* motor_mixing_results, float thrust_output, float roll_output, float pitch_output, float yaw_output) {
    motor_mixing_results[0] = thrust_output - K_ROLL*roll_output - K_PITCH*pitch_output - yaw_output; // Motor 1
    motor_mixing_results[1] = thrust_output + K_ROLL*roll_output - K_PITCH*pitch_output + yaw_output; // Motor 2
    motor_mixing_results[2] = thrust_output + K_ROLL*roll_output + K_PITCH*pitch_output - yaw_output; // Motor 3
    motor_mixing_results[3] = thrust_output - K_ROLL*roll_output + K_PITCH*pitch_output + yaw_output; // Motor 4   
}