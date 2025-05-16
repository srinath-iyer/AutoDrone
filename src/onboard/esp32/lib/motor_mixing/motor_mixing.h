#ifndef MOTOR_MIXING_H
#define MOTOR_MIXING_H

extern float motor_mixing_results[4];
void motor_mixer(float* motor_mixing_results, float thrust_output, float roll_output, float pitch_output, float yaw_output); // changes the actual locations in memory rather than returning something.


#endif