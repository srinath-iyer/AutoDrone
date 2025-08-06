#ifndef POSE_H
#define POSE_H

#include <stdint.h>

typedef struct {
    uint32_t timestamp; // Timestamp in microseconds. Note: times out every 71 minutes.
    float x;
    float y;
    float z;
    float v_x;
    float v_y;
    float v_z;
    float roll;
    float pitch;
    float yaw;
} Pose;

void init_pose(Pose *pose, uint32_t timestamp, float x, float y, float z, float v_x, float v_y, float v_z, float roll, float pitch, float yaw);
void update_pose(Pose *pose, uint32_t updated_timestamp, float dx, float dy, float dz, float droll, float dpitch, float dyaw, float dv_x, float dv_y, float dv_z);
Pose get_local_error_to_setpoint(Pose *global, Pose *setpoint, float cos_yaw, float sin_yaw);
void print_pose(Pose *pose);

extern Pose pose; // Global drone pose.
extern Pose setpoint; // Global setpoint pose.
#endif // POSE_H
