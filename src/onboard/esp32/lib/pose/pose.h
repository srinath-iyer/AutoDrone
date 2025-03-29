#ifndef POSE_H
#define POSE_H

typedef struct {
    float x;
    float y;
    float z;
    float roll;
    float pitch;
    float yaw;
} Pose;

void init_pose(Pose *pose, float x, float y, float z, float roll, float pitch, float yaw);
void update_pose(Pose *pose, float x, float y, float z, float roll, float pitch, float yaw);
Pose get_local_error_to_setpoint(Pose *global, Pose *setpoint, float cos_yaw, float sin_yaw);
void print_pose(const Pose *pose);

#endif // POSE_H
