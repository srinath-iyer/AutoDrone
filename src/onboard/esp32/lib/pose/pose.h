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
void update_pose(Pose *pose, float dx, float dy, float dz, float droll, float dpitch, float dyaw);
void print_pose(const Pose *pose);

#endif // POSE_H
