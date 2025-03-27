#include <stdio.h>
#include "pose.h"

void init_pose(Pose *pose, float x, float y, float z, float roll, float pitch, float yaw) {
    pose->x = x;
    pose->y = y;
    pose->z = z;
    pose->roll = roll;
    pose->pitch = pitch;
    pose->yaw = yaw;
}

void update_pose(Pose *pose, float dx, float dy, float dz, float droll, float dpitch, float dyaw) { // Update pose with deltas.
    pose->x += dx;
    pose->y += dy;
    pose->z += dz;
    pose->roll += droll;
    pose->pitch += dpitch;
    pose->yaw += dyaw;
}

void print_pose(const Pose *pose) {
    printf("Pose - X: %.2f, Y: %.2f, Z: %.2f, Roll: %.2f, Pitch: %.2f, Yaw: %.2f\n",
           pose->x, pose->y, pose->z, pose->roll, pose->pitch, pose->yaw);
}
