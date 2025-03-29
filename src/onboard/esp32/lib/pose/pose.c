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

Pose get_error_to_setpoint(Pose *pose, Pose *setpoint) {
    Pose error;
    error.x = setpoint->x - pose->x;
    error.y = setpoint->y - pose->y;
    error.z = setpoint->z - pose->z;
    error.roll = setpoint->roll - pose->roll;
    error.pitch = setpoint->pitch - pose->pitch;
    error.yaw = setpoint->yaw - pose->yaw;
    return error; // Return the error between current pose and setpoint.
}

Pose convert_from_global_to_local(Pose *global, Pose *setpoint, float cos_yaw, float sin_yaw) {
    // This function converts the global xyz coordinates based off the starting point to a local coordinate system based off the drone's yaw.
    // Uses rotation matrix to convert global coordinates to local coordinates.
    // Returns the local errors for x and y, but global errors for z, roll, pitch, and yaw.
    Pose result = {
        .x = cos_yaw * (setpoint->x - global->x) + sin_yaw * (setpoint->y - global->y),
        .y = -sin_yaw * (setpoint->x - global->x) + cos_yaw * (setpoint->y - global->y),
        .z = global->z - setpoint->z,
        .roll = global->roll - setpoint->roll,
        .pitch = global->pitch - setpoint->pitch,
        .yaw = global->yaw - setpoint->yaw
    };
    return result;
     
}

void print_pose(const Pose *pose) {
    printf("Pose - X: %.2f, Y: %.2f, Z: %.2f, Roll: %.2f, Pitch: %.2f, Yaw: %.2f\n",
           pose->x, pose->y, pose->z, pose->roll, pose->pitch, pose->yaw);
}
