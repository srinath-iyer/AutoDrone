#include <stdio.h>
#include "pose.h"
#include "constants.h"
#include <math.h>

void init_pose(Pose *pose, uint32_t timestamp, float x, float y, float z, float roll, float pitch, float yaw) {
    pose->timestamp = timestamp; // Useful for queued setpoints/sensor fusion stuff with the RPi.
    pose->x = x;
    pose->y = y;
    pose->z = z;
    pose->roll = roll;
    pose->pitch = pitch;
    pose->yaw = yaw;
}

void update_pose(Pose *pose, uint32_t updated_timestamp, float dx, float dy, float dz, float droll, float dpitch, float dyaw) { // Update pose with deltas.
    // Update pose is meant to be only used for the drone pose Pose struct, which is initizlied at origin and updated to the drone's current position.
    pose->timestamp = updated_timestamp;
    pose->x += dx;
    pose->y += dy;
    pose->z += dz;
    pose->roll += droll;
    pose->pitch += dpitch;
    pose->yaw += dyaw;
}

Pose get_local_error_to_setpoint(Pose *global, Pose *setpoint, float cos_yaw, float sin_yaw) {
    // This function converts the global xyz coordinates based off the starting point to a local coordinate system based off the drone's yaw.
    // Uses rotation matrix to convert global coordinates to local coordinates.
    // Returns the local errors for x and y, but global errors for z, roll, pitch, and yaw.
    // Useful for PID because we want to know the distances with reference to the correct coordinate reference (ie. roll and pitch)
    Pose result = {
        .timestamp = global->timestamp, // Useless.
        .x = cos_yaw * (setpoint->x - global->x) + sin_yaw * (setpoint->y - global->y),
        .y = -sin_yaw * (setpoint->x - global->x) + cos_yaw * (setpoint->y - global->y),
        .z = global->z - setpoint->z,
        .roll = global->roll - setpoint->roll,
        .pitch = global->pitch - setpoint->pitch,
        .yaw = global->yaw - setpoint->yaw
    };
    return result;
     
}

void print_pose(Pose *pose) {
    printf("%lu: Pose - X: %.2f, Y: %.2f, Z: %.2f, Roll: %.2f, Pitch: %.2f, Yaw: %.2f\n",
           (unsigned long)pose->timestamp, pose->x, pose->y, pose->z, pose->roll, pose->pitch, pose->yaw);
}

float atan2f_to_angle(float y, float x) {
    // Convert atan2f output to angle in degrees
    float angle = atan2f(y, x) * (180.0 / PI_FLOAT);
    if (angle < 0) {
        angle += 360.0; // Normalize to [0, 360)
    }
    return angle;
}