#include "sensor_fusion.h"
#include <mpu6050.h>
#include <pose.h>
#include <math.h>
#include <stdio.h>
#include "constants.h"
#include <comms.h>
#include "esp_timer.h"
float past_accelerations[3];
void sensor_fuse(){
//    float alpha = 0.98;
//    float dt = 0.004;
   
//    // Unbiased values: Accel is useful for position estimates
//    float unbiased_accel_x = mpu6050.accel_x - ACCEL_X_BIAS;
//    float unbiased_accel_y = mpu6050.accel_y - ACCEL_Y_BIAS;
//    float unbiased_accel_z = mpu6050.accel_z - ACCEL_Z_BIAS;
//    float unbiased_gyro_x = mpu6050.gyro_x - GYRO_X_BIAS; // Roll (-)
//    float unbiased_gyro_y = mpu6050.gyro_y - GYRO_Y_BIAS; // Pitch (+)
//    float unbiased_gyro_z = mpu6050.gyro_z - GYRO_Z_BIAS; // Yaw (+)
//    // Roll, Pitch, Yaw:
//    float accel_angle = atan2f(mpu6050.accel_y, mpu6050.accel_z) * 180 / PI_FLOAT;
//    float gyro_rate = unbiased_gyro_x;
//    float gyro_angle = pose.roll + gyro_rate * dt;
//    pose.roll = -(alpha * gyro_angle + (1 - alpha) * accel_angle);
//    float accel_pitch = atan2f(-mpu6050.accel_x, sqrtf(mpu6050.accel_y * mpu6050.accel_y + mpu6050.accel_z * mpu6050.accel_z)) * 180.0f / PI_FLOAT;
//    float gyro_pitch = pose.pitch + unbiased_gyro_y * dt;
//    pose.pitch = alpha * gyro_pitch + (1.0f - alpha) * accel_pitch; // Complementary filter
//    float yaw_lpf = 0.6;
//    pose.yaw = yaw_lpf*(pose.yaw + unbiased_gyro_z * dt) + (1-yaw_lpf)*pose.yaw;
//    // Acceleration:
//    float filtered_accel_x = 0.1 * unbiased_accel_x + 0.9 * past_accelerations[0];
//    float filtered_accel_y = 0.1 * unbiased_accel_y + 0.9 * past_accelerations[1];
//    float filtered_accel_z = 0.1 * unbiased_accel_z + 0.9 * past_accelerations[2];
//    past_accelerations[0] = filtered_accel_x;
//    past_accelerations[1] = filtered_accel_y;
//    past_accelerations[2] = filtered_accel_z;
//    // Velocity:
//    float vel_lpf = 1.0;
//    pose.v_x = vel_lpf*(pose.v_x + filtered_accel_x * dt) + (1-vel_lpf)*pose.v_x;
//    pose.v_y = vel_lpf*(pose.v_y + filtered_accel_y * dt) + (1-vel_lpf)*pose.v_y;
//    pose.v_z = vel_lpf*(pose.v_z + filtered_accel_z * dt) + (1-vel_lpf)*pose.v_z;

//    //Position:
//    float pos_lpf = 1.0;
//    pose.x = pos_lpf*(pose.x + pose.v_x * dt) + (1-pos_lpf)*pose.x;
//    pose.y = pos_lpf*(pose.y + pose.v_y * dt) + (1-pos_lpf)*pose.y;
//    pose.z = pos_lpf*(pose.z + pose.v_z * dt) + (1-pos_lpf)*pose.z;
    float alpha = 0.98f;
    float dt = 0.004f;  // 250 Hz update rate

    // -------------------
    // Unbiased IMU inputs
    // -------------------
    float ax = mpu6050.accel_x - ACCEL_X_BIAS;
    float ay = mpu6050.accel_y - ACCEL_Y_BIAS;
    float az = mpu6050.accel_z - ACCEL_Z_BIAS;

    float gx = mpu6050.gyro_x - GYRO_X_BIAS;
    float gy = mpu6050.gyro_y - GYRO_Y_BIAS;
    float gz = mpu6050.gyro_z - GYRO_Z_BIAS;

    // ----------------------------
    // Roll and pitch (complementary filter)
    // ----------------------------
    float accel_roll = atan2f(ay, az) * 180.0f / PI_FLOAT;
    float gyro_roll = pose.roll + gx * dt;
    pose.roll = -(alpha * gyro_roll + (1 - alpha) * accel_roll);

    float accel_pitch = atan2f(-ax, sqrtf(ay * ay + az * az)) * 180.0f / PI_FLOAT;
    float gyro_pitch = pose.pitch + gy * dt;
    pose.pitch = alpha * gyro_pitch + (1 - alpha) * accel_pitch;

    // ----------------------------
    // Yaw (integration with LPF)
    // ----------------------------
    float yaw_lpf = 0.6;
    pose.yaw = yaw_lpf*(pose.yaw + gz * dt) + (1-yaw_lpf)*pose.yaw;

    // ----------------------------
    // LPF on raw acceleration
    // ----------------------------
    float lpf_accel_x = 0.05f * ax + 0.95f * past_accelerations[0];
    float lpf_accel_y = 0.05f * ay + 0.95f * past_accelerations[1];
    float lpf_accel_z = 0.05f * az + 0.95f * past_accelerations[2];
    past_accelerations[0] = lpf_accel_x;
    past_accelerations[1] = lpf_accel_y;
    past_accelerations[2] = lpf_accel_z;

    // ----------------------------
    // Rotate acceleration into world frame
    // ----------------------------
    float roll_rad  = pose.roll  * PI_FLOAT / 180.0f;
    float pitch_rad = pose.pitch * PI_FLOAT / 180.0f;

    float sin_r = sinf(roll_rad), cos_r = cosf(roll_rad);
    float sin_p = sinf(pitch_rad), cos_p = cosf(pitch_rad);

    float ax_w = lpf_accel_x;
    float ay_w = lpf_accel_y * cos_r - lpf_accel_z * sin_r;
    float az_w = lpf_accel_y * sin_r + lpf_accel_z * cos_r;
    az_w = az_w * cos_p - ax_w * sin_p;

    // Subtract gravity (only vertical component)
    float az_corrected = az_w - ACCEL_Z_BIAS;

    // ----------------------------
    // Optional: Zero-velocity update (ZUPT)
    // ----------------------------
    if (fabsf(ax_w) < 0.02f && fabsf(ay_w) < 0.02f && fabsf(az_corrected) < 0.02f) {
        pose.v_x = 0.0f;
        pose.v_y = 0.0f;
        pose.v_z = 0.0f;
    }

    // ----------------------------
    // Velocity integration
    // ----------------------------
    pose.v_x += ax_w * dt;
    pose.v_y += ay_w * dt;
    pose.v_z += az_corrected * dt;

    // ----------------------------
    // Position integration
    // ----------------------------
    pose.x += pose.v_x * dt;
    pose.y += pose.v_y * dt;
    pose.z += pose.v_z * dt;
}