#ifndef SENSOR_FUSION_H
#define SENSOR_FUSION_H

#include <stdint.h>
#include <stdbool.h>

typedef struct {
	float q_w;       /* Quaternion scalar (real) part — 1.0 when orientation is identity */
	float q_x;       /* Quaternion i-vector component (rotation around X) */
	float q_y;       /* Quaternion j-vector component (rotation around Y) */
	float q_z;       /* Quaternion k-vector component (rotation around Z) */
	float roll_deg;  /* Roll  derived from quaternion (degrees, positive = right side down) */
	float pitch_deg; /* Pitch derived from quaternion (degrees, positive = nose up) */
	float yaw_deg;   /* Yaw   derived from quaternion (degrees, positive = clockwise from North) */
} OrientationEstimate;

typedef struct {
	float p_x;  /* Position X in local tangential frame (m, forward / North) */
	float p_y;  /* Position Y in local tangential frame (m, lateral / East) */
	float p_z;  /* Altitude (m, up-positive) */
	float v_x;  /* Velocity X (m/s) */
	float v_y;  /* Velocity Y (m/s) */
	float v_z;  /* Velocity Z (m/s) */
	float b_ax; /* Estimated accelerometer bias, X axis (m/s²) */
	float b_ay; /* Estimated accelerometer bias, Y axis (m/s²) */
	float b_az; /* Estimated accelerometer bias, Z axis (m/s²) */
} NavEstimate;

void sensor_fusion_init(void);
void sensor_fuse();

/*
 * Called by the UART receive handler whenever a new optical-flow packet arrives
 * from the RPi camera. vx_mps / vy_mps are world-frame velocities (the camera
 * firmware or RPi has already rotated them out of the sensor frame).
 *
 * quality: 0.0–1.0 confidence score emitted by the camera's on-chip
 * pixel-correlation DSP. Low texture, a blurry surface, or very low altitude
 * all reduce it. Measurements below 0.15 are automatically discarded.
 *
 * Note: there is no sensor_fusion_update_baro() because the BMP390 is read
 * directly on the ESP32. sensor_fuse() detects a fresh barometer sample by
 * watching the bmp390.new_reading toggle flag — no external push is needed.
 */
void sensor_fusion_update_optical_flow(float vx_mps, float vy_mps, uint32_t timestamp_us, float quality);
void sensor_fusion_update_gps_xy(float x_m, float y_m, uint32_t timestamp_us, float h_acc_m);

OrientationEstimate sensor_fusion_get_orientation(void);
NavEstimate sensor_fusion_get_nav_estimate(void);

extern float past_accelerations[3];
#endif