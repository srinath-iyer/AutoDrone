#ifndef SENSOR_FUSION_CONST_H
#define SENSOR_FUSION_CONST_H

#include "constants.h"

/* State dimensions for the split 1D-per-axis navigation Kalman filters. */
#define AXIS_STATE_DIM 3
#define NAV_STATE_DIM 9

/* Shared math helpers for the fusion module. */
#define DEG2RAD (PI_FLOAT / 180.0f)

/* Numerical guards. */
#define SENSOR_FUSION_MIN_VARIANCE 1e-6f
#define SENSOR_FUSION_QUAT_NORM_EPS 1e-6f
#define SENSOR_FUSION_ACCEL_NORM_EPS 1e-4f

/* Mahony attitude-filter tuning. */
#define SENSOR_FUSION_MAHONY_KP 2.2f
#define SENSOR_FUSION_MAHONY_KI 0.05f
#define SENSOR_FUSION_MAHONY_INT_LIM 0.35f

/* Attitude robustness gates. */
#define SENSOR_FUSION_GYRO_CLIP_DPS 700.0f
#define SENSOR_FUSION_ACCEL_TRUST_MIN_G 0.75f
#define SENSOR_FUSION_ACCEL_TRUST_MAX_G 1.25f
#define SENSOR_FUSION_MAX_ANGLE_RATE_DPS 800.0f
#define SENSOR_FUSION_MIN_ANGLE_STEP_DEG 0.3f

/* Quaternion numeric guards. */
#define SENSOR_FUSION_QUAT_MAX_COMPONENT 4.0f
#define SENSOR_FUSION_QUAT_MAX_NORM 8.0f

/* Predict-step timing and gravity constants. */
#define SENSOR_FUSION_DT_MAX_S 0.05f
#define SENSOR_FUSION_DT_FALLBACK_S 0.004f
#define SENSOR_FUSION_GRAVITY_MPS2 9.81f

/* IMU preprocessing. */
#define SENSOR_FUSION_ACCEL_LPF_ALPHA 0.08f

/* Measurement gating and measurement-noise assumptions. */
#define SENSOR_FUSION_BARO_STD_M 0.08f
#define SENSOR_FUSION_FLOW_STD_MPS 0.20f
#define SENSOR_FUSION_FLOW_MIN_QUALITY 0.15f
#define SENSOR_FUSION_GPS_MIN_STD_M 0.35f

/* Initial covariance diagonal for each 1D axis filter. */
#define SENSOR_FUSION_INIT_POS_VAR 0.5f
#define SENSOR_FUSION_INIT_VEL_VAR 0.8f
#define SENSOR_FUSION_INIT_BIAS_VAR 0.15f

/* Process-noise growth rates for the horizontal (X/Y) axis filters. */
#define SENSOR_FUSION_XY_Q_POS 0.01f
#define SENSOR_FUSION_XY_Q_VEL 0.45f
#define SENSOR_FUSION_XY_Q_BIAS 0.008f

/* Process-noise growth rates for the vertical (Z) axis filter. */
#define SENSOR_FUSION_Z_Q_POS 0.01f
#define SENSOR_FUSION_Z_Q_VEL 0.35f
#define SENSOR_FUSION_Z_Q_BIAS 0.006f

#endif