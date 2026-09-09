#include "sensor_fusion.h"
#include "sensor_fusion_const.h"
#include <mpu6050.h>
#include <pose.h>
#include <math.h>
#include <stdio.h>
#include <comms.h>
#include <bmp390.h>
#include "esp_timer.h"

/*
 * Single-axis (1D) Kalman filter state for one navigation axis (X, Y, or Z).
 *
 * State vector  x = [position, velocity, accel_bias]:
 *   x[0]  position (m) — dead-reckoned by integrating bias-corrected acceleration
 *   x[1]  velocity (m/s) — integrated from bias-corrected acceleration
 *   x[2]  accel bias (m/s²) — slowly-drifting sensor offset; subtracting it before
 *          integration prevents the offset from accumulating as quadratic position drift
 *
 * Three AxisKalman instances run independently for X, Y, and Z.
 * The IMU accelerometer is the control input to the predict step.
 * Absolute sensors (baro altitude, GPS position, optical-flow velocity) drive the
 * scalar update step.
 */
typedef struct {
    float x[AXIS_STATE_DIM];                  /* State vector: [pos_m, vel_mps, accel_bias_mps2] */
    float P[AXIS_STATE_DIM][AXIS_STATE_DIM];  /* 3×3 state covariance — quantifies current estimation uncertainty */
    float q_pos;   /* Process noise spectral density for position  (m²/s): how fast pos uncertainty grows per second */
    float q_vel;   /* Process noise spectral density for velocity  ((m/s)²/s): how fast vel uncertainty grows per second */
    float q_bias;  /* Process noise spectral density for accel bias ((m/s²)²/s): how fast bias is allowed to drift */
} AxisKalman;

/*
 * Internal state for the Mahony complementary quaternion attitude filter.
 *
 * Orientation is stored as a unit quaternion q = q_w + q_x*i + q_y*j + q_z*k,
 * which avoids gimbal lock and supports stable normalisation via a single sqrt.
 *
 * The integral_e* terms accumulate the cross-product gravity-alignment error
 * over time, providing a soft gyro-bias correction via the integral gain Ki.
 */
typedef struct {
    float q_w;          /* Quaternion scalar (real) part — 1.0 at zero rotation */
    float q_x;          /* Quaternion i-vector component (encodes rotation around X) */
    float q_y;          /* Quaternion j-vector component (encodes rotation around Y) */
    float q_z;          /* Quaternion k-vector component (encodes rotation around Z) */
    float integral_ex;  /* Ki integral of gravity-alignment error, X axis */
    float integral_ey;  /* Ki integral of gravity-alignment error, Y axis */
    float integral_ez;  /* Ki integral of gravity-alignment error, Z axis */
} MahonyState;

/*
 * Top-level sensor fusion state. One global instance (g_sf) holds all filter state.
 *
 * Sensors native to the ESP32 (IMU, baro) are read and applied inside sensor_fuse()
 * on every call — no external push needed. Baro specifically uses a toggle-flag pattern:
 * bmp390.new_reading flips each time the BMP390 driver writes a fresh sample, and
 * sensor_fuse() detects the edge here. This is why there is no public
 * sensor_fusion_update_baro() — the update is applied automatically each tick.
 *
 * Sensors with async UART delivery (optical flow from the RPi camera, GPS) stage
 * their latest reading via sensor_fusion_update_*() and it is consumed on the next
 * sensor_fuse() call.
 */
typedef struct {
    bool initialized;           /* True after sensor_fusion_init() has been called */
    uint32_t last_predict_us;   /* esp_timer timestamp (us) of the last predict step, used to compute dt */
    bool has_last_attitude;     /* True once at least one roll/pitch/yaw sample has been published */
    float last_roll_deg;        /* Previous roll value after continuity gating */
    float last_pitch_deg;       /* Previous pitch value after continuity gating */
    float last_yaw_deg;         /* Previous yaw value after continuity gating */
    bool last_bmp_new_reading;  /* Last-seen value of bmp390.new_reading; a change signals a fresh baro sample */
    float last_baro_alt_m;      /* Most recent bias-corrected barometer altitude (m) */
    MahonyState attitude;       /* Mahony quaternion attitude estimator state */
    AxisKalman kf_x;  /* 1D Kalman filter for the X navigation axis (forward / North) */
    AxisKalman kf_y;  /* 1D Kalman filter for the Y navigation axis (lateral / East) */
    AxisKalman kf_z;  /* 1D Kalman filter for the Z navigation axis (altitude, up-positive) */

    /* --- Optical flow (staged from RPi UART, consumed on next sensor_fuse() tick) --- */
    bool has_flow;          /* True when a pending flow measurement is ready to be applied */
    float flow_vx_mps;      /* World-frame velocity, X axis (m/s), from optical flow camera */
    float flow_vy_mps;      /* World-frame velocity, Y axis (m/s), from optical flow camera */
    uint32_t flow_ts_us;    /* Timestamp of the flow packet (us) */
    float flow_quality;     /* 0.0-1.0 camera correlation quality score; below 0.15 is discarded */

    /* --- GPS (staged from gpsd on RPi over WiFi, consumed on next sensor_fuse() tick) --- */
    bool has_gps;           /* True when a pending GPS fix is ready to be applied */
    float gps_x_m;          /* GPS-derived X position in local tangential frame (m) */
    float gps_y_m;          /* GPS-derived Y position in local tangential frame (m) */
    uint32_t gps_ts_us;     /* Timestamp of the GPS fix (us) */
    float gps_hacc_m;       /* Horizontal accuracy estimate from GPS receiver (m), used to scale R */
} SensorFusionContext;

static SensorFusionContext g_sf = {0};

static float clampf(float value, float lo, float hi) {
    if (value < lo) {
        return lo;
    }
    if (value > hi) {
        return hi;
    }
    return value;
}

static float wrap_angle_deg(float angle_deg) {
    while (angle_deg > 180.0f) {
        angle_deg -= 360.0f;
    }
    while (angle_deg <= -180.0f) {
        angle_deg += 360.0f;
    }
    return angle_deg;
}

static float angle_delta_deg(float current_deg, float previous_deg) {
    return wrap_angle_deg(current_deg - previous_deg);
}

static void axis_kf_init(AxisKalman *kf, float q_pos, float q_vel, float q_bias, float init_pos_var, float init_vel_var, float init_bias_var) {
    kf->x[0] = 0.0f;
    kf->x[1] = 0.0f;
    kf->x[2] = 0.0f;

    for (int r = 0; r < AXIS_STATE_DIM; r++) {
        for (int c = 0; c < AXIS_STATE_DIM; c++) {
            kf->P[r][c] = 0.0f;
        }
    }

    kf->P[0][0] = init_pos_var;
    kf->P[1][1] = init_vel_var;
    kf->P[2][2] = init_bias_var;

    kf->q_pos = q_pos;
    kf->q_vel = q_vel;
    kf->q_bias = q_bias;
}

/*
 * Kalman predict step — propagates state and covariance forward by dt seconds
 * using the (bias-corrected) IMU accelerometer as the control input.
 *
 * State vector: x = [p (m), v (m/s), b (m/s²)]
 *
 * State prediction (x_new = A * x_old, expanded into scalar kinematics):
 *   p_new = p + v*dt + ½*(a−b)*dt²    constant-acceleration dead-reckoning
 *   v_new = v + (a−b)*dt              velocity integration
 *   b_new = b                         bias modeled as random walk; unchanged in predict
 *
 * System (state-transition) matrix A — Jacobian of the kinematics wrt [p, v, b]:
 *   A = | 1   dt  −½dt² |
 *       | 0    1    −dt  |
 *       | 0    0      1  |
 *
 * Covariance prediction:
 *   P_new = A * P * A^T + Q,   Q = diag(q_pos·dt, q_vel·dt, q_bias·dt)
 */
static void axis_kf_predict(AxisKalman *kf, float accel_mps2, float dt) {
    const float dt2 = dt * dt;
    const float half_dt2 = 0.5f * dt2;

    const float p = kf->x[0];
    const float v = kf->x[1];
    const float b = kf->x[2];

    /* --- x_new = A * x_old: kinematics written as scalars (avoids a temporary vector) ---
     * This is algebraically identical to the 3x1 = 3x3 * 3x1 matrix multiply. */
    const float a_eff = accel_mps2 - b;            /* subtract estimated bias before integrating */
    kf->x[0] = p + v * dt + half_dt2 * a_eff;      /* p_new = p + v·dt + ½·a_eff·dt² */
    kf->x[1] = v + dt * a_eff;                     /* v_new = v + a_eff·dt */
    kf->x[2] = b;                                  /* b_new = b (unchanged; evolves only via process noise) */

    /* A is the state-transition matrix used below for covariance propagation only.
     * It is NOT used to compute the state update above — that was done as scalar kinematics. */
    float A[AXIS_STATE_DIM][AXIS_STATE_DIM] = {
        {1.0f, dt, -half_dt2},
        {0.0f, 1.0f, -dt},
        {0.0f, 0.0f, 1.0f},
    };

    float AP[AXIS_STATE_DIM][AXIS_STATE_DIM] = {0};
    float APA[AXIS_STATE_DIM][AXIS_STATE_DIM] = {0};

    /* Compute AP = A * P (intermediate matrix product) */
    for (int r = 0; r < AXIS_STATE_DIM; r++) {
        for (int c = 0; c < AXIS_STATE_DIM; c++) {
            for (int k = 0; k < AXIS_STATE_DIM; k++) {
                AP[r][c] += A[r][k] * kf->P[k][c];
            }
        }
    }

    /* Compute APA = AP * A^T = A * P * A^T.
     * A^T is accessed as A[c][k] (indices swapped), since A^T[k][c] == A[c][k]. */
    for (int r = 0; r < AXIS_STATE_DIM; r++) {
        for (int c = 0; c < AXIS_STATE_DIM; c++) {
            for (int k = 0; k < AXIS_STATE_DIM; k++) {
                APA[r][c] += AP[r][k] * A[c][k];
            }
        }
    }

    /* P = A*P*A^T (before adding process noise) */
    for (int r = 0; r < AXIS_STATE_DIM; r++) {
        for (int c = 0; c < AXIS_STATE_DIM; c++) {
            kf->P[r][c] = APA[r][c];
        }
    }

    /* P += Q·dt: grow diagonal uncertainty to account for unmodeled disturbances over time */
    kf->P[0][0] += kf->q_pos * dt;
    kf->P[1][1] += kf->q_vel * dt;
    kf->P[2][2] += kf->q_bias * dt;
}

/*
 * Kalman scalar measurement update — corrects the state estimate when a single
 * scalar observation z arrives (baro altitude, optical-flow velocity, GPS position, etc.).
 * This is a general function: h0/h1/h2 define H, the row vector that maps the
 * state to the measurement.
 *
 * Measurement model:  z = H·x + noise,   H = [h0, h1, h2]
 *   [1, 0, 0]  observes position    (baro altitude, GPS XY)
 *   [0, 1, 0]  observes velocity    (optical-flow vx or vy)
 * R is the sensor noise variance (σ²) for this measurement.
 *
 * Steps:
 *   1. y   = z − H·x              innovation: gap between measurement and prediction
 *   2. PHt = P·H^T                project covariance into measurement space (3×1)
 *   3. S   = H·PHt + R            innovation variance: state uncertainty + sensor noise
 *   4. K   = PHt / S              Kalman gain: how much of the innovation to apply
 *   5. x  += K·y                  state update: nudge state toward the measurement
 *   6. P   = (I − K·H)·P          covariance update: reduce uncertainty in observed direction
 */
static void axis_kf_scalar_update(AxisKalman *kf, float z, float h0, float h1, float h2, float R) {
    /* Step 1 — Innovation: how much the measurement disagrees with our current prediction */
    const float y = z - (h0 * kf->x[0] + h1 * kf->x[1] + h2 * kf->x[2]);

    /* Step 2 — PHt = P·H^T: project covariance into the measurement axis (3×1 vector) */
    float PHt[AXIS_STATE_DIM] = {
        kf->P[0][0] * h0 + kf->P[0][1] * h1 + kf->P[0][2] * h2,
        kf->P[1][0] * h0 + kf->P[1][1] * h1 + kf->P[1][2] * h2,
        kf->P[2][0] * h0 + kf->P[2][1] * h1 + kf->P[2][2] * h2,
    };

    /* Step 3 — S = H·P·H^T + R: scalar innovation variance.
     *   Low S → state and sensor agree well; high S → large disagreement or noisy sensor. */
    float S = h0 * PHt[0] + h1 * PHt[1] + h2 * PHt[2] + R;
    if (S < SENSOR_FUSION_MIN_VARIANCE) {
        S = SENSOR_FUSION_MIN_VARIANCE;  /* guard against division by zero if both P and R collapse */
    }

    /* Step 4 — Kalman gain K = PHt / S.
     *   Large K (small S) → trust the measurement more.
     *   Small K (large S) → trust the prediction more. */
    const float inv_S = 1.0f / S;
    float K[AXIS_STATE_DIM] = {PHt[0] * inv_S, PHt[1] * inv_S, PHt[2] * inv_S};

    /* Step 5 — State update: x += K·y (each state element is corrected proportionally to K) */
    for (int i = 0; i < AXIS_STATE_DIM; i++) {
        kf->x[i] += K[i] * y;
    }

    /* Step 6 — Covariance update: P = (I − K·H)·P.
     *   Build K·H as an outer product (3×3), form (I − K·H), then multiply by current P. */
    float KH[AXIS_STATE_DIM][AXIS_STATE_DIM] = {0};
    KH[0][0] = K[0] * h0; KH[0][1] = K[0] * h1; KH[0][2] = K[0] * h2;
    KH[1][0] = K[1] * h0; KH[1][1] = K[1] * h1; KH[1][2] = K[1] * h2;
    KH[2][0] = K[2] * h0; KH[2][1] = K[2] * h1; KH[2][2] = K[2] * h2;

    float I_KH[AXIS_STATE_DIM][AXIS_STATE_DIM] = {
        {1.0f - KH[0][0], -KH[0][1], -KH[0][2]},
        {-KH[1][0], 1.0f - KH[1][1], -KH[1][2]},
        {-KH[2][0], -KH[2][1], 1.0f - KH[2][2]},
    };

    float newP[AXIS_STATE_DIM][AXIS_STATE_DIM] = {0};
    for (int r = 0; r < AXIS_STATE_DIM; r++) {
        for (int c = 0; c < AXIS_STATE_DIM; c++) {
            for (int k = 0; k < AXIS_STATE_DIM; k++) {
                newP[r][c] += I_KH[r][k] * kf->P[k][c];
            }
        }
    }

    for (int r = 0; r < AXIS_STATE_DIM; r++) {
        for (int c = 0; c < AXIS_STATE_DIM; c++) {
            kf->P[r][c] = newP[r][c];
        }
    }
}

static void quat_normalize(float *qw, float *qx, float *qy, float *qz) {
    if (!isfinite(*qw) || !isfinite(*qx) || !isfinite(*qy) || !isfinite(*qz)) {
        *qw = 1.0f;
        *qx = 0.0f;
        *qy = 0.0f;
        *qz = 0.0f;
        return;
    }

    *qw = clampf(*qw, -SENSOR_FUSION_QUAT_MAX_COMPONENT, SENSOR_FUSION_QUAT_MAX_COMPONENT);
    *qx = clampf(*qx, -SENSOR_FUSION_QUAT_MAX_COMPONENT, SENSOR_FUSION_QUAT_MAX_COMPONENT);
    *qy = clampf(*qy, -SENSOR_FUSION_QUAT_MAX_COMPONENT, SENSOR_FUSION_QUAT_MAX_COMPONENT);
    *qz = clampf(*qz, -SENSOR_FUSION_QUAT_MAX_COMPONENT, SENSOR_FUSION_QUAT_MAX_COMPONENT);

    const float n = sqrtf((*qw) * (*qw) + (*qx) * (*qx) + (*qy) * (*qy) + (*qz) * (*qz));
    if (!isfinite(n) || n <= SENSOR_FUSION_QUAT_NORM_EPS || n > SENSOR_FUSION_QUAT_MAX_NORM) {
        *qw = 1.0f;
        *qx = 0.0f;
        *qy = 0.0f;
        *qz = 0.0f;
        return;
    }

    const float inv_n = 1.0f / n;
    *qw *= inv_n;
    *qx *= inv_n;
    *qy *= inv_n;
    *qz *= inv_n;
}

static void mahony_update(MahonyState *m, float gx_dps, float gy_dps, float gz_dps, float ax, float ay, float az, float dt) {
    const float kp = SENSOR_FUSION_MAHONY_KP;
    const float ki = SENSOR_FUSION_MAHONY_KI;

    gx_dps = clampf(gx_dps, -SENSOR_FUSION_GYRO_CLIP_DPS, SENSOR_FUSION_GYRO_CLIP_DPS);
    gy_dps = clampf(gy_dps, -SENSOR_FUSION_GYRO_CLIP_DPS, SENSOR_FUSION_GYRO_CLIP_DPS);
    gz_dps = clampf(gz_dps, -SENSOR_FUSION_GYRO_CLIP_DPS, SENSOR_FUSION_GYRO_CLIP_DPS);

    float qw = m->q_w;
    float qx = m->q_x;
    float qy = m->q_y;
    float qz = m->q_z;

    const float anorm = sqrtf(ax * ax + ay * ay + az * az);
    if (anorm > SENSOR_FUSION_ACCEL_NORM_EPS) {
        const float g_ratio = anorm / SENSOR_FUSION_GRAVITY_MPS2;
        if (g_ratio >= SENSOR_FUSION_ACCEL_TRUST_MIN_G && g_ratio <= SENSOR_FUSION_ACCEL_TRUST_MAX_G) {
            const float inv_a = 1.0f / anorm;
            ax *= inv_a;
            ay *= inv_a;
            az *= inv_a;

            const float vx = 2.0f * (qx * qz - qw * qy);
            const float vy = 2.0f * (qw * qx + qy * qz);
            const float vz = qw * qw - qx * qx - qy * qy + qz * qz;

            const float ex = ay * vz - az * vy;
            const float ey = az * vx - ax * vz;
            const float ez = ax * vy - ay * vx;

            m->integral_ex += ex * dt;
            m->integral_ey += ey * dt;
            m->integral_ez += ez * dt;

            m->integral_ex = clampf(m->integral_ex, -SENSOR_FUSION_MAHONY_INT_LIM, SENSOR_FUSION_MAHONY_INT_LIM);
            m->integral_ey = clampf(m->integral_ey, -SENSOR_FUSION_MAHONY_INT_LIM, SENSOR_FUSION_MAHONY_INT_LIM);
            m->integral_ez = clampf(m->integral_ez, -SENSOR_FUSION_MAHONY_INT_LIM, SENSOR_FUSION_MAHONY_INT_LIM);

            gx_dps += kp * ex + ki * m->integral_ex;
            gy_dps += kp * ey + ki * m->integral_ey;
            gz_dps += kp * ez + ki * m->integral_ez;
        } else {
            m->integral_ex *= 0.995f;
            m->integral_ey *= 0.995f;
            m->integral_ez *= 0.995f;
        }
    }

    const float gx = gx_dps * DEG2RAD;
    const float gy = gy_dps * DEG2RAD;
    const float gz = gz_dps * DEG2RAD;

    const float half_dt = 0.5f * dt;
    const float dq_w = (-qx * gx - qy * gy - qz * gz) * half_dt;
    const float dq_x = ( qw * gx + qy * gz - qz * gy) * half_dt;
    const float dq_y = ( qw * gy - qx * gz + qz * gx) * half_dt;
    const float dq_z = ( qw * gz + qx * gy - qy * gx) * half_dt;

    qw += dq_w;
    qx += dq_x;
    qy += dq_y;
    qz += dq_z;
    quat_normalize(&qw, &qx, &qy, &qz);

    m->q_w = qw;
    m->q_x = qx;
    m->q_y = qy;
    m->q_z = qz;
}

static void quaternion_to_rpy_deg(const MahonyState *m, float *roll_deg, float *pitch_deg, float *yaw_deg) {
    const float qw = m->q_w;
    const float qx = m->q_x;
    const float qy = m->q_y;
    const float qz = m->q_z;

    const float sinr_cosp = 2.0f * (qw * qx + qy * qz);
    const float cosr_cosp = 1.0f - 2.0f * (qx * qx + qy * qy);
    *roll_deg = atan2f(sinr_cosp, cosr_cosp) * (180.0f / PI_FLOAT);

    const float sinp = 2.0f * (qw * qy - qz * qx);
    if (fabsf(sinp) >= 1.0f) {
        *pitch_deg = copysignf(90.0f, sinp);
    } else {
        *pitch_deg = asinf(sinp) * (180.0f / PI_FLOAT);
    }

    const float siny_cosp = 2.0f * (qw * qz + qx * qy);
    const float cosy_cosp = 1.0f - 2.0f * (qy * qy + qz * qz);
    *yaw_deg = atan2f(siny_cosp, cosy_cosp) * (180.0f / PI_FLOAT);
}

static void rotate_body_to_world(const MahonyState *m, float ax, float ay, float az, float *ax_w, float *ay_w, float *az_w) {
    const float qw = m->q_w;
    const float qx = m->q_x;
    const float qy = m->q_y;
    const float qz = m->q_z;

    const float r11 = 1.0f - 2.0f * (qy * qy + qz * qz);
    const float r12 = 2.0f * (qx * qy - qz * qw);
    const float r13 = 2.0f * (qx * qz + qy * qw);
    const float r21 = 2.0f * (qx * qy + qz * qw);
    const float r22 = 1.0f - 2.0f * (qx * qx + qz * qz);
    const float r23 = 2.0f * (qy * qz - qx * qw);
    const float r31 = 2.0f * (qx * qz - qy * qw);
    const float r32 = 2.0f * (qy * qz + qx * qw);
    const float r33 = 1.0f - 2.0f * (qx * qx + qy * qy);

    *ax_w = r11 * ax + r12 * ay + r13 * az;
    *ay_w = r21 * ax + r22 * ay + r23 * az;
    *az_w = r31 * ax + r32 * ay + r33 * az;
}

static void apply_baro_update_if_ready(void) {
    if (!g_sf.initialized) {
        return;
    }

    if (bmp390.new_reading == g_sf.last_bmp_new_reading) {
        return;
    }

    g_sf.last_bmp_new_reading = bmp390.new_reading;
    g_sf.last_baro_alt_m = bmp390.altitude - BMP390_ALT_BIAS;

    axis_kf_scalar_update(&g_sf.kf_z, g_sf.last_baro_alt_m, 1.0f, 0.0f, 0.0f,
                          SENSOR_FUSION_BARO_STD_M * SENSOR_FUSION_BARO_STD_M);
}

static void apply_flow_update_if_ready(void) {
    if (!g_sf.has_flow) {
        return;
    }

    if (g_sf.flow_quality < SENSOR_FUSION_FLOW_MIN_QUALITY) {
        g_sf.has_flow = false;
        return;
    }

    axis_kf_scalar_update(&g_sf.kf_x, g_sf.flow_vx_mps, 0.0f, 1.0f, 0.0f,
                          SENSOR_FUSION_FLOW_STD_MPS * SENSOR_FUSION_FLOW_STD_MPS);
    axis_kf_scalar_update(&g_sf.kf_y, g_sf.flow_vy_mps, 0.0f, 1.0f, 0.0f,
                          SENSOR_FUSION_FLOW_STD_MPS * SENSOR_FUSION_FLOW_STD_MPS);
    g_sf.has_flow = false;
}

static void apply_gps_update_if_ready(void) {
    if (!g_sf.has_gps) {
        return;
    }

    const float gps_sigma = fmaxf(g_sf.gps_hacc_m, SENSOR_FUSION_GPS_MIN_STD_M);
    const float R = gps_sigma * gps_sigma;
    axis_kf_scalar_update(&g_sf.kf_x, g_sf.gps_x_m, 1.0f, 0.0f, 0.0f, R);
    axis_kf_scalar_update(&g_sf.kf_y, g_sf.gps_y_m, 1.0f, 0.0f, 0.0f, R);
    g_sf.has_gps = false;
}

float past_accelerations[3];

void sensor_fusion_init(void) {
    g_sf.initialized = true;
    g_sf.last_predict_us = (uint32_t)esp_timer_get_time();
    g_sf.has_last_attitude = false;
    g_sf.last_roll_deg = 0.0f;
    g_sf.last_pitch_deg = 0.0f;
    g_sf.last_yaw_deg = 0.0f;
    g_sf.last_bmp_new_reading = bmp390.new_reading;
    g_sf.last_baro_alt_m = bmp390.altitude - BMP390_ALT_BIAS;

    g_sf.attitude.q_w = 1.0f;
    g_sf.attitude.q_x = 0.0f;
    g_sf.attitude.q_y = 0.0f;
    g_sf.attitude.q_z = 0.0f;
    g_sf.attitude.integral_ex = 0.0f;
    g_sf.attitude.integral_ey = 0.0f;
    g_sf.attitude.integral_ez = 0.0f;

    axis_kf_init(&g_sf.kf_x, SENSOR_FUSION_XY_Q_POS, SENSOR_FUSION_XY_Q_VEL, SENSOR_FUSION_XY_Q_BIAS,
                 SENSOR_FUSION_INIT_POS_VAR, SENSOR_FUSION_INIT_VEL_VAR, SENSOR_FUSION_INIT_BIAS_VAR);
    axis_kf_init(&g_sf.kf_y, SENSOR_FUSION_XY_Q_POS, SENSOR_FUSION_XY_Q_VEL, SENSOR_FUSION_XY_Q_BIAS,
                 SENSOR_FUSION_INIT_POS_VAR, SENSOR_FUSION_INIT_VEL_VAR, SENSOR_FUSION_INIT_BIAS_VAR);
    axis_kf_init(&g_sf.kf_z, SENSOR_FUSION_Z_Q_POS, SENSOR_FUSION_Z_Q_VEL, SENSOR_FUSION_Z_Q_BIAS,
                 SENSOR_FUSION_INIT_POS_VAR, SENSOR_FUSION_INIT_VEL_VAR, SENSOR_FUSION_INIT_BIAS_VAR);
    g_sf.kf_z.x[0] = g_sf.last_baro_alt_m;

    g_sf.has_flow = false;
    g_sf.has_gps = false;
}

void sensor_fuse(){
    if (!g_sf.initialized) {
        sensor_fusion_init();
    }

    const uint32_t now_us = (uint32_t)esp_timer_get_time();
    float dt = (now_us - g_sf.last_predict_us) * 1e-6f;
    if (dt <= 0.0f || dt > SENSOR_FUSION_DT_MAX_S) {
        dt = SENSOR_FUSION_DT_FALLBACK_S;
    }
    g_sf.last_predict_us = now_us;

    const float ax_raw = mpu6050.accel_x;
    const float ay_raw = mpu6050.accel_y;
    const float az_raw = mpu6050.accel_z;

    const float gx_dps = mpu6050.gyro_x - GYRO_X_BIAS;
    const float gy_dps = mpu6050.gyro_y - GYRO_Y_BIAS;
    const float gz_dps = mpu6050.gyro_z - GYRO_Z_BIAS;

    mahony_update(&g_sf.attitude, gx_dps, gy_dps, gz_dps, ax_raw, ay_raw, az_raw, dt);

    float roll_deg, pitch_deg, yaw_deg;
    quaternion_to_rpy_deg(&g_sf.attitude, &roll_deg, &pitch_deg, &yaw_deg);

    roll_deg = wrap_angle_deg(roll_deg);
    pitch_deg = wrap_angle_deg(pitch_deg);
    yaw_deg = wrap_angle_deg(yaw_deg);

    if (g_sf.has_last_attitude) {
        float max_step_deg = SENSOR_FUSION_MAX_ANGLE_RATE_DPS * dt;
        if (max_step_deg < SENSOR_FUSION_MIN_ANGLE_STEP_DEG) {
            max_step_deg = SENSOR_FUSION_MIN_ANGLE_STEP_DEG;
        }

        const float d_roll = clampf(angle_delta_deg(roll_deg, g_sf.last_roll_deg), -max_step_deg, max_step_deg);
        const float d_pitch = clampf(angle_delta_deg(pitch_deg, g_sf.last_pitch_deg), -max_step_deg, max_step_deg);
        const float d_yaw = clampf(angle_delta_deg(yaw_deg, g_sf.last_yaw_deg), -max_step_deg, max_step_deg);

        roll_deg = wrap_angle_deg(g_sf.last_roll_deg + d_roll);
        pitch_deg = wrap_angle_deg(g_sf.last_pitch_deg + d_pitch);
        yaw_deg = wrap_angle_deg(g_sf.last_yaw_deg + d_yaw);
    }

    g_sf.last_roll_deg = roll_deg;
    g_sf.last_pitch_deg = pitch_deg;
    g_sf.last_yaw_deg = yaw_deg;
    g_sf.has_last_attitude = true;

    const float accel_lpf = SENSOR_FUSION_ACCEL_LPF_ALPHA;
    float lpf_ax = accel_lpf * ax_raw + (1.0f - accel_lpf) * past_accelerations[0];
    float lpf_ay = accel_lpf * ay_raw + (1.0f - accel_lpf) * past_accelerations[1];
    float lpf_az = accel_lpf * az_raw + (1.0f - accel_lpf) * past_accelerations[2];
    past_accelerations[0] = lpf_ax;
    past_accelerations[1] = lpf_ay;
    past_accelerations[2] = lpf_az;

    float ax_w, ay_w, az_w;
    rotate_body_to_world(&g_sf.attitude, lpf_ax, lpf_ay, lpf_az, &ax_w, &ay_w, &az_w);
    az_w -= SENSOR_FUSION_GRAVITY_MPS2;

    axis_kf_predict(&g_sf.kf_x, ax_w, dt);
    axis_kf_predict(&g_sf.kf_y, ay_w, dt);
    axis_kf_predict(&g_sf.kf_z, az_w, dt);

    apply_baro_update_if_ready();
    apply_flow_update_if_ready();
    apply_gps_update_if_ready();

    pose.timestamp = now_us;
    pose.x = g_sf.kf_x.x[0];
    pose.y = g_sf.kf_y.x[0];
    pose.z = g_sf.kf_z.x[0];
    pose.v_x = g_sf.kf_x.x[1];
    pose.v_y = g_sf.kf_y.x[1];
    pose.v_z = g_sf.kf_z.x[1];
    pose.roll = roll_deg;
    pose.pitch = pitch_deg;
    pose.yaw = yaw_deg;
}

void sensor_fusion_update_optical_flow(float vx_mps, float vy_mps, uint32_t timestamp_us, float quality) {
    g_sf.flow_vx_mps = vx_mps;
    g_sf.flow_vy_mps = vy_mps;
    g_sf.flow_ts_us = timestamp_us;
    g_sf.flow_quality = quality;
    g_sf.has_flow = true;
}

void sensor_fusion_update_gps_xy(float x_m, float y_m, uint32_t timestamp_us, float h_acc_m) {
    g_sf.gps_x_m = x_m;
    g_sf.gps_y_m = y_m;
    g_sf.gps_ts_us = timestamp_us;
    g_sf.gps_hacc_m = h_acc_m;
    g_sf.has_gps = true;
}

OrientationEstimate sensor_fusion_get_orientation(void) {
    OrientationEstimate out = {
        .q_w = g_sf.attitude.q_w,
        .q_x = g_sf.attitude.q_x,
        .q_y = g_sf.attitude.q_y,
        .q_z = g_sf.attitude.q_z,
        .roll_deg = pose.roll,
        .pitch_deg = pose.pitch,
        .yaw_deg = pose.yaw,
    };
    return out;
}

NavEstimate sensor_fusion_get_nav_estimate(void) {
    NavEstimate out = {
        .p_x = g_sf.kf_x.x[0],
        .p_y = g_sf.kf_y.x[0],
        .p_z = g_sf.kf_z.x[0],
        .v_x = g_sf.kf_x.x[1],
        .v_y = g_sf.kf_y.x[1],
        .v_z = g_sf.kf_z.x[1],
        .b_ax = g_sf.kf_x.x[2],
        .b_ay = g_sf.kf_y.x[2],
        .b_az = g_sf.kf_z.x[2],
    };
    return out;
}