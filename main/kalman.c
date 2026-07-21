/**
 * @file    kalman.c
 * @brief   自适应卡尔曼滤波器实现
 * @author  Claude
 * @date    2026-07-21
 * @version 3.0.0
 *
 * @details
 * 数据流: 原始数据 → 自适应卡尔曼(Roll/Pitch) → 四元数 → 欧拉角
 *
 * 自适应特性:
 *   1. 自适应 R: 振动检测，加速度计幅值偏离 1g 时增大 R
 *   2. 自适应 Q: 运动检测，角速度大时增大 Q
 *   3. Yaw 陀螺仪积分 + 死区抑制漂移
 */

#include "kalman.h"
#include "esp_log.h"
#include <stddef.h>

static const char *TAG = "kalman";

/* ==================== 默认参数 ==================== */

#define DEFAULT_Q_ANGLE     0.001f
#define DEFAULT_Q_BIAS      0.003f
#define DEFAULT_R_MEASURE   0.03f

/* ==================== 自适应参数 ==================== */

#define VIBRATION_THRESHOLD     0.08f   /* 振动检测阈值 (g) */
#define VIBRATION_R_SCALE       10.0f   /* 振动时 R 放大倍数 */
#define VIBRATION_SMOOTH        0.1f    /* 振动权重低通滤波系数 */

#define MOTION_THRESHOLD        20.0f   /* 运动检测阈值 (dps) */
#define MOTION_Q_SCALE          5.0f    /* 运动时 Q 放大倍数 */
#define MOTION_SMOOTH           0.2f    /* 运动权重低通滤波系数 */

#define YAW_DEADZONE            0.5f    /* Yaw 死区 (dps) */

/* ==================== 内部结构体 ==================== */

typedef struct {
    float angle;
    float rate;
    float bias;
    float P[2][2];
    float Q_angle;
    float Q_bias;
    float R_measure;
} kalman_filter_t;

/* ==================== 模块状态 ==================== */

static kalman_filter_t s_roll;
static kalman_filter_t s_pitch;
static imu_quat_t s_quat;
static float s_yaw;
static float s_vibration_weight;
static float s_motion_weight;
static float s_R_base;
static float s_Q_base;
static uint8_t s_inited;

/* ==================== 内部函数 ==================== */

static void kalman_filter_init(kalman_filter_t *kf)
{
    kf->angle = 0.0f;
    kf->rate = 0.0f;
    kf->bias = 0.0f;
    kf->P[0][0] = 1.0f; kf->P[0][1] = 0.0f;
    kf->P[1][0] = 0.0f; kf->P[1][1] = 1.0f;
    kf->Q_angle = DEFAULT_Q_ANGLE;
    kf->Q_bias = DEFAULT_Q_BIAS;
    kf->R_measure = DEFAULT_R_MEASURE;
}

static float kalman_filter_update(kalman_filter_t *kf, float new_angle,
                                  float new_rate, float dt,
                                  float adaptive_Q, float adaptive_R)
{
    float S, K[2], y, P00_temp, P01_temp;

    /* Step 1: 先验状态预测 */
    kf->rate = new_rate - kf->bias;
    kf->angle += dt * kf->rate;

    /* Step 2: 更新先验协方差矩阵 */
    kf->P[0][0] += dt * (dt * kf->P[1][1] - kf->P[0][1] - kf->P[1][0] + adaptive_Q);
    kf->P[0][1] -= dt * kf->P[1][1];
    kf->P[1][0] -= dt * kf->P[1][1];
    kf->P[1][1] += kf->Q_bias * dt;

    /* Step 3: 计算卡尔曼增益 */
    S = kf->P[0][0] + adaptive_R;
    K[0] = kf->P[0][0] / S;
    K[1] = kf->P[1][0] / S;

    /* Step 4: 计算最优估计 */
    y = new_angle - kf->angle;
    kf->angle += K[0] * y;
    kf->bias  += K[1] * y;

    /* Step 5: 更新协方差矩阵 */
    P00_temp = kf->P[0][0];
    P01_temp = kf->P[0][1];
    kf->P[0][0] -= K[0] * P00_temp;
    kf->P[0][1] -= K[0] * P01_temp;
    kf->P[1][0] -= K[1] * P00_temp;
    kf->P[1][1] -= K[1] * P01_temp;

    return kf->angle;
}

/* ==================== 公共 API ==================== */

void kalman_init(void)
{
    ESP_LOGI(TAG, "=== 初始化自适应卡尔曼滤波器 ===");

    kalman_filter_init(&s_roll);
    kalman_filter_init(&s_pitch);

    s_quat.q0 = 1.0f;
    s_quat.q1 = 0.0f;
    s_quat.q2 = 0.0f;
    s_quat.q3 = 0.0f;
    s_yaw = 0.0f;
    s_vibration_weight = 1.0f;
    s_motion_weight = 1.0f;
    s_R_base = DEFAULT_R_MEASURE;
    s_Q_base = DEFAULT_Q_ANGLE;
    s_inited = 1;

    ESP_LOGI(TAG, "基础参数: Q_angle=%.4f, Q_bias=%.4f, R_measure=%.4f",
             (double)DEFAULT_Q_ANGLE, (double)DEFAULT_Q_BIAS, (double)DEFAULT_R_MEASURE);
}

void kalman_update(float gx, float gy, float gz,
                   float ax, float ay, float az,
                   float dt, kalman_output_t *output)
{
    if (!s_inited) {
        kalman_init();
    }

    /* ===== 1. 振动检测：自适应 R ===== */
    float acc_mag_sq = ax * ax + ay * ay + az * az;
    float acc_mag = 0.0f;
    if (acc_mag_sq > 0.0f) {
        acc_mag = 1.0f / imu_inv_sqrt(acc_mag_sq);
    }
    float acc_deviation = imu_fabs(acc_mag - 1.0f);

    float target_vibration_weight;
    if (acc_deviation < VIBRATION_THRESHOLD) {
        target_vibration_weight = 1.0f;
    } else {
        float ratio = (acc_deviation - VIBRATION_THRESHOLD) / (VIBRATION_THRESHOLD * 4.0f);
        if (ratio > 1.0f) ratio = 1.0f;
        target_vibration_weight = 1.0f + ratio * VIBRATION_R_SCALE;
    }
    s_vibration_weight += VIBRATION_SMOOTH * (target_vibration_weight - s_vibration_weight);
    float adaptive_R = s_R_base * s_vibration_weight;

    /* ===== 2. 运动检测：自适应 Q ===== */
    float gyro_magnitude = imu_fabs(gx) + imu_fabs(gy) + imu_fabs(gz);

    float target_motion_weight;
    if (gyro_magnitude < MOTION_THRESHOLD) {
        target_motion_weight = 1.0f;
    } else {
        float ratio = (gyro_magnitude - MOTION_THRESHOLD) / MOTION_THRESHOLD;
        if (ratio > 1.0f) ratio = 1.0f;
        target_motion_weight = 1.0f + ratio * MOTION_Q_SCALE;
    }
    s_motion_weight += MOTION_SMOOTH * (target_motion_weight - s_motion_weight);
    float adaptive_Q = s_Q_base * s_motion_weight;

    /* ===== 3. 加速度计角度 ===== */
    float acc_roll = imu_safe_atan2(ay, az) * IMU_RAD2DEG;
    float acc_pitch = -imu_safe_atan2(ax, 1.0f / imu_inv_sqrt(ay * ay + az * az)) * IMU_RAD2DEG;

    /* ===== 4. 卡尔曼滤波 (Roll/Pitch) ===== */
    float filtered_roll  = kalman_filter_update(&s_roll,  acc_roll,  gx, dt, adaptive_Q, adaptive_R);
    float filtered_pitch = kalman_filter_update(&s_pitch, acc_pitch, gy, dt, adaptive_Q, adaptive_R);

    /* ===== 5. Yaw 陀螺仪积分 + 死区 ===== */
    if (gz > YAW_DEADZONE || gz < -YAW_DEADZONE) {
        s_yaw += gz * dt;
    }

    /* ===== 6. Roll/Pitch → 四元数，Yaw 通过四元数乘法叠加 ===== */
    imu_euler_to_quat(filtered_roll, filtered_pitch, 0.0f, &s_quat);

    /* Yaw 用四元数乘法叠加（避免重算 sin/cos） */
    if (imu_fabs(s_yaw) > 0.001f) {
        float half_yaw_rad = s_yaw * IMU_DEG2RAD * 0.5f;
        float cy = imu_safe_cos(half_yaw_rad);
        float sy = imu_safe_sin(half_yaw_rad);
        imu_quat_t yaw_q = { cy, 0.0f, 0.0f, sy };
        imu_quat_t result;
        imu_quat_multiply(&s_quat, &yaw_q, &result);
        s_quat = result;
    }

    imu_quat_normalize(&s_quat);

    /* ===== 7. 四元数 → 欧拉角 ===== */
    imu_euler_t euler;
    imu_quat_to_euler(&s_quat, &euler);

    /* ===== 8. 输出 ===== */
    if (output != NULL) {
        output->quat = s_quat;
        output->euler = euler;
        output->vibration_weight = s_vibration_weight;
        output->motion_weight = s_motion_weight;
        output->adaptive_R = adaptive_R;
        output->adaptive_Q = adaptive_Q;
    }
}

void kalman_reset(void)
{
    kalman_filter_init(&s_roll);
    kalman_filter_init(&s_pitch);
    s_quat.q0 = 1.0f; s_quat.q1 = 0.0f; s_quat.q2 = 0.0f; s_quat.q3 = 0.0f;
    s_yaw = 0.0f;
    s_vibration_weight = 1.0f;
    s_motion_weight = 1.0f;
    ESP_LOGI(TAG, "卡尔曼滤波器已重置");
}
