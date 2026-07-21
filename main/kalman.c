/**
 * @file    kalman.c
 * @brief   自适应卡尔曼滤波器实现
 * @author  Claude
 * @date    2026-07-21
 * @version 2.0.0
 *
 * @details
 * 基于 Arduino 卡尔曼滤波程序移植，添加自适应功能：
 *
 * 自适应特性：
 *   1. 自适应测量噪声 R：检测振动时增大 R，降低对加速度计的信任
 *   2. 自适应过程噪声 Q：检测快速运动时增大 Q，提高响应速度
 *   3. Yaw 陀螺仪积分 + 死区抑制漂移
 *
 * 参数调优基于 719FLY 互补滤波经验：
 *   - 振动阈值：0.08g（偏离 1g 的程度）
 *   - 运动阈值：20 dps（角速度幅值）
 *   - R 自适应范围：0.03 ~ 0.3
 *   - Q 自适应范围：0.001 ~ 0.01
 */

#include "kalman.h"
#include "imu_math.h"
#include "esp_log.h"
#include <string.h>

static const char *TAG = "kalman";

/* ==================== 默认参数（基础值） ==================== */

#define DEFAULT_Q_ANGLE     0.001f      // 过程噪声（角度）
#define DEFAULT_Q_BIAS      0.003f      // 过程噪声（偏差）
#define DEFAULT_R_MEASURE   0.03f       // 测量噪声

/* ==================== 自适应参数 ==================== */

/*
 * 振动检测原理（与互补滤波一致）：
 *   静止时加速度计幅值 ≈ 1.0g
 *   振动时幅值偏离 1.0g
 *   偏离越大 → R 越大 → 越不信任加速度计
 */
#define VIBRATION_THRESHOLD     0.08f   // 振动检测阈值 (g)
#define VIBRATION_R_SCALE       10.0f   // 振动时 R 放大倍数
#define VIBRATION_SMOOTH        0.1f    // 振动权重低通滤波系数

/*
 * 运动检测原理：
 *   角速度幅值 = |gx| + |gy| + |gz|
 *   幅值越大 → Q 越大 → 响应越快
 */
#define MOTION_THRESHOLD        20.0f   // 运动检测阈值 (dps)
#define MOTION_Q_SCALE          5.0f    // 运动时 Q 放大倍数
#define MOTION_SMOOTH           0.2f    // 运动权重低通滤波系数

/* ==================== Yaw 参数 ==================== */

#define YAW_DEADZONE    0.5f            // Yaw 死区 (°/s)

/* ==================== 模块状态 ==================== */

static kalman_filter_t s_roll;          // Roll 角卡尔曼滤波器
static kalman_filter_t s_pitch;         // Pitch 角卡尔曼滤波器
static kalman_output_t s_output;        // 输出缓存
static uint8_t s_inited = 0;            // 初始化标志
static float s_yaw = 0.0f;             // Yaw 角（陀螺仪积分）

/* 自适应状态 */
static float s_vibration_weight = 1.0f; // 振动权重 (1.0=无振动, 0.1=强振动)
static float s_motion_weight = 1.0f;    // 运动权重 (1.0=静止, 5.0=快速运动)
static float s_R_base = DEFAULT_R_MEASURE;
static float s_Q_base = DEFAULT_Q_ANGLE;

/* ==================== 内部函数 ==================== */

/**
 * @brief   初始化单个卡尔曼滤波器
 */
static void kalman_filter_init(kalman_filter_t *kf)
{
    kf->angle = 0.0f;
    kf->rate = 0.0f;
    kf->bias = 0.0f;

    // 初始化协方差矩阵
    kf->P[0][0] = 1.0f;
    kf->P[0][1] = 0.0f;
    kf->P[1][0] = 0.0f;
    kf->P[1][1] = 1.0f;

    // 设置默认参数
    kf->Q_angle = DEFAULT_Q_ANGLE;
    kf->Q_bias = DEFAULT_Q_BIAS;
    kf->R_measure = DEFAULT_R_MEASURE;
}

/**
 * @brief   更新单个卡尔曼滤波器（带自适应）
 */
static float kalman_filter_update(kalman_filter_t *kf, float new_angle, float new_rate, float dt,
                                   float adaptive_Q, float adaptive_R)
{
    float S, K[2];
    float y;
    float P00_temp, P01_temp;

    kf->dt = dt;

    /* Step 1: 先验状态预测 */
    kf->rate = new_rate - kf->bias;
    kf->angle += dt * kf->rate;

    /* Step 2: 更新先验协方差矩阵（使用自适应 Q） */
    kf->P[0][0] += dt * (dt * kf->P[1][1] - kf->P[0][1] - kf->P[1][0] + adaptive_Q);
    kf->P[0][1] -= dt * kf->P[1][1];
    kf->P[1][0] -= dt * kf->P[1][1];
    kf->P[1][1] += kf->Q_bias * dt;

    /* Step 3: 计算卡尔曼增益（使用自适应 R） */
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

/* ==================== 公共 API 实现 ==================== */

/**
 * @brief   初始化卡尔曼滤波器
 */
void kalman_init(void)
{
    ESP_LOGI(TAG, "=== 初始化自适应卡尔曼滤波器 ===");

    kalman_filter_init(&s_roll);
    kalman_filter_init(&s_pitch);

    s_yaw = 0.0f;
    s_vibration_weight = 1.0f;
    s_motion_weight = 1.0f;
    s_R_base = DEFAULT_R_MEASURE;
    s_Q_base = DEFAULT_Q_ANGLE;

    memset(&s_output, 0, sizeof(s_output));
    s_inited = 1;

    ESP_LOGI(TAG, "基础参数: Q_angle=%.4f, Q_bias=%.4f, R_measure=%.4f",
             (double)DEFAULT_Q_ANGLE, (double)DEFAULT_Q_BIAS, (double)DEFAULT_R_MEASURE);
    ESP_LOGI(TAG, "自适应参数: 振动阈值=%.2fg, 运动阈值=%.1fdps",
             (double)VIBRATION_THRESHOLD, (double)MOTION_THRESHOLD);
}

/**
 * @brief   更新卡尔曼滤波器
 */
void kalman_update(float gx, float gy, float gz,
                   float ax, float ay, float az,
                   float dt, kalman_output_t *output)
{
    if (!s_inited) {
        kalman_init();
    }

    float acc_roll, acc_pitch;

    /* ===== 1. 振动检测：自适应 R（测量噪声） ===== */
    // 计算加速度计幅值（单位：g）
    float acc_mag_sq = ax * ax + ay * ay + az * az;
    float acc_mag = 0.0f;
    if (acc_mag_sq > 0.0f) {
        acc_mag = imu_inv_sqrt(acc_mag_sq);
        acc_mag = 1.0f / acc_mag;  // 转换为实际幅值
    }
    // 偏离 1g 的程度（输入数据单位已经是 g）
    float acc_deviation = imu_fabs(acc_mag - 1.0f);

    // 计算目标振动权重
    float target_vibration_weight;
    if (acc_deviation < VIBRATION_THRESHOLD) {
        target_vibration_weight = 1.0f;  // 稳定，使用基础 R
    } else {
        // 线性映射：偏离越大，R 越大
        float ratio = (acc_deviation - VIBRATION_THRESHOLD) / (VIBRATION_THRESHOLD * 4.0f);
        if (ratio > 1.0f) ratio = 1.0f;
        target_vibration_weight = 1.0f + ratio * VIBRATION_R_SCALE;
    }

    // 低通滤波平滑权重变化
    s_vibration_weight += VIBRATION_SMOOTH * (target_vibration_weight - s_vibration_weight);

    // 自适应 R：振动时 R 增大
    float adaptive_R = s_R_base * s_vibration_weight;

    /* ===== 2. 运动检测：自适应 Q（过程噪声） ===== */
    float gyro_magnitude = imu_fabs(gx) + imu_fabs(gy) + imu_fabs(gz);

    // 计算目标运动权重
    float target_motion_weight;
    if (gyro_magnitude < MOTION_THRESHOLD) {
        target_motion_weight = 1.0f;  // 静止，使用基础 Q
    } else {
        // 线性映射：角速度越大，Q 越大
        float ratio = (gyro_magnitude - MOTION_THRESHOLD) / MOTION_THRESHOLD;
        if (ratio > 1.0f) ratio = 1.0f;
        target_motion_weight = 1.0f + ratio * MOTION_Q_SCALE;
    }

    // 低通滤波平滑权重变化
    s_motion_weight += MOTION_SMOOTH * (target_motion_weight - s_motion_weight);

    // 自适应 Q：快速运动时 Q 增大
    float adaptive_Q = s_Q_base * s_motion_weight;

    /* ===== 3. 计算加速度计角度 ===== */
    // Roll: atan2(ay, az)
    acc_roll = imu_safe_atan2(ay, az) * IMU_RAD2DEG;

    // Pitch: -atan2(ax, sqrt(ay² + az²))
    acc_pitch = -imu_safe_atan2(ax, imu_inv_sqrt(ay * ay + az * az)) * IMU_RAD2DEG;

    /* ===== 4. 更新卡尔曼滤波器 ===== */
    s_output.roll = kalman_filter_update(&s_roll, acc_roll, gx, dt, adaptive_Q, adaptive_R);
    s_output.pitch = kalman_filter_update(&s_pitch, acc_pitch, gy, dt, adaptive_Q, adaptive_R);

    /* ===== 5. Yaw 陀螺仪积分 + 死区 ===== */
    if (gz > YAW_DEADZONE || gz < -YAW_DEADZONE) {
        s_yaw += gz * dt;
    }
    s_output.yaw = s_yaw;

    /* ===== 6. 保存数据 ===== */
    s_output.roll_rate = gx;
    s_output.pitch_rate = gy;
    s_output.vibration_weight = s_vibration_weight;
    s_output.motion_weight = s_motion_weight;
    s_output.adaptive_R = adaptive_R;
    s_output.adaptive_Q = adaptive_Q;

    // 输出结果
    if (output != NULL) {
        *output = s_output;
    }
}

/**
 * @brief   获取当前输出
 */
void kalman_get_output(kalman_output_t *output)
{
    if (output != NULL) {
        *output = s_output;
    }
}

/**
 * @brief   重置滤波器
 */
void kalman_reset(void)
{
    kalman_filter_init(&s_roll);
    kalman_filter_init(&s_pitch);
    s_yaw = 0.0f;
    s_vibration_weight = 1.0f;
    s_motion_weight = 1.0f;
    memset(&s_output, 0, sizeof(s_output));
    ESP_LOGI(TAG, "卡尔曼滤波器已重置");
}

/**
 * @brief   设置基础参数
 */
void kalman_set_params(float Q_angle, float Q_bias, float R_measure)
{
    s_Q_base = Q_angle;
    s_roll.Q_bias = Q_bias;
    s_pitch.Q_bias = Q_bias;
    s_R_base = R_measure;

    ESP_LOGI(TAG, "基础参数设置: Q_angle=%.4f, Q_bias=%.4f, R_measure=%.4f",
             (double)Q_angle, (double)Q_bias, (double)R_measure);
}
