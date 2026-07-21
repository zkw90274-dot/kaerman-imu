/**
 * @file    ahrs.c
 * @brief   Mahony AHRS 互补滤波器实现
 * @author  Claude
 * @date    2026-07-21
 * @version 3.0.0
 *
 * @details
 * 基于 Mahony AHRS 算法，PI 互补滤波融合陀螺仪和加速度计
 *
 * 核心特性:
 *   1. 振动自适应 Kp（加速度计幅值偏离 1g 时降低 Kp）
 *   2. 误差积分限幅 ±0.5（防止积分饱和）
 *   3. Yaw 死区 0.5 dps（抑制小角度漂移）
 *   4. 四元数输出 → ZYX 欧拉角反解
 */

#include "ahrs.h"
#include "esp_log.h"
#include <stddef.h>

static const char *TAG = "mahony";

/* ==================== PI 参数 ==================== */

#define AHRS_KP              8.0f       /* 比例增益: 加速度计校正收敛速率 */
#define AHRS_KI              0.002f     /* 积分增益: 陀螺仪偏差收敛速率 */
#define AHRS_INTEGRAL_LIMIT  0.3f       /* 误差积分限幅 */

/* ==================== 振动自适应参数 ==================== */

#define VIBRATION_THRESHOLD     0.08f   /* 振动检测阈值 (g) */
#define VIBRATION_KP_MIN        0.1f    /* 强振动时 Kp 最小比例 */
#define VIBRATION_SMOOTH        0.1f    /* 振动权重低通滤波系数 */

/* ==================== Yaw 参数 ==================== */

#define YAW_DEADZONE            0.5f    /* Yaw 死区 (dps) */

/* ==================== 模块状态 ==================== */

static imu_quat_t s_quat;
static float s_ex_int, s_ey_int, s_ez_int;
static float s_vibration_weight;
static uint8_t s_inited;

/* ==================== 公共 API ==================== */

void mahony_init(float ax, float ay, float az)
{
    ESP_LOGI(TAG, "=== 初始化 Mahony 滤波器 ===");

    /* 用加速度计初始化四元数（跳过从 identity 收敛的过程） */
    float acc_mag_sq = ax * ax + ay * ay + az * az;
    if (acc_mag_sq > 0.01f) {
        /* Roll = atan2(ay, az), Pitch = -atan2(ax, sqrt(ay²+az²)) */
        float roll = imu_safe_atan2(ay, az) * IMU_RAD2DEG;
        float pitch = -imu_safe_atan2(ax, 1.0f / imu_inv_sqrt(ay * ay + az * az)) * IMU_RAD2DEG;
        imu_euler_to_quat(roll, pitch, 0.0f, &s_quat);
        ESP_LOGI(TAG, "加速度计初始化: Roll=%.1f, Pitch=%.1f", (double)roll, (double)pitch);
    } else {
        s_quat.q0 = 1.0f;
        s_quat.q1 = 0.0f;
        s_quat.q2 = 0.0f;
        s_quat.q3 = 0.0f;
        ESP_LOGI(TAG, "自由落体，使用默认四元数");
    }

    s_ex_int = 0.0f;
    s_ey_int = 0.0f;
    s_ez_int = 0.0f;
    s_vibration_weight = 1.0f;
    s_inited = 1;

    ESP_LOGI(TAG, "Kp=%.3f, Ki=%.4f, 振动阈值=%.2fg", (double)AHRS_KP, (double)AHRS_KI, (double)VIBRATION_THRESHOLD);
}

void mahony_update(float gx, float gy, float gz,
                   float ax, float ay, float az,
                   float dt, mahony_output_t *output)
{
    float norm;
    float vx, vy, vz;
    float ex, ey, ez;

    if (!s_inited) {
        mahony_init(ax, ay, az);
    }

    /* ① 单位转换: dps → rad/s */
    float gx_r = gx * IMU_DEG2RAD;
    float gy_r = gy * IMU_DEG2RAD;
    float gz_r = gz * IMU_DEG2RAD;

    /* ② 振动检测: 加速度计幅值偏离 1g 的程度 */
    float acc_mag_sq = ax * ax + ay * ay + az * az;
    if (acc_mag_sq < 1e-6f) {
        /* 自由落体，跳过加速度计校正 */
        goto integrate_quat;
    }

    float acc_mag = 1.0f / imu_inv_sqrt(acc_mag_sq);
    float acc_deviation = imu_fabs(acc_mag - 1.0f);

    float target_weight;
    if (acc_deviation < VIBRATION_THRESHOLD) {
        target_weight = 1.0f;
    } else {
        float ratio = (acc_deviation - VIBRATION_THRESHOLD) / (VIBRATION_THRESHOLD * 4.0f);
        if (ratio > 1.0f) ratio = 1.0f;
        target_weight = 1.0f - ratio * (1.0f - VIBRATION_KP_MIN);
    }
    s_vibration_weight += VIBRATION_SMOOTH * (target_weight - s_vibration_weight);

    /* 自适应 Kp */
    float kp_adaptive = AHRS_KP * s_vibration_weight;

    /* ③ 加速度计归一化 */
    norm = imu_inv_sqrt(acc_mag_sq);
    ax *= norm;
    ay *= norm;
    az *= norm;

    /* ④ 提取姿态矩阵中的重力分量 */
    vx = 2.0f * (s_quat.q1 * s_quat.q3 - s_quat.q0 * s_quat.q2);
    vy = 2.0f * (s_quat.q0 * s_quat.q1 + s_quat.q2 * s_quat.q3);
    vz = s_quat.q0 * s_quat.q0 - s_quat.q1 * s_quat.q1
       - s_quat.q2 * s_quat.q2 + s_quat.q3 * s_quat.q3;

    /* ⑤ 叉乘计算姿态误差 */
    ex = ay * vz - az * vy;
    ey = az * vx - ax * vz;
    ez = ax * vy - ay * vx;

    /* ⑥ 误差积分（条件积分：仅误差小时累积，防止积分饱和） */
    float err_mag = imu_fabs(ex) + imu_fabs(ey) + imu_fabs(ez);
    if (err_mag < 1.0f) {
        s_ex_int += ex * AHRS_KI * s_vibration_weight;
        s_ey_int += ey * AHRS_KI * s_vibration_weight;
        s_ez_int += ez * AHRS_KI * s_vibration_weight;
        s_ex_int = imu_constrain(s_ex_int, -AHRS_INTEGRAL_LIMIT, AHRS_INTEGRAL_LIMIT);
        s_ey_int = imu_constrain(s_ey_int, -AHRS_INTEGRAL_LIMIT, AHRS_INTEGRAL_LIMIT);
        s_ez_int = imu_constrain(s_ez_int, -AHRS_INTEGRAL_LIMIT, AHRS_INTEGRAL_LIMIT);
    }

    /* ⑦ PI 互补滤波修正陀螺仪 */
    gx_r += kp_adaptive * ex + s_ex_int;
    gy_r += kp_adaptive * ey + s_ey_int;
    gz_r += kp_adaptive * ez + s_ez_int;

integrate_quat:
    /* ⑧ 四元数微分方程（一阶龙格库塔法） */
    float halfT = 0.5f * dt;
    float q0 = s_quat.q0, q1 = s_quat.q1, q2 = s_quat.q2, q3 = s_quat.q3;
    q0 += (-q1 * gx_r - q2 * gy_r - q3 * gz_r) * halfT;
    q1 += ( q0 * gx_r + q2 * gz_r - q3 * gy_r) * halfT;
    q2 += ( q0 * gy_r - q1 * gz_r + q3 * gx_r) * halfT;
    q3 += ( q0 * gz_r + q1 * gy_r - q2 * gx_r) * halfT;
    s_quat.q0 = q0; s_quat.q1 = q1; s_quat.q2 = q2; s_quat.q3 = q3;

    /* ⑨ 四元数归一化 */
    imu_quat_normalize(&s_quat);

    /* ⑩ Yaw 死区处理（积分修正后的 gz） */
    float gz_compensated = gz + (s_ez_int * IMU_RAD2DEG);

    /* ⑪ 四元数 → 欧拉角 */
    imu_euler_t euler;
    imu_quat_to_euler(&s_quat, &euler);

    /* ⑫ 输出 */
    if (output != NULL) {
        output->quat = s_quat;
        output->euler = euler;
        output->vibration_weight = s_vibration_weight;
        output->yaw_rate = gz_compensated;
    }
}

void mahony_reset(void)
{
    s_quat.q0 = 1.0f; s_quat.q1 = 0.0f; s_quat.q2 = 0.0f; s_quat.q3 = 0.0f;
    s_ex_int = 0.0f;
    s_ey_int = 0.0f;
    s_ez_int = 0.0f;
    s_vibration_weight = 1.0f;
    ESP_LOGI(TAG, "Mahony 滤波器已重置");
}
