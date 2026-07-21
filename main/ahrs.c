/**
 * @file    ahrs.c
 * @brief   姿态航向参考系统（AHRS）实现
 * @author  Claude
 * @date    2026-07-21
 * @version 2.0.0
 *
 * @details
 * 基于 719 飞行器实验室的四元数姿态解算算法
 *
 * 核心改进点:
 *   1. 陀螺仪零漂自动标定（500 样本）
 *   2. 误差积分限幅 ±0.5（防止积分饱和）
 *   3. 振动自适应 Kp（检测加速度计幅值偏离 1g 程度，动态降低 Kp）
 *   4. Yaw 死区 0.5°/s（抑制小角度漂移）
 *   5. 使用 imu_math 自定义函数（不依赖 math.h）
 *   6. 精确采样率控制（200Hz）
 */

#include "ahrs.h"
#include "imu_math.h"
#include "icm42688.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "ahrs";

/* ==================== 常量定义 ==================== */

/* 互补滤波 PI 参数 (对齐 719FLY 经验值) */
#define AHRS_KP          1.5f               /* 比例增益: 控制加速度计校正收敛速率 */
#define AHRS_KI          0.005f             /* 积分增益: 控制陀螺仪偏差收敛速率 */

/* 误差积分限幅 (防止积分饱和) */
#define AHRS_INTEGRAL_LIMIT  0.5f

/* 零漂标定参数 */
#define CALIBRATE_SAMPLE_COUNT  500         /* 标定采样数 */

/* Yaw 角积分死区 (°/s): 低于此值不积分，抑制漂移 */
#define YAW_DEADZONE    0.5f

/* ==================== 振动自适应参数 ==================== */

/*
 * 振动检测原理:
 *   静止时加速度计幅值 ≈ 1.0g
 *   振动时幅值偏离 1.0g，偏离量 = |acc_mag - 1.0|
 *   偏离越大 → 振动越强 → Kp 越小 (更信任陀螺仪)
 *
 *   Kp_adaptive = Kp * weight
 *   weight = 1.0 (无振动) → 0.05 (强振动)
 */
#define VIBRATION_THRESHOLD     0.08f       /* 振动检测阈值 (g) */
#define VIBRATION_KP_MIN        0.1f        /* 强振动时 Kp 最小比例 */
#define VIBRATION_SMOOTH        0.1f        /* 振动权重低通滤波系数 */

/* ==================== 模块状态 ==================== */

/* 四元数状态 */
static float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;

/* PI 误差积分项 */
static float exInt = 0.0f, eyInt = 0.0f, ezInt = 0.0f;

/* 振动自适应权重 */
static float s_vibration_weight = 1.0f;

/* 零漂标定临时变量 */
static float s_cal_sum_x = 0, s_cal_sum_y = 0, s_cal_sum_z = 0;
static uint16_t s_cal_cnt = 0;

/* 完整状态 */
static ahrs_state_t s_state = {0};

/* ==================== 初始化 ==================== */

void ahrs_init(void)
{
    ESP_LOGI(TAG, "=== 初始化 AHRS ===");

    // 初始化 ICM42688
    if (icm42688_init(NULL) != 0) {
        ESP_LOGE(TAG, "ICM42688 init failed!");
        return;
    }

    vTaskDelay(pdMS_TO_TICKS(50));

    ahrs_angle_clear();
    ESP_LOGI(TAG, "=== AHRS 初始化完成 ===");
}

void ahrs_angle_clear(void)
{
    q0 = 1.0f;
    q1 = 0.0f;
    q2 = 0.0f;
    q3 = 0.0f;
    exInt = 0.0f;
    eyInt = 0.0f;
    ezInt = 0.0f;
    s_vibration_weight = 1.0f;
    s_state.euler.yaw = 0.0f;
    ESP_LOGI(TAG, "四元数已重置 (角度清零)");
}

/* ==================== 零漂标定 ==================== */

int8_t ahrs_calibrate_null_drift(void)
{
    icm42688_sensor_data_t data;

    icm42688_get_all_data(&data);
    s_cal_sum_x += data.gx;
    s_cal_sum_y += data.gy;
    s_cal_sum_z += data.gz;
    s_cal_cnt++;

    if (s_cal_cnt < CALIBRATE_SAMPLE_COUNT) {
        if ((s_cal_cnt % 100) == 0) {
            ESP_LOGI(TAG, "标定中... %d/%d", s_cal_cnt, CALIBRATE_SAMPLE_COUNT);
        }
        return 0;
    }

    s_state.null_drift_x = s_cal_sum_x / CALIBRATE_SAMPLE_COUNT;
    s_state.null_drift_y = s_cal_sum_y / CALIBRATE_SAMPLE_COUNT;
    s_state.null_drift_z = s_cal_sum_z / CALIBRATE_SAMPLE_COUNT;
    s_state.calibrated = 1;

    ESP_LOGI(TAG, "零漂标定完成: X=%.4f, Y=%.4f, Z=%.4f °/s (%d 样本)",
             (double)s_state.null_drift_x,
             (double)s_state.null_drift_y,
             (double)s_state.null_drift_z,
             CALIBRATE_SAMPLE_COUNT);

    s_cal_cnt = 0;
    s_cal_sum_x = 0;
    s_cal_sum_y = 0;
    s_cal_sum_z = 0;
    return 1;
}

/* ==================== 姿态更新 (核心) ==================== */

void ahrs_update(void)
{
    float norm;
    float vx, vy, vz;
    float ex, ey, ez;
    float gx, gy, gz;
    icm42688_sensor_data_t data;

    /* ① 读取传感器原始数据 */
    icm42688_get_all_data(&data);

    /* ② 单位转换: dps → rad/s，零漂补偿 */
    gx = (data.gx - s_state.null_drift_x) * IMU_DEG2RAD;
    gy = (data.gy - s_state.null_drift_y) * IMU_DEG2RAD;
    gz = (data.gz - s_state.null_drift_z) * IMU_DEG2RAD;

    /* 加速度计: g → m/s² */
    float ax = data.ax * IMU_G;
    float ay = data.ay * IMU_G;
    float az = data.az * IMU_G;

    /* ③ 振动检测: 加速度计幅值偏离 1g 的程度 */
    float acc_mag = imu_inv_sqrt(ax * ax + ay * ay + az * az);
    float acc_deviation = imu_fabs(1.0f - acc_mag * IMU_G);

    /* 计算目标权重 */
    float target_weight;
    if (acc_deviation < VIBRATION_THRESHOLD) {
        target_weight = 1.0f;
    } else {
        float ratio = (acc_deviation - VIBRATION_THRESHOLD) / (VIBRATION_THRESHOLD * 4.0f);
        if (ratio > 1.0f) ratio = 1.0f;
        target_weight = 1.0f - ratio * (1.0f - VIBRATION_KP_MIN);
    }

    /* 低通滤波平滑权重变化 */
    s_vibration_weight += VIBRATION_SMOOTH * (target_weight - s_vibration_weight);

    /* 自适应 Kp */
    float kp_adaptive = AHRS_KP * s_vibration_weight;

    /* ④ 加速度计归一化 (自由落体时跳过) */
    if (ax * ay * az == 0.0f)
        return;

    norm = imu_inv_sqrt(ax * ax + ay * ay + az * az);
    ax *= norm;
    ay *= norm;
    az *= norm;

    /* ⑤ 提取姿态矩阵中的重力分量 (机体坐标系) */
    vx = 2.0f * (q1 * q3 - q0 * q2);
    vy = 2.0f * (q0 * q1 + q2 * q3);
    vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

    /* ⑥ 叉乘计算姿态误差 */
    ex = (ay * vz - az * vy);
    ey = (az * vx - ax * vz);
    ez = (ax * vy - ay * vx);

    /* ⑦ 误差积分 (带限幅，振动时积分也降低权重) */
    exInt += ex * AHRS_KI * s_vibration_weight;
    eyInt += ey * AHRS_KI * s_vibration_weight;
    ezInt += ez * AHRS_KI * s_vibration_weight;

    /* 积分限幅 */
    exInt = imu_constrain(exInt, -AHRS_INTEGRAL_LIMIT, AHRS_INTEGRAL_LIMIT);
    eyInt = imu_constrain(eyInt, -AHRS_INTEGRAL_LIMIT, AHRS_INTEGRAL_LIMIT);
    ezInt = imu_constrain(ezInt, -AHRS_INTEGRAL_LIMIT, AHRS_INTEGRAL_LIMIT);

    /* ⑧ PI 互补滤波修正陀螺仪 (使用自适应 Kp) */
    gx += kp_adaptive * ex + exInt;
    gy += kp_adaptive * ey + eyInt;
    gz += kp_adaptive * ez + ezInt;

    /* ⑨ 四元数微分方程 (一阶龙格库塔法) */
    float halfT = 0.5f * AHRS_DT;
    q0 += (-q1 * gx - q2 * gy - q3 * gz) * halfT;
    q1 += ( q0 * gx + q2 * gz - q3 * gy) * halfT;
    q2 += ( q0 * gy - q1 * gz + q3 * gx) * halfT;
    q3 += ( q0 * gz + q1 * gy - q2 * gx) * halfT;

    /* ⑩ 四元数归一化 */
    norm = imu_inv_sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= norm;
    q1 *= norm;
    q2 *= norm;
    q3 *= norm;

    /* ⑪ 欧拉角反解 */
    float q0q0 = q0 * q0;
    float q1q3 = q1 * q3;
    float q0q2 = q0 * q2;
    float q2q3 = q2 * q3;
    float q0q1 = q0 * q1;

    /* Roll (横滚角) */
    s_state.euler.roll = -imu_safe_asin(2.0f * (q1q3 - q0q2)) * IMU_RAD2DEG;

    /* Pitch (俯仰角) */
    s_state.euler.pitch = -imu_safe_atan2(2.0f * q2q3 + 2.0f * q0q1,
                                           q0q0 - q1 * q1 - q2 * q2 + q3 * q3) * IMU_RAD2DEG;

    /* Yaw (偏航角): 陀螺仪积分 + 死区 */
    float gz_dps = data.gz - s_state.null_drift_z;
    if (gz_dps > YAW_DEADZONE || gz_dps < -YAW_DEADZONE) {
        s_state.euler.yaw += gz_dps * AHRS_DT;
    }

    /* ⑫ 保存数据 */
    s_state.gyro_x = data.gx - s_state.null_drift_x;
    s_state.gyro_y = data.gy - s_state.null_drift_y;
    s_state.gyro_z = gz_dps;
    s_state.vibration_weight = s_vibration_weight;

    /* 保存四元数 */
    s_state.quat.q0 = q0;
    s_state.quat.q1 = q1;
    s_state.quat.q2 = q2;
    s_state.quat.q3 = q3;
}

/* ==================== 启动入口 ==================== */

void ahrs_prepare(void)
{
    ahrs_init();

    ESP_LOGI(TAG, "开始零漂标定，保持传感器静止...");
    while (ahrs_calibrate_null_drift() == 0) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    ESP_LOGI(TAG, "零漂标定完成");
}

/* ==================== 数据获取接口 ==================== */

void ahrs_get_euler(ahrs_euler_t *euler)
{
    if (euler != NULL) {
        *euler = s_state.euler;
    }
}

void ahrs_get_quaternion(ahrs_quat_t *quat)
{
    if (quat != NULL) {
        *quat = s_state.quat;
    }
}

void ahrs_get_state(ahrs_state_t *state)
{
    if (state != NULL) {
        *state = s_state;
    }
}

void ahrs_set_null_drift(float drift_x, float drift_y, float drift_z)
{
    s_state.null_drift_x = drift_x;
    s_state.null_drift_y = drift_y;
    s_state.null_drift_z = drift_z;
    s_state.calibrated = 1;
    ESP_LOGI(TAG, "零漂手动设置: X=%.4f, Y=%.4f, Z=%.4f",
             (double)drift_x, (double)drift_y, (double)drift_z);
}
