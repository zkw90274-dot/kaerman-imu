/**
 * @file    ahrs.h
 * @brief   Mahony AHRS 互补滤波器
 * @author  Claude
 * @date    2026-07-21
 * @version 3.0.0
 *
 * @details
 * 基于 Mahony AHRS 算法，PI 互补滤波融合陀螺仪和加速度计
 * 数据流: 原始数据 → Mahony 互补滤波 → 四元数 → 欧拉角
 *
 * 特性:
 *   - 振动自适应 Kp（加速度计幅值偏离 1g 时降低 Kp）
 *   - 误差积分限幅（防止积分饱和）
 *   - Yaw 死区（抑制小角度漂移）
 *   - 四元数输出，支持 ZYX 欧拉角反解
 */

#ifndef __AHRS_H__
#define __AHRS_H__

#include "imu_math.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ==================== 数据结构体 ==================== */

/**
 * @brief Mahony 滤波器输出（四元数 + 欧拉角）
 */
typedef struct {
    imu_quat_t quat;            /* 姿态四元数 */
    imu_euler_t euler;          /* 欧拉角（度） */
    float vibration_weight;     /* 振动自适应权重 (1.0=无振动) */
    float yaw_rate;             /* Yaw 角速度 (dps，补偿后) */
} mahony_output_t;

/* ==================== API 函数 ==================== */

/**
 * @brief   初始化 Mahony 滤波器（用加速度计初始化四元数）
 * @param   ax, ay, az  当前加速度计数据 (g)
 */
void mahony_init(float ax, float ay, float az);

/**
 * @brief   更新 Mahony 滤波器
 * @param   gx, gy, gz  陀螺仪 (dps，已零漂补偿)
 * @param   ax, ay, az  加速度计 (g)
 * @param   dt          采样时间 (秒)
 * @param   output      输出结果
 */
void mahony_update(float gx, float gy, float gz,
                   float ax, float ay, float az,
                   float dt, mahony_output_t *output);

/**
 * @brief   重置滤波器
 */
void mahony_reset(void);

#ifdef __cplusplus
}
#endif

#endif /* __AHRS_H__ */
