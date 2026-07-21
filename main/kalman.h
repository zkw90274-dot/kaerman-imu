/**
 * @file    kalman.h
 * @brief   自适应卡尔曼滤波器
 * @author  Claude
 * @date    2026-07-21
 * @version 3.0.0
 *
 * @details
 * 数据流: 原始数据 → 自适应卡尔曼(Roll/Pitch) → 四元数 → 欧拉角
 *
 * 特性:
 *   - 自适应 R: 振动检测，加速度计幅值偏离 1g 时增大 R
 *   - 自适应 Q: 运动检测，角速度大时增大 Q
 *   - Yaw 陀螺仪积分 + 死区抑制漂移
 *   - 四元数输出，支持 ZYX 欧拉角反解
 */

#ifndef __KALMAN_H__
#define __KALMAN_H__

#include "imu_math.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ==================== 数据结构体 ==================== */

/**
 * @brief 卡尔曼滤波器输出（四元数 + 欧拉角）
 */
typedef struct {
    imu_quat_t quat;            /* 姿态四元数 */
    imu_euler_t euler;          /* 欧拉角（度） */
    float vibration_weight;     /* 振动权重 (1.0=无振动) */
    float motion_weight;        /* 运动权重 (1.0=静止) */
    float adaptive_R;           /* 当前自适应 R 值 */
    float adaptive_Q;           /* 当前自适应 Q 值 */
} kalman_output_t;

/* ==================== API 函数 ==================== */

/**
 * @brief   初始化卡尔曼滤波器
 */
void kalman_init(void);

/**
 * @brief   更新卡尔曼滤波器
 * @param   gx, gy, gz  陀螺仪 (dps)
 * @param   ax, ay, az  加速度计 (g)
 * @param   dt          采样时间 (秒)
 * @param   output      输出结果
 */
void kalman_update(float gx, float gy, float gz,
                   float ax, float ay, float az,
                   float dt, kalman_output_t *output);

/**
 * @brief   重置滤波器
 */
void kalman_reset(void);

#ifdef __cplusplus
}
#endif

#endif /* __KALMAN_H__ */
