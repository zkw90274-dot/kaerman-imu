/**
 * @file    imu_math.h
 * @brief   姿态解算数学函数（跨平台，不依赖 math.h）
 * @author  Claude
 * @date    2026-07-21
 * @version 2.0.0
 *
 * @details
 * 自定义三角函数、快速平方根倒数和四元数工具
 * 参考: 719FLY myMath.c + Quake III invSqrt
 * 可直接移植到任何平台（ESP32/STM32/STC32）
 */

#ifndef __IMU_MATH_H__
#define __IMU_MATH_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ==================== 常量定义 ==================== */

#define IMU_PI          3.14159265358979323846f
#define IMU_RAD2DEG     57.29577951308232f      /* 180/π */
#define IMU_DEG2RAD     0.017453292519943295f   /* π/180 */
#define IMU_G           9.80665f                /* 重力加速度 m/s² */

/* ==================== 四元数结构体 ==================== */

/**
 * @brief 四元数
 */
typedef struct {
    float q0;               /* w */
    float q1;               /* x */
    float q2;               /* y */
    float q3;               /* z */
} imu_quat_t;

/**
 * @brief 欧拉角（度）
 */
typedef struct {
    float roll;             /* 横滚角 */
    float pitch;            /* 俯仰角 */
    float yaw;              /* 偏航角 */
} imu_euler_t;

/* ==================== 通用数学函数 ==================== */

float imu_inv_sqrt(float x);
float imu_safe_sin(float x);
float imu_safe_cos(float x);
float imu_safe_asin(float x);
float imu_safe_atan2(float y, float x);
float imu_fabs(float x);
float imu_constrain(float value, float min_val, float max_val);

/* ==================== 四元数工具函数 ==================== */

/**
 * @brief   欧拉角 → 四元数（ZYX 旋转顺序）
 * @param   roll    横滚角（度）
 * @param   pitch   俯仰角（度）
 * @param   yaw     偏航角（度）
 * @param   q       输出四元数
 */
void imu_euler_to_quat(float roll, float pitch, float yaw, imu_quat_t *q);

/**
 * @brief   四元数 → 欧拉角（ZYX 旋转顺序）
 * @param   q       输入四元数（必须已归一化）
 * @param   euler   输出欧拉角（度）
 */
void imu_quat_to_euler(const imu_quat_t *q, imu_euler_t *euler);

/**
 * @brief   四元数乘法: result = a * b
 */
void imu_quat_multiply(const imu_quat_t *a, const imu_quat_t *b, imu_quat_t *result);

/**
 * @brief   四元数归一化
 */
void imu_quat_normalize(imu_quat_t *q);

#ifdef __cplusplus
}
#endif

#endif /* __IMU_MATH_H__ */
