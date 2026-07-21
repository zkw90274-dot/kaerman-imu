/**
 * @file    imu_math.h
 * @brief   姿态解算数学函数（跨平台，不依赖 math.h）
 * @author  Claude
 * @date    2026-07-21
 * @version 1.0.0
 *
 * @details
 * 自定义三角函数和快速平方根倒数
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

/* ==================== 函数声明 ==================== */

/**
 * @brief   快速平方根倒数 (Quake III 算法)
 * @details 计算 1/sqrt(x)，精度约 0.17%
 * @param   x   输入值 (必须 > 0)
 * @return  1/sqrt(x)
 */
float imu_inv_sqrt(float x);

/**
 * @brief   安全反正弦函数
 * @details 输入超出 [-1,1] 时自动限幅，不会返回 NaN
 * @param   x   输入值
 * @return  asin(x)，单位: 弧度，范围 [-π/2, π/2]
 */
float imu_safe_asin(float x);

/**
 * @brief   安全反正切函数 (四象限)
 * @details 替代 atan2f，不依赖 <math.h>
 * @param   y   Y 分量
 * @param   x   X 分量
 * @return  atan2(y,x)，单位: 弧度，范围 (-π, π]
 */
float imu_safe_atan2(float y, float x);

/**
 * @brief   绝对值
 */
float imu_fabs(float x);

/**
 * @brief   限幅函数
 */
float imu_constrain(float value, float min_val, float max_val);

#ifdef __cplusplus
}
#endif

#endif /* __IMU_MATH_H__ */
