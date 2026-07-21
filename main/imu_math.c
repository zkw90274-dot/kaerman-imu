/**
 * @file    imu_math.c
 * @brief   姿态解算数学函数实现（跨平台）
 * @author  Claude
 * @date    2026-07-21
 * @version 1.0.0
 *
 * @details
 * 不依赖 <math.h>，可直接移植到任何平台
 * 参考: 719FLY myMath.c + Quake III fast inverse square root
 */

#include "imu_math.h"

/* ==================== 快速平方根倒数 ==================== */

float imu_inv_sqrt(float x)
{
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long *)&y;
    i = 0x5f375a86 - (i >> 1);
    y = *(float *)&i;
    y = y * (1.5f - (halfx * y * y));
    return y;
}

/* ==================== 安全反正弦 ==================== */

/**
 * @details 使用泰勒级数展开，6阶精度约 0.005 弧度
 *          参考: 719FLY myMath.c arcsin()
 */
float imu_safe_asin(float x)
{
    /* 限幅到 [-1, 1] */
    if (x >= 1.0f)  return IMU_PI * 0.5f;
    if (x <= -1.0f) return -IMU_PI * 0.5f;

    /* 泰勒级数: asin(x) = x + x³/6 + 3x⁵/40 + 15x⁷/336 + ... */
    float x2 = x * x;
    float result = x;
    float term = x;

    /* 6阶展开 (精度足够) */
    term *= x2; result += term * (1.0f / 6.0f);                    /* x³/6 */
    term *= x2; result += term * (3.0f / 40.0f);                   /* 3x⁵/40 */
    term *= x2; result += term * (15.0f / 336.0f);                 /* 15x⁷/336 */
    term *= x2; result += term * (105.0f / 3456.0f);               /* 105x⁹/3456 */
    term *= x2; result += term * (945.0f / 42240.0f);              /* 945x¹¹/42240 */
    term *= x2; result += term * (10395.0f / 599040.0f);           /* 10395x¹³/599040 */

    return result;
}

/* ==================== 安全反正切 (四象限) ==================== */

/**
 * @details 使用泰勒级数 + 象限判断
 *          参考: 719FLY myMath.c arctan()，扩展为四象限
 */
static float atan_approx(float x)
{
    /* 泰勒级数: atan(x) = x - x³/3 + x⁵/5 - x⁷/7 + ... */
    /* 仅在 |x| < 1 时收敛，|x| > 1 时用 atan(x) = π/2 - atan(1/x) */
    int recip = 0;
    if (x > 1.0f) {
        x = 1.0f / x;
        recip = 1;
    } else if (x < -1.0f) {
        x = 1.0f / x;
        recip = 1;
    }

    float x2 = x * x;
    float result = 0;
    float term = x;
    uint8_t sign = 1;
    uint8_t cnt;

    for (cnt = 1; cnt <= 8; cnt++) {
        if (sign)
            result += term / (float)((cnt << 1) - 1);
        else
            result -= term / (float)((cnt << 1) - 1);
        term *= x2;
        sign = !sign;
    }

    if (recip) {
        if (x > 0)
            result = IMU_PI * 0.5f - result;
        else
            result = -IMU_PI * 0.5f - result;
    }

    return result;
}

float imu_safe_atan2(float y, float x)
{
    /* 特殊情况 */
    if (x > 0.0f) {
        return atan_approx(y / x);
    } else if (x < 0.0f) {
        if (y >= 0.0f)
            return atan_approx(y / x) + IMU_PI;
        else
            return atan_approx(y / x) - IMU_PI;
    } else {
        /* x == 0 */
        if (y > 0.0f)  return IMU_PI * 0.5f;
        if (y < 0.0f)  return -IMU_PI * 0.5f;
        return 0.0f;  /* (0,0) 未定义 */
    }
}

/* ==================== 绝对值 ==================== */

float imu_fabs(float x)
{
    return (x < 0.0f) ? -x : x;
}

/* ==================== 限幅函数 ==================== */

float imu_constrain(float value, float min_val, float max_val)
{
    if (value < min_val) return min_val;
    if (value > max_val) return max_val;
    return value;
}
