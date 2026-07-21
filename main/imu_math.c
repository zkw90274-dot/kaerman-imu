/**
 * @file    imu_math.c
 * @brief   姿态解算数学函数实现（跨平台）
 * @author  Claude
 * @date    2026-07-21
 * @version 2.0.0
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

/* ==================== 安全正弦/余弦 ==================== */

/**
 * @details 泰勒级数展开，范围归约到 [-π, π]
 *          sin(x) = x - x³/6 + x⁵/120 - x⁷/5040
 */
float imu_safe_sin(float x)
{
    /* 范围归约到 [-π, π] */
    while (x > IMU_PI)  x -= 2.0f * IMU_PI;
    while (x < -IMU_PI) x += 2.0f * IMU_PI;

    float x2 = x * x;
    float result = x;
    float term = x;

    term *= x2; result -= term / 6.0f;              /* -x³/6 */
    term *= x2; result += term / 120.0f;            /* +x⁵/120 */
    term *= x2; result -= term / 5040.0f;           /* -x⁷/5040 */
    term *= x2; result += term / 362880.0f;         /* +x⁹/362880 */

    return result;
}

/**
 * @details cos(x) = sin(x + π/2)
 */
float imu_safe_cos(float x)
{
    return imu_safe_sin(x + IMU_PI * 0.5f);
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

/* ==================== 四元数工具函数 ==================== */

void imu_euler_to_quat(float roll, float pitch, float yaw, imu_quat_t *q)
{
    /* ZYX 旋转顺序: 先 Yaw(Z), 再 Pitch(Y), 最后 Roll(X) */
    float cr, sr, cp, sp, cy, sy;
    float half_roll  = roll  * IMU_DEG2RAD * 0.5f;
    float half_pitch = pitch * IMU_DEG2RAD * 0.5f;
    float half_yaw   = yaw   * IMU_DEG2RAD * 0.5f;

    cr = imu_safe_cos(half_roll);
    sr = imu_safe_sin(half_roll);
    cp = imu_safe_cos(half_pitch);
    sp = imu_safe_sin(half_pitch);
    cy = imu_safe_cos(half_yaw);
    sy = imu_safe_sin(half_yaw);

    q->q0 = cr * cp * cy + sr * sp * sy;
    q->q1 = sr * cp * cy - cr * sp * sy;
    q->q2 = cr * sp * cy + sr * cp * sy;
    q->q3 = cr * cp * sy - sr * sp * cy;
}

void imu_quat_to_euler(const imu_quat_t *q, imu_euler_t *euler)
{
    /* ZYX 旋转顺序 */
    float q0q0 = q->q0 * q->q0;
    float q1q1 = q->q1 * q->q1;
    float q2q2 = q->q2 * q->q2;
    float q3q3 = q->q3 * q->q3;

    /* Roll (X轴): atan2(2(q0q1 + q2q3), 1 - 2(q1² + q2²)) */
    euler->roll = imu_safe_atan2(2.0f * (q->q0 * q->q1 + q->q2 * q->q3),
                                  q0q0 - q1q1 - q2q2 + q3q3) * IMU_RAD2DEG;

    /* Pitch (Y轴): asin(2(q0q2 - q3q1))，限幅到 [-90, 90] */
    float sinp = 2.0f * (q->q0 * q->q2 - q->q3 * q->q1);
    euler->pitch = imu_safe_asin(sinp) * IMU_RAD2DEG;

    /* Yaw (Z轴): atan2(2(q0q3 + q1q2), 1 - 2(q2² + q3²)) */
    euler->yaw = imu_safe_atan2(2.0f * (q->q0 * q->q3 + q->q1 * q->q2),
                                 q0q0 + q1q1 - q2q2 - q3q3) * IMU_RAD2DEG;
}

void imu_quat_multiply(const imu_quat_t *a, const imu_quat_t *b, imu_quat_t *result)
{
    result->q0 = a->q0 * b->q0 - a->q1 * b->q1 - a->q2 * b->q2 - a->q3 * b->q3;
    result->q1 = a->q0 * b->q1 + a->q1 * b->q0 + a->q2 * b->q3 - a->q3 * b->q2;
    result->q2 = a->q0 * b->q2 - a->q1 * b->q3 + a->q2 * b->q0 + a->q3 * b->q1;
    result->q3 = a->q0 * b->q3 + a->q1 * b->q2 - a->q2 * b->q1 + a->q3 * b->q0;
}

void imu_quat_normalize(imu_quat_t *q)
{
    float norm = imu_inv_sqrt(q->q0 * q->q0 + q->q1 * q->q1 +
                              q->q2 * q->q2 + q->q3 * q->q3);
    q->q0 *= norm;
    q->q1 *= norm;
    q->q2 *= norm;
    q->q3 *= norm;
}
