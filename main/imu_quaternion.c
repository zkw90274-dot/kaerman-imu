/**
 * @file    imu_quaternion.c
 * @brief   四元数姿态解算实现（标准流程）
 * @author  Claude
 * @date    2026-07-21
 * @version 1.0.0
 *
 * @details
 * 标准 IMU 数据处理流程：
 *   1. 原始数据 → 预处理（低通滤波 + 零漂补偿）
 *   2. 四元数更新（陀螺仪积分）
 *   3. 互补校正（加速度计修正漂移）
 *   4. 欧拉角反解（四元数 → Roll, Pitch, Yaw）
 *
 * 算法参考：Madgwick AHRS / Mahony AHRS
 */

#include "imu_quaternion.h"
#include <math.h>

/* ==================== 配置参数 ==================== */

/* 互补滤波参数 */
#define KP_GAIN         1.5f        // 比例增益（加速度计校正强度）
#define KI_GAIN         0.005f      // 积分增益（陀螺仪偏差收敛）

/* 低通滤波参数 */
#define DEFAULT_ALPHA   0.3f        // 默认滤波系数（0~1，越大越信任新数据）

/* 振动检测参数 */
#define VIBRATION_THRESH    0.08f   // 振动阈值 (g)
#define VIBRATION_SCALE     0.1f    // 振动时 Kp 缩放

/* Yaw 死区 */
#define YAW_DEADZONE    0.5f        // dps

/* ==================== 私有变量 ==================== */

static imu_state_t s_state;

/* PI 控制器积分 */
static float s_ex_int = 0.0f;
static float s_ey_int = 0.0f;
static float s_ez_int = 0.0f;

/* 零漂 */
static float s_drift_x = 0.0f;
static float s_drift_y = 0.0f;
static float s_drift_z = 0.0f;

/* 滤波系数 */
static float s_alpha = DEFAULT_ALPHA;

/* ==================== 快速平方根倒数 ==================== */

static float inv_sqrt(float x)
{
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long *)&y;
    i = 0x5f375a86 - (i >> 1);
    y = *(float *)&i;
    y = y * (1.5f - (halfx * y * y));
    return y;
}

/* ==================== 低通滤波 ==================== */

/**
 * @brief   一阶低通滤波
 * @param   old_val     上一次的值
 * @param   new_val     新的测量值
 * @param   alpha       滤波系数 (0~1)
 * @return  滤波后的值
 */
static float low_pass_filter(float old_val, float new_val, float alpha)
{
    return old_val + alpha * (new_val - old_val);
}

/* ==================== 公共 API 实现 ==================== */

/**
 * @brief   初始化四元数姿态解算
 */
void imu_quat_init(void)
{
    // 初始化四元数（无旋转）
    s_state.quat.q0 = 1.0f;
    s_state.quat.q1 = 0.0f;
    s_state.quat.q2 = 0.0f;
    s_state.quat.q3 = 0.0f;

    // 清零欧拉角
    s_state.euler.roll = 0.0f;
    s_state.euler.pitch = 0.0f;
    s_state.euler.yaw = 0.0f;

    // 清零滤波数据
    s_state.gyro.x = 0.0f;
    s_state.gyro.y = 0.0f;
    s_state.gyro.z = 0.0f;
    s_state.accel.x = 0.0f;
    s_state.accel.y = 0.0f;
    s_state.accel.z = 0.0f;

    // 清零积分
    s_ex_int = 0.0f;
    s_ey_int = 0.0f;
    s_ez_int = 0.0f;

    s_state.vibration = 1.0f;
}

/**
 * @brief   更新姿态（标准流程）
 *
 * 流程：
 *   1. 低通滤波 + 零漂补偿
 *   2. 四元数更新（陀螺仪积分）
 *   3. 互补校正（加速度计修正）
 *   4. 四元数归一化
 *   5. 欧拉角反解
 */
void imu_quat_update(float ax, float ay, float az,
                     float gx, float gy, float gz,
                     float dt)
{
    float norm;
    float vx, vy, vz;
    float ex, ey, ez;
    float q0, q1, q2, q3;

    /* ========== 步骤 1: 预处理 ========== */

    // 1.1 低通滤波加速度计
    s_state.accel.x = low_pass_filter(s_state.accel.x, ax, s_alpha);
    s_state.accel.y = low_pass_filter(s_state.accel.y, ay, s_alpha);
    s_state.accel.z = low_pass_filter(s_state.accel.z, az, s_alpha);

    // 1.2 陀螺仪零漂补偿 + 单位转换 (dps → rad/s)
    s_state.gyro.x = (gx - s_drift_x) * IMU_DEG2RAD;
    s_state.gyro.y = (gy - s_drift_y) * IMU_DEG2RAD;
    s_state.gyro.z = (gz - s_drift_z) * IMU_DEG2RAD;

    // 1.3 振动检测
    float acc_mag = inv_sqrt(ax * ax + ay * ay + az * az);
    float acc_dev = fabsf(1.0f - 1.0f / acc_mag);  // 偏离 1g 的程度
    float kp_adaptive = KP_GAIN;
    if (acc_dev > VIBRATION_THRESH) {
        kp_adaptive = KP_GAIN * VIBRATION_SCALE;  // 振动时降低 Kp
    }
    s_state.vibration = kp_adaptive / KP_GAIN;

    /* ========== 步骤 2: 四元数更新（陀螺仪积分） ========== */

    q0 = s_state.quat.q0;
    q1 = s_state.quat.q1;
    q2 = s_state.quat.q2;
    q3 = s_state.quat.q3;

    float gx_r = s_state.gyro.x;
    float gy_r = s_state.gyro.y;
    float gz_r = s_state.gyro.z;

    // 四元数微分方程
    float half_dt = 0.5f * dt;
    q0 += (-q1 * gx_r - q2 * gy_r - q3 * gz_r) * half_dt;
    q1 += ( q0 * gx_r + q2 * gz_r - q3 * gy_r) * half_dt;
    q2 += ( q0 * gy_r - q1 * gz_r + q3 * gx_r) * half_dt;
    q3 += ( q0 * gz_r + q1 * gy_r - q2 * gx_r) * half_dt;

    /* ========== 步骤 3: 互补校正（加速度计修正） ========== */

    // 加速度计归一化
    float ax_n = s_state.accel.x;
    float ay_n = s_state.accel.y;
    float az_n = s_state.accel.z;
    norm = inv_sqrt(ax_n * ax_n + ay_n * ay_n + az_n * az_n);
    ax_n *= norm;
    ay_n *= norm;
    az_n *= norm;

    // 从四元数提取重力方向（方向余弦矩阵第三列）
    vx = 2.0f * (q1 * q3 - q0 * q2);
    vy = 2.0f * (q0 * q1 + q2 * q3);
    vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

    // 计算误差（叉乘）
    ex = (ay_n * vz - az_n * vy);
    ey = (az_n * vx - ax_n * vz);
    ez = (ax_n * vy - ay_n * vx);

    // PI 控制器
    s_ex_int += ex * KI_GAIN;
    s_ey_int += ey * KI_GAIN;
    s_ez_int += ez * KI_GAIN;

    // 积分限幅
    if (s_ex_int >  0.5f) s_ex_int =  0.5f;
    if (s_ex_int < -0.5f) s_ex_int = -0.5f;
    if (s_ey_int >  0.5f) s_ey_int =  0.5f;
    if (s_ey_int < -0.5f) s_ey_int = -0.5f;
    if (s_ez_int >  0.5f) s_ez_int =  0.5f;
    if (s_ez_int < -0.5f) s_ez_int = -0.5f;

    // 补偿陀螺仪
    gx_r += kp_adaptive * ex + s_ex_int;
    gy_r += kp_adaptive * ey + s_ey_int;
    gz_r += kp_adaptive * ez + s_ez_int;

    // 重新积分四元数
    q0 = s_state.quat.q0;
    q1 = s_state.quat.q1;
    q2 = s_state.quat.q2;
    q3 = s_state.quat.q3;

    q0 += (-q1 * gx_r - q2 * gy_r - q3 * gz_r) * half_dt;
    q1 += ( q0 * gx_r + q2 * gz_r - q3 * gy_r) * half_dt;
    q2 += ( q0 * gy_r - q1 * gz_r + q3 * gx_r) * half_dt;
    q3 += ( q0 * gz_r + q1 * gy_r - q2 * gx_r) * half_dt;

    /* ========== 步骤 4: 四元数归一化 ========== */

    norm = inv_sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= norm;
    q1 *= norm;
    q2 *= norm;
    q3 *= norm;

    // 保存四元数
    s_state.quat.q0 = q0;
    s_state.quat.q1 = q1;
    s_state.quat.q2 = q2;
    s_state.quat.q3 = q3;

    /* ========== 步骤 5: 欧拉角反解 ========== */

    float q0q0 = q0 * q0;
    float q1q3 = q1 * q3;
    float q0q2 = q0 * q2;
    float q2q3 = q2 * q3;
    float q0q1 = q0 * q1;

    // Roll (横滚角) - 绕 X 轴
    s_state.euler.roll = -asinf(2.0f * (q1q3 - q0q2)) * IMU_RAD2DEG;

    // Pitch (俯仰角) - 绕 Y 轴
    s_state.euler.pitch = -atan2f(2.0f * q2q3 + 2.0f * q0q1,
                                   q0q0 - q1 * q1 - q2 * q2 + q3 * q3) * IMU_RAD2DEG;

    // Yaw (偏航角) - 绕 Z 轴（陀螺仪积分 + 死区）
    float gz_dps = gz - s_drift_z;
    if (gz_dps > YAW_DEADZONE || gz_dps < -YAW_DEADZONE) {
        s_state.euler.yaw += gz_dps * dt;
    }
}

/**
 * @brief   获取当前欧拉角
 */
void imu_quat_get_euler(imu_euler_t *euler)
{
    if (euler != NULL) {
        *euler = s_state.euler;
    }
}

/**
 * @brief   获取当前四元数
 */
void imu_quat_get_quaternion(imu_quat_t *quat)
{
    if (quat != NULL) {
        *quat = s_state.quat;
    }
}

/**
 * @brief   获取完整状态
 */
void imu_quat_get_state(imu_state_t *state)
{
    if (state != NULL) {
        *state = s_state;
    }
}

/**
 * @brief   重置姿态
 */
void imu_quat_reset(void)
{
    imu_quat_init();
}

/**
 * @brief   设置零漂
 */
void imu_quat_set_drift(float dx, float dy, float dz)
{
    s_drift_x = dx;
    s_drift_y = dy;
    s_drift_z = dz;
}

/**
 * @brief   设置滤波参数
 */
void imu_quat_set_filter(float alpha)
{
    if (alpha < 0.0f) alpha = 0.0f;
    if (alpha > 1.0f) alpha = 1.0f;
    s_alpha = alpha;
}
