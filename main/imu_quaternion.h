/**
 * @file    imu_quaternion.h
 * @brief   四元数姿态解算（标准流程）
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
 * 优点：
 *   - 避免万向节锁
 *   - 计算效率高
 *   - 平滑无奇点
 */

#ifndef __IMU_QUATERNION_H__
#define __IMU_QUATERNION_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ==================== 常量定义 ==================== */

#define IMU_PI          3.14159265358979323846f
#define IMU_RAD2DEG     57.29577951308232f      /* 180/π */
#define IMU_DEG2RAD     0.017453292519943295f   /* π/180 */

/* ==================== 数据结构体 ==================== */

/**
 * @brief 三轴浮点数据
 */
typedef struct {
    float x;
    float y;
    float z;
} imu_vec3_t;

/**
 * @brief 欧拉角
 */
typedef struct {
    float roll;             // 横滚角（度）
    float pitch;            // 俯仰角（度）
    float yaw;              // 偏航角（度）
} imu_euler_t;

/**
 * @brief 四元数
 */
typedef struct {
    float q0;               // w
    float q1;               // x
    float q2;               // y
    float q3;               // z
} imu_quat_t;

/**
 * @brief IMU 完整状态
 */
typedef struct {
    imu_quat_t quat;        // 四元数
    imu_euler_t euler;      // 欧拉角
    imu_vec3_t gyro;        // 陀螺仪（rad/s，已滤波）
    imu_vec3_t accel;       // 加速度计（g，已滤波）
    float vibration;        // 振动权重
} imu_state_t;

/* ==================== API 函数 ==================== */

/**
 * @brief   初始化四元数姿态解算
 */
void imu_quat_init(void);

/**
 * @brief   更新姿态（标准流程）
 * @param   ax, ay, az  加速度计原始数据 (g)
 * @param   gx, gy, gz  陀螺仪原始数据 (dps)
 * @param   dt          采样时间 (秒)
 */
void imu_quat_update(float ax, float ay, float az,
                     float gx, float gy, float gz,
                     float dt);

/**
 * @brief   获取当前欧拉角
 */
void imu_quat_get_euler(imu_euler_t *euler);

/**
 * @brief   获取当前四元数
 */
void imu_quat_get_quaternion(imu_quat_t *quat);

/**
 * @brief   获取完整状态
 */
void imu_quat_get_state(imu_state_t *state);

/**
 * @brief   重置姿态
 */
void imu_quat_reset(void);

/**
 * @brief   设置零漂
 */
void imu_quat_set_drift(float dx, float dy, float dz);

/**
 * @brief   设置滤波参数
 * @param   alpha   低通滤波系数 (0~1, 越大越信任新数据)
 */
void imu_quat_set_filter(float alpha);

#ifdef __cplusplus
}
#endif

#endif /* __IMU_QUATERNION_H__ */
