/**
 * @file    ahrs.h
 * @brief   姿态航向参考系统（AHRS）- 四元数姿态解算
 * @author  Claude
 * @date    2026-07-21
 * @version 2.0.0
 *
 * @details
 * 基于 719 飞行器实验室的姿态解算算法
 * 使用互补滤波 + 四元数进行姿态解算
 *
 * 优化特性：
 *   1. 陀螺仪零漂自动标定
 *   2. 误差积分限幅（防止积分饱和）
 *   3. 振动自适应 Kp（动态调整加速度计权重）
 *   4. Yaw 死区（抑制小角度漂移）
 *   5. 自定义数学函数（不依赖 math.h）
 *   6. 精确采样率控制
 */

#ifndef __AHRS_H__
#define __AHRS_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ==================== 采样参数 ==================== */

#define AHRS_SAMPLE_RATE     200                        /* IMU 采样频率 (Hz) */
#define AHRS_DT              (1.0f / AHRS_SAMPLE_RATE)  /* 采样时间间隔 (s) */

/* ==================== 数据结构体 ==================== */

/**
 * @brief 三轴浮点数据
 */
typedef struct {
    float x;
    float y;
    float z;
} ahrs_vec3_t;

/**
 * @brief 欧拉角（姿态角）
 */
typedef struct {
    float roll;             // 横滚角（度）
    float pitch;            // 俯仰角（度）
    float yaw;              // 偏航角（度）
} ahrs_euler_t;

/**
 * @brief 四元数
 */
typedef struct {
    float q0;
    float q1;
    float q2;
    float q3;
} ahrs_quat_t;

/**
 * @brief AHRS 完整状态数据
 */
typedef struct {
    ahrs_quat_t quat;           // 四元数
    ahrs_euler_t euler;         // 欧拉角
    float gyro_x;               // X轴角速度 (°/s，零漂补偿后)
    float gyro_y;               // Y轴角速度 (°/s，零漂补偿后)
    float gyro_z;               // Z轴角速度 (°/s，零漂补偿后)
    float null_drift_x;         // X轴零漂 (°/s)
    float null_drift_y;         // Y轴零漂 (°/s)
    float null_drift_z;         // Z轴零漂 (°/s)
    float vibration_weight;     // 振动自适应权重 (1.0=无振动, 接近0=强振动)
    uint8_t calibrated;         // 标定完成标志
} ahrs_state_t;

/* ==================== API 函数 ==================== */

/**
 * @brief   初始化 AHRS
 */
void ahrs_init(void);

/**
 * @brief   更新 AHRS 姿态解算（100Hz 周期调用）
 * @details 完整流程:
 *   ① 读取传感器 → ② 单位转换 → ③ 振动检测 → ④ 加速度归一化
 *   → ⑤ 提取重力分量 → ⑥ 叉乘求误差 → ⑦ PI 积分 (自适应 Kp)
 *   → ⑧ 修正陀螺仪 → ⑨ 更新四元数 → ⑩ 归一化 → ⑪ 欧拉角反解
 */
void ahrs_update(void);

/**
 * @brief   陀螺仪零漂标定（静止状态调用，需 500 个采样）
 * @return  0=标定中, 1=标定完成
 */
int8_t ahrs_calibrate_null_drift(void);

/**
 * @brief   姿态角清零（重置四元数 + Yaw 积分）
 */
void ahrs_angle_clear(void);

/**
 * @brief   AHRS 完整初始化流程（初始化 + 标定）
 */
void ahrs_prepare(void);

/**
 * @brief   获取当前姿态角
 */
void ahrs_get_euler(ahrs_euler_t *euler);

/**
 * @brief   获取当前四元数
 */
void ahrs_get_quaternion(ahrs_quat_t *quat);

/**
 * @brief   获取完整状态
 */
void ahrs_get_state(ahrs_state_t *state);

/**
 * @brief   设置零漂值（手动校准）
 */
void ahrs_set_null_drift(float drift_x, float drift_y, float drift_z);

#ifdef __cplusplus
}
#endif

#endif /* __AHRS_H__ */
