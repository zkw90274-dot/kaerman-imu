/**
 * @file    kalman.h
 * @brief   卡尔曼滤波器 - 融合陀螺仪和加速度计数据
 * @author  Claude
 * @date    2026-07-21
 * @version 1.0.0
 *
 * @details
 * 基于 Arduino 卡尔曼滤波程序移植
 * 使用一维卡尔曼滤波分别处理 Roll 和 Pitch
 *
 * 算法步骤：
 *   1. 先验状态预测：gyro_angle = last_angle + gyro_rate * dt
 *   2. 先验协方差：P = P + Q
 *   3. 卡尔曼增益：K = P / (P + R)
 *   4. 最优估计：angle = gyro_angle + K * (acc_angle - gyro_angle)
 *   5. 更新协方差：P = (1 - K) * P
 */

#ifndef __KALMAN_H__
#define __KALMAN_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ==================== 数据结构体 ==================== */

/**
 * @brief 卡尔曼滤波器状态
 */
typedef struct {
    float angle;            // 最优估计角度
    float rate;             // 角速度（未补偿）
    float P[2][2];          // 误差协方差矩阵
    float Q_angle;          // 过程噪声（角度）
    float Q_bias;           // 过程噪声（偏差）
    float R_measure;        // 测量噪声
    float bias;             // 陀螺仪偏差
    float dt;               // 采样时间间隔
} kalman_filter_t;

/**
 * @brief 卡尔曼滤波器输出
 */
typedef struct {
    float roll;             // 横滚角（度）
    float pitch;            // 俯仰角（度）
    float yaw;              // 偏航角（度，陀螺仪积分）
    float roll_rate;        // 横滚角速度（度/秒）
    float pitch_rate;       // 俯仰角速度（度/秒）
    float vibration_weight; // 振动权重 (1.0=无振动, >1.0=有振动)
    float motion_weight;    // 运动权重 (1.0=静止, >1.0=运动)
    float adaptive_R;       // 当前自适应 R 值
    float adaptive_Q;       // 当前自适应 Q 值
} kalman_output_t;

/* ==================== API 函数 ==================== */

/**
 * @brief   初始化卡尔曼滤波器
 */
void kalman_init(void);

/**
 * @brief   更新卡尔曼滤波器
 * @param   gx      X轴角速度 (dps)
 * @param   gy      Y轴角速度 (dps)
 * @param   gz      Z轴角速度 (dps)
 * @param   ax      X轴加速度 (g)
 * @param   ay      Y轴加速度 (g)
 * @param   az      Z轴加速度 (g)
 * @param   dt      采样时间间隔 (秒)
 * @param   output  输出角度
 */
void kalman_update(float gx, float gy, float gz,
                   float ax, float ay, float az,
                   float dt, kalman_output_t *output);

/**
 * @brief   获取当前输出
 */
void kalman_get_output(kalman_output_t *output);

/**
 * @brief   重置滤波器
 */
void kalman_reset(void);

/**
 * @brief   设置参数
 * @param   Q_angle     过程噪声（角度），默认 0.001
 * @param   Q_bias      过程噪声（偏差），默认 0.003
 * @param   R_measure   测量噪声，默认 0.03
 */
void kalman_set_params(float Q_angle, float Q_bias, float R_measure);

#ifdef __cplusplus
}
#endif

#endif /* __KALMAN_H__ */
