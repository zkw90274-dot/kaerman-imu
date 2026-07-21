/**
 * @file    kalman.c
 * @brief   卡尔曼滤波器实现
 * @author  Claude
 * @date    2026-07-21
 * @version 1.0.0
 *
 * @details
 * 基于 Arduino 卡尔曼滤波程序移植
 * 使用一维卡尔曼滤波分别处理 Roll 和 Pitch
 *
 * 核心算法：
 *   1. 先验状态预测：gyro_angle = last_angle + gyro_rate * dt
 *   2. 先验协方差：P = P + Q
 *   3. 卡尔曼增益：K = P / (P + R)
 *   4. 最优估计：angle = gyro_angle + K * (acc_angle - gyro_angle)
 *   5. 更新协方差：P = (1 - K) * P
 */

#include "kalman.h"
#include "imu_math.h"
#include "esp_log.h"
#include <string.h>

static const char *TAG = "kalman";

/* ==================== 默认参数 ==================== */

#define DEFAULT_Q_ANGLE     0.001f      // 过程噪声（角度）
#define DEFAULT_Q_BIAS      0.003f      // 过程噪声（偏差）
#define DEFAULT_R_MEASURE   0.03f       // 测量噪声

/* ==================== 模块状态 ==================== */

static kalman_filter_t s_roll;          // Roll 角卡尔曼滤波器
static kalman_filter_t s_pitch;         // Pitch 角卡尔曼滤波器
static kalman_output_t s_output;        // 输出缓存
static uint8_t s_inited = 0;            // 初始化标志

/* ==================== 内部函数 ==================== */

/**
 * @brief   初始化单个卡尔曼滤波器
 */
static void kalman_filter_init(kalman_filter_t *kf)
{
    kf->angle = 0.0f;
    kf->rate = 0.0f;
    kf->bias = 0.0f;

    // 初始化协方差矩阵
    kf->P[0][0] = 1.0f;
    kf->P[0][1] = 0.0f;
    kf->P[1][0] = 0.0f;
    kf->P[1][1] = 1.0f;

    // 设置默认参数
    kf->Q_angle = DEFAULT_Q_ANGLE;
    kf->Q_bias = DEFAULT_Q_BIAS;
    kf->R_measure = DEFAULT_R_MEASURE;
}

/**
 * @brief   更新单个卡尔曼滤波器
 * @param   kf          滤波器指针
 * @param   new_angle   加速度计测量的角度
 * @param   new_rate    陀螺仪角速度
 * @param   dt          采样时间间隔
 * @return  最优估计角度
 */
static float kalman_filter_update(kalman_filter_t *kf, float new_angle, float new_rate, float dt)
{
    float S, K[2];
    float y;
    float P00_temp, P01_temp;

    kf->dt = dt;

    /* Step 1: 先验状态预测 */
    // 用陀螺仪积分预测角度
    kf->rate = new_rate - kf->bias;
    kf->angle += dt * kf->rate;

    /* Step 2: 更新先验协方差矩阵 */
    kf->P[0][0] += dt * (dt * kf->P[1][1] - kf->P[0][1] - kf->P[1][0] + kf->Q_angle);
    kf->P[0][1] -= dt * kf->P[1][1];
    kf->P[1][0] -= dt * kf->P[1][1];
    kf->P[1][1] += kf->Q_bias * dt;

    /* Step 3: 计算卡尔曼增益 */
    S = kf->P[0][0] + kf->R_measure;   // 估计误差
    K[0] = kf->P[0][0] / S;
    K[1] = kf->P[1][0] / S;

    /* Step 4: 计算最优估计 */
    y = new_angle - kf->angle;          // 观测残差
    kf->angle += K[0] * y;
    kf->bias  += K[1] * y;

    /* Step 5: 更新协方差矩阵 */
    P00_temp = kf->P[0][0];
    P01_temp = kf->P[0][1];

    kf->P[0][0] -= K[0] * P00_temp;
    kf->P[0][1] -= K[0] * P01_temp;
    kf->P[1][0] -= K[1] * P00_temp;
    kf->P[1][1] -= K[1] * P01_temp;

    return kf->angle;
}

/* ==================== 公共 API 实现 ==================== */

/**
 * @brief   初始化卡尔曼滤波器
 */
void kalman_init(void)
{
    ESP_LOGI(TAG, "=== 初始化卡尔曼滤波器 ===");

    kalman_filter_init(&s_roll);
    kalman_filter_init(&s_pitch);

    memset(&s_output, 0, sizeof(s_output));
    s_inited = 1;

    ESP_LOGI(TAG, "卡尔曼滤波器初始化完成");
}

/**
 * @brief   更新卡尔曼滤波器
 */
void kalman_update(float gx, float gy, float gz,
                   float ax, float ay, float az,
                   float dt, kalman_output_t *output)
{
    if (!s_inited) {
        kalman_init();
    }

    float acc_roll, acc_pitch;

    /* 计算加速度计测量的角度 */
    // Roll: atan(ay / az)
    acc_roll = imu_safe_atan2(ay, az) * IMU_RAD2DEG;

    // Pitch: -atan(ax / sqrt(ay² + az²))
    acc_pitch = -imu_safe_atan2(ax, imu_inv_sqrt(ay * ay + az * az)) * IMU_RAD2DEG;

    /* 更新卡尔曼滤波器 */
    // 注意：这里用 gx, gy 作为角速度输入
    // Roll 角用 gx（绕 X 轴旋转）
    // Pitch 角用 gy（绕 Y 轴旋转）
    s_output.roll = kalman_filter_update(&s_roll, acc_roll, gx, dt);
    s_output.pitch = kalman_filter_update(&s_pitch, acc_pitch, gy, dt);

    // 保存角速度
    s_output.roll_rate = gx;
    s_output.pitch_rate = gy;

    // 输出结果
    if (output != NULL) {
        *output = s_output;
    }
}

/**
 * @brief   获取当前输出
 */
void kalman_get_output(kalman_output_t *output)
{
    if (output != NULL) {
        *output = s_output;
    }
}

/**
 * @brief   重置滤波器
 */
void kalman_reset(void)
{
    kalman_filter_init(&s_roll);
    kalman_filter_init(&s_pitch);
    memset(&s_output, 0, sizeof(s_output));
    ESP_LOGI(TAG, "卡尔曼滤波器已重置");
}

/**
 * @brief   设置参数
 */
void kalman_set_params(float Q_angle, float Q_bias, float R_measure)
{
    s_roll.Q_angle = Q_angle;
    s_roll.Q_bias = Q_bias;
    s_roll.R_measure = R_measure;

    s_pitch.Q_angle = Q_angle;
    s_pitch.Q_bias = Q_bias;
    s_pitch.R_measure = R_measure;

    ESP_LOGI(TAG, "参数设置: Q_angle=%.4f, Q_bias=%.4f, R_measure=%.4f",
             (double)Q_angle, (double)Q_bias, (double)R_measure);
}
