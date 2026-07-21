/*
 * ICM42688 IMU + 自适应卡尔曼滤波器
 *
 * 数据流：
 *   原始数据 → 零漂补偿 → 自适应卡尔曼融合 → 欧拉角
 *
 * 特性：
 *   - 自适应测量噪声 R（振动检测）
 *   - 自适应过程噪声 Q（运动检测）
 *   - Yaw 死区抑制漂移
 *
 * 输出格式：VOFA FireWater
 *   Roll,Pitch,Yaw\n
 */

#include <stdio.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_chip_info.h"
#include "esp_system.h"
#include "esp_log.h"
#include "icm42688.h"
#include "kalman.h"

static const char *TAG = "main";

/* ==================== 采样参数 ==================== */
#define SAMPLE_RATE     200
#define SAMPLE_DT_MS    (1000 / SAMPLE_RATE)
#define SAMPLE_DT_S     (1.0f / SAMPLE_RATE)

/* ==================== 零漂标定 ==================== */
#define CALIBRATE_COUNT 500

static float s_gyro_offset_x = 0, s_gyro_offset_y = 0, s_gyro_offset_z = 0;

/**
 * @brief   陀螺仪零漂标定
 */
static void calibrate_gyro(void)
{
    icm42688_sensor_data_t data;
    float sum_x = 0, sum_y = 0, sum_z = 0;

    ESP_LOGI(TAG, "开始零漂标定，保持传感器静止...");

    for (int i = 0; i < CALIBRATE_COUNT; i++) {
        icm42688_get_all_data(&data);
        sum_x += data.gx;
        sum_y += data.gy;
        sum_z += data.gz;
        vTaskDelay(pdMS_TO_TICKS(5));
    }

    s_gyro_offset_x = sum_x / CALIBRATE_COUNT;
    s_gyro_offset_y = sum_y / CALIBRATE_COUNT;
    s_gyro_offset_z = sum_z / CALIBRATE_COUNT;

    ESP_LOGI(TAG, "零漂标定完成: X=%.4f, Y=%.4f, Z=%.4f °/s",
             (double)s_gyro_offset_x, (double)s_gyro_offset_y, (double)s_gyro_offset_z);
}

/**
 * @brief   IMU + 姿态解算任务
 */
static void imu_task(void *arg)
{
    icm42688_sensor_data_t data;
    kalman_output_t kalman_out;

    // 等待系统稳定
    vTaskDelay(pdMS_TO_TICKS(500));

    // 初始化 ICM42688
    ESP_LOGI(TAG, "Initializing ICM42688...");
    if (icm42688_init(NULL) != 0) {
        ESP_LOGE(TAG, "ICM42688 init failed!");
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "ICM42688 init OK, WHO_AM_I = 0x%02X", icm42688_read_id());
    vTaskDelay(pdMS_TO_TICKS(50));

    // 零漂标定
    calibrate_gyro();

    // 初始化自适应卡尔曼滤波器
    kalman_init();

    ESP_LOGI(TAG, "=== 自适应卡尔曼滤波器启动 ===");
    ESP_LOGI(TAG, "特性: 自适应R(振动) + 自适应Q(运动) + Yaw死区");
    ESP_LOGI(TAG, "采样率: %d Hz", SAMPLE_RATE);
    ESP_LOGI(TAG, "输出格式: Roll,Pitch,Yaw (VOFA FireWater)");

    // 主循环
    while (1) {
        // 1. 读取原始数据
        icm42688_get_all_data(&data);

        // 2. 零漂补偿
        float gx = data.gx - s_gyro_offset_x;
        float gy = data.gy - s_gyro_offset_y;
        float gz = data.gz - s_gyro_offset_z;

        // 3. 自适应卡尔曼滤波
        kalman_update(gx, gy, gz, data.ax, data.ay, data.az, SAMPLE_DT_S, &kalman_out);

        // 4. VOFA 格式输出
        printf("%.2f,%.2f,%.2f\n", kalman_out.roll, kalman_out.pitch, kalman_out.yaw);

        // 精确采样率控制
        vTaskDelay(pdMS_TO_TICKS(SAMPLE_DT_MS));
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "ESP32 ICM42688 IMU - 自适应卡尔曼滤波器");

    // 打印芯片信息
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);
    ESP_LOGI(TAG, "Chip: %s, cores: %d, revision: %d",
             CONFIG_IDF_TARGET, chip_info.cores, chip_info.revision);

    // 创建任务
    xTaskCreate(imu_task, "imu_task", 8192, NULL, 5, NULL);
}
