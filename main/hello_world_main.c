/**
 * @file    hello_world_main.c
 * @brief   ICM42688 IMU 双滤波器姿态解算
 *
 * 数据流:
 *   Pipeline 1: 原始数据 → 卡尔曼滤波 → 四元数 → 欧拉角
 *   Pipeline 2: 原始数据 → Mahony 滤波 → 四元数 → 欧拉角
 *
 * 公共特性: 零漂标定、振动自适应、Yaw 死区
 */

#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_chip_info.h"
#include "esp_log.h"
#include "icm42688.h"
#include "kalman.h"
#include "ahrs.h"

static const char *TAG = "main";

/* ==================== 采样参数 ==================== */

#define SAMPLE_RATE     200
#define SAMPLE_DT_MS    (1000 / SAMPLE_RATE)
#define SAMPLE_DT_S     (1.0f / SAMPLE_RATE)

/* ==================== 零漂标定 ==================== */

#define CALIBRATE_COUNT 500

static float s_gyro_offset_x = 0, s_gyro_offset_y = 0, s_gyro_offset_z = 0;

/**
 * @brief   陀螺仪零漂标定（500 样本平均）
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
 * @brief   IMU + 双滤波器姿态解算任务
 */
static void imu_task(void *arg)
{
    icm42688_sensor_data_t data;
    kalman_output_t kalman_out;
    mahony_output_t mahony_out;

    /* 等待系统稳定 */
    vTaskDelay(pdMS_TO_TICKS(500));

    /* 初始化 ICM42688 */
    ESP_LOGI(TAG, "Initializing ICM42688...");
    if (icm42688_init(NULL) != 0) {
        ESP_LOGE(TAG, "ICM42688 init failed!");
        vTaskDelete(NULL);
        return;
    }
    ESP_LOGI(TAG, "ICM42688 init OK, WHO_AM_I = 0x%02X", icm42688_read_id());
    vTaskDelay(pdMS_TO_TICKS(50));

    /* 零漂标定 */
    calibrate_gyro();

    /* 初始化两个滤波器 */
    kalman_init();

    /* 读取当前加速度计数据，用于 Mahony 四元数初始化 */
    icm42688_sensor_data_t init_data;
    icm42688_get_all_data(&init_data);
    mahony_init(init_data.ax, init_data.ay, init_data.az);

    ESP_LOGI(TAG, "=== 双滤波器启动 ===");
    ESP_LOGI(TAG, "Pipeline 1: 卡尔曼滤波 → 四元数 → 欧拉角");
    ESP_LOGI(TAG, "Pipeline 2: Mahony 滤波 → 四元数 → 欧拉角");
    ESP_LOGI(TAG, "公共特性: 自适应R/Kp(振动) + 自适应Q(运动) + Yaw死区");
    ESP_LOGI(TAG, "采样率: %d Hz", SAMPLE_RATE);

    /* 主循环 */
    while (1) {
        /* 1. 读取原始数据（一次读取，两个滤波器共用） */
        icm42688_get_all_data(&data);

        /* 2. 零漂补偿 */
        float gx = data.gx - s_gyro_offset_x;
        float gy = data.gy - s_gyro_offset_y;
        float gz = data.gz - s_gyro_offset_z;

        /* 3. 卡尔曼滤波 */
        kalman_update(gx, gy, gz, data.ax, data.ay, data.az, SAMPLE_DT_S, &kalman_out);

        /* 4. Mahony 滤波 */
        mahony_update(gx, gy, gz, data.ax, data.ay, data.az, SAMPLE_DT_S, &mahony_out);

        /* 5. VOFA 输出 */
        printf("K:%.2f,%.2f,%.2f,M:%.2f,%.2f,%.2f\n",
               (double)kalman_out.euler.roll, (double)kalman_out.euler.pitch, (double)kalman_out.euler.yaw,
               (double)mahony_out.euler.roll, (double)mahony_out.euler.pitch, (double)mahony_out.euler.yaw);

        /* 精确采样率控制 */
        vTaskDelay(pdMS_TO_TICKS(SAMPLE_DT_MS));
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "ESP32 ICM42688 IMU - 双滤波器姿态解算");

    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);
    ESP_LOGI(TAG, "Chip: %s, cores: %d, revision: %d",
             CONFIG_IDF_TARGET, chip_info.cores, chip_info.revision);

    xTaskCreate(imu_task, "imu_task", 8192, NULL, 5, NULL);
}
