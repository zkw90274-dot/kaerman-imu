/*
 * ICM42688 IMU + AHRS 姿态解算测试（硬件 SPI + 全功能优化）
 *
 * 优化特性：
 *   1. 硬件 SPI 驱动（10MHz）
 *   2. 陀螺仪零漂自动标定
 *   3. 误差积分限幅
 *   4. 振动自适应 Kp
 *   5. Yaw 死区
 *   6. 自定义数学函数（不依赖 math.h）
 *   7. 精确采样率控制（200Hz）
 */

#include <stdio.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_chip_info.h"
#include "esp_system.h"
#include "esp_log.h"
#include "ahrs.h"

static const char *TAG = "main";

/**
 * @brief IMU + AHRS 数据读取任务
 */
static void imu_ahrs_task(void *arg)
{
    ahrs_state_t state;

    // 等待系统稳定
    vTaskDelay(pdMS_TO_TICKS(500));

    // 完整初始化（含零漂标定）
    ahrs_prepare();

    ESP_LOGI(TAG, "Starting attitude estimation @ %d Hz...", AHRS_SAMPLE_RATE);
    ESP_LOGI(TAG, "----------------------------------------------------------------");
    ESP_LOGI(TAG, "  Roll(X)   Pitch(Y)   Yaw(Z)   Vibration  Gyro(dps)");
    ESP_LOGI(TAG, "----------------------------------------------------------------");

    // 循环读取数据并解算姿态
    while (1) {
        // 更新 AHRS 姿态解算
        ahrs_update();

        // 获取完整状态
        ahrs_get_state(&state);

        // 输出姿态角和调试信息
        ESP_LOGI(TAG, "  %+7.2f  %+7.2f  %+7.2f    %.2f    %.1f %.1f %.1f",
                 state.euler.roll, state.euler.pitch, state.euler.yaw,
                 (double)state.vibration_weight,
                 (double)state.gyro_x, (double)state.gyro_y, (double)state.gyro_z);

        // 精确采样率控制
        vTaskDelay(pdMS_TO_TICKS(1000 / AHRS_SAMPLE_RATE));
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "ESP32 ICM42688 + AHRS Attitude Estimation (Optimized)");
    ESP_LOGI(TAG, "Features: HW SPI, Null Drift Cal, Integral Limit, Vibration Adaptive, Yaw Deadzone");

    // 打印芯片信息
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);
    ESP_LOGI(TAG, "Chip: %s, cores: %d, revision: %d",
             CONFIG_IDF_TARGET, chip_info.cores, chip_info.revision);

    // 创建 IMU + AHRS 任务
    xTaskCreate(imu_ahrs_task, "imu_ahrs_task", 8192, NULL, 5, NULL);
}
