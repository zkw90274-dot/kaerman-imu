/*
 * ICM42688 IMU + 姿态解算测试（支持互补滤波/卡尔曼滤波）
 *
 * 滤波器选择：
 *   - 使用 menuconfig 或修改 FILTER_TYPE 宏
 *   - 0: 互补滤波 (AHRS)
 *   - 1: 卡尔曼滤波 (Kalman)
 *
 * 测试模式：
 *   - 0: 正常输出
 *   - 1: 阶跃响应测试（快速翻转传感器）
 *   - 2: 振动测试（敲击传感器）
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
#include "imu_math.h"

/* ==================== 配置 ==================== */
#define FILTER_TYPE     1       // 0=互补滤波, 1=卡尔曼滤波
#define TEST_MODE       0       // 0=正常, 1=阶跃响应, 2=振动测试

#if FILTER_TYPE == 0
    #include "ahrs.h"
    #define FILTER_NAME "AHRS (互补滤波)"
#else
    #include "kalman.h"
    #define FILTER_NAME "Kalman (卡尔曼滤波)"
#endif

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
    uint32_t tick = 0;

    // 等待系统稳定
    vTaskDelay(pdMS_TO_TICKS(500));

    // 初始化 ICM42688（使用默认配置）
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

    // 初始化滤波器
#if FILTER_TYPE == 0
    ahrs_init();
    ahrs_set_null_drift(s_gyro_offset_x, s_gyro_offset_y, s_gyro_offset_z);
#else
    kalman_init();
#endif

    ESP_LOGI(TAG, "=== 姿态解算启动 ===");
    ESP_LOGI(TAG, "滤波器: %s", FILTER_NAME);
    ESP_LOGI(TAG, "采样率: %d Hz", SAMPLE_RATE);
    ESP_LOGI(TAG, "测试模式: %d", TEST_MODE);
    ESP_LOGI(TAG, "----------------------------------------------------------------");

#if TEST_MODE == 1
    ESP_LOGI(TAG, "【阶跃响应测试】快速翻转传感器，观察响应速度");
#elif TEST_MODE == 2
    ESP_LOGI(TAG, "【振动测试】敲击传感器，观察抗振能力");
#endif

#if FILTER_TYPE == 0
    ESP_LOGI(TAG, "  Tick  Roll(X)   Pitch(Y)   Yaw(Z)   Vibration  Gyro(dps)");
#else
    ESP_LOGI(TAG, "  Tick  Roll(X)   Pitch(Y)   Yaw(Z)   Gyro(dps)");
#endif
    ESP_LOGI(TAG, "----------------------------------------------------------------");

    // 主循环
    while (1) {
        // 读取传感器数据
        icm42688_get_all_data(&data);

        // 零漂补偿
        float gx = data.gx - s_gyro_offset_x;
        float gy = data.gy - s_gyro_offset_y;
        float gz = data.gz - s_gyro_offset_z;

#if FILTER_TYPE == 0
        /* 互补滤波模式 */
        ahrs_update();
        ahrs_state_t state;
        ahrs_get_state(&state);

        // VOFA 格式输出: Roll,Pitch,Yaw\n
        printf("%.2f,%.2f,%.2f\n",
               (double)state.euler.roll,
               (double)state.euler.pitch,
               (double)state.euler.yaw);
#else
        /* 卡尔曼滤波模式 */
        kalman_output_t kalman_out;
        kalman_update(gx, gy, gz, data.ax, data.ay, data.az, SAMPLE_DT_S, &kalman_out);

        // VOFA 格式输出: Roll,Pitch,Yaw\n
        printf("%.2f,%.2f,%.2f\n",
               (double)kalman_out.roll,
               (double)kalman_out.pitch,
               (double)kalman_out.yaw);
#endif

        tick++;

        // 精确采样率控制
        vTaskDelay(pdMS_TO_TICKS(SAMPLE_DT_MS));
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "ESP32 ICM42688 IMU Attitude Estimation");
    ESP_LOGI(TAG, "Filter: %s, Test Mode: %d", FILTER_NAME, TEST_MODE);

    // 打印芯片信息
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);
    ESP_LOGI(TAG, "Chip: %s, cores: %d, revision: %d",
             CONFIG_IDF_TARGET, chip_info.cores, chip_info.revision);

    // 创建任务
    xTaskCreate(imu_task, "imu_task", 8192, NULL, 5, NULL);
}
