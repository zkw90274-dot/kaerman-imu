/**
 * @file    icm42688.c
 * @brief   ICM42688 六轴 IMU 驱动实现（硬件 SPI）
 * @author  Claude
 * @date    2026-07-21
 * @version 2.0.0
 *
 * @details
 * 使用 ESP-IDF 硬件 SPI 驱动
 * SPI 读操作：发送寄存器地址 | 0x80，然后读取数据
 * SPI 写操作：发送寄存器地址 & 0x7F，然后发送数据
 */

#include "icm42688.h"
#include "hw_spi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

static const char *TAG = "icm42688";

/* ==================== 私有变量 ==================== */

static hw_spi_t s_spi;                     // 硬件 SPI 句柄
static float s_acc_sensitivity  = 0.244f;  // 加速度计灵敏度 (mg/LSB)
static float s_gyro_sensitivity = 32.8f;   // 陀螺仪灵敏度 (dps/LSB)

/* ==================== 公共 API 实现 ==================== */

/**
 * @brief   读取 WHO_AM_I 寄存器
 */
uint8_t icm42688_read_id(void)
{
    return hw_spi_read_reg(&s_spi, ICM42688_WHO_AM_I);
}

/**
 * @brief   初始化 ICM42688
 */
int8_t icm42688_init(void)
{
    uint8_t reg_val;
    uint8_t device_id;

    // 初始化硬件 SPI
    hw_spi_config_t spi_cfg = HW_SPI_DEFAULT_CONFIG;
    if (hw_spi_init(&s_spi, &spi_cfg) != 0) {
        ESP_LOGE(TAG, "SPI init failed");
        return -1;
    }

    // 等待传感器上电
    vTaskDelay(pdMS_TO_TICKS(50));

    // 软复位
    hw_spi_write_reg(&s_spi, ICM42688_DEVICE_CONFIG, 0x01);
    vTaskDelay(pdMS_TO_TICKS(100));

    // 读取设备 ID
    device_id = hw_spi_read_reg(&s_spi, ICM42688_WHO_AM_I);
    ESP_LOGI(TAG, "WHO_AM_I = 0x%02X", device_id);

    if (device_id != ICM42688_ID) {
        ESP_LOGE(TAG, "Device ID mismatch! Expected 0x%02X, got 0x%02X",
                 ICM42688_ID, device_id);
        return -1;
    }

    // 配置加速度计：±2g，200Hz
    s_acc_sensitivity = 2000.0f / 32768.0f;
    reg_val = hw_spi_read_reg(&s_spi, ICM42688_ACCEL_CONFIG0);
    reg_val &= ~((0x3 << 5) | 0x0F);
    reg_val |= (AFS_2G << 5);
    reg_val |= AODR_200Hz;
    hw_spi_write_reg(&s_spi, ICM42688_ACCEL_CONFIG0, reg_val);

    // 配置陀螺仪：±1000dps，200Hz
    s_gyro_sensitivity = 1000.0f / 32768.0f;
    reg_val = hw_spi_read_reg(&s_spi, ICM42688_GYRO_CONFIG0);
    reg_val &= ~((0x3 << 5) | 0x0F);
    reg_val |= (GFS_1000DPS << 5);
    reg_val |= GODR_200Hz;
    hw_spi_write_reg(&s_spi, ICM42688_GYRO_CONFIG0, reg_val);

    // 配置加速度计滤波器（Bank 2）
    hw_spi_write_reg(&s_spi, ICM42688_REG_BANK_SEL, 0x02);
    reg_val = hw_spi_read_reg(&s_spi, ICM42688_ACCEL_CONFIG_STATIC3);
    reg_val = (reg_val & ~0x70) | (AAVG_4X << 4);
    reg_val = (reg_val & ~0x0F) | AAF_1_16_ODR;
    hw_spi_write_reg(&s_spi, ICM42688_ACCEL_CONFIG_STATIC3, reg_val);

    // 配置陀螺仪滤波器（Bank 1）
    hw_spi_write_reg(&s_spi, ICM42688_REG_BANK_SEL, 0x01);
    reg_val = hw_spi_read_reg(&s_spi, ICM42688_GYRO_CONFIG_STATIC3);
    reg_val = (reg_val & ~0x70) | (GAVG_4X << 4);
    reg_val = (reg_val & ~0x0F) | GF_1_16_ODR;
    hw_spi_write_reg(&s_spi, ICM42688_GYRO_CONFIG_STATIC3, reg_val);

    // 切换回 Bank 0，启动传感器
    hw_spi_write_reg(&s_spi, ICM42688_REG_BANK_SEL, 0x00);
    reg_val = hw_spi_read_reg(&s_spi, ICM42688_PWR_MGMT0);
    reg_val &= ~(1 << 5);      // 清除 GYRO_STANDBY
    reg_val |= (3 << 2);       // 加速度计低噪声模式
    reg_val |= 3;              // 陀螺仪低噪声模式
    hw_spi_write_reg(&s_spi, ICM42688_PWR_MGMT0, reg_val);
    vTaskDelay(pdMS_TO_TICKS(1));

    ESP_LOGI(TAG, "ICM42688 initialized (HW SPI)");
    return 0;
}

/**
 * @brief   获取温度
 */
int8_t icm42688_get_temperature(float *temperature)
{
    uint8_t buffer[2];
    int16_t raw_temp;

    hw_spi_read_regs(&s_spi, ICM42688_TEMP_DATA1, buffer, 2);
    raw_temp = (int16_t)((buffer[0] << 8) | buffer[1]);
    *temperature = (float)raw_temp / 132.48f + 25.0f;

    return 0;
}

/**
 * @brief   获取加速度计数据 (g)
 */
int8_t icm42688_get_accelerometer(icm42688_raw_data_t *raw,
                                   float *ax, float *ay, float *az)
{
    uint8_t buffer[6];
    int16_t raw_x, raw_y, raw_z;

    hw_spi_read_regs(&s_spi, ICM42688_ACCEL_DATA_X1, buffer, 6);

    raw_x = (int16_t)((buffer[0] << 8) | buffer[1]);
    raw_y = (int16_t)((buffer[2] << 8) | buffer[3]);
    raw_z = (int16_t)((buffer[4] << 8) | buffer[5]);

    if (raw != NULL) {
        raw->x = raw_x;
        raw->y = raw_y;
        raw->z = raw_z;
    }

    if (ax != NULL) *ax = (float)raw_x * s_acc_sensitivity / 1000.0f;
    if (ay != NULL) *ay = (float)raw_y * s_acc_sensitivity / 1000.0f;
    if (az != NULL) *az = (float)raw_z * s_acc_sensitivity / 1000.0f;

    return 0;
}

/**
 * @brief   获取陀螺仪数据 (dps)
 */
int8_t icm42688_get_gyroscope(icm42688_raw_data_t *raw,
                               float *gx, float *gy, float *gz)
{
    uint8_t buffer[6];
    int16_t raw_x, raw_y, raw_z;

    hw_spi_read_regs(&s_spi, ICM42688_GYRO_DATA_X1, buffer, 6);

    raw_x = (int16_t)((buffer[0] << 8) | buffer[1]);
    raw_y = (int16_t)((buffer[2] << 8) | buffer[3]);
    raw_z = (int16_t)((buffer[4] << 8) | buffer[5]);

    if (raw != NULL) {
        raw->x = raw_x;
        raw->y = raw_y;
        raw->z = raw_z;
    }

    if (gx != NULL) *gx = (float)raw_x * s_gyro_sensitivity;
    if (gy != NULL) *gy = (float)raw_y * s_gyro_sensitivity;
    if (gz != NULL) *gz = (float)raw_z * s_gyro_sensitivity;

    return 0;
}

/**
 * @brief   一次性读取所有传感器数据
 */
int8_t icm42688_get_all_data(icm42688_sensor_data_t *data)
{
    uint8_t buffer[14];
    int16_t raw_temp, raw_ax, raw_ay, raw_az, raw_gx, raw_gy, raw_gz;

    if (data == NULL) {
        return -1;
    }

    // 从 TEMP_DATA1 开始连续读取 14 字节
    hw_spi_read_regs(&s_spi, ICM42688_TEMP_DATA1, buffer, 14);

    raw_temp = (int16_t)((buffer[0] << 8) | buffer[1]);
    raw_ax   = (int16_t)((buffer[2] << 8) | buffer[3]);
    raw_ay   = (int16_t)((buffer[4] << 8) | buffer[5]);
    raw_az   = (int16_t)((buffer[6] << 8) | buffer[7]);
    raw_gx   = (int16_t)((buffer[8] << 8) | buffer[9]);
    raw_gy   = (int16_t)((buffer[10] << 8) | buffer[11]);
    raw_gz   = (int16_t)((buffer[12] << 8) | buffer[13]);

    data->temperature = (float)raw_temp / 132.48f + 25.0f;
    data->ax = (float)raw_ax * s_acc_sensitivity / 1000.0f;
    data->ay = (float)raw_ay * s_acc_sensitivity / 1000.0f;
    data->az = (float)raw_az * s_acc_sensitivity / 1000.0f;
    data->gx = (float)raw_gx * s_gyro_sensitivity;
    data->gy = (float)raw_gy * s_gyro_sensitivity;
    data->gz = (float)raw_gz * s_gyro_sensitivity;

    return 0;
}
