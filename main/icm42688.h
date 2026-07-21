/**
 * @file    icm42688.h
 * @brief   ICM42688 六轴 IMU 驱动（硬件 SPI 接口）
 * @author  Claude
 * @date    2026-07-21
 * @version 2.0.0
 *
 * @details
 * 使用 ESP-IDF 硬件 SPI 驱动，速度更快更稳定
 * 支持加速度计 + 陀螺仪，SPI 最高 24MHz
 */

#ifndef __ICM42688_H__
#define __ICM42688_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ==================== 寄存器地址定义 ==================== */

/* Bank 0 */
#define ICM42688_DEVICE_CONFIG             0x11
#define ICM42688_TEMP_DATA1                0x1D
#define ICM42688_TEMP_DATA2                0x1E
#define ICM42688_ACCEL_DATA_X1             0x1F
#define ICM42688_GYRO_DATA_X1              0x25
#define ICM42688_INT_STATUS                0x2D
#define ICM42688_DATA_RDY_STATUS           0x35
#define ICM42688_SIGNAL_PATH_RESET         0x4B
#define ICM42688_PWR_MGMT0                 0x4E
#define ICM42688_GYRO_CONFIG0              0x4F
#define ICM42688_ACCEL_CONFIG0             0x50
#define ICM42688_WHO_AM_I                  0x75
#define ICM42688_REG_BANK_SEL              0x76

/* Bank 1 */
#define ICM42688_GYRO_CONFIG_STATIC3       0x0C

/* Bank 2 */
#define ICM42688_ACCEL_CONFIG_STATIC3      0x04

/* ==================== SPI 相关定义 ==================== */

#define ICM42688_SPI_READ_BIT              0x80

/* ==================== 设备 ID ==================== */

#define ICM42688_ID                        0x47

/* ==================== 加速度计量程 ==================== */

#define AFS_2G                             0x03
#define AFS_4G                             0x02
#define AFS_8G                             0x01
#define AFS_16G                            0x00

/* ==================== 陀螺仪量程 ==================== */

#define GFS_2000DPS                        0x00
#define GFS_1000DPS                        0x01
#define GFS_500DPS                         0x02
#define GFS_250DPS                         0x03
#define GFS_125DPS                         0x04

/* ==================== 输出数据率 ==================== */

#define AODR_200Hz                         0x07
#define AODR_100Hz                         0x08

#define GODR_200Hz                         0x07
#define GODR_100Hz                         0x08

/* ==================== 滤波器配置 ==================== */

#define AAVG_4X                            0x02
#define AAF_1_16_ODR                       0x04
#define GAVG_4X                            0x02
#define GF_1_16_ODR                        0x04

/* ==================== 数据结构体 ==================== */

/**
 * @brief IMU 原始数据结构体
 */
typedef struct {
    int16_t x, y, z;
} icm42688_raw_data_t;

/**
 * @brief IMU 传感器数据结构体（转换后的物理值）
 */
typedef struct {
    float ax, ay, az;       // 加速度计 (g)
    float gx, gy, gz;       // 陀螺仪 (dps)
    float temperature;      // 温度 (°C)
} icm42688_sensor_data_t;

/* ==================== API 函数 ==================== */

/**
 * @brief   初始化 ICM42688
 * @return  0=成功, -1=失败
 */
int8_t icm42688_init(void);

/**
 * @brief   读取 WHO_AM_I 寄存器
 */
uint8_t icm42688_read_id(void);

/**
 * @brief   获取温度
 */
int8_t icm42688_get_temperature(float *temperature);

/**
 * @brief   获取加速度计数据 (g)
 */
int8_t icm42688_get_accelerometer(icm42688_raw_data_t *raw,
                                   float *ax, float *ay, float *az);

/**
 * @brief   获取陀螺仪数据 (dps)
 */
int8_t icm42688_get_gyroscope(icm42688_raw_data_t *raw,
                               float *gx, float *gy, float *gz);

/**
 * @brief   一次性读取所有传感器数据
 */
int8_t icm42688_get_all_data(icm42688_sensor_data_t *data);

#ifdef __cplusplus
}
#endif

#endif /* __ICM42688_H__ */
