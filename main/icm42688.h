/**
 * @file    icm42688.h
 * @brief   ICM-42688-P 六轴 IMU 驱动（硬件 SPI 接口）
 * @author  Claude
 * @date    2026-07-21
 * @version 2.0.0
 *
 * @details
 * 基于 ICM-42688-P Datasheet (DS-000347, Rev 1.2) 完善
 * 支持加速度计 + 陀螺仪 + 温度传感器
 * 支持 FIFO 读取、中断配置、自检功能
 */

#ifndef __ICM42688_H__
#define __ICM42688_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ==================== 设备 ID ==================== */

#define ICM42688_WHO_AM_I_VAL              0x47

/* ==================== SPI 相关定义 ==================== */

#define ICM42688_SPI_READ_BIT              0x80
#define ICM42688_SPI_WRITE_BIT             0x00

/* ==================== Bank 0 寄存器地址 ==================== */

/* 配置寄存器 */
#define ICM42688_REG_DEVICE_CONFIG         0x11
#define ICM42688_REG_DRIVE_CONFIG          0x13
#define ICM42688_REG_INT_CONFIG            0x14
#define ICM42688_REG_FIFO_CONFIG           0x16

/* 数据寄存器 */
#define ICM42688_REG_TEMP_DATA1            0x1D
#define ICM42688_REG_TEMP_DATA0            0x1E
#define ICM42688_REG_ACCEL_DATA_X1         0x1F
#define ICM42688_REG_ACCEL_DATA_X0         0x20
#define ICM42688_REG_ACCEL_DATA_Y1         0x21
#define ICM42688_REG_ACCEL_DATA_Y0         0x22
#define ICM42688_REG_ACCEL_DATA_Z1         0x23
#define ICM42688_REG_ACCEL_DATA_Z0         0x24
#define ICM42688_REG_GYRO_DATA_X1          0x25
#define ICM42688_REG_GYRO_DATA_X0          0x26
#define ICM42688_REG_GYRO_DATA_Y1          0x27
#define ICM42688_REG_GYRO_DATA_Y0          0x28
#define ICM42688_REG_GYRO_DATA_Z1          0x29
#define ICM42688_REG_GYRO_DATA_Z0          0x2A

/* 状态寄存器 */
#define ICM42688_REG_INT_STATUS            0x2D
#define ICM42688_REG_FIFO_COUNT_H          0x2E
#define ICM42688_REG_FIFO_COUNT_L          0x2F
#define ICM42688_REG_FIFO_DATA             0x30
#define ICM42688_REG_INT_STATUS2           0x37
#define ICM42688_REG_INT_STATUS3           0x38

/* 电源和传感器配置 */
#define ICM42688_REG_SIGNAL_PATH_RESET     0x4B
#define ICM42688_REG_INTF_CONFIG0          0x4C
#define ICM42688_REG_INTF_CONFIG1          0x4D
#define ICM42688_REG_PWR_MGMT0             0x4E
#define ICM42688_REG_GYRO_CONFIG0          0x4F
#define ICM42688_REG_ACCEL_CONFIG0         0x50
#define ICM42688_REG_GYRO_CONFIG1          0x51
#define ICM42688_REG_GYRO_ACCEL_CONFIG0    0x52
#define ICM42688_REG_ACCEL_CONFIG1         0x53
#define ICM42688_REG_TMST_CONFIG           0x54

/* APEX 配置 */
#define ICM42688_REG_APEX_CONFIG0          0x56
#define ICM42688_REG_SMD_CONFIG            0x57

/* FIFO 配置 */
#define ICM42688_REG_FIFO_CONFIG1          0x5F
#define ICM42688_REG_FIFO_CONFIG2          0x60
#define ICM42688_REG_FIFO_CONFIG3          0x61

/* 中断配置 */
#define ICM42688_REG_INT_CONFIG0           0x63
#define ICM42688_REG_INT_CONFIG1           0x64
#define ICM42688_REG_INT_SOURCE0           0x65
#define ICM42688_REG_INT_SOURCE1           0x66
#define ICM42688_REG_INT_SOURCE3           0x68
#define ICM42688_REG_INT_SOURCE4           0x69

/* 自检和 ID */
#define ICM42688_REG_SELF_TEST_CONFIG      0x70
#define ICM42688_REG_WHO_AM_I              0x75
#define ICM42688_REG_REG_BANK_SEL          0x76

/* ==================== Bank 1 寄存器地址 ==================== */

#define ICM42688_REG_SENSOR_CONFIG0        0x03
#define ICM42688_REG_GYRO_CONFIG_STATIC2   0x0B
#define ICM42688_REG_GYRO_CONFIG_STATIC3   0x0C
#define ICM42688_REG_GYRO_CONFIG_STATIC4   0x0D
#define ICM42688_REG_GYRO_CONFIG_STATIC5   0x0E
#define ICM42688_REG_GYRO_CONFIG_STATIC6   0x0F
#define ICM42688_REG_GYRO_CONFIG_STATIC7   0x10
#define ICM42688_REG_GYRO_CONFIG_STATIC8   0x11
#define ICM42688_REG_GYRO_CONFIG_STATIC9   0x12
#define ICM42688_REG_GYRO_CONFIG_STATIC10  0x13
#define ICM42688_REG_INTF_CONFIG4          0x7A
#define ICM42688_REG_INTF_CONFIG6          0x7C

/* ==================== Bank 2 寄存器地址 ==================== */

#define ICM42688_REG_ACCEL_CONFIG_STATIC2  0x03
#define ICM42688_REG_ACCEL_CONFIG_STATIC3  0x04
#define ICM42688_REG_ACCEL_CONFIG_STATIC4  0x05

/* ==================== Bank 4 寄存器地址 ==================== */

#define ICM42688_REG_APEX_CONFIG1          0x40
#define ICM42688_REG_APEX_CONFIG2          0x41
#define ICM42688_REG_APEX_CONFIG3          0x42
#define ICM42688_REG_APEX_CONFIG4          0x43
#define ICM42688_REG_APEX_CONFIG5          0x44
#define ICM42688_REG_APEX_CONFIG6          0x45
#define ICM42688_REG_APEX_CONFIG7          0x46
#define ICM42688_REG_APEX_CONFIG8          0x47
#define ICM42688_REG_APEX_CONFIG9          0x48
#define ICM42688_REG_ACCEL_WOM_X_THR      0x4A
#define ICM42688_REG_ACCEL_WOM_Y_THR      0x4B
#define ICM42688_REG_ACCEL_WOM_Z_THR      0x4C
#define ICM42688_REG_INT_SOURCE6           0x4D
#define ICM42688_REG_INT_SOURCE7           0x4E
#define ICM42688_REG_INT_SOURCE8           0x4F
#define ICM42688_REG_INT_SOURCE9           0x50
#define ICM42688_REG_INT_SOURCE10          0x51
#define ICM42688_REG_OFFSET_USER0          0x77
#define ICM42688_REG_OFFSET_USER1          0x78
#define ICM42688_REG_OFFSET_USER2          0x79
#define ICM42688_REG_OFFSET_USER3          0x7A
#define ICM42688_REG_OFFSET_USER4          0x7B
#define ICM42688_REG_OFFSET_USER5          0x7C
#define ICM42688_REG_OFFSET_USER6          0x7D
#define ICM42688_REG_OFFSET_USER7          0x7E
#define ICM42688_REG_OFFSET_USER8          0x7F

/* ==================== PWR_MGMT0 位定义 ==================== */

#define ICM42688_PWR_MGMT0_TEMP_DIS        (1 << 5)
#define ICM42688_PWR_MGMT0_IDLE            (1 << 4)
#define ICM42688_PWR_MGMT0_GYRO_MODE_OFF   (0 << 2)
#define ICM42688_PWR_MGMT0_GYRO_MODE_STBY  (1 << 2)
#define ICM42688_PWR_MGMT0_GYRO_MODE_LN    (3 << 2)
#define ICM42688_PWR_MGMT0_ACCEL_MODE_OFF  (0 << 0)
#define ICM42688_PWR_MGMT0_ACCEL_MODE_LP   (2 << 0)
#define ICM42688_PWR_MGMT0_ACCEL_MODE_LN   (3 << 0)

/* ==================== 陀螺仪量程定义 ==================== */

#define ICM42688_GYRO_FS_2000DPS           0x00
#define ICM42688_GYRO_FS_1000DPS           0x01
#define ICM42688_GYRO_FS_500DPS            0x02
#define ICM42688_GYRO_FS_250DPS            0x03
#define ICM42688_GYRO_FS_125DPS            0x04
#define ICM42688_GYRO_FS_62_5DPS           0x05
#define ICM42688_GYRO_FS_31_25DPS          0x06
#define ICM42688_GYRO_FS_15_625DPS         0x07

/* ==================== 加速度计量程定义 ==================== */

#define ICM42688_ACCEL_FS_16G              0x00
#define ICM42688_ACCEL_FS_8G               0x01
#define ICM42688_ACCEL_FS_4G               0x02
#define ICM42688_ACCEL_FS_2G               0x03

/* ==================== 输出数据率定义 ==================== */

#define ICM42688_ODR_32000HZ               0x01
#define ICM42688_ODR_16000HZ               0x02
#define ICM42688_ODR_8000HZ                0x03
#define ICM42688_ODR_4000HZ                0x04
#define ICM42688_ODR_2000HZ                0x05
#define ICM42688_ODR_1000HZ                0x06
#define ICM42688_ODR_200HZ                 0x07
#define ICM42688_ODR_100HZ                 0x08
#define ICM42688_ODR_50HZ                  0x09
#define ICM42688_ODR_25HZ                  0x0A
#define ICM42688_ODR_12_5HZ                0x0B
#define ICM42688_ODR_500HZ                 0x0F

/* ==================== 中断状态位定义 ==================== */

/* INT_STATUS (0x2D) */
#define ICM42688_INT_STATUS_UI_FSYNC       (1 << 6)
#define ICM42688_INT_STATUS_PLL_RDY        (1 << 5)
#define ICM42688_INT_STATUS_RESET_DONE     (1 << 4)
#define ICM42688_INT_STATUS_DATA_RDY       (1 << 3)
#define ICM42688_INT_STATUS_FIFO_THS       (1 << 2)
#define ICM42688_INT_STATUS_FIFO_FULL      (1 << 1)
#define ICM42688_INT_STATUS_AGC_RDY        (1 << 0)

/* INT_STATUS2 (0x37) */
#define ICM42688_INT_STATUS2_SMD           (1 << 3)
#define ICM42688_INT_STATUS2_WOM_Z         (1 << 2)
#define ICM42688_INT_STATUS2_WOM_Y         (1 << 1)
#define ICM42688_INT_STATUS2_WOM_X         (1 << 0)

/* INT_STATUS3 (0x38) */
#define ICM42688_INT_STATUS3_STEP_DET      (1 << 5)
#define ICM42688_INT_STATUS3_STEP_CNT_OVF  (1 << 4)
#define ICM42688_INT_STATUS3_TILT_DET      (1 << 3)
#define ICM42688_INT_STATUS3_WAKE          (1 << 2)
#define ICM42688_INT_STATUS3_SLEEP         (1 << 1)
#define ICM42688_INT_STATUS3_TAP_DET       (1 << 0)

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

/**
 * @brief IMU 配置结构体
 */
typedef struct {
    uint8_t gyro_fs;        // 陀螺仪量程
    uint8_t accel_fs;       // 加速度计量程
    uint8_t gyro_odr;       // 陀螺仪 ODR
    uint8_t accel_odr;      // 加速度计 ODR
} icm42688_config_t;

/* ==================== 默认配置 ==================== */

#define ICM42688_DEFAULT_CONFIG  { \
    .gyro_fs = ICM42688_GYRO_FS_1000DPS, \
    .accel_fs = ICM42688_ACCEL_FS_2G, \
    .gyro_odr = ICM42688_ODR_200HZ, \
    .accel_odr = ICM42688_ODR_200HZ \
}

/* ==================== API 函数 ==================== */

/**
 * @brief   初始化 ICM42688
 * @param   config  配置参数（NULL 使用默认配置）
 * @return  0=成功, -1=失败
 */
int8_t icm42688_init(const icm42688_config_t *config);

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

/**
 * @brief   软复位
 */
int8_t icm42688_reset(void);

/**
 * @brief   自检
 * @return  0=通过, -1=失败
 */
int8_t icm42688_self_test(void);

/**
 * @brief   读取中断状态
 */
uint8_t icm42688_read_int_status(void);

/**
 * @brief   配置中断
 */
int8_t icm42688_config_interrupt(uint8_t int_source);

/**
 * @brief   切换寄存器 Bank
 */
int8_t icm42688_select_bank(uint8_t bank);

/**
 * @brief   获取当前配置
 */
void icm42688_get_config(icm42688_config_t *config);

#ifdef __cplusplus
}
#endif

#endif /* __ICM42688_H__ */
