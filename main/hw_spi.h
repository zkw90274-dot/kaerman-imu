/**
 * @file    hw_spi.h
 * @brief   ESP32 硬件 SPI 驱动
 * @author  Claude
 * @date    2026-07-21
 * @version 1.0.0
 *
 * @details
 * 使用 ESP-IDF 硬件 SPI 主机驱动
 * 比软件 SPI 快 10-50 倍，且 CPU 占用更低
 */

#ifndef __HW_SPI_H__
#define __HW_SPI_H__

#include <stdint.h>
#include "driver/spi_master.h"
#include "driver/gpio.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ==================== 配置常量 ==================== */

/* 默认引脚定义（可根据实际修改） */
#define HW_SPI_HOST         SPI2_HOST
#define HW_SPI_SCLK_PIN     GPIO_NUM_12
#define HW_SPI_MOSI_PIN     GPIO_NUM_11
#define HW_SPI_MISO_PIN     GPIO_NUM_10
#define HW_SPI_CS_PIN       GPIO_NUM_9
#define HW_SPI_CLK_HZ       (10 * 1000 * 1000)  /* 10 MHz */

/* ==================== 数据结构体 ==================== */

/**
 * @brief 硬件 SPI 配置结构体
 */
typedef struct {
    spi_host_device_t host;     // SPI 主机号
    gpio_num_t sclk_pin;        // 时钟引脚
    gpio_num_t mosi_pin;        // 主出从入引脚
    gpio_num_t miso_pin;        // 主入从出引脚
    gpio_num_t cs_pin;          // 片选引脚
    int clock_speed_hz;         // 时钟频率
} hw_spi_config_t;

/**
 * @brief 硬件 SPI 句柄结构体
 */
typedef struct {
    spi_device_handle_t spi_dev;
    gpio_num_t cs_pin;
    uint8_t inited;
} hw_spi_t;

/* ==================== 默认配置 ==================== */

#define HW_SPI_DEFAULT_CONFIG  { \
    .host = HW_SPI_HOST, \
    .sclk_pin = HW_SPI_SCLK_PIN, \
    .mosi_pin = HW_SPI_MOSI_PIN, \
    .miso_pin = HW_SPI_MISO_PIN, \
    .cs_pin = HW_SPI_CS_PIN, \
    .clock_speed_hz = HW_SPI_CLK_HZ \
}

/* ==================== API 函数 ==================== */

/**
 * @brief   初始化硬件 SPI
 * @param   spi     SPI 句柄指针
 * @param   config  配置结构体指针
 * @return  0=成功, -1=失败
 */
int hw_spi_init(hw_spi_t *spi, const hw_spi_config_t *config);

/**
 * @brief   设置 CS 电平
 */
void hw_spi_set_cs(hw_spi_t *spi, uint8_t level);

/**
 * @brief   发送并接收一个字节
 */
uint8_t hw_spi_transfer_byte(hw_spi_t *spi, uint8_t data);

/**
 * @brief   写入寄存器
 */
void hw_spi_write_reg(hw_spi_t *spi, uint8_t reg, uint8_t value);

/**
 * @brief   读取寄存器
 */
uint8_t hw_spi_read_reg(hw_spi_t *spi, uint8_t reg);

/**
 * @brief   读取多个连续寄存器
 */
void hw_spi_read_regs(hw_spi_t *spi, uint8_t reg, uint8_t *buf, uint16_t len);

#ifdef __cplusplus
}
#endif

#endif /* __HW_SPI_H__ */
