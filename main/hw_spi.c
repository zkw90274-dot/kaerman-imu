/**
 * @file    hw_spi.c
 * @brief   ESP32 硬件 SPI 驱动实现
 * @author  Claude
 * @date    2026-07-21
 * @version 1.0.0
 *
 * @details
 * 使用 ESP-IDF 硬件 SPI 主机驱动
 * CS 引脚软件控制，便于兼容不同传感器
 */

#include "hw_spi.h"
#include "esp_log.h"
#include <string.h>

static const char *TAG = "hw_spi";

/* ==================== 公共 API 实现 ==================== */

/**
 * @brief   初始化硬件 SPI
 */
int hw_spi_init(hw_spi_t *spi, const hw_spi_config_t *config)
{
    esp_err_t ret;

    if (spi == NULL || config == NULL) {
        return -1;
    }

    /* 1. 配置 CS 引脚为输出，默认高电平 */
    gpio_config_t cs_cfg = {
        .pin_bit_mask = (1ULL << config->cs_pin),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&cs_cfg);
    gpio_set_level(config->cs_pin, 1);

    /* 2. 初始化 SPI 总线 */
    spi_bus_config_t bus_cfg = {
        .sclk_io_num   = config->sclk_pin,
        .mosi_io_num   = config->mosi_pin,
        .miso_io_num   = config->miso_pin,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 64,
    };

    ret = spi_bus_initialize(config->host, &bus_cfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "spi_bus_initialize failed: %s", esp_err_to_name(ret));
        return -1;
    }

    /* 3. 添加 SPI 设备 */
    spi_device_interface_config_t dev_cfg = {
        .clock_speed_hz = config->clock_speed_hz,
        .mode           = 0,                /* CPOL=0, CPHA=0 */
        .spics_io_num   = -1,               /* 软件控制 CS */
        .queue_size     = 1,
    };

    ret = spi_bus_add_device(config->host, &dev_cfg, &spi->spi_dev);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "spi_bus_add_device failed: %s", esp_err_to_name(ret));
        return -1;
    }

    spi->cs_pin = config->cs_pin;
    spi->inited = 1;

    ESP_LOGI(TAG, "SPI%d init OK (SCLK=%d, MOSI=%d, MISO=%d, CS=%d, CLK=%d Hz)",
             config->host, config->sclk_pin, config->mosi_pin,
             config->miso_pin, config->cs_pin, config->clock_speed_hz);

    return 0;
}

/**
 * @brief   设置 CS 电平
 */
void hw_spi_set_cs(hw_spi_t *spi, uint8_t level)
{
    gpio_set_level(spi->cs_pin, level);
}

/**
 * @brief   发送并接收一个字节
 */
uint8_t hw_spi_transfer_byte(hw_spi_t *spi, uint8_t data)
{
    uint8_t rx_data = 0;

    spi_transaction_t t = {
        .length    = 8,
        .tx_buffer = &data,
        .rx_buffer = &rx_data,
    };

    spi_device_polling_transmit(spi->spi_dev, &t);

    return rx_data;
}

/**
 * @brief   写入寄存器
 */
void hw_spi_write_reg(hw_spi_t *spi, uint8_t reg, uint8_t value)
{
    uint8_t tx_buf[2] = { reg & 0x7F, value };

    spi_transaction_t t = {
        .length    = 8 * 2,
        .tx_buffer = tx_buf,
        .rx_buffer = NULL,
    };

    gpio_set_level(spi->cs_pin, 0);
    spi_device_polling_transmit(spi->spi_dev, &t);
    gpio_set_level(spi->cs_pin, 1);
}

/**
 * @brief   读取寄存器
 */
uint8_t hw_spi_read_reg(hw_spi_t *spi, uint8_t reg)
{
    uint8_t tx_buf[2] = { reg | 0x80, 0x00 };
    uint8_t rx_buf[2] = { 0 };

    spi_transaction_t t = {
        .length    = 8 * 2,
        .tx_buffer = tx_buf,
        .rx_buffer = rx_buf,
    };

    gpio_set_level(spi->cs_pin, 0);
    spi_device_polling_transmit(spi->spi_dev, &t);
    gpio_set_level(spi->cs_pin, 1);

    return rx_buf[1];
}

/**
 * @brief   读取多个连续寄存器
 */
void hw_spi_read_regs(hw_spi_t *spi, uint8_t reg, uint8_t *buf, uint16_t len)
{
    /* 命令字节 + 数据字节，一次事务完成 */
    /* 最大 14 字节 (IMU: temp2 + accel6 + gyro6)，使用栈缓冲区 */
    uint8_t tx_buf[15];
    uint8_t rx_buf[15];

    if (len > 14) {
        ESP_LOGE(TAG, "read_regs: len %d exceeds max 14", len);
        return;
    }

    tx_buf[0] = reg | 0x80;
    memset(tx_buf + 1, 0x00, len);

    spi_transaction_t t = {
        .length    = 8 * (1 + len),
        .tx_buffer = tx_buf,
        .rx_buffer = rx_buf,
    };

    gpio_set_level(spi->cs_pin, 0);
    spi_device_polling_transmit(spi->spi_dev, &t);
    gpio_set_level(spi->cs_pin, 1);

    memcpy(buf, rx_buf + 1, len);
}
