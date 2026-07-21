# kaerman — ESP32 IMU 姿态解算系统

基于 ESP32-S3 + ICM-42688-P 六轴传感器的双滤波器姿态解算系统。

## 功能

- **双滤波器并行运行**：自适应卡尔曼 + Mahony 互补滤波
- **四元数输出** → ZYX 欧拉角（Roll, Pitch, Yaw）
- **自适应参数**：振动检测调整 R/Kp，运动检测调整 Q
- **Yaw 死区**：0.5 dps 以下不积分，抑制漂移
- **Mahony 加速度计初始化**：首帧即准确，无需收敛
- **VOFA 上位机输出**：FireWater 协议

## 硬件

| 功能 | GPIO |
|------|------|
| SPI_SCLK | 12 |
| SPI_MOSI | 11 |
| SPI_MISO | 10 |
| SPI_CS | 9 |

目标芯片：ESP32-S3

## 构建与烧录

需要 [ESP-IDF v5.4.3](https://docs.espressif.com/projects/esp-idf/en/stable/esp32s3/get-started/index.html) 环境。

```bash
idf.py set-target esp32s3
idf.py build
idf.py -p COM18 flash monitor
```

## 串口输出格式

```
K:roll,pitch,yaw,M:roll,pitch,yaw
```

- `K:` — 卡尔曼滤波输出
- `M:` — Mahony 滤波输出

## 项目结构

```
main/
├── hello_world_main.c   # 应用入口，双滤波器主循环
├── hw_spi.c/h           # ESP32 硬件 SPI 驱动
├── icm42688.c/h         # ICM-42688-P 传感器驱动
├── imu_math.c/h         # 数学函数 + 四元数工具（无 math.h 依赖）
├── kalman.c/h           # 自适应卡尔曼滤波器
└── ahrs.c/h             # Mahony 互补滤波器
```

## 依赖

- ESP-IDF v5.4.3
- FreeRTOS（ESP-IDF 内置）

## 参考

- [ICM-42688-P Datasheet](https://invensense.tdk.com/wp-content/uploads/2020/11/ds-000347-icm-42688-p-datasheet.pdf)
- [VOFA+ 上位机](https://vofa.plus/)
