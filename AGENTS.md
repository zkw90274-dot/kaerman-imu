# AGENTS.md

This file provides guidance to Codex (Codex.ai/code) when working with code in this repository.

## 项目概述

ESP32-S3 IMU 姿态解算系统，基于 ICM-42688-P 六轴传感器。双滤波器并行运行，均输出四元数→欧拉角。

## 构建与烧录

```bash
idf.py build                     # 编译
idf.py -p COM18 flash            # 烧录（端口按实际修改）
idf.py -p COM18 monitor          # 串口监控
idf.py -p COM18 flash monitor    # 烧录+监控
```

## 架构

```
ICM42688 (SPI 10MHz, GPIO 9/10/11/12)
    │
    └── icm42688 驱动 → 原始数据 (ax,ay,az,gx,gy,gz)
            │
            ├── 零漂补偿 (500 样本标定，main 中统一处理)
            │
            ├─── Pipeline 1: 卡尔曼滤波 ─── Pipeline 2: Mahony 滤波
            │    ├─ 自适应 R (振动)          │    ├─ 自适应 Kp (振动)
            │    ├─ 自适应 Q (运动)          │    ├─ 条件积分 (防饱和)
            │    ├─ Yaw 陀螺仪积分+死区      │    ├─ 加速度计初始化四元数
            │    └─ Roll/Pitch → 四元数      │    └─ 四元数微分方程
            │                                 │
            └───── imu_math (共享) ──────────┘
                    ├─ 快速平方根倒数
                    ├─ 安全三角函数 (sin/cos/asin/atan2)
                    └─ 四元数工具 (euler↔quat, multiply, normalize)
```

## 模块文件

| 文件 | 职责 |
|------|------|
| `hw_spi.c/h` | ESP32 硬件 SPI 驱动 (SPI2_HOST) |
| `icm42688.c/h` | ICM-42688-P 传感器驱动 |
| `imu_math.c/h` | 数学函数 + 四元数工具（无 math.h 依赖） |
| `kalman.c/h` | 自适应卡尔曼滤波器（Pipeline 1） |
| `ahrs.c/h` | Mahony 互补滤波器（Pipeline 2） |
| `hello_world_main.c` | 应用入口，双滤波器主循环 |

## 关键数据结构

```c
icm42688_sensor_data_t  // ax,ay,az(g) + gx,gy,gz(dps) + temperature(°C)
kalman_output_t         // quat + euler + vibration_weight + motion_weight + adaptive_R/Q
mahony_output_t         // quat + euler + vibration_weight + yaw_rate
imu_quat_t              // q0(w), q1(x), q2(y), q3(z)
imu_euler_t             // roll, pitch, yaw (度)
```

## VOFA 输出格式

```
K:roll,pitch,yaw,M:roll,pitch,yaw
```

## 滤波器参数

| 参数 | 卡尔曼 | Mahony |
|------|--------|--------|
| 主要参数 | Q_angle=0.001, Q_bias=0.003, R_measure=0.03 | Kp=8.0, Ki=0.002 |
| 振动自适应 | R 自动增大 (VIBRATION_THRESHOLD=0.08g) | Kp 自动降低 |
| 运动自适应 | Q 自动增大 (MOTION_THRESHOLD=20dps) | — |
| Yaw 死区 | 0.5 dps | — |
| 初始化 | 无需（加速度计直接解算） | 加速度计初始化四元数 |
| 积分策略 | — | 条件积分（err_mag < 1.0 时累积） |

## 编码规范

- C17，ESP-IDF + FreeRTOS，缩进 4 空格，K&R 大括号
- 命名：文件 `snake_case.c`，函数 `模块_动作()`，宏 `全大写_SNAKE`，结构体 `snake_case_t`
- 禁止动态内存分配，中断变量必须 `volatile`，所有函数必须有 doxygen 注释

## 硬件引脚

| 功能 | GPIO |
|------|------|
| SPI_SCLK | 12 |
| SPI_MOSI | 11 |
| SPI_MISO | 10 |
| SPI_CS | 9 |

## 依赖

- ESP-IDF v5.4.3，目标芯片 ESP32-S3
