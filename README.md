# 嵌入式飞控系统架构设计文档 (基于 QEMU & FreeRTOS)

## 1. 项目概述
本项目旨在 QEMU 仿真环境下（模拟 LM3S6965EVB 开发板），基于 FreeRTOS 实时操作系统实现一套完整的飞控系统。系统通过 UART1 与外部 Gazebo 仿真环境进行数据交互，实现对 Iris 无人机模型的闭环控制。

---

## 2. 系统架构图
系统采用典型的**分层与任务驱动架构**，各模块通过 FreeRTOS 提供的 IPC（进程间通信）机制进行解耦和同步。



---

## 3. 软件逻辑架构

### 3.1 任务（Task）定义与优先级
| 任务名称 | 优先级 | 唤醒机制 | 主要职责 |
| :--- | :--- | :--- | :--- |
| **Sensor Task** | 中 (Normal) | UART1 StreamBuffer 触发 | 解析 MAVLink 数据包，更新全局传感器状态。 |
| **Attitude Task** | 高 (Above Normal) | 任务通知 (Task Notification) | 运行 PID/姿态算法，计算电机控制指令。 |
| **Command Task** | 低 (Below Normal) | 队列等待 (Queue Block) | 将控制指令封装为 MAVLink 包并发送至 Gazebo。 |
| **Logger/Uart0 Task** | 低 (Idle/Low) | 队列等待 (Queue Block) | 异步打印系统日志，避免阻塞控制链路。 |

### 3.2 关键数据流向
1. **下行链路 (Uplink)**: `Gazebo` -> `UART1 (ISR)` -> `StreamBuffer` -> `Sensor Task` -> `全局结构体`。
2. **控制反馈 (Feedback)**: `Sensor Task` -> `Task Notification` -> `Attitude Task` -> `MotorControlQueue`。
3. **上行链路 (Downlink)**: `MotorControlQueue` -> `Command Task` -> `UART1` -> `Gazebo`。
4. **日志链路 (Logging)**: `各模块` -> `sprintf` -> `LogQueue` -> `Uart0 Task` -> `控制台`。

---

## 4. 详细模块设计

### 4.1 通信模块 (UART1)
* **输入处理**：UART1 中断服务例程将接收到的字节流即时推入 `UART1StreamBuffer`，确保中断处理时间极短。
* **协议解析**：Sensor 任务负责 MAVLink 协议状态机的维护。

### 4.2 核心数据池 (Global Structs)
系统维护四个关键的全局同步结构体：
* `CurrentSimTimeUs`: 同步 Gazebo 仿真时间。
* `UAVStatus`: 存储无人机加锁状态、模式等。
* `CurrentBaroData`: 气压计/高度数据。
* `CurrentImuData`: 加速度计与陀螺仪原始数据。

### 4.3 日志系统 (Logger)
为了保证飞控的实时性，日志系统采用**异步设计**：
* 使用 `sprintf` 格式化后，仅将指针或副本发送至 `LogQueue`。
* 由低优先级的 Uart0 任务缓慢处理 I/O 操作。

---

## 5. 实时性与稳定性设计
* **优先级抢占**：姿态控制任务（Attitude）优先级设为最高，确保传感器更新后能立即计算控制量，最小化控制延迟（Latency）。
* **缓冲区保护**：使用 `StreamBuffer` 处理高速字节流，平衡了中断处理速度与任务处理能力。
* **任务解耦**：通过 `Queue` 与 `Notification` 机制，使得传感器频率与控制频率、发送频率可以异步运行。