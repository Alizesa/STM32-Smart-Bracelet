# STM32 Smart Bracelet（智能手环/手表）

基于 **STM32F103C8T6 + FreeRTOS** 的智能手环固件，使用 Keil MDK 5 开发。支持心率、计步/抬腕/跌倒检测、温湿度、蓝牙上报与 NFC 门禁模拟。

## 功能

| 功能 | 说明 |
|---|---|
| 🎨 菜单 UI | 参考式图标菜单：时钟首页 + 返回/心率/门禁/数据 功能页，OLED 128×64 |
| ❤️ 心率 / SpO2 | MAX30102，实时波形 + 心率值 |
| 🏃 计步 / 抬腕 / 跌倒 | MPU6050，跌倒触发全屏告警弹窗 |
| 🌡 温湿度 | DHT11 |
| 📶 蓝牙 | HC-05，周期上报 `DATA:` 数据，支持命令查询 |
| 💳 NFC 门禁 | PN532，**软件模拟 I2C**（SCL=PA2 / SDA=PA3），读卡 UID 模拟开门 |

## 外设与引脚

| 外设 | 接口 | 引脚 |
|---|---|---|
| OLED 0.96" | I2C（软件） | PB8=SCL, PB9=SDA |
| MPU6050 | I2C（软件） | PB10=SCL, PB11=SDA |
| MAX30102 | I2C（软件） | PB6=SCL, PB7=SDA |
| DHT11 | 单总线 | PA1 |
| HC-05 蓝牙 | USART1 | PA9=TX, PA10=RX |
| PN532 NFC | 软件 I2C | **PA2=SCL, PA3=SDA** |
| 按键 | GPIO 上拉 | PB1 / PA6 / PA4 |
| MPU6050 INT（可选） | EXTI | PB12 |

> 注：工程内所有 I2C 均为软件模拟（开漏 + 上拉），未使用硬件 I2C 外设。

## NFC 接线与说明（PN532）

PN532 模块需用 **DIP 拨到 I2C 模式**（上电采样，拨完要重新上电），I2C 地址 `0x24`：

```
模块 SCL → STM32 PA2
模块 SDA → STM32 PA3
模块 VCC → 5V
模块 GND → STM32 GND
```

驱动要点（`Hardware/PN532.c`）：
- 读回不依赖 IRQ：轮询 I2C 读地址，NACK=忙、ACK=就绪后一次连读 40 字节再扫帧。
- 兼容 SCL 时钟拉伸；每段 I2C 事务用 `vTaskSuspendAll()` 保护，避免被高优先级任务抢占破坏位带时序。
- 在线判定以 GetFirmwareVersion 为准（部分兼容板对 SAMConfig 只回 echo、无状态字节，故 SAMConfig 采用 fire-and-forget）。

## 软件架构

- **RTOS**：FreeRTOS（Cortex-M3 port），1ms 节拍。
- **任务**（`User/main.c`）：
  - `KeyScan` / `KeyHdl`：按键扫描与处理
  - `Sensor`：MPU6050 计步 / 姿态（20ms）
  - `HeartRate`：MAX30102 心率测量
  - `Bluetooth`：HC-05 收发与周期上报
  - `NFC`：PN532 巡检读卡
  - `Menu`：OLED 菜单 UI（40ms）
- **共享数据**：传感器/NFC 结果以 volatile 全局量提供给 UI（`gHeartRate`、`gSteps`、`gNfcUid[]` 等）。

## 编译烧录

1. 用 Keil MDK 5 打开 `Project.uvprojx`。
2. 选择目标后编译（`F7`），下载（`F8`）。

源码均为 UTF-8（含中文注释文件带 BOM），Keil 可正常显示中文。

## 目录结构

```
Hardware/   外设驱动（OLED/MPU6050/MAX30102/DHT11/HC05/PN532/Menu/Motion/...）
User/       主程序、任务、中断服务
FreeRtos/   FreeRTOS 源码
Library/    STM32 标准外设库
Start/      启动文件与核心寄存器定义
System/     延时/工具（DWT）
```
