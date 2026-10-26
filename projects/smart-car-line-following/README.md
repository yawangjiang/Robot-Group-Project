# STM32 八路光电传感器智能车循迹

基于 STM32F103C8 的双电机智能车循迹项目。车辆通过八路光电传感器获取赛道偏差，主循环计算当前偏差并调整左右电机速度；工程同时保留 MPU6050、OLED、按键和电机驱动相关模块。

![智能车与八路光电传感器实物](docs/images/prototype.jpg)

## 仓库内容

```text
smart-car-line-following/
├── firmware/keil/                 # STM32 固件与 Keil MDK 工程
│   ├── Hardware/                  # 传感器、电机、OLED、MPU6050 等驱动
│   ├── Library/                   # STM32F10x 标准外设库
│   ├── Start/                     # 启动文件与 CMSIS 文件
│   ├── System/                    # 延时等系统模块
│   ├── User/                      # 主程序与中断配置
│   └── Project.uvprojx            # Keil 工程入口
├── hardware/electronics/
│   └── stm32 8路光电管寻路.eprj2  # 立创 EDA 专业版原理图/PCB 工程
├── hardware/mechanical/
│   └── 支架.SLDPRT                # SolidWorks 支架模型
└── docs/images/prototype.jpg       # 实物照片
```

## 固件环境

- MCU：STM32F103C8（ARM Cortex-M3）
- IDE：Keil MDK / µVision
- 外设库：STM32F10x Standard Peripheral Library
- 主要模块：八路光电循迹、电机 PWM 控制、MPU6050、OLED、按键

## 编译和烧录

1. 使用 Keil µVision 打开 `firmware/keil/Project.uvprojx`。
2. 确认已安装 STM32F1 对应的 Device Pack。
3. 选择 `Target 1`，执行 Build。
4. 按实际调试器和开发板设置下载算法，然后烧录到 STM32F103C8。

编译生成的 `Objects/`、`Listings/`、`DebugConfig/` 以及个人 `Project.uvguix.*` 文件已排除，不进入版本控制。

## 硬件文件

- `hardware/electronics/*.eprj2`：使用立创 EDA 专业版打开，用于查看和修改原理图、PCB。
- `hardware/mechanical/*.SLDPRT`：使用 SolidWorks 打开，用于查看和修改传感器/车体支架。

引脚分配和接线应以 EDA 工程及 `firmware/keil/Hardware/` 下的驱动代码为准。
