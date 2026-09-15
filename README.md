# Sakura Self-Study FOC

基于 STM32F407 的无刷直流电机（BLDC/PMSM）FOC 学习、移植与实机调试项目。工程围绕三相电流采样、SVPWM、AS5047P 磁编码器和串级闭环控制展开，用于记录从电流环到速度环、位置环的实现与验证过程。

> 本仓库是个人学习与工程实践记录，包含 STM32 HAL、CMSIS 等第三方组件。各第三方组件遵循其目录内原有许可证；未明确标注为本人原创的底层或算法代码不作原创声明。

## 已实现功能

- STM32F407IGT6 + Keil MDK / STM32CubeMX 工程
- 三相互补 PWM 与 ADC 同步电流采样
- Clarke/Park 变换、反 Park 变换与 SVPWM
- AS5047P 编码器 SPI 读取、电角度零点标定
- 编码器跨零处理与多圈机械位置累计
- 转矩/电流闭环：`Iq → 电流 PI → SVPWM`
- 速度—电流串级闭环：`速度 → 速度 PI → Iq → 电流 PI`
- 位置—速度—电流串级闭环：`位置 → 速度 → Iq → 电流 PI`
- 串口命令调试与 VOFA 数据输出
- 软件过流保护、目标限幅及模式切换安全停机

## 硬件配置

| 项目 | 配置 |
| --- | --- |
| MCU | STM32F407IGT6 |
| 开发板 | 野火 STM32F407 电机控制板 |
| 电机 | 2804 无刷电机，7 对极 |
| 位置传感器 | AS5047P 绝对值磁编码器，SPI 接口 |
| 母线电压 | 24 V（调试时需结合电机额定参数限流） |
| 调制方式 | SVPWM |
| PWM / FOC 频率 | 20 kHz |
| 软件过流阈值 | 1.6 A |

当前验证配置为 `phase_map = 5`、编码器方向 `+1`、Iq 方向 `+1`。更换电机、编码器或功率板后不可直接沿用，必须重新核对相序、方向、偏置和限流参数。

## 工程结构

```text
selfmoter/
├─ README.md
├─ 电机参数.png
└─ selfmoter_test_open_FOC/
   ├─ Core/                 # STM32CubeMX 生成的启动与外设代码
   ├─ Drivers/              # STM32 HAL 与 CMSIS
   ├─ Foc/
   │  ├─ foc/               # FOC 算法、控制器与硬件适配
   │  ├─ bsp/               # 板级支持、串口命令和测试入口
   │  └─ debug/             # 调试与 VOFA 输出
   ├─ docs/                 # 项目文档
   ├─ MDK-ARM/              # Keil 工程文件
   ├─ FOC_CLOSED_LOOP_GUIDE.md
   └─ selfmoter.ioc         # STM32CubeMX 配置
```

## 关键代码

- `selfmoter_test_open_FOC/Foc/foc/foc_user_config.h`：电机参数、PI 参数、目标值与保护阈值
- `selfmoter_test_open_FOC/Foc/foc/foc_motor.c`：FOC 周期执行与串级闭环逻辑
- `selfmoter_test_open_FOC/Foc/foc/foc_hw.c`：PWM、ADC、电流采样、编码器及驱动接口
- `selfmoter_test_open_FOC/Foc/foc/foc_app.c`：控制对象和模式配置
- `selfmoter_test_open_FOC/Foc/bsp/bsp.c`：串口命令、按键和测试入口
- `selfmoter_test_open_FOC/Core/Src/tim.c`：PWM 与控制频率配置

## 构建与运行

1. 使用 Keil MDK 打开 `selfmoter_test_open_FOC/MDK-ARM/selfmoter.uvprojx`。
2. 安装对应的 STM32F4 Device Family Pack。
3. 核对 `foc_user_config.h` 中的母线电压、极对数、相序、编码器方向、电流方向和保护阈值。
4. 编译并通过调试器烧录到 STM32F407IGT6。
5. 首次上电使用限流电源，并确保能够立即执行 `stop` 或切断驱动使能。

CubeMX 外设配置位于 `selfmoter_test_open_FOC/selfmoter.ioc`。重新生成代码前请先提交或备份用户代码。

## 串口调试命令

| 命令 | 作用 |
| --- | --- |
| `qtest` | 启动转矩/电流环基线测试 |
| `stest` | 启动速度—电流环基线测试 |
| `ptest` | 启动位置—速度—电流环基线测试 |
| `stop` | 停止电机 |
| `iq=20` | 设置 Iq 目标，单位 mA |
| `id=0` | 设置 Id 目标，单位 mA |
| `spd=100` | 设置目标速度，单位 rpm |
| `pos=360` | 设置目标机械位置，单位度，支持多圈 |
| `vlog=current/speed/position/auto/off` | 选择或关闭 VOFA 输出 |

按键控制和三种闭环的详细参数索引见 [`FOC_CLOSED_LOOP_GUIDE.md`](selfmoter_test_open_FOC/FOC_CLOSED_LOOP_GUIDE.md)。

## 当前验证状态

- 电流环在堵转测试下，50 mA 与 100 mA 的 Iq 目标能够跟踪。
- 速度环和位置环已具备代码及测试入口，仍需进一步系统整定并记录阶跃响应、稳态误差、超调和调节时间。
- CAN、CANopen、CiA 402、FreeRTOS 和完整故障管理属于后续规划，当前版本不宣称已实现。

## 安全说明

电机控制涉及高速旋转和功率电子器件。调试前应固定电机、使用限流电源、确认电流采样偏置和过流保护，并预留可靠的硬件断电方式。仓库参数仅对应当前实验平台，不应直接用于其他硬件。

