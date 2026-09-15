# FOC 三种闭环使用与参数索引

## 1. 三种模式

### 力矩/电流闭环（loop1）

控制链：`Iq目标 -> d/q电流PI -> Ud/Uq -> SVPWM`

- 启动：串口发送 `torque` 或 `loop1`
- 运行中修改力矩电流：发送 `iq=30`，单位 mA；负值反向，例如 `iq=-30`
- 默认目标：`Foc/bsp/bsp.c` 中 `BSP_FOC_DEFAULT_TORQUE_IQ_A`
- 封装入口：`BSP_FOC_StartTorque(float iq_amp)`

电机实际力矩近似与 Iq 成正比。当前工程没有力矩传感器，因此命令设置的是 Iq，而不是 N·m。

### 速度—电流双闭环（loop2）

控制链：`速度目标 -> 速度PI -> Iq目标 -> 电流PI -> SVPWM`

- 启动：串口发送 `speed` 或 `loop2`
- 运行中修改速度：发送 `spd=300`，单位 rpm；负值反转
- 默认速度：`Foc/bsp/bsp.c` 中 `BSP_FOC_DEFAULT_SPEED_RPM`
- 最大Iq：`BSP_FOC_DEFAULT_IQ_LIMIT_A`
- 封装入口：`BSP_FOC_StartSpeed(float target_rpm, float iq_limit_amp)`

### 位置—速度—电流三闭环（loop3）

控制链：`位置目标 -> 位置P -> 速度目标 -> 速度PI -> Iq目标 -> 电流PI -> SVPWM`

- 启动：串口发送 `position` 或 `loop3`
- 运行中修改位置：发送 `pos=360`，单位机械角度；支持多圈和负角度
- 默认位置：`Foc/bsp/bsp.c` 中 `BSP_FOC_DEFAULT_POSITION_DEG`
- 最大Iq：`BSP_FOC_DEFAULT_IQ_LIMIT_A`
- 封装入口：`BSP_FOC_StartPosition(float target_deg, float iq_limit_amp)`

任何模式都可发送 `stop` 停止。模式切换会先停PWM、清控制器状态、重新对齐，再进入新模式。

## 2. 参数在哪里修改

正常调试只打开 `Foc/foc/foc_user_config.h`。三个闭环的默认目标、PI参数、限幅、位置加减速和软保持参数已经按三块集中排列。底层文件只引用这里的宏，不需要再屏蔽控制代码。

### 电流内环参数（三种模式共用）

用户调参文件：`Foc/foc/foc_user_config.h`

- q/d轴电流PI：`FOC_USER_CURRENT_KP`、`FOC_USER_CURRENT_KI`
- 电流PI输出电压限制：`FOC_USER_CURRENT_VOLT_LIMIT_V`
- 默认力矩电流：`FOC_USER_DEFAULT_TORQUE_IQ_A`

### 速度中环参数（loop2和loop3共用）

用户调参文件：`Foc/foc/foc_user_config.h`

- `FOC_USER_SPEED_KP`
- `FOC_USER_SPEED_KI`
- `FOC_USER_SPEED_FF_A`
- `FOC_USER_SPEED_IQ_SLEW_A_S`
- `FOC_USER_DEFAULT_SPEED_RPM`
- `FOC_USER_DEFAULT_IQ_LIMIT_A`

底层 `foc_motor.c` 通过宏引用这些值；无需修改旧的 `Kp_s/Ki_s`。

### 位置外环参数（loop3）

用户调参文件：`Foc/foc/foc_user_config.h`

- 位置P/PI：`FOC_USER_POSITION_KP`、`FOC_USER_POSITION_KI`
- 默认位置：`FOC_USER_DEFAULT_POSITION_DEG`
- 位置环最大速度：`FOC_USER_POSITION_SPEED_LIMIT_RPM`
- 加减速：`FOC_USER_POSITION_ACCEL_RPM_S`、`FOC_USER_POSITION_DECEL_RPM_S`
- 到位/重新复位窗口：`FOC_USER_POSITION_HOLD_ENTER_DEG`、`FOC_USER_POSITION_HOLD_EXIT_DEG`
- 软保持速度门限：`FOC_USER_POSITION_HOLD_MAX_RPM`

## 3. 按键

- KEY1：启动当前选择模式
- KEY2：停止
- KEY3：目标增加（Iq、rpm或角度）
- KEY4：目标减小
- KEY5：位置、速度、力矩模式循环切换

当前按键和封装入口统一使用已验证配置：phase map 5、encoder direction +1、Iq direction +1、SVPWM、默认Iq限制100 mA。
