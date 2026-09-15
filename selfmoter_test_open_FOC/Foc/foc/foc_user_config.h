#ifndef FOC_USER_CONFIG_H
#define FOC_USER_CONFIG_H

/*
 * ======================== FOC 用户调参总表 ========================
 * 日常调试只需要修改本文件，不需要屏蔽或放开控制代码。
 * 按 KEY5 选择闭环模式，再按 KEY1 启动；KEY2 停止。
 * 也可通过串口发送 torque、speed 或 position 选择并启动对应模式。
 *
 * 闭环 1：Iq 目标 -> 电流 PI -> Uq/Ud -> SVPWM
 * 闭环 2：速度目标 -> 速度 PI -> Iq 目标 -> 电流 PI -> SVPWM
 * 闭环 3：位置目标 -> 位置 P/PI -> 速度目标 -> 速度 PI
 *                   -> Iq 目标 -> 电流 PI -> SVPWM
 * ================================================================ */

/* ---------------- 电机与硬件参数（已通过实机验证） ---------------- */
#define FOC_USER_BUS_VOLTAGE_V            24.0f
#define FOC_USER_POLE_PAIRS                7      /* 电机极对数 */
#define FOC_USER_PHASE_MAP                 5U     /* 三相线序映射 */
#define FOC_USER_ENCODER_DIR               1.0f   /* 编码器方向 */
#define FOC_USER_IQ_DIR                    1.0f   /* 转矩电流方向 */

/* ---------------- 闭环 1：转矩（电流）闭环 ----------------
 * 电磁转矩近似为 Kt × Iq；未标定 Kt 时，目标量使用 A，而不是 N·m。 */
#define FOC_USER_DEFAULT_TORQUE_IQ_A       0.050f   /* 默认 Iq 目标，单位 A */
#define FOC_USER_CURRENT_KP                 0.560f   /* d/q 轴电流 PI 比例系数 */
#define FOC_USER_CURRENT_KI              1020.000f   /* d/q 轴电流 PI 积分系数 */
#define FOC_USER_CURRENT_VOLT_LIMIT_V       3.000f   /* 电流 PI 输出电压限幅，单位 V */
#define FOC_USER_CURRENT_OVER_A             1.600f   /* 软件过流保护阈值，单位 A */

/* ---------------- 闭环 2：速度-电流串级闭环 ----------------
 * DEFAULT_IQ_LIMIT_A 是速度 PI 能给出的最大转矩电流，位置闭环也使用该限幅。 */
#define FOC_USER_DEFAULT_SPEED_RPM        300.0f     /* 默认目标速度，单位 rpm */
#define FOC_USER_DEFAULT_IQ_LIMIT_A         0.100f   /* 最大 Iq，单位 A */
#define FOC_USER_SPEED_KP                   0.00010f /* 速度 PI 比例系数；低增益用于避免速度环饱和振荡 */
#define FOC_USER_SPEED_KI                   0.00005f /* 恢复稳定积分增益；过大的 Ki 会加重低速冲转-停转循环 */
#define FOC_USER_SPEED_FF_A                 0.00000f /* 常值速度前馈，当前关闭，单位 A */
#define FOC_USER_SPEED_START_BOOST_A        0.00000f /* 关闭实验性启动补偿；15mA 对该空载电机已足以造成明显超速 */
#define FOC_USER_SPEED_START_BOOST_END      1.00000f /* 补偿为 0 时该参数不参与实际控制 */
#define FOC_USER_SPEED_IQ_SLEW_A_S          0.250f   /* Iq 指令变化率，单位 A/s */

/* ---------------- 闭环 3：位置-速度-电流串级闭环 ---------------- */
#define FOC_USER_DEFAULT_POSITION_DEG     360.0f   /* 默认目标位置，单位机械角度 */
#define FOC_USER_POSITION_KP                2.000f /* 位置环比例系数，单位 rpm/度 */
#define FOC_USER_POSITION_KI                0.000f /* 位置环积分系数，稳定版为 0 */
#define FOC_USER_POSITION_LIMIT_DEG      3600.0f   /* 位置指令绝对值限幅，单位度 */
#define FOC_USER_POSITION_SPEED_LIMIT_RPM 200.0f   /* 稳定基线：位置环最大速度，单位 rpm */
#define FOC_USER_POSITION_ACCEL_RPM_S     600.0f   /* 稳定基线：复位加速度，单位 rpm/s */
#define FOC_USER_POSITION_DECEL_RPM_S    1200.0f   /* 稳定基线：复位减速度，单位 rpm/s */
#define FOC_USER_POSITION_STATIC_IQ_A       0.000f /* 关闭静摩擦补偿：15mA会在目标两侧反向激励振荡 */
#define FOC_USER_POSITION_STATIC_FADE_RPM  10.0f   /* 补偿为0时该参数不参与控制 */
#define FOC_USER_POSITION_HOLD_ENTER_DEG    1.50f  /* 进入原点保持区的角度误差 */
#define FOC_USER_POSITION_HOLD_EXIT_DEG     3.00f  /* 退出保持区的角度误差（回差） */
#define FOC_USER_POSITION_HOLD_MAX_RPM     10.00f  /* 进入保持区时允许的最大速度 */

/* ---------------- 五向按键每次改变目标值的步长 ---------------- */
#define FOC_USER_KEY_SPEED_STEP_RPM        50.0f   /* 速度步长，单位 rpm */
#define FOC_USER_KEY_POSITION_STEP_DEG     30.0f   /* 位置步长，单位度 */
#define FOC_USER_KEY_TORQUE_STEP_A          0.010f /* 转矩电流步长，单位 A */

#endif
