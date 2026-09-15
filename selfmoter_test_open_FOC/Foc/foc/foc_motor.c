/**
 ****************************************************************************************************
 * @file        foc_motor.c
 * @author      哔哩哔哩-Rebron大侠
 * @version     V0.0
 * @date        2025-01-11
 * @brief       FOC电机状态机
 * @license     MIT License
 *              Copyright (c) 2025 Reborn大侠
 *              允许任何人使用、复制、修改和分发该代码，但需保留此版权声明。
 ****************************************************************************************************
 */

#include "foc_motor.h"
#include "foc_as5047p.h"
#include "foc_hw.h"
#include "foc_user_config.h"
#include <foc_math.h>
#include <stdio.h>
#include <delay.h>
/**
 * @brief 电机结构体初始化
 * 
 * @param foc_motor 
 */
void FOC_Motor_Init(FOC_MOTOR_t *foc_motor)
{
    foc_motor->motor_state = MOTOR_IDLE;
}


/**
 * @brief 清除电机的FOC各项计算参数
 * 
 * @param foc_motor 
 */
static void FOC_Motor_Clear(FOC_MOTOR_t *foc_motor)
{
    abc_t abc_NULL  = {0};
    qd_t qd_NULL    = {0};
    alphabeta_t alphabeta_NULL = {0};

    /*FOC参数清除*/
    foc_motor->foc_var.I_abc = abc_NULL;
    foc_motor->foc_var.I_qd  = qd_NULL;
    foc_motor->foc_var.U_qd  = qd_NULL;
    foc_motor->foc_var.I_alphabeta = alphabeta_NULL;
    
    foc_motor->foc_var.I_qd_ref = qd_NULL;

    /*关闭PWM*/
    FOC_PWM_StopALL(&foc_motor->pwm);
}

/*
 * AS5047P 闭环运行的几个关键周期:
 * - TIM6 每 50us 进入一次 FOC 中断, 也就是 20kHz 控制节拍.
 * - 编码器每 4 个 FOC tick 读取一次, 降低 SPI 读取对实时控制的压力.
 * - 速度环的输出不是电压, 而是 q 轴电流目标 Iq, Iq 主要决定转矩.
 */
#define FOC_AS5047_RUN_TS              0.00005f
#define FOC_AS5047_UPDATE_DIVIDER      20U
#define FOC_AS5047_SPEED_LOOP_TS       0.001f
/* AS5047 串级控制中实际使用的中间速度环：
 * 转速误差[rpm] -> PI -> q轴目标电流[A]。
 * 速度-电流环和位置-速度-电流环的 KP/KI 均在此处整定。 */
#define FOC_AS5047_SPEED_KP            FOC_USER_SPEED_KP
#define FOC_AS5047_SPEED_KI            FOC_USER_SPEED_KI
#define FOC_AS5047_SPEED_FF_A          FOC_USER_SPEED_FF_A
#define FOC_AS5047_SPEED_START_BOOST_A FOC_USER_SPEED_START_BOOST_A
#define FOC_AS5047_SPEED_START_BOOST_END FOC_USER_SPEED_START_BOOST_END
#define FOC_AS5047_SPEED_IQ_SLEW_A_S   FOC_USER_SPEED_IQ_SLEW_A_S
/* 原点软保持采用回差：进入较小误差带后保持零转矩，只有转轴离开较大的
 * 误差带才重新纠偏，避免在目标点两侧反复主动制动。 */
/* 位置运动和稳定参数统一来自 foc_user_config.h，包括加减速度及保持区。 */
#define FOC_POSITION_HOLD_ENTER_DEG    FOC_USER_POSITION_HOLD_ENTER_DEG
#define FOC_POSITION_HOLD_EXIT_DEG     FOC_USER_POSITION_HOLD_EXIT_DEG
#define FOC_POSITION_HOLD_MAX_RPM      FOC_USER_POSITION_HOLD_MAX_RPM
#define FOC_POSITION_ACCEL_RPM_S       FOC_USER_POSITION_ACCEL_RPM_S
#define FOC_POSITION_DECEL_RPM_S       FOC_USER_POSITION_DECEL_RPM_S
#define FOC_POSITION_STATIC_IQ_A       FOC_USER_POSITION_STATIC_IQ_A
#define FOC_POSITION_STATIC_FADE_RPM   FOC_USER_POSITION_STATIC_FADE_RPM
#define FOC_ZERO_CURRENT_NOISE_A       0.040f
#define FOC_AS5047_ALIGN_ID_A          0.08f
#define FOC_AS5047_MAX_CONSEC_ERRORS   3U

static uint32_t g_as5047_run_align_count = 0U;
static uint32_t g_as5047_run_update_div = 0U;
static bool g_as5047_run_aligned = false;
static bool g_position_soft_hold = false;
static float g_align_ia_sum = 0.0f;
static float g_align_ib_sum = 0.0f;
static float g_align_ic_sum = 0.0f;
static float g_align_iq_sum = 0.0f;
static float g_align_id_sum = 0.0f;
static uint32_t g_align_sample_count = 0U;
static float g_align_results[6][5];
static uint8_t g_align_result_valid[6];

void FOC_Motor_PrintAlignResults(void)
{
    uint32_t phase;

    for (phase = 0U; phase < 6U; phase++)
    {
        if (g_align_result_valid[phase] != 0U)
        {
            printf("[ALIGN_RESULT] phase=%lu Iabc_mA=%d/%d/%d Iqd_mA=%d/%d\r\n",
                   (unsigned long)phase,
                   (int)(g_align_results[phase][0] * 1000.0f),
                   (int)(g_align_results[phase][1] * 1000.0f),
                   (int)(g_align_results[phase][2] * 1000.0f),
                   (int)(g_align_results[phase][3] * 1000.0f),
                   (int)(g_align_results[phase][4] * 1000.0f));
        }
        else
        {
            printf("[ALIGN_RESULT] phase=%lu not-tested\r\n",
                   (unsigned long)phase);
        }
    }
}

static void Motor_AS5047P_Reset(void)
{
    g_as5047_run_align_count = 0U;
    g_as5047_run_update_div = 0U;
    g_as5047_run_aligned = false;
    g_position_soft_hold = false;
    g_align_ia_sum = 0.0f;
    g_align_ib_sum = 0.0f;
    g_align_ic_sum = 0.0f;
    g_align_iq_sum = 0.0f;
    g_align_id_sum = 0.0f;
    g_align_sample_count = 0U;
}

/*
 * 启动时先给 d 轴一个固定电压, 把转子吸到已知电角度.
 * 对齐完成后, 把此刻 AS5047P 读到的电角度记为零点偏移.
 * 后面做 Park 变换时减去这个偏移, 才能让 Id/Iq 坐标和真实转子对齐.
 */
static float Motor_AS5047P_Limit(float value, float limit)
{
    if (value > limit)
    {
        return limit;
    }

    if (value < -limit)
    {
        return -limit;
    }

    return value;
}
/*
 * 串口 iq=<mA> 命令和 KEY1 启动预设, 最后都会变成这里的 Iq 限幅.
 * 速度环即使想快速加速, 输出的 Iq 也不能超过这个限幅.
 * 这样可以限制启动转矩和电压需求, 避免一上电就冲得太猛.
 */
static float Motor_AS5047P_GetIqLimit(void)
{
    float limit = FOC_Debug_GetEncoderIqRef();

    if (limit < 0.0f)
    {
        limit = -limit;
    }

    if (limit < 0.02f)
    {
        limit = 0.02f;
    }

    if (limit > 0.80f)
    {
        limit = 0.80f;
    }

    return limit;
}
/*
 * 速度外环: 目标转速 - 实际转速 = 速度误差.
 * PI 控制器输出 q 轴电流目标 Iq, Iq 是主要产生电磁转矩的电流分量.
 * 这里加入了积分抗饱和, 输出已经顶到 iq_limit 时不继续无意义累积积分.
 */
static float Motor_AS5047P_SpeedIqCtrl(FOC_MOTOR_t *foc_motor,
                                       float target_speed,
                                       float speed,
                                       float iq_limit)
{
    float err = target_speed - speed;
    float iq_p = FOC_AS5047_SPEED_KP * err;
    float iq_i = foc_motor->pid_speed.SumError;
    float iq_ff = 0.0f;
    float iq_raw;
    float iq_limited;
    float iq_previous = foc_motor->pid_speed.ActualValue;
    float iq_slew_step = FOC_AS5047_SPEED_IQ_SLEW_A_S *
                          FOC_AS5047_SPEED_LOOP_TS;
    float speed_progress;
    float boost_scale;
    float iq_delta;
    bool sat_high;
    bool sat_low;

    /* 仅使用比例项调试时，电机达到目标转速后仍需少量转矩克服轴承损耗和齿槽转矩。
     * 该随方向变化的平滑前馈无需增加状态机，即可避免零转矩滑行后再次堵转的循环。 */
    if (target_speed > 1.0f)
    {
        iq_ff = FOC_AS5047_SPEED_FF_A;
    }
    else if (target_speed < -1.0f)
    {
        iq_ff = -FOC_AS5047_SPEED_FF_A;
    }

    /*
     * 可选的低速启动补偿。当前配置为 0 A，即完全关闭。
     * 实测 20 mA 已能使空载电机达到约 450 rpm，而 50 rpm 所需电流只有几 mA，
     * 已小于电流采样约 10~20 mA 的波动。此时加入 15 mA 补偿会产生
     * “加速超调 -> 撤掉转矩 -> 停转 -> 再启动”的极限环，并非 PI 参数太小。
     * 所以先在 300 rpm 左右的可测电流区调通速度环，再考虑低速硬件优化。
     */
    if ((FOC_AS5047_SPEED_START_BOOST_A > 0.0f) &&
        (FOC_Debug_GetEncoderMode() == FOC_CONTROL_SPEED) &&
        (fabsf(target_speed) > 1.0f))
    {
        speed_progress = ((target_speed > 0.0f) ? speed : -speed) /
                         fabsf(target_speed);
        speed_progress = Motor_AS5047P_Limit(speed_progress,
                                             FOC_AS5047_SPEED_START_BOOST_END);
        if (speed_progress < 0.0f)
        {
            speed_progress = 0.0f;
        }

        boost_scale = 1.0f -
                      (speed_progress / FOC_AS5047_SPEED_START_BOOST_END);
        iq_ff += ((target_speed > 0.0f) ? 1.0f : -1.0f) *
                 FOC_AS5047_SPEED_START_BOOST_A * boost_scale;
    }

    iq_raw = iq_p + iq_i + iq_ff;
    sat_high = iq_raw > iq_limit;
    sat_low = iq_raw < -iq_limit;

    if ((!sat_high && !sat_low) ||
        (sat_high && (err < 0.0f)) ||
        (sat_low && (err > 0.0f)))
    {
        iq_i += FOC_AS5047_SPEED_KI * err * FOC_AS5047_SPEED_LOOP_TS;
        foc_motor->pid_speed.SumError = Motor_AS5047P_Limit(iq_i, iq_limit);
    }

    /* 速度模式下，积分项只补偿稳态运行损耗，瞬态制动仍由比例项承担；
     * 这样可防止长期存在的反向积分再次引起粘滑振荡。 */
    if (FOC_Debug_GetEncoderMode() == FOC_CONTROL_SPEED)
    {
        if ((target_speed > 0.0f) &&
            (foc_motor->pid_speed.SumError < 0.0f))
        {
            foc_motor->pid_speed.SumError = 0.0f;
        }
        else if ((target_speed < 0.0f) &&
                 (foc_motor->pid_speed.SumError > 0.0f))
        {
            foc_motor->pid_speed.SumError = 0.0f;
        }
    }

    iq_raw = iq_p + foc_motor->pid_speed.SumError + iq_ff;
    iq_limited = Motor_AS5047P_Limit(iq_raw, iq_limit);

    /* 导出速度环各分量，供调试遥测观察。 */
    foc_motor->pid_speed.Up = iq_p;
    foc_motor->pid_speed.Ui = foc_motor->pid_speed.SumError;
    foc_motor->pid_speed.Ud = iq_ff;

    /* 限制低惯量转子的转矩指令反向速率。目标附近的速度采样存在量化和噪声，
     * 若在单个 1 ms 周期内施加全部修正，会产生可感知的轻微抖动。 */
    iq_delta = iq_limited - iq_previous;
    iq_delta = Motor_AS5047P_Limit(iq_delta, iq_slew_step);
    return iq_previous + iq_delta;
}
/**
 * @brief 切换状态机的下个状态
 * 
 * @param foc_motor 
 * @param new_state 
 */
static void Motor_NextState(FOC_MOTOR_t *foc_motor, MOTOR_State_t new_state)
{
    foc_motor->motor_state = new_state;
}

/*电机开环运行*/

/**
 * @brief 直接输出Uq进行开环运行
 * 
 * @param foc_motor 
 */
void Motor_Run_Open(FOC_MOTOR_t *foc_motor)
{
    FOC_VF_t *vf = &foc_motor->sensorless.vf;

    vf->Uqd.q = 1;
    FOC_VF_Angle_Calc(vf);

    float ElAngle = vf->speed.ElAngle;

    
    qd_t In = {0};
    /*更新电流*/
    
    foc_motor->foc_var.I_abc = (foc_motor->current.I_abc);
    /*克拉克变换*/
    foc_motor->foc_var.I_alphabeta  = FOC_Clarke(foc_motor->foc_var.I_abc);

    In = Foc_Park(foc_motor->foc_var.I_alphabeta, ElAngle);

    FOC_PWM_Run(&foc_motor->pwm, vf->Uqd, ElAngle);

    foc_motor->foc_var.I_qd = In;

}


/**
 * @brief 无感方式运行
 * 
 * @param foc_motor 
 */
void Motor_SL_RUN(FOC_MOTOR_t *foc_motor)
{
    FOC_SENSORLESS_t *foc_sl = &foc_motor->sensorless;

    qd_t In = {0};
    qd_t Out = {0};
    abc_t Iabc = {0};
    float ElAngle = 0.0f;
    alphabeta_t I_alphabeta = {0};
    alphabeta_t Last_Ualphabeta = foc_motor->pwm.alpha_beta;

    /*更新电流*/
    Iabc = foc_motor->current.I_abc;
    /*克拉克变换*/
    I_alphabeta  = FOC_Clarke(Iabc);
    
//    FOC_ABZENC_Angle_Calc(&foc_motor->enc);
//    FOC_HALL_Angle_Calc(&foc_motor->hall);
    

    /*进入无感观测*/
    I_alphabeta = FOC_SL_RUN(&foc_motor->sensorless, I_alphabeta, Last_Ualphabeta, foc_motor->target_speed);
    ElAngle = foc_motor->sensorless.speed.ElAngle;

    /*计算两个观测器之间的角度误差*/
    // ElAngle = foc_motor->enc.speed.ElAngle;
    // foc_sl->err = FOC_ERR(Limit_Angle(ElAngle), Limit_Angle(foc_sl->flux.speed.ElAngle));    

    /*帕克变换*/
    In = Foc_Park(I_alphabeta, ElAngle);

    /*速度环控制*/    
    foc_motor->pid_cur_iq.SetPoint = foc_motor->foc_var.I_qd_ref.q;
    foc_motor->pid_cur_id.SetPoint = foc_motor->foc_var.I_qd_ref.d;

    /*电流环PID*/
    Out.q = PID_Position_ctrl(&foc_motor->pid_cur_iq,  In.q);
    Out.d = PID_Position_ctrl(&foc_motor->pid_cur_id,  In.d) + foc_sl->d_bias;

    switch(foc_sl->sta){
        case VF_STA:
            PID_Clear(&foc_motor->pid_cur_iq);
            PID_Clear(&foc_motor->pid_cur_id);
            PID_Clear(&foc_motor->pid_speed);
            PID_Clear(&foc_motor->pid_speed_pos);
            foc_motor->position_speed_ref = 0.0f;
            Out = foc_sl->vf.Uqd;
            break;
        case HFI_POL_STA:
            PID_Clear(&foc_motor->pid_cur_iq);
            PID_Clear(&foc_motor->pid_cur_id);
            PID_Clear(&foc_motor->pid_speed);
            Out.d = foc_sl->d_bias; 
            Out.q = 0;
            break;
        case HFI_STA:

            /*高频注入的时候不要对d轴电流进行PID控制，否则堵转下容易反转 并且要及时清除d轴PID参数，不然跳转到SMO会出问题*/
            PID_Clear(&foc_motor->pid_cur_id);
            Out.d = foc_sl->d_bias;      

            if(foc_motor->pid_cur_iq.ActualValue_limit != foc_motor->pid_qd_limit_hif){
                /*清除积分，设置高频注入q轴电流限制*/
                PID_Clear(&foc_motor->pid_cur_iq);
                foc_motor->pid_cur_iq.ActualValue_limit = foc_motor->pid_qd_limit_hif;         /*高频注入状态 需要限制电流环*/ 
            }
            
            break;
        case SMO_STA:
            foc_motor->pid_cur_iq.ActualValue_limit = foc_motor->pid_qd_limit;
            break;
        case FLUX_STA:
            foc_motor->pid_cur_iq.ActualValue_limit = foc_motor->pid_qd_limit;
            break;
    }

    // Out = foc_sl->vf.Uqd;
    // Out.q = foc_sl->vf.Uqd.q;
    /* 反 Park 加 PWM 调制: 把 Ud/Uq 转成三相桥臂 U/V/W 的 PWM 输出. */
    FOC_PWM_Run(&foc_motor->pwm, Out, ElAngle);

    /*计算三相反电动势*/

    /*保存参数*/
    foc_motor->foc_var.speed = foc_motor->sensorless.speed;
    foc_motor->foc_var.I_abc = Iabc;
    foc_motor->foc_var.U_qd = foc_motor->pwm.Uqd;
    foc_motor->foc_var.I_qd = In;
    foc_motor->foc_var.I_alphabeta = I_alphabeta;
}

/**
 * @brief ABZ编码器方式运行
 * 
 * @param foc_motor 
 */
void Motor_ABZ_ENC_RUN(FOC_MOTOR_t *foc_motor)
{
    qd_t In = {0};
    qd_t Out = {0};
    abc_t Iabc = {0};
    float ElAngle = 0.0f;
    alphabeta_t I_alphabeta = {0};

    /*更新电流*/
    Iabc = foc_motor->current.I_abc;
    /*克拉克变换*/
    I_alphabeta  = FOC_Clarke(Iabc);
             
//    ElAngle = FOC_ABZENC_Angle_Calc(&foc_motor->enc);

    I_alphabeta = FOC_SL_RUN(&foc_motor->sensorless, I_alphabeta, foc_motor->pwm.alpha_beta, foc_motor->target_speed);

    /*帕克变换*/
    In = Foc_Park(I_alphabeta, ElAngle);

    /*电流环PID*/
    foc_motor->pid_cur_iq.SetPoint = foc_motor->foc_var.I_qd_ref.q;
    foc_motor->pid_cur_id.SetPoint = foc_motor->foc_var.I_qd_ref.d;
    Out.q = PID_Position_ctrl(&foc_motor->pid_cur_iq,  In.q);
    Out.d = PID_Position_ctrl(&foc_motor->pid_cur_id,  In.d);

    /* 反 Park 加 PWM 调制: 把 Ud/Uq 转成三相桥臂 U/V/W 的 PWM 输出. */
    FOC_PWM_Run(&foc_motor->pwm, Out, ElAngle);

    /*保存参数*/
    foc_motor->foc_var.speed = foc_motor->enc.speed;
    foc_motor->foc_var.I_abc = Iabc;
    foc_motor->foc_var.U_qd = foc_motor->pwm.Uqd;
    foc_motor->foc_var.I_qd = In;
    foc_motor->foc_var.I_alphabeta = I_alphabeta;
}

/**
 * @brief AS5047P 编码器闭环运行入口
 *
 * 第一阶段: 只给 d 轴电压, 把转子拉到电角度 0 附近, 然后记录 AS5047P 当前电角度为零点.
 * 第二阶段: 使用 AS5047P 的电角度做 Park/反 Park 变换, 闭合 Id/Iq 电流环.
 * 速度模式下, 外层速度 PI 会先算出 Iq 目标, 再交给这里的电流内环执行.
 *
 * @param foc_motor 电机控制结构体
 */
void Motor_AS5047P_RUN(FOC_MOTOR_t *foc_motor)
{
    qd_t In = {0};
    qd_t ControlIn = {0};
    qd_t Out = {0};
    abc_t Iabc = {0};
    float ElAngle = 0.0f;
    alphabeta_t I_alphabeta = {0};
    uint32_t align_ticks = foc_motor->sensorless.vf.align_ticks;

    if (align_ticks == 0U)
    {
        align_ticks = 5000U;
    }

    Iabc = foc_motor->current.I_abc;
    I_alphabeta = FOC_Clarke(Iabc);

    if (!g_as5047_run_aligned)
    {
        qd_t AlignOut = {0};

        /* 对齐阶段只施加 d 轴电压, q 轴为 0, 目的是定位转子而不是输出转矩. */
        In = Foc_Park(I_alphabeta, 0.0f);

        /* 在固定定子角度闭合 d 轴电流环；原先固定的 0.30 V 电压矢量过弱，
         * 无法保证转子可靠对齐。 */
        foc_motor->pid_cur_iq.SetPoint = 0.0f;
        foc_motor->pid_cur_id.SetPoint = FOC_AS5047_ALIGN_ID_A;
        foc_motor->pid_cur_iq.ActualValue_limit = foc_motor->pid_qd_limit;
        foc_motor->pid_cur_id.ActualValue_limit = foc_motor->pid_qd_limit;
        AlignOut.q = PID_Position_ctrl(&foc_motor->pid_cur_iq, In.q);
        AlignOut.d = PID_Position_ctrl(&foc_motor->pid_cur_id, In.d);

        if (FOC_CURRENT_Over(&foc_motor->current, In))
        {
            printf("[FOC_FAULT] align overcurrent Iqd=%d/%dmA -> stop\r\n",
                   (int)(In.q * 1000.0f),
                   (int)(In.d * 1000.0f));
            FOC_PWM_StopALL(&foc_motor->pwm);
            Motor_AS5047P_Reset();
            FOC_Debug_SetRunRequest(false);
            return;
        }

        FOC_PWM_Run(&foc_motor->pwm, AlignOut, 0.0f);

        if (g_as5047_run_align_count >= (align_ticks / 2U))
        {
            g_align_ia_sum += Iabc.a;
            g_align_ib_sum += Iabc.b;
            g_align_ic_sum += Iabc.c;
            g_align_iq_sum += In.q;
            g_align_id_sum += In.d;
            g_align_sample_count++;
        }

        if (g_as5047_run_align_count < align_ticks)
        {
            g_as5047_run_align_count++;
        }

        foc_motor->sensorless.vf.align_count = g_as5047_run_align_count;
        foc_motor->foc_var.speed.ElAngle = 0.0f;
        foc_motor->foc_var.speed.AvrMecSpeed = 0.0f;
        foc_motor->foc_var.I_abc = Iabc;
        foc_motor->foc_var.U_qd = foc_motor->pwm.Uqd;
        foc_motor->foc_var.I_qd = In;
        foc_motor->foc_var.I_alphabeta = I_alphabeta;

        if (g_as5047_run_align_count >= align_ticks)
        {
            if (g_align_sample_count > 0U)
            {
                float inv_count = 1.0f / (float)g_align_sample_count;
                uint32_t phase = FOC_Debug_GetPhaseMap() % 6U;

                g_align_results[phase][0] = g_align_ia_sum * inv_count;
                g_align_results[phase][1] = g_align_ib_sum * inv_count;
                g_align_results[phase][2] = g_align_ic_sum * inv_count;
                g_align_results[phase][3] = g_align_iq_sum * inv_count;
                g_align_results[phase][4] = g_align_id_sum * inv_count;
                g_align_result_valid[phase] = 1U;

                printf("[ALIGN_DIAG] phase=%lu Iabc_mA=%d/%d/%d Iqd_mA=%d/%d samples=%lu\r\n",
                       (unsigned long)FOC_Debug_GetPhaseMap(),
                       (int)(g_align_ia_sum * inv_count * 1000.0f),
                       (int)(g_align_ib_sum * inv_count * 1000.0f),
                       (int)(g_align_ic_sum * inv_count * 1000.0f),
                       (int)(g_align_iq_sum * inv_count * 1000.0f),
                       (int)(g_align_id_sum * inv_count * 1000.0f),
                       (unsigned long)g_align_sample_count);
            }

            FOC_AS5047P_SetZeroCurrent();
            PID_Clear(&foc_motor->pid_cur_iq);
            PID_Clear(&foc_motor->pid_cur_id);
            PID_Clear(&foc_motor->pid_speed);
            PID_Clear(&foc_motor->pid_speed_pos);
            foc_motor->position_speed_ref = 0.0f;
            g_as5047_run_aligned = true;
            foc_motor->sensorless.vf.align_count = align_ticks;
            printf("[FOC_AS5047] aligned raw=%u alignIq=%dmA alignId=%dmA runIq=%dmA runId=%dmA\r\n",
                   (unsigned int)FOC_AS5047P_GetState()->raw,
                   (int)(In.q * 1000.0f),
                   (int)(In.d * 1000.0f),
                   (int)(FOC_Debug_GetEncoderIqRef() * 1000.0f),
                   (int)(FOC_Debug_GetEncoderIdRef() * 1000.0f));

            if (FOC_Debug_GetAlignOnly())
            {
                printf("[ALIGN_DIAG] complete -> automatic stop\r\n");
                FOC_Debug_SetRunRequest(false);
                return;
            }
        }

        return;
    }

    if (g_as5047_run_update_div == 0U)
    {
        bool encoder_ok = FOC_AS5047P_Update(
            FOC_AS5047_RUN_TS * (float)FOC_AS5047_UPDATE_DIVIDER);
        g_as5047_run_update_div = FOC_AS5047_UPDATE_DIVIDER - 1U;

        if ((!encoder_ok) &&
            (FOC_AS5047P_GetState()->consecutive_error_count >=
             FOC_AS5047_MAX_CONSEC_ERRORS))
        {
            printf("[FOC_FAULT] AS5047 invalid frames=%lu -> stop\r\n",
                   (unsigned long)FOC_AS5047P_GetState()->consecutive_error_count);
            FOC_PWM_StopALL(&foc_motor->pwm);
            Motor_AS5047P_Reset();
            FOC_Debug_SetRunRequest(false);
            return;
        }
    }
    else
    {
        g_as5047_run_update_div--;
    }

    /* 编码器电角度用于 Park 变换, 把 Ialpha/Ibeta 转到随转子旋转的 Id/Iq 坐标系. */
    ElAngle = FOC_AS5047P_GetElectricalAngle();
    In = Foc_Park(I_alphabeta, ElAngle);

    if ((FOC_Debug_GetEncoderMode() == FOC_CONTROL_SPEED) ||
        (FOC_Debug_GetEncoderMode() == FOC_CONTROL_POSITION))
    {
        foc_motor->foc_var.I_qd_ref.q =
            Motor_AS5047P_Limit(foc_motor->foc_var.I_qd_ref.q,
                                Motor_AS5047P_GetIqLimit());
    }
    else
    {
        foc_motor->foc_var.I_qd_ref.q = FOC_Debug_GetEncoderIqRef();
    }

    foc_motor->foc_var.I_qd_ref.d = FOC_Debug_GetEncoderIdRef();

    foc_motor->pid_cur_iq.SetPoint = foc_motor->foc_var.I_qd_ref.q;
    foc_motor->pid_cur_id.SetPoint = foc_motor->foc_var.I_qd_ref.d;
    foc_motor->pid_cur_iq.ActualValue_limit = foc_motor->pid_qd_limit;
    foc_motor->pid_cur_id.ActualValue_limit = foc_motor->pid_qd_limit;

    ControlIn = In;
    if ((fabsf(foc_motor->foc_var.I_qd_ref.q) < 0.001f) &&
        (fabsf(ControlIn.q) < FOC_ZERO_CURRENT_NOISE_A))
    {
        ControlIn.q = 0.0f;
    }
    if ((fabsf(foc_motor->foc_var.I_qd_ref.d) < 0.001f) &&
        (fabsf(ControlIn.d) < FOC_ZERO_CURRENT_NOISE_A))
    {
        ControlIn.d = 0.0f;
    }

    /* 电流内环: 根据 Id/Iq 误差计算 Ud/Uq 电压指令. */
    /* 所有模式共用的电流内环：Id/Iq误差 -> d/q轴PI -> Ud/Uq；
     * 下方通过反Park变换和SVPWM将Ud/Uq转换为三相PWM。 */
    Out.q = PID_Position_ctrl(&foc_motor->pid_cur_iq, ControlIn.q);
    Out.d = PID_Position_ctrl(&foc_motor->pid_cur_id, ControlIn.d);

    if (FOC_CURRENT_Over(&foc_motor->current, In))
    {
        printf("[FOC_AS5047] overcurrent Iqd=%d/%dmA -> stop\r\n",
               (int)(In.q * 1000.0f),
               (int)(In.d * 1000.0f));
        Motor_AS5047P_Reset();
        FOC_Debug_SetRunRequest(false);
        return;
    }

    /* 反 Park 加 PWM 调制: 把 Ud/Uq 转成三相桥臂 U/V/W 的 PWM 输出. */
    FOC_PWM_Run(&foc_motor->pwm, Out, ElAngle);

    foc_motor->foc_var.speed.ElAngle = ElAngle;
    foc_motor->foc_var.speed.MecAngle = FOC_AS5047P_GetMechanicalAngle();
    foc_motor->foc_var.speed.AvrMecSpeed = FOC_AS5047P_GetSpeedRpm();
    foc_motor->foc_var.speed.pole_pairs = foc_motor->pole_pairs;
    foc_motor->foc_var.speed.dir = (FOC_AS5047P_GetSpeedRpm() < 0.0f) ? REVERSAL : COROTATION;
    foc_motor->foc_var.I_abc = Iabc;
    foc_motor->foc_var.U_qd = foc_motor->pwm.Uqd;
    foc_motor->foc_var.I_qd = In;
    foc_motor->foc_var.I_alphabeta = I_alphabeta;
}
/**
 * @brief ABZ编码器校准
 * 
 * @param foc_motor 
 */
bool Motor_ABZ_ENC_CALIB(FOC_MOTOR_t *foc_motor)
{
    qd_t Out = {0, 0.5f};

    bool ret = false;
    float ElAngle = 0.0f;

    FOC_SENSORLESS_t *foc_sl = &foc_motor->sensorless;
    FOC_ABZENC_t *enc = &foc_motor->enc;
    foc_sl->sta = VF_STA;       /*设置为无感的VF运行*/

    foc_sl->vf.Uqd.q = 0;
    foc_sl->vf.Uqd.d = Out.d/20.0f;

    FOC_VF_Angle_Calc(&foc_sl->vf);
    ElAngle = foc_sl->vf.speed.ElAngle;

    switch (foc_motor->abzenc_calib_state){
        case 0:                 /*强拖定位*/
            if(enc->align){
                foc_motor->abzenc_calib_state = 1;
            }
            break;
        case 1:
            /* code */
            if(fabsf(ElAngle)<(foc_sl->vf.step*2)){
                foc_motor->abzenc_calib_state = 2;
            }
            break;
        case 2:
            ElAngle = 0;
            break;
        default:
            foc_sl->sta = FLUX_STA;
            foc_motor->abzenc_calib_state = 0;
            break;
    }


    /* 反 Park 加 PWM 调制: 把 Ud/Uq 转成三相桥臂 U/V/W 的 PWM 输出. */
    FOC_PWM_Run(&foc_motor->pwm, Out, ElAngle);

//    ret = FOC_ABZENC_Angle_Clib(enc);
    if(true==ret){
        foc_sl->sta = FLUX_STA;
        foc_motor->abzenc_calib_state = 0;
    }

    return ret;
}

/**
 * @brief 霍尔传感器方式运行
 * 
 * @param foc_motor 
 */
void Motor_HALL_RUN(FOC_MOTOR_t *foc_motor)
{
    qd_t In = {0};
    qd_t Out = {0};
    abc_t Iabc = {0};
    float ElAngle = 0.0f;
    alphabeta_t I_alphabeta = {0};

    /*更新电流*/
    Iabc = foc_motor->current.I_abc;
    /*克拉克变换*/
    I_alphabeta  = FOC_Clarke(Iabc);
             
//    ElAngle = FOC_HALL_Angle_Calc(&foc_motor->hall);

    /*帕克变换*/
    In = Foc_Park(I_alphabeta, ElAngle);

    /*电流环PID*/
    foc_motor->pid_cur_iq.SetPoint = foc_motor->foc_var.I_qd_ref.q;
    foc_motor->pid_cur_id.SetPoint = foc_motor->foc_var.I_qd_ref.d;
    Out.q = PID_Position_ctrl(&foc_motor->pid_cur_iq,  In.q);
    Out.d = PID_Position_ctrl(&foc_motor->pid_cur_id,  In.d);

    /* 反 Park 加 PWM 调制: 把 Ud/Uq 转成三相桥臂 U/V/W 的 PWM 输出. */
    FOC_PWM_Run(&foc_motor->pwm, Out, ElAngle);
    
    /*保存参数*/
    foc_motor->foc_var.speed = foc_motor->hall.speed;
    foc_motor->foc_var.I_abc = Iabc;
    foc_motor->foc_var.U_qd = foc_motor->pwm.Uqd;
    foc_motor->foc_var.I_qd = In;
    foc_motor->foc_var.I_alphabeta = I_alphabeta;
}


/**
 * @brief 霍尔自学习运行
 * 
 */
bool Motor_HALL_LEARN(FOC_MOTOR_t *foc_motor)
{
    float err = 0;
    FOC_HALL_t *foc_hall = &foc_motor->hall;
    FOC_FLUX_t *foc_flux = &foc_motor->sensorless.flux;
    foc_motor->sensorless.sta = FLUX_STA;                                           /*使用磁链的方式自己自学习运转*/

    foc_hall->Uq = foc_motor->pwm.Uqd.q;
//    FOC_HALL_Angle_Calc(foc_hall);

    /*使用无感匀速跑*/
    Motor_SL_RUN(foc_motor);

    // return false;
    switch(foc_hall->learning_state){
        case 0:
            /*准备正转学习设置*/
            foc_hall->selflearn = true;                 /*自学习开启*/
            foc_motor->target_speed = 1000;             /*设置无感运行速度*/
            foc_hall->learning_tick = 20000*10;         /*设置正转运行tick，大概十秒*/
            foc_hall->learning_state = 1;

            foc_hall->PhaseShift = 0;
            foc_hall->PhaseShift_c = 0;

            for(int i=0;i<6;i++){
                foc_hall->sector_size[i]=0;
                foc_hall->sector_size_c[i]=0;
                foc_hall->sector_pos[i]=0;
                foc_hall->sector_pos_c[i]=0;
            }

            printf("[HALL_LEARN] forward test\r\n");
            break;
        case 1:
            /*正转测试中*/

            foc_hall->learning_tick--;
            if(foc_hall->learning_tick<1000){
                foc_hall->selflearn = false;
            }

            if(foc_hall->learning_tick<=0){
                /*保存正转参数 准备反转自学习*/
                for(int i=0;i<6;i++){
                    foc_hall->hall_learn.sector_pos[i] = foc_hall->hall_sector_pos_debug[i] *1000000;
                    foc_hall->hall_learn.sector_size[i] = foc_hall->hall_sector_size_debug[i] *1000000;

                    foc_hall->sector_pos[i] = foc_hall->hall_sector_pos_debug[i];
                    foc_hall->sector_size[i] = foc_hall->hall_sector_size_debug[i];

                    Move_Filter_Clear(&foc_hall->hall_sector_tick_filter[i]);
                }

                foc_hall->selflearn = true;                 /*自学习开启*/
                foc_hall->learning_tick = 20000*10;         /*设置正转运行tick，大概十秒*/
                foc_hall->learning_state = 2;

                Move_Filter_Clear(&foc_hall->pos_filter);

                
                for(int i=0;i<6;i++){
                    printf("正转 扇区[%d] 位置:%f  大小:%f\r\n", i, foc_hall->hall_sector_pos_debug[i], foc_hall->hall_sector_size_debug[i]);
                }
                
            }
            break;
        case 2:
            /*计算正转方向与0°的偏移值*/
            foc_hall->learning_tick--;
            err = FOC_ERR(Limit_Angle(foc_flux->speed.ElAngle), Limit_Angle(foc_hall->speed.ElAngle));

            Move_Filter_fill(&foc_hall->pos_filter, 100000*err);
            if(foc_hall->learning_tick<=0){
                foc_hall->hall_learn.PhaseShift[0] = Move_Filter_calculate(&foc_hall->pos_filter);
                foc_hall->PhaseShift = (float)foc_hall->hall_learn.PhaseShift[0] /100000.0f;
                
                foc_hall->learning_tick = 20000*10;         /*设置正转运行tick，大概十秒*/
                foc_motor->target_speed = -1000; 
                foc_hall->learning_state = 3;
                
                printf("[HALL_LEARN] forward phase shift: %f\r\n", foc_hall->PhaseShift);

                printf("[HALL_LEARN] reverse test\r\n");
            }
            break;
        case 3:
            /*反转测试中*/
            foc_hall->learning_tick--;
            if(foc_hall->learning_tick<1000){
                foc_hall->selflearn = false;
            }

            if(foc_hall->learning_tick<=0){
                for(int i=0;i<6;i++){
                    foc_hall->hall_learn.sector_pos_c[i] = foc_hall->hall_sector_pos_debug[i] *1000000;
                    foc_hall->hall_learn.sector_size_c[i] = foc_hall->hall_sector_size_debug[i] *1000000;


                    foc_hall->sector_pos_c[i] = foc_hall->hall_sector_pos_debug[i];
                    foc_hall->sector_size_c[i] = foc_hall->hall_sector_size_debug[i];

                    Move_Filter_Clear(&foc_hall->hall_sector_tick_filter[i]);
                }
    
                foc_hall->selflearn = true;                 /*自学习开启*/
                foc_hall->learning_tick = 20000*10;         /*设置反转运行tick，大概十秒*/
                foc_hall->learning_state = 4;

                Move_Filter_Clear(&foc_hall->pos_filter);
                for(int i=0;i<6;i++){
                    printf("反转 扇区[%d] 位置:%f  大小:%f\r\n", i, foc_hall->hall_sector_pos_debug[i], foc_hall->hall_sector_size_debug[i]);
                }
            }
            
            break;
        case 4: /*保存反转参数 准备反转自学习*/
            /*计算反转方向与0°的偏移值*/
            foc_hall->learning_tick--;

            err = FOC_ERR(Limit_Angle(foc_flux->speed.ElAngle), Limit_Angle(foc_hall->speed.ElAngle));

            Move_Filter_fill(&foc_hall->pos_filter, 100000*err);
            if(foc_hall->learning_tick<=0){
                foc_hall->hall_learn.PhaseShift[1] = Move_Filter_calculate(&foc_hall->pos_filter);
                foc_hall->PhaseShift_c = (float)foc_hall->hall_learn.PhaseShift[1] /100000.0f;
            
                foc_motor->target_speed = 0; 
                foc_hall->learning_state = 5;

                printf("[HALL_LEARN] reverse phase shift: %f\r\n", foc_hall->PhaseShift_c);
            }

            break;
        case 5:
            
            foc_hall->selflearn = false;
            // stmflash_write(HALL_ADDR, (uint32_t*)&foc_hall->hall_learn, sizeof(foc_hall->hall_learn)/sizeof(uint32_t));
            foc_motor->target_speed = 0;
            foc_hall->learning_state = 0;

            for(int i=0;i<6;i++){
                Move_Filter_Clear(&foc_hall->hall_sector_tick_filter[i]);
            }

            Move_Filter_Clear(&foc_hall->Period_filter);
            Move_Filter_Clear(&foc_hall->Speed_filter);

            printf("[HALL_LEARN] complete\r\n");
            return true;
        
        default:
            foc_hall->selflearn = false;
            break;
    }

    return false;
}



/**
 * @brief 运行电机状态机
 * 
 * @param foc_motor 
 */
void FOC_Motor_Run(FOC_MOTOR_t *foc_motor)
{
	MOTOR_State_t state = foc_motor->motor_state;

    switch(state){
        case MOTOR_IDLE:  /*空闲状态*/
            FOC_PWM_StopALL(&foc_motor->pwm);                           /*关闭所有PWM通道*/
            Motor_NextState(foc_motor, MOTOR_IDLE_START);               /*跳转到空闲启动*/
            printf("IDLE\r\n");    
            break;
        case MOTOR_IDLE_START:
        {
            int offset_result = FOC_CURRENT_OffsetCalc(&foc_motor->current);
            if(offset_result == 0){
                FOC_PWM_StartAll(&foc_motor->pwm);                      /*打开所有PWM通道*/
                Motor_NextState(foc_motor, MOTOR_CLEAR);        
                printf("IDLE_START\r\n"); 
            } else if(offset_result == -2) {
                FOC_PWM_StopALL(&foc_motor->pwm);
                printf("[FOC_FAULT] current calibration failed -> stop\r\n");
                FOC_Debug_SetRunRequest(false);
            }
            break;
        }
        case MOTOR_IDENTIFY:   
                /*参数辨识做的不好, 暂时不用*/

            break;      
        case MOTOR_CLEAR:
            FOC_Motor_Clear(foc_motor);

            FOC_PWM_StartAll(&foc_motor->pwm);                                  /*打开所有PWM通道*/
                            
            Motor_NextState(foc_motor, MOTOR_START);                            /*跳转到启动*/
            printf("CLEAR\r\n"); 
            break;

        case MOTOR_START:
            if (FOC_Debug_GetEncoderMode() != 0U)
            {
                Motor_AS5047P_Reset();
                Motor_NextState(foc_motor, MOTOR_AS5047_RUN);
                printf("START_AS5047\r\n");
            }
            else
            {
                Motor_NextState(foc_motor, MOTOR_SL_RUN);                           /*跳转到开始启动*/
                printf("START\r\n");
            }
            break;
        case MOTOR_OPEN_RUN:
            Motor_Run_Open(foc_motor);
            break;
        case MOTOR_AS5047_RUN:
            Motor_AS5047P_RUN(foc_motor);
            break;
        case MOTOR_SL_RUN:
            Motor_SL_RUN(foc_motor);
            break;  
        case MOTOR_ENC_RUN:
            /*ABZ型编码器，需要先执行开环，将ABZ电角度对齐*/
            if(foc_motor->enc.align){
                Motor_ABZ_ENC_RUN(foc_motor);
            }else{
                Motor_Run_Open(foc_motor);
            }
            break;
        
        case MOTOR_ABZENC_CALIB:
            if(true==Motor_ABZ_ENC_CALIB(foc_motor)){
                Motor_NextState(foc_motor, MOTOR_ENC_RUN);
            }
            break;
        case MOTOR_HALL_RUN:
            /*霍尔传感器运行*/
            Motor_HALL_RUN(foc_motor);
            break;
        case MOTOR_HALL_LEARN:
            /*霍尔自学习*/
            if(true==Motor_HALL_LEARN(foc_motor)){
                Motor_NextState(foc_motor, MOTOR_SL_RUN);
            }
            break;
        default:
            break;
    }
    
}

/**
 * @brief 电机速度计算
 * 
 * @param foc_motor 
 * @param Ts 该函数的调用周期: 单位s
 */
void FOC_Motor_Speed_Calc(FOC_MOTOR_t *foc_motor, float Ts)
{
    if (FOC_Debug_GetEncoderMode() != 0U)
    {
        return;
    }

    FOC_Sensorless_Speed_Cale(&foc_motor->sensorless, Ts);                   /*无感速度计算*/
//    FOC_ABZENC_Speed_Calc_M(&foc_motor->enc, Ts);                            /*编码器M法速度计算*/

//    FOC_Hall_Speed_Calc(&foc_motor->hall);                                   /*霍尔传感器计算速度*/
}

/**
 * @brief 速度环控制，返回电流环的设定值
 * 
 * @param foc_motor 
 */
void FOC_Motor_Speed_Ctrl(FOC_MOTOR_t *foc_motor)
{
    uint32_t control_mode = FOC_Debug_GetEncoderMode();

#if FOC_ENABLE_POSITION_MODE
    if (control_mode == FOC_CONTROL_POSITION)
    {
        float iq_limit = Motor_AS5047P_GetIqLimit();
        float position_deg = FOC_AS5047P_GetPositionDeg();
        float speed = foc_motor->foc_var.speed.AvrMecSpeed;
        float position_error = foc_motor->target_position_deg - position_deg;
        float raw_speed_ref;
        float speed_ref_delta;
        float slew_step;
        float speed_ref;
        float iq_cmd;
        float static_iq_scale;
        float static_iq;

        /* 已进入原点软保持后，只要仍在较宽的退出误差带内就保持零转矩。 */
        if (g_position_soft_hold)
        {
            if (fabsf(position_error) <= FOC_POSITION_HOLD_EXIT_DEG)
            {
                PID_Clear(&foc_motor->pid_speed_pos);
                PID_Clear(&foc_motor->pid_speed);
                PID_Clear(&foc_motor->pid_cur_iq);
                PID_Clear(&foc_motor->pid_cur_id);
                foc_motor->foc_var.I_qd_ref.q = 0.0f;
                foc_motor->foc_var.I_qd_ref.d = 0.0f;
                foc_motor->position_speed_ref = 0.0f;
                return;
            }

            g_position_soft_hold = false;
        }

        /* 转子高速越过目标点时不能立即撤掉转矩。只有位置误差和速度都足够小
         * 才进入零转矩保持，否则剩余动能会造成较大过冲。 */
        if ((fabsf(position_error) <= FOC_POSITION_HOLD_ENTER_DEG) &&
            (fabsf(speed) <= FOC_POSITION_HOLD_MAX_RPM))
        {
            g_position_soft_hold = true;
            PID_Clear(&foc_motor->pid_speed_pos);
            PID_Clear(&foc_motor->pid_speed);
            PID_Clear(&foc_motor->pid_cur_iq);
            PID_Clear(&foc_motor->pid_cur_id);
            foc_motor->foc_var.I_qd_ref.q = 0.0f;
            foc_motor->foc_var.I_qd_ref.d = 0.0f;
            foc_motor->position_speed_ref = 0.0f;
            return;
        }

        /* 位置外环：位置误差 -> 速度目标。 */
        /* 位置外环：位置误差[度] -> 速度给定[rpm]。 */
        foc_motor->pid_speed_pos.SetPoint = foc_motor->target_position_deg;
        raw_speed_ref = PID_Position_ctrl(&foc_motor->pid_speed_pos, position_deg);

        /* 越过目标点后所需运动方向会反转。若仍按常规斜坡沿用原符号速度给定，转子会继续
         * 远离目标；因此先将速度给定直接置零并清除速度积分，让比例项立即产生制动转矩。
         * 100 mA电流限幅仍作为最终的转矩和加速度保护。 */
        if ((raw_speed_ref * foc_motor->position_speed_ref) < 0.0f)
        {
            foc_motor->position_speed_ref = 0.0f;
            PID_Clear(&foc_motor->pid_speed);
        }

        speed_ref_delta = raw_speed_ref - foc_motor->position_speed_ref;
        slew_step = FOC_POSITION_ACCEL_RPM_S * FOC_AS5047_SPEED_LOOP_TS;
        if (fabsf(raw_speed_ref) < fabsf(foc_motor->position_speed_ref))
        {
            slew_step = FOC_POSITION_DECEL_RPM_S * FOC_AS5047_SPEED_LOOP_TS;
        }
        speed_ref_delta = Motor_AS5047P_Limit(speed_ref_delta, slew_step);
        speed_ref = foc_motor->position_speed_ref + speed_ref_delta;

        foc_motor->position_speed_ref = speed_ref;

        /* 速度中环：速度误差 -> Iq目标。 */
        /* 速度中环：速度误差[rpm] -> PI -> Iq给定[A]。 */
        iq_cmd = Motor_AS5047P_SpeedIqCtrl(foc_motor,
                                          speed_ref,
                                          speed,
                                          iq_limit) *
                 FOC_Debug_GetEncoderSpeedIqDir();

        /* 低速静摩擦补偿：实测位置误差仍有16度时，速度环只给出约6mA，
         * 小于电机克服静摩擦所需电流，因而出现“有速度目标但转子不动”。
         * 这里只在位置误差仍超出保持区、且实际转速低于10rpm时补充最多15mA：
         * - 停转时补偿最大，帮助转子起步；
         * - 随转速升高线性退出，避免改变正常运动段的速度PI；
         * - 方向始终指向位置目标，最终仍受100mA总Iq限幅保护。
         */
        if ((fabsf(position_error) > FOC_POSITION_HOLD_ENTER_DEG) &&
            (FOC_POSITION_STATIC_FADE_RPM > 0.0f))
        {
            static_iq_scale = 1.0f -
                (fabsf(speed) / FOC_POSITION_STATIC_FADE_RPM);
            static_iq_scale = Motor_AS5047P_Limit(static_iq_scale, 1.0f);
            if (static_iq_scale < 0.0f)
            {
                static_iq_scale = 0.0f;
            }

            static_iq = ((position_error >= 0.0f) ? 1.0f : -1.0f) *
                        FOC_POSITION_STATIC_IQ_A * static_iq_scale *
                        FOC_Debug_GetEncoderSpeedIqDir();
            iq_cmd = Motor_AS5047P_Limit(iq_cmd + static_iq, iq_limit);
        }

        foc_motor->pid_speed.SetPoint = speed_ref;
        foc_motor->pid_speed.ActualValue_limit = iq_limit;
        foc_motor->pid_speed.ActualValue = iq_cmd;
        foc_motor->foc_var.I_qd_ref.q = Motor_AS5047P_Limit(iq_cmd, iq_limit);
        foc_motor->foc_var.I_qd_ref.d = FOC_Debug_GetEncoderIdRef();
        return;
    }
#endif

#if FOC_ENABLE_SPEED_MODE
    if (control_mode == FOC_CONTROL_SPEED)
    {
        /* speed模式：速度环先计算Iq给定，再由电流环闭环跟踪。 */
        /* 速度-电流串级：目标转速 -> 速度PI -> Iq；公共电流PI再将Iq误差转换为Uq/SVPWM。 */
        float iq_limit = Motor_AS5047P_GetIqLimit();
        float speed = foc_motor->foc_var.speed.AvrMecSpeed;
        float iq_cmd = Motor_AS5047P_SpeedIqCtrl(foc_motor,
                                                 foc_motor->target_speed,
                                                 speed,
                                                 iq_limit) *
                       FOC_Debug_GetEncoderSpeedIqDir();

        foc_motor->pid_speed.SetPoint = foc_motor->target_speed;
        foc_motor->pid_speed.ActualValue_limit = iq_limit;
        foc_motor->pid_speed.ActualValue = iq_cmd;

        foc_motor->foc_var.I_qd_ref.q = Motor_AS5047P_Limit(iq_cmd, iq_limit);
        foc_motor->foc_var.I_qd_ref.d = FOC_Debug_GetEncoderIdRef();
        return;
    }
#endif

#if FOC_ENABLE_TORQUE_MODE
    if (control_mode == FOC_CONTROL_TORQUE)
    {
        /* enc/转矩模式：使用固定Iq，便于检查编码器角度和电流环。 */
        /* 转矩/电流环：直接使用用户给定的带符号Iq，旁路位置环和速度环，仅闭合电流内环。 */
        foc_motor->foc_var.I_qd_ref.q = FOC_Debug_GetEncoderIqRef();
        foc_motor->foc_var.I_qd_ref.d = FOC_Debug_GetEncoderIdRef();
        PID_Clear(&foc_motor->pid_speed);
        return;
    }
#endif

#if FOC_ENABLE_VF_MODE
    if (control_mode == FOC_CONTROL_VF)
    {
        float speed = foc_motor->foc_var.speed.AvrMecSpeed;

        foc_motor->pid_speed.SetPoint = foc_motor->target_speed;
        foc_motor->foc_var.I_qd_ref.q = PID_Position_ctrl(&foc_motor->pid_speed, speed);
        foc_motor->foc_var.I_qd_ref.d = 0.00001f;
        return;
    }
#endif

    /* 未启用或非法模式的安全回退。 */
    foc_motor->foc_var.I_qd_ref.q = 0.0f;
    foc_motor->foc_var.I_qd_ref.d = 0.0f;
    PID_Clear(&foc_motor->pid_speed);
    PID_Clear(&foc_motor->pid_speed_pos);

}


/**
 * @brief 电机堵转后的操作，可自由开发
 * 
 * @param foc_motor 
 */
void FOC_Motor_Lock(FOC_MOTOR_t *foc_motor)
{
    /*清除PID参数*/
    PID_Clear(&foc_motor->pid_cur_iq);
    PID_Clear(&foc_motor->pid_cur_id);
    PID_Clear(&foc_motor->pid_speed);
    PID_Clear(&foc_motor->pid_speed_pos);
    foc_motor->position_speed_ref = 0.0f;
}

