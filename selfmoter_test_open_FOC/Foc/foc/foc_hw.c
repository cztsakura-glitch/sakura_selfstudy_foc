/**
 ****************************************************************************************************
 * @file        foc_hw.c
 * @author      哔哩哔哩-Rebron大侠
 * @version     V0.0
 * @date        2025-01-11
 * @brief       该FOC库所调用的一些硬件接口
 * @license     MIT License
 *              Copyright (c) 2025 Reborn大侠
 *              允许任何人使用、复制、修改和分发该代码，但需保留此版权声明。
 ****************************************************************************************************
 */



#include "foc_hw.h"
#include "tim.h"
#include "main.h"
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim6;
#include "foc_motor.h"
#include "foc_app.h"
#include "foc_math.h"
#include "foc_as5047p.h"
#include <stdio.h>

#include "bsp.h"
/*
 * 开环测试模式：
 * 1：TIM6 中断调用 main.c 里的 Motor_OpenLoop_Tim6Callback()
 * 0：恢复原来的 FOC 速度计算和速度环
 */
#define MOTOR_OPENLOOP_TEST     0
#define FOC_SPEED_LOOP_DIVIDER  20U
#define FOC_DEBUG_PRINT_DIVIDER 1000U

static volatile uint32_t g_foc_adc_irq_count = 0U;
static volatile uint32_t g_foc_app_run_count = 0U;
static volatile uint32_t g_foc_tim6_irq_count = 0U;
static volatile uint32_t g_foc_speed_loop_count = 0U;
static volatile bool g_foc_pwm_output_enable = false;
static volatile bool g_foc_run_request = false;
static volatile bool g_foc_align_only = false;
static volatile bool g_foc_current_csv_enable = false;
static volatile uint32_t g_foc_vofa_mode = FOC_VOFA_OFF;
static volatile uint32_t g_foc_adc_sw_fail_count = 0U;
static volatile uint32_t g_foc_adc_tim1_sample_cnt = 0U;
/* 对齐诊断确认本控制板与驱动器接线应采用WVU相序。 */
static volatile uint32_t g_foc_phase_map = 5U;
static volatile uint32_t g_foc_encoder_mode = FOC_DEFAULT_CONTROL_MODE;
/* 30 mA不足以持续回位，而80 mA会使轻载位置环在目标附近动作过强，因此取50 mA。 */
static volatile float g_foc_encoder_iq_ref = 0.05f;
static volatile float g_foc_encoder_id_ref = 0.0f;
static volatile float g_foc_encoder_speed_iq_dir = 1.0f;
static float FOC_Debug_ClampFloat(float value, float min_value, float max_value);
/* 逻辑FOC相到物理驱动通道(U、V、W)的映射；PWM输出和三相电流采样必须采用同一排列。 */
static const uint8_t g_foc_phase_channel_map[6][3] = {
    {0U, 1U, 2U},
    {0U, 2U, 1U},
    {1U, 0U, 2U},
    {1U, 2U, 0U},
    {2U, 0U, 1U},
    {2U, 1U, 0U},
};
#if MOTOR_OPENLOOP_TEST
extern void Motor_OpenLoop_Tim6Callback(void);
#endif

void FOC_PWM_HW_SetPhasePwm(uint8_t phase, uint32_t ccr)
{
    uint32_t map = g_foc_phase_map % 6U;
    uint8_t channel;

    if (phase > 2U)
    {
        return;
    }

    channel = g_foc_phase_channel_map[map][phase];

    switch (channel)
    {
        case 0U:
            htim1.Instance->CCR1 = ccr;
            break;
        case 1U:
            htim1.Instance->CCR2 = ccr;
            break;
        case 2U:
            htim1.Instance->CCR3 = ccr;
            break;
        default:
            break;
    }
}
static const char *FOC_Debug_EncoderModeName(void)
{
    if (g_foc_encoder_mode == FOC_CONTROL_POSITION)
    {
        return "position";
    }

    if (g_foc_encoder_mode == FOC_CONTROL_SPEED)
    {
        return "speed";
    }

    if (g_foc_encoder_mode == FOC_CONTROL_TORQUE)
    {
        return "torque";
    }

    return "vf";
}

void FOC_Debug_SetCurrentCsv(bool enable)
{
    FOC_Debug_SetVofaMode(enable ? FOC_VOFA_AUTO : FOC_VOFA_OFF);
}

bool FOC_Debug_CurrentCsvEnabled(void)
{
    return g_foc_current_csv_enable;
}

void FOC_Debug_SetVofaMode(uint32_t mode)
{
    if (mode > FOC_VOFA_AUTO)
    {
        mode = FOC_VOFA_OFF;
    }

    g_foc_vofa_mode = mode;
    g_foc_current_csv_enable = (mode != FOC_VOFA_OFF);

    if (mode == FOC_VOFA_CURRENT)
    {
        /* FireWater 只会自动显示 I0、I1……；这里打印固定映射，便于在 VOFA 中手动重命名。 */
        printf("[VOFA_MAP][current] I0=IqRef_mA I1=IqActual_mA I2=IdRef_mA I3=IdActual_mA I4=Uq_mV I5=Ud_mV; rate=200Hz\r\n");
    }
    else if (mode == FOC_VOFA_SPEED)
    {
        printf("[VOFA_MAP][speed] I0=SpeedRef_rpm I1=SpeedActual_rpm I2=IqRef_mA I3=IqActual_mA I4=SpeedP_mA I5=SpeedI_mA; rate=50Hz\r\n");
    }
    else if (mode == FOC_VOFA_POSITION)
    {
        printf("[VOFA_MAP][position] I0=PosRef_x10deg I1=PosActual_x10deg I2=SpeedRef_rpm I3=SpeedActual_rpm I4=IqRef_mA I5=IqActual_mA; rate=50Hz\r\n");
    }
    else if (mode == FOC_VOFA_AUTO)
    {
        printf("[VOFA] auto: output follows active FOC mode\r\n");
    }
    else
    {
        printf("[VOFA] OFF\r\n");
    }
}

/* AUTO 模式根据当前闭环选择 VOFA 曲线类型，显式模式则直接返回。 */
static uint32_t FOC_Debug_ResolveVofaMode(void)
{
    if (g_foc_vofa_mode != FOC_VOFA_AUTO)
    {
        return g_foc_vofa_mode;
    }

    if (g_foc_encoder_mode == FOC_CONTROL_POSITION)
    {
        return FOC_VOFA_POSITION;
    }
    if (g_foc_encoder_mode == FOC_CONTROL_SPEED)
    {
        return FOC_VOFA_SPEED;
    }
    return FOC_VOFA_CURRENT;
}

uint32_t FOC_Debug_GetVofaPeriodMs(void)
{
    uint32_t mode = FOC_Debug_ResolveVofaMode();

    if (mode == FOC_VOFA_CURRENT)
    {
        return 5U;
    }
    if (mode == FOC_VOFA_SPEED)
    {
        return 20U;
    }
    return 20U;
}

void FOC_Debug_PrintCurrentCsv(void)
{
    float speed_ref = FOC_MOTOR.target_speed;
    uint32_t mode = FOC_Debug_ResolveVofaMode();

    if (g_foc_encoder_mode == FOC_CONTROL_POSITION)
    {
        speed_ref = FOC_MOTOR.position_speed_ref;
    }

    if (mode == FOC_VOFA_CURRENT)
    {
        /* FireWater：帧名:逗号分隔的纯数字\r\n */
        printf("current:%d,%d,%d,%d,%d,%d\r\n",
               (int)(FOC_MOTOR.foc_var.I_qd_ref.q * 1000.0f),
               (int)(FOC_MOTOR.foc_var.I_qd.q * 1000.0f),
               (int)(FOC_MOTOR.foc_var.I_qd_ref.d * 1000.0f),
               (int)(FOC_MOTOR.foc_var.I_qd.d * 1000.0f),
               (int)(FOC_MOTOR.foc_var.U_qd.q * 1000.0f),
               (int)(FOC_MOTOR.foc_var.U_qd.d * 1000.0f));
    }
    else if (mode == FOC_VOFA_SPEED)
    {
        printf("speed:%d,%d,%d,%d,%d,%d\r\n",
               (int)speed_ref,
               (int)FOC_MOTOR.foc_var.speed.AvrMecSpeed,
               (int)(FOC_MOTOR.foc_var.I_qd_ref.q * 1000.0f),
               (int)(FOC_MOTOR.foc_var.I_qd.q * 1000.0f),
               (int)(FOC_MOTOR.pid_speed.Up * 1000.0f),
               (int)(FOC_MOTOR.pid_speed.Ui * 1000.0f));
    }
    else if (mode == FOC_VOFA_POSITION)
    {
        printf("position:%d,%d,%d,%d,%d,%d\r\n",
               (int)(FOC_MOTOR.target_position_deg * 10.0f),
               (int)(FOC_AS5047P_GetPositionDeg() * 10.0f),
               (int)speed_ref,
               (int)FOC_MOTOR.foc_var.speed.AvrMecSpeed,
               (int)(FOC_MOTOR.foc_var.I_qd_ref.q * 1000.0f),
               (int)(FOC_MOTOR.foc_var.I_qd.q * 1000.0f));
    }
}
/* 将统一速度目标换算成 V/F 模式需要的电角速度和电压方向。 */
static void FOC_Debug_UpdateVfFromTarget(void)
{
    FOC_VF_t *vf = &FOC_MOTOR.sensorless.vf;
    float uq = vf->run_uq;
    float target_step;

    if ((uq > -0.01f) && (uq < 0.01f))
    {
        uq = 0.30f;
        vf->run_uq = uq;
    }

    if ((vf->align_ud > -0.01f) && (vf->align_ud < 0.01f))
    {
        vf->align_ud = uq;
    }

    if (vf->align_ticks == 0U)
    {
        vf->align_ticks = 10000U;
    }



    target_step = FOC_MOTOR.target_speed * _2PI *
                  (float)FOC_MOTOR.pole_pairs *
                  vf->Ts / 60.0f;

    vf->k = target_step / uq;
}
static void FOC_Debug_ResetRuntimeState(void)
{
    FOC_MOTOR.current.ad_offset.a = 0.0f;
    FOC_MOTOR.current.ad_offset.b = 0.0f;
    FOC_MOTOR.current.ad_offset.c = 0.0f;
    FOC_MOTOR.current.ad_offset_temp.a = 0.0f;
    FOC_MOTOR.current.ad_offset_temp.b = 0.0f;
    FOC_MOTOR.current.ad_offset_temp.c = 0.0f;
    FOC_MOTOR.current.ad_offset_count = 0U;
    FOC_MOTOR.current.I_abc.a = 0.0f;
    FOC_MOTOR.current.I_abc.b = 0.0f;
    FOC_MOTOR.current.I_abc.c = 0.0f;

    FOC_MOTOR.foc_var.I_qd_ref.q = 0.0f;
    FOC_MOTOR.foc_var.I_qd_ref.d = 0.0f;
    FOC_MOTOR.foc_var.U_qd.q = 0.0f;
    FOC_MOTOR.foc_var.U_qd.d = 0.0f;
    FOC_MOTOR.pwm.Uqd.q = 0.0f;
    FOC_MOTOR.pwm.Uqd.d = 0.0f;
    FOC_MOTOR.sensorless.vf.speed.ElAngle = 0.0f;
    FOC_MOTOR.sensorless.vf.speed.AvrMecSpeed = 0.0f;
    FOC_MOTOR.sensorless.vf.step = 0.0f;
    FOC_MOTOR.sensorless.vf.step_sum = 0.0f;
    FOC_Debug_UpdateVfFromTarget();
    FOC_MOTOR.sensorless.vf.align_count = 0U;
    FOC_MOTOR.sensorless.vf.Uqd.q = 0.0f;
    FOC_MOTOR.sensorless.vf.Uqd.d = FOC_MOTOR.sensorless.vf.align_ud;
    FOC_MOTOR.position_speed_ref = 0.0f;
    PID_Clear(&FOC_MOTOR.pid_speed_pos);

    FOC_Motor_Lock(&FOC_MOTOR);
    FOC_MOTOR.motor_state = MOTOR_IDLE;
}

bool FOC_Debug_GetRunRequest(void)
{
    return g_foc_run_request;
}

void FOC_Debug_SetAlignOnly(bool enable)
{
    g_foc_align_only = enable;
}

bool FOC_Debug_GetAlignOnly(void)
{
    return g_foc_align_only;
}

void FOC_Debug_SetRunRequest(bool enable)
{
    if (enable)
    {
        if (!g_foc_run_request)
        {
            FOC_Debug_ResetRuntimeState();
            g_foc_run_request = true;
            printf("[FOC_CMD] runReq=1 mode=%s target=%d phase=%u vfUq=%dmV runUd=%dmV vfK=%d iq=%dmA id=%dmA iqDir=%d\r\n",
                   FOC_Debug_EncoderModeName(),
                   (int)FOC_MOTOR.target_speed,
                   (unsigned int)g_foc_phase_map,
                   (int)(FOC_MOTOR.sensorless.vf.run_uq * 1000.0f),
                   (int)(FOC_MOTOR.sensorless.vf.run_ud * 1000.0f),
                   (int)(FOC_MOTOR.sensorless.vf.k * 1000000.0f),
                   (int)(g_foc_encoder_iq_ref * 1000.0f),
                   (int)(g_foc_encoder_id_ref * 1000.0f),
                   (g_foc_encoder_speed_iq_dir < 0.0f) ? -1 : 1);
        }
    }
    else
    {
        if (g_foc_run_request)
        {
            printf("[FOC_CMD] runReq=0\r\n");
        }

        g_foc_run_request = false;
        g_foc_align_only = false;
        FOC_PWM_StopALL(&FOC_MOTOR.pwm);
        FOC_Debug_ResetRuntimeState();
    }
}

void FOC_Debug_SetTargetSpeed(float target_speed)
{
    float old_target = FOC_MOTOR.target_speed;
    bool target_cross_zero = ((old_target > 0.0f) && (target_speed < 0.0f)) ||
                             ((old_target < 0.0f) && (target_speed > 0.0f));

    FOC_MOTOR.target_speed = target_speed;
    FOC_Debug_UpdateVfFromTarget();

    if ((g_foc_encoder_mode == 2U) && target_cross_zero)
    {
        PID_Clear(&FOC_MOTOR.pid_speed);
        FOC_MOTOR.foc_var.I_qd_ref.q = 0.0f;
        FOC_MOTOR.pid_cur_iq.SetPoint = 0.0f;
        printf("[FOC_CMD] speed PID clear on dir change\r\n");
    }

    printf("[CMD] speed=%d rpm\r\n", (int)FOC_MOTOR.target_speed);
}
void FOC_Debug_AdjustTargetSpeed(float delta_speed)
{
    FOC_Debug_SetTargetSpeed(FOC_MOTOR.target_speed + delta_speed);
}

void FOC_Debug_SetTargetPositionDeg(float target_position_deg)
{
    float limit = FOC_MOTOR.pid_speed_pos.SetPoint_limit;

    if (limit < 1.0f)
    {
        limit = 3600.0f;
    }

    FOC_MOTOR.target_position_deg =
        FOC_Debug_ClampFloat(target_position_deg, -limit, limit);
    FOC_MOTOR.pid_speed_pos.SetPoint = FOC_MOTOR.target_position_deg;

    printf("[FOC_CMD] positionTarget_x10=%d deg\r\n",
           (int)(FOC_MOTOR.target_position_deg * 10.0f));
}

void FOC_Debug_AdjustTargetPositionDeg(float delta_position_deg)
{
    FOC_Debug_SetTargetPositionDeg(FOC_MOTOR.target_position_deg +
                                   delta_position_deg);
}

float FOC_Debug_GetTargetPositionDeg(void)
{
    return FOC_MOTOR.target_position_deg;
}


uint32_t FOC_Debug_GetPhaseMap(void)
{
    return g_foc_phase_map;
}

void FOC_Debug_SetPhaseMap(uint32_t phase_map)
{
    g_foc_phase_map = phase_map % 6U;
    g_foc_run_request = false;
    FOC_PWM_StopALL(&FOC_MOTOR.pwm);
    FOC_Debug_ResetRuntimeState();
    printf("[FOC_CMD] phase=%u map=%s runReq=0\r\n",
           (unsigned int)g_foc_phase_map,
           (g_foc_phase_map == 0U) ? "UVW" :
           (g_foc_phase_map == 1U) ? "UWV" :
           (g_foc_phase_map == 2U) ? "VUW" :
           (g_foc_phase_map == 3U) ? "VWU" :
           (g_foc_phase_map == 4U) ? "WUV" : "WVU");
}
void FOC_Debug_SetVfUq(float uq_volt)
{
    if (uq_volt > 1.20f)
    {
        uq_volt = 1.20f;
    }
    else if (uq_volt < 0.05f)
    {
        uq_volt = 0.05f;
    }

    FOC_MOTOR.sensorless.vf.run_uq = uq_volt;
    FOC_MOTOR.sensorless.vf.align_ud = uq_volt;
    if (FOC_MOTOR.sensorless.vf.align_count < FOC_MOTOR.sensorless.vf.align_ticks)
    {
        FOC_MOTOR.sensorless.vf.Uqd.q = 0.0f;
        FOC_MOTOR.sensorless.vf.Uqd.d = uq_volt;
    }
    else
    {
        FOC_MOTOR.sensorless.vf.Uqd.q = uq_volt;
        FOC_MOTOR.sensorless.vf.Uqd.d = FOC_MOTOR.sensorless.vf.run_ud;
    }
    FOC_Debug_UpdateVfFromTarget();
    printf("[FOC_CMD] vfUq=%dmV runUd=%dmV alignUd=%dmV vfK=%d\r\n",
           (int)(FOC_MOTOR.sensorless.vf.run_uq * 1000.0f),
           (int)(FOC_MOTOR.sensorless.vf.run_ud * 1000.0f),
           (int)(FOC_MOTOR.sensorless.vf.align_ud * 1000.0f),
           (int)(FOC_MOTOR.sensorless.vf.k * 1000000.0f));
}
void FOC_Debug_SetVfUd(float ud_volt)
{
    if (ud_volt > 0.60f)
    {
        ud_volt = 0.60f;
    }
    else if (ud_volt < -0.60f)
    {
        ud_volt = -0.60f;
    }

    FOC_MOTOR.sensorless.vf.run_ud = ud_volt;
    if (FOC_MOTOR.sensorless.vf.align_count >= FOC_MOTOR.sensorless.vf.align_ticks)
    {
        FOC_MOTOR.sensorless.vf.Uqd.d = ud_volt;
    }

    printf("[FOC_CMD] runUd=%dmV vfUq=%dmV\r\n",
           (int)(FOC_MOTOR.sensorless.vf.run_ud * 1000.0f),
           (int)(FOC_MOTOR.sensorless.vf.run_uq * 1000.0f));
}
void FOC_Debug_SetPwmMode(uint32_t use_spwm)
{
    FOC_PWM_DebugSetSpwm(use_spwm);
    printf("[FOC_CMD] pwm=%s\r\n", FOC_PWM_DebugGetSpwm() ? "spwm" : "svpwm");
}

static float FOC_Debug_ClampFloat(float value, float min_value, float max_value)
{
    if (value > max_value)
    {
        return max_value;
    }

    if (value < min_value)
    {
        return min_value;
    }

    return value;
}

void FOC_Debug_SetEncoderMode(uint32_t mode)
{
    bool enabled = false;

#if FOC_ENABLE_VF_MODE
    enabled = enabled || (mode == FOC_CONTROL_VF);
#endif
#if FOC_ENABLE_TORQUE_MODE
    enabled = enabled || (mode == FOC_CONTROL_TORQUE);
#endif
#if FOC_ENABLE_SPEED_MODE
    enabled = enabled || (mode == FOC_CONTROL_SPEED);
#endif
#if FOC_ENABLE_POSITION_MODE
    enabled = enabled || (mode == FOC_CONTROL_POSITION);
#endif

    if (!enabled)
    {
        printf("[FOC_CMD] requested mode=%lu is disabled\r\n",
               (unsigned long)mode);
        return;
    }

    g_foc_encoder_mode = mode;
    g_foc_run_request = false;
    FOC_PWM_StopALL(&FOC_MOTOR.pwm);
    FOC_Debug_ResetRuntimeState();

    /* 切换控制模式时先清运行请求并清状态, 防止旧模式的积分/对齐状态影响新模式. */
    printf("[FOC_CMD] mode=%s runReq=0 iq=%dmA id=%dmA iqDir=%d\r\n",
           FOC_Debug_EncoderModeName(),
           (int)(g_foc_encoder_iq_ref * 1000.0f),
           (int)(g_foc_encoder_id_ref * 1000.0f),
           (g_foc_encoder_speed_iq_dir < 0.0f) ? -1 : 1);
}

uint32_t FOC_Debug_GetEncoderMode(void)
{
    return g_foc_encoder_mode;
}

/* 设置 AS5047P 闭环允许的 q 轴电流. 速度环输出会被限制在这个范围内. */
void FOC_Debug_SetEncoderIqRef(float iq_amp)
{
    g_foc_encoder_iq_ref = FOC_Debug_ClampFloat(iq_amp, -0.80f, 0.80f);
    printf("[FOC_CMD] iqRef=%dmA\r\n", (int)(g_foc_encoder_iq_ref * 1000.0f));
}

/* 设置 d 轴电流目标. 普通表贴电机调速时通常保持 id=0. */
void FOC_Debug_SetEncoderIdRef(float id_amp)
{
    g_foc_encoder_id_ref = FOC_Debug_ClampFloat(id_amp, -0.50f, 0.50f);
    printf("[FOC_CMD] idRef=%dmA\r\n", (int)(g_foc_encoder_id_ref * 1000.0f));
}

/* 设置速度环输出 Iq 的符号. 如果目标转速为正但实测 rpm 为负, 就需要翻转这个方向. */
void FOC_Debug_SetEncoderSpeedIqDir(float dir)
{
    g_foc_encoder_speed_iq_dir = (dir < 0.0f) ? -1.0f : 1.0f;
    FOC_Motor_Lock(&FOC_MOTOR);
    printf("[FOC_CMD] iqDir=%d\r\n", (g_foc_encoder_speed_iq_dir < 0.0f) ? -1 : 1);
}

float FOC_Debug_GetEncoderIqRef(void)
{
    return g_foc_encoder_iq_ref;
}

float FOC_Debug_GetEncoderIdRef(void)
{
    return g_foc_encoder_id_ref;
}

float FOC_Debug_GetEncoderSpeedIqDir(void)
{
    return g_foc_encoder_speed_iq_dir;
}
/***************************************电流硬件相关**************************************/
#include "adc.h"
/**
 * @brief 电流部分的硬件初始化
 * 
 */
void FOC_CURRENT_HW_Init(void)
{   
    /*通道1不再使用, 该ADC用于母线电压检测*/
//    HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
    // HAL_Delay(100);
//    HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);
//    HAL_Delay(100);
//    HAL_ADCEx_Calibration_Start(&hadc3, ADC_SINGLE_ENDED);
//    HAL_Delay(100);

    /* TIM1通道4通过硬件触发，使ADC注入序列与PWM同步。 */
    printf("[FOC] ADC TIM1-OC4REF/TRGO hardware trigger armed\r\n");
    __HAL_ADC_CLEAR_FLAG(&hadc1, ADC_FLAG_JEOC | ADC_FLAG_JSTRT | ADC_FLAG_OVR);
    if (HAL_ADCEx_InjectedStart_IT(&hadc1) != HAL_OK)
    {
        Error_Handler();
    }
//    HAL_ADCEx_InjectedStart_IT(&hadc2);
//    HAL_ADCEx_InjectedStart_IT(&hadc3);
}
/**
 * @brief 电流部分的硬件复位
 * 
 */
void FOC_CURRENT_HW_DeInit(void)
{
    HAL_ADCEx_InjectedStop_IT(&hadc1);
//    HAL_ADCEx_InjectedStop_IT(&hadc2);
//    HAL_ADCEx_InjectedStop_IT(&hadc3);
}

/*更新FOC电流*/
void FOC_CURRENT_Update(FOC_CURRENT_t *foc_cur)
{
    /* 采样频率20 kHz、系数0.02时，电流测量带宽约为64 Hz；该带宽仍高于当前整定的
     * 电流环带宽，同时可抑制约5 mA/计数的ADC噪声，减轻低速抖动。 */
    const float filter_alpha = 0.02f;
    float physical_current[3];
    uint32_t map = g_foc_phase_map % 6U;
    float ia;
    float ib;
    float ic;
    float common;

    foc_cur->mcu_ad.c = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1) - foc_cur->ad_offset.c;
    foc_cur->mcu_ad.b = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_2) - foc_cur->ad_offset.b;
    foc_cur->mcu_ad.a = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_3) - foc_cur->ad_offset.a;

    /*计算三相电流*/
    physical_current[0] = CC6920SO_CalcCur(0, foc_cur->mcu_ad.a); /* U */
    physical_current[1] = CC6920SO_CalcCur(1, foc_cur->mcu_ad.b); /* V */
    physical_current[2] = CC6920SO_CalcCur(2, foc_cur->mcu_ad.c); /* W */

    /*
     * 将物理 U/V/W 电流还原为 Clarke/Park 使用的逻辑 A/B/C 顺序。
     * 例如线序映射 5（WVU）对应 A=IW、B=IV、C=IU。
     */
    ia = physical_current[g_foc_phase_channel_map[map][0]];
    ib = physical_current[g_foc_phase_channel_map[map][1]];
    ic = physical_current[g_foc_phase_channel_map[map][2]];

    /* 三相电流理论和为零：先去除开关共模噪声，再进行高于电流环带宽的低通滤波。 */
    common = (ia + ib + ic) / 3.0f;
    ia -= common;
    ib -= common;
    ic -= common;

    foc_cur->I_abc.a += filter_alpha * (ia - foc_cur->I_abc.a);
    foc_cur->I_abc.b += filter_alpha * (ib - foc_cur->I_abc.b);
    foc_cur->I_abc.c += filter_alpha * (ic - foc_cur->I_abc.c);
    // foc_cur->I_abc.c = -(foc_cur->I_abc.a + foc_cur->I_abc.b);
}


/****************************************************************************************/




/***************************************PWM硬件相关**************************************/

/**
 * @brief PWM硬件初始化
 * 
 */
void FOC_PWM_HW_Init(void)
{
    uint32_t half = (__HAL_TIM_GET_AUTORELOAD(&htim1) + 1U) / 2U;

    HAL_GPIO_WritePin(MOTOR_EN_GPIO_Port, MOTOR_EN_Pin, GPIO_PIN_RESET);

    if (HAL_TIM_Base_Start(&htim1) != HAL_OK)
    {
        Error_Handler();
    }

    if (HAL_TIM_OC_Start(&htim1, TIM_CHANNEL_4) != HAL_OK)
    {
        Error_Handler();
    }

#if MOTOR_OPENLOOP_TEST
    if (HAL_TIM_Base_Start_IT(&htim6) != HAL_OK)
    {
        Error_Handler();
    }
#endif

    U_H_SET_PWM(half);
    V_H_SET_PWM(half);
    W_H_SET_PWM(half);
    printf("[FOC] TIM1/ADC-sync started half=%u sampleCCR4=%u\r\n",
           (unsigned int)half,
           (unsigned int)htim1.Instance->CCR4);
}
void FOC_PWM_HW_DeInit(void)
{
    HAL_GPIO_WritePin(MOTOR_EN_GPIO_Port, MOTOR_EN_Pin, GPIO_PIN_RESET);

    HAL_TIM_Base_Stop(&htim1);
    HAL_TIM_OC_Stop(&htim1, TIM_CHANNEL_4);
    HAL_TIM_Base_Stop_IT(&htim6);
}

/**
 * @brief PWM硬件通道开关
 * 
 * @param A     A相上管通道
 * @param A_N   A相下管通道
 * @param B     B相上管通道
 * @param B_N   B相下管通道
 * @param C     C相下管通道
 * @param C_N   C相下管通道
 */
void FOC_PWM_HW_ON_OFF(bool A, bool A_N, bool B, bool B_N, bool C, bool C_N)
{
    bool output_enable = A || A_N || B || B_N || C || C_N;

    if (!output_enable)
    {
        HAL_GPIO_WritePin(MOTOR_EN_GPIO_Port, MOTOR_EN_Pin, GPIO_PIN_RESET);
    }

    if(A)   HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    else    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);

    if(A_N) HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
    else    HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_1);

    if(B)   HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    else    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);

    if(B_N) HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
    else    HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_2);

    if(C)   HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    else    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);

    if(C_N) HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
    else    HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_3);

    if (output_enable)
    {
        HAL_GPIO_WritePin(MOTOR_EN_GPIO_Port, MOTOR_EN_Pin, GPIO_PIN_SET);
    }

    g_foc_pwm_output_enable = output_enable;
}

/****************************************************************************************/








void FOC_Debug_PrintStatus(void)
{
    static uint32_t last_adc_ok_count = 0U;
    static uint32_t last_app_run_count = 0U;
    static uint32_t last_tim6_irq_count = 0U;
    static uint32_t last_speed_loop_count = 0U;

    uint32_t adc_ok_count = g_foc_adc_irq_count;
    uint32_t app_run_count = g_foc_app_run_count;
    uint32_t tim6_irq_count = g_foc_tim6_irq_count;
    uint32_t speed_loop_count = g_foc_speed_loop_count;

    uint32_t adc_sr = ADC1->SR;
    uint32_t adc_cr2 = ADC1->CR2;
    uint32_t adc_jsqr = ADC1->JSQR;
    uint32_t jdr1 = ADC1->JDR1;
    uint32_t jdr2 = ADC1->JDR2;
    uint32_t jdr3 = ADC1->JDR3;

    int ia_ma = (int)(FOC_MOTOR.current.I_abc.a * 1000.0f);
    int ib_ma = (int)(FOC_MOTOR.current.I_abc.b * 1000.0f);
    int ic_ma = (int)(FOC_MOTOR.current.I_abc.c * 1000.0f);
    int iq_ma = (int)(FOC_MOTOR.foc_var.I_qd.q * 1000.0f);
    int id_ma = (int)(FOC_MOTOR.foc_var.I_qd.d * 1000.0f);
    int iq_target_ma = (int)(FOC_MOTOR.foc_var.I_qd_ref.q * 1000.0f);
    int id_target_ma = (int)(FOC_MOTOR.foc_var.I_qd_ref.d * 1000.0f);
    int uq_mv = (int)(FOC_MOTOR.foc_var.U_qd.q * 1000.0f);
    int ud_mv = (int)(FOC_MOTOR.foc_var.U_qd.d * 1000.0f);
    int rpm = (int)(FOC_MOTOR.foc_var.speed.AvrMecSpeed);
    int target = (int)(FOC_MOTOR.target_speed);
    int vf_uq_mv = (int)(FOC_MOTOR.sensorless.vf.run_uq * 1000.0f);
    int vf_ud_mv = (int)(FOC_MOTOR.sensorless.vf.run_ud * 1000.0f);
    int vf_align_mv = (int)(FOC_MOTOR.sensorless.vf.align_ud * 1000.0f);
    int vf_align_left = (int)((FOC_MOTOR.sensorless.vf.align_count < FOC_MOTOR.sensorless.vf.align_ticks) ?
                              (FOC_MOTOR.sensorless.vf.align_ticks - FOC_MOTOR.sensorless.vf.align_count) : 0U);
    int vf_step_u = (int)(FOC_MOTOR.sensorless.vf.step * 1000000.0f);
    int vf_k_u = (int)(FOC_MOTOR.sensorless.vf.k * 1000000.0f);
    int phase = (int)g_foc_phase_map;
    int enc_iq_ref_ma = (int)(g_foc_encoder_iq_ref * 1000.0f);
    int enc_id_ref_ma = (int)(g_foc_encoder_id_ref * 1000.0f);
    int enc_iq_dir = (g_foc_encoder_speed_iq_dir < 0.0f) ? -1 : 1;
    const FOC_AS5047P_t *as5047;
    int as5047_el_x10;
    int position_target_x10;
    int position_actual_x10;
    int position_speed_ref_x10;

    if (!((g_foc_encoder_mode != 0U) && g_foc_run_request))
    {
        FOC_AS5047P_Update(0.0f);
        FOC_AS5047P_ReadDiagnostics();
    }

    as5047 = FOC_AS5047P_GetState();
    as5047_el_x10 = (int)(as5047->el_angle * 1800.0f / _2PI);
    position_target_x10 = (int)(FOC_MOTOR.target_position_deg * 10.0f);
    position_actual_x10 = (int)(FOC_AS5047P_GetPositionDeg() * 10.0f);
    position_speed_ref_x10 = (int)(FOC_MOTOR.position_speed_ref * 10.0f);

    printf("[FOC] st=%d sl=%d obs=%d phase=%d pwm=%s mode=%s iqRef=%d idRef=%d iqDir=%d runReq=%u en=%d pg12=%d ",
           (int)FOC_MOTOR.motor_state,
           (int)FOC_MOTOR.sensorless.sta,
           (int)FOC_MOTOR.sensorless.Observer,
           phase,
           FOC_PWM_DebugGetSpwm() ? "spwm" : "svpwm",
           FOC_Debug_EncoderModeName(),
           enc_iq_ref_ma,
           enc_id_ref_ma,
           enc_iq_dir,
           g_foc_run_request ? 1U : 0U,
           g_foc_pwm_output_enable ? 1 : 0,
           HAL_GPIO_ReadPin(MOTOR_EN_GPIO_Port, MOTOR_EN_Pin) == GPIO_PIN_SET ? 1 : 0);
    printf("adcOk=%u(+%u) app=%u(+%u) adcFail=%u sampleCnt=%u t1Cnt=%u cr1=%04lX ccer=%04lX tim6=%u(+%u) spdLoop=%u(+%u) offCnt=%u ",
           (unsigned int)adc_ok_count,
           (unsigned int)(adc_ok_count - last_adc_ok_count),
           (unsigned int)app_run_count,
           (unsigned int)(app_run_count - last_app_run_count),
           (unsigned int)g_foc_adc_sw_fail_count,
           (unsigned int)g_foc_adc_tim1_sample_cnt,
           (unsigned int)__HAL_TIM_GET_COUNTER(&htim1),
           (unsigned long)htim1.Instance->CR1,
           (unsigned long)htim1.Instance->CCER,
           (unsigned int)tim6_irq_count,
           (unsigned int)(tim6_irq_count - last_tim6_irq_count),
           (unsigned int)speed_loop_count,
           (unsigned int)(speed_loop_count - last_speed_loop_count),
           (unsigned int)FOC_MOTOR.current.ad_offset_count);

    printf("ccr=%u/%u/%u target=%d rpm=%d vf=%d/%d/%d/%d align=%d/%d enc=%u/%d/%u edir=%d err=%lu/con=%lu/jump=%lu Iabc_mA=%d/%d/%d Iqd_mA=%d/%d ref=%d/%d Uqd_mV=%d/%d ",
           (unsigned int)htim1.Instance->CCR1,
           (unsigned int)htim1.Instance->CCR2,
           (unsigned int)htim1.Instance->CCR3,
           target,
           rpm,
           vf_uq_mv,
           vf_ud_mv,
           vf_step_u,
           vf_k_u,
           vf_align_mv,
           vf_align_left,
           (unsigned int)as5047->raw,
           as5047_el_x10,
           (unsigned int)as5047->mag,
           (as5047->dir < 0.0f) ? -1 : 1,
           (unsigned long)as5047->error_count,
           (unsigned long)as5047->consecutive_error_count,
           (unsigned long)as5047->jump_reject_count,
           ia_ma,
           ib_ma,
           ic_ma,
           iq_ma,
           id_ma,
           iq_target_ma,
           id_target_ma,
           uq_mv,
           ud_mv);

    printf("jdr=%u/%u/%u raw=%d/%d/%d zero=%d/%d/%d adcReg=SR:%04lX CR2:%08lX JSQR:%08lX\r\n",
           (unsigned int)jdr1,
           (unsigned int)jdr2,
           (unsigned int)jdr3,
           (int)FOC_MOTOR.current.mcu_ad.a,
           (int)FOC_MOTOR.current.mcu_ad.b,
           (int)FOC_MOTOR.current.mcu_ad.c,
           (int)FOC_MOTOR.current.ad_offset.a,
           (int)FOC_MOTOR.current.ad_offset.b,
           (int)FOC_MOTOR.current.ad_offset.c,
           (unsigned long)adc_sr,
           (unsigned long)adc_cr2,
           (unsigned long)adc_jsqr);
    printf("[POSITION] target_x10=%d actual_x10=%d speedRef_x10=%d deg/rpm\r\n",
           position_target_x10,
           position_actual_x10,
           position_speed_ref_x10);

    last_adc_ok_count = adc_ok_count;
    last_app_run_count = app_run_count;
    last_tim6_irq_count = tim6_irq_count;
    last_speed_loop_count = speed_loop_count;
}
/***************************************ABZ编码器硬件相关**************************************/

/**
 * @brief ABZ编码器硬件初始化
 * 
 * @param enc 
 */
void FOC_ABZENC_HW_Init(FOC_ABZENC_t *enc)
{
//    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);         /*开启定时器编码器模式*/
//    HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_2);             /*开启定时器捕获, 捕获编码器Z相*/
}

void FOC_ABZENC_HW_DeInit(FOC_ABZENC_t *enc)
{
//    HAL_TIM_Encoder_Stop(&htim3, TIM_CHANNEL_ALL);          /*开启定时器编码器模式*/
//    HAL_TIM_IC_Stop_IT(&htim2, TIM_CHANNEL_2);              /*开启定时器捕获, 捕获编码器Z相*/
}

/**
 * @brief ABZ编码器中断中调用
 * 
 * @param enc 
 */
//void FOC_ABZENC_HW_IRQHandler(FOC_ABZENC_t *enc)
//{
//    /*定位Z轴*/
//    if(enc->dir==ENC_DOWN){
//        __HAL_TIM_SET_COUNTER(&htim3, 0);                       
//    }else{
//        __HAL_TIM_SET_COUNTER(&htim3, htim3.Init.Period);       
//    }

//    /*定时器溢出了*/
//    if(enc->align==true){
//        enc->over_count += 1;
//    }else{
//        enc->align = true;
//        printf("ABZ编码器已对齐过一次Z轴: OK\r\n");
//    }
//}

/**
 * @brief 获取ABZ编码器当前的脉冲值
 * 
 * @param enc 
 * @return uint32_t 返回脉冲值
 */
//uint32_t FOC_ABZENC_HW_GetCount(FOC_ABZENC_t *enc)
//{
//    return (uint32_t)(__HAL_TIM_GET_COUNTER(&htim3));
//}


/**
 * @brief 获取编码器计数方向 0递增 1递减
 * 
 * @param enc 
 * @return ENC_Dir 返回编码器方向
 */
ENC_Dir FOC_ABZENC_HW_Dir(FOC_ABZENC_t *enc)
{
    ENC_Dir dir;
    if(0==READ_BIT(TIM3->CR1, TIM_CR1_DIR)){
        dir = ENC_UP;
    }else{
        dir = ENC_DOWN;
    }
    return dir;
}




/****************************************************************************************/




/***************************************霍尔编码器相关**************************************/

/**
 * @brief 霍尔编码器硬件初始化
 * 
 * @param foc_hall 
 */
//void FOC_HALL_HW_Init(FOC_HALL_t *foc_hall)
//{
//    /* 霍尔编码器 */
//    HAL_TIM_Base_Start_IT(&htim5);                          /* 开启周期中断 */
//    HAL_TIMEx_HallSensor_Start_IT(&htim5);                  /* 开启霍尔中断 */
//}

//void FOC_HALL_HW_DeInit(FOC_HALL_t *foc_hall)
//{
//    /* 霍尔编码器 */
//    HAL_TIM_Base_Stop_IT(&htim5);                          
//    HAL_TIMEx_HallSensor_Stop_IT(&htim5);   
//}


/**
 * @brief 霍尔中断处理
 * 
 * @param foc_hall 
 */
//void FOC_HALL_HW_IRQ_Handler(FOC_HALL_t *foc_hall)
//{
//    /*获取霍尔的三线状态*/
//    uint8_t HallState = HALL_U_GET 
//                        | HALL_V_GET << 1
//                        | HALL_W_GET << 2;

//    /*记录当前捕获的定时器数值*/
//    uint32_t hHighSpeedCapture = (int64_t)htim5.Instance->CCR1;                


//    /*必须在切换扇区前计算上一个扇区的补偿系数*/
//    Hall_SectorComp_Caculate(foc_hall, hHighSpeedCapture);

//    switch (HallState)
//    {
//        case 4:
//            if(foc_hall->sector_pre==6){
//                foc_hall->speed.dir = COROTATION;
//            }else if(foc_hall->sector_pre==5){
//                foc_hall->speed.dir = REVERSAL;
//            }
//            foc_hall->sector = 0;
//            break;
//        case 5:
//            if(foc_hall->sector_pre==4){
//                foc_hall->speed.dir = COROTATION;
//            }else if(foc_hall->sector_pre==1){
//                foc_hall->speed.dir = REVERSAL;
//            }
//            foc_hall->sector = 1;
//            break;
//        case 1:
//            if(foc_hall->sector_pre==5){
//                foc_hall->speed.dir = COROTATION;
//            }else if(foc_hall->sector_pre==3){
//                foc_hall->speed.dir = REVERSAL;
//            }
//            foc_hall->sector = 2;
//            break;
//        case 3:
//            if(foc_hall->sector_pre==1){
//                foc_hall->speed.dir = COROTATION;
//            }else if(foc_hall->sector_pre==2){
//                foc_hall->speed.dir = REVERSAL;
//            }
//            foc_hall->sector = 3;
//            break;
//        case 2:
//            if(foc_hall->sector_pre==3){
//                foc_hall->speed.dir = COROTATION;
//            }else if(foc_hall->sector_pre==6){
//                foc_hall->speed.dir = REVERSAL;
//            }
//            foc_hall->sector = 4;
//            break;
//        case 6:
//            if(foc_hall->sector_pre==2){
//                foc_hall->speed.dir = COROTATION;
//            }else if(foc_hall->sector_pre==4){
//                foc_hall->speed.dir = REVERSAL;
//            }
//            foc_hall->sector = 5;
//            break;
//    }
//    foc_hall->sector_pre = HallState;

//    /*角度累计, 堵转补偿 积分补偿清零*/
//    // foc_hall->Lock_ElAngle = 0.0f;
//    foc_hall->Sector_ElAngle_Sum = 0.0f;
//    foc_hall->Sector_CompIntegral = 0.0f;  

//    if(foc_hall->speed.dir==COROTATION){
//        foc_hall->Sector_ElAngle = foc_hall->sector_pos[foc_hall->sector];
//    }else{
//        foc_hall->Sector_ElAngle = foc_hall->sector_pos_c[foc_hall->sector];
//    }

//    /*存入滑动滤波器缓冲区*/
//    Move_Filter_fill(&foc_hall->Period_filter, hHighSpeedCapture);
//    /*计算扇区速度*/
//    float AvrCount = Move_Filter_calculate(&foc_hall->Period_filter);               /* 平均每个扇区的计数值 */
//    float t = AvrCount / foc_hall->hall_freq;                                       /* 一个周期的时间 */
//    foc_hall->AvrElSpeed = _PI_3/t;                                                 /* 霍尔计算的电角速度 */
//    foc_hall->AvrElSpeedDpp = foc_hall->AvrElSpeed /foc_hall->foc_freq;             /* 一个foc周期内增加的霍尔扇区内电角度*/

//   
//    /*用于调试霍尔位置偏差的*/
//    Hall_Parameter_Calculate(foc_hall, hHighSpeedCapture);
//    Hall_Parameter_Debug(foc_hall);


//    
//}



/****************************************************************************************/










/*************************************** VBUS 母线电压相关 **************************************/

/* 当前母线电压使用普通 ADC 路径，此处保留 DMA 接口便于后续硬件扩展。 */
void FOC_VBUS_HW_Init(FOC_VBUS_t *vbus)
{
    // HAL_ADC_Start_DMA(&hadc1, vbus->adc_buf, VBUS_ADC_BUF_SIZE);
}


void FOC_VBUS_HW_DeInit(FOC_VBUS_t *vbus)
{
    // HAL_ADC_Stop_DMA(&hadc1);
}

/****************************************************************************************/

















#if 0
static bool FOC_ADC_SoftwareTriggerRun(void)
{
    uint32_t timeout = 1000U;

    if (!g_foc_run_request)
    {
        return false;
    }

    g_foc_adc_tim1_sample_cnt = __HAL_TIM_GET_COUNTER(&htim1);

    __HAL_ADC_CLEAR_FLAG(&hadc1, ADC_FLAG_JEOC | ADC_FLAG_JSTRT | ADC_FLAG_OVR);
    ADC1->CR2 |= ADC_CR2_ADON;
    ADC1->CR2 |= ADC_CR2_JSWSTART;

    while (((ADC1->SR & ADC_SR_JEOC) == 0U) && (timeout > 0U))
    {
        timeout--;
    }

    if (timeout == 0U)
    {
        g_foc_adc_sw_fail_count++;
        return false;
    }

    g_foc_adc_irq_count++;
    FOC_App_Run();
    g_foc_app_run_count++;

    return true;
}
#endif
/***************************************中断相关**************************************/

/*****************ADC************************* */
#include "adc.h"
#include "foc_app.h"
#include "foc_hw.h"

///*adc的分配，adc1用于母线电压检测，adc2和adc3用于电流检测*/

///*注入转换完成中断
//整个FOC的周期是通过PWM固定的周期触发电流采样，电流采样结束后触发中断，执行每个FOC周期需要执行的任务
//*/
//bool adc_flag[3] = {0};
//void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
//{
//    #if 1
//    if(hadc==&hadc1){
//        adc_flag[0] = true;
//    }else if(hadc==&hadc2){
//        adc_flag[1] = true;
//    }else if(hadc==&hadc3){
//        adc_flag[2] = true;
//    }

//    if(adc_flag[0]==true&&adc_flag[1]==true&&adc_flag[2]==true){

//        adc_flag[0] = false;
//        adc_flag[1] = false;
//        adc_flag[2] = false;
//        FOC_App_Run();                                      /*FOC周期运行*/
//    }
//   
//    #else
//    if(hadc==&hadc2){
//        adc_flag[1] = true;
//    }else if(hadc==&hadc3){
//        adc_flag[2] = true;
//    }

//    if(adc_flag[1]==true&&adc_flag[2]==true){
//        adc_flag[1] = false;
//        adc_flag[2] = false;
//        FOC_App_Run();                                      /*FOC周期运行*/
//    }

//     #endif
//}
/* * 适配 F407 单ADC注入组序列的中断回调函数
 * ADC1 会自动按设定的 RANK 顺序转换所有通道，转换完成后触发此中断。
 */
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{
#if MOTOR_OPENLOOP_TEST

    /*
     * 当前是开环测试模式，不执行 FOC_App_Run()。
     */
    (void)hadc;

#else

    if((hadc == &hadc1) && g_foc_run_request)
    {
        static uint32_t speed_loop_div = 0U;

        g_foc_adc_tim1_sample_cnt = __HAL_TIM_GET_COUNTER(&htim1);
        g_foc_adc_irq_count++;
        FOC_App_Run();
        g_foc_app_run_count++;

        speed_loop_div++;
        if (speed_loop_div >= FOC_SPEED_LOOP_DIVIDER)
        {
            speed_loop_div = 0U;
            g_foc_speed_loop_count++;
            FOC_Motor_Speed_Calc(
                &FOC_MOTOR,
                0.00005f * (float)FOC_SPEED_LOOP_DIVIDER);
            FOC_Motor_Speed_Ctrl(&FOC_MOTOR);
        }
    }

#endif
}
/* 普通 ADC 转换完成回调：仅更新母线电压，不在此执行高频电流环。 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if(hadc==&hadc1){
        //printf("HAL_ADC_ConvCpltCallback\r\n");
        
        /*计算母线电压*/
        FOC_VBUS_Calc(&FOC_MOTOR.vbus);
    }
}
                                                           

/**********************************定时器***************************************** */
/*定时器捕获中断*/
//void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
//{
//    if(htim->Instance==TIM2){
//        /*触发了编码器Z相*/
//        FOC_ABZENC_HW_IRQHandler(&FOC_MOTOR.enc);       /*执行编码器对齐函数*/
//    }

//    if(htim->Instance==TIM5){
//        FOC_HALL_HW_IRQ_Handler(&FOC_MOTOR.hall);       /*霍尔捕获中断触发*/
//    }
//}

/*输出捕获中断*/
void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
{   
    /*运行周期为2xpwm周期 20khz*/
     if(htim->Instance==TIM1){

    }
}

/*定时器周期中断*/
/*定时器周期中断*/
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM1)
    {
        /*
         * 当前开环测试不使用 TIM1 周期中断。
         * TIM1 只负责输出三相 PWM。
         */
        return;
    }

    if (htim->Instance == TIM6)
    {
        g_foc_tim6_irq_count++;
#if MOTOR_OPENLOOP_TEST

        /*
         * 开环测试模式：
         * TIM6 频率应为 20kHz。
         * 每次中断更新一次三相正弦 PWM。
         */
        Motor_OpenLoop_Tim6Callback();

#else
        /* 闭环FOC现由TIM1-CC4同步的ADC注入转换回调驱动，不再由该自由运行定时器驱动。 */
#endif
    }
}



/****************************************************************************************/

