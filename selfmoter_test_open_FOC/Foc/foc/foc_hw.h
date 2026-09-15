
#ifndef __FOC_HW_H
#define __FOC_HW_H

#include <stdbool.h>
#include <stdint.h>

#include <tim.h>

/*
 * FOC 硬件适配层：向算法层提供 PWM、电流采样、运行控制和调试接口。
 * 修改 MCU 引脚或定时器映射时，应优先在本层处理，避免算法代码依赖具体外设。
 */

/*
 * 编译期模式开关：不需要某种控制时改为 0，即可屏蔽对应入口。
 * 运行期可通过 mode=torque/speed/position 切换已启用的模式。
 */
#define FOC_ENABLE_VF_MODE          1
#define FOC_ENABLE_TORQUE_MODE      1
#define FOC_ENABLE_SPEED_MODE       1
#define FOC_ENABLE_POSITION_MODE    1

typedef enum
{
    FOC_CONTROL_VF = 0,
    FOC_CONTROL_TORQUE = 1,
    FOC_CONTROL_SPEED = 2,
    FOC_CONTROL_POSITION = 3
} FOC_CONTROL_MODE_t;

/* VOFA+ FireWater 曲线输出模式。 */
typedef enum
{
    FOC_VOFA_OFF = 0,
    FOC_VOFA_CURRENT = 1,
    FOC_VOFA_SPEED = 2,
    FOC_VOFA_POSITION = 3,
    FOC_VOFA_AUTO = 4
} FOC_VOFA_MODE_t;

#define FOC_DEFAULT_CONTROL_MODE    3U  /* 0=VF, 1=转矩, 2=速度, 3=位置 */

#if ((FOC_DEFAULT_CONTROL_MODE == 0U) && !FOC_ENABLE_VF_MODE) || \
    ((FOC_DEFAULT_CONTROL_MODE == 1U) && !FOC_ENABLE_TORQUE_MODE) || \
    ((FOC_DEFAULT_CONTROL_MODE == 2U) && !FOC_ENABLE_SPEED_MODE) || \
    ((FOC_DEFAULT_CONTROL_MODE == 3U) && !FOC_ENABLE_POSITION_MODE)
#error "FOC_DEFAULT_CONTROL_MODE must select an enabled mode"
#endif

/* 三相高桥 PWM 接口；phase 映射由运行时线序配置统一处理。 */
void FOC_PWM_HW_SetPhasePwm(uint8_t phase, uint32_t ccr);

#define U_H_SET_PWM(x)       FOC_PWM_HW_SetPhasePwm(0U, (uint32_t)(x))
#define V_H_SET_PWM(x)       FOC_PWM_HW_SetPhasePwm(1U, (uint32_t)(x))
#define W_H_SET_PWM(x)       FOC_PWM_HW_SetPhasePwm(2U, (uint32_t)(x))
#define U_L_SET_PWM(x)       (x)
#define V_L_SET_PWM(x)       (x)
#define W_L_SET_PWM(x)       (x)

/* 霍尔输入接口：宏返回对应 GPIO 的当前逻辑电平。 */
#define HALL_U_GET             HAL_GPIO_ReadPin(HALL_U_GPIO_Port, HALL_U_Pin)
#define HALL_V_GET             HAL_GPIO_ReadPin(HALL_V_GPIO_Port, HALL_V_Pin)
#define HALL_W_GET             HAL_GPIO_ReadPin(HALL_W_GPIO_Port, HALL_W_Pin) 


#include <foc_current.h>
/* 电流采样硬件生命周期及一次采样更新接口。 */
void FOC_CURRENT_HW_Init(void);
void FOC_CURRENT_HW_DeInit(void);
void FOC_CURRENT_Update(FOC_CURRENT_t *foc_cur);

void FOC_PWM_HW_Init(void);
void FOC_PWM_HW_DeInit(void);
void FOC_PWM_HW_ON_OFF(bool A, bool A_N, bool B, bool B_N, bool C, bool C_N);
/* 以下接口仅用于调试命令、状态观测和运行参数在线调整。 */
void FOC_Debug_PrintStatus(void);
void FOC_Debug_SetCurrentCsv(bool enable);
bool FOC_Debug_CurrentCsvEnabled(void);
void FOC_Debug_PrintCurrentCsv(void);
void FOC_Debug_SetVofaMode(uint32_t mode);
uint32_t FOC_Debug_GetVofaPeriodMs(void);
void FOC_Debug_SetRunRequest(bool enable);
bool FOC_Debug_GetRunRequest(void);
void FOC_Debug_SetAlignOnly(bool enable);
bool FOC_Debug_GetAlignOnly(void);
void FOC_Debug_SetTargetSpeed(float target_speed);
void FOC_Debug_AdjustTargetSpeed(float delta_speed);
void FOC_Debug_SetTargetPositionDeg(float target_position_deg);
void FOC_Debug_AdjustTargetPositionDeg(float delta_position_deg);
float FOC_Debug_GetTargetPositionDeg(void);
void FOC_Debug_SetVfUq(float uq_volt);
void FOC_Debug_SetVfUd(float ud_volt);
void FOC_Debug_SetPwmMode(uint32_t use_spwm);
void FOC_Debug_SetEncoderMode(uint32_t mode);
uint32_t FOC_Debug_GetEncoderMode(void);
void FOC_Debug_SetEncoderIqRef(float iq_amp);
void FOC_Debug_SetEncoderIdRef(float id_amp);
void FOC_Debug_SetEncoderSpeedIqDir(float dir);
float FOC_Debug_GetEncoderIqRef(void);
float FOC_Debug_GetEncoderIdRef(void);
float FOC_Debug_GetEncoderSpeedIqDir(void);
void FOC_Debug_SetPhaseMap(uint32_t phase_map);
uint32_t FOC_Debug_GetPhaseMap(void);

#include <foc_abzenc.h>
void FOC_ABZENC_HW_Init(FOC_ABZENC_t *enc);
void FOC_ABZENC_HW_DeInit(FOC_ABZENC_t *enc);
void FOC_ABZENC_HW_IRQHandler(FOC_ABZENC_t *enc);
uint32_t FOC_ABZENC_HW_GetCount(FOC_ABZENC_t *enc);
ENC_Dir FOC_ABZENC_HW_Dir(FOC_ABZENC_t *enc);


#include <foc_hall.h>
void FOC_HALL_HW_Init(FOC_HALL_t *foc_hall);
void FOC_HALL_HW_DeInit(FOC_HALL_t *foc_hall);
void FOC_HALL_HW_IRQ_Handler(FOC_HALL_t *foc_hall);


#include "foc_vbus.h"
void FOC_VBUS_HW_Init(FOC_VBUS_t *vbus);
void FOC_VBUS_HW_DeInit(FOC_VBUS_t *vbus);

#endif
