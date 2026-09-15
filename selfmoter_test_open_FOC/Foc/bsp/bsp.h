#ifndef _BSP_H
#define _BSP_H

#include "sys.h"
#include "delay.h"
#include "my_usart.h"

/* 板级公共接口：初始化外设，并把按键/串口命令转换为 FOC 控制请求。 */
#include "./cc6920so/cc6920so.h"



void BSP_Init(void);
void BSP_DebugUartPollCommands(void);
void BSP_KeyPoll(void);
void BSP_DebugUartIrqHandler(USART_TypeDef *uart);

/* 转矩闭环：iq_amp 为 q 轴目标电流，单位 A。 */
void BSP_FOC_StartTorque(float iq_amp);
/* 速度闭环：target_rpm 为转速目标，iq_limit_amp 为允许的最大转矩电流。 */
void BSP_FOC_StartSpeed(float target_rpm, float iq_limit_amp);
/* 位置闭环：target_deg 为可多圈机械角度，iq_limit_amp 为最大转矩电流。 */
void BSP_FOC_StartPosition(float target_deg, float iq_limit_amp);
/* 安全停止 PWM 输出，并清除本次运行请求。 */
void BSP_FOC_Stop(void);
  
#endif

