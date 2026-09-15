
#ifndef __FOC_APP_H
#define __FOC_APP_H


#include  "foc_motor.h"

/* 全局电机控制对象，保存目标值、反馈量、控制器与状态机数据。 */
extern FOC_MOTOR_t FOC_MOTOR;

/* 初始化 FOC 各子模块及默认控制参数。 */
extern void FOC_Init(void);

/* 执行一次低频应用层状态机；高频电流环不在此函数中运行。 */
extern void FOC_App_Run(void);

/* 扫描按键并调整控制模式或目标值。 */
void foc_key(void);

#endif




