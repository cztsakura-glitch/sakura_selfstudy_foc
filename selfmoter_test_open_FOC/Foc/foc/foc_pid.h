
#ifndef __PID_H
#define __PID_H

#include <stdbool.h>

typedef struct{

    float ratio;
    float ratio_limit;

    volatile float  SetPoint;           /* 设定目标 */
    volatile float  SetPoint_limit;     /* 设置目标值限制 */
    volatile float  In;                 /* 输入值 */
    volatile float  ActualValue;        /* 实际值 */
    volatile float  ActualValue_limit;  /* 输出实际值限制 */
    volatile float  SumError;           /* 误差累计 */
    volatile float  SumError_limit;     /* 误差饱和 */
    volatile float  Up;                 /* 比例项 */
    volatile float  Ui;                 /* 积分项 */
    volatile float  Ud;                 /* 微分项 */
    volatile float  P;                  /* 比例常数 P */
    volatile float  I;                  /* 积分常数 I */
    volatile float  D;                  /* 微分常数 D */
    volatile float  Error;              /* Error[-1] */
    volatile float  LastError;          /* Error[-1] */
    volatile float  PrevError;          /* Error[-2] */
    volatile float  IngMin;             /*积分最小值*/
    volatile float  IngMax;             /*积分最大值*/
    volatile float  OutMin;             /*输出最小值*/
    volatile float  OutMax;             /*输出最大值*/

    volatile float  Ts;                 /*PID执行周期 单位:s*/
    volatile float  change_limit;       /*变化限制值*/

    volatile bool flag;

}FOC_PID_t;

typedef struct{
    float point;      /* 目标值 */
    float P;
    float I;
    float D;

    float steady_coefficient;   /*稳态系数*/
    float ratio_limit;
}FOC_PID_PARAMETER_t;





void PID_Init(FOC_PID_t *foc_pid, float pid_freq);
void PID_Clear(FOC_PID_t *foc_pid);
void PID_Parameter_Set(FOC_PID_t *foc_pid);
float PID_Increment_ctrl(FOC_PID_t *foc_pid, float real_val);
float PID_Position_ctrl(FOC_PID_t *foc_pid, float real_val);



#endif
