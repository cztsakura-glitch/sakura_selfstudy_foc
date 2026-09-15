
#ifndef _FOC_PWM_H
#define _FOC_PWM_H


#include <stdint.h>

#include "foc_type.h"


/*Foc pwm计算结构体*/
typedef struct
{
    float svpwm_val1;
    float svpwm_val2;

    float power;
    uint32_t Tpwm;                      /*定时器的周期值*/
    uint8_t sector;                     /*当前扇区*/

    float maxt;

    float K;

    qd_t Uqd;                           /*Uq Ud*/
    alphabeta_t alpha_beta;
    abc_t _output;                      /*输出到PWM*/

    abc_t T_abc;                        /*周期的占空比*/
}FOC_PWM_t;

void FOC_PWM_Init(FOC_PWM_t *foc_pwm, uint32_t Tpwm, float power);
void FOC_PWM_Run(FOC_PWM_t *foc_pwm, qd_t Uqd, float ElAngle);
void FOC_PWM_StartAll(FOC_PWM_t *foc_pwm);
void FOC_PWM_StopALL(FOC_PWM_t *foc_pwm);
void FOC_PWM_DebugSetSpwm(uint32_t use_spwm);
uint32_t FOC_PWM_DebugGetSpwm(void);





#endif
















