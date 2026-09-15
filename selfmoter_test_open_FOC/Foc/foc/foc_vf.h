#ifndef _FOC_VF_H
#define _FOC_VF_H

#include "foc_type.h"
#include "foc_filter.h"

typedef struct{
    SPEED_t speed;
    PLL_t pll;
    qd_t Uqd;           /*输出电压*/
    float run_uq;       /*VF run q-axis voltage*/
    float run_ud;       /*VF run d-axis voltage*/
    float align_ud;     /*startup alignment d-axis voltage*/
    uint32_t align_ticks;
    uint32_t align_count;

    float k1;
    float k;            /*比例*/
    float step;         /*单次步进*/
    float step_sum;
    float Ts;
    float ws;           /*角速度*/
}FOC_VF_t;


void FOC_VF_Init(FOC_VF_t *foc_vf, float pole_pairs, float Ts);
void FOC_VF_Angle_Calc(FOC_VF_t *foc_vf);
void FOC_VF_Speed_Calc(FOC_VF_t *foc_vf);










#endif


