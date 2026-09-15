#ifndef _FOC_SMO_H
#define _FOC_SMO_H


#include "foc_type.h"
#include "foc_filter.h"

typedef struct{
    
    float k;                                    /*滑模系数*/
    float Ts;                                   /*周期时间:s*/
    float Ld;                                   /*d轴电感 相电感*/
    float Rs;                                   /*定子电阻，也就是相电阻*/

    PLL_t pll;
    SPEED_t speed;
    alphabeta_t I_err;                          /*估算电流和实际电流的误差*/
    alphabeta_t est_I;                          /*估算的电流*/
    alphabeta_t est_E;                          /*估算的反电动势*/
}FOC_SMO_t;


void FOC_SMO_Init(FOC_SMO_t *foc_smo, float Rs, float Ls, float Ts, uint32_t pole_pairs, float max_speed);
void FOC_SMO_Angle_Cale(FOC_SMO_t *foc_smo);

void FOC_SMO_Angle_Calc(FOC_SMO_t *foc_smo, alphabeta_t Real_I, alphabeta_t Real_U);
void FOC_SMO_Speed_Calc(FOC_SMO_t *foc_smo);
void FOC_SMO_Clear(FOC_SMO_t *foc_smo);


#endif
