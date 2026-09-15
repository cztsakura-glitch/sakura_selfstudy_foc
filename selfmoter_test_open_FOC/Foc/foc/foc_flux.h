#ifndef _FOC_FLUX_H
#define _FOC_FLUX_H


#include "foc_type.h"
#include "foc_filter.h"
#include "foc_pid.h"

typedef struct{
    PLL_t pll;

    SPEED_t speed;

    alphabeta_t I_last;

    float flux;                                 /*磁链大小*/
    float G;                                    /*增益*/
    float flux_squared;                         /*磁链的平方*/
    float err;

    float eta_x1, eta_x2, eta_x1_dt, eta_x2_dt, Ls_I_alpha, Ls_I_beta;         /*用于调试的中间变量*/ 

    float flux_alpha;                           /*通过积分计算观测到的α轴的磁链*/
    float flux_beta;                            /*通过积分计算观测到的β轴的磁链*/

    float Rs;                                   /* 相电阻 */
    float Ls;                                   /* 相电感 */
    float Ts;                                   /* 周期时间：s*/

    /*有效磁链的参数*/
    alphabeta_t flux_I;                         /*电流型有效磁链*/
    alphabeta_t flux_V;                         /*电压型有效磁链*/
    FOC_PID_t   flux_alpha_pid;                     
    FOC_PID_t   flux_beta_pid;                  


}FOC_FLUX_t;




void FOC_FLUX_Init(FOC_FLUX_t *flux, float Rs, float Ls, float Ts, uint32_t pole_pairs, float max_speed);
float FOC_FLUX_Angle_Calc(FOC_FLUX_t *flux, alphabeta_t U_alphabeta, alphabeta_t I_alphabeta);
float FOC_FLUX_Calc(FOC_FLUX_t *flux, alphabeta_t U, alphabeta_t I, float ELspeed, float ElAngle);
void FOC_FLUX_Speed_Calc(FOC_FLUX_t *flux);


void FOC_FLUX_Clear(FOC_FLUX_t *flux);

void FOC_FLUX_EFFECTIVE_Angle_Calc(FOC_FLUX_t *flux, alphabeta_t U_alphabeta, alphabeta_t I_alphabeta);


#endif
