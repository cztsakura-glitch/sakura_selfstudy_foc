#ifndef _FOC_IDENTIFY_H
#define _FOC_IDENTIFY_H


#include "foc_type.h"
#include "foc_filter.h"

#include "foc_hfi.h"
#include "foc_pwm.h"

typedef enum{
    U_V_R_SET = 0,
    U_V_R_GET,
    V_W_R_SET,
    V_W_R_GET,
    W_U_R_SET,
    W_U_R_GET,
}IdenR_t;

typedef enum{
    U_V_L_SET = 0,
    U_V_L_GET,
    V_W_L_SET,
    V_W_L_GET,
    W_U_L_SET,
    W_U_L_GET,
}IdenL_t;

#define CUR_IDEN_COUNT          10000        /*电流采集1000次*/

typedef struct{

    MOVE_FILTER_t filter;       /*滑动滤波器*/

    /*直流电压注入法测电阻*/
    float Rs;                   /*相电阻*/
     
    /*阶跃信号*/
    float times;             /*电流稳定时间*/
    uint32_t cnt;               /*采集次数*/
    qd_t Iqd1;
    qd_t Iqd2;
    qd_t Iqdsum;
    qd_t Lqd_cnt;
    qd_t Lqd;


    float  Line_R;              /*线电阻*/
    float  Line_L;              /*线电感*/

    

    uint32_t Tpwm;
    float  Vbus;                /*母线电压*/
    float  Ts;                  /*周期*/

    /*测电阻*/
    IdenR_t Iden_R_State;       /*辨识电阻的状态机*/
    uint32_t Iden_R_Count;      /*电压次数*/
    float I_sum;                /*累积电流*/

    float I_sum2;                /*累积电流*/

    /*测电感*/
    IdenL_t Iden_L_State;
    float I_dt;
    float I_dt_err;
    float I_dt_last;
    float I_dt_sum;             /*电流微分*/
    float I_last;               
    float I_simple[1000];
    uint32_t Iden_L_Count;      /*累积次数*/

    /**/



    MOVE_FILTER_t I_dt_filter;

    qd_t Inmax;
    qd_t In;
    qd_t Uqd;
    FOC_HFI_t hfi;              /*用高频注入来计算电感*/

}FOC_IDENTIFY_t;


void FOC_Identify_Init(FOC_IDENTIFY_t *iden, uint32_t Tpwm, float Vbus, float Ts);
bool FOC_Identify_R(FOC_IDENTIFY_t *iden, float U, abc_t I);
bool FOC_Identify_L(FOC_IDENTIFY_t *iden, float U, abc_t Iabc);

bool FOC_Identify_L_QD(FOC_IDENTIFY_t *iden, float U, abc_t I, FOC_PWM_t *foc_pwm);

bool FOC_Identify_R_QD(FOC_IDENTIFY_t *iden, float U, abc_t I, FOC_PWM_t *foc_pwm);

void FOC_DC_V_Identify_R(FOC_IDENTIFY_t *iden, float U, abc_t I);


bool FOC_Identify_ByStep_L_QD(FOC_IDENTIFY_t *iden, float U, abc_t I, FOC_PWM_t *foc_pwm);

#endif
