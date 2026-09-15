#ifndef _FOC_HFI_H
#define _FOC_HFI_H

#include <stdint.h>
#include "foc_type.h"
#include "foc_pid.h"
#include "foc_filter.h"
/**/

#define FOC_HFI_FREQ_1K         1000
#define FOC_HFI_FREQ_5K         5000
#define FOC_HFI_FREQ_10K        10000
#define FOC_HFI_FREQ_20K        20000



/*方波高频注入*/
typedef struct{

    bool polarityFlag;                  /*极性辨识标志*/

    float d_bias_vol;                   /*D轴偏置电压*/

    float  hfi_vol_ref;                 /*高频注入电压参考值*/
    float  hfi_vol;                     /*高频注入电压值*/
    int8_t sign;

    
    uint32_t hfi_run_freq;              /*高频注入函数的执行频率*/
    uint32_t hfi_freq;                  /*注入频率*/
    uint32_t hfi_time;                  /*注入时间*/
    uint32_t hfi_time_ref;              /*注入参考时间*/

    float polarity_ELAngle;             /*极性辨识角度*/
    float last_ElAngle;                 /*高频注入上一次计算的电角度*/
    uint32_t sum_count0;                /*极性辨识前需要等高频注入先收敛*/
    uint32_t sum_count;                 /*极性辨识的累计次数*/
    float Id_lf_sum[2];                 /*d轴低频电流累计，0:正方向注入 1:负方向注入*/

    qd_t Iqd_lf;                        /*qd轴低频电流*/
    qd_t Iqd_hf;                        /*qd轴高频电流*/
    qd_t Iqd_last;                      /*上次的qd电流*/
    alphabeta_t alphabeta_h;            /*αβ轴高频电流*/
    alphabeta_t alphabeta_f;            /*αβ轴低频电流*/
    alphabeta_t alphabeta_hlast;
    alphabeta_t alphabeta_last;

    PLL_t pll;
    SPEED_t speed;

    float Ld;
           
}FOC_HFI_t;

void FOC_HFI_InjectPolIdent(FOC_HFI_t *hfi);
void FOC_HFI_Inject(FOC_HFI_t *hfi);
void FOC_HFI_Angle_Calc(FOC_HFI_t *hfi, alphabeta_t alphabeta);
void FOC_HFI_Speed_Calc(FOC_HFI_t *hfi);
void FOC_HFI_Clear(FOC_HFI_t *hfi);
void FOC_HFI_Init(FOC_HFI_t *hfi, float hfi_vol_ref, float d_bias_vol,uint32_t hfi_run_freq, uint32_t hfi_freq, uint32_t pole_pairs);




#endif
