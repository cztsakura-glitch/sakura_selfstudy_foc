#ifndef _FOC_SENSORLESS_H
#define _FOC_SENSORLESS_H

#include "foc_smo.h"
#include "foc_hfi.h"
#include "foc_vf.h"
#include "foc_flux.h"

#include "foc_type.h"


/*观测器*/
typedef enum{
    VF_OBS = 0,                 /*高频注入*/
    HFI_OBS,                    /*VF*/
    SMO_OBS,                    /*滑模*/
    FLUX_OBS,                   /*磁链观测器*/
}OBSERVER_t;

/*无感状态*/
typedef enum{
   VF_STA = 0,                  /*高频注入*/
   HFI_STA,                     /*VF启动*/
   SMO_STA,                     /*滑模*/
   FLUX_STA,                    /*磁链*/
   HFI_POL_STA,                 /*高频注入极性辨识状态*/
}FOC_SENSORLESS_STA_t;


/*无传感器结构体*/
typedef struct{

    uint32_t hfi_freq;          /*高频注入频率*/
    float hfi_vol_ref;          /*高频注入电压*/
    float d_bias_vol;           /*D轴偏置电压*/
    
    bool speed_err;
    qd_t last_Iqd;
    float Iqd_over;             /*过流值*/
    float smo_time;

    float theta;

    float d_bias;               /*合成后的D轴注入电压  包含了高频注入和d轴偏置，如果是滑模或者VF状态这个值会为0*/
    float err;              

    float up_smo;               /*切入滑模的速度临界*/
    float smo_down;             /*切出滑模的速度临界*/

    int32_t dir_err;            /*无感方向错误*/
    
    
    OBSERVER_t Observer;        /*观测器*/
    FOC_SENSORLESS_STA_t sta;

    int32_t calib_time;
    int32_t up_time;              
    int32_t down_time;

    FOC_VF_t  vf;
    FOC_SMO_t smo;
    FOC_HFI_t hfi;
    FOC_FLUX_t flux;
    SPEED_t speed;
}FOC_SENSORLESS_t;


void FOC_Sensorless_Init(FOC_SENSORLESS_t *foc_sensorless, float Rs, float Ls, float freq, uint32_t pole_pairs, float Iqd_over);

alphabeta_t FOC_Sensorless_RUN(FOC_SENSORLESS_t *foc_sl, alphabeta_t I_alphabeta, alphabeta_t Last_Ualphabeta);
alphabeta_t FOC_SL_RUN(FOC_SENSORLESS_t *foc_sl, alphabeta_t I_alphabeta, alphabeta_t Last_Ualphabeta, float tatget_speed);

void FOC_Sensorless_Speed_Cale(FOC_SENSORLESS_t *foc_sl, float Ts);
bool FOC_SL_Speed_ERR(FOC_SENSORLESS_t *foc_sl, qd_t Iqd);
















#endif
