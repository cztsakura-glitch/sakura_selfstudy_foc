/* 调试记录
 * 
 * 电流的采样, 如果不做一阶滤波的话, 会非常多的尖刺, 做了一阶滤波之后, 空载下的电流采集都为0
 * 修改
 * 
*/


#ifndef _FOC_CURRENT_H
#define _FOC_CURRENT_H

#include <stdint.h>
#include <stdbool.h>
#include "foc_type.h"


#define CALIBRATION_TIMES       1000U                   /*AD校准时采集次数*/

/*电流状态*/
typedef enum{
    START_CALIB = 0,        /*启动校准*/
    CAP_CAHGE,              /*电容充电*/
    CALIBING,               /*校准中*/
}FOC_CUR_STATE_t;


/*电流结构体*/
typedef struct 
{
    /*********************采集相关******************************/
    float over;                           /*过流值*/
    

    /*********************校准相关************************/
    abc_t ad_offset;                            /*电流AD偏置 ad采集值*/
    abc_t ad_offset_temp;                       /*临时值*/
    uint32_t ad_offset_count;                   /*偏置采集次数*/ 

    abc_t mcu_ad;                               /*MCU端口的AD值*/
    
    abc_t I_abc;                                /*采集转化的三相电流值*/
    qd_t I_qd;                                  /*采样转化qd轴电流值*/

    FOC_CUR_STATE_t state;                      /*状态*/


    uint32_t cur_over_count;

}FOC_CURRENT_t;


void FOC_CURRENT_Init(FOC_CURRENT_t *foc_cur, float over);
void FOC_CURRENT_DeInit(FOC_CURRENT_t *foc_cur);
int FOC_CURRENT_OffsetCalc(FOC_CURRENT_t *foc_cur);
bool FOC_CURRENT_Over(FOC_CURRENT_t *foc_cur, qd_t I_qd);












#endif
