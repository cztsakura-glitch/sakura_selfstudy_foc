#ifndef _FOC_HALL_H
#define _FOC_HALL_H

#include <stdbool.h>
#include <stdint.h>
#include "foc_type.h"
#include "foc_filter.h"

#define     HALL_DEBUG  1

/*未校准的霍尔6个区域电角度*/
#define     _60_RAD            1.0471975511965977f
#define     _120_RAD           2.0943951023931954f
#define     _180_RAD           3.1415926535897932f
#define     _240_RAD           4.1887902047863909f
#define     _300_RAD           5.2359877559829887f
#define     _360_OR_0_RAD      0.0f    /*360° 或 0°*/

/********************************下面是个别电机调试出来的霍尔参数********************************************/
/* 正转 */
/*校准得到的位置*/
#define     _360_OR_0_RAD_CALIB        0.0f           /*360° 或 0°*/
#define     _60_RAD_CALIB       1.36910939f
#define     _120_RAD_CALIB      2.36427474f
#define     _180_RAD_CALIB      3.16529083f
#define     _240_RAD_CALIB      4.52005482f
#define     _300_RAD_CALIB      5.50507879f

/*扇区的大小*/
#define     HALL_SECTOR_0       1.36910939f
#define     HALL_SECTOR_1       0.995165408f
#define     HALL_SECTOR_2       0.801015973f
#define     HALL_SECTOR_3       1.3547641f
#define     HALL_SECTOR_4       0.985024035f
#define     HALL_SECTOR_5       0.778106928f

/*反转*/
/*校准得到的位置*/
#define     _360_OR_0_RAD_CALIB_C        0.0f           /*360° 或 0°*/
#define     _60_RAD_CALIB_C       1.02722335f
#define     _120_RAD_CALIB_C      1.83008254f
#define     _180_RAD_CALIB_C      3.15670538f
#define     _240_RAD_CALIB_C      4.17776346f
#define     _300_RAD_CALIB_C      4.94935322f

/*扇区的大小*/
#define     HALL_SECTOR_0_C       1.02722335f
#define     HALL_SECTOR_1_C       0.802859187f
#define     HALL_SECTOR_2_C       1.32662284f
#define     HALL_SECTOR_3_C       1.02105796f
#define     HALL_SECTOR_4_C       0.771589518f
#define     HALL_SECTOR_5_C       1.33383298f
/****************************************************************************/


/*霍尔自学习参数*/
typedef struct foc_hall
{
    uint32_t PhaseShift[2];                         /*零点偏移，放大10000*/
    uint32_t sector_size[6];                        /*每个霍尔区间的大小: 单位:弧度  放大1000000*/
    uint32_t sector_pos[6];                         /*每个霍尔区间的起始位置 放大1000000*/

    uint32_t sector_size_c[6];                      /*每个霍尔区间的大小: 单位:弧度*/
    uint32_t sector_pos_c[6];                       /*每个霍尔区间的起始位置*/

    /*放大是为了方便保存*/

}FOC_HALL_LEARN_t;


/*霍尔结构体*/
typedef struct{
    float foc_freq;                                 /*foc执行频率*/
    float hall_freq;                                /*霍尔定时器频率*/

    float Uq;

    SPEED_t speed;
    float PhaseShift;                               /*与0°之间的偏移值*/
    float PhaseShift_c;

    float Sector_ElAngle;                           /*当前的扇区起始角度*/
    float Sector_ElAngle_Sum;                       /*扇区积分的电角度*/
    float Sector_CompIntegral;                      /*扇区补偿积分*/
    float Sector_CompDifferential[6];               /*扇区补偿微分*/
    float AvrElSpeed;                               /*平均电角度转速rad/s*/
    float AvrElSpeedDpp;                            /*平均的dpp, 也就是每个foc周期内增加的电角度*/

    int8_t sector;                                  /*第几个扇区*/
    int8_t sector_pre;                              /*上一个扇区*/

    float sector_tick[6];                           /*每个扇区内的定时器滴答值*/
    float sector_size[6];                           /*每个霍尔区间的大小: 单位:弧度*/
    float sector_pos[6];                            /*每个霍尔区间的起始位置*/
    float sector_size_c[6];                         /*反向时每个霍尔区间的大小: 单位:弧度*/
    float sector_pos_c[6];                          /*反向时每个霍尔区间的起始位置*/

    /*扇区周期计算*/
    MOVE_FILTER_t Period_filter;                    /*扇区时间计算的滑动滤波器*/
    MOVE_FILTER_t Speed_filter;                     /*速度计算的滑动滤波器*/

#if HALL_DEBUG
    float ElSpeed_sector[6];                            /*每个扇区内的速度*/
    MOVE_FILTER_t hall_sector_tick_filter[6];           /*每个扇区时间的滑动滤波器*/
    float hall_sector_size_debug[6];                    /*每个霍尔区间的大小: 单位:弧度*/
    float hall_sector_pos_debug[6];                     /*每个霍尔区间的起始位置*/
#endif


    /*霍尔自学习相关*/
    bool selflearn;                                 /*自学习开关*/
    uint32_t learning_state;                        /*自学习状态机*/
    int32_t  learning_tick;                         /*自学习tick*/

    MOVE_FILTER_t pos_filter;
    FOC_HALL_LEARN_t hall_learn;

}FOC_HALL_t;

float FOC_HALL_Angle_Calc(FOC_HALL_t *foc_hall);
void FOC_Hall_Speed_Calc(FOC_HALL_t *foc_hall);
void FOC_HALL_Init(FOC_HALL_t *foc_hall, uint32_t hall_freq, uint32_t foc_freq, uint32_t pole_pairs);
void FOC_HALL_DeInit(FOC_HALL_t *foc_hall);

void Hall_SectorComp_Caculate(FOC_HALL_t *foc_hall, uint32_t Capture);
void Hall_Parameter_Debug(FOC_HALL_t *foc_hall);
void Hall_Parameter_Calculate(FOC_HALL_t *foc_hall, float hHighSpeedCapture);

#endif
