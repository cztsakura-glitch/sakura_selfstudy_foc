/**
 ****************************************************************************************************
 * @file        foc_hall.c
 * @author      哔哩哔哩-Rebron大侠
 * @version     V0.0
 * @date        2025-01-11
 * @brief       霍尔编码器
 * @license     MIT License
 *              Copyright (c) 2025 Reborn大侠
 *              允许任何人使用、复制、修改和分发该代码，但需保留此版权声明。
 ****************************************************************************************************
 */




#include "foc_hall.h"
#include "foc_math.h"
#include "foc_hw.h"
#include <bsp.h>

#include <stdio.h>



/**
 * @brief 霍尔所使用的定时器的频率
 * 
 * @param foc_hall 
 * @param hall_freq 霍尔所使用的定时器的频率
 * @param foc_freq FOC运行的频率
 * @param pole_pairs 极对数
 */
void FOC_HALL_Init(FOC_HALL_t *foc_hall, uint32_t hall_freq, uint32_t foc_freq, uint32_t pole_pairs)
{
    foc_hall->hall_freq = hall_freq;  
    foc_hall->foc_freq  = foc_freq; 
    foc_hall->speed.pole_pairs = pole_pairs;

    Move_Filter_Init(&foc_hall->Period_filter, 6*4);    /*实测只有缓冲区为24时, 测出来的速度特别稳定, 多了少了都不行, 不知道为什么*/
    Move_Filter_Init(&foc_hall->Speed_filter, 1);

    Move_Filter_Init(&foc_hall->pos_filter, 100);

    /*可以选择使用默认的120°无偏差位置*/
#if 0
    foc_hall->sector_size[0] = _60_RAD;
    foc_hall->sector_size[1] = _60_RAD;
    foc_hall->sector_size[2] = _60_RAD;
    foc_hall->sector_size[3] = _60_RAD;
    foc_hall->sector_size[4] = _60_RAD;
    foc_hall->sector_size[5] = _60_RAD;

    foc_hall->sector_pos[0] = _360_OR_0_RAD;
    foc_hall->sector_pos[1] = _60_RAD;
    foc_hall->sector_pos[2] = _120_RAD;
    foc_hall->sector_pos[3] = _180_RAD;
    foc_hall->sector_pos[4] = _240_RAD;
    foc_hall->sector_pos[5] = _300_RAD;

    foc_hall->sector_size_c[0] = _60_RAD;
    foc_hall->sector_size_c[1] = _60_RAD;
    foc_hall->sector_size_c[2] = _60_RAD;
    foc_hall->sector_size_c[3] = _60_RAD;
    foc_hall->sector_size_c[4] = _60_RAD;
    foc_hall->sector_size_c[5] = _60_RAD;

    foc_hall->sector_pos_c[0] = _360_OR_0_RAD;
    foc_hall->sector_pos_c[1] = _60_RAD;
    foc_hall->sector_pos_c[2] = _120_RAD;
    foc_hall->sector_pos_c[3] = _180_RAD;
    foc_hall->sector_pos_c[4] = _240_RAD;
    foc_hall->sector_pos_c[5] = _300_RAD;

#else
    /*手动校准过的6个扇区位置*/

    foc_hall->sector_size[0] = HALL_SECTOR_0;
    foc_hall->sector_size[1] = HALL_SECTOR_1;
    foc_hall->sector_size[2] = HALL_SECTOR_2;
    foc_hall->sector_size[3] = HALL_SECTOR_3;
    foc_hall->sector_size[4] = HALL_SECTOR_4;
    foc_hall->sector_size[5] = HALL_SECTOR_5;

    foc_hall->sector_pos[0] = _360_OR_0_RAD_CALIB;
    foc_hall->sector_pos[1] = _60_RAD_CALIB;
    foc_hall->sector_pos[2] = _120_RAD_CALIB;
    foc_hall->sector_pos[3] = _180_RAD_CALIB;
    foc_hall->sector_pos[4] = _240_RAD_CALIB;
    foc_hall->sector_pos[5] = _300_RAD_CALIB;

    
    foc_hall->sector_size_c[0] = HALL_SECTOR_5_C;
    foc_hall->sector_size_c[1] = HALL_SECTOR_0_C;
    foc_hall->sector_size_c[2] = HALL_SECTOR_1_C;
    foc_hall->sector_size_c[3] = HALL_SECTOR_2_C;
    foc_hall->sector_size_c[4] = HALL_SECTOR_3_C;
    foc_hall->sector_size_c[5] = HALL_SECTOR_4_C;

    foc_hall->sector_pos_c[0] = _360_OR_0_RAD_CALIB_C;
    foc_hall->sector_pos_c[1] = _60_RAD_CALIB_C;
    foc_hall->sector_pos_c[2] = _120_RAD_CALIB_C;
    foc_hall->sector_pos_c[3] = _180_RAD_CALIB_C;
    foc_hall->sector_pos_c[4] = _240_RAD_CALIB_C;
    foc_hall->sector_pos_c[5] = _300_RAD_CALIB_C;

#endif

#if HALL_DEBUG
    uint16_t i;
    for(i=0; i<6; i++){
        Move_Filter_Init(&foc_hall->hall_sector_tick_filter[i], 120);
    }
#endif 

    FOC_HALL_HW_Init(foc_hall);

    
}


void FOC_HALL_DeInit(FOC_HALL_t *foc_hall)
{
    FOC_HALL_HW_DeInit(foc_hall);
    Move_Filter_Clear(&foc_hall->Period_filter);
    Move_Filter_Clear(&foc_hall->Speed_filter);
}


/**
 * @brief 霍尔电角度计算
 * 
 * @param foc_hall 
 * @return float 返回电角度
 */
float FOC_HALL_Angle_Calc(FOC_HALL_t *foc_hall)
{
    /*一个foc周期内加一次*/
    foc_hall->Sector_ElAngle_Sum    += foc_hall->AvrElSpeedDpp;                                         /*扇区估算角度累加*/
    foc_hall->Sector_CompIntegral   += foc_hall->Sector_CompDifferential[foc_hall->sector];             /*补偿积分先不计算*/

    float Sector_ElAngle_Sum = foc_hall->Sector_ElAngle_Sum + foc_hall->Sector_CompIntegral;

    /*估算的扇区内角度限范围, 不能超过扇区大小*/
    if(foc_hall->speed.dir==COROTATION){
        Sector_ElAngle_Sum = LIMIT_RANGE(Sector_ElAngle_Sum, 0.0f, foc_hall->sector_size[foc_hall->sector]);
    }else{
        Sector_ElAngle_Sum = LIMIT_RANGE(Sector_ElAngle_Sum, 0.0f, foc_hall->sector_size_c[foc_hall->sector]);
    }

    if(foc_hall->speed.dir == COROTATION){
        foc_hall->speed.ElAngle = foc_hall->Sector_ElAngle + Sector_ElAngle_Sum + foc_hall->PhaseShift * PI;
    }else{
        foc_hall->speed.ElAngle = foc_hall->Sector_ElAngle - Sector_ElAngle_Sum - foc_hall->PhaseShift * PI;
    }

    foc_hall->speed.ElAngle = Limit_Angle(foc_hall->speed.ElAngle);
    return foc_hall->speed.ElAngle;
}

/**
 * @brief 霍尔速度计算
 * 
 * @param foc_hall 
 */
void FOC_Hall_Speed_Calc(FOC_HALL_t *foc_hall)
{
    /*速度已经进行过滑动滤波*/
    foc_hall->speed.AvrMecSpeed = foc_hall->AvrElSpeed*60.0f / _2PI / foc_hall->speed.pole_pairs *foc_hall->speed.dir;
}

/**
 * @brief 霍尔参数调试
 * 
 * @param foc_hall 
 */
void Hall_Parameter_Debug(FOC_HALL_t *foc_hall)
{
#if HALL_DEBUG
    uint32_t i;
    for(i=0; i<6; i++){
        foc_hall->ElSpeed_sector[i] = foc_hall->sector_size[i]/(foc_hall->sector_tick[i]/foc_hall->hall_freq);
    }
#endif
}


/**
 * @brief 霍尔参数计算
 * 
 * @param foc_hall 
 * @param hHighSpeedCapture 捕获值
 */
void Hall_Parameter_Calculate(FOC_HALL_t *foc_hall, float hHighSpeedCapture)
{
#if HALL_DEBUG
    /*单个扇区的时间填充*/
    Move_Filter_fill(&foc_hall->hall_sector_tick_filter[foc_hall->sector], hHighSpeedCapture);
    Move_Filter_calculate(&foc_hall->hall_sector_tick_filter[foc_hall->sector]);

    if(foc_hall->speed.dir==COROTATION){
        foc_hall->sector_tick[0] = foc_hall->hall_sector_tick_filter[1].val;
        foc_hall->sector_tick[1] = foc_hall->hall_sector_tick_filter[2].val;
        foc_hall->sector_tick[2] = foc_hall->hall_sector_tick_filter[3].val;
        foc_hall->sector_tick[3] = foc_hall->hall_sector_tick_filter[4].val;
        foc_hall->sector_tick[4] = foc_hall->hall_sector_tick_filter[5].val;
        foc_hall->sector_tick[5] = foc_hall->hall_sector_tick_filter[0].val;
    }else{
        foc_hall->sector_tick[0] = foc_hall->hall_sector_tick_filter[0].val;
        foc_hall->sector_tick[1] = foc_hall->hall_sector_tick_filter[1].val;
        foc_hall->sector_tick[2] = foc_hall->hall_sector_tick_filter[2].val;
        foc_hall->sector_tick[3] = foc_hall->hall_sector_tick_filter[3].val;
        foc_hall->sector_tick[4] = foc_hall->hall_sector_tick_filter[4].val;
        foc_hall->sector_tick[5] = foc_hall->hall_sector_tick_filter[5].val;
    }
    
#if 1
    uint32_t i;
    float time_sum = 0.0f;
    float pos = 0.0f;
    for(i=0; i<6; i++){
        time_sum += foc_hall->sector_tick[i];
    }

    for(i=0; i<6; i++){
        foc_hall->hall_sector_size_debug[i] = foc_hall->sector_tick[i]/time_sum *_2PI;
        foc_hall->ElSpeed_sector[i] = foc_hall->hall_sector_size_debug[i]/(foc_hall->sector_tick[i]/foc_hall->hall_freq);
        foc_hall->hall_sector_pos_debug[i] = pos;
        pos += foc_hall->hall_sector_size_debug[i];
    }
#endif

#endif
}

/**
 * @brief 扇区补偿系数计算 计算每个霍尔扇区周期类的偏差, 平均到每个foc执行周期里面去
 * 
 * @param foc_hall 
 * @param Capture 捕获值
 */
void Hall_SectorComp_Caculate(FOC_HALL_t *foc_hall, uint32_t Capture)
{
    float bias = 0.0f;
    if(foc_hall->speed.dir==COROTATION){
        bias = foc_hall->sector_size[foc_hall->sector] - foc_hall->Sector_ElAngle_Sum;   /*计算每个周期的偏差*/
    }else{
        bias = foc_hall->sector_size[foc_hall->sector] - foc_hall->Sector_ElAngle_Sum;     
    }
    bias = (bias/(Capture/foc_hall->hall_freq)/foc_hall->foc_freq);                                     /*平均到每个扇区内foc的执行周期中*/
    foc_hall->Sector_CompDifferential[foc_hall->sector] = bias;
}



