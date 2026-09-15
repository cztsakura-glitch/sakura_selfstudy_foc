/**
 ****************************************************************************************************
 * @file        foc_encoder.c
 * @author      哔哩哔哩-Rebron大侠
 * @version     V0.0
 * @date        2025-01-11
 * @brief       ABZ编码器
 * @license     MIT License
 *              Copyright (c) 2025 Reborn大侠
 *              允许任何人使用、复制、修改和分发该代码，但需保留此版权声明。
 ****************************************************************************************************
 */

#include "foc_encoder.h"
#include "foc_port.h"
#include "foc_math.h"

#include <stdio.h>

/**
 * @brief 编码器初始化
 * 
 * @param enc 
 * @param period 
 * @param pole_pairs 
 */
void FOC_ENC_Init(FOC_ENC_t *enc, uint32_t period, uint32_t pole_pairs)
{
    enc->period = period;
    enc->align = false;
    enc->speed.pole_pairs = pole_pairs;

    Move_Filter_Init(&enc->move_filter, 1);

}

/**
 * @brief 编码器计算参数清零
 * 
 * @param enc 
 */
void FOC_ENC_Clear(FOC_ENC_t *enc)
{
    enc->align = false;
}


/**
 * @brief 编码器中断
 * 
 * @param enc 
 */
void FOC_ENC_IRQHandler(FOC_ENC_t *enc)
{
    if(enc->dir==ENC_DOWN){
        __HAL_TIM_SET_COUNTER(&htim3, 0);       /*校准电角度*/
    }else{
        __HAL_TIM_SET_COUNTER(&htim3, htim3.Init.Period);       /*校准电角度*/
    }
    
    /*定时器溢出了*/
    if(enc->align==true){
        enc->over_count += 1;
    }else{
        enc->align = true;
        printf("编码器已经对齐: OK\r\n");
    }
}


/**
 * @brief 获取编码器计数值
 * 
 * @return uint32_t 返回定时器滴答值
 */
uint32_t FOC_Get_ENC_Count(void)
{
    return (uint32_t)(__HAL_TIM_GET_COUNTER(&htim3));
} 

/**
 * @brief 获取编码器计数方向 0递增 1递减
 * 
 * @return ENC_Dir 返回编码器方向
 */
ENC_Dir FOC_Get_ENC_Dir(void)
{
    ENC_Dir dir;
    if(0==READ_BIT(TIM3->CR1, TIM_CR1_DIR)){
        dir = ENC_UP;
    }else{
        dir = ENC_DOWN;
    }
    return dir;
}


/**
 * @brief 计算电角度
 * 
 * @param enc 
 * @return float 返回电角度
 */
float FOC_ENC_Angle_Calc(FOC_ENC_t *enc)
{   
    float wtemp1 = 0;       /*计数值*/
    float ElAngle = 0;
    float mecAngle;         /*机械角度*/
    float period = (float)enc->period;

    wtemp1 = (float)FOC_Get_ENC_Count();
    enc->count = FOC_Get_ENC_Count();

    // if(enc->align){
    //     __HAL_TIM_SET_COUNTER(&htim3, 0);       /*校准电角度*/
    //     enc->align = 0;
    // }
    

    // printf("wtemp1 :%d     \r\n", FOC_Get_ENC_Count());

    enc->dir = FOC_Get_ENC_Dir();

    mecAngle = wtemp1/period *_2PI;
    ElAngle = _MecAngle_to_ElAngle(mecAngle, enc->speed.pole_pairs)+enc->PhaseShift;

    ElAngle = fmodf(ElAngle, _2PI);

    

    enc->speed.MecAngle = mecAngle;
    enc->speed.ElAngle = Limit_Angle(ElAngle);

    return ElAngle;
}   


/**
 * @brief T法计算速度
 * 
 * @param enc 
 * @param t 该函数的调用周期: 单位s
 * @return float 返回速度
 */
float FOC_ENC_Speed_Calc_T(FOC_ENC_t *enc, float t)
{
    /*T法计算一圈一个脉冲，低速时计算不准*/
    float speed = 0;
    int32_t over_count = enc->over_count;
    enc->over_count = 0;

    speed = (float)over_count / t *60.0f;
    enc->speed.AvrMecSpeed = speed;

    return speed;
}


/**
 * @brief M法计算速度
 * 
 * @param enc 
 * @param t 该函数的调用周期: 单位s
 * @return float 
 */
float FOC_ENC_Speed_Calc_M(FOC_ENC_t *enc, float t)
{
    /*M法用编码器脉冲个数来计算, 要注意自己的硬件接口，编码器的计数方向*/

    float speed = 0;
    int32_t over_count = enc->over_count;
    enc->over_count = 0;
    int32_t sum_count = 0;

    int32_t CntCapture =  FOC_Get_ENC_Count();       /*获取当前计数值*/
    ENC_Dir dir = FOC_Get_ENC_Dir();
    int32_t dir_t = 0;

    if(dir==ENC_UP){
        dir_t = 1;
        if(over_count>0){
            sum_count = CntCapture + (enc->period - enc->PreviousCapture);
        }else{
            sum_count = CntCapture - enc->PreviousCapture;
        }
    }else{
        dir_t = -1;
        if(over_count>0){
            sum_count = enc->PreviousCapture + (enc->period-CntCapture);
        }else{
            sum_count = enc->PreviousCapture - CntCapture;
        }
    }
    
    if(over_count>1){
        sum_count += (over_count-1)*enc->period;
    }

    enc->PreviousCapture = CntCapture;


    speed = (float)sum_count*60.0f / enc->period / t *dir_t;

    Move_Filter_fill(&enc->move_filter, speed);
    // speed = CntCapture;
    // enc->speed.AvrMecSpeed = speed;
    enc->speed.AvrMecSpeed = Move_Filter_calculate(&enc->move_filter);
    return speed;
}
