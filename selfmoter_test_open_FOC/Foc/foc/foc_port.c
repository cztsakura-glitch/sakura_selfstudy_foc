/**
 ****************************************************************************************************
 * @file        foc_port.c
 * @author      哔哩哔哩-Rebron大侠
 * @version     V0.0
 * @date        2025-01-11
 * @brief       该FOC库所调用的一些硬件接口
 * @license     MIT License
 *              Copyright (c) 2025 Reborn大侠
 *              允许任何人使用、复制、修改和分发该代码，但需保留此版权声明。
 ****************************************************************************************************
 */



#include "foc_port.h"
#include "foc_current.h"
#include "foc_app.h"

#include <bsp.h>

#include <stdlib.h>
#include <stdbool.h>
#include <stdio.h>

extern ADC_HandleTypeDef hadc1;
/**
 * @brief 打开FOC需要的硬件接口
 * 
 */
void FOC_Port_Start(void)
{
    /*PWM相关*/
    HAL_TIM_Base_Start_IT(&htim1);                          /*开启PWM的定时器周期中断*/
    HAL_TIM_OC_Start_IT(&htim1, TIM_CHANNEL_4);             /*开启通道四的触发中断*/

//    /* ABZ增量编码器 */
//    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);         /*开启定时器编码器模式*/
//    HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_2);             /*开启定时器捕获, 捕获编码器Z相*/

    /* 霍尔编码器 */
//    HAL_TIM_Base_Start_IT(&htim5);                          /* 开启周期中断 */
//    HAL_TIMEx_HallSensor_Start_IT(&htim5);                  /* 开启霍尔中断 */

    /*开启定时器的周期中断*/
//    HAL_TIM_Base_Start_IT(&htim6);                          /*开启定时器周期中断， 用于做一些速度计算之类的*/
    

   // U_H_SET_PWM(4200*0.9);
   // V_H_SET_PWM(4200*0.5);
   // W_H_SET_PWM(4200*0.3);
    FOC_Port_Pwm_Start(true, true,true,true,true,true);
}

/**
 * @brief 关闭FOC需要的硬件接口
 * 
 */
void FOC_Port_Stop(void)
{
    /*PWM相关*/
    HAL_TIM_Base_Stop_IT(&htim1);                          
    HAL_TIM_OC_Stop_IT(&htim1, TIM_CHANNEL_4);             

    /* ABZ增量编码器 */
//    HAL_TIM_Encoder_Stop(&htim3, TIM_CHANNEL_ALL);      
//    HAL_TIM_IC_Stop_IT(&htim2, TIM_CHANNEL_2);             

    /* 霍尔编码器 */
//    HAL_TIM_Base_Stop_IT(&htim5);                          
//    HAL_TIMEx_HallSensor_Stop_IT(&htim5);                  

//    HAL_TIM_Base_Stop_IT(&htim6);                          

    FOC_Port_Pwm_Stop();
}

/*打开PWM*/
/**
 * @brief 启动PWM
 * 
 * @param A    A相上管
 * @param A_N  A相下管
 * @param B    B相上管
 * @param B_N  B相下管
 * @param C    C相上管
 * @param C_N  C相下管
 */
void FOC_Port_Pwm_Start(bool A, bool A_N, bool B, bool B_N, bool C, bool C_N)
{
    MOTOR_ENABLE;                               /*半桥芯片使能 我的板子上没有这个使能引脚*/

    /*开启6路PWM*/
    
    if(A){
        HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    }else {
        HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
    }

    if(A_N){
        HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
    }else {
        HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_3);
    }

    if(B){
        HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    }else {
        HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
    }

    if(B_N){
        HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
    }else {
        HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_2);
    }

    if(C){
        HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    }else {
        HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
    }

    if(C_N){
        HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
    }else {
        HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_1);
    }

}

/**
 * @brief 停止PWM
 * 
 */
void FOC_Port_Pwm_Stop(void)
{
    MOTOR_DISENABLE;

    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
    HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_3);
}





/************************************电流采集******************************************/
#include <adc.h>

/**
 * @brief 更新电流 从注入通道获取更新后的值， 本工程用相采 有的方案是用低端电阻采样，需要根据具体的方案进行更改
 * 
 * @param foc_motor 
 */
void Foc_Current_Refresh(FOC_MOTOR_t *foc_motor)
{
    FOC_CURRENT_t *foc_cur = &foc_motor->current;

    
    switch(foc_motor->motor_state){
        case MOTOR_IDLE_START:
            /*进行偏移校准*/
            foc_cur->mcu_ad_C += HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1);
            foc_cur->mcu_ad_B += HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1);
            foc_cur->mcu_ad_A += HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1);

            /*DMA采集方式*/
            // _U_val += CC6920SO_GetAdcVal(0);         
            // _V_val += CC6920SO_GetAdcVal(1);
            // _W_val += CC6920SO_GetAdcVal(2);
   
            break;
        case MOTOR_IDENTIFY:
        case MOTOR_START:
        case MOTOR_OPEN_RUN:
        case MOTOR_HALL_RUN:
        case MOTOR_ENC_RUN:
        case MOTOR_SL_RUN:
            /*以上状态下都进行电流采样  减去零漂*/
            foc_cur->mcu_ad_C = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1) - foc_cur->_C_ad_offset;
            foc_cur->mcu_ad_B = HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_1) - foc_cur->_B_ad_offset;
            foc_cur->mcu_ad_A = HAL_ADCEx_InjectedGetValue(&hadc3, ADC_INJECTED_RANK_1) - foc_cur->_A_ad_offset;

            /*DMA采集方式*/
            // _U_val = CC6920SO_GetAdcVal(0);         
            // _V_val = CC6920SO_GetAdcVal(1);
            // _W_val = CC6920SO_GetAdcVal(2);

            /*计算三相电流*/
            foc_cur->I_abc.a = CC6920SO_CalcCur(0, foc_cur->mcu_ad_A);
            foc_cur->I_abc.b = CC6920SO_CalcCur(1, foc_cur->mcu_ad_B);
            foc_cur->I_abc.c = CC6920SO_CalcCur(2, foc_cur->mcu_ad_C);

            break;
    }

    
}

/******************************************************************************/















