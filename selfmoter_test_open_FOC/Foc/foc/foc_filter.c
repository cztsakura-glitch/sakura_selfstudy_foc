/**
 ****************************************************************************************************
 * @file        foc_filter.c
 * @author      哔哩哔哩-Rebron大侠
 * @version     V0.0
 * @date        2025-01-11
 * @brief       滤波器相关
 * @license     MIT License
 *              Copyright (c) 2025 Reborn大侠
 *              允许任何人使用、复制、修改和分发该代码，但需保留此版权声明。
 ****************************************************************************************************
 */

#include "foc_filter.h"
#include "foc_math.h"

#include <arm_math.h>

/**
 * @brief PLL初始化
 * 
 * @param pll 
 * @param Ts 运行的周期
 */
void PLL_Init(PLL_t *pll, float Ts)
{
    pll->Ts = Ts;

    Move_Filter_Init(&pll->move_filter, 300);
}

/**
 * @brief PLL参数清除
 * 
 * @param pll 
 */
void PLL_Clear(PLL_t *pll)
{
    pll->Angle_Err=0;
    pll->P=0;
    pll->I=0;
    pll->Omega = 0;
    pll->Omega_F=0;

    Move_Filter_Clear(&pll->move_filter);
}

/*高频注入锁相环*/

/**
 * @brief 高频注入提取角度锁相环
 * 
 * @param pll   pll结构体
 * @param alpha alpha轴高频电流
 * @param beta beta轴高频电流
 */
void PLL_Cale_HFI(PLL_t *pll, float alpha, float beta)
{
    float _cos, _sin;

    _sin = arm_sin_f32(pll->Angle);
    _cos = arm_cos_f32(pll->Angle);

    pll->Angle_Err = -alpha * _sin + beta * _cos;
    pll->P = pll->Kp * pll->Angle_Err;
    pll->I += pll->Ki * pll->Angle_Err;

    pll->Omega = pll->P + pll->I;

    pll->Omega_F = pll->Omega_F*0.9f + pll->Omega *0.1f;


    pll->Angle += pll->Omega_F * pll->Ts;
    pll->Angle = Limit_Angle(pll->Angle);
    Move_Filter_fill(&pll->move_filter, pll->Omega_F);
}


/**
 * @brief 滑模观测器锁相环提取角度
 * 
 * @param pll 
 * @param alpha alpha轴反电动势
 * @param beta beta反电动势
 */
void PLL_Cale_SMO(PLL_t *pll, float alpha, float beta)
{
    float _cos, _sin;
    _sin = arm_sin_f32(pll->Angle);
    _cos = arm_cos_f32(pll->Angle);

    pll->Angle_Err = -alpha * _cos - beta * _sin;


    
    pll->P = pll->Kp * pll->Angle_Err;
    pll->I += pll->Ki * pll->Angle_Err * pll->Ts;

    pll->Omega = pll->P + pll->I;

    pll->Omega_F = pll->Omega_F*0.9f + pll->Omega *0.1f;
    // pll->Omega_F = pll->Omega;

    pll->Angle += pll->Omega_F * pll->Ts;
    pll->Angle = Limit_Angle(pll->Angle);
    Move_Filter_fill(&pll->move_filter, pll->Omega_F);
}

/*霍尔锁相环*/
/**
 * @brief 霍尔编码器锁相环
 * 
 * @param pll 
 * @param input_theta 输入的角度
 * @param input_speed 输入的速度
 */
void PLL_Calc_HALL(PLL_t *pll, float input_theta, float input_speed)
{
    pll->Angle_Err = Limit_Angle(input_theta - pll->Angle);

    if(pll->Angle_Err>=PI){
        pll->Angle_Err -= _2PI;
    }
    if(pll->Angle_Err<=-PI){
        pll->Angle_Err += _2PI;
    }

    pll->Omega += pll->Ki * pll->Angle_Err;

    pll->Angle +=  (pll->Omega + pll->Angle_Err*pll->Kp)*pll->Ts;

    pll->Angle = Limit_Angle(pll->Angle);

    pll->Ki = 0.236f*input_speed*input_speed;
    arm_sqrt_f32(pll->Ki, &pll->Kp);

    pll->Kp *= 1.414f;
    pll->Ki *= pll->Ts;
}

/**
 * @brief 滑动滤波初始化
 * 
 * @param filter 
 * @param buf_size 滤波器的buf大小
 */
void Move_Filter_Init(MOVE_FILTER_t *filter, uint32_t buf_size)
{
    uint16_t i;
    if(buf_size>MOVE_FILTER_BUF_MAXSIZE)buf_size=MOVE_FILTER_BUF_MAXSIZE;

    filter->buf_size = buf_size;

    for(i=0; i<buf_size;i++){
        filter->buf[i] = 0;
    }
}

/**
 * @brief 清除滑动滤波器计算参数
 * 
 * @param filter 
 */
void Move_Filter_Clear(MOVE_FILTER_t *filter)
{
    uint32_t i;

    filter->buf_fill = 0;
    filter->buf_sum = 0;
    filter->buf_Index = 0;
    
    for(i=0;i<MOVE_FILTER_BUF_MAXSIZE;i++){
        filter->buf[i] = 0;
    }

    filter->val = 0;
}

/**
 * @brief 填充滑动滤波器
 * 
 * @param filter 
 * @param val 填充值
 */
void Move_Filter_fill(MOVE_FILTER_t *filter, int64_t val)
{
    filter->buf_sum -= filter->buf[filter->buf_Index];
    filter->buf[filter->buf_Index] = val;
    filter->buf_sum += filter->buf[filter->buf_Index];

    filter->buf_Index++;
    if(filter->buf_Index>=filter->buf_size){
        filter->buf_Index = 0;
    }

    if(filter->buf_fill<filter->buf_size){
        filter->buf_fill++;
    }
    
}

/**
 * @brief 滑动滤波器计算
 * 
 * @param filter 
 * @return float 计算的滑动平均值
 */
float Move_Filter_calculate(MOVE_FILTER_t *filter)
{   
    float buf_sum = (float)filter->buf_sum;

    if(filter->buf_fill==0)return 0;

    if(filter->buf_fill < filter->buf_size){
        buf_sum = buf_sum/(float)filter->buf_fill;
    }else{
        buf_sum = buf_sum/(float)filter->buf_size;
    }

    filter->val = buf_sum;
    return buf_sum;
} 
 


