/**
 ****************************************************************************************************
 * @file        foc_smo.c
 * @author      哔哩哔哩-Rebron大侠
 * @version     V0.0
 * @date        2025-01-11
 * @brief       滑模代码
 * @license     MIT License
 *              Copyright (c) 2025 Reborn大侠
 *              允许任何人使用、复制、修改和分发该代码，但需保留此版权声明。
 ****************************************************************************************************
 
 滑模的实质是，通过一个假定的估计反电动势, 去估算当前的电流，通过估算电流和实际电流的偏差去修正估算的反电动势，使得估算的反电动势逐渐去接近真实的反电动势
 然后通过反电动势求电角度


 */


#include "foc_smo.h"
#include "math.h"
#include "foc_math.h"

#include <stdio.h>


/**
 * @brief 饱和函数
 * 
 * @param s 
 * @param delta 
 * @return float 
 */
static float Sat(float s, float delta)
{
    if(s > delta)
        return 1;
    else if(s < -delta)
        return -1;
    else
        return s/delta;
}

/**
 * @brief 滑模初始化
 * 
 * @param foc_smo 
 * @param Rs    相电阻
 * @param Ls    相电感
 * @param Ts    运行周期
 * @param pole_pairs 极对数
 * @param max_speed 最大速度
 */
void FOC_SMO_Init(FOC_SMO_t *foc_smo, float Rs, float Ls, float Ts, uint32_t pole_pairs, float max_speed)
{
    foc_smo->Rs = Rs;
    foc_smo->Ld = Ls;
    foc_smo->Ts = Ts;

    foc_smo->speed.pole_pairs = pole_pairs;
    foc_smo->speed.Max_MecSpeed = max_speed;

    PLL_Init(&foc_smo->pll, Ts);
}

/**
 * @brief 滑模角度计算
 * 
 * @param foc_smo 
 * @param real_I 实际电流
 * @param real_U 实际电压
 */
void FOC_SMO_Angle_Calc(FOC_SMO_t *foc_smo, alphabeta_t real_I, alphabeta_t real_U)
{

    float Ld    = foc_smo->Ld;
    float Rs    = foc_smo->Rs;
    float Ts    = foc_smo->Ts;

    /* 估算的电流微分进行积分 */
    foc_smo->est_I.alpha += (real_U.alpha - real_I.alpha * Rs - foc_smo->est_E.alpha)/Ld *Ts;
    foc_smo->est_I.beta  += (real_U.beta - real_I.beta * Rs - foc_smo->est_E.beta)/Ld *Ts;

    /*计算电流误差*/
    foc_smo->I_err.alpha = foc_smo->est_I.alpha - real_I.alpha;
    foc_smo->I_err.beta = foc_smo->est_I.beta - real_I.beta;

    /*修正估算的反电动势*/
    foc_smo->est_E.alpha = foc_smo->k * Sat(foc_smo->I_err.alpha, 0.5f);
    foc_smo->est_E.beta  = foc_smo->k * Sat(foc_smo->I_err.beta, 0.5f);

    /*反正切法求*/
    // foc_smo->atan_theta = Limit_Angle(atan2f(foc_smo->est_E.beta, foc_smo->est_E.alpha));
    // foc_smo->speed.ElAngle = foc_smo->atan_theta;
    /*锁相环法求*/
    PLL_Cale_SMO(&foc_smo->pll, foc_smo->speed.dir*foc_smo->est_E.alpha, foc_smo->speed.dir*foc_smo->est_E.beta);
    foc_smo->speed.ElAngle = Limit_Angle(foc_smo->pll.Angle + (foc_smo->speed.PhaseShift *PI)*foc_smo->speed.dir);


}

/**
 * @brief 滑模速度计算
 * 
 * @param foc_smo 
 */
void FOC_SMO_Speed_Calc(FOC_SMO_t *foc_smo)
{
    Move_Filter_calculate(&foc_smo->pll.move_filter) /foc_smo->speed.pole_pairs *60.0f/_2PI;
    foc_smo->speed.AvrMecSpeed = foc_smo->pll.move_filter.val /foc_smo->speed.pole_pairs *60.0f/_2PI;

    if(foc_smo->speed.AvrMecSpeed>foc_smo->speed.Max_MecSpeed){
        printf("滑模超速\r\n");
        FOC_SMO_Clear(foc_smo);
    }

    if(fabsf(foc_smo->speed.AvrMecSpeed)<100){
        /*pll计算异常*/
        // printf("pll计算异常\r\n");
        FOC_SMO_Clear(foc_smo);
    }

    
}

/**
 * @brief 滑模参数清零
 * 
 * @param foc_smo 
 */
void FOC_SMO_Clear(FOC_SMO_t *foc_smo)
{
    foc_smo->est_I.alpha = 0;
    foc_smo->est_I.beta = 0;
    foc_smo->est_E.alpha = 0;
    foc_smo->est_E.beta = 0;

    PLL_Clear(&foc_smo->pll);
}




















