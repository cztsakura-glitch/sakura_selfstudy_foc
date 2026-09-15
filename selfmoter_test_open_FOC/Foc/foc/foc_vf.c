/**
 ****************************************************************************************************
 * @file        foc_vf.c
 * @author      哔哩哔哩-Rebron大侠
 * @version     V0.0
 * @date        2025-01-11
 * @brief       VF强拖
 * @license     MIT License
 *              Copyright (c) 2025 Reborn大侠
 *              允许任何人使用、复制、修改和分发该代码，但需保留此版权声明。
 ****************************************************************************************************
 */

#include "foc_vf.h"
#include "foc_math.h"

#define FOC_VF_STEP_RAMP_PER_TICK  0.00000010f

/**
 * @brief VF初始化
 * 
 * @param foc_vf 
 * @param pole_pairs 极对数
 * @param Ts    运行周期
 */
void FOC_VF_Init(FOC_VF_t *foc_vf, float pole_pairs, float Ts)
{
    foc_vf->Ts = Ts;
    foc_vf->speed.pole_pairs = pole_pairs;

    if ((foc_vf->run_uq > -0.01f) && (foc_vf->run_uq < 0.01f))
    {
        foc_vf->run_uq = foc_vf->Uqd.q;
        if ((foc_vf->run_uq > -0.01f) && (foc_vf->run_uq < 0.01f))
        {
            foc_vf->run_uq = 0.30f;
        }
    }

    if ((foc_vf->run_ud > -0.01f) && (foc_vf->run_ud < 0.01f))
    {
        foc_vf->run_ud = 0.10f;
    }

    if ((foc_vf->align_ud > -0.01f) && (foc_vf->align_ud < 0.01f))
    {
        foc_vf->align_ud = foc_vf->Uqd.d;
        if ((foc_vf->align_ud > -0.01f) && (foc_vf->align_ud < 0.01f))
        {
            foc_vf->align_ud = foc_vf->run_uq;
        }
    }

    if ((foc_vf->align_ticks == 0U) && (Ts > 0.0f))
    {
        foc_vf->align_ticks = (uint32_t)(0.5f / Ts);
    }
}

/**
 * @brief VF角度计算
 * 
 * @param foc_vf 
 */
void FOC_VF_Angle_Calc(FOC_VF_t *foc_vf)
{
    float target_step;
    float step_err;

    if (foc_vf->align_count < foc_vf->align_ticks)
    {
        foc_vf->align_count++;
        foc_vf->Uqd.q = 0.0f;
        foc_vf->Uqd.d = foc_vf->align_ud;
        foc_vf->step = 0.0f;
        foc_vf->step_sum = 0.0f;
        foc_vf->speed.ElAngle = 0.0f;
        foc_vf->speed.AvrMecSpeed = 0.0f;
        return;
    }

    if (foc_vf->align_count == foc_vf->align_ticks)
    {
        foc_vf->align_count++;
        foc_vf->step = 0.0f;
        foc_vf->step_sum = 0.0f;
        foc_vf->speed.ElAngle = Limit_Angle(-_PI_2);
    }

    foc_vf->Uqd.q = foc_vf->run_uq;
    foc_vf->Uqd.d = foc_vf->run_ud;
    target_step = foc_vf->run_uq * foc_vf->k;
    step_err = target_step - foc_vf->step_sum;

    if (step_err > FOC_VF_STEP_RAMP_PER_TICK)
    {
        foc_vf->step_sum += FOC_VF_STEP_RAMP_PER_TICK;
    }
    else if (step_err < -FOC_VF_STEP_RAMP_PER_TICK)
    {
        foc_vf->step_sum -= FOC_VF_STEP_RAMP_PER_TICK;
    }
    else
    {
        foc_vf->step_sum = target_step;
    }

    foc_vf->step = foc_vf->step_sum;
    foc_vf->speed.ElAngle = Limit_Angle(foc_vf->speed.ElAngle + foc_vf->step);
}
/**
 * @brief VF速度计算
 * 
 * @param foc_vf 
 */
void FOC_VF_Speed_Calc(FOC_VF_t *foc_vf) 
{
    float ws = foc_vf->step/foc_vf->Ts /foc_vf->speed.pole_pairs *60 /_2PI;

    if(isnan(ws) || isinf(ws)){
        return;
    }
    foc_vf->speed.AvrMecSpeed = ws;
}


