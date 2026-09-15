/**
 ****************************************************************************************************
 * @file        foc_smo.c
 * @author      哔哩哔哩-Rebron大侠
 * @version     V0.0
 * @date        2025-01-11
 * @brief       非线性磁链观测器
 * @license     MIT License
 *              Copyright (c) 2025 Reborn大侠
 *              允许任何人使用、复制、修改和分发该代码，但需保留此版权声明。
 ****************************************************************************************************
 */


#include "foc_flux.h"
#include "foc_math.h"
#include <stdio.h>


/**
 * @brief 磁链初始化
 * 
 * @param flux 
 * @param Rs 相电阻
 * @param Ls 相电感
 * @param Ts 运行周期
 * @param pole_pairs 极对数
 * @param max_speed 最大速度
 */
void FOC_FLUX_Init(FOC_FLUX_t *flux, float Rs, float Ls, float Ts, uint32_t pole_pairs, float max_speed)
{
    flux->Rs = Rs;
    flux->Ls = Ls;
    flux->Ts = Ts;

    flux->flux_squared = flux->flux * flux->flux;

    flux->speed.pole_pairs = pole_pairs;
    flux->speed.Max_MecSpeed = max_speed;
    PLL_Init(&flux->pll, Ts);

    /*酷飞电机*/
    float kp = 4*PI  *1.5f *5 /2;
    float ki = 4*PI*PI   *5*5 /(2*2);

    flux->flux_alpha_pid.SetPoint = 0.00001f;
    flux->flux_alpha_pid.P = kp;
    flux->flux_alpha_pid.I = ki;
    flux->flux_alpha_pid.SumError_limit = 50000000000.0f;
    flux->flux_alpha_pid.SetPoint_limit = 100.0f;          
    flux->flux_alpha_pid.ActualValue_limit = 100.0f;

    flux->flux_beta_pid.SetPoint = 0.00001f;
    flux->flux_beta_pid.P = kp;
    flux->flux_beta_pid.I = ki;
    flux->flux_beta_pid.SumError_limit = 50000000000.0f;
    flux->flux_beta_pid.SetPoint_limit = 100.0f;              
    flux->flux_beta_pid.ActualValue_limit = 100.0f;







}


/**
 * @brief 磁链观测器 PLL正交锁相环
 * 
 * @param pll 
 * @param alpha α轴磁链
 * @param beta  β轴磁链
 * @return float 返回电角度
 */
float FOC_FLUX_Angle_PLL(PLL_t *pll, float alpha, float beta)
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

    return pll->Angle;
}

/**
 * @brief 磁链观测器电角度计算
 * 
 * @param flux 
 * @param U_alphabeta αβ轴电压
 * @param I_alphabeta αβ轴电流
 * @return float 返回电角度
 */
float FOC_FLUX_Angle_Calc(FOC_FLUX_t *flux, alphabeta_t U_alphabeta, alphabeta_t I_alphabeta)
{
    float ElAngle = 0.0f;
    float Rs = flux->Rs;                                                    /*定子电阻*/
    float Ts = flux->Ts;                                                    /*磁链计算的周期*/

    float Ls_I_alpha = flux->Ls * I_alphabeta.alpha;                        /*alpha电流和定子电感的乘积*/
    float Ls_I_beta = flux->Ls * I_alphabeta.beta;                          /*beta电流和定子电感的乘积*/

    float x1 = flux->flux_alpha;                                            /**/
    float x2 = flux->flux_beta;

    float eta_x1 = x1-Ls_I_alpha;                                           /*观测的定子磁链*/
    float eta_x2 = x2-Ls_I_beta;

    float y1 = -Rs *I_alphabeta.alpha + U_alphabeta.alpha;                  /*(永磁体磁链+电流磁链)的微分*/
    float y2 = -Rs *I_alphabeta.beta + U_alphabeta.beta;

    float err = flux->flux_squared - (eta_x1*eta_x1 + eta_x2*eta_x2);       /*误差状态收敛*/
    err = LIMIT_RANGE(err, -flux->flux, flux->flux);                        /*误差限制幅度*/

    float eta_x1_dt     = y1 + flux->G * eta_x1 * err;                      /*(永磁体磁链+电流磁链)的微分+增益*/
    float eta_x2_dt     = y2 + flux->G * eta_x2 * err;

    flux->flux_alpha    += eta_x1_dt * Ts;                                  /*通过积分计算观测到的α轴的磁链*/
    flux->flux_beta     += eta_x2_dt * Ts;                                  /*通过积分计算观测到的β轴的磁链*/

    /*参数保存*/
    flux->err = err;
    flux->eta_x1 = eta_x1;
    flux->eta_x2 = eta_x2;
    flux->eta_x1_dt = eta_x1_dt;
    flux->eta_x2_dt = eta_x2_dt;
    flux->Ls_I_alpha = Ls_I_alpha;
    flux->Ls_I_beta = Ls_I_beta;


    float flux_alpha = flux->flux_alpha-Ls_I_alpha;                         /*α轴的永磁体磁链*/
    float flux_beta = flux->flux_beta-Ls_I_beta;                            /*β轴的永磁体磁链*/

    /*反正切计算电角度*/
    #if 0
    // ElAngle = atan2f(flux->flux_beta-Ls_I_beta,  flux->flux_alpha-Ls_I_alpha);
    #else
    /*PLL求电角度*/
    ElAngle = FOC_FLUX_Angle_PLL(&flux->pll, flux_alpha, flux_beta);
    #endif

    /*电角度限幅*/
    flux->speed.ElAngle = Limit_Angle(ElAngle + (flux->speed.PhaseShift *PI)*flux->speed.dir);

    return ElAngle;
}

/*磁链估计*/
/**
 * @brief 磁链估计
 * 
 * @param flux 
 * @param U 电压
 * @param I 电流
 * @param ELspeed 电角速度
 * @param ElAngle 电角度
 * @return float 返回磁链
 */
float FOC_FLUX_Calc(FOC_FLUX_t *flux, alphabeta_t U, alphabeta_t I, float ELspeed, float ElAngle)
{
	return flux->flux;
}

/**
 * @brief 磁链速度计算
 * 
 * @param flux 
 */
void FOC_FLUX_Speed_Calc(FOC_FLUX_t *flux)
{
    Move_Filter_calculate(&flux->pll.move_filter);

    flux->speed.AvrMecSpeed = flux->pll.move_filter.val /flux->speed.pole_pairs *60.0f/_2PI;

    if(flux->speed.AvrMecSpeed>flux->speed.Max_MecSpeed){
        printf("磁链超速\r\n");
        FOC_FLUX_Clear(flux);
    }

    if(flux->speed.AvrMecSpeed>=0){
        flux->speed.dir = COROTATION;
    }else{
        flux->speed.dir = REVERSAL;
    }
}

/**
 * @brief 磁链参数清零
 * 
 * @param flux 
 */
void FOC_FLUX_Clear(FOC_FLUX_t *flux)
{
    flux->flux_alpha = 0;
    flux->flux_beta = 0;

    PLL_Clear(&flux->pll);
}




/**************************************************下面的是有效磁链相关的代码******************************************************** */


/**
 * @brief 电流型磁链计算
 * 
 * @param flux 
 * @param Iqd qd轴电流
 */
void FOC_FLUX_I_Calc(FOC_FLUX_t *flux, qd_t Iqd)
{   
    float ElAngle = flux->speed.ElAngle;
    float _cos = arm_cos_f32(ElAngle);
    float _sin = arm_sin_f32(ElAngle);

    qd_t flux_qd = {0};
    alphabeta_t flux_alphabeta = {0};
    flux_qd.d =  flux->flux + flux->Ls * Iqd.d;              /*d轴的定子磁链*/
    flux_qd.q =  flux->Ls * Iqd.q;                           /*q轴的定子磁链*/

    flux_alphabeta.alpha    = flux_qd.d * _cos - flux_qd.q *_sin;
    flux_alphabeta.beta     = flux_qd.d * _sin - flux_qd.q *_cos;

    flux->flux_I = flux_alphabeta;
}

/**
 * @brief 电压型磁链计算
 * 
 * @param flux 
 * @param U_alphabeta αβ轴电压
 * @param I_alphabeta αβ轴电流
 */
void FOC_FLUX_V_Calc(FOC_FLUX_t *flux, alphabeta_t U_alphabeta, alphabeta_t I_alphabeta)
{
    float ElAngle = 0.0f;
    float Rs = flux->Rs;                                                    /*定子电阻*/
    float Ts = flux->Ts;                                                    /*磁链计算的周期*/

    float Ls_I_alpha = flux->Ls * I_alphabeta.alpha;                        /*alpha电流和定子电感的乘积*/
    float Ls_I_beta = flux->Ls * I_alphabeta.beta;                          /*beta电流和定子电感的乘积*/



    float y1 = -Rs *I_alphabeta.alpha + U_alphabeta.alpha;                  /*(永磁体磁链+电流磁链)的微分 也就是反电动势*/
    float y2 = -Rs *I_alphabeta.beta + U_alphabeta.beta;

    /*加上PID计算的增益*/
    y1 += flux->flux_alpha_pid.ActualValue;
    y2 += flux->flux_beta_pid.ActualValue;

    flux->flux_V.alpha  += y1 * Ts;                                         /*通过积分计算观测到的α轴的总磁链*/
    flux->flux_V.beta   += y2 * Ts;                                         /*通过积分计算观测到的β轴的总磁链*/

    /*计算PID增益*/
    PID_Position_ctrl(&flux->flux_alpha_pid,  flux->flux_V.alpha - flux->flux_I.alpha);
    PID_Position_ctrl(&flux->flux_beta_pid,  flux->flux_V.beta - flux->flux_I.beta);

    float flux_alpha = flux->flux_V.alpha-Ls_I_alpha;                         /*α轴的永磁体磁链*/
    float flux_beta = flux->flux_V.beta-Ls_I_beta;                            /*β轴的永磁体磁链*/

    /*反正切计算电角度*/
    #if 0
    ElAngle = atan2f(flux_beta,  flux_alpha);
    #else
    /*PLL求电角度*/
    ElAngle = FOC_FLUX_Angle_PLL(&flux->pll, flux_alpha, flux_beta);
    #endif

    /*电角度限幅*/
    flux->speed.ElAngle = Limit_Angle(ElAngle + (flux->speed.PhaseShift *PI)*flux->speed.dir);
}


/**
 * @brief 有效磁链计算电角度
 * 
 * @param flux 
 * @param U_alphabeta αβ轴电压
 * @param I_alphabeta αβ轴电流
 */
void FOC_FLUX_EFFECTIVE_Angle_Calc(FOC_FLUX_t *flux, alphabeta_t U_alphabeta, alphabeta_t I_alphabeta)
{
    /* 这个函数会占用比较多的时间, 注意电流环带宽 */
    float ElAngle = flux->speed.ElAngle;
    qd_t Iqd = {0};
    
    /*计算估测的qd轴电流*/
    Iqd = Foc_Park(I_alphabeta, ElAngle);

    /*计算电流型磁链*/
    FOC_FLUX_I_Calc(flux, Iqd);

    /*通过电流型磁链修正电压型号磁链，并计算出电角度*/
    FOC_FLUX_V_Calc(flux, U_alphabeta, I_alphabeta);
}




















