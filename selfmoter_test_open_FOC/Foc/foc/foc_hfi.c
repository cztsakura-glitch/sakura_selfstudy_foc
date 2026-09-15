/**
 ****************************************************************************************************
 * @file        foc_hfi.c
 * @author      哔哩哔哩-Rebron大侠
 * @version     V0.0
 * @date        2025-01-11
 * @brief       高频注入相关代码 采用的是方波高频注入
 * @license     MIT License
 *              Copyright (c) 2025 Reborn大侠
 *              允许任何人使用、复制、修改和分发该代码，但需保留此版权声明。
 ****************************************************************************************************
 */


#include "foc_math.h"
#include "foc_hfi.h"

#include "foc_pid.h"
#include "foc_app.h"
#include "foc_filter.h"


#include <main.h>
#include <math.h>



#define D_AXLE_BIAS_VOLTAGE                 1.5f        /*用于充磁的D轴偏置电压*/


/**
 * @brief 提取qd轴低频电流分量
 * 
 * @param Iqd 这次的qd轴电流
 * @param Iqd_last 上次的qd轴电流
 * @return qd_t 返回qd轴低频电流分量
 */
qd_t FOC_HFI_Extract_LF(qd_t Iqd, qd_t Iqd_last)
{
    qd_t Iqd_lf = {0};

    Iqd_lf.q = (Iqd.q + Iqd_last.q)*0.5f;
    Iqd_lf.d = (Iqd.d + Iqd_last.d)*0.5f;

    return Iqd_lf;
}

/**
 * @brief 提取qd轴高频电流分量
 * 
 * @param Iqd 这次的qd轴电流
 * @param Iqd_last 上次的qd轴电流
 * @return qd_t 返回qd轴高频电流分量
 */
qd_t FOC_HFI_Extract_HF(qd_t Iqd, qd_t Iqd_last)
{
    qd_t Iqd_hf = {0};

    Iqd_hf.q = (Iqd.q - Iqd_last.q)*0.5f;
    Iqd_hf.d = (Iqd.d - Iqd_last.d)*0.5f;

    return Iqd_hf;
}

/**
 * @brief 提取αβ轴低频电流分量
 * 
 * @param Ialphabeta 这次的alphabeta轴电流
 * @param Ialphabeta_last 上次的alphabeta轴电流
 * @return alphabeta_t 返回αβ轴低频电流分量
 */
alphabeta_t FOC_HFI_alphabeta_LF(alphabeta_t Ialphabeta, alphabeta_t Ialphabeta_last)
{
    alphabeta_t alphabeta_lf = {0};

    alphabeta_lf.alpha = (Ialphabeta.alpha + Ialphabeta_last.alpha)*0.5f;       /*两次的高频分量相+=0*/
    alphabeta_lf.beta = (Ialphabeta.beta + Ialphabeta_last.beta)*0.5f;

    return alphabeta_lf;
}


/**
 * @brief 提取αβ轴高频电流分量
 * 
 * @param Ialphabeta 这次的alphabeta轴电流
 * @param Ialphabeta_last 上次的alphabeta轴电流
 * @return alphabeta_t 返回αβ轴高频电流分量
 */
alphabeta_t FOC_HFI_alphabeta_HF(alphabeta_t Ialphabeta, alphabeta_t Ialphabeta_last)
{
    alphabeta_t alphabeta_hf = {0};                     /*高频分量*/

    alphabeta_hf.alpha = (Ialphabeta.alpha - Ialphabeta_last.alpha)*0.5f;       /*两次的高频分量相-= 2倍绝对值*/   
    alphabeta_hf.beta = (Ialphabeta.beta - Ialphabeta_last.beta)*0.5f;


    return alphabeta_hf;
}




/**
 * @brief 高频注入 + 极性辨识 给d轴添加高频电压, 从d轴正方向注入的话 Vd的响应电流幅值应该比-Vd要大
 * 
 * @param hfi 
 */
void FOC_HFI_InjectPolIdent(FOC_HFI_t *hfi)
{
    if(hfi->hfi_time >= hfi->hfi_time_ref){
        hfi->sign = -hfi->sign;                                                     /*电压取反*/
        hfi->hfi_time = 0;
        hfi->hfi_vol = hfi->sign * hfi->hfi_vol_ref;                                /*极性辨识的时候不能充磁*/
    }else{
        hfi->hfi_time++;
    }

    /*需要高频注入角度已经收敛，且电机停止状态才能进行极性辨识*/
    if( fabsf(hfi->speed.ElAngle - hfi->last_ElAngle) <= 0.05f){
        hfi->sum_count0++;
    }else{
        hfi->sum_count0 = 0;
        hfi->sum_count = 0;
        hfi->Id_lf_sum[0] = 0;
        hfi->Id_lf_sum[1] = 0;
    }
    hfi->last_ElAngle = hfi->speed.ElAngle;

    if(hfi->sum_count0<10000)return;

    /*帕克变换 得到高频qd轴电流*/
    hfi->Iqd_hf = Foc_Park(hfi->alphabeta_h, hfi->speed.ElAngle);

    if(hfi->hfi_vol>0){                             /*上面取过一次反了，所以这里上次注入的是负电压*/     
        hfi->Id_lf_sum[1] += fabsf(hfi->Iqd_hf.d);
    }else{
        hfi->Id_lf_sum[0] += fabsf(hfi->Iqd_hf.d);
    }
    hfi->sum_count++;

    if(hfi->sum_count>=10000){
        hfi->polarityFlag = true;                   /*极性辨识已完成*/
    }
}

/**
 * @brief 高频注入
 * 
 * @param hfi 
 */
void FOC_HFI_Inject(FOC_HFI_t *hfi)
{
    if(hfi->hfi_time >= hfi->hfi_time_ref){
        hfi->sign = -hfi->sign;                                                             /*电压取反*/
        hfi->hfi_time = 0;
        hfi->hfi_vol = hfi->sign * hfi->hfi_vol_ref + hfi->d_bias_vol;                     /*高频电压  +d轴偏置*/
    }else{
        hfi->hfi_time++;
    }
}



/**
 * @brief 高频初始化
 * 
 * @param hfi 
 * @param hfi_vol_ref 注入电压幅值
 * @param d_bias_vol d轴偏置电流幅值
 * @param hfi_run_freq 高频注入函数运行的频率
 * @param hfi_freq 高频注入频率
 * @param pole_pairs 电机极对数
 */
void FOC_HFI_Init(FOC_HFI_t *hfi, float hfi_vol_ref, float d_bias_vol,uint32_t hfi_run_freq, uint32_t hfi_freq, uint32_t pole_pairs)
{
    hfi->sign = 1;
    hfi->hfi_vol_ref = hfi_vol_ref;
    hfi->hfi_run_freq = hfi_run_freq;
    hfi->hfi_freq = hfi_freq;

    hfi->hfi_time_ref = hfi_run_freq/hfi_freq/2;
    hfi->hfi_vol = hfi->hfi_vol_ref;

    hfi->d_bias_vol = d_bias_vol;

    hfi->speed.pole_pairs = pole_pairs;

    PLL_Init(&hfi->pll, 1.0f/hfi_run_freq);
    // hfi->pll.Ki *= (1.0f/hfi_run_freq) ;

    // hfi->pll.Kp = 521.0f;
    // hfi->pll.Ki = 40000.0f;
}

/*高频角度计算*/
/*高频注入获取角度*/

/**
 * @brief 高频注入获取角度
 * 
 * @param hfi 
 * @param alphabeta alphabeta轴电流
 */
void FOC_HFI_Angle_Calc(FOC_HFI_t *hfi, alphabeta_t alphabeta)
{
    alphabeta_t alphabeta_Ih = {0};     /*包络检测后的αβ*/

    int sign = 0;
    if(hfi->hfi_vol>0){
        sign = 1;
    }else{
        sign = -1;
    }

    /*上一次的高频αβ轴分量*/
    hfi->alphabeta_hlast = hfi->alphabeta_h;

    /*提取这一次的高频αβ轴分量*/
    hfi->alphabeta_h = FOC_HFI_alphabeta_HF(alphabeta, hfi->alphabeta_last);
    /*提取这一次的低频分量*/
    hfi->alphabeta_f = FOC_HFI_alphabeta_LF(alphabeta, hfi->alphabeta_last);

    hfi->alphabeta_last = alphabeta;        /*保存这次加了激励的值*/

    // /*这里求αβ轴电流的微分*/
    alphabeta_Ih.alpha = (hfi->alphabeta_h.alpha - hfi->alphabeta_hlast.alpha)*sign;
    alphabeta_Ih.beta = (hfi->alphabeta_h.beta - hfi->alphabeta_hlast.beta)*sign;

    #if 0
    /*直接反正切*/
    hfi->speed.ElAngle = Limit_Angle(atan2f(alphabeta_Ih.beta, alphabeta_Ih.alpha));

    #else
    /*高频注入锁相环*/
    PLL_Cale_HFI(&hfi->pll, alphabeta_Ih.alpha, alphabeta_Ih.beta);
    hfi->speed.ElAngle = Limit_Angle(hfi->pll.Angle);

    /*计算电感*/
    // hfi->Ld = hfi->hfi_vol *arm_cos_f32(hfi->speed.ElAngle)/ (alphabeta_Ih.alpha/hfi->hfi_run_freq);

    /*帕克变换 得到高频qd轴电流*/
    // hfi->Iqd_hf = Foc_Park(hfi->alphabeta_h, hfi->speed.ElAngle);
    // float Id_t=0;
    // if(hfi->Iqd_hf.d > hfi->Iqd_last.d){
    //     Id_t = (hfi->Iqd_hf.d - hfi->Iqd_last.d) * hfi->hfi_run_freq;
    //     hfi->Ld = hfi->hfi_vol_ref / Id_t *1000000.0f;
    // }


    // hfi->speed.ElAngle = Limit_Angle(hfi->pll.Theta + PI);
    #endif
    
}

/**
 * @brief 高频注入的转速计算
 * 
 * @param hfi 
 */
void FOC_HFI_Speed_Calc(FOC_HFI_t *hfi)
{
    hfi->speed.AvrMecSpeed = Move_Filter_calculate(&hfi->pll.move_filter) /hfi->speed.pole_pairs *60.0f/_2PI;
}


/**
 * @brief 高频参数清零
 * 
 * @param hfi 
 */
void FOC_HFI_Clear(FOC_HFI_t *hfi)
{
    PLL_Clear(&hfi->pll);
}











































