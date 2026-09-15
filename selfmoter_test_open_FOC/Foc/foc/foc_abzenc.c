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

#include "foc_abzenc.h"

#include "foc_math.h"
#include "foc_hw.h"

#include <stdio.h>

/**
 * @brief 编码器初始化
 * 
 * @param enc 
 * @param period 
 * @param pole_pairs 
 */
void FOC_ABZENC_Init(FOC_ABZENC_t *enc, uint32_t period, uint32_t pole_pairs)
{
    enc->period = period;
    enc->align = false;
    enc->speed.pole_pairs = pole_pairs;

    

    Move_Filter_Init(&enc->move_filter, 1);

    FOC_ABZENC_HW_Init(enc);
}

/**
 * @brief 编码器计算参数清零
 * 
 * @param enc 
 */
void FOC_ABZENC_DeInit(FOC_ABZENC_t *enc)
{
    enc->align = false;

    FOC_ABZENC_HW_DeInit(enc);
}



/**
 * @brief 计算电角度
 * 
 * @param enc 
 * @return float 返回电角度
 */
float FOC_ABZENC_Angle_Calc(FOC_ABZENC_t *enc)
{   
    float wtemp1 = 0;       /*计数值*/
    float ElAngle = 0;
    float mecAngle;         /*机械角度*/
    float period = (float)enc->period;

    wtemp1 = (float)FOC_ABZENC_HW_GetCount(enc);
    enc->dir = FOC_ABZENC_HW_Dir(enc);

    // printf("wtemp1: %d\r\n", FOC_ABZENC_HW_GetCount(enc));

    mecAngle = wtemp1/period *_2PI;
    ElAngle = _MecAngle_to_ElAngle(mecAngle, enc->speed.pole_pairs) + enc->speed.PhaseShift;

    ElAngle = Limit_Angle(ElAngle);

    if(enc->align_dir<0){
        ElAngle = _2PI-ElAngle;
    }

    enc->speed.MecAngle = mecAngle;
    enc->speed.ElAngle = Limit_Angle(ElAngle);

    return ElAngle;
}   



/**
 * @brief M法计算速度
 * 
 * @param enc 
 * @param t 该函数的调用周期: 单位s
 * @return float 
 */
float FOC_ABZENC_Speed_Calc_M(FOC_ABZENC_t *enc, float t)
{
    /*M法用编码器脉冲个数来计算, 要注意自己的硬件接口，编码器的计数方向*/

    float speed = 0;
    int32_t over_count = enc->over_count;
    enc->over_count = 0;
    int32_t sum_count = 0;

    int32_t CntCapture =  FOC_ABZENC_HW_GetCount(enc);       /*获取当前计数值*/
    ENC_Dir dir = FOC_ABZENC_HW_Dir(enc);
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

    dir_t *=  enc->align_dir>=0?1:-1;


    speed = (float)sum_count*60.0f / enc->period / t *dir_t;

    Move_Filter_fill(&enc->move_filter, speed);
    // speed = CntCapture;
    // enc->speed.AvrMecSpeed = speed;
    enc->speed.AvrMecSpeed = Move_Filter_calculate(&enc->move_filter);
    return speed;
}


bool FOC_ABZENC_Angle_Clib(FOC_ABZENC_t *enc)
{
    bool ret = false;
    float ElAngle = FOC_ABZENC_Angle_Calc(enc);

    switch(enc->calib_state){
        case 0:
            enc->calib_count = 0;
            enc->speed.PhaseShift = 0;
            enc->calib_state = 1;
            break;
        case 1:     /*触发Z相并且判断安装方向*/
            if(ENC_DOWN==FOC_ABZENC_HW_Dir(enc)){
                enc->align_dir--;
            }else{
                enc->align_dir++;
            }

            if(true==enc->align){
                enc->calib_state = 2;
                printf("编码器方向: %d\r\n", enc->align_dir>=0?1:-1);
            }

            break;
        case 2:
            enc->calib_count++;
            if(enc->calib_count>30000){
                printf("校准结束: 编码器偏移值: %f\r\n\r\n", ElAngle);
                enc->speed.PhaseShift = ElAngle *(enc->align_dir>=0?-1:1);
                // enc->speed.PhaseShift = -ElAngle;

                ret = true;
                enc->calib_state = 0;
                enc->calib_count = 0;
            }
            break;
        default:
            enc->calib_state = 0;
            enc->calib_count = 0;
            break;     
    }

    return ret;
}



