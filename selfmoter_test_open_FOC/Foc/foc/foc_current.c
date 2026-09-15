/**
 ****************************************************************************************************
 * @file        foc_current.c
 * @author      哔哩哔哩-Rebron大侠
 * @version     V0.0
 * @date        2025-01-11
 * @brief       电流相关
 * @license     MIT License
 *              Copyright (c) 2025 Reborn大侠
 *              允许任何人使用、复制、修改和分发该代码，但需保留此版权声明。
 ****************************************************************************************************
 */

#include "foc_current.h"
#include "foc_math.h"


#include <adc.h>

#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <bsp.h>
#include <delay.h>
#include <foc_motor.h>

#include <foc_hw.h>



/**
 * @brief 电流结构体初始化
 * 
 * @param foc_cur 
 */

/**
 * @brief 电流结构体初始化
 * 
 * @param foc_cur 
 * @param over 电流过流值
 */
void FOC_CURRENT_Init(FOC_CURRENT_t *foc_cur, float over)
{
    FOC_CURRENT_HW_Init();

    memset(&foc_cur->ad_offset, 0x00, sizeof(foc_cur->ad_offset));
    memset(&foc_cur->ad_offset_temp, 0x00, sizeof(foc_cur->ad_offset));
    foc_cur->ad_offset_count = 0;

    foc_cur->over = over;

}

void FOC_CURRENT_DeInit(FOC_CURRENT_t *foc_cur)
{
    FOC_CURRENT_HW_DeInit();

    memset(&foc_cur->ad_offset, 0x00, sizeof(foc_cur->ad_offset));
    memset(&foc_cur->ad_offset_temp, 0x00, sizeof(foc_cur->ad_offset));

    foc_cur->ad_offset_count = 0;
}

/*电流偏置*/
/**
 * @brief 计算静态时电流偏置
 * 
 * @param foc_cur 
 * @return int 返回0计算完成，其他失败
 */
int FOC_CURRENT_OffsetCalc(FOC_CURRENT_t *foc_cur)
{
    /* 驱动板零电流偏置为1.24V：1.24 / 3.3 * 4095 = 1539 counts。 */
    const float adc_mid = 2048.0f;
    const float max_zero_deviation = 300.0f;

    foc_cur->ad_offset_temp.a += foc_cur->mcu_ad.a;
    foc_cur->ad_offset_temp.b += foc_cur->mcu_ad.b;
    foc_cur->ad_offset_temp.c += foc_cur->mcu_ad.c;

    foc_cur->ad_offset_count++;

    if(foc_cur->ad_offset_count>=CALIBRATION_TIMES){
        foc_cur->ad_offset.a = foc_cur->ad_offset_temp.a / (float)CALIBRATION_TIMES;
        foc_cur->ad_offset.b = foc_cur->ad_offset_temp.b / (float)CALIBRATION_TIMES;
        foc_cur->ad_offset.c = foc_cur->ad_offset_temp.c / (float)CALIBRATION_TIMES;

        memset(&foc_cur->ad_offset_temp, 0x00, sizeof(foc_cur->ad_offset_temp));
        foc_cur->ad_offset_count = 0;
        foc_cur->I_abc.a = 0.0f;
        foc_cur->I_abc.b = 0.0f;
        foc_cur->I_abc.c = 0.0f;

        printf("[CURRENT_CAL] zero A/B/C=%d/%d/%d\r\n",
               (int)foc_cur->ad_offset.a,
               (int)foc_cur->ad_offset.b,
               (int)foc_cur->ad_offset.c);

        if((fabsf(foc_cur->ad_offset.a - adc_mid) > max_zero_deviation) ||
           (fabsf(foc_cur->ad_offset.b - adc_mid) > max_zero_deviation) ||
           (fabsf(foc_cur->ad_offset.c - adc_mid) > max_zero_deviation)){
            printf("[CURRENT_FAULT] zero offset outside 2048 +/- 300\r\n");
            return -2;
        }

        printf("[CURRENT_CAL] OK\r\n");
        return 0;
    }else{
        return -1;
    }
}

/**
 * @brief 过流检测
 * 
 * @param FOC_CURRENT 
 * @param I_qd 
 * @return true 过流
 * @return false 未过流
 */
bool FOC_CURRENT_Over(FOC_CURRENT_t *foc_cur, qd_t I_qd)
{
    if(fabsf(I_qd.q)>foc_cur->over||fabsf(I_qd.d)>foc_cur->over){
        foc_cur->cur_over_count+=1;
    }else{
        if(foc_cur->cur_over_count>0){
            foc_cur->cur_over_count--;
        }
    }

    if(foc_cur->cur_over_count>2){
        foc_cur->cur_over_count = 0;
        return true;
    }else{
        return false;
    }
}













































