/**
 ****************************************************************************************************
 * @file        foc_vbus.c
 * @author      哔哩哔哩-Rebron大侠
 * @version     V0.0
 * @date        2025-01-11
 * @brief       VF强拖
 * @license     MIT License
 *              Copyright (c) 2025 Reborn大侠
 *              允许任何人使用、复制、修改和分发该代码，但需保留此版权声明。
 ****************************************************************************************************
 */

#include <stdio.h>

#include "foc_vbus.h"
#include "foc_math.h"



#include "adc.h"

#include "foc_hw.h"

/**
 * @brief 母线电压初始化
 * 
 * @param foc_vbus 
 */
void FOC_VBUS_Init(FOC_VBUS_t *foc_vbus)
{
    memset(foc_vbus->adc_buf, 0x00, sizeof(foc_vbus->adc_buf));

    // Move_Filter_Init(&foc_vbus->vbuf_filter, 100);

    FOC_VBUS_HW_Init(foc_vbus);


}

void FOC_VBUS_DeInit(FOC_VBUS_t *foc_vbus)
{
    memset(foc_vbus->adc_buf, 0x00, sizeof(foc_vbus->adc_buf));

    // Move_Filter_Clear(&foc_vbus->vbuf_filter);

    FOC_VBUS_HW_DeInit(foc_vbus);
}

/**
 * @brief 母线电压计算
 * 
 * @param foc_vbus 
 * @param ad 
 * @return float 
 */
float FOC_VBUS_Calc(FOC_VBUS_t *foc_vbus)
{
    // Move_Filter_fill(&foc_vbus->vbuf_filter, ad);
    // float muc_ad = Move_Filter_calculate(&foc_vbus->vbuf_filter);
    float muc_ad = 0;
    for(int i=0;i<VBUS_ADC_BUF_SIZE;i++){
        muc_ad += foc_vbus->adc_buf[i];
    }

    muc_ad /= VBUS_ADC_BUF_SIZE;

    foc_vbus->vbus = muc_ad/4095.0f*3.3f *11;

    return foc_vbus->vbus;
}

/**
 * @brief 母线电压保护
 * 
 * @param foc_vbus 
 * @return int 
 */
int FOC_VBUS_Protect(FOC_VBUS_t *foc_vbus)
{
    if(foc_vbus->vbus > 30.0f ||foc_vbus->vbus <= 20.0f){
        foc_vbus->vbus_count++;
    }else{
        foc_vbus->vbus_count = 0;
    }

    if(foc_vbus->vbus_count>=10000){
        printf("母线电压异常\r\n");
        while(1);
    }

    return 0;
}








