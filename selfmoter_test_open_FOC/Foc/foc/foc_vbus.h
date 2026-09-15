#ifndef _FOC_VBUS_H
#define _FOC_VBUS_H

#include "foc_type.h"
#include "foc_filter.h"


#define VBUS_ADC_BUF_SIZE                        10

typedef struct{
    MODULE_STATE_t state;                       /*模块状态*/
    float vbus;
    uint32_t vbus_count;

    uint32_t adc_buf[VBUS_ADC_BUF_SIZE];        /*母线电压AD采集缓冲区*/

    MOVE_FILTER_t vbuf_filter;
}FOC_VBUS_t;



void FOC_VBUS_Init(FOC_VBUS_t *foc_vbus);
void FOC_VBUS_DeInit(FOC_VBUS_t *foc_vbus);
float FOC_VBUS_Calc(FOC_VBUS_t *foc_vbus);

int FOC_VBUS_Protect(FOC_VBUS_t *foc_vbus);

#endif


