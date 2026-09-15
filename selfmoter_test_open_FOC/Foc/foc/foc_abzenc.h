
#ifndef _FOC_ABZENC_H
#define _FOC_ABZENC_H

#include <stdbool.h>

#include "foc_type.h"
#include "foc_filter.h"

typedef struct
{
    bool align;
    int32_t align_dir;

    uint32_t period;            /*一周的计数值*/

    ENC_Dir dir;
    SPEED_t speed;

    int32_t over_count;         /*定时器发生的溢出次数*/                    
    int32_t PreviousCapture;    /*上次捕捉到的计数值*/    

    uint8_t calib_state;                        /*校准状态*/
    uint32_t calib_count;                       /*校准次数*/

    MOVE_FILTER_t move_filter;
}FOC_ABZENC_t;



void FOC_ABZENC_Init(FOC_ABZENC_t *enc, uint32_t period, uint32_t pole_pairs);
void FOC_ABZENC_Clear(FOC_ABZENC_t *enc);

float FOC_ABZENC_Angle_Calc(FOC_ABZENC_t *enc);
float FOC_ABZENC_Speed_Calc_M(FOC_ABZENC_t *enc, float t);


bool FOC_ABZENC_Angle_Clib(FOC_ABZENC_t *enc);

#endif
