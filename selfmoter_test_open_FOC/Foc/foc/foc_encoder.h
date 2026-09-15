
#ifndef _FOC_ENCODER_H
#define _FOC_ENCODER_H

#include <stdbool.h>

#include "foc_type.h"
#include "foc_filter.h"

#define         ENC_ARRAY_SIZE                  50

/*编码器*/
typedef struct 
{
    bool EncAligned;                            /*是否对齐过*/
    
    float PhaseShift;                           /*偏移*/      
    // float PhaseShift_f;                         /*正转同步电角度*/
    // float PhaseShift_c;                         /*反转同步电角度*/

    int32_t period;                            /*编码器的周期计数值*/

    /*定时器发生的溢出次数*/
    int32_t over_count;
    /*上次捕捉到的计数值*/                                        
    int32_t PreviousCapture;
    
    ENC_Dir dir;
    SPEED_t speed;                 /*速度位置结构体*/
    /*存储定时器计数值的缓冲区*/
    int32_t buf_index;              
    int32_t buf[ENC_ARRAY_SIZE]; /*!< Buffer used to store
                                        captured variations of timer counter*/
}FOC_ENCODER_t;

typedef struct
{
    bool align;
    uint32_t period;            /*一周的计数值*/
    uint32_t count;
    ENC_Dir dir;
    SPEED_t speed;
    float PhaseShift; 

    int32_t over_count;         /*定时器发生的溢出次数*/                    
    int32_t PreviousCapture;    /*上次捕捉到的计数值*/    

    MOVE_FILTER_t move_filter;
}FOC_ENC_t;



void FOC_ENC_Init(FOC_ENC_t *enc, uint32_t period, uint32_t pole_pairs);
void FOC_ENC_Clear(FOC_ENC_t *enc);
void FOC_ENC_IRQHandler(FOC_ENC_t *enc);
float FOC_ENC_Angle_Calc(FOC_ENC_t *enc);
float FOC_ENC_Speed_Calc_T(FOC_ENC_t *enc, float t);
float FOC_ENC_Speed_Calc_M(FOC_ENC_t *enc, float t);
uint32_t FOC_Get_ENC_Count(void);
ENC_Dir FOC_Get_ENC_Dir(void);

#endif
