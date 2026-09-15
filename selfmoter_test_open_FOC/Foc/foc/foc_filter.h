#ifndef _FOC_FILTER_H
#define _FOC_FILTER_H

#include <stdint.h>
#include <foc_type.h>
#include <arm_math.h>



#define MOVE_FILTER_BUF_MAXSIZE     500         /*滑动滤波最大区间*/
/*滑动滤波器*/
typedef struct{
    int32_t  buf[MOVE_FILTER_BUF_MAXSIZE];
    int32_t  buf_sum;
    uint32_t buf_size; 
    uint32_t buf_Index;
    uint32_t buf_fill;
    float val;                                  /*计算的平均值*/
}MOVE_FILTER_t;

void Move_Filter_Init(MOVE_FILTER_t *filter, uint32_t buf_size);
void Move_Filter_fill(MOVE_FILTER_t *filter, int64_t val);
void Move_Filter_Clear(MOVE_FILTER_t *filter);
float Move_Filter_calculate(MOVE_FILTER_t *filter);



/*************************************数字锁相环**********************************/

/*锁相环*/
typedef struct{
    float 	Kp;										/*锁相环PI控制器Kp*/
    float 	Ki;										/*锁相环PI控制器Ki*/
    
    float   P;
    float   I;

    float 	Angle_Err;						        /*角度误差*/
    float 	Omega;								    /*电角速度*/
    float 	Omega_F;							    /*滤波后的电角速度*/
    float 	Angle;								    /*锁相环计算得到的电角度*/


    float 	Angle_last;						        /*上一次的电角度*/
    
    float   Ts;                                     /*执行周期*/

    MOVE_FILTER_t move_filter;
}PLL_t;  


void PLL_Init(PLL_t *pll, float Ts);
void PLL_Cale(PLL_t *pll, float alpha, float beta);
void PLL_Cale_SMO(PLL_t *pll, float alpha, float beta);
void PLL_Calc_HALL(PLL_t *pll, float input_theta, float input_speed);


void PLL_Cale_HFI(PLL_t *pll, float alpha, float beta);

void PLL_Clear(PLL_t *pll);












#endif
