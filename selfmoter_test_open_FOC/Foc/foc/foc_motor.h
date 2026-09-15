#ifndef _FOC_MOTOR_H
#define _FOC_MOTOR_H

#include <stdint.h>


#include  "foc_identify.h"
#include  "foc_pwm.h"
#include  "foc_current.h"
#include  "foc_type.h"
#include  "foc_pid.h"
#include  "foc_hfi.h"
#include  "foc_filter.h"
#include  "foc_sensorless.h"
#include  "foc_abzenc.h"
#include  "foc_hall.h"
#include  "foc_flux.h"
#include  "foc_vbus.h"

typedef enum
{
    MOTOR_IDLE = 0,                     /*空闲状态*/
    MOTOR_IDLE_START,    
    MOTOR_IDENTIFY,                     /*参数识别*/
    MOTOR_CLEAR,                        /*清除参数*/
    MOTOR_START,                        /*开始运行*/
    MOTOR_OPEN_RUN,                     /*开环运行*/
    MOTOR_HALL_RUN,                     /*霍尔运行*/
    MOTOR_ENC_RUN,                      /*编码器运行*/
    MOTOR_AS5047_RUN,                   /*AS5047P编码器电流闭环运行*/
    MOTOR_SL_RUN,                       /*无感运行*/
    MOTOR_ZERO_ALIGN,                   /*零点对齐*/

    MOTOR_ABZENC_CALIB,                 /*ABZ编码器校准*/
    MOTOR_HALL_LEARN,                   /*霍尔自学习*/
}MOTOR_State_t;



/*
 * 电机控制主对象：汇总传感器反馈、各级 PID、PWM 输出及运行状态。
 * 该结构体由应用层持有，并在控制中断和低频状态机之间共享。
 */
typedef struct
{
    float last_target_speed;                /*上次速度*/
    float out_contorl_time;                 /*失控时间*/

    MOTOR_State_t motor_state;              /*电机状态机*/
    float target_speed;                     /*目标速度*/
    float target_position_deg;              /*目标机械位置，单位：度，可多圈*/
    float position_speed_ref;               /*位置环输出速度，单位：rpm*/

    float pid_qd_limit;                     /*PID qd轴电流环输出限制*/
    float pid_qd_limit_hif;               

    uint32_t abzenc_calib_state;            /*ABZ编码器校准状态*/

    uint32_t        pole_pairs;             /*极对数*/
    float           power;                  /*供电电压*/

    FOC_ABZENC_t enc;                          /*编码器*/
    FOC_HALL_t hall;                        /*霍尔编码器*/
 
    FOC_SENSORLESS_t sensorless;            /*无感*/

    FOC_PID_t        pid_cur_iq;            /*电流PID Iq*/
    FOC_PID_t        pid_cur_id;            /*电流PID Id*/
    FOC_PID_t        pid_speed_pos;         /*速度环PID 位置式*/
    FOC_PID_t        pid_speed;             /*速度环PID*/

    FOC_VAR_t        foc_var;               /*foc参数*/
    FOC_PWM_t        pwm;                   /*pwm输出结构体*/
    FOC_CURRENT_t    current;               /*电流检测*/
    FOC_IDENTIFY_t   iden;
    FOC_VBUS_t       vbus;



}FOC_MOTOR_t;



extern void FOC_Motor_Init(FOC_MOTOR_t *foc_motor);

/* 判断实测速度是否持续偏离目标，用于失控保护。 */
bool FOC_Motor_Speed_OutContorl(FOC_MOTOR_t *foc_motor);

/*电机状态机运行*/
void FOC_Motor_Run(FOC_MOTOR_t *foc_motor);

/* 更新速度反馈，并执行速度环得到 q 轴电流目标。 */
void FOC_Motor_Speed_Calc(FOC_MOTOR_t *foc_motor, float t);
void FOC_Motor_Speed_Ctrl(FOC_MOTOR_t *foc_motor);
void FOC_Motor_PrintAlignResults(void);
void FOC_Motor_Lock(FOC_MOTOR_t *foc_motor);















#endif
