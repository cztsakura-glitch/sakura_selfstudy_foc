
#ifndef _FOC_TYPE_H
#define _FOC_TYPE_H

#include <stdint.h>
#include <stdbool.h>

/*模块状态*/
typedef enum{
    MODULE_ERR = -1,
    MODULE_OK = 0,
}MODULE_STATE_t;

/*编码器计数方向*/
typedef enum{
    ENC_DOWN = 0,
    ENC_UP,
}ENC_Dir;

/*转动方向*/
typedef enum{
	REVERSAL = -1,              /*反转*/
    COROTATION = 1,             /*正转*/
}ROTATION_Dir_t;

typedef struct
{
  float a;
  float b;
  float c;                      
}abc_t;

/* alphabeta 坐标系*/
typedef struct
{
  float alpha;
  float beta;
}alphabeta_t;


/*qd坐标系*/
typedef struct
{
  float q;
  float d;
} qd_t;

/*速度结构体*/
typedef struct
{
    float ElAngle;              /*当前电角度*/
    float PhaseShift;           /*相位的偏移*/
    float MecAngle;             /*当前机械角度*/
    float AvrMecSpeed;          /*平均机械速度 r/min */
    float Max_MecSpeed;         /*最大机械速度限制*/
    uint32_t pole_pairs;        /*极对数*/
    ROTATION_Dir_t dir;         /*速度方向*/
}SPEED_t;

/*FOC计算参数*/
typedef struct
{
    /*采集电流*/
    abc_t I_abc;   
    abc_t Eu;                   /*三相反电动势*/             
    alphabeta_t I_alphabeta;
    qd_t I_qd;             
    qd_t I_qd_ref;              /*参考目标Iqd*/
    /*输出电压*/
    qd_t U_qd;                  
    alphabeta_t U_alphabeta;    
    /*速度参数*/
    SPEED_t speed;
}FOC_VAR_t;



 



#endif
