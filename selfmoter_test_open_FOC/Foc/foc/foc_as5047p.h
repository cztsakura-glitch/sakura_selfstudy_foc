#ifndef _FOC_AS5047P_H
#define _FOC_AS5047P_H

#include <stdbool.h>
#include <stdint.h>

/* AS5047P 运行数据：同时保存原始帧、诊断状态和换算后的角度/速度。 */
typedef struct
{
    uint32_t pole_pairs;       /* 电机极对数，用于机械角到电角度的换算。 */
    float dir;                 /* 编码器方向：1.0 为正向，-1.0 为反向。 */
    float zero_el_angle;       /* 电角度零点补偿，单位 rad。 */

    /* 最近一次 SPI 读取的原始数据与诊断寄存器帧。 */
    uint16_t raw;
    uint16_t last_raw;
    uint16_t last_rx;
    uint16_t angle_frame;
    uint16_t diaagc_frame;
    uint16_t mag_frame;
    uint16_t diaagc;
    uint16_t mag;

    uint8_t agc;
    uint8_t magl;
    uint8_t magh;
    uint8_t cof;
    uint8_t lf;

    float mech_angle;          /* 单圈机械角度，范围为 [0, 2π)，单位 rad。 */
    float el_angle;            /* 补偿后的电角度，单位 rad。 */
    float speed_rpm;           /* 窗口法估算的机械转速，单位 rpm。 */
    int32_t speed_diff_accum;
    float speed_dt_accum;
    float position_rad;        /* 跨圈累计机械位置，单位 rad。 */
    uint8_t position_initialized;

    uint32_t sample_count;     /* 已接受的有效采样总数。 */
    uint32_t error_count;      /* SPI、奇偶校验或传感器错误累计数。 */
    uint32_t consecutive_error_count;
    uint32_t invalid_sample_count;
    uint32_t jump_reject_count;
    uint32_t zero_rx_count;
    uint32_t zero_angle_count;
} FOC_AS5047P_t;

void FOC_AS5047P_Init(uint32_t pole_pairs);
bool FOC_AS5047P_Update(float dt);
void FOC_AS5047P_ReadDiagnostics(void);
void FOC_AS5047P_SetZeroCurrent(void);
void FOC_AS5047P_SetDirection(float dir);
const FOC_AS5047P_t *FOC_AS5047P_GetState(void);
float FOC_AS5047P_GetElectricalAngle(void);
float FOC_AS5047P_GetMechanicalAngle(void);
float FOC_AS5047P_GetSpeedRpm(void);
float FOC_AS5047P_GetPositionRad(void);
float FOC_AS5047P_GetPositionDeg(void);
void FOC_AS5047P_PrintStatus(void);

#endif
