#include "foc_as5047p.h"

#include "foc_math.h"
#include "main.h"
#include "spi.h"

#include <math.h>
#include <stdio.h>

#define AS5047P_REG_DIAAGC    0x3FFCU
#define AS5047P_REG_MAG       0x3FFDU
#define AS5047P_REG_ANGLEUNC  0x3FFEU
#define AS5047P_REG_ERRFL     0x0001U
#define AS5047P_DATA_MASK     0x3FFFU
#define AS5047P_EF_MASK       0x4000U
#define AS5047P_CPR           16384.0f
#define AS5047P_SPEED_WINDOW_S 0.010f
#define AS5047P_READ_ATTEMPTS  2U

/*
 * AS5047P 是 14 位绝对值磁编码器, 一圈共有 16384 个计数.
 * 本文件负责: SPI 读角度 -> 机械角度 -> 电角度 -> 估算机械转速 rpm.
 */
static FOC_AS5047P_t g_as5047p = {
    .pole_pairs = 7U,
    .dir = 1.0f,
};

static bool g_as5047p_transfer_ok = true;
static bool g_as5047p_last_read_valid = false;

/* AS5047P 命令帧需要偶校验位, 这里统计 16bit 中 1 的个数奇偶性. */
static uint8_t AS5047P_EvenParity(uint16_t value)
{
    uint8_t parity = 0U;

    while (value != 0U)
    {
        parity ^= (uint8_t)(value & 0x0001U);
        value >>= 1U;
    }

    return parity;
}

/* 生成读寄存器命令: bit14=1 表示 read, bit15 放偶校验结果. */
static uint16_t AS5047P_MakeReadCmd(uint16_t addr)
{
    uint16_t cmd = 0x4000U | (addr & AS5047P_DATA_MASK);

    if (AS5047P_EvenParity(cmd) != 0U)
    {
        cmd |= 0x8000U;
    }

    return cmd;
}

/* SPI 传输 16bit 数据. AS5047P 读寄存器有流水线特性, 读命令和返回值分两次传输. */
static uint16_t AS5047P_Transfer16(uint16_t tx_data)
{
    uint16_t rx_data = 0U;
    volatile uint32_t delay_count;

    HAL_GPIO_WritePin(AS5047_CS_GPIO_Port, AS5047_CS_Pin, GPIO_PIN_RESET);

    for (delay_count = 0U; delay_count < 160U; delay_count++)
    {
        __NOP();
    }

    if (hspi2.Init.DataSize == SPI_DATASIZE_8BIT)
    {
        uint8_t tx_buf[2];
        uint8_t rx_buf[2] = {0U, 0U};

        tx_buf[0] = (uint8_t)(tx_data >> 8);
        tx_buf[1] = (uint8_t)(tx_data & 0x00FFU);

        if (HAL_SPI_TransmitReceive(&hspi2, tx_buf, rx_buf, 2U, 10U) == HAL_OK)
        {
            rx_data = ((uint16_t)rx_buf[0] << 8) | (uint16_t)rx_buf[1];
        }
        else
        {
            g_as5047p.error_count++;
            g_as5047p_transfer_ok = false;
        }
    }
    else
    {
        uint16_t tx_word = tx_data;
        uint16_t rx_word = 0U;

        if (HAL_SPI_TransmitReceive(&hspi2,
                                    (uint8_t *)&tx_word,
                                    (uint8_t *)&rx_word,
                                    1U,
                                    10U) == HAL_OK)
        {
            rx_data = rx_word;
        }
        else
        {
            g_as5047p.error_count++;
            g_as5047p_transfer_ok = false;
        }
    }

    for (delay_count = 0U; delay_count < 160U; delay_count++)
    {
        __NOP();
    }

    HAL_GPIO_WritePin(AS5047_CS_GPIO_Port, AS5047_CS_Pin, GPIO_PIN_SET);

    for (delay_count = 0U; delay_count < 160U; delay_count++)
    {
        __NOP();
    }

    g_as5047p.last_rx = rx_data;
    if (rx_data == 0U)
    {
        g_as5047p.zero_rx_count++;
    }

    return rx_data;
}

static uint16_t AS5047P_ReadReg(uint16_t addr)
{
    uint16_t rx = 0U;
    uint16_t data = 0U;
    uint32_t attempt;

    g_as5047p_last_read_valid = false;

    for (attempt = 0U; attempt < AS5047P_READ_ATTEMPTS; attempt++)
    {
        g_as5047p_transfer_ok = true;
        AS5047P_Transfer16(AS5047P_MakeReadCmd(addr));
        rx = AS5047P_Transfer16(0x0000U);
        data = rx & AS5047P_DATA_MASK;

        if (g_as5047p_transfer_ok &&
            ((rx & AS5047P_EF_MASK) == 0U) &&
            (AS5047P_EvenParity(rx) == 0U))
        {
            g_as5047p_last_read_valid = true;

            switch (addr & AS5047P_DATA_MASK)
            {
                case AS5047P_REG_ANGLEUNC:
                    g_as5047p.angle_frame = rx;
                    if (data == 0U)
                    {
                        g_as5047p.zero_angle_count++;
                    }
                    break;

                case AS5047P_REG_DIAAGC:
                    g_as5047p.diaagc_frame = rx;
                    break;

                case AS5047P_REG_MAG:
                    g_as5047p.mag_frame = rx;
                    break;

                default:
                    break;
            }

            return data;
        }

        if (g_as5047p_transfer_ok)
        {
            g_as5047p.error_count++;
        }
        g_as5047p.invalid_sample_count++;

        if ((rx & AS5047P_EF_MASK) != 0U)
        {
            AS5047P_Transfer16(AS5047P_MakeReadCmd(AS5047P_REG_ERRFL));
            AS5047P_Transfer16(0x0000U);
        }
    }

    /* 一次寄存器读取最终失败只计数一次，内部重试不重复累计。 */
    g_as5047p.consecutive_error_count++;
    return data;
}

void FOC_AS5047P_Init(uint32_t pole_pairs)
{
    g_as5047p.pole_pairs = pole_pairs;
    g_as5047p.dir = 1.0f;
    g_as5047p.zero_el_angle = 0.0f;
    g_as5047p.position_rad = 0.0f;
    g_as5047p.position_initialized = 0U;
    g_as5047p.speed_diff_accum = 0;
    g_as5047p.speed_dt_accum = 0.0f;
    HAL_GPIO_WritePin(AS5047_CS_GPIO_Port, AS5047_CS_Pin, GPIO_PIN_SET);
}

/*
 * 读取 AS5047P 机械角度, 并换算成 FOC 使用的电角度.
 * 电角度 = 机械角度 * 极对数 * 编码器方向 - 启动对齐零点.
 * dt > 0 时, 用相邻两次 14bit 角度差分估算机械转速 rpm.
 */
bool FOC_AS5047P_Update(float dt)
{
    uint16_t raw = AS5047P_ReadReg(AS5047P_REG_ANGLEUNC);
    int32_t diff;
    float mech_angle;
    float el_angle;

    /* 含EF、奇偶校验或SPI错误的数据帧不得送入角度环和速度环。 */
    if (!g_as5047p_last_read_valid)
    {
        return false;
    }

    if ((g_as5047p.position_initialized != 0U) &&
        (g_as5047p.sample_count > 0U))
    {
        diff = (int32_t)raw - (int32_t)g_as5047p.raw;
        if (diff > 8192)
        {
            diff -= 16384;
        }
        else if (diff < -8192)
        {
            diff += 16384;
        }

        /* 角度环每1 ms更新一次。电机在额定最高2700 rpm时，14位编码器每次采样
         * 约变化737个计数；阈值既为中断抖动留出余量，又能剔除明显异常数据。 */
        if ((diff > 1024) || (diff < -1024))
        {
            g_as5047p.error_count++;
            g_as5047p.invalid_sample_count++;
            g_as5047p.jump_reject_count++;
            g_as5047p.consecutive_error_count++;
            return false;
        }
    }

    g_as5047p.consecutive_error_count = 0U;

    g_as5047p.last_raw = g_as5047p.raw;
    g_as5047p.raw = raw;
    g_as5047p.sample_count++;

    mech_angle = ((float)raw) * _2PI / AS5047P_CPR;
    el_angle = Limit_Angle((mech_angle * (float)g_as5047p.pole_pairs *
                            g_as5047p.dir) -
                           g_as5047p.zero_el_angle);

    g_as5047p.mech_angle = mech_angle;
    g_as5047p.el_angle = el_angle;

    if (g_as5047p.sample_count > 1U)
    {
        /* 14bit 角度在 0/16383 处跳变, 跨零点时要把差值展开成最短方向. */

        diff = (int32_t)raw - (int32_t)g_as5047p.last_raw;
        if (diff > 8192)
        {
            diff -= 16384;
        }
        else if (diff < -8192)
        {
            diff += 16384;
        }

        if (g_as5047p.position_initialized != 0U)
        {
            g_as5047p.position_rad +=
                ((float)diff) * g_as5047p.dir * _2PI / AS5047P_CPR;
        }

        if (dt > 0.0f)
        {
            /* 先累计编码器计数再估算低速。电角度仍逐次更新，只有机械转速估算
             * 使用10 ms窗口；在5 rpm时窗口内约有14个计数，同时仍能满足该转子的响应要求。 */
            g_as5047p.speed_diff_accum += diff;
            g_as5047p.speed_dt_accum += dt;

            if (g_as5047p.speed_dt_accum >= AS5047P_SPEED_WINDOW_S)
            {
                float measured_rpm =
                    ((float)g_as5047p.speed_diff_accum) * g_as5047p.dir *
                    60.0f / AS5047P_CPR / g_as5047p.speed_dt_accum;

                g_as5047p.speed_rpm =
                    (0.65f * g_as5047p.speed_rpm) + (0.35f * measured_rpm);
                g_as5047p.speed_diff_accum = 0;
                g_as5047p.speed_dt_accum = 0.0f;
            }
        }
    }

    return true;
}

void FOC_AS5047P_ReadDiagnostics(void)
{
    uint16_t diaagc = AS5047P_ReadReg(AS5047P_REG_DIAAGC);
    uint16_t mag = AS5047P_ReadReg(AS5047P_REG_MAG);

    g_as5047p.diaagc = diaagc;
    g_as5047p.mag = mag & AS5047P_DATA_MASK;
    g_as5047p.magl = (uint8_t)((diaagc >> 11) & 0x0001U);
    g_as5047p.magh = (uint8_t)((diaagc >> 10) & 0x0001U);
    g_as5047p.cof = (uint8_t)((diaagc >> 9) & 0x0001U);
    g_as5047p.lf = (uint8_t)((diaagc >> 8) & 0x0001U);
    g_as5047p.agc = (uint8_t)(diaagc & 0x00FFU);
}

/* 对齐完成后调用: 把当前电角度保存为闭环零点. */
void FOC_AS5047P_SetZeroCurrent(void)
{
    /* 对齐过程中的正常位移可能超过闭环跳变阈值，因此建立零点前先无条件接收一次新角度。 */
    g_as5047p.position_initialized = 0U;
    FOC_AS5047P_Update(0.0f);
    g_as5047p.zero_el_angle =
        Limit_Angle(g_as5047p.mech_angle *
                    (float)g_as5047p.pole_pairs *
                    g_as5047p.dir);
    g_as5047p.el_angle = 0.0f;
    /* 位置模式以每次启动对齐完成的位置为机械零点。 */
    g_as5047p.position_rad = 0.0f;
    g_as5047p.position_initialized = 1U;
    g_as5047p.last_raw = g_as5047p.raw;
    g_as5047p.speed_rpm = 0.0f;
    g_as5047p.speed_diff_accum = 0;
    g_as5047p.speed_dt_accum = 0.0f;
}

void FOC_AS5047P_SetDirection(float dir)
{
    g_as5047p.dir = (dir < 0.0f) ? -1.0f : 1.0f;
}

const FOC_AS5047P_t *FOC_AS5047P_GetState(void)
{
    return &g_as5047p;
}

float FOC_AS5047P_GetElectricalAngle(void)
{
    return g_as5047p.el_angle;
}

float FOC_AS5047P_GetMechanicalAngle(void)
{
    return g_as5047p.mech_angle;
}

float FOC_AS5047P_GetSpeedRpm(void)
{
    return g_as5047p.speed_rpm;
}

float FOC_AS5047P_GetPositionRad(void)
{
    return g_as5047p.position_rad;
}

float FOC_AS5047P_GetPositionDeg(void)
{
    return g_as5047p.position_rad * 360.0f / _2PI;
}

void FOC_AS5047P_PrintStatus(void)
{
    int mech_deg10;
    int el_deg10;
    int zero_deg10;
    int rpm;

    FOC_AS5047P_Update(0.0f);
    FOC_AS5047P_ReadDiagnostics();

    mech_deg10 = (int)(g_as5047p.mech_angle * 1800.0f / _2PI);
    el_deg10 = (int)(g_as5047p.el_angle * 1800.0f / _2PI);
    zero_deg10 = (int)(g_as5047p.zero_el_angle * 1800.0f / _2PI);
    rpm = (int)g_as5047p.speed_rpm;

    printf("[AS5047] raw=%u frame=0x%04X last=0x%04X mech_x10=%d el_x10=%d zero_x10=%d dir=%d rpm=%d ",
           (unsigned int)g_as5047p.raw,
           (unsigned int)g_as5047p.angle_frame,
           (unsigned int)g_as5047p.last_rx,
           mech_deg10,
           el_deg10,
           zero_deg10,
           (g_as5047p.dir < 0.0f) ? -1 : 1,
           rpm);

    printf("mag=%u agc=%u flags=ML%u/MH%u/COF%u/LF%u err=%lu zeroRx=%lu zeroAng=%lu samples=%lu\r\n",
           (unsigned int)g_as5047p.mag,
           (unsigned int)g_as5047p.agc,
           (unsigned int)g_as5047p.magl,
           (unsigned int)g_as5047p.magh,
           (unsigned int)g_as5047p.cof,
           (unsigned int)g_as5047p.lf,
           (unsigned long)g_as5047p.error_count,
           (unsigned long)g_as5047p.zero_rx_count,
           (unsigned long)g_as5047p.zero_angle_count,
           (unsigned long)g_as5047p.sample_count);
}
