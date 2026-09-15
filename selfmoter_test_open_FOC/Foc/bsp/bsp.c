#include "bsp.h"
#include <string.h>
#include <stdbool.h>
#include <stdlib.h>
#include <tim.h>
#include "foc_app.h"
#include "foc_hw.h"
#include "foc_as5047p.h"
#include "foc_user_config.h"

/*
 * 板级支持层：集中完成调试串口、按键扫描以及 FOC 启停命令的适配。
 * 本文件只负责把外部命令转换为控制请求，实时电流环仍由定时器中断执行。
 */

/* 关闭半主机依赖，使 printf 可以在脱离调试器时重定向到板载串口。 */
#if defined(__ARMCC_VERSION) && (__ARMCC_VERSION >= 6010050)
__asm(".global __use_no_semihosting\n\t");
__asm(".global __ARM_use_no_argv \n\t");
#else
#pragma import(__use_no_semihosting)

struct __FILE
{
    int handle;
};
#endif

FILE __stdout;

int _ttywrch(int ch)
{
    return ch;
}

void _sys_exit(int x)
{
    (void)x;
}

char *_sys_command_string(char *cmd, int len)
{
    (void)cmd;
    (void)len;
    return NULL;
}

void uart_init(uint32_t baudrate)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    uint32_t pclk;
    uint32_t pclk2;

    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_USART1_CLK_ENABLE();
    __HAL_RCC_USART2_CLK_ENABLE();
    __HAL_RCC_USART3_CLK_ENABLE();

    /*
     * 三路调试串口使用彼此独立且不占用 TIM1 电机 PWM 的引脚：
     * USART1：PB6 发送、PB7 接收；USART2：PD5 发送、PD6 接收；
     * USART3：PB10 发送、PB11 接收。
     */
    GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_10 | GPIO_PIN_11;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_5 | GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    USART1->CR1 = 0U;
    USART1->CR2 = 0U;
    USART1->CR3 = 0U;

    USART2->CR1 = 0U;
    USART2->CR2 = 0U;
    USART2->CR3 = 0U;

    USART3->CR1 = 0U;
    USART3->CR2 = 0U;
    USART3->CR3 = 0U;

    pclk = HAL_RCC_GetPCLK1Freq();
    pclk2 = HAL_RCC_GetPCLK2Freq();

    USART1->BRR = (pclk2 + (baudrate / 2U)) / baudrate;
    USART2->BRR = (pclk + (baudrate / 2U)) / baudrate;
    USART3->BRR = (pclk + (baudrate / 2U)) / baudrate;

    USART1->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE | USART_CR1_UE;
    USART2->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE | USART_CR1_UE;
    USART3->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE | USART_CR1_UE;

    HAL_NVIC_SetPriority(USART1_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
    HAL_NVIC_SetPriority(USART2_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(USART2_IRQn);
    HAL_NVIC_SetPriority(USART3_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(USART3_IRQn);
}

static void uart_putc(USART_TypeDef *uart, int ch)
{
    while ((uart->SR & USART_SR_TXE) == 0U)
    {
    }

    uart->DR = (uint8_t)ch;
}

int fputc(int ch, FILE *f)
{
    (void)f;

    if (__HAL_RCC_USART1_IS_CLK_ENABLED())
    {
        uart_putc(USART1, ch);
    }

    if (__HAL_RCC_USART2_IS_CLK_ENABLED())
    {
        uart_putc(USART2, ch);
    }

    if (__HAL_RCC_USART3_IS_CLK_ENABLED())
    {
        uart_putc(USART3, ch);
    }

    return ch;
}

#define DEBUG_UART_RX_FIFO_SIZE 256U

static char g_debug_uart_cmd[32];
static uint32_t g_debug_uart_cmd_len = 0U;
static volatile uint16_t g_debug_uart_rx_fifo[DEBUG_UART_RX_FIFO_SIZE];
static volatile uint32_t g_debug_uart_rx_head = 0U;
static volatile uint32_t g_debug_uart_rx_tail = 0U;
static volatile uint32_t g_debug_uart_rx_irq_count = 0U;
static volatile uint32_t g_debug_uart_rx_drop_count = 0U;


/*
 * 板载 5 个用户按键只修改目标状态, 真正的电机控制仍在 FOC 状态机里完成.
 * KEY1: 启动当前模式; KEY2: 停止; KEY5: 位置/速度/转矩模式循环切换.
 * KEY3/KEY4 根据当前模式调节位置、速度或转矩电流目标.
 */
/* ===================== 用户闭环参数入口 =====================
 * 转矩（电流）目标：             BSP_FOC_DEFAULT_TORQUE_IQ_A
 * 速度-电流目标：                BSP_FOC_DEFAULT_SPEED_RPM
 * 位置-速度-电流目标：           BSP_FOC_DEFAULT_POSITION_DEG
 * 速度/位置模式最大转矩电流：    BSP_FOC_DEFAULT_IQ_LIMIT_A
 * 运行中也可用 iq=、spd=、pos= 修改对应目标。
 * 在功率级通过更大电流验证前，建议保持已验证的 0.10 A 限幅。
 * PI 和运动规划参数统一在 Foc/foc/foc_user_config.h 中修改。
 * ============================================================ */
#define BSP_FOC_DEFAULT_TORQUE_IQ_A   FOC_USER_DEFAULT_TORQUE_IQ_A
#define BSP_FOC_DEFAULT_SPEED_RPM     FOC_USER_DEFAULT_SPEED_RPM
#define BSP_FOC_DEFAULT_POSITION_DEG  FOC_USER_DEFAULT_POSITION_DEG
#define BSP_FOC_DEFAULT_IQ_LIMIT_A    FOC_USER_DEFAULT_IQ_LIMIT_A
#define BSP_FOC_PHASE_MAP             FOC_USER_PHASE_MAP
#define BSP_FOC_ENCODER_DIR           FOC_USER_ENCODER_DIR
#define BSP_FOC_IQ_DIR                FOC_USER_IQ_DIR

#define BSP_KEY_DEBOUNCE_MS           40U
#define BSP_KEY_SPEED_STEP_RPM        FOC_USER_KEY_SPEED_STEP_RPM
#define BSP_KEY_DEFAULT_SPEED_RPM     BSP_FOC_DEFAULT_SPEED_RPM
#define BSP_KEY_MAX_SPEED_RPM         300.0f
#define BSP_KEY_IQ_LIMIT_A            BSP_FOC_DEFAULT_IQ_LIMIT_A
#define BSP_KEY_POSITION_STEP_DEG     FOC_USER_KEY_POSITION_STEP_DEG
#define BSP_KEY_TORQUE_STEP_A         FOC_USER_KEY_TORQUE_STEP_A
#define BSP_KEY_PRESSED_LEVEL         GPIO_PIN_SET

#define BSP_KEY1_GPIO_Port            GPIOA
#define BSP_KEY1_Pin                  GPIO_PIN_0
#define BSP_KEY2_GPIO_Port            GPIOG
#define BSP_KEY2_Pin                  GPIO_PIN_2
#define BSP_KEY3_GPIO_Port            GPIOC
#define BSP_KEY3_Pin                  GPIO_PIN_13
#define BSP_KEY4_GPIO_Port            GPIOG
#define BSP_KEY4_Pin                  GPIO_PIN_3
#define BSP_KEY5_GPIO_Port            GPIOG
#define BSP_KEY5_Pin                  GPIO_PIN_4

typedef struct
{
    GPIO_TypeDef *port;
    uint16_t pin;
    const char *name;
    GPIO_PinState raw_state;
    GPIO_PinState stable_state;
    uint32_t last_change_tick;
} BSP_Key_t;

static BSP_Key_t g_bsp_keys[] =
{
    {BSP_KEY1_GPIO_Port, BSP_KEY1_Pin, "KEY1/START", GPIO_PIN_RESET, GPIO_PIN_RESET, 0U},
    {BSP_KEY2_GPIO_Port, BSP_KEY2_Pin, "KEY2/STOP", GPIO_PIN_RESET, GPIO_PIN_RESET, 0U},
    {BSP_KEY3_GPIO_Port, BSP_KEY3_Pin, "KEY3/PLUS", GPIO_PIN_RESET, GPIO_PIN_RESET, 0U},
    {BSP_KEY4_GPIO_Port, BSP_KEY4_Pin, "KEY4/MINUS", GPIO_PIN_RESET, GPIO_PIN_RESET, 0U},
    {BSP_KEY5_GPIO_Port, BSP_KEY5_Pin, "KEY5/MODE", GPIO_PIN_RESET, GPIO_PIN_RESET, 0U},
};

static float g_bsp_key_speed_cmd = BSP_KEY_DEFAULT_SPEED_RPM;
static bool g_bsp_key_inited = false;

/* 按键命令层：完成限幅、消抖、模式轮换和目标值增减。 */
static float BSP_KeyClampSpeed(float speed)
{
    if (speed > BSP_KEY_MAX_SPEED_RPM)
    {
        return BSP_KEY_MAX_SPEED_RPM;
    }

    if (speed < -BSP_KEY_MAX_SPEED_RPM)
    {
        return -BSP_KEY_MAX_SPEED_RPM;
    }

    return speed;
}

static float BSP_KeyClampTorque(float iq_amp)
{
    if (iq_amp > BSP_KEY_IQ_LIMIT_A)
    {
        return BSP_KEY_IQ_LIMIT_A;
    }
    if (iq_amp < -BSP_KEY_IQ_LIMIT_A)
    {
        return -BSP_KEY_IQ_LIMIT_A;
    }
    return iq_amp;
}

static float BSP_KeyAbsSpeed(float speed)
{
    return (speed < 0.0f) ? -speed : speed;
}

static float BSP_KeyReadTargetSpeed(void)
{
    float speed = FOC_MOTOR.target_speed;

    if (BSP_KeyAbsSpeed(speed) >= 1.0f)
    {
        g_bsp_key_speed_cmd = BSP_KeyClampSpeed(speed);
    }

    return g_bsp_key_speed_cmd;
}

static void BSP_KeySetTargetSpeed(float speed)
{
    g_bsp_key_speed_cmd = BSP_KeyClampSpeed(speed);
    FOC_Debug_SetTargetSpeed(g_bsp_key_speed_cmd);
    printf("[KEY] target=%d rpm\r\n", (int)g_bsp_key_speed_cmd);
}

/* KEY1 使用安全限流启动当前选择的控制模式。 */
static void BSP_KeyStartMotor(void)
{
    uint32_t mode = FOC_Debug_GetEncoderMode();
    float speed = BSP_KeyReadTargetSpeed();
    float iq_setting = BSP_KEY_IQ_LIMIT_A;

    if (BSP_KeyAbsSpeed(speed) < 1.0f)
    {
        speed = BSP_KEY_DEFAULT_SPEED_RPM;
    }

    g_bsp_key_speed_cmd = BSP_KeyClampSpeed(speed);

    /* 重新配置前先停止一次, 避免上一次电流环/速度环积分状态残留. */
    FOC_Debug_SetRunRequest(false);
    FOC_Debug_SetAlignOnly(false);
    FOC_Debug_SetPhaseMap(BSP_FOC_PHASE_MAP);
    FOC_Debug_SetEncoderMode(mode);
    FOC_Debug_SetPwmMode(0U);
    FOC_AS5047P_SetDirection(BSP_FOC_ENCODER_DIR);
    FOC_Debug_SetEncoderSpeedIqDir(BSP_FOC_IQ_DIR);

    if (mode == FOC_CONTROL_TORQUE)
    {
        iq_setting = FOC_Debug_GetEncoderIqRef();
        if ((iq_setting > -0.02f) && (iq_setting < 0.02f))
        {
            iq_setting = BSP_FOC_DEFAULT_TORQUE_IQ_A;
        }
    }

    FOC_Debug_SetEncoderIqRef(iq_setting);
    FOC_Debug_SetEncoderIdRef(0.0f);
    FOC_Debug_SetTargetSpeed(g_bsp_key_speed_cmd);
    FOC_Debug_SetRunRequest(true);

    printf("[KEY] KEY1 start mode=%lu speed=%d rpm position_x10=%d iq=%dmA\r\n",
           (unsigned long)mode,
           (int)g_bsp_key_speed_cmd,
           (int)(FOC_Debug_GetTargetPositionDeg() * 10.0f),
           (int)(iq_setting * 1000.0f));
}

static void BSP_KeyStopMotor(void)
{
    BSP_FOC_Stop();
    printf("[KEY] KEY2 stop\r\n");
}

/* KEY3/KEY4 只改变 target_speed, 速度环会自动调节 Iq 来追目标转速. */
static void BSP_KeyTargetUp(void)
{
    uint32_t mode = FOC_Debug_GetEncoderMode();
    float speed = BSP_KeyReadTargetSpeed();

    if (mode == FOC_CONTROL_POSITION)
    {
        FOC_Debug_AdjustTargetPositionDeg(BSP_KEY_POSITION_STEP_DEG);
        printf("[KEY] KEY3 position +%d deg\r\n", (int)BSP_KEY_POSITION_STEP_DEG);
        return;
    }

    if (mode == FOC_CONTROL_TORQUE)
    {
        FOC_Debug_SetEncoderIqRef(BSP_KeyClampTorque(
            FOC_Debug_GetEncoderIqRef() + BSP_KEY_TORQUE_STEP_A));
        printf("[KEY] KEY3 torque current +%dmA\r\n",
               (int)(BSP_KEY_TORQUE_STEP_A * 1000.0f));
        return;
    }

    if (BSP_KeyAbsSpeed(speed) < 1.0f)
    {
        speed = BSP_KEY_DEFAULT_SPEED_RPM;
    }
    else if (speed > 0.0f)
    {
        speed += BSP_KEY_SPEED_STEP_RPM;
    }
    else
    {
        speed -= BSP_KEY_SPEED_STEP_RPM;
    }

    BSP_KeySetTargetSpeed(speed);
    printf("[KEY] KEY3 speed up\r\n");
}

static void BSP_KeyTargetDown(void)
{
    uint32_t mode = FOC_Debug_GetEncoderMode();
    float speed = BSP_KeyReadTargetSpeed();

    if (mode == FOC_CONTROL_POSITION)
    {
        FOC_Debug_AdjustTargetPositionDeg(-BSP_KEY_POSITION_STEP_DEG);
        printf("[KEY] KEY4 position -%d deg\r\n", (int)BSP_KEY_POSITION_STEP_DEG);
        return;
    }

    if (mode == FOC_CONTROL_TORQUE)
    {
        FOC_Debug_SetEncoderIqRef(BSP_KeyClampTorque(
            FOC_Debug_GetEncoderIqRef() - BSP_KEY_TORQUE_STEP_A));
        printf("[KEY] KEY4 torque current -%dmA\r\n",
               (int)(BSP_KEY_TORQUE_STEP_A * 1000.0f));
        return;
    }

    if (speed > BSP_KEY_SPEED_STEP_RPM)
    {
        speed -= BSP_KEY_SPEED_STEP_RPM;
    }
    else if (speed < -BSP_KEY_SPEED_STEP_RPM)
    {
        speed += BSP_KEY_SPEED_STEP_RPM;
    }
    else
    {
        speed = 0.0f;
    }

    BSP_KeySetTargetSpeed(speed);
    printf("[KEY] KEY4 speed down\r\n");
}

static void BSP_KeyCycleMode(void)
{
    uint32_t mode = FOC_Debug_GetEncoderMode();

    if (mode == FOC_CONTROL_POSITION)
    {
        mode = FOC_CONTROL_SPEED;
    }
    else if (mode == FOC_CONTROL_SPEED)
    {
        mode = FOC_CONTROL_TORQUE;
    }
    else
    {
        mode = FOC_CONTROL_POSITION;
    }

    FOC_Debug_SetEncoderMode(mode);
    printf("[KEY] KEY5 mode=%lu (position=3 speed=2 torque=1)\r\n",
           (unsigned long)mode);
}

static void BSP_KeyHandlePress(uint32_t index)
{
    switch (index)
    {
        case 0U:
            BSP_KeyStartMotor();
            break;
        case 1U:
            BSP_KeyStopMotor();
            break;
        case 2U:
            BSP_KeyTargetUp();
            break;
        case 3U:
            BSP_KeyTargetDown();
            break;
        case 4U:
            BSP_KeyCycleMode();
            break;
        default:
            break;
    }
}

/* 在主循环中轮询按键并消抖，避免干扰FOC中断。 */
static void BSP_KeyInit(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    uint32_t tick = HAL_GetTick();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOG_CLK_ENABLE();

    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

    GPIO_InitStruct.Pin = BSP_KEY1_Pin;
    HAL_GPIO_Init(BSP_KEY1_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = BSP_KEY3_Pin;
    HAL_GPIO_Init(BSP_KEY3_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = BSP_KEY2_Pin | BSP_KEY4_Pin | BSP_KEY5_Pin;
    HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

    for (uint32_t i = 0U; i < (sizeof(g_bsp_keys) / sizeof(g_bsp_keys[0])); i++)
    {
        g_bsp_keys[i].raw_state = HAL_GPIO_ReadPin(g_bsp_keys[i].port, g_bsp_keys[i].pin);
        g_bsp_keys[i].stable_state = g_bsp_keys[i].raw_state;
        g_bsp_keys[i].last_change_tick = tick;
    }

    g_bsp_key_inited = true;
    printf("[KEY] KEY1=start KEY2=stop KEY3=target+ KEY4=target- KEY5=mode(position/speed/torque)\r\n");
}

void BSP_KeyPoll(void)
{
    uint32_t tick;

    if (!g_bsp_key_inited)
    {
        return;
    }

    tick = HAL_GetTick();

    for (uint32_t i = 0U; i < (sizeof(g_bsp_keys) / sizeof(g_bsp_keys[0])); i++)
    {
        GPIO_PinState raw = HAL_GPIO_ReadPin(g_bsp_keys[i].port, g_bsp_keys[i].pin);

        if (raw != g_bsp_keys[i].raw_state)
        {
            g_bsp_keys[i].raw_state = raw;
            g_bsp_keys[i].last_change_tick = tick;
        }

        if (((tick - g_bsp_keys[i].last_change_tick) >= BSP_KEY_DEBOUNCE_MS) &&
            (g_bsp_keys[i].stable_state != raw))
        {
            g_bsp_keys[i].stable_state = raw;

            if (raw == BSP_KEY_PRESSED_LEVEL)
            {
                printf("[KEY] %s pressed\r\n", g_bsp_keys[i].name);
                BSP_KeyHandlePress(i);
            }
        }
    }
}

/* 串口接收层：中断仅压入轻量事件，完整命令留到主循环解析。 */
static uint8_t BSP_DebugUartPortId(USART_TypeDef *uart)
{
    if (uart == USART1)
    {
        return 1U;
    }

    if (uart == USART2)
    {
        return 2U;
    }

    if (uart == USART3)
    {
        return 3U;
    }

    return 0U;
}

static const char *BSP_DebugUartPortName(uint8_t port_id)
{
    switch (port_id)
    {
        case 1U:
            return "USART1/PB7";
        case 2U:
            return "USART2/PD6";
        case 3U:
            return "USART3/PB11";
        default:
            return "UART/irq";
    }
}

static void BSP_DebugUartRxPush(uint8_t port_id, uint8_t byte)
{
    uint32_t next_head = (g_debug_uart_rx_head + 1U) % DEBUG_UART_RX_FIFO_SIZE;

    if (next_head == g_debug_uart_rx_tail)
    {
        g_debug_uart_rx_drop_count++;
        return;
    }

    g_debug_uart_rx_fifo[g_debug_uart_rx_head] =
        ((uint16_t)port_id << 8) | (uint16_t)byte;
    g_debug_uart_rx_head = next_head;
    g_debug_uart_rx_irq_count++;
}

static bool BSP_DebugUartRxPop(uint16_t *event)
{
    if (g_debug_uart_rx_tail == g_debug_uart_rx_head)
    {
        return false;
    }

    *event = g_debug_uart_rx_fifo[g_debug_uart_rx_tail];
    g_debug_uart_rx_tail =
        (g_debug_uart_rx_tail + 1U) % DEBUG_UART_RX_FIFO_SIZE;

    return true;
}

void BSP_DebugUartIrqHandler(USART_TypeDef *uart)
{
    uint32_t sr = uart->SR;
    uint8_t port_id = BSP_DebugUartPortId(uart);

    if ((sr & (USART_SR_ORE | USART_SR_NE | USART_SR_FE | USART_SR_PE)) != 0U)
    {
        volatile uint32_t dummy = uart->DR;
        (void)dummy;
        g_debug_uart_rx_drop_count++;
        return;
    }

    if ((sr & USART_SR_RXNE) != 0U)
    {
        BSP_DebugUartRxPush(port_id, (uint8_t)uart->DR);
    }
}

/* 输出缓存值而不主动访问 SPI，避免调试打印干扰实时控制周期。 */
static void BSP_DebugPrintCachedAs5047(void)
{
    const FOC_AS5047P_t *as5047 = FOC_AS5047P_GetState();
    int mech_deg10 = (int)(as5047->mech_angle * 1800.0f / 6.28318530718f);
    int el_deg10 = (int)(as5047->el_angle * 1800.0f / 6.28318530718f);
    int zero_deg10 = (int)(as5047->zero_el_angle * 1800.0f / 6.28318530718f);

    printf("[AS5047] cached raw=%u frame=0x%04X mech_x10=%d el_x10=%d zero_x10=%d dir=%d rpm=%d mag=%u agc=%u err=%lu samples=%lu\r\n",
           (unsigned int)as5047->raw,
           (unsigned int)as5047->angle_frame,
           mech_deg10,
           el_deg10,
           zero_deg10,
           (as5047->dir < 0.0f) ? -1 : 1,
           (int)as5047->speed_rpm,
           (unsigned int)as5047->mag,
           (unsigned int)as5047->agc,
           (unsigned long)as5047->error_count,
           (unsigned long)as5047->sample_count);
}

/* 三种闭环共用的硬件准备过程。切换模式时先停止 PWM 并清除控制器、对齐状态，
 * 防止上一模式的积分量或目标值带入新模式。 */
static void BSP_FOC_PrepareMode(uint32_t mode, float iq_setting)
{
    FOC_Debug_SetRunRequest(false);
    FOC_Debug_SetAlignOnly(false);
    FOC_Debug_SetPhaseMap(BSP_FOC_PHASE_MAP);
    FOC_Debug_SetEncoderMode(mode);
    FOC_Debug_SetPwmMode(0U);
    FOC_AS5047P_SetDirection(BSP_FOC_ENCODER_DIR);
    FOC_Debug_SetEncoderSpeedIqDir(BSP_FOC_IQ_DIR);
    FOC_Debug_SetEncoderIqRef(iq_setting);
    FOC_Debug_SetEncoderIdRef(0.0f);
    FOC_Debug_SetCurrentCsv(false);
}

/* 闭环 1：转矩（电流）闭环。
 * iq_amp 是带符号的 q 轴电流目标；绝对值决定转矩大小，符号决定转矩方向。 */
void BSP_FOC_StartTorque(float iq_amp)
{
    BSP_FOC_PrepareMode(FOC_CONTROL_TORQUE, iq_amp);
    FOC_Debug_SetRunRequest(true);
    printf("[FOC_MODE] torque-current iq=%dmA phase=5 encdir=1 iqdir=1\r\n",
           (int)(iq_amp * 1000.0f));
}

/* 闭环 2：速度-电流串级闭环。
 * target_rpm -> 速度 PI -> Iq 目标 -> 电流 PI -> SVPWM。
 * iq_limit_amp 限制速度 PI 输出，也就是限制最大转矩。 */
void BSP_FOC_StartSpeed(float target_rpm, float iq_limit_amp)
{
    if (iq_limit_amp < 0.0f)
    {
        iq_limit_amp = -iq_limit_amp;
    }

    BSP_FOC_PrepareMode(FOC_CONTROL_SPEED, iq_limit_amp);
    FOC_Debug_SetTargetSpeed(target_rpm);
    FOC_Debug_SetRunRequest(true);
    printf("[FOC_MODE] speed-current target=%drpm iqLimit=%dmA\r\n",
           (int)target_rpm,
           (int)(iq_limit_amp * 1000.0f));
}

/* 闭环 3：位置-速度-电流串级闭环。
 * target_deg -> 位置 P -> 速度目标 -> 速度 PI -> Iq 目标 -> 电流 PI -> SVPWM。
 * target_deg 表示多圈累计机械位置，单位为度。 */
void BSP_FOC_StartPosition(float target_deg, float iq_limit_amp)
{
    if (iq_limit_amp < 0.0f)
    {
        iq_limit_amp = -iq_limit_amp;
    }

    BSP_FOC_PrepareMode(FOC_CONTROL_POSITION, iq_limit_amp);
    FOC_Debug_SetTargetPositionDeg(target_deg);
    FOC_Debug_SetRunRequest(true);
    printf("[FOC_MODE] position-speed-current target_x10=%d iqLimit=%dmA\r\n",
           (int)(target_deg * 10.0f),
           (int)(iq_limit_amp * 1000.0f));
}

void BSP_FOC_Stop(void)
{
    /* 统一停止入口：关闭运行请求，FOC 状态机随后撤销 PWM。 */
    FOC_Debug_SetRunRequest(false);
    printf("[FOC_MODE] stopped\r\n");
}

static void BSP_DebugStartTorque50(void)
{
    BSP_FOC_StartTorque(BSP_FOC_DEFAULT_TORQUE_IQ_A);
}

static void BSP_DebugStartSpeedPreset(float target_speed, float iq_limit, const char *name)
{
    /* 重新配置前先停止一次, 避免上一次电流环/速度环积分状态残留. */

    BSP_FOC_StartSpeed(target_speed, iq_limit);
    printf("[UART_CMD] preset %s applied\r\n", name);
}

static void BSP_DebugStartSpeed50(void)
{
    BSP_DebugStartSpeedPreset(50.0f, 0.05f, "speed50");
}

/* 原子化低速闭环测试：部分串口助手连续发送多条命令时可能溢出或丢字节，
 * 导致模式停留在VF。该预设用一条命令写入整套已验证配置。 */
static void BSP_DebugStartSpeed5Test(void)
{
    BSP_FOC_StartSpeed(BSP_FOC_DEFAULT_SPEED_RPM,
                       BSP_FOC_DEFAULT_IQ_LIMIT_A);
    printf("[SPEED_TEST] mode=speed target=300rpm iqLimit=100mA Kp=0.00010 Ki=0.00005 startBoost=OFF iqSlew=0.25A/s\r\n");
}

static void BSP_DebugStartRev50(void)
{
    BSP_DebugStartSpeedPreset(-50.0f, 0.08f, "rev50");
}

static void BSP_DebugStartFwd80(void)
{
    BSP_DebugStartSpeedPreset(80.0f, 0.08f, "fwd80");
}

static void BSP_DebugStartPositionTest(void)
{
    /* ptest 是位置-速度-电流三级闭环的一键基线测试。
     * 必须先启动闭环、再打开曲线：通用启动函数会主动关闭上一个模式的日志，
     * 如果顺序相反，位置曲线设置会被清理，只剩普通 [FOC] 调试文本。 */
    BSP_FOC_StartPosition(BSP_FOC_DEFAULT_POSITION_DEG,
                           BSP_FOC_DEFAULT_IQ_LIMIT_A);
    FOC_Debug_SetVofaMode(FOC_VOFA_POSITION);
    printf("[POSITION_TEST] phase=5 encdir=1 iqdir=1 target=360deg iqLimit=100mA "
           "Kp=2.00 Ki=0 speedLimit=200rpm accel=600rpm/s decel=1200rpm/s "
           "staticAssist=OFF "
           "softHold=1.5/3deg@10rpm crossBrake=1; VOFA position log=50Hz\r\n");
}

/* 第一阶段：仅调试电流环。d轴阶跃建立电流但不产生持续转矩，
 * 因此转子应完成对齐并保持，而不是持续加速。 */
static void BSP_DebugStartCurrentTest(void)
{
    FOC_Debug_SetRunRequest(false);
    FOC_Debug_SetAlignOnly(false);
    /* 电流环测试必须沿用实机验证过的相序和编码器方向。
     * 若在测试函数内另写一套常量，Park 变换方向会与正常运行不一致，
     * 测得的 PI 曲线就没有可比性。 */
    FOC_Debug_SetPhaseMap(BSP_FOC_PHASE_MAP);
    FOC_Debug_SetEncoderMode(FOC_CONTROL_TORQUE);
    FOC_Debug_SetPwmMode(0U);
    FOC_AS5047P_SetDirection(BSP_FOC_ENCODER_DIR);
    FOC_Debug_SetEncoderSpeedIqDir(BSP_FOC_IQ_DIR);
    FOC_Debug_SetEncoderIqRef(0.0f);
    FOC_Debug_SetEncoderIdRef(0.03f);
    FOC_Debug_SetCurrentCsv(true);
    FOC_Debug_SetRunRequest(true);
    printf("[CURRENT_TEST] stage=1 stationary d-axis test iq=0mA id=30mA phase=%lu encdir=%d\r\n",
           (unsigned long)BSP_FOC_PHASE_MAP,
           (int)BSP_FOC_ENCODER_DIR);
}

/* 第一阶段补充：验证q轴电流环。每个预设都从停机并复位后的状态启动，
 * 防止前一次100 mA测试的转子惯性影响后续较小电流测试。 */
static void BSP_DebugStartQCurrentLevel(int32_t iq_mA)
{
    BSP_FOC_StartTorque((float)iq_mA / 1000.0f);
    printf("[CURRENT_TEST] stage=1b mode=torque iq=%ldmA id=0mA phase=5 encdir=1; independent cold-start level\r\n",
           (long)iq_mA);
}

static void BSP_DebugStartQCurrentTest(void)
{
    BSP_DebugStartQCurrentLevel(100);
}

static void BSP_DebugStartAlignTest(void)
{
    FOC_Debug_SetRunRequest(false);
    FOC_Debug_SetEncoderMode(FOC_CONTROL_POSITION);
    FOC_Debug_SetPwmMode(0U);
    FOC_Debug_SetEncoderIqRef(0.03f);
    FOC_Debug_SetEncoderIdRef(0.0f);
    FOC_Debug_SetTargetPositionDeg(0.0f);
    FOC_Debug_SetAlignOnly(true);
    FOC_Debug_SetRunRequest(true);
    printf("[UART_CMD] aligntest phase=%lu; auto-stop after alignment\r\n",
           (unsigned long)FOC_Debug_GetPhaseMap());
}

/* 命令分发入口：把无参数命令和 key=value 参数映射为具体控制操作。 */
static void BSP_DebugUartExecuteCommand(const char *port, const char *cmd)
{
    if ((strcmp(cmd, "alignresults") == 0) ||
        (strcmp(cmd, "ar?") == 0))
    {
        FOC_Motor_PrintAlignResults();
    }
    else if (strcmp(cmd, "aligntest") == 0)
    {
        BSP_DebugStartAlignTest();
    }
    else if ((strcmp(cmd, "torque") == 0) ||
             (strcmp(cmd, "loop1") == 0))
    {
        BSP_FOC_StartTorque(BSP_FOC_DEFAULT_TORQUE_IQ_A);
    }
    else if ((strcmp(cmd, "speed") == 0) ||
             (strcmp(cmd, "loop2") == 0))
    {
        BSP_FOC_StartSpeed(BSP_FOC_DEFAULT_SPEED_RPM,
                           BSP_FOC_DEFAULT_IQ_LIMIT_A);
    }
    else if ((strcmp(cmd, "position") == 0) ||
             (strcmp(cmd, "loop3") == 0))
    {
        BSP_FOC_StartPosition(BSP_FOC_DEFAULT_POSITION_DEG,
                              BSP_FOC_DEFAULT_IQ_LIMIT_A);
    }
    else if (strcmp(cmd, "ptest") == 0)
    {
        BSP_DebugStartPositionTest();
    }
    else if ((strcmp(cmd, "stest") == 0) ||
             (strcmp(cmd, "speed5") == 0) ||
             (strcmp(cmd, "spd5") == 0))
    {
        BSP_DebugStartSpeed5Test();
    }
    else if ((strcmp(cmd, "ctest") == 0) ||
             (strcmp(cmd, "currenttest") == 0))
    {
        BSP_DebugStartCurrentTest();
    }
    else if ((strcmp(cmd, "qtest") == 0) ||
             (strcmp(cmd, "iqtest") == 0))
    {
        BSP_DebugStartQCurrentTest();
    }
    else if (strcmp(cmd, "q50") == 0)
    {
        BSP_DebugStartQCurrentLevel(50);
    }
    else if (strcmp(cmd, "q40") == 0)
    {
        BSP_DebugStartQCurrentLevel(40);
    }
    else if (strcmp(cmd, "q30") == 0)
    {
        BSP_DebugStartQCurrentLevel(30);
    }
    else if (strcmp(cmd, "q20") == 0)
    {
        BSP_DebugStartQCurrentLevel(20);
    }
    else if ((strcmp(cmd, "vlog=current") == 0) ||
             (strcmp(cmd, "vlog=iq") == 0))
    {
        FOC_Debug_SetVofaMode(FOC_VOFA_CURRENT);
    }
    else if ((strcmp(cmd, "vlog=speed") == 0) ||
             (strcmp(cmd, "vlog=spd") == 0))
    {
        FOC_Debug_SetVofaMode(FOC_VOFA_SPEED);
    }
    else if ((strcmp(cmd, "vlog=position") == 0) ||
             (strcmp(cmd, "vlog=pos") == 0))
    {
        FOC_Debug_SetVofaMode(FOC_VOFA_POSITION);
    }
    else if ((strcmp(cmd, "vlog=auto") == 0) ||
             (strcmp(cmd, "vlog=1") == 0))
    {
        FOC_Debug_SetVofaMode(FOC_VOFA_AUTO);
    }
    else if ((strcmp(cmd, "vlog=off") == 0) ||
             (strcmp(cmd, "vlog=0") == 0))
    {
        FOC_Debug_SetVofaMode(FOC_VOFA_OFF);
    }
    else if ((strcmp(cmd, "clog=1") == 0) ||
             (strcmp(cmd, "clog=on") == 0))
    {
        FOC_Debug_SetCurrentCsv(true);
    }
    else if ((strcmp(cmd, "clog=0") == 0) ||
             (strcmp(cmd, "clog=off") == 0))
    {
        FOC_Debug_SetCurrentCsv(false);
    }
    else if ((strcmp(cmd, "1") == 0) ||
        (strcmp(cmd, "run") == 0) ||
        (strcmp(cmd, "start") == 0))
    {
        printf("[UART_CMD] %s cmd=%s -> run ON\r\n", port, cmd);
        FOC_Debug_SetRunRequest(true);
    }
    else if ((strcmp(cmd, "0") == 0) ||
             (strcmp(cmd, "stop") == 0) ||
             (strcmp(cmd, "off") == 0))
    {
        printf("[UART_CMD] %s cmd=%s -> run OFF\r\n", port, cmd);
        FOC_Debug_SetRunRequest(false);
    }
    else if ((strcmp(cmd, "2") == 0) ||
             (strcmp(cmd, "speed50") == 0) ||
             (strcmp(cmd, "spd50") == 0) ||
             (strcmp(cmd, "s50") == 0))
    {
        printf("[UART_CMD] %s cmd=%s -> preset speed50\r\n", port, cmd);
        BSP_DebugStartSpeed50();
    }
    else if ((strcmp(cmd, "3") == 0) ||
             (strcmp(cmd, "torque50") == 0) ||
             (strcmp(cmd, "enc50") == 0) ||
             (strcmp(cmd, "t50") == 0))
    {
        printf("[UART_CMD] %s cmd=%s -> preset torque50\r\n", port, cmd);
        BSP_DebugStartTorque50();
    }
    else if ((strcmp(cmd, "4") == 0) ||
             (strcmp(cmd, "rev50") == 0) ||
             (strcmp(cmd, "r50") == 0))
    {
        printf("[UART_CMD] %s cmd=%s -> preset rev50\r\n", port, cmd);
        BSP_DebugStartRev50();
    }
    else if ((strcmp(cmd, "5") == 0) ||
             (strcmp(cmd, "fwd80") == 0) ||
             (strcmp(cmd, "f80") == 0))
    {
        printf("[UART_CMD] %s cmd=%s -> preset fwd80\r\n", port, cmd);
        BSP_DebugStartFwd80();
    }
    else if ((strncmp(cmd, "spd=", 4) == 0) ||
             (strncmp(cmd, "speed=", 6) == 0) ||
             (strncmp(cmd, "target=", 7) == 0))
    {
        const char *value = strchr(cmd, '=');

        if (value != NULL)
        {
            FOC_Debug_SetTargetSpeed((float)atoi(value + 1));
        }
    }
    else if ((strncmp(cmd, "pos=", 4) == 0) ||
             (strncmp(cmd, "position=", 9) == 0) ||
             (strncmp(cmd, "angle=", 6) == 0))
    {
        const char *value = strchr(cmd, '=');

        if (value != NULL)
        {
            /* 位置指令按整数机械角度解析。速度命令同样使用 atoi，实机串口测试
             * 更稳定；同时避免部分 ARM C 库配置下 strtof 的解析结果异常。
             * 例：pos=360 必须得到 360 deg，而不是遥测中的 360(即36.0deg)。 */
            int target_deg = atoi(value + 1);

            FOC_Debug_SetTargetPositionDeg((float)target_deg);
            printf("[UART_CMD] position=%d deg\r\n", target_deg);
        }
    }
    else if ((strcmp(cmd, "pos?") == 0) ||
             (strcmp(cmd, "position?") == 0))
    {
        printf("[POSITION] target_x10=%d actual_x10=%d speedRef_x10=%d\r\n",
               (int)(FOC_Debug_GetTargetPositionDeg() * 10.0f),
               (int)(FOC_AS5047P_GetPositionDeg() * 10.0f),
               (int)(FOC_MOTOR.position_speed_ref * 10.0f));
    }
    else if ((strncmp(cmd, "alignphase=", 11) == 0) ||
             (strncmp(cmd, "ap=", 3) == 0))
    {
        const char *value = strchr(cmd, '=');

        if (value != NULL)
        {
            FOC_Debug_SetPhaseMap((uint32_t)atoi(value + 1));
            BSP_DebugStartAlignTest();
        }
    }
    else if ((strncmp(cmd, "phase=", 6) == 0) ||
             (strncmp(cmd, "ph=", 3) == 0))
    {
        const char *value = strchr(cmd, '=');

        if (value != NULL)
        {
            FOC_Debug_SetPhaseMap((uint32_t)atoi(value + 1));
        }
    }
    else if ((strncmp(cmd, "vq=", 3) == 0) ||
             (strncmp(cmd, "uq=", 3) == 0))
    {
        const char *value = strchr(cmd, '=');

        if (value != NULL)
        {
            FOC_Debug_SetVfUq((float)atoi(value + 1) / 1000.0f);
        }
    }
    else if ((strncmp(cmd, "vd=", 3) == 0) ||
             (strncmp(cmd, "ud=", 3) == 0))
    {
        const char *value = strchr(cmd, '=');

        if (value != NULL)
        {
            FOC_Debug_SetVfUd((float)atoi(value + 1) / 1000.0f);
        }
    }
    else if (strncmp(cmd, "pwm=", 4) == 0)
    {
        const char *value = strchr(cmd, '=');

        if (value != NULL)
        {
            value++;
            if ((strcmp(value, "spwm") == 0) ||
                (strcmp(value, "sin") == 0) ||
                (strcmp(value, "1") == 0))
            {
                FOC_Debug_SetPwmMode(1U);
            }
            else
            {
                FOC_Debug_SetPwmMode(0U);
            }
        }
    }
    else if (strncmp(cmd, "mode=", 5) == 0)
    {
        const char *value = strchr(cmd, '=');

        if (value != NULL)
        {
            value++;
            if ((strcmp(value, "speed") == 0) ||
                (strcmp(value, "spd") == 0) ||
                (strcmp(value, "vel") == 0) ||
                (strcmp(value, "2") == 0))
            {
                FOC_Debug_SetEncoderMode(2U);
            }
            else if ((strcmp(value, "position") == 0) ||
                     (strcmp(value, "pos") == 0) ||
                     (strcmp(value, "angle") == 0) ||
                     (strcmp(value, "3") == 0))
            {
                FOC_Debug_SetEncoderMode(FOC_CONTROL_POSITION);
            }
            else if ((strcmp(value, "enc") == 0) ||
                     (strcmp(value, "torque") == 0) ||
                     (strcmp(value, "current") == 0) ||
                     (strcmp(value, "as") == 0) ||
                     (strcmp(value, "as5047") == 0) ||
                     (strcmp(value, "1") == 0))
            {
                FOC_Debug_SetEncoderMode(FOC_CONTROL_TORQUE);
            }
            else
            {
                FOC_Debug_SetEncoderMode(0U);
            }
        }
    }
    else if ((strncmp(cmd, "iq=", 3) == 0) ||
             (strncmp(cmd, "i=", 2) == 0))
    {
        const char *value = strchr(cmd, '=');

        if (value != NULL)
        {
            FOC_Debug_SetEncoderIqRef((float)atoi(value + 1) / 1000.0f);
        }
    }
    else if (strncmp(cmd, "id=", 3) == 0)
    {
        const char *value = strchr(cmd, '=');

        if (value != NULL)
        {
            FOC_Debug_SetEncoderIdRef((float)atoi(value + 1) / 1000.0f);
        }
    }
    else if ((strncmp(cmd, "iqdir=", 6) == 0) ||
             (strncmp(cmd, "tdir=", 5) == 0))
    {
        const char *value = strchr(cmd, '=');

        if (value != NULL)
        {
            if ((FOC_Debug_GetEncoderMode() != 0U) && FOC_Debug_GetRunRequest())
            {
                printf("[UART_CMD] stop before iqdir\r\n");
            }
            else
            {
                FOC_Debug_SetEncoderSpeedIqDir((float)atoi(value + 1));
            }
        }
    }
    else if ((strcmp(cmd, "enc?") == 0) ||
             (strcmp(cmd, "enc") == 0) ||
             (strcmp(cmd, "as?") == 0) ||
             (strcmp(cmd, "as") == 0) ||
             (strcmp(cmd, "as5047") == 0))
    {
        if ((FOC_Debug_GetEncoderMode() != 0U) && FOC_Debug_GetRunRequest())
        {
            BSP_DebugPrintCachedAs5047();
        }
        else
        {
            FOC_AS5047P_PrintStatus();
        }
    }
    else if ((strcmp(cmd, "enczero") == 0) ||
             (strcmp(cmd, "zeroenc") == 0))
    {
        if ((FOC_Debug_GetEncoderMode() != 0U) && FOC_Debug_GetRunRequest())
        {
            printf("[UART_CMD] stop before enczero\r\n");
        }
        else
        {
            FOC_AS5047P_SetZeroCurrent();
            FOC_AS5047P_PrintStatus();
        }
    }
    else if (strncmp(cmd, "encdir=", 7) == 0)
    {
        const char *value = strchr(cmd, '=');

        if (value != NULL)
        {
            if ((FOC_Debug_GetEncoderMode() != 0U) && FOC_Debug_GetRunRequest())
            {
                printf("[UART_CMD] stop before encdir\r\n");
            }
            else
            {
                FOC_AS5047P_SetDirection((float)atoi(value + 1));
                FOC_AS5047P_PrintStatus();
            }
        }
    }
    else if (strcmp(cmd, "+") == 0)
    {
        FOC_Debug_AdjustTargetSpeed(10.0f);
    }
    else if (strcmp(cmd, "-") == 0)
    {
        FOC_Debug_AdjustTargetSpeed(-10.0f);
    }
    else if (strcmp(cmd, "?") == 0)
    {
        printf("[UART_CMD] loops: torque/loop1, speed/loop2, position/loop3; targets: iq=<mA>, spd=<rpm>, pos=<deg>; stop; tests: qtest/stest/ptest; VOFA: vlog=current/speed/position/auto/off (clog=1/0 compatible); help: ?\r\n");
        FOC_Debug_PrintStatus();
    }
    else
    {
        printf("[UART_CMD] %s unknown cmd=%s\r\n", port, cmd);
    }
}

/* 按空白字符切分命令；超长命令会被丢弃，避免写越界。 */
static void BSP_DebugUartHandleByte(const char *port, uint8_t byte)
{
    /* 不逐字节打印。逐字节阻塞式 printf 会占满串口带宽，连续发送命令时容易丢字节。 */
    if ((byte == '\r') || (byte == '\n') || (byte == ' ') || (byte == '\t'))
    {
        if (g_debug_uart_cmd_len > 0U)
        {
            g_debug_uart_cmd[g_debug_uart_cmd_len] = '\0';
            BSP_DebugUartExecuteCommand(port, g_debug_uart_cmd);
            g_debug_uart_cmd_len = 0U;
        }
        return;
    }

    if ((g_debug_uart_cmd_len == 0U) &&
        ((byte == '1') || (byte == '0') ||
         (byte == '2') || (byte == '3') ||
         (byte == '4') || (byte == '5') ||
         (byte == '+') || (byte == '-') || (byte == '?')))
    {
        char cmd[2];

        cmd[0] = (char)byte;
        cmd[1] = '\0';
        g_debug_uart_cmd_len = 0U;
        BSP_DebugUartExecuteCommand(port, cmd);
        return;
    }

    if ((byte >= 'A') && (byte <= 'Z'))
    {
        byte = (uint8_t)(byte - 'A' + 'a');
    }
    if ((byte == '?') && (g_debug_uart_cmd_len > 0U))
    {
        if (g_debug_uart_cmd_len < (sizeof(g_debug_uart_cmd) - 1U))
        {
            g_debug_uart_cmd[g_debug_uart_cmd_len] = '?';
            g_debug_uart_cmd_len++;
        }

        g_debug_uart_cmd[g_debug_uart_cmd_len] = '\0';
        BSP_DebugUartExecuteCommand(port, g_debug_uart_cmd);
        g_debug_uart_cmd_len = 0U;
        return;
    }

    if (g_debug_uart_cmd_len < (sizeof(g_debug_uart_cmd) - 1U))
    {
        g_debug_uart_cmd[g_debug_uart_cmd_len] = (char)byte;
        g_debug_uart_cmd_len++;
    }
    else
    {
        g_debug_uart_cmd_len = 0U;
        printf("[UART_CMD] %s cmd buffer overflow\r\n", port);
    }
}

static void BSP_DebugUartPollOne(USART_TypeDef *uart, const char *port)
{
    uint32_t guard = 16U;

    while (guard > 0U)
    {
        uint32_t sr = uart->SR;

        if ((sr & (USART_SR_ORE | USART_SR_NE | USART_SR_FE | USART_SR_PE)) != 0U)
        {
            volatile uint32_t dummy = uart->DR;
            (void)dummy;
            printf("[UART_CMD] %s rx error SR=0x%04lX\r\n", port, (unsigned long)sr);
            guard--;
            continue;
        }

        if ((sr & USART_SR_RXNE) == 0U)
        {
            break;
        }

        BSP_DebugUartHandleByte(port, (uint8_t)uart->DR);
        guard--;
    }
}

void BSP_DebugUartPollCommands(void)
{
    uint16_t rx_event;

    while (BSP_DebugUartRxPop(&rx_event))
    {
        uint8_t byte = (uint8_t)(rx_event & 0x00FFU);
        uint8_t port_id = (uint8_t)(rx_event >> 8);

        BSP_DebugUartHandleByte(BSP_DebugUartPortName(port_id), byte);
    }

    if (__HAL_RCC_USART1_IS_CLK_ENABLED())
    {
        BSP_DebugUartPollOne(USART1, "USART1/PB7");
    }

    if (__HAL_RCC_USART2_IS_CLK_ENABLED())
    {
        BSP_DebugUartPollOne(USART2, "USART2/PD6");
    }

    if (__HAL_RCC_USART3_IS_CLK_ENABLED())
    {
        BSP_DebugUartPollOne(USART3, "USART3/PB11");
    }

    /* 命令只在收到换行符或空白分隔符后执行。
     * 不再使用 50 ms 空闲超时，避免将传输较慢的半条命令提前执行。 */
}
/* 板级初始化顺序：系统延时、调试串口、按键，最后启动 FOC 模块。 */
void BSP_Init(void)
{

    delay_init(168);
    uart_init(115200);

    delay_ms(100);

    BSP_KeyInit();
//    MX_USB_Device_Init();

    printf("\r\n[BOOT] UART debug ready: USART1 PB6/PB7, USART2 PD5/PD6, USART3 PB10/PB11, 115200 8N1\r\n");
    printf("BSP_Init: OK\r\n");


    FOC_Init();
    printf("[UART_CMD] commands: run/stop, mode=position/speed/torque/vf, pos=<deg>, pos?, spd=<rpm>, iq=<mA>, id=<mA>, speed50, torque50, phase=0..5, pwm=svpwm/spwm, enc?, enczero, encdir=1/-1, ?\r\n");
   

    
}








