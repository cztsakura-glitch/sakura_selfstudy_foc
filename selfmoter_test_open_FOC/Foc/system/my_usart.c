/**
 ****************************************************************************************************
 * @file        usart.c
 * @author      正点原子团队(ALIENTEK)
 * @version     V1.0
 * @date        2023-08-01
 * @brief       串口初始化代码(一般是串口1)，支持printf
 * @license     Copyright (c) 2020-2032, 广州市星翼电子科技有限公司
 ****************************************************************************************************
 * @attention
 *
 * 实验平台:正点原子 STM32G474开发板
 * 在线视频:www.yuanzige.com
 * 技术论坛:www.openedv.com
 * 公司网址:www.alientek.com
 * 购买地址:openedv.taobao.com
 *
 * 修改说明
 * V1.0 20230801
 * 第一次发布
 *
 ****************************************************************************************************
 */
#include "my_usart.h"
#include "sys.h"

//#include "usart.h"



/******************************************************************************************/
/* 加入以下代码, 支持printf函数, 而不需要选择use MicroLIB */

#if 1
#if (__ARMCC_VERSION >= 6010050)                    /* 使用AC6编译器时 */
__asm(".global __use_no_semihosting\n\t");          /* 声明不使用半主机模式 */
__asm(".global __ARM_use_no_argv \n\t");            /* AC6下需要声明main函数为无参数格式，否则部分例程可能出现半主机模式 */

#else
/* 使用AC5编译器时, 要在这里定义__FILE 和 不使用半主机模式 */
#pragma import(__use_no_semihosting)

struct __FILE
{
    int handle;
    /* Whatever you require here. If the only file you are using is */
    /* standard output using printf() for debugging, no file handling */
    /* is required. */
};

#endif

/* 不使用半主机模式，至少需要重定义_ttywrch\_sys_exit\_sys_command_string函数,以同时兼容AC6和AC5模式 */
int _ttywrch(int ch)
{
    ch = ch;
    return ch;
}

/* 定义_sys_exit()以避免使用半主机模式 */
void _sys_exit(int x)
{
    x = x;
}

char *_sys_command_string(char *cmd, int len)
{
    return NULL;
}

/* FILE 在 stdio.h里面定义. */
FILE __stdout;

/* 重定义fputc函数, printf函数最终会通过调用fputc输出字符串到串口 */
int fputc(int ch, FILE *f)
{
    while ((USART1->ISR & 0X40) == 0);              /* 等待上一个字符发送完成 */

    USART1->TDR = (uint8_t)ch;                      /* 将要发送的字符 ch 写入到TDR寄存器 */
    return ch;
}
#endif
/***********************************************END*******************************************/

uint8_t usart1_rxbuf_byte;  /*单个字节的buf*/
uint8_t usart1_rxbuf[RX_BUF_SZIE];
void uart_init(uint32_t baudrate)
{
    huart1.Instance = USART1;                         /* USART1 */
    huart1.Init.BaudRate = baudrate;                    /* 波特率 */
    huart1.Init.WordLength = UART_WORDLENGTH_8B;        /* 字长为8位数据格式 */
    huart1.Init.StopBits = UART_STOPBITS_1;             /* 一个停止位 */
    huart1.Init.Parity = UART_PARITY_NONE;              /* 无奇偶校验位 */
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;        /* 无硬件流控 */
    huart1.Init.Mode = UART_MODE_TX_RX;                 /* 收发模式 */
    HAL_UART_Init(&huart1);                             /* HAL_UART_Init()会使能UART1 */
    
    /* 该函数会开启接收中断：标志位UART_IT_RXNE，并且设置接收缓冲以及接收缓冲接收最大数据量 */
    // HAL_UART_Receive_IT(&huart1, (uint8_t *)usart1_rxbuf_byte, 1);
}




 

 




