#ifndef _USART_H
#define _USART_H
#include "sys.h"
#include "stdio.h"	

/* 调试串口接收缓冲区容量，单位为字节。 */
#define RX_BUF_SZIE     1024

/* USART1 中断接收缓冲区，由板级命令轮询逻辑读取。 */
extern uint8_t usart1_rxbuf[RX_BUF_SZIE];

/* 初始化调试串口，bound 为波特率。 */
extern void uart_init(uint32_t bound);

#endif
