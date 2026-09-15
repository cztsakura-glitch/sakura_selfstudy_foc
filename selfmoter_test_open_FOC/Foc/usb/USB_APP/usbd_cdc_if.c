/**
 ****************************************************************************************************
 * @file        usbd_cdc_if.c
 * @author      正点原子团队(ALIENTEK)
 * @version     V1.0
 * @date        2023-08-01
 * @brief       USB CDC 驱动代码
 * @license     Copyright (c) 2020-2032, 广州市星翼电子科技有限公司
 ****************************************************************************************************
 * @attention
 *
 * 实验平台：正点原子 STM32G474开发板
 * 在线视频：www.yuanzige.com
 * 技术论坛：http://www.openedv.com/forum.php
 * 公司网址：www.alientek.com
 * 购买地址：zhengdianyuanzi.tmall.com
 *
 * 修改说明
 * V1.0 20230801
 * 第一次发布
 *
 ****************************************************************************************************
 */
#include "usbd_cdc_if.h"
#include "string.h"
#include "stdarg.h"
#include "stdio.h"
#include "my_usart.h"
#include "delay.h"

/* USB虚拟串口相关配置参数 */
USBD_CDC_LineCodingTypeDef LineCoding =
{
    115200,     /* 波特率 */
    0x00,       /* 停止位,默认1位 */
    0x00,       /* 校验位,默认无 */
    0x08        /* 数据位,默认8位 */
};


/* usb_printf发送缓冲区, 用于vsprintf */
uint8_t g_usb_usart_printf_buffer[USB_USART_REC_LEN];

/* USB接收的数据缓冲区,最大USART_REC_LEN个字节,用于USBD_CDC_SetRxBuffer函数 */
uint8_t g_usb_rx_buffer[USB_USART_REC_LEN];


/* 用类似串口1接收数据的方法,来处理USB虚拟串口接收到的数据 */
uint8_t g_usb_usart_rx_buffer[USB_USART_REC_LEN];       /* 接收缓冲,最大USART_REC_LEN个字节 */


/* 接收状态
 * bit15   , 接收完成标志
 * bit14   , 接收到0x0d
 * bit13~0 , 接收到的有效字节数目
 */
uint16_t g_usb_usart_rx_sta = 0;  /* 接收状态标记 */

extern USBD_HandleTypeDef hUsbDeviceFS;
static int8_t CDC_Init_FS(void);
static int8_t CDC_DeInit_FS(void);
static int8_t CDC_Control_FS(uint8_t cmd, uint8_t* pbuf, uint16_t length);
static int8_t CDC_Receive_FS(uint8_t* pbuf, uint32_t *Len);

USBD_CDC_ItfTypeDef USBD_Interface_fops_FS =
{
  CDC_Init_FS,
  CDC_DeInit_FS,
  CDC_Control_FS,
  CDC_Receive_FS,
};

/**
 * @brief       初始化 CDC
 * @param       无
 * @retval      USB状态
 *   @arg       USBD_OK(0)   , 正常;
 *   @arg       USBD_BUSY(1) , 忙;
 *   @arg       USBD_FAIL(2) , 失败;
 */
static int8_t CDC_Init_FS(void)
{
    USBD_CDC_SetRxBuffer(&hUsbDeviceFS, g_usb_rx_buffer);
    return USBD_OK;
}

/**
 * @brief       复位 CDC
 * @param       无
 * @retval      USB状态
 *   @arg       USBD_OK(0)   , 正常;
 *   @arg       USBD_BUSY(1) , 忙;
 *   @arg       USBD_FAIL(2) , 失败;
 */
static int8_t CDC_DeInit_FS(void)
{
    return USBD_OK;
}

/**
 * @brief       控制 CDC 的设置
 * @param       cmd     : 控制命令
 * @param       buf     : 命令数据缓冲区/参数保存缓冲区
 * @param       length  : 数据长度
 * @retval      USB状态
 *   @arg       USBD_OK(0)   , 正常;
 *   @arg       USBD_BUSY(1) , 忙;
 *   @arg       USBD_FAIL(2) , 失败;
 */
static int8_t CDC_Control_FS(uint8_t cmd, uint8_t* pbuf, uint16_t length)
{
    USBD_DEBUG("控制指令: 0x%x\r\n", cmd);
  /* USER CODE BEGIN 5 */
  switch (cmd)
  {
      case CDC_SEND_ENCAPSULATED_COMMAND:
        /* Add your code here */
        break;

      case CDC_GET_ENCAPSULATED_RESPONSE:
        /* Add your code here */
        break;

      case CDC_SET_COMM_FEATURE:
        /* Add your code here */
        break;

      case CDC_GET_COMM_FEATURE:
        /* Add your code here */
        break;

      case CDC_CLEAR_COMM_FEATURE:
        /* Add your code here */
        break;

      case CDC_SET_LINE_CODING:
        LineCoding.bitrate    = (uint32_t)(pbuf[0] | (pbuf[1] << 8) |\
                                (pbuf[2] << 16) | (pbuf[3] << 24));
        LineCoding.format     = pbuf[4];
        LineCoding.paritytype = pbuf[5];
        LineCoding.datatype   = pbuf[6];
        USBD_DEBUG("设置串口参数\r\n");

        /* 打印配置参数 */
        USBD_DEBUG("linecoding.format:%d\r\n", LineCoding.format);
        USBD_DEBUG("linecoding.paritytype:%d\r\n", LineCoding.paritytype);
        USBD_DEBUG("linecoding.datatype:%d\r\n", LineCoding.datatype);
        USBD_DEBUG("linecoding.bitrate:%d\r\n", LineCoding.bitrate);
        break;

      case CDC_GET_LINE_CODING:
        USBD_DEBUG("获取串口参数\r\n");

        pbuf[0] = (uint8_t)(LineCoding.bitrate);
        pbuf[1] = (uint8_t)(LineCoding.bitrate >> 8);
        pbuf[2] = (uint8_t)(LineCoding.bitrate >> 16);
        pbuf[3] = (uint8_t)(LineCoding.bitrate >> 24);
        pbuf[4] = LineCoding.format;
        pbuf[5] = LineCoding.paritytype;
        pbuf[6] = LineCoding.datatype;
        break;

      case CDC_SET_CONTROL_LINE_STATE:
        /* Add your code here */
        break;

      case CDC_SEND_BREAK:
         /* Add your code here */
        break;

      default:
        break;
  }

  return (USBD_OK);
  /* USER CODE END 5 */
}

/**
 * @brief       CDC 数据接收函数
 * @param       buf     : 接收数据缓冲区
 * @param       len     : 接收到的数据长度
 * @retval      USB状态
 *   @arg       USBD_OK(0)   , 正常;
 *   @arg       USBD_BUSY(1) , 忙;
 *   @arg       USBD_FAIL(2) , 失败;
 */
static int8_t CDC_Receive_FS(uint8_t* Buf, uint32_t *Len)
{
    USBD_CDC_ReceivePacket(&hUsbDeviceFS);
    cdc_vcp_data_rx(Buf, *Len);
    return USBD_OK;
}

/**
 * @brief       处理从 USB 虚拟串口接收到的数据
 * @param       buf     : 接收数据缓冲区
 * @param       len     : 接收到的数据长度
 * @retval      无
 */
void cdc_vcp_data_rx (uint8_t *buf, uint32_t Len)
{
    uint8_t i;
    uint8_t res;

    for (i = 0; i < Len; i++)
    {
        res = buf[i];

        if ((g_usb_usart_rx_sta & 0x8000) == 0)     /* 接收未完成 */
        {
            if (g_usb_usart_rx_sta & 0x4000)        /* 接收到了0x0d */
            {
                if (res != 0x0a)
                {
                    g_usb_usart_rx_sta = 0; /* 接收错误,重新开始 */
                }
                else
                {
                    g_usb_usart_rx_sta |= 0x8000;  /* 接收完成了 */
                }
            }
            else    /* 还没收到0x0D */
            {
                if (res == 0x0D)
                {
                    g_usb_usart_rx_sta |= 0x4000;   /* 标记接收到了0x0D */
                }
                else
                {
                    g_usb_usart_rx_buffer[g_usb_usart_rx_sta & 0x3FFF] = res;
                    g_usb_usart_rx_sta++;

                    if (g_usb_usart_rx_sta > (USB_USART_REC_LEN - 1))
                    {
                        g_usb_usart_rx_sta = 0; /* 接收数据溢出 重新开始接收 */
                    }
                }
            }
        }
    }
}

/**
 * @brief       通过 USB 发送数据
 * @param       buf     : 要发送的数据缓冲区
 * @param       len     : 数据长度
 * @retval      无
 */
void cdc_vcp_data_tx(uint8_t *data, uint32_t Len)
{
    USBD_CDC_SetTxBuffer(&hUsbDeviceFS, data, Len);
    USBD_CDC_TransmitPacket(&hUsbDeviceFS);
    // delay_ms(CDC_POLLING_INTERVAL);
}

/**
 * @brief       通过 USB 格式化输出函数
 * @note        通过USB VCP实现printf输出
 *              确保一次发送数据长度不超USB_USART_REC_LEN字节
 * @param       格式化输出
 * @retval      无
 */
void usb_printf(char *fmt, ...)
{
    uint16_t i;
    va_list ap;
    va_start(ap, fmt);
    vsprintf((char *)g_usb_usart_printf_buffer, fmt, ap);
    va_end(ap);
    i = strlen((const char *)g_usb_usart_printf_buffer);    /* 此次发送数据的长度 */
    cdc_vcp_data_tx(g_usb_usart_printf_buffer, i);          /* 发送数据 */
}



