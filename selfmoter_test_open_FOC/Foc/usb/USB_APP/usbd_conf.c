/**
 ****************************************************************************************************
 * @file        usbd_conf.c
 * @author      正点原子团队(ALIENTEK)
 * @version     V1.0
 * @date        2023-08-01
 * @brief       usbd_conf 驱动代码
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
#include "stm32g4xx.h"
#include "stm32g4xx_hal.h"
#include "usbd_def.h"
#include "usbd_core.h"
#include "sys.h"
#include "my_usart.h"
#include "delay.h" 
#include "usbd_cdc.h"


/* USB连接状态
 * 0,没有连接;
 * 1,已经连接;
 */
volatile uint8_t g_device_state = 0;    /* 默认没有连接 */
/* PCD定义 */
extern PCD_HandleTypeDef hpcd_USB_FS;

static USBD_StatusTypeDef USBD_Get_USB_Status(HAL_StatusTypeDef hal_status);
/* USER CODE BEGIN 1 */
static void SystemClockConfig_Resume(void);
extern void USBD_Clock_Config(void);
/* USER CODE END 1 */
extern void SystemClock_Config(void);

/*******************************************************************************
                       LL Driver Callbacks (PCD -> USB Device Library)
*******************************************************************************/


/******************************************************************************************/
/* 以下是: USBD LL PCD 驱动的回调函数(PCD->USB Device Library) */

/**
 * @brief       USBD 配置阶段回调函数
 * @param       hpcd    : PCD结构体指针
 * @retval      无
 */
void HAL_PCD_SetupStageCallback(PCD_HandleTypeDef *hpcd)
{
    USBD_LL_SetupStage((USBD_HandleTypeDef*)hpcd->pData, (uint8_t *)hpcd->Setup);
}

/**
 * @brief       USBD OUT 阶段回调函数
 * @param       hpcd    : PCD结构体指针
 * @param       epnum   : 端点号
 * @retval      无
 */
void HAL_PCD_DataOutStageCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum)
{
    USBD_LL_DataOutStage((USBD_HandleTypeDef*)hpcd->pData, epnum, hpcd->OUT_ep[epnum].xfer_buff);
}


void HAL_PCD_DataInStageCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum)
{
    USBD_LL_DataInStage((USBD_HandleTypeDef*)hpcd->pData, epnum, hpcd->IN_ep[epnum].xfer_buff);
}


void HAL_PCD_SOFCallback(PCD_HandleTypeDef *hpcd)
{
    USBD_LL_SOF((USBD_HandleTypeDef*)hpcd->pData);
}


void HAL_PCD_ResetCallback(PCD_HandleTypeDef *hpcd)
{
    USBD_SpeedTypeDef speed = USBD_SPEED_FULL;

    /* Set USB Current Speed */
    switch (hpcd->Init.speed)
    {
        case PCD_SPEED_FULL:
            printf("USB Device Library  [FS]\r\n");
            speed = USBD_SPEED_FULL;
            break;

        default:
            printf("USB Device Library  [FS?]\r\n");
            speed = USBD_SPEED_FULL;
            break;
    }
    /* Set Speed. */
    USBD_LL_SetSpeed((USBD_HandleTypeDef*)hpcd->pData, speed);

    /* Reset Device. */
    USBD_LL_Reset((USBD_HandleTypeDef*)hpcd->pData);
}



void HAL_PCD_SuspendCallback(PCD_HandleTypeDef *hpcd) 
{
    g_device_state = 0;
    printf("Device In suspend mode.\r\n");
    USBD_LL_Suspend((USBD_HandleTypeDef*)hpcd->pData);

    if (hpcd->Init.low_power_enable)
    {
        SCB->SCR |= (uint32_t)((uint32_t)(SCB_SCR_SLEEPDEEP_Msk | SCB_SCR_SLEEPONEXIT_Msk));
    }
}


void HAL_PCD_ResumeCallback(PCD_HandleTypeDef *hpcd)
{
    printf("Device Resumed\r\n");
    if (hpcd->Init.low_power_enable)
    {
        /* Reset SLEEPDEEP bit of Cortex System Control Register. */
        SCB->SCR &= (uint32_t)~((uint32_t)(SCB_SCR_SLEEPDEEP_Msk | SCB_SCR_SLEEPONEXIT_Msk));
        SystemClockConfig_Resume();
    }

    USBD_LL_Resume((USBD_HandleTypeDef*)hpcd->pData);
}


void HAL_PCD_ISOOUTIncompleteCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum)
{
    USBD_LL_IsoOUTIncomplete((USBD_HandleTypeDef*)hpcd->pData, epnum);
}

/**
 * @brief       USBD ISO IN 事务完成回调函数
 * @param       hpcd    : PCD结构体指针
 * @param       epnum   : 端点号
 * @retval      无
 */
void HAL_PCD_ISOINIncompleteCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum)
{
    USBD_LL_IsoINIncomplete((USBD_HandleTypeDef*)hpcd->pData, epnum);
}

/**
 * @brief       USBD 连接成功回调函数
 * @param       hpcd    : PCD结构体指针
 * @retval      无
 */

void HAL_PCD_ConnectCallback(PCD_HandleTypeDef *hpcd)
{
    g_device_state = 1;
    printf("连接\r\n");
    USBD_LL_DevConnected((USBD_HandleTypeDef*)hpcd->pData);
}

/**
 * @brief       USBD 断开连接回调函数
 * @param       hpcd    : PCD结构体指针
 * @retval      无
 */
void HAL_PCD_DisconnectCallback(PCD_HandleTypeDef *hpcd)
{
    g_device_state = 0;
    printf("USB Device Disconnected.\r\n");
    USBD_LL_DevDisconnected((USBD_HandleTypeDef*)hpcd->pData);
}

/*******************************************************************************
                       LL Driver Interface (USB Device Library --> PCD)
*******************************************************************************/

/**
 * @brief       USBD 底层初始化函数
 * @param       pdev    : USBD句柄指针
 * @retval      USB状态
 *   @arg       USBD_OK(0)   , 正常;
 *   @arg       USBD_BUSY(1) , 忙;
 *   @arg       USBD_FAIL(2) , 失败;
 */
USBD_StatusTypeDef USBD_LL_Init(USBD_HandleTypeDef *pdev)
{
    /* Init USB Ip. */
    hpcd_USB_FS.pData = pdev;
    /* Link the driver to the stack. */
    pdev->pData = &hpcd_USB_FS;

    hpcd_USB_FS.Instance = USB;
    hpcd_USB_FS.Init.dev_endpoints = 8;                     /* 端点数为8 */
    hpcd_USB_FS.Init.speed = PCD_SPEED_FULL;                /* USB全速(12Mbps) */     
    hpcd_USB_FS.Init.phy_itface = PCD_PHY_EMBEDDED;         /* 使用内部PHY */
    hpcd_USB_FS.Init.Sof_enable = DISABLE;                  /* 不使能SOF中断 */
    hpcd_USB_FS.Init.low_power_enable = DISABLE;            /* 不使能低功耗模式 */
    hpcd_USB_FS.Init.lpm_enable = DISABLE;
    hpcd_USB_FS.Init.battery_charging_enable = DISABLE;


    HAL_PCD_Init(&hpcd_USB_FS);

    /* USER CODE BEGIN EndPoint_Configuration */
    HAL_PCDEx_PMAConfig(&hpcd_USB_FS, 0x00 , PCD_SNG_BUF, 0x14);
    HAL_PCDEx_PMAConfig(&hpcd_USB_FS, 0x80 , PCD_SNG_BUF, 0x54);
    /* USER CODE END EndPoint_Configuration */
    /* USER CODE BEGIN EndPoint_Configuration_CDC */
    HAL_PCDEx_PMAConfig(&hpcd_USB_FS, CDC_IN_EP, PCD_SNG_BUF, 0x94);
    HAL_PCDEx_PMAConfig(&hpcd_USB_FS, CDC_OUT_EP, PCD_SNG_BUF, 0xD4);
    HAL_PCDEx_PMAConfig(&hpcd_USB_FS, CDC_CMD_EP, PCD_SNG_BUF, 0x114);
    /* USER CODE END EndPoint_Configuration_CDC */
    return USBD_OK;
}

/**
 * @brief       USBD 底层取消初始化(回复默认复位状态)函数
 * @param       pdev    : USBD句柄指针
 * @retval      USB状态
 *   @arg       USBD_OK(0)   , 正常;
 *   @arg       USBD_BUSY(1) , 忙;
 *   @arg       USBD_FAIL(2) , 失败;
 */
USBD_StatusTypeDef USBD_LL_DeInit(USBD_HandleTypeDef *pdev)
{
    HAL_StatusTypeDef hal_status = HAL_OK;
    USBD_StatusTypeDef usb_status = USBD_OK;

    hal_status = HAL_PCD_DeInit(pdev->pData);

    usb_status =  USBD_Get_USB_Status(hal_status);

    return usb_status;
}

/**
 * @brief       USBD 底层驱动开始工作
 * @param       pdev    : USBD句柄指针
 * @retval      USB状态
 *   @arg       USBD_OK(0)   , 正常;
 *   @arg       USBD_BUSY(1) , 忙;
 *   @arg       USBD_FAIL(2) , 失败;
 */
USBD_StatusTypeDef USBD_LL_Start(USBD_HandleTypeDef *pdev)
{
    HAL_StatusTypeDef hal_status = HAL_OK;
    USBD_StatusTypeDef usb_status = USBD_OK;

    hal_status = HAL_PCD_Start(pdev->pData);

    usb_status =  USBD_Get_USB_Status(hal_status);

    return usb_status;
}

/**
 * @brief       USBD 底层驱动停止工作
 * @param       pdev    : USBD句柄指针
 * @retval      USB状态
 *   @arg       USBD_OK(0)   , 正常;
 *   @arg       USBD_BUSY(1) , 忙;
 *   @arg       USBD_FAIL(2) , 失败;
 */
USBD_StatusTypeDef USBD_LL_Stop(USBD_HandleTypeDef *pdev)
{
    HAL_StatusTypeDef hal_status = HAL_OK;
    USBD_StatusTypeDef usb_status = USBD_OK;

    hal_status = HAL_PCD_Stop(pdev->pData);

    usb_status =  USBD_Get_USB_Status(hal_status);

    return usb_status;
}

/**
 * @brief       USBD 初始化(打开)某个端点
 * @param       pdev    : USBD句柄指针
 * @param       ep_addr : 端点号
 * @param       ep_type : 端点类型
 * @param       ep_mps  : 端点最大包容量(字节)
 * @retval      USB状态
 *   @arg       USBD_OK(0)   , 正常;
 *   @arg       USBD_BUSY(1) , 忙;
 *   @arg       USBD_FAIL(2) , 失败;
 */
USBD_StatusTypeDef USBD_LL_OpenEP(USBD_HandleTypeDef *pdev, uint8_t ep_addr, uint8_t ep_type, uint16_t ep_mps)
{
    HAL_StatusTypeDef hal_status = HAL_OK;
    USBD_StatusTypeDef usb_status = USBD_OK;

    hal_status = HAL_PCD_EP_Open(pdev->pData, ep_addr, ep_mps, ep_type);

    usb_status =  USBD_Get_USB_Status(hal_status);

    return usb_status;
}

/**
 * @brief       USBD 取消初始化(关闭)某个端点
 * @param       pdev    : USBD句柄指针
 * @param       ep_addr : 端点号
 * @retval      USB状态
 *   @arg       USBD_OK(0)   , 正常;
 *   @arg       USBD_BUSY(1) , 忙;
 *   @arg       USBD_FAIL(2) , 失败;
 */
USBD_StatusTypeDef USBD_LL_CloseEP(USBD_HandleTypeDef *pdev, uint8_t ep_addr)
{
    HAL_StatusTypeDef hal_status = HAL_OK;
    USBD_StatusTypeDef usb_status = USBD_OK;

    hal_status = HAL_PCD_EP_Close(pdev->pData, ep_addr);

    usb_status =  USBD_Get_USB_Status(hal_status);

    return usb_status;
}

/**
 * @brief       USBD 清空某个端点的数据
 * @param       pdev    : USBD句柄指针
 * @param       ep_addr : 端点号
 * @retval      USB状态
 *   @arg       USBD_OK(0)   , 正常;
 *   @arg       USBD_BUSY(1) , 忙;
 *   @arg       USBD_FAIL(2) , 失败;
 */
USBD_StatusTypeDef USBD_LL_FlushEP(USBD_HandleTypeDef *pdev, uint8_t ep_addr)
{
    HAL_StatusTypeDef hal_status = HAL_OK;
    USBD_StatusTypeDef usb_status = USBD_OK;

    hal_status = HAL_PCD_EP_Flush(pdev->pData, ep_addr);

    usb_status =  USBD_Get_USB_Status(hal_status);

    return usb_status;
}

/**
 * @brief       USBD 给某个端点设置一个暂停状态
 * @param       pdev    : USBD句柄指针
 * @param       ep_addr : 端点号
 * @retval      USB状态
 *   @arg       USBD_OK(0)   , 正常;
 *   @arg       USBD_BUSY(1) , 忙;
 *   @arg       USBD_FAIL(2) , 失败;
 */
USBD_StatusTypeDef USBD_LL_StallEP(USBD_HandleTypeDef *pdev, uint8_t ep_addr)
{
    HAL_StatusTypeDef hal_status = HAL_OK;
    USBD_StatusTypeDef usb_status = USBD_OK;

    hal_status = HAL_PCD_EP_SetStall(pdev->pData, ep_addr);

    usb_status =  USBD_Get_USB_Status(hal_status);

    return usb_status;
}

/**
 * @brief       USBD 取消某个端点的暂停状态
 * @param       pdev    : USBD句柄指针
 * @param       ep_addr : 端点号
 * @retval      USB状态
 *   @arg       USBD_OK(0)   , 正常;
 *   @arg       USBD_BUSY(1) , 忙;
 *   @arg       USBD_FAIL(2) , 失败;
 */
USBD_StatusTypeDef USBD_LL_ClearStallEP(USBD_HandleTypeDef *pdev, uint8_t ep_addr)
{
    HAL_StatusTypeDef hal_status = HAL_OK;
    USBD_StatusTypeDef usb_status = USBD_OK;

    hal_status = HAL_PCD_EP_ClrStall(pdev->pData, ep_addr);

    usb_status =  USBD_Get_USB_Status(hal_status);

    return usb_status;
}

/**
 * @brief       USBD 返回是否处于暂停状态
 * @param       pdev    : USBD句柄指针
 * @param       ep_addr : 端点号
 * @retval      0, 非暂停; 1, 暂停;
 */
uint8_t USBD_LL_IsStallEP(USBD_HandleTypeDef *pdev, uint8_t ep_addr)
{
    PCD_HandleTypeDef *hpcd = (PCD_HandleTypeDef*) pdev->pData;

    if((ep_addr & 0x80) == 0x80)
    {
        return hpcd->IN_ep[ep_addr & 0x7F].is_stall;
    }
    else
    {
        return hpcd->OUT_ep[ep_addr & 0x7F].is_stall;
    }
}

/**
 * @brief       USBD 为设备指定新的USB地址
 * @param       pdev    : USBD句柄指针
 * @param       dev_addr: 新的设备地址,USB1_OTG_HS/USB2_OTG_HS
 * @retval      USB状态
 *   @arg       USBD_OK(0)   , 正常;
 *   @arg       USBD_BUSY(1) , 忙;
 *   @arg       USBD_FAIL(2) , 失败;
 */
USBD_StatusTypeDef USBD_LL_SetUSBAddress(USBD_HandleTypeDef *pdev, uint8_t dev_addr)
{
    HAL_StatusTypeDef hal_status = HAL_OK;
    USBD_StatusTypeDef usb_status = USBD_OK;
    g_device_state = 1;                             /* 能执行到该函数,说明USB连接成功了 */
    hal_status = HAL_PCD_SetAddress(pdev->pData, dev_addr);

    usb_status =  USBD_Get_USB_Status(hal_status);

    return usb_status;
}

/**
 * @brief       USBD 通过端点发送数据
 * @param       pdev    : USBD句柄指针
 * @param       ep_addr : 端点号
 * @param       pbuf    : 数据缓冲区首地址
 * @param       size    : 要发送的数据大小
 * @retval      USB状态
 *   @arg       USBD_OK(0)   , 正常;
 *   @arg       USBD_BUSY(1) , 忙;
 *   @arg       USBD_FAIL(2) , 失败;
 */
USBD_StatusTypeDef USBD_LL_Transmit(USBD_HandleTypeDef *pdev, uint8_t ep_addr, uint8_t *pbuf, uint32_t size)
{
    HAL_StatusTypeDef hal_status = HAL_OK;
    USBD_StatusTypeDef usb_status = USBD_OK;

    hal_status = HAL_PCD_EP_Transmit(pdev->pData, ep_addr, pbuf, size);

    usb_status =  USBD_Get_USB_Status(hal_status);

    return usb_status;
}

/**
 * @brief       USBD 准备一个端点接收数据
 * @param       pdev    : USBD句柄指针
 * @param       ep_addr : 端点号
 * @param       pbuf    : 数据缓冲区首地址
 * @param       size    : 要接收的数据大小
 * @retval      USB状态
 *   @arg       USBD_OK(0)   , 正常;
 *   @arg       USBD_BUSY(1) , 忙;
 *   @arg       USBD_FAIL(2) , 失败;
 */
USBD_StatusTypeDef USBD_LL_PrepareReceive(USBD_HandleTypeDef *pdev, uint8_t ep_addr, uint8_t *pbuf, uint32_t size)
{
    HAL_StatusTypeDef hal_status = HAL_OK;
    USBD_StatusTypeDef usb_status = USBD_OK;

    hal_status = HAL_PCD_EP_Receive(pdev->pData, ep_addr, pbuf, size);

    usb_status =  USBD_Get_USB_Status(hal_status);

    return usb_status;
}

/**
 * @brief       USBD 获取最后一个传输包的大小
 * @param       pdev    : USBD句柄指针
 * @param       ep_addr : 端点号
 * @retval      包大小
 */
uint32_t USBD_LL_GetRxDataSize(USBD_HandleTypeDef *pdev, uint8_t ep_addr)
{
    return HAL_PCD_EP_GetRxCount((PCD_HandleTypeDef*) pdev->pData, ep_addr);
}

/**
 * @brief       USBD 延时函数(以ms为单位)
 * @param       Delay   : 延时的ms数
 * @retval      无
 */
void USBD_LL_Delay(uint32_t Delay)
{
    delay_ms(Delay);
}


/* USER CODE BEGIN 5 */
/**
  * @brief  Configures system clock after wake-up from USB resume callBack:
  *         enable HSI, PLL and select PLL as system clock source.
  * @retval None
  */
void SystemClockConfig_Resume(void)
{
    USBD_Clock_Config();
}
/* USER CODE END 5 */

/**
 * @brief       返回USB的状态(HAL状态)
 * @param       hal_status : 当前的状态
 * @retval      无
 */
USBD_StatusTypeDef USBD_Get_USB_Status(HAL_StatusTypeDef hal_status)
{
    USBD_StatusTypeDef usb_status = USBD_OK;

    switch (hal_status)
    {
        case HAL_OK :
            usb_status = USBD_OK;
            break;
        case HAL_ERROR :
            usb_status = USBD_FAIL;
            break;
        case HAL_BUSY :
            usb_status = USBD_BUSY;
            break;
        case HAL_TIMEOUT :
            usb_status = USBD_FAIL;
            break;
        default :
            usb_status = USBD_FAIL;
            break;
    }
    return usb_status;
}
