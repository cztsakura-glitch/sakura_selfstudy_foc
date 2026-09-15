/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           USB_Device/CDC_Standalone/USB_Device/Target/usbd_conf.h
  * @author         MCD Application Team
  * @brief          Header for usbd_conf.c file.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USBD_CONF__H__
#define __USBD_CONF__H__

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "stm32g4xx.h"
#include "stm32g4xx_hal.h"
#include "sys.h"
     
#define  USBD_DEBUG(...)  
// #define  USBD_DEBUG(...)   printf(__VA_ARGS__);

/*---------- -----------*/
#define USBD_MAX_NUM_INTERFACES         1U
/*---------- -----------*/
#define USBD_MAX_NUM_CONFIGURATION      1U
/*---------- -----------*/
#define USBD_MAX_STR_DESC_SIZ           0x100U
/*---------- -----------*/
#define USBD_DEBUG_LEVEL                0U
/*---------- -----------*/
#define USBD_LPM_ENABLED                0U
/*---------- -----------*/
#define USBD_SELF_POWERED               1U

/****************************************/
/* #define for FS and HS identification */
#define DEVICE_FS                       0

#define USBD_malloc(x)              mymalloc(SRAMIN,x)  

/** Alias for memory release. */
#define USBD_free(x)                myfree(SRAMIN,x)

/** Alias for memory set. */
#define USBD_memset                 memset

/** Alias for memory copy. */
#define USBD_memcpy                 memcpy

/** Alias for delay. */
#define USBD_Delay                  delay_ms

/* DEBUG macros */

#if (USBD_DEBUG_LEVEL > 0)
#define USBD_UsrLog(...)    printf(__VA_ARGS__);\
                            printf("\n");
#else
#define USBD_UsrLog(...)
#endif

#if (USBD_DEBUG_LEVEL > 1)

#define USBD_ErrLog(...)    printf("ERROR: ") ;\
                            printf(__VA_ARGS__);\
                            printf("\n");
#else
#define USBD_ErrLog(...)
#endif

#if (USBD_DEBUG_LEVEL > 2)
#define USBD_DbgLog(...)    printf("DEBUG : ") ;\
                            printf(__VA_ARGS__);\
                            printf("\n");
#else
#define USBD_DbgLog(...)
#endif

/* Exported functions -------------------------------------------------------*/
void *USBD_static_malloc(uint32_t size);
void USBD_static_free(void *p);

#ifdef __cplusplus
}
#endif

#endif /* __USBD_CONF__H__ */

