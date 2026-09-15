/**
 ****************************************************************************************************
 * @file        delay.c
 * @author      正点原子团队(ALIENTEK)
 * @version     V1.1
 * @date        2023-02-27
 * @brief       使用SysTick的普通计数模式对延迟进行管理(支持OS)
 *              提供delay_init初始化函数， delay_us和delay_ms等延时函数
 * @license     Copyright (c) 2022-2032, 广州市星翼电子科技有限公司
 ****************************************************************************************************
 * @attention
 *
 * 实验平台:正点原子 STM32开发板
 * 在线视频:www.yuanzige.com
 * 技术论坛:www.openedv.com
 * 公司网址:www.alientek.com
 * 购买地址:openedv.taobao.com
 *
 * 修改说明
 * V1.0 20221222
 * 第一次发布
 * V1.1 20230227
 * 修改SYS_SUPPORT_OS部分代码, 默认仅支持UCOSII 2.93.01版本, 其他OS请参考实现
 * 修改delay_init不再使用8分频,全部统一使用MCU时钟
 * 修改delay_us使用时钟摘取法延时, 兼容OS
 * 修改delay_ms直接使用delay_us延时实现.
 *
 ****************************************************************************************************
 */
#include "delay.h"
#include "sys.h"



static uint32_t g_fac_us = 0;       /* us延时倍乘数 */
volatile uint32_t u32SysTickCount=0;//系统自开机以来的系统Tick计数，单位 ms


/**
 * @brief     初始化延迟函数
 * @param     sysclk: 系统时钟频率, 即CPU频率(HCLK),等于系统时钟主频，单位MHz
 * @retval    无
 */  
void delay_init(uint16_t sysclk)
{
    uint32_t reload = 0;
    HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);/* SYSTICK使用外部时钟源,频率为HCLK */
    g_fac_us = sysclk;                                  /* 不论是否使用OS,g_fac_us都需要使用 */

    reload = sysclk *1000;                              /*1ms触发一次中断*/
    SysTick->CTRL|=SysTick_CTRL_TICKINT_Msk;//开启SYSTICK中断
	SysTick->LOAD=reload; 					//每1/OS_TICKS_PER_SEC秒中断一次	
	SysTick->CTRL|=SysTick_CTRL_ENABLE_Msk; //开启SYSTICK
}

/**
 * @brief     延时nus
 * @note      无论是否使用OS, 都是用时钟摘取法来做us延时
 * @param     nus: 要延时的us数
 * @note      nus取值范围: 0 ~ (2^32 / fac_us) (fac_us一般等于系统主频, 自行套入计算)
 * @retval    无
 */
void delay_us(uint32_t nus)
{
    uint32_t ticks;
	uint32_t told,tnow,tcnt=0;
	uint32_t reload=SysTick->LOAD;				//LOAD的值	    	 
	ticks=nus*g_fac_us; 						//需要的节拍数 
	told=SysTick->VAL;        				//刚进入时的计数器值
	while(1)
	{
		tnow=SysTick->VAL;	
		if(tnow!=told)
		{	    
			if(tnow<told)tcnt+=told-tnow;	//这里注意一下SYSTICK是一个递减的计数器就可以了.
			else tcnt+=reload-tnow+told;	    
 			told=tnow;
			if(tcnt>=ticks)break;			//时间超过/等于要延迟的时间,则退出.
		}  
	};

}

//延时nms
//nms:要延时的ms数
void delay_ms(uint16_t nms)
{
	uint32_t i;
	for(i=0;i<nms;i++) delay_us(1000);
}



//返 回 值: 返回当前的系统Tick值
//说    明: 获取系统自开机以来的系统Tick值【1ms/Tick】
//------------------------------------------------------------------------------
uint32_t SysGetCurrentTick(void)
{
    return u32SysTickCount;
}

//返 回 值: 设置当前的系统Tick值
//说    明: 设置系统自开机以来的系统Tick值【1ms/Tick】
//------------------------------------------------------------------------------
void SysSetCurrentTick(uint32_t *p)
{
    *p = u32SysTickCount;
}
//------------------------------------------------------------------------------
//输入参数: u32PreTick - 只能是上一次调用SysTick_GetCurrent()时获得的系统Tick值
//输出参数: 无
//返 回 值: 返回当前系统Tick(u32SysTickCount)与之前Tick(u32PreTice)的差值
//说    明: 获取自上一次获取系统Tick(u32PreTice)时已流逝的Tick数【1ms/Tick】
//------------------------------------------------------------------------------
uint32_t SysGetLapseTick(uint32_t u32PreTick)
{
    volatile uint32_t u32CurTick = u32SysTickCount;
    return ((u32CurTick >= u32PreTick) ? (u32CurTick - u32PreTick):
            (0xFFFFFFFF - u32PreTick + u32CurTick));
}








