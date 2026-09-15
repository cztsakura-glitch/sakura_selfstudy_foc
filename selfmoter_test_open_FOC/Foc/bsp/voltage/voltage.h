#ifndef _VOLTAGE_H
#define _VOLTAGE_H

/* 母线电压采样接口：初始化 ADC、读取换算电压以及输出调试信息。 */
void Voltage_Init(void);
/* 返回最近一次换算得到的母线电压，单位 V。 */
float get_voltage(void);
void print_adc(void);

/* 供调试和保护逻辑读取的母线电压缓存，单位 V。 */
extern float vbus;










#endif
