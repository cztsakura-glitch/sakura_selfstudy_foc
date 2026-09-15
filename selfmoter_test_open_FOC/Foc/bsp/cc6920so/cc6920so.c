#include "cc6920so.h"

#include "main.h"
#include "adc.h"
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>



/* DMA方式读取ADC */
#ifdef ADC_DMA

#define ADC_BUF_SIZE                        10

uint32_t uvwAdVal[3];                        /*UVW三相的ad值*/

static uint32_t UdmaAdBuf[ADC_BUF_SIZE];    /*UVW三相的dma缓冲区*/
static uint32_t VdmaAdBuf[ADC_BUF_SIZE];
static uint32_t WdmaAdBuf[ADC_BUF_SIZE];
float Iabc[3] = {0};

void CC6920SO_Init(void)
{
    /*DMA方式*/
    if(HAL_OK != HAL_ADC_Start_DMA(&hadc1, WdmaAdBuf, ADC_BUF_SIZE)){
        printf("HAL_ADC_Start_DMA: NO\r\n");
        while(1);
    }
    if(HAL_OK != HAL_ADC_Start_DMA(&hadc2, VdmaAdBuf, ADC_BUF_SIZE)){
        printf("HAL_ADC_Start_DMA: NO\r\n");
        while(1);
    }
    if(HAL_OK != HAL_ADC_Start_DMA(&hadc3, UdmaAdBuf, ADC_BUF_SIZE)){
        printf("HAL_ADC_Start_DMA: NO\r\n");
        while(1);
    }
}

/*计算平均AD值 ch分别为UVW通道*/
static void CC6920SO_CalcAvr(uint8_t ch){
    uint32_t *adc_buf = NULL;

    uint32_t adc_sum = 0;
    uint32_t i = 0;
    switch(ch){
        case 0:
            adc_buf = UdmaAdBuf;
            break;
        case 1:
            adc_buf = VdmaAdBuf;
            break;
        case 2:
            adc_buf = WdmaAdBuf;
            break;
        default:
            return;
    }

    for(i=0; i<ADC_BUF_SIZE; i++){
        adc_sum += adc_buf[i];
    }

    uvwAdVal[ch] = adc_sum/ADC_BUF_SIZE;
}

/*DMA方式采用 规则通道完成 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
   
    uint8_t ch = 5;
    if(hadc==&hadc1){                   /*W相*/      
        ch = 2;
    }else if(hadc==&hadc2){             /*V相*/
        ch = 1;
    }else if(hadc==&hadc3){             /*U相*/
        ch = 0;
    }
    if(ch<3){
        CC6920SO_CalcAvr(ch);           /*计算平均值*/
        Iabc[ch] = CC6920SO_CalcCur(ch, uvwAdVal[ch]);
    }
}

#else

void CC6920SO_Init(void)
{
    /* 注入通道方式：仅开启ADC1 */
    if(HAL_OK != HAL_ADCEx_InjectedStart_IT(&hadc1)){
        printf("HAL_ADCEx_InjectedStart_IT: NO\r\n");
    }
}

#endif



#if 1
/* 野火驱动板电流换算：
    硬件：0.02R 采样电阻 + AMC1200隔离运放(Gain=8)
    公式：Delta_V = I * 0.02 * 8 = I * 0.16
*/
float CC6920SO_CalcCur(uint8_t ch, float adVal)
{
    if(ch>2) return 0.0f;

    /* 注意！传进来的 adVal 是减去偏置后的差值，可能为负数，必须先转成有符号的 int32_t */
    float Delta_V = adVal / 4096.0f * 3.3f;

    /* 电流 I = 电压差 / 0.16 */
    float I = Delta_V / 0.16f;
    
    /* 台架验证结果：按本工程Clarke/Park变换的符号约定，
     * 正向相电压指令应得到正向电流。 */
    return I;
}
#endif

//#if 1
///* 野火驱动板电流换算：
//    硬件：0.02R 采样电阻 + AMC1200隔离运放(Gain=8)
//    公式：Delta_V = I * 0.02 * 8 = I * 0.16
//*/
//float CC6920SO_CalcCur(uint8_t ch, float adVal)
//{
//    if(ch>2) return 0.0f;

//    /* 注意！传进来的 adVal 是减去偏置后的差值，可能为负数，必须先转成有符号的 int32_t */
//    float Delta_V = adVal / 4096.0f * 3.3f;

//    /* 电流 I = 电压差 / 0.16 */
//    float I = Delta_V / 0.16f;
//    
//    return I;
//}
//#endif

//#if 0
///* 20A电流换算 通道顺序UVW

//    VOUT = VCC/ 2 +0.100 × IP(A)
//*/
//float CC6920SO_CalcCur(uint8_t ch, float adVal)
//{
//    if(ch>2)return 0;

//    float Uout;
//    float Uadc;     
//    float I;

//    /*ADC采集到的电压*/
//    Uadc = (float)adVal/4096.0f *3.3f;

//    /*霍尔元件输出的电压*/
//    Uout = Uadc *3.0f /2.0f;    

//    I = (Uout-2.5f)/0.1f;       /*电流计算  Uout = I*0.1*+0.25 */
//    
//    return I;                   /* 这里要注意电流的方向，因为后面的克拉克计算电流是按照从坐标轴原点流向外面，流出电机为正方向的 */
//}
//#endif

//#if 0
///* 40A电流换算 通道顺序UVW

//    VOUT = VCC/ 2 +0.05 × IP(A)
//*/
//float CC6920SO_CalcCur(uint8_t ch, float adVal)
//{
//    if(ch>2)return 0;

//    float Uout;
//    float Uadc;     
//    float I;

//    /*ADC采集到的电压*/
//    Uadc = (float)adVal/4096.0f *3.3f;

//    /*霍尔元件输出的电压*/
//    Uout = Uadc *3.0f /2.0f;    

//    I = (Uout-2.5f)/0.05f;       /*电流计算  Uout = I*0.05*+0.25 */
//    
//    return I;                   /* 这里要注意电流的方向，因为后面的克拉克计算电流是按照从坐标轴原点流向外面，流出电机为正方向的 */
//}
//#endif

//#if 1
///* 野火驱动板电流换算：
//    硬件：0.02R 采样电阻 + AMC1200隔离运放(Gain=8)
//    公式：Delta_V = I * 0.02 * 8 = I * 0.16
//*/
//float CC6920SO_CalcCur(uint8_t ch, float adVal)
//{
//    if(ch>2) return 0.0f;

//    /* 注意！传进来的 adVal 是减去偏置后的差值，可能为负数，必须先转成有符号的 int32_t */
//    float Delta_V = adVal / 4096.0f * 3.3f;

//    /* 电流 I = 电压差 / 0.16 */
//    float I = Delta_V / 0.16f;
//    
//    /* * 极性说明：
//     * FOC算法要求测到的电流是“流入电机为正（或流出为正）”。
//     * 如果后期你发现电机一上电就啸叫、发抖，说明电流极性反了，
//     * 只需要把下面这行改为 return -I; 即可。
//     */
//    return I;
//}
//#endif


