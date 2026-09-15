#include "voltage.h"
#include "main.h"
#include "adc.h"

#include <stdio.h>


extern DMA_HandleTypeDef hdma_adc1;

#define ADC_BUF_SIZE                1*1000
static uint32_t adc_dma_buf[ADC_BUF_SIZE] ={0};

/*母线电压*/
float vbus = 0;


/*电压采集初始化*/
void Voltage_Init(void)
{
    if(HAL_OK != HAL_ADC_Start_DMA(&hadc1, adc_dma_buf, ADC_BUF_SIZE)){
        printf("HAL_ADC_Start_DMA: NO\r\n");
        while(1);
    }

    if(HAL_OK != HAL_ADCEx_InjectedStart_IT(&hadc1)){
        printf("HAL_ADC_Start_DMA: NO\r\n");
        while(1);
    }
//    if(HAL_OK != HAL_ADCEx_InjectedStart_IT(&hadc2)){
//        printf("HAL_ADC_Start_DMA: NO\r\n");
//        while(1);
//    }
//    if(HAL_OK != HAL_ADCEx_InjectedStart_IT(&hadc3)){
//        printf("HAL_ADC_Start_DMA: NO\r\n");
//        while(1);
//    }





}

//static void adc_calculate_average(void)
//{
//    int32_t _adc1_vbus = 0;
//    int32_t i;

//    float v     = 0.0f;
//    for(i=0; i<ADC_BUF_SIZE; i++){
//        _adc1_vbus += adc_dma_buf[i];
//    }


//    _adc1_vbus = _adc1_vbus/i;

//    v = ((float)_adc1_vbus/4096.0f)*3.3f;
//    vbus = v*25.0f;
//}

/*ADC DMA传输回调函数*/
//void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
//{
//    if(hadc==&hadc1){
//        //printf("规则通道完成\r\n");
//        adc_calculate_average();
//    }
//}




void print_adc(void)
{
   // printf("U: %dA  V: %dA  W: %dA\r\n", adc1_u, adc1_v, adc1_w);
    //printf("U: %.4fA  V: %.4fA  W: %.4fA\r\n", u_cur, v_cur, w_cur);
    printf("v_bus: %.4fV\r\n\r\n", vbus);

    //printf("adc1_in17: %.4f    ", ((float)adc1_u/65536.0f)*3.3f);
    //printf("adc1_in19: %.4f\r\n", ((float)adc1_in19/65536.0f)*3.3f);


    
}


