// /**
//  ****************************************************************************************************
//  * @file        foc_smo.c
//  * @author      哔哩哔哩-Rebron大侠
//  * @version     V0.0
//  * @date        2025-01-11
//  * @brief       参数识别
//  * @license     MIT License
//  *              Copyright (c) 2025 Reborn大侠
//  *              允许任何人使用、复制、修改和分发该代码，但需保留此版权声明。
//  ****************************************************************************************************
//  */

// #include "foc_identify.h"

// #include "foc_hw.h"
// #include "foc_math.h"

// #include <stdio.h>
// #include <delay.h>


// /**
//  * @brief 参数辨识相关初始化
//  * 
//  * @param iden 
//  * @param Tpwm PWM周期
//  * @param Vbus 母线电压
//  * @param Ts 运行周期
//  */
// void FOC_Identify_Init(FOC_IDENTIFY_t *iden, uint32_t Tpwm, float Vbus, float Ts)
// {
//     iden->Tpwm = Tpwm;
//     iden->Vbus = Vbus;
//     iden->Ts = Ts;

//     FOC_HFI_Init(&iden->hfi, 0.0f, 0, 20000.0f, FOC_HFI_FREQ_10K, 7);

//     Move_Filter_Init(&iden->I_dt_filter, 500);

//     Move_Filter_Init(&iden->filter, 500);   
// }

// /**
//  * @brief 直流电压注入法 辨识电阻
//  * 
//  * @param iden 
//  * @param U 
//  * @param I 
//  */
// void FOC_DC_V_Identify_R(FOC_IDENTIFY_t *iden, float U, abc_t I)
// {
//     float Is = 0.0f;
//     // U += 0.0135f;      /*死区补偿电压*/

//     /*A 相上管， B相下管  C相下管*/
//     FOC_Port_Pwm_Start(true,true,true,true,true,true);

//     U_H_SET_PWM((U)/iden->Vbus * iden->Tpwm);
//     V_H_SET_PWM( 0/iden->Vbus * iden->Tpwm);
//     W_H_SET_PWM( 0/iden->Vbus * iden->Tpwm);


//     Move_Filter_fill(&iden->filter, I.a*1000.0f);
//     Is = Move_Filter_calculate(&iden->filter) /1000.0f;

//     iden->Rs = U/Is/1.5f;
// }






// /*电阻参数识别 */
// #if 0
// bool FOC_Identify_R(FOC_IDENTIFY_t *iden, float U, abc_t I)
// {
//     /*U上V下识别*/
//     float temp = 0.0f;

//     switch(iden->Iden_R_State){
//         case U_V_R_SET:
//             FOC_Port_Pwm_Start(true, true, true, true, false, false);
//             U += 0.0135f;      /*死区补偿电压*/
//             U_H_SET_PWM((U+temp)/iden->Vbus * iden->Tpwm);
//             V_H_SET_PWM(temp /iden->Vbus * iden->Tpwm);

            

//             iden->I_sum = 0;
//             iden->Iden_R_Count = 0;
//             iden->Line_R = 0.0f;

//             iden->Iden_R_State = U_V_R_GET;

//             delay_ms(100);
//             break;
//         case U_V_R_GET:
//             iden->I_sum += I.a;
//             iden->Iden_R_Count++;
//             if(iden->Iden_R_Count>=CUR_IDEN_COUNT){
//                 iden->I_sum = iden->I_sum/(float)iden->Iden_R_Count;
//                 iden->Line_R +=  fabsf(U / iden->I_sum);
//                 iden->Iden_R_State = V_W_R_SET;

//                 printf("电阻1: %f\r\n", fabsf(U / iden->I_sum));
//             }
//             break;

//         case V_W_R_SET:
//             FOC_Port_Pwm_Start(false, false, true, true, true, true);
//             U += 0.0135f;      /*死区补偿电压*/
//             V_H_SET_PWM((U+temp)/iden->Vbus * iden->Tpwm);
//             W_H_SET_PWM(temp /iden->Vbus * iden->Tpwm);

//             iden->I_sum = 0;
//             iden->Iden_R_Count = 0;
//             iden->Iden_R_State = V_W_R_GET;

//             delay_ms(100);
//             break;
//         case V_W_R_GET:
//             iden->I_sum += I.b;
//             iden->Iden_R_Count++;
//             if(iden->Iden_R_Count>=CUR_IDEN_COUNT){
//                 iden->I_sum = iden->I_sum/(float)iden->Iden_R_Count;
//                 iden->Line_R +=  fabsf(U / iden->I_sum);
//                 iden->Iden_R_State = W_U_R_SET;
//                 printf("电阻2: %f\r\n", fabsf(U / iden->I_sum));
//             }
//             break;
//         case W_U_R_SET:
//             FOC_Port_Pwm_Start(true, true, false, false, true, true);
//             U += 0.0135f;      /*死区补偿电压*/
//             W_H_SET_PWM((U+temp)/iden->Vbus * iden->Tpwm);
//             U_H_SET_PWM(temp/iden->Vbus * iden->Tpwm);

//             iden->I_sum = 0;
//             iden->Iden_R_Count = 0;
//             iden->Iden_R_State = W_U_R_GET;

//             delay_ms(100);
//             break;
//         case W_U_R_GET:
//             iden->I_sum += I.c;
//             iden->Iden_R_Count++;
//             if(iden->Iden_R_Count>=CUR_IDEN_COUNT){
//                 iden->I_sum = iden->I_sum/(float)iden->Iden_R_Count;
//                 iden->Line_R +=  fabsf(U / iden->I_sum);
//                 iden->Iden_R_State = U_V_R_SET;
//                 printf("电阻3: %f\r\n", fabsf(U / iden->I_sum));

//                 goto RET;
//             }
//             break;
//     }

    
    
//     return false;

// RET:
//     /*辨识完成*/
//     U_H_SET_PWM(0);
//     V_H_SET_PWM(0);
//     W_H_SET_PWM(0);
//     FOC_Port_Pwm_Stop();

//     printf("***********************************电阻辨识完成 :%f\r\n", iden->Line_R/3.0f);
//     return true;
// }

// #else

// /*三角波生成频率*/

// float generateTriangleWave(float min, float max, float current, float freq)
// {
//     float temp = 0.001f;



//     return current+temp;
// }


// bool FOC_Identify_R(FOC_IDENTIFY_t *iden, float U, abc_t I)
// {
//     /*U上V下识别*/
//     switch(iden->Iden_R_State){
//         case U_V_R_SET:
//             FOC_Port_Pwm_Start(true, true, true, true, true, true);
//             U += 0.0135f;      /*死区补偿电压*/
//             U_H_SET_PWM((U)/iden->Vbus * iden->Tpwm);
//             V_H_SET_PWM(0);
//             W_H_SET_PWM(0);

//             iden->I_sum = 0;
//             iden->Iden_R_Count = 0;
//             iden->Line_R = 0.0f;

//             iden->Iden_R_State = U_V_R_GET;

//             delay_ms(1000);
//             break;
//         case U_V_R_GET:
//             iden->I_sum += I.a;
//             iden->Iden_R_Count++;
//             if(iden->Iden_R_Count>=CUR_IDEN_COUNT){
//                 iden->I_sum = iden->I_sum/(float)iden->Iden_R_Count;
//                 iden->Line_R = fabsf(U / iden->I_sum) /2.0f *3.0f ;
//                 iden->Iden_R_State = V_W_R_SET;
//                 goto RET;
//             }
//             // /*存入滑动滤波器缓冲区*/
//             // Move_Filter_fill(&iden->I_dt_filter, I.a*1000.0f);
//             // /*计算扇区速度*/
//             // iden->I_dt_sum = Move_Filter_calculate(&iden->I_dt_filter)/1000.0f;  
//             // iden->Line_R = fabsf(U / iden->I_dt_sum) /2.0f *3.0f;

//             break;


//     }

//     return false;

// RET:
//     /*辨识完成*/
//     U_H_SET_PWM(0);
//     V_H_SET_PWM(0);
//     W_H_SET_PWM(0);
//     FOC_Port_Pwm_Stop();

//     printf("***********************************电阻辨识完成 :%f\r\n", iden->Line_R);
//     return true;
// }

// /* 直接三相坐标辨识电感 */
// bool FOC_Identify_L(FOC_IDENTIFY_t *iden, float U, abc_t Iabc)
// {
//     float I_err = 0.0f;
//     float I = 0;


//     switch(iden->Iden_L_State){
//         case U_V_L_SET:
//             FOC_Port_Pwm_Start(true, true, true, true, false, false);

//             iden->Iden_L_State = U_V_L_GET;

//             iden->Uqd.d = 0.0f;
//             // iden->hfi.hfi_vol_ref = U + 0.0135f;

//             break;
//         case U_V_L_GET:

//             /*更新电流*/
//             I = Iabc.a;

                
//             I_err = fabsf(I - iden->I_last);
//             iden->I_last = I;

//             iden->I_dt = I_err / iden->Ts /1000000.0f;                              /*计算电流微分*/

//             iden->I_dt_err = (iden->I_dt - iden->I_dt_last);
//             iden->I_dt_last = iden->I_dt;

//             iden->I_dt_sum += iden->I_dt;                                           /*累计微分进行平均*/
//             iden->Iden_R_Count++;
//             if(iden->Iden_R_Count>=CUR_IDEN_COUNT){
//                 iden->Line_L = iden->hfi.hfi_vol_ref / (iden->I_dt_sum /(float)iden->Iden_R_Count);
//                 iden->I_dt_sum = 0;
//                 iden->Iden_R_Count = 0;
//                 iden->Iden_R_State = V_W_R_SET;
//                 goto RET;
//             }
//             FOC_HFI_Inject(&iden->hfi);                                             /*高频注入*/ 
            
//             U_H_SET_PWM((iden->hfi.hfi_vol + iden->Vbus/2)/iden->Vbus * iden->Tpwm);
//             V_H_SET_PWM((iden->Vbus/2.0f) / iden->Vbus * iden->Tpwm);
//             W_H_SET_PWM((iden->Vbus/2.0f) / iden->Vbus * iden->Tpwm);
            
//             // Out = iden->Uqd;
//             // Out.d += iden->hfi.hfi_vol;                                        
//             // FOC_PWM_Run(foc_pwm, Out, ElAngle);

//             // Debug_Task();
//             break;
//         case V_W_L_SET:
//             break;
//     }

//     return false;
// RET:
//     /*辨识完成*/
//     U_H_SET_PWM(0);
//     V_H_SET_PWM(0);
//     W_H_SET_PWM(0);
//     FOC_Port_Pwm_Stop();

//     printf("线电感辨识完成 :%f\r\n", iden->Line_L);
// 	return true;

// }


// #endif

// /**
//  * @brief 离线辨识 用QD轴电流来辨识电阻
//  * 
//  * @param iden 
//  * @param U 电压
//  * @param I 电流
//  * @param foc_pwm pwm结构体
//  * @return true 辨识完成
//  * @return false 辨识未完成
//  */
// bool FOC_Identify_R_QD(FOC_IDENTIFY_t *iden, float U, abc_t I, FOC_PWM_t *foc_pwm)
// {
//     float Is = 0;
//     qd_t In = {0};
//     qd_t Out = {0};
//     abc_t Iabc = {0};

//     alphabeta_t I_alphabeta = {0};

//     switch(iden->Iden_L_State){
//         case U_V_L_SET:
//             FOC_Port_Pwm_Start(true, true, true, true, true, true);

//             Out.d = U;
//             FOC_PWM_Run(foc_pwm, Out, 1);
//             iden->Iden_L_State = U_V_L_GET;

//             delay_ms(3000);
//             break;
//         case U_V_L_GET:
//             /*更新电流*/
//             Iabc = I;
//             /*克拉克变换*/
//             I_alphabeta  = FOC_Clarke(Iabc);

//             Is = I_alphabeta.alpha*I_alphabeta.alpha + I_alphabeta.beta*I_alphabeta.beta;

//             arm_sqrt_f32(Is, &Is);

//             /*帕克变换*/
//             In = Foc_Park(I_alphabeta, 1);

//             iden->I_sum += In.d;

//             iden->Iden_R_Count++;
//             if(iden->Iden_R_Count>=CUR_IDEN_COUNT){
//                 iden->I_sum = iden->I_sum/(float)iden->Iden_R_Count;
//                 iden->Line_R = fabsf(U / iden->I_sum);
//                 iden->Iden_R_State = V_W_R_SET;
//                 goto RET;
//             }
            
            
//             break;
//     }

//     return false;
// RET:
//     /*辨识完成*/
//     U_H_SET_PWM(0);
//     V_H_SET_PWM(0);
//     W_H_SET_PWM(0);
//     FOC_Port_Pwm_Stop();

//     printf("相电阻辨识完成 :%f\r\n", iden->Line_R);
//     return true;
// }



// /*电感参数识别  用QD轴高频电流来辨识电阻*/
// #if 0

// /*使用阶跃信号来辨识*/
// bool FOC_Identify_L_QD(FOC_IDENTIFY_t *iden, float U, abc_t I, FOC_PWM_t *foc_pwm)
// {   
//     float I_err = 0.0f;
//     qd_t In = {0};
//     qd_t Out = {0};
//     abc_t Iabc = {0};
//     float ElAngle = 0.0f;
//     alphabeta_t I_alphabeta = {0};

//     switch(iden->Iden_L_State){
//         case U_V_L_SET:
//             FOC_Port_Pwm_Start(true, true, true, true, true, true);

//             iden->Iden_L_State = U_V_L_GET;

//             iden->Uqd.d = U;
//             iden->hfi.hfi_vol_ref = U;

//             break;
//         case U_V_L_GET:
//             /*获取电流微分*/
//             /*进行高频注入*/

//             /*更新电流*/
//             Iabc = I;
//             /*克拉克变换*/
//             I_alphabeta  = FOC_Clarke(Iabc);
//             /*帕克变换*/
//             In = Foc_Park(I_alphabeta, ElAngle);


//             iden->In = In;

//             if(iden->Uqd.d>0){
//                 iden->I_simple[iden->Iden_L_Count++] = In.d;
//                 if(iden->Iden_L_Count>998)iden->Iden_L_Count = 998;
//             }else{
//                 iden->Iden_L_Count = 0;
//             }

            

            

//             I_err = fabsf(In.d - iden->I_last);                                 /*电流差*/
//             iden->I_last = In.d;

//             iden->Ts = (float)__HAL_TIM_GET_COUNTER(&htim2) /1000000.0f;
//             __HAL_TIM_SET_COUNTER(&htim2, 0);       /*校准电角度*/
//             iden->I_dt = I_err /iden->Ts;
            
//             /*存入滑动滤波器缓冲区*/
//             Move_Filter_fill(&iden->I_dt_filter, I_err*1000000.0f);
//             /*计算扇区速度*/
//             iden->I_dt = Move_Filter_calculate(&iden->I_dt_filter)/1000000.0f /iden->Ts;               /* 电流微分 */
            
//             iden->Line_L = (iden->hfi.hfi_vol_ref)/iden->I_dt *1000000.0f;
            
//             Out = iden->Uqd;
//             FOC_PWM_Run(foc_pwm, Out, ElAngle);
//             break;
//         case V_W_L_SET:

//             break;
//     }

//     return false;
// RET:
//     /*辨识完成*/
//     U_H_SET_PWM(0);
//     V_H_SET_PWM(0);
//     W_H_SET_PWM(0);
//     FOC_Port_Pwm_Stop();

//     printf("线电感辨识完成 :%f\r\n", iden->Line_L);
// 	return true;
// }

// #else

// bool FOC_Identify_L_QD(FOC_IDENTIFY_t *iden, float U, abc_t I, FOC_PWM_t *foc_pwm)
// {   
//     float I_err = 0.0f;
//     qd_t In = {0};
//     qd_t Out = {0};
//     abc_t Iabc = {0};
//     float ElAngle = 0.0f;
//     alphabeta_t I_alphabeta = {0};

//     switch(iden->Iden_L_State){
//         case U_V_L_SET:
//             FOC_Port_Pwm_Start(true, true, true, true, true, true);

//             iden->Iden_L_State = U_V_L_GET;

//            iden->Uqd.d = 0.0f;
//             iden->hfi.hfi_vol_ref = U;

//             break;
//         case U_V_L_GET:

//             /*更新电流*/
//             Iabc = I;
//             /*克拉克变换*/
//             I_alphabeta  = FOC_Clarke(Iabc);

//             FOC_HFI_Angle_Calc(&iden->hfi, I_alphabeta);                            /*提取高频分量*/
//             I_alphabeta = iden->hfi.alphabeta_h;                              
    
//             /*帕克变换*/
//             In = Foc_Park(I_alphabeta, ElAngle);                                    /*提取高频QD轴电流*/
//             iden->In = In;

//             if(iden->hfi.hfi_vol_ref>0){
//                 iden->I_simple[iden->Iden_L_Count++] = In.d;
//                 if(iden->Iden_L_Count>998)iden->Iden_L_Count = 998;
//             }else{
//                 iden->Iden_L_Count = 0;
//             }


//             I_err = fabsf(In.d - iden->I_last);                                     /*电流差*/
//             iden->I_last = In.d;

//             iden->Ts = (float)__HAL_TIM_GET_COUNTER(&htim2) /1000000.0f;
//             __HAL_TIM_SET_COUNTER(&htim2, 0);       /*校准电角度*/

//             iden->I_dt = I_err / iden->Ts /1000000.0f;                              /*计算电流微分*/

//             iden->I_dt_err = (iden->I_dt - iden->I_dt_last);
//             iden->I_dt_last = iden->I_dt;

//             iden->I_dt_sum += iden->I_dt;     /*计算单次电感*/
//             iden->Iden_R_Count++;
//             if(iden->Iden_R_Count>=CUR_IDEN_COUNT){
//                 iden->Line_L = iden->hfi.hfi_vol_ref / (iden->I_dt_sum /(float)iden->Iden_R_Count);
//                 iden->I_dt_sum = 0;
//                 iden->Iden_R_Count = 0;
//                  iden->Iden_R_State = V_W_R_SET;
//                  goto RET;
//             }
//             FOC_HFI_Inject(&iden->hfi);                                             /*高频注入*/   
//             Out = iden->Uqd;
//             Out.d += iden->hfi.hfi_vol;                                        
//             FOC_PWM_Run(foc_pwm, Out, ElAngle);

//             // Debug_Task();
            
//             break;
//         case V_W_L_SET:
//             break;
//     }

//     return false;
// RET:
//     /*辨识完成*/
//     U_H_SET_PWM(0);
//     V_H_SET_PWM(0);
//     W_H_SET_PWM(0);
//     FOC_Port_Pwm_Stop();

//     printf("线电感辨识完成 :%f\r\n", iden->Line_L);
// 	return true;
// }


// #endif

// /**
//  * @brief qd轴阶跃响应法辨识qd轴电感
//  * 
//  * @param iden 
//  * @param U 
//  * @param I 
//  * @param foc_pwm 
//  */
// bool FOC_Identify_ByStep_L_QD(FOC_IDENTIFY_t *iden, float U, abc_t I, FOC_PWM_t *foc_pwm)
// {
//     qd_t I_qd = {0};
//     alphabeta_t I_alphabeta = {0};

//     qd_t Uqd = {0};

//     Uqd.q = U;

//     FOC_Port_Pwm_Start(true,true,true,true,true,true);
  
//     FOC_PWM_Run(foc_pwm, Uqd, 0);

//     iden->times += iden->Ts;
//     if(iden->times>0.0001f && iden->Lqd_cnt.d<10){
//         iden->cnt++;
//         if(iden->cnt==1){

//             I_alphabeta = FOC_Clarke(I);
//             I_qd = Foc_Park(I_alphabeta, 0);

//             iden->Iqd1.d = I_qd.d;
//         }

//         if(iden->cnt==41){
//             iden->cnt = 0;
//             iden->Lqd_cnt.d += 1;
//             I_alphabeta = FOC_Clarke(I);
//             I_qd = Foc_Park(I_alphabeta, 0);
//             iden->Iqd2.d = I_qd.d;

//             iden->Iqdsum.d += 0.002f*(U-0.07024f*(iden->Iqd1.d+iden->Iqd2.d)*0.5f)/(iden->Iqd2.d-iden->Iqd1.d);
//         }
//     }

//     if(iden->Lqd_cnt.d>=10){
//         iden->Lqd.d = iden->Iqdsum.d/10.0f;

//         printf("d轴电感: %f H\r\n", iden->Lqd.d);
//         FOC_Port_Pwm_Stop();
//         return true;
//     }


//     return false;
// }



