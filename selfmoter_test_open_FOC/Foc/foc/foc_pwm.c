/**
 ****************************************************************************************************
 * @file        foc_pwm.c
 * @author      哔哩哔哩-Rebron大侠
 * @version     V0.0
 * @date        2025-01-11
 * @brief       SVPWM实现
 * @license     MIT License
 *              Copyright (c) 2025 Reborn大侠
 *              允许任何人使用、复制、修改和分发该代码，但需保留此版权声明。
 ****************************************************************************************************
 */


#include "foc_pwm.h"
#include "foc_hw.h"
#include "foc_math.h"
#include "stdlib.h"

static volatile uint32_t g_foc_pwm_debug_spwm = 0U;

void FOC_PWM_DebugSetSpwm(uint32_t use_spwm)
{
    g_foc_pwm_debug_spwm = (use_spwm != 0U) ? 1U : 0U;
}

uint32_t FOC_PWM_DebugGetSpwm(void)
{
    return g_foc_pwm_debug_spwm;
}

/*
扇区判断条件
扇区Ⅰ: Uα>0, Uβ>0, Uβ/Uα <√3,                                                       最终结果  √3Uα-Uβ>0
扇区Ⅱ: Uα?, Uβ>0,  Uα>0时 Uβ/Uα>√3, √3Uα-Uβ<0; Uα<0时, Uβ/Uα<-√3, -√3Uα-Uβ<0;       最终结果 Uα>0,√3Uα-Uβ<0  Uα<0,-√3Uα-Uβ<0
扇区Ⅲ: Uα<0,Uβ>0, Uβ/Uα<-√3,                                                        最终结果    -√3Uα-Uβ >0
扇区Ⅳ: Uα<0,Uβ<0, Uβ/Uα <√3,                                                        最终结果   √3Uα-Uβ<0
扇区Ⅴ: Uα?, Uβ<0, Uα<0时 Uβ/Uα>√3, √3Uα-Uβ>0; Uα>0时, Uβ/Uα<-√3,-√3Uα-Uβ>0;          最终结果 Uα>0,-√3Uα-Uβ>0, Uα<0,√3Uα-Uβ>0;    
扇区Ⅵ: Ua>0,Uβ<0, Uβ/Uα>-√3,                                                        最终结果  -√3Uα-Uβ<0;

U1 = Uβ;        U1>0 A=1; U1<0 A=0;
U2 = √3Uα-Uβ;   U2>0 B=1; U2<0 B=0;
U3 = -√3Uα-Uβ;  U3>0 C=1; U3<0 C=0;

扇区Ⅰ: A=1; B=1; C=0;   N=3
扇区Ⅱ: A=1; B=0; C=0;   N=1
扇区Ⅲ: A=1; B=0; C=1;   N=5
扇区Ⅳ: A=0; B=0; C=1;   N=4                        
扇区Ⅴ: A=0; B=1; C=1;   N=6
扇区Ⅵ: A=0; B=1; C=0;   N=2




扇区Ⅰ: Uα>0, Uβ>0, Uβ/Uα <√3;   判断结果 √3Uα-Uβ>0;     
扇区Ⅱ: Uα? , Uβ>0
        Uα>0时, Uβ/Uα>√3;        判断结果 √3Uα-Uβ<0;
        Uα<0时, Uβ/Uα<-√3;       判断结果 -√3Uα-Uβ<0;
扇区Ⅲ: Uα<0, Uβ>0, Uβ/Uα>-√3;   判断结果 -√3Uα-Uβ>0; 
扇区Ⅳ: Uα<0, Uβ<0, Uβ/Uα<√3;    判断结果 √3Uα-Uβ<0
扇区Ⅴ: Uα? , Uβ<0,
        Uα>0时 Uβ/Uα<-√3;        判断结果 -√3Uα-Uβ>0
        Uα<0时 Uβ/Uα>√3;         判断结果 √3Uα-Uβ>0
扇区Ⅵ  Ua>0,Uβ<0, Uβ/Uα>-√3;    判断结果 -√3Uα-Uβ<0

A = Uβ;           
B = √3Uα -Uβ;
C = -√3Uα -Uβ;
大于0的时候=1 小于0的时候=0;

扇区Ⅰ: A=1; B=1; C=0;       N=3
扇区Ⅱ: A=1; B=0; C=0;       N=1
扇区Ⅲ: A=1; B=0; C=1;       N=5
扇区Ⅳ: A=0; B=0; C=1;       N=4
扇区Ⅴ: A=0; B=1; C=1;       N=6
扇区Ⅵ: A=0; B=1; C=0;       N=2
N = A+2B+4C  
*/

/**
 * @brief 扇区判断
 * 
 * @param foc_pwm 
 * @param alphabeta αβ电压
 * @return uint8_t 返回扇区号
 */
uint8_t SectorJudgment(FOC_PWM_t *foc_pwm, alphabeta_t alphabeta)
{
    float A = alphabeta.beta;
    float B = SQRT_3*alphabeta.alpha - alphabeta.beta;
    float C = -SQRT_3*alphabeta.alpha - alphabeta.beta;

    uint8_t N=0;
    uint8_t sector = 0;

    if(A>0)N +=1;
    if(B>0)N +=2;
    if(C>0)N +=4;

    switch(N){
        case 3:
            sector = 1;
            break;
        case 1:
            sector = 2;
            break;
        case 5:
            sector = 3;
            break;
        case 4:
            sector = 4;
            break;
        case 6:
            sector = 5;
            break;
        case 2:
            sector = 6;
            break;
    }
    return sector;
}

/*计算矢量时间

U4 = Udc*(T4/Ts)
U6 = Udc*(T6/Ts)

U4 = Udc*2/3
U6 = Udc*2/3

以扇区为例
扇区Ⅰ:
    Uα = U4 + U6*cos60°;
    Uβ = U6 * sin60°

    U6 = Uβ/sin60°
    T6/Ts *(2/3*Udc) = Uβ *2/√3;
    T6 = (√3*Ts)/Udc *Uβ;

    Uα = U4 + Uβ*cos60°/sin60°
    U4 = Uα -Uβ/√3;
    T4/Ts *(2/3*Udc) = Uα -Uβ/√3;
    T4 = (√3*Ts)/(2*Udc)*(√3Uα-Uβ);

    最终结果
    T6 = (√3*Ts)/Udc *Uβ;
    T4 = (√3*Ts)/Udc*(√3Uα/2 -Uβ/2);

扇区Ⅱ:
    Uα = U6*cos60° - U2*cos60°
    Uβ = U6*cos30° + U2*cos30°

    推导最终结果
    T6 = (√3*Ts)/Udc*(√3Uα/2+Uβ/2);
    T2 = - (√3*Ts)/Udc*(√3Uα/2-Uβ/2);

扇区Ⅲ:
    Uα = -U2*cos60° -U3
    Uβ = U2*sin60

    T2 = (√3*Ts)/Udc *Uβ;
    T3 = -(√3*Ts)/Udc*(√3Uα/2+Uβ/2);
后面的就不推导了，都是体力活了。


->Uqd+电角度->Uαβ->计算扇区和相邻矢量的时间。

*/


/*计算矢量时间

U4 = Udc*(T4/Ts)
U6 = Udc*(T6/Ts)

U4 = Udc*2/3
U6 = Udc*2/3

以扇区为例
扇区Ⅰ:
    Uα = U4 + U6*cos60°;
    Uβ = U6 * sin60°

    U6 = Uβ/sin60°
    T6/Ts *(2/3*Udc) = Uβ *2/√3;
    T6 = (√3*Ts)/Udc *Uβ;

    Uα = U4 + Uβ*cos60°/sin60°
    U4 = Uα -Uβ/√3;
    T4/Ts *(2/3*Udc) = Uα -Uβ/√3;
    T4 = (√3*Ts)/(2*Udc)*(√3Uα-Uβ);

    最终结果
    T6 = (√3*Ts)/Udc *Uβ;
    T4 = (√3*Ts)/Udc*(√3Uα/2 -Uβ/2);

扇区Ⅱ:
    Uα = U6*cos60° - U2*cos60°
    Uβ = U6*cos30° + U2*cos30°

    推导最终结果
    T6 = (√3*Ts)/Udc*(√3Uα/2+Uβ/2);
    T2 = - (√3*Ts)/Udc*(√3Uα/2-Uβ/2);

扇区Ⅲ:
    Uα = -U2*cos60° -U3
    Uβ = U2*sin60

    T2 = (√3*Ts)/Udc *Uβ;
    T3 = -(√3*Ts)/Udc*(√3Uα/2+Uβ/2);
后面的就不推导了，都是体力活了。


->Uqd+电角度->Uαβ->计算扇区和相邻矢量的时间。

*/

/*
计算矢量时间

U4 = Udc*(T4/Ts)
U6 = Udc*(T6/Ts)


以扇区为例
扇区Ⅰ:
    Uα = U4 + U6*cos60°;
    Uβ = U6 * sin60°
*/

/**
 * @brief 扇区判断
 * 
 * @param foc_pwm 
 * @param alphabeta 
 * @param sector 
 * @return uint8_t 
 */
static uint8_t Sector_Judgment(FOC_PWM_t *foc_pwm, alphabeta_t alphabeta, uint8_t *sector)
{
    foc_pwm->svpwm_val1 = alphabeta.alpha *SQRT_3_DIV_2;
    foc_pwm->svpwm_val2 = alphabeta.beta/2.0f;

    float A = alphabeta.beta;
    float B = foc_pwm->svpwm_val1 - foc_pwm->svpwm_val2;
    float C = -foc_pwm->svpwm_val1 - foc_pwm->svpwm_val2;

    uint8_t N = 0;
    if(A>0){
        N += 1;
    }

    if(B>0){
        N += 2;
    }

    if(C>0){
        N += 4;
    }
#if 1    /*进行转换 将N 转换成 Ⅰ Ⅱ Ⅲ Ⅳ Ⅴ Ⅵ */ 
    switch(N){
        case 3:
            *sector = 1;
            break;
        case 1:
            *sector = 2;
            break;
        case 5:
            *sector = 3;
            break;
        case 4:
            *sector = 4;
            break;
        case 6:
            *sector = 5;
            break;
        case 2:
            *sector = 6;
            break;
    }
#endif
    return N;
}


/**
 * @brief 矢量时间计算
 * 
 * @param foc_pwm 
 * @param sector 扇区
 * @param alphabeta αβ轴电压
 * @param Tpwm PWM周期
 * @param Udc 母线电压
 */
static void VectorActionTime(FOC_PWM_t *foc_pwm, uint8_t sector, alphabeta_t alphabeta, uint32_t Tpwm, float Udc)
{
    Udc = Udc*1.5f;

    float K = (float)Tpwm * SQRT_3/Udc;

    float X = K *(alphabeta.beta);
    float Y = K *(foc_pwm->svpwm_val1 + foc_pwm->svpwm_val2);
    float Z = K *(-foc_pwm->svpwm_val1 + foc_pwm->svpwm_val2);

    uint32_t T4 = 0, T6 = 0;
    uint32_t Ta = 0, Tb = 0, Tc = 0;
    uint32_t T1 = 0, T2 = 0, T3 = 0;
    float sum = 0.0f;
    float scale = 1.0f;

    switch(sector){
        case 1:
            T4 = Z;
            T6 = Y;
            break;
        case 2:
            T4 = Y;
            T6 = -X;
            break;
        case 3:
            T4 = -Z;
            T6 = X;
            break;
        case 4:
            T4 = -X;
            T6 = Z;
            break;
        case 5:
            T4 = X;
            T6 = -Y;
            break;
        case 6:
            T4 = -Y;
            T6 = -Z;
            break;
    }

    if(T4 + T6 > (uint32_t)((float)Tpwm * foc_pwm->K)){
        sum = (float)(T4 + T6);
        scale = ((float)Tpwm * foc_pwm->K) / sum;
        T4 = (uint32_t)((float)T4 * scale);
        T6 = (uint32_t)((float)T6 * scale);
    }

    Ta = (Tpwm-T4-T6)/4;
    Tb = Ta + T4/2;
    Tc = Tb + T6/2;

    switch(sector){
        case 1:
            T1 = Tb;
            T2 = Ta;
            T3 = Tc;
            break;
        case 2:
            T1 = Ta;
            T2 = Tc;
            T3 = Tb;
            break;
        case 3:
            T1 = Ta;
            T2 = Tb;
            T3 = Tc;
            break;
        case 4:
            T1 = Tc;
            T2 = Tb;
            T3 = Ta;
            break;
        case 5:
            T1 = Tc;
            T2 = Ta;
            T3 = Tb;
            break;
        case 6:
            T1 = Tb;
            T2 = Tc;
            T3 = Ta;
            break;
    }

    foc_pwm->T_abc.a = T1;
    foc_pwm->T_abc.b = T2;
    foc_pwm->T_abc.c = T3;
    

    // U_H_SET_PWM(T1+ 0.0135f*(Tpwm/2.0f));
    // V_H_SET_PWM(T2+ 0.0135f*(Tpwm/2.0f));
    // W_H_SET_PWM(T3+ 0.0135f*(Tpwm/2.0f));
    U_H_SET_PWM(T1+4.249f);                     /*加上死区补偿*/
    V_H_SET_PWM(T2+4.249f);
    W_H_SET_PWM(T3+4.249f);

    foc_pwm->_output.a = T1/(Tpwm/2.0f)*24.0f;
    foc_pwm->_output.b = T2/(Tpwm/2.0f)*24.0f;
    foc_pwm->_output.c = T3/(Tpwm/2.0f)*24.0f;

    

    // if(foc_pwm->_output.a > foc_pwm->maxt)foc_pwm->maxt = foc_pwm->_output.a;


}


/**
 * @brief 输出svpwm
 * 
 * @param foc_pwm 
 * @param U_qd qd轴电压
 * @param angle_el 电角度
 */
static void FOC_RUN_SVPWM(FOC_PWM_t *foc_pwm, qd_t U_qd, float angle_el)
{
    uint8_t N;
    alphabeta_t input = {0};

    input = FOC_Rev_Park(U_qd, angle_el);
    foc_pwm->alpha_beta = input;

    N = Sector_Judgment(foc_pwm, input, &foc_pwm->sector);
    VectorActionTime(foc_pwm, N, input, foc_pwm->Tpwm, foc_pwm->power);
}


/**
 * @brief 输出SPWM
 * 
 * @param foc_pwm 
 * @param U_qd qd轴电压
 * @param angle_el 电角度
 */
void FOC_RUN_SPWM(FOC_PWM_t *foc_pwm, qd_t U_qd, float angle_el)
{
    abc_t output             = {0};
    alphabeta_t input        = {0};

    float half_power = foc_pwm->power/2.0f;
    uint32_t Tpwm = foc_pwm->Tpwm/2;


    input = FOC_Rev_Park(U_qd, angle_el);
    foc_pwm->alpha_beta = input;
    /*克拉克逆变换*/
    output = FOC_Rev_Clarke(input);
    

    output.a += half_power;
    output.b += half_power;
    output.c += half_power;

    foc_pwm->_output = output;

    U_H_SET_PWM(output.a/foc_pwm->power *Tpwm+4.249f);                     /*加上死区补偿*/
    V_H_SET_PWM(output.b/foc_pwm->power *Tpwm+4.249f);
    W_H_SET_PWM(output.c/foc_pwm->power *Tpwm+4.249f);

}

/**
 * @brief 运行pwm
 * 
 * @param foc_pwm 
 * @param Uqd qd轴电压
 * @param ElAngle 电角度
 */
void FOC_PWM_Run(FOC_PWM_t *foc_pwm, qd_t Uqd, float ElAngle)
{
    foc_pwm->Uqd = Uqd;

    if (g_foc_pwm_debug_spwm != 0U)
    {
        FOC_RUN_SPWM(foc_pwm, Uqd, ElAngle);
    }
    else
    {
        FOC_RUN_SVPWM(foc_pwm, Uqd, ElAngle);
    }
}


/**
 * @brief FOC SVWPM初始化
 * 
 * @param foc_pwm 
 * @param Tpwm 
 * @param power 
 */
void FOC_PWM_Init(FOC_PWM_t *foc_pwm, uint32_t Tpwm, float power)
{
    foc_pwm->Tpwm = Tpwm;
    foc_pwm->power = power;

    foc_pwm->K = 1.0f;

    FOC_PWM_HW_Init();
}
void FOC_PWM_DeInit(FOC_PWM_t *foc_pwm)
{
    FOC_PWM_HW_DeInit();
    FOC_PWM_HW_ON_OFF(false, false, false, false, false, false);

    FOC_PWM_StopALL(foc_pwm);
}

/**
 * @brief 打开PWM所有通道
 * 
 * @param foc_pwm 
 */
void FOC_PWM_StartAll(FOC_PWM_t *foc_pwm)
{
    FOC_PWM_HW_ON_OFF(true, true, true, true, true, true);
}

/**关闭PWM所有通道 */
void FOC_PWM_StopALL(FOC_PWM_t *foc_pwm)
{
    FOC_PWM_HW_ON_OFF(false, false, false, false, false, false);
}



