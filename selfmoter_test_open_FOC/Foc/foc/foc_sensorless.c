/**
 ****************************************************************************************************
 * @file        foc_sensorless.c
 * @author      哔哩哔哩-Rebron大侠
 * @version     V0.0
 * @date        2025-01-11
 * @brief       无感综合
 * @license     MIT License
 *              Copyright (c) 2025 Reborn大侠
 *              允许任何人使用、复制、修改和分发该代码，但需保留此版权声明。
 ****************************************************************************************************
 */


#include "foc_sensorless.h"
#include "foc_math.h"
#include "stdio.h"


// #define MAX_DEG_ERR					                1.1745329252f				/*原子的电机角度偏差有点大*/
#define MAX_DEG_ERR					                0.6745329252f				
#define SW_TIMES                                    1000

/**
 * @brief 无感观测器初始化
 * 
 * @param foc_sl 
 * @param Rs            相电阻
 * @param Ls            相电感
 * @param freq          运行频率
 * @param pole_pairs    极对数
 * @param Iqd_over      qd轴过流值
 */
void FOC_Sensorless_Init(FOC_SENSORLESS_t *foc_sl, float Rs, float Ls, float freq, uint32_t pole_pairs, float Iqd_over)
{
    FOC_HFI_Init(&foc_sl->hfi, foc_sl->hfi_vol_ref, foc_sl->d_bias_vol, freq, foc_sl->hfi_freq, pole_pairs);

    FOC_SMO_Init(&foc_sl->smo, Rs, Ls, 1.0f/freq, pole_pairs, 100000);

    // FOC_FLUX_Init(&foc_sl->flux, Rs/3.0f*2.0f, Ls/3.0f*2.0f, 1.0f/freq, pole_pairs, 100000);
    FOC_FLUX_Init(&foc_sl->flux, Rs, Ls, 1.0f/freq, pole_pairs, 100000);

    FOC_VF_Init(&foc_sl->vf, pole_pairs, 1.0f/freq);

    foc_sl->Iqd_over = Iqd_over;

    foc_sl->sta = VF_STA;
}





/**
 * @brief 无感方向错误
 * 
 * @param foc_sl 
 * @param taget_speed 目标速度
 */
void FOC_SL_DirErr(FOC_SENSORLESS_t *foc_sl, float taget_speed)
{
    switch(foc_sl->Observer){
        case HFI_OBS:
            if(fabsf(foc_sl->speed.AvrMecSpeed)<1000)break;

            if(FOC_SIGN_Judgment(foc_sl->hfi.speed.AvrMecSpeed)!=FOC_SIGN_Judgment(taget_speed)){
                if(FOC_ERR(foc_sl->hfi.speed.ElAngle, foc_sl->smo.speed.ElAngle)>MAX_DEG_ERR){
                    foc_sl->dir_err++;
                    if(foc_sl->dir_err> SW_TIMES){
                        foc_sl->dir_err=0;
                        foc_sl->hfi.polarity_ELAngle = PI;
                     //   foc_sl->hfi.pll.Angle = Limit_Angle(foc_sl->hfi.pll.Angle + 0.99f*PI);
                        // printf("高频注入方向纠正\r\n");
                    }
                }else{
                    if(foc_sl->dir_err>0)foc_sl->dir_err--;
                }
            }else{
                foc_sl->dir_err=0;
            }
        default:
            break;
    }
}

/**
 * @brief 高频注入--滑模切换
 * 
 * @param foc_sl 
 * @param speed 当前速度
 */
void FOC_SL_HFI_SMO_SW(FOC_SENSORLESS_t *foc_sl, float speed)
{
    float ElAngle_err = foc_sl->err;

    speed = fabs(speed);
 
    switch(foc_sl->sta){
        case HFI_POL_STA:
            
            if(foc_sl->hfi.polarityFlag){

                if((foc_sl->hfi.Id_lf_sum[0]) > (foc_sl->hfi.Id_lf_sum[1])){
                    foc_sl->hfi.polarity_ELAngle = 0;
                }else{
                    foc_sl->hfi.polarity_ELAngle = PI;
                    foc_sl->hfi.pll.Angle = Limit_Angle(foc_sl->hfi.pll.Angle + PI);
                }
                foc_sl->hfi.Id_lf_sum[0] = 0;
                foc_sl->hfi.Id_lf_sum[1] = 0;
                foc_sl->hfi.sum_count0 = 0;
                foc_sl->hfi.sum_count = 0;

                foc_sl->sta = HFI_STA;                  /*极性辨识完成, 进入高频运行*/
            }
            break;
        case HFI_STA:
            if( speed > foc_sl->up_smo){
                if(ElAngle_err<=MAX_DEG_ERR){
                    foc_sl->up_time++;                  /*保持角度差一直低于 xx°*/
                }else{
                    if(foc_sl->up_time>0){
                        foc_sl->up_time--;
                    }
                }
                if(foc_sl->up_time>SW_TIMES&&ElAngle_err<=MAX_DEG_ERR){        /*切换到滑模 */
                    FOC_HFI_Clear(&foc_sl->hfi);
                    foc_sl->sta = SMO_STA;       
                    foc_sl->Observer = SMO_OBS;          
                    foc_sl->up_time = 0;
                }
            }else {
                foc_sl->down_time=0;
                foc_sl->up_time=0;
            }

            break;
        case SMO_STA:
            if( speed < foc_sl->smo_down){
                foc_sl->down_time++;
                if(foc_sl->down_time>2){
                    foc_sl->sta = HFI_STA;
                    foc_sl->down_time = 0;
                    foc_sl->speed_err = false;
                    FOC_SMO_Clear(&foc_sl->smo);
                }
            }else{
                foc_sl->down_time=0;
                foc_sl->up_time=0;
            }
            break;
        default:
            break;
    }
}


/**
 * @brief VF强拖--滑模切换
 * 
 * @param foc_sl 
 * @param speed 当前速度
 */
void FOC_SL_VF_SMO_SW(FOC_SENSORLESS_t *foc_sl, float speed)
{
    float ElAngle_err = foc_sl->err;

    speed = fabs(speed);
 
    switch(foc_sl->sta){
        case VF_STA:
            if( speed > foc_sl->up_smo){
                if(ElAngle_err<=MAX_DEG_ERR){
                    foc_sl->up_time++;                  /*保持角度差一直低于 xx°*/
                }else{
                    if(foc_sl->up_time>0){
                        foc_sl->up_time--;
                    }
                }
                if(foc_sl->up_time>SW_TIMES&&ElAngle_err<=MAX_DEG_ERR){        /*切换到滑模 */
                    foc_sl->sta = SMO_STA;       
                    foc_sl->Observer = SMO_OBS;          
                    foc_sl->up_time = 0;
                }
            }else {
                foc_sl->down_time=0;
                foc_sl->up_time=0;
            }

            break;
        case SMO_STA:
            if( speed < foc_sl->smo_down){
                foc_sl->down_time++;
                if(foc_sl->down_time>2){
                    foc_sl->sta = VF_STA;
                    foc_sl->down_time = 0;
                    //FOC_SMO_Clear(&foc_sl->smo);
                }
            }else{
                foc_sl->down_time=0;
                foc_sl->up_time=0;
            }
            break;
        default:
            break;
    }
}


/**
 * @brief 无感运行 VF--高频注入--滑模 返回αβ轴低频电流
 * 
 * @param foc_sl 
 * @param I_alphabeta αβ轴电流
 * @param Last_Ualphabeta αβ轴电压
 * @param tatget_speed 目标速度
 * @return alphabeta_t 返回αβ轴低频电流
 */
alphabeta_t FOC_SL_RUN(FOC_SENSORLESS_t *foc_sl, alphabeta_t I_alphabeta, alphabeta_t Last_Ualphabeta, float tatget_speed)
{
    FOC_SMO_t *smo = &foc_sl->smo;
    FOC_HFI_t *hfi = &foc_sl->hfi;
    FOC_VF_t *vf = &foc_sl->vf;
    FOC_FLUX_t *flux = &foc_sl->flux;                                   

    foc_sl->d_bias = 0;                                                 /*偏置电流归零*/

    switch(foc_sl->sta){
        case VF_STA:
            FOC_VF_Angle_Calc(vf);                                      /*计算VF强拖角度*/
            foc_sl->Observer = VF_OBS;
            FOC_SMO_Angle_Calc(smo, I_alphabeta, Last_Ualphabeta);      /*计算滑模角度*/
            // FOC_FLUX_Angle_Calc(flux, Last_Ualphabeta, I_alphabeta); 
                                                 
                                              /*强拖角度*/
            break;
        case HFI_POL_STA:
            FOC_HFI_Angle_Calc(hfi, I_alphabeta);                       /*计算高频电角度*/
            FOC_HFI_InjectPolIdent(hfi);                                /*极性辨识注入*/    
            foc_sl->d_bias = hfi->hfi_vol;                              /*添加偏置电流*/
            I_alphabeta = hfi->alphabeta_f;                             /*提取低频固定轴坐标系电流*/
            foc_sl->Observer = HFI_OBS;                                 /*使用高频注入观测器*/

            break;
        case HFI_STA:
            FOC_HFI_Angle_Calc(hfi, I_alphabeta);                       /*计算高频电角度*/
            FOC_HFI_Inject(hfi);                                        /*高频注入*/    
            foc_sl->d_bias = hfi->hfi_vol;                              /*添加偏置电流*/
            I_alphabeta = hfi->alphabeta_f;                             /*提取低频固定轴坐标系电流*/
            foc_sl->Observer = HFI_OBS;                                 /*使用高频注入观测器*/
            FOC_SMO_Angle_Calc(smo, I_alphabeta, Last_Ualphabeta);      /*计算滑模角度*/
            // FOC_FLUX_Angle_Calc(flux, Last_Ualphabeta, I_alphabeta); 
            break;

        case SMO_STA:
            /*计算滑模*/
            foc_sl->Observer = SMO_OBS;                                 /*使用滑模观测器*/
            FOC_SMO_Angle_Calc(smo, I_alphabeta, Last_Ualphabeta);      /*计算滑模角度*/
            // FOC_FLUX_Angle_Calc(flux, Last_Ualphabeta, I_alphabeta); 

            break;
        case FLUX_STA:
            foc_sl->Observer = FLUX_OBS;                                 /*使用磁链观测器*/

            FOC_FLUX_EFFECTIVE_Angle_Calc(flux, Last_Ualphabeta, I_alphabeta);      /*使用有效磁链，有效磁链会占用比较大的带宽*/
            // FOC_SMO_Angle_Calc(smo, I_alphabeta, Last_Ualphabeta);      /*计算滑模角度*/
            // FOC_FLUX_Angle_Calc(flux, Last_Ualphabeta, I_alphabeta); 
            break;
    }

    FOC_SL_DirErr(foc_sl, tatget_speed);                                /*判断高频注入方向是否错误*/

    /*根据观测器，选择参考速度*/
    switch(foc_sl->Observer){
        case VF_OBS:
            foc_sl->speed = vf->speed;
            break;
        case HFI_OBS:
            foc_sl->speed = hfi->speed;
            break;
        case SMO_OBS:
            foc_sl->speed = smo->speed;
            hfi->speed = smo->speed;
            hfi->pll.Angle = smo->speed.ElAngle;
            break;
        case FLUX_OBS:
            foc_sl->speed = flux->speed;
            hfi->speed = flux->speed;
            hfi->pll.Angle = flux->speed.ElAngle;
            break;
        default:
            break;
    }

    foc_sl->err = FOC_ERR(foc_sl->smo.speed.ElAngle, foc_sl->hfi.speed.ElAngle);    /*计算两个观测器之间的角度误差*/
    /*高频注入-滑模切换*/
    FOC_SL_HFI_SMO_SW(foc_sl, foc_sl->speed.AvrMecSpeed); 

    return I_alphabeta;
}


/**
 * @brief 无感速度计算
 * 
 * @param foc_sl 
 * @param Ts 该函数的调用周期: 单位s
 */
void FOC_Sensorless_Speed_Cale(FOC_SENSORLESS_t *foc_sl, float Ts)
{   
    FOC_SMO_t *smo = &foc_sl->smo;

    FOC_HFI_t *hfi = &foc_sl->hfi;
    FOC_VF_t  *vf  = &foc_sl->vf;
    FOC_FLUX_t *flux = &foc_sl->flux;

    FOC_HFI_Speed_Calc(hfi);
    FOC_SMO_Speed_Calc(smo);
    FOC_VF_Speed_Calc(vf);
    FOC_FLUX_Speed_Calc(flux);


    if(foc_sl->speed.AvrMecSpeed>=0){
        smo->speed.dir = COROTATION;
    }else{
        smo->speed.dir = REVERSAL;
    }
}


/**
 * @brief 无感速度失控
 * 
 * @param foc_sl 
 * @param Iqd qd轴电流
 * @return true 失控
 * @return false 未失控
 */
bool FOC_SL_Speed_ERR(FOC_SENSORLESS_t *foc_sl, qd_t Iqd)
{
    qd_t temp = {0};
    switch(foc_sl->Observer){
        case SMO_OBS:
            temp.q = fabsf(Iqd.q);
            if(temp.q > (foc_sl->Iqd_over)){
                foc_sl->speed_err = true;
                foc_sl->smo_time=0;
                printf("无感速度失控\r\n");
                return true;
            }
            break;
        default:
            break;
    }

    return false;
}




















