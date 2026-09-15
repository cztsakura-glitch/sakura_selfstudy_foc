
#ifndef __FOC_PORT_H
#define __FOC_PORT_H


#include <tim.h>
#include <sys.h>

#include <stdint.h>
#include "foc_type.h"



/*PWM接口*/
#define U_H_SET_PWM(x)       (htim1.Instance->CCR1 =x)
#define V_H_SET_PWM(x)       (htim1.Instance->CCR2 =x)
#define W_H_SET_PWM(x)       (htim1.Instance->CCR3 =x)
#define U_L_SET_PWM(x)       (x)
#define V_L_SET_PWM(x)       (x)
#define W_L_SET_PWM(x)       (x)

/*霍尔接口*/
#define HALL_U_GET             HAL_GPIO_ReadPin(HALL_U_GPIO_Port, HALL_U_Pin)
#define HALL_V_GET             HAL_GPIO_ReadPin(HALL_V_GPIO_Port, HALL_V_Pin)
#define HALL_W_GET             HAL_GPIO_ReadPin(HALL_W_GPIO_Port, HALL_W_Pin) 

/*电机使能*/
#define MOTOR_ENABLE         
#define MOTOR_DISENABLE        




void FOC_Port_Start(void);
void FOC_Port_Stop(void);

void FOC_Port_Pwm_Start(bool A, bool A_N, bool B, bool B_N, bool C, bool C_N);
void FOC_Port_Pwm_Stop(void);









#endif
