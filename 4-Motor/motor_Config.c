///******************** (C) COPYRIGHT 2022 ：Rui Peng ********************************
// * Arthur     ：Rui Peng
// * file 	    motor_Config.c
// * website    ：https://arclab.hku.hk/
//**********************************************************************************/
 
#include "motor_Config.h"
#include "main.h"
#include  <math.h>    //Keil library  
 
motor_pwm   motor_pwm_input;
motor_speed motor_cur_speed;
motor_speed motor_des_speed;

int16_t  GetData_tim3 = 0, last_GetData_tim3 = 0;//?????
int16_t  GetData_tim4 = 0, last_GetData_tim4 = 0;//?????


void motor_direction(motor_pwm motor_pwm_input)
{
	// enable motor:  STBY = 1
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

	
	//Left motor
	if(motor_pwm_input.pwm_LeftMotor > 0)
	{
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);
	}
	else if(motor_pwm_input.pwm_LeftMotor < 0)
	{
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET);
	}
	
	//Right motor
	if(motor_pwm_input.pwm_RightMotor > 0)
	{
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
	}
	else if(motor_pwm_input.pwm_RightMotor < 0)
	{
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
	}

}

float test111 = 0;
int encoder_period = 20000;
void motor_speed_compute(float T)
{
	
	//Left motor
	if( (GetData_tim4 - last_GetData_tim4) >  (encoder_period / 2) )
	{
		last_GetData_tim4 = last_GetData_tim4 + encoder_period;
	}
	if( (GetData_tim4 - last_GetData_tim4) < -(encoder_period / 2) )
	{
		last_GetData_tim4 = last_GetData_tim4 - encoder_period;
	}
	
	motor_cur_speed.speed_LeftMotor  = -(GetData_tim4 - last_GetData_tim4) / T * 1000;
	last_GetData_tim4 = GetData_tim4;
	
	//Right motor
	if( (GetData_tim3 - last_GetData_tim3) >  (encoder_period / 2) )
	{
		last_GetData_tim3 = last_GetData_tim3 + encoder_period;
	}
	if( (GetData_tim3 - last_GetData_tim3) < -(encoder_period / 2) )
	{
		last_GetData_tim3 = last_GetData_tim3 - encoder_period;
	}

	motor_cur_speed.speed_RightMotor = (GetData_tim3 - last_GetData_tim3) / T * 1000;
	last_GetData_tim3 = GetData_tim3;

}



int abs_func(int a)
{
	if (a > 0)
		return a;
	else
		return -a;
}



 