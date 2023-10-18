///******************** (C) COPYRIGHT 2022 ：Rui Peng ********************************
// * Arthur     ：Rui Peng
// * file 	    motor_PID_Ctrl.c
// * website    ：https://arclab.hku.hk/
//**********************************************************************************/
 
#include "motor_PID_Ctrl.h"
#include  <math.h>    //Keil library  

extern TIM_HandleTypeDef htim2;

motor_pid left_motor_pid;
motor_pid right_motor_pid;



void motor_PID_init(void)
{
	left_motor_pid.kp = 50;
	left_motor_pid.ki = 4;
	left_motor_pid.iout_limit = 1000;
	left_motor_pid.kd = 0.5;

	right_motor_pid.kp = 50;
	right_motor_pid.ki = 4;
	right_motor_pid.iout_limit = 1000;
	right_motor_pid.kd = 0.5;

}


void motor_speed_Ctrl(motor_speed motor_des_speed, motor_speed motor_cur_speed)
{
	// left motor:
	left_motor_pid.err = motor_des_speed.speed_LeftMotor - motor_cur_speed.speed_LeftMotor;
	
	left_motor_pid.pout  = left_motor_pid.kp * left_motor_pid.err;
	left_motor_pid.iout += left_motor_pid.ki * left_motor_pid.err;
	left_motor_pid.iout  = output_limit(left_motor_pid.iout, left_motor_pid.iout_limit);
	
	left_motor_pid.last_err = left_motor_pid.err;
	
	left_motor_pid.out   = left_motor_pid.pout + left_motor_pid.iout + left_motor_pid.dout;
	motor_pwm_input.pwm_LeftMotor = left_motor_pid.out;
	
	
	// right motor:
	right_motor_pid.err = motor_des_speed.speed_RightMotor - motor_cur_speed.speed_RightMotor;
	
	right_motor_pid.pout  = right_motor_pid.kp * right_motor_pid.err;
	right_motor_pid.iout += right_motor_pid.ki * right_motor_pid.err;
	right_motor_pid.iout  = output_limit(right_motor_pid.iout, right_motor_pid.iout_limit);
	
	right_motor_pid.dout  = right_motor_pid.kd * (right_motor_pid.err - right_motor_pid.last_err);
	right_motor_pid.last_err = right_motor_pid.err;
		
	right_motor_pid.out   = right_motor_pid.pout + right_motor_pid.iout + right_motor_pid.dout;
	motor_pwm_input.pwm_RightMotor = right_motor_pid.out;


	
	// motor output:
	motor_direction(motor_pwm_input);

	__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_1, motor_pwm_input.pwm_LeftMotor);
	__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_2, motor_pwm_input.pwm_RightMotor);

	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);

}



float output_limit(float value, float limit)
{
	if(value > limit)
	{
		value = limit;
	}

	if(value < -limit)
	{
		value = -limit;
	}
		
	return value;
}



 