///******************** (C) COPYRIGHT 2022 ：Rui Peng ********************************
// * Arthur     ： Rui Peng
// * file 	    ： Attitude_Ctrl.c
// * website    ： https://arclab.hku.hk/
//**********************************************************************************/


#include "Attitude_Ctrl.h"
#include  <math.h>    //Keil library  
#include "motor_Config.h"
#include "ekf_states.h"


attitude_pid  atti_pid;
attitude_pid  rate_pid;


void attitude_ctrl_init(void)
{
	
	atti_pid.kp = 0.6;
	
	rate_pid.kp = 1;
}


void angle_ctrl(float des_angle, float cur_angle)
{
	atti_pid.err = des_angle - cur_angle;
	
	atti_pid.pout = atti_pid.kp * atti_pid.err;
	
	atti_pid.out  = atti_pid.pout + atti_pid.iout + atti_pid.dout;
	
	
	float des_rate = atti_pid.out;
	
	rate_ctrl( des_rate, gStateData.gyro_x);
}



void rate_ctrl(float des_rate, float cur_rate)
{
	rate_pid.err = des_rate - cur_rate;
	
	rate_pid.pout = rate_pid.kp * rate_pid.err;
	
	rate_pid.out  = rate_pid.pout + rate_pid.iout + rate_pid.dout;
	
	motor_des_speed.speed_LeftMotor  = rate_pid.out;
	motor_des_speed.speed_RightMotor = rate_pid.out;
}









