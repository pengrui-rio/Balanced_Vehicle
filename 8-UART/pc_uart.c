///******************** (C) COPYRIGHT 2022 ：Rui Peng ********************************
// * Arthur     ：Rui Peng
// * file 	    ：pc_uart.c
// * website    ：https://arclab.hku.hk/
//**********************************************************************************/


#include "pc_uart.h"
#include "motor_Config.h"


extern UART_HandleTypeDef huart1;

////#include "usart.h"

 
unsigned char tx_buf_toROS[20] = {0x00};
 

void SendMessage_to_PC(void)
{
	// position response
	int des_left  = motor_des_speed.speed_LeftMotor 				* 10;
	int cur_left  = motor_cur_speed.speed_LeftMotor 				* 10;
	int des_right = motor_des_speed.speed_RightMotor 				* 10;
	int cur_right = motor_cur_speed.speed_RightMotor 				* 10;
 
	int qw  = gStateData.qib.q0 * 10000;
	int qx  = gStateData.qib.q1 * 10000;
	int qy  = gStateData.qib.q2 * 10000;
	int qz  = gStateData.qib.q3 * 10000;

 //	//////////////////////////////////////////////////////

	tx_buf_toROS[0]  = 0x7E & 0xff; //head

 	tx_buf_toROS[1] =  des_left & 0xff; 
	tx_buf_toROS[2] = (des_left >> 8) & 0xff; 
	tx_buf_toROS[3] =  cur_left & 0xff; 
	tx_buf_toROS[4] = (cur_left >> 8) & 0xff; 
	tx_buf_toROS[5] =  des_right & 0xff; 
	tx_buf_toROS[6] = (des_right >> 8) & 0xff; 
	tx_buf_toROS[7] =  cur_right & 0xff; 
	tx_buf_toROS[8] = (cur_right >> 8) & 0xff; 

 	tx_buf_toROS[9] =  qw & 0xff; 
	tx_buf_toROS[10] = (qw >> 8) & 0xff; 
	tx_buf_toROS[11] =  qx & 0xff; 
	tx_buf_toROS[12] = (qx >> 8) & 0xff; 
	tx_buf_toROS[13] =  qy & 0xff; 
	tx_buf_toROS[14] = (qy >> 8) & 0xff; 
	tx_buf_toROS[15] =  qz & 0xff; 
	tx_buf_toROS[16] = (qz >> 8) & 0xff; 


 	// check sum
	unsigned char checksum = 0x00;
	for(int i = 1; i <= 16 ; i++)    // 1 - last one  
	{ 
		checksum += tx_buf_toROS[i];
	}
	tx_buf_toROS[17] = ( ~checksum ) & 0xff;   // +1
	// check sum
						
	tx_buf_toROS[18] = 0x7F & 0xff;      //end        

 	HAL_UART_Transmit_IT(&huart1, tx_buf_toROS, sizeof(tx_buf_toROS));                     

}





