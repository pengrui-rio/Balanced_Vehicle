///******************** (C) COPYRIGHT 2022 ：Rui Peng ********************************
// * Arthur     ： Rui Peng
// * file 	    ： imu_preprocess.c
// * website    ： https://arclab.hku.hk/
//**********************************************************************************/
 
#include "imu_preprocess.h"
#include "main.h"
#include  <math.h>    //Keil library  
#include "mpu9250.h"

Acc_t  acc;
Gyro_t gyro;
Mag_t  mag;

mpu_data_t mpu_data;

int gyro_cali_flag = 0;
float wx_offset = 0, wy_offset = 0, wz_offset = 0;

int accl_cali_flag = 0;
float ax_up_offset = 0, ax_down_offset = 0, ax_up_cali_flag = 0, ax_down_cali_flag = 0;
float ay_up_offset = 0, ay_down_offset = 0, ay_up_cali_flag = 0, ay_down_cali_flag = 0;
float az_up_offset = 0, az_down_offset = 0, az_up_cali_flag = 0, az_down_cali_flag = 0;

void mpu_offset_call(void)
{	
	// gyro offset: only calibrate once
	
	while(gyro_cali_flag)
	{
		for(int i = 0; i < 300 ; i++)
		{ 	
			MPU_Read();
			
			wx_offset += gyro.wx;
			wy_offset += gyro.wy;
			wz_offset += gyro.wz;

			delay_ms(5);
		} 
		
		mpu_data.wx_offset = wx_offset / 300.0;
		mpu_data.wy_offset = wy_offset / 300.0;
		mpu_data.wz_offset = wz_offset / 300.0;

		break;
	}
	
	if(gyro_cali_flag == 0) // use calibrated offset
	{
		mpu_data.wx_offset = -12.5;   
		mpu_data.wy_offset =  5.17;   
		mpu_data.wz_offset = -20.62;  
	}
	
	///////////////////////////////////////////////////////////////////////////////////
	// accl offset:  only calibrate once
	int accl_cnt = 0;
	float accl_loop = 500;

	while(accl_cali_flag)
	{
//		READ_MPU9250_ACCEL();
		
		//////////////////
		// z up calibrate:
		if( (acc.ax > -100 && acc.ax < 100) && (acc.ay > -100 && acc.ay < 100) && (acc.az > 0) && (az_up_cali_flag == 0) )
		{
			accl_cnt++;
			az_up_offset += acc.az;
			delay_ms(5);

			if(accl_cnt >= accl_loop)
			{
				az_up_offset = az_up_offset / accl_loop;
				az_up_cali_flag = 1;
				accl_cnt = 0;
			}
		} 
		// z down calibrate:
		if( (acc.ax > -100 && acc.ax < 100) && (acc.ay > -100 && acc.ay < 100) && (acc.az < 0) && (az_down_cali_flag == 0) )
		{
			accl_cnt++;
			az_down_offset += acc.az;
			delay_ms(5);

			if(accl_cnt >= accl_loop)
			{
				az_down_offset = az_down_offset / accl_loop;
				az_down_cali_flag = 1;
				accl_cnt = 0;
			}
		} 
		//////////////////
		
		//////////////////
		// y up calibrate:
		if( (acc.ax > -100 && acc.ax < 100) && (acc.az > -100 && acc.az < 100) && (acc.ay > 0) && (ay_up_cali_flag == 0) )
		{
			accl_cnt++;
			ay_up_offset += acc.ay;
			delay_ms(5);

			if(accl_cnt >= accl_loop)
			{
				ay_up_offset = ay_up_offset / accl_loop;
				ay_up_cali_flag = 1;
				accl_cnt = 0;
			}
		} 
		// y down calibrate:
		if( (acc.ax > -100 && acc.ax < 100) && (acc.az > -100 && acc.az < 100) && (acc.ay < 0) && (ay_down_cali_flag == 0) )
		{
			accl_cnt++;
			ay_down_offset += acc.ay;
			delay_ms(5);

			if(accl_cnt >= accl_loop)
			{
				ay_down_offset = ay_down_offset / accl_loop;
				ay_down_cali_flag = 1;
				accl_cnt = 0;
			}
		} 
		//////////////////

		//////////////////
		// x up calibrate:
		if( (acc.ay > -100 && acc.ay < 100) && (acc.az > -100 && acc.az < 100) && (acc.ax > 0) && (ax_up_cali_flag == 0) )
		{
			accl_cnt++;
			ax_up_offset += acc.ax;
			delay_ms(5);

			if(accl_cnt >= accl_loop)
			{
				ax_up_offset = ax_up_offset / accl_loop;
				ax_up_cali_flag = 1;
				accl_cnt = 0;
			}
		} 
		// x down calibrate:
		if( (acc.ay > -100 && acc.ay < 100) && (acc.az > -100 && acc.az < 100) && (acc.ax < 0) && (ax_down_cali_flag == 0) )
		{
			accl_cnt++;
			ax_down_offset += acc.ax;
			delay_ms(5);

			if(accl_cnt >= accl_loop)
			{
				ax_down_offset = ax_down_offset / accl_loop;
				ax_down_cali_flag = 1;
				accl_cnt = 0;
			}
		} 
		//////////////////
		
		if(az_up_cali_flag && az_down_cali_flag && ay_up_cali_flag && ay_down_cali_flag && ax_up_cali_flag && ax_down_cali_flag)
		{
			mpu_data.ax_offset = (ax_up_offset + ax_down_offset) / 2.0;  
			mpu_data.ay_offset = (ay_up_offset + ay_down_offset) / 2.0;  
			mpu_data.az_offset = (az_up_offset + az_down_offset) / 2.0;  

			break;
		}
	}
	
	if(accl_cali_flag == 0)  // use calibrated offset
	{
		mpu_data.ax_offset = 459.37;  
		mpu_data.ay_offset = 305.09;  
		mpu_data.az_offset = -348.14;   
	}

}


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

 
/**
	* @brief  get the data of imu
  * @param  
	* @retval 
  * @usage  call in main() function
	*/


void MPU_Read(void)
{		
	short Gyro[3],Acc[3];//���ݼĴ�����
	
	MPU_Get_Accelerometer(Acc, Acc+1, Acc+2);			 //��ȡ�Ǽ��ٶ�ԭʼ����
	acc.ax  = Acc[0];
	acc.ay  = Acc[1];
	acc.az  = Acc[2];
	
	MPU_Get_Gyroscope(Gyro, Gyro+1, Gyro+2);			 //��ȡ���ٶ�ԭʼ����
	gyro.wx = Gyro[0];
	gyro.wy = Gyro[1];
	gyro.wz = Gyro[2];
	
//	short Mag[3] ;
//	MPU_Get_Magnetometer(Mag, Mag+1, Mag+2);			 //��ȡ������ԭʼ����
//	mag.mx  = Mag[0];
//	mag.my  = Mag[1];
//	mag.mz  = Mag[2];

}

                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                
void read_imu_data(void)
{
//	READ_MPU9250_ACCEL();
	MPU_Read();
	
	mpu_data.ax = acc.ax - mpu_data.ax_offset;
	mpu_data.ay = acc.ay - mpu_data.ay_offset;
	mpu_data.az = acc.az - mpu_data.az_offset;

	/////////
	
//	READ_MPU9250_GYRO();

	mpu_data.wx = (gyro.wx - mpu_data.wx_offset) / 16.384f / 57.3f / 5.0f;
	mpu_data.wy = (gyro.wy - mpu_data.wy_offset) / 16.384f / 57.3f / 5.0f;
	mpu_data.wz = (gyro.wz - mpu_data.wz_offset) / 16.384f / 57.3f / 5.0f;

	
	/////////
//	
//	READ_MPU9250_MAG();
//	mpu_data.mx = mag.mx - mpu_data.mx_offset;
//	mpu_data.my = mag.my - mpu_data.my_offset;
//	mpu_data.mz = mag.mz - mpu_data.mz_offset;

}







