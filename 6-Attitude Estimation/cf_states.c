///******************** (C) COPYRIGHT 2022 ：Rui Peng ********************************
// * Arthur     ： Rui Peng
// * file 	    ： cf_states.c
// * website    ： https://arclab.hku.hk/
//**********************************************************************************/


#include <math.h>
#include "cf_states.h"
#include "mpu9250.h"
#include "imu_preprocess.h"

typedef struct __AHRS_state_data {

	float psi;
	float theta;
	float phi;

	quat qib;		 	 // Quaternion states "qib" = Quaternion from Inertial to Body
	
	// Entries for storing processed sensor data
	float gyro_x;
	float gyro_y;
	float gyro_z;
	
	float accel_x;
	float accel_y;
	float accel_z;
	
	float mag_x;
	float mag_y;
	float mag_z;

	int new_gyro_data;
	int new_accel_data;
	int new_mag_data;

	float vel_x_imu;
	float vel_y_imu;
	float vel_z_imu;

} AHRS_state_data;

AHRS_state_data gStateData = {0};


void CF_Init(void)
{
	//initialize original quaternion q
	float qib_init_cnt = 0;
	float p  = 0;  
	float r  = 0;  
	float y  = 0;

	while(1)
	{
		read_imu_data();
		
		float ax = mpu_data.ax ;  
		float ay = mpu_data.ay ;  
		float az = mpu_data.az ;
		float	accl_norm = sqrt(ax*ax + ay*ay + az*az);

		ax = ax / accl_norm;
		ay = ay / accl_norm;
		az = az / accl_norm;

		qib_init_cnt++;
		p += asin( ay ) ;//* 180.0 / 3.1415926;
		r += atan( -ax / az ) ;//* 180.0 / 3.1415926;
		
		if(qib_init_cnt > 500)
			break;
	}
	p = p / qib_init_cnt; 	
	r = r / qib_init_cnt;   
	
	gStateData.theta  = p * 180.0 / 3.1415926;
	gStateData.phi    = r * 180.0 / 3.1415926;
	
	gStateData.qib.q0  = cosf(p/2.0) * cosf(r/2.0) * cosf(y/2.0) - sinf(p/2.0) * sinf(r/2.0) * sinf(y/2.0);
	gStateData.qib.q1  = cosf(r/2.0) * cosf(y/2.0) * sinf(p/2.0) - cosf(p/2.0) * sinf(r/2.0) * sinf(y/2.0);
	gStateData.qib.q2  = cosf(p/2.0) * cosf(y/2.0) * sinf(r/2.0) + cosf(r/2.0) * sinf(p/2.0) * sinf(y/2.0);
	gStateData.qib.q3  = cosf(p/2.0) * cosf(r/2.0) * sinf(y/2.0) + cosf(y/2.0) * sinf(p/2.0) * sinf(r/2.0);
	quat_norm( &gStateData.qib );	
	//initialize original quaternion q

}



	
float Kp = 50, Ki = 0.001;
float eInt[3]={0};
float SamplePeriod = 0.001;

void CF_6_axis_update(AHRS_state_data* state_data, float T)
{
	float q1 = gStateData.qib.q0, q2 = gStateData.qib.q1, q3 = gStateData.qib.q2, q4 = gStateData.qib.q3;   // short name local variable for readability
	float norm;
	float vx, vy, vz;
	float ex, ey, ez;
	float pa, pb, pc;

	float ax = gStateData.accel_x;
	float ay = gStateData.accel_y;
	float az = gStateData.accel_z;
	
	float gx = gStateData.gyro_x;
	float gy = gStateData.gyro_y;
	float gz = gStateData.gyro_z;

	// Normalise accelerometer measurement
	norm = sqrt(ax * ax + ay * ay + az * az);
	if (norm == 0.0f) return; // handle NaN
	norm = 1 / norm;        // use reciprocal for division
	ax *= norm;
	ay *= norm;
	az *= norm;

	// Estimated direction of gravity
	vx = 2.0f * (q2 * q4 - q1 * q3);
	vy = 2.0f * (q1 * q2 + q3 * q4);
	vz = q1 * q1 - q2 * q2 - q3 * q3 + q4 * q4;

	// Error is cross product between estimated direction and measured direction of gravity
	ex = (ay * vz - az * vy);
	ey = (az * vx - ax * vz);
	ez = (ax * vy - ay * vx);
	
	if (Ki > 0.0f)
	{
			eInt[0] += ex;      // accumulate integral error
			eInt[1] += ey;
			eInt[2] += ez;
	}
	else
	{
			eInt[0] = 0.0f;     // prevent integral wind up
			eInt[1] = 0.0f;
			eInt[2] = 0.0f;
	}

	// Apply feedback terms
	gx = gx + Kp * ex + Ki * eInt[0];
	gy = gy + Kp * ey + Ki * eInt[1];
	gz = gz + Kp * ez + Ki * eInt[2];

	// Integrate rate of change of quaternion
	pa = q2;
	pb = q3;
	pc = q4;
	q1 = q1 + (-q2 * gx - q3 * gy - q4 * gz) * (0.5f * SamplePeriod);
	q2 = pa +  (q1 * gx + pb * gz - pc * gy) * (0.5f * SamplePeriod);
	q3 = pb +  (q1 * gy - pa * gz + pc * gx) * (0.5f * SamplePeriod);
	q4 = pc +  (q1 * gz + pa * gy - pb * gx) * (0.5f * SamplePeriod);

	// Normalise quaternion
	norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);
	norm = 1.0f / norm;
	
	gStateData.qib.q0 = q1 * norm;
	gStateData.qib.q1 = q2 * norm;
	gStateData.qib.q2 = q3 * norm;
	gStateData.qib.q3 = q4 * norm;
	
}


void CF_process(float T)
{
	
	Get_New_Sensor_Data();

	CF_6_axis_update  (&gStateData, T);
	
	compute_euler_angles(&gStateData);
}








