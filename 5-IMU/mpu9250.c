///******************** (C) COPYRIGHT 2022 ：Rui Peng ********************************
// * Arthur     ：Rui Peng
// * file 	    ：mpu9250.c
// * website    ：https://arclab.hku.hk/
//**********************************************************************************/


#include "mpu9250.h"
#include "mpuiic.h"
#include "imu_preprocess.h"


#define MPU9250_ADDR     		0X68    //MPU6500µÄÆ÷¼þIICµØÖ·
#define MPU6500_ID				0X71  	//MPU6500µÄÆ÷¼þID

//MPU9250ÄÚ²¿·â×°ÁËÒ»¸öAK8963´ÅÁ¦¼Æ,µØÖ·ºÍIDÈçÏÂ:
#define AK8963_ADDR				0X0C	//AK8963µÄI2CµØÖ·
#define AK8963_ID			  	0X48	//AK8963µÄÆ÷¼þID


//AK8963µÄÄÚ²¿¼Ä´æÆ÷
#define MAG_WIA				  	0x00	//AK8963µÄÆ÷¼þID¼Ä´æÆ÷µØÖ·
#define MAG_CNTL1      		    0X0A    
#define MAG_CNTL2         		0X0B

#define MAG_XOUT_L				0X03	
#define MAG_XOUT_H				0X04
#define MAG_YOUT_L				0X05
#define MAG_YOUT_H				0X06
#define MAG_ZOUT_L				0X07
#define MAG_ZOUT_H				0X08

//MPU6500µÄÄÚ²¿¼Ä´æÆ÷
#define MPU_SELF_TESTX_REG		0X0D	//×Ô¼ì¼Ä´æÆ÷X
#define MPU_SELF_TESTY_REG		0X0E	//×Ô¼ì¼Ä´æÆ÷Y
#define MPU_SELF_TESTZ_REG		0X0F	//×Ô¼ì¼Ä´æÆ÷Z
#define MPU_SELF_TESTA_REG		0X10	//×Ô¼ì¼Ä´æÆ÷A
#define MPU_SAMPLE_RATE_REG		0X19	//²ÉÑùÆµÂÊ·ÖÆµÆ÷
#define MPU_CFG_REG			    0X1A	//ÅäÖÃ¼Ä´æÆ÷
#define MPU_GYRO_CFG_REG 		0X1B	//ÍÓÂÝÒÇÅäÖÃ¼Ä´æÆ÷
#define MPU_ACCEL_CFG_REG	    0X1C	//¼ÓËÙ¶È¼ÆÅäÖÃ¼Ä´æÆ÷
#define MPU_MOTION_DET_REG		0X1F	//ÔË¶¯¼ì²â·§ÖµÉèÖÃ¼Ä´æÆ÷
#define MPU_FIFO_EN_REG 		0X23	//FIFOÊ¹ÄÜ¼Ä´æÆ÷
#define MPU_I2CMST_CTRL_REG		0X24	//IICÖ÷»ú¿ØÖÆ¼Ä´æÆ÷
#define MPU_I2CSLV0_ADDR_REG	0X25	//IIC´Ó»ú0Æ÷¼þµØÖ·¼Ä´æÆ÷
#define MPU_I2CSLV0_REG 		0X26	//IIC´Ó»ú0Êý¾ÝµØÖ·¼Ä´æÆ÷
#define MPU_I2CSLV0_CTRL_REG	0X27	//IIC´Ó»ú0¿ØÖÆ¼Ä´æÆ÷
#define MPU_I2CSLV1_ADDR_REG	0X28	//IIC´Ó»ú1Æ÷¼þµØÖ·¼Ä´æÆ÷
#define MPU_I2CSLV1_REG			0X29	//IIC´Ó»ú1Êý¾ÝµØÖ·¼Ä´æÆ÷
#define MPU_I2CSLV1_CTRL_REG	0X2A	//IIC´Ó»ú1¿ØÖÆ¼Ä´æÆ÷
#define MPU_I2CSLV2_ADDR_REG	0X2B	//IIC´Ó»ú2Æ÷¼þµØÖ·¼Ä´æÆ÷
#define MPU_I2CSLV2_REG			0X2C	//IIC´Ó»ú2Êý¾ÝµØÖ·¼Ä´æÆ÷
#define MPU_I2CSLV2_CTRL_REG	0X2D	//IIC´Ó»ú2¿ØÖÆ¼Ä´æÆ÷
#define MPU_I2CSLV3_ADDR_REG	0X2E	//IIC´Ó»ú3Æ÷¼þµØÖ·¼Ä´æÆ÷
#define MPU_I2CSLV3_REG			0X2F	//IIC´Ó»ú3Êý¾ÝµØÖ·¼Ä´æÆ÷
#define MPU_I2CSLV3_CTRL_REG	0X30	//IIC´Ó»ú3¿ØÖÆ¼Ä´æÆ÷
#define MPU_I2CSLV4_ADDR_REG	0X31	//IIC´Ó»ú4Æ÷¼þµØÖ·¼Ä´æÆ÷
#define MPU_I2CSLV4_REG			0X32	//IIC´Ó»ú4Êý¾ÝµØÖ·¼Ä´æÆ÷
#define MPU_I2CSLV4_DO_REG		0X33	//IIC´Ó»ú4Ð´Êý¾Ý¼Ä´æÆ÷
#define MPU_I2CSLV4_CTRL_REG	0X34	//IIC´Ó»ú4¿ØÖÆ¼Ä´æÆ÷
#define MPU_I2CSLV4_DI_REG		0X35	//IIC´Ó»ú4¶ÁÊý¾Ý¼Ä´æÆ÷

#define MPU_I2CMST_STA_REG		0X36	//IICÖ÷»ú×´Ì¬¼Ä´æÆ÷
#define MPU_INTBP_CFG_REG		0X37	//ÖÐ¶Ï/ÅÔÂ·ÉèÖÃ¼Ä´æÆ÷
#define MPU_INT_EN_REG			0X38	//ÖÐ¶ÏÊ¹ÄÜ¼Ä´æÆ÷
#define MPU_INT_STA_REG			0X3A	//ÖÐ¶Ï×´Ì¬¼Ä´æÆ÷

#define MPU_ACCEL_XOUTH_REG		0X3B	//¼ÓËÙ¶ÈÖµ,XÖá¸ß8Î»¼Ä´æÆ÷
#define MPU_ACCEL_XOUTL_REG		0X3C	//¼ÓËÙ¶ÈÖµ,XÖáµÍ8Î»¼Ä´æÆ÷
#define MPU_ACCEL_YOUTH_REG		0X3D	//¼ÓËÙ¶ÈÖµ,YÖá¸ß8Î»¼Ä´æÆ÷
#define MPU_ACCEL_YOUTL_REG		0X3E	//¼ÓËÙ¶ÈÖµ,YÖáµÍ8Î»¼Ä´æÆ÷
#define MPU_ACCEL_ZOUTH_REG		0X3F	//¼ÓËÙ¶ÈÖµ,ZÖá¸ß8Î»¼Ä´æÆ÷
#define MPU_ACCEL_ZOUTL_REG		0X40	//¼ÓËÙ¶ÈÖµ,ZÖáµÍ8Î»¼Ä´æÆ÷

#define MPU_TEMP_OUTH_REG		0X41	//ÎÂ¶ÈÖµ¸ß°ËÎ»¼Ä´æÆ÷
#define MPU_TEMP_OUTL_REG		0X42	//ÎÂ¶ÈÖµµÍ8Î»¼Ä´æÆ÷

#define MPU_GYRO_XOUTH_REG		0X43	//ÍÓÂÝÒÇÖµ,XÖá¸ß8Î»¼Ä´æÆ÷
#define MPU_GYRO_XOUTL_REG		0X44	//ÍÓÂÝÒÇÖµ,XÖáµÍ8Î»¼Ä´æÆ÷
#define MPU_GYRO_YOUTH_REG		0X45	//ÍÓÂÝÒÇÖµ,YÖá¸ß8Î»¼Ä´æÆ÷
#define MPU_GYRO_YOUTL_REG		0X46	//ÍÓÂÝÒÇÖµ,YÖáµÍ8Î»¼Ä´æÆ÷
#define MPU_GYRO_ZOUTH_REG		0X47	//ÍÓÂÝÒÇÖµ,ZÖá¸ß8Î»¼Ä´æÆ÷
#define MPU_GYRO_ZOUTL_REG		0X48	//ÍÓÂÝÒÇÖµ,ZÖáµÍ8Î»¼Ä´æÆ÷

#define MPU_I2CSLV0_DO_REG		0X63	//IIC´Ó»ú0Êý¾Ý¼Ä´æÆ÷
#define MPU_I2CSLV1_DO_REG		0X64	//IIC´Ó»ú1Êý¾Ý¼Ä´æÆ÷
#define MPU_I2CSLV2_DO_REG		0X65	//IIC´Ó»ú2Êý¾Ý¼Ä´æÆ÷
#define MPU_I2CSLV3_DO_REG		0X66	//IIC´Ó»ú3Êý¾Ý¼Ä´æÆ÷

#define MPU_I2CMST_DELAY_REG	0X67	//IICÖ÷»úÑÓÊ±¹ÜÀí¼Ä´æÆ÷
#define MPU_SIGPATH_RST_REG		0X68	//ÐÅºÅÍ¨µÀ¸´Î»¼Ä´æÆ÷
#define MPU_MDETECT_CTRL_REG	0X69	//ÔË¶¯¼ì²â¿ØÖÆ¼Ä´æÆ÷
#define MPU_USER_CTRL_REG		0X6A	//ÓÃ»§¿ØÖÆ¼Ä´æÆ÷
#define MPU_PWR_MGMT1_REG		0X6B	//µçÔ´¹ÜÀí¼Ä´æÆ÷1
#define MPU_PWR_MGMT2_REG		0X6C	//µçÔ´¹ÜÀí¼Ä´æÆ÷2 
#define MPU_FIFO_CNTH_REG		0X72	//FIFO¼ÆÊý¼Ä´æÆ÷¸ß°ËÎ»
#define MPU_FIFO_CNTL_REG		0X73	//FIFO¼ÆÊý¼Ä´æÆ÷µÍ°ËÎ»
#define MPU_FIFO_RW_REG			0X74	//FIFO¶ÁÐ´¼Ä´æÆ÷
#define MPU_DEVICE_ID_REG		0X75	//Æ÷¼þID¼Ä´æÆ÷




void delay_ms(int m)
{
  int i;
  
  for(; m != 0; m--)	
       for (i=0; i<5000; i++);
}


//��ʼ��MPU9250
//����ֵ:0,�ɹ�
//    ����,�������
int16_t MPU9250_Init(void)
{
	int16_t res=0;
	IIC_Init();     //��ʼ��IIC����
	MPU_Write_Byte(MPU9250_ADDR,MPU_PWR_MGMT1_REG,0X80);//��λMPU9250
	delay_ms(1000);  //��ʱ100ms
	MPU_Write_Byte(MPU9250_ADDR,MPU_PWR_MGMT1_REG,0X00);//����MPU9250
	MPU_Set_Gyro_Fsr(3);					        	//�����Ǵ�����,��2000dps
	MPU_Set_Accel_Fsr(2);					       	 	//���ٶȴ�����,��2g
	MPU_Set_Rate(1000);						       	 	//���ò�����50Hz
	MPU_Write_Byte(MPU9250_ADDR,MPU_INT_EN_REG,0X00);   //�ر������ж�
	MPU_Write_Byte(MPU9250_ADDR,MPU_USER_CTRL_REG,0X00);//I2C��ģʽ�ر�
	MPU_Write_Byte(MPU9250_ADDR,MPU_FIFO_EN_REG,0X00);	//�ر�FIFO
	MPU_Write_Byte(MPU9250_ADDR,MPU_INTBP_CFG_REG,0X82);//INT���ŵ͵�ƽ��Ч������bypassģʽ������ֱ�Ӷ�ȡ������

	res=MPU_Read_Byte(MPU9250_ADDR,MPU_DEVICE_ID_REG);  //��ȡMPU6500��ID
	if(res==MPU6500_ID) //����ID��ȷ
	{
			MPU_Write_Byte(MPU9250_ADDR,MPU_PWR_MGMT1_REG,0X01);  	//����CLKSEL,PLL X��Ϊ�ο�
			MPU_Write_Byte(MPU9250_ADDR,MPU_PWR_MGMT2_REG,0X00);  	//���ٶ��������Ƕ�����
			MPU_Set_Rate(1000);						       	//���ò�����Ϊ50Hz   
	} 

	res=MPU_Read_Byte(AK8963_ADDR,MAG_WIA);    			//��ȡAK8963 ID   
	if(res==AK8963_ID)
	{
			MPU_Write_Byte(AK8963_ADDR, MAG_CNTL1, 0X11);		//����AK8963Ϊ���β���ģʽ
	} 

	mpu_offset_call();

	return 0;
}

//����MPU9250�����Ǵ����������̷�Χ
//fsr:0,��250dps;1,��500dps;2,��1000dps;3,��2000dps
//����ֵ:0,���óɹ�
//    ����,����ʧ�� 
int16_t MPU_Set_Gyro_Fsr(int16_t fsr)
{
	return MPU_Write_Byte(MPU9250_ADDR,MPU_GYRO_CFG_REG,fsr<<3);//���������������̷�Χ  
}

//����MPU9250���ٶȴ����������̷�Χ
//fsr:0,��2g;1,��4g;2,��8g;3,��16g
//����ֵ:0,���óɹ�
//    ����,����ʧ�� 
int16_t MPU_Set_Accel_Fsr(int16_t fsr)
{
	return MPU_Write_Byte(MPU9250_ADDR,MPU_ACCEL_CFG_REG,fsr<<3);//���ü��ٶȴ����������̷�Χ  
}

//����MPU9250�����ֵ�ͨ�˲���
//lpf:���ֵ�ͨ�˲�Ƶ��(Hz)
//����ֵ:0,���óɹ�
//    ����,����ʧ�� 
int16_t MPU_Set_LPF(int16_t lpf)
{
	int16_t data=0;
	if(lpf>=188)data=1;
	else if(lpf>=98)data=2;
	else if(lpf>=42)data=3;
	else if(lpf>=20)data=4;
	else if(lpf>=10)data=5;
	else data=6; 
	return MPU_Write_Byte(MPU9250_ADDR,MPU_CFG_REG,data);//�������ֵ�ͨ�˲���  
}

//����MPU9250�Ĳ�����(�ٶ�Fs=1KHz)
//rate:4~1000(Hz)
//����ֵ:0,���óɹ�
//    ����,����ʧ�� 
int16_t MPU_Set_Rate(int16_t rate)
{
	int16_t data;
	if(rate>1000)rate=1000;
	if(rate<4)rate=4;
	data=1000/rate-1;
	data=MPU_Write_Byte(MPU9250_ADDR,MPU_SAMPLE_RATE_REG,data);	//�������ֵ�ͨ�˲���
 	return MPU_Set_LPF(rate/2);	//�Զ�����LPFΪ�����ʵ�һ��
}

//�õ��¶�ֵ
//����ֵ:�¶�ֵ(������100��)
short MPU_Get_Temperature(void)
{
    int16_t buf[2]; 
    short raw;
	float temp;
	MPU_Read_Len(MPU9250_ADDR,MPU_TEMP_OUTH_REG,2,buf); 
    raw=((int16_t)buf[0]<<8)|buf[1];  
    temp=21+((double)raw)/333.87;  
    return temp*100;;
}
//�õ�������ֵ(ԭʼֵ)
//gx,gy,gz:������x,y,z���ԭʼ����(������)
//����ֵ:0,�ɹ�
//    ����,�������
int16_t MPU_Get_Gyroscope(short *gx,short *gy,short *gz)
{
	int16_t buf[6],res; 
	res=MPU_Read_Len(MPU9250_ADDR,MPU_GYRO_XOUTH_REG,6,buf);
	if(res==0)
	{
		*gx=((int16_t)buf[0]<<8)|buf[1];  
		*gy=((int16_t)buf[2]<<8)|buf[3];  
		*gz=((int16_t)buf[4]<<8)|buf[5];
	} 	
	return res;;
}
//�õ����ٶ�ֵ(ԭʼֵ)
//gx,gy,gz:������x,y,z���ԭʼ����(������)
//����ֵ:0,�ɹ�
//    ����,�������
int16_t MPU_Get_Accelerometer(short *ax,short *ay,short *az)
{
	int16_t buf[6],res;  
	res=MPU_Read_Len(MPU9250_ADDR,MPU_ACCEL_XOUTH_REG,6,buf);
	if(res==0)
	{
		*ax=((int16_t)buf[0]<<8)|buf[1];  
		*ay=((int16_t)buf[2]<<8)|buf[3];  
		*az=((int16_t)buf[4]<<8)|buf[5];
	} 	
	return res;;
}

//�õ�������ֵ(ԭʼֵ)
//mx,my,mz:������x,y,z���ԭʼ����(������)
//����ֵ:0,�ɹ�
//    ����,�������
int16_t MPU_Get_Magnetometer(short *mx,short *my,short *mz)
{
	int16_t buf[6],res; 
	delay_ms(15);	
	res=MPU_Read_Len(AK8963_ADDR,MAG_XOUT_L,6,buf);
	if(res==0)
	{
		*mx=((int16_t)buf[1]<<8)|buf[0];  
		*my=((int16_t)buf[3]<<8)|buf[2];  
		*mz=((int16_t)buf[5]<<8)|buf[4];
	} 	
	MPU_Write_Byte(AK8963_ADDR,MAG_CNTL1,0X11); //AK8963ÿ�ζ����Ժ���Ҫ��������Ϊ���β���ģʽ
	return res;;
}

//IIC����д
//addr:������ַ 
//reg:�Ĵ�����ַ
//len:д�볤��
//buf:������
//����ֵ:0,����
//    ����,�������
int16_t MPU_Write_Len(int16_t addr,int16_t reg,int16_t len,int16_t *buf)
{
    int16_t i;
    IIC_Start();
    IIC_Send_Byte((addr<<1)|0); //����������ַ+д����
    if(IIC_Wait_Ack())          //�ȴ�Ӧ��
    {
        IIC_Stop();
        return 1;
    }
    IIC_Send_Byte(reg);         //д�Ĵ�����ַ
    IIC_Wait_Ack();             //�ȴ�Ӧ��
    for(i=0;i<len;i++)
    {
        IIC_Send_Byte(buf[i]);  //��������
        if(IIC_Wait_Ack())      //�ȴ�ACK
        {
            IIC_Stop();
            return 1;
        }
    }
    IIC_Stop();
    return 0;
} 

//IIC������
//addr:������ַ
//reg:Ҫ��ȡ�ļĴ�����ַ
//len:Ҫ��ȡ�ĳ���
//buf:��ȡ�������ݴ洢��
//����ֵ:0,����
//    ����,�������
int16_t MPU_Read_Len(int16_t addr,int16_t reg,int16_t len,int16_t *buf)
{ 
    IIC_Start();
    IIC_Send_Byte((addr<<1)|0); //����������ַ+д����
    if(IIC_Wait_Ack())          //�ȴ�Ӧ��
    {
        IIC_Stop();
        return 1;
    }
    IIC_Send_Byte(reg);         //д�Ĵ�����ַ
    IIC_Wait_Ack();             //�ȴ�Ӧ��
	  IIC_Start();                
    IIC_Send_Byte((addr<<1)|1); //����������ַ+������
    IIC_Wait_Ack();             //�ȴ�Ӧ��
    while(len)
    {
        if(len==1)*buf=IIC_Read_Byte(0);//������,����nACK 
				else *buf=IIC_Read_Byte(1);		//������,����ACK  
				len--;
				buf++;  
    }
    IIC_Stop();                 //����һ��ֹͣ����
    return 0;       
}

//IICдһ���ֽ� 
//devaddr:����IIC��ַ
//reg:�Ĵ�����ַ
//data:����
//����ֵ:0,����
//    ����,�������
int16_t MPU_Write_Byte(int16_t addr,int16_t reg,int16_t data)
{
    IIC_Start();
    IIC_Send_Byte((addr<<1)|0); //����������ַ+д����
    if(IIC_Wait_Ack())          //�ȴ�Ӧ��
    {
        IIC_Stop();
        return 1;
    }
    IIC_Send_Byte(reg);         //д�Ĵ�����ַ
    IIC_Wait_Ack();             //�ȴ�Ӧ��
    IIC_Send_Byte(data);        //��������
    if(IIC_Wait_Ack())          //�ȴ�ACK
    {
        IIC_Stop();
        return 1;
    }
    IIC_Stop();
    return 0;
}

//IIC��һ���ֽ� 
//reg:�Ĵ�����ַ 
//����ֵ:����������
int16_t MPU_Read_Byte(int16_t addr,int16_t reg)
{
    int16_t res;
    IIC_Start();
    IIC_Send_Byte((addr<<1)|0); //����������ַ+д����
    IIC_Wait_Ack();             //�ȴ�Ӧ��
    IIC_Send_Byte(reg);         //д�Ĵ�����ַ
    IIC_Wait_Ack();             //�ȴ�Ӧ��
	  IIC_Start();                
    IIC_Send_Byte((addr<<1)|1); //����������ַ+������
    IIC_Wait_Ack();             //�ȴ�Ӧ��
    res=IIC_Read_Byte(0);		    //������,����nACK  
    IIC_Stop();                 //����һ��ֹͣ����
    return res;  
}

