///******************** (C) COPYRIGHT 2022 ：Rui Peng ********************************
// * Arthur     ： Rui Peng
// * file 	    ： mpuiic.c
// * website    ： https://arclab.hku.hk/
//**********************************************************************************/


#include "mpuiic.h"
#include "mpu9250.h"
#include <stdio.h>



//IO operation	 
#define BITBAND(addr, bitnum) ((addr & 0xF0000000)+0x2000000+((addr &0xFFFFF)<<5)+(bitnum<<2)) 
#define MEM_ADDR(addr)  *((volatile unsigned long  *)(addr)) 
#define BIT_ADDR(addr, bitnum)   MEM_ADDR(BITBAND(addr, bitnum)) 

#define GPIOB_ODR_Addr    (GPIOB_BASE+12) //0x40010C0C 
#define GPIOB_IDR_Addr    (GPIOB_BASE+8)  //0x40010C08 

#define PBout(n)   BIT_ADDR(GPIOB_ODR_Addr,n)  //Êä³ö 
#define PBin(n)    BIT_ADDR(GPIOB_IDR_Addr,n)  //ÊäÈë 

#define IIC_SCL    PBout(12) //SCL
#define IIC_SDA    PBout(13) //SDA	 
#define MPU_READ_SDA   PBin(13)  //ÊäÈëSDA 
//IO operation	 


//��ʼ��IIC
void IIC_Init(void)
{					     
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pins : PB12 PB13 */
	GPIO_InitStruct.Pin  = GPIO_PIN_12|GPIO_PIN_13;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}
//IIC��ʱ����

void MPU_SDA_OUT(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	__HAL_RCC_GPIOB_CLK_ENABLE();

	GPIO_InitStruct.Pin  = GPIO_PIN_13;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

void MPU_SDA_IN(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	__HAL_RCC_GPIOB_CLK_ENABLE();

	GPIO_InitStruct.Pin  = GPIO_PIN_13;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}


static void IIC_Delay_us(int16_t us)
{    
   int16_t i=0;  
   while(us--)
   {
      i=1;  
      while(i--) ;    
   }
}
//����IIC��ʼ�ź�
void IIC_Start(void)
{
	MPU_SDA_OUT();     //sda�����
	IIC_SDA=1;	  	  
	IIC_SCL=1;
	IIC_Delay_us(4);
 	IIC_SDA=0;//START:when CLK is high,DATA change form high to low 
	IIC_Delay_us(4);
	IIC_SCL=0;//ǯסI2C���ߣ�׼�����ͻ�������� 
}	  
//����IICֹͣ�ź�
void IIC_Stop(void)
{
	MPU_SDA_OUT();//sda�����
	IIC_SCL=0;
	IIC_SDA=0;//STOP:when CLK is high DATA change form low to high
 	IIC_Delay_us(4);
	IIC_SCL=1;  
	IIC_SDA=1;//����I2C���߽����ź�
	IIC_Delay_us(4);							   	
}
//�ȴ�Ӧ���źŵ���
//����ֵ��1������Ӧ��ʧ��
//        0������Ӧ��ɹ�
int16_t IIC_Wait_Ack(void)
{
	int16_t ucErrTime=0;
	MPU_SDA_IN();      //SDA����Ϊ����  
	IIC_SDA=1;IIC_Delay_us(1);   
	IIC_SCL=1;IIC_Delay_us(1);	 
	while(MPU_READ_SDA)
	{
		ucErrTime++;
		if(ucErrTime>250)
		{
			IIC_Stop();
			return 1;
		}
	}
	IIC_SCL=0;//ʱ�����0 	   
	return 0;  
} 
//����ACKӦ��
void IIC_Ack(void)
{
	IIC_SCL=0;
	MPU_SDA_OUT();
	IIC_SDA=0;
	IIC_Delay_us(2);
	IIC_SCL=1;
	IIC_Delay_us(2);
	IIC_SCL=0;
}
//������ACKӦ��		    
void IIC_NAck(void)
{
	IIC_SCL=0;
	MPU_SDA_OUT();
	IIC_SDA=1;
	IIC_Delay_us(2);
	IIC_SCL=1;
	IIC_Delay_us(2);
	IIC_SCL=0;
}					 				     
//IIC����һ���ֽ�
//���شӻ�����Ӧ��
//1����Ӧ��
//0����Ӧ��			  
void IIC_Send_Byte(int16_t txd)
{                        
	int16_t t;   
	MPU_SDA_OUT(); 	    
	IIC_SCL=0;//����ʱ�ӿ�ʼ���ݴ���
	for(t=0;t<8;t++)
	{              
		IIC_SDA = (txd&0x80)>>7;
		txd<<=1; 	  
		IIC_Delay_us(2);
		IIC_SCL=1;
		IIC_Delay_us(2); 
		IIC_SCL=0;	
		IIC_Delay_us(2);
	}	 
} 	    
//��1���ֽڣ�ack=1ʱ������ACK��ack=0������nACK   
int16_t IIC_Read_Byte(unsigned char ack)
{
	unsigned char i,receive=0;
	MPU_SDA_IN();//SDA����Ϊ����
	for(i=0;i<8;i++ )
	{
		IIC_SCL=0; 
		IIC_Delay_us(2); 
		IIC_SCL=1;
		receive<<=1;
		if(MPU_READ_SDA)receive++;   
		IIC_Delay_us(1); 
	}					 
	if (!ack)
			IIC_NAck();//����nACK
	else
			IIC_Ack(); //����ACK   
	return receive;
}


////�г�IIC���������дӻ���ַ
//void IIC_Slave_List(void)
//{
//	int16_t i=0,res = 0;
//	for(i=0;i<255;i++)
//	{
//		IIC_Start();
//    IIC_Send_Byte((i<<1)|0);
//		res = IIC_Wait_Ack();          //�ȴ�Ӧ��
//		if(res == 0)
//			printf("IIC_ADDR = %#x\r\n",i);
//		IIC_Stop();
//	}printf("\r\n");
//}
////��ȡIIC������ĳ�����������мĴ���
//void IIC_Slave_Register(int16_t Slave_Addr)
//{
//	int16_t i = 0,res = 0;
//	for(i = 0 ; i<255 ; i++)
//	{
//		res = MPU_Read_Byte(Slave_Addr,i);
//		printf("IIC_%#x_%#x = %#x\r\n",Slave_Addr,i,res);
//	}printf("\r\n\r\n");
//}














