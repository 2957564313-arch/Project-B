#include "stm32f10x.h"                  // Device header
#include "Delay.h"

void MyI2C_W_SCL(uint8_t BitValue)	//传1置高电平，传0置低电平 W代表写
{
	GPIO_WriteBit(GPIOB,GPIO_Pin_10,(BitAction)BitValue);
	Delay_us(10);
}

void MyI2C_W_SDA(uint8_t BitValue)	//传1置高电平，传0置低电平
{
	GPIO_WriteBit(GPIOB,GPIO_Pin_11,(BitAction)BitValue);
	Delay_us(10);
}

uint8_t MyI2C_R_SDA(void)	//读SDA
{
	uint8_t BitValue;
	BitValue = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_11);
	Delay_us(10);
	return BitValue;
}

void MyI2C_Init(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode =GPIO_Mode_Out_OD ; //开漏输出，但是可以输入
	GPIO_InitStructure.GPIO_Pin =GPIO_Pin_10|GPIO_Pin_11 ;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB,&GPIO_InitStructure);
	
	GPIO_SetBits(GPIOB,GPIO_Pin_10|GPIO_Pin_11);	//SCL和SDA置高电平，I2C总线处于空闲状态
}

void MyI2C_Start(void)
{
	MyI2C_W_SDA(1); //释放SCL和SDA
	MyI2C_W_SCL(1);
	MyI2C_W_SDA(0); //拉低SCL和SDA
	MyI2C_W_SCL(0);
}

void MyI2C_Stop(void)
{
	MyI2C_W_SDA(0);
	MyI2C_W_SCL(1);
	MyI2C_W_SDA(1);
}

void MyI2C_SendByte(uint8_t Byte)
{
	uint8_t i;
	for(i = 0; i < 8; i ++)	//读取8位数据，第一次读最高位，第二次次高位
	{
		MyI2C_W_SDA(Byte & (0x80 >> 1));	//用按位与的方式，取出数据的某一位或某几位（这里是一位））,Byte是 xxxx xxxx, 0x80是 1000 0000,结果得x000 0000
		MyI2C_W_SCL(1);
		MyI2C_W_SCL(0);
	}
}

uint8_t MyI2C_ReceiveByte(void)
{
	uint8_t i,Byte = 0x00;
	MyI2C_W_SDA(1);
	for(i = 0; i < 8; i ++)
	{
		MyI2C_W_SCL(1);	//高电平读取数据
		if (MyI2C_R_SDA() == 1){Byte |= (0x80 >> i);}
		MyI2C_W_SCL(0); //低电平传入下一位数据
	}
	return Byte;	
}

void MyI2C_SendAck(uint8_t AckBit)
{	
	MyI2C_W_SDA(AckBit);
	MyI2C_W_SCL(1);
	MyI2C_W_SCL(0);

}

uint8_t MyI2C_ReceiveAck(void)
{
	//函数进来时SDA低电平，主机释放SDA
	uint8_t AckBit;
	MyI2C_W_SDA(1);
	MyI2C_W_SCL(1);
	AckBit = MyI2C_R_SDA();
	MyI2C_W_SCL(0);
	return AckBit;	
}


