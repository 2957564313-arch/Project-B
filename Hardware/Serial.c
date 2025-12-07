#include "stm32f10x.h"                  // Device header
#include <stdio.h>
#include <stdarg.h>


char Serial_RxPacket[100];
uint8_t Serial_RxFlag;

void Serial_Init(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	
	GPIO_InitTypeDef GPIO_InitStruture;
	GPIO_InitStruture.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStruture.GPIO_Pin =GPIO_Pin_9 ;
	GPIO_InitStruture.GPIO_Speed =GPIO_Speed_50MHz ;
	GPIO_Init(GPIOA,&GPIO_InitStruture);
	
	GPIO_InitStruture.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStruture.GPIO_Pin =GPIO_Pin_10 ;
	GPIO_InitStruture.GPIO_Speed =GPIO_Speed_50MHz ;
	GPIO_Init(GPIOA,&GPIO_InitStruture);
	
	USART_InitTypeDef USART_InitStruture;
	USART_InitStruture.USART_BaudRate =9600 ;
	USART_InitStruture.USART_HardwareFlowControl =USART_HardwareFlowControl_None ;
	USART_InitStruture.USART_Mode =USART_Mode_Tx|USART_Mode_Rx ;
	USART_InitStruture.USART_Parity = USART_Parity_No;
	USART_InitStruture.USART_StopBits = USART_StopBits_1;
	USART_InitStruture.USART_WordLength = USART_WordLength_8b;
	USART_Init(USART1,&USART_InitStruture);
	
	USART_ITConfig(USART1,USART_IT_RXNE,ENABLE);
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	
	NVIC_InitTypeDef NVIC_InitStruture;
	NVIC_InitStruture.NVIC_IRQChannel =USART1_IRQn ;
	NVIC_InitStruture.NVIC_IRQChannelCmd =ENABLE ;
	NVIC_InitStruture.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStruture.NVIC_IRQChannelSubPriority =1 ;
	NVIC_Init(&NVIC_InitStruture);
	
	
	
	
	USART_Cmd(USART1,ENABLE);
	
}



void USART1_IRQHandler(void)
{
	static uint8_t RxState = 0;
	static uint8_t pRxPacket = 0;		//指示接收到哪一个了
	
	if(USART_GetITStatus(USART1,USART_IT_RXNE) == SET )
	{
		uint8_t RxData = USART_ReceiveData(USART1);
		
		if(RxState == 0)
		{
			if(RxData == '@' && Serial_RxFlag == 0)		//防止发太快时来不及处理
			{
				RxState = 1;
				pRxPacket = 0;
			}
		}
		else if(RxState == 1)
		{
			if(RxData == '\r')
			{
				RxState = 2;
			}
			else 
			{
				Serial_RxPacket[pRxPacket] = RxData ;
				pRxPacket++;
			}
		}
		else if(RxState == 2)
		{
			if(RxData == '\n')
			{
				RxState = 0;
				Serial_RxPacket[pRxPacket] = '\0';
				Serial_RxFlag = 1;
				
			}
		}
		USART_ClearITPendingBit(USART1,USART_IT_RXNE);
	}
}

void Serial_SendByte(uint8_t Byte)
{
	USART_SendData(USART1,Byte);	//发送以后还要等待一下标志位
	while(USART_GetFlagStatus(USART1,USART_FLAG_TXE) == RESET);		//这里的标志位无需手动清零,会自动清零
	
}

void Serial_SendArray(uint8_t *Array, uint16_t Length)		//传递数组需要指针
{
	uint16_t i;
	for(i = 0; i<Length; i++)
	{
		Serial_SendByte(Array[i]);
	}
}


void Serial_SendString(uint8_t *String)		//字符串自带结束标志位'\0'，所以不需要再传递长度参数了
{
	uint8_t i;
	for(i = 0; String[i] != '\0'; i++)
	{
		Serial_SendByte(String[i]);
	}
}

//uint32_t Serial_Pow(uint32_t X, uint32_t Y)
//{
//	uint32_t Result = 1;
//	while(Y -- )
//	{
//		Result *= X;
//	}
//	return Result;
//}

//void Serial_SendNumber(uint32_t Number,uint8_t Length)
//{
//	uint8_t i;
//	for(i = 0;i<Length;i++)
//	{
//		Serial_SendByte(Number/Serial_Pow(10,Length - i -1)%10 + '0');
//	}
//}

//int fputc(int ch,FILE *f)		//参数照这样写就好了，不用管那么多
//{
//	Serial_SendByte(ch);
//	return ch;
//}

//void Serial_Printf(char *format, ...)	//简化sprintf，首先头文件stdarg.h，这个函数用于封装，前面接受格式化字符串，后面接收后面的可变参数列表
//{
//	uint8_t String[100];
//	va_list arg;
//	va_start(arg,format);
//	vsprintf(String,format,arg);
//	va_end(arg);
//	Serial_SendString(String);
//}



