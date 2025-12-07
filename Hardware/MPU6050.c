#include "stm32f10x.h"                  // Device header
#include "MyI2C.h"
#include "MPU6050_Reg.h"

#define MPU6050_ADDRESS 0xD0 //MPU6050的从机写地址

void MPU6050_WriteReg(uint8_t RegAddress, uint8_t data)
{
	MyI2C_Start();
	MyI2C_SendByte(MPU6050_ADDRESS);	//找到从机
	MyI2C_ReceiveAck();	//应答位，判断从机有没有接受到数据
	MyI2C_SendByte(RegAddress); //发送下一个字节——指定具体寄存器地址
	MyI2C_ReceiveAck();
	MyI2C_SendByte(data);	//再下一个字节——指定要写入指定寄存器的数据（这一步才是关键，前两个是来指定寄存器的）
	MyI2C_ReceiveAck();
	MyI2C_Stop();
}

uint8_t MPU6050_ReadReg(uint8_t RegAddress)
{
	uint8_t Data;
	
	MyI2C_Start();
	MyI2C_SendByte(MPU6050_ADDRESS);
	MyI2C_ReceiveAck();	
	MyI2C_SendByte(RegAddress);
	MyI2C_ReceiveAck();
	
	MyI2C_Start();//转入读的时序，重新指定读写位，必须重新开始
	MyI2C_SendByte(MPU6050_ADDRESS | 0x01 );//读地址
	MyI2C_ReceiveAck();
	Data = MyI2C_ReceiveByte();
	MyI2C_SendAck(1);	//0-给从机应答（读取下一个字节），1-不给
	MyI2C_Stop();
	
	return Data;
}

void MPU6050_Init(void)
{
	MyI2C_Init();
	MPU6050_WriteReg(MPU6050_PWR_MGMT_1, 0x01);	//解除睡眠，选择陀螺仪时钟
	MPU6050_WriteReg(MPU6050_PWR_MGMT_2, 0x00); //6轴均不待机
	MPU6050_WriteReg(MPU6050_SMPLRT_DIV, 0x09); //采样分频为10
	MPU6050_WriteReg(MPU6050_CONFIG, 0x06);		//滤波参数给最大
	MPU6050_WriteReg(MPU6050_GYRO_CONFIG, 0x18);	//陀螺仪加速度计量程给最大
	MPU6050_WriteReg(MPU6050_ACCEL_CONFIG, 0x18);
}

uint8_t MPU6050_GetID(void)
{
	return MPU6050_ReadReg(MPU6050_WHO_AM_I);
}

void MPU6050_GetData(int16_t *AccX, int16_t *AccY, int16_t *AccZ,
						int16_t *GyroX, int16_t *GyroY, int16_t *GyroZ)
{
	uint8_t DataH,DataL;
	
	DataH = MPU6050_ReadReg(MPU6050_ACCEL_XOUT_H); //读取高八位的值
	DataL = MPU6050_ReadReg(MPU6050_ACCEL_XOUT_L); //读取低八位的值
	*AccX = (DataH << 8) | DataL;
	
	DataH = MPU6050_ReadReg(MPU6050_ACCEL_YOUT_H); 
	DataL = MPU6050_ReadReg(MPU6050_ACCEL_YOUT_L);
	*AccY = (DataH << 8) | DataL;
	
	DataH = MPU6050_ReadReg(MPU6050_ACCEL_ZOUT_H); 
	DataL = MPU6050_ReadReg(MPU6050_ACCEL_ZOUT_L);
	*AccZ = (DataH << 8) | DataL;
	
	DataH = MPU6050_ReadReg(MPU6050_GYRO_XOUT_H); 
	DataL = MPU6050_ReadReg(MPU6050_GYRO_XOUT_L);
	*GyroX = (DataH << 8) | DataL;
	
	DataH = MPU6050_ReadReg(MPU6050_GYRO_YOUT_H); 
	DataL = MPU6050_ReadReg(MPU6050_GYRO_YOUT_L);
	*GyroY = (DataH << 8) | DataL;
	
	DataH = MPU6050_ReadReg(MPU6050_GYRO_ZOUT_H); 
	DataL = MPU6050_ReadReg(MPU6050_GYRO_ZOUT_L);
	*GyroZ = (DataH << 8) | DataL;
}



