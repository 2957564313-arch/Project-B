#include "stm32f10x.h"
#include "MyI2C.h"
#include "MPU6050_Reg.h"
#include "MPU6050.h"

#define MPU6050_ADDRESS  0xD0   /* 8位地址：0x68<<1 */

void MPU6050_WriteReg(uint8_t RegAddress, uint8_t Data)
{
    MyI2C_Start();
    MyI2C_SendByte(MPU6050_ADDRESS);      /* 写地址 */
    MyI2C_ReceiveAck();
    MyI2C_SendByte(RegAddress);          /* 寄存器地址 */
    MyI2C_ReceiveAck();
    MyI2C_SendByte(Data);                /* 数据 */
    MyI2C_ReceiveAck();
    MyI2C_Stop();
}

uint8_t MPU6050_ReadReg(uint8_t RegAddress)
{
    uint8_t Data;

    MyI2C_Start();
    MyI2C_SendByte(MPU6050_ADDRESS);     /* 先写寄存器地址 */
    MyI2C_ReceiveAck();
    MyI2C_SendByte(RegAddress);
    MyI2C_ReceiveAck();

    MyI2C_Start();                        /* 重新起始，切换为读 */
    MyI2C_SendByte(MPU6050_ADDRESS | 0x01);
    MyI2C_ReceiveAck();
    Data = MyI2C_ReceiveByte();
    MyI2C_SendAck(1);                    /* NACK，结束读 */
    MyI2C_Stop();

    return Data;
}

void MPU6050_Init(void)
{
    MyI2C_Init();

    /* 解除睡眠，选择陀螺仪时钟 */
    MPU6050_WriteReg(MPU6050_PWR_MGMT_1, 0x01);
    /* 6 轴全部开启，不待机 */
    MPU6050_WriteReg(MPU6050_PWR_MGMT_2, 0x00);
    /* 采样分频：f_sample = GyroOutputRate / (1 + SMPLRT_DIV)
     * 默认 GyroOutputRate=8kHz，分频10 => 8000/(1+9)=800Hz
     * 你后面可以根据需要再调
     */
    MPU6050_WriteReg(MPU6050_SMPLRT_DIV, 0x09);
    /* 配置：低通滤波参数（这里给最大滤波） */
    MPU6050_WriteReg(MPU6050_CONFIG, 0x06);
    /* 陀螺仪量程：±2000dps */
    MPU6050_WriteReg(MPU6050_GYRO_CONFIG, 0x18);
    /* 加速度计量程：±16g */
    MPU6050_WriteReg(MPU6050_ACCEL_CONFIG, 0x18);
}

uint8_t MPU6050_GetID(void)
{
    return MPU6050_ReadReg(MPU6050_WHO_AM_I);
}

/**
 * @brief  连续读取 6 个轴的 12 个字节数据
 */
void MPU6050_GetData(int16_t *AccX, int16_t *AccY, int16_t *AccZ,
                     int16_t *GyroX, int16_t *GyroY, int16_t *GyroZ)
{
    uint8_t H, L;

    H = MPU6050_ReadReg(MPU6050_ACCEL_XOUT_H);
    L = MPU6050_ReadReg(MPU6050_ACCEL_XOUT_L);
    *AccX = (int16_t)((H << 8) | L);

    H = MPU6050_ReadReg(MPU6050_ACCEL_YOUT_H);
    L = MPU6050_ReadReg(MPU6050_ACCEL_YOUT_L);
    *AccY = (int16_t)((H << 8) | L);

    H = MPU6050_ReadReg(MPU6050_ACCEL_ZOUT_H);
    L = MPU6050_ReadReg(MPU6050_ACCEL_ZOUT_L);
    *AccZ = (int16_t)((H << 8) | L);

    H = MPU6050_ReadReg(MPU6050_GYRO_XOUT_H);
    L = MPU6050_ReadReg(MPU6050_GYRO_XOUT_L);
    *GyroX = (int16_t)((H << 8) | L);

    H = MPU6050_ReadReg(MPU6050_GYRO_YOUT_H);
    L = MPU6050_ReadReg(MPU6050_GYRO_YOUT_L);
    *GyroY = (int16_t)((H << 8) | L);

    H = MPU6050_ReadReg(MPU6050_GYRO_ZOUT_H);
    L = MPU6050_ReadReg(MPU6050_GYRO_ZOUT_L);
    *GyroZ = (int16_t)((H << 8) | L);
}
