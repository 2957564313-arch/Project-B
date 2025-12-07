#ifndef __MPU6050_H
#define __MPU6050_H

#include "stm32f10x.h"

/**
 * @brief 向 MPU6050 写寄存器
 */
void MPU6050_WriteReg(uint8_t RegAddress, uint8_t Data);

/**
 * @brief 从 MPU6050 读寄存器
 */
uint8_t MPU6050_ReadReg(uint8_t RegAddress);

/**
 * @brief MPU6050 初始化
 * @note  使用内部 8MHz，陀螺仪 ±2000dps，加速度计 ±16g，滤波参数最大
 */
void MPU6050_Init(void);

/**
 * @brief 读取 MPU6050 六轴原始数据
 */
void MPU6050_GetData(int16_t *AccX, int16_t *AccY, int16_t *AccZ,
                     int16_t *GyroX, int16_t *GyroY, int16_t *GyroZ);

/**
 * @brief 读取 WHO_AM_I 寄存器
 * @return 设备 ID，正常应为 0x68
 */
uint8_t MPU6050_GetID(void);

#endif
