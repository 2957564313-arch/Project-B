/* Serial.h —— MCU-B 串口发送模块 */

#ifndef __SERIAL_H
#define __SERIAL_H

#include "stm32f10x.h"


/**
 * @brief  USART1 初始化（PA9 TX, PA10 RX）
 * @param  baud 波特率，如 115200
 */
void Serial_Init(uint32_t baud);

/* 基本发送函数 */
void Serial_SendByte(uint8_t Byte);
void Serial_SendArray(uint8_t *Array, uint16_t Length);
void Serial_SendString(uint8_t *String);

/**
 * @brief  发送心跳包 "@H\r\n"
 */
void Serial_SendHeartbeat(void);

/**
 * @brief  发送 IMU 姿态数据包
 * @param  pitch_deg  俯仰角（度）
 * @param  roll_deg   横滚角（度）
 * @param  yaw_deg    航向角（度）
 * @note   格式固定为：@IMU,P=xx.xx,R=yy.yy,Y=zz.zz\r\n
 *        MCU-A 那边的 parse_packet() 正是按这个格式解析的
 */
void Serial_SendIMUPacket(float pitch_deg, float roll_deg, float yaw_deg);

#endif
