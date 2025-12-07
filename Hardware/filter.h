/* filter.h —— 姿态滤波与解算接口 */

#ifndef __FILTER_H
#define __FILTER_H

#include "stm32f10x.h"

/**
 * @brief  姿态滤波器初始化
 * @param  sampleFreqHz 采样频率（单位 Hz），例如 100.0f
 */
void Filter_Init(float sampleFreqHz);

/**
 * @brief  姿态滤波更新（传入一次 IMU 采样）
 * @param  gx_dps,gy_dps,gz_dps  陀螺仪角速度，单位：deg/s
 * @param  ax_g,ay_g,az_g        加速度，单位：g（可近似，算法内部会归一化）
 */
void Filter_Update(float gx_dps, float gy_dps, float gz_dps,
                   float ax_g,  float ay_g,  float az_g);

/**
 * @brief  获取当前姿态欧拉角
 * @param  pitch_deg 俯仰角（绕Y轴，前仰为正）
 * @param  roll_deg  横滚角（绕X轴，右滚为正）
 * @param  yaw_deg   航向角（绕Z轴，逆时针为正）
 */
void Filter_GetEuler(float *pitch_deg, float *roll_deg, float *yaw_deg);

#endif
