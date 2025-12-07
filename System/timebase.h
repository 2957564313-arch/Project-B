#ifndef __TIMEBASE_H
#define __TIMEBASE_H

#include "stm32f10x.h"

/**
 * @brief 初始化 TIM2 为 1ms 周期中断，作为系统毫秒计数器
 */
void TimeBase_Init(void);

/**
 * @brief 获取从上电以来的毫秒数
 */
uint32_t TimeBase_Millis(void);

#endif
