#include "timebase.h"

static volatile uint32_t s_msTicks = 0;

/**
 * @brief  使用 TIM2 产生 1ms 周期中断
 *         注意：APB1 默认 36MHz，TIM2 时钟为 36MHz（未倍频时）
 *         我们配置为：36MHz / 36 / 1000 = 1kHz => 1ms
 */
void TimeBase_Init(void)
{
    /* 开 TIM2 时钟 */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

    /* 计算分频值：36MHz / 36 = 1MHz (1us一个计数) */
    TIM_TimeBaseInitTypeDef tb;
    tb.TIM_Prescaler         = 36 - 1;           // PSC
    tb.TIM_CounterMode       = TIM_CounterMode_Up;
    tb.TIM_Period            = 1000 - 1;         // ARR -> 1ms
    tb.TIM_ClockDivision     = TIM_CKD_DIV1;
    tb.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM2, &tb);

    /* 允许更新中断 */
    TIM_ClearFlag(TIM2, TIM_FLAG_Update);
    TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);

    /* NVIC 配置 */
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    NVIC_InitTypeDef nvic;
    nvic.NVIC_IRQChannel = TIM2_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 1;
    nvic.NVIC_IRQChannelSubPriority        = 1;
    nvic.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&nvic);

    /* 启动定时器 */
    TIM_Cmd(TIM2, ENABLE);
}

/**
 * @brief  TIM2 中断服务函数：每 1ms 进来一次
 */
void TIM2_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM2, TIM_IT_Update) == SET)
    {
        TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
        s_msTicks++;
    }
}

/**
 * @brief  返回从系统上电以来的毫秒数
 */
uint32_t TimeBase_Millis(void)
{
    return s_msTicks;
}
