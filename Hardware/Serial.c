#include "Serial.h"
#include <stdio.h>
#include <stdlib.h>

/**
 * @brief  把浮点（度）转成 "xx.x" 形式并打包发送：
 *         格式：@IMU,P=pp.p,R=rr.r,Y=yy.y\r\n
 *         注意：这里内部只用 %d，不用 %f，避免浮点 printf 的坑
 */
void Serial_SendIMUPacket(float pitch_deg, float roll_deg, float yaw_deg)
{
    char buf[64];

    /* 放大 10 倍转整数，顺便四舍五入 */
    int32_t p10 = (int32_t)(pitch_deg * 10.0f);
    int32_t r10 = (int32_t)(roll_deg  * 10.0f);
    int32_t y10 = (int32_t)(yaw_deg   * 10.0f);

    /* 分离整数部分和小数一位（带符号） */
    int32_t p_int = p10 / 10;
    int32_t r_int = r10 / 10;
    int32_t y_int = y10 / 10;

    int32_t p_frac = abs(p10 % 10);
    int32_t r_frac = abs(r10 % 10);
    int32_t y_frac = abs(y10 % 10);

    int len = sprintf(buf,
                      "@IMU,P=%ld.%01ld,R=%ld.%01ld,Y=%ld.%01ld\r\n",
                      (long)p_int, (long)p_frac,
                      (long)r_int, (long)r_frac,
                      (long)y_int, (long)y_frac);

    Serial_SendArray((uint8_t *)buf, (uint16_t)len);
}


/**
 * @brief  USART1 初始化：PA9 TX, PA10 RX
 */
void Serial_Init(uint32_t baud)
{
    /* 开启时钟 */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

    /* GPIO 配置 */
    GPIO_InitTypeDef gpio;
    /* TX: PA9 复用推挽输出 */
    gpio.GPIO_Mode  = GPIO_Mode_AF_PP;
    gpio.GPIO_Pin   = GPIO_Pin_9;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &gpio);

    /* RX: PA10 上拉输入（目前 B 端只发不收，但先配置好） */
    gpio.GPIO_Mode  = GPIO_Mode_IPU;
    gpio.GPIO_Pin   = GPIO_Pin_10;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &gpio);

    /* USART 参数 */
    USART_InitTypeDef us;
    us.USART_BaudRate            = baud;
    us.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    us.USART_Mode                = USART_Mode_Tx | USART_Mode_Rx;
    us.USART_Parity              = USART_Parity_No;
    us.USART_StopBits            = USART_StopBits_1;
    us.USART_WordLength          = USART_WordLength_8b;
    USART_Init(USART1, &us);

    USART_Cmd(USART1, ENABLE);
}

/**
 * @brief  发送 1 字节（阻塞）
 */
void Serial_SendByte(uint8_t Byte)
{
    USART_SendData(USART1, Byte);
    while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
}

/**
 * @brief  发送数组
 */
void Serial_SendArray(uint8_t *Array, uint16_t Length)
{
    for (uint16_t i = 0; i < Length; i++)
    {
        Serial_SendByte(Array[i]);
    }
}

/**
 * @brief  发送 C 字符串（以 '\0' 结束）
 */
void Serial_SendString(uint8_t *String)
{
    while (*String)
    {
        Serial_SendByte(*String++);
    }
}

/**
 * @brief  printf 重定向到 USART1（可选）
 */
int fputc(int ch, FILE *f)
{
    Serial_SendByte((uint8_t)ch);
    return ch;
}

/**
 * @brief  发送心跳包 "@H\r\n"
 */
void Serial_SendHeartbeat(void)
{
    Serial_SendString((uint8_t*)"@H\r\n");
}
