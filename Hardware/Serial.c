/* Serial.c —— MCU-B 串口发送模块 */

#include "Serial.h"

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

/**
 * @brief  发送 IMU 数据包
 * @note   要和 MCU-A 的解析格式严格一致
 */
void Serial_SendIMUPacket(float pitch_deg, float roll_deg, float yaw_deg)
{
    char buf[64];
    int len = sprintf(buf, "@IMU,P=%.2f,R=%.2f,Y=%.2f\r\n",
                      pitch_deg, roll_deg, yaw_deg);
    if (len > 0)
    {
        Serial_SendArray((uint8_t*)buf, (uint16_t)len);
    }
}
