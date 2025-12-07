/* main.c  —— MCU-B 主程序 */

#include "stm32f10x.h"
#include "Delay.h"
#include "timebase.h"
#include "Serial.h"
#include "MyI2C.h"
#include "MPU6050.h"
#include "filter.h"

/**
 * @brief  主函数：MCU-B
 * @note   功能：
 *         1. 初始化时基、串口、I2C、MPU6050、姿态滤波器
 *         2. 周期性读取 MPU6050 原始数据
 *         3. 通过 Mahony 算法计算 Pitch/Roll/Yaw
 *         4. 按协议通过 USART1 发送给 MCU-A
 */
int main(void)
{
    /* 1ms 系统时基（TIM2） */
    TimeBase_Init();

    /* SysTick 延时（与你 A 工程一样） */
    /* Delay 模块不需要额外 init */

    /* 串口：USART1，115200，PA9/PA10 */
    Serial_Init(115200);

    /* 软件 I2C + MPU6050 初始化 */
    MyI2C_Init();
    MPU6050_Init();

    /* 可选：读 ID 做个自检 */
    {
        uint8_t id = MPU6050_GetID();
        /* 你也可以串口打印调试：printf("MPU6050 ID=0x%02X\r\n", id); */
        (void)id;
    }

    /* 姿态滤波器初始化：采样频率 100Hz，对应 dt=10ms */
    Filter_Init(100.0f);

    /* 周期控制变量 */
    uint32_t lastImuMs = 0;     /* 上次 IMU 更新时间 */
    uint32_t lastHbMs  = 0;     /* 上次心跳包发送时间 */

    /* MPU6050 原始数据变量 */
    int16_t accX, accY, accZ;
    int16_t gyroX, gyroY, gyroZ;

    /* 姿态角变量（度） */
    float pitch, roll, yaw;

    while (1)
    {
        uint32_t now = TimeBase_Millis();

        /* ================= 1. 100Hz 读取 IMU + 姿态更新 ================= */
        if (now - lastImuMs >= 10U)   /* 10ms 一次 => 100Hz */
        {
            lastImuMs = now;

            /* 读取 MPU6050 原始六轴数据 */
            MPU6050_GetData(&accX, &accY, &accZ,
                            &gyroX, &gyroY, &gyroZ);

            /* 将原始值转换为物理量：
             * Gyro: ±2000dps 配置，对应比例 16.4 LSB/(deg/s)
             * Acc : ±16g 配置，对应比例 2048 LSB/g
             */
            float gx_dps = (float)gyroX / 16.4f;
            float gy_dps = (float)gyroY / 16.4f;
            float gz_dps = (float)gyroZ / 16.4f;

            float ax_g = (float)accX / 2048.0f;
            float ay_g = (float)accY / 2048.0f;
            float az_g = (float)accZ / 2048.0f;

            /* === 姿态滤波 + 解算（内部使用 Mahony IMU） === */
            Filter_Update(gx_dps, gy_dps, gz_dps,
                          ax_g, ay_g, az_g);

            Filter_GetEuler(&pitch, &roll, &yaw);

            /* === 发送 IMU 数据包给 A 机 === */
            Serial_SendIMUPacket(pitch, roll, yaw);
        }

        /* ================= 2. 200ms 发送一次心跳包 ================= */
        if (now - lastHbMs >= 200U)
        {
            lastHbMs = now;
            Serial_SendHeartbeat();   /* 格式：@H\r\n */
        }

        /* 稍微歇一歇 CPU（不影响时基和逻辑） */
        Delay_ms(2);
    }
}
