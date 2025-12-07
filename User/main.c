#include "stm32f10x.h"
#include "Delay.h"
#include "timebase.h"
#include "MyI2C.h"
#include "MPU6050.h"
#include "filter.h"
#include "Serial.h"

int main(void)
{
    TimeBase_Init();           /* 1ms 系统时基，用于定时采样与发包 */
    Delay_ms(100);             /* 上电稍微等一下 */

    Serial_Init(115200);       /* 串口发往 A 机 */
    MyI2C_Init();              /* 软件 I2C */
    MPU6050_Init();            /* 配置 MPU6050 寄存器 */

    /* 可选：读 ID 做一次调试 */
    // uint8_t id = MPU6050_GetID();
    // ...（发给 PC 看一下）

    /* 滤波器初始化：100Hz 采样 */
    Filter_Init(100.0f);

    uint32_t lastSampleMs = 0;  /* 上一次采样时间戳（ms） */
    uint32_t lastSendMs   = 0;  /* 上一次发 IMU 包时间戳（ms） */
    uint32_t lastHbMs     = 0;  /* 上一次发心跳时间戳（ms） */

    while (1)
    {
        uint32_t now = TimeBase_Millis();

        /* ---------- 1. 100Hz 读取 MPU6050 + 姿态解算 ---------- */
        if (now - lastSampleMs >= 10U)  /* 10ms -> 100Hz */
        {
            lastSampleMs += 10U;

            int16_t ax_raw, ay_raw, az_raw;
            int16_t gx_raw, gy_raw, gz_raw;

            MPU6050_GetData(&ax_raw, &ay_raw, &az_raw,
                            &gx_raw, &gy_raw, &gz_raw);

            /* 量程与你的寄存器设置保持一致：
             * GYRO_CONFIG = 0x18 -> ±2000 dps -> 16.4 LSB/(deg/s)
             * ACCEL_CONFIG = 0x18 -> ±16 g     -> 2048 LSB/g
             */
            float ax_g  = (float)ax_raw / 2048.0f;
            float ay_g  = (float)ay_raw / 2048.0f;
            float az_g  = (float)az_raw / 2048.0f;

            float gx_dps = (float)gx_raw / 16.4f;
            float gy_dps = (float)gy_raw / 16.4f;
            float gz_dps = (float)gz_raw / 16.4f;

            /* 更新滤波器（限幅 + 低通 + Mahony） */
            Filter_Update(gx_dps, gy_dps, gz_dps,
                          ax_g,   ay_g,   az_g);
        }

        /* ---------- 2. 50Hz 发 IMU 包给 A 机 ---------- */
        if (now - lastSendMs >= 20U)  /* 20ms -> 50Hz，足够流畅 */
        {
            lastSendMs += 20U;

            float pitch, roll, yaw;
            Filter_GetEuler(&pitch, &roll, &yaw);

            Serial_SendIMUPacket(pitch, roll, yaw);
        }

        /* ---------- 3. 5Hz 心跳包（可选：不发也行） ---------- */
        if (now - lastHbMs >= 200U)
        {
            lastHbMs += 200U;
            Serial_SendHeartbeat();  /* 你原来写的 @H\r\n 那个 */
        }

        /* 小 delay，降低空转占用，但不要太大 */
        Delay_ms(1);
    }
}
