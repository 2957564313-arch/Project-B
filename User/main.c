#include "stm32f10x.h"
#include "Delay.h"
#include "timebase.h"
#include "MyI2C.h"
#include "MPU6050.h"
#include "filter.h"
#include "Serial.h"

/* ----- 陀螺仪零偏（deg/s） ----- */
static float g_gx_bias = 0.0f;
static float g_gy_bias = 0.0f;
static float g_gz_bias = 0.0f;

/**
 * @brief  启动时做一次陀螺仪零偏校准
 * @note   调用时请保证板子静止不动！
 */
static void Gyro_Calibrate(void)
{
    const int N = 150; 
    int32_t sum_gx = 0;
    int32_t sum_gy = 0;
    int32_t sum_gz = 0;

    int16_t ax, ay, az;
    int16_t gx, gy, gz;

    for (int i = 0; i < N; i++)
    {
        MPU6050_GetData(&ax, &ay, &az,
                        &gx, &gy, &gz);

        sum_gx += gx;
        sum_gy += gy;
        sum_gz += gz;

        /* ★ 每隔 10 次发送一个心跳，约 10*5ms = 50ms 发一次 ★ */
        if ((i % 10) == 0)
        {
            Serial_SendHeartbeat();   // @H\r\n
        }

        Delay_ms(5);
    }

    float avg_gx = (float)sum_gx / (float)N;
    float avg_gy = (float)sum_gy / (float)N;
    float avg_gz = (float)sum_gz / (float)N;

    g_gx_bias = avg_gx / 16.4f;
    g_gy_bias = avg_gy / 16.4f;
    g_gz_bias = avg_gz / 16.4f;
}



int main(void)
{
    TimeBase_Init();           
    Delay_ms(100);             

    Serial_Init(115200);       
    MyI2C_Init();              
    MPU6050_Init();            

    /* ★ 上电后静止放好，先做一次陀螺仪零偏校准 ★ */
    Gyro_Calibrate();

    /* 滤波器初始化：100Hz 采样 */
    Filter_Init(100.0f);

    uint32_t lastSampleMs = 0;
    uint32_t lastSendMs   = 0;
    uint32_t lastHbMs     = 0;

    while (1)
    {
        uint32_t now = TimeBase_Millis();

        if (now - lastSampleMs >= 10U)  /* 10ms -> 100Hz */
        {
            lastSampleMs += 10U;

            int16_t ax_raw, ay_raw, az_raw;
            int16_t gx_raw, gy_raw, gz_raw;

            MPU6050_GetData(&ax_raw, &ay_raw, &az_raw,
                            &gx_raw, &gy_raw, &gz_raw);

            float ax_g  = (float)ax_raw / 2048.0f;
            float ay_g  = (float)ay_raw / 2048.0f;
            float az_g  = (float)az_raw / 2048.0f;

            /* 先转成 deg/s */
            float gx_dps = (float)gx_raw / 16.4f;
            float gy_dps = (float)gy_raw / 16.4f;
            float gz_dps = (float)gz_raw / 16.4f;

            /* ★ 再减去启动时测出来的零偏 ★ */
            gx_dps -= g_gx_bias;
            gy_dps -= g_gy_bias;
            gz_dps -= g_gz_bias;

            /* 更新滤波器（限幅 + 低通 + Mahony） */
            Filter_Update(gx_dps, gy_dps, gz_dps,
                          ax_g,   ay_g,   az_g);
        }

        if (now - lastSendMs >= 20U)  /* 50Hz 发 IMU */
        {
            lastSendMs += 20U;

            float pitch, roll, yaw;
            Filter_GetEuler(&pitch, &roll, &yaw);
            Serial_SendIMUPacket(pitch, roll, yaw);
        }

        if (now - lastHbMs >= 200U)
        {
            lastHbMs += 200U;
            Serial_SendHeartbeat();
        }

        Delay_ms(1);
    }
}
