/* filter.c —— Mahony IMU 姿态解算 + 欧拉角输出 */

#include "filter.h"
#include <math.h>

/* ------------------------- Mahony 参数与状态 ------------------------- */

/* 采样频率（Hz），由 Filter_Init 设置 */
static float s_sampleFreq = 100.0f;

/* Mahony 增益参数：twoKp=2*Kp, twoKi=2*Ki */
static volatile float twoKp = 2.0f * 0.5f;  /* Kp = 0.5 */
static volatile float twoKi = 2.0f * 0.0f;  /* Ki = 0.0，先不做积分 */

/* 四元数（姿态） */
static volatile float q0 = 1.0f;
static volatile float q1 = 0.0f;
static volatile float q2 = 0.0f;
static volatile float q3 = 0.0f;

/* 积分误差项 */
static volatile float integralFBx = 0.0f;
static volatile float integralFBy = 0.0f;
static volatile float integralFBz = 0.0f;

/* --------------------------- 内部工具函数 --------------------------- */

/* 快速平方根倒数 */
static float invSqrt(float x)
{
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long*)&y;
    i = 0x5f3759df - (i >> 1);
    y = *(float*)&i;
    y = y * (1.5f - (halfx * y * y));
    return y;
}

/**
 * @brief Mahony IMU 模式更新（不使用磁力计）
 * @param gx,gy,gz  角速度（单位：rad/s）
 * @param ax,ay,az  加速度（单位：任意，内部会归一化）
 */
static void MahonyAHRSupdateIMU(float gx, float gy, float gz,
                                float ax, float ay, float az)
{
    float recipNorm;
    float halfvx, halfvy, halfvz;
    float halfex, halfey, halfez;
    float qa, qb, qc;

    /* 加速度全 0 就不更新，避免 NaN */
    if ((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))
        return;

    /* 归一化加速度 */
    recipNorm = invSqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    /* 估计的重力方向 */
    halfvx = q1 * q3 - q0 * q2;
    halfvy = q0 * q1 + q2 * q3;
    halfvz = q0 * q0 - 0.5f + q3 * q3;

    /* 测量的重力方向 与 估计的重力方向 之间的误差（叉乘） */
    halfex = (ay * halfvz - az * halfvy);
    halfey = (az * halfvx - ax * halfvz);
    halfez = (ax * halfvy - ay * halfvx);

    /* 积分项（如果启用 Ki） */
    if (twoKi > 0.0f)
    {
        float dt = 1.0f / s_sampleFreq;
        integralFBx += twoKi * halfex * dt;
        integralFBy += twoKi * halfey * dt;
        integralFBz += twoKi * halfez * dt;

        gx += integralFBx;
        gy += integralFBy;
        gz += integralFBz;
    }
    else
    {
        integralFBx = 0.0f;
        integralFBy = 0.0f;
        integralFBz = 0.0f;
    }

    /* 比例项修正角速度 */
    gx += twoKp * halfex;
    gy += twoKp * halfey;
    gz += twoKp * halfez;

    /* 四元数微分方程积分 */
    gx *= 0.5f * (1.0f / s_sampleFreq);
    gy *= 0.5f * (1.0f / s_sampleFreq);
    gz *= 0.5f * (1.0f / s_sampleFreq);

    qa = q0;
    qb = q1;
    qc = q2;

    q0 += (-qb * gx - qc * gy - q3 * gz);
    q1 += (qa * gx + qc * gz - q3 * gy);
    q2 += (qa * gy - qb * gz + q3 * gx);
    q3 += (qa * gz + qb * gy - qc * gx);

    /* 归一化四元数 */
    recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;
}

/* --------------------------- 对外接口实现 --------------------------- */

void Filter_Init(float sampleFreqHz)
{
    if (sampleFreqHz > 0.0f)
    {
        s_sampleFreq = sampleFreqHz;
    }

    /* 重置四元数为单位姿态 */
    q0 = 1.0f;
    q1 = 0.0f;
    q2 = 0.0f;
    q3 = 0.0f;

    integralFBx = 0.0f;
    integralFBy = 0.0f;
    integralFBz = 0.0f;

    /* 你后续可以在这里根据需要调整 Kp/Ki */
    twoKp = 2.0f * 0.5f;
    twoKi = 2.0f * 0.0f;
}

/**
 * @brief  每次 IMU 采样后调用一次，更新姿态
 */
void Filter_Update(float gx_dps, float gy_dps, float gz_dps,
                   float ax_g,  float ay_g,  float az_g)
{
    /* 将 deg/s 转换为 rad/s（Mahony 算法中使用的是 rad/s） */
    const float DEG_TO_RAD = 0.017453292519943295f;  /* pi/180 */

    float gx = gx_dps * DEG_TO_RAD;
    float gy = gy_dps * DEG_TO_RAD;
    float gz = gz_dps * DEG_TO_RAD;

    /* ax,ay,az 单位可以是 g，算法内部会做归一化 */
    MahonyAHRSupdateIMU(gx, gy, gz, ax_g, ay_g, az_g);
}

/**
 * @brief  将当前四元数转换为欧拉角（单位：度）
 */
void Filter_GetEuler(float *pitch_deg, float *roll_deg, float *yaw_deg)
{
    float roll, pitch, yaw;

    /* 欧拉角转换（右手坐标系） */
    float sinr_cosp = 2.0f * (q0 * q1 + q2 * q3);
    float cosr_cosp = 1.0f - 2.0f * (q1 * q1 + q2 * q2);
    roll = atan2f(sinr_cosp, cosr_cosp);      /* X 轴 */

    float sinp = 2.0f * (q0 * q2 - q3 * q1);
    if (fabsf(sinp) >= 1.0f)
        pitch = copysignf((float)M_PI / 2.0f, sinp);  /* 超范围时取 90° */
    else
        pitch = asinf(sinp);                           /* Y 轴 */

    float siny_cosp = 2.0f * (q0 * q3 + q1 * q2);
    float cosy_cosp = 1.0f - 2.0f * (q2 * q2 + q3 * q3);
    yaw = atan2f(siny_cosp, cosy_cosp);        /* Z 轴 */

    const float RAD_TO_DEG = 57.29577951308232f;  /* 180/pi */

    if (pitch_deg) *pitch_deg = pitch * RAD_TO_DEG;
    if (roll_deg)  *roll_deg  = roll  * RAD_TO_DEG;
    if (yaw_deg)   *yaw_deg   = yaw   * RAD_TO_DEG;
}
