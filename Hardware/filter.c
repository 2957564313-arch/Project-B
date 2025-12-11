/* filter.c —— 限幅 + 一阶低通 + Mahony IMU 姿态解算（MCU-B） */

#include "filter.h"
#include <math.h>

/* ====================== 可调参数区域 BEGIN ====================== */

/* 默认采样频率（Hz），和你 main 里 10ms 对应 100Hz 保持一致 */
#define FILTER_SAMPLE_FREQ_DEFAULT   100.0f

/* Mahony 增益：Kp 越大，静止后恢复越快；Ki 先保持很小只做长期校正 */
#define MAHONY_KP    1.2f       /* 原来 4.0f 太大了，容易“发力过猛+奇怪动态” */
#define MAHONY_KI    0.01f      /* 给一点点积分，纠长期偏差，不至于明显影响动态 */

/* 陀螺仪 / 加速度 限幅 */
#define GYRO_LIMIT_DPS    500.0f
#define ACC_LIMIT_G       4.0f

/* 一阶低通滤波系数（0~1，越大越“跟手”） */
#define GYRO_LPF_ALPHA    1.0f     /* 陀螺不过滤，保持你现在的动态 */
#define ACC_LPF_ALPHA     0.40f    /* 从 0.25f 提到 0.40f，让静止收敛更快一点 */

/* 陀螺仪死区（抑制极慢漂移） */
#define GYRO_DEADZONE_DPS  0.05f   /* 从 0.1f 降一点，避免过早砍小角速度 */


/* ====================== 可调参数区域 END ====================== */


/* ------------------------- Mahony 参数与状态 ------------------------- */

/* 采样频率（Hz），由 Filter_Init 设置 */
static float s_sampleFreq = FILTER_SAMPLE_FREQ_DEFAULT;

/* Mahony 增益（内部用 twoKp=2*Kp, twoKi=2*Ki） */
static volatile float twoKp = 2.0f * MAHONY_KP;
static volatile float twoKi = 2.0f * MAHONY_KI;

/* 四元数（当前姿态） */
static volatile float q0 = 1.0f;
static volatile float q1 = 0.0f;
static volatile float q2 = 0.0f;
static volatile float q3 = 0.0f;

/* Mahony 误差积分项（如果启用 Ki） */
static volatile float integralFBx = 0.0f;
static volatile float integralFBy = 0.0f;
static volatile float integralFBz = 0.0f;

/* ------------------------- 输入限幅 + 一阶低通 ------------------------- */

/* 一阶低通滤波系数（运行时可通过 Filter_Init 调整为上面宏值） */
static float s_alphaGyro = GYRO_LPF_ALPHA;
static float s_alphaAcc  = ACC_LPF_ALPHA;

/* 低通滤波后的值 */
static float s_gx_f = 0.0f;
static float s_gy_f = 0.0f;
static float s_gz_f = 0.0f;
static float s_ax_f = 0.0f;
static float s_ay_f = 0.0f;
static float s_az_f = 1.0f;   /* 默认认为刚开始静止，重力朝 +Z */

/* 是否已经完成第一次滤波初始化 */
static uint8_t s_filterInited = 0;
static float s_kpScale = 1.0f;   /* 静止时会 >1.0，加快收敛 */



/* --------------------------- 内部工具函数 --------------------------- */

/**
 * @brief  快速平方根倒数 1/sqrt(x)
 */
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
 * @brief  Mahony IMU 模式更新（不使用磁力计）
 * @param  gx,gy,gz  角速度（单位：rad/s）
 * @param  ax,ay,az  加速度（任意单位，内部会归一化）
 */
static void MahonyAHRSupdateIMU(float gx, float gy, float gz,
                                float ax, float ay, float az)
{
    float recipNorm;
    float halfvx, halfvy, halfvz;
    float halfex, halfey, halfez;
    float qa, qb, qc;

    /* 加速度全 0 就不更新，避免除 0 */
    if ((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))
        return;

    /* 1. 归一化加速度测量量 */
    recipNorm = invSqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    /* 2. 根据当前四元数估计重力方向 */
    halfvx = q1 * q3 - q0 * q2;
    halfvy = q0 * q1 + q2 * q3;
    halfvz = q0 * q0 - 0.5f + q3 * q3;

    /* 3. 测量重力方向 vs 估计重力方向 的误差（叉乘） */
    halfex = (ay * halfvz - az * halfvy);
    halfey = (az * halfvx - ax * halfvz);
    halfez = (ax * halfvy - ay * halfvx);

    /* 4. 积分误差项（如果启用 Ki） */
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

        /* 5. 比例项修正角速度（根据静止检测放大或缩小 Kp） */
    float twoKp_eff = twoKp * s_kpScale;
    gx += twoKp_eff * halfex;
    gy += twoKp_eff * halfey;
    gz += twoKp_eff * halfez;


    /* 6. 四元数微分方程积分 */
    gx *= 0.5f * (1.0f / s_sampleFreq);
    gy *= 0.5f * (1.0f / s_sampleFreq);
    gz *= 0.5f * (1.0f / s_sampleFreq);

    qa = q0;
    qb = q1;
    qc = q2;

    q0 += (-qb * gx - qc * gy - q3 * gz);
    q1 += ( qa * gx + qc * gz - q3 * gy);
    q2 += ( qa * gy - qb * gz + q3 * gx);
    q3 += ( qa * gz + qb * gy - qc * gx);

    /* 7. 归一化四元数 */
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
    else
    {
        s_sampleFreq = FILTER_SAMPLE_FREQ_DEFAULT;
    }

    /* 重置姿态为单位四元数 */
    q0 = 1.0f;
    q1 = 0.0f;
    q2 = 0.0f;
    q3 = 0.0f;

    integralFBx = 0.0f;
    integralFBy = 0.0f;
    integralFBz = 0.0f;

    /* 低通滤波初值：默认认为一开始静止，重力朝 +Z */
    s_gx_f = 0.0f;
    s_gy_f = 0.0f;
    s_gz_f = 0.0f;
    s_ax_f = 0.0f;
    s_ay_f = 0.0f;
    s_az_f = 1.0f;

    s_filterInited = 0;

    /* 把 Mahony 增益、低通系数同步为宏设定值 */
    twoKp = 2.0f * MAHONY_KP;
    twoKi = 2.0f * MAHONY_KI;
    s_alphaGyro = GYRO_LPF_ALPHA;
    s_alphaAcc  = ACC_LPF_ALPHA;
}

/**
 * @brief  每次 IMU 采样后调用一次，完成：
 *         1. 陀螺仪 & 加速度 限幅 + 死区
 *         2. 一阶低通滤波
 *         3. Mahony 姿态更新
 * @param  gx_dps,gy_dps,gz_dps  陀螺仪角速度，单位：deg/s
 * @param  ax_g,ay_g,az_g        加速度，单位：g
 */
void Filter_Update(float gx_dps, float gy_dps, float gz_dps,
                   float ax_g,  float ay_g,  float az_g)
{
    /* ---------------------- 0. 陀螺仪死区（抑制微弱漂移） ---------------------- */
    if (fabsf(gx_dps) < GYRO_DEADZONE_DPS) gx_dps = 0.0f;
    if (fabsf(gy_dps) < GYRO_DEADZONE_DPS) gy_dps = 0.0f;
    if (fabsf(gz_dps) < GYRO_DEADZONE_DPS) gz_dps = 0.0f;

    /* ---------------------- 1. 限幅（抗突变防炸） ---------------------- */

    /* 陀螺仪限幅 */
    if (gx_dps >  GYRO_LIMIT_DPS) gx_dps =  GYRO_LIMIT_DPS;
    if (gx_dps < -GYRO_LIMIT_DPS) gx_dps = -GYRO_LIMIT_DPS;

    if (gy_dps >  GYRO_LIMIT_DPS) gy_dps =  GYRO_LIMIT_DPS;
    if (gy_dps < -GYRO_LIMIT_DPS) gy_dps = -GYRO_LIMIT_DPS;

    if (gz_dps >  GYRO_LIMIT_DPS) gz_dps =  GYRO_LIMIT_DPS;
    if (gz_dps < -GYRO_LIMIT_DPS) gz_dps = -GYRO_LIMIT_DPS;

    /* 加速度限幅 */
    if (ax_g >  ACC_LIMIT_G) ax_g =  ACC_LIMIT_G;
    if (ax_g < -ACC_LIMIT_G) ax_g = -ACC_LIMIT_G;

    if (ay_g >  ACC_LIMIT_G) ay_g =  ACC_LIMIT_G;
    if (ay_g < -ACC_LIMIT_G) ay_g = -ACC_LIMIT_G;

    if (az_g >  ACC_LIMIT_G) az_g =  ACC_LIMIT_G;
    if (az_g < -ACC_LIMIT_G) az_g = -ACC_LIMIT_G;

    /* ---------------------- 2. 一阶低通滤波 ---------------------- */

    if (!s_filterInited)
    {
        /* 第一次调用，用当前值初始化滤波器，避免起跳 */
        s_gx_f = gx_dps;
        s_gy_f = gy_dps;
        s_gz_f = gz_dps;
        s_ax_f = ax_g;
        s_ay_f = ay_g;
        s_az_f = az_g;
        s_filterInited = 1;
    }
    else
    {
        /* y(k) = y(k-1) + alpha * (x(k) - y(k-1)) */
        s_gx_f = s_gx_f + s_alphaGyro * (gx_dps - s_gx_f);
        s_gy_f = s_gy_f + s_alphaGyro * (gy_dps - s_gy_f);
        s_gz_f = s_gz_f + s_alphaGyro * (gz_dps - s_gz_f);

        s_ax_f = s_ax_f + s_alphaAcc * (ax_g - s_ax_f);
        s_ay_f = s_ay_f + s_alphaAcc * (ay_g - s_ay_f);
        s_az_f = s_az_f + s_alphaAcc * (az_g - s_az_f);
    }

	    /* ---------------- 2.5 静止检测：静止时加大 Kp，加快收敛 ---------------- */
    {
        /* 加速度模长，理论上静止时 ≈1g */
        float acc_norm = sqrtf(s_ax_f * s_ax_f +
                               s_ay_f * s_ay_f +
                               s_az_f * s_az_f);

        /* 陀螺总角速度（简单用 L1 范数） */
        float gyro_sum = fabsf(gx_dps) + fabsf(gy_dps) + fabsf(gz_dps);

        /* 判定阈值你可以按需要微调 */
        uint8_t is_static =
            (fabsf(acc_norm - 1.0f) < 0.10f) &&   /* 加速度模长离 1g 不超过 0.10g */
            (gyro_sum < 3.0f);                    /* 三个轴角速度和 < 3deg/s */

        if (is_static)
        {
            /* 静止：把 Kp 放大 3 倍，加快拉回真实姿态 */
            s_kpScale = 3.0f;
        }
        else
        {
            /* 运动中：用正常的 Kp */
            s_kpScale = 1.0f;
        }
    }

    /* ---------------------- 3. Mahony 姿态更新 ---------------------- */

    /* deg/s -> rad/s */
    const float DEG_TO_RAD = 0.017453292519943295f;  /* pi/180 */

    float gx = s_gx_f * DEG_TO_RAD;
    float gy = s_gy_f * DEG_TO_RAD;
    float gz = s_gz_f * DEG_TO_RAD;

    /* 加速度用滤波后的值 */
    MahonyAHRSupdateIMU(gx, gy, gz, s_ax_f, s_ay_f, s_az_f);
}

/**
 * @brief  四元数 -> 欧拉角（单位：度）
 * @param  pitch_deg 俯仰角
 * @param  roll_deg  横滚角
 * @param  yaw_deg   航向角
 */
void Filter_GetEuler(float *pitch_deg, float *roll_deg, float *yaw_deg)
{
    float roll, pitch, yaw;

    /* roll (X 轴旋转) */
    float sinr_cosp = 2.0f * (q0 * q1 + q2 * q3);
    float cosr_cosp = 1.0f - 2.0f * (q1 * q1 + q2 * q2);
    roll = atan2f(sinr_cosp, cosr_cosp);

    /* pitch (Y 轴旋转) */
    float sinp = 2.0f * (q0 * q2 - q3 * q1);
    float half_pi = 1.57079632679f;  /* ≈ π/2 */

    if (fabsf(sinp) >= 1.0f)
    {
        /* 超出范围时，直接钳制为 ±90° */
        pitch = (sinp > 0.0f) ? half_pi : -half_pi;
    }
    else
    {
        pitch = asinf(sinp);
    }

    /* yaw (Z 轴旋转) */
    float siny_cosp = 2.0f * (q0 * q3 + q1 * q2);
    float cosy_cosp = 1.0f - 2.0f * (q2 * q2 + q3 * q3);
    yaw = atan2f(siny_cosp, cosy_cosp);

    const float RAD_TO_DEG = 57.29577951308232f;  /* 180/pi */

    if (pitch_deg) *pitch_deg = pitch * RAD_TO_DEG;
    if (roll_deg)  *roll_deg  = roll  * RAD_TO_DEG;
    if (yaw_deg)   *yaw_deg   = yaw   * RAD_TO_DEG;
}
