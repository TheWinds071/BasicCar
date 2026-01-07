/* main.c / IMU.c file
编写者：lisn3188
网址：www.chiplab7.com
作者E-mail：lisn3188@163.com
编译环境：MDK-Lite  Version: 4.23
初版时间: 2012-04-25
测试： 本程序已在第七实验室的mini IMU上完成测试
功能：
姿态解算 IMU
将传感器的输出值进行姿态解算。得到目标载体的俯仰角和横滚角 和航向角
------------------------------------
 */

#include "IMU.h"
#include "usart.h"
#include "inv_imu_driver.h"
//#include "eeprom.h"

#include <math.h>
#include <string.h>
#include <stdint.h>

#include "SEGGER_RTT.h"

/* ========= FPU 友好：全部使用 float + xxxf ========= */
#ifndef PI_F
#define PI_F        3.14159265358979323846f
#endif
#define DEG2RAD_F   (PI_F / 180.0f)
#define RAD2DEG_F   (180.0f / PI_F)

/* XYZ结构体 */
xyz_f_t north, west;

/* 如果这些变量不在中断里异步修改，建议去掉 volatile（可显著减少内存读写） */
volatile float exInt, eyInt, ezInt;  // 误差积分
volatile float q0, q1, q2, q3;       // 全局四元数

volatile float yaw[5] = {0,0,0,0,0};  // 处理航向的增值
int16_t Ax_offset = 0, Ay_offset = 0;

float TTangles_gyro[7]; // 彤彤滤波角度
float Kp = 10.5f;

extern int bsp_IcmGetRawData(float accel_mg[3], float gyro_dps[3], float *temp_degc);
extern int setup_imu(int use_ln, int accel_en, int gyro_en);

/* ========== 更适合 H750 FPU 的 invSqrt ==========
 * 有硬件浮点时：1/sqrtf(x) 通常会直接生成 VSQRT.F32 + VDIV/VMUL
 * 无硬件浮点时：fallback 使用安全写法（memcpy 避免严格别名 UB）
 */
static inline float invSqrt1(float x)
{
#if defined(__ARM_FP) && (__ARM_FP > 0)
    // x 必须 > 0
    return 1.0f / sqrtf(x);
#else
    float halfx = 0.5f * x;
    float y = x;
    uint32_t i;
    memcpy(&i, &y, sizeof(i));
    i = 0x5f3759dfu - (i >> 1);
    memcpy(&y, &i, sizeof(y));
    y = y * (1.5f - (halfx * y * y));
    y = y * (1.5f - (halfx * y * y));
    return y;
#endif
}

/**************************实现函数********************************************
*函数原型:	   void IMU_init(void)
*功　　能:	  初始化IMU相关
*******************************************************************************/
void IMU_init(void)
{
    //while(!ICM_Init());	   //初始化ICM42688配置
    if (0x00 == setup_imu(1,1,1))
    {
        q0 = 1.0f;
        q1 = 0.0f;
        q2 = 0.0f;
        q3 = 0.0f;

        exInt = 0.0f;
        eyInt = 0.0f;
        ezInt = 0.0f;

        HAL_Delay(100);
        return;
    }
    RTT_Log("IMU ERROR!!\r\n");
}

/* ========== 陀螺方差估计：全 float，避免 double 触发软浮点 ========== */
#define GYRO_VAR_WIN   100

static float Gyro_fill[3][GYRO_VAR_WIN];
static float Gyro_total[3];
static float sqrGyro_total[3];
static int GyroinitFlag = 0;
static int GyroCount = 0;

// 方差：Var = E[x^2] - (E[x])^2
void calGyroVariance(const float data[3], int length, float sqrResult[3], float avgResult[3])
{
    int i;

    if (GyroinitFlag == 0)
    {
        for (i = 0; i < 3; i++)
        {
            Gyro_fill[i][GyroCount] = data[i];
            Gyro_total[i] += data[i];
            sqrGyro_total[i] += data[i] * data[i];
            sqrResult[i] = 100.0f;
            avgResult[i] = 0.0f;
        }
    }
    else
    {
        for (i = 0; i < 3; i++)
        {
            float oldv = Gyro_fill[i][GyroCount];
            Gyro_total[i]    -= oldv;
            sqrGyro_total[i] -= oldv * oldv;

            Gyro_fill[i][GyroCount] = data[i];

            float newv = Gyro_fill[i][GyroCount];
            Gyro_total[i]    += newv;
            sqrGyro_total[i] += newv * newv;
        }
    }

    GyroCount++;
    if (GyroCount >= length)
    {
        GyroCount = 0;
        GyroinitFlag = 1;
    }

    if (GyroinitFlag == 0) return;

    const float invN = 1.0f / (float)length;
    for (i = 0; i < 3; i++)
    {
        float mean = Gyro_total[i] * invN;
        avgResult[i] = mean;

        float ex2 = sqrGyro_total[i] * invN;
        float var = ex2 - mean * mean;

        sqrResult[i] = (var > 0.0f) ? var : 0.0f;
    }
}

float gyro_offset[3] = {0};
int CalCount = 0;

/**************************实现函数********************************************
*函数原型:	   void IMU_getValues(float * values)
*功　　能:	 读取加速度/陀螺仪/（可选磁力计）当前值
*说明：你现有 bsp_IcmGetRawData 只提供 accel+gyro+temp
*      所以这里 values[6..8] 默认填 0，避免未初始化
*******************************************************************************/
void IMU_getValues(float * values)
{
    float accgyroval[7]; // accel[0..2], gyro[3..5], temp[6]

    float sqrResult_gyro[3];
    float avgResult_gyro[3];

    bsp_IcmGetRawData(accgyroval, &accgyroval[3], &accgyroval[6]);

    TTangles_gyro[0] = accgyroval[0];
    TTangles_gyro[1] = accgyroval[1];
    TTangles_gyro[2] = accgyroval[2];
    TTangles_gyro[3] = accgyroval[3];
    TTangles_gyro[4] = accgyroval[4];
    TTangles_gyro[5] = accgyroval[5];
    TTangles_gyro[6] = accgyroval[6];

    calGyroVariance(&TTangles_gyro[3], GYRO_VAR_WIN, sqrResult_gyro, avgResult_gyro);

    if ((sqrResult_gyro[0] < 0.02f || sqrResult_gyro[1] < 0.02f) &&
        (sqrResult_gyro[2] < 0.02f) &&
        (CalCount >= (GYRO_VAR_WIN - 1)))
    {
        gyro_offset[0] = avgResult_gyro[0];
        gyro_offset[1] = avgResult_gyro[1];
        gyro_offset[2] = avgResult_gyro[2];

        exInt = 0.0f;
        eyInt = 0.0f;
        ezInt = 0.0f;

        CalCount = 0;
        Kp = 0.5f;
    }
    else if (CalCount < GYRO_VAR_WIN)
    {
        CalCount++;
    }

    values[0] = accgyroval[0];
    values[1] = accgyroval[1];
    values[2] = accgyroval[2];

    values[3] = accgyroval[3] - gyro_offset[0];
    values[4] = accgyroval[4] - gyro_offset[1];
    values[5] = accgyroval[5] - gyro_offset[2];

    // 你当前驱动没有给磁力计：这里填 0，后续 AHRS 会自动跳过磁力计归一化
    values[6] = 0.0f; // mx
    values[7] = 0.0f; // my
    values[8] = 0.0f; // mz

    // 量程说明：你注释里写 1000dps, 32.8 LSB/(dps) 对应 1度/s
}

/**************************实现函数********************************************
*函数原型:	   void IMU_AHRSupdate
*功　　能:	 更新AHRS 更新四元数
*******************************************************************************/
//#define Kp 0.5f
#define Ki 0.001f

void IMU_AHRSupdate(float gx, float gy, float gz,
                    float ax, float ay, float az,
                    float mx, float my, float mz)
{
    float norm;
    float vx, vy, vz;
    float ex, ey, ez, halfT;
    float tempq0, tempq1, tempq2, tempq3;

    // 预计算
    float q0q0 = q0*q0;
    float q0q1 = q0*q1;
    float q0q2 = q0*q2;
    float q0q3 = q0*q3;
    float q1q1 = q1*q1;
    float q1q2 = q1*q2;
    float q1q3 = q1*q3;
    float q2q2 = q2*q2;
    float q2q3 = q2*q3;
    float q3q3 = q3*q3;

    halfT = 0.01f;

    // 加速度归一化（防 0）
    float acc2 = ax*ax + ay*ay + az*az;
    if (acc2 > 0.0f)
    {
        norm = invSqrt1(acc2);
        ax *= norm;
        ay *= norm;
        az *= norm;
    }
    else
    {
        // 加速度无效则不更新（按你原逻辑可以继续，但会引入 NaN 风险）
        return;
    }

    // 磁力计归一化（你的算法磁力计部分目前注释掉了，但这里仍做保护）
    float mag2 = mx*mx + my*my + mz*mz;
    if (mag2 > 0.0f)
    {
        norm = invSqrt1(mag2);
        mx *= norm;
        my *= norm;
        mz *= norm;
    }
    // else: 无磁力计就跳过（你下面误差项也没用磁力计）

    // 重力方向估计（由四元数换算）
    vx = 2.0f*(q1q3 - q0q2);
    vy = 2.0f*(q0q1 + q2q3);
    vz = q0q0 - q1q1 - q2q2 + q3q3;

    /* north / west 计算保持原样（全 float） */
    north.x = 1.0f - 2.0f*(q3q3 + q2q2);
    north.y = 2.0f * (-q0q3 + q1q2);
    north.z = 2.0f * ( q0q2 - q1q3);

    west.x  = 2.0f * ( q0q3 + q1q2);
    west.y  = 1.0f - 2.0f*(q3q3 + q1q1);
    west.z  = 2.0f * (-q0q1 + q2q3);

    // 误差：测得重力与估计重力的叉积
    ex = (ay*vz - az*vy);
    ey = (az*vx - ax*vz);
    ez = (ax*vy - ay*vx);

    // 你原来是 “三个都不为0才修正”，我保留这个条件以不改变行为
    if (ex != 0.0f && ey != 0.0f && ez != 0.0f)
    {
        exInt += ex * Ki * halfT;
        eyInt += ey * Ki * halfT;
        ezInt += ez * Ki * halfT;

        gx += Kp*ex + exInt;
        gy += Kp*ey + eyInt;
        gz += Kp*ez + ezInt;
    }

    // 四元数微分
    tempq0 = q0 + (-q1*gx - q2*gy - q3*gz)*halfT;
    tempq1 = q1 + ( q0*gx + q2*gz - q3*gy)*halfT;
    tempq2 = q2 + ( q0*gy - q1*gz + q3*gx)*halfT;
    tempq3 = q3 + ( q0*gz + q1*gy - q2*gx)*halfT;

    // 归一化
    norm = invSqrt1(tempq0*tempq0 + tempq1*tempq1 + tempq2*tempq2 + tempq3*tempq3);
    q0 = tempq0 * norm;
    q1 = tempq1 * norm;
    q2 = tempq2 * norm;
    q3 = tempq3 * norm;
}

/**************************实现函数********************************************
*函数原型:	   void IMU_getQ(float * q)
*功　　能:	 更新四元数 返回当前四元数
*******************************************************************************/
float mygetqval[9];

void IMU_getQ(float * q)
{
    IMU_getValues(mygetqval);

    IMU_AHRSupdate(mygetqval[3] * DEG2RAD_F,
                   mygetqval[4] * DEG2RAD_F,
                   mygetqval[5] * DEG2RAD_F,
                   mygetqval[0], mygetqval[1], mygetqval[2],
                   mygetqval[6], mygetqval[7], mygetqval[8]);

    q[0] = q0;
    q[1] = q1;
    q[2] = q2;
    q[3] = q3;
}

/**************************实现函数********************************************
*函数原型:	   void IMU_getYawPitchRoll(float * angles)
*功　　能:	 更新四元数 返回解算姿态角
*******************************************************************************/
void IMU_getYawPitchRoll(float * angles)
{
    float q[4];
    IMU_getQ(q);

    angles[0] = -atan2f(2.0f*q[1]*q[2] + 2.0f*q[0]*q[3],
                        -2.0f*q[2]*q[2] - 2.0f*q[3]*q[3] + 1.0f) * RAD2DEG_F; // yaw

    angles[1] = -asinf(-2.0f*q[1]*q[3] + 2.0f*q[0]*q[2]) * RAD2DEG_F; // pitch

    angles[2] =  atan2f(2.0f*q[2]*q[3] + 2.0f*q[0]*q[1],
                        -2.0f*q[1]*q[1] - 2.0f*q[2]*q[2] + 1.0f) * RAD2DEG_F; // roll
}

void IMU_TT_getgyro(float * zsjganda)
{
    zsjganda[0] = TTangles_gyro[0];
    zsjganda[1] = TTangles_gyro[1];
    zsjganda[2] = TTangles_gyro[2];
    zsjganda[3] = TTangles_gyro[3];
    zsjganda[4] = TTangles_gyro[4];
    zsjganda[5] = TTangles_gyro[5];
    zsjganda[6] = TTangles_gyro[6];
}

void MPU6050_InitAng_Offset(void)
{
}

/* ------------------End of File---------------------------- */
