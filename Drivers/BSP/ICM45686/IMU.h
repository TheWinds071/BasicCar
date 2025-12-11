#ifndef __IMU_H
#define __IMU_H

#include <math.h>
#include <stdint.h>

#ifndef M_PI
#define M_PI  3.1415926535f
#endif

typedef struct
{
    float x;
    float y;
    float z;
} xyz_f_t;

extern xyz_f_t north, west;
extern volatile float q0, q1, q2, q3; // 全局四元数
extern float gyro_offset[3];          // 陀螺仪零偏

// API
void IMU_init(void);
void IMU_getValues(float * values);
void IMU_getYawPitchRoll(float * ypr);
void IMU_TT_getgyro(float * zsjganda);

/* 核心解算函数，现在支持传入 dt 以适应不同频率 */
void IMU_AHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);

#endif