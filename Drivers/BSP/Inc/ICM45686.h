#ifndef __ICM45686_H
#define __ICM45686_H


/*
 * ICM45686.h
 *
 *  Created on: Dec 17, 2024
 *      Author: aoi25
 */

#include <stdint.h>
#include <stdbool.h> // for bool type if needed elsewhere, though bitset not directly translated
#include <string.h> // 用于 memset 函数
#include <math.h>   // 用于 powf 函数 (C 语言中浮点幂运算)
#include "main.h"

// =============================================================================
// 枚举定义 (Enum Definitions)
// =============================================================================

// 寄存器地址
typedef enum {
    ICM45686_REG_ACCEL_DATA_X1_UI = 0x00, // 修正：根据官方驱动，数据寄存器起始地址为0x0B
    ICM45686_REG_PWR_MGMT0        = 0x10,
    ICM45686_REG_ACCEL_CONFIG     = 0x1B,
    ICM45686_REG_GYRO_CONFIG      = 0x1C,
    ICM45686_REG_WHO_AM_I         = 0x72, // 修正：根据官方驱动，WHO_AM_I地址为0x75
    ICM45686_REG_BANK_SEL         = 0x76  // 添加：寄存器组选择寄存器
} ICM45686_Register_t;

// 传感器模式
typedef enum {
    ICM45686_MODE_OFF = 0x00,
    ICM45686_MODE_STANDBY,
    ICM45686_MODE_LOW_POWER,
    ICM45686_MODE_LOW_NOISE
} ICM45686_Mode_t;

// 加速度计量程
typedef enum {
    ICM45686_ACCEL_SCALE_32G = 0x00,
    ICM45686_ACCEL_SCALE_16G,
    ICM45686_ACCEL_SCALE_08G,
    ICM45686_ACCEL_SCALE_04G,
    ICM45686_ACCEL_SCALE_02G
} ICM45686_AccelScale_t;

// 陀螺仪量程
typedef enum {
    ICM45686_GYRO_SCALE_4000DPS = 0x00,
    ICM45686_GYRO_SCALE_2000DPS,
    ICM45686_GYRO_SCALE_1000DPS,
    ICM45686_GYRO_SCALE_0500DPS,
    ICM45686_GYRO_SCALE_0250DPS,
    ICM45686_GYRO_SCALE_0125DPS,
    ICM45686_GYRO_SCALE_0062DPS,
    ICM45686_GYRO_SCALE_0031DPS,
    ICM45686_GYRO_SCALE_0015DPS,
    ICM45686_GYRO_SCALE_0006DPS
} ICM45686_GyroScale_t;

// 输出数据速率 (ODR)
typedef enum {
    ICM45686_ODR_RATE_6400HZ = 3,
    ICM45686_ODR_RATE_3200HZ,
    ICM45686_ODR_RATE_1600HZ,
    ICM45686_ODR_RATE_0800HZ,
    ICM45686_ODR_RATE_0400HZ,
    ICM45686_ODR_RATE_0200HZ,
    ICM45686_ODR_RATE_0100HZ,
    ICM45686_ODR_RATE_0050HZ,
    ICM45686_ODR_RATE_0025HZ,
    ICM45686_ODR_RATE_0012HZ,
    ICM45686_ODR_RATE_0006HZ,
    ICM45686_ODR_RATE_0003HZ,
    ICM45686_ODR_RATE_0001HZ
} ICM45686_ODR_t;



//四元数
typedef struct
{
  float q0;
  float q1;
  float q2;
  float q3;
} Quaternion;


typedef struct
{
	int16_t accX;
	int16_t accY;
	int16_t accZ;
	int16_t gyroX;
	int16_t gyroY;
	int16_t gyroZ;
}_st_Mpu;

typedef struct
{
	float roll;
	float pitch;
	float yaw;
}_st_AngE;

void GetAngle(const _st_Mpu *pMpu,_st_AngE *pAngE, float dt);




typedef struct {
    // ---- 内部数据 (Internal Data) ----
    uint8_t raw_data[12];        // 存储原始数据缓冲区
    uint8_t pre_data;            // 暂不明确用途，保留

    float accel_scale_value;     // 加速度计量程转换值
    float gyro_scale_value;      // 陀螺仪量程转换值
    uint8_t accel_mode_tmp;      // 加速度计模式临时存储
    uint8_t gyro_mode_tmp;       // 陀螺仪模式临时存储

    int32_t accel_offset[3];     // 加速度计校准偏移
    int32_t gyro_offset[3];      // 陀螺仪校准偏移

    float G;                     // 重力加速度常量


} ICM45686_t;


uint8_t ICM45686_Connection(void);
uint8_t ICM45686_GetRawData(ICM45686_t *imu, _st_Mpu *mpu_data);
uint8_t ICM45686_AccelConfig(ICM45686_t *imu, ICM45686_Mode_t Mode, ICM45686_AccelScale_t Scale, ICM45686_ODR_t ODR);
uint8_t ICM45686_GyroConfig(ICM45686_t *imu, ICM45686_Mode_t Mode, ICM45686_GyroScale_t Scale, ICM45686_ODR_t ODR);
uint8_t ICM45686_SetBank(uint8_t bank); // 添加：切换寄存器组函数声明
void ICM45686_Init(ICM45686_t *imu);
void ICM45686_ConvertToPhysicalUnits(ICM45686_t *imu,
                                     int16_t raw_accel[3], int16_t raw_gyro[3],
                                     float accel_mps2[3], float gyro_dps[3]);

/* --- 新增：适配 main.c 的高层接口 --- */
void IMU_init(void);
void IMU_getYawPitchRoll(float *ypr);

#endif