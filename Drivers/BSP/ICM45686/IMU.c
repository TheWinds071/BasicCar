/* main.c file
编写者：lisn3188
网址：www.chiplab7.com
作者E-mail：lisn3188@163.com
编译环境：MDK-Lite  Version: 4.23 (Updated for STM32H7 FPU)
初版时间: 2012-04-25
修改时间: 2025-12-10 (Optimized for Cortex-M7 FPU)
测试： 本程序已在第七实验室的mini IMU上完成测试
功能：
姿态解算 IMU
将传感器的输出值进行姿态解算。得到目标载体的俯仰角和横滚角 和航向角
------------------------------------
 */
#include "IMU.h"
#include "inv_imu_driver.h"
#include <math.h> // 必须包含 math.h 以支持 sqrtf, atan2f, asinf

//#include "eeprom.h"
/* XYZ结构体 */

/* 加速度：由南向北方向的加速度在加速度计的分量 *//* 加速度：由东向西方向的加速度在加速度计的分量 */
xyz_f_t north,west;
volatile float exInt, eyInt, ezInt;  // 误差积分
volatile float q0, q1, q2, q3; // 全局四元数

volatile float yaw[5]= {0,0,0,0,0};  //处理航向的增值
int16_t Ax_offset=0,Ay_offset=0;
float TTangles_gyro[7]; //彤彤滤波角度

float Kp = 10.5f;


void MadgwickAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);

extern int bsp_IcmGetRawData(float accel_mg[3], float gyro_dps[3], float *temp_degc);

/**************************实现函数********************************************
*函数原型:	   float invSqrt1(float x)
*功　　能:	   快速计算 1/Sqrt(x)
*修改说明:     针对H7 FPU优化。
* Cortex-M7 拥有硬件 FPU，直接调用 sqrtf 编译为 VSQRT 指令，
* 比软件模拟的魔数算法更快且精度更高。
*******************************************************************************/
float invSqrt1(float x) {
    if (x <= 0.0f) return 0.0f;
    return 1.0f / sqrtf(x);
}

extern int setup_imu(int use_ln, int accel_en, int gyro_en);
/**************************实现函数********************************************
*函数原型:	   void IMU_init(void)
*功　　能:	  初始化IMU相关
			  初始化各个传感器
			  初始化四元数
			  将积分清零
			  更新系统时间
输入参数：无
输出参数：没有
*******************************************************************************/
void IMU_init(void)
{
	//while(!ICM_Init());	   //初始化ICM42688配置
	if (0x00 == setup_imu(1,1,1))
	{
		// initialize quaternion
		q0 = 1.0f;  //初始化四元数
		q1 = 0.0f;
		q2 = 0.0f;
		q3 = 0.0f;
		exInt = 0.0f; // float suffix
		eyInt = 0.0f;
		ezInt = 0.0f;
        HAL_Delay(100);
		return;
	}
	printf("IMU ERROR!!\r\n");
}

static double Gyro_fill[3][300];
static double Gyro_total[3];
static double sqrGyro_total[3];
static int GyroinitFlag = 0;
static int GyroCount = 0;

// 方差变形公式 S^2 = (X1^2 + X2^2 + X3^2 + ... +Xn^2)/n - X平均^2
// 函数名称: calVariance
// 功能描述: 计算方差
// 输入参数: data[] --- 参与计算方差的数据缓冲区
//           length --- 数据长度
// 输出参数:                                                                                      */
//           sqrResult[] --- 方差结果
//           avgResult[] --- 平均数

void calGyroVariance(float data[], int length, float sqrResult[], float avgResult[])
{
	int i;
	double tmplen;
	if (GyroinitFlag == 0)
	{
		for (i = 0; i< 3; i++)
		{
			Gyro_fill[i][GyroCount] = data[i];
			Gyro_total[i] += data[i];
			sqrGyro_total[i] += data[i] * data[i];
			sqrResult[i] = 100;
			avgResult[i] = 0;
		}
	}
	else
	{
		for (i = 0; i< 3; i++)
		{
			Gyro_total[i] -= Gyro_fill[i][GyroCount];
			sqrGyro_total[i] -= Gyro_fill[i][GyroCount] * Gyro_fill[i][GyroCount];
			Gyro_fill[i][GyroCount] = data[i];
			Gyro_total[i] += Gyro_fill[i][GyroCount];
			sqrGyro_total[i] += Gyro_fill[i][GyroCount] * Gyro_fill[i][GyroCount];
		}
	}
	GyroCount++;
	if (GyroCount >= length)
	{
		GyroCount = 0;
		GyroinitFlag = 1;
	}
	if (GyroinitFlag == 0)
	{
		return;
	}
	tmplen = length;
	for (i = 0; i< 3; i++)
	{
		avgResult[i] = (float)(Gyro_total[i] / tmplen);
		sqrResult[i] = (float)((sqrGyro_total[i] - Gyro_total[i] * Gyro_total[i] / tmplen) / tmplen);
	}
}
float gyro_offset[3] = {0};
int CalCount = 0;
/**************************实现函数********************************************
*函数原型:	   void IMU_getValues(float * values)
*功　　能:	 读取加速度 陀螺仪 磁力计 的当前值
输入参数： 将结果存放的数组首地址
输出参数：没有
*******************************************************************************/
void IMU_getValues(float * values) {
	float accgyroval[7];
//	icm42688RealData_t accval;
//	icm42688RealData_t gyroval;

	float sqrResult_gyro[3];
	float avgResult_gyro[3];
	//读取加速度和陀螺仪的当前ADC
	bsp_IcmGetRawData(accgyroval, &accgyroval[3], &accgyroval[6]);
    TTangles_gyro[0] =  accgyroval[0];
    TTangles_gyro[1] =  accgyroval[1];
    TTangles_gyro[2] =  accgyroval[2];
	TTangles_gyro[3] =  accgyroval[3];
	TTangles_gyro[4] =  accgyroval[4];
	TTangles_gyro[5] =  accgyroval[5];
	TTangles_gyro[6] =  accgyroval[6];

	calGyroVariance(&TTangles_gyro[3], 100, sqrResult_gyro, avgResult_gyro);
	if ((sqrResult_gyro[0] < 0.02f || sqrResult_gyro[1] < 0.02f) && sqrResult_gyro[2] < 0.02f && CalCount >= 99)
	{
		gyro_offset[0] = avgResult_gyro[0];
		gyro_offset[1] = avgResult_gyro[1];
		gyro_offset[2] = avgResult_gyro[2];
		exInt = 0;
		eyInt = 0;
		ezInt = 0;
		CalCount = 0;
        Kp = 0.5f;
	}
	else if (CalCount < 100)
	{
		CalCount++;
	}
    values[0] =  accgyroval[0];
    values[1] =  accgyroval[1];
    values[2] =  accgyroval[2];
	values[3] =  accgyroval[3] - gyro_offset[0];
	values[4] =  accgyroval[4] - gyro_offset[1];
	values[5] =  accgyroval[5] - gyro_offset[2];


		//这里已经将量程改成了 1000度每秒  32.8 对应 1度每秒
}


/**************************实现函数********************************************
*函数原型:	   void IMU_AHRSupdate
*功　　能:	 更新AHRS 更新四元数
输入参数： 当前的测量值。
输出参数：没有
* H7 Optimization: Replaced literals with float literals (2 -> 2.0f) to avoid double promotion.
*******************************************************************************/
//#define Kp 0.5f   // proportional gain governs rate of convergence to accelerometer/magnetometer
#define Ki 0.001f   // integral gain governs rate of convergence of gyroscope biases

void IMU_AHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz) {
  float norm;
  //float hx, hy, hz, bx, bz;
  float vx, vy, vz;//, wx, wy, wz;
  float ex, ey, ez,halfT;
  float tempq0,tempq1,tempq2,tempq3;

  // 先把这些用得到的值算好
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

  halfT =  0.01f;

  // Normalise accelerometer measurement
  norm = invSqrt1(ax*ax + ay*ay + az*az);
  ax = ax * norm;
  ay = ay * norm;
  az = az * norm;
  //把加计的三维向量转成单位向量。

  // Normalise magnetometer measurement
  norm = invSqrt1(mx*mx + my*my + mz*mz);
  mx = mx * norm;
  my = my * norm;
  mz = mz * norm;

  // estimated direction of gravity and flux (v and w)
  // Optimization: Added 'f' suffix to constants to enforce Single Precision FPU instructions
  vx = 2.0f*(q1q3 - q0q2);
  vy = 2.0f*(q0q1 + q2q3);
  vz = q0q0 - q1q1 - q2q2 + q3q3;

  /* 加速度：由南向北方向的加速度在加速度计X分量 */
	north.x = 1.0f - 2.0f*(q3*q3 + q2*q2);
	/* 加速度：由南向北方向的加速度在加速度计Y分量 */
	north.y = 2.0f* (-q0*q3 + q1*q2);
	/* 加速度：由南向北方向的加速度在加速度计Z分量 */
	north.z = 2.0f* (+q0*q2  - q1*q3);
	/* 加速度：由东向西方向的加速度在加速度计X分量 */
	west.x = 2.0f* (+q0*q3 + q1*q2);
	/* 加速度：由东向西方向的加速度在加速度计Y分量 */
	west.y = 1.0f - 2.0f*(q3*q3 + q1*q1);
	/* 加速度：由东向西方向的加速度在加速度计Z分量 */
	west.z = 2.0f* (-q0*q1 + q2*q3);

  // error is sum of cross product between reference direction of fields and direction measured by sensors
  ex = (ay*vz - az*vy);// + (my*wz - mz*wy);
  ey = (az*vx - ax*vz);// + (mz*wx - mx*wz);
  ez = (ax*vy - ay*vx);// + (mx*wy - my*wx);

if(ex != 0.0f && ey != 0.0f && ez != 0.0f){
  exInt = exInt + ex * Ki * halfT;
  eyInt = eyInt + ey * Ki * halfT;
  ezInt = ezInt + ez * Ki * halfT;

  // 用叉积误差来做PI修正陀螺零偏
  gx = gx + Kp*ex + exInt;
  gy = gy + Kp*ey + eyInt;
  gz = gz + Kp*ez + ezInt;

  }

  // 四元数微分方程
  tempq0 = q0 + (-q1*gx - q2*gy - q3*gz)*halfT;
  tempq1 = q1 + (q0*gx + q2*gz - q3*gy)*halfT;
  tempq2 = q2 + (q0*gy - q1*gz + q3*gx)*halfT;
  tempq3 = q3 + (q0*gz + q1*gy - q2*gx)*halfT;

  // 四元数规范化
  norm = invSqrt1(tempq0*tempq0 + tempq1*tempq1 + tempq2*tempq2 + tempq3*tempq3);
  q0 = tempq0 * norm;
  q1 = tempq1 * norm;
  q2 = tempq2 * norm;
  q3 = tempq3 * norm;
}


/**************************实现函数********************************************
*函数原型:	   void IMU_getQ(float * q)
*功　　能:	 更新四元数 返回当前的四元数组值
输入参数： 将要存放四元数的数组首地址
输出参数：没有
*******************************************************************************/
float mygetqval[9];	//用于存放传感器转换结果的数组
void IMU_getQ(float * q) {

  IMU_getValues(mygetqval);
  //将陀螺仪的测量值转成弧度每秒
  //加速度和磁力计保持 ADC值　不需要转换
  // M_PI 通常定义为 double，强制转换为 float 或使用 f 后缀常量
  const float DEG_TO_RAD = 3.1415926535f / 180.0f;

  IMU_AHRSupdate(mygetqval[3] * DEG_TO_RAD,
                 mygetqval[4] * DEG_TO_RAD,
                 mygetqval[5] * DEG_TO_RAD,
                 mygetqval[0], mygetqval[1], mygetqval[2],
                 mygetqval[6], mygetqval[7], mygetqval[8]);

  q[0] = q0; //返回当前值
  q[1] = q1;
  q[2] = q2;
  q[3] = q3;
}


/**************************实现函数********************************************
*函数原型:	   void IMU_getYawPitchRoll(float * angles)
*功　　能:	 更新四元数 返回当前解算后的姿态数据
输入参数： 将要存放姿态角的数组首地址
输出参数：没有
* H7 Optimization: Used atan2f and asinf instead of double versions.
*******************************************************************************/
void IMU_getYawPitchRoll(float * angles) {
  float q[4]; //　四元数
  // volatile float gx=0.0, gy=0.0, gz=0.0; // unused variable
  IMU_getQ(q); //更新全局四元数

  const float RAD_TO_DEG = 180.0f / 3.1415926535f;

  // 使用 atan2f 和 asinf 代替 atan2 和 asin，避免双精度转换
  angles[0] = -atan2f(2.0f * q[1] * q[2] + 2.0f * q[0] * q[3], -2.0f * q[2]*q[2] - 2.0f * q[3] * q[3] + 1.0f) * RAD_TO_DEG; // yaw
  angles[1] = -asinf(-2.0f * q[1] * q[3] + 2.0f * q[0] * q[2]) * RAD_TO_DEG; // pitch
  angles[2] = atan2f(2.0f * q[2] * q[3] + 2.0f * q[0] * q[1], -2.0f * q[1] * q[1] - 2.0f * q[2] * q[2] + 1.0f) * RAD_TO_DEG; // roll
 // if(angles[0]<0)angles[0]+=360.0f;  //将 -+180度  转成0-360度
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
//------------------End of File----------------------------