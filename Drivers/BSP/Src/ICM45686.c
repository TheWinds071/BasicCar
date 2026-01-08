#include "ICM45686.h"
#include "main.h"
#include "spi.h"

// 定义CS引脚操作宏
#define ICM45686_CS_LOW()   HAL_GPIO_WritePin(SPI6_CS_GPIO_Port, SPI6_CS_Pin, GPIO_PIN_RESET);
#define ICM45686_CS_HIGH()   HAL_GPIO_WritePin(SPI6_CS_GPIO_Port, SPI6_CS_Pin, GPIO_PIN_SET);

/**
 * @brief  SPI1 读写传输函数 (HAL库封装)
 * @param  tx_data: 发送数据缓冲区指针
 * @param  rx_data: 接收数据缓冲区指针
 * @param  len:     数据长度
 */
void SPI6_TransmitReceive(uint8_t *tx_data, uint8_t *rx_data, uint32_t len) {
    /* * 参数说明：
     * &hspi1 : SPI句柄，通常在 spi.c 或 main.c 中定义
     * tx_data: 发送缓冲区
     * rx_data: 接收缓冲区
     * len    : 数据长度
     * 1000   : 超时时间 (毫秒)，可根据实际需求调整
     */
    if (HAL_SPI_TransmitReceive(&hspi6, tx_data, rx_data, len, 1000) != HAL_OK) {
        // 如果需要，可以在这里添加错误处理代码
        // 例如：Error_Handler();
    }
}

/**
 * @brief 检查ICM45686传感器的连接状态。
 * 通过读取WHO_AM_I寄存器来验证设备ID是否符合预期。
 * @return uint8_t 0 表示连接成功，1 表示连接失败（达到最大尝试次数）。
 */
uint8_t ICM45686_Connection(void) {
    uint8_t product_id = 0x00;
    uint8_t error_count = 0;

    // ICM45686的Product ID预期值为0xE9
    // 循环读取WHO_AM_I寄存器，直到获取到正确的Product ID或达到最大尝试次数
    while (product_id != 0xE9) {
        // 拉低CS线以开始SPI通信
        ICM45686_CS_LOW();

        // 读取WHO_AM_I寄存器
        // 使用修正后的寄存器地址 0x75
        uint8_t tx_buffer[2] = {ICM45686_REG_WHO_AM_I | 0x80, 0x00}; // 读命令和虚拟字节
        uint8_t rx_buffer[2] = {0, 0};

        SPI6_TransmitReceive(tx_buffer, rx_buffer, 2); // 发送读命令并接收数据

        // 拉高CS线以结束SPI通信
        ICM45686_CS_HIGH();

        product_id = rx_buffer[1]; // Product ID通常在第二个字节（接收虚拟字节后的数据）

        error_count++;

        // 如果尝试次数达到100次，认为连接失败
        if (error_count >= 100) {
            return 1; // 连接失败
        }

        // 可以添加一个短延时，防止在while循环中过于频繁地访问SPI
        // 具体延时时间取决于您的系统和IMU响应速度
        // 例如：si_sleep_us(1000); // 假设存在一个微秒级延时函数
    }

    return 0; // 连接成功
}

/**
 * @brief 初始化ICM45686传感器状态结构体。
 * @param imu 指向ICM45686_t结构体的指针。
 */
void ICM45686_Init(ICM45686_t *imu) {
    memset(imu->raw_data, 0, sizeof(imu->raw_data));
    imu->pre_data = 0;
    imu->accel_scale_value = 0.0f;
    imu->gyro_scale_value = 0.0f;
    imu->accel_mode_tmp = 0;
    imu->gyro_mode_tmp = 0;
    // memset(imu->accel_offset, 0, sizeof(imu->accel_offset));
    // memset(imu->gyro_offset, 0, sizeof(imu->gyro_offset));
    imu->G = 9.80665f; // 初始化重力加速度常量
}

/**
 * @brief 从ICM45686寄存器读取数据。
 * @param reg_addr 寄存器地址。
 * @param data 指向存储读取数据的缓冲区的指针。
 * @param len 要读取的字节数。
 * @return uint8_t 0表示成功，非0表示失败。
 */
static uint8_t ICM45686_ReadRegister(uint8_t reg_addr, uint8_t *data, uint32_t len) {
    ICM45686_CS_LOW();
    // SPI读操作：发送寄存器地址（最高位为1），然后发送len个虚拟字节接收数据
    uint8_t tx_buffer[len + 1];
    uint8_t rx_buffer[len + 1];

    tx_buffer[0] = reg_addr | 0x80; // 设置读位 (Bit 7 = 1 for Read)
    for (uint32_t i = 0; i < len; i++) {
        tx_buffer[i + 1] = 0x00; // 虚拟字节
    }

    SPI6_TransmitReceive(tx_buffer, rx_buffer, len + 1);
    ICM45686_CS_HIGH();

    for (uint32_t i = 0; i < len; i++) {
        data[i] = rx_buffer[i + 1]; // 实际数据从第二个字节开始
    }
    return 0; // 假设成功
}

/**
 * @brief 向ICM45686寄存器写入数据。
 * @param reg_addr 寄存器地址。
 * @param data 指向要写入数据的缓冲区的指针。
 * @param len 要写入的字节数。
 * @return uint8_t 0表示成功，非0表示失败。
 */
static uint8_t ICM45686_WriteRegister(uint8_t reg_addr, const uint8_t *data, uint32_t len) {
    ICM45686_CS_LOW();
    // SPI写操作：发送寄存器地址（最高位为0），然后发送len个数据字节
    uint8_t tx_buffer[len + 1];
    uint8_t rx_buffer[len + 1]; // 接收缓冲区也需要，即使不读取数据

    tx_buffer[0] = reg_addr & 0x7F; // 清除读位 (Bit 7 = 0 for Write)
    for (uint32_t i = 0; i < len; i++) {
        tx_buffer[i + 1] = data[i];
    }

    SPI6_TransmitReceive(tx_buffer, rx_buffer, len + 1);
    ICM45686_CS_HIGH();
    return 0; // 假设成功
}

/**
 * @brief 切换ICM45686的寄存器组。
 * @param bank 要切换到的寄存器组编号 (0-4)。
 * @return uint8_t 0表示成功，非0表示失败。
 */
uint8_t ICM45686_SetBank(uint8_t bank) {
    uint8_t data = bank;
    return ICM45686_WriteRegister(ICM45686_REG_BANK_SEL, &data, 1);
}

/**
 * @brief 配置加速度计。
 *
 * 通过PWR_MGMT0寄存器设置电源模式，并通过ACCEL_CONFIG寄存器设置刻度(Scale)和输出数据率(ODR)。
 * 最多会进行100次重试。
 *
 * @param imu 指向ICM45686_t结构体的指针，用于保存传感器状态。
 * @param Mode 电源模式设置。
 * @param Scale 加速度计的刻度设置。
 * @param ODR 输出数据率设置。
 *
 * @return uint8_t 0: 配置成功, 1: PWR_MGMT0配置失败, 2: ACCEL_CONFIG配置失败
 */
uint8_t ICM45686_AccelConfig(ICM45686_t *imu, ICM45686_Mode_t Mode, ICM45686_AccelScale_t Scale, ICM45686_ODR_t ODR) {
    uint8_t current_pwr_mgmt0_val = 0x00;
    uint8_t error_count = 0;
    uint8_t command_byte;

    // --- 配置 PWR_MGMT0 寄存器中的加速度计电源模式 ---
    // 读取当前的PWR_MGMT0寄存器值
    if (ICM45686_ReadRegister(ICM45686_REG_PWR_MGMT0, &current_pwr_mgmt0_val, 1) != 0) {
        return 1; // 读取失败
    }

    // 构造写入PWR_MGMT0的命令字节
    // 清除加速度计模式位 (PWR_MGMT0的低2位)，然后设置新模式
    // 假设ICM45686_Mode_t的值可以直接映射到寄存器位。根据数据手册确认位掩码。
    // 这里假设加速度计模式位在PWR_MGMT0的低2位 (0x03)。
    command_byte = (current_pwr_mgmt0_val & ~0x03) | ((uint8_t)Mode & 0x03);

    // 循环写入并验证，直到值匹配或达到最大重试次数
    error_count = 0;
    uint8_t verify_pwr_mgmt0_val = 0;
    while (verify_pwr_mgmt0_val != command_byte) {
        if (ICM45686_WriteRegister(ICM45686_REG_PWR_MGMT0, &command_byte, 1) != 0) {
            // 写入失败，可以考虑在这里返回错误或继续重试
        }
        if (ICM45686_ReadRegister(ICM45686_REG_PWR_MGMT0, &verify_pwr_mgmt0_val, 1) != 0) {
            // 读取验证失败，可以考虑在这里返回错误或继续重试
        }

        error_count++;
        if (error_count > 100) {
            return 1; // PWR_MGMT0配置失败
        }
    }

    // 保存写入的加速度计模式位值到状态结构体
    imu->accel_mode_tmp = (uint8_t)Mode & 0x03; // 只保存模式位

    // --- 配置 ACCEL_CONFIG 寄存器中的 ODR 和 Scale ---
    // 假设 ODR 对应低4位 (0x0F)，Scale 对应高4位 (0xF0)，且 Scale 值需左移4位
    command_byte = ((uint8_t)ODR & 0x0F) | (((uint8_t)Scale & 0x0F) << 4);

    // 循环写入并验证，直到值匹配或达到最大重试次数
    uint8_t current_accel_config_val = 0;
    error_count = 0;
    while (current_accel_config_val != command_byte) {
        if (ICM45686_WriteRegister(ICM45686_REG_ACCEL_CONFIG, &command_byte, 1) != 0) {
            // 写入失败
        }
        if (ICM45686_ReadRegister(ICM45686_REG_ACCEL_CONFIG, &current_accel_config_val, 1) != 0) {
            // 读取验证失败
        }

        error_count++;
        if (error_count > 100) {
            return 2; // ACCEL_CONFIG配置失败
        }
    }

    // 保存 Scale 的实际值
    // 原始公式：AccelScaleValue = 32.0 / pow(2,(uint8_t)Scale);
    imu->accel_scale_value = 32.0f / powf(2.0f, (float)Scale); // 使用powf for float

    return 0; // 配置成功
}

/**
 * @brief 配置陀螺仪。
 *
 * 通过PWR_MGMT0寄存器设置电源模式，并通过GYRO_CONFIG寄存器设置刻度(Scale)和输出数据率(ODR)。
 * 最多会进行100次重试。
 *
 * @param imu 指向ICM45686_t结构体的指针，用于保存传感器状态。
 * @param Mode 电源模式设置。
 * @param Scale 陀螺仪的刻度设置。
 * @param ODR 输出数据率设置。
 *
 * @return uint8_t 0: 配置成功, 1: PWR_MGMT0配置失败, 2: GYRO_CONFIG配置失败
 */
uint8_t ICM45686_GyroConfig(ICM45686_t *imu, ICM45686_Mode_t Mode, ICM45686_GyroScale_t Scale, ICM45686_ODR_t ODR) {
    uint8_t current_pwr_mgmt0_val = 0x00;
    uint8_t error_count = 0;
    uint8_t command_byte;

    // --- 配置 PWR_MGMT0 寄存器中的陀螺仪电源模式 ---
    // 读取当前的PWR_MGMT0寄存器值
    if (ICM45686_ReadRegister(ICM45686_REG_PWR_MGMT0, &current_pwr_mgmt0_val, 1) != 0) {
        return 1; // 读取失败
    }

    // 构造写入PWR_MGMT0的命令字节
    // 清除陀螺仪模式位 (PWR_MGMT0的位2和3)，然后设置新模式
    // 原始代码是 `(uint8_t)Mode << 2`，所以假设陀螺仪模式位是位2和3 (0x0C = 0b00001100)
    command_byte = (current_pwr_mgmt0_val & ~0x0C) | (((uint8_t)Mode & 0x03) << 2);

    // 循环写入并验证，直到值匹配或达到最大重试次数
    error_count = 0;
    uint8_t verify_pwr_mgmt0_val = 0;
    while (verify_pwr_mgmt0_val != command_byte) {
        if (ICM45686_WriteRegister(ICM45686_REG_PWR_MGMT0, &command_byte, 1) != 0) {
            // 写入失败
        }
        if (ICM45686_ReadRegister(ICM45686_REG_PWR_MGMT0, &verify_pwr_mgmt0_val, 1) != 0) {
            // 读取验证失败
        }

        error_count++;
        if (error_count > 100) {
            return 1; // PWR_MGMT0配置失败
        }
    }

    // 保存写入的陀螺仪模式位值到状态结构体
    imu->gyro_mode_tmp = (uint8_t)Mode & 0x03; // 只保存模式位

    // --- 配置 GYRO_CONFIG 寄存器中的 ODR 和 Scale ---
    // 假设 ODR 对应低4位 (0x0F)，Scale 对应高4位 (0xF0)，且 Scale 值需左移4位
    command_byte = ((uint8_t)ODR & 0x0F) | (((uint8_t)Scale & 0x0F) << 4);

    // 循环写入并验证，直到值匹配或达到最大重试次数
    uint8_t current_gyro_config_val = 0;
    error_count = 0;
    while (current_gyro_config_val != command_byte) {
        if (ICM45686_WriteRegister(ICM45686_REG_GYRO_CONFIG, &command_byte, 1) != 0) {
            // 写入失败
        }
        if (ICM45686_ReadRegister(ICM45686_REG_GYRO_CONFIG, &current_gyro_config_val, 1) != 0) {
            // 读取验证失败
        }

        error_count++;
        if (error_count > 100) {
            return 2; // GYRO_CONFIG配置失败
        }
    }

    // 保存 Scale 的实际值
    // 原始公式：GyroScaleValue = 4000 / pow(2, (uint8_t)Scale);
    imu->gyro_scale_value = 4000.0f / powf(2.0f, (float)Scale); // 使用powf for float

    return 0; // 配置成功
}

/**
 * @brief 从加速度计和陀螺仪获取原始ADC数据。
 *
 * @param imu 指向ICM45686_t结构体的指针，用于保存传感器状态和原始数据。

 *
 * @return uint8_t 0: 成功, 1: 失败 (读取错误)
 */
uint8_t ICM45686_GetRawData(ICM45686_t *imu, _st_Mpu *mpu_data) {
    // 从传感器连续读取12个字节的原始数据 (Accel X,Y,Z, Gyro X,Y,Z, 各2字节)
    // 使用修正后的数据寄存器起始地址 0x0B
    // 注意：ICM45686通常是小端模式，数据手册通常会详细说明寄存器顺序。
    // 如果数据手册指示寄存器是 ACCEL_XOUT_L, ACCEL_XOUT_H, ACCEL_YOUT_L, ...
    // 那么，一次性读取12个字节，它们的顺序就是 LSB, MSB, LSB, MSB...
    if (ICM45686_ReadRegister(ICM45686_REG_ACCEL_DATA_X1_UI, imu->raw_data, 12) != 0) {
        return 1; // 读取失败
    }

    // 处理获取到的数据：将8位字节组合成16位原始ADC值
    // 根据官方驱动，ICM-45686使用小端模式 (Little-Endian)：低字节在前，高字节在后
    // 加速度计数据：低字节在前，高字节在后
    mpu_data->accX = (int16_t)(imu->raw_data[0] | (imu->raw_data[1] << 8));
    mpu_data->accY = (int16_t)(imu->raw_data[2] | (imu->raw_data[3] << 8));
    mpu_data->accZ = (int16_t)(imu->raw_data[4] | (imu->raw_data[5] << 8));

    // 陀螺仪数据：低字节在前，高字节在后
    mpu_data->gyroX = (int16_t)(imu->raw_data[6] | (imu->raw_data[7] << 8));
    mpu_data->gyroY = (int16_t)(imu->raw_data[8] | (imu->raw_data[9] << 8));
    mpu_data->gyroZ = (int16_t)(imu->raw_data[10] | (imu->raw_data[11] << 8));

    return 0; // 成功
}

/**
 * @brief 将原始ADC值转换为物理单位。
 *
 * 这是一个示例函数，展示如何将 ICM45686_GetRawData 获取的原始ADC值
 * 转换为实际的物理单位（加速度：m/s?，角速度：dps）。
 *
 * @param imu 指向ICM45686_t结构体的指针，包含量程转换值。
 * @param raw_accel 原始加速度计ADC值数组 (X, Y, Z轴)。
 * @param raw_gyro 原始陀螺仪ADC值数组 (X, Y, Z轴)。
 * @param accel_mps2 输出：加速度值，单位 m/s? (X, Y, Z轴)。
 * @param gyro_dps 输出：角速度值，单位 dps (X, Y, Z轴)。
 */
void ICM45686_ConvertToPhysicalUnits(ICM45686_t *imu,
                                     int16_t raw_accel[3], int16_t raw_gyro[3],
                                     float accel_mps2[3], float gyro_dps[3]) {
    // 单位转换
    for (int i = 0; i < 3; i++) {
        // 加速度计：ADC值 -> g -> m/s?
        accel_mps2[i] = (raw_accel[i] / 32768.0f) * imu->accel_scale_value * imu->G;

        // 陀螺仪：ADC值 -> dps
        gyro_dps[i] = (raw_gyro[i] / 32768.0f) * imu->gyro_scale_value;
    }
}


//四元数姿态解算
static float NormAccz;
// const float M_PI = 3.1415926535;
const float RtA = 57.2957795f;
const float Gyro_G = 0.03051756f;
const float Gyro_Gr = 0.0005326f;
#define squa( Sq )   (((float)Sq)*((float)Sq))
float Q_rsqrt(float number);

void GetAngle(const _st_Mpu *pMpu,_st_AngE *pAngE, float dt)
{
	volatile struct V
	{
		float x;
		float y;
		float z;
	} Gravity,Acc,Gyro,AccGravity;

	static struct V GyroIntegError = {0};
	static  float KpDef = 0.8f ;
	static  float KiDef = 0.0003f;
	static Quaternion NumQ = {1, 0, 0, 0};  // 四元素
	float q0_t,q1_t,q2_t,q3_t;
	float NormQuat;
	float HalfTime = dt * 0.5f;
	// 提取等效旋转矩阵中的重力分量
	Gravity.x = 2*(NumQ.q1 * NumQ.q3 - NumQ.q0 * NumQ.q2);
	Gravity.y = 2*(NumQ.q0 * NumQ.q1 + NumQ.q2 * NumQ.q3);
	Gravity.z = 1-2*(NumQ.q1 * NumQ.q1 + NumQ.q2 * NumQ.q2);

	// 加速度归一化
  NormQuat = Q_rsqrt(squa(pMpu->accX)+ squa(pMpu->accY) +squa(pMpu->accZ));

  Acc.x = pMpu->accX * NormQuat;
  Acc.y = pMpu->accY * NormQuat;
  Acc.z = pMpu->accZ * NormQuat;

 	//向量叉乘得出的值
	AccGravity.x = (Acc.y * Gravity.z - Acc.z * Gravity.y);
	AccGravity.y = (Acc.z * Gravity.x - Acc.x * Gravity.z);
	AccGravity.z = (Acc.x * Gravity.y - Acc.y * Gravity.x);

	//再做加速度积分补偿角速度的补偿值
  GyroIntegError.x += AccGravity.x * KiDef;
  GyroIntegError.y += AccGravity.y * KiDef;
  GyroIntegError.z += AccGravity.z * KiDef;

	//角速度融合加速度积分补偿值
  Gyro.x = pMpu->gyroX * Gyro_Gr + KpDef * AccGravity.x  +  GyroIntegError.x;//弧度制
  Gyro.y = pMpu->gyroY * Gyro_Gr + KpDef * AccGravity.y  +  GyroIntegError.y;
  Gyro.z = pMpu->gyroZ * Gyro_Gr + KpDef * AccGravity.z  +  GyroIntegError.z;

	// 一阶龙格库塔法, 更新四元数
	q0_t = (-NumQ.q1*Gyro.x - NumQ.q2*Gyro.y - NumQ.q3*Gyro.z) * HalfTime;
	q1_t = ( NumQ.q0*Gyro.x - NumQ.q3*Gyro.y + NumQ.q2*Gyro.z) * HalfTime;
	q2_t = ( NumQ.q3*Gyro.x + NumQ.q0*Gyro.y - NumQ.q1*Gyro.z) * HalfTime;
	q3_t = (-NumQ.q2*Gyro.x + NumQ.q1*Gyro.y + NumQ.q0*Gyro.z) * HalfTime;

	NumQ.q0 += q0_t;
	NumQ.q1 += q1_t;
	NumQ.q2 += q2_t;
	NumQ.q3 += q3_t;
	// 四元数归一化
	NormQuat = Q_rsqrt(squa(NumQ.q0) + squa(NumQ.q1) + squa(NumQ.q2) + squa(NumQ.q3));
	NumQ.q0 *= NormQuat;
	NumQ.q1 *= NormQuat;
	NumQ.q2 *= NormQuat;
	NumQ.q3 *= NormQuat;

	// 四元数转欧拉角
	{
		/*机体坐标系下的Z方向向量*/
		float vecxZ = 2 * NumQ.q0 *NumQ.q2 - 2 * NumQ.q1 * NumQ.q3 ;/*矩阵(3,1)项*/
		float vecyZ = 2 * NumQ.q2 *NumQ.q3 + 2 * NumQ.q0 * NumQ.q1;/*矩阵(3,2)项*/
		float veczZ =  1 - 2 * NumQ.q1 *NumQ.q1 - 2 * NumQ.q2 * NumQ.q2;	/*矩阵(3,3)项*/
#ifdef	YAW_GYRO
		*(float *)pAngE = atan2f(2 * NumQ.q1 *NumQ.q2 + 2 * NumQ.q0 * NumQ.q3, 1 - 2 * NumQ.q2 *NumQ.q2 - 2 * NumQ.q3 * NumQ.q3) * RtA;  //yaw
#else
		float yaw_G = pMpu->gyroZ * Gyro_G;
		if((yaw_G > 1.0f) || (yaw_G < -1.0f)) //数据太小可以认为是干扰，不是偏航动作
		{
			pAngE->yaw  -= yaw_G * dt;

		    // --- [Start] 修改：限制角度在 -180 到 180 之间 ---
		    if (pAngE->yaw > 180.0f)
		    {
		        pAngE->yaw -= 360.0f;
		    }
		    else if (pAngE->yaw < -180.0f)
		    {
		        pAngE->yaw += 360.0f;
		    }
		    // --- [End] 修改结束 ---
		}
#endif
		pAngE->pitch  =  asin(vecxZ)* RtA;

		pAngE->roll	= atan2f(vecyZ,veczZ) * RtA;	//PITCH

		NormAccz = pMpu->accX* vecxZ + pMpu->accY * vecyZ + pMpu->accZ * veczZ;	/*Z轴加速度*/
	}
}

float Q_rsqrt(float number)
{
	long i;
	float x2, y;
	const float threehalfs = 1.5F;

	x2 = number * 0.5F;
	y  = number;
	i  = * ( long * ) &y;
	i  = 0x5f3759df - ( i >> 1 );
	y  = * ( float * ) &i;
	y  = y * ( threehalfs - ( x2 * y * y ) );
	return y;
}
/* ... (保留上面的所有驱动代码，不要删除) ... */

// =============================================================================
// 适配 main.c 的应用层接口 (User Application Interface)
// =============================================================================

// 定义全局实例
static ICM45686_t g_imu_dev;
static _st_Mpu g_mpu_raw;
static _st_AngE g_angle_dat;

// 定义姿态解算的采样时间 (Delta Time)
// 注意：该值必须与 main.c 中 TIM7 的中断周期一致
// 如果 TIM7 是 1ms 中断一次，请改为 0.001f；如果是 5ms，请改为 0.005f
#define IMU_UPDATE_DT  0.02f

/**
 * @brief 初始化 IMU (main.c 调用)
 * 包括：结构体初始化、连接检查、量程配置
 */
void IMU_init(void) {
    // 1. 初始化结构体
    ICM45686_Init(&g_imu_dev);

    // 2. 检查连接 (尝试连接，若失败可以在此处加死循环或报错灯)
    if (ICM45686_Connection() != 0) {
        // 连接失败处理，例如：
        // printf("IMU Connect Failed!\r\n");
    }

    // 3. 配置加速度计
    // 推荐：低噪声模式, 量程 8G, ODR 200Hz
    ICM45686_AccelConfig(&g_imu_dev,
                         ICM45686_MODE_LOW_NOISE,
                         ICM45686_ACCEL_SCALE_08G,
                         ICM45686_ODR_RATE_0200HZ);

    // 4. 配置陀螺仪
    // 关键：量程必须设置为 1000DPS，以匹配 GetAngle 函数中的 Gyro_Gr (0.0005326) 参数
    ICM45686_GyroConfig(&g_imu_dev,
                        ICM45686_MODE_LOW_NOISE,
                        ICM45686_GYRO_SCALE_1000DPS,
                        ICM45686_ODR_RATE_0200HZ);

    // 也可以在此处对 g_angle_dat 进行清零
    memset(&g_angle_dat, 0, sizeof(g_angle_dat));
}

/**
 * @brief 获取欧拉角 (main.c 在定时器中调用)
 * @param ypr 返回的数组指针 [0]:Yaw, [1]:Pitch, [2]:Roll
 */
void IMU_getYawPitchRoll(float *ypr) {
    // 1. 读取原始数据 (ADC值)
    if (ICM45686_GetRawData(&g_imu_dev, &g_mpu_raw) == 0) {

        // 2. 执行姿态解算 (四元数/互补滤波)
        // 注意：传入预定义的采样时间 dt
        GetAngle(&g_mpu_raw, &g_angle_dat, IMU_UPDATE_DT);

        // 3. 输出结果到数组
        // 根据 main.c 的习惯，通常顺序是 Yaw, Pitch, Roll 或者 Pitch, Roll, Yaw
        // 这里按照常见的 YPR 顺序赋值
        ypr[0] = g_angle_dat.yaw;
        ypr[1] = g_angle_dat.pitch;
        ypr[2] = g_angle_dat.roll;
    }
}









