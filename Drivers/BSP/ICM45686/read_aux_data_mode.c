#include <stdio.h>
#include "inv_imu_driver.h"
#include "main.h"

#define ICM_USE_HARD_SPI
#include "SEGGER_RTT.h"
#include "spi.h"

#define UI_I2C  0 /**< identifies I2C interface. */
#define UI_SPI1 1 /**< identifies 4-wire SPI interface. */


#define INV_MSG(level,msg, ...) 	      RTT_Log("%d," msg "\r\n", __LINE__, ##__VA_ARGS__)

/* --- DWT Implementation Start --- */
/* 确保 SystemCoreClock 可见 (通常在 main.h 或 stm32xxx_hal.h 中定义) */
extern uint32_t SystemCoreClock;

#ifndef DWT_CTRL
#define DWT_CTRL   (*(volatile uint32_t *)0xE0001000)
#define DWT_CYCCNT (*(volatile uint32_t *)0xE0001004)
#define DEMCR      (*(volatile uint32_t *)0xE000EDFC)
#define TRCENA     (1 << 24)
#define CYCCNTENA  (1 << 0)
#endif

static void dwt_init(void)
{
    DEMCR |= TRCENA;       /* 开启 Trace 功能 */
    DWT_CYCCNT = 0;        /* 清除计数器 */
    DWT_CTRL |= CYCCNTENA; /* 开启周期计数器 */
}

static void dwt_delay_us(uint32_t us)
{
    uint32_t start_tick = DWT_CYCCNT;
    /* 计算需要的时钟周期数 */
    uint32_t ticks = us * (SystemCoreClock / 1000000);
    while ((DWT_CYCCNT - start_tick) < ticks);
}

static void dwt_delay_ms(uint32_t ms)
{
    /* 循环调用 1ms 的延时，避免在大延时情况下 ticks 计算溢出 */
    while (ms--)
    {
        dwt_delay_us(1000);
    }
}
/* --- DWT Implementation End --- */

static inv_imu_device_t  imu_dev; /* Driver structure */

int si_print_error_if_any(int rc);

/* 修改: 将宏定义中的 delay_ms 替换为 dwt_delay_ms */
#define SI_CHECK_RC(rc)                                                                            \
	do {                                                                                           \
		if (si_print_error_if_any(rc)) {                                                           \
			INV_MSG(INV_MSG_LEVEL_ERROR, "At %s (line %d)", __FILE__, __LINE__);                   \
			dwt_delay_ms(100);                                                                   \
			return rc;                                                                             \
		}                                                                                          \
	} while (0)

/*
 * Error codes
 */
int si_print_error_if_any(int rc)
{
	if (rc != 0) {
		switch (rc) {
		case INV_IMU_ERROR:
			RTT_Log("Unspecified error (%d)", rc);
			break;
		case INV_IMU_ERROR_TRANSPORT:
			RTT_Log("Error occurred at transport level (%d)", rc);
			break;
		case INV_IMU_ERROR_TIMEOUT:
			RTT_Log("Action did not complete in the expected time window (%d)",rc);
			break;
		case INV_IMU_ERROR_BAD_ARG:
			RTT_Log("Invalid argument provided (%d)", rc);
			break;
		case INV_IMU_ERROR_EDMP_BUF_EMPTY:
			RTT_Log("EDMP buffer is empty (%d)", rc);
			break;
		default:
			RTT_Log("Unknown error (%d)", rc);
			break;
		}
	}

	return rc;
}
/*******************************************************************************
* 名    称： icm42688_read_regs
* 功    能： 连续读取多个寄存器的值
* 入口参数： reg: 起始寄存器地址 *buf数据指针,uint16_t len长度
* 出口参数： 无
* 作　　者： Baxiange
* 创建日期： 2024-07-25
* 修    改：
* 修改日期：
* 备    注： 使用SPI读取寄存器时要注意:最高位为读写位，详见datasheet page50.
*******************************************************************************/
static int icm45686_read_regs(uint8_t reg, uint8_t* buf, uint32_t len)
{
    uint8_t regval = 0;
#if defined(ICM_USE_HARD_SPI)
    reg |= 0x80;
    HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);
    /* 写入要读的寄存器地址 */
    HAL_SPI_TransmitReceive(&hspi1, &reg, &regval, 1, 1000);
    /* 读取寄存器数据 */
    while(len)
	{
		HAL_SPI_TransmitReceive(&hspi1, &reg, buf, 1, 1000);
		len--;
		buf++;
	}
    HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);
#elif defined(ICM_USE_I2C)
	IIC_Read_nByte(ICM_I2C_ADDR, reg, len, buf);
#endif
	return 0;
}

static uint8_t io_write_reg(uint8_t reg, uint8_t value)
{
    uint8_t regval = 0;
#if defined(ICM_USE_HARD_SPI)
    HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);
    /* 写入要读的寄存器地址 */
    HAL_SPI_TransmitReceive(&hspi1, &reg, &regval, 1, 1000);
    /* 读取寄存器数据 */
    HAL_SPI_TransmitReceive(&hspi1, &value, &regval, 1, 1000);
    HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);
#elif defined(ICM_USE_I2C)
	IIC_Write_nByte(ICM_I2C_ADDR, reg, 1, &value);
#endif
    return 0;
}

static int icm45686_write_regs(uint8_t reg, const uint8_t* buf, uint32_t len)
{
	int rc;

	for (uint32_t i = 0; i < len; i++)
	{
		rc = io_write_reg(reg + i, buf[i]);
		if (rc)
			return rc;
	}
	return 0;
}

/* Initializes IMU device and apply configuration. */
int setup_imu(int use_ln, int accel_en, int gyro_en)
{
	int                      rc     = 0;
	uint8_t                  whoami = 0;
	inv_imu_int_pin_config_t int_pin_config;
	inv_imu_int_state_t      int_config;
	int retrycount = 0;

    /* --- 新增: 初始化 DWT 计数器 --- */
    dwt_init();

	imu_dev.transport.read_reg   = icm45686_read_regs;
	imu_dev.transport.write_reg  = icm45686_write_regs;
    /* --- 修改: 将 delay_us 改为 dwt_delay_us --- */
	imu_dev.transport.sleep_us   = dwt_delay_us;

#if defined(ICM_USE_HARD_SPI)
	/* Init transport layer */
	imu_dev.transport.serif_type = UI_SPI1;

#endif
#if defined(ICM_USE_I2C)
	imu_dev.transport.serif_type = UI_I2C;
	IIC_Init();
#endif
	/* Wait 3 ms to ensure device is properly supplied  */
    /* --- 修改: delay_ms(3) -> dwt_delay_ms(3) --- */
	dwt_delay_ms(3);

	/* In SPI, configure slew-rate to prevent bus corruption on DK-SMARTMOTION-REVG */
	if (imu_dev.transport.serif_type == UI_SPI3 || imu_dev.transport.serif_type == UI_SPI4) {
		drive_config0_t drive_config0;
		drive_config0.pads_spi_slew = DRIVE_CONFIG0_PADS_SPI_SLEW_TYP_10NS;
		rc |= inv_imu_write_reg(&imu_dev, DRIVE_CONFIG0, 1, (uint8_t *)&drive_config0);
		SI_CHECK_RC(rc);
        /* --- 修改: delay_us(2) -> dwt_delay_us(2) --- */
		dwt_delay_us(2); /* Takes effect 1.5 us after the register is programmed */
	}

	/* Check whoami */
	rc |= inv_imu_get_who_am_i(&imu_dev, &whoami);
	SI_CHECK_RC(rc);
	while(whoami != INV_IMU_WHOAMI)
	{
		rc |= inv_imu_get_who_am_i(&imu_dev, &whoami);
		SI_CHECK_RC(rc);
        /* --- 修改: delay_ms(50) -> dwt_delay_ms(50) --- */
		dwt_delay_ms(50);
		retrycount++;
		if (retrycount > 5)
		{
			break;
		}
	}

	if (whoami != INV_IMU_WHOAMI) {
		INV_MSG(INV_MSG_LEVEL_ERROR, "Erroneous WHOAMI value.");
		INV_MSG(INV_MSG_LEVEL_ERROR, "  - Read 0x%02x", whoami);
		INV_MSG(INV_MSG_LEVEL_ERROR, "  - Expected 0x%02x", INV_IMU_WHOAMI);
		return -1;
	}

	rc |= inv_imu_soft_reset(&imu_dev);
	SI_CHECK_RC(rc);

	/*
	 * Configure interrupts pins
	 * - Polarity High
	 * - Pulse mode
	 * - Push-Pull drive
	 */
	int_pin_config.int_polarity = INTX_CONFIG2_INTX_POLARITY_HIGH;
	int_pin_config.int_mode     = INTX_CONFIG2_INTX_MODE_PULSE;
	int_pin_config.int_drive    = INTX_CONFIG2_INTX_DRIVE_PP;
	rc |= inv_imu_set_pin_config_int(&imu_dev, INV_IMU_INT1, &int_pin_config);
	SI_CHECK_RC(rc);

	/* Interrupts configuration */
	memset(&int_config, INV_IMU_DISABLE, sizeof(int_config));
	int_config.INV_UI_DRDY = INV_IMU_ENABLE;
	rc |= inv_imu_set_config_int(&imu_dev, INV_IMU_INT1, &int_config);
	SI_CHECK_RC(rc);

	/* Set FSR */
	rc |= inv_imu_set_accel_fsr(&imu_dev, ACCEL_CONFIG0_ACCEL_UI_FS_SEL_4_G);
	rc |= inv_imu_set_gyro_fsr(&imu_dev, GYRO_CONFIG0_GYRO_UI_FS_SEL_1000_DPS);
	SI_CHECK_RC(rc);

	/* Set ODR */
	rc |= inv_imu_set_accel_frequency(&imu_dev, ACCEL_CONFIG0_ACCEL_ODR_200_HZ);
	rc |= inv_imu_set_gyro_frequency(&imu_dev, GYRO_CONFIG0_GYRO_ODR_200_HZ);
	SI_CHECK_RC(rc);

	/* Set BW = ODR/4 */
	rc |= inv_imu_set_accel_ln_bw(&imu_dev, IPREG_SYS2_REG_131_ACCEL_UI_LPFBW_DIV_4);
	rc |= inv_imu_set_gyro_ln_bw(&imu_dev, IPREG_SYS1_REG_172_GYRO_UI_LPFBW_DIV_4);
	SI_CHECK_RC(rc);

	/* Sensor registers are not available in ULP, so select RCOSC clock to use LP mode. */
	rc |= inv_imu_select_accel_lp_clk(&imu_dev, SMC_CONTROL_0_ACCEL_LP_CLK_RCOSC);
	SI_CHECK_RC(rc);

	/* Set power modes */
	if (use_ln) {
		if (accel_en)
			rc |= inv_imu_set_accel_mode(&imu_dev, PWR_MGMT0_ACCEL_MODE_LN);
		if (gyro_en)
			rc |= inv_imu_set_gyro_mode(&imu_dev, PWR_MGMT0_GYRO_MODE_LN);
	} else {
		if (accel_en)
			rc |= inv_imu_set_accel_mode(&imu_dev, PWR_MGMT0_ACCEL_MODE_LP);
		if (gyro_en)
			rc |= inv_imu_set_gyro_mode(&imu_dev, PWR_MGMT0_GYRO_MODE_LP);
	}

	SI_CHECK_RC(rc);

	return rc;
}

int bsp_IcmGetRawData(float accel_mg[3], float gyro_dps[3], float *temp_degc)
{
	int rc = 0;
	inv_imu_sensor_data_t d;

	rc |= inv_imu_get_register_data(&imu_dev, &d);
	SI_CHECK_RC(rc);

	accel_mg[0] = (float)((d.accel_data[0] * 4 /* mg */) / 32.768);
	accel_mg[1] = (float)((d.accel_data[1] * 4 /* mg */) / 32.768);
	accel_mg[2] = (float)((d.accel_data[2] * 4 /* mg */) / 32.768);
	gyro_dps[0] = (float)((d.gyro_data[0] * 1000 /* dps */) / 32768.0);
	gyro_dps[1] = (float)((d.gyro_data[1] * 1000 /* dps */) / 32768.0);
	gyro_dps[2] = (float)((d.gyro_data[2] * 1000 /* dps */) / 32768.0);
	*temp_degc  = (float)(25 + (d.temp_data / 128.0));
	return 0;
}