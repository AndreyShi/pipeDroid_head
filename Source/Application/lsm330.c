/*
 * lsm330.c
 *
 *  Created on: 26 ���. 2016 �.
 *      Author: real_bastard
 */
#include <math.h>
#include "stm32f4xx.h"
#include "main.h"
#include "lsm330.h"

#define I2C_TIMEOUT_MAX				10000
#define ACC							0
#define GYR							1

#define LSM330_ACC_SAD0L			(0x00)
#define LSM330_ACC_SAD0H			(0x02)
#define LSM330_ACC_I2C_SADROOT		(0x30)
#define LSM330_ACC_I2C_SAD_L		(LSM330_ACC_I2C_SADROOT| LSM330_ACC_SAD0L)
#define LSM330_ACC_I2C_SAD_H		(LSM330_ACC_I2C_SADROOT| LSM330_ACC_SAD0H)

#define LSM330_GYR_SAD0L			(0x00)
#define LSM330_GYR_SAD0H			(0x02)
#define LSM330_GYR_I2C_SADROOT		(0xD4)
#define LSM330_GYR_I2C_SAD_L		(LSM330_GYR_I2C_SADROOT| LSM330_GYR_SAD0L)
#define LSM330_GYR_I2C_SAD_H		(LSM330_GYR_I2C_SADROOT| LSM330_GYR_SAD0H)

#define G_MAX			23920640	/* ug */
#define	I2C_RETRY_DELAY		5		/* Waiting for signals [ms] */
#define	I2C_RETRIES		5		/* Number of retries */
#define	I2C_AUTO_INCREMENT	0x80		/* Autoincrement i2c address */

#define SENSITIVITY_2G		60		/* ug/LSB	*/
#define SENSITIVITY_4G		120		/* ug/LSB	*/
#define SENSITIVITY_6G		180		/* ug/LSB	*/
#define SENSITIVITY_8G		240		/* ug/LSB	*/
#define SENSITIVITY_16G		730		/* ug/LSB	*/

#define	LSM330_ACC_FS_MASK	(0x38)

/* Output Data Rates ODR */
#define	LSM330_ODR_MASK		0XF0
#define LSM330_PM_OFF		0x00		/* OFF */
#define	LSM330_ODR3_125		0x10		/*    3.125 Hz */
#define	LSM330_ODR6_25		0x20		/*    6.25  Hz */
#define	LSM330_ODR12_5		0x30		/*   12.5   Hz */
#define	LSM330_ODR25		0x40		/*   25     Hz */
#define	LSM330_ODR50		0x50		/*   50     Hz */
#define	LSM330_ODR100		0x60		/*  100     Hz */
#define	LSM330_ODR400		0x70		/*  400     Hz */
#define	LSM330_ODR800		0x80		/*  800     Hz */
#define	LSM330_ODR1600		0x90		/* 1600     Hz */

/* Registers configuration Mask and settings */
/* CTRLREG1 */
#define LSM330_INTEN_MASK		0x01
#define LSM330_INTEN_OFF		0x00
#define LSM330_INTEN_ON			0x01

/* CTRLREG2 */
#define LSM330_HIST1_MASK		0xE0
#define LSM330_SM1INT_PIN_MASK		0x08
#define LSM330_SM1INT_PINB		0x08
#define LSM330_SM1INT_PINA		0x00
#define LSM330_SM1_EN_MASK		0x01
#define LSM330_SM1_EN_ON		0x01
#define LSM330_SM1_EN_OFF		0x00
/* */

/* CTRLREG3 */
#define LSM330_HIST2_MASK		0xE0
#define LSM330_SM2INT_PIN_MASK		0x08
#define LSM330_SM2INT_PINB		0x08
#define LSM330_SM2INT_PINA		0x00
#define LSM330_SM2_EN_MASK		0x01
#define LSM330_SM2_EN_ON		0x01
#define LSM330_SM2_EN_OFF		0x00
/* */

/* CTRLREG4 */
#define LSM330_INT_ACT_MASK		(0x01 << 6)
#define LSM330_INT_ACT_H		(0x01 << 6)
#define LSM330_INT_ACT_L		0x00

#define LSM330_INT2_EN_MASK		(0x01 << 4)
#define LSM330_INT2_EN_ON		(0x01 << 4)
#define LSM330_INT2_EN_OFF		0x00

#define LSM330_INT1_EN_MASK		(0x01 << 3)
#define LSM330_INT1_EN_ON		(0x01 << 3)
#define LSM330_INT1_EN_OFF		0x00
/* */

#define	OUT_AXISDATA_REG		LSM330_OUTX_L
#define WHOAMI_LSM330_ACC		0x40	/* Expected content for WAI */

/*	CONTROL REGISTERS	*/
#define	LSM330_WHO_AM_I			0x0F	/* WhoAmI register Address */

#define	LSM330_OUTX_L			0x28	/* Output X LSByte */
#define	LSM330_OUTX_H			0x29	/* Output X MSByte */
#define	LSM330_OUTY_L			0x2A	/* Output Y LSByte */
#define	LSM330_OUTY_H			0x2B	/* Output Y MSByte */
#define	LSM330_OUTZ_L			0x2C	/* Output Z LSByte */
#define	LSM330_OUTZ_H			0x2D	/* Output Z MSByte */
#define	LSM330_LC_L			0x16	/* LSByte Long Counter Status */
#define	LSM330_LC_H			0x17	/* MSByte Long Counter Status */

#define	LSM330_STATUS_REG		0x27	/* Status */

#define	LSM330_CTRL_REG1		0x21	/* control reg 1 */
#define	LSM330_CTRL_REG2		0x22	/* control reg 2 */
#define	LSM330_CTRL_REG3		0x23	/* control reg 3 */
#define	LSM330_CTRL_REG4		0x20	/* control reg 4 */
#define	LSM330_CTRL_REG5		0x24	/* control reg 3 */
#define	LSM330_CTRL_REG6		0x25	/* control reg 4 */

#define	LSM330_OFF_X			0x10	/* Offset X Corr */
#define	LSM330_OFF_Y			0x11	/* Offset Y Corr */
#define	LSM330_OFF_Z			0x12	/* Offset Z Corr */

#define	LSM330_CS_X			0x13	/* Const Shift X */
#define	LSM330_CS_Y			0x14	/* Const Shift Y */
#define	LSM330_CS_Z			0x15	/* Const Shift Z */

#define	LSM330_VFC_1			0x1B	/* Vect Filter Coeff 1 */
#define	LSM330_VFC_2			0x1C	/* Vect Filter Coeff 2 */
#define	LSM330_VFC_3			0x1D	/* Vect Filter Coeff 3 */
#define	LSM330_VFC_4			0x1E	/* Vect Filter Coeff 4 */

/*	end CONTROL REGISTRES	*/

/* RESUME STATE INDICES */
#define	LSM330_RES_LC_L				0
#define	LSM330_RES_LC_H				1

#define	LSM330_RES_CTRL_REG1			2
#define	LSM330_RES_CTRL_REG2			3
#define	LSM330_RES_CTRL_REG3			4
#define	LSM330_RES_CTRL_REG4			5
#define	LSM330_RES_CTRL_REG5			6

#define	LSM330_RES_TIM4_1			20
#define	LSM330_RES_TIM3_1			21
#define	LSM330_RES_TIM2_1_L			22
#define	LSM330_RES_TIM2_1_H			23
#define	LSM330_RES_TIM1_1_L			24
#define	LSM330_RES_TIM1_1_H			25

#define	LSM330_RES_THRS2_1			26
#define	LSM330_RES_THRS1_1			27
#define	LSM330_RES_SA_1				28
#define	LSM330_RES_MA_1				29
#define	LSM330_RES_SETT_1			30

#define	LSM330_RES_TIM4_2			31
#define	LSM330_RES_TIM3_2			32
#define	LSM330_RES_TIM2_2_L			33
#define	LSM330_RES_TIM2_2_H			34
#define	LSM330_RES_TIM1_2_L			35
#define	LSM330_RES_TIM1_2_H			36

#define	LSM330_RES_THRS2_2			37
#define	LSM330_RES_THRS1_2			38
#define	LSM330_RES_DES_2			39
#define	LSM330_RES_SA_2				40
#define	LSM330_RES_MA_2				41
#define	LSM330_RES_SETT_2			42

#define	LSM330_RESUME_ENTRIES			43

#define	LSM330_STATE_PR_SIZE			16
/* end RESUME STATE INDICES */

/* STATE PROGRAMS ENABLE_stm CONTROLS */
#define	LSM330_SM1_DIS_SM2_DIS			0x00
#define	LSM330_SM1_DIS_SM2_EN			0x01
#define	LSM330_SM1_EN_SM2_DIS			0x02
#define	LSM330_SM1_EN_SM2_EN			0x03

/* INTERRUPTS ENABLE_stm CONTROLS */
#define	LSM330_INT1_DIS_INT2_DIS		0x00
#define	LSM330_INT1_DIS_INT2_EN			0x01
#define	LSM330_INT1_EN_INT2_DIS			0x02
#define	LSM330_INT1_EN_INT2_EN	0x03

//------------------------
/***************************************************
 LSM330 Registers for gyro and accelerometer
 ****************************************************/
#define WHO_AM_I 0x0F
#define CTRL_REG1 0x20
#define CTRL_REG2 0x21
#define CTRL_REG3 0x22
#define CTRL_REG4 0x23
#define CTRL_REG5 0x24
#define CTRL_REG6 0x25
#define REFERENCE_A 0x26
#define REFERENCE 0x25
#define OUT_TEMP 0x26
#define STATUS_REG 0x27
#define OUT_X_L 0x28
#define OUT_X_H 0x29
#define OUT_Y_L 0x2A
#define OUT_Y_H 0x2B
#define OUT_Z_L 0x2C
#define OUT_Z_H 0x2D
#define FIFO_CTRL_REG 0x2E
#define FIFO_SRC_REG 0x2F
#define INT1_CFG 0x30
#define INT1_SRC 0x31
#define INT1_TSH_XH 0x32
#define INT1_TSH_XL 0x33
#define INT1_TSH_YH 0x34
#define INT1_TSH_YL 0x35
#define INT1_TSH_ZH 0x36
#define INT1_TSH_ZL 0x37
#define INT1_DURATION 0x38


// Public gyro values
int16_t gyroX_; // in counts
int16_t gyroY_; // in counts
int16_t gyroZ_; // in counts
float scaledGyroX_; // in dps
float scaledGyroY_; // in dps
float scaledGyroZ_; // in dps

// Public accelerometer values
int16_t accelX_; // in counts
int16_t accelY_; // in counts
int16_t accelZ_; // in counts
float scaledAccelX_; // in g
float scaledAccelY_; // in g
float scaledAccelZ_; // in g

//   private:
uint8_t writeRegister(uint8_t sensor, uint8_t address, uint8_t data);
uint8_t readRegister(uint8_t sensor, uint8_t address);


void setupAccelerometer(int accelSensitivity);
void setupGyroscope(int gyroSensitivity);
// Accelerometer private methods
void setAccelSensitivity(int sensitivity);

// Gyro private methods
void setGyroSensitivity(int sensitivity);
void calculateGyroOffsets(void);
void calculateGyroThresholds(void);

// Gyro attributes
int gyroSensitivity_; // Gyro sensitivity and accuracy (in dps)
float gyroScale_; // Gyro multiplicator scale factor (in dps/count)
int gyroCtrlReg4Val_; // Gyro control register 4 value according to set sensitivity
float gyroOffsetX_; // in counts
float gyroOffsetY_; // in counts
float gyroOffsetZ_; // in counts
float gyroThresholdX_; // in counts
float gyroThresholdY_; // in counts
float gyroThresholdZ_; // in counts

// Accel attributes
int accelSensitivity_; // Accelerometer sensitivity and accuracy (in g)
float accelScale_; // Accelerometer multiplicator scale factor (in g/count)
int accelCtrlReg4Val_; // Accelerometer control register 4 value according to set sensitivity

/**
 * @brief  Enables the I2C Clock and configures the different GPIO ports.
 * @param  None
 * @retval None
 */
static void I2C_Config(void) {
	GPIO_InitTypeDef GPIO_InitStructure;
	I2C_InitTypeDef I2C_InitStruct;
	/* RCC Configuration */
	/*I2C Peripheral clock enable */
	RCC_APB1PeriphClockCmd(I2C_CLK, ENABLE_stm);

	/*SDA GPIO clock enable */
	RCC_AHB1PeriphClockCmd(I2C_SDA_GPIO_CLK, ENABLE_stm);

	/*SCL GPIO clock enable */
	RCC_AHB1PeriphClockCmd(I2C_SCL_GPIO_CLK, ENABLE_stm);

	/* Reset I2Cx IP */
	RCC_APB1PeriphResetCmd(I2C_CLK, ENABLE_stm);

	/* Release reset signal of I2Cx IP */
	RCC_APB1PeriphResetCmd(I2C_CLK, DISABLE_stm);

	/* GPIO Configuration */
	/*Configure I2C SCL pin */
	GPIO_InitStructure.GPIO_Pin = I2C_SCL_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(I2C_SCL_GPIO_PORT, &GPIO_InitStructure);

	/*Configure I2C SDA pin */
	GPIO_InitStructure.GPIO_Pin = I2C_SDA_PIN;
	GPIO_Init(I2C_SDA_GPIO_PORT, &GPIO_InitStructure);

	/* Connect PXx to I2C_SCL */
	GPIO_PinAFConfig(I2C_SCL_GPIO_PORT, I2C_SCL_SOURCE, I2C_SCL_AF);

	/* Connect PXx to I2C_SDA */
	GPIO_PinAFConfig(I2C_SDA_GPIO_PORT, I2C_SDA_SOURCE, I2C_SDA_AF);

	/* Configure I2C Filters */
	I2C_AnalogFilterCmd(I2C, ENABLE_stm);
	I2C_DigitalFilterConfig(I2C, 0x0F);

	/* I2C ENABLE_stm */
	I2C_Cmd(I2C, ENABLE_stm);

	/* Set the I2C structure parameters */
	I2C_InitStruct.I2C_Mode = I2C_Mode_I2C;
	I2C_InitStruct.I2C_DutyCycle = I2C_DutyCycle_2;
	I2C_InitStruct.I2C_OwnAddress1 = 0xFE;
	I2C_InitStruct.I2C_Ack = I2C_Ack_Enable;
	I2C_InitStruct.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	I2C_InitStruct.I2C_ClockSpeed = 100000;

	/* Initialize the I2C peripheral w/ selected parameters */
	I2C_Init(I2C1, &I2C_InitStruct);
}

/*
 * Setup lsm330 ipc device
 *
 * in  :
 * out :
 */
int lsm330_setup(int gyroSensitivity, int accelSensitivity) {
	I2C_Config();
	if (readRegister(GYR,WHO_AM_I)==0xFF)
		return 0;
	setupAccelerometer(accelSensitivity);
	//setupGyroscope(gyroSensitivity);
	return 1;
}

/*
 * Setup lsm330 accelerometer (spi communication)
 *
 * in  :
 * out :
 */
void setupAccelerometer(int accelSensitivity) {
	setAccelSensitivity(accelSensitivity);

	// Check the datasheet (p29)

	// Normal (1.344 kHz) / low-power mode (5.376 kHz) data rate
	// Normal mode selected (default)
	// x,y,z axis enabled (default)
	//writeRegister(ACC, CTRL_REG1, 0b10010111);
	writeRegister(ACC, CTRL_REG1, LSM330_ODR1600|0x07);
	// High-pass filter mode selection : Normal mode (reset reading HP_RESET_FILTER) (default)
	// High-pass filter cutoff frequency selection
	// Filtered data selection : internal filter bypassed (default)
	// High-pass filter enabled for CLICK function : filter bypassed
	// High-pass filter enabled for AOI function on interrupt 2 : filter bypassed
	// High-pass filter enabled for AOI function on interrupt 1 : filter bypassed
	writeRegister(ACC, CTRL_REG2, 0b00000000);

	// CLICK interrupt on INT1_A disabled (default)
	// AOI1 interrupt on INT1_A disabled (default)
	// DRDY1 interrupt on INT1_A disabled (default)
	// DRDY2 interrupt on INT1_A disabled (default)
	// FIFO watermark interrupt on INT1_A enabled
	// FIFO overrun interrupt on INT1_A disabled (default)
	writeRegister(ACC, CTRL_REG3, 0b00001000);

	// Continuous block data update (default)
	// Big/little endian data selection : Data LSB at lower address (default)
	// Normal mode enable
	// SPI serial interface mode selection : 4 wire interface (default)
	writeRegister(ACC, CTRL_REG4, accelCtrlReg4Val_);

	// normal mode, no reboot memory content (default)
	// FIFO enabled
	// interrupt request not latched (default)
	// 4D detection disabled
	writeRegister(ACC, CTRL_REG5, 0b01000000);
}

/*
 * Setup lsm330 gyroscope (spi communication)
 *
 * in  :
 * out :
 */
void setupGyroscope(int gyroSensitivity) {
	setGyroSensitivity(gyroSensitivity);

	//////////////////////////////////////////////////////////////////
	// Check the LSM330 datasheet to understand more clearly my setup
	//////////////////////////////////////////////////////////////////

	// Enable x, y, z and turn off power down:
	writeRegister(GYR, CTRL_REG1, 0b00001111);

	// If you'd like to adjust/use the HPF, you can edit the line below to configure CTRL_REG2:
	writeRegister(GYR, CTRL_REG2, 0b00000000);

	// Configure CTRL_REG3 to generate data ready interrupt on INT2
	// No interrupts used on INT1, if you'd like to configure INT1
	// or INT2 otherwise, consult the datasheet:
	writeRegister(GYR, CTRL_REG3, 0b00001000);

	// CTRL_REG4 controls the full-scale range, among other things:
	writeRegister(GYR, CTRL_REG4, gyroCtrlReg4Val_);

	// CTRL_REG5 controls high-pass filtering of outputs, use it
	// if you'd like:
	writeRegister(GYR, CTRL_REG5, 0b00000000);

	calculateGyroOffsets();
	calculateGyroThresholds();
}

/*
 * Set lsm330 gyroscope sensitivity
 *
 * in  : sensitivity (in dps)
 *
 * out :
 *
 * N.B.: Must only be called in gyro setup function
 */
void setGyroSensitivity(int sensitivity) {
	if (sensitivity == 500) {
		gyroSensitivity_ = sensitivity;
		gyroScale_ = 0.0175; // from the datasheet
		gyroCtrlReg4Val_ = 0x10;
	} else if (sensitivity == 2000) {
		gyroSensitivity_ = sensitivity;
		gyroScale_ = 0.070; // from the datasheet
		gyroCtrlReg4Val_ = 0x20;
	} else {
		// default value : 250 dps
		gyroSensitivity_ = 250;
		gyroScale_ = 0.00875; // from the datasheet
		gyroCtrlReg4Val_ = 0x00;
	}
}

/*
 * Set lsm330 accelerometer sensitivity
 *
 * in  : sensitivity (in g)
 *
 * out :
 *
 * N.B.: Must only be called in accelerometer setup function
 */
void setAccelSensitivity(int sensitivity) {
	if (sensitivity == 4) {
		accelSensitivity_ = sensitivity;
		accelScale_ = 0.002; // from the datasheet
		accelCtrlReg4Val_ = 0b00011000;
	} else if (sensitivity == 8) {
		accelSensitivity_ = sensitivity;
		accelScale_ = 0.004; // from the datasheet
		accelCtrlReg4Val_ = 0b00101000;
	} else if (sensitivity == 16) {
		accelSensitivity_ = sensitivity;
		accelScale_ = 0.00078; // manually calibrated
		accelCtrlReg4Val_ = 0b00111000;
	} else {
		// default value : 2 g
		accelSensitivity_ = 2;
		accelScale_ = 0.0000637; // manually calibrated
		accelCtrlReg4Val_ = 0b00001000;
	}
}

/**
 * @brief  Writes a byte at a specific  register
 * @param  Addr: register address.
 * @param  Data: Data to be written to the specific register
 * @retval 0x00 if write operation is OK.
 *       0xFF if timeout condition occured (device not connected or bus error).
 */
uint8_t writeRegister(uint8_t sensor, uint8_t address, uint8_t data) {
//uint8_t OV2640_WriteReg(uint16_t Addr, uint8_t Data)
	uint32_t timeout = I2C_TIMEOUT_MAX;
	uint8_t i2c_address;
	// This address is to tell the LSM330 that we're writing
	address &= 0x7F;

	/* Generate the Start Condition */
	I2C_GenerateSTART(I2C, ENABLE_stm);

	/* Test on I2C1 EV5 and clear it */
	timeout = I2C_TIMEOUT_MAX; /* Initialize timeout value */
	while (!I2C_CheckEvent(I2C, I2C_EVENT_MASTER_MODE_SELECT)) {
		/* If the timeout delay is exeeded, exit with error code */
		if ((timeout--) == 0)
			return 0xFF;
	}

	if (sensor == ACC)
		i2c_address = LSM330_ACC_I2C_SAD_L;
	if (sensor == GYR)
		i2c_address = LSM330_GYR_I2C_SAD_L;

	/* Send DCMI selcted device slave Address for write */
	I2C_Send7bitAddress(I2C, i2c_address, I2C_Direction_Transmitter);

	/* Test on I2C1 EV6 and clear it */
	timeout = I2C_TIMEOUT_MAX; /* Initialize timeout value */
	while (!I2C_CheckEvent(I2C, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)) {
		/* If the timeout delay is exeeded, exit with error code */
		if ((timeout--) == 0)
			return 0xFF;
	}

	/* Send I2C1 location address LSB */
	I2C_SendData(I2C, (uint8_t) (address));

	/* Test on I2C1 EV8 and clear it */
	timeout = I2C_TIMEOUT_MAX; /* Initialize timeout value */
	while (!I2C_CheckEvent(I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED)) {
		/* If the timeout delay is exeeded, exit with error code */
		if ((timeout--) == 0)
			return 0xFF;
	}

	/* Send Data */
	I2C_SendData(I2C, data);

	/* Test on I2C1 EV8 and clear it */
	timeout = I2C_TIMEOUT_MAX; /* Initialize timeout value */
	while (!I2C_CheckEvent(I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED)) {
		/* If the timeout delay is exeeded, exit with error code */
		if ((timeout--) == 0)
			return 0xFF;
	}

	/* Send I2C1 STOP Condition */
	I2C_GenerateSTOP(I2C, ENABLE_stm);

	/* If operation is OK, return 0 */
	return 0;
}

/**
 * @brief  Reads a byte from a specific  register
 * @param  Addr:  register address.
 * @retval data read from the specific register or 0xFF if timeout condition
 *         occured.
 */
uint8_t readRegister(uint8_t sensor, uint8_t address) {
	uint32_t timeout = I2C_TIMEOUT_MAX;
	uint8_t Data = 0;
	uint8_t i2c_address;

	// This address is to tell the LSM330 that we're reading
	address |= 0x80;
	/* Generate the Start Condition */
	I2C_GenerateSTART(I2C, ENABLE_stm);

	/* Test on I2C1 EV5 and clear it */
	timeout = I2C_TIMEOUT_MAX; /* Initialize timeout value */
	while (!I2C_CheckEvent(I2C, I2C_EVENT_MASTER_MODE_SELECT)) {
		/* If the timeout delay is exeeded, exit with error code */
		if ((timeout--) == 0){
			/*if ((I2C->SR1&I2C_EVENT_SLAVE_ACK_FAILURE)!=0){
				I2C->SR1&=~I2C_EVENT_SLAVE_ACK_FAILURE;

			}
			I2C_GenerateSTOP(I2C, ENABLE_stm);*/
			return 0xFF;
		}
	}
	if (sensor == ACC)
		i2c_address = LSM330_ACC_I2C_SAD_L;
	if (sensor == GYR)
		i2c_address = LSM330_GYR_I2C_SAD_L;

	/* Send DCMI selcted device slave Address for write */
	I2C_Send7bitAddress(I2C, i2c_address, I2C_Direction_Transmitter);

	/* Test on I2C1 EV6 and clear it */
	timeout = I2C_TIMEOUT_MAX; /* Initialize timeout value */
	while (!I2C_CheckEvent(I2C, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)) {
		/* If the timeout delay is exeeded, exit with error code */
		if ((timeout--) == 0)
			return 0xFF;
	}

	/* Send I2C1 location address LSB */
	I2C_SendData(I2C, (uint8_t) (address));

	/* Test on I2C1 EV8 and clear it */
	timeout = I2C_TIMEOUT_MAX; /* Initialize timeout value */
	while (!I2C_CheckEvent(I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED)) {
		/* If the timeout delay is exeeded, exit with error code */
		if ((timeout--) == 0)
			return 0xFF;
	}

	/* Clear AF flag if arised */
	I2C1->SR1 |= (uint16_t) 0x0400;

	/* Generate the Start Condition */
	I2C_GenerateSTART(I2C, ENABLE_stm);

	/* Test on I2C1 EV6 and clear it */
	timeout = I2C_TIMEOUT_MAX; /* Initialize timeout value */
	while (!I2C_CheckEvent(I2C, I2C_EVENT_MASTER_MODE_SELECT)) {
		/* If the timeout delay is exeeded, exit with error code */
		if ((timeout--) == 0)
			return 0xFF;
	}

	/* Send DCMI selcted device slave Address for write */
	I2C_Send7bitAddress(I2C, i2c_address, I2C_Direction_Receiver);

	/* Test on I2C1 EV6 and clear it */
	timeout = I2C_TIMEOUT_MAX; /* Initialize timeout value */
	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED)) {
		/* If the timeout delay is exeeded, exit with error code */
		if ((timeout--) == 0)
			return 0xFF;
	}

	/* Prepare an NACK for the next data received */
	I2C_AcknowledgeConfig(I2C, DISABLE_stm);

	/* Test on I2C1 EV7 and clear it */
	timeout = I2C_TIMEOUT_MAX; /* Initialize timeout value */
	while (!I2C_CheckEvent(I2C, I2C_EVENT_MASTER_BYTE_RECEIVED)) {
		/* If the timeout delay is exeeded, exit with error code */
		if ((timeout--) == 0)
			return 0xFF;
	}

	/* Prepare Stop after receiving data */
	I2C_GenerateSTOP(I2C, ENABLE_stm);

	/* Receive the Data */
	Data = I2C_ReceiveData(I2C);

	/* return the read data */
	return Data;
}

/*
 * Calculate Gyro offsets for better data reading later
 *
 * in  :
 * out :
 *
 * WARNING: The gyro must not be in movement when calculating the offsets
 */
void calculateGyroOffsets(void) {
	int16_t totalGx = 0;
	int16_t totalGy = 0;
	int16_t totalGz = 0;

	int nbIterations = 100;

	for (int i = 0; i < nbIterations; ++i) {
		getGyroValues();

		totalGx += gyroX_;
		totalGy += gyroY_;
		totalGz += gyroZ_;

		Delay_ms(10);
	}

	gyroOffsetX_ = (float) totalGx / (float) nbIterations;
	gyroOffsetY_ = (float) totalGy / (float) nbIterations;
	gyroOffsetZ_ = (float) totalGz / (float) nbIterations;
}

/*
 * Calculate Gyro thresholds for better data reading later
 *
 * in  :
 * out :
 *
 * WARNING: The gyro must not be in movement when calculating the offsets
 */
void calculateGyroThresholds(void) {
	int16_t totalGx = 0;
	int16_t totalGy = 0;
	int16_t totalGz = 0;

	int nbIterations = 100;

	for (int i = 0; i < nbIterations; ++i) {
		getGyroValues();

		totalGx = fabs((float) gyroX_ - (float) gyroOffsetX_);
		totalGy = fabs((float) gyroY_ - (float) gyroOffsetY_);
		totalGz = fabs((float) gyroZ_ - (float) gyroOffsetZ_);

		if (totalGx > gyroThresholdX_) {
			gyroThresholdX_ = totalGx;
		}
		if (totalGy > gyroThresholdY_) {
			gyroThresholdY_ = totalGy;
		}
		if (totalGz > gyroThresholdZ_) {
			gyroThresholdZ_ = totalGz;
		}

		Delay_ms(10);
	}
}

///////////////////////////////////////////////////////////
// Get gyroscope values function
///////////////////////////////////////////////////////////
void getGyroValues(void) {
	gyroX_ = (readRegister(GYR, 0x29) & 0xFF) << 8;
	gyroX_ |= (readRegister(GYR, 0x28) & 0xFF);

	gyroY_ = (readRegister(GYR, 0x2B) & 0xFF) << 8;
	gyroY_ |= (readRegister(GYR, 0x2A) & 0xFF);

	gyroZ_ = (readRegister(GYR, 0x2D) & 0xFF) << 8;
	gyroZ_ |= (readRegister(GYR, 0x2C) & 0xFF);

	float deltaGx = (float) gyroX_ - (float) gyroOffsetX_;
	float deltaGy = (float) gyroY_ - (float) gyroOffsetY_;
	float deltaGz = (float) gyroZ_ - (float) gyroOffsetZ_;

	// scale gyro values
	if (fabs(deltaGx) < gyroThresholdX_) {
		scaledGyroX_ = 0;
	} else {
		scaledGyroY_ = deltaGx * gyroScale_;
	}
	if (fabs(deltaGy) < gyroThresholdY_) {
		scaledGyroY_ = 0;
	} else {
		scaledGyroY_ = deltaGy * gyroScale_;
	}
	if (fabs(deltaGz) < gyroThresholdZ_) {
		scaledGyroZ_ = 0;
	} else {
		scaledGyroZ_ = deltaGz * gyroScale_;
	}
}

///////////////////////////////////////////////////////////
// Get accelerometer values function
///////////////////////////////////////////////////////////
void getAccelValues(void) {
	accelX_ = (readRegister(ACC, 0x29) & 0xFF) << 8;
	accelX_ |= (readRegister(ACC, 0x28) & 0xFF);

	accelY_ = (readRegister(ACC, 0x2B) & 0xFF) << 8;
	accelY_ |= (readRegister(ACC, 0x2A) & 0xFF);

	accelZ_ = (readRegister(ACC, 0x2D) & 0xFF) << 8;
	accelZ_ |= (readRegister(ACC, 0x2C) & 0xFF);

	scaledAccelX_ = (float) accelX_ * accelScale_;
	scaledAccelY_ = (float) accelY_ * accelScale_;
	scaledAccelZ_ = (float) accelZ_ * accelScale_;
}

