/*
 * lsm330.h
 *
 *  Created on: 26 ���. 2016 �.
 *      Author: real_bastard
 */

#ifndef LSM330_H_
#define LSM330_H_

/********************************************************************************

 SYSFS interface
 - range: set full scale
 -> accelerometer: 	2,4,6,8,16 				[g]
 -> gyroscope:		250,500,2000				[dps]
 - pollrate_ms: set 1/ODR
 -> accelerometer:	LSM330_ACC_MIN_POLL_PERIOD_MS < t	[ms]
 -> gyroscope:		LSM330_GYR_MIN_POLL_PERIOD_MS < t	[ms]
 - enable_device: enable/disable sensor					[1/0]
 INPUT subsystem: NOTE-> output data INCLUDE the sensitivity in accelerometer,
 but NOT INCLUDE the sensitivity in gyroscope.
 - accelerometer:	abs_x, abs_y, abs_z		[ug]
 - gyroscope:		abs_x, abs_y, abs_z		[raw data]
 *******************************************************************************/


/* Poll Interval */
#define	LSM330_ACC_MIN_POLL_PERIOD_MS		1

#define LSM330_GYR_MIN_POLL_PERIOD_MS		2

/* Accelerometer Sensor Full Scale */
#define LSM330_ACC_G_2G				(0x00)
#define LSM330_ACC_G_4G				(0x08)
#define LSM330_ACC_G_6G				(0x10)
#define LSM330_ACC_G_8G				(0x18)
#define LSM330_ACC_G_16G			(0x20)

/* Gyroscope Sensor Full Scale */
#define LSM330_GYR_FS_250DPS			(0x00)
#define LSM330_GYR_FS_500DPS			(0x10)
#define LSM330_GYR_FS_2000DPS			(0x30)

// setup methods
int lsm330_setup(int gyroSensitivity, int accelSensitivity);


void getGyroValues(void);

void getAccelValues(void);

// Public gyro values
extern int16_t gyroX_; // in counts
extern int16_t gyroY_; // in counts
extern int16_t gyroZ_; // in counts
extern float scaledGyroX_; // in dps
extern float scaledGyroY_; // in dps
extern float scaledGyroZ_; // in dps

// Public accelerometer values
extern int16_t accelX_; // in counts
extern int16_t accelY_; // in counts
extern int16_t accelZ_; // in counts
extern float scaledAccelX_; // in g
extern float scaledAccelY_; // in g
extern float scaledAccelZ_; // in g

#endif /* LSM330_H_ */
