/************************************************************
  Copyright (C), 1988-1999, Huawei Tech. Co., Ltd.
  FileName: board_sensors.h
  Author: hantao(00185954)       Version : 0.1      Date:  2011-07-11
  Description:	.h file for sensors
  Version:
  Function List:
  History:
  <author>  <time>   <version >   <desc>
***********************************************************/
/*==============================================================================
History

Problem NO.         Name        Time         Reason

==============================================================================*/

#ifndef	__BOARD_SENSORS_H__
#define	__BOARD_SENSORS_H__

/* zkf55108 2011/10/26 add begin */
/*Device Id Name*/
#define AKM8975C_I2C_NAME		"akm8975"
#define	LIS3DH_I2C_NAME			"lis3dh_acc"
#define	L3G4200D_I2C_NAME		"l3g4200d_gyr"
#define	TMD2771_I2C_NAME		"tmd2771"
#define APDS990x_I2C_NAME		"apds990x"
#define MMA8452_I2C_NAME		"mma8452"
#define ADXL34X_I2C_NAME		"adxl34x"

#define KXTIK_I2C_NAME           "kxtik"

/*Slave Adress*/
#define	AKM8975C_I2C_ADDR		0x0c
#define	LIS3DH_I2C_ADDR			0x18
#define APDS990x_I2C_ADDR		0x39
#define MMA8452_I2C_ADDR		0x1C
#define L3G4200D_I2C_ADDR		0x68
#define TAOS_DEVICE_ADDR1		0x29
#define TAOS_DEVICE_ADDR2		0x39
#define TAOS_DEVICE_ADDR3		0x49
#define TMD2771_I2C_ADDR		TAOS_DEVICE_ADDR2
#define ADXL34X_I2C_ADDR		0x53
#define KXTIC_I2C_ADDR          0x0F

/*Input Device Name*/
#ifndef ACCL_INPUT_DEV_NAME
#define	ACCL_INPUT_DEV_NAME		"acc_input"
#endif
#ifndef COMPASS_INPUT_DEV_NAME
#define	COMPASS_INPUT_DEV_NAME	"akm_input"
#endif
#ifndef GYRO_INPUT_DEV_NAME
#define	GYRO_INPUT_DEV_NAME		"gyro_input"
#endif
#ifndef LIGHT_INPUT_DEV_NAME
#define LIGHT_INPUT_DEV_NAME	"als_input"
#endif
#ifndef PROX_INPUT_DEV_NAME
#define PROX_INPUT_DEV_NAME		"ps_input"
#endif

enum input_name {
	ACC,
	AKM,
	GYRO,
	ALS,
	PS,
	PO188,
	SENSOR_MAX
};

int set_sensor_input(enum input_name name, const char *input_num);
/* zkf55108 2011/10/26 add end */
#endif
