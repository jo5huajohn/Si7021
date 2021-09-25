/*
 * Si7021.h
 *
 *  Created on: 9 Jul 2021
 *      Author: joshua
 */

#ifndef INC_SI7021_H_
#define INC_SI7021_H_

#include "stm32f4xx_hal.h"

typedef enum
{
	HOLD = 0,
	NO_HOLD,
	PREVIOUS_TEMP
} measurementmode_t;

typedef enum
{
	RES_RH12_TEMP14 = 0,
	RES_RH8_TEMP12,
	RES_RH10_TEMP13,
	RES_RH11_TEMP11
} resolution_t;

typedef enum
{
	VDD_OK = 0,
	VDD_LOW
} vddstatus_t;

typedef enum
{
	HEATER_DISABLE = 0,
	HEATER_ENABLE
} heater_t;

typedef enum
{
	MA_309 = 0,
	MA_918,
	MA_1524,
	MA_2739,
	MA_5169,
	MA_9420

} heatercurrent_t;

typedef enum
{
	ENG_SAMPLE = 0,
	SI7013,
	SI7020,
	SI7021,
	NOT_DEFINED = -1
} device_t;

typedef enum
{
	V1 = 1,
	V2 = 2,
	UNK = -1
} firmware_t;

/* I2C SLAVE ADDRESS R/W */

#define SI7021_I2C_SLAVE_ADDR					0x40 << 1U

/* SI7021 I2C COMMANDS */

#define SI7021_CMD_RELATIVE_HUMIDITY_HOLD		0xE5
#define SI7021_CMD_RELATIVE_HUMIDITY_NO_HOLD	0xF5
#define SI7021_CMD_TEMPERATURE_HOLD				0xE3
#define SI7021_CMD_TEMPERATURE_NO_HOLD			0xF3
#define SI7021_CMD_PREV_TEMP 					0xE0
#define SI7021_CMD_RESET						0xFE
#define SI7021_CMD_WRITE_USR_REG_1				0xE6
#define SI7021_CMD_READ_USR_REG_1				0xE7
#define SI7021_CMD_WRITE_HTR_CTRL_REG			0x51
#define SI7021_CMD_READ_HTR_CTRL_REG			0x11
#define SI7021_CMD_READ_ELECTRONIC_ID_BYTE_1_0	0xFA
#define SI7021_CMD_READ_ELECTRONIC_ID_BYTE_1_1	0x0F
#define SI7021_CMD_READ_ELECTRONIC_ID_BYTE_2_0	0xFC
#define SI7021_CMD_READ_ELECTRONIC_ID_BYTE_2_1	0xC9
#define SI7021_CMD_READ_FIRMWARE_REV_0			0x84
#define SI7021_CMD_READ_FIRMWARE_REV_1			0XB8

/* USER REGISTER 1 */

#define SI7021_USR_REG_1_D7                     0x1U << 7U		/* 0b10000000 */
#define SI7021_USR_REG_1_D6                     0x1U << 6U		/* 0b01000000 */
#define SI7021_USR_REG_1_D2                     0x1U << 2U		/* 0b00000100 */
#define SI7021_USR_REG_1_D0	                    0x1U << 0U		/* 0b00000001 */

/* HEATER CONTROL REGISTER */

#define SI7021_HEATER_CONTROL_REG_D3            0x1U << 3U      /* 0b00001000 */
#define SI7021_HEATER_CONTROL_REG_D2            0x1U << 2U      /* 0b00000100 */
#define SI7021_HEATER_CONTROL_REG_D1            0x1U << 1U      /* 0b00000010 */
#define SI7021_HEATER_CONTROL_REG_D0            0x1U << 0U      /* 0b00000001 */

/* FUNCTIONS */

float Si7021_Read_Rel_Humidity(I2C_HandleTypeDef *hi2c, measurementmode_t measurement_mode);
float Si7021_Read_Temp_C(I2C_HandleTypeDef *hi2c, measurementmode_t measurement_mode);

HAL_StatusTypeDef Si7021_Reset(I2C_HandleTypeDef *hi2c);

HAL_StatusTypeDef Si7021_Write_RH_Temp_Resolution(I2C_HandleTypeDef *hi2c, uint8_t resolution);
HAL_StatusTypeDef Si7021_Write_Heater_State(I2C_HandleTypeDef *hi2c, uint8_t state);

resolution_t Si7021_Read_RH_Temp_Resolution(I2C_HandleTypeDef *hi2c);
vddstatus_t Si7021_Read_Vdd_Status(I2C_HandleTypeDef *hi2c);
heater_t Si7021_Read_Heater_Status(I2C_HandleTypeDef *hi2c);

HAL_StatusTypeDef Si7021_Write_Heater_Current(I2C_HandleTypeDef *hi2c, heatercurrent_t current);
heatercurrent_t Si7021_Read_Heater_Current(I2C_HandleTypeDef *hi2c);

uint64_t Si7021_Read_Electronic_Serial_Number(I2C_HandleTypeDef *hi2c);
device_t Si7021_Read_Device(I2C_HandleTypeDef *hi2c);

firmware_t Si7021_Read_Firmware_Revision(I2C_HandleTypeDef *hi2c);

#endif /* INC_SI7021_H_ */
