/*
 * Si7021.c
 *
 *  Created on: 9 Jul 2021
 *      Author: joshua
 */

#include "Si7021.h"

uint8_t si7021_status = 0;
uint8_t cmd[2] = { };
uint8_t si7021_buff[2] = { };
uint8_t si7021_elec_id[8] = { };

float Si7021_Read_Rel_Humidity(I2C_HandleTypeDef *hi2c, measurementmode_t measurement_mode)
{
    static float rh_code = 0.0;

    if (HOLD == measurement_mode)
    {
        cmd[0] = SI7021_CMD_RELATIVE_HUMIDITY_HOLD;
        HAL_I2C_Master_Transmit(hi2c, SI7021_I2C_SLAVE_ADDR, &cmd[0], 1, 100);
    }

    else if (NO_HOLD == measurement_mode)
    {
        cmd[0] = SI7021_CMD_RELATIVE_HUMIDITY_NO_HOLD;
        HAL_I2C_Master_Transmit(hi2c, SI7021_I2C_SLAVE_ADDR, &cmd[0], 1, 100);
        HAL_Delay(20);
    }

    else
    {
        return -1;
    }

    HAL_I2C_Master_Receive(hi2c, SI7021_I2C_SLAVE_ADDR, si7021_buff, 2, 100);
    rh_code = (125 * ((si7021_buff[0] << 8) | si7021_buff[1]) / 65536) - 6;

        return rh_code;
}

float Si7021_Read_Temp_C(I2C_HandleTypeDef *hi2c, measurementmode_t measurement_mode)
{
    static float temp_code = 0.0;

    if (HOLD == measurement_mode)
    {
        cmd[0] = SI7021_CMD_TEMPERATURE_HOLD;
        HAL_I2C_Master_Transmit(hi2c, SI7021_I2C_SLAVE_ADDR, &cmd[0], 1, 100);
    }

    else if (NO_HOLD == measurement_mode)
    {
        cmd[0] = SI7021_CMD_TEMPERATURE_NO_HOLD;
        HAL_I2C_Master_Transmit(hi2c, SI7021_I2C_SLAVE_ADDR, &cmd[0], 1, 100);
        HAL_Delay(10);
    }

    else if (PREVIOUS_TEMP == measurement_mode)
    {
        cmd[0] = SI7021_CMD_PREV_TEMP;
        HAL_I2C_Master_Transmit(hi2c, SI7021_I2C_SLAVE_ADDR, &cmd[0], 1, 100);
    }

    else
    {
        return -1;
    }

    HAL_I2C_Master_Receive(hi2c, SI7021_I2C_SLAVE_ADDR, si7021_buff, 2, 100);
    temp_code = (175.72 * ((si7021_buff[0] << 8) | si7021_buff[1]) / 65536) - 46.85;

    return temp_code;
}

HAL_StatusTypeDef Si7021_Reset(I2C_HandleTypeDef *hi2c)
{
	cmd[0] = SI7021_CMD_RESET;

	si7021_status = HAL_I2C_Master_Transmit(hi2c, SI7021_I2C_SLAVE_ADDR, &cmd[0], 1, 100);

	return si7021_status;
}

HAL_StatusTypeDef Si7021_Write_RH_Temp_Resolution(I2C_HandleTypeDef *hi2c, resolution_t resolution)
{
    cmd[0] = SI7021_CMD_READ_USR_REG_1;

    HAL_I2C_Master_Transmit(hi2c, SI7021_I2C_SLAVE_ADDR, &cmd[0], 1, 100);
    HAL_I2C_Master_Receive(hi2c, SI7021_I2C_SLAVE_ADDR, &si7021_buff[0], 1, 100);

    cmd[0] = SI7021_CMD_WRITE_USR_REG_1;

    switch (resolution)
    {
        case RES_RH12_TEMP14:
            cmd[1] = si7021_buff[0] & ~(SI7021_USR_REG_1_D7 | SI7021_USR_REG_1_D0);
        break;

        case RES_RH8_TEMP12:
            cmd[1] = (si7021_buff[0] & ~SI7021_USR_REG_1_D7) | SI7021_USR_REG_1_D0;
        break;

        case RES_RH10_TEMP13:
            cmd[1] = (si7021_buff[0] | SI7021_USR_REG_1_D7) & ~SI7021_USR_REG_1_D0;
        break;

        case RES_RH11_TEMP11:
            cmd[1] = si7021_buff[0] | SI7021_USR_REG_1_D7 | SI7021_USR_REG_1_D0;
        break;
    }

    si7021_status = HAL_I2C_Master_Transmit(hi2c, SI7021_I2C_SLAVE_ADDR, cmd, 2, 100);

    return si7021_status;
}

HAL_StatusTypeDef Si7021_Write_Heater_State(I2C_HandleTypeDef *hi2c, heater_t state)
{
    cmd[0] = SI7021_CMD_READ_USR_REG_1;

    HAL_I2C_Master_Transmit(hi2c, SI7021_I2C_SLAVE_ADDR, &cmd[0], 1, 100);
    HAL_I2C_Master_Receive(hi2c, SI7021_I2C_SLAVE_ADDR, &si7021_buff[0], 1, 100);

    cmd[0] = SI7021_CMD_WRITE_USR_REG_1;
    cmd[1] = (HEATER_ENABLE == state) ? (si7021_buff[0] | SI7021_USR_REG_1_D2) : (si7021_buff[0] & ~SI7021_USR_REG_1_D2);

    si7021_status = HAL_I2C_Master_Transmit(hi2c, SI7021_I2C_SLAVE_ADDR, cmd, 2, 100);

    return si7021_status;
}

resolution_t Si7021_Read_RH_Temp_Resolution(I2C_HandleTypeDef *hi2c)
{
    resolution_t resolution;
    cmd[0] = SI7021_CMD_READ_USR_REG_1;

    HAL_I2C_Master_Transmit(hi2c, SI7021_I2C_SLAVE_ADDR, &cmd[0], 1, 100);
    HAL_I2C_Master_Receive(hi2c, SI7021_I2C_SLAVE_ADDR, &si7021_buff[0], 1, 100);

    switch (si7021_buff[0] & (SI7021_USR_REG_1_D7| SI7021_USR_REG_1_D0))
    {
        case 0x00:
            resolution = RES_RH12_TEMP14;
        break;

        case 0x01:
            resolution = RES_RH8_TEMP12;
        break;

        case 0x80:
            resolution = RES_RH10_TEMP13;
        break;

        case 0x81:
            resolution = RES_RH11_TEMP11;
        break;

        default:
            return -1;
        break;
    }

    return resolution;
}

vddstatus_t Si7021_Read_Vdd_Status(I2C_HandleTypeDef *hi2c)
{
    cmd[0] = SI7021_CMD_READ_USR_REG_1;
    HAL_I2C_Master_Transmit(hi2c, SI7021_I2C_SLAVE_ADDR, &cmd[0], 1, 100);
    HAL_I2C_Master_Receive(hi2c, SI7021_I2C_SLAVE_ADDR, &si7021_buff[0], 1, 100);

    return SI7021_USR_REG_1_D6 == (si7021_buff[0] & SI7021_USR_REG_1_D6) ? VDD_LOW : VDD_OK;
}

heater_t Si7021_Read_Heater_Status(I2C_HandleTypeDef *hi2c)
{
    cmd[0] = SI7021_CMD_READ_USR_REG_1;

    HAL_I2C_Master_Transmit(hi2c, SI7021_I2C_SLAVE_ADDR, &cmd[0], 1, 100);
    HAL_I2C_Master_Receive(hi2c, SI7021_I2C_SLAVE_ADDR, &si7021_buff[0], 1, 100);

    return SI7021_USR_REG_1_D2 == (si7021_buff[0] & SI7021_USR_REG_1_D2) ? HEATER_ENABLE : HEATER_DISABLE;
}

HAL_StatusTypeDef Si7021_Write_Heater_Current(I2C_HandleTypeDef *hi2c, heatercurrent_t current)
{
    cmd[0] = SI7021_CMD_READ_HTR_CTRL_REG;

    HAL_I2C_Master_Transmit(hi2c, SI7021_I2C_SLAVE_ADDR, &cmd[0], 1, 100);
    HAL_I2C_Master_Receive(hi2c, SI7021_I2C_SLAVE_ADDR, &si7021_buff[0], 1, 100);

    cmd[0] = SI7021_CMD_WRITE_HTR_CTRL_REG;

    switch (current)
    {
        case MA_309:
            cmd[1] = si7021_buff[0] & ~(SI7021_HEATER_CONTROL_REG_D3 | SI7021_HEATER_CONTROL_REG_D2 | SI7021_HEATER_CONTROL_REG_D1 | SI7021_HEATER_CONTROL_REG_D0);
        break;

        case MA_918:
            cmd[1] = (si7021_buff[0] | SI7021_HEATER_CONTROL_REG_D0) & ~(SI7021_HEATER_CONTROL_REG_D3 | SI7021_HEATER_CONTROL_REG_D2 | SI7021_HEATER_CONTROL_REG_D1);
        break;

        case MA_1524:
            cmd[1] = (si7021_buff[0] | SI7021_HEATER_CONTROL_REG_D1) & ~(SI7021_HEATER_CONTROL_REG_D3 | SI7021_HEATER_CONTROL_REG_D2 | SI7021_HEATER_CONTROL_REG_D0);
        break;

        case MA_2739:
            cmd[1] = (si7021_buff[0] | SI7021_HEATER_CONTROL_REG_D2) & ~(SI7021_HEATER_CONTROL_REG_D3 | SI7021_HEATER_CONTROL_REG_D1 | SI7021_HEATER_CONTROL_REG_D0);
        break;

        case MA_5169:
            cmd[1] = (si7021_buff[0] | SI7021_HEATER_CONTROL_REG_D3) & ~(SI7021_HEATER_CONTROL_REG_D2 | SI7021_HEATER_CONTROL_REG_D1 | SI7021_HEATER_CONTROL_REG_D0);
        break;

        case MA_9420:
            cmd[1] = si7021_buff[0] | SI7021_HEATER_CONTROL_REG_D3 | SI7021_HEATER_CONTROL_REG_D2 | SI7021_HEATER_CONTROL_REG_D1 | SI7021_HEATER_CONTROL_REG_D0;
        break;
    }

    si7021_status = HAL_I2C_Master_Transmit(hi2c, SI7021_I2C_SLAVE_ADDR, cmd, 2, 100);

    return si7021_status;
}

heatercurrent_t Si7021_Read_Heater_Current(I2C_HandleTypeDef *hi2c)
{
    heatercurrent_t current;
    cmd[0] = SI7021_CMD_READ_HTR_CTRL_REG;

    HAL_I2C_Master_Transmit(hi2c, SI7021_I2C_SLAVE_ADDR, &cmd[0], 1, 100);
    HAL_I2C_Master_Receive(hi2c, SI7021_I2C_SLAVE_ADDR, &si7021_buff[0], 1, 100);

    switch (si7021_buff[0] & (SI7021_HEATER_CONTROL_REG_D3 | SI7021_HEATER_CONTROL_REG_D2 | SI7021_HEATER_CONTROL_REG_D1 | SI7021_HEATER_CONTROL_REG_D0))
    {
        case 0x00:
            current = MA_309;
        break;

        case 0x01:
            current = MA_918;
        break;

        case 0x02:
            current = MA_1524;
        break;

        case 0x04:
            current = MA_2739;
        break;

        case 0x08:
            current = MA_5169;
        break;

        case 0x0F:
            current = MA_9420;
        break;

        default:
            return -1;
        break;
    }

    return current;
}

uint64_t Si7021_Read_Electronic_Serial_Number(I2C_HandleTypeDef *hi2c)
{
    uint64_t serial_num = 0;

    cmd[0] = SI7021_CMD_READ_ELECTRONIC_ID_BYTE_1_0;
    cmd[1] = SI7021_CMD_READ_ELECTRONIC_ID_BYTE_1_1;

    HAL_I2C_Master_Transmit(hi2c, SI7021_I2C_SLAVE_ADDR, cmd, 2, 100);
    HAL_I2C_Master_Receive(hi2c, SI7021_I2C_SLAVE_ADDR, si7021_elec_id, 4, 100);

    cmd[0] = SI7021_CMD_READ_ELECTRONIC_ID_BYTE_2_0;
    cmd[1] = SI7021_CMD_READ_ELECTRONIC_ID_BYTE_2_1;

    HAL_I2C_Master_Transmit(hi2c, SI7021_I2C_SLAVE_ADDR, cmd, 2, 100);
    HAL_I2C_Master_Receive(hi2c, SI7021_I2C_SLAVE_ADDR, &si7021_elec_id[4], 4, 100);

    for (int i = 0; i <  8; i++)
    {
        serial_num <<= 8;
        serial_num |= (uint64_t)si7021_elec_id[i];
    }

    return serial_num;
}

device_t Si7021_Read_Device(I2C_HandleTypeDef *hi2c)
{
    cmd[0] = SI7021_CMD_READ_ELECTRONIC_ID_BYTE_1_0;
    cmd[1] = SI7021_CMD_READ_ELECTRONIC_ID_BYTE_1_1;

    HAL_I2C_Master_Transmit(hi2c, SI7021_I2C_SLAVE_ADDR, cmd, 2, 100);
    HAL_I2C_Master_Receive(hi2c, SI7021_I2C_SLAVE_ADDR, si7021_elec_id, 4, 100);

    cmd[0] = SI7021_CMD_READ_ELECTRONIC_ID_BYTE_2_0;
    cmd[1] = SI7021_CMD_READ_ELECTRONIC_ID_BYTE_2_1;

    HAL_I2C_Master_Transmit(hi2c, SI7021_I2C_SLAVE_ADDR, cmd, 2, 100);
    HAL_I2C_Master_Receive(hi2c, SI7021_I2C_SLAVE_ADDR, &si7021_elec_id[4], 4, 100);

    switch(si7021_elec_id[4])
    {
        case (0x00 || 0xFF):
            return ENG_SAMPLE;
        break;

        case 0x0D:
            return SI7013;
        break;

        case 0x14:
            return SI7020;
        break;

        case 0x15:
            return SI7021;
        break;

        default:
            return NOT_DEFINED;
        break;
    }

}

firmware_t Si7021_Read_Firmware_Revision(I2C_HandleTypeDef *hi2c)
{
    cmd[0] = SI7021_CMD_READ_FIRMWARE_REV_0;
    cmd[1] = SI7021_CMD_READ_FIRMWARE_REV_1;

    HAL_I2C_Master_Transmit(hi2c, SI7021_I2C_SLAVE_ADDR, cmd, 2, 100);
    HAL_I2C_Master_Receive(hi2c, SI7021_I2C_SLAVE_ADDR, &si7021_buff[0], 1, 100);

    switch(si7021_buff[0])
    {
        case 0xFF:
            return V1;
        break;

        case 0x20:
            return V2;
        break;

        default:
            return UNK;
        break;
    }
}
