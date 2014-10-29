/*
 *  DS18D20.h
 *
 *  Author: Kestutis Bivainis
 *
 *  Original source code from
 *  http://kazus.ru/forums/showthread.php?t=100566
 */

#ifndef _DS18B20_H
#define _DS18B20_H

#include <stdint.h>

#define DS18B20_CONVERT_T_CMD         0x44
#define DS18B20_WRITE_STRATCHPAD_CMD  0x4E
#define DS18B20_READ_STRATCHPAD_CMD   0xBE
#define DS18B20_COPY_STRATCHPAD_CMD   0x48
#define DS18B20_RECALL_E_CMD          0xB8
#define DS18B20_READ_POWER_SUPPLY_CMD 0xB4

#define DS18B20_STRATCHPAD_SIZE       0x09
#define DS18B20_SERIAL_NUM_SIZE       0x08

//maximum number of 1-wire devices on bus
#define One_Wire_Device_Number_MAX      10

uint8_t DS18B20_Search_Rom(uint8_t *devices_found);
uint8_t DS18B20_Search_Rom2(uint8_t *devices_found, uint8_t (* SN_ROM)[One_Wire_Device_Number_MAX][DS18B20_SERIAL_NUM_SIZE]);
uint8_t DS18B20_Search_Rom_One_Device(uint8_t (*Serial_Num)[DS18B20_SERIAL_NUM_SIZE]);

uint8_t DS18B20_Start_Conversion_by_ROM(uint8_t (*Serial_Num)[DS18B20_SERIAL_NUM_SIZE]);
uint8_t DS18B20_Get_Conversion_Result_by_ROM_CRC(uint8_t (*Serial_Num)[DS18B20_SERIAL_NUM_SIZE], uint32_t *temperature);

uint8_t DS18B20_Start_Conversion_Skip_Rom(void);
uint8_t DS18B20_Read_Temp_NoCRC_Skip_Rom(uint32_t *temperature);

#endif
