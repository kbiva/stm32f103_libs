/*
 *  flash.h
 *
 *  Author: Kestutis Bivainis
 *
 */

#ifndef _FLASH_H_
#define _FLASH_H_

#include <stdint.h>
#include "stm32f10x_conf.h"

#define FLASH_PageSize      256

#define FLASH_WREN       0x06  /* Write enable */
#define FLASH_WRDI       0x04  /* Write Disable */
#define FLASH_RDID       0x9F  /* Read identification */
#define FLASH_RDSR1      0x05  /* Read Status Register 1 */
#define FLASH_RDSR2      0x35  /* Read Status Register 2 */
#define FLASH_WRSR       0x01  /* Write Status Register */
#define FLASH_READ       0x03  /* Read Data Bytes */
#define FLASH_FAST_READ  0x0B  /* Read Data Bytes at Higher Speed */
#define FLASH_PP         0x02  /* Page Program */
#define FLASH_SE4        0x20  /* Sector Erase 4Kb */
#define FLASH_SE32       0x52  /* Sector Erase 32Kb */
#define FLASH_SE64       0xD8  /* Sector Erase 64Kb */
#define FLASH_BE         0xC7  /* Bulk Erase */
#define FLASH_DD         0xB9  /* Deep Power-down */
#define FLASH_RES        0xAB  /* Release from Deep Power-down */
#define FLASH_RDUID      0x4B  /* Read unique ID */

// Status Register 1
#define FLASH_WIP        0x01  /* Write In Progress */
#define FLASH_WEL        0x02  /* Write Enable */
#define FLASH_BP0        0x04  /* Write Protect bit 0 */
#define FLASH_BP1        0x08  /* Write Protect bit 1 */
#define FLASH_BP2        0x10  /* Write Protect bit 2 */
#define FLASH_TB         0x20  /* Top/Bottom Protect */
#define FLASH_SEC        0x40  /* Sector Protect */
#define FLASH_SRP0       0x80  /* Status Register0 Write Protect */

// Status Register 2
#define FLASH_SRP1       0x01  /* Status Register2 Write Protect */
#define FLASH_QE         0x02  /* Quad Enable */
#define FLASH_RESERVED   0x04  /* Reserved */
#define FLASH_LB1        0x08  /* Security Register Lock Bit 0 */
#define FLASH_LB2        0x10  /* Security Register Lock Bit 1 */
#define FLASH_LB3        0x20  /* Security Register Lock Bit 2 */
#define FLASH_CMP        0x40  /* Complement Protect */
#define FLASH_SUS        0x80  /* Suspend Status */


#define Dummy_Byte 0xA5


// Write Protect
#define FLASH_WP_NONE 0
#define FLASH_WP_1    1
#define FLASH_WP_2    2
#define FLASH_WP_3    3
#define FLASH_WP_4    4
#define FLASH_WP_5    5
#define FLASH_WP_6    6
#define FLASH_WP_ALL  7

void Flash_ReadID(uint8_t *data);
void Flash_ReadUID(uint8_t *data);
void Flash_Init(SPI_TypeDef* spi_number,GPIO_TypeDef* CS_port,uint16_t CS_pin);
void Flash_Sector4EraseAddr(uint32_t address);
void Flash_Sector4EraseNr(uint32_t sector);
void Flash_Sector32EraseAddr(uint32_t address);
void Flash_Sector32EraseNr(uint32_t sector);
void Flash_Sector64EraseAddr(uint32_t address); // address 0-2097151
void Flash_Sector64EraseNr(uint32_t sector);    // sector 0-31
void Flash_BulkErase(void);
void Flash_WaitBusy(void);
void Flash_Read(uint8_t *buffer,uint32_t start_address,uint32_t length);
void Flash_ReadPage(uint8_t *buffer,uint32_t page); // page 0 - 8191
void Flash_FastRead(uint8_t *buffer,uint32_t start_address,uint32_t length);
void Flash_FastReadPage(uint8_t *buffer,uint32_t page); // page 0 - 8191
void Flash_Write(uint8_t *buffer,uint32_t start_address,uint32_t length); // length 0 - 255
void Flash_WritePage(uint8_t *buffer,uint32_t page); // page 0 - 8191
void Flash_DeepPowerDown(void);
void Flash_ReleaseDeepPowerDown(void);
uint8_t Flash_ReleaseDeepPowerDownReadRES(void);
uint8_t Flash_ReadStatusRegister1(void);
uint8_t Flash_ReadStatusRegister2(void);
void Flash_WriteStatusRegister1(uint8_t status);
void Flash_WriteEnable(void);
void Flash_WriteDisable(void);
void Flash_WriteProtect(uint8_t val);

#endif


