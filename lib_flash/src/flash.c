/*
 *  flash.c
 *
 *  Author: Kestutis Bivainis
 *
 */

#include "flash.h"
#include "stm32f10x_conf.h"

static GPIO_TypeDef* GPIO_CS;
static uint16_t GPIO_Pin_CS;
static SPI_TypeDef *spi;

static uint8_t Flash_SendByte(uint8_t byte) {

  while(SPI_I2S_GetFlagStatus(spi, SPI_I2S_FLAG_TXE) == RESET);
  SPI_I2S_SendData(spi, byte);
  while(SPI_I2S_GetFlagStatus(spi, SPI_I2S_FLAG_RXNE) == RESET);
  return SPI_I2S_ReceiveData(spi);
}

static uint8_t Flash_ReadByte(void) {

  return (Flash_SendByte(Dummy_Byte));
}

void Flash_ReadID(uint8_t *data) {

  uint8_t i;

  GPIO_ResetBits(GPIO_CS, GPIO_Pin_CS);
  Flash_SendByte(FLASH_RDID);
  for(i = 0; i < 4; i++) {
    data[i] = Flash_ReadByte();
  }
  GPIO_SetBits(GPIO_CS, GPIO_Pin_CS);
}

void Flash_ReadUID(uint8_t *data) {

  uint8_t i;

  GPIO_ResetBits(GPIO_CS, GPIO_Pin_CS);
  Flash_SendByte(FLASH_RDUID);
  for(i = 0; i < 4; i++) {
    data[i] = Flash_ReadByte();
  }
  for(i = 0; i < 8; i++) {
    data[i] = Flash_ReadByte();
  }
  GPIO_SetBits(GPIO_CS, GPIO_Pin_CS);
}

void Flash_Init(SPI_TypeDef* spi_number,GPIO_TypeDef* CS_port,uint16_t CS_pin) {

  GPIO_CS=CS_port;
  GPIO_Pin_CS=CS_pin;
  spi=spi_number;
}

void Flash_Sector4EraseAddr(uint32_t address) {

  Flash_WriteEnable();
  Flash_WaitBusy();
  GPIO_ResetBits(GPIO_CS, GPIO_Pin_CS);
  Flash_SendByte(FLASH_SE4);
  Flash_SendByte(address>>16);
  Flash_SendByte(address>>8);
  Flash_SendByte(address);
  GPIO_SetBits(GPIO_CS, GPIO_Pin_CS);
}

void Flash_Sector4EraseNr(uint32_t sector) {

  Flash_WriteEnable();
  Flash_WaitBusy();
  GPIO_ResetBits(GPIO_CS, GPIO_Pin_CS);
  Flash_SendByte(FLASH_SE4);
  Flash_SendByte(sector>>4);
  Flash_SendByte(sector<<4);
  Flash_SendByte(sector<<12);
  GPIO_SetBits(GPIO_CS, GPIO_Pin_CS);
}

void Flash_Sector32EraseAddr(uint32_t address) {

  Flash_WriteEnable();
  Flash_WaitBusy();
  GPIO_ResetBits(GPIO_CS, GPIO_Pin_CS);
  Flash_SendByte(FLASH_SE32);
  Flash_SendByte(address>>16);
  Flash_SendByte(address>>8);
  Flash_SendByte(address);
  GPIO_SetBits(GPIO_CS, GPIO_Pin_CS);
}

void Flash_Sector32EraseNr(uint32_t sector) {

  Flash_WriteEnable();
  Flash_WaitBusy();
  GPIO_ResetBits(GPIO_CS, GPIO_Pin_CS);
  Flash_SendByte(FLASH_SE32);
  Flash_SendByte(sector>>1);
  Flash_SendByte(sector<<7);
  Flash_SendByte(sector<<15);
  GPIO_SetBits(GPIO_CS, GPIO_Pin_CS);
}

void Flash_Sector64EraseAddr(uint32_t address) {

  Flash_WriteEnable();
  Flash_WaitBusy();
  GPIO_ResetBits(GPIO_CS, GPIO_Pin_CS);
  Flash_SendByte(FLASH_SE64);
  Flash_SendByte(address>>16);
  Flash_SendByte(address>>8);
  Flash_SendByte(address);
  GPIO_SetBits(GPIO_CS, GPIO_Pin_CS);
}

void Flash_Sector64EraseNr(uint32_t sector) {

  Flash_WriteEnable();
  Flash_WaitBusy();
  GPIO_ResetBits(GPIO_CS, GPIO_Pin_CS);
  Flash_SendByte(FLASH_SE64);
  Flash_SendByte(sector);
  Flash_SendByte(sector<<8);
  Flash_SendByte(sector<<16);
  GPIO_SetBits(GPIO_CS, GPIO_Pin_CS);
}

void Flash_BulkErase(void) {

  Flash_WriteEnable();
  Flash_WaitBusy();
  GPIO_ResetBits(GPIO_CS, GPIO_Pin_CS);
  Flash_SendByte(FLASH_BE);
  GPIO_SetBits(GPIO_CS, GPIO_Pin_CS);
}

void Flash_WaitBusy(void) {

  uint8_t status;

  GPIO_ResetBits(GPIO_CS, GPIO_Pin_CS);
  Flash_SendByte(FLASH_RDSR1);
  do {
    status = Flash_ReadByte();
  }
  while ((status & FLASH_WIP) == SET);
  GPIO_SetBits(GPIO_CS, GPIO_Pin_CS);
}

void Flash_Read(uint8_t *buffer,uint32_t start_address,uint32_t length) {

  uint32_t i;

  Flash_WaitBusy();
  GPIO_ResetBits(GPIO_CS, GPIO_Pin_CS);
  Flash_SendByte(FLASH_READ);
  Flash_SendByte(start_address>>16);
  Flash_SendByte(start_address>>8);
  Flash_SendByte(start_address);
  for (i=0;i<length;i++) {
    buffer[i] = Flash_ReadByte();
  }
  GPIO_SetBits(GPIO_CS, GPIO_Pin_CS);
}

void Flash_ReadPage(uint8_t *buffer,uint32_t page) {

  uint32_t i;

  Flash_WaitBusy();
  GPIO_ResetBits(GPIO_CS, GPIO_Pin_CS);
  Flash_SendByte(FLASH_READ);
  Flash_SendByte(page>>8);
  Flash_SendByte(page);
  Flash_SendByte(0);
  for (i=0;i<256;i++) {
    buffer[i] = Flash_ReadByte();
  }
  GPIO_SetBits(GPIO_CS, GPIO_Pin_CS);
}

void Flash_FastRead(uint8_t *buffer,uint32_t start_address,uint32_t length) {

  uint32_t i;

  Flash_WaitBusy();
  GPIO_ResetBits(GPIO_CS, GPIO_Pin_CS);
  Flash_SendByte(FLASH_FAST_READ);
  Flash_SendByte(start_address>>16);
  Flash_SendByte(start_address>>8);
  Flash_SendByte(start_address);
  Flash_SendByte(Dummy_Byte);
  for (i=0;i<length;i++) {
    buffer[i] = Flash_ReadByte();
  }
  GPIO_SetBits(GPIO_CS, GPIO_Pin_CS);
}

void Flash_FastReadPage(uint8_t *buffer,uint32_t page) {

  uint32_t i;

  Flash_WaitBusy();
  GPIO_ResetBits(GPIO_CS, GPIO_Pin_CS);
  Flash_SendByte(FLASH_FAST_READ);
  Flash_SendByte(page>>8);
  Flash_SendByte(page);
  Flash_SendByte(0);
  Flash_SendByte(Dummy_Byte);
  for (i=0;i<256;i++) {
    buffer[i] = Flash_ReadByte();
  }
  GPIO_SetBits(GPIO_CS, GPIO_Pin_CS);
}

void Flash_Write(uint8_t *buffer,uint32_t start_address,uint32_t length) {

  uint32_t i;

  Flash_WriteEnable();
  Flash_WaitBusy();
  GPIO_ResetBits(GPIO_CS, GPIO_Pin_CS);
  Flash_SendByte(FLASH_PP);
  Flash_SendByte(start_address>>16);
  Flash_SendByte(start_address>>8);
  Flash_SendByte(start_address);
  for (i=0;i<length;i++) {
    Flash_SendByte(buffer[i]);
  }
  GPIO_SetBits(GPIO_CS, GPIO_Pin_CS);
}

void Flash_WritePage(uint8_t *buffer,uint32_t page) {

  uint32_t i;

  Flash_WriteEnable();
  Flash_WaitBusy();
  GPIO_ResetBits(GPIO_CS, GPIO_Pin_CS);
  Flash_SendByte(FLASH_PP);
  Flash_SendByte(page>>8);
  Flash_SendByte(page);
  Flash_SendByte(0);
  for (i=0;i<256;i++) {
    Flash_SendByte(buffer[i]);
  }
  GPIO_SetBits(GPIO_CS, GPIO_Pin_CS);
}

void Flash_DeepPowerDown(void) {

  Flash_WaitBusy();
  GPIO_ResetBits(GPIO_CS, GPIO_Pin_CS);
  Flash_SendByte(FLASH_DD);
  GPIO_SetBits(GPIO_CS, GPIO_Pin_CS);
}

void Flash_ReleaseDeepPowerDown(void) {

  GPIO_ResetBits(GPIO_CS, GPIO_Pin_CS);
  Flash_SendByte(FLASH_RES);
  GPIO_SetBits(GPIO_CS, GPIO_Pin_CS);
}

uint8_t Flash_ReleaseDeepPowerDownReadRES(void) {

  uint8_t byte;

  GPIO_ResetBits(GPIO_CS, GPIO_Pin_CS);
  Flash_SendByte(FLASH_RES);
  Flash_SendByte(Dummy_Byte);
  Flash_SendByte(Dummy_Byte);
  Flash_SendByte(Dummy_Byte);
  byte=Flash_ReadByte();
  GPIO_SetBits(GPIO_CS, GPIO_Pin_CS);
  return byte;
}

uint8_t Flash_ReadStatusRegister1(void) {

  uint8_t status;

  GPIO_ResetBits(GPIO_CS, GPIO_Pin_CS);
  Flash_SendByte(FLASH_RDSR1);
  status = Flash_ReadByte();
  GPIO_SetBits(GPIO_CS, GPIO_Pin_CS);
  return status;
}

uint8_t Flash_ReadStatusRegister2(void) {

  uint8_t status;

  GPIO_ResetBits(GPIO_CS, GPIO_Pin_CS);
  Flash_SendByte(FLASH_RDSR2);
  status = Flash_ReadByte();
  GPIO_SetBits(GPIO_CS, GPIO_Pin_CS);
  return status;
}

void Flash_WriteStatusRegister(uint8_t status) {

  Flash_WriteEnable();
  Flash_WaitBusy();
  GPIO_ResetBits(GPIO_CS, GPIO_Pin_CS);
  Flash_SendByte(FLASH_WRSR);
  Flash_SendByte(status);
  GPIO_SetBits(GPIO_CS, GPIO_Pin_CS);
}

void Flash_WriteEnable(void) {

  Flash_WaitBusy();
  GPIO_ResetBits(GPIO_CS, GPIO_Pin_CS);
  Flash_SendByte(FLASH_WREN);
  GPIO_SetBits(GPIO_CS, GPIO_Pin_CS);
}

void Flash_WriteDisable(void) {

  Flash_WaitBusy();
  GPIO_ResetBits(GPIO_CS, GPIO_Pin_CS);
  Flash_SendByte(FLASH_WRDI);
  GPIO_SetBits(GPIO_CS, GPIO_Pin_CS);
}

void Flash_WriteProtect(uint8_t val) {

  Flash_WriteEnable();
  Flash_WaitBusy();
  GPIO_ResetBits(GPIO_CS, GPIO_Pin_CS);
  Flash_SendByte(FLASH_WRSR);
  Flash_SendByte(val<<2);
  GPIO_SetBits(GPIO_CS, GPIO_Pin_CS);
}
