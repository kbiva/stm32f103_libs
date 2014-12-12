# README #

* lib_flash

library for onboard 16Mbit SPI flash access.

### How do I get set up? ###

SPI and CS pin must be configured prior calling Flash_Init();
Adjustable parameters:

* SPI number
* CS port and pin

### Usage example ###
```C 
  uint8_t buf[256];
  ...
  Flash_Init(SPI1,GPIOA,GPIO_Pin_4);
  ...
  Flash_Read(buf,0x1F0000,256);
```
