# README #

Various libraries for the HY-STM32 board, used by me.
All libraries depends on lib_STM32F10x_StdPeriph.

* lib_1-Wire

1-Wire library. Depends on lib_delay.
* lib_BSP_HY-STM32
  
HY-STM32 board support library for onboard components: 4 LEDs, 4 buttons and buzzer.
* lib_delay
  
Microseconds delay.
* lib_DS18B20
  
DS18D20 digital temperature sensor library. Depends on lib_1-Wire.
* lib_FatFs
  
FatFs library. Depends on lib_SD_FatFs.
* lib_ILI9328
  
Renesas HD66773R LCD library (Samsung E700). Depends on lib_delay.
* lib_HD66773R

Himax HX8352A LCD library (LG KF700). Depends on lib_delay.
* lib_HX8352A

Ilitek ILI9328 LCD library. Depends on lib_delay.
* lib_LGDP4532
  
LG LGDP4532 LCD library. Depends on lib_delay.
* lib_MC2PA8201
  
MagnaChip MC2PA8201 LCD library (Nokia E63,E73). Depends on lib_delay.
* lib_PCF8833
  
Philips PCF8833 LCD library (Nokia 3100,6100,6020,6030). Depends on lib_delay.
* lib_R61523
  
Renesas R61523 LCD library (Sony Ericsson U5 Vivaz). Depends on lib_delay.
* lib_S6B33BG
  
Samsung S6B33BG LCD library (Samsung GT-E1050). Depends on lib_delay.
* lib_rtc
  
RTC
* lib_SD_FatFs
  
FatFs lower layer API (SD support). Depends on lib_FatFs,lib_rtc,lib_SD_SDIO.
* lib_SD_SDIO
  
SD SDIO low level API.
* lib_SSD1289
  
Solomon Systech SSD1289 LCD library. Depends on lib_delay.
* lib_STM32F10x_StdPeriph
  
Standart Peripheral Library for STMF103xx v3.6.1
* lib_touch_XPT2046
  
XPT2046 touch controller.
* lib_usart
  
USART
