# README #

Various libraries for the HY-STM32 board, used by me.
All libraries depends on lib_STM32F10x_StdPeriph.

* <b>lib_1-Wire</b> - 1-Wire library. Depends on lib_delay.
* <b>lib_BSP_HY-STM32</b> - HY-STM32 board support library for onboard components: 4 LEDs, 4 buttons and buzzer.
* <b>lib_delay</b> - Microseconds delay.
* <b>lib_DS18B20</b> - DS18D20 digital temperature sensor library. Depends on lib_1-Wire.
* <b>lib_ENC28J60</b> - ENC28J60 ethernet with TCPIP stack library.
* <b>lib_FatFs</b> - FatFs library. Depends on lib_SD_FatFs.
* <b>lib_flash</b> - library for onboard 16Mbit SPI flash access.
* <b>lib_ILI9328</b> - Ilitek ILI9328 LCD library. Depends on lib_delay.
* <b>lib_HD66773R</b> -Renesas HD66773R LCD library (Samsung E700). Depends on lib_delay.
* <b>lib_HX8352A</b> - Himax HX8352A LCD library (LG KF700). Depends on lib_delay.
* <b>lib_LGDP4532</b> - LG LGDP4532 LCD library. Depends on lib_delay.
* <b>lib_MC2PA8201</b> - MagnaChip MC2PA8201 LCD library (Nokia E61,E63,E73). Depends on lib_delay.
* <b>lib_MFRC522</b> - RFID Chip MFRC522 driver. Depends on lib_delay.
* <b>lib_NokiaN70</b> - Nokia N70 LCD controller library (Nokia N70) .Similar to lib_MC2PA8201 with minor differencies. Depends on lib_delay.
* <b>lib_PCF8833</b> - Philips PCF8833 LCD library (Nokia 3100,6100,6020,6030). Depends on lib_delay.
* <b>lib_R61523</b> - Renesas R61523 LCD library (Sony Ericsson U5 Vivaz). Depends on lib_delay.
* <b>lib_S6B33BG</b> - Samsung S6B33BG LCD library (Samsung GT-E1050). Depends on lib_delay.
* <b>lib_rtc</b> - RTC
* <b>lib_SD_FatFs</b> - FatFs lower layer API (SD support). Depends on lib_FatFs,lib_rtc,lib_SD_SDIO.
* <b>lib_SD_SDIO</b> - SD SDIO low level API.
* <b>lib_SPFD54124B</b> - Orise Tech SPFD54124B LCD library (Nokia 6151). Depends on lib_delay.
* <b>lib_SSD1289</b> - Solomon Systech SSD1289 LCD library. Depends on lib_delay.
* <b>lib_STM32F10x_StdPeriph</b> - Standart Peripheral Library for STMF103xx v3.6.1
* <b>lib_tjpegd</b> - TJpgDec library.
* <b>lib_touch_XPT2046</b> - XPT2046 touch controller.
* <b>lib_usart</b> - USART
* <b>lib_xorshift</b> - Xorshift pseudorandom number generator.

