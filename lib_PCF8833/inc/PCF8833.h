/*
 *  PCF8833.h
 *
 *  Author: Kestutis Bivainis
 *
 */

#ifndef __PCF8833_H
#define __PCF8833_H

#include "stm32f10x_conf.h"

// Nokia 6100
//  8bit BGR 3:3:2 (LUT)
// 12bit BGR 4:4:4 (native)
// 16bit BGR 5:6:5 (dithered)

// Nokia 3100
//  8bit RGB 3:3:2 (LUT)
// 12bit RGB 4:4:4 (native)
// 16bit RGB 5:6:5 (dithered)

// Nokia 6020, Nokia 6030
//  8bit RGB 3:3:2 (LUT?)
// 12bit RGB 4:4:4 (interpolated?)
// 16bit RGB 5:6:5 (native)


typedef enum _PCF8833_COLOR_MODE {
  PCF8833_COLOR_8BIT = 0x02,
  PCF8833_COLOR_12BIT = 0x03,
  PCF8833_COLOR_16BIT = 0x05
} PCF8833_COLOR_MODE;

typedef enum _PCF8833_RBG_MODE {
  PCF8833_MODE_RGB = 0x00,
  PCF8833_MODE_BGR = 0x08,
} PCF8833_RGB_MODE;

typedef enum _PCF8833_ACCESS_MODE {
  PCF8833_ACCESS_BITBANG = 1,
  PCF8833_ACCESS_SPI9BITS
} PCF8833_ACCESS_MODE;

typedef enum _PCF8833_DISPLAY_INVERSION {
  PCF8833_DISPLAY_INVERSION_ON = 0x21,
  PCF8833_DISPLAY_INVERSION_OFF = 0x20
} PCF8833_DISPLAY_INVERSION;

typedef enum _PCF8833_ORIENTATION_MODE {
  // for 3100,6100,6030
  PCF8833_ORIENTATION_PORTRAIT =       0xE0,
  PCF8833_ORIENTATION_LANDSCAPE =      0x40,
  PCF8833_ORIENTATION_PORTRAIT_REV =   0x20,
  PCF8833_ORIENTATION_LANDSCAPE_REV =  0x80,
  // for 6020
  PCF8833_ORIENTATION_PORTRAIT1 =      0xA0,
  PCF8833_ORIENTATION_LANDSCAPE1 =     0x00,
  PCF8833_ORIENTATION_PORTRAIT1_REV =  0x60,
  PCF8833_ORIENTATION_LANDSCAPE1_REV = 0xC0
} PCF8833_ORIENTATION_MODE;

typedef enum _PCF8833_FONT_SIZE {
  PCF8833_FONT_6x8 = 0,
  PCF8833_FONT_8x8 = 1,
  PCF8833_FONT_8x14 = 2
} PCF8833_FONT_SIZE;

// SPI1 on ABP2 bus
#define PCF8833_SPI             SPI1
#define PCF8833_SPI_Bus_Enable  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE)
#define PCF8833_SPI_Bus_Disable RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, DISABLE)

#define PCF8833_CS_Pin         GPIO_Pin_4            // CS = PA[4]
#define PCF8833_CS_Speed       GPIO_Speed_50MHz
#define PCF8833_CS_Mode_BB     GPIO_Mode_Out_PP
#define PCF8833_CS_Mode_SPI    GPIO_Mode_AF_PP
#define PCF8833_CS_Port        GPIOA
#define PCF8833_CS_Bus         RCC_APB2Periph_GPIOA

#define PCF8833_SCLK_Pin       GPIO_Pin_5            // SCLK = PA[5]
#define PCF8833_SCLK_Speed     GPIO_Speed_50MHz
#define PCF8833_SCLK_Mode_BB   GPIO_Mode_Out_PP
#define PCF8833_SCLK_Mode_SPI  GPIO_Mode_AF_PP
#define PCF8833_SCLK_Port      GPIOA
#define PCF8833_SCLK_Bus       RCC_APB2Periph_GPIOA

#define PCF8833_SDATA_Pin      GPIO_Pin_7            // SDATA = PA[7]
#define PCF8833_SDATA_Speed    GPIO_Speed_50MHz
#define PCF8833_SDATA_Mode_BB  GPIO_Mode_Out_PP
#define PCF8833_SDATA_Mode_SPI GPIO_Mode_AF_PP
#define PCF8833_SDATA_Port     GPIOA
#define PCF8833_SDATA_Bus      RCC_APB2Periph_GPIOA

/*
// SPI2 on ABP1 bus
#define PCF8833_SPI             SPI2
#define PCF8833_SPI_Bus_Enable  RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE)
#define PCF8833_SPI_Bus_Disable RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, DISABLE)

#define PCF8833_CS_Pin         GPIO_Pin_12            // CS = PB[12]
#define PCF8833_CS_Speed       GPIO_Speed_50MHz
#define PCF8833_CS_Mode_BB     GPIO_Mode_Out_PP
#define PCF8833_CS_Mode_SPI    GPIO_Mode_AF_PP
#define PCF8833_CS_Port        GPIOB
#define PCF8833_CS_Bus         RCC_APB2Periph_GPIOB

#define PCF8833_SCLK_Pin       GPIO_Pin_13            // SCLK = PB[13]
#define PCF8833_SCLK_Speed     GPIO_Speed_50MHz
#define PCF8833_SCLK_Mode_BB   GPIO_Mode_Out_PP
#define PCF8833_SCLK_Mode_SPI  GPIO_Mode_AF_PP
#define PCF8833_SCLK_Port      GPIOB
#define PCF8833_SCLK_Bus       RCC_APB2Periph_GPIOB

#define PCF8833_SDATA_Pin      GPIO_Pin_15            // SDATA = PB[15]
#define PCF8833_SDATA_Speed    GPIO_Speed_50MHz
#define PCF8833_SDATA_Mode_BB  GPIO_Mode_Out_PP
#define PCF8833_SDATA_Mode_SPI GPIO_Mode_AF_PP
#define PCF8833_SDATA_Port     GPIOB
#define PCF8833_SDATA_Bus      RCC_APB2Periph_GPIOB
*/

#define PCF8833_RST_Pin        GPIO_Pin_7            // RESET = PB[7]
#define PCF8833_RST_Speed      GPIO_Speed_50MHz
#define PCF8833_RST_Mode       GPIO_Mode_Out_PP
#define PCF8833_RST_Port       GPIOB
#define PCF8833_RST_Bus        RCC_APB2Periph_GPIOB

typedef struct _PCF8833_PIN {
  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_TypeDef* GPIOx;
  uint32_t GPIO_Bus;
} PCF8833_PIN;

// *************************************************************************************
// LCD Include File for Philips PCF8833 STN RGB- 132x132x3 Driver
//
// Taken from Philips data sheet Feb 14, 2003
// *************************************************************************************
// Philips PCF8833 LCD controller command codes

typedef enum _PCF8833_COMMANDS {
  PCF8833_NOP =      0x00,// nop
  PCF8833_SWRESET =  0x01,// software reset
  PCF8833_BSTROFF =  0x02,// booster voltage OFF
  PCF8833_BSTRON =   0x03,// booster voltage ON
  PCF8833_RDDIDIF =  0x04,// read display identification
  PCF8833_RDDST =    0x09,// read display status
  PCF8833_SLEEPIN =  0x10,// sleep in
  PCF8833_SLEEPOUT = 0x11,// sleep out
  PCF8833_PTLON =    0x12,// partial display mode
  PCF8833_NORON =    0x13,// display normal mode
  PCF8833_INVOFF =   0x20,// inversion OFF
  PCF8833_INVON =    0x21,// inversion ON
  PCF8833_DALO =     0x22,// all pixel OFF
  PCF8833_DAL =      0x23,// all pixel ON
  PCF8833_SETCON =   0x25,// write contrast
  PCF8833_DISPOFF =  0x28,// display OFF
  PCF8833_DISPON =   0x29,// display ON
  PCF8833_CASET =    0x2A,// column address set
  PCF8833_PASET =    0x2B,// page address set
  PCF8833_RAMWR =    0x2C,// memory write
  PCF8833_RGBSET =   0x2D,// colour set
  PCF8833_PTLAR =    0x30,// partial area
  PCF8833_VSCRDEF =  0x33,// vertical scrolling definition
  PCF8833_TEOFF =    0x34,// test mode
  PCF8833_TEON =     0x35,// test mode
  PCF8833_MADCTL =   0x36,// memory access control
  PCF8833_SEP =      0x37,// vertical scrolling start address
  PCF8833_IDMOFF =   0x38,// idle mode OFF
  PCF8833_IDMON =    0x39,// idle mode ON
  PCF8833_COLMOD =   0x3A,// interface pixel format
  PCF8833_SETVOP =   0xB0,// set Vop
  PCF8833_BRSOFF =   0xB4,// bottom row swap
  PCF8833_BRSON =    0xB5,// bottom row swap
  PCF8833_TRSOFF =   0xB6,// top row swap
  PCF8833_TRSON =    0xB7,// top row swap
  PCF8833_FINVOFF =  0xB8,// display control
  PCF8833_DISCTR =   0xB9,// display control
  PCF8833_DOROFF =   0xBA,// data order
  PCF8833_DORON =    0xBB,// data order
  PCF8833_TCDFE =    0xBD,// enable/disable DF temperature compensation
  PCF8833_TCVOPE =   0xBF,// enable/disable Vop temp comp
  PCF8833_EC =       0xC0,// internal or external oscillator
  PCF8833_SETMUL =   0xC2,// set multiplication factor
  PCF8833_TCVOPAB =  0xC3,// set TCVOP slopes A and B
  PCF8833_TCVOPCD =  0xC4,// set TCVOP slopes c and d
  PCF8833_TCDF =     0xC5,// set divider frequency
  PCF8833_DF8COLOR = 0xC6,// set divider frequency 8-color mode
  PCF8833_SETBS =    0xC7,// set bias system
  PCF8833_RDTEMP =   0xC8,// temperature read back
  PCF8833_NLI =      0xC9,// n-line inversion
  PCF8833_RDID1 =    0xDA,// read ID1
  PCF8833_RDID2 =    0xDB,// read ID2
  PCF8833_RDID3 =    0xDC,// read ID3
} PCF8833_COMMANDS;

// mirror Y, mirror X, BGR
#define NOKIA6100_MADCTL_DATA 0xC8
// no mirror Y, mirror X, BGR
#define NOKIA6100_MADCTL_DATA_BMP 0x48
// contrast
#define NOKIA6100_SETCON_DATA 0x3A

// mirror Y, no mirror X, RGB
#define NOKIA3100_MADCTL_DATA 0x80
// no mirror Y, no mirror X, RGB
#define NOKIA3100_MADCTL_DATA_BMP 0x00
// contrast
#define NOKIA3100_SETCON_DATA 0x3C

// mirror Y, no mirror X, RGB
#define NOKIA6030_MADCTL_DATA 0x08
// no mirror Y, no mirror X, RGB
#define NOKIA6030_MADCTL_DATA_BMP 0x00
// contrast
#define NOKIA6030_SETCON_DATA 0x3A

// mirror Y, no mirror X, RGB
#define NOKIA6020_MADCTL_DATA 0x80
// no mirror Y, no mirror X, RGB
#define NOKIA6020_MADCTL_DATA_BMP 0x00
// contrast
#define NOKIA6020_SETCON_DATA 0x3C

void PCF8833_SPI9bits(uint16_t bits9);
void PCF8833_SPI9bits_Flush(void);
void PCF8833_Data_Bitbang(uint8_t data);
void PCF8833_Command_Bitbang(uint8_t command);

//void PCF8833_Init(PCF8833_InitTypeDef* PCF8833_InitStruct);
void PCF8833_Init(PCF8833_ACCESS_MODE access_mode);
void PCF8833_ColorMode(PCF8833_COLOR_MODE color_mode);
void PCF8833_SetOrientation(PCF8833_ORIENTATION_MODE orientation_mode,uint8_t mirror);
void PCF8833_DisplayInversion(PCF8833_DISPLAY_INVERSION display_inversion);
void PCF8833_SetRGB(PCF8833_RGB_MODE rgb_mode);
void PCF8833_SetupColor(uint8_t *color_map,uint8_t size);
void PCF8833_SetContrast(uint8_t contrast);

void PCF8833_ClearScreen(uint16_t color);

void PCF8833_SetFont(PCF8833_FONT_SIZE font_size);
void PCF8833_SetTextColours(uint16_t fColor, uint16_t bColor);
void PCF8833_PutChar(char c, uint8_t x, uint8_t y);
void PCF8833_PutStr(char *pString, uint8_t x, uint8_t y);

void PCF8833_SetPixel(uint8_t x, uint8_t y, uint16_t color);
void PCF8833_Line(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1, uint16_t color);
void PCF8833_Rectangle(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1, uint8_t fill, uint16_t color);
void PCF8833_Circle(uint8_t x0, uint8_t y0, uint8_t radius, uint16_t color);
void PCF8833_SetWindow(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1);

uint8_t PCF8833_GetWidth(void);
uint8_t PCF8833_GetHeight(void);

void PCF8833_Sleep(void);
void PCF8833_Wakeup(void);

// for Nokia 3100 6100 6030 LCD in 8bit mode
extern uint8_t RGB8ColorMap[];

// only for Nokia 6020 LCD in 8bit mode
extern uint8_t RGB8ColorMap_Nokia6020[];

// only for Nokia 6020 LCD in 12bit mode
extern uint8_t RGB12ColorMap_Nokia6020[];

#endif  // __PCF8833_H
