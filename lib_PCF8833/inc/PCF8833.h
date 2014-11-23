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


typedef enum _COLOR_MODE {
  COLOR_8BIT = 0x02,
  COLOR_12BIT = 0x03,
  COLOR_16BIT = 0x05
} COLOR_MODE;

typedef enum _RBG_MODE {
  MODE_RGB = 0x00,
  MODE_BGR = 0x08,
} RGB_MODE;

typedef enum _ACCESS_MODE {
  ACCESS_BITBANG = 1,
  ACCESS_SPI9BITS
} ACCESS_MODE;

typedef enum _DISPLAY_INVERSION {
  DISPLAY_INVERSION_ON = 0x21,
  DISPLAY_INVERSION_OFF = 0x20
} DISPLAY_INVERSION;


typedef enum _ORIENTATION_MODE {
  // for 3100,6100,6030
  ORIENTATION_PORTRAIT =       0xE0,
  ORIENTATION_LANDSCAPE =      0x40,
  ORIENTATION_PORTRAIT_REV =   0x20,
  ORIENTATION_LANDSCAPE_REV =  0x80,
  // for 6020
  ORIENTATION_PORTRAIT1 =      0xA0,
  ORIENTATION_LANDSCAPE1 =     0x00,
  ORIENTATION_PORTRAIT1_REV =  0x60,
  ORIENTATION_LANDSCAPE1_REV = 0xC0
} ORIENTATION_MODE;

typedef enum _FONT_SIZE {
  FONT_6x8 = 0,
  FONT_8x8 = 1,
  FONT_8x16 = 2
} FONT_SIZE;

#define CS_Pin         GPIO_Pin_4            // CS = PA[4]
#define CS_Speed       GPIO_Speed_50MHz
#define CS_Mode_BB     GPIO_Mode_Out_PP
#define CS_Mode_SPI    GPIO_Mode_AF_PP
#define CS_Port        GPIOA
#define CS_Bus         RCC_APB2Periph_GPIOA
#define CS_AFIO_Bus    RCC_APB2Periph_AFIO

#define SCLK_Pin       GPIO_Pin_5            // SCLK = PA[5]
#define SCLK_Speed     GPIO_Speed_50MHz
#define SCLK_Mode_BB   GPIO_Mode_Out_PP
#define SCLK_Mode_SPI  GPIO_Mode_AF_PP
#define SCLK_Port      GPIOA
#define SCLK_Bus       RCC_APB2Periph_GPIOA
#define SCLK_AFIO_Bus  RCC_APB2Periph_AFIO

#define SDATA_Pin      GPIO_Pin_7            // SDATA = PA[7]
#define SDATA_Speed    GPIO_Speed_50MHz
#define SDATA_Mode_BB  GPIO_Mode_Out_PP
#define SDATA_Mode_SPI GPIO_Mode_AF_PP
#define SDATA_Port     GPIOA
#define SDATA_Bus      RCC_APB2Periph_GPIOA
#define SDATA_AFIO_Bus RCC_APB2Periph_AFIO

#define RST_Pin        GPIO_Pin_7            // RESET = PB[7]
#define RST_Speed      GPIO_Speed_50MHz
#define RST_Mode       GPIO_Mode_Out_PP
#define RST_Port       GPIOB
#define RST_Bus        RCC_APB2Periph_GPIOB

typedef struct _PIN {
  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_TypeDef* GPIOx;
  uint32_t GPIO_Bus;
} PIN;

typedef struct _PIN_SPI {
  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_TypeDef* GPIOx;
  uint32_t GPIO_Bus;
  uint32_t AFIO_Bus;
} PIN_SPI;

// *************************************************************************************
// LCD Include File for Philips PCF8833 STN RGB- 132x132x3 Driver
//
// Taken from Philips data sheet Feb 14, 2003
// *************************************************************************************
// Philips PCF8833 LCD controller command codes

#define NOP      0x00 // nop
#define SWRESET  0x01  // software reset
#define BSTROFF  0x02  // booster voltage OFF
#define BSTRON   0x03  // booster voltage ON
#define RDDIDIF  0x04  // read display identification
#define RDDST    0x09  // read display status
#define SLEEPIN  0x10  // sleep in
#define SLEEPOUT 0x11 // sleep out
#define PTLON    0x12 // partial display mode
#define NORON    0x13 // display normal mode
#define INVOFF   0x20 // inversion OFF
#define INVON    0x21 // inversion ON
#define DALO     0x22 // all pixel OFF
#define DAL      0x23 // all pixel ON
#define SETCON   0x25 // write contrast
#define DISPOFF  0x28 // display OFF
#define DISPON   0x29 // display ON
#define CASET    0x2A // column address set
#define PASET    0x2B // page address set
#define RAMWR    0x2C // memory write
#define RGBSET   0x2D // colour set
#define PTLAR    0x30 // partial area
#define VSCRDEF  0x33 // vertical scrolling definition
#define TEOFF    0x34 // test mode
#define TEON     0x35 // test mode
#define MADCTL   0x36 // memory access control
#define SEP      0x37 // vertical scrolling start address
#define IDMOFF   0x38 // idle mode OFF
#define IDMON    0x39  // idle mode ON
#define COLMOD   0x3A  // interface pixel format
#define SETVOP   0xB0  // set Vop
#define BRSOFF   0xB4 // bottom row swap
#define BRSON    0xB5 // bottom row swap
#define TRSOFF   0xB6 // top row swap
#define TRSON    0xB7 // top row swap
#define FINVOFF  0xB8  // display control
#define DISCTR   0xB9  // display control
#define DOROFF   0xBA // data order
#define DORON    0xBB // data order
#define TCDFE    0xBD // enable/disable DF temperature compensation
#define TCVOPE   0xBF // enable/disable Vop temp comp
#define EC       0xC0 // internal or external oscillator
#define SETMUL   0xC2  // set multiplication factor
#define TCVOPAB  0xC3  // set TCVOP slopes A and B
#define TCVOPCD  0xC4  // set TCVOP slopes c and d
#define TCDF     0xC5  // set divider frequency
#define DF8COLOR 0xC6 // set divider frequency 8-color mode
#define SETBS    0xC7  // set bias system
#define RDTEMP   0xC8  // temperature read back
#define NLI      0xC9 // n-line inversion
#define RDID1    0xDA // read ID1
#define RDID2    0xDB // read ID2
#define RDID3    0xDC // read ID3


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

void PCF8833_Init(ACCESS_MODE access_mode);
void PCF8833_ColorMode(COLOR_MODE color_mode);
void PCF8833_SetOrientation(ORIENTATION_MODE orientation_mode,uint8_t mirror);
void PCF8833_DisplayInversion(DISPLAY_INVERSION display_inversion);
void PCF8833_SetRGB(RGB_MODE rgb_mode);
void PCF8833_SetupColor(uint8_t *color_map,uint8_t size);
void PCF8833_SetContrast(uint8_t contrast);

void PCF8833_ClearScreen(uint16_t color);

void PCF8833_SetFont(FONT_SIZE font_size);
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
