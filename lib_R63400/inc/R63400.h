/*
 *  R63400.h
 *
 *  Library for Renesas R63400.
 *  Tested with LCD from Sony Ericsson K800i mobile phone.
 *
 *  Author: Kestutis Bivainis
 *
 */
#ifndef R63400_H
#define R63400_H

#include "stm32f10x_conf.h"

typedef enum _COLOR_MODE {
  COLOR_16BIT = 0x0000,// 2 bytes/px (RRRRRGGG GGGBBBBB)
  COLOR_18BIT = 0xE000 // 3 bytes/px (RRRRRRxx GGGGGGxx BBBBBBxx)
} COLOR_MODE;

typedef enum _ORIENTATION_MODE {
  ORIENTATION_PORTRAIT = 0x0000,
  ORIENTATION_LANDSCAPE = 0x0018,
  ORIENTATION_PORTRAIT_REV = 0x0030,
  ORIENTATION_LANDSCAPE_REV = 0x0028
} ORIENTATION_MODE;

typedef enum _FONT_SIZE {
  FONT_6x8 = 0,
  FONT_8x8 = 1,
  FONT_8x14 = 2
} FONT_SIZE;

enum COMMANDS {
  ENTRY_MODE              = 0x0003,
  DISPLAY_CONTROL_1       = 0x0007,
  GRAM_ADDRESS_HORZ       = 0x0200,
  GRAM_ADDRESS_VERT       = 0x0201,
  MEMORY_READ_WRITE       = 0x0202,
  WINDOW_HORZ_START       = 0x0210,
  WINDOW_HORZ_END         = 0x0211,
  WINDOW_VERT_START       = 0x0212,
  WINDOW_VERT_END         = 0x0213,
};

enum {
  R63400_OK = 1,
  R63400_ERROR = 0
};

#define DB0_Pin   GPIO_Pin_14
#define DB0_Speed GPIO_Speed_50MHz
#define DB0_Mode  GPIO_Mode_AF_PP
#define DB0_Port  GPIOD
#define DB0_Bus   RCC_APB2Periph_GPIOD

#define DB1_Pin   GPIO_Pin_15
#define DB1_Speed GPIO_Speed_50MHz
#define DB1_Mode  GPIO_Mode_AF_PP
#define DB1_Port  GPIOD
#define DB1_Bus   RCC_APB2Periph_GPIOD

#define DB2_Pin   GPIO_Pin_0
#define DB2_Speed GPIO_Speed_50MHz
#define DB2_Mode  GPIO_Mode_AF_PP
#define DB2_Port  GPIOD
#define DB2_Bus   RCC_APB2Periph_GPIOD

#define DB3_Pin   GPIO_Pin_1
#define DB3_Speed GPIO_Speed_50MHz
#define DB3_Mode  GPIO_Mode_AF_PP
#define DB3_Port  GPIOD
#define DB3_Bus   RCC_APB2Periph_GPIOD

#define DB4_Pin   GPIO_Pin_7
#define DB4_Speed GPIO_Speed_50MHz
#define DB4_Mode  GPIO_Mode_AF_PP
#define DB4_Port  GPIOE
#define DB4_Bus   RCC_APB2Periph_GPIOE

#define DB5_Pin   GPIO_Pin_8
#define DB5_Speed GPIO_Speed_50MHz
#define DB5_Mode  GPIO_Mode_AF_PP
#define DB5_Port  GPIOE
#define DB5_Bus   RCC_APB2Periph_GPIOE

#define DB6_Pin   GPIO_Pin_9
#define DB6_Speed GPIO_Speed_50MHz
#define DB6_Mode  GPIO_Mode_AF_PP
#define DB6_Port  GPIOE
#define DB6_Bus   RCC_APB2Periph_GPIOE

#define DB7_Pin   GPIO_Pin_10
#define DB7_Speed GPIO_Speed_50MHz
#define DB7_Mode  GPIO_Mode_AF_PP
#define DB7_Port  GPIOE
#define DB7_Bus   RCC_APB2Periph_GPIOE

#define RW_Pin    GPIO_Pin_5
#define RW_Speed  GPIO_Speed_50MHz
#define RW_Mode   GPIO_Mode_AF_PP
#define RW_Port   GPIOD
#define RW_Bus    RCC_APB2Periph_GPIOD

#define RD_Pin    GPIO_Pin_4
#define RD_Speed  GPIO_Speed_50MHz
#define RD_Mode   GPIO_Mode_AF_PP
#define RD_Port   GPIOD
#define RD_Bus    RCC_APB2Periph_GPIOD

#define RS_Pin    GPIO_Pin_11
#define RS_Speed  GPIO_Speed_50MHz
#define RS_Mode   GPIO_Mode_AF_PP
#define RS_Port   GPIOD
#define RS_Bus    RCC_APB2Periph_GPIOD

#define CS_Pin    GPIO_Pin_7
#define CS_Speed  GPIO_Speed_50MHz
#define CS_Mode   GPIO_Mode_AF_PP
#define CS_Port   GPIOD
#define CS_Bus    RCC_APB2Periph_GPIOD

#define RST_Pin   GPIO_Pin_1
#define RST_Speed GPIO_Speed_50MHz
#define RST_Mode  GPIO_Mode_Out_PP
#define RST_Port  GPIOE
#define RST_Bus   RCC_APB2Periph_GPIOE

typedef struct _PIN {
  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_TypeDef* GPIOx;
  uint32_t GPIO_Bus;
} PIN;


#define LCD_REG8     (*((volatile uint8_t*)0x60000000))
#define LCD_DAT8     (*((volatile uint8_t*)0x60010000))

uint8_t R63400_Init(uint8_t AddressSetupTime,uint8_t DataSetupTime);
void R63400_ColorMode(COLOR_MODE color_mode);
void R63400_OrientationMode(ORIENTATION_MODE orientation_mode);

void R63400_ClearScreen(uint32_t color);
void R63400_SetPixel(uint16_t x, uint16_t y, uint32_t color);
void R63400_FillPixel(uint16_t x0,uint16_t y0,uint16_t x1,uint16_t y1,uint32_t *color);
void R63400_FillFromBuffer(uint16_t x0,uint16_t y0,uint16_t x1,uint16_t y1,uint8_t *data);
void R63400_Fill(uint16_t x0,uint16_t y0,uint16_t x1,uint16_t y1,uint32_t color);

void R63400_Shutdown(void);

void R63400_SetWindow(uint16_t x0,uint16_t y0,uint16_t x1,uint16_t y1);

uint16_t R63400_GetWidth(void);
uint16_t R63400_GetHeight(void);

void R63400_SetFont(FONT_SIZE font_size);
void R63400_SetTextColors(uint32_t fColor, uint32_t bColor);
void R63400_PutChar(char c, uint16_t x, uint16_t y);
void R63400_PutStr(char *pString, uint16_t x, uint16_t y);
void R63400_PutStrCEOL(char *pString, uint16_t x, uint16_t y);
void R63400_PutStrCentered(char *pString, uint16_t y);

void R63400_ReadMemory(uint16_t x0,uint16_t y0,uint16_t x1,uint16_t y1,uint16_t *buf);
uint16_t R63400_ReadID(void);

#endif
