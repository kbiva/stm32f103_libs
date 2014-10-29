/*
 *  S6B33BG.h
 *
 *  Author: Kestutis Bivainis
 *
 */
#ifndef S6B33BG_H
#define S6B33BG_H

#include "stm32f10x_conf.h"

typedef enum _COLOR_MODE {
  COLOR_12BIT = 0x20,
  COLOR_16BIT = 0x00
} COLOR_MODE;

typedef enum _ORIENTATION_MODE {
  ORIENTATION_PORTRAIT = 0x0C,
  ORIENTATION_LANDSCAPE = 0x08,
  ORIENTATION_PORTRAIT_REV = 0x00,
  ORIENTATION_LANDSCAPE_REV = 0x04,
} ORIENTATION_MODE;

enum {
  S6B33BG_OK = 1,
  S6B33BG_ERROR = 0
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

typedef enum _COMMANDS {
  S6B33BG_NOP = 0x00,
  S6B33BG_OSCILATION_MODE = 0x02,
  S6B33BG_DRIVER_OUTPUT_MODE = 0x10,
  S6B33BG_MONITOR_SIGNAL_CTRL = 0x18,
  S6B33BG_DC_DC_SELECT = 0x20,
  S6B33BG_DC_DC_CLK_DIVISION = 0x24,
  S6B33BG_DC_DC_AMP = 0x26,
  S6B33BG_TEMPERATURE_COMPENSATION = 0x28,
  S6B33BG_CONTRAST_CTRL = 0x2A,
  S6B33BG_STANDBY_MODE_OFF = 0x2C,
  S6B33BG_STANDBY_MODE_ON = 0x2D,
  S6B33BG_ADDRESSING_MODE = 0x30,
  S6B33BG_ROW_VECTOR_MODE = 0x32,
  S6B33BG_NBLOCK_INVERSION = 0x34,
  S6B33BG_DRIVING_MODE = 0x36,
  S6B33BG_ENTRY_MODE = 0x40,
  S6B33BG_ROW_ADDRESS_AREA = 0x42,
  S6B33BG_COLUMN_ADDRESS_AREA = 0x43,
  S6B33BG_RAM_SKIP_AREA = 0x45,
  S6B33BG_DISPLAY_OFF = 0x50,
  S6B33BG_DISPLAY_ON = 0x51,
  S6B33BG_SPECIFIED_DIPLAY_PATTERN = 0x53,
  S6B33BG_PARTIAL_DISPLAY_MODE = 0x55,
  S6B33BG_PARTIAL_DISPLAY_START_LINE = 0x56,
  S6B33BG_PARTIAL_DISPLAY_END_LINE = 0x57,
} COMMANDS;

typedef enum _FONT_SIZE {
  FONT_6x8 = 0,
  FONT_8x8 = 1,
  FONT_8x16 = 2
} FONT_SIZE;

#define LCD_REG8 (*((volatile uint8_t*)0x60000000))
#define LCD_DAT8 (*((volatile uint8_t*)0x60010000))

uint8_t S6B33BG_Init(void);
void S6B33BG_ColorMode(COLOR_MODE color_mode);
void S6B33BG_OrientationMode(ORIENTATION_MODE orientation_mode);

void S6B33BG_ClearScreen(uint32_t color);
void S6B33BG_SetPixel(uint8_t x, uint8_t y, uint32_t color);
void S6B33BG_FillPixel(uint8_t x0,uint8_t y0,uint8_t x1,uint8_t y1,uint32_t *color);
void S6B33BG_Fill(uint8_t x0,uint8_t y0,uint8_t x1,uint8_t y1,uint32_t color);

void S6B33BG_SetWindow(uint8_t x0,uint8_t y0,uint8_t x1,uint8_t y1);

uint8_t S6B33BG_GetWidth(void);
uint8_t S6B33BG_GetHeight(void);

void S6B33BG_Sleep(void);
void S6B33BG_Wakeup(void);

void S6B33BG_SetFont(FONT_SIZE font_size);
void S6B33BG_SetTextColours(uint32_t fColor, uint32_t bColor);
void S6B33BG_PutChar(char c, uint8_t x, uint8_t y);
void S6B33BG_PutStr(char *pString, uint8_t x, uint8_t y);

#endif
