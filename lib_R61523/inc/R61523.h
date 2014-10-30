/*
 *  R61523.h
 *
 *  Author: Kestutis Bivainis
 *
 */
 
#ifndef R61523_H
#define R61523_H

#include "stm32f10x_conf.h"

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

#define DB8_Pin   GPIO_Pin_11
#define DB8_Speed GPIO_Speed_50MHz
#define DB8_Mode  GPIO_Mode_AF_PP
#define DB8_Port  GPIOE
#define DB8_Bus   RCC_APB2Periph_GPIOE

#define DB9_Pin   GPIO_Pin_12
#define DB9_Speed GPIO_Speed_50MHz
#define DB9_Mode  GPIO_Mode_AF_PP
#define DB9_Port  GPIOE
#define DB9_Bus   RCC_APB2Periph_GPIOE

#define DB10_Pin   GPIO_Pin_13
#define DB10_Speed GPIO_Speed_50MHz
#define DB10_Mode  GPIO_Mode_AF_PP
#define DB10_Port  GPIOE
#define DB10_Bus   RCC_APB2Periph_GPIOE

#define DB11_Pin   GPIO_Pin_14
#define DB11_Speed GPIO_Speed_50MHz
#define DB11_Mode  GPIO_Mode_AF_PP
#define DB11_Port  GPIOE
#define DB11_Bus   RCC_APB2Periph_GPIOE

#define DB12_Pin   GPIO_Pin_15
#define DB12_Speed GPIO_Speed_50MHz
#define DB12_Mode  GPIO_Mode_AF_PP
#define DB12_Port  GPIOE
#define DB12_Bus   RCC_APB2Periph_GPIOE

#define DB13_Pin   GPIO_Pin_8
#define DB13_Speed GPIO_Speed_50MHz
#define DB13_Mode  GPIO_Mode_AF_PP
#define DB13_Port  GPIOD
#define DB13_Bus   RCC_APB2Periph_GPIOD

#define DB14_Pin   GPIO_Pin_9
#define DB14_Speed GPIO_Speed_50MHz
#define DB14_Mode  GPIO_Mode_AF_PP
#define DB14_Port  GPIOD
#define DB14_Bus   RCC_APB2Periph_GPIOD

#define DB15_Pin   GPIO_Pin_10
#define DB15_Speed GPIO_Speed_50MHz
#define DB15_Mode  GPIO_Mode_AF_PP
#define DB15_Port  GPIOD
#define DB15_Bus   RCC_APB2Periph_GPIOD

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

#define START_X 0
#define END_X   639
#define START_Y 0
#define END_Y   359

typedef enum _COLOR_MODE {
  COLOR_16BIT = 0x0005,
  COLOR_18BIT = 0x0006,
  COLOR_24BIT = 0x0007
} COLOR_MODE;

typedef enum _ORIENTATION_MODE {
  ORIENTATION_PORTRAIT = 0x0040,
  ORIENTATION_LANDSCAPE = 0x00E0,
  ORIENTATION_PORTRAIT_REV = 0x0080,
  ORIENTATION_LANDSCAPE_REV = 0x0020
} ORIENTATION_MODE;

enum COMMANDS {
  NOP                     = 0x00,
  SOFTWARE_RESET          = 0x01,
  SLEEP_IN                = 0x10,
  SLEEP_OUT               = 0x11,
  PARTIAL_MODE_ON         = 0x12,
  NORMAL_DISPLAY_MODE_ON  = 0x13,
  DISPLAY_OFF             = 0x28,
  DISPLAY_ON              = 0x29,
  SET_COLUMN_ADDRESS      = 0x2a,
  SET_PAGE_ADDRESS        = 0x2b,
  MEMORY_WRITE            = 0x2c,
  MEMORY_READ             = 0x2e,
  PARTIAL_AREA            = 0x30,
  SET_TEAR_OFF            = 0x34,
  SET_TEAR_ON             = 0x35,
  SET_ADDRESS_MODE        = 0x36,
  IDLE_MODE_OFF           = 0x38,
  IDLE_MODE_ON            = 0x39,
  SET_PIXEL_FORMAT        = 0x3a,
  MCAP                    = 0xb0,
  SET_FRAME_AND_INTERFACE = 0xb3,
  BACKLIGHT_CONTROL_1     = 0xb8,
  BACKLIGHT_CONTROL_2     = 0xb9,
  BACKLIGHT_CONTROL_3     = 0xba,
  DEVICE_CODE_READ        = 0xbf,
  PANEL_DRIVING_SETTING   = 0xc0,
  GAMMA_SET_A             = 0xc8,
  GAMMA_SET_B             = 0xc9,
  POWER_SETTING_COMMON    = 0xd0,
  GAMMA_SET_C             = 0xca
};

#define LCD_REG16     (*((volatile uint16_t*)0x60000000))
#define LCD_DAT16     (*((volatile uint16_t*)0x60020000)) 

void R61523_Init(void);
void R61523_OrientationMode(ORIENTATION_MODE orientation_mode);
void R61523_ColorMode(COLOR_MODE color_mode);

void R61523_Gamma(uint16_t g1_,uint16_t g2_,uint16_t g3_,uint16_t g4_,uint16_t g5_,
                    uint16_t g6_,uint16_t g7_,uint16_t g8_,uint16_t g9_,uint16_t g10_,uint16_t g11_,uint16_t g12_,uint16_t g13_);
void R61523_ClearScreen(uint32_t color);
void R61523_SetPixel(uint16_t x,uint16_t y,uint32_t color);
void R61523_FillPixel(uint16_t x0,uint16_t y0,uint16_t x1,uint16_t y1,uint32_t *color);
void R61523_FillRectangle(uint16_t x0,uint16_t y0,uint16_t x1,uint16_t y1,uint32_t color);

void R61523_SetWindow(uint16_t x0,uint16_t y0,uint16_t x1,uint16_t y1);

void R61523_Sleep(void);
void R61523_Wakeup(void);

uint16_t R61523_GetWidth(void);
uint16_t R61523_GetHeight(void);

#endif
