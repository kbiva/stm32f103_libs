/*
 *  SSD1289.h
 *
 *  Author: Kestutis Bivainis
 *
 */

#ifndef SSD1289_H
#define SSD1289_H

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
#define END_X   319
#define START_Y 0
#define END_Y   239

typedef enum _COLOR_MODE {
  COLOR_16BIT = 0x6030,
  COLOR_18BIT = 0x4070
} COLOR_MODE;

typedef enum _ORIENTATION_MODE {
  ORIENTATION_PORTRAIT = 0x693F,
  ORIENTATION_LANDSCAPE = 0x293F,
  ORIENTATION_PORTRAIT_REV = 0x2B3F,
  ORIENTATION_LANDSCAPE_REV = 0x6B3F
} ORIENTATION_MODE;

enum COMMANDS {
  OSCILLATOR = 0x00,
  DRIVER_OUTPUT_CONTROL = 0x01,
  DRIVING_WAVEFORM_CONTROL = 0x02,
  POWER_CONTROL1 =0x03,
  COMPARE_REGISTER1 = 0x05,
  COMPARE_REGISTER2 = 0x06,
  DISPLAY_CONTROL = 0x07,
  FRAME_CYCLE_CONTROL = 0x0b,
  POWER_CONTROL2 =0x0c,
  POWER_CONTROL3 =0x0d,
  POWER_CONTROL4 =0x0e,
  GATE_SCAN_POSITION = 0xf,
  SLEEP_MODE = 0x10,
  ENTRY_MODE = 0x11,
  HORIZONTAL_PORCH = 0x16,
  VERTICAL_PORCH = 0x17,
  POWER_CONTROL5 =0x1e,
  GRAM_WRITE_DATA = 0x22,
  GRAM_WRITE_DATA_MASK1 = 0x23,
  GRAM_WRITE_DATA_MASK2 = 0x24,
  FRAME_FREQUENCY = 0x25,
  GAMMA_0 = 0x30,
  GAMMA_1 = 0x31,
  GAMMA_2 = 0x32,
  GAMMA_3 = 0x33,
  GAMMA_4 = 0x34,
  GAMMA_5 = 0x35,
  GAMMA_6 = 0x36,
  GAMMA_7 = 0x37,
  GAMMA_8 = 0x3A,
  GAMMA_9 = 0x3B,
  VERTICAL_SCROLL_CONTROL1 = 0x41,
  VERTICAL_SCROLL_CONTROL2 = 0x42,
  HORIZONTAL_POSITION = 0x44,
  VERTICAL_POSITION_START = 0x45,
  VERTICAL_POSITION_END = 0x46,
  FIRST_WINDOW_START = 0x48,
  FIRST_WINDOW_END = 0x49,
  SECOND_WINDOW_START = 0x4a,
  SECOND_WINDOW_END = 0x4b,
  SET_GDDRAM_X = 0x4e,
  SET_GDDRAM_Y = 0x4f
};

#define LCD_REG16     (*((volatile uint16_t*)0x60000000))
#define LCD_DAT16     (*((volatile uint16_t*)0x60020000))

void SSD1289_Init(void);
void SSD1289_OrientationMode(ORIENTATION_MODE orientation_mode);
void SSD1289_ColorMode(COLOR_MODE color_mode);

void SSD1289_Gamma(uint16_t g1_,uint16_t g2_,uint16_t g3_,uint16_t g4_,uint16_t g5_,
                    uint16_t g6_,uint16_t g7_,uint16_t g8_,uint16_t g9_,uint16_t g10_);
void SSD1289_ClearScreen(uint32_t color);
void SSD1289_SetPixel(uint16_t x,uint16_t y,uint32_t color);
void SSD1289_FillPixel(uint16_t x0,uint16_t y0,uint16_t x1,uint16_t y1,uint32_t *color);
void SSD1289_FillRectangle(uint16_t x0,uint16_t y0,uint16_t x1,uint16_t y1,uint32_t color);

void SSD1289_SetScrollPosition(uint16_t pos);
void SSD1289_SetWindow(uint16_t x0,uint16_t y0,uint16_t x1,uint16_t y1);

void SSD1289_Sleep(void);
void SSD1289_Wakeup(void);

uint16_t SSD1289_GetWidth(void);
uint16_t SSD1289_GetHeight(void);

#endif
