/*
 *  HX8352A.h
 *
 *  Author: Kestutis Bivainis
 *
 */

#ifndef HX8352A_H
#define HX8352A_H

#include "stm32f10x_conf.h"

typedef enum _COLOR_MODE {
  COLOR_16BIT = 1,
  COLOR_18BIT
} COLOR_MODE;

typedef enum _ORIENTATION_MODE {
  ORIENTATION_PORTRAIT = 0x0A,
  ORIENTATION_LANDSCAPE = 0xAA,
  ORIENTATION_PORTRAIT_REV = 0xCA,
  ORIENTATION_LANDSCAPE_REV = 0x6A
} ORIENTATION_MODE;

#define DB0_Pin    GPIO_Pin_14
#define DB0_Speed  GPIO_Speed_50MHz
#define DB0_Mode   GPIO_Mode_AF_PP
#define DB0_Port   GPIOD
#define DB0_Bus    RCC_APB2Periph_GPIOD

#define DB1_Pin    GPIO_Pin_15
#define DB1_Speed  GPIO_Speed_50MHz
#define DB1_Mode   GPIO_Mode_AF_PP
#define DB1_Port   GPIOD
#define DB1_Bus    RCC_APB2Periph_GPIOD

#define DB2_Pin    GPIO_Pin_0
#define DB2_Speed  GPIO_Speed_50MHz
#define DB2_Mode   GPIO_Mode_AF_PP
#define DB2_Port   GPIOD
#define DB2_Bus    RCC_APB2Periph_GPIOD

#define DB3_Pin    GPIO_Pin_1
#define DB3_Speed  GPIO_Speed_50MHz
#define DB3_Mode   GPIO_Mode_AF_PP
#define DB3_Port   GPIOD
#define DB3_Bus    RCC_APB2Periph_GPIOD

#define DB4_Pin    GPIO_Pin_7
#define DB4_Speed  GPIO_Speed_50MHz
#define DB4_Mode   GPIO_Mode_AF_PP
#define DB4_Port   GPIOE
#define DB4_Bus    RCC_APB2Periph_GPIOE

#define DB5_Pin    GPIO_Pin_8
#define DB5_Speed  GPIO_Speed_50MHz
#define DB5_Mode   GPIO_Mode_AF_PP
#define DB5_Port   GPIOE
#define DB5_Bus    RCC_APB2Periph_GPIOE

#define DB6_Pin    GPIO_Pin_9
#define DB6_Speed  GPIO_Speed_50MHz
#define DB6_Mode   GPIO_Mode_AF_PP
#define DB6_Port   GPIOE
#define DB6_Bus    RCC_APB2Periph_GPIOE

#define DB7_Pin    GPIO_Pin_10
#define DB7_Speed  GPIO_Speed_50MHz
#define DB7_Mode   GPIO_Mode_AF_PP
#define DB7_Port   GPIOE
#define DB7_Bus    RCC_APB2Periph_GPIOE

#define DB8_Pin    GPIO_Pin_11
#define DB8_Speed  GPIO_Speed_50MHz
#define DB8_Mode   GPIO_Mode_AF_PP
#define DB8_Port   GPIOE
#define DB8_Bus    RCC_APB2Periph_GPIOE

#define DB9_Pin    GPIO_Pin_12
#define DB9_Speed  GPIO_Speed_50MHz
#define DB9_Mode   GPIO_Mode_AF_PP
#define DB9_Port   GPIOE
#define DB9_Bus    RCC_APB2Periph_GPIOE

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

enum {
  HX8352A_OK = 1,
  HX8352A_ERROR = 0
};

enum COMMANDS {
  DISPLAY_MODE_CONTROL                = 0x01,
  COLUMN_ADDRESS_START_H              = 0x02,
  COLUMN_ADDRESS_START_L              = 0x03,
  COLUMN_ADDRESS_END_H                = 0x04,
  COLUMN_ADDRESS_END_L                = 0x05,
  ROW_ADDRESS_START_H                 = 0x06,
  ROW_ADDRESS_START_L                 = 0x07,
  ROW_ADDRESS_END_H                   = 0x08,
  ROW_ADDRESS_END_L                   = 0x09,
  VERTICAL_SCROLL_TOP_FIXED_AREA_H    = 0x0E,
  VERTICAL_SCROLL_TOP_FIXED_AREA_L    = 0x0F,
  VERTICAL_SCROLL_HEIGHT_H            = 0x10,
  VERTICAL_SCROLL_HEIGHT_L            = 0x11,
  VERTICAL_SCROLL_BOTTOM_FIXED_AREA_H = 0x12,
  VERTICAL_SCROLL_BOTTOM_FIXED_AREA_L = 0x13,
  VERTICAL_SCROLL_START_ADDRESS_H     = 0x14,
  VERTICAL_SCROLL_START_ADDRESS_L     = 0x15,
  MEMORY_ACCESS_CONTROL               = 0x16,
  OSC_CONTROL_1                       = 0x17,
  OSC_CONTROL_2                       = 0x18,
  POWER_CONTROL_1                     = 0x19,
  POWER_CONTROL_2                     = 0x1A,
  POWER_CONTROL_3                     = 0x1B,
  POWER_CONTROL_4                     = 0x1C,
  POWER_CONTROL_5                     = 0x1D,
  POWER_CONTROL_6                     = 0x1E,
  VCOM_CONTROL                        = 0x1F,
  MEMORY_WRITE                        = 0x22,
  DISPLAY_CONTROL_1                   = 0x23,
  DISPLAY_CONTROL_2                   = 0x24,
  DISPLAY_CONTROL_3                   = 0x25,
  DISPLAY_CONTROL_4                   = 0x26,
  DISPLAY_CONTROL_5                   = 0x27,
  DISPLAY_CONTROL_6                   = 0x28,
  DISPLAY_CONTROL_7                   = 0x29,
  DISPLAY_CONTROL_8                   = 0x2A,
  CYCLE_CONTROL_1                     = 0x2B,
  CYCLE_CONTROL_2                     = 0x2C,
  CYCLE_CONTROL_3                     = 0x2D,
  CYCLE_CONTROL_4                     = 0x2E,
  CYCLE_CONTROL_5                     = 0x2F,
  CYCLE_CONTROL_6                     = 0x30,
  CYCLE_CONTROL_7                     = 0x31,
  CYCLE_CONTROL_8                     = 0x32,
  CYCLE_CONTROL_10                    = 0x34,
  CYCLE_CONTROL_11                    = 0x35,
  CYCLE_CONTROL_12                    = 0x36,
  CYCLE_CONTROL_13                    = 0x37,
  CYCLE_CONTROL_14                    = 0x38,
  CYCLE_CONTROL_15                    = 0x39,
  INTERFACE_CONTROL_1                 = 0x3A,
  SOURCE_CONTROL_1                    = 0x3C,
  SOURCE_CONTROL_2                    = 0x3D,
  GAMMA_CONTROL_1                     = 0x3E,
  GAMMA_CONTROL_2                     = 0x3F,
  GAMMA_CONTROL_3                     = 0x40,
  GAMMA_CONTROL_4                     = 0x41,
  GAMMA_CONTROL_5                     = 0x42,
  GAMMA_CONTROL_6                     = 0x43,
  GAMMA_CONTROL_7                     = 0x44,
  GAMMA_CONTROL_8                     = 0x45,
  GAMMA_CONTROL_9                     = 0x46,
  GAMMA_CONTROL_10                    = 0x47,
  GAMMA_CONTROL_11                    = 0x48,
  GAMMA_CONTROL_12                    = 0x49,
  PANEL_CONTROL                       = 0x55,
  IP_CONTROL                          = 0x5A,
  DGC_LUT_WRITE                       = 0x5C,
  TEST_MODE_CONTROL                   = 0x83,
  VDDD_CONTROL                        = 0x85,
  SOURCE_GAMMA_RESISTOR_1             = 0x8B,
  SOURCE_GAMMA_RESISTOR_2             = 0x8C,
  SYNC_FUNCTION                       = 0x91,
  PWM_CONTROL_1                       = 0x95,
  PWM_CONTROL_2                       = 0x96,
  PWM_CONTROL_3                       = 0x97,
};

#define LCD_REG16     (*((volatile uint16_t *) 0x60000000))
#define LCD_DAT16     (*((volatile uint16_t *) 0x60020000))

uint8_t HX8352A_Init(COLOR_MODE color_mode);
void HX8352A_OrientationMode(ORIENTATION_MODE orientation_mode);

void HX8352A_SetWindow(uint16_t x0,uint16_t y0,uint16_t x1,uint16_t y1);

void HX8352A_ClearScreen(uint32_t color);
void HX8352A_ScrollArea(uint16_t y,uint16_t pos);
void HX8352A_SetPixel(uint16_t x,uint16_t y,uint32_t color);
void HX8352A_Fill(uint16_t x0,uint16_t y0,uint16_t x1,uint16_t y1,uint32_t color);
void HX8352A_FillPixel(uint16_t x0,uint16_t y0,uint16_t x1,uint16_t y1,uint32_t *color);
void HX8352A_DrawRectangle(uint16_t x0,uint16_t y0,uint16_t x1,uint16_t y1,uint32_t color);

void HX8352A_StandBy(void);
void HX8352A_Wakeup(void);

uint16_t HX8352A_GetWidth(void);
uint16_t HX8352A_GetHeight(void);

#endif
