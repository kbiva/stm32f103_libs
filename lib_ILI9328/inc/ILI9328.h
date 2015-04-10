/*
 *  ILI9328.h
 *
 *  Author: Kestutis Bivainis
 *
 */

#ifndef ILI9328_H
#define ILI9328_H

#include "stm32f10x_conf.h"

typedef enum _COLOR_MODE {
  COLOR_16BIT = 0x1000,
  COLOR_18BIT = 0xD000
} COLOR_MODE;

typedef enum _ORIENTATION_MODE {
  ORIENTATION_PORTRAIT = 0x0000,
  ORIENTATION_LANDSCAPE = 0x0028,
  ORIENTATION_PORTRAIT_REV = 0x0030,
  ORIENTATION_LANDSCAPE_REV = 0x0018
} ORIENTATION_MODE;

enum {
  ILI9328_OK = 1,
  ILI9328_ERROR = 0
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
  ILI9328_DRIVER_OUTPUT_CTRL = 0x01,
  ILI9328_LCD_DRIVING_CTRL = 0x02,
  ILI9328_ENTRY_MODE = 0x03,
  ILI9328_RESIZE_CTRL = 0x04,
  ILI9328_DISPLAY_CTRL1 = 0x07,
  ILI9328_DISPLAY_CTRL2 = 0x08,
  ILI9328_DISPLAY_CTRL3 = 0x09,
  ILI9328_DISPLAY_CTRL4 = 0x0A,
  ILI9328_RGB_DISPLAY_IF_CTRL1 = 0x0C,
  ILI9328_FRAME_MARKER_POS = 0x0D,
  ILI9328_RGB_DISPLAY_IF_CTRL2 = 0x0F,
  ILI9328_POWER_CTRL1 = 0x10,
  ILI9328_POWER_CTRL2 = 0x11,
  ILI9328_POWER_CTRL3 = 0x12,
  ILI9328_POWER_CTRL4 = 0x13,
  ILI9328_GRAM_HORZ_AD = 0x20,
  ILI9328_GRAM_VERT_AD = 0x21,
  ILI9328_RW_GRAM = 0x22,
  ILI9328_POWER_CTRL7 = 0x29,
  ILI9328_FRAME_RATE_COLOR_CTRL = 0x2B,
  ILI9328_GAMMA_CTRL1 = 0x30,
  ILI9328_GAMMA_CTRL2 = 0x31,
  ILI9328_GAMMA_CTRL3 = 0x32,
  ILI9328_GAMMA_CTRL4 = 0x35,
  ILI9328_GAMMA_CTRL5 = 0x36,
  ILI9328_GAMMA_CTRL6 = 0x37,
  ILI9328_GAMMA_CTRL7 = 0x38,
  ILI9328_GAMMA_CTRL8 = 0x39,
  ILI9328_GAMMA_CTRL9 = 0x3C,
  ILI9328_GAMMA_CTRL10 = 0x3D,
  ILI9328_HORZ_START_AD = 0x50,
  ILI9328_HORZ_END_AD = 0x51,
  ILI9328_VERT_START_AD = 0x52,
  ILI9328_VERT_END_AD = 0x53,
  ILI9328_GATE_SCAN_CTRL1 = 0x60,
  ILI9328_GATE_SCAN_CTRL2 = 0x61,
  ILI9328_GATE_SCAN_CTRL3 = 0x6A,
  ILI9328_PART_IMG1_DISP_POS = 0x80,
  ILI9328_PART_IMG1_START_AD = 0x81,
  ILI9328_PART_IMG1_END_AD = 0x82,
  ILI9328_PART_IMG2_DISP_POS = 0x83,
  ILI9328_PART_IMG2_START_AD = 0x84,
  ILI9328_PART_IMG2_END_AD = 0x85,
  ILI9328_PANEL_IF_CTRL1 = 0x90,
  ILI9328_PANEL_IF_CTRL2 = 0x92,
  ILI9328_PANEL_IF_CTRL4 = 0x95,
} COMMANDS;

#define LCD_REG8     (*((volatile uint8_t*)0x60000000))
#define LCD_DAT8     (*((volatile uint8_t*)0x60010000))

uint8_t ILI9328_Init(uint8_t AddressSetupTime,uint8_t DataSetupTime);
void ILI9328_ColorMode(COLOR_MODE color_mode);
void ILI9328_OrientationMode(ORIENTATION_MODE orientation_mode);

void ILI9328_ClearScreen(uint32_t color);
void ILI9328_SetPixel(uint16_t x, uint16_t y, uint32_t color);
void ILI9328_FillPixel(uint16_t x0,uint16_t y0,uint16_t x1,uint16_t y1,uint32_t *color);
void ILI9328_Fill(uint16_t x0,uint16_t y0,uint16_t x1,uint16_t y1,uint32_t color);

void ILI9328_Sleep(void);
void ILI9328_Wakeup(void);

void ILI9328_SetWindow(uint16_t x0,uint16_t y0,uint16_t x1,uint16_t y1);
uint16_t ILI9328_GetWidth(void);
uint16_t ILI9328_GetHeight(void);

#endif
