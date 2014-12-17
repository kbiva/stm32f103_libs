/*
 *  HD66773R.h
 *
 *  Author: Kestutis Bivainis
 *
 */
#ifndef HD66773R_H
#define HD66773R_H

#include "stm32f10x_conf.h"

typedef enum _ORIENTATION_MODE {
  ORIENTATION_PORTRAIT = 0x0030,
  ORIENTATION_LANDSCAPE = 0x0018,
  ORIENTATION_PORTRAIT_REV = 0x0000,
  ORIENTATION_LANDSCAPE_REV = 0x0028
} ORIENTATION_MODE;

enum {
  HD66773R_OK = 1,
  HD66773R_ERROR = 0
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

typedef enum _COMMANDS {
  STARTOSCILLATION = 0x00,
  DRIVEROUTPUTCTRL = 0x01,
  DRIVINGWAVECTRL = 0x02,
  POWERCTRL1 = 0x03,
  POWERCTRL2 = 0x04,
  ENTRYMODE = 0x05,
  COMPAREREG = 0x06,
  DISPLAYCTRL = 0x07,
  FRAMECYCLECTRL = 0x0B,
  POWERCTRL3 = 0x0C,
  POWERCTRL4 = 0x0D,
  POWERCTRL5 = 0x0E,
  GATESCANPOS = 0x0F,
  VERTICALSCROLLCTRL = 0x11,
  SCREENDRIVEPOS1 = 0x14,
  SCREENDRIVEPOS2 = 0x15,
  HORIZONTALADDRESS = 0x16,
  VERTICALADDRESS = 0x17,
  WRITEDATAMASK = 0x20,
  ADDRESSSET = 0x21,
  GRAMSTARTWRITING = 0x22,
  GAMMACTRL1 = 0x30,
  GAMMACTRL2 = 0x31,
  GAMMACTRL3 = 0x32,
  GAMMACTRL4 = 0x33,
  GAMMACTRL5 = 0x34,
  GAMMACTRL6 = 0x35,
  GAMMACTRL7 = 0x36,
  GAMMACTRL8 = 0x37,
  GAMMACTRL9 = 0x3A,
  GAMMACTRL10 = 0x3B,
} COMMANDS;

typedef enum _FONT_SIZE {
  FONT_6x8 = 0,
  FONT_8x8 = 1,
  FONT_8x14 = 2
} FONT_SIZE;

#define LCD_REG16     (*((volatile uint16_t*)0x60000000))
#define LCD_DAT16     (*((volatile uint16_t*)0x60020000))

uint8_t HD66773R_Init(void);
void HD66773R_OrientationMode(ORIENTATION_MODE orientation_mode);

void HD66773R_ClearScreen(uint32_t color);
void HD66773R_SetPixel(uint8_t x, uint8_t y, uint32_t color);
void HD66773R_FillPixel(uint8_t x0,uint8_t y0,uint8_t x1,uint8_t y1,uint32_t *color);
void HD66773R_Fill(uint8_t x0,uint8_t y0,uint8_t x1,uint8_t y1,uint32_t color);

void HD66773R_SetWindow(uint8_t x0,uint8_t y0,uint8_t x1,uint8_t y1);

uint8_t HD66773R_GetWidth(void);
uint8_t HD66773R_GetHeight(void);

void HD66773R_Sleep(void);
void HD66773R_StandBy(void);
void HD66773R_WakeupFromSleep(void);
void HD66773R_WakeupFromStandBy(void);

void HD66773R_SetFont(FONT_SIZE font_size);
void HD66773R_SetTextColours(uint32_t fColor, uint32_t bColor);
void HD66773R_PutChar(char c, uint8_t x, uint8_t y);
void HD66773R_PutStr(char *pString, uint8_t x, uint8_t y);

#endif
