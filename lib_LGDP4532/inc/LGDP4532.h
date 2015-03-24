/*
 *  LGDP4532.h
 *
 *  Author: Kestutis Bivainis
 *
 */
#ifndef LGDP4532_H
#define LGDP4532_H

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

typedef enum _COLOR_MODE {
  COLOR_16BIT = 0x1000,
  COLOR_18BIT = 0xD000
} COLOR_MODE;

typedef enum _ORIENTATION_MODE {
  ORIENTATION_PORTRAIT = 0x0030,
  ORIENTATION_LANDSCAPE = 0x0018,
  ORIENTATION_PORTRAIT_REV = 0x0000,
  ORIENTATION_LANDSCAPE_REV = 0x0028
} ORIENTATION_MODE;

typedef enum _COMMANDS {
  STARTOSCILLATION = 0x00,
  DRIVEROUTPUTCONTROL = 0x01,
  DRIVINGWAVECONTROL = 0x02,
  ENTRYMODE = 0x03,
  RESIZINGCONTROL = 0x04,
  DISPLAYCTRL1 = 0x07,
  DISPLAYCTRL2 = 0x08,
  DISPLAYCTRL3 = 0x09,
  DISPLAYCTRL4 = 0x0A,
  EPROMCONTROLREGISTER1 = 0x40,
  EPROMCONTROLREGISTER2 = 0x41,
  EPROMCONTROLREGISTER3 = 0x42,
  RGBDISPLAYINTERFACECTRL1 = 0x0C,
  RGBDISPLAYINTERFACECTRL2 = 0x0F,
  FRAMEMARKERPOSITION = 0x0D,
  POWERCTRL1 = 0x10,
  POWERCTRL2 = 0x11,
  POWERCTRL3 = 0x12,
  POWERCTRL4 = 0x13,
  POWERCTRL7 = 0x29,
  HORIZONTALADDRESS = 0x20,
  VERTICALADDRESS = 0x21,
  GRAMSTARTWRITING = 0x22,
  FRAMERATEANDCOLORCONTROL = 0x2B,
  GAMMACONTROL_RED1 = 0x30,
  GAMMACONTROL_RED2 = 0x31,
  GAMMACONTROL_RED3 = 0x32,
  GAMMACONTROL_RED4 = 0x33,
  GAMMACONTROL_RED5 = 0x34,
  GAMMACONTROL_RED6 = 0x35,
  GAMMACONTROL_RED7 = 0x36,
  GAMMACONTROL_RED8 = 0x37,
  GAMMACONTROL_RED9 = 0x38,
  GAMMACONTROL_RED10 = 0x39,
  GAMMACONTROL_RED11 = 0x3A,
  GAMMACONTROL_RED12 = 0x3B,
  GAMMACONTROL_RED13 = 0x3C,
  GAMMACONTROL_RED14 = 0x3D,
  GAMMACONTROL_RED15 = 0x3E,
  GAMMACONTROL_RED16 = 0x3F,
  HORIZONTALRAMPOSITIONSTART = 0x50,
  HORIZONTALRAMPOSITIONEND = 0x51,
  VERTICALRAMPOSITIONSTART = 0x52,
  VERTICALRAMPOSITIONEND = 0x53,
  GATESCANCONTROL1 = 0x60,
  GATESCANCONTROL2 = 0x61,
  GATESCANCONTROLSCROLL = 0x6A,
  PARTIALIMAGE1DISPLAYPOSITION = 0x80,
  PARTIALIMAGE1RAMSTARTADDRESS = 0x81,
  PARTIALIMAGE1RAMENDADDRESS = 0x82,
  PARTIALIMAGE2DISPLAYPOSITION = 0x83,
  PARTIALIMAGE2RAMSTARTADDRESS = 0x84,
  PARTIALIMAGE2RAMENDADDRESS = 0x85,
  PANELINTERFACECONTROL1 = 0x90,
  PANELINTERFACECONTROL2 = 0x92,
  PANELINTERFACECONTROL3 = 0x93,
  PANELINTERFACECONTROL4 = 0x95,
  PANELINTERFACECONTROL5 = 0x97,
  PANELINTERFACECONTROL6 = 0x98,
  REGULATORCONTROL = 0x15,
  TESTREGISTER1 = 0xA0,
  TESTREGISTER2 = 0xA1,
  TESTREGISTER3 = 0xA2,
  TESTREGISTER4 = 0xA3,
  GAMMACONTROL_GREEN1 = 0xB0,
  GAMMACONTROL_GREEN2 = 0xB1,
  GAMMACONTROL_GREEN3 = 0xB2,
  GAMMACONTROL_GREEN4 = 0xB3,
  GAMMACONTROL_GREEN5 = 0xB4,
  GAMMACONTROL_GREEN6 = 0xB5,
  GAMMACONTROL_GREEN7 = 0xB6,
  GAMMACONTROL_GREEN8 = 0xB7,
  GAMMACONTROL_GREEN9 = 0xB8,
  GAMMACONTROL_GREEN10 = 0xB9,
  GAMMACONTROL_GREEN11 = 0xBA,
  GAMMACONTROL_GREEN12 = 0xBB,
  GAMMACONTROL_GREEN13 = 0xBC,
  GAMMACONTROL_GREEN14 = 0xBD,
  GAMMACONTROL_GREEN15 = 0xBE,
  GAMMACONTROL_GREEN16 = 0xBF,
  GAMMACONTROL_BLUE1 = 0xC0,
  GAMMACONTROL_BLUE2 = 0xC1,
  GAMMACONTROL_BLUE3 = 0xC2,
  GAMMACONTROL_BLUE4 = 0xC3,
  GAMMACONTROL_BLUE5 = 0xC4,
  GAMMACONTROL_BLUE6 = 0xC5,
  GAMMACONTROL_BLUE7 = 0xC6,
  GAMMACONTROL_BLUE8 = 0xC7,
  GAMMACONTROL_BLUE9 = 0xC8,
  GAMMACONTROL_BLUE10 = 0xC9,
  GAMMACONTROL_BLUE11 = 0xCA,
  GAMMACONTROL_BLUE12 = 0xCB,
  GAMMACONTROL_BLUE13 = 0xCC,
  GAMMACONTROL_BLUE14 = 0xCD,
  GAMMACONTROL_BLUE15 = 0xCE,
  GAMMACONTROL_BLUE16 = 0xCF,
  TIMINGCTRL1 = 0xE3,
  TIMINGCTRL2 = 0xE7,
  TIMINGCTRL3 = 0xEF
} COMMANDS;

typedef enum _FONT_SIZE {
  FONT_6x8 = 0,
  FONT_8x8 = 1,
  FONT_8x14 = 2
} FONT_SIZE;

#define LCD_REG16     (*((volatile uint16_t*)0x60000000))
#define LCD_DAT16     (*((volatile uint16_t*)0x60020000))

void LGDP4532_Init(void);
//void LGDP4532_Gamma(uint16_t g1_,uint16_t g2_,uint16_t g3_,uint16_t g4_,uint16_t g5_,
//                    uint16_t g6_,uint16_t g7_,uint16_t g8_,uint16_t g9_,uint16_t g10_);
void LGDP4532_ClearScreen(uint32_t color);
void LGDP4532_SetPixel(uint16_t x,uint16_t y,uint32_t color);
void LGDP4532_FillPixel(uint16_t x0,uint16_t y0,uint16_t x1,uint16_t y1,uint32_t *color);

void LGDP4532_Fill(uint16_t x0,uint16_t y0,uint16_t x1,uint16_t y1,uint32_t color);
void LGDP4532_FillFromBuffer(uint16_t x0,uint16_t y0,uint16_t x1,uint16_t y1,uint8_t *data);

void LGDP4532_SetScrollPosition(uint16_t pos);
void LGDP4532_SetWindow(uint16_t x0,uint16_t y0,uint16_t x1,uint16_t y1);

void LGDP4532_Sleep(void);
void LGDP4532_Wakeup(void);

void LGDP4532_FillPixel_16bit(uint16_t x0,uint16_t y0,uint16_t x1,uint16_t y1,uint16_t *color);
void LGDP4532_Circle(uint16_t x0, uint16_t y0, uint16_t radius, uint32_t color);

void LGDP4532_OrientationMode(ORIENTATION_MODE orientation_mode);
void LGDP4532_ColorMode(COLOR_MODE color_mode);

uint16_t LGDP4532_GetWidth(void);
uint16_t LGDP4532_GetHeight(void);

void LGDP4532_SetFont(FONT_SIZE font_size);
void LGDP4532_SetTextColors(uint32_t fColor, uint32_t bColor);
void LGDP4532_PutChar(char c, uint16_t x, uint16_t y);
void LGDP4532_PutStr(char *pString, uint16_t x, uint16_t y);
void LGDP4532_PutStrCEOL(char *pString, uint16_t x, uint16_t y);
void LGDP4532_PutStrCentered(char *pString, uint16_t y);

#endif
