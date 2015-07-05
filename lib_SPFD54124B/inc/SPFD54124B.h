/*
 *  SPFD54124B.h
 *
 *  Author: Kestutis Bivainis
 *
 */
#ifndef SPFD54124B_H
#define SPFD54124B_H

#include "stm32f10x_conf.h"

typedef enum _COLOR_MODE {
  COLOR_12BIT = 0x03,
  COLOR_16BIT = 0x05,
  COLOR_18BIT = 0x06
} COLOR_MODE;

typedef enum _ORIENTATION_MODE {
  ORIENTATION_PORTRAIT = 0x00,
  ORIENTATION_LANDSCAPE = 0x60,
  ORIENTATION_PORTRAIT_REV = 0xC0,
  ORIENTATION_LANDSCAPE_REV = 0xA0
} ORIENTATION_MODE;

typedef enum SPFD54124B_GAMMA {
  GAMMA_CURVE1 = 1,
  GAMMA_CURVE2 = 2,
  GAMMA_CURVE3 = 4,
  GAMMA_CURVE4 = 8
} GAMMA_VALUE;

typedef enum SPFD54124B_TE {
  TE_VBLANK = 0x00,
  TE_VBLANK_HBLANK = 0x01,
} TE;

typedef enum _FONT_SIZE {
  FONT_6x8 = 0,
  FONT_8x8 = 1,
  FONT_8x14 = 2
} FONT_SIZE;

enum {
  SPFD54124B_OK = 1,
  SPFD54124B_ERROR = 0
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

enum COMMANDS {
  NOP                               = 0x00,
  SOFTWARE_RESET                    = 0x01,
  READ_DISPLAY_ID                   = 0x04,
  READ_DISPLAY_STATUS               = 0x09,
  READ_DISPLAY_POWER_MODE           = 0x0a,
  READ_DISPLAY_MADCTL               = 0x0b,
  READ_DISPLAY_PIXEL_FORMAT         = 0x0c,
  READ_DISPLAY_IMAGE_MODE           = 0x0d,
  READ_DISPLAY_SIGNAL_MODE          = 0x0e,
  READ_DISPLAY_SELF_DIAGNOSTICS     = 0x0f,
  SLEEP_IN                          = 0x10,
  SLEEP_OUT                         = 0x11,
  PARTIAL_MODE_ON                   = 0x12,
  NORMAL_DISPLAY_MODE_ON            = 0x13,
  DISPLAY_INVERSION_OFF             = 0x20,
  DISPLAY_INVERSION_ON              = 0x21,
  GAMMA_SET                         = 0x26,
  DISPLAY_OFF                       = 0x28,
  DISPLAY_ON                        = 0x29,
  COLUMN_ADDRESS_SET                = 0x2a,
  PAGE_ADDRESS_SET                  = 0x2b,
  MEMORY_WRITE                      = 0x2c,
  COLOUR_SET                        = 0x2d,
  MEMORY_READ                       = 0x2e,
  PARTIAL_AREA                      = 0x30,
  VERTICAL_SCROLLING_DEFINITION     = 0x33,
  TEARING_EFFECT_LINE_OFF           = 0x34,
  TEARING_EFFECT_LINE_ON            = 0x35,
  MEMORY_ACCESS_CONTROL             = 0x36,
  VERTICAL_SCROLLING_START_ADDRESS  = 0x37,
  IDLE_MODE_OFF                     = 0x38,
  IDLE_MODE_ON                      = 0x39,
  INTERFACE_PIXEL_FORMAT            = 0x3a,
  READ_ID1                          = 0xda,
  READ_ID2                          = 0xdb,
  READ_ID3                          = 0xdc
};

#define LCD_REG8     (*((volatile uint8_t*)0x60000000))
#define LCD_DAT8     (*((volatile uint8_t*)0x60010000))

uint8_t SPFD54124B_Init(uint8_t AddressSetupTime,uint8_t DataSetupTime);
void SPFD54124B_ColorMode(COLOR_MODE color_mode);
void SPFD54124B_OrientationMode(ORIENTATION_MODE orientation_mode);

void SPFD54124B_ClearScreen(uint32_t color);
void SPFD54124B_SetPixel(uint8_t x, uint8_t y, uint32_t color);
void SPFD54124B_FillPixel(uint8_t x0,uint8_t y0,uint8_t x1,uint8_t y1,uint32_t *color);
void SPFD54124B_FillFromBuffer(uint8_t x0,uint8_t y0,uint8_t x1,uint8_t y1,uint8_t *data);
void SPFD54124B_Fill(uint8_t x0,uint8_t y0,uint8_t x1,uint8_t y1,uint32_t color);
void SPFD54124B_SetScrollPosition(uint8_t pos);
void SPFD54124B_ScrollArea(uint8_t y,uint8_t pos);
void SPFD54124B_DisplayOff(void);
void SPFD54124B_DisplayOn(void);
void SPFD54124B_Sleep(void);
void SPFD54124B_Wakeup(void);
void SPFD54124B_Gamma(GAMMA_VALUE val);
void SPFD54124B_IdleModeOn(void);
void SPFD54124B_IdleModeOff(void);
void SPFD54124B_DisplayInversionOn(void);
void SPFD54124B_DisplayInversionOff(void);
void SPFD54124B_TearingEffectLineOn(TE val);
void SPFD54124B_TearingEffectLineOff(void);
void SPFD54124B_PartialArea(uint8_t y0,uint8_t y1);
void SPFD54124B_PartialMode(void);
void SPFD54124B_NormalDisplayMode(void);
void SPFD54124B_SetWindow(uint8_t x0,uint8_t y0,uint8_t x1,uint8_t y1);
void SPFD54124B_SetWriteWindow(uint8_t x0,uint8_t y0,uint8_t x1,uint8_t y1);
void SPFD54124B_SetReadWindow(uint8_t x0,uint8_t y0,uint8_t x1,uint8_t y1);
uint16_t SPFD54124B_GetWidth(void);
uint16_t SPFD54124B_GetHeight(void);

void SPFD54124B_SetFont(FONT_SIZE font_size);
void SPFD54124B_SetTextColors(uint32_t fColor, uint32_t bColor);
void SPFD54124B_PutChar(char c, uint8_t x, uint8_t y);
void SPFD54124B_PutStr(char *pString, uint8_t x, uint8_t y);
void SPFD54124B_PutStrCEOL(char *pString, uint8_t x, uint8_t y);
void SPFD54124B_PutStrCentered(char *pString, uint8_t y);

void SPFD54124B_ReadRegister(uint8_t reg,uint8_t length,uint8_t *val);
void SPFD54124B_ReadMemory(uint8_t x0,uint8_t y0,uint8_t x1,uint8_t y1,uint8_t *buf);

#endif
