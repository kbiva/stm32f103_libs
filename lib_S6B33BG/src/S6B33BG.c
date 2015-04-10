/*
 *  S6B33BG.c
 *
 *  Author: Kestutis Bivainis
 *
 */

#include "stm32f10x_conf.h"
#include "S6B33BG.h"
#include "colors.h"
#include "delay.h"
#include "font6x8.h"
#include "font8x8.h"
#include "font8x14.h"

static FONT_SIZE S6B33BG_font_size;
static uint32_t S6B33BG_text_foreground_color=WHITE;
static uint32_t S6B33BG_text_background_color=BLACK;

static COLOR_MODE S6B33BG_color_mode;

static unsigned char *FontTable[] = {
    (unsigned char *)FONT6x8,
    (unsigned char *)FONT8x8,
    (unsigned char *)FONT8x14
};

static PIN pins[]={
  {{DB0_Pin,DB0_Speed,DB0_Mode},DB0_Port,DB0_Bus},
  {{DB1_Pin,DB1_Speed,DB1_Mode},DB1_Port,DB1_Bus},
  {{DB2_Pin,DB2_Speed,DB2_Mode},DB2_Port,DB2_Bus},
  {{DB3_Pin,DB3_Speed,DB3_Mode},DB3_Port,DB3_Bus},
  {{DB4_Pin,DB4_Speed,DB4_Mode},DB4_Port,DB4_Bus},
  {{DB5_Pin,DB5_Speed,DB5_Mode},DB5_Port,DB5_Bus},
  {{DB6_Pin,DB6_Speed,DB6_Mode},DB6_Port,DB6_Bus},
  {{DB7_Pin,DB7_Speed,DB7_Mode},DB7_Port,DB7_Bus},
  {{RW_Pin, RW_Speed, RW_Mode}, RW_Port, RW_Bus},
  {{RD_Pin, RD_Speed, RD_Mode}, RD_Port, RD_Bus},
  {{RS_Pin, RS_Speed, RS_Mode}, RS_Port, RS_Bus},
  {{CS_Pin, CS_Speed, CS_Mode}, CS_Port, CS_Bus},
  {{RST_Pin,RST_Speed,RST_Mode},RST_Port,RST_Bus},
};


static void FSMC_LCD_Init(uint8_t AddressSetupTime,uint8_t DataSetupTime) {

  FSMC_NORSRAMInitTypeDef FSMC_NORSRAMInitStructure;
  FSMC_NORSRAMTimingInitTypeDef  FSMC_NORSRAMTimingInitStructure;

  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_FSMC, ENABLE);

  FSMC_NORSRAMTimingInitStructure.FSMC_AddressSetupTime = AddressSetupTime;
  FSMC_NORSRAMTimingInitStructure.FSMC_AddressHoldTime = 0x00;
  FSMC_NORSRAMTimingInitStructure.FSMC_DataSetupTime = DataSetupTime;
  FSMC_NORSRAMTimingInitStructure.FSMC_BusTurnAroundDuration = 0x00;
  FSMC_NORSRAMTimingInitStructure.FSMC_CLKDivision = 0x00;
  FSMC_NORSRAMTimingInitStructure.FSMC_DataLatency = 0x00;
  FSMC_NORSRAMTimingInitStructure.FSMC_AccessMode = FSMC_AccessMode_B;

  FSMC_NORSRAMInitStructure.FSMC_Bank = FSMC_Bank1_NORSRAM1;
  FSMC_NORSRAMInitStructure.FSMC_DataAddressMux = FSMC_DataAddressMux_Disable;
  FSMC_NORSRAMInitStructure.FSMC_MemoryType = FSMC_MemoryType_SRAM;
  FSMC_NORSRAMInitStructure.FSMC_MemoryDataWidth = FSMC_MemoryDataWidth_8b;
  FSMC_NORSRAMInitStructure.FSMC_BurstAccessMode = FSMC_BurstAccessMode_Enable;
  //FSMC_NORSRAMInitStructure.FSMC_BurstAccessMode = FSMC_BurstAccessMode_Disable;
  FSMC_NORSRAMInitStructure.FSMC_WaitSignalPolarity = FSMC_WaitSignalPolarity_Low;
  FSMC_NORSRAMInitStructure.FSMC_WrapMode = FSMC_WrapMode_Disable;
  FSMC_NORSRAMInitStructure.FSMC_WaitSignalActive = FSMC_WaitSignalActive_BeforeWaitState;
  FSMC_NORSRAMInitStructure.FSMC_WriteOperation = FSMC_WriteOperation_Enable;
  FSMC_NORSRAMInitStructure.FSMC_WaitSignal = FSMC_WaitSignal_Disable;
  FSMC_NORSRAMInitStructure.FSMC_ExtendedMode = FSMC_ExtendedMode_Disable;
  FSMC_NORSRAMInitStructure.FSMC_WriteBurst = FSMC_WriteBurst_Enable;
  //FSMC_NORSRAMInitStructure.FSMC_WriteBurst = FSMC_WriteBurst_Disable;
  FSMC_NORSRAMInitStructure.FSMC_AsynchronousWait=FSMC_AsynchronousWait_Disable;
  FSMC_NORSRAMInitStructure.FSMC_ReadWriteTimingStruct = &FSMC_NORSRAMTimingInitStructure;
  FSMC_NORSRAMInitStructure.FSMC_WriteTimingStruct = &FSMC_NORSRAMTimingInitStructure;

  FSMC_NORSRAMInit(&FSMC_NORSRAMInitStructure);

  FSMC_NORSRAMCmd(FSMC_Bank1_NORSRAM1, ENABLE);
}

static void GPIO_Configuration(void){

  uint32_t i;

  for(i=0;i<sizeof(pins)/sizeof(PIN);i++) {
    RCC_APB2PeriphClockCmd(pins[i].GPIO_Bus,ENABLE);
    GPIO_Init(pins[i].GPIOx,&pins[i].GPIO_InitStructure);
  }
}

static void lcd_rst(void) {

  GPIO_ResetBits(RST_Port, RST_Pin);
  DWT_Delay(100000);
  GPIO_SetBits(RST_Port, RST_Pin);
  DWT_Delay(100000);
}

static __forceinline
void wr_cmd(uint8_t index) {
  LCD_REG8 = index;
}

static __forceinline
void wr_dat(uint8_t val) {
  LCD_DAT8 = val;
}

uint8_t S6B33BG_Init(uint8_t AddressSetupTime,uint8_t DataSetupTime) {

  GPIO_Configuration();
  FSMC_LCD_Init(AddressSetupTime,DataSetupTime);

  lcd_rst();

  wr_cmd(S6B33BG_STANDBY_MODE_OFF);
  DWT_Delay(20000);
  wr_cmd(S6B33BG_OSCILATION_MODE);
  wr_cmd(0x01);// OSC ON
  DWT_Delay(20000);
  wr_cmd(S6B33BG_DC_DC_AMP);
  wr_cmd(0x01);// DCDC1 ON
  DWT_Delay(20000);
  wr_cmd(S6B33BG_DC_DC_AMP);
  wr_cmd(0x09);// AMP ON
  DWT_Delay(20000);
  wr_cmd(S6B33BG_DC_DC_AMP);
  wr_cmd(0x0B);// DCDC2 ON
  DWT_Delay(20000);
  wr_cmd(S6B33BG_DC_DC_AMP);
  wr_cmd(0x0F);// DCDC3 ON
  DWT_Delay(20000);
  S6B33BG_OrientationMode(ORIENTATION_PORTRAIT);
  wr_cmd(S6B33BG_DC_DC_SELECT);
  wr_cmd(0x02);
  wr_cmd(S6B33BG_DC_DC_CLK_DIVISION);
  wr_cmd(0x08);
  wr_cmd(S6B33BG_TEMPERATURE_COMPENSATION);
  wr_cmd(0x02);
  wr_cmd(S6B33BG_CONTRAST_CTRL);
  wr_cmd(0x41);// Contrast
  S6B33BG_ColorMode(COLOR_16BIT);
  wr_cmd(S6B33BG_ROW_VECTOR_MODE);
  wr_cmd(0x0E);
  wr_cmd(S6B33BG_NBLOCK_INVERSION);
  wr_cmd(0xD0);
  wr_cmd(S6B33BG_DRIVING_MODE);
  wr_cmd(0x00);
  wr_cmd(S6B33BG_ENTRY_MODE);
  wr_cmd(0x80);// 8bit data bus
  wr_cmd(S6B33BG_RAM_SKIP_AREA);
  wr_cmd(0x00);
  wr_cmd(S6B33BG_SPECIFIED_DIPLAY_PATTERN);
  wr_cmd(0x00);
  wr_cmd(S6B33BG_PARTIAL_DISPLAY_MODE);
  wr_cmd(0x00);
  DWT_Delay(100000);
  wr_cmd(S6B33BG_DISPLAY_ON);

  return S6B33BG_OK;
}

void S6B33BG_ColorMode(COLOR_MODE color_mode) {

  wr_cmd(S6B33BG_ADDRESSING_MODE);
  wr_cmd(0x1B|color_mode);
  S6B33BG_color_mode=color_mode;

}

void S6B33BG_OrientationMode(ORIENTATION_MODE orientation_mode) {

  wr_cmd(S6B33BG_DRIVER_OUTPUT_MODE);
  wr_cmd(0x03|orientation_mode);
}

void S6B33BG_ClearScreen(uint32_t color) {

  S6B33BG_Fill(0,0,131,131,color);
}

void S6B33BG_Fill(uint8_t x0,uint8_t y0,uint8_t x1,uint8_t y1,uint32_t color) {

  uint32_t i,j=(x1-x0+1)*(y1-y0+1);
  uint8_t b1,b2,r,g,b;

  S6B33BG_SetWindow(x0,y0,x1,y1);

  r=color>>16;
  g=color>>8;
  b=color;

  switch(S6B33BG_color_mode) {
    case COLOR_12BIT:
      b1=(r&0xF0)>>4;
      b2=(g&0xF0)|((b&0xF0)>>4);
      break;
    case COLOR_16BIT:
      b1=(r&0xF8)|((g&0xE0)>>5);
      b2=((g&0x1C)<<3)|((b&0xF8)>>3);
      break;
  }

  for(i=0;i<j;i++) {
    wr_dat(b1);
    wr_dat(b2);
  }
}

void S6B33BG_SetWindow(uint8_t x0,uint8_t y0,uint8_t x1,uint8_t y1) {

  wr_cmd(S6B33BG_ROW_ADDRESS_AREA);
  wr_cmd(y0);
  wr_cmd(y1);
  wr_cmd(S6B33BG_COLUMN_ADDRESS_AREA);
  wr_cmd(x0);
  wr_cmd(x1);
}

void S6B33BG_FillPixel(uint8_t x0,uint8_t y0,uint8_t x1,uint8_t y1,uint32_t *color) {

  uint32_t i,j=(x1-x0+1)*(y1-y0+1);

  S6B33BG_SetWindow(x0,y0,x1,y1);

  switch(S6B33BG_color_mode) {
    case COLOR_12BIT:
      for(i=0;i<j;i++) {
        wr_dat(((color[i]>>20)&0x0F));
        wr_dat(((color[i]>>12)&0xF0)|((color[i]>>4)&0x0F));
      }
      break;
    case COLOR_16BIT:
      for(i=0;i<j;i++) {
        wr_dat(((color[i]>>16)&0xF8)|(((color[i]>>13)&0x07)));
        wr_dat(((color[i]>>5)&0xE0)|((color[i]>>3)&0x1F));
      }
      break;
  }
}

void S6B33BG_SetPixel(uint8_t x, uint8_t y, uint32_t color) {

  uint8_t r,g,b,b1,b2;

  S6B33BG_SetWindow(x,y,x,y);

  r=color>>16;
  g=color>>8;
  b=color;

  switch(S6B33BG_color_mode) {
    case COLOR_12BIT:
      b1=(r&0xF0)>>4;
      b2=(g&0xF0)|((b&0xF0)>>4);
      break;
    case COLOR_16BIT:
      b1=(r&0xF8)|((g>>5)&0x07);
      b2=((g<<3)&0x0E)|((b>>3)&0x1F);
      break;
  }
  wr_dat(b1);
  wr_dat(b2);
}

uint8_t S6B33BG_GetWidth(void) {

  return 132;
}

uint8_t S6B33BG_GetHeight(void) {

  return 132;
}

void S6B33BG_Sleep(void) {

  wr_cmd(S6B33BG_DISPLAY_OFF);
  wr_cmd(S6B33BG_STANDBY_MODE_ON);
  wr_cmd(S6B33BG_DC_DC_AMP);
  wr_cmd(0x00);
  wr_cmd(S6B33BG_OSCILATION_MODE);
  wr_cmd(0x00);
  DWT_Delay(300000);
}

void S6B33BG_Wakeup(void) {

  wr_cmd(S6B33BG_STANDBY_MODE_OFF);
  DWT_Delay(20000);
  wr_cmd(S6B33BG_OSCILATION_MODE);
  wr_cmd(0x01);// OSC ON
  DWT_Delay(20000);
  wr_cmd(S6B33BG_DC_DC_AMP);
  wr_cmd(0x01);// DCDC1 ON
  DWT_Delay(20000);
  wr_cmd(S6B33BG_DC_DC_AMP);
  wr_cmd(0x09);// AMP ON
  DWT_Delay(20000);
  wr_cmd(S6B33BG_DC_DC_AMP);
  wr_cmd(0x0B);// DCDC2 ON
  DWT_Delay(20000);
  wr_cmd(S6B33BG_DC_DC_AMP);
  wr_cmd(0x0F);// DCDC3 ON
  DWT_Delay(100000);
  wr_cmd(S6B33BG_DISPLAY_ON);
}

void S6B33BG_SetFont(FONT_SIZE font_size) {

  S6B33BG_font_size=font_size;
}

void S6B33BG_SetTextColours(uint32_t fColor, uint32_t bColor) {

  S6B33BG_text_foreground_color = fColor;
  S6B33BG_text_background_color = bColor;
}

void S6B33BG_PutChar(char c, uint8_t x, uint8_t y) {

  uint32_t i,j;
  uint32_t nCols;
  uint32_t nRows;
  uint32_t nBytes;
  uint8_t PixelRow;
  uint8_t Mask;
  uint32_t Word0;
  uint32_t Word1;
  unsigned char *pFont;
  unsigned char *pChar;
  uint8_t r,g,b;

  // get pointer to the beginning of the selected font table
  pFont = (unsigned char *)FontTable[S6B33BG_font_size];

  // get the nColumns, nRows and nBytes
  nCols = *pFont;
  nRows = *(pFont + 1);
  nBytes = *(pFont + 2);

  // get pointer to the first byte of the desired character
  pChar = pFont + (nBytes * (c - 0x1F));

  S6B33BG_SetWindow(x,y,x + nCols - 1,y + nRows - 1);

  // loop on each row
  for (i = 0; i < nRows; i++) {

    // copy pixel row from font table and then decrement row
    PixelRow = *pChar++;

    // loop on each pixel in the row (left to right)
    // Note: we do two pixels each loop
    Mask = 0x80;
    for (j = 0; j < nCols; j += 2) {

      // if pixel bit set, use foreground color; else use the background color
      // now get the pixel color for two successive pixels
      if (PixelRow & Mask)
        Word0 = S6B33BG_text_foreground_color;
      else
        Word0 = S6B33BG_text_background_color;

      Mask >>= 1;

      if (PixelRow & Mask)
        Word1 = S6B33BG_text_foreground_color;
      else
        Word1 = S6B33BG_text_background_color;

      Mask >>= 1;

      switch(S6B33BG_color_mode) {
        case COLOR_12BIT:
          r=Word0>>16;
          g=Word0>>8;
          b=Word0;
          wr_dat((r&0xF0)>>4);
          wr_dat((g&0xF0)|((b&0xF0)>>4));
          r=Word1>>16;
          g=Word1>>8;
          b=Word1;
          wr_dat((r&0xF0)>>4);
          wr_dat((g&0xF0)|((b&0xF0)>>4));
          break;
        case COLOR_16BIT:
          r=Word0>>16;
          g=Word0>>8;
          b=Word0;
          wr_dat((r&0xF8)|((g>>5)&0x07));
          wr_dat(((g<<3)&0x0E)|((b>>3)&0x1F));
          r=Word1>>16;
          g=Word1>>8;
          b=Word1;
          wr_dat((r&0xF8)|((g>>5)&0x07));
          wr_dat(((g<<3)&0x0E)|((b>>3)&0x1F));
          break;
      }
    }
  }
}

void S6B33BG_PutStr(char *pString, uint8_t x, uint8_t y) {

  if(y+FontTable[S6B33BG_font_size][1]>S6B33BG_GetHeight())
    return;

  // loop until null-terminator is seen
  while (*pString) {
    if (x+FontTable[S6B33BG_font_size][0]>S6B33BG_GetWidth()) break;
    // draw the character
    S6B33BG_PutChar(*pString++, x, y);
    x+=FontTable[S6B33BG_font_size][0];
  }
}
