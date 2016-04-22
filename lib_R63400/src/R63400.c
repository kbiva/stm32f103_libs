/*
 *  R63400.c
 *
 *  Library for Renesas R63400.
 *  Tested with LCD from Sony Ericsson K800i mobile phone.
 *
 *  Author: Kestutis Bivainis
 *
 */

#include "stm32f10x_conf.h"
#include "R63400.h"
#include "delay.h"
#include "colors.h"
#include "font6x8.h"
#include "font8x8.h"
#include "font8x14.h"
#include <string.h>

static uint32_t R63400_text_foreground_color=WHITE;
static uint32_t R63400_text_background_color=BLACK;
static FONT_SIZE R63400_font_size;

static unsigned char *FontTable[] = {
    (unsigned char *)FONT6x8,
    (unsigned char *)FONT8x8,
    (unsigned char *)FONT8x14
};

static COLOR_MODE R63400_color_mode=COLOR_16BIT;
static ORIENTATION_MODE R63400_orientation_mode=ORIENTATION_LANDSCAPE;

static uint16_t DISPLAY_WIDTH=320;
static uint16_t DISPLAY_HEIGHT=240;

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

static void GPIO_Configuration(void) {

  uint32_t i;

  for(i=0;i<sizeof(pins)/sizeof(PIN);i++) {
    RCC_APB2PeriphClockCmd(pins[i].GPIO_Bus,ENABLE);
    GPIO_Init(pins[i].GPIOx,&pins[i].GPIO_InitStructure);
  }
}

static void FSMC_LCD_Init(uint8_t AddressSetupTime,uint8_t DataSetupTime) {

  FSMC_NORSRAMInitTypeDef FSMC_NORSRAMInitStructure;
  FSMC_NORSRAMTimingInitTypeDef  FSMC_NORSRAMTimingInitStructure;

  /* Enable the FSMC Clock */
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
  FSMC_NORSRAMInitStructure.FSMC_MemoryType = FSMC_MemoryType_NOR;
  FSMC_NORSRAMInitStructure.FSMC_MemoryDataWidth = FSMC_MemoryDataWidth_8b;
  //FSMC_NORSRAMInitStructure.FSMC_BurstAccessMode = FSMC_BurstAccessMode_Enable;
  FSMC_NORSRAMInitStructure.FSMC_BurstAccessMode = FSMC_BurstAccessMode_Disable;
  FSMC_NORSRAMInitStructure.FSMC_WaitSignalPolarity = FSMC_WaitSignalPolarity_Low;
  FSMC_NORSRAMInitStructure.FSMC_WrapMode = FSMC_WrapMode_Disable;
  FSMC_NORSRAMInitStructure.FSMC_WaitSignalActive = FSMC_WaitSignalActive_BeforeWaitState;
  FSMC_NORSRAMInitStructure.FSMC_WriteOperation = FSMC_WriteOperation_Enable;
  FSMC_NORSRAMInitStructure.FSMC_WaitSignal = FSMC_WaitSignal_Disable;
  FSMC_NORSRAMInitStructure.FSMC_ExtendedMode = FSMC_ExtendedMode_Disable;
  //FSMC_NORSRAMInitStructure.FSMC_WriteBurst = FSMC_WriteBurst_Enable;
  FSMC_NORSRAMInitStructure.FSMC_WriteBurst = FSMC_WriteBurst_Disable;
  FSMC_NORSRAMInitStructure.FSMC_ReadWriteTimingStruct = &FSMC_NORSRAMTimingInitStructure;
  FSMC_NORSRAMInitStructure.FSMC_WriteTimingStruct = &FSMC_NORSRAMTimingInitStructure;

  FSMC_NORSRAMInit(&FSMC_NORSRAMInitStructure);

  /* Enable FSMC Bank1_SRAM Bank */
  FSMC_NORSRAMCmd(FSMC_Bank1_NORSRAM1, ENABLE);
}

static void lcd_rst(void) {

  GPIO_ResetBits(RST_Port, RST_Pin);
  DWT_Delay(10000);
  GPIO_SetBits(RST_Port, RST_Pin);
  DWT_Delay(100000);
}

static __forceinline void wr_cmd(uint16_t index) {
  LCD_REG8 = index>>8;
  LCD_REG8 = index;
}

static __forceinline void wr_dat(uint16_t val) {
  LCD_DAT8 = val>>8;
  LCD_DAT8 = val;
}

static __forceinline uint16_t rd_dat(void) {
  uint16_t val;
  val = LCD_DAT8;
  val<<=8;
  val |= LCD_DAT8;
  return val;
}

static __forceinline uint16_t rd_reg(uint16_t index) {
  wr_cmd(index);
  return rd_dat();
}

static __forceinline void wr_reg(uint16_t index,uint16_t val) {
  wr_cmd(index);
  wr_dat(val);
}

uint16_t R63400_ReadID(void) {

  return rd_reg(0x0000);
}

uint8_t R63400_Init(uint8_t AddressSetupTime,uint8_t DataSetupTime) {

  GPIO_Configuration();
  FSMC_LCD_Init(AddressSetupTime,DataSetupTime);

  lcd_rst();

  if(R63400_ReadID()==0x3400) {//Renesas R63400

    // initialization sequence was obtained from the Sony Ericsson K800i phone firmware
    wr_reg(0x0001,0x0110);
    wr_reg(0x0002,0x0500);
    wr_reg(0x0008,0x0808);
    wr_reg(0x0009,0x0000);
    wr_reg(0x000B,0x0000);
    wr_reg(0x000C,0x0000);
    wr_reg(0x000D,0x0000);
    wr_reg(0x0010,0x0012);
    wr_reg(0x0011,0x1E06);
    wr_reg(0x0012,0x1E06);
    wr_reg(0x0013,0x0906);
    wr_reg(0x0014,0x0910);
    wr_reg(0x0015,0x091A);
    wr_reg(0x0018,0x0002);
    wr_reg(0x0019,0x0301);
    wr_reg(0x0030,0x0000);
    wr_reg(0x0300,0x0707);
    wr_reg(0x0301,0x0104);
    wr_reg(0x0302,0x0100);
    wr_reg(0x0303,0x0303);
    wr_reg(0x0304,0x0302);
    wr_reg(0x0305,0x0003);
    wr_reg(0x0306,0x0108);
    wr_reg(0x0307,0x0707);
    wr_reg(0x0308,0x0104);
    wr_reg(0x0309,0x0100);
    wr_reg(0x030A,0x0303);
    wr_reg(0x030B,0x0302);
    wr_reg(0x030C,0x0003);
    wr_reg(0x030D,0x171E);
    wr_reg(0x0310,0x0707);
    wr_reg(0x0311,0x0405);
    wr_reg(0x0312,0x0404);
    wr_reg(0x0313,0x0303);
    wr_reg(0x0314,0x0102);
    wr_reg(0x0315,0x0005);
    wr_reg(0x0316,0x0008);
    wr_reg(0x0317,0x0707);
    wr_reg(0x0318,0x0405);
    wr_reg(0x0319,0x0404);
    wr_reg(0x031A,0x0303);
    wr_reg(0x031B,0x0102);
    wr_reg(0x031C,0x0005);
    wr_reg(0x031D,0x171F);
    wr_reg(0x0320,0x0707);
    wr_reg(0x0321,0x0403);
    wr_reg(0x0322,0x0404);
    wr_reg(0x0323,0x0302);
    wr_reg(0x0324,0x0302);
    wr_reg(0x0325,0x0107);
    wr_reg(0x0326,0x0009);
    wr_reg(0x0327,0x0707);
    wr_reg(0x0328,0x0403);
    wr_reg(0x0329,0x0404);
    wr_reg(0x032A,0x0302);
    wr_reg(0x032B,0x0302);
    wr_reg(0x032C,0x0107);
    wr_reg(0x032D,0x161F);
    wr_reg(0x0400,0x0027);
    wr_reg(DISPLAY_CONTROL_1,0x0001);
    wr_reg(0x0110,0x0001);
    wr_reg(0x0100,0x1730);
    wr_reg(0x0101,0x0117);
    wr_reg(0x0102,0x101B);
    wr_reg(0x0103,0x3100);
    wr_reg(0x0105,0x1406);
    wr_reg(0x0102,0x103B);
    DWT_Delay(140000);
    wr_reg(ENTRY_MODE,R63400_color_mode|R63400_orientation_mode); // (bit9 0x0200) high speed off
    wr_reg(0x0004,0x0000);
    wr_reg(GRAM_ADDRESS_HORZ,0x0000);
    wr_reg(GRAM_ADDRESS_VERT,0x0000);
    wr_reg(WINDOW_HORZ_START,0x0000);
    wr_reg(WINDOW_HORZ_END,DISPLAY_HEIGHT-1);
    wr_reg(WINDOW_VERT_START,0x0000);
    wr_reg(WINDOW_VERT_END,DISPLAY_WIDTH-1);
    wr_reg(0x0401,0x0001);
    wr_reg(0x0402,0x0000);
    wr_reg(0x0403,DISPLAY_WIDTH-1);
    wr_reg(0x0404,0x0000);
    DWT_Delay(20000);
    wr_reg(DISPLAY_CONTROL_1,0x0022);
    wr_reg(0x0030,0x0005);
    DWT_Delay(20000);
    wr_reg(DISPLAY_CONTROL_1,0x0123);
    DWT_Delay(16000);

    return R63400_OK;
  }
  else {
    return R63400_ERROR;
  }

}

void R63400_ColorMode(COLOR_MODE color_mode) {

  R63400_color_mode=color_mode;
  wr_reg(ENTRY_MODE,R63400_color_mode|R63400_orientation_mode);
}

void R63400_OrientationMode(ORIENTATION_MODE orientation_mode) {

  R63400_orientation_mode=orientation_mode;
  wr_reg(ENTRY_MODE,R63400_color_mode|R63400_orientation_mode);
}

void R63400_ClearScreen(uint32_t color) {

  R63400_Fill(0,0,R63400_GetWidth()-1,R63400_GetHeight()-1,color);
}

void R63400_Fill(uint16_t x0,uint16_t y0,uint16_t x1,uint16_t y1,uint32_t color) {

  uint32_t i,j=(x1-x0+1)*(y1-y0+1);
  uint8_t r,g,b;

  R63400_SetWindow(x0,y0,x1,y1);

  r=color>>16;
  g=color>>8;
  b=color;

  switch(R63400_color_mode) {
    case COLOR_16BIT:
      for(i=0;i<j;i++) {
        // 2 bytes/px
        wr_dat(((r&0xF8)<<8)|((g&0xFC)<<3)|((b&0xF8)>>3));
      }
      break;
    case COLOR_18BIT:
      for(i=0;i<j;i++) {
        // 3 bytes/px
        LCD_DAT8=r&0xFC;
        LCD_DAT8=g&0xFC;
        LCD_DAT8=b&0xFC;
      }
      break;
  }
}

void R63400_SetWindow(uint16_t x0,uint16_t y0,uint16_t x1,uint16_t y1) {

  switch(R63400_orientation_mode) {
    case ORIENTATION_LANDSCAPE:
      wr_reg(WINDOW_HORZ_START,y0);
      wr_reg(WINDOW_HORZ_END,y1);
      wr_reg(WINDOW_VERT_START,DISPLAY_WIDTH-1-x1);
      wr_reg(WINDOW_VERT_END,DISPLAY_WIDTH-1-x0);
      wr_reg(GRAM_ADDRESS_HORZ,y0);
      wr_reg(GRAM_ADDRESS_VERT,DISPLAY_WIDTH-1-x0);
      break;
    case ORIENTATION_LANDSCAPE_REV:
      wr_reg(WINDOW_HORZ_START,DISPLAY_HEIGHT-1-y1);
      wr_reg(WINDOW_HORZ_END,DISPLAY_HEIGHT-1-y0);
      wr_reg(WINDOW_VERT_START,x0);
      wr_reg(WINDOW_VERT_END,x1);
      wr_reg(GRAM_ADDRESS_HORZ,DISPLAY_HEIGHT-1-y0);
      wr_reg(GRAM_ADDRESS_VERT,x0);
      break;
    case ORIENTATION_PORTRAIT_REV:
      wr_reg(WINDOW_HORZ_START,x0);
      wr_reg(WINDOW_HORZ_END,x1);
      wr_reg(WINDOW_VERT_START,y0);
      wr_reg(WINDOW_VERT_END,y1);
      wr_reg(GRAM_ADDRESS_HORZ,x0);
      wr_reg(GRAM_ADDRESS_VERT,y0);
      break;
    case ORIENTATION_PORTRAIT:
      wr_reg(WINDOW_HORZ_START,DISPLAY_HEIGHT-1-x1);
      wr_reg(WINDOW_HORZ_END,DISPLAY_HEIGHT-1-x0);
      wr_reg(WINDOW_VERT_START,DISPLAY_WIDTH-1-y1);
      wr_reg(WINDOW_VERT_END,DISPLAY_WIDTH-1-y0);
      wr_reg(GRAM_ADDRESS_HORZ,DISPLAY_HEIGHT-1-x0);
      wr_reg(GRAM_ADDRESS_VERT,DISPLAY_WIDTH-1-y0);
      break;
  }
  wr_cmd(MEMORY_READ_WRITE);
}

void R63400_FillPixel(uint16_t x0,uint16_t y0,uint16_t x1,uint16_t y1,uint32_t *color) {

  uint32_t i,j=(x1-x0+1)*(y1-y0+1);

  R63400_SetWindow(x0,y0,x1,y1);

  switch(R63400_color_mode) {
    case COLOR_16BIT:
      for(i=0;i<j;i++) {
        // 2 bytes/px
        wr_dat(((color[i]&0xF80000)>>8)|((color[i]&0xFC00)>>5)|((color[i]&0xF8)>>3));
      }
      break;
    case COLOR_18BIT:
      for(i=0;i<j;i++) {
        // 3 bytes/px
        LCD_DAT8=(color[i]>>16)&0xFC;
        LCD_DAT8=(color[i]>>8)&0xFC;
        LCD_DAT8=(color[i])&0xFC;
      }
      break;
  }
}

void R63400_FillFromBuffer(uint16_t x0,uint16_t y0,uint16_t x1,uint16_t y1,uint8_t *data) {

  uint32_t i,j=(x1-x0+1)*(y1-y0+1);

  R63400_SetWindow(x0,y0,x1,y1);

  switch(R63400_color_mode) {
    case COLOR_16BIT:
      for(i=0;i<j;i++) {
        // 2 bytes/px
        LCD_DAT8=*data++;
        LCD_DAT8=*data++;
      }
      break;
    case COLOR_18BIT:
      for(i=0;i<j;i++) {
        // 3 bytes/px
        LCD_DAT8=*data++;
        LCD_DAT8=*data++;
        LCD_DAT8=*data++;
      }
      break;
  }
}

void R63400_SetPixel(uint16_t x, uint16_t y, uint32_t color) {

  uint8_t r,g,b;

  R63400_SetWindow(x,y,x,y);

  r=color>>16;
  g=color>>8;
  b=color;

  switch(R63400_color_mode) {
    case COLOR_16BIT:
      // 2 bytes/px
      wr_dat(((r&0xF8)<<8)|((g&0xFC)<<3)|((b&0xF8)>>3));
      break;
    case COLOR_18BIT:
      // 3 bytes/px
      LCD_DAT8=r&0xFC;
      LCD_DAT8=g&0xFC;
      LCD_DAT8=b&0xFC;
      break;
  }
}

void R63400_Shutdown(void) {

  // shutdown sequence was obtained from the Sony Ericsson K800i phone firmware
  wr_reg(0x0018,0x0002);
  wr_reg(0x0019,0x0001);
  DWT_Delay(96000);
  wr_reg(DISPLAY_CONTROL_1,0x0022);
  DWT_Delay(60000);
  wr_reg(DISPLAY_CONTROL_1,0x0000);
  wr_reg(0x0030,0x0000);
  wr_reg(0x0100,0x0630);
  wr_reg(0x0103,0x0F00);
  DWT_Delay(1000);
  wr_reg(0x0100,0x0000);
  wr_reg(0x0102,0x1008);
  DWT_Delay(30000);
}

uint16_t R63400_GetWidth(void) {

  uint16_t ret;

  switch(R63400_orientation_mode){
    case ORIENTATION_LANDSCAPE:
    case ORIENTATION_LANDSCAPE_REV:
      ret=DISPLAY_WIDTH;
    break;
    case ORIENTATION_PORTRAIT:
    case ORIENTATION_PORTRAIT_REV:
      ret=DISPLAY_HEIGHT;
    break;
  }
  return ret;
}

uint16_t R63400_GetHeight(void) {

  uint16_t ret;

  switch(R63400_orientation_mode){
    case ORIENTATION_LANDSCAPE:
    case ORIENTATION_LANDSCAPE_REV:
      ret=DISPLAY_HEIGHT;
    break;
    case ORIENTATION_PORTRAIT:
    case ORIENTATION_PORTRAIT_REV:
      ret=DISPLAY_WIDTH;
    break;
  }
  return ret;
}

void R63400_SetFont(FONT_SIZE font_size) {

  R63400_font_size=font_size;
}

void R63400_SetTextColors(uint32_t fColor, uint32_t bColor) {

  R63400_text_foreground_color = fColor;
  R63400_text_background_color = bColor;
}

void R63400_PutChar(char c, uint16_t x, uint16_t y) {

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

  // get pointer to the beginning of the selected font table
  pFont = (unsigned char *)FontTable[R63400_font_size];

  // get the nColumns, nRows and nBytes
  nCols = *pFont;
  nRows = *(pFont + 1);
  nBytes = *(pFont + 2);

  // get pointer to the first byte of the desired character
  pChar = pFont + (nBytes * (c - 0x1F));

  R63400_SetWindow(x,y,x + nCols - 1,y + nRows - 1);

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
        Word0 = R63400_text_foreground_color;
      else
        Word0 = R63400_text_background_color;
      Mask >>= 1;

      if (PixelRow & Mask)
        Word1 = R63400_text_foreground_color;
      else
        Word1 = R63400_text_background_color;
      Mask >>= 1;

      switch(R63400_color_mode){
        case COLOR_16BIT:
          // 4 bytes 2 pixels
          wr_dat(((Word0&0xF80000)>>8)|((Word0&0xFC00)>>5)|((Word0&0xF8)>>3));
          wr_dat(((Word1&0xF80000)>>8)|((Word1&0xFC00)>>5)|((Word1&0xF8)>>3));
          break;
        case COLOR_18BIT:
          // 6 bytes 2 pixels
          wr_dat(((Word0&0xFC0000)>>8)|((Word0&0xFC00)>>8));
          wr_dat(((Word0&0xFC)<<8)|((Word1&0xFC0000)>>16));
          wr_dat(((Word1&0xFC00))|((Word1&0xFC)));
          break;
      }
    }
  }
}

void R63400_PutStr(char *pString, uint16_t x, uint16_t y) {

  if(y+FontTable[R63400_font_size][1]>R63400_GetHeight())
    return;

  // loop until null-terminator is seen
  while (*pString) {
    if (x+FontTable[R63400_font_size][0]>R63400_GetWidth()) break;
    // draw the character
    R63400_PutChar(*pString++, x, y);
    x+=FontTable[R63400_font_size][0];
  }
}

void R63400_PutStrCEOL(char *pString, uint16_t x, uint16_t y) {

  if(y+FontTable[R63400_font_size][1]>R63400_GetHeight())
    return;

  // loop until null-terminator is seen
  while (*pString) {
    if (x+FontTable[R63400_font_size][0]>R63400_GetWidth()) break;
    // draw the character
    R63400_PutChar(*pString++, x, y);
    x+=FontTable[R63400_font_size][0];
  }
  while(x+FontTable[R63400_font_size][0]<=R63400_GetWidth()) {
    R63400_PutChar(' ', x, y);
    x+=FontTable[R63400_font_size][0];
  }
}

void R63400_PutStrCentered(char *pString, uint16_t y) {

  uint32_t length=strlen(pString)*FontTable[R63400_font_size][0];

  R63400_PutStr(pString,length>R63400_GetWidth()?0:(R63400_GetWidth()-length)/2,y);
}

void R63400_ReadMemory(uint16_t x0,uint16_t y0,uint16_t x1,uint16_t y1,uint16_t *data) {

  uint8_t i,j=(x1-x0+1)*(y1-y0+1);

  R63400_SetWindow(x0,y0,x1,y1);
  // first read is dummy read
  *data=rd_dat();

  // always read 2 bytes/px
  for(i=0;i<j;i++) {
    *data++=rd_dat();
  }
}
