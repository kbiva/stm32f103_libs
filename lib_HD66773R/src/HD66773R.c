/*
 *  HD66773R.c
 *
 *  Author: Kestutis Bivainis
 *
 */

#include "stm32f10x_conf.h"
#include "HD66773R.h"
#include "colors.h"
#include "delay.h"
#include "font6x8.h"
#include "font8x8.h"
#include "font8x16.h"

static FONT_SIZE HD66773R_font_size;
static uint32_t HD66773R_text_foreground_color=WHITE;
static uint32_t HD66773R_text_background_color=BLACK;

static ORIENTATION_MODE HD66773R_orientation_mode;

static unsigned char *FontTable[] = {
    (unsigned char *)FONT6x8,
    (unsigned char *)FONT8x8,
    (unsigned char *)FONT8x16
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
  {{DB8_Pin,DB8_Speed,DB8_Mode},DB8_Port,DB8_Bus},
  {{DB9_Pin,DB9_Speed,DB9_Mode},DB9_Port,DB9_Bus},
  {{DB10_Pin,DB10_Speed,DB10_Mode},DB10_Port,DB10_Bus},
  {{DB11_Pin,DB11_Speed,DB11_Mode},DB11_Port,DB11_Bus},
  {{DB12_Pin,DB12_Speed,DB12_Mode},DB12_Port,DB12_Bus},
  {{DB13_Pin,DB13_Speed,DB13_Mode},DB13_Port,DB13_Bus},
  {{DB14_Pin,DB14_Speed,DB14_Mode},DB14_Port,DB14_Bus},
  {{DB15_Pin,DB15_Speed,DB15_Mode},DB15_Port,DB15_Bus},
  {{RW_Pin, RW_Speed, RW_Mode}, RW_Port, RW_Bus},
  {{RD_Pin, RD_Speed, RD_Mode}, RD_Port, RD_Bus},
  {{RS_Pin, RS_Speed, RS_Mode}, RS_Port, RS_Bus},
  {{CS_Pin, CS_Speed, CS_Mode}, CS_Port, CS_Bus},
  {{RST_Pin,RST_Speed,RST_Mode},RST_Port,RST_Bus},
};

static void FSMC_LCD_Init(void) {

  FSMC_NORSRAMInitTypeDef FSMC_NORSRAMInitStructure;
  FSMC_NORSRAMTimingInitTypeDef  FSMC_NORSRAMTimingInitStructure;

  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_FSMC, ENABLE);

  FSMC_NORSRAMTimingInitStructure.FSMC_AddressSetupTime = 0x00;
  FSMC_NORSRAMTimingInitStructure.FSMC_AddressHoldTime = 0x00;
  FSMC_NORSRAMTimingInitStructure.FSMC_DataSetupTime = 0x02;
  FSMC_NORSRAMTimingInitStructure.FSMC_BusTurnAroundDuration = 0x00;
  FSMC_NORSRAMTimingInitStructure.FSMC_CLKDivision = 0x00;
  FSMC_NORSRAMTimingInitStructure.FSMC_DataLatency = 0x00;
  FSMC_NORSRAMTimingInitStructure.FSMC_AccessMode = FSMC_AccessMode_B;

  FSMC_NORSRAMInitStructure.FSMC_Bank = FSMC_Bank1_NORSRAM1;
  FSMC_NORSRAMInitStructure.FSMC_DataAddressMux = FSMC_DataAddressMux_Disable;
  FSMC_NORSRAMInitStructure.FSMC_MemoryType = FSMC_MemoryType_NOR;
  FSMC_NORSRAMInitStructure.FSMC_MemoryDataWidth = FSMC_MemoryDataWidth_16b;
  FSMC_NORSRAMInitStructure.FSMC_BurstAccessMode = FSMC_BurstAccessMode_Disable;
  FSMC_NORSRAMInitStructure.FSMC_WaitSignalPolarity = FSMC_WaitSignalPolarity_Low;
  FSMC_NORSRAMInitStructure.FSMC_WrapMode = FSMC_WrapMode_Disable;
  FSMC_NORSRAMInitStructure.FSMC_WaitSignalActive = FSMC_WaitSignalActive_BeforeWaitState;
  FSMC_NORSRAMInitStructure.FSMC_WriteOperation = FSMC_WriteOperation_Enable;
  FSMC_NORSRAMInitStructure.FSMC_WaitSignal = FSMC_WaitSignal_Disable;
  FSMC_NORSRAMInitStructure.FSMC_ExtendedMode = FSMC_ExtendedMode_Disable;
  FSMC_NORSRAMInitStructure.FSMC_WriteBurst = FSMC_WriteBurst_Disable;
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
  DWT_Delay(10000);
  GPIO_SetBits(RST_Port, RST_Pin);
  DWT_Delay(10000);
}

static __forceinline
void wr_cmd(uint16_t index) {
  LCD_REG16 = index;
}

static __forceinline
void wr_reg(uint16_t index,uint16_t val) {
  LCD_REG16 = index;
  LCD_DAT16 = val;
}

static __forceinline
void wr_dat(uint16_t val) {
  LCD_DAT16 = val;
}

static __forceinline
uint16_t rd_reg(uint16_t index) {
  LCD_REG16 = index;
  return (LCD_DAT16);
}

static __forceinline
uint16_t rd_dat(void) {
  return (LCD_DAT16);
}

static void Initializing_Function1(void) {

  wr_reg(DRIVEROUTPUTCTRL,0x0113);//(396x160)
  wr_reg(DRIVINGWAVECTRL,0x0700);
  wr_reg(ENTRYMODE,0x1000|ORIENTATION_PORTRAIT);
  wr_reg(COMPAREREG,0x0000);
  wr_reg(DISPLAYCTRL,0x0104);
  wr_reg(FRAMECYCLECTRL,0x0000);
}

static void Power_Setting_Function(void) {

  wr_reg(POWERCTRL3,0x0000);
  wr_reg(POWERCTRL4,0x0401);
  wr_reg(POWERCTRL5,0x0D18);
  DWT_Delay(40000);
  wr_reg(POWERCTRL1,0x0214);
  wr_reg(POWERCTRL2,0x8000);
  DWT_Delay(40000);
  wr_reg(POWERCTRL5,0x2910);
  DWT_Delay(40000);
  wr_reg(POWERCTRL4,0x0512);
}

static void Initializing_Function2(void) {

  //wr_reg(GAMMACTRL1,0x0100);
  //wr_reg(GAMMACTRL2,0x0707);
  //wr_reg(GAMMACTRL3,0x0102);
  //wr_reg(GAMMACTRL4,0x0502);
  //wr_reg(GAMMACTRL5,0x0506);
  //wr_reg(GAMMACTRL6,0x0000);
  //wr_reg(GAMMACTRL7,0x0706);
  //wr_reg(GAMMACTRL8,0x0205);
  //wr_reg(GAMMACTRL9,0x0000);
  //wr_reg(GAMMACTRL10,0x000F);

  wr_reg(GATESCANPOS,0x0000);
  wr_reg(VERTICALSCROLLCTRL,0x0000);
  wr_reg(SCREENDRIVEPOS1,0x9F00);
  wr_reg(SCREENDRIVEPOS2,0x9F9F);
  wr_reg(HORIZONTALADDRESS,0x7F00);//127
  wr_reg(VERTICALADDRESS,0x9F00);//159
}

static void Display_On_Function(void) {

  wr_reg(DISPLAYCTRL,0x0105);
  DWT_Delay(40000);
  wr_reg(DISPLAYCTRL,0x0125);
  wr_reg(DISPLAYCTRL,0x0127);
  DWT_Delay(40000);
  wr_reg(DISPLAYCTRL,0x0137);
}

static void Display_Off_Function(void) {

  wr_reg(DISPLAYCTRL,0x0136);
  DWT_Delay(40000);
  wr_reg(DISPLAYCTRL,0x0126);
  DWT_Delay(40000);
  wr_reg(DISPLAYCTRL,0x0104);
}

uint8_t HD66773R_Init(void) {

  volatile uint32_t id=0;

  GPIO_Configuration();

  FSMC_LCD_Init();

  lcd_rst();

  wr_reg(STARTOSCILLATION,0x0001);
  DWT_Delay(10000);

  id=rd_reg(0x00); // Device code read

  if(id==0x0773) {

    Initializing_Function1();

    Power_Setting_Function();

    Initializing_Function2();

    DWT_Delay(10000);

    Display_On_Function();

    HD66773R_orientation_mode=ORIENTATION_PORTRAIT;

    return HD66773R_OK;
  }
  else {
    return HD66773R_ERROR;
  }
}

void HD66773R_OrientationMode(ORIENTATION_MODE orientation_mode) {

  wr_reg(ENTRYMODE,0x1000|orientation_mode);
  HD66773R_orientation_mode=orientation_mode;
}

void HD66773R_ClearScreen(uint32_t color) {

  HD66773R_Fill(0,0,HD66773R_GetWidth()-1,HD66773R_GetHeight()-1,color);
}

void HD66773R_Fill(uint8_t x0,uint8_t y0,uint8_t x1,uint8_t y1,uint32_t color) {

  uint32_t i,j=(x1-x0+1)*(y1-y0+1);
  uint16_t c;

  HD66773R_SetWindow(x0,y0,x1,y1);

  c=(color & 0xf80000) >> 8 | (color & 0xfc00) >> 5 | (color & 0xf8) >> 3;
  for(i=0;i<j;i++) {
    wr_dat(c);
  }
}

void HD66773R_SetWindow(uint8_t x0,uint8_t y0,uint8_t x1,uint8_t y1) {

  switch(HD66773R_orientation_mode){
    case ORIENTATION_LANDSCAPE:
      wr_reg(HORIZONTALADDRESS,(y1<<8)|y0);
      wr_reg(VERTICALADDRESS,((159-x0)<<8)|(159-x1));
      wr_reg(ADDRESSSET,((159-x0)<<8)|y0);
    break;
    case ORIENTATION_LANDSCAPE_REV:
      wr_reg(HORIZONTALADDRESS,((127-y0)<<8)|(127-y1));
      wr_reg(VERTICALADDRESS,(x1<<8)|x0);
      wr_reg(ADDRESSSET,(x0<<8)|(127-y0));
    break;
    case ORIENTATION_PORTRAIT:
       wr_reg(HORIZONTALADDRESS,(x1<<8)|x0);
      wr_reg(VERTICALADDRESS,(y1<<8)|y0);
      wr_reg(ADDRESSSET,(y0<<8)|x0);
    break;
    case ORIENTATION_PORTRAIT_REV:
      wr_reg(HORIZONTALADDRESS,((127-x0)<<8)|(127-x1));
      wr_reg(VERTICALADDRESS,((159-y0)<<8)|(159-y1));
      wr_reg(ADDRESSSET,((159-y0)<<8)|(127-x0));
    break;
  }

  wr_cmd(GRAMSTARTWRITING);
}

void HD66773R_FillPixel(uint8_t x0,uint8_t y0,uint8_t x1,uint8_t y1,uint32_t *color) {

  uint32_t i,j=(x1-x0+1)*(y1-y0+1);
  uint16_t c;

  HD66773R_SetWindow(x0,y0,x1,y1);

  for(i=0;i<j;i++) {
    c=(color[i] & 0xf80000) >> 8 | (color[i] & 0xfc00) >> 5 | (color[i] & 0xf8) >> 3;
    wr_dat(c);
  }
}

void HD66773R_SetPixel(uint8_t x,uint8_t y,uint32_t color) {

  uint16_t c;

  HD66773R_SetWindow(x,y,x,y);

  c=(color & 0xf80000) >> 8 | (color & 0xfc00) >> 5 | (color & 0xf8) >> 3;
  wr_dat(c);
}

uint8_t HD66773R_GetWidth(void) {

  uint8_t ret;

  switch(HD66773R_orientation_mode){
    case ORIENTATION_LANDSCAPE:
    case ORIENTATION_LANDSCAPE_REV:
      ret=160;
    break;
    case ORIENTATION_PORTRAIT:
    case ORIENTATION_PORTRAIT_REV:
      ret=128;
    break;
  }
  return ret;
}

uint8_t HD66773R_GetHeight(void) {

  uint8_t ret;

  switch(HD66773R_orientation_mode){
    case ORIENTATION_LANDSCAPE:
    case ORIENTATION_LANDSCAPE_REV:
      ret=128;
    break;
    case ORIENTATION_PORTRAIT:
    case ORIENTATION_PORTRAIT_REV:
      ret=160;
    break;
  }
  return ret;
}

void HD66773R_Sleep(void) {

  Display_Off_Function();
  wr_reg(POWERCTRL1,0x0080);
  wr_reg(POWERCTRL1,0x0216);
}

void HD66773R_StandBy(void) {

  Display_Off_Function();
  wr_reg(POWERCTRL1,0x0080);
  wr_reg(POWERCTRL1,0x0215);
}

void HD66773R_WakeupFromSleep(void) {

  wr_reg(POWERCTRL1,0x0214);
  Power_Setting_Function();
  Display_On_Function();
}

void HD66773R_WakeupFromStandBy(void) {

  wr_reg(STARTOSCILLATION,0x0001);
  DWT_Delay(10000);
  wr_reg(POWERCTRL1,0x0214);
  Initializing_Function1();
  Power_Setting_Function();
  Initializing_Function2();
  Display_On_Function();
}

void HD66773R_SetFont(FONT_SIZE font_size) {

  HD66773R_font_size=font_size;
}

void HD66773R_SetTextColours(uint32_t fColor, uint32_t bColor) {

  HD66773R_text_foreground_color = fColor;
  HD66773R_text_background_color = bColor;
}

void HD66773R_PutChar(char c, uint8_t x, uint8_t y) {

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
  pFont = (unsigned char *)FontTable[HD66773R_font_size];

  // get the nColumns, nRows and nBytes
  nCols = *pFont;
  nRows = *(pFont + 1);
  nBytes = *(pFont + 2);

  // get pointer to the first byte of the desired character
  pChar = pFont + (nBytes * (c - 0x1F));

  HD66773R_SetWindow(x,y,x + nCols - 1,y + nRows - 1);

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
        Word0 = HD66773R_text_foreground_color;
      else
        Word0 = HD66773R_text_background_color;

      Mask >>= 1;

      if (PixelRow & Mask)
        Word1 = HD66773R_text_foreground_color;
      else
        Word1 = HD66773R_text_background_color;

      Mask >>= 1;

      wr_dat((Word0 & 0xf80000) >> 8 | (Word0 & 0xfc00) >> 5 | (Word0 & 0xf8) >> 3);
      wr_dat((Word1 & 0xf80000) >> 8 | (Word1 & 0xfc00) >> 5 | (Word1 & 0xf8) >> 3);
    }
  }
}

void HD66773R_PutStr(char *pString, uint8_t x, uint8_t y) {

  // loop until null-terminator is seen
  while (*pString) {
    // draw the character
    HD66773R_PutChar(*pString++, x, y);

    switch(HD66773R_font_size) {
      case FONT_6x8:
        x+=6;
      break;
      case FONT_8x8:
        x+=8;
      break;
      case FONT_8x16:
        x+=8;
      break;
    }
    if (x > HD66773R_GetWidth()-1) break;
  }
}
