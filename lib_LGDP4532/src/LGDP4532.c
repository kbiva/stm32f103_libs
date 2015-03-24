/*
 *  LGDP4532.c
 *
 *  Author: Kestutis Bivainis
 *
 */

#include "stm32f10x_conf.h"
#include "LGDP4532.h"
#include "delay.h"
#include "colors.h"
#include "font6x8.h"
#include "font8x8.h"
#include "font8x14.h"
#include <string.h>

static uint32_t LGDP4532_text_foreground_color=WHITE;
static uint32_t LGDP4532_text_background_color=BLACK;
static FONT_SIZE LGDP4532_font_size;

static unsigned char *FontTable[] = {
    (unsigned char *)FONT6x8,
    (unsigned char *)FONT8x8,
    (unsigned char *)FONT8x14
};

static COLOR_MODE LGDP4532_color_mode;
static ORIENTATION_MODE LGDP4532_orientation_mode;

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


static void GPIO_Configuration(void){

  uint32_t i;

  for(i=0;i<sizeof(pins)/sizeof(PIN);i++) {
    RCC_APB2PeriphClockCmd(pins[i].GPIO_Bus,ENABLE);
    GPIO_Init(pins[i].GPIOx,&pins[i].GPIO_InitStructure);
  }
}

static void FSMC_LCD_Init(void) {

  FSMC_NORSRAMInitTypeDef FSMC_NORSRAMInitStructure;
  FSMC_NORSRAMTimingInitTypeDef  FSMC_NORSRAMTimingInitStructure;

  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_FSMC, ENABLE);

  FSMC_NORSRAMTimingInitStructure.FSMC_AddressSetupTime = 0x00;
  FSMC_NORSRAMTimingInitStructure.FSMC_AddressHoldTime = 0x00;
  FSMC_NORSRAMTimingInitStructure.FSMC_DataSetupTime = 0x01;
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

static void lcd_rst(void) {

  GPIO_ResetBits(RST_Port, RST_Pin);
  DWT_Delay(100000);
  GPIO_SetBits(RST_Port, RST_Pin);
  DWT_Delay(100000);
}

static __forceinline void wr_cmd(uint16_t index) {
  LCD_REG16 = index;
}

static __forceinline void wr_reg(uint16_t index,uint16_t val) {
  LCD_REG16 = index;
  LCD_DAT16 = val;
}

static __forceinline void wr_dat(uint16_t val) {
  LCD_DAT16 = val;
}

static __forceinline uint16_t rd_reg(uint16_t index) {
  LCD_REG16 = index;
  return (LCD_DAT16);
}

static __forceinline uint16_t rd_dat(void) {
  return (LCD_DAT16);
}

void LGDP4532_Init(void) {

  //volatile uint32_t id=0;

  GPIO_Configuration();
  FSMC_LCD_Init();

  lcd_rst();

  LGDP4532_color_mode=COLOR_16BIT;
  LGDP4532_orientation_mode=ORIENTATION_LANDSCAPE;

  //id=rd_reg(0x00); // Device code read id = 4532

// power on
  wr_reg(STARTOSCILLATION,0x0001); // Start oscillation
  DWT_Delay(17);
  wr_reg(REGULATORCONTROL,0x0030); // Regulator Control
  wr_reg(POWERCTRL2,0x0040); // Power Control 2 //set dc1,dc0,vc2:0:0040
  wr_reg(POWERCTRL1,0x1628); // Power Control 1 //set bt,sap,ap:1628
  wr_reg(POWERCTRL3,0x0000); // Power Control 3 //set vrh
  wr_reg(POWERCTRL4,0x104d); // Power Control 4 //set vdv,vcm
  DWT_Delay(17);
  wr_reg(POWERCTRL3,0x0010); // Power Control 3 //set vrh:0010
  DWT_Delay(17);
  wr_reg(POWERCTRL1,0x2620); // Power Control 1 //set bt,sap,ap:2620
  wr_reg(POWERCTRL4,0x344d); // Power Control 4 //set vdv,vcm
  DWT_Delay(17);
// end power on

  wr_reg(DRIVEROUTPUTCONTROL,0x0100); // Driver output control //set sm,ss
  wr_reg(DRIVINGWAVECONTROL,0x0300); // LCD Driving Wave Control ///set line/frame inversion ,BC0,EOR,NW5-0

  wr_reg(ENTRYMODE,LGDP4532_color_mode|LGDP4532_orientation_mode);

  wr_reg(DISPLAYCTRL2,0x0604); // Display Control 2 //set fp,bp 0604
  wr_reg(DISPLAYCTRL3,0x0000); // Display Control 3 // PTG normal scan
  wr_reg(DISPLAYCTRL4,0x0008); // Display Control 4 // FMARK on, interval 1 frame

  wr_reg(EPROMCONTROLREGISTER2,0x0002); // EPROM Control Register 2
  wr_reg(GATESCANCONTROL1,0x2700); // Driver Output Control // set GS bit, lines 320
  wr_reg(GATESCANCONTROL2,0x0001); // Base Image Display Control
  wr_reg(PANELINTERFACECONTROL1,0x0182); // Panel Interface Control 1 //set DIV1-0,RTN4-0 ,0199
  wr_reg(PANELINTERFACECONTROL3,0x0001); // Panel Interface Control 3
  wr_reg(TESTREGISTER4,0x0010); // Test Register 4
  DWT_Delay(17);
  //Delay(10);

//set gamma
  wr_reg(GAMMACONTROL_RED1,0x0000); // Red Gamma Control 1-16
  wr_reg(GAMMACONTROL_RED2,0x0502); // Red Gamma Control 1-16
  wr_reg(GAMMACONTROL_RED3,0x0307); // Red Gamma Control 1-16
  wr_reg(GAMMACONTROL_RED4,0x0305); // Red Gamma Control 1-16
  wr_reg(GAMMACONTROL_RED5,0x0004); // Red Gamma Control 1-16
  wr_reg(GAMMACONTROL_RED6,0x0402); // Red Gamma Control 1-16
  wr_reg(GAMMACONTROL_RED7,0x0707); // Red Gamma Control 1-16
  wr_reg(GAMMACONTROL_RED8,0x0503); // Red Gamma Control 1-16
  wr_reg(GAMMACONTROL_RED9,0x1505); // Red Gamma Control 1-16
  wr_reg(GAMMACONTROL_RED10,0x1505); // Red Gamma Control 1-16
  DWT_Delay(17);
//end gamma set

//Display on
  wr_reg(DISPLAYCTRL1,0x0001); // Display Control 1
  DWT_Delay(17);
  wr_reg(DISPLAYCTRL1,0x0021); // Display Control 1
  wr_reg(DISPLAYCTRL1,0x0023); // Display Control 1
  DWT_Delay(17);
  wr_reg(DISPLAYCTRL1,0x0033); // Display Control 1
  DWT_Delay(17);
  wr_reg(DISPLAYCTRL1,0x0133); // Display Control 1
//end display on

}

void LGDP4532_ColorMode(COLOR_MODE color_mode) {

  wr_reg(ENTRYMODE,color_mode|LGDP4532_orientation_mode);
  LGDP4532_color_mode=color_mode;
}

void LGDP4532_OrientationMode(ORIENTATION_MODE orientation_mode) {

  wr_reg(ENTRYMODE,LGDP4532_color_mode|orientation_mode);
  LGDP4532_orientation_mode=orientation_mode;
}

/*
void LGDP4532_Gamma(uint16_t g1_,uint16_t g2_,uint16_t g3_,uint16_t g4_,uint16_t g5_,
                    uint16_t g6_,uint16_t g7_,uint16_t g8_,uint16_t g9_,uint16_t g10_) {
  wr_reg(GAMMACONTROL1,g1_); // Red Gamma Control 1-16
  wr_reg(GAMMACONTROL2,g2_); // Red Gamma Control 1-16
  wr_reg(GAMMACONTROL3,g3_); // Red Gamma Control 1-16
  wr_reg(GAMMACONTROL4,g4_); // Red Gamma Control 1-16
  wr_reg(GAMMACONTROL5,g5_); // Red Gamma Control 1-16
  wr_reg(GAMMACONTROL6,g6_); // Red Gamma Control 1-16
  wr_reg(GAMMACONTROL7,g7_); // Red Gamma Control 1-16
  wr_reg(GAMMACONTROL8,g8_); // Red Gamma Control 1-16
  wr_reg(GAMMACONTROL9,g9_); // Red Gamma Control 1-16
  wr_reg(GAMMACONTROL10,g10_); // Red Gamma Control 1-16
}
*/

void LGDP4532_SetPixel(uint16_t x,uint16_t y,uint32_t color) {

  uint16_t dest1,dest2;

  switch(LGDP4532_orientation_mode){
    case ORIENTATION_LANDSCAPE:
      wr_reg(HORIZONTALADDRESS,y);
      wr_reg(VERTICALADDRESS,319-x);
    break;
    case ORIENTATION_LANDSCAPE_REV:
      wr_reg(HORIZONTALADDRESS,239-y);
      wr_reg(VERTICALADDRESS,x);
      break;
    case ORIENTATION_PORTRAIT:
      wr_reg(HORIZONTALADDRESS,x);
      wr_reg(VERTICALADDRESS,y);
    break;
    case ORIENTATION_PORTRAIT_REV:
      wr_reg(HORIZONTALADDRESS,239-x);
      wr_reg(VERTICALADDRESS,319-y);
    break;
  }
  wr_cmd(GRAMSTARTWRITING);
  switch(LGDP4532_color_mode){
    case COLOR_16BIT:
      dest1=(color & 0xf80000) >> 8 | (color & 0xfc00) >> 5 | (color & 0xf8) >> 3;
      wr_dat(dest1);
      break;
    case COLOR_18BIT:
      dest1=color>>22;
      dest2=(color & 0xfc0000) >> 6 | (color & 0xfc00) >> 4 | (color & 0xfc) >> 2;
      wr_dat(dest1);
      wr_dat(dest2);
      break;
  }
}

void LGDP4532_ClearScreen(uint32_t color) {

  LGDP4532_Fill(0,0,LGDP4532_GetWidth()-1,LGDP4532_GetHeight()-1,color);
}

void LGDP4532_Fill(uint16_t x0,uint16_t y0,uint16_t x1,uint16_t y1,uint32_t color) {

  uint32_t i,j=(x1-x0+1)*(y1-y0+1);
  uint16_t dest1,dest2;

  LGDP4532_SetWindow(x0,y0,x1,y1);

  switch(LGDP4532_color_mode){
    case COLOR_16BIT:
      for(i=0;i<j;i++) {
        dest1=(color & 0xf80000) >> 8 | (color & 0xfc00) >> 5 | (color & 0xf8) >> 3;
        wr_dat(dest1);
      }
      break;
    case COLOR_18BIT:
      for(i=0;i<j;i++) {
        dest1=color>>22;
        dest2=(color & 0xfc0000) >> 6 | (color & 0xfc00) >> 4 | (color & 0xfc) >> 2;
        wr_dat(dest1);
        wr_dat(dest2);
      }
      break;
  }
}

void LGDP4532_FillPixel(uint16_t x0,uint16_t y0,uint16_t x1,uint16_t y1,uint32_t *color) {

  uint32_t i,j;
  uint16_t dest1,dest2;

  LGDP4532_SetWindow(x0,y0,x1,y1);

  j=(x1-x0+1)*(y1-y0+1);

  switch(LGDP4532_color_mode){
    case COLOR_16BIT:
      for(i=0;i<j;i++) {
        dest1=(color[i] & 0xf80000) >> 8 | (color[i] & 0xfc00) >> 5 | (color[i] & 0xf8) >> 3;
        wr_dat(dest1);
      }
      break;
    case COLOR_18BIT:
      for(i=0;i<j;i++) {
        dest1=color[i]>>22;
        dest2=(color[i] & 0xfc0000) >> 6 | (color[i] & 0xfc00) >> 4 | (color[i] & 0xfc) >> 2;
        wr_dat(dest1);
        wr_dat(dest2);
      }
      break;
  }
}

void LGDP4532_FillFromBuffer(uint16_t x0,uint16_t y0,uint16_t x1,uint16_t y1,uint8_t *data) {

  uint32_t i,j=(x1-x0+1)*(y1-y0+1);
  uint16_t p;

  LGDP4532_SetWindow(x0,y0,x1,y1);

  switch(LGDP4532_color_mode) {
    case COLOR_16BIT:
      for(i=0;i<j;i++) {
        p=(*data++)<<8;
        p|=*data++;
        wr_dat(p);
      }
      break;
    case COLOR_18BIT:
      for(i=0;i<j;i++) {
        p=(*data)>>6;
        wr_dat(p);
        p=(*data++)<<10;
        p|=(*data++)<<4;
        p|=(*data++)>>2;
        wr_dat(p);
      }
      break;
  }
}

void LGDP4532_FillPixel_16bit(uint16_t x0,uint16_t y0,uint16_t x1,uint16_t y1,uint16_t *color) {

  uint32_t i,j;

  LGDP4532_SetWindow(x0,y0,x1,y1);

  j=(x1-x0+1)*(y1-y0+1);
  for(i=0;i<j;i++) {
    wr_dat(color[i]);
  }
}

void LGDP4532_SetScrollPosition(uint16_t pos) {
  wr_reg(GATESCANCONTROLSCROLL,pos);
}

void LGDP4532_SetWindow(uint16_t x0,uint16_t y0,uint16_t x1,uint16_t y1) {

  switch(LGDP4532_orientation_mode){
    case ORIENTATION_LANDSCAPE:
      wr_reg(HORIZONTALRAMPOSITIONSTART,y0);
      wr_reg(HORIZONTALRAMPOSITIONEND,y1);
      wr_reg(VERTICALRAMPOSITIONSTART,319-x1);
      wr_reg(VERTICALRAMPOSITIONEND,319-x0);
      wr_reg(HORIZONTALADDRESS,y0);
      wr_reg(VERTICALADDRESS,319-x0);
    break;
    case ORIENTATION_LANDSCAPE_REV:
      wr_reg(HORIZONTALRAMPOSITIONSTART,239-y1);
      wr_reg(HORIZONTALRAMPOSITIONEND,239-y0);
      wr_reg(VERTICALRAMPOSITIONSTART,x0);
      wr_reg(VERTICALRAMPOSITIONEND,x1);
      wr_reg(HORIZONTALADDRESS,239-y0);
      wr_reg(VERTICALADDRESS,x0);
    break;
    case ORIENTATION_PORTRAIT:
      wr_reg(HORIZONTALRAMPOSITIONSTART,x0);
      wr_reg(HORIZONTALRAMPOSITIONEND,x1);
      wr_reg(VERTICALRAMPOSITIONSTART,y0);
      wr_reg(VERTICALRAMPOSITIONEND,y1);
      wr_reg(HORIZONTALADDRESS,x0);
      wr_reg(VERTICALADDRESS,y0);
    break;
    case ORIENTATION_PORTRAIT_REV:
      wr_reg(HORIZONTALRAMPOSITIONSTART,239-x1);
      wr_reg(HORIZONTALRAMPOSITIONEND,239-x0);
      wr_reg(VERTICALRAMPOSITIONSTART,319-y1);
      wr_reg(VERTICALRAMPOSITIONEND,319-y0);
      wr_reg(HORIZONTALADDRESS,239-x0);
      wr_reg(VERTICALADDRESS,319-y0);
    break;
  }

  wr_cmd(GRAMSTARTWRITING);
}

void LGDP4532_Sleep(void) {

  wr_reg(DISPLAYCTRL1,0x0000);
  wr_reg(POWERCTRL1,0x0002);
}

void LGDP4532_Wakeup(void){

  wr_reg(POWERCTRL1,0x1670);
  wr_reg(DISPLAYCTRL1,0x0133);
}

void LGDP4532_Circle(uint16_t x0, uint16_t y0, uint16_t radius, uint32_t color) {

  int f = 1 - radius;
  int ddF_x = 0;
  int ddF_y = -2 * radius;
  int x = 0;
  int y = radius;
  LGDP4532_SetPixel(x0, y0 + radius, color);
  LGDP4532_SetPixel(x0, y0 - radius, color);
  LGDP4532_SetPixel(x0 + radius, y0, color);
  LGDP4532_SetPixel(x0 - radius, y0, color);
  while (x < y)
  {
    if (f >= 0)
    {
      y--;
      ddF_y += 2;
      f += ddF_y;
    }
    x++;
    ddF_x += 2;
    f += ddF_x + 1;
    LGDP4532_SetPixel(x0 + x, y0 + y, color);
    LGDP4532_SetPixel(x0 - x, y0 + y, color);
    LGDP4532_SetPixel(x0 + x, y0 - y, color);
    LGDP4532_SetPixel(x0 - x, y0 - y, color);
    LGDP4532_SetPixel(x0 + y, y0 + x, color);
    LGDP4532_SetPixel(x0 - y, y0 + x, color);
    LGDP4532_SetPixel(x0 + y, y0 - x, color);
    LGDP4532_SetPixel(x0 - y, y0 - x, color);
  }
}

uint16_t LGDP4532_GetWidth(void) {

  uint16_t ret;

  switch(LGDP4532_orientation_mode){
    case ORIENTATION_LANDSCAPE:
    case ORIENTATION_LANDSCAPE_REV:
      ret=320;
    break;
    case ORIENTATION_PORTRAIT:
    case ORIENTATION_PORTRAIT_REV:
      ret=240;
    break;
  }
  return ret;
}

uint16_t LGDP4532_GetHeight(void) {

  uint16_t ret;

  switch(LGDP4532_orientation_mode){
    case ORIENTATION_LANDSCAPE:
    case ORIENTATION_LANDSCAPE_REV:
      ret=240;
    break;
    case ORIENTATION_PORTRAIT:
    case ORIENTATION_PORTRAIT_REV:
      ret=320;
    break;
  }
  return ret;
}

void LGDP4532_SetFont(FONT_SIZE font_size) {

  LGDP4532_font_size=font_size;
}

void LGDP4532_SetTextColors(uint32_t fColor, uint32_t bColor) {

  LGDP4532_text_foreground_color = fColor;
  LGDP4532_text_background_color = bColor;
}

void LGDP4532_PutChar(char c, uint16_t x, uint16_t y) {

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
  uint16_t data1,data2;

  // get pointer to the beginning of the selected font table
  pFont = (unsigned char *)FontTable[LGDP4532_font_size];

  // get the nColumns, nRows and nBytes
  nCols = *pFont;
  nRows = *(pFont + 1);
  nBytes = *(pFont + 2);

  // get pointer to the first byte of the desired character
  pChar = pFont + (nBytes * (c - 0x1F));

  LGDP4532_SetWindow(x,y,x + nCols - 1,y + nRows - 1);

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
        Word0 = LGDP4532_text_foreground_color;
      else
        Word0 = LGDP4532_text_background_color;
      Mask >>= 1;

      if (PixelRow & Mask)
        Word1 = LGDP4532_text_foreground_color;
      else
        Word1 = LGDP4532_text_background_color;
      Mask >>= 1;

      switch(LGDP4532_color_mode){
        case COLOR_16BIT:
          data1=(Word0 & 0xf80000) >> 8 | (Word0 & 0xfc00) >> 5 | (Word0 & 0xf8) >> 3;
          wr_dat(data1);
          data2=(Word1 & 0xf80000) >> 8 | (Word1 & 0xfc00) >> 5 | (Word1 & 0xf8) >> 3;
          wr_dat(data2);
          break;
        case COLOR_18BIT:
          data1=Word0>>22;
          data2=(Word0 & 0xfc0000) >> 6 | (Word0 & 0xfc00) >> 4 | (Word0 & 0xfc) >> 2;
          wr_dat(data1);
          wr_dat(data2);
          data1=Word1>>22;
          data2=(Word1 & 0xfc0000) >> 6 | (Word1 & 0xfc00) >> 4 | (Word1 & 0xfc) >> 2;
          wr_dat(data1);
          wr_dat(data2);
        break;
      }
    }
  }
}

void LGDP4532_PutStr(char *pString, uint16_t x, uint16_t y) {

  if(y+FontTable[LGDP4532_font_size][1]>LGDP4532_GetHeight())
    return;

  // loop until null-terminator is seen
  while (*pString) {
    if (x+FontTable[LGDP4532_font_size][0]>LGDP4532_GetWidth()) break;
    // draw the character
    LGDP4532_PutChar(*pString++, x, y);
    x+=FontTable[LGDP4532_font_size][0];
  }
}

void LGDP4532_PutStrCEOL(char *pString, uint16_t x, uint16_t y) {

  if(y+FontTable[LGDP4532_font_size][1]>LGDP4532_GetHeight())
    return;

  // loop until null-terminator is seen
  while (*pString) {
    if (x+FontTable[LGDP4532_font_size][0]>LGDP4532_GetWidth()) break;
    // draw the character
    LGDP4532_PutChar(*pString++, x, y);
    x+=FontTable[LGDP4532_font_size][0];
  }
  while(x+FontTable[LGDP4532_font_size][0]<=LGDP4532_GetWidth()) {
    LGDP4532_PutChar(' ', x, y);
    x+=FontTable[LGDP4532_font_size][0];
  }
}

void LGDP4532_PutStrCentered(char *pString, uint16_t y) {

  uint32_t length=strlen(pString)*FontTable[LGDP4532_font_size][0];

  LGDP4532_PutStr(pString,length>LGDP4532_GetWidth()?0:(LGDP4532_GetWidth()-length)/2,y);
}

void LGDP4532_ReadRegister(uint8_t reg,uint8_t length,uint16_t *val) {

  uint8_t i;

  // first read is dummy read
  val[0]=rd_reg(reg);

  for(i=0;i<length;i++) {
    val[i]=rd_dat();
  }
}

void LGDP4532_ReadMemory(uint16_t x0,uint16_t y0,uint16_t x1,uint16_t y1,uint16_t *data) {

  uint8_t i,j=(x1-x0+1)*(y1-y0+1);

  LGDP4532_SetWindow(x0,y0,x1,y1);
  // first read is dummy read
  *data=rd_dat();

  // in 18 bit color LSB of R and B dot data are not read out
  for(i=0;i<j;i++) {
    *data++=rd_dat();
  }
}
