/*
 *  MC2PA8201.h
 *
 *  Author: Kestutis Bivainis
 *
 */

#include "stm32f10x_conf.h"
#include "MC2PA8201.h"
#include "delay.h"

static COLOR_MODE MC2PA8201_color_mode;
static ORIENTATION_MODE MC2PA8201_orientation_mode;

static uint8_t RGB16bit[192]={
  0x00,0x02,0x04,0x06,0x08,0x0A,0x0C,0x0E,0x10,0x12,0x14,0x16,0x18,0x1A,0x1C,0x1E,
  0x20,0x22,0x24,0x26,0x28,0x2A,0x2C,0x2E,0x30,0x32,0x34,0x36,0x38,0x3A,0x3C,0x3F,

  0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x0A,0x0B,0x0C,0x0D,0x0E,0x0F,
  0x10,0x11,0x12,0x13,0x14,0x15,0x16,0x17,0x18,0x19,0x1A,0x1B,0x1C,0x1D,0x1E,0x1F,
  0x20,0x21,0x22,0x23,0x24,0x25,0x26,0x27,0x28,0x29,0x2A,0x2B,0x2C,0x2D,0x2E,0x2F,
  0x30,0x31,0x32,0x33,0x34,0x35,0x36,0x37,0x38,0x39,0x3A,0x3B,0x3C,0x3D,0x3E,0x3F,

  0x00,0x02,0x04,0x06,0x08,0x0A,0x0C,0x0E,0x10,0x12,0x14,0x16,0x18,0x1A,0x1C,0x1E,
  0x20,0x22,0x24,0x26,0x28,0x2A,0x2C,0x2E,0x30,0x32,0x34,0x36,0x38,0x3A,0x3C,0x3F,
};

static uint8_t RGB12_18bit[192]={
  0x00,0x04,0x08,0x0C,0x10,0x14,0x18,0x1C,0x20,0x24,0x28,0x2C,0x30,0x34,0x38,0x3C,
  0x40,0x44,0x48,0x4C,0x50,0x54,0x58,0x5C,0x60,0x64,0x68,0x6C,0x70,0x74,0x78,0x7C,
  0x80,0x84,0x88,0x8C,0x90,0x94,0x98,0x9C,0xA0,0xA4,0xA8,0xAC,0xB0,0xB4,0xB8,0xBC,
  0xC0,0xC4,0xC8,0xCC,0xD0,0xD4,0xD8,0xDC,0xE0,0xE4,0xE8,0xEC,0xF0,0xF4,0xF8,0xFF,

  0x00,0x04,0x08,0x0C,0x10,0x14,0x18,0x1C,0x20,0x24,0x28,0x2C,0x30,0x34,0x38,0x3C,
  0x40,0x44,0x48,0x4C,0x50,0x54,0x58,0x5C,0x60,0x64,0x68,0x6C,0x70,0x74,0x78,0x7C,
  0x80,0x84,0x88,0x8C,0x90,0x94,0x98,0x9C,0xA0,0xA4,0xA8,0xAC,0xB0,0xB4,0xB8,0xBC,
  0xC0,0xC4,0xC8,0xCC,0xD0,0xD4,0xD8,0xDC,0xE0,0xE4,0xE8,0xEC,0xF0,0xF4,0xF8,0xFF,

  0x00,0x04,0x08,0x0C,0x10,0x14,0x18,0x1C,0x20,0x24,0x28,0x2C,0x30,0x34,0x38,0x3C,
  0x40,0x44,0x48,0x4C,0x50,0x54,0x58,0x5C,0x60,0x64,0x68,0x6C,0x70,0x74,0x78,0x7C,
  0x80,0x84,0x88,0x8C,0x90,0x94,0x98,0x9C,0xA0,0xA4,0xA8,0xAC,0xB0,0xB4,0xB8,0xBC,
  0xC0,0xC4,0xC8,0xCC,0xD0,0xD4,0xD8,0xDC,0xE0,0xE4,0xE8,0xEC,0xF0,0xF4,0xF8,0xFF
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

static void FSMC_LCD_Init(void)
{
  FSMC_NORSRAMInitTypeDef FSMC_NORSRAMInitStructure;
  FSMC_NORSRAMTimingInitTypeDef  FSMC_NORSRAMTimingInitStructure;

  /* Enable the FSMC Clock */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_FSMC, ENABLE);
  // Nokia E63 = 3,2
  // Nokia E73 = 1,1
  FSMC_NORSRAMTimingInitStructure.FSMC_AddressSetupTime = 0x01;
  FSMC_NORSRAMTimingInitStructure.FSMC_AddressHoldTime = 0x00;
  FSMC_NORSRAMTimingInitStructure.FSMC_DataSetupTime = 0x02;
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

  GPIO_ResetBits(GPIOE, GPIO_Pin_1);
  DWT_Delay(100000);
  GPIO_SetBits(GPIOE, GPIO_Pin_1 );
  DWT_Delay(100000);
}

static __forceinline void wr_cmd(uint8_t index) {
  LCD_REG8 = index;
}

static __forceinline void wr_dat(uint8_t val) {
  LCD_DAT8 = val;
}

static __forceinline void wr_reg(uint8_t index,uint8_t val) {
  LCD_REG8 = index;
  LCD_DAT8 = val;
}

uint8_t MC2PA8201_Init(void) {

  GPIO_Configuration();
  FSMC_LCD_Init();

  lcd_rst();

  wr_cmd(SLEEP_OUT);
  DWT_Delay(120000);
  wr_cmd(DISPLAY_INVERSION_OFF);
  wr_cmd(IDLE_MODE_OFF);
  wr_cmd(NORMAL_DISPLAY_MODE_ON);

  //wr_reg(GAMMA_SET,0x08);

  //MC2PA8201_ScrollArea(240);

  DWT_Delay(125000);
  wr_cmd(DISPLAY_ON);
  DWT_Delay(100000);

  return MC2PA8201_OK;
}

void MC2PA8201_ColorMode(COLOR_MODE color_mode) {

  uint32_t i;

  MC2PA8201_color_mode=color_mode;
  wr_reg(INTERFACE_PIXEL_FORMAT,MC2PA8201_color_mode);

  wr_cmd(COLOUR_SET);
  switch(MC2PA8201_color_mode){
    case COLOR_12BIT:
      for(i=0;i<192;i++) {
        wr_dat(RGB12_18bit[i]);
      }
      break;
    case COLOR_16BIT:
      for(i=0;i<192;i++) {
        wr_dat(RGB16bit[i]);
      }
      break;
    case COLOR_18BIT:
      for(i=0;i<192;i++) {
        wr_dat(RGB12_18bit[i]);
      }
      break;
    case COLOR_24BIT:
      break;
  }
}

void MC2PA8201_OrientationMode(ORIENTATION_MODE orientation_mode) {

  MC2PA8201_orientation_mode=orientation_mode;
  wr_reg(MEMORY_ACCESS_CONTROL,orientation_mode);
}

void MC2PA8201_ClearScreen(uint32_t color) {
  uint32_t i;
  uint8_t b1,b2,b3,r,g,b;

  switch(MC2PA8201_orientation_mode){
    case ORIENTATION_PORTRAIT:
    case ORIENTATION_PORTRAIT_REV:
      MC2PA8201_SetWindow(0,0,239,319);
    break;
    case ORIENTATION_LANDSCAPE:
    case ORIENTATION_LANDSCAPE_REV:
      MC2PA8201_SetWindow(0,0,319,239);
    break;
  }

  r=color>>16;
  g=color>>8;
  b=color;

  switch(MC2PA8201_color_mode) {
    case COLOR_12BIT:
      b1=(r&0xF0)|(g>>4);
      b2=(b&0xF0)|(r>>4);
      b3=(g&0xF0)|(b>>4);

      for(i=0;i<240*320/2;i++) {
        wr_dat(b1);
        wr_dat(b2);
        wr_dat(b3);
      }
      break;
    case COLOR_16BIT:
      b1=(r&0xF8)|((g&0xE0)>>5);
      b2=((g&0x1C)<<3)|((b&0xF8)>>3);
      for(i=0;i<240*320;i++) {
        wr_dat(b1);
        wr_dat(b2);
      }
      break;
    case COLOR_18BIT:
      b1=r&0xFC;
      b2=g&0xFC;
      b3=b&0xFC;
      for(i=0;i<240*320;i++) {
        wr_dat(b1);
        wr_dat(b2);
        wr_dat(b3);
      }
      break;
    case COLOR_24BIT:
      b1=r;
      b2=g;
      b3=b;
       for(i=0;i<240*320;i++) {
        wr_dat(r);
        wr_dat(g);
        wr_dat(b);
      }
      break;
  }
}

void MC2PA8201_Fill(uint16_t x0,uint16_t y0,uint16_t x1,uint16_t y1,uint32_t color) {

  uint32_t i,j=(x1-x0+1)*(y1-y0+1);
  uint8_t b1,b2,b3,r,g,b;

  MC2PA8201_SetWindow(x0,y0,x1,y1);

  r=color>>16;
  g=color>>8;
  b=color;

  switch(MC2PA8201_color_mode) {
    case COLOR_12BIT:
      b1=(r&0xF0)|(g>>4);
      b2=(b&0xF0)|(r>>4);
      b3=(g&0xF0)|(b>>4);
      for(i=0;i<j;i+=2) {
        wr_dat(b1);
        wr_dat(b2);
        wr_dat(b3);
      }
      break;
    case COLOR_16BIT:
      b1=(r&0xF8)|((g&0xE0)>>5);
      b2=((g&0x1C)<<3)|((b&0xF8)>>3);
      for(i=0;i<j;i++) {
        wr_dat(b1);
        wr_dat(b2);
      }
      break;
    case COLOR_18BIT:
      for(i=0;i<j;i++) {
        wr_dat(r&0xFC);
        wr_dat(g&0xFC);
        wr_dat(b&0xFC);
      }
      break;
    case COLOR_24BIT:
       for(i=0;i<j;i++) {
        wr_dat(r);
        wr_dat(g);
        wr_dat(b);
      }
      break;
  }
}

void MC2PA8201_SetWindow(uint16_t x0,uint16_t y0,uint16_t x1,uint16_t y1) {

  wr_cmd(COLUMN_ADDRESS_SET);
  wr_dat(x0>>8);
  wr_dat(x0);
  wr_dat(x1>>8);
  wr_dat(x1);

  wr_cmd(PAGE_ADDRESS_SET);
  wr_dat(y0>>8);
  wr_dat(y0);
  wr_dat(y1>>8);
  wr_dat(y1);

  wr_cmd(MEMORY_WRITE);
}

void MC2PA8201_FillPixel(uint16_t x0,uint16_t y0,uint16_t x1,uint16_t y1,uint32_t *color) {

  uint32_t i,j=(x1-x0+1)*(y1-y0+1);

  MC2PA8201_SetWindow(x0,y0,x1,y1);

  switch(MC2PA8201_color_mode) {
    case COLOR_12BIT:
      for(i=0;i<j;i+=2) {
        wr_dat(((color[i]>>16)&0xF0)|((color[i]>>12)&0x0F));
        wr_dat(((color[i])&0xF0)|((color[i+1]>>16)&0x0F));
        wr_dat(((color[i+1]>>8)&0xF0)|((color[i+1]>>4)&0x0F));
      }
      break;
    case COLOR_16BIT:
      for(i=0;i<j;i++) {
        wr_dat(((color[i]>>16)&0xF8)|(((color[i]>>13)&0x07)));
        wr_dat(((color[i]>>5)&0xE0)|((color[i]>>3)&0x1F));
      }
      break;
    case COLOR_18BIT:
      for(i=0;i<j;i++) {
        wr_dat((color[i]>>16)&0xFC);
        wr_dat((color[i]>>8)&0xFC);
        wr_dat((color[i])&0xFC);
      }
      break;
    case COLOR_24BIT:
       for(i=0;i<j;i++) {
        wr_dat(color[i]>>16);
        wr_dat(color[i]>>8);
        wr_dat(color[i]);
      }
      break;
  }
}

void MC2PA8201_SetPixel(uint16_t x, uint16_t y, uint32_t color)
{
  uint8_t r,g,b;

  MC2PA8201_SetWindow(x,y,x,y);

  r=color>>16;
  g=color>>8;
  b=color;

  switch(MC2PA8201_color_mode) {
    case COLOR_12BIT:
      wr_dat((r&0xF0)|(g>>4));
      wr_dat((b&0xF0)|(r>>4));
      wr_dat((g&0xF0)|(b>>4));
      break;
    case COLOR_16BIT:
      wr_dat((r&0xF8)|((g>>5)&0x07));
      wr_dat(((g<<3)&0x0E)|((b>>3)&0x1F));
      break;
    case COLOR_18BIT:
      wr_dat(r&0xFC);
      wr_dat(g&0xFC);
      wr_dat(b&0xFC);
      break;
    case COLOR_24BIT:
      wr_dat(r);
      wr_dat(g);
      wr_dat(b);
      break;
  }
}

void MC2PA8201_SetScrollPosition(uint16_t pos) {
  wr_cmd(VERTICAL_SCROLLING_START_ADDRESS);
  wr_dat(pos>>8);
  wr_dat(pos);
}

void MC2PA8201_ScrollArea(uint16_t y,uint16_t pos) {
  uint16_t bfa=240-pos-y;
  wr_cmd(VERTICAL_SCROLLING_DEFINITION);
  wr_dat(y>>8);
  wr_dat(y);
  wr_dat(pos>>8);
  wr_dat(pos);
  wr_dat(bfa>>8);
  wr_dat(bfa);
}

void MC2PA8201_PartialArea(uint16_t y0,uint16_t y1) {
  wr_cmd(PARTIAL_AREA);
  wr_dat(y0>>8);
  wr_dat(y0);
  wr_dat(y1>>8);
  wr_dat(y1);
}

void MC2PA8201_PartialMode(void) {
  wr_cmd(PARTIAL_MODE_ON);
}

void MC2PA8201_NormalDisplayMode(void) {
  wr_cmd(NORMAL_DISPLAY_MODE_ON);
}

void MC2PA8201_Sleep(void) {
  wr_cmd(DISPLAY_OFF);
  wr_cmd(SLEEP_IN);
  DWT_Delay(5000);
}

void MC2PA8201_Wakeup(void) {
  wr_cmd(SLEEP_OUT);
  DWT_Delay(120000);
  wr_cmd(DISPLAY_ON);
  DWT_Delay(5000);
}

void MC2PA8201_Gamma(uint8_t val) {
  wr_reg(GAMMA_SET,val);
}

void MC2PA8201_IdleModeOn(void) {
  wr_cmd(IDLE_MODE_ON);
}

void MC2PA8201_IdleModeOff(void) {
  wr_cmd(IDLE_MODE_OFF);
}

void MC2PA8201_DisplayInversionOn(void) {
  wr_cmd(DISPLAY_INVERSION_ON);
}

void MC2PA8201_DisplayInversionOff(void) {
  wr_cmd(DISPLAY_INVERSION_OFF);
}

uint16_t MC2PA8201_GetWidth(void) {

  uint16_t ret;

  switch(MC2PA8201_orientation_mode){
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

uint16_t MC2PA8201_GetHeight(void) {

  uint16_t ret;

  switch(MC2PA8201_orientation_mode){
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
