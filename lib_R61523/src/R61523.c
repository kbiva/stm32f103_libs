/*
 *  R61523.c
 *
 *  Author: Kestutis Bivainis
 *
 */
#include "stm32f10x_conf.h"
#include "R61523.h"
#include "delay.h"

static COLOR_MODE R61523_color_mode;
static ORIENTATION_MODE R61523_orientation_mode;

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

static void GPIO_Configuration(void) {

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
  //FSMC_NORSRAMInitStructure.FSMC_BurstAccessMode = FSMC_BurstAccessMode_Enable;
  FSMC_NORSRAMInitStructure.FSMC_WaitSignalPolarity = FSMC_WaitSignalPolarity_Low;
  FSMC_NORSRAMInitStructure.FSMC_WrapMode = FSMC_WrapMode_Disable;
  FSMC_NORSRAMInitStructure.FSMC_WaitSignalActive = FSMC_WaitSignalActive_BeforeWaitState;
  FSMC_NORSRAMInitStructure.FSMC_WriteOperation = FSMC_WriteOperation_Enable;
  FSMC_NORSRAMInitStructure.FSMC_WaitSignal = FSMC_WaitSignal_Enable;
  FSMC_NORSRAMInitStructure.FSMC_ExtendedMode = FSMC_ExtendedMode_Disable;
  FSMC_NORSRAMInitStructure.FSMC_WriteBurst = FSMC_WriteBurst_Disable;
  //FSMC_NORSRAMInitStructure.FSMC_WriteBurst = FSMC_WriteBurst_Enable;
  FSMC_NORSRAMInitStructure.FSMC_AsynchronousWait = FSMC_AsynchronousWait_Disable;
  FSMC_NORSRAMInitStructure.FSMC_ReadWriteTimingStruct = &FSMC_NORSRAMTimingInitStructure;
  FSMC_NORSRAMInitStructure.FSMC_WriteTimingStruct = &FSMC_NORSRAMTimingInitStructure;

  FSMC_NORSRAMInit(&FSMC_NORSRAMInitStructure);

  FSMC_NORSRAMCmd(FSMC_Bank1_NORSRAM1, ENABLE);
}

static void lcd_rst(void){

  GPIO_ResetBits(RST_Port, RST_Pin);
  DWT_Delay(100000);
  GPIO_SetBits(RST_Port, RST_Pin);
  DWT_Delay(100000);
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

void R61523_Init(void) {

  volatile uint32_t id=0;
  volatile uint16_t tmp=0;

  GPIO_Configuration();
  FSMC_LCD_Init();

  lcd_rst();

  wr_cmd(MCAP);
  wr_dat(0x0004);
  wr_cmd(BACKLIGHT_CONTROL_2);
  wr_dat(0x0001);// PWMON=1
  wr_dat(0x0000);// BDCV=0 (off)
  wr_dat(0x0003);// 13.7kHz
  wr_dat(0x0018);// PWMWM=1, LEDPWME=1
  wr_cmd(SLEEP_OUT);
  DWT_Delay(120000);

  wr_cmd(DEVICE_CODE_READ);
  tmp=rd_dat();
  id=rd_dat()<<24;
  id|=rd_dat()<<16;
  id|=rd_dat()<<8;
  id|=rd_dat();

  if(id==0x01221523) {

    wr_cmd(NORMAL_DISPLAY_MODE_ON);

    R61523_OrientationMode(ORIENTATION_LANDSCAPE);
    R61523_ColorMode(COLOR_16BIT);

    wr_cmd(SET_FRAME_AND_INTERFACE);
    wr_dat(0x0080);
    wr_dat(0x0010);

    wr_cmd(DISPLAY_ON);
  }
}

void R61523_OrientationMode(ORIENTATION_MODE orientation_mode) {

  wr_cmd(SET_ADDRESS_MODE);
  wr_dat(orientation_mode);
  R61523_orientation_mode=orientation_mode;
}

void R61523_ColorMode(COLOR_MODE color_mode) {

  wr_cmd(SET_PIXEL_FORMAT);
  wr_dat(color_mode);
  R61523_color_mode=color_mode;
}

void R61523_Gamma(uint16_t g1_,uint16_t g2_,uint16_t g3_,uint16_t g4_,uint16_t g5_,
                  uint16_t g6_,uint16_t g7_,uint16_t g8_,uint16_t g9_,uint16_t g10_,
                  uint16_t g11_,uint16_t g12_,uint16_t g13_) {

  uint8_t i;

  wr_cmd(GAMMA_SET_A);
  for(i=0;i<2;i++) {
    wr_dat(g1_);
    wr_dat(g1_);
    wr_dat(g4_<<4|g3_);
    wr_dat(g6_<<4|g5_);
    wr_dat(g7_);
    wr_dat(g9_<<4|g8_);
    wr_dat(g11_<<4|g10_);
    wr_dat(g12_);
    wr_dat(g13_);
  }
  wr_cmd(GAMMA_SET_B);
  for(i=0;i<2;i++) {
    wr_dat(g1_);
    wr_dat(g1_);
    wr_dat(g4_<<4|g3_);
    wr_dat(g6_<<4|g5_);
    wr_dat(g7_);
    wr_dat(g9_<<4|g8_);
    wr_dat(g11_<<4|g10_);
    wr_dat(g12_);
    wr_dat(g13_);
  }
  wr_cmd(GAMMA_SET_C);
  for(i=0;i<2;i++) {
    wr_dat(g1_);
    wr_dat(g1_);
    wr_dat(g4_<<4|g3_);
    wr_dat(g6_<<4|g5_);
    wr_dat(g7_);
    wr_dat(g9_<<4|g8_);
    wr_dat(g11_<<4|g10_);
    wr_dat(g12_);
    wr_dat(g13_);
  }
}

void R61523_SetPixel(uint16_t x,uint16_t y,uint32_t color) {

  uint16_t dest1,dest2;

  R61523_SetWindow(x,y,x,y);

  switch(R61523_color_mode){
    case COLOR_16BIT:
      dest1=(color & 0xf80000) >> 19 | (color & 0xfc00) >> 5 | (color & 0xf8) << 8;
      wr_dat(dest1);
      break;
    case COLOR_18BIT:
      dest1=color&0xfc;
      dest2=((color >> 16) | (color & 0xFF00)) & 0xfcfc;
      wr_dat(dest1);
      wr_dat(dest2);
      break;
    case COLOR_24BIT:
      dest1=color&0xff;
      dest2=(color >> 16) | (color & 0xFF00);
      wr_dat(dest1);
      wr_dat(dest2);
      break;
  }
}

void R61523_ClearScreen(uint32_t color) {

  R61523_FillRectangle(START_X,START_Y,END_X,END_Y,color);
}

void R61523_FillPixel(uint16_t x0,uint16_t y0,uint16_t x1,uint16_t y1,uint32_t *color) {

  uint32_t i,j;
  uint16_t dest1,dest2;

  R61523_SetWindow(x0,y0,x1,y1);

  j=(x1-x0+1)*(y1-y0+1);
  switch(R61523_color_mode){
    case COLOR_16BIT:
      for(i=0;i<j;i++) {
        dest1=(color[i] & 0xf80000) >> 19 | (color[i] & 0xfc00) >> 5 | (color[i] & 0xf8) << 8;
        wr_dat(dest1);
      }
       break;
    case COLOR_18BIT:
      for(i=0;i<j;i++) {
        dest1=color[i]&0xfc;
        dest2=((color[i] >> 16) | (color[i] & 0xFF00)) & 0xfcfc;
        wr_dat(dest1);
        wr_dat(dest2);
      }
       break;
    case COLOR_24BIT:
      for(i=0;i<j;i++) {
        dest1=color[i]&0xff;
        dest2=(color[i] >> 16) | (color[i] & 0xFF00);
        wr_dat(dest1);
        wr_dat(dest2);
      }
       break;
  }
}

void R61523_FillRectangle(uint16_t x0,uint16_t y0,uint16_t x1,uint16_t y1,uint32_t color) {

  uint32_t i,j;
  uint16_t dest1,dest2;

  R61523_SetWindow(x0,y0,x1,y1);

  j=(x1-x0+1)*(y1-y0+1);
  switch(R61523_color_mode){
    case COLOR_16BIT:
      dest1=(color & 0xf80000) >> 19 | (color & 0xfc00) >> 5 | (color & 0xf8) << 8;
      for(i=0;i<j;i++) {
        wr_dat(dest1);
      }
       break;
    case COLOR_18BIT:
      dest1=color&0xfc;
      dest2=((color >> 16) | (color & 0xFF00)) & 0xfcfc;
      for(i=0;i<j;i++) {
        wr_dat(dest1);
        wr_dat(dest2);
      }
    case COLOR_24BIT:
      dest1=color&0xff;
      dest2=(color >> 16) | (color & 0xFF00);
      for(i=0;i<j;i++) {
        wr_dat(dest1);
        wr_dat(dest2);
      }
      break;
  }
}

uint16_t R61523_GetWidth(void) {

  uint16_t ret;

  switch(R61523_orientation_mode){
    case ORIENTATION_LANDSCAPE:
    case ORIENTATION_LANDSCAPE_REV:
      ret=640;
    break;
    case ORIENTATION_PORTRAIT:
    case ORIENTATION_PORTRAIT_REV:
      ret=360;
    break;
  }
  return ret;
}

uint16_t R61523_GetHeight(void) {

  uint16_t ret;

  switch(R61523_orientation_mode){
    case ORIENTATION_LANDSCAPE:
    case ORIENTATION_LANDSCAPE_REV:
      ret=360;
    break;
    case ORIENTATION_PORTRAIT:
    case ORIENTATION_PORTRAIT_REV:
      ret=640;
    break;
  }
  return ret;
}

void R61523_SetWindow(uint16_t x0,uint16_t y0,uint16_t x1,uint16_t y1) {

  wr_cmd(SET_COLUMN_ADDRESS);
  wr_dat(x0>>8);
  wr_dat(x0&0xff);
  wr_dat(x1>>8);
  wr_dat(x1&0xff);
  wr_cmd(SET_PAGE_ADDRESS);
  wr_dat(y0>>8);
  wr_dat(y0&0xff);
  wr_dat(y1>>8);
  wr_dat(y1&0xff);
  wr_cmd(MEMORY_WRITE);
}


void R61523_Sleep(void) {

  wr_cmd(DISPLAY_OFF);
  wr_cmd(SLEEP_IN);
  DWT_Delay(120000);
}

void R61523_Wakeup(void){

  wr_cmd(SLEEP_OUT);
  DWT_Delay(120000);
  wr_cmd(DISPLAY_ON);
}

