/*
 *  SSD1289.c
 *
 *  Author: Kestutis Bivainis
 *
 */

#include "stm32f10x_conf.h"
#include "SSD1289.h"
#include "delay.h"

static COLOR_MODE SSD1289_color_mode;
static ORIENTATION_MODE SSD1289_orientation_mode;
static uint32_t SSD1289_am;

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
  FSMC_NORSRAMTimingInitTypeDef  FSMC_NORSRAMTimingInitStructure1;

  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_FSMC, ENABLE);

  FSMC_NORSRAMTimingInitStructure.FSMC_AddressSetupTime = 0x00;
  FSMC_NORSRAMTimingInitStructure.FSMC_AddressHoldTime = 0x00;
  FSMC_NORSRAMTimingInitStructure.FSMC_DataSetupTime = 0x02;
  FSMC_NORSRAMTimingInitStructure.FSMC_BusTurnAroundDuration = 0x00;
  FSMC_NORSRAMTimingInitStructure.FSMC_CLKDivision = 0x00;
  FSMC_NORSRAMTimingInitStructure.FSMC_DataLatency = 0x00;
  FSMC_NORSRAMTimingInitStructure.FSMC_AccessMode = FSMC_AccessMode_B;

  FSMC_NORSRAMTimingInitStructure1.FSMC_AddressSetupTime = 0x00;
  FSMC_NORSRAMTimingInitStructure1.FSMC_AddressHoldTime = 0x00;
  FSMC_NORSRAMTimingInitStructure1.FSMC_DataSetupTime = 0x05;
  FSMC_NORSRAMTimingInitStructure1.FSMC_BusTurnAroundDuration = 0x00;
  FSMC_NORSRAMTimingInitStructure1.FSMC_CLKDivision = 0x00;
  FSMC_NORSRAMTimingInitStructure1.FSMC_DataLatency = 0x00;
  FSMC_NORSRAMTimingInitStructure1.FSMC_AccessMode = FSMC_AccessMode_B;

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
  FSMC_NORSRAMInitStructure.FSMC_ReadWriteTimingStruct = &FSMC_NORSRAMTimingInitStructure1;
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

void SSD1289_Init(void) {

  volatile uint16_t id=0;

  GPIO_Configuration();
  FSMC_LCD_Init();

  lcd_rst();

  id=rd_reg(0x00); // Device code read id = 0x8989
  if(id==0x8989) {
    // power on
    wr_reg(DISPLAY_CONTROL,0x0021); // GON = 1, DTE = 0, D[1:0] = 01
    wr_reg(OSCILLATOR,0x0001); // oscillator ON
    wr_reg(DISPLAY_CONTROL,0x0023); // GON = 1, DTE = 0, D[1:0] = 11
    wr_reg(SLEEP_MODE,0x0000); // Exit sleep mode
    DWT_Delay(30000);
    wr_reg(DISPLAY_CONTROL,0x0033); // GON = 1, DTE = 1, D[1:0] = 11
    wr_reg(DRIVING_WAVEFORM_CONTROL,0x0600);
    SSD1289_ColorMode(COLOR_16BIT);
    SSD1289_OrientationMode(ORIENTATION_LANDSCAPE);
  }
}

void SSD1289_OrientationMode(ORIENTATION_MODE orientation_mode) {

  wr_reg(DRIVER_OUTPUT_CONTROL,orientation_mode);

  switch(orientation_mode) {
    case ORIENTATION_LANDSCAPE:
    case ORIENTATION_LANDSCAPE_REV:
       SSD1289_am=0x0008;
      break;
    case ORIENTATION_PORTRAIT:
    case ORIENTATION_PORTRAIT_REV:
      SSD1289_am=0x0000;
      break;
  }
  SSD1289_ColorMode(SSD1289_color_mode);
  SSD1289_orientation_mode=orientation_mode;
}

void SSD1289_ColorMode(COLOR_MODE color_mode) {

  wr_reg(ENTRY_MODE,color_mode|SSD1289_am);
  SSD1289_color_mode=color_mode;
}

void SSD1289_Gamma(uint16_t g1_,uint16_t g2_,uint16_t g3_,uint16_t g4_,uint16_t g5_,
                    uint16_t g6_,uint16_t g7_,uint16_t g8_,uint16_t g9_,uint16_t g10_) {

  wr_reg(GAMMA_0,g1_);
  wr_reg(GAMMA_1,g2_);
  wr_reg(GAMMA_2,g3_);
  wr_reg(GAMMA_3,g4_);
  wr_reg(GAMMA_4,g5_);
  wr_reg(GAMMA_5,g6_);
  wr_reg(GAMMA_6,g7_);
  wr_reg(GAMMA_7,g8_);
  wr_reg(GAMMA_8,g9_);
  wr_reg(GAMMA_9,g10_);
}

void SSD1289_SetPixel(uint16_t x,uint16_t y,uint32_t color) {

  uint16_t dest1,dest2;

  SSD1289_SetWindow(x,y,x,y);

  switch(SSD1289_color_mode){
    case COLOR_16BIT:
      dest1=(color & 0xf80000) >> 8 | (color & 0xfc00) >> 5 | (color & 0xf8) >> 3;
      wr_dat(dest1);
      break;
    case COLOR_18BIT:
      dest1=(color&0xfcfc00)>>8;
      dest2=color&0xfc;
      wr_dat(dest1);
      wr_dat(dest2);
      break;
  }
}

void SSD1289_ClearScreen(uint32_t color) {

  SSD1289_FillRectangle(0,0,SSD1289_GetWidth()-1,SSD1289_GetHeight()-1,color);
}

void SSD1289_FillPixel(uint16_t x0,uint16_t y0,uint16_t x1,uint16_t y1,uint32_t *color) {

  uint32_t i,j;
  uint16_t dest1,dest2;

  SSD1289_SetWindow(x0,y0,x1,y1);

  j=(x1-x0+1)*(y1-y0+1);
  switch(SSD1289_color_mode){
    case COLOR_16BIT:
      for(i=0;i<j;i++) {
        dest1=(color[i] & 0xf80000) >> 8 | (color[i] & 0xfc00) >> 5 | (color[i] & 0xf8) >> 3;
        wr_dat(dest1);
      }
       break;
    case COLOR_18BIT:
      for(i=0;i<j;i++) {
        dest1=(color[i]&0xfcfc00)>>8;
        dest2=color[i]&0xfc;
        wr_dat(dest1);
        wr_dat(dest2);
      }
       break;
  }
}

void SSD1289_FillRectangle(uint16_t x0,uint16_t y0,uint16_t x1,uint16_t y1,uint32_t color) {

  uint32_t i,j;
  uint16_t dest1,dest2;

  SSD1289_SetWindow(x0,y0,x1,y1);

  j=(x1-x0+1)*(y1-y0+1);
  switch(SSD1289_color_mode){
    case COLOR_16BIT:
      dest1=(color & 0xf80000) >> 8 | (color & 0xfc00) >> 5 | (color & 0xf8) >> 3;
      for(i=0;i<j;i++) {
        wr_dat(dest1);
      }
       break;
    case COLOR_18BIT:
      dest1=(color&0xfcfc00)>>8;
      dest2=color&0xfc;
      for(i=0;i<j;i++) {
        wr_dat(dest1);
        wr_dat(dest2);
      }
      break;
  }
}

void SSD1289_SetScrollPosition(uint16_t pos) {
  wr_reg(GATE_SCAN_POSITION,pos);
}

void SSD1289_SetWindow(uint16_t x0,uint16_t y0,uint16_t x1,uint16_t y1) {

  switch(SSD1289_orientation_mode) {
    case ORIENTATION_LANDSCAPE:
    case ORIENTATION_LANDSCAPE_REV:
      wr_reg(HORIZONTAL_POSITION,(y1<<8)|y0);
      wr_reg(SET_GDDRAM_X,y0);
      wr_reg(VERTICAL_POSITION_START,x0);
      wr_reg(VERTICAL_POSITION_END,x1);
      wr_reg(SET_GDDRAM_Y,x0);
    break;
    case ORIENTATION_PORTRAIT:
    case ORIENTATION_PORTRAIT_REV:
      wr_reg(HORIZONTAL_POSITION,(x1<<8)|x0);
      wr_reg(SET_GDDRAM_X,x0);
      wr_reg(VERTICAL_POSITION_START,y0);
      wr_reg(VERTICAL_POSITION_END,y1);
      wr_reg(SET_GDDRAM_Y,y0);
    break;
  }
  wr_cmd(GRAM_WRITE_DATA);
}

void SSD1289_Sleep(void) {
  wr_reg(SLEEP_MODE,0x0001);
}

void SSD1289_Wakeup(void){
  wr_reg(SLEEP_MODE,0x0000);
}

uint16_t SSD1289_GetWidth(void) {

  uint16_t ret;

  switch(SSD1289_orientation_mode){
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

uint16_t SSD1289_GetHeight(void) {

  uint16_t ret;

  switch(SSD1289_orientation_mode){
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
