/*
 *  HX8352A.c
 *
 *  Author: Kestutis Bivainis
 *
 */

#include "stm32f10x_conf.h"
#include "HX8352A.h"
#include "delay.h"

static COLOR_MODE HX8352A_color_mode;
static ORIENTATION_MODE HX8352A_orientation_mode;

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
  FSMC_NORSRAMTimingInitStructure.FSMC_BusTurnAroundDuration = 0x0;
  FSMC_NORSRAMTimingInitStructure.FSMC_CLKDivision = 0x00;
  FSMC_NORSRAMTimingInitStructure.FSMC_DataLatency = 0x00;
  FSMC_NORSRAMTimingInitStructure.FSMC_AccessMode = FSMC_AccessMode_B;

  FSMC_NORSRAMInitStructure.FSMC_Bank = FSMC_Bank1_NORSRAM1;
  FSMC_NORSRAMInitStructure.FSMC_DataAddressMux = FSMC_DataAddressMux_Disable;
  FSMC_NORSRAMInitStructure.FSMC_MemoryType = FSMC_MemoryType_SRAM;
  FSMC_NORSRAMInitStructure.FSMC_MemoryDataWidth = FSMC_MemoryDataWidth_16b;
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

  FSMC_NORSRAMCmd(FSMC_Bank1_NORSRAM1, ENABLE);
}

static void lcd_rst(void) {

  GPIO_ResetBits(RST_Port, RST_Pin);
  DWT_Delay(15000);
  GPIO_SetBits(RST_Port, RST_Pin);
  DWT_Delay(100000);
}

static __forceinline void wr_cmd(uint16_t index) {
  LCD_REG16 = index;
}

static __forceinline void wr_dat(uint16_t val) {
  LCD_DAT16 = val;
}

static __forceinline void wr_reg(uint16_t index,uint16_t val) {
  wr_cmd(index);
  wr_dat(val);
}

static __forceinline uint16_t rd_reg(uint16_t index) {
  wr_cmd(index);
  return (LCD_DAT16);
}

static __forceinline uint16_t rd_dat(void) {
  return (LCD_DAT16);
}

static void HX8352A_DGC_LUT(void) {

  uint32_t i;

  wr_reg(IP_CONTROL,0x01);
  wr_cmd(DGC_LUT_WRITE);
  for(i=0;i<3;i++) {
    wr_dat(0x00);
    wr_dat(0x03);
    wr_dat(0x0A);
    wr_dat(0x0F);
    wr_dat(0x13);
    wr_dat(0x16);
    wr_dat(0x19);
    wr_dat(0x1C);
    wr_dat(0x1E);
    wr_dat(0x1F);
    wr_dat(0x25);
    wr_dat(0x2A);
    wr_dat(0x30);
    wr_dat(0x35);
    wr_dat(0x39);
    wr_dat(0x3D);
    wr_dat(0x41);
    wr_dat(0x45);
    wr_dat(0x48);
    wr_dat(0x4C);
    wr_dat(0x4F);
    wr_dat(0x53);
    wr_dat(0x58);
    wr_dat(0x5D);
    wr_dat(0x61);
    wr_dat(0x66);
    wr_dat(0x6A);
    wr_dat(0x6E);
    wr_dat(0x72);
    wr_dat(0x76);
    wr_dat(0x7A);
    wr_dat(0x7E);
    wr_dat(0x82);
    wr_dat(0x85);
    wr_dat(0x89);
    wr_dat(0x8D);
    wr_dat(0x90);
    wr_dat(0x94);
    wr_dat(0x97);
    wr_dat(0x9A);
    wr_dat(0x9D);
    wr_dat(0xA2);
    wr_dat(0xA5);
    wr_dat(0xA9);
    wr_dat(0xAC);
    wr_dat(0xB0);
    wr_dat(0xB4);
    wr_dat(0xB8);
    wr_dat(0xBC);
    wr_dat(0xC0);
    wr_dat(0xC3);
    wr_dat(0xC8);
    wr_dat(0xCC);
    wr_dat(0xD2);
    wr_dat(0xD6);
    wr_dat(0xDC);
    wr_dat(0xDF);
    wr_dat(0xE2);
    wr_dat(0xE5);
    wr_dat(0xE8);
    wr_dat(0xEC);
    wr_dat(0xEF);
    wr_dat(0xF4);
    wr_dat(0xFF);
  }
}

static void Display_On_Function(void) {

  wr_reg(SOURCE_CONTROL_1,0xFF);
  wr_reg(SOURCE_CONTROL_2,0x0E);
  wr_reg(CYCLE_CONTROL_10,0x38);
  wr_reg(CYCLE_CONTROL_11,0x38);
  wr_reg(DISPLAY_CONTROL_2,0x38);
  DWT_Delay(50000);
  wr_reg(DISPLAY_CONTROL_2,0x3C);
  wr_reg(DISPLAY_MODE_CONTROL,0x02);
  //wr_reg(PANEL_CONTROL,0x00);//?
}

static void Display_Off_Function(void) {

// display Off
  wr_reg(DISPLAY_CONTROL_2,0x38);      //GON=1, DTE=1, D=10
  DWT_Delay(40000);

  wr_reg(DISPLAY_CONTROL_2,0x28);      //GON=1, DTE=0, D=10
  DWT_Delay(40000);

  wr_reg(DISPLAY_CONTROL_2,0x00);      //GON=0, DTE=0, D=00
}

static void Power_Supply_Function(void) {

  wr_reg(OSC_CONTROL_1,0x91);
  wr_reg(DISPLAY_CONTROL_1,0x01);// TE on
  wr_reg(CYCLE_CONTROL_1,0x14);
  DWT_Delay(20000);
  wr_reg(POWER_CONTROL_3,0x13);
  wr_reg(POWER_CONTROL_2,0x11);
  wr_reg(POWER_CONTROL_4,0x00);
  wr_reg(POWER_CONTROL_5,0x08);
  wr_reg(VCOM_CONTROL,0x3B);
  DWT_Delay(30000);
  wr_reg(POWER_CONTROL_1,0x0A);
  wr_reg(POWER_CONTROL_1,0x1A);
  DWT_Delay(50000);
  wr_reg(POWER_CONTROL_1,0x12);
  DWT_Delay(50000);
  wr_reg(POWER_CONTROL_6,0x2E);
}

static void Power_On_Function(void) {

  // power on
  wr_reg(TEST_MODE_CONTROL,0x02);
  wr_reg(VDDD_CONTROL,0x02);
  wr_reg(SOURCE_GAMMA_RESISTOR_1,0x00);
  wr_reg(SOURCE_GAMMA_RESISTOR_2,0xB3);
  //wr_reg(SYNC_FUNCTION,0x01);//?
  wr_reg(TEST_MODE_CONTROL,0x00);
}

static void Power_Off_Function(void) {

  // power Off
  wr_reg(POWER_CONTROL_6,0x14);      // VCOMG=0, VDV=1_0100
  DWT_Delay(10000);
  wr_reg(POWER_CONTROL_1,0x02);    // GASENB=0, PON=0, DK=0, XDK=0, VLCD_TRI=1, STB=0
  DWT_Delay(10000);
  wr_reg(POWER_CONTROL_1,0x0A);    // GASENB=0, PON=0, DK=1, XDK=0, VLCD_TRI=1, STB=0
  DWT_Delay(10000);
  wr_reg(POWER_CONTROL_3,0x40);    // AP=000
  DWT_Delay(10000);
  wr_reg(SOURCE_CONTROL_1,0x00);   // N_SAP=1100 0000
  DWT_Delay(10000);
}

uint8_t HX8352A_Init(COLOR_MODE color_mode) {

  volatile uint32_t id=0;

  GPIO_Configuration();

  FSMC_LCD_Init();

  lcd_rst();

  HX8352A_color_mode=color_mode;

  id=rd_reg(0x00); // Device code read id = 0x0052

  if(id==0x0052) {

    DWT_Delay(160000);

    Power_On_Function();

    Power_Supply_Function();

    wr_reg(TEST_MODE_CONTROL,0x02);
    wr_reg(0x93,0x10);
    wr_reg(TEST_MODE_CONTROL,0x00);

    HX8352A_DGC_LUT();

    Display_On_Function();

    HX8352A_OrientationMode(ORIENTATION_PORTRAIT_REV);

    HX8352A_ScrollArea(0,480);

    return HX8352A_OK;
  }
  else {
    return HX8352A_ERROR;
  }
}

void HX8352A_OrientationMode(ORIENTATION_MODE orientation_mode) {

  wr_reg(MEMORY_ACCESS_CONTROL,orientation_mode);
  HX8352A_orientation_mode=orientation_mode;
}

void HX8352A_SetWindow(uint16_t x0,uint16_t y0,uint16_t x1,uint16_t y1) {

  switch(HX8352A_orientation_mode){
    case ORIENTATION_PORTRAIT:
    case ORIENTATION_PORTRAIT_REV:
      wr_reg(COLUMN_ADDRESS_START_H,x0>>8);
      wr_reg(COLUMN_ADDRESS_START_L,x0&0xff);
      wr_reg(COLUMN_ADDRESS_END_H,x1>>8);
      wr_reg(COLUMN_ADDRESS_END_L,x1&0xff);
      wr_reg(ROW_ADDRESS_START_H,y0>>8);
      wr_reg(ROW_ADDRESS_START_L,y0&0xff);
      wr_reg(ROW_ADDRESS_END_H,y1>>8);
      wr_reg(ROW_ADDRESS_END_L,y1&0xff);
      break;
    case ORIENTATION_LANDSCAPE:
    case ORIENTATION_LANDSCAPE_REV:
      wr_reg(COLUMN_ADDRESS_START_H,y0>>8);
      wr_reg(COLUMN_ADDRESS_START_L,y0&0xff);
      wr_reg(COLUMN_ADDRESS_END_H,y1>>8);
      wr_reg(COLUMN_ADDRESS_END_L,y1&0xff);
      wr_reg(ROW_ADDRESS_START_H,x0>>8);
      wr_reg(ROW_ADDRESS_START_L,x0&0xff);
      wr_reg(ROW_ADDRESS_END_H,x1>>8);
      wr_reg(ROW_ADDRESS_END_L,x1&0xff);
      break;
  }
  wr_cmd(MEMORY_WRITE);
}

void HX8352A_ClearScreen(uint32_t color) {

  uint32_t i;
  uint16_t dest1,dest2;

  HX8352A_SetWindow(0,0,HX8352A_GetWidth()-1,HX8352A_GetHeight()-1);

  switch(HX8352A_color_mode){
    case COLOR_16BIT:
      dest1=((color&0xf80000)>>8)|((color&0xfc00)>>5)|((color&0xf8)>>3);
      for(i=0;i<240*480;i++) {
        wr_dat(dest1);
      }
      break;
    case COLOR_18BIT:
      dest1=((color&0xf80000)>>8)|((color&0xfc00)>>5)|((color&0xf8)>>3);
      dest2=((color&0x0c)>>2)|0xAAAC;
      for(i=0;i<240*480;i++) {
        wr_dat(dest1);
        wr_dat(dest2);
      }
      break;
  }
}

void HX8352A_ScrollArea(uint16_t y,uint16_t pos) {

  uint16_t bfa;
  bfa=480-pos-y;
  wr_reg(VERTICAL_SCROLL_TOP_FIXED_AREA_H,y>>8);
  wr_reg(VERTICAL_SCROLL_TOP_FIXED_AREA_L,y&0xFF);
  wr_reg(VERTICAL_SCROLL_HEIGHT_H,pos>>8);
  wr_reg(VERTICAL_SCROLL_HEIGHT_L,pos&0xff);
  wr_reg(VERTICAL_SCROLL_BOTTOM_FIXED_AREA_H,bfa>>8);
  wr_reg(VERTICAL_SCROLL_BOTTOM_FIXED_AREA_L,bfa&0xFF);
}

void HX8352A_SetPixel(uint16_t x,uint16_t y,uint32_t color){

  uint16_t dest1,dest2;

  HX8352A_SetWindow(x,y,x,y);

  switch(HX8352A_color_mode){
    case COLOR_16BIT:
      dest1=(color&0xf80000)>>8|(color&0xfc00)>>5|(color&0xf8)>>3;
      wr_dat(dest1);
      break;
    case COLOR_18BIT:
      dest1=(color&0xf80000)>>8|(color&0xfc00)>>5|(color&0xf8)>>3;
      dest2=((color&0x0c)>>2)|0xAAAC;
      wr_dat(dest1);
      wr_dat(dest2);
      break;
  }
}

void HX8352A_FillPixel(uint16_t x0,uint16_t y0,uint16_t x1,uint16_t y1,uint32_t *color) {

  uint32_t i;
  uint16_t dest1,dest2;

  HX8352A_SetWindow(x0,y0,x1,y1);

  switch(HX8352A_color_mode){
    case COLOR_16BIT:
      for(i=0;i<(x1-x0+1)*(y1-y0+1);i++) {
        dest1=((color[i]&0xf80000)>>8)|((color[i]&0xfc00)>>5)|((color[i]&0xf8)>>3);
        wr_dat(dest1);
      }
      break;
    case COLOR_18BIT:
      for(i=0;i<(x1-x0+1)*(y1-y0+1);i++) {
        dest1=((color[i]&0xf80000)>>8)|((color[i]&0xfc00)>>5)|((color[i]&0xf8)>>3);
        dest2=((color[i]&0x0c)>>2)|0xAAAC;
        wr_dat(dest1);
        wr_dat(dest2);
      }
      break;
  }
}

void HX8352A_DrawRectangle(uint16_t x0,uint16_t y0,uint16_t x1,uint16_t y1,uint32_t color) {

  if(x0>x1) {
    uint16_t swap;
    swap=x0;
    x0=x1;
    x1=swap;
  }
  if(y0>y1) {
    uint16_t swap;
    swap=y0;
    y0=y1;
    y1=swap;
  }
  HX8352A_Fill(x0,y0,x0,y1,color);
  HX8352A_Fill(x1,y0,x1,y1,color);
  HX8352A_Fill(x0,y0,x1,y0,color);
  HX8352A_Fill(x0,y1,x1,y1,color);
}

void HX8352A_Fill(uint16_t x0,uint16_t y0,uint16_t x1,uint16_t y1,uint32_t color) {

  uint32_t i;
  uint16_t dest1,dest2;

  HX8352A_SetWindow(x0,y0,x1,y1);

  switch(HX8352A_color_mode){
    case COLOR_16BIT:
      dest1=((color&0xf80000)>>8)|((color&0xfc00)>>5)|((color&0xf8)>>3);
      for(i=0;i<(x1-x0+1)*(y1-y0+1);i++) {
        wr_dat(dest1);
      }
      break;
    case COLOR_18BIT:
      dest1=((color&0xf80000)>>8)|((color&0xfc00)>>5)|((color&0xf8)>>3);
      dest2=((color&0x0c)>>2)|0xAAAC;
      for(i=0;i<(x1-x0+1)*(y1-y0+1);i++) {
        wr_dat(dest1);
        wr_dat(dest2);
      }
      break;
  }
}

void HX8352A_StandBy(void) {

  Display_Off_Function();

  Power_Off_Function();

  // into STB mode
  wr_reg(POWER_CONTROL_1,0x0B);    // GASENB=0, PON=0, DK=0, XDK=0, VLCD_TRI=1, STB=1
  DWT_Delay(10000);

  // stop oscillation
  wr_reg(OSC_CONTROL_1,0x90);      // RADJ=1001, OSC_EN=0
}

void HX8352A_Wakeup(void) {

  // start oscillation
  wr_reg(OSC_CONTROL_1,0x91);      // RADJ=1001, OSC_EN=1
  DWT_Delay(10000);

  // exit STB mode
  wr_reg(POWER_CONTROL_1,0x0A);    // GASENB=0, PON=0, DK=0, XDK=0, VLCD_TRI=1, STB=0

  Power_Supply_Function();

  Display_On_Function();
}

uint16_t HX8352A_GetWidth(void) {

  uint16_t ret;

  switch(HX8352A_orientation_mode){
    case ORIENTATION_LANDSCAPE:
    case ORIENTATION_LANDSCAPE_REV:
      ret=480;
    break;
    case ORIENTATION_PORTRAIT:
    case ORIENTATION_PORTRAIT_REV:
      ret=240;
    break;
  }
  return ret;
}

uint16_t HX8352A_GetHeight(void) {

  uint16_t ret;

  switch(HX8352A_orientation_mode){
    case ORIENTATION_LANDSCAPE:
    case ORIENTATION_LANDSCAPE_REV:
      ret=240;
    break;
    case ORIENTATION_PORTRAIT:
    case ORIENTATION_PORTRAIT_REV:
      ret=480;
    break;
  }
  return ret;
}
