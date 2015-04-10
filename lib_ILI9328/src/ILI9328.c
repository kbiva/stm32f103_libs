/*
 *  ILI9328.c
 *
 *  Author: Kestutis Bivainis
 *
 */

#include "stm32f10x_conf.h"
#include "ILI9328.h"
#include "delay.h"

static COLOR_MODE ILI9328_color_mode;
static ORIENTATION_MODE ILI9328_orientation_mode;

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


#define TFTLCD_DELAY 0xFF

static const uint16_t ILI9328_regValues[]  = {

  ILI9328_DRIVER_OUTPUT_CTRL,    0x0100,
  ILI9328_LCD_DRIVING_CTRL,      0x0700, // set 1 line inversion
  ILI9328_ENTRY_MODE,            0x1000, // set GRAM write direction and BGR=1
  ILI9328_RESIZE_CTRL,           0x0000, // Resize register
  ILI9328_DISPLAY_CTRL2,         0x0202, // set the back porch and front porch
  ILI9328_DISPLAY_CTRL3,         0x0000, // set non-display area refresh cycle ISC[3:0]
  ILI9328_DISPLAY_CTRL4,         0x0000, // FMARK function
  ILI9328_RGB_DISPLAY_IF_CTRL1,  0x0000, // RGB interface setting
  ILI9328_FRAME_MARKER_POS,      0x0000, // Frame marker Position
  ILI9328_RGB_DISPLAY_IF_CTRL2,  0x0000, // RGB interface polarity
  // *************Power On sequence ****************
  ILI9328_POWER_CTRL1,           0x0000, // SAP, BT[3:0], AP, DSTB, SLP, STB
  ILI9328_POWER_CTRL2,           0x0007, // DC1[2:0], DC0[2:0], VC[2:0]
  ILI9328_POWER_CTRL3,           0x0000, // VREG1OUT voltage
  ILI9328_POWER_CTRL4,           0x0000, // VDV[4:0] for VCOM amplitude
  ILI9328_DISPLAY_CTRL1,         0x0001,
  TFTLCD_DELAY,200,
  ILI9328_POWER_CTRL1,           0x1290,
  ILI9328_POWER_CTRL2,           0x0227,
  TFTLCD_DELAY,50,
  ILI9328_POWER_CTRL3,           0x001b, // External reference voltage= Vci;
  TFTLCD_DELAY,50,
  ILI9328_POWER_CTRL4,           0x1900,
  ILI9328_POWER_CTRL7,           0x000f,
  ILI9328_FRAME_RATE_COLOR_CTRL, 0x000c, // Set Frame Rate
  TFTLCD_DELAY,50,
  ILI9328_GRAM_HORZ_AD,          0x0000, // GRAM horizontal Address
  ILI9328_GRAM_VERT_AD,          0x0000, // GRAM Vertical Address
    // ----------- Adjust the Gamma Curve ----------//
    //ILI932X_GAMMA_CTRL1,0x0000,
    //ILI932X_GAMMA_CTRL2,0x0201,
    //ILI932X_GAMMA_CTRL3,0x0003,
    //ILI932X_GAMMA_CTRL4,0x0305,
    //ILI932X_GAMMA_CTRL5,0x0004,
    //ILI932X_GAMMA_CTRL6,0x0407,
    //ILI932X_GAMMA_CTRL7,0x0605,
    //ILI932X_GAMMA_CTRL8,0x0707,
    //ILI932X_GAMMA_CTRL9,0x0503,
    //ILI932X_GAMMA_CTRL10,0x0004,
    //------------------ Set GRAM area ---------------//
  ILI9328_HORZ_START_AD,         0x0000, // Horizontal GRAM Start Address
  ILI9328_HORZ_END_AD,           0x00EF, // Horizontal GRAM End Address
  ILI9328_VERT_START_AD,         0x0000, // Vertical GRAM Start Address
  ILI9328_VERT_END_AD,           0x013F, // Vertical GRAM End Address
  ILI9328_GATE_SCAN_CTRL1,       0xA700, // Gate Scan Line
  ILI9328_GATE_SCAN_CTRL2,       0x0001, // NDL,VLE, REV
  ILI9328_GATE_SCAN_CTRL3,       0x0000, // set scrolling line
  //-------------- Partial Display Control ---------//
  ILI9328_PART_IMG1_DISP_POS,    0x0000,
  ILI9328_PART_IMG1_START_AD,    0x0000,
  ILI9328_PART_IMG1_END_AD,      0x0000,
  ILI9328_PART_IMG2_DISP_POS,    0x0000,
  ILI9328_PART_IMG2_START_AD,    0x0000,
  ILI9328_PART_IMG2_END_AD,      0x0000,
  //-------------- Panel Control -------------------//
  ILI9328_PANEL_IF_CTRL1,        0x0010,
  ILI9328_PANEL_IF_CTRL2,        0x0600,
  ILI9328_DISPLAY_CTRL1,         0x0133, // Display ON

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

  FSMC_NORSRAMTimingInitStructure.FSMC_AddressSetupTime = 0x01;
  FSMC_NORSRAMTimingInitStructure.FSMC_AddressHoldTime = 0x00;
  FSMC_NORSRAMTimingInitStructure.FSMC_DataSetupTime = 0x02;
  FSMC_NORSRAMTimingInitStructure.FSMC_BusTurnAroundDuration = 0x00;
  FSMC_NORSRAMTimingInitStructure.FSMC_CLKDivision = 0x00;
  FSMC_NORSRAMTimingInitStructure.FSMC_DataLatency = 0x00;
  FSMC_NORSRAMTimingInitStructure.FSMC_AccessMode = FSMC_AccessMode_B;

  FSMC_NORSRAMInitStructure.FSMC_Bank = FSMC_Bank1_NORSRAM1;
  FSMC_NORSRAMInitStructure.FSMC_DataAddressMux = FSMC_DataAddressMux_Disable;
  FSMC_NORSRAMInitStructure.FSMC_MemoryType = FSMC_MemoryType_SRAM;
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
  FSMC_NORSRAMInitStructure.FSMC_AsynchronousWait=FSMC_AsynchronousWait_Disable;
  FSMC_NORSRAMInitStructure.FSMC_ReadWriteTimingStruct = &FSMC_NORSRAMTimingInitStructure;
  FSMC_NORSRAMInitStructure.FSMC_WriteTimingStruct = &FSMC_NORSRAMTimingInitStructure;

  FSMC_NORSRAMInit(&FSMC_NORSRAMInitStructure);

  FSMC_NORSRAMCmd(FSMC_Bank1_NORSRAM1, ENABLE);
}

static void lcd_rst(void) {

  GPIO_ResetBits(RST_Port, RST_Pin);
  DWT_Delay(10000);
  GPIO_SetBits(RST_Port, RST_Pin);
  DWT_Delay(50000);
}

static __forceinline
void wr_cmd(uint8_t index) {
  LCD_REG8 = 0;
  LCD_REG8 = index;
}

static __forceinline
void wr_dat(uint8_t val) {
  LCD_DAT8 = val;
}

static __forceinline
void wr_reg(uint8_t index,uint16_t val) {
  wr_cmd(index);
  wr_dat(val>>8);
  wr_dat(val);
}

static __forceinline
uint8_t rd_reg(uint8_t index) {
  wr_cmd(index);
  return (LCD_DAT8);
}

static __forceinline
uint8_t rd_dat(void) {
  return (LCD_DAT8);
}


uint8_t ILI9328_Init(void) {

  volatile uint8_t id1=0,id2=0;

  GPIO_Configuration();
  FSMC_LCD_Init();

  lcd_rst();

  id1=rd_reg(0x00); // 93
  id2=rd_dat();     // 28

  if((id1==0x93) && (id2==0x28)) {

    uint8_t i,a;
    uint16_t d;

    while(i < sizeof(ILI9328_regValues) / sizeof(uint16_t)) {
      a = ILI9328_regValues[i++];
      d = ILI9328_regValues[i++];
      if(a == TFTLCD_DELAY)
        DWT_Delay(d*1000);
      else {
        wr_reg(a,d);
      }
    }

    ILI9328_color_mode=COLOR_16BIT;
    ILI9328_orientation_mode=ORIENTATION_PORTRAIT;

    return ILI9328_OK;
  }

  return ILI9328_ERROR;
}

void ILI9328_ColorMode(COLOR_MODE color_mode) {

  wr_reg(ILI9328_ENTRY_MODE,color_mode|ILI9328_orientation_mode);
  ILI9328_color_mode=color_mode;

}

void ILI9328_OrientationMode(ORIENTATION_MODE orientation_mode) {

  wr_reg(ILI9328_ENTRY_MODE,orientation_mode|ILI9328_color_mode);
  ILI9328_orientation_mode=orientation_mode;
}

void ILI9328_ClearScreen(uint32_t color) {

  ILI9328_Fill(0,0,ILI9328_GetWidth()-1,ILI9328_GetHeight()-1,color);
}

void ILI9328_Fill(uint16_t x0,uint16_t y0,uint16_t x1,uint16_t y1,uint32_t color) {

  uint32_t i,j=(x1-x0+1)*(y1-y0+1);
  uint8_t b1,b2,r,g,b;

  ILI9328_SetWindow(x0,y0,x1,y1);

  r=color>>16;
  g=color>>8;
  b=color;

  switch(ILI9328_color_mode) {

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

  }
}

void ILI9328_SetWindow(uint16_t x0,uint16_t y0,uint16_t x1,uint16_t y1) {

  switch(ILI9328_orientation_mode){
    case ORIENTATION_LANDSCAPE:
      wr_reg(ILI9328_HORZ_START_AD,239-y1);
      wr_reg(ILI9328_HORZ_END_AD,239-y0);
      wr_reg(ILI9328_VERT_START_AD,x0);
      wr_reg(ILI9328_VERT_END_AD,x1);
      wr_reg(ILI9328_GRAM_HORZ_AD,239-y0);
      wr_reg(ILI9328_GRAM_VERT_AD,x0);
    break;
    case ORIENTATION_LANDSCAPE_REV:
      wr_reg(ILI9328_HORZ_START_AD,y0);
      wr_reg(ILI9328_HORZ_END_AD,y1);
      wr_reg(ILI9328_VERT_START_AD,319-x1);
      wr_reg(ILI9328_VERT_END_AD,319-x0);
      wr_reg(ILI9328_GRAM_HORZ_AD,y0);
      wr_reg(ILI9328_GRAM_VERT_AD,319-x0);
    break;
    case ORIENTATION_PORTRAIT:
      wr_reg(ILI9328_HORZ_START_AD,239-x1);
      wr_reg(ILI9328_HORZ_END_AD,239-x0);
      wr_reg(ILI9328_VERT_START_AD,319-y1);
      wr_reg(ILI9328_VERT_END_AD,319-y0);
      wr_reg(ILI9328_GRAM_HORZ_AD,239-x0);
      wr_reg(ILI9328_GRAM_VERT_AD,319-y0);
    break;
    case ORIENTATION_PORTRAIT_REV:
      wr_reg(ILI9328_HORZ_START_AD,x0);
      wr_reg(ILI9328_HORZ_END_AD,x1);
      wr_reg(ILI9328_VERT_START_AD,y0);
      wr_reg(ILI9328_VERT_END_AD,y1);
      wr_reg(ILI9328_GRAM_HORZ_AD,x0);
      wr_reg(ILI9328_GRAM_VERT_AD,y0);
    break;
  }
  wr_cmd(ILI9328_RW_GRAM);
}

void ILI9328_FillPixel(uint16_t x0,uint16_t y0,uint16_t x1,uint16_t y1,uint32_t *color) {

  uint32_t i,j=(x1-x0+1)*(y1-y0+1);

  ILI9328_SetWindow(x0,y0,x1,y1);

  switch(ILI9328_color_mode) {
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
  }
}

void ILI9328_SetPixel(uint16_t x, uint16_t y, uint32_t color)
{
  uint8_t r,g,b;

  switch(ILI9328_orientation_mode){
    case ORIENTATION_LANDSCAPE:
      wr_reg(ILI9328_GRAM_HORZ_AD,239-y);
      wr_reg(ILI9328_GRAM_VERT_AD,x);
    break;
    case ORIENTATION_LANDSCAPE_REV:
      wr_reg(ILI9328_GRAM_HORZ_AD,y);
      wr_reg(ILI9328_GRAM_VERT_AD,319-x);
      break;
    case ORIENTATION_PORTRAIT:
      wr_reg(ILI9328_GRAM_HORZ_AD,239-x);
      wr_reg(ILI9328_GRAM_VERT_AD,319-y);
    break;
    case ORIENTATION_PORTRAIT_REV:
      wr_reg(ILI9328_GRAM_HORZ_AD,x);
      wr_reg(ILI9328_GRAM_VERT_AD,y);
    break;
  }

  wr_cmd(ILI9328_RW_GRAM);

  r=color>>16;
  g=color>>8;
  b=color;

  switch(ILI9328_color_mode) {
    case COLOR_16BIT:
      wr_dat((r&0xF8)|((g>>5)&0x07));
      wr_dat(((g<<3)&0x0E)|((b>>3)&0x1F));
      break;
    case COLOR_18BIT:
      wr_dat(r&0xFC);
      wr_dat(g&0xFC);
      wr_dat(b&0xFC);
      break;
  }
}

void ILI9328_Sleep(void) {

  wr_reg(ILI9328_DISPLAY_CTRL1, 0x0131); // Set D1=0, D0=1
  DWT_Delay(10000);
  wr_reg(ILI9328_DISPLAY_CTRL1, 0x0130); // Set D1=0, D0=0
  DWT_Delay(10000);
  wr_reg(ILI9328_DISPLAY_CTRL1, 0x0000); // display OFF
  // ************* Power OFF sequence **************//
  wr_reg(ILI9328_POWER_CTRL1, 0x0080);   // SAP, BT[3:0], APE, AP, DSTB, SLP
  wr_reg(ILI9328_POWER_CTRL2, 0x0000);   // DC1[2:0], DC0[2:0], VC[2:0]
  wr_reg(ILI9328_POWER_CTRL3, 0x0000);   // VREG1OUT voltage
  wr_reg(ILI9328_POWER_CTRL4, 0x0000);   // VDV[4:0] for VCOM amplitude
  DWT_Delay(200000);                     // Dis-charge capacitor power voltage
  wr_reg(ILI9328_POWER_CTRL1, 0x0082);   // SAP, BT[3:0], APE, AP, DSTB, SLP
}

void ILI9328_Wakeup(void) {

  //*************Power On sequence ******************//
  wr_reg(ILI9328_POWER_CTRL1, 0x0080);   // SAP, BT[3:0], AP, DSTB, SLP
  wr_reg(ILI9328_POWER_CTRL2, 0x0000);   // DC1[2:0], DC0[2:0], VC[2:0]
  wr_reg(ILI9328_POWER_CTRL3, 0x0000);   // VREG1OUT voltage
  wr_reg(ILI9328_POWER_CTRL4, 0x0000);   // VDV[4:0] for VCOM amplitude
  wr_reg(ILI9328_DISPLAY_CTRL1, 0x0001);
  DWT_Delay(200000);                     // Dis-charge capacitor power voltage
  wr_reg(ILI9328_POWER_CTRL1, 0x1290);   // SAP, BT[3:0], AP, DSTB, SLP, STB
  wr_reg(ILI9328_POWER_CTRL2, 0x0227);   // DC1[2:0], DC0[2:0], VC[2:0]
  DWT_Delay(50000);
  wr_reg(ILI9328_POWER_CTRL3, 0x009D);   // External reference voltage =Vci;
  DWT_Delay(50000);
  wr_reg(ILI9328_POWER_CTRL4, 0x1A00);   // VDV[4:0] for VCOM amplitude
  wr_reg(ILI9328_POWER_CTRL7, 0x001D);   // VCM[5:0] for VCOMH
  DWT_Delay(50000);
  wr_reg(ILI9328_DISPLAY_CTRL1, 0x0133); // display ON

}

uint16_t ILI9328_GetWidth(void) {

  uint16_t ret;

  switch(ILI9328_orientation_mode){
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

uint16_t ILI9328_GetHeight(void) {

  uint16_t ret;

  switch(ILI9328_orientation_mode){
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
