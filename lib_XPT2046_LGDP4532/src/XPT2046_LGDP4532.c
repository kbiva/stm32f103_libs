/*
 *  XPT2046_LGDP4532.c
 *
 *  Author: Kestutis Bivainis
 *
 *  library for the touchpanel with XPT2046 controller
 *  connected to the LCD with LGDP4532 controler.
 *
 *  Uses EXTI9_5_IRQHandler interrupt handler.
 *
 *  Tested with:
 *  2.4" TFT LCD Display Module, Model: HY-280_262k
 *  HY-STM32 board with STM32F103VET6
 *
 *  Based on work from Andy Brown.
 *  Converted from C++ to C with some modifications.
 */
/*
 * This file is a part of the open source stm32plus library.
 * Copyright (c) 2011,2012,2013,2014 Andy Brown <www.andybrown.me.uk>
 * Please see website for licensing terms.
 */

#include "XPT2046_LGDP4532.h"
#include "delay.h"
#include "LGDP4532.h"
#include "colors.h"

// XPT2046_clicked is set in the interrupt handler when touch is made
volatile uint8_t XPT2046_clicked;

static PIN pins[]={
  {{XPT2046_SCK_Pin,XPT2046_SCK_Speed,XPT2046_SCK_Mode},XPT2046_SCK_Port,XPT2046_SCK_Bus},
  {{XPT2046_MISO_Pin,XPT2046_MISO_Speed,XPT2046_MISO_Mode},XPT2046_MISO_Port,XPT2046_MISO_Bus},
  {{XPT2046_MOSI_Pin,XPT2046_MOSI_Speed,XPT2046_MOSI_Mode},XPT2046_MOSI_Port,XPT2046_MOSI_Bus},
  {{XPT2046_CS_Pin,XPT2046_CS_Speed,XPT2046_CS_Mode},XPT2046_CS_Port,XPT2046_CS_Bus},
  {{XPT2046_IRQ_Pin,XPT2046_IRQ_Speed,XPT2046_IRQ_Mode,},XPT2046_IRQ_Port,XPT2046_IRQ_Bus},
};

static void getSamples(uint16_t *xvalues,uint16_t *yvalues,uint16_t sampleCount);
static void getSamplesFor(uint8_t firstCommand,uint8_t lastCommand,uint16_t *values,uint16_t sampleCount);
static uint8_t getUserInput(
  int xpercent,int ypercent,uint16_t *panelPointX, uint16_t *panelPointY, uint16_t *touchPointX,uint16_t *touchPointY);
static void displayHitPoint(uint16_t ptX,uint16_t ptY);
static uint8_t getCoordinatesForCalibration(uint16_t *x, uint16_t *y);
static uint8_t postProcess(int16_t *x,int16_t *y,uint32_t sequenceNumber);
static uint16_t fastMedian(uint16_t *samples);
static void calculateCalibrationValues
  (uint16_t p1PanelX,uint16_t p1PanelY,uint16_t p2PanelX,uint16_t p2PanelY,uint16_t p3PanelX,uint16_t p3PanelY,
   uint16_t p1TouchX,uint16_t p1TouchY,uint16_t p2TouchX,uint16_t p2TouchY,uint16_t p3TouchX,uint16_t p3TouchY);
static uint16_t XPT2046_receive(void);
static uint8_t XPT2046_send(uint8_t cmd);
static void GPIO_Configuration(void);

static float _ax,_bx,_dx;
static float _ay,_by,_dy;

// set calbrated values
void XPT2046_LGDP4532_SetCalibrationValues(float ax,float bx,float dx,float ay,float by,float dy) {

  _ax=ax; _bx=bx; _dx=dx; _ay=ay; _by=by; _dy=dy;
}

// get calbrated values
void XPT2046_LGDP4532_GetCalibrationValues(float *ax,float *bx,float *dx,float *ay,float *by,float *dy) {

  *ax=_ax; *bx=_bx; *dx=_dx; *ay=_ay; *by=_by; *dy=_dy;
}

void EXTI9_5_IRQHandler(void) {

  if(EXTI_GetITStatus(XPT2046_EXTI_Line) != RESET) {
    XPT2046_clicked=1;
    EXTI_ClearITPendingBit(XPT2046_EXTI_Line);
  }
}

static void GPIO_Configuration(void) {

  uint32_t i;

  for(i=0;i<sizeof(pins)/sizeof(PIN);i++) {
    RCC_APB2PeriphClockCmd(pins[i].GPIO_Bus,ENABLE);
    GPIO_Init(pins[i].GPIOx,&pins[i].GPIO_InitStructure);
  }
}

// init TouchPanel
void XPT2046_LGDP4532_Init(void) {

  SPI_InitTypeDef SPI_InitStruct;
  NVIC_InitTypeDef NVIC_InitStructure;
  EXTI_InitTypeDef EXTI_InitStructure;

  GPIO_Configuration();

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);

  SPI_InitStruct.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
  SPI_InitStruct.SPI_Mode = SPI_Mode_Master;
  SPI_InitStruct.SPI_DataSize = SPI_DataSize_8b;
  SPI_InitStruct.SPI_CPOL = SPI_CPOL_Low;
  SPI_InitStruct.SPI_CPHA = SPI_CPHA_1Edge;
  SPI_InitStruct.SPI_NSS = SPI_NSS_Soft;
  SPI_InitStruct.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256;
  SPI_InitStruct.SPI_FirstBit = SPI_FirstBit_MSB;
  SPI_InitStruct.SPI_CRCPolynomial = 7;

  SPI_Init(SPI1, &SPI_InitStruct);
  SPI_Cmd(SPI1, ENABLE);

  RCC_APB2PeriphClockCmd(XPT2046_IRQ_AFIO_Bus,ENABLE);
  GPIO_EXTILineConfig(XPT2046_IRQ_Portsource, XPT2046_IRQ_PinSource);
  EXTI_InitStructure.EXTI_Line = XPT2046_EXTI_Line;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);
  RCC_APB2PeriphClockCmd(XPT2046_IRQ_AFIO_Bus,DISABLE);

  NVIC_InitStructure.NVIC_IRQChannel = XPT2046_EXTI_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0F;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  GPIO_SetBits(XPT2046_CS_Port,XPT2046_CS_Pin);
}

static uint8_t XPT2046_send(uint8_t cmd) {

  while (!(SPI1->SR & SPI_I2S_FLAG_TXE)){}; //while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
  SPI1->DR=cmd;                             //SPI_I2S_SendData(SPI1,cmd);
  while (!(SPI1->SR & SPI_I2S_FLAG_RXNE)){};//while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET);
  return SPI1->DR;                          //return SPI_I2S_ReceiveData(SPI1);
}

static uint16_t XPT2046_receive(void) {

  uint16_t buf;
  uint8_t temp;

  temp=XPT2046_send(0x00);
  buf=(temp&0x7F)<<5;
  temp=XPT2046_send(0x00);
  buf|=temp>>3;
  return buf;
}


// Calculates the coeffient values used in the co-ordinate mapping.
static void calculateCalibrationValues
  (uint16_t p1PanelX,uint16_t p1PanelY,uint16_t p2PanelX,uint16_t p2PanelY,uint16_t p3PanelX,uint16_t p3PanelY,
   uint16_t p1TouchX,uint16_t p1TouchY,uint16_t p2TouchX,uint16_t p2TouchY,uint16_t p3TouchX,uint16_t p3TouchY) {

  int32_t delta,deltaX1,deltaX2,deltaX3,deltaY1,deltaY2,deltaY3;

  // intermediate values for the calculation
  delta=((int32_t)(p1TouchX-p3TouchX)*(int32_t)(p2TouchY-p3TouchY))-((int32_t)(p2TouchX-p3TouchX)*(int32_t)(p1TouchY-p3TouchY));

  deltaX1=((int32_t)(p1PanelX-p3PanelX)*(int32_t)(p2TouchY-p3TouchY))-((int32_t)(p2PanelX-p3PanelX)*(int32_t)(p1TouchY-p3TouchY));
  deltaX2=((int32_t)(p1TouchX-p3TouchX)*(int32_t)(p2PanelX-p3PanelX))-((int32_t)(p2TouchX-p3TouchX)*(int32_t)(p1PanelX-p3PanelX));
  deltaX3=p1PanelX*((int32_t)p2TouchX*(int32_t)p3TouchY - (int32_t)p3TouchX*(int32_t)p2TouchY) -
          p2PanelX*((int32_t)p1TouchX*(int32_t)p3TouchY - (int32_t)p3TouchX*(int32_t)p1TouchY) +
          p3PanelX*((int32_t)p1TouchX*(int32_t)p2TouchY - (int32_t)p2TouchX*(int32_t)p1TouchY);

  deltaY1=((int32_t)(p1PanelY-p3PanelY)*(int32_t)(p2TouchY-p3TouchY))-((int32_t)(p2PanelY-p3PanelY)*(int32_t)(p1TouchY-p3TouchY));
  deltaY2=((int32_t)(p1TouchX-p3TouchX)*(int32_t)(p2PanelY-p3PanelY))-((int32_t)(p2TouchX-p3TouchX)*(int32_t)(p1PanelY-p3PanelY));
  deltaY3=p1PanelY*((int32_t)p2TouchX*(int32_t)p3TouchY - (int32_t)p3TouchX*(int32_t)p2TouchY) -
          p2PanelY*((int32_t)p1TouchX*(int32_t)p3TouchY - (int32_t)p3TouchX*(int32_t)p1TouchY) +
          p3PanelY*((int32_t)p1TouchX*(int32_t)p2TouchY - (int32_t)p2TouchX*(int32_t)p1TouchY);

  // final values
  _ax=(float)deltaX1/(float)delta;
  _bx=(float)deltaX2/(float)delta;
  _dx=(float)deltaX3/(float)delta;

  _ay=(float)deltaY1/(float)delta;
  _by=(float)deltaY2/(float)delta;
  _dy=(float)deltaY3/(float)delta;
}


// Translate co-ordinates from raw to display.
static void translate(int16_t rawX,int16_t rawY,int16_t *dispX,int16_t *dispY) {

  *dispX=_ax*rawX+_bx*rawY+_dx;
  *dispY=_ay*rawX+_by*rawY+_dy;
}

static void getSamplesFor(uint8_t firstCommand,uint8_t lastCommand,uint16_t *values,uint16_t sampleCount) {

  XPT2046_send(firstCommand);
  // ignore first sample read because of worst quality
  XPT2046_send(0x00);
  XPT2046_send(0x00);

  while(sampleCount--!=1) {

    XPT2046_send(firstCommand);
    // add the 12 bit response. The MSB of the first response is discarded
    // because it's a NUL bit output during the BUSY line period
    *values++=XPT2046_receive();
  }

  // last sample
  XPT2046_send(lastCommand);
  *values=XPT2046_receive();
}

static void getSamples(uint16_t *xvalues,uint16_t *yvalues,uint16_t sampleCount) {

  // mask all interrupts while we're doing the sampling
  __disable_irq();

  // set CS = low
  GPIO_ResetBits(XPT2046_CS_Port,XPT2046_CS_Pin);

  // get the X values and don't power down afterwards
  getSamplesFor(CHX|START|PD0|PD1,CHX|START|PD0|PD1,xvalues,sampleCount);

  // get the Y samples and power down afterwards
  getSamplesFor(CHY|START|PD0|PD1,CHY|START,yvalues,sampleCount);

  // set CS = high
  GPIO_SetBits(XPT2046_CS_Port,XPT2046_CS_Pin);

  // if PENIRQ went low during the sampling then the EXTI interrupt will be pending so
  // we clear the pending bit now before re-enabling interrupts
  EXTI_ClearITPendingBit(EXTI9_5_IRQn);
  __enable_irq();
}

static uint16_t fastMedian(uint16_t *samples) {

  // do a fast median selection (reference code available on internet). This code basically
  // avoids sorting the entire samples array

  #define PIX_SORT(a,b) { if ((a)>(b)) PIX_SWAP((a),(b)); }
  #define PIX_SWAP(a,b) { uint16_t temp=(a);(a)=(b);(b)=temp; }

  PIX_SORT(samples[0],samples[5]);
  PIX_SORT(samples[0],samples[3]);
  PIX_SORT(samples[1],samples[6]);
  PIX_SORT(samples[2],samples[4]);
  PIX_SORT(samples[0],samples[1]);
  PIX_SORT(samples[3],samples[5]);
  PIX_SORT(samples[2],samples[6]);
  PIX_SORT(samples[2],samples[3]);
  PIX_SORT(samples[3],samples[6]);
  PIX_SORT(samples[4],samples[5]);
  PIX_SORT(samples[1],samples[4]);
  PIX_SORT(samples[1],samples[3]);
  PIX_SORT(samples[3],samples[4]);

  return samples[3];
}

static uint8_t postProcess(int16_t *x,int16_t *y,uint32_t sequenceNumber) {

  static int16_t _samplesRequired=4;
  static int32_t _xTotal,_yTotal;

  if(sequenceNumber==0) {
    _xTotal=*x;
    _yTotal=*y;
  }
  else {
    _xTotal+=*x;
    _yTotal+=*y;
  }

  if(sequenceNumber==_samplesRequired-1) {

    *x=_xTotal/_samplesRequired;
    *y=_yTotal/_samplesRequired;

    return 1;
  }

  return 0;
}

uint8_t XPT2046_LGDP4532_GetCoordinates(int16_t *x, int16_t *y) {

  uint16_t xvalues[7],yvalues[7];
  int16_t sequenceNumber = 0;
  uint16_t rawX,rawY;

  do {
    // get the samples
    getSamples(xvalues,yvalues,7);

    // the panel must have been touched all this time
    if(GPIO_ReadInputDataBit(XPT2046_IRQ_Port,XPT2046_IRQ_Pin))
      return 0;

    // get the median values
    rawX=fastMedian(xvalues);
    rawY=fastMedian(yvalues);

    // translate the co-ordinates using the calibration routine
    translate(rawX,rawY,x,y);

    // pass the co-ordinate to the post processor
    // keep going until the post processor signals it's finished
   } while(postProcess(x,y,sequenceNumber++));

   return 1;
}

static uint8_t getCoordinatesForCalibration(uint16_t *x, uint16_t *y) {

  uint16_t xvalues[7],yvalues[7];

  // get the samples
  getSamples(xvalues,yvalues,7);

  // the panel must have been touched all this time
  if(GPIO_ReadInputDataBit(XPT2046_IRQ_Port,XPT2046_IRQ_Pin))
    return 0;

  // get the median values
  *x=fastMedian(xvalues);
  *y=fastMedian(yvalues);

  return 1;
}

static void displayHitPoint(uint16_t ptX,uint16_t ptY) {

  int16_t i,j,x,y;

  x=ptX-1;
  y=ptY-1;

  for(i=0;i<3;i++)
    for(j=0;j<3;j++)
      LGDP4532_SetPixel(x+j,y+i,RED);
}


// Display a point for the user to click on and wait for that to happen
static uint8_t getUserInput(
  int xpercent,int ypercent,uint16_t *panelPointX, uint16_t *panelPointY, uint16_t *touchPointX,uint16_t *touchPointY) {

  uint16_t pX,pY;
  int32_t xsum,ysum;
  int i;

  // convert the percent co-ords into actual panel locations
  *panelPointX=(LGDP4532_GetWidth()*xpercent)/100;
  *panelPointY=(LGDP4532_GetHeight()*ypercent)/100;

  // display the hit-point for the user
  LGDP4532_ClearScreen(BLACK);
  LGDP4532_PutStrCEOL("Tap and hold the red point",44,10);
  displayHitPoint(*panelPointX,*panelPointY);

  // wait for a click to happen in the interrupt handler
  for(XPT2046_clicked=0;!XPT2046_clicked;){};

  // get sampling 50 points
  xsum=0;
  ysum=0;

  for(i=0;i<50;i++) {

    if(!getCoordinatesForCalibration(&pX,&pY)) {
      return 0;
    }
    xsum+=pX;
    ysum+=pY;
  }

  // remove the hit point
  LGDP4532_PutStrCEOL("Stop pressing the screen",44,10);

  *touchPointX=xsum/50;
  *touchPointY=ysum/50;

  // wait to allow for debouncing
  DWT_Delay(3000000);

  return 1;
}

// Interactively calibrate the touch screen with the user.
// This routine will present a UI to the user for calibration
// and block until it's done.
uint8_t XPT2046_LGDP4532_Calibrate(void) {

  uint16_t p1PanelX,p2PanelX,p3PanelX,p1TouchX,p2TouchX,p3TouchX,
           p1PanelY,p2PanelY,p3PanelY,p1TouchY,p2TouchY,p3TouchY;

  // point 1 is at 25%,50%, 2 is at 75%,25% and 3 is at 75%,75%
  if(!getUserInput(25,50,&p1PanelX,&p1PanelY,&p1TouchX,&p1TouchY)) {
    return 0;
  }
  if(!getUserInput(75,25,&p2PanelX,&p2PanelY,&p2TouchX,&p2TouchY)){
    return 0;
  }
  if(!getUserInput(75,75,&p3PanelX,&p3PanelY,&p3TouchX,&p3TouchY)){
    return 0;
  }

  calculateCalibrationValues(p1PanelX,p1PanelY,p2PanelX,p2PanelY,p3PanelX,p3PanelY,
                             p1TouchX,p1TouchY,p2TouchX,p2TouchY,p3TouchX,p3TouchY);
  return 1;
}
