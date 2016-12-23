/*
 *  XPT2046_LGDP4532.h
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

#ifndef _XPT2046_LGDP4532_H_
#define _XPT2046_LGDP4532_H_

#include "stm32f10x_conf.h"

// XPT2046 on SPI1

#define XPT2046_SCK_Pin        GPIO_Pin_5
#define XPT2046_SCK_Speed      GPIO_Speed_50MHz
#define XPT2046_SCK_Mode       GPIO_Mode_AF_PP
#define XPT2046_SCK_Port       GPIOA
#define XPT2046_SCK_Bus        RCC_APB2Periph_GPIOA

#define XPT2046_MISO_Pin       GPIO_Pin_6
#define XPT2046_MISO_Speed     GPIO_Speed_50MHz
#define XPT2046_MISO_Mode      GPIO_Mode_AF_PP
#define XPT2046_MISO_Port      GPIOA
#define XPT2046_MISO_Bus       RCC_APB2Periph_GPIOA

#define XPT2046_MOSI_Pin       GPIO_Pin_7
#define XPT2046_MOSI_Speed     GPIO_Speed_50MHz
#define XPT2046_MOSI_Mode      GPIO_Mode_AF_PP
#define XPT2046_MOSI_Port      GPIOA
#define XPT2046_MOSI_Bus       RCC_APB2Periph_GPIOA

#define XPT2046_CS_Pin         GPIO_Pin_7
#define XPT2046_CS_Speed       GPIO_Speed_50MHz
#define XPT2046_CS_Mode        GPIO_Mode_Out_PP
#define XPT2046_CS_Port        GPIOB
#define XPT2046_CS_Bus         RCC_APB2Periph_GPIOB

#define XPT2046_IRQ_Pin        GPIO_Pin_6
#define XPT2046_IRQ_Speed      GPIO_Speed_50MHz
#define XPT2046_IRQ_Mode       GPIO_Mode_IPD
#define XPT2046_IRQ_Port       GPIOB
#define XPT2046_IRQ_Bus        RCC_APB2Periph_GPIOB
#define XPT2046_IRQ_AFIO_Bus   RCC_APB2Periph_AFIO
#define XPT2046_IRQ_PinSource  GPIO_PinSource6
#define XPT2046_IRQ_Portsource GPIO_PortSourceGPIOB
#define XPT2046_EXTI_Line      EXTI_Line6
#define XPT2046_EXTI_IRQn      EXTI9_5_IRQn

typedef struct _XPT2046_PIN {
  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_TypeDef* GPIOx;
  uint32_t GPIO_Bus;
} XPT2046_PIN;

enum ControlBits {
  START = 0x80,
  A2    = 0x40,
  A1    = 0x20,
  A0    = 0x10,
  MODE1 = 0x08,
  MODE0 = 0x04,
  PD1   = 0x02,
  PD0   = 0x01
};

#define  CHX A0
#define  CHY A0|A2

void XPT2046_LGDP4532_Init(void);
uint8_t XPT2046_LGDP4532_Calibrate(void);
void XPT2046_LGDP4532_SetCalibrationValues(float ax,float bx,float dx,float ay,float by,float dy);
void XPT2046_LGDP4532_GetCalibrationValues(float *ax,float *bx,float *dx,float *ay,float *by,float *dy);
uint8_t XPT2046_LGDP4532_GetCoordinates(int16_t *x, int16_t *y);

#endif
