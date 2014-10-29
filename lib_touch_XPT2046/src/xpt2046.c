/*
 *  usart.h
 *
 *  Author: Kestutis Bivainis
 *
 */

#include "stm32f10x_conf.h"
#include "xpt2046.h"
#include "delay.h"

void Touch_Init(void) {

  SPI_InitTypeDef  SPI_InitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;

  /* Enable SPI1 GPIOA clocks */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1|RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB|RCC_APB2Periph_AFIO,ENABLE);

  SPI_Cmd(SPI1, DISABLE);

  /* Configure SPI1 pins: SCK, MISO and MOSI */
  GPIO_InitStructure.GPIO_Pin = SPI1_SCK | SPI1_MISO | SPI1_MOSI;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  /* Configure PB7 as Output push-pull, used as Touch Chip select */
  GPIO_InitStructure.GPIO_Pin = SPI1_TOUCH_CS;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  //PENIRQ
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_6;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  GPIOB->BSRR=SPI1_TOUCH_CS;

  /* SPI1 configuration */
  SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
  SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
  SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
  SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
  SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
  SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_16;
  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
  SPI_InitStructure.SPI_CRCPolynomial = 7;
  SPI_Init(SPI1, &SPI_InitStructure);

  /* Enable SPI1  */
  SPI_Cmd(SPI1, ENABLE);
}

uint16_t Touch_Read(uint8_t cmd) {

  uint16_t a;

  GPIOB->BRR=SPI1_TOUCH_CS;
  //DWT_Delay(1);
  while (!(SPI1->SR & SPI_I2S_FLAG_TXE)){};
  SPI1->DR=cmd;
  while (!(SPI1->SR & SPI_I2S_FLAG_RXNE)){};
  a=SPI1->DR;
  //DWT_Delay(1);
  while (!(SPI1->SR & SPI_I2S_FLAG_TXE)){};
  SPI1->DR=0;
  while (!(SPI1->SR & SPI_I2S_FLAG_RXNE)){};
  a=SPI1->DR;
  a=a<<8;
  while (!(SPI1->SR & SPI_I2S_FLAG_TXE)){};
  SPI1->DR=0;
  while (!(SPI1->SR & SPI_I2S_FLAG_RXNE)){};
  a|=SPI1->DR;
  //DWT_Delay(1);
  GPIOB->BSRR=SPI1_TOUCH_CS;
  return a>>3;
}

