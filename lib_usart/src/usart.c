/*
 *  usart.h
 *
 *  Author: Kestutis Bivainis
 *
 */

#include <stdio.h>
#include "stm32f10x_conf.h"
#include "usart.h"

struct __FILE {int handle;};
FILE __stdout;
FILE __stdin;

void USART1_Init(uint32_t baud) {
  GPIO_InitTypeDef GPIO_InitStructure;
  USART_InitTypeDef USART_InitStructure;

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_AFIO |
                         USART1_PIN_TX_Bus | USART1_PIN_RX_Bus, ENABLE );

  //Set USART1 Tx (PA.09) as AF push-pull
  GPIO_InitStructure.GPIO_Pin = USART1_PIN_TX;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(USART1_PIN_TX_Port, &GPIO_InitStructure);

  //Set USART1 Rx (PA.10) as input floating
  GPIO_InitStructure.GPIO_Pin = USART1_PIN_RX;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(USART1_PIN_RX_Port, &GPIO_InitStructure);

  USART_InitStructure.USART_BaudRate = baud;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;

  USART_Init(USART1, &USART_InitStructure);

  //Enable USART1
  USART_ClearFlag(USART1,USART_FLAG_TC);
  USART_Cmd(USART1, ENABLE);
}

void USART2_Init(uint32_t baud) {
  GPIO_InitTypeDef GPIO_InitStructure;
  USART_InitTypeDef USART_InitStructure;

  RCC_APB2PeriphClockCmd(USART2_PIN_TX_Bus | USART2_PIN_RX_Bus , ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

  //Set USART2 Tx (PA.02) as AF push-pull
  GPIO_InitStructure.GPIO_Pin = USART2_PIN_TX;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(USART2_PIN_TX_Port, &GPIO_InitStructure);

  //Set USART2 Rx (PA.03) as input floating
  GPIO_InitStructure.GPIO_Pin = USART2_PIN_RX;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(USART2_PIN_RX_Port, &GPIO_InitStructure);

  USART_InitStructure.USART_BaudRate = baud;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;

  USART_Init(USART2, &USART_InitStructure);

  //Enable USART2
  USART_ClearFlag(USART2,USART_FLAG_TC);
  USART_Cmd(USART2, ENABLE);
}

int SendChar (int ch) {
  while (!(USART2->SR & USART_FLAG_TXE)){};
  USART2->DR = (ch & 0xFF);
  return (ch);
}

int GetChar (void) {
  while (!(USART2->SR & USART_FLAG_RXNE)){};
  return ((int)(USART2->DR & 0xFF));
}

int fputc(int ch, FILE *f)
{
  //USART_SendData(USART2, (uint8_t) ch);
  USART2->DR = (ch & (uint16_t)0x01FF);
  //while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
  while (!(USART2->SR & USART_FLAG_TXE)){};
  return ch;
}

int fgetc(FILE *f) {
  while (!(USART2->SR & USART_FLAG_RXNE)){};
  return ((int)(USART2->DR & 0xFF));
  //return (GetChar());
}
