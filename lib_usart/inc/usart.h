/*
 *  usart.h
 *
 *  Author: Kestutis Bivainis
 *
 */

#ifndef USART_H_
#define USART_H_

#include <stdint.h>

#define USART1_PIN_TX GPIO_Pin_9
#define USART1_PIN_TX_Port GPIOA
#define USART1_PIN_TX_Bus  RCC_APB2Periph_GPIOA

#define USART1_PIN_RX GPIO_Pin_10
#define USART1_PIN_RX_Port GPIOA
#define USART1_PIN_RX_Bus  RCC_APB2Periph_GPIOA

#define USART2_PIN_TX GPIO_Pin_2
#define USART2_PIN_TX_Port GPIOA
#define USART2_PIN_TX_Bus  RCC_APB2Periph_GPIOA

#define USART2_PIN_RX GPIO_Pin_3
#define USART2_PIN_RX_Port GPIOA
#define USART2_PIN_RX_Bus  RCC_APB2Periph_GPIOA

void USART1_Init(uint32_t baud);
void USART2_Init(uint32_t baud);

#endif /* USART_H_ */
