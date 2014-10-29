/*
 *  usart.h
 *
 *  Author: Kestutis Bivainis
 *
 */

#ifndef _XPT2046_H_
#define _XPT2046_H_

#include <stdint.h>

#define SPI1_TOUCH_CS GPIO_Pin_7
#define SPI1_SCK  GPIO_Pin_5
#define SPI1_MISO GPIO_Pin_6
#define SPI1_MOSI GPIO_Pin_7


//0B10010000 measure Y
#define CMD_RDX  0x90
//0B11010000 measure X
#define CMD_RDY  0xD0
//0B10110000 measure Z1
#define CMD_RDZ1 0xB0
//0B11000000 measure Z2
#define CMD_RDZ2 0xC0


void Touch_Init(void);
uint16_t Touch_Read(uint8_t cmd);

#endif

