/*
 *  delay.h
 *
 *  Author: Kestutis Bivainis
 */

#ifndef DELAY_H
#define DELAY_H

#include <stdint.h>

void DWT_Init(void);
uint32_t DWT_Get(void);
uint8_t DWT_Compare(int32_t tp);
void DWT_Delay(uint32_t us);

#endif
