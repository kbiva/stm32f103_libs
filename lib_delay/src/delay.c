/*
 *  delay.c
 *
 *  Author: Kestutis Bivainis
 */

#include "delay.h"
#include "stm32f10x.h"

extern uint32_t SystemCoreClock;

void DWT_Init(void) {

  if (!(CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk)) {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0; 
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
  }
}

uint32_t DWT_Get(void) {
  
  return DWT->CYCCNT;
}

__inline
uint8_t DWT_Compare(int32_t tp) {
  
  return (((int32_t)DWT_Get() - tp) < 0);
}

// microseconds delay
void DWT_Delay(uint32_t us) {

  int32_t tp = DWT_Get() + us * (SystemCoreClock/1000000);
  while (DWT_Compare(tp));
}
