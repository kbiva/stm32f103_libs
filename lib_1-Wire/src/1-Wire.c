/*
 *  1-Wire.c
 *
 *  Author: Kestutis Bivainis
 *
 *  Original source code from 
 *  http://kazus.ru/forums/showthread.php?t=100566
 */

#include "1-Wire.h"
#include "stm32f10x_conf.h"
#include "delay.h"

void One_Wire_Pin_In(void) {
	
  GPIO_InitTypeDef GPIO_InitStruct;
  GPIO_InitStruct.GPIO_Pin=ONE_WIRE_PIN;
  GPIO_InitStruct.GPIO_Mode=GPIO_Mode_IN_FLOATING;
  GPIO_Init(ONE_WIRE_PORT, &GPIO_InitStruct);
}

void One_Wire_Pin_Out(void) {
	
  GPIO_InitTypeDef GPIO_InitStruct;
  GPIO_InitStruct.GPIO_Pin=ONE_WIRE_PIN;
  GPIO_InitStruct.GPIO_Speed=GPIO_Speed_50MHz;
  GPIO_InitStruct.GPIO_Mode=GPIO_Mode_Out_PP;
  GPIO_Init(ONE_WIRE_PORT, &GPIO_InitStruct);
}

void One_Wire_Init(void) {
	
  RCC_APB2PeriphClockCmd(ONE_WIRE_BUS, ENABLE);
}

uint8_t One_Wire_Reset(void) {
	
  uint8_t tmp;
  One_Wire_Pin_In();
  if((ONE_WIRE_PIN_READ)==0)
    return One_Wire_Bus_Low_Error;
  One_Wire_Pin_Out();
  ONE_WIRE_PIN_LOW;
  DWT_Delay(Time_Reset_Low);
  ONE_WIRE_PIN_HIGH;
  One_Wire_Pin_In();
  DWT_Delay(Time_Pulse_Delay_High);
  if (ONE_WIRE_PIN_READ)
    tmp=One_Wire_Error_No_Echo;
  else
    tmp=One_Wire_Success;
  DWT_Delay(Time_After_Reset);
  return tmp;
}

void One_Wire_Write_Byte(uint8_t Byte) {
	
  uint8_t cnt;
  for(cnt=0;cnt!=8;cnt++)
    One_Wire_Write_Bit(Byte&(1<<cnt));
}

void One_Wire_Write_Bit(uint8_t Bit) {
	
  One_Wire_Pin_Out();
  ONE_WIRE_PIN_LOW;
  if(Bit) {
		DWT_Delay(Time_Pulse_Delay_Low);
    ONE_WIRE_PIN_HIGH;
    DWT_Delay(Time_Pulse_Delay_High);    
  }
  else {
    DWT_Delay(Time_Pulse_Delay_High);
    ONE_WIRE_PIN_HIGH;
    DWT_Delay(Time_Pulse_Delay_Low);
  }
  One_Wire_Pin_In();
}

uint8_t One_Wire_Read_Byte(void) {
	
  uint8_t tmp=0;
  uint8_t cnt;
  for(cnt=0;cnt!=8;cnt++)
    if(One_Wire_Read_Bit())
      tmp|=(1<<cnt);
  DWT_Delay(Time_Pulse_Delay_High);
  return tmp;
}

uint8_t One_Wire_Read_Bit(void) {
	
  uint8_t tmp;
  One_Wire_Pin_Out();
  ONE_WIRE_PIN_LOW;
  DWT_Delay(Time_Hold_Down);
  One_Wire_Pin_In();
  DWT_Delay(Time_Pulse_Delay_Low);
  if(ONE_WIRE_PIN_READ)
    tmp = 1;
  else
    tmp = 0;
  DWT_Delay(Time_Pulse_Delay_High);
  return tmp;
}
