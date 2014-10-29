/*
 *  1-WireCRC.c
 *
 *  Author: Kestutis Bivainis
 *
 *  Original source code from 
 *  http://kazus.ru/forums/showthread.php?t=100566 
 */

#include "1-WireCRC.h"

uint8_t Crc8Dallas(uint8_t len, uint8_t *pData) {
   uint8_t crc = 0;
   uint8_t i;

   while(len--) {
     crc ^= *pData++;
    for(i=0;i<8;i++)
       crc = crc&0x01 ? (crc>>1)^0x8C : crc>>1;
   }
  return crc;
}

uint8_t Crc8(uint32_t len, uint8_t *pcBlock) {
  uint8_t crc = 0xFF;
   uint8_t i;

   while (len--)  {
     crc ^= *pcBlock++;
    for(i=0;i<8;i++)
       crc = crc&0x80 ? (crc<<1)^0x31 : crc<<1;
   }
   return crc;
}
