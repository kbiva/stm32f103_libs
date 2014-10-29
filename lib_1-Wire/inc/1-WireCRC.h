/*
 *  1-WireCRC.h
 *
 *  Author: Kestutis Bivainis
 *
 *  Original source code from
 *  http://kazus.ru/forums/showthread.php?t=100566
 */

#ifndef _1_WIRECRC_H
#define _1_WIRECRC_H

#include <stdint.h>

uint8_t Crc8Dallas(uint8_t len, uint8_t *pData);
uint8_t Crc8(uint32_t len, uint8_t *pcBlock);

#endif
