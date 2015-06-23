/*
 *  xorshift.h
 *
 *  Author: Kestutis Bivainis
 */

#ifndef XORSHIFT_H_
#define XORSHIFT_H_

#include <stdint.h>

void init_xorshift(uint32_t p1,uint32_t p2,uint32_t p3,uint32_t p4);
uint32_t xor128(void);

#endif
