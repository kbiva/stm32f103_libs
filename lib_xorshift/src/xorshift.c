/*
 *  xorshift.c
 *
 *  Xorshift pseudorandom number generator
 *
 *  Author: Kestutis Bivainis
 */

#include "xorshift.h"

static uint32_t x=123456789;
static uint32_t y=362436069;
static uint32_t z=521288629;
static uint32_t w=88675123;

void init_xorshift(uint32_t p1,uint32_t p2,uint32_t p3,uint32_t p4) {

  x=p1;
  y=p2;
  z=p3;
  w=p4;
}

uint32_t xor128(void) {

  uint32_t t;

  t = x ^ (x << 11);
  x = y;
  y = z;
  z = w;
  return w = w ^ (w >> 19) ^ (t ^ (t >> 8));
}
