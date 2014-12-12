/*
 *  counter.h
 *
 *  Author: Kestutis Bivainis
 *
 *  Adapted from:
 *  http://we.easyelectronics.ru/electro-and-pc/podklyuchenie-mikrokontrollera-k-lokalnoy-seti.html
 */

#ifndef _COUNTER_H
#define _COUNTER_H

#include <stdint.h>

extern uint32_t tick_count;
extern uint32_t second_count;

#define gettc()  tick_count
#define rtime()  second_count

#endif
