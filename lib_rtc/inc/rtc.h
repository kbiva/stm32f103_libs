#ifndef RTC_H_
#define RTC_H_

#include <stdint.h>

#define BKP_DR1_VALUE 0xA5A5

typedef struct {
  uint16_t year;  /* 1..4095 */
  uint8_t  month; /* 1..12 */
  uint8_t  mday;  /* 1.. 31 */
  uint8_t  wday;  /* 0..6, Sunday = 0*/
  uint8_t  hour;  /* 0..23 */
  uint8_t  min;   /* 0..59 */
  uint8_t  sec;   /* 0..59 */
  uint8_t  dst;   /* 0 Winter, !=0 Summer */
} RTC_t;

#define FIRSTYEAR   2000                // start year
#define FIRSTDAY    6                   // 0 = Sunday

int RTC_Init(void);
void RTC_gettime (RTC_t*);                                      /* Get time */
void RTC_settime (const RTC_t*);                                /* Set time */

#endif
