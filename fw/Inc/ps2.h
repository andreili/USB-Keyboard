#ifndef __PS2_H__
#define __PS2_H__

#include "stm32f4xx_hal.h"

/* 12kHZ = 168MHz/14/1000:     Prescaler = 14 Period = 1000 */
/* 1 HZ  = 168MHz/168000/1000: Prescaler = 168000, Period = 1000 */
#define PS2_CLK_PERIOD      1000
#define PS2_CLK_PRESCALER   14

void PS2_DataIRQHandler(void);
void PS2_ClockIRQHandler(void);
void PS2_init(void);

#endif
