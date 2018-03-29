#ifndef __PS2_H__
#define __PS2_H__

#include "stm32f4xx_hal.h"

void PS2_add_event(uint8_t is_released, uint8_t HID_scancode);
void PS2_add_event_sys(uint8_t is_released, uint8_t HID_scancode_sys);

#endif
