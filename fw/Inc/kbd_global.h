#ifndef _KBD_GLOBAL_H_
#define _KBD_GLOBAL_H_

#include "stm32f4xx_hal.h"
#include "usbh_hid_keybd.h"

#define KBD_MATRIX_ROW 12
#define KBD_MATRIX_COL 12

// data from USB keyboard
extern HID_KEYBD_Info_TypeDef     keybd_info;
// matrix of a keys
extern uint16_t kbd_data[KBD_MATRIX_ROW];

#define SW_MODE_RK86		0x01
#define SW_MODE_MC7007	0x02

#define SW_MODE_PS2			0x01	// BIT 0
#define SW_MODE_MATRIX	0x02	// BIT 1
#define SW_MODE_ZXBUS		0x04	// BIT 2
#define SW_MODE_RES1		0x08	// BIT 3
#define SW_MODE_RES2		0x10	// BIT 4
#define SW_MODE_RES3		0x20	// BIT 5

extern uint8_t usb_mode;
extern uint8_t PS2_SendRequest;

#endif
