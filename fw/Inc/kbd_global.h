#ifndef _KBD_GLOBAL_H_
#define _KBD_GLOBAL_H_

#include "stm32f4xx_hal.h"
#include "usbh_hid_keybd.h"

#define KBD_VERSION ((uint32_t)0x00000003)

//#define GUI_ENABLE
#define KBD_DEBUG
//#define IWDG_USE

#define KBD_MATRIX_ROW 12
#define KBD_MATRIX_COL 12

/* 12kHZ = 168MHz/14/1000:     Prescaler = 14 Period = 1000 */
/* 1 HZ  = 168MHz/168000/1000: Prescaler = 168000, Period = 1000 */
#define PS2_CLK_PERIOD      1000
#define PS2_CLK_PRESCALER   14

// data from USB keyboard
extern HID_KEYBD_Info_TypeDef     keybd_info;
// matrix of a keys
extern uint16_t kbd_data[KBD_MATRIX_ROW];

#define SW_MODE_RK86		0x01
#define SW_MODE_MC7007	0x02

#define SW_MODE_MATRIX	0x00
#define SW_MODE_PS2			0x01
#define SW_MODE_ZXBUS		0x02
#define SW_MODE_ORION		0x03
#define SW_MODE_RES2		0x04
#define SW_MODE_RES3		0x05
#define SW_MODE_RES4		0x06
#define SW_MODE_RES5		0x07

extern uint8_t usb_mode;
extern uint8_t matrix_mode;
extern uint8_t PS2_SendRequest;

typedef struct
{ 
	void (*init)(void);
	void (*proc)(void);
	void (*interrupt)(void);
	void (*periodic)(TIM_HandleTypeDef *htim);
} kbd_proc_t;

extern const kbd_proc_t proc_matrix;
extern const kbd_proc_t proc_ps2;
extern const kbd_proc_t proc_zxbus;
extern const kbd_proc_t proc_orion;
extern const kbd_proc_t proc_dummy;

typedef struct 
{
  uint8_t* mtx_lat;
  uint8_t* mtx_loc;
	uint8_t* mtx_fns;
} kbd_matrix_t;

extern kbd_matrix_t kbd_sel;
void matrix_sel(void);

#ifdef KBD_DEBUG
#define DEBUG_PR(...) printf(__VA_ARGS__)
#else
#define DEBUG_PR(...)
#endif

#endif
