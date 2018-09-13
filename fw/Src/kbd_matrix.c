#include "kbd_matrix.h"
#include "stm32f4xx_hal.h"
#include "kbd_matrix_data.h"
#include "usbh_hid_keybd.h"
#include <inttypes.h>
#include "kbd_global.h"

/*
* GPIO map:
*		PE00-PE11 - input
*		PD04-PD15 - output
*
*
*/

kbd_matrix_t kbd_sel;

uint8_t matrix_mode;

uint16_t kbd_data[KBD_MATRIX_ROW];

uint16_t proc_row(uint16_t col_data)
{
	return 0;
}

void matrix_init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct;
	
  GPIO_InitStruct.Pin = GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | 
												GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13 | 
												GPIO_PIN_14 | GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
	
  GPIO_InitStruct.Pin = GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | 
												GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13 | 
												GPIO_PIN_14 | GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
}

#define CHECK_MTX(scancode, matrix) \
	{ \
		uint8_t pos = matrix[scancode]; \
		if (pos != 0xff) \
			kbd_data[(pos & 0xf0) >> 4] &= ~(1 << (pos & 0x0f)); \
	}
	
extern HID_KEYBD_Info_TypeDef     keybd_info;
	
void matrix_sel(void)
{
	switch (matrix_mode)
	{
		default:
		case SW_MODE_RK86:
			kbd_sel = kbd_rk86;
			break;
		case SW_MODE_MC7007:
			kbd_sel = kbd_mc7007;
			break;
	}
}

void matrix_proc()
{
	for (int i=0 ; i<KBD_MATRIX_ROW ; ++i)
		kbd_data[i] = 0xffff;
	
	if (keybd_info.lctrl || keybd_info.rctrl)
		CHECK_MTX(0, kbd_sel.mtx_fns);
	if (keybd_info.lshift || keybd_info.rshift)
		CHECK_MTX(1, kbd_sel.mtx_fns);
	if (keybd_info.lalt || keybd_info.ralt)
		CHECK_MTX(2, kbd_sel.mtx_fns);
	if (keybd_info.lgui || keybd_info.rgui)
		CHECK_MTX(3, kbd_sel.mtx_fns);
	
	uint8_t* mtx;
	if ((keybd_info.lshift || keybd_info.rshift) && (kbd_sel.mtx_lat_shifted != 0))
		mtx = kbd_sel.mtx_lat_shifted;
	else
		mtx = kbd_sel.mtx_lat;

	for (int i=0 ; i<6 ; ++i)
	{
		uint8_t key_sc = keybd_info.keys[i];
		if (key_sc == 0)
			continue;
		CHECK_MTX(key_sc, mtx);
	}
}

void matrix_int(void)
{
	__disable_irq();
	uint16_t row = ~(PORT_INP->IDR >> 4);
	uint16_t i, idx;
	// encode a bit index to row number
	for (i=1, idx=0 ; i<0x1000 ; i<<=1, ++idx)
		if (row & i)
		{
			row = idx;
			break;
		}
	idx = ~(kbd_data[row] << 4);
	PORT_OUT->ODR = (PORT_OUT->ODR & 0x000f) | idx;
	__enable_irq();
}


const kbd_proc_t proc_matrix =
{
	.init = matrix_init,
	.proc = matrix_proc,
	.interrupt = matrix_int,
	.periodic = NULL
};
