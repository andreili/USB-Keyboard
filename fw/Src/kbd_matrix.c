#include "kbd_matrix.h"
#include "stm32f4xx_hal.h"
#include "kbd_matrix_data.h"
#include "usbh_hid_keybd.h"
#include "cmsis_os.h"
#include <inttypes.h>
#include "kbd_global.h"

/*
* GPIO map:
*		PB00-PB11 - input
*		PC04-PC15 - output
*
*
*/

uint16_t kbd_data[KBD_MATRIX_ROW];

uint16_t proc_row(uint16_t col_data)
{
	return 0;
}

void init_matrix(void)
{
  GPIO_InitTypeDef GPIO_InitStruct;
	
  GPIO_InitStruct.Pin = GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | 
												GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13 | 
												GPIO_PIN_14 | GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
	
  GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | 
												GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9 | 
												GPIO_PIN_10 | GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
}

#define CHECK_MTX(scancode, matrix) \
	{ \
		uint8_t row = (matrix[scancode] & 0xf0) >> 4; \
		if (row != 15) \
		{ \
			kbd_data[row] |= (1 << (matrix[scancode] & 0x0f)); \
		} \
	}
	
extern HID_KEYBD_Info_TypeDef     keybd_info;

void fill_matrix(uint32_t mode)
{
	__disable_irq();
	
	const uint8_t* kbd_matrix = kbd_mc7007_lat;
	const uint8_t* kbd_matrix_f = kbd_mc7007_f;
	/*switch (mode)
	{
		default:
		case SW_MODE_RK86:
			kbd_matrix = kbd_rk86;
			break;
		case SW_MODE_MC7007:
			kbd_matrix = kbd_mc7007;
			break;
	}*/
		
	for (int i=0 ; i<KBD_MATRIX_ROW ; ++i)
		kbd_data[i] = 0;
	
	if (keybd_info.lctrl || keybd_info.rctrl)
		CHECK_MTX(0, kbd_matrix_f);
	if (keybd_info.lshift || keybd_info.rshift)
		CHECK_MTX(1, kbd_matrix_f);
	if (keybd_info.lalt || keybd_info.ralt)
		CHECK_MTX(2, kbd_matrix_f);
	if (keybd_info.lgui || keybd_info.rgui)
		CHECK_MTX(3, kbd_matrix_f);
	
	for (int i=0 ; i<6 ; ++i)
	{
		uint8_t key_sc = keybd_info.keys[i];
		
		if (key_sc == 0)
			continue;
		
		CHECK_MTX(key_sc, kbd_matrix);
	}
	
	__enable_irq();
}

/*void proc_matrix(void)
{
	__disable_irq();
	uint16_t row = ~PORT_INP->IDR;
	uint16_t i, idx;
	// encode a bit index to row number
	for (i=1, idx=0 ; i<0x1000 ; i<<=1, ++idx)
		if (row & i)
		{
			row = idx;
			break;
		}
	PORT_OUT->ODR = (PORT_OUT->ODR & 0x000f) | (~kbd_data[row]);
	__enable_irq();
}*/
