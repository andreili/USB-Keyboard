#include "kbd_matrix.h"
#include "stm32f4xx.h"
#include "kbd_matrix_data.h"
#include "hid_proc.h"
#include "cmsis_os.h"

/*
* GPIO map:
*		PB00-PB11 - input
*		PC04-PC15 - output
*
*
*/

uint16_t kbd_data[KBD_MATRIX_ROW];
SemaphoreHandle_t matrix_fill_sem = NULL;
extern SemaphoreHandle_t usb_kbd_fill_sem;

uint16_t proc_row(uint16_t col_data)
{
	return 0;
}

void init_matrix(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCCR_INP, ENABLE);
	RCC_AHB1PeriphClockCmd(RCCR_OUT, ENABLE);
	
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | 
																GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_12 | GPIO_Pin_13 | 
																GPIO_Pin_14 | GPIO_Pin_15;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
  GPIO_Init(PORT_OUT, &GPIO_InitStructure);
	
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | 
																GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9 | 
																GPIO_Pin_10 | GPIO_Pin_11;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
  GPIO_Init(PORT_INP, &GPIO_InitStructure);
	
	matrix_fill_sem = xSemaphoreCreateMutex();
}

#define CHECK_MTX(scancode, matrix) \
	{ \
		if (matrix[scancode].row != 0x0000) \
		{ \
			kbd_data[matrix[scancode].row - 1] |= matrix[scancode].col; \
		} \
	}

void fill_matrix(uint32_t mode)
{
	xSemaphoreTake(matrix_fill_sem, 10);
	
	const key_mtx_t* kbd_matrix = kbd_mc7007;
	const key_mtx_t* kbd_matrix_f = kbd_mc7007_f;
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
	
	if (keys_pressed[0] != 0)
	{
		CHECK_MTX(((keys_pressed[0] & 0xff00) >> 8), kbd_matrix_f);
	}
	
	for (int i=0 ; i<KBR_MAX_NBR_PRESSED ; ++i)
	{
		uint16_t key_sc = keys_pressed[i] & 0x00ff;
		
		if (key_sc == 0)
			continue;
		
		CHECK_MTX(key_sc, kbd_matrix);
	}
	
	xSemaphoreGive(matrix_fill_sem);
}

void proc_matrix(void)
{
	xSemaphoreTake(matrix_fill_sem, 10);
	// TODO: encode ROW bits to number!
	uint16_t row = ~PORT_INP->IDR;
	PORT_OUT->ODR = ~kbd_data[row];
	xSemaphoreGive(matrix_fill_sem);
}
