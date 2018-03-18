#include "kbd_matrix.h"
#include "stm32f4xx.h"
#include "kbd_matrix_data.h"
#include "hid_proc.h"
#include "cmsis_os.h"

/*
* GPIO map:
*		PE00-PE11 - input
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
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
	
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | 
																GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_12 | GPIO_Pin_13 | 
																GPIO_Pin_14 | GPIO_Pin_15;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
  GPIO_Init(GPIOD, &GPIO_InitStructure);
	
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | 
																GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9 | 
																GPIO_Pin_10 | GPIO_Pin_11;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
  GPIO_Init(GPIOE, &GPIO_InitStructure);
	
	matrix_fill_sem = xSemaphoreCreateMutex();
}

void fill_matrix(uint32_t mode)
{
	xSemaphoreTake(matrix_fill_sem, 10);
	
	uint16_t* kbd_matrix = kbd_mc7007;
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
	
	for (int i=0 ; i<KBR_MAX_NBR_PRESSED ; ++i)
	{
		uint16_t key = keys_pressed[i];
		uint16_t alt_keys = (key & 0xff00) >> 8;
		uint16_t key_sc = keys_pressed[i] & 0x00ff;
		
		if (key_sc == 0)
			continue;
		
		for (int r=0 ; r<KBD_MATRIX_ROW ; ++r)
		{
			int row_offs = r * KBD_MATRIX_ROW;
			uint16_t row_d = kbd_data[r];
			
			for (int c=0 ; c<KBD_MATRIX_COL ; ++c)
			{
				int idx = row_offs + c;
				if (/*(kbd_matrix[r][c] == alt_keys) ||*/ (kbd_matrix[idx] == key_sc))
					row_d |= (1 << c);
			}
			
			kbd_data[r] = row_d;
		}
	}
	
	xSemaphoreGive(matrix_fill_sem);
}

void proc_matrix(void)
{
	xSemaphoreTake(matrix_fill_sem, 10);
	//
	xSemaphoreGive(matrix_fill_sem);
}
