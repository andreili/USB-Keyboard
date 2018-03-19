#include "gui_all.h"
#include "cmsis_os.h"
#include "hid_proc.h"
#include "kbd_matrix.h"
#include "lcd_driver.h"
#include <stdio.h>

void init_GUI(void)
{
	LCD_Initialization();
}

#define MATRIXCELL_SIZE_X 16
#define MATRIXCELL_SIZE_Y 16

void draw_cross(int x, int y, uint16_t color)
{
	x += 2;
	y += 2;
	for (int i=0 ; i<(MATRIXCELL_SIZE_X-3) ; ++i)
	{
		LCD_SetPoint(x+i, y+i, color);
		LCD_SetPoint(x+i, y+((MATRIXCELL_SIZE_X-3) - i), color);
	}
}

extern int dev_mode;
extern int boot_OK;

void main_GUI(void)
{
  osDelay(10);
	char buf[256];
	
	GUI_Text(0, 0, "USB boot mode", White, Black);
	while (dev_mode == 0)
	{
		if (boot_OK == 0)
			GUI_Text(10, 20, "FW load state: 0", White, Black);
		else
			GUI_Text(10, 20, "USB boot mode", White, Black);
	}
	
	LCD_clear();
	
	GUI_Text(18, 16, "0 1 2 3 4 5 6 7 8 9 A B", White, Black);
	buf[1] = 0;
	for (int i=0 ; i<KBD_MATRIX_ROW ; ++i)
	{
		if (i < 10)
			buf[0] = 48 + i;
		else
			buf[0] = 55 + i;
		GUI_Text(0, 32 + i*16, buf, White, Black);
	}
	for (int i=0 ; i<=KBD_MATRIX_ROW ; ++i)
	{
		int y = 32 + MATRIXCELL_SIZE_Y * i;
		LCD_DrawLine(16, y, 16 + MATRIXCELL_SIZE_X * KBD_MATRIX_COL, y, White);
	}
	for (int i=0 ; i<=KBD_MATRIX_COL ; ++i)
	{
		int x = 16 + MATRIXCELL_SIZE_X * i;
		LCD_DrawLine(x, 32, x, 32 + MATRIXCELL_SIZE_Y * KBD_MATRIX_ROW, White);
	}
	
	while (1)
	{		
		LED_3_ON();
		
		sprintf(buf, "HID: %04X %04X %04X %04X %04X %04X", keys_pressed[0], keys_pressed[1], 
				keys_pressed[2], keys_pressed[3], keys_pressed[4], keys_pressed[5]);
		GUI_Text(0, 0, buf, White, Black);
		
		for (int i=0 ; i<KBD_MATRIX_ROW ; ++i)
		{
			for (int j=0 ; j<KBD_MATRIX_COL ; ++j)
			{
				int x = 16 + MATRIXCELL_SIZE_X * j;
				int y = 32 + MATRIXCELL_SIZE_Y * i;
				if (kbd_data[i] & (1 << j))
				{
					draw_cross(x, y, White);
				}
				else
				{
					draw_cross(x, y, Black);
				}
			}
		}
		
		for (int i=0 ; i<12 ; ++i)
		{
			sprintf(buf, "%02i=%04X", i, kbd_data[i]);
			GUI_Text(250, 20 + i * 18, buf, White, Black);
		}
		
		LED_3_OFF();
		
    osDelay(100);
	}
}
