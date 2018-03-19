#include "gui_all.h"
#include "cmsis_os.h"
#include <GUI.h>
#include "WM.h"
#include "FRAMEWIN.h"
#include "hid_proc.h"
#include "kbd_matrix.h"

static FRAMEWIN_Handle _ahFrameWin[2];

void init_GUI(void)
{
	GUI_Init();
}

#define MATRIXCELL_SIZE_X 15
#define MATRIXCELL_SIZE_Y 16

void main_GUI(void)
{
	int xCenter = LCD_GET_XSIZE() / 2;
  int y;
		
	GUI_SetBkColor(GUI_BLACK);
	GUI_SetColor(GUI_LIGHTGRAY);
	GUI_Clear();
  osDelay(10);
	char buf[256];
	
	while (1)
	{		
		GUI_SetFont(&GUI_Font16_ASCII);
		LED_3_ON();
		
		sprintf(buf, "HID scancodes: %04X %04X %04X %04X %04X %04X", keys_pressed[0], keys_pressed[1], 
				keys_pressed[2], keys_pressed[3], keys_pressed[4], keys_pressed[5]);
		GUI_DispStringAt(buf, 0, 0);
		
		GUI_DispStringAt("Keyboard matrix: ", 0, 16);
		GUI_DispStringAt("0  1  2  3  4  5  6  7  8  9  A  B", 18, 32);
		GUI_DispStringAt("0\n1\n2\n3\n4\n5\n6\n7\n8\n9\nA\nB", 0, 48);
		for (int i=0 ; i<KBD_MATRIX_ROW ; ++i)
		{
			for (int j=0 ; j<KBD_MATRIX_COL ; ++j)
			{
				int x = 16 + MATRIXCELL_SIZE_X * j;
				int y = 48 + MATRIXCELL_SIZE_Y * i;
				if (kbd_data[i] & (1 << j))
				{
					GUI_FillRect(x, y, x + MATRIXCELL_SIZE_X, y + MATRIXCELL_SIZE_Y);
				}
				else
				{
					GUI_ClearRect(x, y, x + MATRIXCELL_SIZE_X, y + MATRIXCELL_SIZE_Y);
					GUI_DrawRect(x, y, x + MATRIXCELL_SIZE_X, y + MATRIXCELL_SIZE_Y);
				}
			}
		}
		
		for (int i=0 ; i<12 ; ++i)
		{
			sprintf(buf, "%i=%04X", i, kbd_data[i]);
			GUI_DispStringAt(buf, 255, 30 + i * 20);
		}
		
		WM_ExecIdle();
		LED_3_OFF();
		
    osDelay(100);
	}
}
