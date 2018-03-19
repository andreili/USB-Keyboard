#ifndef _LCD_DRIVER_H_
#define _LCD_DRIVER_H_

#include <math.h>
#include "stm32f4xx.h"

//#define DISP_ORIENTATION					0
#define DISP_ORIENTATION					90
//#define DISP_ORIENTATION					180
//#define DISP_ORIENTATION					270

#if  ( DISP_ORIENTATION == 90 ) || ( DISP_ORIENTATION == 270 )
	#define  MAX_X  320
	#define  MAX_Y  240   
#elif  ( DISP_ORIENTATION == 0 ) || ( DISP_ORIENTATION == 180 )
	#define  MAX_X  240
	#define  MAX_Y  320   
#endif

#define White          0xFFFF
#define Black          0x0000
#define Grey           0xF7DE
#define Blue           0x001F
#define Blue2          0x051F
#define Red            0xF800
#define Magenta        0xF81F
#define Green          0x07E0
#define Cyan           0x7FFF
#define Yellow         0xFFE0

#define RGB565CONVERT(red, green, blue) (int) (((red >> 3) << 11) | ((green >> 2) << 5) | (blue >> 3))

void LCD_Initializtion(void);
uint16_t LCD_GetPoint(uint16_t Xpos,uint16_t Ypos);
void LCD_SetPoint(uint16_t Xpos,uint16_t Ypos,uint16_t point);
void LCD_DrawLine( uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1 , uint16_t color );

void LCD_WriteIndex(uint16_t index);
void LCD_WriteData(uint16_t data);
void LCD_WriteReg(uint16_t LCD_Reg,uint16_t LCD_RegValue);
void LCD_SetCursor( uint16_t Xpos, uint16_t Ypos );
uint16_t LCD_ReadData(void);
void LCD_fill_mem(void);
	
#endif
