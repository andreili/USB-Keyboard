#include "lcd_driver.h"
#include "cmsis_os.h"
#include "kbd_matrix.h"
#include "AsciiLib.h"

// ILI9325

extern TIM_HandleTypeDef htim4;

#define BUF_MAX_Y (MAX_Y/16)

#define LCD_REG              (*((volatile unsigned short *) 0x60000000)) /* RS = 0 */
#define LCD_RAM              (*((volatile unsigned short *) 0x60020000)) /* RS = 1 */

#define   BKCOLOR LCD_BKCOLORINDEX
#define   COLOR LCD_COLORINDEX

uint16_t lcd_buff[MAX_X][BUF_MAX_Y];

void LCD_Initialization(void)
{
	//DeviceCode = 0x0123;
	//DeviceCode = LCD_ReadReg(0x0000);		/* Read LCD ID	*/	
	
	LCD_WriteReg(0x00e7,0x0010);      
	LCD_WriteReg(0x0000,0x0001);  	/* start internal osc */
	LCD_WriteReg(0x0001,0x0100);     
	LCD_WriteReg(0x0002,0x0700); 	/* power on sequence */
	LCD_WriteReg(0x0003,(1<<12)|(1<<5)|(1<<4)|(0<<3) ); 	/* importance */
	LCD_WriteReg(0x0004,0x0000);                                   
	LCD_WriteReg(0x0008,0x0207);	           
	LCD_WriteReg(0x0009,0x0000);         
	LCD_WriteReg(0x000a,0x0000); 	/* display setting */        
	LCD_WriteReg(0x000c,0x0001);	/* display setting */        
	LCD_WriteReg(0x000d,0x0000); 			        
	LCD_WriteReg(0x000f,0x0000);
	/* Power On sequence */
	LCD_WriteReg(0x0010,0x0000);   
	LCD_WriteReg(0x0011,0x0007);
	LCD_WriteReg(0x0012,0x0000);                                                                 
	LCD_WriteReg(0x0013,0x0000);                 
	osDelay(50);  /* delay 50 ms */		
	LCD_WriteReg(0x0010,0x1590);   
	LCD_WriteReg(0x0011,0x0227);
	osDelay(50);  /* delay 50 ms */		
	LCD_WriteReg(0x0012,0x009c);                  
	osDelay(50);  /* delay 50 ms */		
	LCD_WriteReg(0x0013,0x1900);   
	LCD_WriteReg(0x0029,0x0023);
	LCD_WriteReg(0x002b,0x000e);
	osDelay(50);  /* delay 50 ms */		
	LCD_WriteReg(0x0020,0x0000);                                                            
	LCD_WriteReg(0x0021,0x0000);           
	osDelay(50);  /* delay 50 ms */		
	LCD_WriteReg(0x0030,0x0007); 
	LCD_WriteReg(0x0031,0x0707);   
	LCD_WriteReg(0x0032,0x0006);
	LCD_WriteReg(0x0035,0x0704);
	LCD_WriteReg(0x0036,0x1f04); 
	LCD_WriteReg(0x0037,0x0004);
	LCD_WriteReg(0x0038,0x0000);        
	LCD_WriteReg(0x0039,0x0706);     
	LCD_WriteReg(0x003c,0x0701);
	LCD_WriteReg(0x003d,0x000f);
	osDelay(50);  /* delay 50 ms */		
	LCD_WriteReg(0x0050,0x0000);        
	LCD_WriteReg(0x0051,0x00ef);   
	LCD_WriteReg(0x0052,0x0000);     
	LCD_WriteReg(0x0053,0x013f);
	LCD_WriteReg(0x0060,0xa700);        
	LCD_WriteReg(0x0061,0x0001); 
	LCD_WriteReg(0x006a,0x0000);
	LCD_WriteReg(0x0080,0x0000);
	LCD_WriteReg(0x0081,0x0000);
	LCD_WriteReg(0x0082,0x0000);
	LCD_WriteReg(0x0083,0x0000);
	LCD_WriteReg(0x0084,0x0000);
	LCD_WriteReg(0x0085,0x0000);
		
	LCD_WriteReg(0x0090,0x0010);     
	LCD_WriteReg(0x0092,0x0000);  
	LCD_WriteReg(0x0093,0x0003);
	LCD_WriteReg(0x0095,0x0110);
	LCD_WriteReg(0x0097,0x0000);        
	LCD_WriteReg(0x0098,0x0000);  
	/* display on sequence */    
	LCD_WriteReg(0x0007,0x0133);

	LCD_WriteReg(0x0020,0x0000);
	LCD_WriteReg(0x0021,0x0000);					
	osDelay(50);   /* delay 50 ms */
	
	HAL_TIM_Base_Start_IT(&htim4);
}

/******************************************************************************
* Function Name  : LCD_BGR2RGB
* Description    : RRRRRGGGGGGBBBBB convert to BBBBBGGGGGGRRRRR
* Input          : RGB color
* Output         : None
* Return         : RGB color
* Attention		 :
*******************************************************************************/
uint16_t LCD_BGR2RGB(uint16_t color)
{
	uint16_t  r, g, b, rgb;
	
	b = ( color>>0 )  & 0x1f;
	g = ( color>>5 )  & 0x3f;
	r = ( color>>11 ) & 0x1f;
	
	rgb =  (b<<11) + (g<<5) + (r<<0);
	
	return( rgb );
}

uint16_t LCD_GetPoint(uint16_t Xpos,uint16_t Ypos)
{
	//return LCD_BGR2RGB(lcd_buff[Xpos][Ypos]);
	return 0;
	/*uint16_t dummy;
	
	LCD_SetCursor(Xpos,Ypos);

	LCD_WriteIndex(0x0022);
	
		dummy = LCD_ReadData();
		dummy = LCD_ReadData(); 
		return  LCD_BGR2RGB(dummy);*/
}

/******************************************************************************
* Function Name  : LCD_SetPoint
* Description    : 
* Input          : - Xpos: Row Coordinate
*                  - Ypos: Line Coordinate 
* Output         : None
* Return         : None
* Attention		 : None
*******************************************************************************/
void LCD_SetPoint(uint16_t Xpos,uint16_t Ypos,uint16_t point)
{
	if( Xpos >= MAX_X || Ypos >= MAX_Y )
		return;
	
	Ypos = MAX_Y - Ypos;
	int Yof = Ypos >> 4;
	int Ybt = Ypos & 0xf; 
	
	if (point >= 0x7777)
		lcd_buff[Xpos][Yof] |= (1 << Ybt);
	else
		lcd_buff[Xpos][Yof] &= ~(1 << Ybt);
}

/******************************************************************************
* Function Name  : LCD_DrawLine
* Description    : Bresenham's line algorithm
* Input          : - x0:
*                  - y0:
*       				   - x1:
*       		       - y1:
*                  - color:
* Output         : None
* Return         : None
* Attention		 : None
*******************************************************************************/	 
void LCD_DrawLine( uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1 , uint16_t color )
{
  short dx,dy;
  short temp;

  if( x0 > x1 )
  {
    temp = x1;
    x1 = x0;
    x0 = temp;   
  }
  if( y0 > y1 )
  {
    temp = y1;
    y1 = y0;
    y0 = temp;   
  }

  dx = x1-x0;
  dy = y1-y0;

  if( dx == 0 )
  {
    do
    { 
      LCD_SetPoint(x0, y0, color);
      y0++;
    }while( y1 >= y0 ); 
    return; 
  }
  if( dy == 0 )
  {
    do
    {
      LCD_SetPoint(x0, y0, color);
      x0++;
    }
    while( x1 >= x0 ); 
		return;
  }

	/* Bresenham's line algorithm  */
  if( dx > dy )
  {
    temp = 2 * dy - dx;
    while( x0 != x1 )
    {
	    LCD_SetPoint(x0,y0,color);
	    x0++;
	    if( temp > 0 )
	    {
	      y0++;
	      temp += 2 * dy - 2 * dx; 
	 	  }
      else         
      {
			  temp += 2 * dy;
			}       
    }
    LCD_SetPoint(x0,y0,color);
  }  
  else
  {
    temp = 2 * dx - dy;
    while( y0 != y1 )
    {
	 	  LCD_SetPoint(x0,y0,color);     
      y0++;                 
      if( temp > 0 )           
      {
        x0++;               
        temp+=2*dy-2*dx; 
      }
      else
			{
        temp += 2 * dy;
			}
    } 
    LCD_SetPoint(x0,y0,color);
	}
} 

/*******************************************************************************
* Function Name  : LCD_WriteReg
* Description    : 
* Input          : - index:
* Output         : None
* Return         : None
* Attention		 : None
*******************************************************************************/
__inline void LCD_WriteIndex(uint16_t index)
{
	//GPIO_ResetBits(GPIOD , GPIO_Pin_7);		 //CS=0;
	LCD_REG	= index;
	//GPIO_SetBits(GPIOD , GPIO_Pin_7);		 //CS=1;
}

/*******************************************************************************
* Function Name  : LCD_ReadData
* Description    : 
* Input          : None
* Output         : None
* Return         : 
* Attention		 : None
*******************************************************************************/
__inline uint16_t LCD_ReadData(void)
{
	//uint32_t tmp;
	//GPIO_ResetBits(GPIOD , GPIO_Pin_7);		 //CS=0;
	//tmp = LCD_RAM;
	//GPIO_SetBits(GPIOD , GPIO_Pin_7);		 //CS=1;
	return LCD_RAM;
}

void LCD_SetCursor( uint16_t Xpos, uint16_t Ypos )
{				   
    #if  ( DISP_ORIENTATION == 90 ) || ( DISP_ORIENTATION == 270 )

		uint16_t temp;
		Ypos = ( MAX_Y - 1 ) - Ypos;
		temp = Ypos;
		Ypos = Xpos;
		Xpos = temp; 

	#elif  ( DISP_ORIENTATION == 0 ) || ( DISP_ORIENTATION == 180 )

		Ypos = ( MAX_Y - 1 ) - Ypos;
			
	#endif
	
	LCD_WriteReg(0x0020, Xpos );     
	LCD_WriteReg(0x0021, Ypos );  
}


/*******************************************************************************
* Function Name  : LCD_WriteReg
* Description    : Writes to the selected LCD register.
* Input          : - LCD_Reg: address of the selected register.
*                  - LCD_RegValue: value to write to the selected register.
* Output         : None
* Return         : None
* Attention		 : None
*******************************************************************************/
__inline void LCD_WriteReg(uint16_t LCD_Reg,uint16_t LCD_RegValue)
{ 
	/* Write 16-bit Index, then Write Reg */  
	LCD_WriteIndex(LCD_Reg);         
	/* Write 16-bit Reg */
	LCD_WriteData(LCD_RegValue);  
}

/*******************************************************************************
* Function Name  : LCD_WriteReg
* Description    : 
* Input          : - index:
* Output         : None
* Return         : None
* Attention		 : None
*******************************************************************************/
__inline void LCD_WriteData(uint16_t data)
{
	//GPIO_ResetBits(GPIOD , GPIO_Pin_7);		 //CS=0;
	LCD_RAM = data;
	//GPIO_SetBits(GPIOD , GPIO_Pin_7);		 //CS=1;
}

void LCD_fill_mem(void)
{
	int32_t x, y;
	LCD_SetCursor(0,0);
	LCD_WriteIndex(0x0022);
	for(x=0 ; x<MAX_X ; ++x)
	{
		for(y=0 ; y<BUF_MAX_Y ; ++y)
		{
			uint32_t d = lcd_buff[x][y];
			for (int i=0 ; i<16 ; ++i)
			{
				if ((d & (1 << i)))
					LCD_RAM = White;
				else
					LCD_RAM = Black;
			}
		}
	}
}

void LCD_clear(void)
{
	for(int x=0 ; x<MAX_X ; ++x)
		for(int y=0 ; y<BUF_MAX_Y ; ++y)
			lcd_buff[x][y] = 0;
}

void PutChar( uint16_t Xpos, uint16_t Ypos, uint8_t ASCI, uint16_t charColor, uint16_t bkColor )
{
	uint16_t i, j;
	uint8_t buffer[16], tmp_char;
	GetASCIICode(buffer,ASCI);
	for( i=0; i<16; i++ )
	{
			tmp_char = buffer[i];
			for( j=0; j<8; j++ )
			{
					if( ((tmp_char >> (7 - j)) & 0x01) == 0x01 )
							LCD_SetPoint(Xpos + j, Ypos + i, charColor);
					else
							LCD_SetPoint(Xpos + j, Ypos + i, bkColor);
			}
	}
}

void GUI_Text(uint16_t Xpos, uint16_t Ypos, char *str,uint16_t Color, uint16_t bkColor)
{
    char TempChar;
    do
    {
        TempChar = *str++;  
				if (TempChar == '\n')
        {
            Xpos = 0;
            Ypos += 16;
						continue;
        } 
				
        PutChar( Xpos, Ypos, TempChar, Color, bkColor );    
        if( Xpos < MAX_X - 8 )
        {
            Xpos += 8;
        } 
        else if ( Ypos < MAX_Y - 16 )
        {
            Xpos = 0;
            Ypos += 16;
        }   
        else
        {
            Xpos = 0;
            Ypos = 0;
        }    
    }
    while ( *str != 0 );
}
