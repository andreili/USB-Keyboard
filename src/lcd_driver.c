#include "lcd_driver.h"
#include "LCD_ConfDefaults.h"            /* Configuration header file */
#include "LCD_Protected.h"
#include "GUI.h"
#include "cmsis_os.h"
#include "kbd_matrix.h"

// ILI9325

#define BUF_MAX_Y (MAX_Y/16)

#define LCD_REG              (*((volatile unsigned short *) 0x60000000)) /* RS = 0 */
#define LCD_RAM              (*((volatile unsigned short *) 0x60020000)) /* RS = 1 */

#define   BKCOLOR LCD_BKCOLORINDEX
#define   COLOR LCD_COLORINDEX

uint16_t lcd_buff[MAX_X][BUF_MAX_Y];

extern GUI_CONTEXT GUI_Context;

#if (LCD_BITSPERPIXEL <= 8) && (GUI_NUM_LAYERS < 2)
  #define LCD_BKCOLORINDEX GUI_Context.LCD.aColorIndex8[0]
  #define LCD_COLORINDEX   GUI_Context.LCD.aColorIndex8[1]
  #define LCD_ACOLORINDEX  GUI_Context.LCD.aColorIndex8
#else
  #define LCD_BKCOLORINDEX GUI_Context.LCD.aColorIndex16[0]
  #define LCD_COLORINDEX   GUI_Context.LCD.aColorIndex16[1]
  #define LCD_ACOLORINDEX  GUI_Context.LCD.aColorIndex16
#endif

extern LCD_PIXELINDEX LCD__aConvTable[LCD_MAX_LOG_COLORS];

int LCD_L0_Init(void)
{
  LCD_Initializtion();
  return 0;
}

void LCD_L0_SetPixelIndex(int x, int y, int PixelIndex)
{
  LCD_SetPoint(x,y,PixelIndex);
}

unsigned int LCD_L0_GetPixelIndex(int x, int y)
{
  return LCD_GetPoint(x,y);
}

void LCD_L0_SetOrg(int x,int y)
{
}

void LCD_L0_XorPixel(int x, int y)
{
  LCD_PIXELINDEX Index = LCD_GetPoint(x,y);
  LCD_SetPoint(x,y,LCD_NUM_COLORS-1-Index);
}

void LCD_L0_DrawHLine(int x0, int y,  int x1)
{
  LCD_DrawLine(x0,y,x1,y,LCD_COLORINDEX);
}

void LCD_L0_DrawVLine(int x, int y0,  int y1)
{
  LCD_DrawLine(x,y0,x,y1,LCD_COLORINDEX);
}

void LCD_L0_FillRect(int x0, int y0, int x1, int y1) 
{
#if !LCD_SWAP_XY
  for (; y0 <= y1; y0++) {
    LCD_L0_DrawHLine(x0,y0, x1);
  }
#else
  for (; x0 <= x1; x0++) {
    LCD_L0_DrawVLine(x0,y0, y1);
  }
#endif
}

void LCD_L0_On(void)
{
}

void DrawBitLine1BPP(int x, int y, U8 const*p, int Diff, int xsize, const LCD_PIXELINDEX*pTrans)
{
  LCD_PIXELINDEX pixels;
  LCD_PIXELINDEX Index0 = *(pTrans+0);
  LCD_PIXELINDEX Index1 = *(pTrans+1);
/*
// Jump to right entry point
*/
  pixels = *p;

  WriteTBit0:
   if (pixels&(1<<7)) LCD_SetPoint(x+0, y, Index1);
    if (!--xsize)
      return;
  WriteTBit1:
    if (pixels&(1<<6)) LCD_SetPoint(x+1, y, Index1);
    if (!--xsize)
      return;
  WriteTBit2:
    if (pixels&(1<<5)) LCD_SetPoint(x+2, y, Index1);
    if (!--xsize)
      return;
  WriteTBit3:
    if (pixels&(1<<4)) LCD_SetPoint(x+3, y, Index1);
    if (!--xsize)
      return;
  WriteTBit4:
    if (pixels&(1<<3)) LCD_SetPoint(x+4, y, Index1);
    if (!--xsize)
      return;
  WriteTBit5:
    if (pixels&(1<<2)) LCD_SetPoint(x+5, y, Index1);
    if (!--xsize)
      return;
  WriteTBit6:
    if (pixels&(1<<1)) LCD_SetPoint(x+6, y, Index1);
    if (!--xsize)
      return;
  WriteTBit7:
    if (pixels&(1<<0)) LCD_SetPoint(x+7, y, Index1);
    if (!--xsize)
      return;
    x+=8;
    pixels = *(++p);
    goto WriteTBit0;
	
}

void LCD_L0_DrawBitmap   (int x0, int y0,
                       int xsize, int ysize,
                       int BitsPerPixel, 
                       int BytesPerLine,
                       const uint8_t* pData, int Diff,
                       const LCD_PIXELINDEX* pTrans)
{
  int i;
  switch (BitsPerPixel)
  {
  case 1:
    for (i=0; i<ysize; i++)
    {
      DrawBitLine1BPP(x0, i+y0, pData, Diff, xsize, pTrans);
      pData += BytesPerLine;
    }
    break;
  }
}

void LCD_L0_SetLUTEntry(uint8_t Pos, LCD_COLOR color)
{
}

/*******************************************************************************
* Function Name  : LCD_CtrlLinesConfig
* Description    : Configures LCD Control lines (FSMC Pins) in alternate function
                   Push-Pull mode.
* Input          : None
* Output         : None
* Return         : None
* Attention		 : None
*******************************************************************************/
static void LCD_CtrlLinesConfig(void)
{
	GPIO_InitTypeDef GPIO_InitStructure; 
	
	/* Enable GPIOs clock */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD | RCC_AHB1Periph_GPIOE , ENABLE);
	
	/* Enable FSMC clock */
	RCC_AHB3PeriphClockCmd(RCC_AHB3Periph_FSMC, ENABLE); 

	/*-- GPIOs Configuration ------------------------------------------------------*/
	/*
	+-------------------+--------------------+------------------+------------------+
	+                       SRAM pins assignment                                   +

	+-------------------+--------------------+
	*/
	/* GPIOD configuration */
		/* Set PD.00(D2), PD.01(D3), PD.04(NOE), PD.05(NWE), PD.08(D13), PD.09(D14),
	 PD.10(D15), PD.11(A16), PD.14(D0), PD.15(D1) as alternate function push pull */
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource0, GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource1, GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource4, GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource5, GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource8, GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource9, GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource10, GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource11, GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource14, GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource15, GPIO_AF_FSMC);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_4 | GPIO_Pin_5 | 
	                            GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11 |
								 GPIO_Pin_14 | GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
	
	GPIO_Init(GPIOD, &GPIO_InitStructure);
	
	/* Set PE.07(D4), PE.08(D5), PE.09(D6), PE.10(D7), PE.11(D8), PE.12(D9), PE.13(D10),
	 PE.14(D11), PE.15(D12) as alternate function push pull */	
	/* GPIOE configuration */
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource2 , GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource7 , GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource8 , GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource9 , GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource10 , GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource11 , GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource12 , GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource13 , GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource14 , GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource15 , GPIO_AF_FSMC);
	
	GPIO_InitStructure.GPIO_Pin =GPIO_Pin_2 | GPIO_Pin_7 | GPIO_Pin_8  | GPIO_Pin_9  | GPIO_Pin_10 |
	                             GPIO_Pin_11| GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
	
	GPIO_Init(GPIOE, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
	GPIO_Init(GPIOD, &GPIO_InitStructure);
	GPIO_ResetBits(GPIOD,GPIO_Pin_7);
}

/*******************************************************************************
* Function Name  : LCD_FSMCConfig
* Description    : Configures the Parallel interface (FSMC) for LCD(Parallel mode)
* Input          : None
* Output         : None
* Return         : None
* Attention		 : None
*******************************************************************************/
static void LCD_FSMCConfig(void)
{
	FSMC_NORSRAMInitTypeDef  FSMC_NORSRAMInitStructure;
	FSMC_NORSRAMTimingInitTypeDef FSMC_NORSRAMTimingInitStructure;
	
	FSMC_NORSRAMInitStructure.FSMC_Bank = FSMC_Bank1_NORSRAM1;
	FSMC_NORSRAMInitStructure.FSMC_DataAddressMux = FSMC_DataAddressMux_Disable;
	FSMC_NORSRAMInitStructure.FSMC_MemoryType = FSMC_MemoryType_SRAM;
	FSMC_NORSRAMInitStructure.FSMC_MemoryDataWidth = FSMC_MemoryDataWidth_16b;
	FSMC_NORSRAMInitStructure.FSMC_BurstAccessMode = FSMC_BurstAccessMode_Disable;
	FSMC_NORSRAMInitStructure.FSMC_WaitSignalPolarity = FSMC_WaitSignalPolarity_Low;
	FSMC_NORSRAMInitStructure.FSMC_WrapMode = FSMC_WrapMode_Disable;
	FSMC_NORSRAMInitStructure.FSMC_WaitSignalActive = FSMC_WaitSignalActive_BeforeWaitState;
	FSMC_NORSRAMInitStructure.FSMC_WriteOperation = FSMC_WriteOperation_Enable;
	FSMC_NORSRAMInitStructure.FSMC_WaitSignal = FSMC_WaitSignal_Disable;
	FSMC_NORSRAMInitStructure.FSMC_AsynchronousWait = FSMC_AsynchronousWait_Disable;
	FSMC_NORSRAMInitStructure.FSMC_ExtendedMode = FSMC_ExtendedMode_Disable;
	FSMC_NORSRAMInitStructure.FSMC_WriteBurst = FSMC_WriteBurst_Disable;
	FSMC_NORSRAMInitStructure.FSMC_ReadWriteTimingStruct = &FSMC_NORSRAMTimingInitStructure;
//	FSMC_NORSRAMInit(&FSMC_NORSRAMInitStructure); 


	/* FSMC写速度设置 */
	FSMC_NORSRAMTimingInitStructure.FSMC_AddressSetupTime = 15;//1;   /* 地址建立时间  */
	FSMC_NORSRAMTimingInitStructure.FSMC_AddressHoldTime = 0;	   
	FSMC_NORSRAMTimingInitStructure.FSMC_DataSetupTime = 15;//1;	   /* 数据建立时间  */
	FSMC_NORSRAMTimingInitStructure.FSMC_BusTurnAroundDuration = 0x00;
	FSMC_NORSRAMTimingInitStructure.FSMC_CLKDivision = 0x00;
	FSMC_NORSRAMTimingInitStructure.FSMC_DataLatency = 0x00;
	FSMC_NORSRAMTimingInitStructure.FSMC_AccessMode = FSMC_AccessMode_A;	/* FSMC 访问模式 */
	FSMC_NORSRAMInitStructure.FSMC_WriteTimingStruct = &FSMC_NORSRAMTimingInitStructure;	  
	
	FSMC_NORSRAMInit(&FSMC_NORSRAMInitStructure); 
	
	/* Enable FSMC Bank4_SRAM Bank */
	FSMC_NORSRAMCmd(FSMC_Bank1_NORSRAM1, ENABLE);  
}

/*******************************************************************************
* Function Name  : LCD_Configuration
* Description    : Configure the LCD Control pins and FSMC Parallel interface
* Input          : None
* Output         : None
* Return         : None
* Attention		 : None
*******************************************************************************/
static void LCD_Configuration(void)
{
	u32 i=0x1fffff;
	/* Configure the LCD Control pins --------------------------------------------*/
	LCD_CtrlLinesConfig();
	while(i--);
	/* Configure the FSMC Parallel interface -------------------------------------*/
	LCD_FSMCConfig();
}

void LCD_Initializtion(void)
{
	LCD_Configuration();

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

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);
 
	TIM_TimeBaseInitTypeDef timerInitStructure; 
	timerInitStructure.TIM_Prescaler = (SystemCoreClock / 4) / 1000;
	timerInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	timerInitStructure.TIM_Period = 200;
	timerInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	timerInitStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM5, &timerInitStructure);
	TIM_Cmd(TIM5, ENABLE);
	
	TIM_ITConfig(TIM5, TIM_IT_Update, ENABLE);
	
	NVIC_InitTypeDef nvicStructure;
	nvicStructure.NVIC_IRQChannel = TIM5_IRQn;
	nvicStructure.NVIC_IRQChannelPreemptionPriority = 0;
	nvicStructure.NVIC_IRQChannelSubPriority = 1;
	nvicStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvicStructure);
}

void TIM5_IRQHandler(void)
{
	/*LCD_SetCursor(0,0);
	LCD_WriteIndex(0x0022);
	LCD_RAM = 0;*/
	if (TIM_GetITStatus(TIM5, TIM_IT_Update) != RESET)
	{
		TIM_ClearITPendingBit(TIM5, TIM_IT_Update);
		LCD_fill_mem();
	}
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
	{
		return;
	}
	//LCD_SetCursor(Xpos,Ypos);
	//LCD_WriteReg(0x0022,point);
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
	MX_BIT_5_ON();
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
	MX_BIT_5_OFF();
}