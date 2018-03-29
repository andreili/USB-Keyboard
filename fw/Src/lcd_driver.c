#include "lcd_driver.h"
#include "cmsis_os.h"
#include "kbd_matrix.h"
#include "AsciiLib.h"

// ILI9325

extern TIM_HandleTypeDef htim4;

#define LCD_REG              (*((volatile unsigned short *) 0x60000000)) /* RS = 0 */
#define LCD_RAM              (*((volatile unsigned short *) 0x60020000)) /* RS = 1 */

#define   BKCOLOR LCD_BKCOLORINDEX
#define   COLOR LCD_COLORINDEX

uint16_t lcd_buff[MAX_X][BUF_MAX_Y];
SRAM_HandleTypeDef hsram1;

void LCD_LL_init(void)
{
  FSMC_NORSRAM_TimingTypeDef Timing;

  /** Perform the SRAM1 memory initialization sequence
  */
  hsram1.Instance = FSMC_NORSRAM_DEVICE;
  hsram1.Extended = FSMC_NORSRAM_EXTENDED_DEVICE;
  /* hsram1.Init */
  hsram1.Init.NSBank = FSMC_NORSRAM_BANK1;
  hsram1.Init.DataAddressMux = FSMC_DATA_ADDRESS_MUX_DISABLE;
  hsram1.Init.MemoryType = FSMC_MEMORY_TYPE_SRAM;
  hsram1.Init.MemoryDataWidth = FSMC_NORSRAM_MEM_BUS_WIDTH_16;
  hsram1.Init.BurstAccessMode = FSMC_BURST_ACCESS_MODE_DISABLE;
  hsram1.Init.WaitSignalPolarity = FSMC_WAIT_SIGNAL_POLARITY_LOW;
  hsram1.Init.WrapMode = FSMC_WRAP_MODE_DISABLE;
  hsram1.Init.WaitSignalActive = FSMC_WAIT_TIMING_BEFORE_WS;
  hsram1.Init.WriteOperation = FSMC_WRITE_OPERATION_ENABLE;
  hsram1.Init.WaitSignal = FSMC_WAIT_SIGNAL_DISABLE;
  hsram1.Init.ExtendedMode = FSMC_EXTENDED_MODE_DISABLE;
  hsram1.Init.AsynchronousWait = FSMC_ASYNCHRONOUS_WAIT_DISABLE;
  hsram1.Init.WriteBurst = FSMC_WRITE_BURST_DISABLE;
  hsram1.Init.PageSize = FSMC_PAGE_SIZE_NONE;
  /* Timing */
  Timing.AddressSetupTime = 8;
  Timing.AddressHoldTime = 1;
  Timing.DataSetupTime = 8;
  Timing.BusTurnAroundDuration = 0;
  Timing.CLKDivision = 0;
  Timing.DataLatency = 0;
  Timing.AccessMode = FSMC_ACCESS_MODE_A;
  /* ExtTiming */

  if (HAL_SRAM_Init(&hsram1, &Timing, NULL) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
}

static uint32_t FSMC_Initialized = 0;

static void HAL_FSMC_MspInit(void){
  /* USER CODE BEGIN FSMC_MspInit 0 */

  /* USER CODE END FSMC_MspInit 0 */
  GPIO_InitTypeDef GPIO_InitStruct;
  if (FSMC_Initialized) {
    return;
  }
  FSMC_Initialized = 1;
  /* Peripheral clock enable */
  __HAL_RCC_FSMC_CLK_ENABLE();
  
  /** FSMC GPIO Configuration  
  PE7   ------> FSMC_D4
  PE8   ------> FSMC_D5
  PE9   ------> FSMC_D6
  PE10   ------> FSMC_D7
  PE11   ------> FSMC_D8
  PE12   ------> FSMC_D9
  PE13   ------> FSMC_D10
  PE14   ------> FSMC_D11
  PE15   ------> FSMC_D12
  PD8   ------> FSMC_D13
  PD9   ------> FSMC_D14
  PD10   ------> FSMC_D15
  PD11   ------> FSMC_A16
  PD14   ------> FSMC_D0
  PD15   ------> FSMC_D1
  PD0   ------> FSMC_D2
  PD1   ------> FSMC_D3
  PD4   ------> FSMC_NOE
  PD5   ------> FSMC_NWE
  PD7   ------> FSMC_NE1
  */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  GPIO_InitStruct.Pin = GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10 
                          |GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14 
                          |GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF12_FSMC;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11 
                          |GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_0|GPIO_PIN_1 
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF12_FSMC;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* USER CODE BEGIN FSMC_MspInit 1 */

  /* USER CODE END FSMC_MspInit 1 */
}

void HAL_SRAM_MspInit(SRAM_HandleTypeDef* hsram){
  /* USER CODE BEGIN SRAM_MspInit 0 */

  /* USER CODE END SRAM_MspInit 0 */
  HAL_FSMC_MspInit();
  /* USER CODE BEGIN SRAM_MspInit 1 */

  /* USER CODE END SRAM_MspInit 1 */
}

static uint32_t FSMC_DeInitialized = 0;

static void HAL_FSMC_MspDeInit(void){
  /* USER CODE BEGIN FSMC_MspDeInit 0 */

  /* USER CODE END FSMC_MspDeInit 0 */
  if (FSMC_DeInitialized) {
    return;
  }
  FSMC_DeInitialized = 1;
  /* Peripheral clock enable */
  __HAL_RCC_FSMC_CLK_DISABLE();
  
  /** FSMC GPIO Configuration  
  PE7   ------> FSMC_D4
  PE8   ------> FSMC_D5
  PE9   ------> FSMC_D6
  PE10   ------> FSMC_D7
  PE11   ------> FSMC_D8
  PE12   ------> FSMC_D9
  PE13   ------> FSMC_D10
  PE14   ------> FSMC_D11
  PE15   ------> FSMC_D12
  PD8   ------> FSMC_D13
  PD9   ------> FSMC_D14
  PD10   ------> FSMC_D15
  PD11   ------> FSMC_A16
  PD14   ------> FSMC_D0
  PD15   ------> FSMC_D1
  PD0   ------> FSMC_D2
  PD1   ------> FSMC_D3
  PD4   ------> FSMC_NOE
  PD5   ------> FSMC_NWE
  PD7   ------> FSMC_NE1
  */
  HAL_GPIO_DeInit(GPIOE, GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10 
                          |GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14 
                          |GPIO_PIN_15);

  HAL_GPIO_DeInit(GPIOD, GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11 
                          |GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_0|GPIO_PIN_1 
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_7);

  /* USER CODE BEGIN FSMC_MspDeInit 1 */

  /* USER CODE END FSMC_MspDeInit 1 */
}

void HAL_SRAM_MspDeInit(SRAM_HandleTypeDef* hsram){
  /* USER CODE BEGIN SRAM_MspDeInit 0 */

  /* USER CODE END SRAM_MspDeInit 0 */
  HAL_FSMC_MspDeInit();
  /* USER CODE BEGIN SRAM_MspDeInit 1 */

  /* USER CODE END SRAM_MspDeInit 1 */
}

void LCD_Initialization(void)
{
	LCD_LL_init();
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
