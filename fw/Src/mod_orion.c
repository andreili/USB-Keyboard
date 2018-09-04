#include "mod_orion.h"
#include "kbd_global.h"

void matrix_proc(void);

#define LED_3_ON()    	GPIOD->BSRR = GPIO_BSRR_BS3
#define LED_3_OFF()    	GPIOD->BSRR = GPIO_BSRR_BR3

uint16_t vector;
uint8_t addr_lo;

#define VECTOR_MASK_ADDR_LO	0x03

#define READ_DBUS() GPIOD->IDR >> 8
//#define WRITE_DBUS(data) GPIOD->ODR = (GPIOD->ODR & 0x00ff) | (data << 8)
#define WRITE_DBUS(data) GPIOD->BSRR = (((~data) << 8) | (data << 8));

#define BIT_BAND_SRAM(RAM,BIT) (*(volatile uint32_t*)(SRAM_BB_BASE+32*((uint32_t)((void*)(RAM))-SRAM_BASE)+4*((uint32_t)(BIT))))

/*
PINS:
	+------------+---------+--------+--------+----------+
	| ZXMC board | MCU pin | Wire # | Signal | FPGA pin |
	+------------+---------+--------+--------+----------+
	|    ZA15    |   PE15  |    1   |  WRN   |   AB17   |
	|    ZA14    |   PE14  |    2   |  IDX2  |   AA21   |
	|    ZA13    |   PE13  |    3   |  IDX1  |   AB21   |
	|    ZA12    |   PE12  |    4   |  IDX0  |   AC23   |
	|    ZD7     |   PD15  |    5   |   D7   |   AD24   |
	|    ZD0     |   PD8   |    6   |   D0   |   AE23   |
	|    ZD1     |   PD9   |    7   |   D1   |   AE24   |
	|    ZD2     |   PD10  |    8   |   D2   |   AF25   |
	|    ZD6     |   PD14  |    9   |   D6   |   AF26   |
	|    ZD5     |   PD13  |   10   |   D5   |   AG25   |
	|    ZD3     |   PD11  |   11   |   D3   |   AG26   |
	|    ZD4     |   PD12  |   12   |   D4   |   AH24   |
	|    ZA0     |   PE0   |   13   |  INT   |   AH27   |
	|    ZA1     |   PE1   |   14   |  RDN   |   AJ27   |
	+------------+---------+--------+--------+----------+
*/

#define WRN_PORT ZA15_GPIO_Port
#define WRN_PIN (uint32_t)ZA15_Pin
#define RDN_PORT ZA1_GPIO_Port
#define RDN_PIN (uint32_t)ZA1_Pin
#define IDX_PORT ZA12_GPIO_Port
#define IDX_PIN_NO (uint32_t)12

#define WRN_0() WRN_PORT->BSRR = (WRN_PIN << GPIO_BSRR_BR0_Pos)
#define WRN_1() WRN_PORT->BSRR = WRN_PIN
#define RDN_0() RDN_PORT->BSRR = (RDN_PIN << GPIO_BSRR_BR0_Pos)
#define RDN_1() RDN_PORT->BSRR = RDN_PIN
//#define WR_IDX(idx) IDX_PORT->ODR = (IDX_PORT->ODR & (0x07 << IDX_PIN_NO)) | (idx << IDX_PIN_NO)
#define WR_IDX(idx) IDX_PORT->BSRR = (((~(uint32_t)idx) << IDX_PIN_NO) | ((uint32_t)idx << IDX_PIN_NO));

void orion_init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct;

  GPIO_InitStruct.Pin = ZD0_Pin|ZD1_Pin|ZD2_Pin|ZD3_Pin 
                          |ZD4_Pin|ZD5_Pin|ZD6_Pin|ZD7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = ZA15_Pin|ZA1_Pin|ZA12_Pin|ZA13_Pin|ZA14_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = ZA0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

	WRN_1();
	RDN_1();
	WR_IDX(7); // default - connected directly to data bus (if RDN=0)
}

void orion_proc_loop()
{
	// Loop task - see main.c, function StartDefaultTask
	// Period - 1ms
	
	// update the keyboard matrix
	matrix_proc();
}

#define orion_proc_col(col) row_data |= kbd_data[column]

void orion_proc()
{
	// read interrupt vector
	vector = READ_DBUS();
	addr_lo = vector & VECTOR_MASK_ADDR_LO;
	
	// read column, part 1
	WR_IDX(0);
	RDN_0();
	uint16_t column =  READ_DBUS();
	
	// read column, part 2
	WR_IDX(1);
	column |= ((READ_DBUS() & 0x07) << 8);
	
	// end of reads
	RDN_1();
	
	uint8_t row_data = 0;
	
	// proceed columns
	if (BIT_BAND_SRAM(&column, 0) == 0)
		orion_proc_col(0);
	if (BIT_BAND_SRAM(&column, 1) == 0)
		orion_proc_col(1);
	if (BIT_BAND_SRAM(&column, 2) == 0)
		orion_proc_col(2);
	if (BIT_BAND_SRAM(&column, 3) == 0)
		orion_proc_col(3);
	if (BIT_BAND_SRAM(&column, 4) == 0)
		orion_proc_col(4);
	if (BIT_BAND_SRAM(&column, 5) == 0)
		orion_proc_col(5);
	if (BIT_BAND_SRAM(&column, 6) == 0)
		orion_proc_col(6);
	if (BIT_BAND_SRAM(&column, 7) == 0)
		orion_proc_col(7);
	if (matrix_mode == SW_MODE_MC7007)
	{
		if (BIT_BAND_SRAM(&column, 8) == 0)
			orion_proc_col(8);
		if (BIT_BAND_SRAM(&column, 9) == 0)
			orion_proc_col(9);
		if (BIT_BAND_SRAM(&column, 10) == 0)
			orion_proc_col(10);
	}

	// write row data to CPLD
	WR_IDX(0);
	WRN_0();
	WRITE_DBUS(row_data);
	WRN_1();
}

const kbd_proc_t proc_orion =
{
	.init = orion_init,
	.proc = orion_proc_loop,
	.interrupt = orion_proc,
	.periodic = NULL
};
