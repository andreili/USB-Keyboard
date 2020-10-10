#include "mod_orion.h"
#include "kbd_global.h"

void matrix_proc(void);

#define LED_3_ON()    	GPIOD->BSRR = GPIO_BSRR_BS3
#define LED_3_OFF()    	GPIOD->BSRR = GPIO_BSRR_BR3

typedef enum
{
	KBD_MC7007,
	KBD_RK86,
} EKbdType;

#define READ_DBUS() GPIOE->IDR
#define WRITE_DBUS(data) GPIOE->ODR = data;

#define BIT_BAND_SRAM(RAM,BIT) (*(volatile uint32_t*)(SRAM_BB_BASE+32*((uint32_t)((void*)(RAM))-SRAM_BASE)+4*((uint32_t)(BIT))))

/*
PINS:
	+------------+---------+--------+--------+----------+
	| ZXMC board | MCU pin | Wire # | Signal | FPGA pin |
	+------------+---------+--------+--------+----------+
	|    ZA0     |   PE0   |    1   |   D0   |   AB17   |
	|    ZA1     |   PE1   |    2   |   D1   |   AA21   |
	|    ZA2     |   PE2   |    3   |   D2   |   AB21   |
	|    ZA3     |   PE3   |    4   |   D3   |   AC23   |
	|    INT     |   PD0   |    5   |  INT   |   AD24   |
	|    NMI     |   PD1   |    6   |   WR   |   AE23   |
	|    MREQ    |   PD7   |    7   |   RD   |   AE24   |
	|    IORQ    |   PD4   |    8   |  IDX0  |   AF25   |
	|    ZRD     |   PD5   |    9   |  IDX1  |   AF26   |
	|    ZWR     |   PD6   |   10   |  IDX2  |   AG25   |
	|    ZA7     |   PE7   |   11   |   D7   |   AG26   |
	|    ZA6     |   PE6   |   12   |   D6   |   AH24   |
	|    ZA5     |   PE5   |   13   |   D5   |   AH27   |
	|    ZA4     |   PE4   |   14   |   D4   |   AJ27   |
	|    ZM1     |   PB10  |   15   |   OK   |   AK29   |
	|    ZA8     |   PE8   |   16   |   D8   |   AK28   |
	|    ZA9     |   PE9   |   17   |   D9   |   AK27   |
	|    ZA10    |   PE10  |   18   |  D10   |   AJ26   |
	+------------+---------+--------+--------+----------+
*/

#define WRN_PORT ZNMI_GPIO_Port
#define WRN_PIN (uint32_t)ZNMI_Pin
#define RDN_PORT ZMREQ_GPIO_Port
#define RDN_PIN (uint32_t)ZMREQ_Pin
#define OK_PORT ZM1_GPIO_Port
#define OK_PIN (uint32_t)ZM1_Pin
#define IDX_PORT ZIORQ_GPIO_Port
#define IDX_PIN_NO (uint32_t)4

#define WRN_0() WRN_PORT->BSRR = (WRN_PIN << GPIO_BSRR_BR0_Pos)
#define WRN_1() WRN_PORT->BSRR = WRN_PIN
#define RDN_0() RDN_PORT->BSRR = (RDN_PIN << GPIO_BSRR_BR0_Pos)
#define RDN_1() RDN_PORT->BSRR = RDN_PIN
#define OK_0() OK_PORT->BSRR = (OK_PIN << GPIO_BSRR_BR0_Pos)
#define OK_1() OK_PORT->BSRR = OK_PIN
//#define WR_IDX(idx) IDX_PORT->ODR = (IDX_PORT->ODR & (0x07 << IDX_PIN_NO)) | (idx << IDX_PIN_NO)
#define WR_IDX(idx) IDX_PORT->BSRR = (((uint32_t)idx << IDX_PIN_NO) | (((~(uint32_t)idx) & 0x07) << (IDX_PIN_NO + 16)));

#define RDN_STROBE_0() \
	RDN_0(); \
	__asm__("nop"); \
	__asm__("nop"); \
	__asm__("nop"); \
	__asm__("nop"); \
	__asm__("nop"); \
	__asm__("nop"); \
	__asm__("nop");
#define WRN_PULSE() \
	WRN_0(); \
	__asm__("nop"); \
	__asm__("nop"); \
	__asm__("nop"); \
	__asm__("nop"); \
	__asm__("nop"); \
	__asm__("nop"); \
	__asm__("nop"); \
	WRN_1(); 

#define VECTOR_MASK_ADL 0x03
#define VECTOR_MASK_KBD 0x04
#define VECTOR_MASK_WRN	0x80

uint8_t vv_regs[4];
#define VV_MODE_PORT_C_LO_INPUT(data) 	(BIT_BAND_SRAM(data, 0))
#define VV_MODE_PORT_B_INPUT(data) 		(BIT_BAND_SRAM(data, 1))
#define VV_MODE_B_C_LO_MODE(data) 			(BIT_BAND_SRAM(data, 2))
#define VV_MODE_PORT_C_UP_INPUT(data) 	(BIT_BAND_SRAM(data, 3))
#define VV_MODE_PORT_A_INPUT(data)			(BIT_BAND_SRAM(data, 4))
#define VV_MODE_A_C_HI_MODE0(data) 			(BIT_BAND_SRAM(data, 5))
#define VV_MODE_A_C_HI_MODE1(data) 			(BIT_BAND_SRAM(data, 6))
#define VV_MODE_CONTROL_BYTE(data) 			(BIT_BAND_SRAM(data, 7))

#define VV_ROW(row) BIT_BAND_SRAM(vv_regs[1], row)
#define VV_COL(col) BIT_BAND_SRAM(vv_regs[0], col)
#define KBD_CELL(row, col) BIT_BAND_SRAM(kbd_data[row], col)

void orion_init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct;
	
  HAL_GPIO_DeInit(GPIOE, ZA0_Pin|ZA1_Pin);

  GPIO_InitStruct.Pin = ZA0_Pin|ZA1_Pin|ZA2_Pin|ZA3_Pin|ZA4_Pin|ZA5_Pin|
												ZA6_Pin|ZA7_Pin|ZA8_Pin|ZA9_Pin|ZA10_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = ZNMI_Pin|ZMREQ_Pin|ZIORQ_Pin|ZRD_Pin|ZWR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = ZM1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = ZINT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

	HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

	WRN_1();
	RDN_1();
	OK_0();
	WR_IDX(7); // default - connected directly to data bus (if RDN=0)
}

void proc_mtx(void);

void orion_proc_loop()
{
	matrix_proc();
	
	for (int i=0 ; i<8 ; ++i)
	{
		WR_IDX(i);
		WRITE_DBUS(kbd_data[i]);
		__disable_irq();
		WRN_PULSE();
		__enable_irq();
	}
	WRITE_DBUS(0x7ff);
}

void orion_proc()
{
	RDN_STROBE_0();
	RDN_1();
	/*uint32_t rdata;
	uint16_t vector;
	GPIOE->ODR = 0xffff;
	vector = READ_DBUS();
	uint8_t addr_lo = vector & VECTOR_MASK_ADL;
	WR_IDX(addr_lo);
	RDN_STROBE_0();
	rdata = READ_DBUS();
	RDN_1();*/
}

const kbd_proc_t proc_orion =
{
	.init = orion_init,
	.proc = orion_proc_loop,
	.interrupt = orion_proc,
	.periodic = NULL
};
