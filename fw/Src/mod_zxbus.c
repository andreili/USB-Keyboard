#include "mod_zxbus.h"
#include "kbd_global.h"

#define LED_3_ON()    	GPIOD->BSRR = GPIO_BSRR_BS3
#define LED_3_OFF()    	GPIOD->BSRR = GPIO_BSRR_BR3

#define ZXBUS_PORT							GPIOE
#define ZXBUS_PIN_IORQ					2
#define ZXBUS_PIN_RD						3
#define ZXBUS_PIN_WR						4

#define ZXBUS_DATA_PORT					GPIOB
#define ZXBUS_DATA_MASK					0x0F
#define ZXBUS_DATA_OFFS					0

#define ZXBUS_PORT_ADDR_KBD			0xFE
#define ZXBUS_PORT_ADDR_KJOY		0xFF

void zxbus_init(void)
{
}

void zxbus_proc_kbd(uint8_t row)
{
	uint16_t col = ~kbd_data[row];
	col <<= ZXBUS_DATA_OFFS;	// change offset
	col |= (ZXBUS_DATA_PORT->IDR & (~ZXBUS_DATA_MASK));	// read input data and add to DB
	ZXBUS_DATA_PORT->ODR = col;	// write to output
}

void zxbus_proc_int_rd()
{
	uint8_t port_addr = 0xFE;	// Z80:A0-A7
	uint8_t accu_val = 0xFE;	// Z80:A8-A15
	
	switch (port_addr)
	{
		case ZXBUS_PORT_ADDR_KBD:
			zxbus_proc_kbd(~accu_val);
			break;
		case ZXBUS_PORT_ADDR_KJOY:
			break;
		default:
			break;
	}
}

void zxbus_proc_int_wr()
{
}

void zxbus_proc(int GPIO_Pin)
{
	switch (GPIO_Pin)
	{
		case ZXBUS_PIN_IORQ:
			if ((ZXBUS_PORT->IDR & (1 << ZXBUS_PIN_RD)) == 0)
				zxbus_proc_int_rd();
			if ((ZXBUS_PORT->IDR & (1 << ZXBUS_PIN_WR)) == 0)
				zxbus_proc_int_wr();
			break;
		case ZXBUS_PIN_RD:
			if ((ZXBUS_PORT->IDR & (1 << ZXBUS_PIN_IORQ)) == 0)
				zxbus_proc_int_rd();
			break;
		case ZXBUS_PIN_WR:
			if ((ZXBUS_PORT->IDR & (1 << ZXBUS_PIN_IORQ)) == 0)
				zxbus_proc_int_wr();
			break;
	}
}
