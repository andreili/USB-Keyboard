#ifndef _KBD_MATRIX_H_
#define _KBD_MATRIX_H_
#include <inttypes.h>

#define MX_BIT_0_ON()		GPIOC->BSRRL = GPIO_Pin_4
#define MX_BIT_0_OFF()	GPIOC->BSRRH = GPIO_Pin_4
#define MX_BIT_1_ON()		GPIOC->BSRRL = GPIO_Pin_5
#define MX_BIT_1_OFF()	GPIOC->BSRRH = GPIO_Pin_5
#define MX_BIT_2_ON()		GPIOC->BSRRL = GPIO_Pin_6
#define MX_BIT_2_OFF()	GPIOC->BSRRH = GPIO_Pin_6
#define MX_BIT_3_ON()		GPIOC->BSRRL = GPIO_Pin_7
#define MX_BIT_3_OFF()	GPIOC->BSRRH = GPIO_Pin_7
#define MX_BIT_4_ON()		GPIOC->BSRRL = GPIO_Pin_8
#define MX_BIT_4_OFF()	GPIOC->BSRRH = GPIO_Pin_8
#define MX_BIT_5_ON()		GPIOC->BSRRL = GPIO_Pin_9
#define MX_BIT_5_OFF()	GPIOC->BSRRH = GPIO_Pin_9
#define MX_BIT_6_ON()		GPIOC->BSRRL = GPIO_Pin_10
#define MX_BIT_6_OFF()	GPIOC->BSRRH = GPIO_Pin_10
#define MX_BIT_7_ON()		GPIOC->BSRRL = GPIO_Pin_11
#define MX_BIT_7_OFF()	GPIOC->BSRRH = GPIO_Pin_11
#define MX_BIT_8_ON()		GPIOC->BSRRL = GPIO_Pin_12
#define MX_BIT_8_OFF()	GPIOC->BSRRH = GPIO_Pin_12
#define MX_BIT_9_ON()		GPIOC->BSRRL = GPIO_Pin_13
#define MX_BIT_9_OFF()	GPIOC->BSRRH = GPIO_Pin_13
#define MX_BIT_10_ON()	GPIOC->BSRRL = GPIO_Pin_14
#define MX_BIT_10_OFF()	GPIOC->BSRRH = GPIO_Pin_14
#define MX_BIT_11_ON()	GPIOC->BSRRL = GPIO_Pin_15
#define MX_BIT_11_OFF()	GPIOC->BSRRH = GPIO_Pin_15

#define KBD_MATRIX_ROW 12
#define KBD_MATRIX_COL 12

#define SW_MODE_RK86		0x01
#define SW_MODE_MC7007	0x02

extern uint16_t kbd_data[KBD_MATRIX_ROW];

uint16_t proc_row(uint16_t col_data);
void init_matrix(void);
void fill_matrix(uint32_t mode);
void proc_matrix(void);

#endif
