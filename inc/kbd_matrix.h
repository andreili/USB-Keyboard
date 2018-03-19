#ifndef _KBD_MATRIX_H_
#define _KBD_MATRIX_H_
#include <inttypes.h>

#define PORT_INP GPIOB
#define RCCR_INP RCC_AHB1Periph_ ## GPIOB
#define PORT_OUT GPIOC
#define RCCR_OUT RCC_AHB1Periph_ ## GPIOC

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
