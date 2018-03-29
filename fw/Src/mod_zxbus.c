#include "mod_zxbus.h"
#include "kbd_global.h"

#define LED_3_ON()    	GPIOD->BSRR = GPIO_BSRR_BS3
#define LED_3_OFF()    	GPIOD->BSRR = GPIO_BSRR_BR3

#define ZXBUS_PORT							GPIOE
#define ZXBUS_PIN_IORQ					2
#define ZXBUS_PIN_RD						3
#define ZXBUS_PIN_WR						4

#define ZXBUS_DIR_PORT					GPIOC
#define ZXBUS_PIN_DIR						3
#define ZXBUS_PIN_OE						2

#define ZXBUS_DATA_PORT					GPIOB
#define ZXBUS_DATA_MASK					0x0F
#define ZXBUS_DATA_OFFS					0

#define ZXBUS_PORT_ADDR_KBD			0xFE
#define ZXBUS_PORT_ADDR_KJOY		0xFF

#define MTX5_Pin GPIO_PIN_13
#define MTX5_GPIO_Port GPIOC
#define MTX6_Pin GPIO_PIN_14
#define MTX6_GPIO_Port GPIOC
#define MTX7_Pin GPIO_PIN_15
#define MTX7_GPIO_Port GPIOC
#define FS_PWR_Pin GPIO_PIN_0
#define FS_PWR_GPIO_Port GPIOC
#define PS2_CLK_Pin GPIO_PIN_2
#define PS2_CLK_GPIO_Port GPIOC
#define PS2_DAT_Pin GPIO_PIN_3
#define PS2_DAT_GPIO_Port GPIOC
#define M1n_Pin GPIO_PIN_0
#define M1n_GPIO_Port GPIOA
#define MTX0_Pin GPIO_PIN_3
#define MTX0_GPIO_Port GPIOA
#define MTX1_Pin GPIO_PIN_4
#define MTX1_GPIO_Port GPIOA
#define MTX2_Pin GPIO_PIN_5
#define MTX2_GPIO_Port GPIOA
#define MTX3_Pin GPIO_PIN_6
#define MTX3_GPIO_Port GPIOA
#define IORQn_Pin GPIO_PIN_0
#define IORQn_GPIO_Port GPIOB
#define RDn_Pin GPIO_PIN_1
#define RDn_GPIO_Port GPIOB
#define WRn_Pin GPIO_PIN_2
#define WRn_GPIO_Port GPIOB
#define WRn_EXTI_IRQn EXTI2_IRQn
#define A0_Pin GPIO_PIN_0
#define A0_GPIO_Port GPIOE
#define A1_Pin GPIO_PIN_1
#define A1_GPIO_Port GPIOE
#define A2_Pin GPIO_PIN_2
#define A2_GPIO_Port GPIOE
#define A3_Pin GPIO_PIN_3
#define A3_GPIO_Port GPIOE
#define A4_Pin GPIO_PIN_4
#define A4_GPIO_Port GPIOE
#define A5_Pin GPIO_PIN_5
#define A5_GPIO_Port GPIOE
#define A6_Pin GPIO_PIN_6
#define A6_GPIO_Port GPIOE
#define A7_Pin GPIO_PIN_7
#define A7_GPIO_Port GPIOE
#define NMIn_Pin GPIO_PIN_10
#define NMIn_GPIO_Port GPIOB
#define A8_Pin GPIO_PIN_8
#define A8_GPIO_Port GPIOD
#define A9_Pin GPIO_PIN_9
#define A9_GPIO_Port GPIOD
#define A10_Pin GPIO_PIN_10
#define A10_GPIO_Port GPIOD
#define A11_Pin GPIO_PIN_11
#define A11_GPIO_Port GPIOD
#define A12_Pin GPIO_PIN_12
#define A12_GPIO_Port GPIOD
#define A13_Pin GPIO_PIN_13
#define A13_GPIO_Port GPIOD
#define A14_Pin GPIO_PIN_14
#define A14_GPIO_Port GPIOD
#define A15_Pin GPIO_PIN_15
#define A15_GPIO_Port GPIOD
#define FS_PWR_REL_Pin GPIO_PIN_8
#define FS_PWR_REL_GPIO_Port GPIOA
#define HS_PWR_Pin GPIO_PIN_9
#define HS_PWR_GPIO_Port GPIOA
#define ZB_DIR_Pin GPIO_PIN_10
#define ZB_DIR_GPIO_Port GPIOA
#define LED1_Pin GPIO_PIN_0
#define LED1_GPIO_Port GPIOD
#define LED2_Pin GPIO_PIN_1
#define LED2_GPIO_Port GPIOD
#define SD_CD_Pin GPIO_PIN_3
#define SD_CD_GPIO_Port GPIOD
#define LED3_Pin GPIO_PIN_4
#define LED3_GPIO_Port GPIOD
#define OUT0_Pin GPIO_PIN_5
#define OUT0_GPIO_Port GPIOD
#define OUT1_Pin GPIO_PIN_6
#define OUT1_GPIO_Port GPIOD
#define OUT2_Pin GPIO_PIN_7
#define OUT2_GPIO_Port GPIOD
#define MREQn_Pin GPIO_PIN_8
#define MREQn_GPIO_Port GPIOB
#define INTn_Pin GPIO_PIN_9
#define INTn_GPIO_Port GPIOB
#define D0_Pin GPIO_PIN_8
#define D0_GPIO_Port GPIOE
#define D1_Pin GPIO_PIN_9
#define D1_GPIO_Port GPIOE
#define D2_Pin GPIO_PIN_10
#define D2_GPIO_Port GPIOE
#define D3_Pin GPIO_PIN_11
#define D3_GPIO_Port GPIOE
#define D4_Pin GPIO_PIN_12
#define D4_GPIO_Port GPIOE
#define D5_Pin GPIO_PIN_13
#define D5_GPIO_Port GPIOE
#define D6_Pin GPIO_PIN_14
#define D6_GPIO_Port GPIOE
#define D7_Pin GPIO_PIN_15
#define D7_GPIO_Port GPIOE

void zxbus_init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, D2_Pin|D3_Pin|D4_Pin|D5_Pin 
                          |D6_Pin|D7_Pin|D0_Pin|D1_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, M1n_Pin|ZB_DIR_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOB, IORQn_Pin|RDn_Pin|WRn_Pin|NMIn_Pin|MREQn_Pin|INTn_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(ZXBUS_DIR_PORT, ZXBUS_PIN_DIR | ZXBUS_PIN_OE, GPIO_PIN_SET);

  /*Configure GPIO pins : D2_Pin D3_Pin D4_Pin D5_Pin 
                           D6_Pin D7_Pin D0_Pin D1_Pin */
  GPIO_InitStruct.Pin = D2_Pin|D3_Pin|D4_Pin|D5_Pin 
                          |D6_Pin|D7_Pin|D0_Pin|D1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : M1n_Pin */
  GPIO_InitStruct.Pin = M1n_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(M1n_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : M1n_Pin */
  GPIO_InitStruct.Pin = ZXBUS_PIN_DIR | ZXBUS_PIN_OE;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(ZXBUS_DIR_PORT, &GPIO_InitStruct);

  /*Configure GPIO pins : IORQn_Pin RDn_Pin WRn_Pin NMIn_Pin 
                           MREQn_Pin INTn_Pin */
  GPIO_InitStruct.Pin = IORQn_Pin|RDn_Pin|WRn_Pin|NMIn_Pin 
                          |MREQn_Pin|INTn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : A0_Pin A1_Pin A2_Pin A3_Pin 
                           A4_Pin A5_Pin A6_Pin A7_Pin */
  GPIO_InitStruct.Pin = A0_Pin|A1_Pin|A2_Pin|A3_Pin 
                          |A4_Pin|A5_Pin|A6_Pin|A7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : A8_Pin A9_Pin A10_Pin A11_Pin 
                           A12_Pin A13_Pin A14_Pin A15_Pin 
                           OUT0_Pin OUT1_Pin SD_CD_Pin OUT2_Pin 
                           OUT3_Pin OUT4_Pin OUT5_Pin */
  GPIO_InitStruct.Pin = A8_Pin|A9_Pin|A10_Pin|A11_Pin 
                          |A12_Pin|A13_Pin|A14_Pin|A15_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);
	
	/* reset buffer OE pin */
	ZXBUS_DIR_PORT->BSRR = (ZXBUS_PIN_OE << GPIO_BSRR_BR0_Pos);
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

#define ZXBUS_IORD() ((ZXBUS_PORT->IDR & ((1 << ZXBUS_PIN_RD) | (1 << ZXBUS_PIN_IORQ))) == 0)
#define ZXBUS_IOWR() ((ZXBUS_PORT->IDR & ((1 << ZXBUS_PIN_WR) | (1 << ZXBUS_PIN_IORQ))) == 0)

void zxbus_proc()
{
	// set buffer direction
	if ((ZXBUS_PORT->IDR & (1 << ZXBUS_PIN_WR)) == 0)
		// set bit - write mode
		ZXBUS_DIR_PORT->BSRR = (ZXBUS_PIN_DIR);
	else
		// reset bit - read mode
		ZXBUS_DIR_PORT->BSRR = (ZXBUS_PIN_DIR << GPIO_BSRR_BR0_Pos);

	if (ZXBUS_IORD())
		zxbus_proc_int_rd();
	if (ZXBUS_IOWR())
		zxbus_proc_int_wr();
}

const kbd_proc_t proc_zxbus =
{
	.init = zxbus_init,
	.proc = NULL,
	.interrupt = zxbus_proc,
	.periodic = NULL
};
