#include "ps2.h"
#include "main.h"

typedef enum { IDLE, SEND, REQUEST, RECEIVE } PS2_StateTypeDef;
typedef enum { START, DATA, PARITY, STOP, ACK, FINISHED } PS2_TransferStateTypeDef;

#define BITBAND_SRAM_REF   0x20000000
#define BITBAND_SRAM_BASE  0x22000000
#define BITBAND_SRAM(ptr,n) ((volatile uint32_t*)((BITBAND_SRAM_BASE + \
                                (((uint32_t)ptr)-BITBAND_SRAM_REF)*32 + (n*4))))
#define BITBAND_PERI_REF   0x40000000
#define BITBAND_PERI_BASE  0x42000000
#define BITBAND_PERI(ptr,n) ((volatile uint32_t*)((BITBAND_PERI_BASE + \
                                (((uint32_t)ptr)-BITBAND_PERI_REF)*32 + (n*4))))

extern TIM_HandleTypeDef htim2;

PS2_StateTypeDef PS2_State = IDLE;
PS2_TransferStateTypeDef PS2_TransferState = START;

uint8_t PS2_SendRequest = RESET;
uint8_t PS2_OutputData = 0x00;
uint8_t PS2_OutputBitNr = 0;

uint8_t PS2_InputData = 0;
uint8_t PS2_InputBitNr = 0;

uint8_t PS2_Parity = 0;

PS2_TransferStateTypeDef PS2_SendDataIRQHandler(void)
{
	uint8_t PS2_DataBit = 0;
	PS2_TransferStateTypeDef PS2_NextState = PS2_TransferState;

	switch(PS2_TransferState) 
	{
	case START:
		/* Initalize  */
		PS2_OutputBitNr = 0;
		PS2_Parity = 0;

		/* Send START Bit */
		PS2_DataBit = 0;
		PS2_NextState = DATA;
		break;
	case DATA:
		/*  next DATA Bit */
		PS2_DataBit = *(BITBAND_SRAM(&PS2_OutputData, PS2_OutputBitNr));
		PS2_OutputBitNr++;

		/* Calculate Paritiy */
		PS2_Parity ^= PS2_DataBit;
		if(PS2_OutputBitNr > 7) 
		{
			/* Finalize paritiy bit */
			PS2_Parity = !PS2_Parity;
			PS2_NextState = PARITY;
		}
		break;
	case PARITY:
		/* Send PARITY Bit */
		PS2_DataBit = PS2_Parity;
		PS2_NextState = STOP;
		break;
	case STOP:
		/* Send STOP Bit */
		PS2_DataBit = 1;
		PS2_NextState = FINISHED;
		break;
	case ACK:
	case FINISHED:
		/* This should never happen! */
		break;
	}

	/* Write output bit */
	HAL_GPIO_WritePin(PS2_DAT_GPIO_Port, PS2_DAT_Pin, PS2_DataBit);

	return PS2_NextState;
}

PS2_TransferStateTypeDef PS2_ReceiveDataIRQHandler(void)
{
	PS2_TransferStateTypeDef PS2_NextState = PS2_TransferState;
	uint8_t PS2_DataBit = 0;

	/* Read input */
	PS2_DataBit = HAL_GPIO_ReadPin(PS2_DAT_GPIO_Port, PS2_DAT_Pin);

	switch(PS2_TransferState) 
	{
	case START:
		/* Initalize */
		PS2_OutputBitNr = 0;
		PS2_Parity = 0;

		/* Check START Bit for errors */
		if(PS2_DataBit != 0) 
		{
			/* Error */ // XXX abort here
		}

		PS2_NextState = DATA;
		break;
	case DATA:
		/*  next DATA Bit */
		*(BITBAND_SRAM(&PS2_InputData, PS2_InputBitNr)) = PS2_DataBit;
		PS2_InputBitNr++;

		/* Calculate Paritiy */
		PS2_Parity ^= PS2_DataBit;
		if(PS2_InputBitNr > 7) 
		{
			/* Finalize paritiy bit */
			PS2_Parity = !PS2_Parity;
			PS2_NextState = PARITY;
		}
		break;
	case PARITY:
		if(PS2_DataBit != PS2_Parity) 
		{
			/* Error */
		}
		PS2_NextState = STOP;
		break;
	case STOP:
		if(PS2_DataBit != 1) 
		{
			/* Error */
		}
		PS2_NextState = ACK;
		break;
	case ACK:
		/* Acknoledge everything */
		HAL_GPIO_WritePin(PS2_DAT_GPIO_Port, PS2_DAT_Pin, GPIO_PIN_RESET);
		PS2_NextState = FINISHED;
		break;
	case FINISHED:
		/* Release DATA Pin */
		HAL_GPIO_WritePin(PS2_DAT_GPIO_Port, PS2_DAT_Pin, GPIO_PIN_SET);
		break;
	}

	return PS2_NextState;
}

void PS2_DataIRQHandler(void)
{
	if(PS2_State == IDLE || PS2_State == REQUEST) 
	{
		/* Nothing to do */
		return;
	}

	/* Check if the communication was canceled */
	if(HAL_GPIO_ReadPin(PS2_CLK_GPIO_Port, PS2_CLK_Pin) == GPIO_PIN_RESET) 
	{
		/* Release DATA Pin */
		HAL_GPIO_WritePin(PS2_DAT_GPIO_Port, PS2_DAT_Pin, GPIO_PIN_SET);
		PS2_State = IDLE;
		return;
	}

	if(PS2_State == SEND) 
	{
		PS2_TransferState = PS2_SendDataIRQHandler();
	} 
	else if(PS2_State == RECEIVE) 
	{
		PS2_TransferState = PS2_ReceiveDataIRQHandler();
	}
}

void PS2_CheckRequestToReceive(void)
{
	if(PS2_State == IDLE) 
	{
		/* Idle, CLK should be set, otherwise we have a receive request */
		if(HAL_GPIO_ReadPin(PS2_CLK_GPIO_Port, PS2_CLK_Pin) == GPIO_PIN_RESET) 
		{
			PS2_State = REQUEST;
		}
	} 
	else if(PS2_State == REQUEST)
	{
		/* Check if CLK is set again, then the transfer can start */
		if(HAL_GPIO_ReadPin(PS2_CLK_GPIO_Port, PS2_CLK_Pin) == GPIO_PIN_SET) 
		{
			// TODO check data here?
			PS2_State = RECEIVE;
			PS2_TransferState = START;
		}
	}
}

void PS2_CheckRequestToSend(void)
{
	if(PS2_State == IDLE && PS2_SendRequest == GPIO_PIN_SET) 
	{
		PS2_State = SEND;
		PS2_TransferState = START;
	}
}

void PS2_ClearFinishedSend(void)
{
	if(PS2_State == SEND && PS2_TransferState == FINISHED) 
	{
		PS2_State = IDLE;
		PS2_SendRequest = GPIO_PIN_RESET;
	}
}

void PS2_ClearFinishedReceive(void)
{
	if((PS2_State == RECEIVE) && (PS2_TransferState == FINISHED)) 
	{
		PS2_State = IDLE;
	}
}

void PS2_ClockIRQHandler(void)
{
	if (__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim2))
	{
		/* Counter Direction DOWN, CLK Falling Edge */

		if(PS2_State == SEND || PS2_State == RECEIVE) 
		{
			HAL_GPIO_WritePin(PS2_CLK_GPIO_Port, PS2_CLK_Pin, GPIO_PIN_RESET);
		}

		PS2_ClearFinishedReceive();
	} 
	else 
	{
		/* Counter Direction UP, CLK Rising Edge */

		PS2_CheckRequestToReceive();

		if(PS2_State == SEND || PS2_State == RECEIVE) 
		{
			HAL_GPIO_WritePin(PS2_CLK_GPIO_Port, PS2_CLK_Pin, GPIO_PIN_SET);
		}

		PS2_ClearFinishedSend();
		PS2_CheckRequestToSend();
	}
}

const GPIO_InitTypeDef GPIO_PS2_init = 
{
	.Pin = PS2_CLK_Pin|PS2_DAT_Pin,
	.Mode = GPIO_MODE_OUTPUT_OD,
	.Pull = GPIO_PULLUP,
	.Speed = GPIO_SPEED_FREQ_LOW,
	.Alternate = 0
};

void PS2_init(void)
{
  HAL_GPIO_WritePin(GPIOC, FS_PWR_Pin|PS2_CLK_Pin|PS2_DAT_Pin, GPIO_PIN_SET);
  HAL_GPIO_Init(GPIOC, &GPIO_PS2_init);
	
	HAL_TIM_OC_Start_IT(&htim2, TIM_CHANNEL_1);
	HAL_TIM_OC_Start_IT(&htim2, TIM_CHANNEL_2);
}
