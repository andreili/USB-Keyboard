#include "ps2.h"
#include "main.h"
#include "ringbuffer.h"
#include "usbh_hid_keybd.h"

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

#define PS2_BUF_SIZE (30*4)
RingBufferU8 ps2_buf_info;
uint8_t ps2_buf[PS2_BUF_SIZE];

#define PS2_SCANCODE_INVALID 0xff
																			//0			1			2			3			4			5			6			7			8			9			a			b			c			d			e			f
const uint8_t HID_toPS2_table[256] = {0x00, 0x00, 0xfc, 0x00, 0x1c, 0x32, 0x21, 0x23, 0x24, 0x2b, 0x34, 0x33, 0x43, 0x3b, 0x42, 0x4b,		// 00
																			0x3a, 0x31, 0x44, 0x4d, 0x15, 0x2d, 0x1b, 0x2c, 0x3c, 0x2a, 0x1d, 0x22, 0x35, 0x1a, 0x16, 0x1e,		// 10
																			0x26, 0x25, 0x2e, 0x36, 0x3d, 0x3e, 0x46, 0x45, 0x5a, 0x76, 0x66, 0x0d, 0x29, 0x4e, 0x55, 0x54,		// 20
																			0x5b, 0x5d, 0x5d, 0x4c, 0x52, 0x0e, 0x41, 0x49, 0x4a, 0x58, 0x05, 0x06, 0x04, 0x0c, 0x03, 0x0b,		// 30
																			0x83, 0x0a, 0x01, 0x09, 0x78, 0x07, 0xfe, 0x7e, 0xfe, 0x70, 0x6c, 0x7d, 0x71, 0x69, 0x7a, 0x74,		// 40
																			0x6b, 0x72, 0x75, 0x77, 0x4a, 0x7c, 0x7b, 0x79, 0x5a, 0x69, 0x72, 0x7a, 0x6b, 0x73, 0x74, 0x6c,		// 50
																			0x75, 0x7d, 0x70, 0x71, 0x61, 0x2f, 0x37, 0x0f, 0x08, 0x10, 0x18, 0x20, 0x28, 0x30, 0x38,	0x40,		// 60
																			0x48, 0x50, 0x57, 0x5f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,		// 70
																			0xff, 0xff, 0xff, 0xff, 0xff, 0x6d, 0xff, 0x51, 0x13, 0x6a, 0x64, 0x67, 0x27, 0xff, 0xff, 0xff,		// 80
																			0xf2, 0xf1, 0x63, 0x62, 0x5f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,		// 90
																			0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,		// a0
																			0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,		// b0
																			0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,		// c0
																			0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,		// d0
																			0x14, 0x12, 0x11, 0x1f, 0x14, 0x59, 0x11, 0x27, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,		// e0
																			0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff};	// f0
																			//0			1			2			3			4			5			6			7			8			9			a			b			c			d			e			f
const uint8_t HID_toPS2_exten[256] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,		// 00
																			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,		// 10
																			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,		// 20
																			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,		// 30
																			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,		// 40
																			0xFF, 0xFF, 0xFF, 0x00, 0xFF, 0x00, 0x00, 0x00, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,		// 50
																			0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,		// 60
																			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,		// 70
																			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,		// 80
																			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,		// 90
																			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,		// a0
																			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,		// b0
																			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,		// c0
																			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,		// d0
																			0x00, 0x00, 0x00, 0xFF, 0xFF, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,		// e0
																			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};	// f0
const uint8_t HID_toPS2_table_sys[8]={0x14, 0x12, 0x11, 0x1f, 0x14, 0x59, 0x11, 0x27};
const uint8_t HID_toPS2_exten_sys[8]={0x00, 0x00, 0x00, 0xFF, 0xFF, 0x00, 0xFF, 0xFF};

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
	if((PS2_State == IDLE) && (PS2_SendRequest == SET)) 
	{
		PS2_State = SEND;
		PS2_TransferState = START;
	}
}

void PS2_ClearFinishedSend(void)
{
	if((PS2_State == SEND) && (PS2_TransferState == FINISHED))
	{
		PS2_State = IDLE;
		PS2_SendRequest = RESET;
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
		
		if ((PS2_SendRequest == RESET) && (RingBufferU8_available(&ps2_buf_info)))
		{
			PS2_OutputData = RingBufferU8_readByte(&ps2_buf_info);
			PS2_SendRequest = SET;
		}
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
	.Mode = GPIO_MODE_OUTPUT_PP,
	.Pull = GPIO_PULLUP,
	.Speed = GPIO_SPEED_FREQ_HIGH,
	.Alternate = 0
};

void PS2_init(void)
{
  HAL_GPIO_WritePin(PS2_CLK_GPIO_Port, PS2_CLK_Pin|PS2_DAT_Pin, GPIO_PIN_SET);
  HAL_GPIO_Init(PS2_CLK_GPIO_Port, &GPIO_PS2_init);
	
	RingBufferU8_init(&ps2_buf_info, ps2_buf, PS2_BUF_SIZE);
	
	HAL_TIM_OC_Start_IT(&htim2, TIM_CHANNEL_1);
	HAL_TIM_OC_Start_IT(&htim2, TIM_CHANNEL_2);
	HAL_TIM_Base_Start_IT(&htim2);
}

void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
		PS2_ClockIRQHandler();
	else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
		PS2_DataIRQHandler();
}

void PS2_add_event(uint8_t is_released, uint8_t HID_scancode)
{
	if ((HID_scancode == 0) && (HID_toPS2_table[HID_scancode] != PS2_SCANCODE_INVALID))
		return;

	if (HID_toPS2_table[HID_scancode] != 0xfe)
	{
		if (HID_toPS2_exten[HID_scancode])
			// add extended code
			RingBufferU8_writeByte(&ps2_buf_info, 0xE0);

		if (is_released)
			// key released
			RingBufferU8_writeByte(&ps2_buf_info, 0xF0);
	}
	
	switch (HID_scancode)
	{
		case KEY_PRINTSCREEN:
			if (is_released)
			{
				RingBufferU8_writeByte(&ps2_buf_info, 0xE0);
				RingBufferU8_writeByte(&ps2_buf_info, 0xF0);
				RingBufferU8_writeByte(&ps2_buf_info, 0x7c);
				RingBufferU8_writeByte(&ps2_buf_info, 0xE0);
				RingBufferU8_writeByte(&ps2_buf_info, 0xF0);
				RingBufferU8_writeByte(&ps2_buf_info, 0x12);
			}
			else
			{
				RingBufferU8_writeByte(&ps2_buf_info, 0xE0);
				RingBufferU8_writeByte(&ps2_buf_info, 0x12);
				RingBufferU8_writeByte(&ps2_buf_info, 0xE0);
				RingBufferU8_writeByte(&ps2_buf_info, 0x7c);
			}
			break;
		case KEY_PAUSE:
			if (!is_released)
			{
				RingBufferU8_writeByte(&ps2_buf_info, 0xE1);
				RingBufferU8_writeByte(&ps2_buf_info, 0x14);
				RingBufferU8_writeByte(&ps2_buf_info, 0x77);
				RingBufferU8_writeByte(&ps2_buf_info, 0xE1);
				RingBufferU8_writeByte(&ps2_buf_info, 0xF0);
				RingBufferU8_writeByte(&ps2_buf_info, 0x14);
				RingBufferU8_writeByte(&ps2_buf_info, 0xF0);
				RingBufferU8_writeByte(&ps2_buf_info, 0x77);
			}
			break;
		default:
			RingBufferU8_writeByte(&ps2_buf_info, HID_toPS2_table[HID_scancode]);
			break;
	}
}

void PS2_add_event_sys(uint8_t is_released, uint8_t HID_scancode_sys)
{
	if (HID_scancode_sys == 0)
		return;
	
	for (int i=0 ; i<8 ; ++i)
	{
		if ((HID_scancode_sys & (1 << i)) != 0)
		{
			if (HID_toPS2_exten_sys[i])
				// add extended code
				RingBufferU8_writeByte(&ps2_buf_info, 0xE0);

			if (is_released)
				// key released
				RingBufferU8_writeByte(&ps2_buf_info, 0xF0);
			
			RingBufferU8_writeByte(&ps2_buf_info, HID_toPS2_table_sys[i]);
		}
	}
}
