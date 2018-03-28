#include "fw_updater.h"
#include "cmsis_os.h"
#include "hid_proc.h"
#include "lcd_driver.h"

extern uint8_t Rx_Buff[ETH_RXBUFNB][ETH_RX_BUF_SIZE];
extern uint8_t Tx_Buff[ETH_TXBUFNB][ETH_TX_BUF_SIZE];
extern uint16_t lcd_buff[MAX_X][BUF_MAX_Y];

void update_fw(uint8_t *buf, uint32_t fw_size)
{
	LED_4_ON();
	osDelay(100);
	
	//disable watchdog
	IWDG->KR = 0x5555;
	IWDG->RLR = 4;
	while ((IWDG->SR & IWDG_SR_RVU) != 0)
		IWDG->KR = 0xAAAA; // kick the dog (*)
	
	__disable_irq();
	__disable_fault_irq();
	LED_4_ON();
	
	HAL_FLASH_Unlock();
	FLASH_EraseInitTypeDef EraseInitStruct;
	uint32_t error_sector_num = 0;
  EraseInitStruct.TypeErase = FLASH_TYPEERASE_MASSERASE;
  EraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3;
  EraseInitStruct.Banks = FLASH_BANK_1;
	HAL_FLASHEx_Erase(&EraseInitStruct, &error_sector_num);
	HAL_FLASH_Lock();
	
	uint32_t wr_addr_start;
	uint32_t fw_offs = 0;
	
	while (fw_offs < fw_size)
	{
		uint32_t fw_dw = *((uint32_t*)&buf[fw_offs]);
		wr_addr_start = FLASH_BASE + fw_offs;
		HAL_FLASH_Unlock();
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, fw_offs, fw_dw);
		HAL_FLASH_Lock();
		fw_offs += 4;
	}
	
	HAL_FLASH_Lock();
	
	int idx = 0;
	while (1)
	{
		++idx;
		if (idx == 5000000)
			LED_4_ON();
		else if (idx == 10000000)
		{
			idx = 0;
			LED_4_OFF();
		}
	}
}
