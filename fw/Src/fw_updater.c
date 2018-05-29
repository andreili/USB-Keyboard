#include "fw_updater.h"
#include "kbd_global.h"

#define LED_3_ON()    	GPIOD->BSRR = GPIO_BSRR_BS3
#define LED_3_OFF()    	GPIOD->BSRR = GPIO_BSRR_BR3
#define LED_4_ON()    	GPIOD->BSRR = GPIO_BSRR_BS4
#define LED_4_OFF()    	GPIOD->BSRR = GPIO_BSRR_BR4

void update_fw(uint8_t *buf, uint32_t fw_size)
{
	LED_4_ON();
	HAL_Delay(100);
	
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
