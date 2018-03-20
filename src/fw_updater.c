#include "fw_updater.h"
#include "cmsis_os.h"
#include "hid_proc.h"

void update_fw(uint8_t *buf, uint32_t fw_size)
{
	LED_4_ON();
	osDelay(100);
	
	__disable_irq();
	__disable_fault_irq();
	LED_4_ON();
	LED_3_ON();
	
	FLASH_Unlock();
	FLASH_EraseAllSectors(VoltageRange_3);
	FLASH_Lock();
	
	uint32_t page_idx = 0;
	uint32_t wr_addr_start;
	uint32_t fw_offs = 0;
	
	while (fw_offs < fw_size)
	{
		uint32_t fw_dw = *((uint32_t*)&buf[fw_offs]);
		wr_addr_start = FLASH_BASE + fw_offs;
		FLASH_Unlock();
		FLASH_ProgramWord(fw_offs, fw_dw); 
		FLASH_Lock();
		fw_offs += 4;
	}
	
	FLASH_Lock();
	
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
