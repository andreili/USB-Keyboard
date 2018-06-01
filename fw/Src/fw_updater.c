#include "fw_updater.h"
#include "kbd_global.h"
#include "main.h"
#include "fatfs.h"

#define LED_1_ON()    	LED1_GPIO_Port->BSRR = (LED1_Pin)
#define LED_1_OFF()    	LED1_GPIO_Port->BSRR = (LED1_Pin << GPIO_BSRR_BR0_Pos)
#define LED_2_ON()    	LED2_GPIO_Port->BSRR = (LED2_Pin)
#define LED_2_OFF()    	LED2_GPIO_Port->BSRR = (LED2_Pin << GPIO_BSRR_BR0_Pos)
#define LED_3_ON()    	LED3_GPIO_Port->BSRR = (LED3_Pin)
#define LED_3_OFF()    	LED3_GPIO_Port->BSRR = (LED3_Pin << GPIO_BSRR_BR0_Pos)

#define FW_MAX_SIZE (90*1024)
__ALIGN_BEGIN uint8_t fw_buf[FW_MAX_SIZE] __ALIGN_END;
uint32_t fw_size;
static uint32_t crc32r_table[256];

#define CRC32_POLY_R 0xEDB88320

static void crc32_init(void)
{
	int i, j;
	uint32_t cr;
	for (i = 0; i < 256; ++i) 
	{
		cr = i;
		for (j = 8; j > 0; --j)
			cr = cr & 0x00000001 ? (cr >> 1) ^ CRC32_POLY_R : (cr >> 1);
		crc32r_table[i] = cr;
	}
}

uint32_t crc32_byte(uint32_t init_crc, uint8_t *buf, int len)
{
	uint32_t v;
	uint32_t crc;
	crc = ~init_crc;
	while(len > 0) 
	{
		v = *buf++;
		crc = ( crc >> 8 ) ^ crc32r_table[( crc ^ (v ) ) & 0xff];
		len --;
	}
	return ~crc;
}

uint8_t fw_update_check(void)
{
	if (f_mount(&USBHFatFS, USBHPath, 1) != FR_OK)
		return 0;
	
	FIL f;
	uint32_t crc32;
	DEBUG_PR("Check updates.\n\r");
	// check signature
	if (f_open(&f, "1:/usb_keyb.bin.sign", FA_READ) == FR_OK)
	{
		uint32_t version;
		f_read(&f, &version, sizeof(uint32_t), NULL);
		f_read(&f, &crc32, sizeof(uint32_t), NULL);
		f_close(&f);
		
		// check version
		if (version == KBD_VERSION)
			// current version, exit
			return 0;
	}
	else
		return 0;
	
	DEBUG_PR("Version mismatched, read firmware...\n\r");
	
	// read firmware to RAM
	if (f_open(&f, "1:/usb_keyb.bin", FA_READ) == FR_OK)
	{
		f_read(&f, fw_buf, FW_MAX_SIZE, &fw_size);
		f_close(&f);
		
		uint32_t crc_bin;
		crc32_init();
		crc_bin = crc32_byte(0, (uint8_t*)fw_buf, fw_size);
		
		// check crc32
		if (crc_bin != crc32)
		{
			DEBUG_PR("Invalid checksum! (%08X != %08X) Aborted.\n\r", crc_bin, crc32);
			return 0;
		}
		else
		{
			DEBUG_PR("Readed firmware (%i bytes)\n\r", fw_size);
			return 1;
		}
	}
	
	return 0;
}

inline void flash_erase(void)
{
	HAL_FLASH_Unlock();
	FLASH_EraseInitTypeDef EraseInitStruct;
	uint32_t error_sector_num = 0;
  EraseInitStruct.TypeErase = FLASH_TYPEERASE_MASSERASE;
  EraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3;
  EraseInitStruct.Banks = FLASH_BANK_1;
	HAL_FLASHEx_Erase(&EraseInitStruct, &error_sector_num);
	HAL_FLASH_Lock();
}

inline void flash_write(void)
{
	uint32_t wr_addr_start;
	uint32_t fw_offs = 0;
	
	while (fw_offs < fw_size)
	{
		uint32_t fw_dw = *((uint32_t*)&fw_buf[fw_offs]);
		wr_addr_start = FLASH_BASE + fw_offs;
		HAL_FLASH_Unlock();
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, fw_offs, fw_dw);
		HAL_FLASH_Lock();
		fw_offs += 4;
	}
}

void fw_update(void)
{
	LED_3_ON();
	HAL_Delay(100);
	
	#ifdef IWDG_USE
	//disable watchdog
	IWDG->KR = 0x5555;
	IWDG->RLR = 4;
	while ((IWDG->SR & IWDG_SR_RVU) != 0)
		IWDG->KR = 0xAAAA; // kick the dog (*)
	#endif
	
	DEBUG_PR("Flashing...\n\r");
	
	__disable_irq();
	__disable_fault_irq();
	LED_3_ON();
	
	flash_erase();
	flash_write();
	
	HAL_FLASH_Lock();
	
	int idx = 0;
	while (1)
	{
		++idx;
		if (idx == 5000000)
			LED_3_ON();
		else if (idx == 10000000)
		{
			idx = 0;
			LED_3_OFF();
		}
	}
}
