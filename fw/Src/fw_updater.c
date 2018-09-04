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

#define FW_MAX_SIZE (256*1024)
static uint32_t crc32r_table[256];
uint32_t fw_size;
uint8_t fw_read_buf[1024];

#define FW_NEW_SECT_START (FLASH_SECTOR_6)
#define FW_NEW_SECT_COUNT (2)
#define FW_NEW_ADDR_START (FLASH_BASE + 0X00040000)

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

void prepare_new_fw_region(void)
{
	FLASH_EraseInitTypeDef EraseInitStruct;
	uint32_t error_sector_num = 0;
  EraseInitStruct.TypeErase = FLASH_TYPEERASE_SECTORS;
  EraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3;
  EraseInitStruct.Sector = FW_NEW_SECT_START;
	EraseInitStruct.NbSectors = FW_NEW_SECT_COUNT;
	HAL_FLASH_Unlock();
	__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);
	HAL_FLASHEx_Erase(&EraseInitStruct, &error_sector_num);
	FLASH_WaitForLastOperation(1000);
	HAL_FLASH_Lock();
}

void flash_write(uint8_t* buf, uint32_t size, uint32_t start_addr)
{
	uint32_t wr_addr_start = start_addr;
	uint32_t fw_offs = 0;
	
	while (size)
	{
		uint32_t fw_dw = *((uint32_t*)&buf[fw_offs]);
		HAL_FLASH_Unlock();
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, wr_addr_start, fw_dw);
		FLASH_WaitForLastOperation(1000);
		HAL_FLASH_Lock();
		fw_offs += 4;
		wr_addr_start += 4;
		size -= 4;
	}
}

uint8_t fw_update_check(void)
{
	if (f_mount(&USBHFatFS, USBHPath, 1) != FR_OK)
		return 0;
	
	FIL f;
	uint32_t crc32;
	uint32_t version;
	DEBUG_PR("Check updates.\n\r");
	// check signature
	if (f_open(&f, "1:/usb_keyb.bin.sign", FA_READ) == FR_OK)
	{
		f_read(&f, &version, sizeof(uint32_t), NULL);
		f_read(&f, &crc32, sizeof(uint32_t), NULL);
		f_close(&f);
		
		// check version
		if (version == KBD_VERSION)
		{
			// current version, exit
			DEBUG_PR("Version matched, continue.\n\r");
			return 0;
		}
	}
	else
		return 0;
	
	DEBUG_PR("Version mismatched:\n\r");
	DEBUG_PR("\tPrepare flash region...\n\r");
	prepare_new_fw_region();
	
	DEBUG_PR("\tReading firmware...\n\r");
	if (f_open(&f, "1:/usb_keyb.bin", FA_READ) == FR_OK)
	{
		uint32_t crc_bin = 0;
		crc32_init();
		
		// read firmware to reserved flash
		uint32_t fw_read = 0;
		fw_size = 0;
		uint32_t write_addr = FW_NEW_ADDR_START;
		do
		{
			f_read(&f, fw_read_buf, 1024, &fw_read);
			crc_bin = crc32_byte(crc_bin, (uint8_t*)fw_read_buf, fw_read);
			flash_write(fw_read_buf, 1024, write_addr);
			write_addr += 1024;
			fw_size += fw_read;
		} while (fw_read != 0);
		f_close(&f);
		
		// round to 4 bytes
		fw_size += 4 - (fw_size % 4);
		
		// check crc32
		if (crc_bin != crc32)
		{
			DEBUG_PR("Invalid checksum! (%08X != %08X) Aborted.\n\r", crc_bin, crc32);
			return 0;
		}
		else
		{
			DEBUG_PR("Readed firmware (%i bytes), version #%i\n\r", fw_size, version);
			return 1;
		}
	}
	
	return 0;
}

void flash_erase(void)
{
	FLASH_EraseInitTypeDef EraseInitStruct;
	uint32_t error_sector_num = 0;
  EraseInitStruct.TypeErase = FLASH_TYPEERASE_SECTORS;
  EraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3;
  EraseInitStruct.Sector = FLASH_SECTOR_0;
	EraseInitStruct.NbSectors = 4;
	HAL_FLASH_Unlock();
	__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);
	HAL_FLASHEx_Erase(&EraseInitStruct, &error_sector_num);
	FLASH_WaitForLastOperation(1000);
	HAL_FLASH_Lock();
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
	flash_write((uint8_t*)FW_NEW_ADDR_START, fw_size, FLASH_BASE);
	
	//HAL_FLASH_Lock();
	
	int idx = 0;
	while (1)
	{
		++idx;
		if (idx == 5000000)
		{
			LED_1_ON();
			LED_2_ON();
			LED_3_ON();
		}
		else if (idx == 10000000)
		{
			idx = 0;
			LED_1_OFF();
			LED_2_OFF();
			LED_3_OFF();
		}
	}
}
