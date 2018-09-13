#include "updater.h"
#include "stm32_inc.h"
#define LED1_PIN GPIO_PIN_8
#define LED2_PIN GPIO_PIN_9
#define LED3_PIN GPIO_PIN_10

FATFS SDFatFS;

#define FW_START_ADDR (FLASH_BASE + 0x4000)
#define FW_VERSION_ADDR (FLASH_BASE + 0x7ff00)
#define FW_VERSION (*((uint32_t*)FW_VERSION_ADDR))

#define CRC32_POLY_R 0xEDB88320
static uint32_t crc32r_table[256];

#define FW_BUF_SIZE 1024
uint8_t fw_read_buf[FW_BUF_SIZE];
#define FW_START_SECTOR 1
#define FW_SECTORS_COUNT 6

void crc32_init(void)
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

void fw_write(uint8_t* buf, uint32_t size, uint32_t start_addr)
{
    uint32_t fw_offs = 0;

    while (fw_offs < size)
    {
        uint32_t fw_dw = *((uint32_t*)&buf[fw_offs]);
        STM32_FLASH::unlock();
        STM32_FLASH::program(FLASH_TypeProgram::WORD, start_addr, fw_dw);
        STM32_FLASH::lock();
        start_addr += 4;
        fw_offs += 4;
    }
}

bool apply_updates()
{
    FIL f;
    if (f_open(&f, "1:/usb_keyb.bin.sign", FA_READ) != FR_OK)
        return false;

    uint32_t version, crc32;
    f_read(&f, &version, sizeof(uint32_t), nullptr);
    f_read(&f, &crc32, sizeof(uint32_t), nullptr);
    f_close(&f);
    xprintf("Finded version %d, current version is %d\n\r", version, FW_VERSION);

    // check version
    if (FW_VERSION != version)
        return false;
    uart6.send_str("Version mismatched. Updating:\n\r\tErase flash...\n\r", TXRX_MODE::INTERRUPT);

    uint32_t err;
    if (STM32_FLASH::erase(FLASH_TypeErase::SECTORS, FLASH_VoltageRange::V_2P7_TO_3P6,
                           FLASH_BANK_1, FW_START_SECTOR, FW_SECTORS_COUNT, err) != STM32_RESULT_OK)
    {
        uart6.send_str("Failed flash erase!\n\r", TXRX_MODE::INTERRUPT);
        return false;
    }

    if (f_open(&f, "usb_keyb.bin", FA_READ) != FR_OK)
    {
        uart6.send_str("Unable to open file 'usb_keyb.bin'!\n\r", TXRX_MODE::INTERRUPT);
        return false;
    }

    crc32_init();

    UINT fw_size = 0;
    UINT fw_readed = 0;
    uint32_t crc32_new = 0;
    uint32_t fw_offset = FW_START_ADDR;
    do
    {
        f_read(&f, fw_read_buf, FW_BUF_SIZE, &fw_readed);
        crc32_new = crc32_byte(crc32_new, fw_read_buf, fw_readed);
        fw_write(fw_read_buf, FW_BUF_SIZE, fw_offset);
        fw_offset += FW_BUF_SIZE;
        fw_size += fw_readed;
    } while (fw_readed > 0);
    f_close(&f);

    if (crc32_new != crc32)
    {
        xprintf("Invalid checksum! (%08X != %08X) Aborted.\n\rr", crc32_new, crc32);
        gpioa.pin_ON(LED3_PIN);
        while (1);
    }

    xprintf("Readed firmware (%d bytes), version #%d\n\r", fw_size, version);
    fw_write((uint8_t*)&version, sizeof(uint32_t), FW_VERSION_ADDR);

    return true;
}

void check_updates()
{
    if (!STM32_FATFS_DRIVER::is_card_present())
        uart6.send_str("SD-card was not detected, skipping initialization of FATFs\n\r", TXRX_MODE::INTERRUPT);
    else
    {
        FAT_FS::init();
        uart6.send_str("Pass to mount SD-card\n\r", TXRX_MODE::INTERRUPT);
        while (f_mount(&SDFatFS, SD_path, 1) != FR_OK)
        {
            uart6.send_str("Unable to mount FAT partition! Check SD-card!\n", TXRX_MODE::INTERRUPT);
            STM32_SYSTICK::delay(1000);
        }
        uart6.send_str("FAT partition mounted\n\rCheck for updates...\n\r", TXRX_MODE::INTERRUPT);
        if (apply_updates())
            uart6.send_str("All updated!\n\r", TXRX_MODE::INTERRUPT);
        else
            uart6.send_str("Nothing to update.\n\r", TXRX_MODE::INTERRUPT);
    }
}
