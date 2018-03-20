#include "ms_proc.h"
#include "hid_proc.h"
#include "fw_updater.h"

#define USH_USR_FS_INIT       0
#define USH_USR_FS_READLIST   1
#define USH_USR_FS_READFW		  2
#define USH_USR_FS_DRAW       3

uint8_t USBH_USR_ApplicationState = USH_USR_FS_INIT;
FATFS fatfs;
FIL file;

void USBH_USR_MS_Init(void)
{
}

void USBH_USR_MS_DeviceAttached(void)
{
}

void USBH_USR_MS_UnrecoveredError (void)
{
}

void USBH_USR_MS_DeviceDisconnected (void)
{
}

void USBH_USR_MS_ResetDevice(void)
{
}

void USBH_USR_MS_DeviceSpeedDetected(uint8_t DeviceSpeed)
{
}

void USBH_USR_MS_Device_DescAvailable(void *DeviceDesc)
{
}

void USBH_USR_MS_DeviceAddressAssigned(void)
{
}

void USBH_USR_MS_Configuration_DescAvailable(USBH_CfgDesc_TypeDef * cfgDesc,
                                             USBH_InterfaceDesc_TypeDef *itfDesc,
                                             USBH_EpDesc_TypeDef *epDesc)
{
}

void USBH_USR_MS_Manufacturer_String(void *ManufacturerString)
{
}

void USBH_USR_MS_Product_String(void *ProductString)
{
}

void USBH_USR_MS_SerialNum_String(void *SerialNumString)
{
}

void USBH_USR_MS_EnumerationDone(void)
{
}

void USBH_USR_MS_DeviceNotSupported(void)
{
}

USBH_USR_Status USBH_USR_MS_UserInput(void)
{
	return USBH_USR_NO_RESP;
}

void USBH_USR_MS_OverCurrentDetected (void)
{
}

int fw_size = 0;
uint32_t fw_crc = 0;
uint32_t fw_crc_base = 0;
__ALIGN_BEGIN uint8_t buf_fw[60*1024] __ALIGN_END;
#define FW_READ_SIZE 512

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


int read_fw(void)
{
	UINT readed;

	if(f_open(&file, "usb_keyb.crc", FA_READ) == FR_OK)
	{
		f_read(&file, &fw_crc_base, sizeof(uint32_t), &readed);
		fw_crc_base = __builtin_bswap32(fw_crc_base);
		f_close(&file);
	}
	else
		return 0;
	
	if(f_open(&file, "usb_keyb.bin", FA_READ) == FR_OK)
	{
		int pos = 0;
		fw_size = file.fsize;
		while (pos < fw_size)
		{
			f_read(&file, &buf_fw[pos], FW_READ_SIZE, &readed);
			pos += readed;
		}
		
		crc32_init();
		fw_crc = crc32_byte(0, &buf_fw[0], fw_size);
		
		f_close(&file);
	}
	else
		return 0;
	
	if (fw_crc == fw_crc_base)
	{
		update_fw(buf_fw, fw_size);
		return 1;
	}
	else
		return 0;
}

extern USB_OTG_CORE_HANDLE           USB_OTG_Core;
int boot_mounted = 0;
int boot_OK = 0;

int USBH_USR_MS_Application(void)
{
  
  switch(USBH_USR_ApplicationState)
  {
  case USH_USR_FS_INIT: 
    if (f_mount( 0, &fatfs ) != FR_OK) 
		{
			USBH_USR_ApplicationState = USH_USR_FS_DRAW; 
      return(-1);
    }
    USBH_USR_ApplicationState = USH_USR_FS_READFW;
    break;
    
  case USH_USR_FS_READFW:
		LED_3_ON();
    USB_OTG_BSP_mDelay(100);
	
    if (USBH_MSC_Param.MSWriteProtect == DISK_WRITE_PROTECTED) 
		{
      USBH_USR_ApplicationState = USH_USR_FS_DRAW;
      break;
    }
		
		boot_mounted = 1;
    
    //f_mount(0, &fatfs);
		
		boot_OK = read_fw();
		
		boot_mounted = 0;

    USBH_USR_ApplicationState = USH_USR_FS_DRAW; 
    break;
    
  case USH_USR_FS_DRAW:
    f_mount(0, NULL); 
		LED_3_OFF();
    break;
  default: break;
  }
  return(0);
}

void USBH_USR_MS_DeInit(void)
{
  USBH_USR_ApplicationState = USH_USR_FS_INIT;
}

const USBH_Usr_cb_TypeDef USR_MS_cb =
{
  USBH_USR_MS_Init,
  USBH_USR_MS_DeInit,
  USBH_USR_MS_DeviceAttached,
  USBH_USR_MS_ResetDevice,
  USBH_USR_MS_DeviceDisconnected,
  USBH_USR_MS_OverCurrentDetected,
  USBH_USR_MS_DeviceSpeedDetected,
  USBH_USR_MS_Device_DescAvailable,
  USBH_USR_MS_DeviceAddressAssigned,
  USBH_USR_MS_Configuration_DescAvailable,
  USBH_USR_MS_Manufacturer_String,
  USBH_USR_MS_Product_String,
  USBH_USR_MS_SerialNum_String,
  USBH_USR_MS_EnumerationDone,
  USBH_USR_MS_UserInput,
  USBH_USR_MS_Application,
  USBH_USR_MS_DeviceNotSupported,
  USBH_USR_MS_UnrecoveredError
};
