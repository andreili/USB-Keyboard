#include "ms_proc.h"
#include "hid_proc.h"

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

extern USB_OTG_CORE_HANDLE           USB_OTG_Core;
int boot_OK = 0;

int USBH_USR_MS_Application(void)
{
  uint8_t writeTextBuff[] = "STM32 Connectivity line Host Demo application using FAT_FS   ";
  uint16_t bytesWritten, bytesToWrite;
  
  switch(USBH_USR_ApplicationState)
  {
  case USH_USR_FS_INIT: 
    /* Initialises the File System*/
    if ( f_mount( 0, &fatfs ) != FR_OK ) 
		{
      /* efs initialisation fails*/
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
    
    f_mount(0, &fatfs);
    
    if (f_open(&file, "0:STM32.TXT",FA_CREATE_ALWAYS | FA_WRITE) == FR_OK) { 
      /* Write buffer to file */
      bytesToWrite = sizeof(writeTextBuff); 
      f_write (&file, writeTextBuff, bytesToWrite, (void *)&bytesWritten);   
      
      /*close file and filesystem*/
      f_close(&file);
      f_mount(0, NULL); 
    }
		boot_OK = 1;

    USBH_USR_ApplicationState = USH_USR_FS_DRAW; 
    break;
    
  case USH_USR_FS_DRAW:
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

USBH_Usr_cb_TypeDef USR_MS_cb =
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
