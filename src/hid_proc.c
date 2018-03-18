#include "hid_proc.h"

#define KBR_MAX_NBR_PRESSED 10

uint16_t keys_pressed[KBR_MAX_NBR_PRESSED];

static void  KEYBRD_Init (void)
{
	for (int i=0 ; i<KBR_MAX_NBR_PRESSED ; ++i)
		keys_pressed[i] = 0;
}

static void KEYBRD_Decode(uint8_t *pbuf, uint16_t length)
{
	uint16_t i;
	uint16_t pressed_count = 0;
	
	uint16_t ctrl_keys = pbuf[0] << 8;
	
	for (i=0 ; i<KBR_MAX_NBR_PRESSED ; ++i)
		keys_pressed[i] = 0;
	
	for (i=2 ; i<length ; ++i)
	{
		if (pbuf[i] >= KEY_A)
			keys_pressed[pressed_count++] = ctrl_keys | pbuf[i];
	}
}

HID_cb_TypeDef HID_KEYBRD_cb= 
{
  KEYBRD_Init,
  KEYBRD_Decode
};

static void  MOUSE_Init ( void)
{
}

static void  MOUSE_Decode(uint8_t *data, uint16_t length)
{
  /*HID_MOUSE_Data.button = data[0];

  HID_MOUSE_Data.x      = data[1];
  HID_MOUSE_Data.y      = data[2];
  
  USR_MOUSE_ProcessData(&HID_MOUSE_Data);*/
}

HID_cb_TypeDef HID_MOUSE_cb = 
{
  MOUSE_Init,
  MOUSE_Decode,
};

void USBH_USR_HID_Init(void)
{
	LED_3_OFF();
}

void USBH_USR_HID_DeInit(void)
{
	LED_3_OFF();
}

void USBH_USR_HID_DeviceAttached(void)
{
	LED_3_ON();
}

void USBH_USR_HID_DeviceDisconnected (void)
{
	LED_3_OFF();
}

void USBH_USR_HID_OverCurrentDetected (void)
{
}

void USBH_USR_HID_UnrecoveredError (void)
{
}

void USBH_USR_HID_ResetDevice(void)
{
}

void USBH_USR_HID_DeviceSpeedDetected(uint8_t DeviceSpeed)
{
}

void USBH_USR_HID_Device_DescAvailable(void *DeviceDesc)
{
}

void USBH_USR_HID_DeviceAddressAssigned(void)
{
}

void USBH_USR_HID_Configuration_DescAvailable(USBH_CfgDesc_TypeDef * cfgDesc,
                                          USBH_InterfaceDesc_TypeDef *itfDesc,
                                          USBH_EpDesc_TypeDef *epDesc)
{
}

void USBH_USR_HID_Manufacturer_String(void *ManufacturerString)
{
}

void USBH_USR_HID_Product_String(void *ProductString)
{
}

void USBH_USR_HID_SerialNum_String(void *SerialNumString)
{
}

void USBH_USR_HID_EnumerationDone(void)
{
}

void USBH_USR_HID_DeviceNotSupported(void)
{
}

USBH_USR_Status USBH_USR_HID_UserInput(void)
{
	return USBH_USR_RESP_OK;
}

/*void USR_MOUSE_Init	(void)
{
	HID_MOUSE_ButtonReleased(0);
  HID_MOUSE_ButtonReleased(1);
  HID_MOUSE_ButtonReleased(2);
}

void USR_MOUSE_ProcessData(HID_MOUSE_Data_TypeDef *data)
{
	uint8_t idx = 1;   
  static uint8_t b_state[3] = { 0, 0 , 0};
  
  if ((data->x != 0) && (data->y != 0)) {
    HID_MOUSE_UpdatePosition(data->x , data->y);
  }
  
  for ( idx = 0 ; idx < 3 ; idx ++) {
    
    if (data->button & 1 << idx) {
      if (b_state[idx] == 0) {
        HID_MOUSE_ButtonPressed (idx);
        b_state[idx] = 1;
      }
    } else {
      if (b_state[idx] == 1) {
        HID_MOUSE_ButtonReleased (idx);
        b_state[idx] = 0;
      }
    }
  }
}

void  USR_KEYBRD_Init (void)
{
}

void  USR_KEYBRD_ProcessData (uint8_t data)
{
}*/

USBH_Usr_cb_TypeDef USR_HID_Callbacks =
{
	USBH_USR_HID_Init,
  USBH_USR_HID_DeInit,
  USBH_USR_HID_DeviceAttached,
  USBH_USR_HID_ResetDevice,
  USBH_USR_HID_DeviceDisconnected,
  USBH_USR_HID_OverCurrentDetected,
  USBH_USR_HID_DeviceSpeedDetected,
  USBH_USR_HID_Device_DescAvailable,
  USBH_USR_HID_DeviceAddressAssigned,
  USBH_USR_HID_Configuration_DescAvailable,
  USBH_USR_HID_Manufacturer_String,
  USBH_USR_HID_Product_String,
  USBH_USR_HID_SerialNum_String,
  USBH_USR_HID_EnumerationDone,
  USBH_USR_HID_UserInput,
  NULL,
  USBH_USR_HID_DeviceNotSupported,
  USBH_USR_HID_UnrecoveredError
};
