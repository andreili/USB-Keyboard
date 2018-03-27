/**
  ******************************************************************************
  * @file    usbh_hid_keybd.c 
  * @author  MCD Application Team
  * @version V3.2.2
  * @date    07-July-2015
  * @brief   This file is the application layer for USB Host HID Keyboard handling
  *          QWERTY and AZERTY Keyboard are supported as per the selection in 
  *          usbh_hid_keybd.h              
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2015 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */ 


/* Includes ------------------------------------------------------------------*/
#include "usbh_hid_keybd.h"
#include "usbh_hid_parser.h"

/** @addtogroup USBH_LIB
* @{
*/

/** @addtogroup USBH_CLASS
* @{
*/

/** @addtogroup USBH_HID_CLASS
* @{
*/

/** @defgroup USBH_HID_KEYBD 
* @brief    This file includes HID Layer Handlers for USB Host HID class.
* @{
*/ 

/** @defgroup USBH_HID_KEYBD_Private_TypesDefinitions
* @{
*/ 
/**
* @}
*/ 


/** @defgroup USBH_HID_KEYBD_Private_Defines
* @{
*/ 
/**
* @}
*/ 
#ifndef AZERTY_KEYBOARD
  #define QWERTY_KEYBOARD
#endif
#define  KBD_LEFT_CTRL                                  0x01
#define  KBD_LEFT_SHIFT                                 0x02
#define  KBD_LEFT_ALT                                   0x04
#define  KBD_LEFT_GUI                                   0x08
#define  KBD_RIGHT_CTRL                                 0x10
#define  KBD_RIGHT_SHIFT                                0x20
#define  KBD_RIGHT_ALT                                  0x40
#define  KBD_RIGHT_GUI                                  0x80
#define  KBR_MAX_NBR_PRESSED                            6

/** @defgroup USBH_HID_KEYBD_Private_Macros
* @{
*/ 
/**
* @}
*/ 

/** @defgroup USBH_HID_KEYBD_Private_FunctionPrototypes
* @{
*/ 
USBH_StatusTypeDef USBH_HID_KeybdDecode(uint8_t* pData, int length);
/**
* @}
*/ 
 
/** @defgroup USBH_HID_KEYBD_Private_Variables
* @{
*/

HID_KEYBD_Info_TypeDef     keybd_info;
uint32_t                   keybd_report_data[2];

/**
  * @brief  USBH_HID_KeybdInit 
  *         The function init the HID keyboard.
  * @param  phost: Host handle
  * @retval USBH Status
  */
USBH_StatusTypeDef USBH_HID_KeybdInit(USBH_HandleTypeDef *phost)
{
  uint32_t x;
  HID_HandleTypeDef *HID_Handle =  (HID_HandleTypeDef *) phost->pActiveClass->pData;  

	keybd_info.alt_keys = 0;
  
  
  for(x=0; x< (sizeof(keybd_report_data)/sizeof(uint32_t)); x++)
  {
    keybd_report_data[x]=0;
  }
  
  if(HID_Handle->length > (sizeof(keybd_report_data)/sizeof(uint32_t)))
  {
    HID_Handle->length = (sizeof(keybd_report_data)/sizeof(uint32_t));
  }
  HID_Handle->pData = (uint8_t*)keybd_report_data;
  fifo_init(&HID_Handle->fifo, phost->device.Data, HID_QUEUE_SIZE * sizeof(keybd_report_data));
  
  return USBH_OK;    
}

/**
  * @brief  USBH_HID_KeybdDecode 
  *         The function decode keyboard data.
  * @param  phost: Host handle
  * @retval USBH Status
  */
USBH_StatusTypeDef USBH_HID_KeybdDecode(uint8_t* pData, int length)
{
  uint8_t x;
  
  if(length == 0)
  {
    return USBH_FAIL;
	}

	if (pData[2] != 1)
  {
    keybd_info.alt_keys = pData[0];
    
    for(x=0; x < sizeof(keybd_info.keys); x++)
			keybd_info.keys[x] = pData[x + 2];
    
    return USBH_OK; 
  }
  return   USBH_FAIL;  
}
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

