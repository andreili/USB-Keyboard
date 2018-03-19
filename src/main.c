#include "usb_bsp.h"
#include "usbh_core.h"
#include "ms_proc.h"
#include "hid_proc.h"

#include "kbd_matrix.h"
#include "cmsis_os.h"
#include "gui_all.h"

//#define ENABLE_USB_BOOT
//#define ENABLE_PS2

#define USE_GUI

void GUI_X_Init(void)
{
}

void GUI_X_InitOS (void)
{ 
}


void GUI_X_Lock(void)
{ 
}


void GUI_X_Unlock(void)
{ 
}


uint32_t GUI_X_GetTaskId(void)
{ 
  return 0;
}

__ALIGN_BEGIN USB_OTG_CORE_HANDLE           USB_OTG_Core 	__ALIGN_END ;
__ALIGN_BEGIN USBH_HOST                     USB_Host 			__ALIGN_END ;
/**
* @}
*/ 

void led_init(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13/* | GPIO_Pin_14 | GPIO_Pin_15*/;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_Init(GPIOD, &GPIO_InitStructure);
}

#define SW_CFG_MATRIX	0x01
#define SW_CFG_PS2		0x02
#define SW_CFG_VV55		0x04
#define SW_CFG_UNUSED	0x08

uint32_t sw_cfg, sw_mode;

void sw_init(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
	
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
}

uint32_t __inline get_sw_cfg(void)
{
	return (GPIOC->IDR & 0x0f);
}

uint32_t __inline get_sw_mode(void)
{
	return ((GPIOA->IDR & 0xff) < 4);
}

void config_SYSTICK()
{
	SysTick_Config(SystemCoreClock/1000);
  SysTick->CTRL |= 0x00000004U;
  //HAL_NVIC_SetPriority(SysTick_IRQn, 15, 0);
	uint32_t prioritygroup = 0x00U;
	prioritygroup = NVIC_GetPriorityGrouping();
	NVIC_SetPriority(SysTick_IRQn, NVIC_EncodePriority(prioritygroup, 15, 0));
}

osThreadId USBTaskHandle;
osThreadId GUITaskHandle;
osThreadId MatrixTaskHandle;
osThreadId PS2TaskHandle;

void task_USB(void const * argument);
void task_GUI(void const * argument);
void task_matrix(void const * argument);
void task_ps2(void const * argument);

int dev_mode = 0;

int main(void)
{
	led_init();
	sw_init();
	config_SYSTICK();
	
	// get configuration
	sw_cfg = (GPIOC->IDR & 0x0f);
	sw_mode = ((GPIOA->IDR & 0xff) < 4);
	
  osThreadDef(USBTask, task_USB, osPriorityNormal, 0, 128);
  USBTaskHandle = osThreadCreate(osThread(USBTask), NULL);

	if ((sw_cfg & SW_CFG_MATRIX) == SW_CFG_MATRIX)
	{
		// Initialize a needed GPIO & etc.
		init_matrix();
		//osThreadDef(MatrixTask, task_matrix, osPriorityNormal, 0, 128);
		//MatrixTaskHandle = osThreadCreate(osThread(MatrixTask), NULL);
	}

	#ifdef USE_GUI
  osThreadDef(GUITask, task_GUI, osPriorityBelowNormal, 0, 512);
  GUITaskHandle = osThreadCreate(osThread(GUITask), NULL);
	#endif

  osThreadDef(PS2Task, task_ps2, osPriorityBelowNormal, 0, 128);
  PS2TaskHandle = osThreadCreate(osThread(PS2Task), NULL);
	
  osKernelStart();
	
	while (1);
}

void task_USB(void const * argument)
{
	dev_mode = 0;
	
#ifdef ENABLE_USB_BOOT
  USBH_Init(&USB_OTG_Core, 
#ifdef USE_USB_OTG_FS
            USB_OTG_FS_CORE_ID,
#else
            USB_OTG_HS_CORE_ID,
#endif
            &USB_Host,
            &USBH_MSC_cb, 
            &USR_MS_cb);
  while (1)
  {
    USBH_Process(&USB_OTG_Core , &USB_Host);
    osDelay(1);
	}
#endif
	
	dev_mode = 1;
	
  /* Init Host Library */
  USBH_Init(&USB_OTG_Core, 
#ifdef USE_USB_OTG_FS
            USB_OTG_FS_CORE_ID,
#else
            USB_OTG_HS_CORE_ID,
#endif
            &USB_Host,
            &HID_cb, 
            &USR_HID_Callbacks);
	
  while (1)
  {
		MX_BIT_3_ON();
		
    USBH_Process(&USB_OTG_Core , &USB_Host);
		// fill keyboard matrix
		fill_matrix(sw_mode);
		
		if ((sw_cfg & SW_CFG_MATRIX) == SW_CFG_MATRIX)
			proc_matrix();
		#ifdef ENABLE_PS2
		if ((sw_cfg & SW_CFG_PS2) == SW_CFG_PS2)
			proc_ps2();
		#endif 
		
		MX_BIT_3_OFF();
		
    osDelay(10);     
  }
}

void task_GUI(void const * argument)
{
	init_GUI();
	main_GUI();
}

void task_matrix(void const * argument)
{
	init_matrix();
  for(;;)
  {
		proc_matrix();
  }
}

void task_ps2(void const * argument)
{
  for(;;)
  {
    osDelay(1);
  }
}


#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line
  number,ex: printf("Wrong parameters value: file %s on line %d\r\n", 
  file, line) */
  while (1);
}

#endif

