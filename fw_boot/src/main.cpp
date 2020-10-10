#include "main.h"

#define LED_PORT gpiob
#define LED1_PIN STM32_GPIO::PIN_3
#define LED2_PIN STM32_GPIO::PIN_4
#define LED3_PIN STM32_GPIO::PIN_5

#define UART uart3

void xfunc_out(unsigned char c)
{
    UART.send_char(c);
}

void (*main_fw_jump)(void);
#define MAIN_FW_START_ADDR 0x08004000

void usb_fs_proc(USBHCore* core, USBHCore::EHostUser reason)
{
    UNUSED(core);
    switch (reason)
    {
    case USBHCore::EHostUser::SELECT_CONFIGURATION:
        xprintf("Select configuration\n\r");
        break;
    case USBHCore::EHostUser::CLASS_ACTIVE:
        xprintf("Class active\n\r");
        break;
    case USBHCore::EHostUser::CLASS_SELECTED:
        xprintf("Class selected\n\r");
        break;
    case USBHCore::EHostUser::CONNECTION:
        xprintf("Connection\n\r");
        break;
    case USBHCore::EHostUser::DISCONNECTION:
        xprintf("Disconnection\n\r");
        break;
    case USBHCore::EHostUser::UNRECOVERED_ERROR:
        xprintf("Unrecovered error\n\r");
        break;
    }
}

void disconnect_callback(STM32_HCD *hcd)
{
    static_cast<USBHCore*>(hcd->get_data())->LL_disconnect();
    debug_out("USB disconnected\n\r");
}

void connect_callback(STM32_HCD *hcd)
{
    static_cast<USBHCore*>(hcd->get_data())->LL_connect();
    debug_out("USB connected\n\r");
}

void SOF_callback(STM32_HCD *hcd)
{
    static_cast<USBHCore*>(hcd->get_data())->LL_inc_timer();
}

void HC_notify_URB_change_callback(STM32_HCD *hcd, uint8_t ch_num, STM32_HCD::EURBState urb_state)
{
#if (USBH_USE_OS == 1)
    reinterpret_cast<USBHCore*>(hcd->get_data())->LL_notify_URB_change();
#else
    UNUSED(hcd);
#endif
    UNUSED(ch_num);
    UNUSED(urb_state);
}

TCHAR SD_path[4], msc_path[2][4];
FATFS MSCFatFS[2];
SDDriver sd_driver;
MSCDriver msc_driver[2];

USBH_HID usbh_hid[2];
USBH_MSC usbh_msc[2];

TProcWorker ProcWorker;
TProc2 Proc2;

int main()
{
    /*STM32_RCC::enable_clk_GPIOA();
    STM32_RCC::enable_clk_GPIOB();
    STM32_RCC::enable_clk_GPIOC();
    STM32_RCC::enable_clk_GPIOD();
    STM32_RCC::enable_clk_GPIOE();*/

    LED_PORT.set_config(LED1_PIN | LED2_PIN | LED3_PIN, STM32_GPIO::EMode::OUTPUT_PP, 0, STM32_GPIO::ESpeed::LOW, STM32_GPIO::EPull::NOPULL);
    LED_PORT.pin_OFF(LED1_PIN | LED2_PIN | LED3_PIN);
    UART.init(STM32_BRATE_UART6);
    UART.send_str("\n\rUSB keyboard & mouse bootloader\n\r", TXRX_MODE::DIRECT);
    UART.send_str("USBH_init\n\r", TXRX_MODE::DIRECT);
    USBH_init();
    UART.send_str("FAT_init\n\r", TXRX_MODE::DIRECT);
    FAT_init();
    //UART.send_str("OS::run\n\r", TXRX_MODE::DIRECT);

    //OS::run();

    //UART.send_str("Jump to main firmware\n\r", TXRX_MODE::DIRECT);
    //USBH_HID::KbdReport keys[2];
    //memset(keys, 0, 2 * sizeof(USBH_HID::KbdReport));

    while (1)
    {
        usb_HS.process();
        usb_FS.process();
        /*usbh_hid[0].decode(reinterpret_cast<uint8_t*>(&keys[0]));
        usbh_hid[1].decode(reinterpret_cast<uint8_t*>(&keys[1]));*/
    }
}

void USBH_init()
{
    STM32_USB_PWR_FS_PORT.set_config(STM32_USB_PWR_FS_PIN, STM32_GPIO::EMode::OUTPUT_PP, 0, STM32_GPIO::ESpeed::LOW, STM32_GPIO::EPull::PULLUP);
    STM32_USB_PWR_FS_PORT.pin_OFF(STM32_USB_PWR_FS_PIN);

    usb_FS.init(usb_fs_proc, HOST_FS);
    usb_FS.register_class(&usbh_hid[0]);
    usb_FS.register_class(&usbh_msc[0]);
    usb_FS.start();

    usb_HS.init(usb_fs_proc, HOST_HS);
    usb_HS.register_class(&usbh_hid[1]);
    usb_HS.register_class(&usbh_msc[1]);
    usb_HS.start();
}

void FAT_init()
{
    FAT_FS::init();
    sd_driver.init_gpio();
    msc_driver[0].link_data(reinterpret_cast<void*>(&usbh_msc[0]));
    msc_driver[1].link_data(reinterpret_cast<void*>(&usbh_msc[1]));
    FAT_FS::link_driver(&sd_driver, SD_path, 0);
    FAT_FS::link_driver(&msc_driver[0], msc_path[0], 0);
    xprintf("MSC FS path: '%s'\n\r", msc_path[0]);
    FAT_FS::link_driver(&msc_driver[1], msc_path[1], 0);
    xprintf("MSC HS path: '%s'\n\r", msc_path[1]);
}

void __attribute__((noreturn)) Error_Handler()
{
    while (1)
        gpiod.pin_ON(STM32_GPIO::PIN_14);
}

namespace OS
{
    template <>
    OS_PROCESS void TProcWorker::exec()
    {
        //UART.send_str("Thread worker\n\r", TXRX_MODE::DIRECT);
        bool msc_mounted[2];
        msc_mounted[0] = false;
        msc_mounted[1] = false;
        //check_updates();
        for(;;)
        {
            usb_HS.process();
            usb_FS.process();
            sleep(1);
            /*LED_PORT.pin_toggle(LED2_PIN);
            sleep(1);
            LED_PORT.pin_toggle(LED2_PIN);
            sleep(4990);*/
           /* if ((usb_FS.get_active_class() != nullptr) && (usb_FS.get_active_class()->get_class_code() == USB_MSC_CLASS))
            {
                if (usbh_msc[0].is_ready())
                    xprintf("MSC FS path: '%s'\n\r", msc_path[0]);
            }
            if ((usb_HS.get_active_class() != nullptr) && (usb_HS.get_active_class()->get_class_code() == USB_MSC_CLASS))
            {
                if (usbh_msc[1].is_ready())
                {
                    xprintf("MSC HS path: '%s'\n\r", msc_path[1]);
                    if (f_mount(&MSCFatFS[1], msc_path[1], 1) == FR_OK)
                    {
                        msc_mounted[1] = true;
                        xprintf("Mounted MSC HS path: '%s'\n\r", msc_path[1]);
                    }
                    if (msc_mounted[1])
                    {
                    }
                    //
                }
            }*/
        }

        UART.deinit();
        STM32_RCC::deinit();
        STM32_SYSTICK::deinit();
        __disable_irq();
        //STM32_FLASH::enable_remap_system_flash();
        main_fw_jump = (void (*)(void)) (*((uint32_t *)(MAIN_FW_START_ADDR + 4)));
        __set_MSP(*(uint32_t *)MAIN_FW_START_ADDR);
        main_fw_jump();
    }

    template <>
    OS_PROCESS void TProc2::exec()
    {
        //UART.send_str("Thread debug\n\r", TXRX_MODE::DIRECT);
        for (;;)
        {
            LED_PORT.pin_toggle(LED1_PIN);
            sleep(1);
            LED_PORT.pin_toggle(LED1_PIN);
            sleep(990);
            //xprintf("FS messages: %d\n\r", usb_FS.get_message_depth());
            //xprintf("HS messages: %d\n\r", usb_HS.get_message_depth());
            //UART.send_str("Thread debugger\n\r", TXRX_MODE::INTERRUPT);
            /*if (STM32_SYSTICK::get_tick() > 60*1000)
            {
                //RTOS::stop();
                UART.send_str("Thread debugger\n\r", TXRX_MODE::DIRECT);
                //RTOS::print_stacks();
                while (1);
            }*/
        }
    }
}

#if scmRTOS_IDLE_HOOK_ENABLE
void OS::idle_process_user_hook()
{
    __WFI();
}
#endif

extern "C" void __init_system_timer()
{
    STM32_SYSTICK::init();
}

uint32_t cl = 0;

void ISR::SysTickTimer()
{
    if (++cl == (1000*100))
    {
        xprintf("ProcWorker stack slack: %d\n\r", ProcWorker.stack_slack());
        xprintf("Proc2 stack slack: %d\n\r", Proc2.stack_slack());
    }
    LED_PORT.pin_toggle(LED3_PIN);
    OS::system_timer_isr();
    OS::raise_context_switch();
}
