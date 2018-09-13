#include "stm32_inc.h"
#include "my_func.h"
#include "xprintf.h"
#include "usbhcore.h"
#include "updater.h"

#define LED_PORT gpioa
#define LED1_PIN GPIO_PIN_8
#define LED2_PIN GPIO_PIN_9
#define LED3_PIN GPIO_PIN_10

void xfunc_out(unsigned char c)
{
    uart6.send_char(c);
}

void (*main_fw_jump)(void);
#define MAIN_FW_START_ADDR 0x08004000

int main()
{
    STM32_RCC::enable_clk_GPIOA();
    LED_PORT.set_config(LED1_PIN | LED2_PIN | LED3_PIN, GPIO_MODE_OUTPUT_PP, 0, GPIO_SPEED_FREQ_LOW, GPIO_NOPULL);
    gpioa.pin_ON(LED1_PIN | LED2_PIN);
    uart6.init(STM32_BRATE_UART6);
    uart6.send_str("\n\rUSB keyboard & mouse bootloader\n\r", TXRX_MODE::INTERRUPT);
    check_updates();
    uart6.send_str("Jump to main firmware\n\r", TXRX_MODE::DIRECT);
    uart6.deinit();
    STM32_RCC::deinit();
    STM32_SYSTICK::deinit();
    __disable_irq();
    STM32_FLASH::enable_remap_system_flash();
    main_fw_jump = (void (*)(void)) (*((uint32_t *)(MAIN_FW_START_ADDR + 4)));
    __set_MSP(*(uint32_t *)MAIN_FW_START_ADDR);
    main_fw_jump();
    return 0;
}

void Error_Handler()
{
    while (1)
        gpioa.pin_ON(LED3_PIN);
}
