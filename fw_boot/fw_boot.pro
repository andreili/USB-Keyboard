QT -= core gui opengl

APP = fw_boot
TARGET = fw_boot

CONFIG += c++11
DEVICE = STM32F407xx

LIB_DIR = ../../STM32F4

QMAKE_LFLAGS += -T$$PWD/$${LIB_DIR}/cortex-m4/stm32f407xx.ld

DEFINES += $${DEVICE}

DESTDIR = ./
#win32:EXT = .exe

#LIBS += libc.a

QMAKE_CXXFLAGS += -fno-rtti -fstack-usage
#win32:QMAKE_LFLAGS += -L/mingw32/lib/gcc/arm-none-eabi/8.3.0/
linux:QMAKE_LFLAGS += -L/usr/lib/gcc/arm-none-eabi/7.2.0/
#QMAKE_LFLAGS += -stdlib=libc++ -stdlib=libstdc++

createlist.target = all
createlist.commands += arm-none-eabi-objdump -S $${OUT_PWD}/$${APP}$${EXT} > $$join(APP,,,".lst")
createlist.commands += && arm-none-eabi-nm -nSC $${OUT_PWD}/$${APP}$${EXT} > $$join(APP,,,".map")
createlist.commands += && arm-none-eabi-objcopy -Obinary $${OUT_PWD}/$${APP}$${EXT} $${OUT_PWD}/$${APP}.bin
!win32:createlist.commands += && arm-none-eabi-nm -nalS --size-sort $${OUT_PWD}/$${APP}$${EXT} | grep " T " | tail
createlist.commands += && arm-none-eabi-size $${OUT_PWD}/$${APP}$${EXT}
QMAKE_EXTRA_TARGETS += createlist

INCLUDEPATH += ./inc/ $${LIB_DIR}/Drivers/CMSIS/Devices/ $${LIB_DIR}/Drivers/CMSIS/inc/ $${LIB_DIR}/inc/ $${LIB_DIR}/StdLib/inc/
INCLUDEPATH += $${LIB_DIR}/Drivers/FATfs/inc/ $${LIB_DIR}/Third_Party/FatFs/ $${LIB_DIR}/Drivers/USB/inc/
INCLUDEPATH += $${LIB_DIR}/Third_Party/scmRTOS/core/ $${LIB_DIR}/Third_Party/scmRTOS/port/cortex/mx-gcc/

SOURCES += \
    src/main.cpp \
    $${LIB_DIR}/Drivers/CMSIS/Devices/ISRstm32f407xx.cpp \
    $${LIB_DIR}/src/init.cpp \
    $${LIB_DIR}/src/stm32_flash.cpp \
    $${LIB_DIR}/src/stm32_gpio.cpp \
    $${LIB_DIR}/src/stm32_nvic.cpp \
    $${LIB_DIR}/src/stm32_pwr.cpp \
    $${LIB_DIR}/src/stm32_rcc.cpp \
    $${LIB_DIR}/src/stm32_rtc.cpp \
    $${LIB_DIR}/src/stm32_sd.cpp \
    $${LIB_DIR}/src/stm32_sdram.cpp \
    $${LIB_DIR}/src/stm32_spi.cpp \
    $${LIB_DIR}/src/stm32_systick.cpp \
    $${LIB_DIR}/src/stm32_uart.cpp \
    $${LIB_DIR}/StdLib/src/xprintf.cpp \
    $${LIB_DIR}/src/stm32_dma.cpp \
    $${LIB_DIR}/Drivers/FATfs/src/fatfs.cpp \
    $${LIB_DIR}/Drivers/FATfs/src/sddriver.cpp \
    $${LIB_DIR}/Third_Party/FatFs/ff.c \
    $${LIB_DIR}/Third_Party/FatFs/option/unicode.c \
    $${LIB_DIR}/Third_Party/FatFs/option/ccsbcs.c \
    $${LIB_DIR}/Third_Party/FatFs/option/syscall.c \
    src/updater.cpp \
    ../../STM32F4/Drivers/USB/src/usbhcore.cpp \
    ../../STM32F4/src/stm32_hcd.cpp \
    ../../STM32F4/Drivers/USB/src/usbh_hid.cpp \
    ../../STM32F4/StdLib/src/fifo.cpp \
    ../../STM32F4/Drivers/USB/src/usbh_msc.cpp \
    ../../STM32F4/Drivers/FATfs/src/mscdriver.cpp \
    ../../STM32F4/src/stm32_tim.cpp \
    ../../STM32F4/StdLib/src/my_func.cpp \
    ../../STM32F4/Third_Party/scmRTOS/core/os_kernel.cpp \
    ../../STM32F4/Third_Party/scmRTOS/core/os_services.cpp \
    ../../STM32F4/Third_Party/scmRTOS/core/usrlib.cpp \
    ../../STM32F4/Third_Party/scmRTOS/port/cortex/mx-gcc/os_target.cpp

HEADERS += \
    inc/stm32_conf.h \
    $${LIB_DIR}/Drivers/CMSIS/Devices/ISRstm32f407xx.h \
    $${LIB_DIR}/Drivers/CMSIS/Devices/stm32f407xx.h \
    $${LIB_DIR}/Drivers/CMSIS/Devices/stm32f4xx.h \
    $${LIB_DIR}/inc/init.h \
    $${LIB_DIR}/inc/stm32_flash.h \
    $${LIB_DIR}/inc/stm32_gpio.h \
    $${LIB_DIR}/inc/stm32_inc.h \
    $${LIB_DIR}/inc/stm32_nvic.h \
    $${LIB_DIR}/inc/stm32_pwr.h \
    $${LIB_DIR}/inc/stm32_rcc.h \
    $${LIB_DIR}/inc/stm32_rtc.h \
    $${LIB_DIR}/inc/stm32_sd.h \
    $${LIB_DIR}/inc/stm32_sdram.h \
    $${LIB_DIR}/inc/stm32_spi.h \
    $${LIB_DIR}/inc/stm32_systick.h \
    $${LIB_DIR}/inc/stm32_uart.h \
    $${LIB_DIR}/StdLib/inc/bitbanding.h \
    $${LIB_DIR}/StdLib/inc/xprintf.h \
    $${LIB_DIR}/inc/stm32_def.h \
    $${LIB_DIR}/inc/stm32_dma.h \
    $${LIB_DIR}/Drivers/FATfs/inc/diskio.h \
    $${LIB_DIR}/Drivers/FATfs/inc/diskiodriver.h \
    $${LIB_DIR}/Drivers/FATfs/inc/fatfs.h \
    $${LIB_DIR}/Drivers/FATfs/inc/ffconf.h \
    $${LIB_DIR}/Drivers/FATfs/inc/sddriver.h \
    $${LIB_DIR}/Third_Party/FatFs/ff.h \
    $${LIB_DIR}/Third_Party/FatFs/integer.h \
    inc/updater.h \
    inc/usbh_config.h \
    ../../STM32F4/Drivers/USB/inc/usbh_class.h \
    ../../STM32F4/Drivers/USB/inc/usbhcore.h \
    ../../STM32F4/inc/stm32_hcd.h \
    ../../STM32F4/Drivers/USB/inc/usbh_hid.h \
    ../../STM32F4/StdLib/inc/fifo.h \
    ../../STM32F4/Drivers/USB/inc/usbh_msc.h \
    ../../STM32F4/Drivers/FATfs/inc/mscdriver.h \
    ../../STM32F4/inc/stm32_tim.h \
    ../../STM32F4/Third_Party/scmRTOS/core/os_kernel.h \
    ../../STM32F4/Third_Party/scmRTOS/core/os_services.h \
    ../../STM32F4/Third_Party/scmRTOS/core/scmRTOS.h \
    ../../STM32F4/Third_Party/scmRTOS/core/scmRTOS_defs.h \
    ../../STM32F4/Third_Party/scmRTOS/core/usrlib.h \
    inc/main.h


