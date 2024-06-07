#ifndef RT_CONFIG_H__
#define RT_CONFIG_H__

/* Automatically generated file; DO NOT EDIT. */
/* RT-Thread Configuration */

/* RT-Thread Kernel */

#define RT_NAME_MAX 8
#define RT_ALIGN_SIZE 4
#define RT_THREAD_PRIORITY_32
#define RT_THREAD_PRIORITY_MAX 32
#define RT_TICK_PER_SECOND 1000
#define RT_USING_OVERFLOW_CHECK
#define RT_USING_HOOK
#define RT_USING_IDLE_HOOK
#define RT_IDLE_HOOK_LIST_SIZE 4
#define IDLE_THREAD_STACK_SIZE 1024
#define RT_DEBUG
#define RT_DEBUG_COLOR

/* Inter-Thread communication */

#define RT_USING_SEMAPHORE
#define RT_USING_MUTEX
#define RT_USING_EVENT
#define RT_USING_MAILBOX
#define RT_USING_MESSAGEQUEUE

/* Memory Management */

#define RT_USING_MEMPOOL
#define RT_USING_SMALL_MEM
#define RT_USING_HEAP

/* Kernel Device Object */

#define RT_USING_DEVICE
#define RT_USING_CONSOLE
#define RT_CONSOLEBUF_SIZE 128
#define RT_CONSOLE_DEVICE_NAME "uart2"
#define RT_VER_NUM 0x40002
#define ARCH_ARM
#define RT_USING_CPU_FFS
#define ARCH_ARM_CORTEX_M
#define ARCH_ARM_CORTEX_M4

/* RT-Thread Components */

#define RT_USING_COMPONENTS_INIT
#define RT_USING_USER_MAIN
#define RT_MAIN_THREAD_STACK_SIZE 2048
#define RT_MAIN_THREAD_PRIORITY 10

/* C++ features */


/* Command shell */


/* Device virtual file system */


/* Device Drivers */

#define RT_USING_DEVICE_IPC
#define RT_PIPE_BUFSZ 512
#define RT_USING_SERIAL
#define RT_SERIAL_USING_DMA
#define RT_SERIAL_RB_BUFSZ 512
#define RT_USING_CAN
#define RT_USING_PIN
#define RT_USING_PWM
#define RT_USING_SPI
#define RT_USING_WDT

/* Using USB */


/* POSIX layer and C standard library */

#define RT_LIBC_USING_TIME

/* Network */

/* Socket abstraction layer */


/* Network interface device */


/* light weight TCP/IP stack */


/* AT commands */


/* VBUS(Virtual Software BUS) */


/* Utilities */


/* RT-Thread online packages */

/* IoT - internet of things */


/* Wi-Fi */

/* Marvell WiFi */


/* Wiced WiFi */


/* IoT Cloud */


/* security packages */


/* language packages */

/* JSON: JavaScript Object Notation, a lightweight data-interchange format */


/* XML: Extensible Markup Language */


/* multimedia packages */

/* LVGL: powerful and easy-to-use embedded GUI library */


/* u8g2: a monochrome graphic library */


/* PainterEngine: A cross-platform graphics application framework written in C language */


/* tools packages */


/* system packages */

/* enhanced kernel services */


/* acceleration: Assembly language or algorithmic acceleration packages */


/* CMSIS: ARM Cortex-M Microcontroller Software Interface Standard */


/* Micrium: Micrium software products porting for RT-Thread */


/* peripheral libraries and drivers */


/* Kendryte SDK */


/* AI packages */


/* miscellaneous packages */

/* project laboratory */

/* samples: kernel and components samples */


/* entertainment: terminal games and other interesting software packages */

#define SOC_FAMILY_STM32
#define SOC_SERIES_STM32F4

/* Choose Public Libraries */

/* Whether to use the HardWare Libraries */

#define BSP_USING_BUZZER
#define BUZZER_PWM_DEVICE_NAME "pwm2"
#define BUZZER_PWM_CHANNEL 4
#define BSP_USING_REMOTE_DT7
#define REMOTE_UART_DEVICE_NAME "uart6"
#define BSP_USING_RGB_LIGHT
#define RLIGHT_PWM_DEVICE_NAME "pwm2"
#define RLIGHT_PWM_CHANNEL 1
#define GLIGHT_PWM_DEVICE_NAME "pwm2"
#define GLIGHT_PWM_CHANNEL 3
#define BLIGHT_PWM_DEVICE_NAME "pwm2"
#define BLIGHT_PWM_CHANNEL 2
#define BSP_USING_SPI_Master
#define SPI_MASTER_USE_HSPIN 1
#define USE_STC_SLAVE
#define NEED_SLAVE_NUM 2
#define CS_PORT_0 "C"
#define CS_PIN_0 4
#define RST_PORT_0 "B"
#define RST_PIN_0 6
#define CS_PORT_1 "B"
#define CS_PIN_1 0
#define RST_PORT_1 "B"
#define RST_PIN_1 7

/* Whether to use the SoftWare Libraries */

#define BSP_USING_CPU_USAGE
#define BSP_USING_PID
#define BSP_USING_MOTORLIB
#define BSP_USING_SETPLANNING
#define BSP_USING_QUEUE
#define BSP_USING_LIST
#define BSP_USING_UTILS
#define BSP_UTILS_USING_ARM_MATH
#define BSP_USING_DATASERVER
#define DATA_MAX_SIZE 2048
#define DataServer_Handle_MAX 100

/* Use the filter module(s) */

#define BSP_USING_EXACTSMOOTH
#define BSP_USING_ACCCLAMP

/* Use the key-menu module(s) */

#define BSP_USING_KEY_CALLBACK
#define BSP_USING_KEY_MENU

/* Monitor Settings */

#define BSP_USING_MONITOR_CHASSIS
#define BSP_USING_MONITOR
#define MONITOR_NEED_ALARM
#define RTT_SWITCH_PIN_NUM 2
#define RTT_SWITCH_PIN_PORT "A"

/* Hardware Drivers Config */

#define SOC_STM32F446RE

/* Onboard Peripheral Drivers */

#define CORE_USING_MONITOR
#define BSP_USING_WDT

/* On-chip Peripheral Drivers */

#define BSP_USING_ON_CHIP_FLASH
#define BSP_USING_GPIO
#define BSP_USING_UART
#define BSP_USING_UART1
#define BSP_UART1_RX_USING_DMA
#define BSP_UART1_TX_USING_DMA
#define BSP_USING_UART3
#define BSP_USING_UART6
#define BSP_UART6_RX_USING_DMA
#define BSP_UART6_TX_USING_DMA
#define BSP_USING_PWM
#define BSP_USING_PWM2
#define BSP_USING_PWM2_CH1
#define BSP_USING_PWM2_CH2
#define BSP_USING_PWM2_CH3
#define BSP_USING_PWM2_CH4
#define BSP_USING_SPI
#define BSP_USING_SPI1
#define BSP_USING_CAN
#define BSP_USING_CAN1
#define BSP_USING_CAN2
#define BSP_USING_DSP

/* Board extended module Drivers */


#endif
