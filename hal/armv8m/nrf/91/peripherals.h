/*
 * Phoenix-RTOS
 *
 * plo - operating system loader
 *
 * Peripherals definitions for armv8m33-nrf9160
 *
 * Copyright 2020, 2021 Phoenix Systems
 * Author: Hubert Buczynski, Aleksander Kaminski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */


#ifndef _PERIPHERALS_H_
#define _PERIPHERALS_H_

/* Periperals configuration */

/* Interrupts ICTR should be read to verify this, 480 is max for m33 processor */
#define SIZE_INTERRUPTS 480

/* UART */
#define UART_MAX_CNT 4

#ifndef UART0
#define UART0 1
#endif

#ifndef UART1
#define UART1 0
#endif

#ifndef UART2
#define UART2 0
#endif

#ifndef UART3
#define UART3 0
#endif

#ifndef UART_CONSOLE
#define UART_CONSOLE 2
#endif

#define UART_BAUDRATE 9600

#define UART0_BASE ((void *)0x50008000)
#define UART1_BASE ((void *)0x50009000)
#define UART2_BASE ((void *)0x5000A000)
#define UART3_BASE ((void *)0x5000B000)

// #define UART1_CLK pctl_usart1
// #define UART2_CLK pctl_usart2
// #define UART3_CLK pctl_usart3
// #define UART4_CLK pctl_uart4
// #define UART5_CLK pctl_uart5

#define UART0_IRQ uarte0
#define UART1_IRQ uarte1
#define UART2_IRQ uarte2
#define UART3_IRQ uarte3

#define FLASH_PROGRAM_1_ADDR    0x00000000
#define FLASH_PROGRAM_BANK_SIZE (1024 * 1024)

#endif
