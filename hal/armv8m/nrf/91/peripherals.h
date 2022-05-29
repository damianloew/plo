/*
 * Phoenix-RTOS
 *
 * plo - operating system loader
 *
 * Peripherals definitions for armv8m33-nrf9160
 *
 * Copyright 2022 Phoenix Systems
 * Author: Damian Loewnau
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */


#ifndef _PERIPHERALS_H_
#define _PERIPHERALS_H_

/* Periperals configuration */

/* Based on INTLINESNUM value (ICTR cpu register) */
#define SIZE_INTERRUPTS 256

/* UART */
#define UART_MAX_CNT 4

/* supported variants: uart0 + uart1 / uart2 + uart3 */
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

/* currently supported baud rates: 9600, 115200 */
#define UART_BAUDRATE 115200

/* sizes of uart dma memory regions - 1 because of max value of maxcnt */
#define UART_TX_DMA_SIZE 8191
#define UART_RX_DMA_SIZE 8191

#define UART0_BASE ((void *)0x50008000)
#define UART1_BASE ((void *)0x50009000)
#define UART2_BASE ((void *)0x5000A000)
#define UART3_BASE ((void *)0x5000B000)

#define UART0_IRQ uarte0
#define UART1_IRQ uarte1
#define UART2_IRQ uarte2
#define UART3_IRQ uarte3

/* default uart instance for nrf9160 dk, connected to VCOM0 */
#define UART0_TX 29
#define UART0_RX 28
#define UART0_RTS 27
#define UART0_CTS 26

/* second uart interface on nrf9160 dk called nRF91_UART_2 on the board's schematic*/
#define UART1_TX 1
#define UART1_RX 0
#define UART1_RTS 14
#define UART1_CTS 15

#define UART2_TX UART0_TX
#define UART2_RX UART0_RX
#define UART2_RTS UART0_RTS
#define UART2_CTS UART0_CTS

#define UART3_TX UART1_TX
#define UART3_RX UART1_RX
#define UART3_RTS UART1_RTS
#define UART3_CTS UART1_CTS

/* ram7: section 2 and 3 */
#define UART0_TX_DMA 0x2003C000
#define UART0_RX_DMA 0x2003E000

/* ram7: section 0 and 1 */
#define UART1_TX_DMA 0x20038000
#define UART1_RX_DMA 0x2003A000

#define UART2_TX_DMA UART0_TX_DMA
#define UART2_RX_DMA UART0_RX_DMA

#define UART3_TX_DMA UART1_TX_DMA
#define UART3_RX_DMA UART1_RX_DMA

#define FLASH_PROGRAM_1_ADDR    0x00000000
#define FLASH_PROGRAM_BANK_SIZE (1024 * 1024)

#endif
