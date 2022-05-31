/*
 * Phoenix-RTOS
 *
 * plo - operating system loader
 *
 * Console
 *
 * Copyright 2022 Phoenix Systems
 * Authors: Damian Loewnau
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include <hal/hal.h>
#include "nrf91.h"


static struct {
	volatile u32 *base;
} halconsole_common;


/* Maximum number of characters that will be sent is specifed by UART_TX_DMA_SIZE */
void hal_consolePrint(const char *s)
{
	volatile char *tx_dma_buff = (volatile char *)UART0_TX_DMA;
	int cnt = 0;

	do {
		tx_dma_buff[cnt] = s[cnt];
		cnt++;
	} while (s[cnt-1] != '\0');

	if (cnt > UART_TX_DMA_SIZE) {
		cnt = UART_TX_DMA_SIZE;
	}

	*(halconsole_common.base + uarte_txd_ptr) = UART0_TX_DMA;
	*(halconsole_common.base + uarte_txd_maxcnt) = cnt;
	*(halconsole_common.base + uarte_starttx) = 1u;
	while ( *(halconsole_common.base + uarte_events_txstarted) != 1u )
		;
	*(halconsole_common.base + uarte_events_txstarted) = 0u;

	while ( *(halconsole_common.base + uarte_events_endtx) != 1u )
		;
	*(halconsole_common.base + uarte_events_endtx) = 0u;
}


/* Init pins according to nrf9160 product specification */
static void console_configPins(void)
{
	_nrf91_gpioConfig(UART0_TX, output, nopull);
	_nrf91_gpioConfig(UART0_RX, input, nopull);
	_nrf91_gpioConfig(UART0_RTS, output, nopull);
	_nrf91_gpioConfig(UART0_CTS, input, pulldown);

	_nrf91_gpioSet(UART0_TX, high);
	_nrf91_gpioSet(UART0_RTS, high);
}


/* UART0 supported for the hal console */
void console_init(void)
{
	halconsole_common.base = UART0_BASE;
	// halconsole_common.base = UART1_BASE;
	console_configPins();

	/* disable uarte instance */
	*(halconsole_common.base + uarte_enable) = 0u;
	hal_cpuDataMemoryBarrier();

	/* Select pins */
	*(halconsole_common.base + uarte_psel_txd) = UART0_TX;
	*(halconsole_common.base + uarte_psel_rxd) = UART0_RX;
	*(halconsole_common.base + uarte_psel_rts) = UART0_RTS;
	*(halconsole_common.base + uarte_psel_cts) = UART0_CTS;
	// /* Select pins */
	// *(halconsole_common.base + uarte_psel_txd) = UART1_TX;
	// *(halconsole_common.base + uarte_psel_rxd) = UART1_RX;
	// *(halconsole_common.base + uarte_psel_rts) = UART1_RTS;
	// *(halconsole_common.base + uarte_psel_cts) = UART1_CTS;

	switch (UART_BAUDRATE) {
		case 9600:
			*(halconsole_common.base + uarte_baudrate) = baud_9600;
			break;
		case 115200:
		default:
			*(halconsole_common.base + uarte_baudrate) = baud_115200;
	}

	/* Default settings - hardware flow control disabled, exclude parity bit, one stop bit */
	*(halconsole_common.base + uarte_config) = 0u;

	/* Set default max number of bytes in specific buffers */
	*(halconsole_common.base + uarte_txd_maxcnt) = UART_TX_DMA_SIZE;
	*(halconsole_common.base + uarte_rxd_maxcnt) = UART_RX_DMA_SIZE;

	/* Set default memory regions for uart dma */
	*(halconsole_common.base + uarte_txd_ptr) = UART0_TX_DMA;
	*(halconsole_common.base + uarte_rxd_ptr) = UART0_RX_DMA;
	// *(halconsole_common.base + uarte_txd_ptr) = UART1_TX_DMA;
	// *(halconsole_common.base + uarte_rxd_ptr) = UART1_RX_DMA;

	/* disable all uart interrupts */
	*(halconsole_common.base + uarte_intenclr) = 0xFFFFFFFF;
	hal_cpuDataMemoryBarrier();

	/* enable uarte instance */
	*(halconsole_common.base + uarte_enable) = 0x8;
	hal_cpuDataMemoryBarrier();
}
