/*
 * Phoenix-RTOS
 *
 * plo - operating system loader
 *
 * Console
 *
 * Copyright 2021 Phoenix Systems
 * Authors: Aleksander Kaminski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include <hal/hal.h>
#include "nrf91.h"


static struct {
	volatile u32 *base;
	unsigned cpufreq;
} halconsole_common;


// enum { cr1 = 0, cr2, cr3, brr, gtpr, rtor, rqr, isr, icr, rdr, tdr };
enum { uarte_startrx = 0, uarte_stoprx, uarte_starttx, uarte_stoptx, 
uarte_events_cts = 64, uarte_events_txdrdy = 71, uarte_events_endtx, uarte_events_error, uarte_events_txstarted = 84, 
uarte_inten = 192, uarte_errorsrc = 288, uarte_intenset, uarte_intenclr, uarte_enable = 320, 
uarte_psel_rts = 322, uarte_psel_txd, uarte_psel_cts, uarte_psel_rxd, uarte_baudrate = 329, 
uarte_rxd_ptr = 333, uarte_rxd_maxcnt, uarte_rxd_amount, uarte_txd_ptr = 337, uarte_txd_maxcnt, uarte_txd_amount, 
uarte_config = 347 };

enum { baud_9600 = 0x00275000, baud_115200 = 0x01D60000 };


void hal_consolePrint(const char *s)
{
	char *ram0 = (char *)0x20000000;
	int i;

	for (i = 0; *s != '\0'; i++) {
		ram0[i] = *s++;
	}

	*(halconsole_common.base + uarte_txd_ptr) = (volatile u32 *)ram0;
	*(halconsole_common.base + uarte_txd_maxcnt) = (u32)i;
	*(halconsole_common.base + uarte_starttx) = 1u;
	while ( *(halconsole_common.base + uarte_events_txstarted) != 1u )
		;
	*(halconsole_common.base + uarte_events_txstarted) = 0u;

	while ( *(halconsole_common.base + uarte_events_endtx) != 1u )
		;
	*(halconsole_common.base + uarte_events_endtx) = 0u;
}


void console_init(void)
{
	struct {
		void *base;
	} uarts[] = {
		{ (void *)UART0_BASE },
		{ (void *)UART1_BASE },
		{ (void *)UART2_BASE },
		{ (void *)UART3_BASE }
	};

	/* default uart instance for nrf9160 dk, connected to VCOM0 */
	const u8 uart = 0, txpin = 29, rxpin = 28, rtspin = 27, ctspin = 26, led1pin = 2, led2pin = 3; 
	unsigned int reg = 6, errsrc = 0;

	halconsole_common.base = uarts[uart].base;

	/* Init pins according to nrf9160 product specification */
	_nrf91_gpioConfig(txpin, output, nopull);
	_nrf91_gpioConfig(rxpin, input, nopull);
	_nrf91_gpioConfig(rtspin, output, nopull);
	_nrf91_gpioConfig(ctspin, input, pulldown);

	_nrf91_gpioSet(txpin, high);
	_nrf91_gpioSet(rtspin, high);

	/* Select pins */
	*(halconsole_common.base + uarte_psel_txd) = txpin;
	*(halconsole_common.base + uarte_psel_rxd) = rxpin;
	*(halconsole_common.base + uarte_psel_rts) = rtspin;
	*(halconsole_common.base + uarte_psel_cts) = ctspin;

	/* Set baud rate to 9600, TODO: verify why uart with 115200 br doesn't work properly */
	*(halconsole_common.base + uarte_baudrate) = baud_9600;

	/* Default settings - hardware flow control disabled, exclude parity bit, one stop bit */
	*(halconsole_common.base + uarte_config) = 0u;

	/* Set default max number of bytes in specific buffers to 4095 */
	*(halconsole_common.base + uarte_txd_maxcnt) = 0xFFF;
	*(halconsole_common.base + uarte_rxd_maxcnt) = 0xFFF;

	/* Set default uart sources: ram0 and ram1 start addresses */
	*(halconsole_common.base + uarte_txd_ptr) = 0x20000000;
	*(halconsole_common.base + uarte_txd_ptr) = 0x20008000;

	/* disable all uart interrupts */
	*(halconsole_common.base + uarte_intenclr) = 0xFFFFFFFF;

	*(halconsole_common.base + uarte_enable) = 0x8;

	/* Wait for cts activation - assuming that it should be active all the time */
	while ( *(halconsole_common.base + uarte_events_cts) != 1u )
		;
	*(halconsole_common.base + uarte_events_cts) = 0u;
}
