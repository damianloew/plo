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
enum { startrx = 0, stoprx, starttx, stoptx, enable = 320, 
psel_rts = 322, psel_txd, psel_cts, psel_rxd, baudrate = 329, 
rxd_ptr = 333, rxd_maxcnt, rxd_amount, txd_ptr = 337, txd_maxcnt, txd_amount, 
config = 347 };

// void hal_consolePrint(const char *s)
// {
// 	while (*s) {
// 		if (~(*(halconsole_common.base + isr)) & 0x80)
// 			continue;

// 		*(halconsole_common.base + tdr) = *(s++);
// 	}

// 	while (~(*(halconsole_common.base + isr)) & 0x80)
// 		;

// 	return;
// }


void console_init(void)
{
	/* TODO: check if there is a possibility to change uart clock and then add it in structure*/
	struct {
		void *base;
	} uarts[] = {
		{ (void *)UART0_BASE },
		{ (void *)UART1_BASE },
		{ (void *)UART2_BASE },
		{ (void *)UART3_BASE }
	};

	char *ram0 = (char *)0x20000000;
	char *ram1 = (char *)0x20008000;

	char *hello_str = "hello";
	//const int uart = 1, txpin = 1, rxpin = 0, rtspin = 14, ctspin = 15; 
	//uart0 - uart2 in schematics,
	const int uart = 0, txpin = 29, rxpin = 28, rtspin = 27, ctspin = 26; 

	halconsole_common.base = uarts[uart].base;

	/* Init pins - no pull by default */
	_nrf91_gpioConfig(txpin, output);
	_nrf91_gpioConfig(rxpin, input); // TODO: why underscore ?
	_nrf91_gpioConfig(rtspin, output);
	_nrf91_gpioConfig(ctspin, input);

	_nrf91_gpioSet(txpin, high);
	_nrf91_gpioSet(rtspin, high);

	/* Select pins */
	*(halconsole_common.base + psel_txd) = (txpin << 1);
	*(halconsole_common.base + psel_rxd) = (rxpin << 1);
	*(halconsole_common.base + psel_rts) = (rtspin << 1);
	*(halconsole_common.base + psel_cts) = (ctspin << 1);

	/* Set baud rate to 115200 */
	*(halconsole_common.base + baudrate) = 0x01D60000;

	/* Set max number of bytes in specific buffers to 4095 */
	*(halconsole_common.base + txd_maxcnt) = 0xFFF;
	*(halconsole_common.base + rxd_maxcnt) = 0xFFF;

	/* ram 0 and ram 1 start addresses */
	*(halconsole_common.base + txd_ptr) = (u32)ram0;
	*(halconsole_common.base + rxd_ptr) = (u32)ram1;

	ram0[0] = 'H';
	ram0[1] = 'e';
	ram0[2] = 'l';
	ram0[3] = 'l';
	ram0[4] = 'o';
	ram0[5] = '\0';

	*(halconsole_common.base + enable) = 0x8; //from doc

	*(halconsole_common.base + starttx) = 0x1;

	// TODO: set config ??
	// _stm32_rccSetDevClock(uarts[uart].uart, 1);

	// halconsole_common.cpufreq = _stm32_rccGetCPUClock();

	// /* Set up UART to 9600,8,n,1 16-bit oversampling */
	// *(halconsole_common.base + cr1) &= ~1UL; /* disable USART */
	// hal_cpuDataMemoryBarrier();
	// *(halconsole_common.base + cr1) = 0xa;
	// *(halconsole_common.base + cr2) = 0;
	// *(halconsole_common.base + cr3) = 0;
	// *(halconsole_common.base + brr) = halconsole_common.cpufreq / 115200; /* 115200 baud rate */
	// hal_cpuDataMemoryBarrier();
	// *(halconsole_common.base + cr1) |= 1;
	// hal_cpuDataMemoryBarrier();
}
