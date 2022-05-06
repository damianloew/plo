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
	const u8 uart = 1, txpin = 1, rxpin = 0, rtspin = 14, ctspin = 15, led1pin = 2, led2pin = 3; 
	unsigned int reg = 6, errsrc = 0;
	//uart0 - uart1 in schematics,
	//uart1 - uart 2 in schematics
	// const int uart = 0, txpin = 29, rxpin = 28, rtspin = 27, ctspin = 26; 

	halconsole_common.base = uarts[uart].base;

	// _nrf91_gpioConfig(2, output);
	// _nrf91_gpioSet(2, high);

	// _nrf91_gpioConfig(led2pin, output);
	// _nrf91_gpioSet(led2pin, high);
	// _nrf91_gpioSet(led2pin, low);


	/* Init pins - no pull by default */
	_nrf91_gpioConfig(txpin, output, nopull);
	_nrf91_gpioConfig(rxpin, input, nopull); // TODO: why underscore ?
	_nrf91_gpioConfig(rtspin, output, nopull);
	_nrf91_gpioConfig(ctspin, input, nopull);

	_nrf91_gpioSet(txpin, low);
	_nrf91_gpioSet(rtspin, low);

	/* Select pins */
	*(halconsole_common.base + uarte_psel_txd) = txpin;
	*(halconsole_common.base + uarte_psel_rxd) = rxpin;
	*(halconsole_common.base + uarte_psel_rts) = rtspin;
	*(halconsole_common.base + uarte_psel_cts) = ctspin;

	/* Set baud rate to 115200 */
	*(halconsole_common.base + uarte_baudrate) = 0x01D60000;

	/* Set max number of bytes in specific buffers to 4095 */
	*(halconsole_common.base + uarte_txd_maxcnt) = 0xFFF;
	*(halconsole_common.base + uarte_rxd_maxcnt) = 0xFFF;

	/* ram 0 and ram 1 start addresses */
	*(halconsole_common.base + uarte_txd_ptr) = (u32)ram0;
	*(halconsole_common.base + uarte_rxd_ptr) = (u32)ram1;

	/* disable all interrupts */
	*(halconsole_common.base + uarte_intenclr) = 0xFFFFFFFF;

	ram0[0] = 'H';
	ram0[1] = 'e';
	ram0[2] = 'l';
	ram0[3] = 'l';
	ram0[4] = 'o';
	ram0[5] = '\0';

	*(halconsole_common.base + uarte_enable) = 0x8; //from doc
	/* to remove */
	while ( (*(halconsole_common.base + uarte_enable) & (0x8)) != (0x8) ) ;
	reg = *(halconsole_common.base + uarte_events_error);
	// /* Wait for cts activation */
	// while ( *(halconsole_common.base + uarte_events_cts) != 1u ) ;

	*(halconsole_common.base + uarte_starttx) = 1u;

	reg = *(halconsole_common.base + uarte_events_error);

	while ( *(halconsole_common.base + uarte_events_txstarted) != 1u ) ;

	while ( *(halconsole_common.base + uarte_events_txdrdy) != 1u ) ;

	while ( *(halconsole_common.base + uarte_events_endtx) != 1u ) ;

	errsrc = *(halconsole_common.base + uarte_errorsrc);

	reg = 2;
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
