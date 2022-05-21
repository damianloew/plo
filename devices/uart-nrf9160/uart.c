/*
 * Phoenix-RTOS
 *
 * plo - operating system loader
 *
 * STM32L4x6 Serial driver
 *
 * Copyright 2021 Phoenix Systems
 * Author: Aleksander Kaminski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include <hal/hal.h>
#include <lib/errno.h>
#include <lib/lib.h>
#include <devices/devs.h>


typedef struct {
	volatile unsigned int *base;
	volatile unsigned int cnt;
} uart_t;


struct {
	uart_t uarts[UART_MAX_CNT];
} uart_common;


enum { cr1 = 0, cr2, cr3, brr, gtpr, rtor, rqr, isr, icr, rdr, tdr };


static int uartLut[] = { UART0, UART1, UART2, UART3 };


static const struct {
	void *base;
	unsigned int irq;
	unsigned char txpin;
	unsigned char rxpin;
	unsigned char rtspin;
	unsigned char ctspin;
/* there can be used other pins as well */
} uartInfo[] = {
	{ (void *)UART0_BASE, UART0_IRQ, 29, 28, 27, 26 },
	{ (void *)UART1_BASE, UART1_IRQ, 1, 0, 14, 15 },
	{ (void *)UART2_BASE, UART2_IRQ, 29, 28, 27, 26 },
	{ (void *)UART3_BASE, UART3_IRQ, 1, 0, 14, 15 }
};


static uart_t *uart_getInstance(unsigned int minor)
{
	if (minor >= UART_MAX_CNT)
		return NULL;

	if (uartLut[minor] == 0)
		return NULL;

	return &uart_common.uarts[minor];
}


static int uart_handleIntr(unsigned int irq, void *buff)
{
	// uart_t *uart = (uart_t *)buff;

	// if (*(uart->base + isr) & ((1 << 5) | (1 << 3))) {
	// 	/* Clear overrun error bit */
	// 	*(uart->base + icr) |= (1 << 3);

	// 	/* Rxd buffer not empty */
	// 	uart->rxdfifo[uart->rxdw++] = *(uart->base + rdr);
	// 	uart->rxdw %= sizeof(uart->rxdfifo);

	// 	if (uart->rxdr == uart->rxdw)
	// 		uart->rxdr = (uart->rxdr + 1) % sizeof(uart->rxdfifo);

	// 	uart->rxflag = 1;
	// }

	return 0;
}

/* Device interface */

static ssize_t uart_read(unsigned int minor, addr_t offs, void *buff, size_t len, time_t timeout)
{
	// uart_t *uart;
	// size_t cnt;
	// time_t start;

	uart_t *uart;
	size_t cnt;
	char *ram1 = (char *)0x20008000;
	char *chbuff = (char *)buff;

	if ((uart = uart_getInstance(minor)) == NULL)
		return -EINVAL;

	if (len == 0)
		return 0;
	/* read last n unread bytes */
	for (cnt = 0; cnt < len; cnt++, uart->cnt++) {
		((unsigned char *)buff)[cnt] = ram1[uart->cnt];
	}
	// hal_interruptsDisable();



	// for (cnt = 0; cnt < len; ++cnt) {
	// 	uart->rxflag = 0;
	// 	if (uart->rxdr == uart->rxdw) {
	// 		hal_interruptsEnable();
	// 		start = hal_timerGet();
	// 		while (!uart->rxflag) {
	// 			if ((hal_timerGet() - start) >= timeout)
	// 				return -ETIME;
	// 		}
	// 		hal_interruptsDisable();
	// 	}
	// 	((unsigned char *)buff)[cnt] = uart->rxdfifo[uart->rxdr++];
	// 	uart->rxdr %= sizeof(uart->rxdfifo);
	// }
	// hal_interruptsEnable();

	return (ssize_t)cnt;
}


static ssize_t uart_write(unsigned int minor, const void *buff, size_t len)
{
	uart_t *uart;
	size_t cnt;
	char *ram0 = (char *)0x20000000;
	char *chbuff = (char *)buff;

	if ((uart = uart_getInstance(minor)) == NULL)
		return -EINVAL;

	for (cnt = 0; cnt < len; cnt++) {
		ram0[cnt] = *chbuff++;
	}

	*(uart->base + uarte_txd_ptr) = (volatile u32 *)ram0;
	*(uart->base + uarte_txd_maxcnt) = len;
	*(uart->base + uarte_starttx) = 1u;
	while ( *(uart->base + uarte_events_txstarted) != 1u )
		;
	*(uart->base + uarte_events_txstarted) = 0u;

	while ( *(uart->base + uarte_events_endtx) != 1u )
		;
	*(uart->base + uarte_events_endtx) = 0u;

	return (ssize_t)cnt;
}


static ssize_t uart_safeWrite(unsigned int minor, addr_t offs, const void *buff, size_t len)
{
	return uart_write(minor, buff, len);
}


static int uart_sync(unsigned int minor)
{
	// uart_t *uart;

	// if ((uart = uart_getInstance(minor)) == NULL)
	// 	return -EINVAL;

	// while (!(*(uart->base + isr) & 0x40))
	// 	;

	return EOK;
}


static int uart_done(unsigned int minor)
{
	// uart_t *uart;

	// if ((uart = uart_getInstance(minor)) == NULL)
	// 	return -EINVAL;

	// /* Wait for transmission activity complete */
	// (void)uart_sync(minor);

	// *(uart->base + cr1) = 0;
	// hal_cpuDataMemoryBarrier();
	// _stm32_rccSetDevClock(uartInfo[minor].dev, 0);
	// hal_cpuDataMemoryBarrier();
	// _stm32_gpioConfig(uartInfo[minor].rxport, uartInfo[minor].rxpin, 0, 0, 0, 0, 0);
	// _stm32_gpioConfig(uartInfo[minor].txport, uartInfo[minor].txpin, 0, 0, 0, 0, 0);

	// hal_interruptsSet(uartInfo[minor].irq, NULL, NULL);

	return EOK;
}


static int uart_map(unsigned int minor, addr_t addr, size_t sz, int mode, addr_t memaddr, size_t memsz, int memmode, addr_t *a)
{
	if (minor >= UART_MAX_CNT)
		return -EINVAL;

	/* Device mode cannot be higher than map mode to copy data */
	if ((mode & memmode) != mode)
		return -EINVAL;

	/* uart is not mappable to any region */
	return dev_isNotMappable;
}


static int uart_init(unsigned int minor)
{
	uart_t *uart;

	if ((uart = uart_getInstance(minor)) == NULL)
		return -EINVAL;

	/* Init pins according to nrf9160 product specification */
	_nrf91_gpioConfig(uartInfo[minor].txpin, output, nopull); //TODO: change to uart->txpin
	_nrf91_gpioConfig(uartInfo[minor].rxpin, input, nopull);
	_nrf91_gpioConfig(uartInfo[minor].rtspin, output, nopull);
	_nrf91_gpioConfig(uartInfo[minor].ctspin, input, pulldown);

	_nrf91_gpioSet(uartInfo[minor].txpin, high);
	_nrf91_gpioSet(uartInfo[minor].rtspin, high);

	uart->base = uartInfo[minor].base;

	/* Select pins */
	*(uart->base + uarte_psel_txd) = uartInfo[minor].txpin;
	*(uart->base + uarte_psel_rxd) = uartInfo[minor].rxpin;
	*(uart->base + uarte_psel_rts) = uartInfo[minor].rtspin;
	*(uart->base + uarte_psel_cts) = uartInfo[minor].ctspin;

	/* Set baud rate to 9600, TODO: verify why uart with 115200 br doesn't work properly */
	*(uart->base + uarte_baudrate) = baud_9600;

	/* Default settings - hardware flow control disabled, exclude parity bit, one stop bit */
	*(uart->base + uarte_config) = 0u;

	/* Set default max number of bytes in specific buffers to 4095 */
	*(uart->base + uarte_txd_maxcnt) = 0xFFF;
	*(uart->base + uarte_rxd_maxcnt) = 0xFFF;

	/* Set default uart sources: ram0 and ram1 start addresses */
	*(uart->base + uarte_txd_ptr) = 0x20000000;
	*(uart->base + uarte_txd_ptr) = 0x20008000;

	/* disable all uart interrupts */
	*(uart->base + uarte_intenclr) = 0xFFFFFFFF;

	*(uart->base + uarte_enable) = 0x8;

	/* Wait for cts activation - assuming that it should be active all the time - there is no this event because it was cleared by hal console once*/
	// while ( *(uart->base + uarte_events_cts) != 1u )
	// 	;
	// *(uart->base + uarte_events_cts) = 0u;

	/* Counter that indicates how many bytes were read from ram1 buffer */
	uart->cnt = 0;
	/* Start uart receiver */
	*(uart->base + uarte_startrx) = 1u;

	// uart->base = uart->base;
	// _stm32_rccSetDevClock(uartInfo[minor].dev, 1);

	// uart->rxdr = 0;
	// uart->rxdw = 0;
	// uart->rxflag = 0;

	// *(uart->base + cr1) = 0;
	// hal_cpuDataMemoryBarrier();


	// hal_interruptsSet(uartInfo[minor].irq, uart_handleIntr, (void *)uart);

	return EOK;
}


__attribute__((constructor)) static void uart_reg(void)
{
	static const dev_handler_t h = {
		.init = uart_init,
		.done = uart_done,
		.read = uart_read,
		.write = uart_safeWrite,
		.sync = uart_sync,
		.map = uart_map,
	};

	devs_register(DEV_UART, UART_MAX_CNT, &h);
}
