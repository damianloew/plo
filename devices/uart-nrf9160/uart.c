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

#define BUFFER_SIZE 0x20

typedef struct {
	volatile unsigned int *base;
	unsigned int cnt;
	unsigned int irq;

	u16 rxFifoSz;
	u16 txFifoSz;

	u8 dataRx[BUFFER_SIZE];
	cbuffer_t cbuffRx;

	u8 dataTx[BUFFER_SIZE];
	cbuffer_t cbuffTx;
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
	char c;
	char *ram1 = (char *)0x20008000;
	char *ram0 = (char *)0x20000000;
	u32 flags;
	uart_t *uart = (uart_t *)buff;
	u32 rxcount = 0;

	// /* clear endrx event flag */
	// *(uart->base + uarte_events_endrx) = 0u;

	/* clear endrx event flag */
	*(uart->base + uarte_events_rxdrdy) = 0u;

	_nrf91_gpioSet(2, high);

	if (uart == NULL)
		return 0;

	// /* clear rxdrdy event flag */
	// *(uart->base + uarte_events_rxdrdy) = 0u;

	/* maybe not needed ? */
	// *(uart->base + uarte_flushrx) = 1u;

	// *(uart->base + uarte_startrx) = 1u;
	// /* clear rxto event flag */
	// *(uart->base + uarte_events_rxto) = 0u;

	/* Check how many bytes have been read */
	// rxcount = *(uart->base + uarte_rxd_amount);

	// for (int i = 0; i < rxcount; i++) {
	lib_cbufWrite(&uart->cbuffRx, &ram1[uart->cnt], 1);
	uart->cnt++;
	// }

	// /* Error flags: parity, framing, noise, overrun */
	// flags = *(uart->base + statr) & (0xf << 16);

	// /* RX overrun: invalidate fifo */
	// if (flags & (1 << 19))
	// 	*(uart->base + fifor) |= 1 << 14;

	// *(uart->base + statr) |= flags;

	// /* Receive */
	// while (uart_getRXcount(uart)) {
	// 	c = *(uart->base + datar);
	// 	lib_cbufWrite(&uart->cbuffRx, &c, 1);
	// }

	/* Transmit */
	// for (int i = 0; i < rxcount; i++) {
	// 	lib_cbufRead(&uart->cbuffTx, &ram0[i], 1);
	// }

	// int i = 0;
	// while (!lib_cbufEmpty(&uart->cbuffTx)) {
	// 	lib_cbufRead(&uart->cbuffTx, &ram0[i], 1);
	// 	uart_send(uart, rxcount);
	// 	i++;
	// }
	// uart_send(uart, (i+1));

	// while (uart_getTXcount(uart) < uart->txFifoSz) {
	// 	if (!lib_cbufEmpty(&uart->cbuffTx)) {
	// 		lib_cbufRead(&uart->cbuffTx, &c, 1);
	// 		*(uart->base + datar) = c;
	// 	}
	// 	else {
	// 		*(uart->base + ctrlr) &= ~(1 << 23);
	// 		break;
	// 	}
	// }

	return 0;


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

	// return 0;
}

/* Device interface */

static ssize_t uart_read(unsigned int minor, addr_t offs, void *buff, size_t len, time_t timeout)
{

	// uart_t *uart;
	// size_t cnt;
	// char *ram1 = (char *)0x20008000;
	// char *chbuff = (char *)buff;

	// if ((uart = uart_getInstance(minor)) == NULL)
	// 	return -EINVAL;

	// if (len == 0)
	// 	return 0;
	// /* read last n unread bytes */
	// for (cnt = 0; cnt < len; cnt++, uart->cnt++) {
	// 	((unsigned char *)buff)[cnt] = ram1[uart->cnt];
	// }

	// return (ssize_t)cnt;

	/* below is copy from imxrt106x */
	size_t res;
	uart_t *uart;
	time_t start;

	if ((uart = uart_getInstance(minor)) == NULL)
		return -EINVAL;

	start = hal_timerGet();
	while (lib_cbufEmpty(&uart->cbuffRx)) {
		if ((hal_timerGet() - start) >= timeout)
			return -ETIME;
	}

	hal_interruptsDisable();
	res = lib_cbufRead(&uart->cbuffRx, buff, len);
	hal_interruptsEnable();

	return res;
}


void uart_send(uart_t *uart, size_t len)
{
	*(uart->base + uarte_txd_maxcnt) = len;
	*(uart->base + uarte_starttx) = 1u;
	while ( *(uart->base + uarte_events_txstarted) != 1u )
		;
	*(uart->base + uarte_events_txstarted) = 0u;

	while ( *(uart->base + uarte_events_endtx) != 1u )
		;
	*(uart->base + uarte_events_endtx) = 0u;
}


static ssize_t uart_write(unsigned int minor, const void *buff, size_t len)
{
	/* sth wrong here */
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
	uart_send(uart, len);

	return (ssize_t)cnt;
}


static ssize_t uart_safeWrite(unsigned int minor, addr_t offs, const void *buff, size_t len)
{
	return uart_write(minor, buff, len);
}


static int uart_sync(unsigned int minor)
{
	// /* copy from imxrt106x */
	// uart_t *uart;

	// if ((uart = uart_getInstance(minor)) == NULL)
	// 	return -EINVAL;

	// while (!lib_cbufEmpty(&uart->cbuffTx))
	// 	;

	// /* Wait for transmission activity complete */
	// while ((*(uart->base + statr) & (1 << 22)) == 0)
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

	lib_cbufInit(&uart->cbuffTx, uart->dataTx, BUFFER_SIZE);
	lib_cbufInit(&uart->cbuffRx, uart->dataRx, BUFFER_SIZE);
	/* TODO: sizes???? not used anywhere ? */
	uart->rxFifoSz = 32; //fifoSzLut[*(uart->base + fifor) & 0x7];
	uart->txFifoSz = 32; //fifoSzLut[(*(uart->base + fifor) >> 4) & 0x7];

	/* TODO: Skip controller initialization if it has been already done by hal */
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
	*(uart->base + uarte_rxd_maxcnt) = 10;

	/* Set default uart sources: ram0 and ram1 start addresses */
	*(uart->base + uarte_txd_ptr) = 0x20000000;
	*(uart->base + uarte_rxd_ptr) = 0x20008000;

	/* disable all uart interrupts TODO: enable rx interrupts ? */
	*(uart->base + uarte_intenclr) = 0xFFFFFFFF;
	/* enable rx timeout interruts */
	*(uart->base + uarte_intenset) = 0x4; //0x10; //0x20000; //timoeut //204

	*(uart->base + uarte_enable) = 0x8;

	/* Wait for cts activation - assuming that it should be active all the time - there is no this event because it was cleared by hal console once*/
	// while ( *(uart->base + uarte_events_cts) != 1u )
	// 	;
	// *(uart->base + uarte_events_cts) = 0u;

	/* Counter that indicates how many bytes were read from ram1 buffer */
	// uart->cnt = 0;
	/* Start uart receiver */
	uart->cnt = 0;
	*(uart->base + uarte_startrx) = 1u;

	// uart->base = uart->base;
	// _stm32_rccSetDevClock(uartInfo[minor].dev, 1);

	// uart->rxdr = 0;
	// uart->rxdw = 0;
	// uart->rxflag = 0;

	// *(uart->base + cr1) = 0;
	// hal_cpuDataMemoryBarrier();

	lib_printf("\ndev/uart: Initializing uart(%d.%d)", DEV_UART, minor);
	hal_interruptsSet(uartInfo[minor].irq, uart_handleIntr, (void *)uart);

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
