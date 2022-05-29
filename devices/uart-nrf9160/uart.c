/*
 * Phoenix-RTOS
 *
 * plo - operating system loader
 *
 * nRF9160 UART driver
 *
 * Copyright 2022 Phoenix Systems
 * Author: Damian Loewnau
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
	unsigned char baudrate;
	volatile char *tx_dma;
	volatile char *rx_dma;
	unsigned char txpin;
	unsigned char rxpin;
	unsigned char rtspin;
	unsigned char ctspin;
/* there can be used other pins as well */
} uartInfo[] = {
	{ (void *)UART0_BASE, UART0_IRQ, UART_BAUDRATE, UART0_TX_DMA, UART0_RX_DMA, UART0_TX, UART0_RX, UART0_RTS, UART0_CTS },
	{ (void *)UART1_BASE, UART1_IRQ, UART_BAUDRATE, UART1_TX_DMA, UART1_RX_DMA, UART1_TX, UART1_RX, UART1_RTS, UART1_CTS },
	{ (void *)UART2_BASE, UART2_IRQ, UART_BAUDRATE, UART2_TX_DMA, UART2_RX_DMA, UART2_TX, UART2_RX, UART2_RTS, UART2_CTS },
	{ (void *)UART3_BASE, UART3_IRQ, UART_BAUDRATE, UART3_TX_DMA, UART3_RX_DMA, UART3_TX, UART3_RX, UART3_RTS, UART3_CTS }
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
	volatile char *rx_dma_buff = (char *)UART0_RX_DMA;
	volatile char *tx_dma_buff = (char *)UART0_TX_DMA;
	uart_t *uart = (uart_t *)buff;

	if (uart == NULL)
		return -EINVAL;

	if (uart->base + uarte_events_rxdrdy) {
		/* clear rxdrdy event flag */
		*(uart->base + uarte_events_rxdrdy) = 0u;
		lib_cbufWrite(&uart->cbuffRx, &rx_dma_buff[uart->cnt], 1);
		uart->cnt++;
	}
	if (uart->base + uarte_events_endrx) {
		/* clear endrx event flag */
		*(uart->base + uarte_events_endrx) = 0u;
		uart->cnt = 0;
		*(uart->base + uarte_startrx) = 1u;
		hal_cpuDataMemoryBarrier();
	}

	return 0;
}

/* Device interface */

static ssize_t uart_read(unsigned int minor, addr_t offs, void *buff, size_t len, time_t timeout)
{
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


static int uart_send(uart_t *uart, size_t len)
{
	*(uart->base + uarte_txd_maxcnt) = len;
	*(uart->base + uarte_starttx) = 1u;
	while ( *(uart->base + uarte_events_txstarted) != 1u )
		;
	*(uart->base + uarte_events_txstarted) = 0u;

	while ( *(uart->base + uarte_events_endtx) != 1u )
		;
	*(uart->base + uarte_events_endtx) = 0u;

	return EOK;
}


static ssize_t uart_write(unsigned int minor, const void *buff, size_t len)
{
	uart_t *uart;
	size_t cnt;

	if ((uart = uart_getInstance(minor)) == NULL)
		return -EINVAL;

	hal_memcpy((void *)UART0_TX_DMA, buff, len);

	uart_send(uart, len);

	return (ssize_t)cnt;
}


static ssize_t uart_safeWrite(unsigned int minor, addr_t offs, const void *buff, size_t len)
{
	return uart_write(minor, buff, len);
}


static int uart_sync(unsigned int minor)
{
	uart_t *uart;

	if ((uart = uart_getInstance(minor)) == NULL)
		return -EINVAL;

	while (!lib_cbufEmpty(&uart->cbuffTx))
		;

	/* endtx flag is asserted in uart_send so mustn't be checked here */

	return EOK;
}


static int uart_done(unsigned int minor)
{
	uart_t *uart;

	if ((uart = uart_getInstance(minor)) == NULL)
		return -EINVAL;

	/* Wait for transmission activity complete */
	(void)uart_sync(minor);

	*(uart->base + uarte_stoptx) = 1u;
	*(uart->base + uarte_stoprx) = 1u;
	/* disable all uart interrupts */
	*(uart->base + uarte_intenclr) = 0xFFFFFFFF;
	hal_cpuDataMemoryBarrier();
	*(uart->base + uarte_enable) = 0u;
	hal_cpuDataMemoryBarrier();

	hal_interruptsSet(uartInfo[minor].irq, NULL, NULL);

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


/* Init pins according to nrf9160 product specification */
static int uart_configPins(unsigned int minor)
{
	_nrf91_gpioConfig(uartInfo[minor].txpin, output, nopull);
	_nrf91_gpioConfig(uartInfo[minor].rxpin, input, nopull);
	_nrf91_gpioConfig(uartInfo[minor].rtspin, output, nopull);
	_nrf91_gpioConfig(uartInfo[minor].ctspin, input, pulldown);

	_nrf91_gpioSet(uartInfo[minor].txpin, high);
	_nrf91_gpioSet(uartInfo[minor].rtspin, high);

	return EOK;
}


static int uart_init(unsigned int minor)
{
	uart_t *uart;

	if ((uart = uart_getInstance(minor)) == NULL)
		return -EINVAL;

	uart->base = uartInfo[minor].base;
	
	if (!(*(uart->base + uarte_enable) & 0x08)) {
		uart_configPins(minor);
		/* Select pins */
		*(uart->base + uarte_psel_txd) = uartInfo[minor].txpin;
		*(uart->base + uarte_psel_rxd) = uartInfo[minor].rxpin;
		*(uart->base + uarte_psel_rts) = uartInfo[minor].rtspin;
		*(uart->base + uarte_psel_cts) = uartInfo[minor].ctspin;
	}
	else {
		/* Disable uarte instance - changing uarte config */
		*(uart->base + uarte_enable) = 0u;
		hal_cpuDataMemoryBarrier();
	}

	switch (uartInfo[minor].baudrate) {
		case 9600:
			*(uart->base + uarte_baudrate) = baud_9600;
			break;
		case 115200:
		default:
			*(uart->base + uarte_baudrate) = baud_115200;
	}

	/* Default settings - hardware flow control disabled, exclude parity bit, one stop bit */
	*(uart->base + uarte_config) = 0u;
	/* Set default max number of bytes in specific buffers */
	*(uart->base + uarte_txd_maxcnt) = UART_TX_DMA_SIZE;
	*(uart->base + uarte_rxd_maxcnt) = UART_RX_DMA_SIZE;

	/* Set default memory regions for uart dma */
	*(uart->base + uarte_txd_ptr) = uartInfo[minor].tx_dma;
	*(uart->base + uarte_rxd_ptr) = uartInfo[minor].rx_dma;

	/* Disable all uart interrupts */
	*(uart->base + uarte_intenclr) = 0xFFFFFFFF;
	/* Enable rx timeout interruts */
	*(uart->base + uarte_intenset) = 0x14;
	hal_cpuDataMemoryBarrier();

	lib_cbufInit(&uart->cbuffTx, uart->dataTx, BUFFER_SIZE);
	lib_cbufInit(&uart->cbuffRx, uart->dataRx, BUFFER_SIZE);

	/* Enable uarte instance */
	*(uart->base + uarte_enable) = 0x8;
	hal_cpuDataMemoryBarrier();
	uart->cnt = 0;
	*(uart->base + uarte_startrx) = 1u;
	hal_cpuDataMemoryBarrier();

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
