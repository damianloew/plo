/*
 * Phoenix-RTOS
 *
 * Operating system kernel
 *
 * nRF9160 basic peripherals control functions
 *
 * Copyright 2022 Phoenix Systems
 * Author: Damian Loewnau
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#ifndef _HAL_NRF91_H_
#define _HAL_NRF91_H_

#include "../types.h"


enum { input = 0, output };
enum { low = 0, high };
enum { nopull = 0, pulldown, pullup = 3};


/* nRF9160 peripheral id's - same as irq numbers */
enum { spu = 3, regulators, clock = 5, power = 5, ctrlapperi, spi0 = 8, twi0 = 8, uarte0 = 8,
spi1 = 9, twi1 = 9, uarte1 = 9, spi2 = 10, twi2 = 10, uarte2 = 10, spi3 = 11, twi3 = 11, uarte3 = 11,
gpiote0 = 13, saadc, timer0, timer1, timer2, rtc0 = 20, rtc1, ddpic = 23, wdt, 
egu0 = 27, egu1, egu2, egu3, egu4, egu5, pwm0, pwm1, pwm2, pwm3, pdm = 38, 
i2s = 40, ipc = 42, fpu = 44, gpiote1 = 49, kmu = 57, nvmc = 57, vmc, cc_host_rgf = 64,
cryptocell = 64, gpio = 66 };


enum { uarte_startrx = 0, uarte_stoprx, uarte_starttx, uarte_stoptx, uarte_flushrx = 11,
uarte_events_cts = 64, uarte_events_rxdrdy = 66, uarte_events_endrx = 68, uarte_events_txdrdy = 71, uarte_events_endtx, uarte_events_error, uarte_events_rxto = 81, uarte_events_txstarted = 84, 
uarte_inten = 192, uarte_intenset, uarte_intenclr, uarte_errorsrc = 288, uarte_enable = 320, 
uarte_psel_rts = 322, uarte_psel_txd, uarte_psel_cts, uarte_psel_rxd, uarte_baudrate = 329, 
uarte_rxd_ptr = 333, uarte_rxd_maxcnt, uarte_rxd_amount, uarte_txd_ptr = 337, uarte_txd_maxcnt, uarte_txd_amount, 
uarte_config = 347 };


enum { baud_9600 = 0x00275000, baud_115200 = 0x01D60000 };


extern int _nrf91_gpioConfig(u8 pin, u8 dir, u8 pull);


extern int _nrf91_gpioSet(u8 pin, u8 val);


extern void _nrf91_timerClearEvent(void);


extern void _nrf91_timerDone(void);


extern int _nrf91_timerInit(u32 interval);


extern void _nrf91_init(void);

#endif
