/*
 * Phoenix-RTOS
 *
 * Operating system loader
 *
 * Timer driver
 *
 * Copyright 2021 Phoenix Systems
 * Author: Aleksander Kaminski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include <hal/hal.h>
#include "nrf91.h"

/* based on 0x14 in peripheral base address */
#define RTC0_IRQ 20
// #define RTC0_IRQ 36

struct {
	volatile time_t time;
	unsigned int interval;
} timer_common;


static int timer_isr(unsigned int irq, void *data)
{
	(void)irq;
	(void)data;

	_nrf91_rtcClearEvent();
	/* in fact += 1.007 ms */
	timer_common.time += 1;
	hal_cpuDataSyncBarrier();
	return 0;
}


time_t hal_timerGet(void)
{
	time_t val;

	hal_interruptsDisable();
	val = timer_common.time;
	hal_interruptsEnable();

	return val;
}


void timer_done(void)
{
	_nrf91_rtckDone();
	hal_interruptsSet(RTC0_IRQ, NULL, NULL);
}


void timer_init(void)
{
	timer_common.time = 0;
	timer_common.interval = 1000;
	_nrf91_rtcInit(timer_common.interval);
	hal_interruptsSet(RTC0_IRQ, timer_isr, NULL);
}
