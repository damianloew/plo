/*
 * Phoenix-RTOS
 *
 * plo - operating system loader
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

#include <hal/hal.h>
#include "nrf91.h"

static struct {
	volatile u32 *power;
	volatile u32 *clock;
	volatile u32 *gpio;
	volatile u32 *timer[3];
} nrf91_common;


enum { power_tasks_constlat = 30, power_tasks_lowpwr, power_inten = 192, power_intenset, power_intenclr, power_status = 272};


enum { clock_tasks_hfclkstart, clock_inten = 192, clock_intenset, clock_intenclr, clock_hfclkrun = 258, clock_hfclkstat };


enum { gpio_out = 1, gpio_outset, gpio_outclr, gpio_in, gpio_dir, gpio_dirsetout, gpio_dirsetin, gpio_cnf = 128 };


enum { timer_tasks_start = 0, timer_tasks_stop, timer_tasks_count, timer_tasks_clear, timer_tasks_shutdown,
	timer_tasks_capture0 = 16, timer_tasks_capture1, timer_tasks_capture2, timer_tasks_capture3, timer_tasks_capture4, timer_tasks_capture5,
	timer_events_compare0 = 80, timer_events_compare1, timer_events_compare2, timer_events_compare3, timer_events_compare4, timer_events_compare5,
	timer_intenset = 193, timer_intenclr, timer_mode = 321, timer_bitmode, timer_prescaler = 324,
	timer_cc0 = 336, timer_cc1, timer_cc2, timer_cc3, timer_cc4, timer_cc5 };


/* TIMER */


int _nrf91_timerInit(u32 interval)
{
	/* Stop timer0 */
	*(nrf91_common.timer[0] + timer_tasks_stop) = 1u;
	hal_cpuDataMemoryBarrier();
	/* Set mode to timer */
	*(nrf91_common.timer[0] + timer_mode) = 0u;
	/* Set 16-bit mode */
	*(nrf91_common.timer[0] + timer_bitmode) = 0u;
	/* 1 tick per 1 us */
	*(nrf91_common.timer[0] + timer_prescaler) = 4u;
	/* 1 compare event per 1ms */
	*(nrf91_common.timer[0] + timer_cc0) = 1000u;
	/* Enable interrupts from compare0 events */
	*(nrf91_common.timer[0] + timer_intenset) = 0x10000;
	/* Clear timer0 */
	*(nrf91_common.timer[0] + timer_tasks_clear) = 1u;
	hal_cpuDataMemoryBarrier();
	/* Start timer0 */
	*(nrf91_common.timer[0] + timer_tasks_start) = 1u;
	hal_cpuDataMemoryBarrier();
	return 0;
}


void _nrf91_timerDone(void)
{
	/* Stop timer */
	*(nrf91_common.timer[0] + timer_tasks_stop) = 1u;
	hal_cpuDataMemoryBarrier();
	/* Disable all timer0 interrupts */
	*(nrf91_common.timer[0] + timer_intenclr) = 0xFFFFFFFF;
	/* Clear timer0 */
	*(nrf91_common.timer[0] + timer_tasks_clear) = 1u;
	hal_cpuDataMemoryBarrier();
}


void _nrf91_timerClearEvent(void)
{
	/* Clear compare event */
	*(nrf91_common.timer[0] + timer_events_compare0) = 0u;
	/* Clear counter */
	*(nrf91_common.timer[0] + timer_tasks_clear) = 1u;
}


/* GPIO */


int _nrf91_gpioConfig(u8 pin, u8 dir, u8 pull)
{
	if (pin > 31)
		return -1;

	if (dir == output) {
		*(nrf91_common.gpio + gpio_dirsetout) = (1u << pin);
	}
	else if (dir == input) {
		*(nrf91_common.gpio + gpio_dirsetin) = (1u << pin);
		/* connect input buffer */
		*(nrf91_common.gpio + gpio_cnf + pin) &= ~0x2;
	}
	
	if (pull) {
		*(nrf91_common.gpio + gpio_cnf + pin) = (pull << 2);
	}

	hal_cpuDataMemoryBarrier();

	return 0;
}


int _nrf91_gpioSet(u8 pin, u8 val)
{
	if (pin > 31)
		return -1;

	if (val == high) {
		*(nrf91_common.gpio + gpio_outset) = (1u << pin);
	}
	else if (val == low) {
		*(nrf91_common.gpio + gpio_outclr) = (1u << pin);
	}
	hal_cpuDataMemoryBarrier();

	return 0;
}


void _nrf91_init(void)
{
	nrf91_common.power = (void *)0x50005000;
	nrf91_common.clock = (void *)0x50005000;
	nrf91_common.gpio = (void *)0x50842500;
	nrf91_common.timer[0] = (void *)0x5000F000;
	nrf91_common.timer[1] = (void *)0x50010000;
	nrf91_common.timer[2] = (void *)0x50011000;

	/* Enable low power mode */
	*(nrf91_common.power + power_tasks_lowpwr) = 1u;

	/* Disable all power interrupts */
	*(nrf91_common.power + power_intenclr) = 0x64;

	/* Disable all clock interrupts */
	*(nrf91_common.power + power_intenclr) = 0x3;

	*(nrf91_common.clock + clock_tasks_hfclkstart) = 1u;
	/* Wait untill HXFO start and clear event flag */
	while ( *(nrf91_common.clock + clock_hfclkrun) != 1u )
		;
	*(nrf91_common.clock + clock_hfclkrun) = 0u;

	hal_cpuDataMemoryBarrier();
}
