/*
 * Phoenix-RTOS
 *
 * Operating system loader
 *
 * Initial loader's routines
 *
 * Copyright 2012, 2017, 2020-2021 Phoenix Systems
 * Copyright 2001, 2005 Pawel Pisarczyk
 * Authors: Pawel Pisarczyk, Lukasz Kosinski, Hubert Buczynski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include <hal/hal.h>
#include <lib/lib.h>
#include <cmds/cmd.h>
#include <devices/devs.h>
#include <syspage.h>

#include "nrf91.h"

int main(void)
{
	hal_init();
	syspage_init();
	lib_printf("Sending dummy string to catch gibberish output right after restart");
	lib_printf(CONSOLE_CLEAR CONSOLE_BOLD "Phoenix-RTOS loader v. " VERSION CONSOLE_NORMAL);
	lib_printf(CONSOLE_CURSOR_HIDE CONSOLE_MAGENTA "\nhal: %s", hal_cpuInfo());
	_nrf91_gpioConfig(2, output, nopull);
	devs_init();
	// lib_printf("__bss_end[] = %x\n", __bss_end);
	cmd_run();

	lib_printf(CONSOLE_CURSOR_SHOW CONSOLE_NORMAL);
	cmd_prompt();

	devs_done();
	hal_done();

	int timestamp;
	u8 state;
	_nrf91_gpioConfig(2, output, nopull);
	for (int i = 0; i <=10; i++) {
		timestamp = hal_timerGet();
		while ((hal_timerGet() - timestamp) <= 1000) ;
		if (state == low) {
			_nrf91_gpioSet(2, high);
			state = high;
		}
		else {
			_nrf91_gpioSet(2, low);
			state = low;
		}
	}
	while (1) ;
	return 0;
}
