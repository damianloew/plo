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
#include <../libphoenix/include/time.h>
extern int usleep(useconds_t usecs);

// #include <lib/lib.h>
// #include <cmds/cmd.h>
// #include <devices/devs.h>
// #include <syspage.h>
#define OUTSET 0x008


int main(void)
{
	volatile unsigned int *vmc_powerset_secure_base = (unsigned int *)0x5003A604; //+ 0x608;
	volatile unsigned int *gpio_secure_base = (unsigned int *)0x50842500;
	volatile unsigned int *gpio_nonsecure_base = (unsigned int *)0x40842500;
	volatile unsigned int *outset_secure = gpio_secure_base + 0x008;
	volatile unsigned int *outclr_secure = gpio_secure_base + 0x00C;
	volatile unsigned int *dirset_secure = gpio_secure_base + 0x018;
	volatile unsigned int *outset_nonsecure = gpio_nonsecure_base + 0x008;
	volatile unsigned int *outclr_nonsecure = gpio_nonsecure_base + 0x00C;
	volatile unsigned int *dirset_nonsecure = gpio_nonsecure_base + 0x018;

	*vmc_powerset_secure_base = 0xFFFFF;//0xF << 16;
	// *dirset_secure = 1u << 2;
	// *outset_secure = 1u << 2;
	// *dirset_nonsecure = 1u << 2;
	// *outset_nonsecure = 1u << 2;

	// *outclr = 1u << 2;
	// for(int i = 0; i < 9; i++)
	// {
	// 	*outset = 1u << 2;
	// 	usleep(500000);
	// 	*outclr = 1u << 2;
	// 	usleep(500000);
	// }

	// hal_init();
	// hal_consolePrint("hello from nrf9160 plo!\n");



	// syspage_init();

	// lib_printf(CONSOLE_CLEAR CONSOLE_BOLD "Phoenix-RTOS loader v. " VERSION CONSOLE_NORMAL);
	// lib_printf(CONSOLE_CURSOR_HIDE CONSOLE_MAGENTA "\nhal: %s", hal_cpuInfo());
	// devs_init();
	// cmd_run();

	// lib_printf(CONSOLE_CURSOR_SHOW CONSOLE_NORMAL);
	// cmd_prompt();

	// devs_done();
	// hal_done();

	return 0;
}
