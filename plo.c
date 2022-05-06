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
#include "nrf91.h"

extern int usleep(useconds_t usecs);

// #include <lib/lib.h>
// #include <cmds/cmd.h>
// #include <devices/devs.h>
// #include <syspage.h>
#define OUTSET 0x008


int main(void)
{
	int j;
	volatile unsigned int *vmc_powerset_secure_base = (unsigned int *)0x5003A604; //+ 0x608;
	volatile unsigned int *gpio_secure_base_dir_base = (unsigned int *)0x50842500;
	volatile unsigned int *gpio_secure_base_dirset = (unsigned int *)0x50842518;
	volatile unsigned int *gpio_secure_base_outset = (unsigned int *)0x50842508;
	volatile unsigned int *gpio_secure_base_outclr = (unsigned int *)0x5084250C;
	// volatile unsigned int *gpio_nonsecure_base = (unsigned int *)0x40842500;
	// volatile unsigned int *outset_secure = gpio_secure_base + 0x008;
	// volatile unsigned int *outclr_secure = gpio_secure_base + 0x00C;
	// volatile unsigned int *dirset_secure = gpio_secure_base + 0x018;
	volatile unsigned int *clock_tasks_hfclk_start = (unsigned int *)0x50005000;
	volatile unsigned int *clock_status_hfclk_start = (unsigned int *)0x50005408;

	// *clock_tasks_hfclk_start = 0x01;
	// while (*clock_status_hfclk_start != 0x01) {
	// 	;
	// }

	*vmc_powerset_secure_base = 0xFFFFF;//0xF << 16;
	// *(gpio_secure_base_dir_base + 6) = (1u << 2); // *gpio_secure_base_dirset = 1u << 2;
	// *gpio_secure_base_dirset = 1u << 3;

	// *gpio_secure_base_outset = 1u << 3;
	// *gpio_secure_base_outclr = 1u << 2;

	// for(int i=0; i <=2; i++) {;}

	// for(int i = 0; i <= 3000000000; i++) {
	// 	j++;
	// }
	// *gpio_secure_base_outclr = 1u << 2;
	// for(int i = 0; i <= 100000000000000; i++) {
	// 	j++;
	// }
	// *gpio_secure_base_outset = 1u << 3;

	// *gpio_secure_base_outset = 1u << 2;

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
	hal_init();

	// *gpio_secure_base_dirset = 1u << 3;

	// *gpio_secure_base_outset = 1u << 3;

	_nrf91_gpioConfig(2, output, pullup);
	_nrf91_gpioConfig(3, output, pulldown);
	_nrf91_gpioConfig(2, output, pullup);
	_nrf91_gpioConfig(3, output, pulldown);

	_nrf91_gpioSet(2, high);
	_nrf91_gpioSet(3, high);

	// _nrf91_gpioSet(2, low);
	// _nrf91_gpioSet(3, low);

	// *gpio_secure_base_dirset = 1u << 2;
	// *gpio_secure_base_outset = 1u << 2;


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

	while (1) ;
	return 0;
}
