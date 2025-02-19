/*
 * Phoenix-RTOS
 *
 * Operating system loader
 *
 * Interrupt handling - NVIC (Nested Vectored Interrupt Controller)
 *
 * Copyright 2017, 2019, 2020 Phoenix Systems
 * Author: Aleksander Kaminski, Jan Sikorski, Hubert Buczynski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include <hal/hal.h>


typedef struct {
	void *data;
	int (*isr)(unsigned int, void *);
} intr_handler_t;


enum { nvic_iser = 0, nvic_icer = 32, nvic_ispr = 64, nvic_icpr = 96, nvic_iabr = 128,
	nvic_ip = 192, nvic_stir = 896 };


struct{
	intr_handler_t irqs[SIZE_INTERRUPTS];
	volatile u32 *nvic;
} irq_common;


static void interrupts_nvicSetIRQ(s8 irqn, u8 state)
{
	volatile u32 *ptr = irq_common.nvic + ((u8)irqn >> 5) + (state ? nvic_iser : nvic_icer);
	*ptr = 1 << (irqn & 0x1F);

	hal_cpuDataSyncBarrier();
	hal_cpuInstrBarrier();
}


static void interrupts_nvicSetPriority(s8 irqn, u32 priority)
{
	volatile u8 *ptr;

	ptr = ((u8 *)(irq_common.nvic + nvic_ip)) + irqn;

	*ptr = (priority << 4) & 0x0ff;
}


void hal_interruptsEnable(void)
{
	__asm__ volatile("cpsie if");
}


void hal_interruptsDisable(void)
{
	__asm__ volatile("cpsid if");
}


int hal_interruptsSet(unsigned int irq, int (*isr)(unsigned int, void *), void *data)
{
	if (irq >= SIZE_INTERRUPTS)
		return -1;

	hal_interruptsDisable();
	irq_common.irqs[irq].isr = isr;
	irq_common.irqs[irq].data = data;

	if (isr == NULL) {
		interrupts_nvicSetIRQ(irq - 0x10, 0);
	}
	else {
		interrupts_nvicSetPriority(irq - 0x10, 0);
		interrupts_nvicSetIRQ(irq - 0x10, 1);
	}
	hal_interruptsEnable();

	return 0;
}


int hal_interruptDispatch(unsigned int irq)
{
	if (irq_common.irqs[irq].isr == NULL)
		return -1;

	irq_common.irqs[irq].isr(irq, irq_common.irqs[irq].data);

	return 0;
}


void interrupts_init(void)
{
	irq_common.nvic = (void *)0xe000e100;
}
