/*
 * Phoenix-RTOS
 *
 * Operating system loader
 *
 * Interrupt handling - NVIC (Nested Vectored Interrupt Controller)
 *
 * Copyright 2017, 2019, 2020 Phoenix Systems
 * Author: Aleksander Kaminski, Jan Sikorski, Hubert Buczynski, Damian Loewnau
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
	nvic_ip = 192 };


struct{
	intr_handler_t irqs[SIZE_INTERRUPTS];
	volatile u32 *nvic; //was u32 before
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
	volatile u32 *ptr;

	ptr = ((u32 *)(irq_common.nvic + nvic_ip)) + (irqn / 4); //TODO: complete this function

	*ptr = (priority << (8 * (irqn % 4)));
}


void hal_interruptsEnable(void)
{
	__asm__ volatile("cpsie if");
}


void hal_interruptsDisable(void)
{
	__asm__ volatile("cpsid if");
}


/* if isr is set to NULL - disables the interrupt */
int hal_interruptsSet(unsigned int irq, int (*isr)(unsigned int, void *), void *data)
{
	if (irq >= SIZE_INTERRUPTS)
		return -1;

	hal_interruptsDisable();
	irq_common.irqs[irq].isr = isr;
	irq_common.irqs[irq].data = data;

	if (isr == NULL) {
		/* there was irq - 0x10 before, but it requires adding 16 to irq number */
		interrupts_nvicSetIRQ(irq, 0);
	}
	else {
		interrupts_nvicSetPriority(irq, 0);
		interrupts_nvicSetIRQ(irq, 1);
	}
	hal_interruptsEnable();

	return 0;
}


int hal_interruptDispatch(unsigned int exceptionNr)
{
	unsigned int irq = exceptionNr - 16;
	if (irq_common.irqs[irq].isr == NULL) //was null
		return -1;

	irq_common.irqs[irq].isr(irq, irq_common.irqs[irq].data);

	return 0;
}


void interrupts_init(void)
{
	/* nvic_iser0 register address */
	irq_common.nvic = (void *)0xe000e100;
}
