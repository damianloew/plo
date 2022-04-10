/*
 * Phoenix-RTOS
 *
 * plo - operating system loader
 *
 * Low-level initialization for Cortex-M7 (ARMv8) architecture
 *
 * Copyright 2012, 2016-2017, 2020, 2021 Phoenix Systems
 * Author: Jacek Popko, Pawel Pisarczyk, Jakub Sejdak, Aleksander Kaminski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#define __ASSEMBLY__

// #include "config.h"

// set unified instruction set syntax - unified for arm and thumb
.syntax unified

//assemble the following code into a init section (x - an executable section)
.section .init, "x"

//make the symbol _init_vectors visible for ld
.globl _init_vectors

//sets the type of a symbol to object
.type _init_vectors, %object

//.word is like word type (?) - 4 bytes, there are the next values like in structure (?)
_init_vectors:
.word _stack
.word _start

.word _exceptions_dispatch  /* NMI */
.word _exceptions_dispatch  /* HardFault */
.word _exceptions_dispatch  /* MemMgtFault */
.word _exceptions_dispatch  /* BusFault */
.word _exceptions_dispatch  /* UsageFault */
.word _exceptions_dispatch  /* Secure Fault */ // #doc: p84 in armv8m rm
.word 0
.word 0
.word 0
.word 0                     /* SVC */
.word 0                     /* Debug */
.word 0
.word _interrupts_dispatch  /* PendSV */
.word _interrupts_dispatch  /* Systick */

//repeat 217 times word _inter....
.rept 217 // TODO: why 217, can't find anything in doc
.word _interrupts_dispatch  /* _interrupts_dispatch */
.endr
//sets the size 
.size _init_vectors, .-_init_vectors

//selects thumb instruction set being generated
.thumb
//specifies that the following symbol is the name of a Thumb encoded function (_start ?)
.thumb_func

// type of a start symbol set to function
.globl _start
.type _start, %function
_start:
	cpsid if 	//enables irq and fiq interrupts

	/* Init vector table and stack pointer */
	ldr r0, =0xe000ed08 //loads r0 with the following constant - vector table offset register address, verified in armv8m doc
	ldr r1, =_init_vectors //loads the init vector pointer
	str r1, [r0] //store r1 in r0 address ? so r1 content is placed on stack (?) our _init_vectors is placed in vector table
	//instruction synchronization barier - flushes the pipeline in the processor, so that all instructions following the ISB are fetched from cache or memory
	isb
	//Data Memory Barrier acts as a memory barrier. It ensures that all explicit memory accesses that appear in program order before
	//the DMB instruction are observed before any explicit memory accesses that appear in program order after the DMB instruction
	dmb

	//the stack value (init vectors) is back in the r1 , no (?) load r0 with r1???
	ldr r0, [r1]
	//bit clear - r0 = r0 & 0111 - clears all above fourth bit in stack (?) TODO: comments for the further part of code!
	bic r0, 7
	msr msp, r0
	ldr r8, =_startc
	bx r8
.size _start, .-_start
.ltorg


.globl _interrupts_dispatch
.type _interrupts_dispatch, %function
_interrupts_dispatch:
	mrs r0, ipsr
	stmdb sp!, {r0, r4-r11, lr}
	bl hal_interruptDispatch
	ldmia sp!, {r0, r4-r11, lr}
	dmb

	bx lr
.size _interrupts_dispatch, .-_interrupts_dispatch
.ltorg


.globl _exceptions_dispatch
.type _exceptions_dispatch, %function

_exceptions_dispatch:
	cpsid if
	/* TODO: implement exception dispatcher */
	1: b 1b
.size _exceptions_dispatch, .-_exceptions_dispatch
.ltorg
