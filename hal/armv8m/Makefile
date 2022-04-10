#
# Makefile for Phoenix-RTOS loader (ARMv7M HAL)
#
# Copyright 2021 Phoenix Systems
#
# %LICENSE%
#

ifneq (, $(findstring nrf91, $(TARGET_SUBFAMILY)))
	include hal/armv8m/nrf/Makefile
endif

OBJS += $(addprefix $(PREFIX_O)hal/$(TARGET_SUFF)/, cpu.o interrupts.o mpu.o string.o)
