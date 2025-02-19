/*
 * Phoenix-RTOS
 *
 * Operating system kernel
 *
 * STM32L4x6 basic peripherals control functions
 *
 * Copyright 2017, 2020, 2021 Phoenix Systems
 * Author: Aleksander Kaminski, Pawel Pisarczyk, Hubert Buczynski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#ifndef _HAL_STM32L4_H_
#define _HAL_STM32L4_H_

#include "../types.h"

/* STM32L4 peripherals */
enum {
	/* AHB1 */
	pctl_dma1 = 0, pctl_dma2, pctl_flash = 8, pctl_crc = 12, pctl_tsc = 16, pctl_dma2d,

	/* AHB2 */
	pctl_gpioa = 32, pctl_gpiob, pctl_gpioc, pctl_gpiod, pctl_gpioe, pctl_gpiof, pctl_gpiog, pctl_gpioh, pctl_gpioi,
	pctl_otgfs = 32 + 12, pctl_adc, pctl_dcmi, pctl_aes = 32 + 16, pctl_hash, pctl_rng,

	/* AHB3 */
	pctl_fmc = 64, pctl_qspi = 64 + 8,

	/* APB1_1 */
	pctl_tim2 = 96, pctl_tim3, pctl_tim4, pctl_tim5, pctl_tim6, pctl_tim7, pctl_lcd = 96 + 9, pctl_rtcapb,
	pctl_wwdg, pctl_spi2 = 96 + 14, pctl_spi3, pctl_usart2 = 96 + 17, pctl_usart3, pctl_uart4, pctl_uart5, pctl_i2c1,
	pctl_i2c2, pctl_i2c3, pctl_crs, pctl_can1, pctl_can2, pctl_pwr = 96 + 28, pctl_dac1, pctl_opamp, pctl_lptim1,

	/* APB1_2 */
	pctl_lpuart1 = 128, pctl_i2c4, pctl_swpmi1, pctl_lptim2 = 128 + 5,

	/* APB2 */
	pctl_syscfg = 160, pctl_fw = 160 + 7, pctl_sdmmc1 = 160 + 10, pctl_tim1, pctl_spi1, pctl_tim8, pctl_usart1,
	pctl_tim15, pctl_tim16, pctl_tim17, pctl_sai1 = 160 + 21, pctl_sai2, pctl_dfsdm1 = 160 + 24,

	/* MISC */
	pctl_rtc = 192
};


/* STM32L4 Interrupt numbers */
enum { wwdq_irq = 16, pvd_pvm_irq, rtc_tamper_stamp_irq, rtc_wkup_irq, flash_irq, rcc_irq,
	exti0_irq, exti1_irq, exti2_irq, exti3_irq, exti4_irq, dma1_ch1_irq, dma1_ch2_irq,
	dma1_ch3_irq, dma1_ch4_irq, dma1_ch5_irq, dma1_ch6_irq, dma1_ch7_irq, adc1_2_irq,
	can1_tx_irq, can1_rx0_irq, can1_rx1_irq, can1_sce_irq, exti9_5_irq, tim1_brk_irq,
	tim1_up_irq, tim1_trg_com_irq, tim1_cc_irq, tim2_irq, tim3_irq, tim4_irq, i2c1_ev_irq,
	i2c1_er_irq, i2c2_ev_irq, i2c2_er_irq, spi1_irq, spi2_irq, usart1_irq, usart2_irq,
	usart3_irq, exti15_10_irq, rtc_alarm_irq, dfsdm1_flt3_irq, tim8_brk_irq, tim8_up_irq,
	tim8_trg_com_irq, tim8_cc_irq, adc3_irq, fmc_irq, sdmmc1_irq, tim5_irq, spi3_irq,
	uart4_irq, uart5_irq, tim6_dacunder_irq, tim7_irq, dma2_ch1_irq, dma2_ch2_irq,
	dma2_ch3_irq, dma2_ch4_irq, dma2_ch5_irq, dfsdm1_flt0_irq, dfsdm1_flt1_irq, dfsdm1_flt2_irq,
	comp_irq, lptim1_irq, lptim2_irq, otg_fs_irq, dm2_ch6_irq, dma2_ch7_irq, lpuart1_irq,
	quadspi_irq, i2c3_ev_irq, i2c3_er_irq, sai1_irq, sai2_irq, swpmi1_irq, tsc_irq, lcd_irq,
	aes_irq, rng_irq, fpu_irq, hash_irq, i2c4_ev_irq, i2c4_er_irq, dcmi_irq, can2_tx_irq,
	can2_rx0_irq, can2_rx1_irq, can2_sce_irq, dma2d_irq };


/* Sets peripheral clock */
extern int _stm32_rccSetDevClock(unsigned int d, u32 hz);


/* Sets CPU clock to the closest smaller MSI frequency */
extern int _stm32_rccSetCPUClock(u32 hz);


extern int _stm32_rccGetDevClock(unsigned int d, u32 *hz);


extern u32 _stm32_rccGetCPUClock(void);


extern void _stm32_rccClearResetFlags(void);


extern int _stm32_gpioConfig(unsigned int d, u8 pin, u8 mode, u8 af, u8 otype, u8 ospeed, u8 pupd);


extern int _stm32_gpioSet(unsigned int d, u8 pin, u8 val);


extern int _stm32_gpioSetPort(unsigned int d, u16 val);


extern int _stm32_gpioGet(unsigned int d, u8 pin, u8 *val);


extern int _stm32_gpioGetPort(unsigned int d, u16 *val);


/* Range = 0 - forbidden, 1 - 1.8V, 2 - 1.5V, 3 - 1.2V */
extern void _stm32_pwrSetCPUVolt(u8 range);


extern void _stm32_pwrEnterLPRun(u32 state);


extern time_t _stm32_pwrEnterLPStop(time_t us);


extern void _stm32_rtcUnlockRegs(void);


extern void _stm32_rtcLockRegs(void);


extern u32 _stm32_rtcGetms(void);


extern int _stm32_systickInit(u32 interval);


extern void _stm32_systickDone(void);


extern void _stm32_wdgReload(void);


extern int _stm32_systickInit(u32 interval);


extern void _stm32_init(void);

#endif
