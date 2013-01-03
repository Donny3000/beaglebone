/*
 * gpio.h
 *
 *  Created on: Dec 24, 2012
 *      Author: Donald R. Poole, Jr.
 */

#ifndef GPIO_H_
#define GPIO_H_

#define GPIO1_START_ADDR 0x4804C000
#define GPIO1_END_ADDR 0x4804DFFF
#define GPIO1_SIZE (GPIO1_END_ADDR - GPIO1_START_ADDR)

// GPIO memory-mapped register addresses offsets
#define GPIO_REVISION			0x0
#define GPIO_SYSCONFIG			0x10
#define GPIO_IRQSTATUS_RAW_0	0x24
#define GPIO_IRQSTATUS_RAW_1	0x28
#define GPIO_IRQSTATUS_0		0x2C
#define GPIO_IRQSTATUS_1		0x30
#define GPIO_IRQSTATUS_SET_0	0x34
#define GPIO_IRQSTATUS_SET_1    0x38
#define GPIO_IRQSTATUS_CLR_0	0x3C
#define GPIO_IRQSTATUS_CLR_1    0x40
#define GPIO_IRQWAKEN_0			0x44
#define GPIO_IRQWAKEN_1			0x48
#define GPIO_SYSSTATUS          0x114
#define GPIO_CTRL				0x130
#define GPIO_OE					0x134
#define GPIO_DATAIN				0x138
#define GPIO_DATAOUT			0x13C
#define GPIO_LEVELDETECT0		0x140
#define GPIO_LEVELDETECT1		0x144
#define GPIO_RISINGDETECT		0x148
#define GPIO_FALLINGDETECT		0x14C
#define GPIO_DEBOUNCENABLE		0x150
#define GPIO_DEBOUNCINGTIME		0x154
#define GPIO_CLEARDATAOUT		0x190
#define GPIO_SETDATAOUT			0x194

// User LEDs
#define USR0_LED				(1<<21) // GPIO1_21
#define USR1_LED				(1<<22) // GPIO1_22
#define USR2_LED				(1<<23) // GPIO1_23
#define USR3_LED				(1<<24) // GPIO1_24

// Expansion Header P8 GPIOs
#define GPIO1_0					(1<<0)
#define GPIO1_1					(1<<1)
#define GPIO1_2					(1<<2)
#define GPIO1_3					(1<<3)
#define GPIO1_4					(1<<4)
#define GPIO1_5					(1<<5)
#define GPIO1_6					(1<<6)
#define GPIO1_7					(1<<7)

#endif /* GPIO_H_ */
