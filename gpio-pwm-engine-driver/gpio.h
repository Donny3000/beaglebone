/*
 * gpio.h
 *
 *  Created on: Dec 24, 2012
 *      Author: Donald R. Poole, Jr.
 */

#ifndef GPIO_H_
#define GPIO_H_

/*******************************/
/* CAN Bus Registers Addresses */
/*******************************/

// CAN Bus 0
#define DCAN0_START_ADDR		0x481CC000
#define DCAN0_END_ADDR			0x481CDFFF
#define DCAN0_SIZE				(DCAN0_END_ADDR - DCAN0_START_ADDR)
// CAB Bus 1
#define DCAN1_START_ADDR		0x481D0000
#define DCAN1_END_ADDR			0x481D3FFF
#define DCAN1_SIZE				(DCAN1_END_ADDR - DCAN1_START_ADDR)

/***********************/
/* GPIO Bank Addresses */
/***********************/

// GPIO Bank 0
#define GPIO0_START_ADDR 		0x44E07000
#define GPIO0_END_ADDR 			0x44E08FFF
#define GPIO0_REG_START			0x7000
#define GPIO0_SIZE				(GPIO0_END_ADDR - GPIO0_START_ADDR)
// GPIO Bank 1
#define GPIO1_START_ADDR 		0x4804C000
#define GPIO1_END_ADDR 			0x4804DFFF
#define GPIO1_REG_START			0xC000
#define GPIO1_SIZE				(GPIO1_END_ADDR - GPIO1_START_ADDR)
// GPIO Bank 2
#define GPIO2_START_ADDR 		0x481AC000
#define GPIO2_END_ADDR 			0x481ADFFF
#define GPIO2_REG_START			0xC000
#define GPIO2_SIZE				(GPIO2_END_ADDR - GPIO2_START_ADDR)
// GPIO Bank 3
#define GPIO3_START_ADDR 		0x481AE000
#define GPIO3_END_ADDR 			0x481AFFFF
#define GPIO3_REG_START			0xE000
#define GPIO3_SIZE				(GPIO3_END_ADDR - GPIO3_START_ADDR)

/************************************************/
/*GPIO memory-mapped register addresses offsets */
/************************************************/
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

// GPIO NUEMONICs
/*
 * IEN  - Input Enable
 * IDIS - Input Disable
 * PTD  - Pull type Down
 * PTU  - Pull type Up
 * DIS  - Pull type selection is inactive
 * EN   - Pull type selection is active
 * M0   - Mode 0
 *
 * Control register field description
 * --------------------------------------------------------------------
 *  Bit |          | Value  | Description
 * --------------------------------------------------------------------
 * 31-7 | Reserved |        | Reserved. Read returns 0
 * --------------------------------------------------------------------
 * 6    | SLEWCTRL |        | Select between faster of slower slew rate
 *      |          | 0      | Fast
 *      |          | 1      | Slow
 * --------------------------------------------------------------------
 * 5    | RXACTIVE |        | Input enabled value for the pad. Set to 0
 *      |          |        | for output only. Set to 1 for input of output
 *      |          | 0      | Receiver disabled
 *      |          | 1      | Receiver Enabled
 */

#define SLEW_FAST               (0 << 6)
#define SLEW_SLOW               (1 << 6)
#define INPUT_EN                (1 << 5)
#define INPUT_DIS               (0 << 5)
#define PULLUP                  (1 << 4)
#define PULLDOWN                (0 << 4)
#define PULLUPDOWN_EN           (0 << 3)
#define PULLUPDOWN_DIS          (1 << 3)

#define M0                      0
#define M1                      1
#define M2                      2
#define M3                      3
#define M4                      4
#define M5                      5
#define M6                      6
#define M7                      7

// GPIO pin pad configuration registers offsets
#define GPMC_CONF_ADDR_START    0x44E10000
#define GPMC_CONF_ADDR_END      0x44E11FFF
#define GPMC_CONF_ADDR_SIZE     (GPMC_CONF_ADDR_END - GPMC_CONF_ADDR_START)
#define CONF_GPMC_AD0           0x800 // GPIO1_0
#define CONF_GPMC_AD1           0x804 // GPIO1_1
#define CONF_GPMC_AD2           0x808 // GPIO1_2
#define CONF_GPMC_AD3           0x80C // GPIO1_3
#define CONF_GPMC_AD4           0x810 // GPIO1_4
#define CONF_GPMC_AD5           0x814 // GPIO1_5
#define CONF_GPMC_AD6           0x818 // GPIO1_6
#define CONF_GPMC_AD7           0x81C // GPIO1_7

#define CONF_LCD_DATA0          0x8A0 // GPIO2_6
#define CONF_LCD_DATA1          0x8A4 // GPIO2_7
#define CONF_LCD_DATA2          0x8A8 // GPIO2_8
#define CONF_LCD_DATA3          0x8AC // GPIO2_9
#define CONF_LCD_DATA4          0x8B0 // GPIO2_10
#define CONF_LCD_DATA5          0x8B4 // GPIO2_11
#define CONF_LCD_DATA6          0x8B8 // GPIO2_12
#define CONF_LCD_DATA7          0x8BC // GPIO2_13

#define GPIO_0					(1<<0)
#define GPIO_1					(1<<1)
#define GPIO_2					(1<<2)
#define GPIO_3					(1<<3)
#define GPIO_4					(1<<4)
#define GPIO_5					(1<<5)
#define GPIO_6					(1<<6)
#define GPIO_7					(1<<7)
#define GPIO_8					(1<<8)
#define GPIO_9					(1<<9)
#define GPIO_10					(1<<10)
#define GPIO_11					(1<<11)
#define GPIO_12					(1<<12)
#define GPIO_13					(1<<13)
#define GPIO_14					(1<<14)
#define GPIO_15					(1<<15)
#define GPIO_16					(1<<16)
#define GPIO_17					(1<<17)
#define GPIO_18					(1<<18)
#define GPIO_19					(1<<19)
#define GPIO_20					(1<<20)
#define USR0_LED				(1<<21) // GPIO1_21
#define USR1_LED				(1<<22) // GPIO1_22
#define USR2_LED				(1<<23) // GPIO1_23
#define USR3_LED				(1<<24) // GPIO1_24
#define GPIO_25					(1<<25)
#define GPIO_26					(1<<26)
#define GPIO_27					(1<<27)
#define GPIO_28					(1<<28)
#define GPIO_29					(1<<29)
#define GPIO_30					(1<<30)
#define GPIO_31					(1<<31)

#endif /* GPIO_H_ */
