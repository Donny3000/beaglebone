#ifndef __GPE_PWM_H
#define __GPE_PWM_H

#include <linux/module.h>
#include <linux/gpio.h>
#include <linux/clk.h>
#include <linux/pwm.h>
#include <linux/platform_device.h>

#include <plat/clock.h>

#include "gpe-types.h"

#define PWM_MIN_TICK_COUNT 0xC3
#define PWM_MAX_TICK_COUNT 0x186
#define PWM_PRD_TICK_COUNT 0xF42
#define PWM_NS_PER_STEP    5120

/*
 * Efficient divide routine using bitwise operations and shifts
 */
uint64_t div(uint64_t numerator, uint64_t denominator);

/*
 * Initialize the GPE
 *
 * @param nchannels - number of required channels. The value must be
 * between 1 and 8.
 *
 * @return 0 on success or error code in case of failure.
 */
int init_pwm(uchar nchannels);

/*
 * Initialize the ePWM 0 Module
 *
 * @param - None
 * 
 * @return - 0 on success, -1 otherwise.
 */
int ehrpwm_0_config(void);

/*
 * Initialize the ePWM 1 Module
 *
 * @param - None
 * 
 * @return - 0 on success, -1 otherwise.
 */
int ehrpwm_1_config(void);

/*
 * Initialize the ePWM 2 Module
 *
 * @param - None
 * 
 * @return - 0 on success, -1 otherwise.
 */
int ehrpwm_2_config(void);

/*
 * Enable the clocks controlling the PWM sub-systems
 *
 * @param epwmss_clk_mod - The address of the L4 Peripheral clock configuration
 * registers
 *
 * @return - 0 on success, -1 otherwise.
 */
int config_l4_clk(ulong epwmss_clk_mod);

/*
 * Configure the PWMSS configuration module
 *
 * @param addr - The address of the PWMSS configuration registers
 * @param size - The size of the address space being mapped.
 *
 * @returns - 0 on success, -1 otherwise.
 */
int pwmss_config(ulong addr, uint size);

/*
 * Initialize and configure the Time-Based sub-module
 *
 * @param addr - Start address of the Time-Based sub-module register
 * @param size - The size of the Time-Based sub-module register memory area
 *
 * @returns - 0 on success, -1 otherwise.
 */
int ehrpwm_config_tb_module(ulong addr, uint size);

/*
 * Initialize and configure the Counter-Compare sub-module
 *
 * @param addr - Start address of the Counter-Compare sub-module register
 * @param size - The size of the Counter-Compare sub-module register memory area
 *
 * @returns - 0 on success, -1 otherwise.
 */
int ehrpwm_config_cc_module(ulong addr, uint size);

/*
 * Initialize and configure the Action-Qualifier sub-module
 *
 * @param addr - Start address of the Action-Qualifier sub-module register
 * @param size - The size of the Action-Qualifier sub-module register memory area
 *
 * @returns - 0 on success, -1 otherwise.
 */
int ehrpwm_config_aq_module(ulong addr, uint size);

/*
 * Set the width of the pulse for the specified GPE channel.
 *
 * @param channel - The GPE channel to set the width of.
 * @param ticks - The number of ticks to set the pulse width to.  The range is
 * 0-196.  Where 0 is 1ms and 196 is 2ms.
 *
 * @returns - None
 */
void set_pwm_width(uint channel, ushort ticks);

/*
 * Return the current PWM width in percents of the maximum width.
 *
 * @param channel - specify the output channel of interest.
 *
 * @return PWM width withing 0 to 100 range
 */
nanosecs_rel_t get_pwm_width(uint channel);

/*
 * Release acquired PWM GPIO lines and stops real-time threads started.
 *
 * @returns N\A
 */
void cleanup_pwm(void);

/*
 * Stop the pulse-width modulation sub-system
 *
 * @param addr - The start address of the PWMSS
 * @param size - The size of the PWMSS memory region
 *
 * @return - None
 */
void shutdown_pwmss(ulong addr, uint size);

#endif
