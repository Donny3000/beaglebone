#ifndef __GPE_PWM_H
#define __GPE_PWM_H

#include <linux/module.h>
#include <linux/gpio.h>
#include <linux/clk.h>
#include <linux/pwm/pwm.h>
#include <linux/pwm/ehrpwm.h>
#include <linux/platform_device.h>
#include <rtdm/rtdm_driver.h>

#include <plat/clock.h>

#include "gpe-types.h"

#define SWITCHING_FREQ  1000000
#define CALC_PULSE_WIDTH(min, max, percentage) (min + div((max - min) * percentage, 100))

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
 * @param addr - The address of the L4 Peripheral clock configuration registers
 * @param size - The size of the address space being mapped.
 *
 * @return - 0 on success, -1 otherwise.
 */
int config_l4_clk(ulong addr, uint size);

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
 * @param percentage - The pulse width specified in percents of the
 * maximal allowed width. Should be in range between 0 and 100.
 *
 * @returns - None
 */
void set_pwm_width(int channel, int percentage);

/*
 * Return the current PWM width in percents of the maximum width.
 *
 * @param channel - specify the output channel of interest.
 *
 * @return PWM width withing 0 to 100 range
 */
nanosecs_rel_t get_pwm_width(int channel);

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
