#ifndef __GPE_PWM_H
#define __GPE_PWM_H

#include <linux/module.h>
#include <linux/gpio.h>
#include <rtdm/rtdm_driver.h>
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
int init_pwm(gpe_ch_desc_t *channels, uchar nchannels);

/*
 * 'On Time' timer for PWM generation
 * *timer - The handle as returned by rtdm_timer_init().
 */
void pulse_high(rtdm_timer_t *timer);

/*
 * 'Off Time' timer for PWM gerneration
 * *timer - The handle as returned by rtdm_timer_init().
 */
void pulse_low(rtdm_timer_t *timer);

/*
 * Set the width of the pulse for the specified GPE channel.
 *
 * @param channel - The GPE channel to set the width of.
 *
 * @param percentage - The pulse width specified in percents of the
 * maximal allowed width. Should be in range between 0 and 100.
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

#endif
