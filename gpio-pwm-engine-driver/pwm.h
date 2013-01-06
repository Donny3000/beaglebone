#ifndef __PWM_TASK_PROC_H
#define __PWM_TASK_PROC_H

#define SWITCHING_FREQ  1000000

typedef unsigned char uchar;
typedef unsigned int  uint;
typedef unsigned long ulong;

typedef struct gpe_ch_desc
{
    uint channel;
    uint pwmMinWidth;
    uint pwmMaxWidth;
} gpe_ch_desc_t;

/*
 * Efficient divide routine using
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
 * maximal allowed width. Should be in range between 0 and 100. The
 * actual width which will be set is implementation specific and may
 * vary depending on devices to be controlled. Current implementation
 * will set the width in range between 600 and 2000 usec which is the
 * typical range for model servos such as for example Futaba servos.
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

/**
 * Release acquired resources and stops real-time threads started for
 * PWM generation.
 */
void cleanup_pwm(void);

#endif
