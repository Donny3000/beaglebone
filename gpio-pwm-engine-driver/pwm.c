/*
 * pwm.c
 *
 *  Created on: Dec 24, 2012
 *      Author: Donald R. Poole, Jr.
 */
#include <linux/module.h>
#include <rtdm/rtdm_driver.h>
#include "gpio.h"
#include "pwm.h"

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Donald R. Poole, Jr. <donny3000@gmail.com>");
MODULE_DESCRIPTION("Generates PWMs with GPIO");

#define RANGE_MAP100(c, d, x) (c + div100((d - c) * x))
#define	SWITCHING_FREQ	20000000            // 1 ms period
#define PULSE_WIDTH		(SWITCHING_FREQ / 2)  // For testing, create a 50% duty cycle

/*
 * Begin private variables
 */
static void __iomem     *pwm_bank = NULL;               // GPIO bank register address
static volatile uint	*gpio_oe_reg = NULL;            // GPIO register to enable a pin of input/ouput
static volatile uint	*gpio_setdataout_reg = NULL;    // GPIO register to bring a pin high
static volatile uint	*gpio_cleardataout_reg = NULL;  // GPIO register to bring a pin low
static uchar            num_of_gpe_chs;                 // The number of GPE channels to initialize
static rtdm_timer_t     up_timer;                       // The timer descriptor for up (high) timer
static rtdm_timer_t     down_timers[8];                 // The timer descriptors for the down (low) timers. Up to 8
static nanosecs_rel_t   up_periods[8];                  // The timer durations for the up (high) periods
static uchar            configured[8];                  // Boolean container to hold confiugration state of GPE channel

// Initialized with default pulse ranges
static int channel_ranges[8][2] = {
    {950, 2050},
    {950, 2050},
    {950, 2050},
    {950, 2050},
    {950, 2050},
    {950, 2050},
    {950, 2050},
    {950, 2050}
};

int div100(long long dividend)
{
    long long divisor = 0x28f5c29;
    return ((divisor * dividend) >> 32) & 0xffffffff;
}

void pulse_high(rtdm_timer_t *timer)
{
    int retval;
    uchar channel = 0;

    // Raise the line high
    iowrite32(GPIO_6, gpio_setdataout_reg);
    if( !configured[channel] )
    {
        configured[channel] = 1;
        rtdm_timer_stop( &down_timers[channel] );
        retval = rtdm_timer_start(&down_timers[channel], up_periods[channel], SWITCHING_FREQ, RTDM_TIMERMODE_RELATIVE);
        if( retval )
        {
            rtdm_printk("GPE: Error occurred reconfiguring 'Off Time' timer#%i (%i)\n.", channel, retval);
        }
    }
}

void pulse_low(rtdm_timer_t *timer)
{
    iowrite32(GPIO_6, gpio_cleardataout_reg);
}

void setpwmwidth(int channel, int percentage)
{
    up_periods[channel] = 1000 * RANGE_MAP100(channel_ranges[channel][0], channel_ranges[channel][1], percentage);
    configured[channel] = 0;
}

nanosecs_rel_t getpwmwidth(int channel)
{
    return up_periods[channel];
}

int init_pwm(gpe_ch_desc_t *channels, uchar num_gpe_ch)
{
	int retval, i;
    uint regVal;

    num_of_gpe_chs = num_gpe_ch;
    for(i = 0; i < num_gpe_ch; i++)
    {
        channel_ranges[ channels[i].channel ][0] = channels[i].pwmMinWidth;
        channel_ranges[ channels[i].channel ][1] = channels[i].pwmMaxWidth;
        up_periods[ channels[i].channel ] = 1000 * RANGE_MAP100(channel_ranges[i][0], channel_ranges[i][1], 50);
        configured[i] = 1;
    }
    rtdm_printk("GPE: Initialized %i PWM pulse lengths.\n", i);

	rtdm_printk("GPE: Memory mapping GPIO Bank 1 for PWMs...\n");

	// Remap GPIO bank 1 address for the GPE channels
    pwm_bank = ioremap(GPIO1_START_ADDR, GPIO1_SIZE);
	if( !pwm_bank )
    {
		rtdm_printk("GPE: ERROR: GPIO memory mapping failed.\n");
		return 0;
    }

	// Set GPIO register address
	gpio_oe_reg = pwm_bank + GPIO_OE;

	// Reset the GPIO_OE register
    iowrite32(0xFFFFFFFF, gpio_oe_reg);
    // Enable the output of the GPE channels
    regVal = ioread32( gpio_oe_reg );
    regVal &= ~GPIO_6;
    iowrite32(regVal, gpio_oe_reg);

	// Set the SETDATAOUT register address
	gpio_setdataout_reg = pwm_bank + GPIO_SETDATAOUT;

	// Set the CLEARDATAOUT register address
	gpio_cleardataout_reg = pwm_bank + GPIO_CLEARDATAOUT;

	rtdm_printk("GPE: Memory-mapping PMW Bank complete.\n");

    retval = rtdm_timer_init(&up_timer, &pulse_high, "up_timer");
    if( retval )
    {
        rtdm_printk("GPE: ERROR: Failed to initialize the GPE 'On Time' timer (%i).\n", retval);
        return retval;
    }
    rtdm_printk("GPE: Initialized 'On Time' timer!\n");
 
	for(i = 0; i < num_of_gpe_chs; i++)
	{
		retval = rtdm_timer_init(&down_timers[i], &pulse_low, "down_timer");
        if( retval )
        {
            rtdm_printk("GPE: ERROR: Received error %i while initializing 'Off Timer' timer#%i\n", retval, i);
            return retval;
        }
        rtdm_printk("GPE: Successfully initialized 'Off Time' timer#%i.\n", i);
	}

    retval = rtdm_timer_start(&up_timer, SWITCHING_FREQ, SWITCHING_FREQ, RTDM_TIMERMODE_RELATIVE);
    if( retval )
    {
        rtdm_printk("GPE: Error occurred starting starting 'On Time' timer (%i)\n", retval);
        return retval;
    }
    rtdm_printk("GPE: Successfully initialized and started all GPE PWM channel timers.\n");
	
    return 0;
}

void cleanup_pwm()
{
	int i;

    rtdm_timer_destroy( &up_timer );
	for(i=0; i < num_of_gpe_chs; i++)
    {
		rtdm_timer_destroy( &down_timers[i] );
        rtdm_printk("GPE: Shutdown GPE channel %i\n", i);
    }

    iounmap( pwm_bank );
}
