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

/*
 * Begin private variables
 */
static void __iomem     *pwm_bank = NULL;               // Base address of the GPIO bank
static volatile uint	*gpio_oe_reg = NULL;            // GPIO register to enable a pin of input/ouput
static volatile uint	*gpio_setdataout_reg = NULL;    // GPIO register to bring a pin high
static volatile uint	*gpio_cleardataout_reg = NULL;  // GPIO register to bring a pin low
static uchar            num_of_gpe_chs;                 // The number of GPE channels to initialize
static rtdm_timer_t     up_timers[8];                   // The timer descriptor for up (high) timer
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

static uint channel_to_gpio[8] = {
    GPIO_6,
    GPIO_7,
    GPIO_2,
    GPIO_3,
    GPIO_5,
    GPIO_4,
    GPIO_1,
    GPIO_0
};

uint64_t div(uint64_t numerator, uint64_t denominator)
{
    uint64_t q = 0;
    uint64_t r = 0;
    int bits = sizeof( uint64_t ) * 8 - 1;

    for(; bits >= 0; bits--)
    {
        r = r << 1;

        // Set the lsb of r equal to bit i of the numerator
        if(numerator & (1 << bits))
            r |= 1;
        else
            r &= ~1;

        if(r >= denominator)
        {
            r = r - denominator;
            q |= (1 << bits);
        }
    }

    return q;
}

int div100(long long dividend)
{
    long long divisor = 0x28f5c29;
    return ((divisor * dividend) >> 32) & 0xffffffff;
}

void pulse_high(rtdm_timer_t *timer)
{
    int retval;
    uchar channel;

    // Find the channel
    for(channel = 0; channel < num_of_gpe_chs; channel++)
    {
        if(timer == &up_timers[channel])
            break;
    }
    iowrite32(channel_to_gpio[channel], gpio_setdataout_reg);

    // Check to see if the channel needs to change duty cycle
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
    uchar channel;

    // Find the channel
    for(channel = 0; channel < num_of_gpe_chs; channel++)
    {
        if(timer == &down_timers[channel])
            break;
    }

    iowrite32(channel_to_gpio[channel], gpio_cleardataout_reg);
}

void setpwmwidth(int channel, int percentage)
{
    up_periods[channel] = CALC_PULSE_WIDTH(channel_ranges[channel][0], channel_ranges[channel][1], percentage);
    configured[channel] = 0;
}

nanosecs_rel_t getpwmwidth(int channel)
{
    return up_periods[channel];
}

int init_pwm(gpe_ch_desc_t *channels, uchar num_gpe_ch)
{
    void __iomem *mem;
	int retval, i;
    uint regVal;
    uchar pinconf;

    // Set the global number of channels
    num_of_gpe_chs = num_gpe_ch;

    for(i = 0; i < num_gpe_ch; i++)
    {
        channel_ranges[ channels[i].channel ][0] = channels[i].pwmMinWidth;
        channel_ranges[ channels[i].channel ][1] = channels[i].pwmMaxWidth;
        up_periods[ channels[i].channel ] = CALC_PULSE_WIDTH(channel_ranges[i][0], channel_ranges[i][1], 50);
rtdm_printk("GPE: Min Width: %i\n", channels[i].pwmMinWidth);
rtdm_printk("GPE: Max Width: %i\n", channels[i].pwmMaxWidth);
rtdm_printk("GPE: Channel %i up period: %llu\n", channels[i].channel, up_periods[ channels[i].channel ]);
        configured[i] = 1;
    }
    rtdm_printk("GPE: Initialized %i PWM pulse lengths.\n", i);

    // Configure the pin for GPIO output
    rtdm_printk("GPE: Configuring GPIO pin as follows: SLEW_FAST | INPUT_DIS | PULLUP | PULLUPDOWN_EN | MODE_7....\n");
    mem = ioremap(GPMC_CONF_ADDR_START, GPMC_CONF_ADDR_SIZE);
    if( !mem )
    {
        rtdm_printk("GPE: ERROR: Failed to remap memory for pin configuration.\n");
        return 0;
    }

    // Configure GPIO1 pins 0-7 for output
    pinconf = (SLEW_FAST | INPUT_DIS | PULLUP | PULLUPDOWN_EN | M7);
    iowrite8(pinconf, mem + CONF_GPMC_AD0);
    iowrite8(pinconf, mem + CONF_GPMC_AD1);
    iowrite8(pinconf, mem + CONF_GPMC_AD2);
    iowrite8(pinconf, mem + CONF_GPMC_AD3);
    iowrite8(pinconf, mem + CONF_GPMC_AD4);
    iowrite8(pinconf, mem + CONF_GPMC_AD5);
    iowrite8(pinconf, mem + CONF_GPMC_AD6);
    iowrite8(pinconf, mem + CONF_GPMC_AD7);
    // close the pin conf address space
    iounmap( mem );
    rtdm_printk("GPE: Configuration of GPIO pins complete\n");

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
    
    // Set the GPE channels to output
    regVal = ioread32( gpio_oe_reg );
    for(i = 0; i < num_of_gpe_chs; i++)
    {
        regVal &= ~(channel_to_gpio[i]);
    }
    iowrite32(regVal, gpio_oe_reg);

	// Set the SETDATAOUT register address
	gpio_setdataout_reg = pwm_bank + GPIO_SETDATAOUT;

	// Set the CLEARDATAOUT register address
	gpio_cleardataout_reg = pwm_bank + GPIO_CLEARDATAOUT;

	rtdm_printk("GPE: Memory-mapping PMW Bank complete.\n");

    // Initialize the GPE channel timers
    for(i = 0; i < num_of_gpe_chs; i++)
    {
        retval = rtdm_timer_init(&up_timers[i], &pulse_high, "up_timers");
        if( retval )
        {
            rtdm_printk("GPE: ERROR: Received error %i while initializing 'On Timer' timer#%i\n", retval, i);
            return retval;
        }
        rtdm_printk("GPE: Successfully initialized 'On Timer' timer#%i\n", i);
 
		retval = rtdm_timer_init(&down_timers[i], &pulse_low, "down_timer");
        if( retval )
        {
            rtdm_printk("GPE: ERROR: Received error %i while initializing 'Off Timer' timer#%i\n", retval, i);
            return retval;
        }
        rtdm_printk("GPE: Successfully initialized 'Off Timer' timer#%i\n", i);
	}

    // Start the 'On Timers'
    for(i = 0; i < num_of_gpe_chs; i++)
    {
        retval = rtdm_timer_start(&up_timers[i], SWITCHING_FREQ, SWITCHING_FREQ, RTDM_TIMERMODE_RELATIVE);
        if( retval )
        {
            rtdm_printk("GPE: Error occurred starting starting 'On Time' timer (%i)\n", retval);
            return retval;
        }
    }
    rtdm_printk("GPE: Successfully initialized and started all GPE PWM channel timers.\n");
	
    return 0;
}

void cleanup_pwm()
{
	int i;

	for(i=0; i < num_of_gpe_chs; i++)
    {
        rtdm_timer_destroy( &up_timers[i] );
		rtdm_timer_destroy( &down_timers[i] );
        rtdm_printk("GPE: Shutdown GPE channel %i\n", i);
    }

    iounmap( pwm_bank );
}
