/*
 * gpe-pwm.c
 *
 *  Created on: Dec 24, 2012
 *      Author: Donald R. Poole, Jr.
 */
#include "gpio.h"
#include "gpe-pwm.h"

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Donald R. Poole, Jr. <donny3000@gmail.com>");
MODULE_DESCRIPTION("Generates PWMs with GPIO");

/*
 * Begin private variables
 */
static void __iomem     *pwm_bank = NULL;                           // Base address of the GPIO bank
static volatile uint	*gpio_oe_reg = NULL;                        // GPIO register to enable a pin of input/ouput
static volatile uint	*gpio_setdataout_reg = NULL;                // GPIO register to bring a pin high
static volatile uint	*gpio_cleardataout_reg = NULL;              // GPIO register to bring a pin low
static u8               num_of_gpe_chs;                             // The number of GPE channels to initialize

static struct           gpio gpe_output_chs[8] = {                  // GPE Output channel description structure
    {GPIO_PIN_NUM(1, 0), GPIOF_OUT_INIT_LOW, "GPE_Output_1"},
    {GPIO_PIN_NUM(1, 1), GPIOF_OUT_INIT_LOW, "GPE_Output_2"},
    {GPIO_PIN_NUM(1, 2), GPIOF_OUT_INIT_LOW, "GPE_Output_3"},
    {GPIO_PIN_NUM(1, 3), GPIOF_OUT_INIT_LOW, "GPE_Output_4"},
    {GPIO_PIN_NUM(1, 4), GPIOF_OUT_INIT_LOW, "GPE_Output_5"},
    {GPIO_PIN_NUM(1, 5), GPIOF_OUT_INIT_LOW, "GPE_Output_6"},
    {GPIO_PIN_NUM(1, 6), GPIOF_OUT_INIT_LOW, "GPE_Output_7"},
    {GPIO_PIN_NUM(1, 7), GPIOF_OUT_INIT_LOW, "GPE_Output_8"}
};
static uint             gpio_requested[8];                          // Store the success/failure of requesting the gpio. If failed, don't try to free it.

static rtdm_timer_t     up_timers[8];                               // The timer descriptor for up (high) timer
static rtdm_timer_t     down_timers[8];                             // The timer descriptors for the down (low) timers. Up to 8
static nanosecs_rel_t   up_periods[8];                              // The timer durations for the up (high) periods
static u8               configured[8];                              // Boolean container to hold confiugration state of GPE channel

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

void set_pwm_width(int channel, int percentage)
{
    up_periods[channel] = CALC_PULSE_WIDTH(channel_ranges[channel][0], channel_ranges[channel][1], percentage);
    configured[channel] = 0;
}

nanosecs_rel_t get_pwm_width(int channel)
{
    return up_periods[channel];
}

int init_pwm(gpe_ch_desc_t *channels, uchar num_gpe_ch)
{
    void __iomem *mem;
	int retval, i;
    uint regVal;

    // Set the global number of channels
    num_of_gpe_chs = num_gpe_ch;

    for(i = 0; i < num_gpe_ch; i++)
    {
        channel_ranges[ channels[i].channel ][0] = channels[i].pwmMinWidth;
        channel_ranges[ channels[i].channel ][1] = channels[i].pwmMaxWidth;
        up_periods[ channels[i].channel ] = CALC_PULSE_WIDTH(channel_ranges[i][0], channel_ranges[i][1], 50);
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
    // Write the pin configuration to the 8 output channels ()
    for(i=CONF_GPMC_AD0; i<=CONF_GPMC_AD7; i+=4)
    {
        iowrite8((SLEW_FAST | INPUT_DIS | PULLUP | PULLUPDOWN_DIS | M7), mem + i);
    }
    // close the pin conf address space
    iounmap( mem );

    // Request to GPIO pins from the system for use.
    for(i=0; i < num_of_gpe_chs; i++)
    {
        retval = gpio_request_one(gpe_output_chs[i].gpio, gpe_output_chs[i].flags, gpe_output_chs[i].label);
        if( 0 != retval )
        {
            gpio_requested[i] = 0;
            rtdm_printk("GPE: Error: Failed to request GPIO pin#%i (error %i)\n", i, retval);
        }
        else
        {
            gpio_requested[i] = 1;
        }
    }
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

	for(i = 0; i < num_of_gpe_chs; i++)
    {
        rtdm_timer_destroy( &up_timers[i] );
		rtdm_timer_destroy( &down_timers[i] );
        rtdm_printk("GPE: Shutdown GPE channel %i\n", i);
    }

    // Release the Output GPIOs
    for(i = 0; i < num_of_gpe_chs; i++)
    {
        if( gpio_requested[i] )
            gpio_free( gpe_output_chs[i].gpio );
    }

    iounmap( pwm_bank );
}
