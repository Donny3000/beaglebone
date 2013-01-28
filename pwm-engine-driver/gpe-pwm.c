/*
 * gpe-pwm.c
 *
 *  Created on: Dec 24, 2012
 *      Author: Donald R. Poole, Jr.
 */
#include "pwm.h"
#include "gpio.h"
#include "gpe-pwm.h"

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Donald R. Poole, Jr. <donny3000@gmail.com>");
MODULE_DESCRIPTION("Beaglebone ePWM Generator LKM");

// Static pointers to the memory regions of the 3 ePWM sub-modules. This should
// should provide faster write access when setting the PWM duty-cycles.
static volatile void __iomem *pwmss_0 = NULL;
static volatile void __iomem *pwmss_1 = NULL;
static volatile void __iomem *pwmss_2 = NULL;

static ushort           num_of_gpe_chs;                             // The number of GPE channels to initialize
/*
 * The following three array definitions are
 * the mappings and associated data to configure
 * ePWM modules on the device.
 */
static uint             channel_to_gpio[6] = {
    GPIO_14,
    GPIO_15,
    GPIO_18,
    GPIO_19,
    GPIO_22,
    GPIO_23
};
static uint             gpio_config[6][2] = {
    {CONF_MCASP0_ACLKX, (SLEW_FAST | INPUT_DIS | PULLDOWN | PULLUPDOWN_EN | M1)},
    {CONF_MCASP0_FSX, (SLEW_FAST | INPUT_DIS | PULLDOWN | PULLUPDOWN_EN | M1)},
    {CONF_GPMC_A2, (SLEW_FAST | INPUT_DIS | PULLDOWN | PULLUPDOWN_EN | M6)},
    {CONF_GPMC_A3, (SLEW_FAST | INPUT_DIS | PULLDOWN | PULLUPDOWN_EN | M6)},
    {CONF_GPMC_AD8, (SLEW_FAST | INPUT_DIS | PULLDOWN | PULLUPDOWN_EN | M4)},
    {CONF_GPMC_AD9, (SLEW_FAST | INPUT_DIS | PULLDOWN | PULLUPDOWN_EN | M4)}
};
static struct           gpio gpe_output_chs[6] = {                  
    {GPIO_PIN_NUM(3, 14), GPIOF_OUT_INIT_LOW, "ePWM Channel 1"},     // EHRPWM0A
    {GPIO_PIN_NUM(3, 15), GPIOF_OUT_INIT_LOW, "ePWM Channel 2"},     // EHRPWM0B
    {GPIO_PIN_NUM(1, 18), GPIOF_OUT_INIT_LOW, "ePWM Channel 3"},    // EHRPWM2A
    {GPIO_PIN_NUM(1, 19), GPIOF_OUT_INIT_LOW, "ePWM Channel 4"},    // EHRPWM2B
    {GPIO_PIN_NUM(0, 22), GPIOF_OUT_INIT_LOW, "ePWM Channel 5"},    // EHRPWM1A
    {GPIO_PIN_NUM(0, 23), GPIOF_OUT_INIT_LOW, "ePWM Channel 6"},    // EHRPWM1B
};
static uint             gpio_requested[6];                          // Store the success/failure of requesting the gpio. If failed, don't try to free it.

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

int init_pwm(uchar num_gpe_ch)
{
	volatile void __iomem *mem;
	int retval, i;

    // Set the global number of channels
    num_of_gpe_chs = num_gpe_ch;
    
    /*
     * TODO: Need to see how to implement this
     * When the ePWM peripheral clock is enabled it may be possible that
     * interrupt flags may be set due to spurious events due to the ePWM
     * registers not being properly initialized. The proper procedure for
     * initializing the ePWM peripheral is:
     *
     * 1. Disable global interrupts (CPU INTM flag)
     * 2. Disable ePWM interrupts
     * 3. Initialize peripheral registers
     * 4. Clear any spurious ePWM flags
     * 5. Enable ePWM interrupts
     * 6. Enable global interrupts
     */

    /*
     * Begin the GPIO pin configuration
     */
    printk(KERN_ALERT "GPE-PWM: Configuring GPIO pin as follows: SLEW_FAST | INPUT_DIS | PULLUP | PULLUPDOWN_EN | MODE_7....\n");
    mem = ioremap(GPMC_CONF_ADDR_START, GPMC_CONF_ADDR_SIZE);
    if( !mem )
    {
        printk(KERN_ALERT "GPE-PWM: ERROR: Failed to remap memory for pin configuration.\n");
        return -1;
    }

    // Write the pin configuration to the 4 ePWM output channels
    for(i = 0; i < num_of_gpe_chs; i++)
    {
        iowrite8(gpio_config[i][1], mem + gpio_config[i][0]);
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
            printk(KERN_ALERT "GPE-PWM: Error: Failed to request ePWM module#%i GPIO pin#%i (error %i)\n", i%2, i, retval);
        }
        else
        {
            printk(KERN_ALERT "GPE-PWM: Requesting GPIO#%i from the system for %s\n", gpe_output_chs[i].gpio, gpe_output_chs[i].label);
            gpio_requested[i] = 1;
        }
    }
    
    printk(KERN_ALERT "GPE-PWM: Configuration of GPIO pins complete\n");
    
    /*
     * Begin setting the GPIOs to output
     */
    if(num_of_gpe_chs == 4)
    {
    	ehrpwm_0_config();
    	//ehrpwm_1_config();
    }

    if(num_of_gpe_chs == 6)
    {
        ehrpwm_2_config();
    }

    return 0;
}

int ehrpwm_0_config(void)
{
	volatile void __iomem *mem;
	volatile uint *addr;
	uint regVal;
	
	printk(KERN_ALERT "GPE-PWM: Memory mapping GPIO Bank 3 for ePWM channels 1 & 2\n");
    // Remap GPIO bank 3 address GPE channels 1 & 2
    mem = ioremap(GPIO3_START_ADDR, GPIO3_SIZE);
    if( !mem )
    {
        printk(KERN_ALERT "GPE-PWM: ERROR: GPIO memory mapping for ePWM channels 1 & 2 failed.\n");
        return -1;
    }
    addr = mem + GPIO_OE;
    // Reset the GPIO_OE register
    iowrite32(0xFFFFFFFF, addr);
    // Set the GPE channels 1 & 2 to output
    regVal = ioread32( addr );
    regVal &= ~(channel_to_gpio[0]);
    regVal &= ~(channel_to_gpio[1]);
    iowrite32(regVal, addr);
    iounmap( mem );
    
    // Enable the L4 peripheral clock for the sub-system first
    if(-1 == config_l4_clk(CM_PER_EPWMSS0_CLKCTRL))
    {
        printk(KERN_ALERT "GPE-PWM: ERROR: Failed to enable L4 Peripheral clock for PWMSS#0\n");
        return -1;
    }

    // Configure the pulse-width modulation sub-system
    if(-1 == pwmss_config(PWM_SUB_0_CONFIG_START, PWM_SUB_0_CONFIG_SIZE))
    {
        printk(KERN_ALERT "GPE-PWM: ERROR: Failed to configure PWMSS#0\n");
        return -1;
    }

    // Go ahead and save the memory-mapped PWMSS memory regions for
    // later when we need to change the duty-cyle of the PWM signal.
    pwmss_0 = ioremap(PWM_SUB_0_EPWM_START, PWM_SUB_0_EPWM_SIZE);
    if( !pwmss_0 )
    {
        printk(KERN_ALERT "GPE-PWM: ERROR: Failed to remap ePWMSS#0 register address.\n");
        return -1;
    }

    // First, we will setup the Time-Based sub-module.  This module sets up the
    // period/frequency of the PWM and the counter style (Up, Down, Up & Down).
    if(-1 == ehrpwm_config_tb_module(PWM_SUB_0_EPWM_START, PWM_SUB_0_EPWM_SIZE))
    {
    	printk(KERN_ALERT "GPE-PWM: ERROR: Failed to configure ePWMSS#0 Time-Based sub-module\n");
    	return -1;
    }
    
    // Now, setup the Counter-Compare sub-module.  This module sets the points
    // during the period counter where we want events (pin toggles) to occurr.
    // We only set on place initially, which is the fall-edge of the PWM signal.
    // Should be (1ms).
    if(-1 == ehrpwm_config_cc_module(PWM_SUB_0_EPWM_START, PWM_SUB_0_EPWM_SIZE))
    {
    	printk(KERN_ALERT "GPE-PWM: ERROR: Failed to configure ePWMSS#0 Counter-Compare sub-module\n");
    	return -1;
    }
    
    // Finally, we setup the Action-Qualifier sub-module.  This module
    // configures the actual events for the two output channels: Output high
    // when the counter reaches 0 and output low when the counter reaches the
    // tick count setup in the previous step.
    if(-1 == ehrpwm_config_aq_module(PWM_SUB_0_EPWM_START, PWM_SUB_0_EPWM_SIZE))
    {
    	printk(KERN_ALERT "GPE-PWM: ERROR: Failed to configure ePWMSS#0 Action-Qualifier sub-module\n");
    	return -1;
    }
    
    return 0;
}

int ehrpwm_1_config(void)
{
	volatile void __iomem *mem;
	volatile uint *addr;
	uint regVal;
	
	printk(KERN_ALERT "GPE-PWM: Memory mapping GPIO Bank 1 for ePWM channels 2 & 4\n");
    // Remap GPIO bank 1 address GPE channels 1 & 2
    mem = ioremap(GPIO1_START_ADDR, GPIO1_SIZE);
    if( !mem )
    {
        printk(KERN_ALERT "GPE-PWM: ERROR: GPIO memory mapping for ePWM channels 2 & 4 failed.\n");
        return -1;
    }
    addr = mem + GPIO_OE;
    // Reset the GPIO_OE register
    iowrite32(0xFFFFFFFF, addr);
    // Set the GPE channels 2 & 4 to output
    regVal = ioread32( addr );
    regVal &= ~(channel_to_gpio[2]);
    regVal &= ~(channel_to_gpio[3]);
    iowrite32(regVal, addr);
    iounmap( mem );
    
    // Enable the L4 peripheral clock for the sub-system first
    if(-1 == config_l4_clk(CM_PER_EPWMSS1_CLKCTRL))
    {
        printk(KERN_ALERT "GPE-PWM: ERROR: Failed to enable L4 Peripheral clock for PWMSS#1\n");
        return -1;
    }

    // Configure the pulse-width modulation sub-system
    if(-1 == pwmss_config(PWM_SUB_1_CONFIG_START, PWM_SUB_1_CONFIG_SIZE))
    {
        printk(KERN_ALERT "GPE-PWM: ERROR: Failed to configure PWMSS#1\n");
        return -1;
    }

    // Go ahead and save the memory-mapped PWMSS memory regions for
    // later when we need to change the duty-cyle of the PWM signal.
    pwmss_1 = ioremap(PWM_SUB_1_EPWM_START, PWM_SUB_1_EPWM_SIZE);
    if( !pwmss_1 )
    {
        printk(KERN_ALERT "GPE-PWM: ERROR: Failed to remap ePWMSS#1 register address.\n");
        return -1;
    }

    // First, we will setup the Time-Based sub-module.  This module sets up the
    // period/frequency of the PWM and the counter style (Up, Down, Up & Down).
    if(-1 == ehrpwm_config_tb_module(PWM_SUB_1_EPWM_START, PWM_SUB_1_EPWM_SIZE))
    {
    	printk(KERN_ALERT "GPE-PWM: ERROR: Failed to configure ePWMSS#1 Time-Based sub-module\n");
    	return -1;
    }
    
    // Now, setup the Counter-Compare sub-module.  This module sets the points
    // during the period counter where we want events (pin toggles) to occurr.
    // We only set on place initially, which is the fall-edge of the PWM signal.
    // Should be (1ms).
    if(-1 == ehrpwm_config_cc_module(PWM_SUB_1_EPWM_START, PWM_SUB_1_EPWM_SIZE))
    {
    	printk(KERN_ALERT "GPE-PWM: ERROR: Failed to configure ePWMSS#1 Counter-Compare sub-module\n");
    	return -1;
    }
    
    // Finally, we setup the Action-Qualifier sub-module.  This module
    // configures the actual events for the two output channels: Output high
    // when the counter reaches 0 and output low when the counter reaches the
    // tick count setup in the previous step.
    if(-1 == ehrpwm_config_aq_module(PWM_SUB_1_EPWM_START, PWM_SUB_1_EPWM_SIZE))
    {
    	printk(KERN_ALERT "GPE-PWM: ERROR: Failed to configure ePWMSS#1 Action-Qualifier sub-module\n");
    	return -1;
    }

    return 0;
}

int ehrpwm_2_config(void)
{
	volatile void __iomem *mem;
	volatile uint *addr;
	uint regVal;
	
	printk(KERN_ALERT "GPE-PWM: Memory mapping GPIO Bank 1 for ePWM channels 5 & 6\n");
	// Remap GPIO bank 0 address for the GPE channels
    mem = ioremap(GPIO0_START_ADDR, GPIO0_SIZE);
    if( !mem )
    {
        printk(KERN_ALERT "GPE-PWM: ERROR: GPIO memory mapping for ePWM channels 5 & 6 failed.\n");
        return -1;
    }
    addr = mem + GPIO_OE;

    // Reset the GPIO_OE register
    iowrite32(0xFFFFFFFF, addr);
    // Set the GPE channels to output
    regVal = ioread32( addr );
    regVal &= ~(channel_to_gpio[4]);
    regVal &= ~(channel_to_gpio[5]);
    iowrite32(regVal, addr);
    iounmap( mem );

    printk(KERN_ALERT "GPE-PWM: Memory-mapping PMW Bank complete.\n");
    
    // Enable the L4 peripheral clock for the sub-system first
    if(-1 == config_l4_clk(CM_PER_EPWMSS2_CLKCTRL))
    {
        printk(KERN_ALERT "GPE-PWM: ERROR: Failed to enable L4 Peripheral clock for PWMSS#2\n");
        return -1;
    }

    // Configure the pulse-width modulation sub-system
    if(-1 == pwmss_config(PWM_SUB_2_CONFIG_START, PWM_SUB_2_CONFIG_SIZE))
    {
        printk(KERN_ALERT "GPE-PWM: ERROR: Failed to configure PWMSS#2\n");
        return -1;
    }

    // Go ahead and save the memory-mapped PWMSS memory regions for
    // later when we need to change the duty-cyle of the PWM signal.
    pwmss_2 = ioremap(PWM_SUB_2_EPWM_START, PWM_SUB_2_EPWM_SIZE);
    if( !pwmss_2 )
    {
        printk(KERN_ALERT "GPE-PWM: ERROR: Failed to remap ePWMSS#2 register address.\n");
        return -1;
    }

    // First, we will setup the Time-Based sub-module.  This module sets up the
    // period/frequency of the PWM and the counter style (Up, Down, Up & Down).
    if(-1 == ehrpwm_config_tb_module(PWM_SUB_2_EPWM_START, PWM_SUB_2_EPWM_SIZE))
    {
    	printk(KERN_ALERT "GPE-PWM: ERROR: Failed to configure ePWMSS#2 Time-Based sub-module\n");
    	return -1;
    }
    
    // Now, setup the Counter-Compare sub-module.  This module sets the points
    // during the period counter where we want events (pin toggles) to occurr.
    // We only set on place initially, which is the fall-edge of the PWM signal.
    // Should be (1ms).
    if(-1 == ehrpwm_config_cc_module(PWM_SUB_2_EPWM_START, PWM_SUB_2_EPWM_SIZE))
    {
    	printk(KERN_ALERT "GPE-PWM: ERROR: Failed to configure ePWMSS#2 Counter-Compare sub-module\n");
    	return -1;
    }
    
    // Finally, we setup the Action-Qualifier sub-module.  This module
    // configures the actual events for the two output channels: Output high
    // when the counter reaches 0 and output low when the counter reaches the
    // tick count setup in the previous step.
    if(-1 == ehrpwm_config_aq_module(PWM_SUB_2_EPWM_START, PWM_SUB_2_EPWM_SIZE))
    {
    	printk(KERN_ALERT "GPE-PWM: ERROR: Failed to configure ePWMSS#2 Action-Qualifier sub-module\n");
    	return -1;
    }

    return 0;
}

int config_l4_clk(ulong epwmss_clk_mod)
{
    volatile void __iomem *mem;

    mem = ioremap(CM_PER_REG_START, CM_PER_REG_SIZE);
    if( !mem )
    {
        printk(KERN_ALERT "GPE-PWM: ERROR: Clock Module memory mapping failed\n");
        return -1;
    }
    // Enable the module and disable idle mode.
    iowrite32(0x2, mem + epwmss_clk_mod);
    iounmap( mem );

    return 0;
}

int pwmss_config(ulong addr, uint size)
{
    volatile void __iomem *mem;

    // Configure the ePWMSS
    mem = ioremap(addr, size);
    if( !mem )
    {
        printk(KERN_ALERT "GPE-PWM: ERROR: ePWMSS Configuration register memory mapping failed\n");
        return -1;
    }
    /*
     * Confiugred as follows:
     * No software reset => 0x0
     * Not Sensitive to Emulation => 0x1
     * Smart-Idle => 0x2
     * Smart-Standby => 0x2
     */
    iowrite8(0x2A, mem + PWMSS_CONFIG_SYSCONFIG);
    // Now, turn on the clock to the ePWMSSx  module.  We will turn off the
    // clocks to the eQEP and eCAP modules and turn on the clock to the
    // ePWM module.
    iowrite32(0x122, mem + PWMSS_CONFIG_CLKCONFIG);
    // Close the memory region
    iounmap( mem );

    return 0;
}

int ehrpwm_config_tb_module(ulong addr, uint size)
{
	volatile void __iomem *mem;

	// Configure the ePWM0 Time-base module
    mem = ioremap(addr, size);
    if( !mem )
    {
        printk(KERN_ALERT "GPE-PWM: Error: ePWM time-based memory mapping failed\n");
        return -1;
    }
    // Configure the Time-base Control Register and set the PWM clock to
    // 195,312.5Hz scaled down from the 100MHz system clock.
    iowrite16(0x1D30, mem + EPWM_TBCTL);
    // Set the period of the time-based counter. With the clock set to 195.3kHz
    // and the period of an RC servo pulse width (frame) at 20ms, the number of
    // clock ticks should be about 3906-3907 clk ticks.
    iowrite16(PWM_PRD_TICK_COUNT, mem + EPWM_TBPRD);
    // Close the mapped memory region
    iounmap( mem );
    
    return 0;
}

int ehrpwm_config_cc_module(ulong addr, uint size)
{
	volatile void __iomem *mem;

	// Configure the ePWM0 counter-compare module to establish the duty-cycle
    mem = ioremap(addr, size);
	if( !mem )
	{
		printk(KERN_ALERT "GPE-PWM: ERROR: ePWM counter-compare memory mapping failed\n");
		return -1;
	}
	iowrite8(0x5A, mem + EPWM_CMPCTL);
	// Set the falling-edge of the EPWMxA signal.  The initial pulse-width is 1ms
	// which equals 195-196 ticks, so that's what we will set it to.
	iowrite16(PWM_MIN_TICK_COUNT, mem + EPWM_CMPA);
    // Set the falling-edge of the EPWMxB signal.
    iowrite16(PWM_MIN_TICK_COUNT, mem + EPWM_CMPB);
	// Close the memory region
	iounmap( mem );
	
	return 0;
}

int ehrpwm_config_aq_module(ulong addr, uint size)
{
	volatile void __iomem *mem;

	// Configure the ePWM0 action-qualifier module to toggle the ePWM lines
    mem = ioremap(addr, size);
	if( !mem )
	{
		printk(KERN_ALERT "GPE-PWM: ERROR: ePWM action-qualifier memory mapping failed\n");
		return -1;
	}
	/*
	 * Toogle Action for EPWMxA channel
	 *
	 * Count == Zero => Output High 0x2 (Bits 0-1)
	 * Count == Period => Do Nothing 0x0 (Bits 2-3)
	 * Count == CMPA && Incrementing => Output Low 0x1 (Bits 4-5)
	 * Count == CMPA && Decrementing => Do Nothing 0x0 (Bits 6-7)
	 * Count == CMPB && Incrementing => Do Nothing 0x0 (Bits 8-9)
	 * Count == CMPB && Decrementing => Do Nothing 0x0 (Bits 10-11)
	 */
	iowrite16(0x12, mem + EPWM_AQCTLA);
	
	/*
	 * Toogle Action for EPWMxB channel
	 *
	 * Count == Zero => Output High 0x2 (Bits 0-1)
	 * Count == Period => Do Nothing 0x0 (Bits 2-3)
	 * Count == CMPA && Incrementing => Do Nothing 0x0 (Bits 4-5)
	 * Count == CMPA && Decrementing => Do Nothing 0x0 (Bits 6-7)
	 * Count == CMPB && Incrementing => Output Low 0x1 (Bits 8-9)
	 * Count == CMPB && Decrementing => Do Nothing 0x0 (Bits 10-11)
	 */
	iowrite16(0x102, mem + EPWM_AQCTLB);
	// Close the memory region
	iounmap( mem );
	
	return 0;
}

void set_pwm_width(uint channel, ushort ticks)
{
    if((channel < 0 || channel > num_of_gpe_chs) || ticks > PWM_MAX_TICK_COUNT)
    {
        printk(KERN_ALERT "GPE-PWM: ERROR: Invalid channel (%i) or tick count (%i) received\n", channel, ticks);
        return;
    }

    switch( channel )
    {
        case 0:
	        // Set the falling-edge of the EPWMxA/B signal.  The initial pulse-width is 1ms
	        // which equals 195-196 ticks, so that's what we will set it to.
	        iowrite16(PWM_MIN_TICK_COUNT + ticks, pwmss_0 + EPWM_CMPA);
            break;

        case 1:
	        // Set the falling-edge of the EPWMxA/B signal.  The initial pulse-width is 1ms
	        // which equals 195-196 ticks, so that's what we will set it to.
	        iowrite16(PWM_MIN_TICK_COUNT + ticks, pwmss_0 + EPWM_CMPB);
            break;

        case 2:
	        // Set the falling-edge of the EPWMxA/B signal.  The initial pulse-width is 1ms
	        // which equals 195-196 ticks, so that's what we will set it to.
	        iowrite16(PWM_MIN_TICK_COUNT + ticks, pwmss_1 + EPWM_CMPA);
            break;

        case 3:
	        // Set the falling-edge of the EPWMxA/B signal.  The initial pulse-width is 1ms
	        // which equals 195-196 ticks, so that's what we will set it to.
	        iowrite16(PWM_MIN_TICK_COUNT + ticks, pwmss_1 + EPWM_CMPB);
            break;

        case 4:
	        // Set the falling-edge of the EPWMxA/B signal.  The initial pulse-width is 1ms
	        // which equals 195-196 ticks, so that's what we will set it to.
	        iowrite16(PWM_MIN_TICK_COUNT + ticks, pwmss_2 + EPWM_CMPA);
            break;

        case 5:
	        // Set the falling-edge of the EPWMxA/B signal.  The initial pulse-width is 1ms
	        // which equals 195-196 ticks, so that's what we will set it to.
	        iowrite16(PWM_MIN_TICK_COUNT + ticks, pwmss_2 + EPWM_CMPB);
            break;
    }
}

nanosecs_rel_t get_pwm_width(uint channel)
{
    nanosecs_rel_t pw = 0;

    if(channel < 0 || channel > num_of_gpe_chs)
    {
        printk(KERN_ALERT "GPE-PWM: ERROR: Invalid channel (%i) received\n", channel);
        return 0;
    }

    switch( channel )
    {
        case 0:
	        // Get the falling-edge of the EPWMxA/B signal.  The initial pulse-width is 1ms
	        pw = ioread16(pwmss_0 + EPWM_CMPA) * PWM_NS_PER_STEP;
            break;

        case 1:
	        // Get the falling-edge of the EPWMxA/B signal.  The initial pulse-width is 1ms
	        pw = ioread16(pwmss_0 + EPWM_CMPB) * PWM_NS_PER_STEP;
            break;

        case 2:
	        // Get the falling-edge of the EPWMxA/B signal.  The initial pulse-width is 1ms
	        pw = ioread16(pwmss_1 + EPWM_CMPA) * PWM_NS_PER_STEP;
            break;

        case 3:
	        // Get the falling-edge of the EPWMxA/B signal.  The initial pulse-width is 1ms
	        pw = ioread16(pwmss_1 + EPWM_CMPB) * PWM_NS_PER_STEP;
            break;

        case 4:
	        // Get the falling-edge of the EPWMxA/B signal.  The initial pulse-width is 1ms
	        pw = ioread16(pwmss_2 + EPWM_CMPA) * PWM_NS_PER_STEP;
            break;

        case 5:
	        // Get the falling-edge of the EPWMxA/B signal.  The initial pulse-width is 1ms
	        pw = ioread16(pwmss_2 + EPWM_CMPB) * PWM_NS_PER_STEP;
            break;
    }

    return pw;
}

void cleanup_pwm()
{
	int i;

    // Release the Output GPIOs
    for(i = 0; i < num_of_gpe_chs; i++)
    {
        if( gpio_requested[i] )
        {
            printk(KERN_ALERT "GPE-PWM: Shutdown GPE channel %i\n", i);
            gpio_free( gpe_output_chs[i].gpio );
        }
    }

    // Shutdown the PWMSSs
    shutdown_pwmss(PWM_SUB_0_EPWM_START, PWM_SUB_0_EPWM_SIZE);
    shutdown_pwmss(PWM_SUB_1_EPWM_START, PWM_SUB_1_EPWM_SIZE);
    shutdown_pwmss(PWM_SUB_2_EPWM_START, PWM_SUB_2_EPWM_SIZE);

    // Release the Counter-Compare sub-module memory
    if( pwmss_0 )
    {
        iounmap( pwmss_0 );
    }

    if( pwmss_1 )
    {
        iounmap( pwmss_1 );
    }

    if( pwmss_2 )
    {
        iounmap( pwmss_2 );
    }
}

void shutdown_pwmss(ulong addr, uint size)
{

    volatile void __iomem *mem;

    // Configure the ePWMSS
    mem = ioremap(addr, size);
    if( !mem )
    {
        printk(KERN_ALERT "GPE-PWM: ERROR: ePWMSS Configuration register memory mapping failed\n");
    }
    // Now, turn off the clock to the ePWMSS 0 module.  We will turn off the
    // clocks to the eQEP and eCAP modules as well.
    iowrite32(0x222, mem + PWMSS_CONFIG_CLKCONFIG);
    // Close the memory region
    iounmap( mem );
}
