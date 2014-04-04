/*
 * gpe-irq.c
 *
 * GPE IRQ functionality definitions
 *
 * Author: Donald R. Poole, Jr.
 */

#include "quad-drivers-gpio.h"
#include "gpe-irq.h"

static uchar         num_of_receiver_chs = 4; // The number of channels being read from the RC Rx
static irq_ch_desc_t irq_chs[4] = {
    {{GPIO_PIN_NUM(2, 8),  GPIOF_IN, "GPE_Throttle"}, irq_handler_throttle},
    {{GPIO_PIN_NUM(2, 9),  GPIOF_IN, "GPE_Roll"},     irq_handler_roll},
    {{GPIO_PIN_NUM(2, 10), GPIOF_IN, "GPE_Pitch"},    irq_handler_pitch},
    {{GPIO_PIN_NUM(2, 11), GPIOF_IN, "GPE_Yaw"},      irq_handler_yaw}
};
static uint       irq_gpio_requested[4]; // Store the success/failure of requesting the IRQ gpio pin.
static uint       irq_requested[4];      // Store the success/failure of requesting the GPE IRQ channels, in addition the assigned IRQ number.
static rtdm_irq_t receiver_irqs[4];      // IRQ descriptors for the PWMs coming from the receiver (Throttle, roll, pitch & yaw)

int init_gpe_irq()
{
    void __iomem *mem;
	int i, retval, irq;
    uint regval;

    // 1st Step: Request the GPIO lines for the IRQ channels
    for(i = 0; i < num_of_receiver_chs; i++)
    {
        retval = gpio_request_one(irq_chs[i].gpio_desc.gpio, irq_chs[i].gpio_desc.flags, irq_chs[i].gpio_desc.label);
        if( 0 != retval )
        {
            irq_gpio_requested[i] = 0;
            rtdm_printk("GPE-IRQ: Error: Failed to request GPIO pin#%i for IRQ (error %i)\n", i, retval);
            return retval;
        }

		irq_gpio_requested[i] = 1;
		irq = gpio_to_irq( irq_chs[i].gpio_desc.gpio );
		if( irq >= 0 )
		{
			irq_requested[i] = 1;
			retval = rtdm_irq_request(&receiver_irqs[i], irq, irq_chs[i].isr, RTDM_IRQTYPE_EDGE, irq_chs[i].gpio_desc.label, NULL);
			switch( retval )
			{
				case -EINVAL:
					rtdm_printk("GPE-IRQ: ERROR: rtdm_irq_request() received and invalid parameter.\n");
					break;
				case -EBUSY:
					rtdm_printk("GPE-IRQ: ERROR: The requested IRQ line#%i from GPIO#%i is already in use\n", irq, irq_chs[i].gpio_desc.gpio);
					break;
				case 0:
				default:
					rtdm_printk("GPE-IRQ: Initialized GPIO #%i to IRQ #%i\n", irq_chs[i].gpio_desc.gpio, irq);
					break;
			}
		}
		else
		{
			irq_requested[i] = 0;
			rtdm_printk("GPE-IRQ: ERROR: Failed to obtain IRQ number for GPIO #%i (error %i)\n", irq_chs[i].gpio_desc.gpio, retval);
			return retval;
		}
    }

    // 2nd Step: Configure the pin for GPIO input
    rtdm_printk("GPE-IRQ: Configuring IRQ GPIO pin as follows: SLEW_FAST | INPUT_EN | PULLUP | PULLUPDOWN_DIS | MODE_7....\n");
    mem = ioremap(GPMC_CONF_ADDR_START, GPMC_CONF_ADDR_SIZE);
    if( !mem )
    {
        rtdm_printk("GPE-IRQ: ERROR: Failed to remap memory for IRQ pin configuration.\n");
        return mem;
    }

    // Configure GPIO1 pins 0-7 for output
    // Write the pin configuration to the 8 output channels ()
    for(i=CONF_LCD_DATA2; i<=CONF_LCD_DATA5; i+=4)
    {
        iowrite8((SLEW_FAST | INPUT_EN | PULLUP | PULLUPDOWN_DIS | M7), mem + i);
    }
    // close the pin conf address space
    iounmap( mem );

    // GPIO Bank 2: Setup the IRQ registers with the appropriate values
    mem = ioremap(GPIO2_START_ADDR, GPIO2_SIZE);
    if( !mem )
    {
        rtdm_printk("GPE-IRQ: ERROR: Failed to remap memory for GPIO Bank 2 IRQ pin configuration.\n");
        return mem;
    }

    // Enable the IRQ ability for GPIO0_26 & GPIO0_27
    regval = ioread32(mem + GPIO_IRQSTATUS_0);
    regval |= (GPIO_8 | GPIO_9 | GPIO_10 | GPIO_11);
    iowrite32(regval, mem + GPIO_IRQSTATUS_0);
    regval = ioread32(mem + GPIO_IRQSTATUS_1);
    regval |= (GPIO_8 | GPIO_9 | GPIO_10 | GPIO_11);
    iowrite32(regval, mem + GPIO_IRQSTATUS_1);
    
    // Set Pin 26 & 27 for rising- and falling- edge detection
    regval = ioread32(mem + GPIO_RISINGDETECT);
    regval |= (GPIO_8 | GPIO_9 | GPIO_10 | GPIO_11);
    iowrite32(regval, mem + GPIO_RISINGDETECT);
    regval = ioread32(mem + GPIO_FALLINGDETECT);
    regval |= (GPIO_8 | GPIO_9 | GPIO_10 | GPIO_11);
    iowrite32(regval, mem + GPIO_FALLINGDETECT);
    // Release the mapped memory
    iounmap( mem );

    // 3rd Step: Setup the IRQ controller to read the H/W interrupts
    mem = ioremap(INTC_REG_START, INTC_REG_SIZE);
    if( !mem )
    {
        rtdm_printk("GPE-IRQ: ERROR: Failed to remap IRQ Control Registers memory\n");
        return 0;
    }

    // Disable clock gating strategy for the. Will use more
    // power with this strategy disabled, but will decrease
    // latency.
    iowrite32(0x0, mem + INTC_SYSCONFIG);

    // Disable the input synchronizer auto-gating and the
    // functional clock auto-idle modes.  This will use more
    // power, but will decrease latency.
    iowrite32(0x1, mem + INTC_IDLE);

    // For each IRQ line, set the highest priorit (0) and
    // route to the IRQ, not the FIQ (GPIOs cannot be routed
    // to FIQ)
    iowrite8(0x0, mem + INTC_ILR72); // GPIO2_8 Interrupt Line
    iowrite8(0x0, mem + INTC_ILR73); // GPIO2_9 Interrupt Line
    iowrite8(0x0, mem + INTC_ILR74); // GPIO2_10 Interrupt Line
    iowrite8(0x0, mem + INTC_ILR75); // GPIO2_11 Interrupt Line

    // Unmask the the GPIO lines to enable interrupts on those GPIO lines
    iowrite32((GPIO_8 | GPIO_9 | GPIO_10 | GPIO_11), mem + INTC_MIR_CLEAR2);
    
    // Close the IRQ Control registers memory
    iounmap( mem );
    
    return 0;
}

nanosecs_abs_t throttle_pwm_start, throttle_pwm_width;
uint throttle_init = 0;
int irq_handler_throttle(rtdm_irq_t *irq_handle)
{
    if( !throttle_init )
    {
        throttle_pwm_start = rtdm_clock_read();
        throttle_init = 1;
    }
    else
    {
        throttle_pwm_width = rtdm_clock_read() - throttle_pwm_start;
        rtdm_printk("Throttle Pulse Width: %llu ns\n", throttle_pwm_width);
        throttle_init = 0;
    }

    return 0;
}

nanosecs_abs_t yaw_pwm_start, yaw_pwm_width;
uint yaw_init = 0;
int irq_handler_yaw(rtdm_irq_t *irq_handle)
{
    if( !yaw_init )
    {
        yaw_pwm_start = rtdm_clock_read();
        yaw_init = 1;
    }
    else
    {
        yaw_pwm_width = rtdm_clock_read() - yaw_pwm_start;
        rtdm_printk("Yaw Pulse Width: %llu ns\n", yaw_pwm_width);
        yaw_init = 0;
    }

    return 0;
}

nanosecs_abs_t pitch_pwm_start, pitch_pwm_width;
uint pitch_init = 0;
int irq_handler_pitch(rtdm_irq_t *irq_handle)
{
    if( !pitch_init )
    {
        pitch_pwm_start = rtdm_clock_read();
        pitch_init = 1;
    }
    else
    {
        pitch_pwm_width = rtdm_clock_read() - pitch_pwm_start;
        rtdm_printk("Pitch Pulse Width: %llu ns\n", pitch_pwm_width);
        pitch_init = 0;
    }

    return 0;
}

nanosecs_abs_t roll_pwm_start, roll_pwm_width;
uint roll_init = 0;
int irq_handler_roll(rtdm_irq_t *irq_handle)
{
    if( !roll_init )
    {
        roll_pwm_start = rtdm_clock_read();
        roll_init = 1;
    }
    else
    {
        roll_pwm_width = rtdm_clock_read() - roll_pwm_start;
        rtdm_printk("Roll Pulse Width: %llu ns\n", roll_pwm_width);
        roll_init = 0;
    }

    return 0;
}

void cleanup_gpe_irq()
{
    int i;

    rtdm_printk("GPE-IRQ: Releasing IRQ resources...\n");
    for(i = 0; i < num_of_receiver_chs; i++)
    {
        if( irq_requested[i] )
            rtdm_irq_free( &receiver_irqs[i] );

        if( irq_gpio_requested[i] )
            gpio_free( irq_chs[i].gpio_desc.gpio );
    }
    rtdm_printk("GPE-IRQ: GPE IRQ resources released\n");
}
