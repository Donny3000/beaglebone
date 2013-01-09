/*
 * gpe-irq.c
 *
 * GPE IRQ functionality definitions
 *
 * Author: Donald R. Poole, Jr.
 */

#include "gpio.h"
#include "gpe-irq.h"

static u8                   num_of_receiver_chs=4;                              // The number of channels being read from the RC Rx
static gpe_irq_ch_desc_t    irq_chs[4] = {
    {{GPIO_PIN_NUM(2, 8), GPIOF_IN, "GPE_Throttle"}, irq_handler_throttle},
    {{GPIO_PIN_NUM(2, 9), GPIOF_IN, "GPE_Roll"}, irq_handler_roll},
    {{GPIO_PIN_NUM(2, 10), GPIOF_IN, "GPE_Pitch"}, irq_handler_pitch},
    {{GPIO_PIN_NUM(2, 11), GPIOF_IN, "GPE_Yaw"}, irq_handler_yaw}
};
static uint                 irq_gpio_requested[4];                              // Store the success/failure of requesting the IRQ gpio pin.
static uint                 irq_requested[4];                                   // Store the success/failure of requesting the GPE IRQ channels, in addition the assigne IRQ number.
static rtdm_irq_t           receiver_irqs[4];                                   // IRQ descriptors for the PWMs coming from the receiver (Throttle, roll, pitch & yaw)

int init_irq()
{
    void __iomem *mem;
	int i, retval, irq;
    uint regval;
    
    // Configure the pin for GPIO input
    rtdm_printk("GPE-IRQ: Configuring IRQ GPIO pin as follows: SLEW_FAST | INPUT_EN | PULLDOWN | PULLUPDOWN_EN | MODE_7....\n");
    mem = ioremap(GPMC_CONF_ADDR_START, GPMC_CONF_ADDR_SIZE);
    if( !mem )
    {
        rtdm_printk("GPE-IRQ: ERROR: Failed to remap memory for IRQ pin configuration.\n");
        return 0;
    }

    // Configure GPIO1 pins 0-7 for output
    // Write the pin configuration to the 8 output channels ()
    for(i=CONF_LCD_DATA2; i<=CONF_LCD_DATA5; i+=4)
    {
        iowrite8((SLEW_FAST | INPUT_EN | PULLDOWN | PULLUPDOWN_EN | M7), mem + i);
    }
    // close the pin conf address space
    iounmap( mem );

    // GPIO Bank 2: Setup the IRQ registers with the appropriate values
    mem = ioremap(GPIO2_START_ADDR, GPIO2_SIZE);
    if( !mem )
    {
        rtdm_printk("GPE-IRQ: ERROR: Failed to remap memory for GPIO Bank 2 IRQ pin configuration.\n");
        return 0;
    }

    // Enable the IRQ ability for GPIO0_26 & GPIO0_27
    regval = ioread32(mem + GPIO_IRQSTATUS_0);
    regval |= (GPIO_8 | GPIO_9 | GPIO_10 | GPIO_11);
    iowrite32(regval, mem + GPIO_IRQSTATUS_0);

    // Set Pin 26 & 27 for rising- and falling- edge detection
    regval = ioread32(mem + GPIO_RISINGDETECT);
    regval |= (GPIO_8 | GPIO_9 | GPIO_10 | GPIO_11);
    iowrite32(regval, mem + GPIO_RISINGDETECT);
    regval = ioread32(mem + GPIO_FALLINGDETECT);
    regval |= (GPIO_8 | GPIO_9 | GPIO_10 | GPIO_11);
    iowrite32(regval, mem + GPIO_FALLINGDETECT);
    
    // Release the mapped memory
    iounmap( mem );

    // Request the GPE IRQ channels
    for(i = 0; i < num_of_receiver_chs; i++)
    {
        retval = gpio_request_one(irq_chs[i].gpio_desc.gpio, irq_chs[i].gpio_desc.flags, irq_chs[i].gpio_desc.label);
        if( 0 != retval )
        {
            irq_gpio_requested[i] = 0;
            rtdm_printk("GPE-IRQ: Error: Failed to request GPIO pin#%i for IRQ (error %i)\n", i, retval);
            return retval;
        }
        else
        {
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
    }

    return 0;
}

int irq_handler_throttle(rtdm_irq_t *irq_handle)
{
    return 0;
}

int irq_handler_yaw(rtdm_irq_t *irq_handle)
{
    return 0;
}

int irq_handler_pitch(rtdm_irq_t *irq_handle)
{
    return 0;
}

int irq_handler_roll(rtdm_irq_t *irq_handle)
{
    return 0;
}

void cleanup_irq()
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
