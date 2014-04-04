/*
 * mpu9150-irq.c
 *
 *  Created on: Apr 3, 2014
 *      Author: Donald R. Poole, Jr.
 */

#include "quad-drivers-gpio.h"
#include "mpu9150-irq.h"

static irq_ch_desc_t mpu9150_irq_desc = { // P8_10 (gpio2_4) => gpmc_wen or TIMER6
	{{GPIO_PIN_NUM(2, 4), GPIOF_IN, "MPU9150_INT"}, irq_handler_mpu9150}
};
static uint          irq_gpio_requested;  // Store the success/failure of requesting the IRQ gpio pin.
static uint          irq_requested;       // Store the success/failure of requesting the MPU9150 INT IRQ channel, in addition the assigned IRQ number.
static rtdm_irq_t    mpu9150_irq;         // IRQ descriptor for the MPU9150 INT line

int init_mpu9150_irq(void)
{
	void __iomem *mem;
	int i, retval, irq;
	uint regval;

	// 1st Step: Request the GPIO lines for the IRQ channels
	retval = gpio_request_one(mpu9150_irq_desc.gpio_desc.gpio, mpu9150_irq_desc.gpio_desc.flags, mpu9150_irq_desc.gpio_desc.label);
	if(retval != 0)
	{
		irq_gpio_requested = 0;
		rtdm_printk("MPU9150-IRQ: Error: Failed to request GPIO pin#%i for IRQ (error %i)\n", i, retval);
		return retval;
	}

	irq_gpio_requested = 1;
	irq = gpio_to_irq( mpu9150_irq_desc.gpio_desc.gpio );

	if(irq >= 0)
	{
		irq_requested = 1;
		retval = rtdm_irq_request(&mpu9150_irq, irq, mpu9150_irq_desc.isr, RTDM_IRQTYPE_EDGE, mpu9150_irq_desc.gpio_desc.label, NULL);
		switch( retval )
		{
			case -EINVAL:
				rtdm_printk("MPU9150-IRQ: ERROR: rtdm_irq_request() received and invalid parameter.\n");
				break;
			case -EBUSY:
				rtdm_printk("MPU9150-IRQ: ERROR: The requested IRQ line#%i from GPIO#%i is already in use\n", irq, mpu9150_irq_desc.gpio_desc.gpio);
				break;
			case 0:
			default:
				rtdm_printk("MPU9150-IRQ: Initialized GPIO #%i to IRQ #%i\n", mpu9150_irq_desc.gpio_desc.gpio, irq);
				break;
		}
	}
	else
	{
		irq_requested = 0;
		rtdm_printk("MPU9150-IRQ: ERROR: Failed to obtain IRQ number for GPIO #%i (error %i)\n", mpu9150_irq_desc.gpio_desc.gpio, retval);
		return retval;
	}

	// 2nd Step: Configure the pin for GPIO input
	rtdm_printk("MPU9150-IRQ: Configuring IRQ GPIO pin as follows: SLEW_FAST | INPUT_EN | PULLUP | PULLUPDOWN_DIS | MODE_7....\n");
	mem = ioremap(GPMC_CONF_ADDR_START, GPMC_CONF_ADDR_SIZE);
	if( !mem )
	{
		rtdm_printk("MPU9150-IRQ: ERROR: Failed to remap memory for IRQ pin configuration.\n");
		return mem;
	}

	// Configure GPIO2_4 pin 0-7 output
	// Write the pin configuration
	iowrite8((SLEW_FAST | INPUT_EN | PULLUP | PULLUPDOWN_DIS | M7), CONF_GPMC_WEN);

	// close the pin conf address space
	iounmap( mem );

	// GPIO Bank 2: Setup the IRQ registers with the appropriate values
	mem = ioremap(GPIO2_START_ADDR, GPIO2_SIZE);
	if( !mem )
	{
		rtdm_printk("MPU9150-IRQ: ERROR: Failed to remap memory for GPIO Bank 2 IRQ pin configuration.\n");
		return mem;
	}

	// Enable the IRQ ability for GPIO2_4
	regval = ioread32(mem + GPIO_IRQSTATUS_0);
	regval |= GPIO_4;
	iowrite32(regval, mem + GPIO_IRQSTATUS_0);

	// Set Pin 4 for rising- and falling- edge detection
	regval = ioread32(mem + GPIO_RISINGDETECT);
	regval |= GPIO_4;
	iowrite32(regval, mem + GPIO_RISINGDETECT);

	// Release the mapped memory
	iounmap( mem );

	// 3rd Step: Setup the IRQ controller to read the H/W interrupts
	mem = ioremap(INTC_REG_START, INTC_REG_SIZE);
	if( !mem )
	{
		rtdm_printk("MPU9150-IRQ: ERROR: Failed to remap IRQ Control Registers memory\n");
		return mem;
	}

	// Disable clock gating strategy for the. Will use more
	// power with this strategy disabled, but will decrease
	// latency.
	iowrite32(0x0, mem + INTC_SYSCONFIG);

	// Disable the input synchronizer auto-gating and the
	// functional clock auto-idle modes.  This will use more
	// power, but will decrease latency.
	iowrite32(0x1, mem + INTC_IDLE);

	// For each IRQ line, set the highest priority (0) and
	// route to the IRQ, not the FIQ (GPIOs cannot be routed
	// to FIQ)
	iowrite8(0x0, mem + INTC_ILR68);

	// Unmask the the GPIO lines to enable interrupts on those GPIO lines
	iowrite32(GPIO_4, mem + INTC_MIR_CLEAR2);

	// Close the IRQ Control registers memory
	iounmap( mem );

	return 0;
}

int irq_handler_mpu9150(rtdm_irq_t *irq_handle)
{
	return 0;
}

void cleanup_mpu9150_irq(void)
{
}
