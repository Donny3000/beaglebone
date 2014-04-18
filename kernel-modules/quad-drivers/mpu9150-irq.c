/*
 * mpu9150-irq.c
 *
 *  Created on: Apr 3, 2014
 *      Author: Donald R. Poole, Jr.
 */

#include <linux/module.h>
#include <linux/gpio.h>
#include <native/pipe.h>
#include <native/mutex.h>

#include "mpu9150.h"
#include "quad-drivers-gpio.h"
#include "quad-drivers-types.h"
#include "mpu9150-drv.h"
#include "mpu9150-irq.h"

static irq_ch_desc_t mpu9150_irq_desc = { // P8_10 (gpio2_4) => gpmc_wen or TIMER6
	{GPIO_PIN_NUM(2, 4), GPIOF_IN, "MPU9150_INT"},
	irq_handler_mpu9150
};

static uint          irq_gpio_requested;            // Store the success/failure of requesting the IRQ gpio pin.
static uint          irq_requested;                 // Store the success/failure of requesting the MPU9150 INT IRQ channel, in addition the assigned IRQ number.
static RT_PIPE       pipe_desc;
static rtdm_irq_t    mpu9150_irq;                   // IRQ descriptor for the MPU9150 INT line
static const char    *synch = "<s>";
static const size_t   synchSize = sizeof(synch);

int init_mpu9150_irq(void)
{
	void __iomem *mem;
	int retval, irq;
	uint regval;

	rtdm_printk("MPU9150-IRQ: Initializing interrupt interface...\n");

	// Preliminary Setup: Step 1: Create a message pipe for use within the ISR
	retval = rt_pipe_create(&pipe_desc, "mpu9150", P_MINOR_AUTO, MPU9150_FIFO_DEPTH+synchSize);
	if( retval )
	{
		rtdm_printk("MPU9150-IRQ: Error: Failed to create data flow pipe\n");
		return retval;
	}

	// 1st Step: Request the GPIO lines for the IRQ channels and create the IRQ object
	retval = gpio_request_one(mpu9150_irq_desc.gpio_desc.gpio, mpu9150_irq_desc.gpio_desc.flags, mpu9150_irq_desc.gpio_desc.label);
	if(retval != 0)
	{
		irq_gpio_requested = 0;
		rtdm_printk("MPU9150-IRQ: Error: Failed to request GPIO pin#%i for IRQ (error %i)\n", mpu9150_irq_desc.gpio_desc.gpio, retval);
		return retval;
	}
	irq_gpio_requested = 1;

	irq = gpio_to_irq( mpu9150_irq_desc.gpio_desc.gpio );
	if(irq >= 0)
	{
		irq_requested = 1;
		if((retval = rtdm_irq_request(&mpu9150_irq, irq, mpu9150_irq_desc.isr, 0, mpu9150_irq_desc.gpio_desc.label, NULL)) < 0)
		{
			switch( retval )
			{
				case -EINVAL:
					rtdm_printk("MPU9150-IRQ: ERROR: Invalid parameter was given to rtdm_irq_request().\n");
					break;
				case -EBUSY:
					rtdm_printk("MPU9150-IRQ: ERROR: The requested IRQ line#%i from GPIO#%i is already in use\n", irq, mpu9150_irq_desc.gpio_desc.gpio);
					break;
				default:
					rtdm_printk("MPU9150-IRQ: Unknown error occurred while requesting RTDM IRQ.\n");
			}

			return retval;
		}

		rtdm_printk("MPU9150-IRQ: Initialized GPIO #%i to IRQ #%i\n", mpu9150_irq_desc.gpio_desc.gpio, irq);
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
		return -ENOMEM;
	}

	// Configure GPIO2_4 pin 0-7 output
	// Write the pin configuration
	iowrite8((SLEW_FAST | INPUT_EN | PULLUP | PULLUPDOWN_DIS | M7), mem + CONF_GPMC_WEN);

	// close the pin conf address space
	iounmap( mem );

	// GPIO Bank 2: Setup the IRQ registers with the appropriate values
	mem = ioremap(GPIO2_START_ADDR, GPIO2_SIZE);
	if( !mem )
	{
		rtdm_printk("MPU9150-IRQ: ERROR: Failed to remap memory for GPIO Bank 2 IRQ pin configuration.\n");
		return -ENOMEM;
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
		return -ENOMEM;
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

	rtdm_printk("MPU9150-IRQ: Interrupt interface initialization complete!\n");

	return 0;
}

static int is_busy = 0;
// The assumption here is that when the data is ready (DATA_RDY_EN) from the
// MPU-9150, then the interrupt line will be asserted high.
int irq_handler_mpu9150(rtdm_irq_t *irq_handle)
{
	int retval, value, i;
	ushort count, count_l;
	RT_PIPE_MSG *msg = NULL;
	char *msgPtr = NULL;

	// Check to make sure the MPU-9150 was found
	if( !mpu9150_device )
		return RTDM_IRQ_HANDLED;

	if( !is_busy )
	{
		is_busy = 1;

		// Step 1: Check to see what interrupt was sent
		if((value = i2c_smbus_read_byte_data(mpu9150_device, MPU9150_INT_STATUS)) >= 0)
		{
			if(value & 0x1) // Data Ready Interrupt
			{
				// Step 2a: Read the high byte count from the FIFO count
				if((count = i2c_smbus_read_byte_data(mpu9150_device, MPU9150_FIFO_COUNTH)) < 0)
					return RTDM_IRQ_HANDLED;

				// Step 2b: Read the low byte count from the FIFO count
				if((count_l = i2c_smbus_read_byte_data(mpu9150_device, MPU9150_FIFO_COUNTL)) < 0)
					return RTDM_IRQ_HANDLED;

				// Step 2c: Get the total if all went well
				count |= (count_l & 0xFF);

				msg = rt_pipe_alloc(&pipe_desc, count+synchSize);
				if( !msg )
					return RTDM_IRQ_HANDLED;
				msgPtr = P_MSGPTR(msg);

				// Step 3a: Copy the sync string
				strncpy(msgPtr, synch, synchSize);
				// Step 3b: Read the FIFO data and store the data in the pipe
				// buffer
				for(i = 0; i < count; i++)
				{
					if((value = i2c_smbus_read_byte_data(mpu9150_device, MPU9150_FIFO_R_W)) < 0)
						*(msgPtr + synchSize + i) = (char) (value & 0xFF);
				}

				// Step 4: Send the msg down the pipe
				if((retval = rt_pipe_send(&pipe_desc, msg, P_MSGSIZE(msg), P_NORMAL)) < 0)
				{
					rtdm_printk("MPU9150-IRQ: Failed to send pipe message: ");
					if(retval == -EINVAL)
						rtdm_printk("Invalid pipe descriptor given.\n");
					else if(retval == -EIDRM)
						rtdm_printk("Pipe descriptor is closed.\n");
					else if(retval == -ENODEV || retval == -EBADF)
						rtdm_printk("Pipe is scrambled.\n");

					return RTDM_IRQ_HANDLED;
				}
			}
			else
				return RTDM_IRQ_NONE;
		}

		is_busy = 0;
	}

	rtdm_printk("Done!\n");

	return RTDM_IRQ_HANDLED;
}

void cleanup_mpu9150_irq(void)
{
	rtdm_printk("MPU9150-IRQ: Releasing IRQ resources...\n");

	if( irq_requested )
	{
		rtdm_irq_disable( &mpu9150_irq );
		rtdm_irq_free( &mpu9150_irq );
	}

	if( irq_gpio_requested )
		gpio_free( mpu9150_irq_desc.gpio_desc.gpio );

	// Close the pipe
	rt_pipe_delete( &pipe_desc );

	rtdm_printk("MPU9150-IRQ: IRQ resources released\n");
}
