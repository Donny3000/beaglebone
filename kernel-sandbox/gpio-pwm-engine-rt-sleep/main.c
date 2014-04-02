/*
 * blink.c
 *
 *  Created on: Dec 24, 2012
 *      Author: Donald R. Poole, Jr.
 */

#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <signal.h>
#include "gpio.h"
// Xenomai Specific Includes
#include <native/task.h>
#include <native/timer.h>

#define	PULSE_PERIOD	1000000 // 1 ms period
#define PULSE_WIDTH		(PULSE_PERIOD / 2)

typedef unsigned char uchar;
typedef unsigned int  uint;
typedef unsigned long ulong;

/*
 * Begin private variables
 */
static volatile void	*gpio = NULL;
static volatile uint	*gpio_oe_addr = NULL;
static volatile uint	*gpio_setdataout_addr = NULL;
static volatile uint	*gpio_cleardataout_addr = NULL;
static uint				BREAK = 0;
static uchar 			GPIO_BANK = 0;
static int 				RC_NUM = 1;
static RT_TASK			pwm_task[8];
static ulong			overruns = 0;
// File descriptor for /dev/mem
static int				fd = -1;

/*
 * Begin function declarations
 */
void pwm_func(void *arg);
void close_ctrl_loop(int sig);
int initpwm(uchar gpio_bank);
void cleanuppwm(void);

int main(void)
{
	signal(SIGINT, close_ctrl_loop);
	signal(SIGKILL, close_ctrl_loop);

	initpwm( 1 );

	pause();

	return 0;
}

void close_ctrl_loop(int sig)
{
	printf("Caught Ctrl-C or Kill signal.\n");
	cleanuppwm();
}

void pwm_func(void *arg)
{
	uint regVal = *gpio_oe_addr;
	uint err = rt_task_set_periodic(NULL, TM_NOW, PULSE_PERIOD);
	if( err )
		fprintf(stderr, "PWM: rt_task_set_periodic() returned error " \
				"code %i\n", err);

	// Enable the output of the specified pin
	regVal &= ~GPIO_6;
	*gpio_oe_addr = regVal;

	// Create pulse
	while( !BREAK )
    {
		//set_data_out has offset 0x194
		*gpio_setdataout_addr = GPIO_6;
		rt_task_sleep( PULSE_WIDTH );

		//clear_data_out has offset 0x190
		*gpio_cleardataout_addr = GPIO_6;
		rt_task_wait_period( &overruns );
		if( overruns )
			fprintf(stderr, "PWM: rt_task_wait_period() encountered %i "\
					"overruns\n", (int) overruns);

    }
}

int initpwm(uchar gpio_bank)
{
	int retval, i;

	// Check to make sure we have a gpio bank and it's 0 through 3.
	if(gpio_bank < 0 || gpio_bank > 3)
	{
		fprintf(stderr, "GPIO Bank %i is invalid.  Expected value of 0-3.\n", (int) gpio_bank);
		return 1;
	}
	GPIO_BANK = gpio_bank;

	printf("Memory-mapping GPIO Bank %i...\n", (int) GPIO_BANK);
	//O_SYNC makes the memory uncacheable
	fd = open("/dev/mem", O_RDWR | O_SYNC);
	if(fd < 0)
	{
		fprintf(stderr, "Could not open memory\n");
		return 0;
	}

	// Map memory for the correct memory bank.
	switch( GPIO_BANK )
	{
	case 0:
		gpio = mmap(NULL, GPIO0_SIZE, PROT_READ | PROT_WRITE,
					MAP_SHARED, fd, GPIO0_START_ADDR);
		break;

	case 1:
		gpio = mmap(NULL, GPIO1_SIZE, PROT_READ | PROT_WRITE,
					MAP_SHARED, fd, GPIO1_START_ADDR);
		break;

	case 2:
		gpio = mmap(NULL, GPIO2_SIZE, PROT_READ | PROT_WRITE,
					MAP_SHARED, fd, GPIO2_START_ADDR);
		break;

	case 3:
		gpio = mmap(NULL, GPIO3_SIZE, PROT_READ | PROT_WRITE,
					MAP_SHARED, fd, GPIO3_START_ADDR);
		break;
	}

	if(gpio == MAP_FAILED)
    {
		fprintf(stderr, "GPIO Mapping failed\n");
		close(fd);
		return 0;
    }

	// Set GPIO register address
	gpio_oe_addr = gpio + GPIO_OE;

	// Reset the GPIO_OE register
	*gpio_oe_addr = 0xFFFFFFFF;

	// Set the SETDATAOUT register address
	gpio_setdataout_addr = gpio + GPIO_SETDATAOUT;

	// Set the CLEARDATAOUT register address
	gpio_cleardataout_addr = gpio + GPIO_CLEARDATAOUT;

	printf("Memory-mapping GPIO Bank %i complete\n", (int) GPIO_BANK);

	printf("Starting PWM generation tasks.\n");

	mlockall(MCL_CURRENT | MCL_FUTURE);

	for(i = 0; i < RC_NUM; i++)
	{
		retval = rt_task_create(&pwm_task[i], NULL, 0, 99, 0);
		switch( retval )
		{
        	case -ENOMEM:
        		fprintf(stderr,
        				"Not enough real-time memory to create the pwm task#%i\n",
        				i);
        		return retval;

        	case -EEXIST:
        		fprintf(stderr, "The name for pwm task#%i is already in use\n",
        				i);
        		return retval;

        	case -EPERM:
        		fprintf(stderr, "rt_task_create() was called from an asynchronous context\n");
        		return retval;

        	case 0: // is returned upon success
        	default:
        		break;
		}

		rt_task_start(&pwm_task[i], &pwm_func, NULL);
	}

	printf("Finished!\n");

	return 0;
}

void cleanuppwm()
{
	int i = 0;

	for(; i < RC_NUM; i++)
		rt_task_delete( &pwm_task[i] );

	usleep(20000*2);

	if(fd != -1)
		close(fd);
}

/*int main(void)
{
    volatile void *gpio_addr = NULL;
    volatile unsigned int *gpio_oe_addr = NULL;
    volatile unsigned int *gpio_setdataout_addr = NULL;
    volatile unsigned int *gpio_cleardataout_addr = NULL;
    unsigned int reg;

    // Need to use O_SYNC here to tell the mmap that the memory is uncacheable.
    int fd = open("/dev/mem", O_RDWR | O_SYNC);
    gpio_addr = mmap(NULL, GPIO1_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, fd, GPIO1_START_ADDR);
    close( fd );
    printf("Mapping %X - %X (size: %X)\n", GPIO1_START_ADDR, GPIO1_END_ADDR, GPIO1_SIZE);

    gpio_oe_addr = gpio_addr + GPIO_OE;
    gpio_setdataout_addr = gpio_addr + GPIO_SETDATAOUT;
    gpio_cleardataout_addr = gpio_addr + GPIO_CLEARDATAOUT;

    if(gpio_addr == MAP_FAILED)
    {
        printf( "Unable to map GPIO\n" );
        exit( 1 );
    }

    reg = *gpio_oe_addr;
    reg = reg & (0xFFFFFFFF - GPIO_6);
    *gpio_oe_addr = reg;

    printf("Toggling GPIO1_6\n");
    for(;;)
    {
    	*gpio_setdataout_addr = GPIO_6;
    	//rt_task_sleep( 5000 );
    	*gpio_cleardataout_addr = GPIO_6;
    	//rt_task_wait_period( NULL );
    }
    printf("GPIO mapped to %p\n", gpio_addr);
    printf("GPIO OE mapped to %p\n", gpio_oe_addr);
    printf("GPIO SETDATAOUTADDR mapped to %p\n", gpio_setdataout_addr);
    printf("GPIO CLEARDATAOUT mapped to %p\n", gpio_cleardataout_addr);

    reg = *gpio_oe_addr;
    printf("GPIO1 configuration: %X\n", reg);
    reg = reg & (0xFFFFFFFF - USR1_LED);
    *gpio_oe_addr = reg;
    printf("GPIO1 configuration: %X\n", reg);

    printf( "Start blinking LED USR1\n" );
    while( 1 )
    {
        printf( "ON\n" );
        *gpio_setdataout_addr= USR1_LED;
        sleep( 1 );
        printf( "OFF\n" );
        *gpio_cleardataout_addr = USR1_LED;
        sleep( 1 );
    }

    return 0;
}*/
