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
#include "gpio.h"

int main(void)
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
    reg = reg & (0xFFFFFFFF - GPIO1_6);
    *gpio_oe_addr = reg;

    printf("Toggling GPIO1_6\n");
    for(;;)
    {
    	*gpio_setdataout_addr = GPIO1_6;
    	usleep( 5000 );
    	*gpio_cleardataout_addr = GPIO1_6;
    	usleep( 5000 );
    }
    /*printf("GPIO mapped to %p\n", gpio_addr);
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
    }*/

    return 0;
}
