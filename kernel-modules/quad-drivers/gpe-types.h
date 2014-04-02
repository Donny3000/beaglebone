#ifndef __TYPES_H
#define __TYPES_H

/*
 * types.h
 *
 * GPE type definitions
 *
 * Author: Donald R. Poole, Jr.
 */

#include <rtdm/rtdm_driver.h>

#define GPIO_PIN_NUM(bank, gpio) (32 * (bank) + (gpio))

typedef unsigned char uchar;
typedef unsigned int  uint;
typedef unsigned long ulong;

/*
 * Type definition describing the GPE channels
 */
typedef struct gpe_ch_desc {
    uint channel;
    uint pwmMinWidth;
    uint pwmMaxWidth;
} gpe_ch_desc_t;

typedef struct rcrx_irq_ch_desc {
    struct gpio gpio_desc;
    rtdm_irq_handler_t isr;
} rcrx_irq_ch_desc_t;

#endif
