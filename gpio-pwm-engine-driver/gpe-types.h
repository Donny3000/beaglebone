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

#define GPIO_PIN_NUM(bank, pin) (32 * bank + pin)
#define CALC_PULSE_WIDTH(min, max, percentage) (min + div((max - min) * percentage, 100))

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

typedef struct gpe_irq_ch_desc {
    struct gpio gpio_desc;
    rtdm_irq_handler_t isr;
} gpe_irq_ch_desc_t;

#endif
