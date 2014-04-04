#ifndef __GPE_IRQ_H
#define __GPE_IRQ_H

/*
 * gpe-irq.h
 *
 * GPE IRQ channel functionality declarations.
 *
 * Author: Donald R. Poole, Jr.
 */

#include <linux/module.h>
#include <linux/gpio.h>
#include <rtdm/rtdm_driver.h>
#include "quad-drivers-types.h"

/*
 * Initialize the GPE IRQ channels
 *
 * @return 0 on success or error code in case of failure.
 */
int init_gpe_irq(void);

/*
 * Interrupt handler for the Throttle control
 * 
 * @param irq_handle - The IRQ handle as return by rtdm_irq_request()
 *
 * @return 0 or a combination of RTDM_RQ_XXX
 */
int irq_handler_throttle(rtdm_irq_t *irq_handle);

/*
 * Interrupt handler for the Yaw control
 * 
 * @param irq_handle - The IRQ handle as return by rtdm_irq_request()
 *
 * @return 0 or a combination of RTDM_RQ_XXX
 */
int irq_handler_yaw(rtdm_irq_t *irq_handle);

/*
 * Interrupt handler for the Pitch control
 * 
 * @param irq_handle - The IRQ handle as return by rtdm_irq_request()
 *
 * @return 0 or a combination of RTDM_RQ_XXX
 */
int irq_handler_pitch(rtdm_irq_t *irq_handle);

/*
 * Interrupt handler for the Roll control
 * 
 * @param irq_handle - The IRQ handle as return by rtdm_irq_request()
 *
 * @return 0 or a combination of RTDM_RQ_XXX
 */
int irq_handler_roll(rtdm_irq_t *irq_handle);

/*
 * Release the IRQ gpio lines
 */
void cleanup_gpe_irq(void);

#endif
