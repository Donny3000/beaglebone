/*
 * mpu9150-irq.h
 *
 *  Created on: Apr 3, 2014
 *      Author: Donald R. Poole, Jr.
 */

#ifndef MPU9150_IRQ_H_
#define MPU9150_IRQ_H_

#include <linux/module.h>
#include <linux/gpio.h>
#include <rtdm/rtdm_driver.h>
#include "mpu9150.h"
#include "quad-drivers-types.h"

/*
 * Initialize the MPU9150 IRQ channel
 *
 * @return 0 on success or error code in case of failure.
 */
int init_mpu9150_irq(void);

/*
 * Interrupt handler for the MPU9150 INT signal
 *
 * @param irq_handle - The IRQ handle as return by rtdm_irq_request()
 *
 * @return 0 or a combination of RTDM_RQ_XXX
 */
int irq_handler_mpu9150(rtdm_irq_t *irq_handle);

/*
 * Release the IRQ gpio line
 */
void cleanup_mpu9150_irq(void);

#endif /* MPU9150_IRQ_H_ */
