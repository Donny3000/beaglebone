/*
 * mpu9150-drv.h
 *
 *  Created on: Apr 5, 2014
 *      Author: victory
 */

#ifndef MPU9150_DRV_H_
#define MPU9150_DRV_H_

#include <linux/i2c.h>

// Global module variable to hold the i2c client device information.
extern struct i2c_client *mpu9150_device;

/*
 * Wake-up the MPU9150
 *
 * @return 0 on success or error code in case of failure.
 */

int wakeup_mpu9150(const struct i2c_client *mpu9150);

/*
 * Put the MPU9150 into low-power mode
 *
 * @return 0 on success or error code in case of failure.
 */

int sleep_mpu9150(const struct i2c_client *mpu9150);

/*
 * Initialize the MPU9150
 *
 * @return 0 on success or error code in case of failure.
 */
int init_mpu9150_device(const struct i2c_client *mpu9150);

#endif /* MPU9150_DRV_H_ */
