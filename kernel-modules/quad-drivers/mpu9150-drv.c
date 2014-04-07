/*
 * i2c-mpu9150.c
 *
 *  Created on: Apr 4, 2014
 *      Author: Donald R. Poole, Jr.
 */

#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>

#include "mpu9150-drv.h"
#include "mpu9150-irq.h"

struct i2c_client *mpu9150_device = NULL;
EXPORT_SYMBOL_GPL(mpu9150_device);

/*****************************************************************************
 *              Begin the I2C driver callback functions                      *
 *****************************************************************************/

static int find_mpu9150_device(struct i2c_client *device)
{
	int address = device->addr;
	if(address == MPU9150_I2C_ADDRESS || address == (MPU9150_I2C_ADDRESS+1))
	{
		if( !mpu9150_device )
		{
			mpu9150_device = (struct i2c_client *) kzalloc(sizeof(struct i2c_client), GFP_KERNEL);
			if(mpu9150_device == NULL)
			{
				printk(KERN_ERR "I2C-MPU9150:%s: no memory\n", __func__);
				return -ENOMEM;
			}

			printk("I2C-MPU9150:%s: Found Invensense MPU-9150 Device at I2C address 0x%X", __func__, address);
			mpu9150_device = device;
		}

		return 0;
	}

	return -ENODEV;
}

static int mpu9150_i2c_detect(struct i2c_client *client, struct i2c_board_info *info)
{
	return find_mpu9150_device( client );
}

static int mpu9150_i2c_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	if(!i2c_check_functionality(client->adapter,
			I2C_FUNC_SMBUS_BYTE_DATA | I2C_FUNC_SMBUS_WORD_DATA |
			I2C_FUNC_SMBUS_I2C_BLOCK))
	{
		printk(KERN_ERR "I2C-MPU9150:%s: Needed I2C functionality is not supported\n", __func__);
		return -ENODEV;
	}

	// Check the address of the client to see if we are an MPU-9150 device
	return find_mpu9150_device( client );
}

static int mpu9150_i2c_remove(struct i2c_client *client)
{
	struct mpu9150_device_t *dev = i2c_get_clientdata( client );
	kfree( dev );

	if( mpu9150_device )
	{
		kfree( mpu9150_device );
		mpu9150_device = NULL;
	}

	return 0;
}

#ifdef CONFIG_PM
static int mpu9150_i2c_suspend(struct i2c_client *client, pm_message_t msg)
{
	struct mpu9150_device_t *dev __attribute__((__unused__)) = i2c_get_clientdata( client );
	// TODO: Need to put the MPU-9150 to sleep
	return 0;
}

static int mpu9150_i2c_resume(struct i2c_client *client)
{
	struct mpu9150_device_t *dev __attribute__((__unused__)) = i2c_get_clientdata( client );
	// TODO: Need to wake up the MPU-9150
	return 0;
}
#else

#define mpu9150_i2c_suspend NULL
#define mpu9150_i2c_resume  NULL

#endif

/*****************************************************************************
 *            Begin Declaring the I2C Drivers Setup Structure                *
 *****************************************************************************/
// Tell the I2C framework which chips we support
static const struct i2c_device_id mpu9150_i2c_id[] = {
	{ "i2c-mpu9150", 0 },
	{}
};

// List of valid address for the MPU9150
static unsigned short mpu9150_i2c_addresses[] = {
	0x68, 0x69
};
MODULE_DEVICE_TABLE(i2c, mpu9150_i2c_id);

// Structure representing the device driver for a particular class of
// I2C/SMBus slave devices.
static struct i2c_driver mpu9150_i2c_driver = {
	.probe    = mpu9150_i2c_probe,
	.remove   = mpu9150_i2c_remove,
	.id_table = mpu9150_i2c_id,
	.driver   = {
		.name = "i2c-mpu9150",
	},

	/* Device auto-detection */
	.class        = I2C_CLASS_HWMON,
	.detect       = mpu9150_i2c_detect,
	.address_list = mpu9150_i2c_addresses,

	//.shutdown     = mpu9150_shutdown,  /* optional */
	.suspend      = mpu9150_i2c_suspend, /* optional */
	.resume       = mpu9150_i2c_resume,  /* optional */
	//.command      = mpu9150_command,   /* optional, deprecated */
};

static int __init mpu9150_i2c_init_driver(void)
{
	int retval;

	if((retval = init_mpu9150_irq()) < 0)
		return retval;

	return i2c_add_driver( &mpu9150_i2c_driver );
}

static void __exit mpu9150_i2c_exit_driver(void)
{
	i2c_del_driver( &mpu9150_i2c_driver );
}

module_init( mpu9150_i2c_init_driver );
module_exit( mpu9150_i2c_exit_driver );

MODULE_AUTHOR("Donald R. Poole, Jr.");
MODULE_DESCRIPTION("Invensense MPU-9150 9-Axis IMU I2C Xenomai RTDM Client Driver");
MODULE_LICENSE("GPL");
