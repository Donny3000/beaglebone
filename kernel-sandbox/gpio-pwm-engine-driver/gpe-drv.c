#include <linux/init.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <rtdm/rtdm_driver.h>
#include "gpe-irq.h"
#include "gpe-pwm.h"
#include "gpe-types.h"

#define DEVICE_NAME     "gpe"
#define SOME_SUB_CLASS  4711

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Donald R. Poole, Jr. <donny3000@gmail.com>");
MODULE_DESCRIPTION("GPE bridge between userspace and kernelspace");

static ushort num_of_chs = 1;
module_param(num_of_chs, ushort, S_IRUGO);

/*
 * The context of a device instance
 *
 * A context is created each time a device is opened and passed to
 * other device handlers when they are called.
 *
 */
typedef struct context_s
{
} context_t;


/*
 * Open the GPE device
 *
 * This function is called when the device shall be opened.
 *
 */
static int pwm_rtdm_open_nrt(struct rtdm_dev_context *context,
          rtdm_user_info_t *user_info, 
          int oflags)
{
    return 0;
}

/*
 * Close the GPE device
 *
 * This function is called when the device shall be closed.
 *
 */
static int pwm_rtdm_close_nrt(struct rtdm_dev_context *context,
           rtdm_user_info_t * user_info)
{
    return 0;
}

/*
 * Read from the GPE device
 *
 * This function is called when the device is read in non-realtime
 * context.
 *
 */
static ssize_t pwm_rtdm_read_nrt(struct rtdm_dev_context *context,
          rtdm_user_info_t * user_info, void *buf,
          size_t nbyte)
{
    size_t size;
    char uptime[32];

    size = sprintf(uptime, "%llu", get_pwm_width(0));
    rtdm_printk("GPE-DRVR: Sending Pulse Width of %s ns\n", uptime);
    if(rtdm_safe_copy_to_user(user_info, buf, uptime, size))
    {
        rtdm_printk("GPE-DRVR: ERROR: can't copy data from GPE driver\n");
    }

    return size;
}

/*
 * Write in the GPE device
 *
 * This function is called when the device is written in non-realtime context.
 *
 */
static ssize_t pwm_rtdm_write_nrt(struct rtdm_dev_context *context,
           rtdm_user_info_t * user_info,
           const void *buf, size_t nbyte)
{
    int i;
    int duty_perc = simple_strtoul(buf, NULL, 0);
    rtdm_printk("GPE-DRVR: Received Pulse Width of %i%%\n", duty_perc);
    for(i = 0; i < num_of_chs; i++)
        set_pwm_width(i, duty_perc);
    
    return nbyte;
}

/*
 * Driver operations describing the RTDM devices.
 */
static struct rtdm_device gpe_device = {
    .struct_version = RTDM_DEVICE_STRUCT_VER,

    .device_flags = RTDM_NAMED_DEVICE,
    .context_size = sizeof(context_t),
    .device_name = DEVICE_NAME,

    .open_nrt = pwm_rtdm_open_nrt,

    .ops = {
        .close_nrt = pwm_rtdm_close_nrt,
        .read_nrt = pwm_rtdm_read_nrt,
        .write_nrt = pwm_rtdm_write_nrt,
    },

    .device_class = RTDM_CLASS_EXPERIMENTAL,
    .device_sub_class = SOME_SUB_CLASS,
    .profile_version = 1,
    .driver_name = "GPE",
    .driver_version = RTDM_DRIVER_VER(0, 1, 2),
    .peripheral_name = "GPIO PWM Emgine (GPE)",
    .provider_name = "Donald R. Poole, Jr.",
    .proc_name = gpe_device.device_name,
};

/*
 * This function is called when the module is loaded
 *
 * It registers the GPE with the system.
 *
 */
static int __init rtdm_gpio_pwm_engine_init( void )
{
    int res, i;
    gpe_ch_desc_t gpe_chs[ num_of_chs ];

    if(num_of_chs < 1 || num_of_chs > 8)
    {
        rtdm_printk("GPE-DRVR: ERROR: Received an invalid number of channels. Valid values are 1-8.\n");
        return -EINVAL;
    }

    res = rtdm_dev_register( &gpe_device );
    if(res == 0)
    {
        rtdm_printk("GPE-DRVR: PWM driver registered without errors.\n");
    }
    else
    {
        rtdm_printk("GPE-DRVR: ERROR: PWM driver registration failed.\n");
        switch( res )
        {
            case -EINVAL:
                rtdm_printk("GPE-DRVR: ERROR: The device structure contains invalid entries. " \
                    "Check kernel log for further details.\n");
                break;

            case -ENOMEM:
                rtdm_printk("GPE-DRVR: ERROR: The context for an exclusive device cannot be allocated.\n");
                break;

            case -EEXIST:
                rtdm_printk("GPE-DRVR: ERROR: The specified device name of protocol ID is already in use.\n");
                break;

            case -EAGAIN:
                rtdm_printk("GPE-DRVR: ERROR: Some /proc entry cannot be created.\n");
                break;

            default:
                rtdm_printk("GPE-DRVR: ERROR: Unknown error code returned.\n");
                break;
        }
    }

    // Initialize the IRQ channels
    res = init_irq();
    if( res )
    {
        rtdm_printk("GPE-DRVR: ERROR: IRQ Initialization error occurred: %i\n", res);
    }
    else
    {
        rtdm_printk("GPE-DRVR: IRQ Initialization complete\n");
    }

    // Initialize channel parameters
    for(i = 0; i < num_of_chs; i++)
    {
        gpe_chs[i].channel = i;
        gpe_chs[i].pwmMinWidth = 0;
        gpe_chs[i].pwmMaxWidth = SWITCHING_FREQ;
    }

    res = init_pwm(gpe_chs, sizeof(gpe_chs) / sizeof(gpe_chs[0]) );
    if( res )
    {
        rtdm_printk("GPE-DRVR: ERROR: Initialization error occurred: %i\n", res);
    }
    else
    {
        rtdm_printk("GPE-DRVR: GPIO PWM Engine Initialization complete\n");
    }

    return res;
}

/*
 * This function is called when the module is unloaded
 *
 * It unregister the GPE, polling at 1000 ms for pending users.
 *
 */
static void __exit rtdm_gpio_pwm_engine_exit( void )
{
    rtdm_printk("GPE-DRVR: Preparing for shutdown...\n");
    cleanup_irq();
    cleanup_pwm();
    rtdm_dev_unregister(&gpe_device, 1000);
    rtdm_printk("GPE-DRVR: Shutdown complete.\n");
}

module_init( rtdm_gpio_pwm_engine_init );
module_exit( rtdm_gpio_pwm_engine_exit );
