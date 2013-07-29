/* Driver for SIM CARD dector

 *  Copyright 2010 S10 Tech. Co., Ltd.

 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Code Aurora Forum nor
 *       the names of its contributors may be used to endorse or promote
 *       products derived from this software without specific prior written
 *       permission.
 */

#include <linux/interrupt.h>
#include <mach/gpio.h>
#include <linux/platform_device.h>
#include <linux/sim_detect.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/mux.h>
#include <linux/delay.h>

#include <linux/wakelock.h>

#define SIM_ONDUTY        1
#define SIM_OFFDUTY       0


static int g_sim_state       = SIM_OFFDUTY;

/* for sim detect anti-vibrate or robust */
static unsigned int g_sim_pluge_times = 0;

/* store the dectect gpio */
static int g_DetGpio = 0;

static struct wake_lock sim_wake_lock;

static int sim_detect_remove(struct platform_device *pdev);

/*Macro switch*/
#ifdef DEBUG_SIM
#define SIM_DMSG(format, args...) printk(KERN_INFO "[%s] (line: %u) " format "\n", \
__FUNCTION__, __LINE__, ##args)
#else
#define SIM_DMSG(format, args...)
#endif

#define SIM_ERRMSG(format, args...) printk(KERN_ERR "[%s] (line: %u) " format "\n", \
__FUNCTION__, __LINE__, ##args)


MODULE_PARM_DESC(g_sim_state, "sim card initial status");
static int sim_get_status(char *buffer, struct kernel_param *kp)
{
    int ret;

    switch( g_sim_state )
    {
        case SIM_OFFDUTY:
            ret = sprintf(buffer, "%s", "off");
            break;
        case SIM_ONDUTY:
            ret = sprintf(buffer, "%s", "on");
            break;
        default:
            ret = sprintf(buffer, "%s", "unknow");
            break;
    }

    return ret;
}
module_param_call(g_sim_state, NULL, sim_get_status,
                    &g_sim_state, 0644);


/*get the driver's attribute*/
static ssize_t sim_attr_show(struct device_driver *driver, char *buf)
{
    int buf_len = 0;

    if( (g_sim_state == SIM_ONDUTY) || (g_sim_state == SIM_OFFDUTY) )
    {
        *((unsigned int *)buf) = g_sim_pluge_times;
        buf_len += sizeof(g_sim_pluge_times);
        *((bool *)(buf + buf_len)) = (SIM_ONDUTY == g_sim_state) ? true : false;
        buf_len += sizeof(bool);
        return buf_len;
    }
    else
    {
        return -1;
    }
}
static struct driver_attribute driver_attr_state =
    __ATTR(state, S_IRUGO, sim_attr_show, NULL);


/*when the interrupt happens, delay 200ms to call work read the sim card's state*/
static void sim_detect_delay_func(struct work_struct *work)
{
    int i = 0;
    int iStateOld = gpio_get_value(g_DetGpio);
    int iStateNew = iStateOld;

    SIM_DMSG("sim_detect_delay_func enter\n");

    for(i=0; i<10; i++) //delay 200ms to check if it's really touch
    {
        msleep(20);
        iStateNew = gpio_get_value(g_DetGpio);
        if(iStateNew != iStateOld)
        {
            wake_unlock(&sim_wake_lock);
            return;  //no really touch, just return
        }
    }

    //really touch, continue
    if(g_sim_state == iStateNew)//state not change, just return
    {
        wake_unlock(&sim_wake_lock);
        return;
    }

	printk(KERN_INFO "sim card status change from %s to %s\n",
           g_sim_state == SIM_ONDUTY ? "on duty":"off duty",
           iStateNew == SIM_ONDUTY ? "on duty":"off duty" );

    if( iStateNew != g_sim_state )
    {
        g_sim_state = iStateNew;
        g_sim_pluge_times++;
    }

    wake_unlock(&sim_wake_lock);
    return;
}

static DECLARE_DELAYED_WORK(sim_detect_event, sim_detect_delay_func);

static irqreturn_t sim_detect_interrupt(int irq, void *dev_id)
{
    wake_lock(&sim_wake_lock);
    schedule_delayed_work(&sim_detect_event, msecs_to_jiffies(200));
    return IRQ_HANDLED;
}


static int sim_detect_probe(struct platform_device *pdev)
{
    int ret;
    struct sim_detect_platform_data *pdata = pdev->dev.platform_data;

    int irq_gpio = pdata->det_irq_gpio;

    g_DetGpio = irq_gpio;

    if (pdata->det_gpio_config)
    {
        ret = pdata->det_gpio_config(NORMAL);
        if (ret < 0)
        {
            SIM_ERRMSG("sim_detect: set gpio error %d", ret);
            goto gpio_config_failed;
        }
    }

    ret = gpio_request(irq_gpio, "sim_detect");
    if (ret < 0)
    {
        SIM_ERRMSG("sim_detect: request gpio error %d", ret);
        goto gpio_request_failed;
    }

    ret = gpio_direction_input(irq_gpio);
    if (ret < 0)
    {
        SIM_ERRMSG("gpio direction input failed %d", ret);
        goto gpio_direction_input_failed;
    }

    wake_lock_init(&sim_wake_lock, WAKE_LOCK_SUSPEND, "sim detect");

    ret = request_irq(gpio_to_irq(irq_gpio), sim_detect_interrupt,
                    IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING | IRQF_NO_SUSPEND,
                    "sim_detect", NULL);
    if (ret < 0)
    {
        SIM_ERRMSG("request sim_detect_interrupt error. ret=%d\n", ret);
        goto request_irq_failed;
    }

    g_sim_state = gpio_get_value(irq_gpio);
    printk(KERN_INFO "sim card %s\n", g_sim_state == SIM_ONDUTY ? "on duty":"off duty");

    return ret;
request_irq_failed:
    wake_lock_destroy(&sim_wake_lock);
gpio_direction_input_failed:
    gpio_free(irq_gpio);
gpio_request_failed:
    pdata->det_gpio_config(LOWPOWER);
gpio_config_failed:
    return ret;
}

static int  sim_detect_suspend(struct platform_device *pdev, pm_message_t state)
{
    struct sim_detect_platform_data *pdata = pdev->dev.platform_data;

    if (!pdata) {
        dev_err(&pdev->dev, "sim detect is NULL\n");
        return 0;
    }

    pdata->det_gpio_config(LOWPOWER);

    return 0;
}

static int  sim_detect_resume(struct platform_device *pdev)
{
    struct sim_detect_platform_data *pdata = pdev->dev.platform_data;

    if (!pdata) {
        dev_err(&pdev->dev, "sim detect is NULL\n");
        return 0;
    }

    pdata->det_gpio_config(NORMAL);

    return 0;
}

static struct platform_driver sim_detect_driver =
{
    .probe     = sim_detect_probe,
    .remove    = sim_detect_remove,
    .suspend   = sim_detect_suspend,
    .resume    = sim_detect_resume,
    .driver    =
    {
        .name  = "sim_detect",
        .owner = THIS_MODULE,
    },
};

static int sim_detect_remove(struct platform_device *pdev)
{
    struct sim_detect_platform_data *pdata = pdev->dev.platform_data;
    disable_irq(gpio_to_irq(pdata->det_irq_gpio));
    driver_remove_file(&sim_detect_driver.driver, &driver_attr_state);
    wake_lock_destroy(&sim_wake_lock);
    gpio_free(pdata->det_irq_gpio);
    pdata->det_gpio_config(LOWPOWER);
    return 0;
}

static int __init sim_detect_init(void)
{
    int rc = 0;
    rc = platform_driver_register(&sim_detect_driver);
    if (rc < 0)
    {
        SIM_ERRMSG("register sim detect driver error: %d\n", rc);
        return rc;
    }

    if ( (rc = driver_create_file(&sim_detect_driver.driver, &driver_attr_state)) )
    {
        platform_driver_unregister(&sim_detect_driver);
        SIM_ERRMSG("failed to create sysfs entry(state): %d\n", rc);
    }
    return rc;
}

static void __exit sim_detect_exit(void)
{
    return platform_driver_unregister(&sim_detect_driver);
}

module_init(sim_detect_init);
module_exit(sim_detect_exit);

MODULE_LICENSE("Dual BSD/GPL");
MODULE_DESCRIPTION("Sim Card Detect Driver");
