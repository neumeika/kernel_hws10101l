/**************************************************
 Copyright (C), 2010-2012, Huawei Tech. Co., Ltd.
 File Name: kernel/drivers/misc/mrus51s.c
**************************************************/
#include <linux/module.h> 
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/types.h>
#include <linux/sysfs.h>
#include <linux/hkadc/hiadc_hal.h>
#include <linux/regulator/consumer.h>
#include <linux/interrupt.h>
#include <mach/gpio.h>
#include <linux/mux.h>
#include <linux/mrus51s.h>
#include <linux/platform_device.h>

#include <linux/slab.h>

#define MRUS51S_TIMER_INTERVAL  HZ //jiffies 1S

/*keep the current status:
1: open 
0: close
*/
#define MR_OPEN  (1)
#define MR_CLOSE (0)
int giMRsensorValue = 1;

#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif


#ifdef CONFIG_HAS_EARLYSUSPEND
struct early_suspend mrus51s_earlysuspend;
static void mrus51s_early_suspend(struct early_suspend *h);
static void mrus51s_late_resume(struct early_suspend *h);
#endif

//#define DEBUG_MRUS51S

/*Macro switch*/
#ifdef DEBUG_MRUS51S
#define MRUS_DMSG(format, args...) printk(KERN_INFO "[%s] (line: %u) " format "\n", \
__FUNCTION__, __LINE__, ##args)
#else
#define MRUS_DMSG(format, args...)
#endif

#define MRUS_ERRMSG(format, args...) printk(KERN_ERR "[%s] (line: %u) " format "\n", \
__FUNCTION__, __LINE__, ##args)


static ssize_t mrus51s_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    return sprintf(buf, "%d\n", giMRsensorValue);
}
static struct device_attribute mrus51s_attr =
    __ATTR(mrus51s_info, 0444, mrus51s_show, NULL);


static irqreturn_t mrus51s_interrupt(int irq, void *handle)
{            
    int status_old = 0;
    int status_new = 0;
    int irq_gpio = irq_to_gpio(irq);

    struct mrus51s_platform_data_struct *mr = handle;

    static unsigned long lTimerPre = 0;

    unsigned long lTimerCur = jiffies;

    wake_lock_timeout(&mr->mrus_wake_lock, 5*HZ); //delay 5 s, waite for timer schdule the work, then light up the lcd.
    if(time_before(lTimerCur, lTimerPre + MRUS51S_TIMER_INTERVAL)) 
    {
        mod_timer(&(mr->timer_detect), jiffies+MRUS51S_TIMER_INTERVAL/5);//after 0.2s, read a value, then report the value 
    }
    else
    {
        mod_timer(&(mr->timer_detect), jiffies+1);//read a value, then report the value immediately.
    }

    lTimerPre = jiffies;
    return IRQ_HANDLED;
}

static void mrus51s_work(struct work_struct *work)
{
    struct mrus51s_platform_data_struct *mrus = container_of(work, struct mrus51s_platform_data_struct, work);

    char *disconnected[2] = { "MR_STATE=OPEN", NULL };
    char *connected[2]    = { "MR_STATE=CLOSE", NULL };
    char **uevent_envp = NULL;
    int gpio_val = 0;

    gpio_val = gpio_get_value(mrus->irq_gpio);

    MRUS_ERRMSG("old status: giMRsensorValue: %d, new status gpio: %d", giMRsensorValue, gpio_val);

    giMRsensorValue = gpio_val;
    if(gpio_val == MR_OPEN)
    {
        uevent_envp = disconnected;
    }
    else 
    {
        uevent_envp = connected;
    }

    kobject_uevent_env(&mrus->dev->kobj, KOBJ_CHANGE, uevent_envp);
}

static void timer_detect_func(unsigned long arg)
{
    struct mrus51s_platform_data_struct *mrus = (struct mrus51s_platform_data_struct *)arg;

    queue_work(mrus->mrus51s_wq, &mrus->work); 
}

static int __devinit mrus51s_probe(struct platform_device *pdev)
{
    int ret = 0;
    
    struct mrus51s_platform_data_struct *msplatdev = pdev->dev.platform_data;
    struct mrus51s_platform_data_struct *mrus = kzalloc(sizeof(struct mrus51s_platform_data_struct), GFP_KERNEL);
    int irq_gpio = msplatdev->irq_gpio;

    mrus->dev = &(pdev->dev); 
    mrus->irq_gpio = irq_gpio;
    mrus->gpio_config = msplatdev->gpio_config;

    if (msplatdev->gpio_config) {
        ret = msplatdev->gpio_config(NORMAL);
        if (ret < 0) {
            MRUS_ERRMSG("mrus51s: set gpio error %d", ret);
            goto gpio_config_failed;
        }    
    }    
    
    ret = gpio_request(irq_gpio, "mrus51s");
    if (ret < 0) {
        MRUS_ERRMSG("mrus51s: request gpio error %d", ret);
        goto gpio_request_failed;
    }    

    ret = gpio_direction_input(irq_gpio);
    if (ret < 0) {
           MRUS_ERRMSG("gpio direction input failed %d", ret);
        goto gpio_direction_input_failed;
    } 

    ret = device_create_file(&pdev->dev, &mrus51s_attr);
    if (ret < 0) {
        MRUS_ERRMSG("creat sys file failed %d", ret);
        goto device_create_file_failed;
    } 

#ifdef CONFIG_HAS_EARLYSUSPEND
    mrus51s_earlysuspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN;
    mrus51s_earlysuspend.suspend = mrus51s_early_suspend;
    mrus51s_earlysuspend.resume = mrus51s_late_resume;
    register_early_suspend(&mrus51s_earlysuspend);
#endif

    giMRsensorValue = 1;
    //giMRsensorValue = gpio_get_value(mrus->irq_gpio);;

    wake_lock_init(&mrus->mrus_wake_lock, WAKE_LOCK_SUSPEND, "mrus51s sleep");

    mrus->mrus51s_wq = create_singlethread_workqueue("mrus51s_wq"); 
    if (!mrus->mrus51s_wq) 
    {
        MRUS_ERRMSG("create workque failed \n");
        goto create_singlethread_workqueue_failed;
    }

    INIT_WORK(&mrus->work, mrus51s_work);
    init_timer(&mrus->timer_detect);
    mrus->timer_detect.data = (unsigned long)mrus;  //pointer the current platfrom data
    mrus->timer_detect.function = &timer_detect_func;

     ret = request_irq(gpio_to_irq(irq_gpio), (irq_handler_t)mrus51s_interrupt, 
                      IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING | IRQF_NO_SUSPEND,
                      "mrus51s", mrus);
    if (ret < 0) {
        MRUS_ERRMSG("mrus51s: request irq failed %d", ret);
        goto request_threaded_irq_failed;
    }

    device_init_wakeup(&pdev->dev, true);
    MRUS_DMSG("mrus51s_probe succuss");
    return ret;
request_threaded_irq_failed:
    del_timer_sync(&mrus->timer_detect);
create_singlethread_workqueue_failed:
    device_remove_file(&pdev->dev, &mrus51s_attr);
device_create_file_failed:
gpio_direction_input_failed:
    gpio_free(irq_gpio);
gpio_request_failed:
    msplatdev->gpio_config(LOWPOWER);
gpio_config_failed:
    return ret;
}

static void mrus51s_early_suspend(struct early_suspend *h)
{
    ;
}

static void mrus51s_late_resume(struct early_suspend *h)
{
    ;
}

static struct platform_driver mrus51s_driver = {
    .probe = mrus51s_probe,
    .driver = {
        .name  = "mrus51s",
        .owner = THIS_MODULE,
    },
};

static int __init mrus51s_init(void)
{
    return platform_driver_register(&mrus51s_driver);
}

static void __exit mrus51s_exit (void)
{
    platform_driver_unregister(&mrus51s_driver);
}

late_initcall(mrus51s_init);
module_exit(mrus51s_exit);

MODULE_DESCRIPTION("mrus51s MR sensor");
MODULE_LICENSE("GPL");

