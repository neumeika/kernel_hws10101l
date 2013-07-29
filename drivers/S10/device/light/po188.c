
/**************************************************
 Copyright (C), 2009-2012, Huawei Tech. Co., Ltd.
 File Name: kernel/drivers/input/po188.c
 Author:  l00190626       Version:  1   Date:20120114
 Description:po188 driver
 Version: V1.0
 Function List:
 History:
**************************************************/
#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/uaccess.h>
#include <linux/earlysuspend.h>
#include <linux/err.h>
#include <linux/workqueue.h>
#include <linux/timer.h>
#include <linux/miscdevice.h>
#include <linux/sysfs.h>
#include <linux/kobject.h>
#include <linux/jiffies.h>
#include <linux/input.h>
#include <linux/hkadc/hiadc_hal.h>
#include <linux/regulator/consumer.h>
#include <linux/delay.h>


#include <linux/board_sensors.h>

/* Macro definition */
#define PO188_NAME "po188"
#define PO188_DEV_NAME "lightsensor"

#define SENSOR_POLLING_JIFFIES       1*HZ     //poll time set to 1 second

#define PO188_ADC_CHANNEL ADC_ADCIN3
#define TIMES    3

#define PO188_IOC_MAGIC 'l'
#define LIGHTSENSOR_IOCTL_GET_ENABLED _IOR(PO188_IOC_MAGIC, 1, int *)
#define LIGHTSENSOR_IOCTL_ENABLE      _IOW(PO188_IOC_MAGIC, 2, int *)
#define LIGHTSENSOR_IOCTL_GET_DELAY   _IOR(PO188_IOC_MAGIC, 3, int *)
#define LIGHTSENSOR_IOCTL_SET_DELAY   _IOW(PO188_IOC_MAGIC, 4, int *)


#define PO188_VCC_VOLTAGE 3300000

#define PO188_REPORT_THRESHOLD  20 //40

struct regulator* gPo188Regulator;

/*Macro switch*/
#ifdef DEBUG_PO188
#define PO188_DMSG(format, args...) printk(KERN_INFO "[%s] (line: %u) " format "\n", \
__FUNCTION__, __LINE__, ##args)
#else
#define PO188_DMSG(format, args...)
#endif

#define PO188_ERRMSG(format, args...) printk(KERN_ERR "[%s] (line: %u) " format "\n", \
__FUNCTION__, __LINE__, ##args)


/* Type definition */
struct po188de_driver_struct
{
    struct miscdevice        dev;
    struct file_operations   fops;

    /*when it's time to read the value, insert the read work into this workqueue to prepare execute*/
    struct workqueue_struct  *po188_wq;

    /*read data in a interval designated by this timer. */
    struct timer_list        timer;

    struct input_dev         *input_dev;
    struct early_suspend     early_suspend;
    //struct mutex           lock;
    spinlock_t    s_lock;

    /*keep this device's enable or disable status. when disable, don't execute suspend and resume function*/
    int   status_on;
    int   voltage_now;
    int   delay_time;
    int   last_voltage;
};

/* Static variable definition*/
static struct po188de_driver_struct po188_driver;

/* Function declaration */
static void po188_work_func(struct work_struct *work);
static DECLARE_WORK(po188_cb_work, po188_work_func);

/* following are the sysfs callback functions */
static ssize_t po188_vol_show(struct kobject *kobj,
                    struct kobj_attribute *attr, char *buf)
{
    int iVol = 0;
    spin_lock(&po188_driver.s_lock);
    iVol = po188_driver.voltage_now;
    spin_unlock(&po188_driver.s_lock);
    return sprintf(buf, "%d\n", iVol);
}

/* Define the device attributes */
static struct kobj_attribute po188_vol_attribute =
    __ATTR(light, 0644, po188_vol_show, NULL);

static ssize_t po188_delay_show(struct kobject *kobj,
                   struct kobj_attribute *attr, char *buf)
{
    int val;
     
    spin_lock(&po188_driver.s_lock);
    val = po188_driver.delay_time;
    spin_unlock(&po188_driver.s_lock);
     
    return sprintf(buf, "%d\n", val);
}

static ssize_t po188_delay_store(struct kobject *kobj,
                   struct kobj_attribute *attr,
                   const char *buf, size_t count)
{
    unsigned long interval_ms = 0;

    if (strict_strtoul(buf, 10, &interval_ms))
        return -EINVAL;

    if (!interval_ms)
        return -EINVAL;

    spin_lock(&po188_driver.s_lock);
    po188_driver.delay_time = interval_ms;
    spin_unlock(&po188_driver.s_lock);

    return count;
}

/* Define the device attributes */
static struct kobj_attribute po188_delay_attribute =
    __ATTR(delay_rate, 0664, po188_delay_show, po188_delay_store);

static struct attribute* po188_attributes[] =
{
    &po188_vol_attribute.attr,
    &po188_delay_attribute.attr,
    NULL
};

static struct attribute_group po188_defattr_group =
{
    .attrs = po188_attributes,
};

static inline int po188_get_converted_level(s32 fsr)
{
    int value;

    if (fsr <= 20)
        value = 20;
    else if (fsr <= 40)
        value = 40;
    else if (fsr <= 60)
        value = 60;
    else if (fsr <= 90)
        value = 90;
    else if (fsr <= 200)
        value = 200;
    else if (fsr <= 340)
        value = 340;
    else if (fsr <= 1200)
        value = 1200;
    else if (fsr <= 1900)
        value = 1900;
    else if (fsr <= 2600)
        value = 2600;
    else if (fsr <= 3200)
        value = 3200;
    else
        value = 3300;
    return value;
}

static int po188_read_adc( int channel_no)
{
    unsigned char reserve = 0;
    int value = 0;

    value = k3_adc_get_value(channel_no, &reserve);
/*
    if (value >= 0)
    {
        po188_driver.voltage_now = value;
    }
    else //if error, rerurn error code and ignore the value
    {
        po188_driver.voltage_now = 0;        
    }*/
    return value;
}

static void po188_report_data(int iCurVoltage)
{
    input_report_abs(po188_driver.input_dev, ABS_MISC,  iCurVoltage);            
    input_sync(po188_driver.input_dev);

    //po188_driver.last_voltage = po188_driver.voltage_now;
    PO188_DMSG(" report light value to HAL (%d)\n", iCurVoltage);
}   

static void po188_work_func(struct work_struct *work)
{     
    /*static int voltage[TIMES];//remember the last 3 value
    static int iVolIndex=0;   
    static int iSumCount=0;
    int i       = 0;
    int iSum    = 0;
    int iAveVol = 0;*/
    int iVoltage_new = 0;
    int iVoltage_last = 0;
    int ret = 0; 
     
    ret = po188_read_adc(PO188_ADC_CHANNEL);    
    if (ret < 0) //read value failed. invalid value, don't report. 
    {
        PO188_ERRMSG("po188 have not read adc value");
        msleep(5000);  //sleep 5s

        //delay 1s to read.
        mod_timer(&po188_driver.timer, jiffies + msecs_to_jiffies(po188_driver.delay_time));
        return;
    }

    //iVoltage_new = po188_get_converted_level(ret);
    iVoltage_new = ret;
/*    voltage[iVolIndex]=po188_driver.voltage_now;
    iVolIndex = (iVolIndex+1)%TIMES;

    if (iSumCount<TIMES)
    {
        iSumCount++;
    }

    for(i=0; i<iSumCount; i++)// count the current average value
    {
        iSum += voltage[i];
    }
    //remember the last 3 average voltage value
    iAveVol = iSum / iSumCount;  */

    spin_lock(&po188_driver.s_lock);
    po188_driver.voltage_now = iVoltage_new;
    iVoltage_last = po188_driver.last_voltage;
    spin_unlock(&po188_driver.s_lock);

    //check if it's reach the threshold data and report to hal level        
/*    if ( (po188_driver.last_voltage - iAveVol >= PO188_REPORT_THRESHOLD) ||
         (iAveVol - po188_driver.last_voltage >= PO188_REPORT_THRESHOLD) )
    {
        po188_report_data();
    }  */
//    if ( (iVoltage_last - iVoltage_new >= PO188_REPORT_THRESHOLD) ||
//         (iVoltage_new - iVoltage_last >= PO188_REPORT_THRESHOLD) )
    {    
        po188_report_data(iVoltage_new);
        spin_lock(&po188_driver.s_lock);
        po188_driver.last_voltage = iVoltage_new;
        spin_unlock(&po188_driver.s_lock);
    }

    mod_timer(&po188_driver.timer, jiffies + msecs_to_jiffies(po188_driver.delay_time));
}

static void po188_start_cb_thread(unsigned long data)
{
    queue_work(po188_driver.po188_wq, &po188_cb_work);
}

static long po188_ioctl(struct file* file, unsigned int cmd, unsigned long param)
{
    int ret = 0;
    int flags = 0;
    void __user *argp = (void __user *)param;
    int iDelayTime = 0;

    switch (cmd)
    {
        case LIGHTSENSOR_IOCTL_ENABLE:              
            if (copy_from_user(&flags, argp, sizeof(flags))) 
            {
                ret = -EFAULT;
            }
            else
            {
                if ( 0==flags )
                {    
                    PO188_DMSG("active disable pol88\n");

                    spin_lock(&po188_driver.s_lock);
                    po188_driver.status_on = false;
                    spin_unlock(&po188_driver.s_lock);
                    cancel_work_sync(&po188_cb_work);
                    flush_workqueue(po188_driver.po188_wq);
                    del_timer(&po188_driver.timer);

                    ret = k3_adc_close_channal(PO188_ADC_CHANNEL);
                    if (ret < 0)
                    {
                        PO188_ERRMSG("k3_adc_close_channal error\n");
                    }

                    ret = regulator_disable(gPo188Regulator);
                    if (ret < 0) {
                        PO188_ERRMSG("disable po188 vcc drive error"); 
                    }
                }
                else if (1 == flags)
                {
                    PO188_DMSG("active enable pol88\n");
                    ret = regulator_enable(gPo188Regulator);
                    if (ret < 0) {
                        PO188_ERRMSG("enable po188 vcc drive error");            
                        return ret;//regulator_enable error, return.
                    }

                    ret = k3_adc_open_channel(PO188_ADC_CHANNEL);
                    if (ret < 0)
                    {
                        PO188_ERRMSG("k3_adc_open_channel error\n");
                        regulator_disable(gPo188Regulator);
                        return ret;
                    } 

                    mod_timer(&po188_driver.timer, jiffies + msecs_to_jiffies(po188_driver.delay_time));
                    spin_lock(&po188_driver.s_lock);
                    po188_driver.status_on = true;
                    spin_unlock(&po188_driver.s_lock);
                }
            }
            break;

        case LIGHTSENSOR_IOCTL_GET_ENABLED:            
            spin_lock(&po188_driver.s_lock);
            flags = po188_driver.status_on;
            spin_unlock(&po188_driver.s_lock);
            if (copy_to_user(argp, &flags, sizeof(flags))) 
            {
                ret = -EFAULT;
            }
            break;

        case LIGHTSENSOR_IOCTL_GET_DELAY:
            spin_lock(&po188_driver.s_lock);
            iDelayTime = po188_driver.delay_time;
            spin_unlock(&po188_driver.s_lock);
            if (copy_to_user(argp, &iDelayTime, sizeof(iDelayTime))) 
            {
                ret = -EFAULT;
            }     

            break;

        case LIGHTSENSOR_IOCTL_SET_DELAY:

            if (copy_from_user(&iDelayTime, argp, sizeof(iDelayTime))) 
            {
                ret =  -EFAULT;
            }
            else
            {
                spin_lock(&po188_driver.s_lock);
                po188_driver.delay_time = iDelayTime;
                spin_unlock(&po188_driver.s_lock);
            }
            break;
        default:
            PO188_ERRMSG("CMD INVALID.\n");
            ret = -EINVAL;
            break;
    }

    return ret;
}

static void Po188_suspend(struct early_suspend *h)
{
    int ret = 0;
    if( po188_driver.status_on )
    {
        cancel_work_sync(&po188_cb_work);
        flush_workqueue(po188_driver.po188_wq);

        del_timer(&po188_driver.timer);

        ret = k3_adc_close_channal(PO188_ADC_CHANNEL);
        if (ret < 0)
        {
            PO188_ERRMSG("k3_adc_close_channal error\n");
        }

        ret = regulator_disable(gPo188Regulator);
        if (ret < 0) {
            PO188_ERRMSG("disable po188 vcc drive error");
        }
    }
}

static void Po188_resume(struct early_suspend *h)
{
    int ret = 0;
    if( po188_driver.status_on )
    {
        ret = regulator_enable(gPo188Regulator);
        if (ret < 0) {
            PO188_ERRMSG("enable po188 vcc drive error");
            return;
        }

        ret = k3_adc_open_channel(PO188_ADC_CHANNEL);
        if (ret < 0)
        {
            PO188_ERRMSG("k3_adc_open_channel error\n");
            regulator_disable(gPo188Regulator);
            return;
        }
        mod_timer(&po188_driver.timer, jiffies + msecs_to_jiffies(po188_driver.delay_time));
    }
}

static int __init po188_init(void)
{
    int err = -1;
    int ret = -1;
    struct kobject *kobj = NULL;
    struct regulator* ldo15_3v3;

    po188_driver.dev.name            = PO188_DEV_NAME;
    po188_driver.dev.minor           = MISC_DYNAMIC_MINOR;
    po188_driver.fops.unlocked_ioctl = po188_ioctl;
    po188_driver.dev.fops            = &po188_driver.fops;
    po188_driver.last_voltage        = 0;

    //set the initial delay value: 1s
    po188_driver.delay_time  = SENSOR_POLLING_JIFFIES; 
//    mutex_init(&po188_driver.lock);

    if ((err = misc_register(&po188_driver.dev)))
    {
        PO188_ERRMSG("misc_register failed");
        goto misc_register_failed;
    }

    //regulater config
    ldo15_3v3 = regulator_get(po188_driver.dev.this_device, "light-vcc");
    if (IS_ERR(ldo15_3v3)) {
        PO188_ERRMSG("cannot get po188 vcc drive");
        goto regulator_get_failed;
    }
    gPo188Regulator = ldo15_3v3;

    ret = regulator_set_voltage(ldo15_3v3, PO188_VCC_VOLTAGE, PO188_VCC_VOLTAGE);
    if (ret < 0) {
        PO188_ERRMSG("set po188 vcc drive error");
        goto regulator_set_voltage_failed;
    }
/*    ret = regulator_enable(ldo15_3v3);
    if (ret < 0) {
        PO188_ERRMSG("enable po188 vcc drive error");
        goto regulator_enable_failed;
    }*/

    po188_driver.po188_wq = create_singlethread_workqueue("po188_wq"); 
    if (!po188_driver.po188_wq) 
    {
        PO188_ERRMSG("create workque failed \n");
        goto create_singlethread_workqueue_failed;
    }

    //set time, and time overrun function
    init_timer(&po188_driver.timer);
    po188_driver.timer.expires = jiffies + msecs_to_jiffies( po188_driver.delay_time);
    po188_driver.timer.data = 0;
    po188_driver.timer.function = po188_start_cb_thread;
    //add_timer(&po188_driver.timer);

    po188_driver.input_dev = input_allocate_device();
    if (po188_driver.input_dev == NULL) {
        PO188_ERRMSG("po188_init : Failed to allocate input device\n");
        goto input_allocate_device_failed;
    }

    po188_driver.input_dev->name = PO188_DEV_NAME;
    input_set_drvdata(po188_driver.input_dev, &po188_driver);
    ret = input_register_device(po188_driver.input_dev);
    if (ret) {
        PO188_ERRMSG("Unable to register %s input device\n", po188_driver.input_dev->name);
        goto input_register_device_failed;
    }

    err = set_sensor_input(PO188, po188_driver.input_dev->dev.kobj.name);
    if (err) {
        PO188_ERRMSG("set_sensor_input error\n");
        goto set_sensor_input_failed;
    }

    set_bit(EV_ABS,po188_driver.input_dev->evbit);
    set_bit(EV_SYN,po188_driver.input_dev->evbit);
    input_set_abs_params(po188_driver.input_dev, ABS_MISC, 0, 10000, 0, 0);

/*no need early_suspend, the system can call enable/disable when suspend and resume*/
/*    po188_driver.early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
    po188_driver.early_suspend.suspend = Po188_suspend;
    po188_driver.early_suspend.resume = Po188_resume;
    register_early_suspend(&po188_driver.early_suspend);*/
    kobj = kobject_create_and_add(PO188_NAME, NULL);
    if (kobj == NULL) {
        goto kobject_create_and_add_failed;
    }
    if (sysfs_create_group(kobj, &po188_defattr_group)) {

        goto sysfs_create_group_failed;
    }

    spin_lock_init(&po188_driver.s_lock); 
    po188_driver.status_on = false;  //status_on must have init value

/*    //open adc channel
    ret = k3_adc_open_channel(PO188_ADC_CHANNEL);
    if (ret < 0)
    {
        PO188_ERRMSG("k3_adc_open_channel error\n");
        goto k3_adc_open_channel_failed;
    }*/

    return 0; 
 
//k3_adc_open_channel_failed:        
//  sysfs_remove_group(&po188_driver.dev.parent->kobj, &po188_defattr_group);
sysfs_create_group_failed:
    kobject_put(kobj);
kobject_create_and_add_failed:
set_sensor_input_failed:
    input_unregister_device(po188_driver.input_dev);
input_register_device_failed:
    input_free_device(po188_driver.input_dev);
input_allocate_device_failed:
    destroy_workqueue(po188_driver.po188_wq);
create_singlethread_workqueue_failed:
//   regulator_disable(ldo15_3v3);
//regulator_enable_failed:
regulator_set_voltage_failed:
    regulator_put(ldo15_3v3);
regulator_get_failed:
    misc_deregister(&po188_driver.dev);
misc_register_failed:
    return err;
}

static void __exit po188_exit (void)
{
    cancel_work_sync(&po188_cb_work);
    flush_workqueue(po188_driver.po188_wq);
    destroy_workqueue(po188_driver.po188_wq);
    del_timer(&po188_driver.timer);
    sysfs_remove_group(&po188_driver.dev.parent->kobj, &po188_defattr_group);
    input_unregister_device(po188_driver.input_dev);
    input_free_device(po188_driver.input_dev);
    regulator_disable(gPo188Regulator);
    misc_deregister(&po188_driver.dev);
}

device_initcall_sync(po188_init);
module_exit(po188_exit);

MODULE_DESCRIPTION("po188 light sensor driver");
MODULE_VERSION("1.0");
MODULE_LICENSE("GPL");
