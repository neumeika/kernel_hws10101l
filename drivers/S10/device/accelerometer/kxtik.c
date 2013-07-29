/*
 * drivers/input/misc/kxtik.c - KXTIK accelerometer driver
 *
 * Written by Kuching Tan <kuchingtan@kionix.com>
 *
 * Copyright (C) 2010-2011  Huawei.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/input/kxtik.h>
#include <linux/miscdevice.h>
#include <linux/io.h>
#include <asm/uaccess.h>
#include <linux/earlysuspend.h>
#include <linux/board_sensors.h>
#include <linux/hrtimer.h>


static signed short gs_sensor_data[3]={0};

static int kxtik_debug_mask = 0;
module_param_named(kxtik_debug, kxtik_debug_mask, int, S_IRUGO | S_IWUSR );

#define KXTIK_DMSG(format, args...) do {\
    if (kxtik_debug_mask) \
        printk(KERN_INFO "[%s] (line: %u) " format "\n", \
        __FUNCTION__, __LINE__, ##args);\
    } while (0)


/*
 * The following table lists the maximum appropriate poll interval for each
 * available output data rate.
 */
static const struct {
    unsigned int cutoff;
    u8 mask;
} kxtik_odr_table[] = {
    { 10,    ODR200F },
    { 20,    ODR100F },
    { 40,    ODR50F  },
    { 80,    ODR25F  },
    { 0,    ODR12_5F},
};

struct kxtik_data {
    struct i2c_client *client;
    struct kxtik_platform_data pdata;
    struct input_dev *input_dev;
    struct hrtimer timer;
    struct work_struct  poll_work;
    struct workqueue_struct *workqueue;
    unsigned int poll_interval;
    u8 shift;
    u8 ctrl_reg1;
    u8 data_ctrl;
    struct early_suspend early_suspend;
    spinlock_t    s_lock;/* reentrant protection for struct */
};

#ifdef CONFIG_HAS_EARLYSUSPEND
static void kxtik_early_suspend(struct early_suspend *h);
static void kxtik_late_resume(struct early_suspend *h);
#else
#define tik_suspend    NULL
#define tik_resume    NULL
#endif


static void kxtik_report_acceleration_data(struct kxtik_data *tik)
{
    s16 acc_data[3]; /* Data bytes from hardware xL, xH, yL, yH, zL, zH */
    s16 x, y, z;
    int err = -1;

    err = i2c_smbus_read_i2c_block_data(tik->client, KX_XOUT_L, 6, (u8 *)acc_data);
    if (err < 0)
        dev_err(&tik->client->dev, "accelerometer data read failed\n");

    spin_lock(&tik->s_lock);
    x = ((s16) le16_to_cpu(acc_data[tik->pdata.axis_map_x])) >> tik->shift;
    y = ((s16) le16_to_cpu(acc_data[tik->pdata.axis_map_y])) >> tik->shift;
    z = ((s16) le16_to_cpu(acc_data[tik->pdata.axis_map_z])) >> tik->shift;

    x = tik->pdata.negate_x ? -x : x;
    y = tik->pdata.negate_y ? -y : y;
    z = tik->pdata.negate_z ? -z : z;

    //used by compass
    gs_sensor_data[0]= x;
    gs_sensor_data[1]= y;
    gs_sensor_data[2]= z;
    spin_unlock(&tik->s_lock);

    KXTIK_DMSG("acceleration_data: %d, %d, %d", x, y, z);

    input_report_abs(tik->input_dev, ABS_X, x);
    input_report_abs(tik->input_dev, ABS_Y, y);
    input_report_abs(tik->input_dev, ABS_Z, z);
    input_sync(tik->input_dev);
}

static void kxtik_poll_work(struct work_struct *work)
{
    struct kxtik_data *tik = container_of((struct work_struct *)work, struct kxtik_data, poll_work);

    kxtik_report_acceleration_data(tik);
}


static enum hrtimer_restart kxtik_timer_func(struct hrtimer *timer)
{
    struct kxtik_data *tik = container_of(timer, struct kxtik_data, timer);
    int sesc  = tik->poll_interval/1000;
    int nsesc = (tik->poll_interval % 1000) * 1000000;

    queue_work(tik->workqueue, &tik->poll_work);
    hrtimer_start(&tik->timer, ktime_set(sesc, nsesc), HRTIMER_MODE_REL);
    return HRTIMER_NORESTART;
}

static int kxtik_update_g_range(struct kxtik_data *tik, u8 new_g_range)
{
    switch (new_g_range) {
        case KXTIK_G_2G:
            tik->shift = 4;
            break;
        case KXTIK_G_4G:
            tik->shift = 3;
            break;
        case KXTIK_G_8G:
            tik->shift = 2;
            break;
        default:
            return -EINVAL;
    }

    tik->ctrl_reg1 &= 0xe7;
    tik->ctrl_reg1 |= new_g_range;

    return 0;
}

static int kxtik_update_odr(struct kxtik_data *tik, unsigned int poll_interval)
{
    int err = -1;
    int i = 0;
    u8 u8CtrlReg = 0;

    /* Use the lowest ODR that can support the requested poll interval */
    for (i = 0; i < ARRAY_SIZE(kxtik_odr_table); i++) {
        tik->data_ctrl = kxtik_odr_table[i].mask;
        if (poll_interval < kxtik_odr_table[i].cutoff)
            break;
    }

    err = i2c_smbus_write_byte_data(tik->client, KX_CTRL_REG1, 0);
    if (err < 0)
        return err;

    err = i2c_smbus_write_byte_data(tik->client, KX_DATA_CTRL, tik->data_ctrl);
    if (err < 0)
        return err;

    /* lock */
    spin_lock(&tik->s_lock);
    u8CtrlReg = tik->ctrl_reg1;
    spin_unlock(&tik->s_lock);

    err = i2c_smbus_write_byte_data(tik->client, KX_CTRL_REG1, u8CtrlReg);
    if (err < 0)
        return err;

    return 0;
}

static int kxtik_device_power_on(struct kxtik_data *tik)
{
    int err = -1;
    u8 u8CtrlRegOn = 0;

    if (tik->pdata.power_on)
        return tik->pdata.power_on();

    /* ensure that PC1 is cleared before updating control registers */
    err = i2c_smbus_write_byte_data(tik->client, KX_CTRL_REG1, 0);
    if (err < 0)
        return err;

    /* turn on outputs */
    spin_lock(&tik->s_lock);
    tik->ctrl_reg1 |= KX_PC1_ON;
    u8CtrlRegOn = tik->ctrl_reg1;
    spin_unlock(&tik->s_lock);
    err = i2c_smbus_write_byte_data(tik->client, KX_CTRL_REG1, u8CtrlRegOn);
    if (err < 0)
        return err;

    return 0;
}

static int kxtik_device_power_off(struct kxtik_data *tik)
{
    int err = -1;
    u8 u8CtrlRegOf = 0;

    if (tik->pdata.power_off)
        return tik->pdata.power_off();

    spin_lock(&tik->s_lock);
    tik->ctrl_reg1 &= KX_PC1_OFF;
    u8CtrlRegOf = tik->ctrl_reg1;
    spin_unlock(&tik->s_lock);

    err = i2c_smbus_write_byte_data(tik->client, KX_CTRL_REG1, u8CtrlRegOf);
    if (err < 0)
        dev_err(&tik->client->dev, "soft power off failed\n");

    return 0;
}

static int kxtik_enable(struct kxtik_data *tik)
{
    int err = -1;

    err = kxtik_device_power_on(tik);
    if (err < 0)
        return err;

    hrtimer_start(&tik->timer, ktime_set(0, 10000000), HRTIMER_MODE_REL);

    return 0;
}

static void kxtik_disable(struct kxtik_data *tik)
{
    hrtimer_cancel(&tik->timer);
    cancel_work_sync(&tik->poll_work);
    flush_workqueue(tik->workqueue);
    kxtik_device_power_off(tik);
}


static int kxtik_input_open(struct input_dev *input)
{
    struct kxtik_data *tik = input_get_drvdata(input);

    return kxtik_enable(tik);
}

static void kxtik_input_close(struct input_dev *dev)
{
    struct kxtik_data *tik = input_get_drvdata(dev);

    kxtik_disable(tik);
}


static void __devinit kxtik_init_input_device(struct kxtik_data *tik,
                          struct input_dev *input_dev)
{
    __set_bit(EV_ABS, input_dev->evbit);
    input_set_abs_params(input_dev, ABS_X, -KX_G_MAX, KX_G_MAX, KX_FUZZ, KX_FLAT);
    input_set_abs_params(input_dev, ABS_Y, -KX_G_MAX, KX_G_MAX, KX_FUZZ, KX_FLAT);
    input_set_abs_params(input_dev, ABS_Z, -KX_G_MAX, KX_G_MAX, KX_FUZZ, KX_FLAT);

    input_dev->name = ACCL_INPUT_DEV_NAME;
    input_dev->id.bustype = BUS_I2C;
    input_dev->dev.parent = &tik->client->dev;
}

static int __devinit kxtik_setup_input_device(struct kxtik_data *tik)
{
    struct input_dev *input_dev = NULL;
    int err = -1;

    input_dev = input_allocate_device();
    if (!input_dev) {
        dev_err(&tik->client->dev, "input device allocate failed\n");
        return -ENOMEM;
    }

    tik->input_dev = input_dev;

    input_dev->open  = NULL;//kxtik_input_open;
    input_dev->close = NULL;//kxtik_input_close;
    input_set_drvdata(input_dev, tik);

    kxtik_init_input_device(tik, input_dev);

    err = input_register_device(tik->input_dev);
    if (err) {
        dev_err(&tik->client->dev,
            "unable to register input polled device %s: %d\n",
            tik->input_dev->name, err);
        input_free_device(tik->input_dev);
        return err;
    }

    return 0;
}


/*
 * When IRQ mode is selected, we need to provide an interface to allow the user
 * to change the output data rate of the part.  For consistency, we are using
 * the set_poll method, which accepts a poll interval in milliseconds, and then
 * calls update_odr() while passing this value as an argument.  In IRQ mode, the
 * data outputs will not be read AT the requested poll interval, rather, the
 * lowest ODR that can support the requested interval.  The client application
 * will be responsible for retrieving data from the input node at the desired
 * interval.
 */

/* Returns currently selected poll interval (in ms) */
static ssize_t kxtik_get_poll(struct device *dev,
                struct device_attribute *attr, char *buf)
{
    struct kxtik_data *tik = dev_get_drvdata(dev);

    return sprintf(buf, "%d\n", tik->poll_interval);
}

/* Allow users to select a new poll interval (in ms) */
static ssize_t kxtik_set_poll(struct device *dev, struct device_attribute *attr,
                        const char *buf, size_t count)
{
    struct kxtik_data *tik = dev_get_drvdata(dev);
    unsigned int interval = 0;

    interval = (unsigned int)simple_strtoul(buf, NULL, 10);

    spin_lock(&tik->s_lock);
    /* Set current interval to the greater of the minimum interval or
     * the requested interval*/
    tik->poll_interval = max(interval, tik->pdata.min_interval);
    interval = tik->poll_interval;
    spin_unlock(&tik->s_lock);

    kxtik_update_odr(tik, interval);
    return count;
}

/* Allow users to enable/disable the device */
static ssize_t kxtik_set_enable(struct device *dev, struct device_attribute *attr,
                        const char *buf, size_t count)
{
    unsigned int enable = 0;
    struct kxtik_data *tik = dev_get_drvdata(dev);

    enable = (unsigned int)simple_strtoul(buf, NULL, 10);

    if(enable)
        kxtik_enable(tik);
    else
        kxtik_disable(tik);

    return count;
}

static ssize_t attr_get_accl_data(struct device *dev, struct device_attribute *attr, 
                                      char *buf)
{
    struct kxtik_data *tik = dev_get_drvdata(dev);

    spin_lock(&tik->s_lock);
    *((int16_t *)&buf[0]) = gs_sensor_data[0];
    *((int16_t *)&buf[2]) = gs_sensor_data[1];
    *((int16_t *)&buf[4]) = gs_sensor_data[2];
    spin_unlock(&tik->s_lock);

    return ACCL_DATA_SIZE;
}


/*used for coordinate change*/
static int sicoordinate_acc = -1;
static ssize_t coordinate_acc_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    return sprintf(buf, "coordinate_acc =  %d\n",sicoordinate_acc);
}

static ssize_t coordinate_acc_store(struct device *dev, struct device_attribute *attr,
                const char *buf, size_t size)
{
    unsigned long status = 0;
    struct kxtik_data *tik = dev_get_drvdata(dev);

    if (strict_strtoul(buf, 10, &status))
        return -EINVAL;
    if ((status != 0) && (status != 1)) {
        printk(KERN_ERR "[%s %d] Must be 1 or 0\n", __func__, __LINE__);
    } else {
        sicoordinate_acc = status;
        spin_lock(&tik->s_lock);
        if (sicoordinate_acc == 1) {
            tik->pdata.axis_map_x = 1;
            tik->pdata.axis_map_y = 0;
            tik->pdata.negate_x   = 1;
        } else if (sicoordinate_acc == 0) {
            tik->pdata.axis_map_x = 0;
            tik->pdata.axis_map_y = 1;
            tik->pdata.negate_x   = 0;
        }
        spin_unlock(&tik->s_lock);
    }

    return size;
}


static DEVICE_ATTR(pollrate_ms, S_IRUGO|S_IWUSR, kxtik_get_poll, kxtik_set_poll);
static DEVICE_ATTR(enable, S_IRUGO|S_IWUSR, NULL, kxtik_set_enable);
static DEVICE_ATTR(accl_data, S_IRUGO, attr_get_accl_data, NULL);
static DEVICE_ATTR(coordinate_acc, S_IRUGO|S_IWUSR, coordinate_acc_show, coordinate_acc_store);


static struct attribute *kxtik_attributes[] = {
    &dev_attr_pollrate_ms.attr,
    &dev_attr_enable.attr,
    &dev_attr_accl_data.attr,
    &dev_attr_coordinate_acc.attr,
    NULL
};

static struct attribute_group kxtik_attribute_group = {
    .attrs = kxtik_attributes
};

static int kxtik_verify(struct kxtik_data *tik)
{
    int retval = -1;

    retval = kxtik_device_power_on(tik);
    if (retval < 0)
        return retval;

    retval = i2c_smbus_read_byte_data(tik->client, KX_WHO_AM_I);
    if (retval < 0) {
        dev_err(&tik->client->dev, "read err int source\n");
        goto out;
    }

    if (retval == 0x05) {
        retval = 0;
    }
    else {
        retval = -EIO;
    }

out:
    kxtik_device_power_off(tik);
    return retval;
}

static int kxtik_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    const struct kxtik_platform_data *pdata = client->dev.platform_data;
    struct kxtik_data *tik = NULL;
    int ret = -1;

    if (!pdata) {
        dev_err(&client->dev, "platform data is NULL; exiting\n");
        ret = -ENODEV;
        goto err_pdataNULL_failed;
    }

    ret = i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_BYTE_DATA);
    if (!ret) {
        printk(KERN_ERR "gs_probe: need I2C_FUNC_I2C\n");
        ret = -ENODEV;
        goto err_check_functionality_failed;
    }

    tik = kzalloc(sizeof(struct kxtik_data), GFP_KERNEL);
    if (NULL==tik) {
        dev_err(&client->dev,
            "failed to allocate memory for module data\n");
        ret = -ENOMEM;
        goto err_kzalloc_failed;
    }

    tik->client = client;
    tik->pdata = *pdata;

    if (pdata->init) { //init no used.
        ret = pdata->init();
        if (ret < 0)
            goto err_free_mem;
    }

    ret = kxtik_verify(tik);
    if (ret < 0) {
        dev_err(&client->dev, "device not recognized\n");
        goto err_recognize_exit;
    }

    i2c_set_clientdata(client, tik);

    tik->ctrl_reg1 = tik->pdata.res_12bit | tik->pdata.g_range;
    tik->poll_interval = tik->pdata.poll_interval;

    ret = kxtik_update_g_range(tik, tik->pdata.g_range);
    if (ret < 0){
        dev_err(&client->dev, "update_g_range error\n");
        goto err_update_g_range;
    }

    ret = kxtik_update_odr(tik, tik->poll_interval);
    if (ret < 0){
        dev_err(&client->dev, "update_odr error\n");
        goto err_update_odr;
    }  

    tik->workqueue =  create_workqueue("KXTIK_Workqueue");
    INIT_WORK(&tik->poll_work, kxtik_poll_work);

    hrtimer_init(&tik->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
    tik->timer.function = kxtik_timer_func;

    ret = kxtik_setup_input_device(tik);
    if (ret)
        goto err_pdata_exit;

    ret = sysfs_create_group(&client->dev.kobj, &kxtik_attribute_group);
    if (ret) {
        dev_err(&client->dev, "sysfs create failed: %d\n", ret);
        goto err_destroy_input;
    }

      dev_info(&client->dev, "kxtik_probe success: Start KXTIK\n");

    #ifdef CONFIG_HAS_EARLYSUSPEND
        tik->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
        tik->early_suspend.suspend = kxtik_early_suspend;
        tik->early_suspend.resume = kxtik_late_resume;
        register_early_suspend(&tik->early_suspend);
    #endif

#ifdef CONFIG_S10_SENSORS_INPUT_INFO
    ret = set_sensor_input(ACC, tik->input_dev->dev.kobj.name);
    if (ret) {
        dev_err(&client->dev, "%s set_sensor_input failed\n", __func__);
        goto err_destroy_set_sensor_input;
    }
#endif

    spin_lock_init(&tik->s_lock); 

    return 0;

err_destroy_set_sensor_input:
    unregister_early_suspend(&tik->early_suspend);
    sysfs_remove_group(&client->dev.kobj, &kxtik_attribute_group);
err_destroy_input:
    input_unregister_device(tik->input_dev);
err_pdata_exit:
    hrtimer_cancel(&tik->timer);
    destroy_workqueue(tik->workqueue);
err_update_odr:
err_update_g_range:
err_recognize_exit:    
    if (tik->pdata.exit)
        tik->pdata.exit();
err_free_mem:
    kfree(tik);
    tik = NULL;
err_kzalloc_failed:
err_check_functionality_failed:
err_pdataNULL_failed:
    return ret;
}


static int kxtik_remove(struct i2c_client *client)
{
    struct kxtik_data *tik = i2c_get_clientdata(client);

    hrtimer_cancel(&tik->timer);  
    cancel_work_sync(&tik->poll_work);
    flush_workqueue(tik->workqueue);
    destroy_workqueue(tik->workqueue);

    unregister_early_suspend(&tik->early_suspend);
    input_unregister_device(tik->input_dev);
    if (tik->pdata.exit)
        tik->pdata.exit();

    kfree(tik);
    return 0;
}


static int kxtik_suspend(struct i2c_client *client, pm_message_t mesg)
{
    struct kxtik_data *tik = i2c_get_clientdata(client);
    kxtik_disable(tik);
    return 0;
}

static int kxtik_resume(struct i2c_client *client)
{
    struct kxtik_data *tik = i2c_get_clientdata(client);
    kxtik_enable(tik);
    return 0;
}


#ifdef CONFIG_HAS_EARLYSUSPEND
static void kxtik_early_suspend(struct early_suspend *h)
{
    struct kxtik_data *tik = NULL;
    tik = container_of(h, struct kxtik_data, early_suspend);
    kxtik_suspend(tik->client, PMSG_SUSPEND);
}

static void kxtik_late_resume(struct early_suspend *h)
{
    struct kxtik_data *tik = NULL;
    tik = container_of(h, struct kxtik_data, early_suspend);
    kxtik_resume(tik->client);
}
#endif

static const struct i2c_device_id kxtik_id[] = {
    { "kxtik", 0 },
    { },
};

MODULE_DEVICE_TABLE(i2c, kxtik_id);

static struct i2c_driver kxtik_driver = {
    .driver = {
        .name     = "kxtik",
        .owner    = THIS_MODULE,
#ifndef CONFIG_HAS_EARLYSUSPEND
        .suspend  = tik_suspend,
        .resume   = tik_resume,
#endif
    },
    .probe       = kxtik_probe,
    .remove      = __devexit_p(kxtik_remove),
    .id_table    = kxtik_id,
};

static int __init kxtik_init(void)
{
    return i2c_add_driver(&kxtik_driver);
}

static void __exit kxtik_exit(void)
{
    i2c_del_driver(&kxtik_driver);
}

module_init(kxtik_init);
module_exit(kxtik_exit);

MODULE_DESCRIPTION("accessor  Driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("polling");
