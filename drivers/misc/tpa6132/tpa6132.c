#include <linux/init.h>
#include <linux/module.h>
#include <linux/version.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/notifier.h>
#include <linux/mutex.h>
#include <linux/proc_fs.h>
#include <linux/kthread.h>
#include <linux/clk.h>
#include <linux/gpio.h>
#include <linux/mux.h>
#include <linux/slab.h>
#include <linux/miscdevice.h>

#include <linux/platform_device.h>

#include "tpa6132.h"

#define LOG_TAG "TPA6132"

#define PRINT_INFO  1
#define PRINT_WARN  1
#define PRINT_DEBUG 1
#define PRINT_ERR   1

#if PRINT_INFO
#define logi(fmt, ...) printk("[" LOG_TAG "][I]" fmt "\n", ##__VA_ARGS__)
#else
#define logi(fmt, ...)
#endif

#if PRINT_WARN
#define logw(fmt, ...) printk("[" LOG_TAG "][W]" fmt "\n", ##__VA_ARGS__)
#else
#define logw(fmt, ...)
#endif

#if PRINT_DEBUG
#define logd(fmt, ...) printk("[" LOG_TAG "][D]" fmt "\n", ##__VA_ARGS__)
#else
#define logd(fmt, ...)
#endif

#if PRINT_ERR
#define loge(fmt, ...) printk("[" LOG_TAG "][E]" fmt "\n", ##__VA_ARGS__)
#else
#define loge(fmt, ...)
#endif

#define IOMUX_BLOCK_NAME "block_audio_eph"

static struct tpa6132_platform_data *pdata = NULL;
static struct mutex tpa6132_lock;
static struct iomux_block *tpa6132_iomux_block = NULL;
static struct block_config *tpa6132_block_config = NULL;
static unsigned int gTpa6132EnFlag = 0;
static struct kobject *kobj = NULL;

static int tpa6132_open(struct inode *inode, struct file *file)
{
    return 0;
}

static int tpa6132_release(struct inode *inode, struct file *file)
{
    return 0;
}

static long tpa6132_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
    int ret = 0;

    mutex_lock(&tpa6132_lock);
    switch (cmd) {
    case TPA6132_ENABLE:
        ret = blockmux_set(tpa6132_iomux_block, tpa6132_block_config, NORMAL);
        if (0 > ret) {
            loge("%s: set iomux to gpio normal error", __FUNCTION__);
            goto err_exit;
        }
        gpio_set_value(pdata->gpio_tpa6132_en, 1);
        gTpa6132EnFlag = 1;
        break;
    case TPA6132_DISABLE:
        gpio_set_value(pdata->gpio_tpa6132_en, 0);
        gTpa6132EnFlag = 0;
        ret = blockmux_set(tpa6132_iomux_block, tpa6132_block_config, LOWPOWER);
        if (0 > ret) {
            loge("%s: set iomux to gpio lowpower error", __FUNCTION__);
            goto err_exit;
        }
        break;
    default:
        loge("%s: invalid command %d", __FUNCTION__, _IOC_NR(cmd));
        ret = -EINVAL;
        break;
    }

err_exit:
    mutex_unlock(&tpa6132_lock);
    return ret;
}

static const struct file_operations tpa6132_fops = {
    .owner          = THIS_MODULE,
    .open           = tpa6132_open,
    .release        = tpa6132_release,
    .unlocked_ioctl = tpa6132_ioctl,
};

static struct miscdevice tpa6132_device = {
    .minor  = MISC_DYNAMIC_MINOR,
    .name   = TPA6132_NAME,
    .fops   = &tpa6132_fops,
};

static ssize_t tpa6132_attr_show(struct kobject *kobj, struct kobj_attribute *attr,
        char *buf)
{
    return sprintf(buf, "%d", gTpa6132EnFlag);
}

static ssize_t tpa6132_attr_store(struct kobject *kobj, struct kobj_attribute *attr,
			 const char *buf, size_t count)
{
    int iret = 0;

    if(NULL != buf) {
        iret = sscanf(buf,"%d", &gTpa6132EnFlag);
    }

    if(-1 == iret) {
        printk(KERN_ERR "sscanf gtpa6132EnFlag error\n");
        return iret;
    } else {
		gpio_set_value(pdata->gpio_tpa6132_en, gTpa6132EnFlag);

//        return iret;
		return count;
    }
}

static struct kobj_attribute tpa6132_attribute =
    __ATTR(switch, 0664, tpa6132_attr_show, tpa6132_attr_store);

static struct attribute* tpa6132_attributes[] =
{
    &tpa6132_attribute.attr,
    NULL
};

static struct attribute_group tpa6132_defattr_group =
{
    .attrs = tpa6132_attributes,
};

static int __devinit tpa6132_probe(struct platform_device *pdev)
{
    int ret = -ENODEV;

    logi("%s", __FUNCTION__);

    pdata = pdev->dev.platform_data;
    if (NULL == pdata) {
        loge("%s: platform data is NULL", __FUNCTION__);
        return -EINVAL;
    }

    tpa6132_iomux_block = iomux_get_block(IOMUX_BLOCK_NAME);
    if (!tpa6132_iomux_block) {
        loge("%s: get iomux block error", __FUNCTION__);
        return -ENODEV;
    }

    tpa6132_block_config = iomux_get_blockconfig(IOMUX_BLOCK_NAME);
    if (!tpa6132_block_config) {
        loge("%s: get block config error", __FUNCTION__);
        return -ENODEV;
    }

    /* request gpio */
    ret = gpio_request(pdata->gpio_tpa6132_en, TPA6132_NAME);
    if (0 > ret) {
        loge("%s: gpio request enable pin failed", __FUNCTION__);
        return ret;
    }

    /* set gpio output & set value low */
    ret = gpio_direction_output(pdata->gpio_tpa6132_en, 0);
    if (0 > ret) {
        loge("%s: set gpio direction failed", __FUNCTION__);
        gpio_free(pdata->gpio_tpa6132_en);
        return ret;
    }

    ret = misc_register(&tpa6132_device);
    if (ret) {
        loge("%s: tpa6132_device register failed", __FUNCTION__);
        gpio_free(pdata->gpio_tpa6132_en);
        return ret;
    }


    kobj = kobject_create_and_add("tpa6132_en", NULL);
    if (kobj == NULL)
    {
        return -EINVAL;
    }
    if (sysfs_create_group(kobj, &tpa6132_defattr_group))
    {
        kobject_put(kobj);
    }

    return ret;
}

static int __devexit tpa6132_remove(struct platform_device *pdev)
{
    logi("%s", __FUNCTION__);
    gpio_free(pdata->gpio_tpa6132_en);
	sysfs_remove_group(kobj, &tpa6132_defattr_group);
    kobject_put(kobj);
    return 0;
}

static struct platform_driver tpa6132_driver = {
    .driver = {
        .name  = TPA6132_NAME,
        .owner = THIS_MODULE,
    },
    .probe  = tpa6132_probe,
    .remove = __devexit_p(tpa6132_remove),
};

static int __init tpa6132_init(void)
{
    logi("%s", __FUNCTION__);
    mutex_init(&tpa6132_lock);
    return platform_driver_register(&tpa6132_driver);
}

static void __exit tpa6132_exit(void)
{
    logi("%s", __FUNCTION__);
    platform_driver_unregister(&tpa6132_driver);
}

module_init(tpa6132_init);
module_exit(tpa6132_exit);

MODULE_DESCRIPTION("tpa6132 driver");
MODULE_LICENSE("GPL");
