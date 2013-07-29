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

#include "spk_5vboost.h"

#define LOG_TAG "SPK_5VBOOST"

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

#define IOMUX_BLOCK_NAME "block_audio_spk"

static struct mutex spk_5vboost_lock;
static struct spk_5vboost_platform_data *pdata = NULL;

static struct iomux_block *spk_5vboost_iomux_block = NULL;
static struct block_config *spk_5vboost_block_config = NULL;

static int spk_5vboost_open(struct inode *inode, struct file *file)
{
    return 0;
}

static int spk_5vboost_release(struct inode *inode, struct file *file)
{
    return 0;
}

static long spk_5vboost_ioctl(struct file *file,
                            unsigned int cmd,
                            unsigned long arg)
{
    int ret = 0;

    mutex_lock(&spk_5vboost_lock);
    switch (cmd) {
    case SPK_5VBOOST_ENABLE:
        ret = blockmux_set(spk_5vboost_iomux_block, spk_5vboost_block_config, NORMAL);
        if (0 > ret) {
            loge("%s: set iomux to gpio normal error", __FUNCTION__);
            goto err_exit;
        }
        gpio_set_value(pdata->gpio_5vboost_en, 1);
        break;
    case SPK_5VBOOST_DISABLE:
        gpio_set_value(pdata->gpio_5vboost_en, 0);
        ret = blockmux_set(spk_5vboost_iomux_block, spk_5vboost_block_config, LOWPOWER);
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
    mutex_unlock(&spk_5vboost_lock);
    return ret;
}

static const struct file_operations spk_5vboost_fops = {
    .owner          = THIS_MODULE,
    .open           = spk_5vboost_open,
    .release        = spk_5vboost_release,
    .unlocked_ioctl = spk_5vboost_ioctl,
};

static struct miscdevice spk_5vboost_device = {
    .minor  = MISC_DYNAMIC_MINOR,
    .name   = SPK_5VBOOST_NAME,
    .fops   = &spk_5vboost_fops,
};

static int __devinit spk_5vboost_probe(struct platform_device *pdev)
{
    int ret = -ENODEV;

    logi("%s", __FUNCTION__);

    pdata = pdev->dev.platform_data;
    if (NULL == pdata) {
        loge("%s: platform data is NULL", __FUNCTION__);
        return -EINVAL;
    }

    spk_5vboost_iomux_block = iomux_get_block(IOMUX_BLOCK_NAME);
    if (!spk_5vboost_iomux_block) {
        loge("%s: get iomux block error", __FUNCTION__);
        return -ENODEV;
    }

    spk_5vboost_block_config = iomux_get_blockconfig(IOMUX_BLOCK_NAME);
    if (!spk_5vboost_block_config) {
        loge("%s: get block config error", __FUNCTION__);
        return -ENODEV;
    }

    /* request gpio */
    ret = gpio_request(pdata->gpio_5vboost_en, SPK_5VBOOST_NAME);
    if (0 > ret) {
        loge("%s: gpio request enable pin failed", __FUNCTION__);
        return ret;
    }

    /* set gpio output & set value low */
    ret = gpio_direction_output(pdata->gpio_5vboost_en, 0);
    if (0 > ret) {
        loge("%s: set gpio direction failed", __FUNCTION__);
        gpio_free(pdata->gpio_5vboost_en);
        return ret;
    }

    ret = misc_register(&spk_5vboost_device);
    if (ret) {
        loge("%s: spk_5vboost_device register failed", __FUNCTION__);
        gpio_free(pdata->gpio_5vboost_en);
        return ret;
    }

    return ret;
}

static int __devexit spk_5vboost_remove(struct platform_device *pdev)
{
    logi("%s", __FUNCTION__);
    gpio_free(pdata->gpio_5vboost_en);
    return 0;
}

static struct platform_driver spk_5vboost_driver = {
    .driver = {
        .name  = SPK_5VBOOST_NAME,
        .owner = THIS_MODULE,
    },
    .probe  = spk_5vboost_probe,
    .remove = __devexit_p(spk_5vboost_remove),
};

static int __init spk_5vboost_init(void)
{
    logi("%s", __FUNCTION__);
    mutex_init(&spk_5vboost_lock);
    return platform_driver_register(&spk_5vboost_driver);
}

static void __exit spk_5vboost_exit(void)
{
    logi("%s", __FUNCTION__);
    platform_driver_unregister(&spk_5vboost_driver);
}

module_init(spk_5vboost_init);
module_exit(spk_5vboost_exit);

MODULE_DESCRIPTION("spk_5vboost driver");
MODULE_LICENSE("GPL");
