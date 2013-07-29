/*
 * Copyright 2010 S10 Tech. Co., Ltd.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Code Aurora nor
 *       the names of its contributors may be used to endorse or promote
 *       products derived from this software without specific prior written
 *       permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NON-INFRINGEMENT ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
/*
 * k3 8903 charger driver
 * author: wufan w00163571 2011-12-22
 * modified: liang l00180803 2012-01-05
 */

#include <linux/spinlock.h>
#include <linux/errno.h>
#include <linux/module.h>
#include <linux/list.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/bitops.h>
#include <linux/workqueue.h>
#include <linux/gpio.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <mach/irqs.h>
#include <linux/slab.h>
#include <linux/clk.h>
#include <linux/irqnr.h>
#include <mach/early-debug.h>
#include <linux/err.h>
#include <mach/gpio.h>

#include <linux/power/k3_psy_monitor.h>
#include <linux/power/k3_max8903_charger.h>

#include <linux/notifier.h>

struct max8903_dev_data
{
    int gpio_dok;
	int gpio_en;
	int gpio_ctrl0;
	int gpio_ctrl1;
	int charging_fault_irq;

    struct k3_charger_dev* chg_dev;
};

static struct max8903_dev_data *dev_id;
extern struct blocking_notifier_head notifier_list_psy;

static int max8903_charging_switch(void* charger_data, int charging_switch_flag)
{
	int ret = 0;
//	struct ms8903_dev_data *dev_id = (struct ms8903_dev_data *)charger_data;
	ret = gpio_request(dev_id->gpio_en, "8903_charging_enable");
	if(ret)
	{
		MAX8903_DBG("MAX8903 request enable gpio failed!\n");
		return ret;
	}

    MAX8903_DBG("%s switch:%d\n", __func__, charging_switch_flag);
	if(charging_switch_flag)
	{
		gpio_direction_output(dev_id->gpio_en, 0);
	}
	else
	{
		gpio_direction_output(dev_id->gpio_en, 1);
	}
	gpio_free(dev_id->gpio_en);
	return ret;
}

static int max8903_get_dok_value(void)
{
    int ret = 0;

    ret = gpio_request(dev_id->gpio_dok, "8903_charging_DOK");
	if(ret)
	{
		MAX8903_DBG("MAX8903 request dok gpio failed!\n");
		return ret;
	}
    ret = gpio_get_value(dev_id->gpio_dok);

    gpio_free(dev_id->gpio_dok);
    return ret;
}

static irqreturn_t max8903_dcin_handler(int irq, void *dev_id_t)
{
    //printk(KERN_INFO "max8903_dcin_handler running!");
	long int event = 0;
	bool ac_in;
    struct max8903_dev_data *dev_data = dev_id_t;

	ac_in = gpio_get_value(dev_data->gpio_dok) ? false : true;
    //printk(KERN_EMERG "LEON:max8903_dcin_handler!!!!!!!!!!,ac_in=%d\n", ac_in);

    if(!ac_in){
        event = CHG_AC_REMOVED_EVENT;
    }
    else{
        event = CHG_AC_INSERT_EVENT;
    }
    blocking_notifier_call_chain(&notifier_list_psy, event, NULL);

	return IRQ_HANDLED;

}

static int max8903_dok_irq_request(void)
{
    int ret = 0;
    //printk(KERN_EMERG "LEON:max8903_dok_irq_request!!!!!!!!!!\n");

    ret = request_threaded_irq(gpio_to_irq(dev_id->gpio_dok),
         NULL, max8903_dcin_handler,
         IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING,
         "MAX8903_DCIN", dev_id);
    if (ret) {
        printk(KERN_ERR "Cannot request irq %d for DC (%d)\n",
                gpio_to_irq(dev_id->gpio_dok), ret);
    }

    return ret;
}

static void max8903_dok_irq_free(void)
{

    //printk(KERN_EMERG "LEON:max8903_dok_irq_free!!!!!!!!!!\n");

    free_irq(gpio_to_irq(dev_id->gpio_dok), dev_id);

}

static int max8903_charging_control(void* charger_data, int charging_current_level)
{
	int ret = 0;
//	struct ms8903_dev_data *dev_id = (struct ms8903_dev_data *)charger_data;
	ret = gpio_request(dev_id->gpio_ctrl0, "8903_charging_ctrl0");
	if(ret)
	{
		MAX8903_ERR("MS8903 request enable gpio failed!\n");
		return ret;
	}
	ret = gpio_request(dev_id->gpio_ctrl1, "8903_charging_ctrl1");
	if(ret)
	{
		gpio_free(dev_id->gpio_ctrl0);
		MAX8903_ERR("MS8903 request enable gpio failed!\n");
		return ret;
	}
    MAX8903_DBG("%s current_level:%d\n", __func__, charging_current_level);

	switch(charging_current_level)
	{
		case CHARGING_CURRENT_LEVEL0:
        ret = gpio_direction_output(dev_id->gpio_ctrl0, 0);
        if (ret < 0){
            printk(KERN_ERR "%s, gpio_direction_output %d with ret %d.\n", __func__, dev_id->gpio_ctrl0, ret);
            return ret;
        }

		ret = gpio_direction_output(dev_id->gpio_ctrl1, 0);
        if (ret < 0){
            printk(KERN_ERR "%s, gpio_direction_output %d with ret %d.\n", __func__, dev_id->gpio_ctrl1, ret);
            return ret;
        }
		break;
		case CHARGING_CURRENT_LEVEL1:
        ret = gpio_direction_output(dev_id->gpio_ctrl0, 0);
        if (ret < 0){
            printk(KERN_ERR "%s, gpio_direction_output %d with ret %d.\n", __func__, dev_id->gpio_ctrl0, ret);
            return ret;
        }

		ret = gpio_direction_output(dev_id->gpio_ctrl1, 1);
        if (ret < 0){
            printk(KERN_ERR "%s, gpio_direction_output %d with ret %d.\n", __func__, dev_id->gpio_ctrl1, ret);
            return ret;
        }
        break;
		case CHARGING_CURRENT_LEVEL2:
		ret = gpio_direction_output(dev_id->gpio_ctrl0, 1);
        if (ret < 0){
            printk(KERN_ERR "%s, gpio_direction_output %d with ret %d.\n", __func__, dev_id->gpio_ctrl0, ret);
            return ret;
        }
		ret = gpio_direction_output(dev_id->gpio_ctrl1, 1);
        if (ret < 0){
            printk(KERN_ERR "%s, gpio_direction_output %d with ret %d.\n", __func__, dev_id->gpio_ctrl1, ret);
            return ret;
        }
		break;
		default:
        ret = gpio_direction_output(dev_id->gpio_ctrl0, 0);
        if (ret < 0){
            printk(KERN_ERR "%s, gpio_direction_output %d with ret %d.\n", __func__, dev_id->gpio_ctrl0, ret);
            return ret;
        }
		ret = gpio_direction_output(dev_id->gpio_ctrl1, 0);
        if (ret < 0){
            printk(KERN_ERR "%s, gpio_direction_output %d with ret %d.\n", __func__, dev_id->gpio_ctrl1, ret);
            return ret;
        }
		break;
	}

	gpio_free(dev_id->gpio_ctrl0);
	gpio_free(dev_id->gpio_ctrl1);
	return ret;
}

static irqreturn_t max8903_charger_handler(int irq, void *dev_id)
{
	struct max8903_dev_data *max8903_data = dev_id;
	disable_irq_nosync(max8903_data->charging_fault_irq);
	enable_irq(max8903_data->charging_fault_irq);
	return IRQ_HANDLED;
}

static int __init max8903_charger_probe(struct platform_device *pdev)
{
	int ret                                         = 0;
	struct max8903_dev_data* dev_data               = NULL;
    struct max8903_platform_data * platform_data    = NULL;
    struct k3_charger_dev* charger_8903             = NULL;

	MAX8903_DBG("k3_max8903_charger:ms8903_charger_probe!!\n");

    platform_data = pdev->dev.platform_data;
    if(!platform_data){
        ret = -EINVAL;
        goto fail;
    }

	dev_data = kzalloc(sizeof(struct max8903_dev_data), GFP_KERNEL);
	if(!dev_data){
		ret = -ENOMEM;
        goto fail;
	}

    dev_data->gpio_dok = platform_data->gpio_dok;
	dev_data->gpio_en = platform_data->switch_gpio;
	dev_data->gpio_ctrl0 = platform_data->ctr0_gpio;
	dev_data->gpio_ctrl1 = platform_data->ctr1_gpio;
	dev_data->charging_fault_irq = gpio_to_irq(platform_data->fat_intr_gpio);

    ret = (platform_data->io_mux_block_init)(platform_data);
    if(ret)
        goto io_mux_block_init_fail;

	ret = request_irq(dev_data->charging_fault_irq,
			max8903_charger_handler, IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING , DRIVER_NAME, dev_data);
	if(ret)
	{
		ret =  -ENXIO;
        goto request_irq_fail;
	}

	dev_set_drvdata(&pdev->dev, dev_data);

	//register dev_data to battery report driver
	charger_8903 = kzalloc(sizeof(struct k3_charger_dev), GFP_KERNEL);
	if(!charger_8903){
		dev_err(&pdev->dev, "MAX8903 malloc dev data mem failed!\n");
		kfree(dev_data);
		dev_data = NULL;
		ret = -ENOMEM;
        goto k3_charger_dev_fail;
	}
	strcpy(charger_8903->name, MAIN_CHARGER);
	charger_8903->charging_switch = max8903_charging_switch;
	charger_8903->charging_current_control = max8903_charging_control;
    charger_8903->get_gpio_value = max8903_get_dok_value;
    charger_8903->irq_request = max8903_dok_irq_request;
    charger_8903->irq_free = max8903_dok_irq_free;

    dev_data->chg_dev = charger_8903;
    dev_id = dev_data;

#if 0
    /*********/
    ret = gpio_request(dev_id->gpio_en, "8903_charging_enable");
	if(ret)
	{
		MAX8903_DBG("MAX8903 request enable gpio failed!\n");
		return ret;
	}
    gpio_direction_output(dev_id->gpio_en, 1);
    gpio_free(dev_id->gpio_en);
    /*********/
#endif

	k3_psy_monitor_register_charger(charger_8903);

    MAX8903_DBG("k3_max8903_charger:ms8903_charger_probe:end!!\n");

	return 0;

 k3_charger_dev_fail:
 request_irq_fail:
 io_mux_block_init_fail:
    kfree(dev_data);
 fail:
    return ret;
}

static  int max8903_charger_remove(struct platform_device *pdev)
{
	struct max8903_dev_data* dev_data = NULL;
	dev_data = platform_get_drvdata(pdev);
	platform_set_drvdata(pdev, NULL);

	kfree(dev_id->chg_dev);
    kfree(dev_id);
    dev_id = NULL;

    return 0;
}

#ifdef CONFIG_PM
static int k3_max8903_charger_suspend(struct platform_device *pdev, pm_message_t state)
{
	printk("k3_max8903_charger_suspend +\n");

    struct max8903_platform_data * platform_data    = NULL;
    int ret = 0;

    platform_data = pdev->dev.platform_data;

    gpio_request(platform_data->switch_gpio, "8903_charging_enable");
    gpio_direction_input(platform_data->switch_gpio);
    gpio_free(platform_data->switch_gpio);

    if(platform_data->io_mux_block_exit){

        ret = (platform_data->io_mux_block_exit)(platform_data);
        if(ret < 0){
            dev_err(&pdev->dev, "MAX8903 gpio suspend failed!\n");
        }
    }
    
	printk("k3_max8903_charger_suspend -\n");

	return ret;
}

static int k3_max8903_charger_resume(struct platform_device *pdev)
{
    
	printk("k3_max8903_charger_resume +\n");
    struct max8903_platform_data * platform_data    = NULL;
    int ret = 0;

    platform_data = pdev->dev.platform_data;

    if(platform_data->io_mux_block_init){

        ret = (platform_data->io_mux_block_init)(platform_data);
        if(ret < 0){
            dev_err(&pdev->dev, "MAX8903 gpio resume failed!\n");
        }
    }

    printk("k3_max8903_charger_resume -\n");

	return ret;
}
#else
#define k3_max8903_charger_suspend	NULL
#define k3_max8903_charger_resume	NULL
#endif

static struct platform_driver max8903_charger_driver = {
	.probe = max8903_charger_probe,
	.remove = max8903_charger_remove,
	.suspend	= k3_max8903_charger_suspend,
	.resume		= k3_max8903_charger_resume,
//	.shutdown	= k3_max8903_charger_shutdown,
	.driver		= {
		.name	= DRIVER_NAME,
	},
};

static int __init max8903_charger_init(void)
{
	int ret;
	ret = platform_driver_register(&max8903_charger_driver);
	if (ret != 0)
		printk("Failed to register ms8903_charger_driver driver: %d\n", ret);
	return 0;
}
static void __exit max8903_charger_exit(void)
{
	platform_driver_unregister(&max8903_charger_driver);
}

//Note:Max8903 driver must load before BC1.1 driver
fs_initcall(max8903_charger_init);
module_exit(max8903_charger_exit);

MODULE_LICENSE("GPL");
