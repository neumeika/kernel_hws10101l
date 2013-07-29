/* drivers/misc/vibrator_k3.c
 *
 * Copyright (C) 2008 Google, Inc.
 * Author: y36721
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
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/hrtimer.h>
#include <linux/err.h>
#include <linux/irq.h>
#include <linux/spinlock.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/workqueue.h>

#include "timed_output.h"
#include <linux/vibrator/k3_vibrator.h>
#include <linux/io.h>
#include <linux/ioport.h>
#include <linux/clk.h>

struct k3_vibrator_data {
	struct timed_output_dev dev;
	struct k3_vibrator_platform_data *plat_data;
	struct hrtimer timer;
	struct mutex lock;
	struct clk *clk;
	void __iomem *k3_vibrator_base;
	int value;
	u8 freq;
	u8 power;
	u8 mode;
};

struct k3_vibrator_data *k3_vibrator_pdata;

static void k3_vibrator_onoff_handler(struct work_struct *data);

static struct workqueue_struct *done_queue;

static int g_iset_val = 0;
static unsigned long long g_pre_set_time = 0;

static DECLARE_WORK(done_work, k3_vibrator_onoff_handler);

static void k3_vibrator_reg_write(u8 vibrator_set, u32 vibrator_address)
{
	writel(vibrator_set, k3_vibrator_pdata->k3_vibrator_base + vibrator_address);
}

static void k3_vibrator_set_cfg(struct k3_vibrator_data *pdata, int value)
{
	if (value <= TIMEOUT_MORE) {
		pdata->freq = pdata->plat_data->high_freq;
		pdata->power = pdata->plat_data->high_power;
	} else {
		pdata->freq = pdata->plat_data->low_freq;
		pdata->power = pdata->plat_data->low_power;
	}

	return ;
}

static void k3_vibrator_onoff(int on)
{
	struct k3_vibrator_data *pdata = k3_vibrator_pdata;
	pdata->value = on;
	queue_work(done_queue, &done_work);
}

#include <linux/hkadc/hiadc_hal.h>
#define K3_VPH_PWR_ADC_CH   ADC_VBATMON
static int k3_vibrator_get_vphpwr_vol(unsigned int *voltage)
{
    unsigned char reserve = 0;
    int value = 0;
    int ret = 0;

    /* open temperature channel */
    ret = k3_adc_open_channel(K3_VPH_PWR_ADC_CH);
    if (ret < 0)
    {
        printk(KERN_ERR "%s:open adc channel failed(ret=%d channel_no=%d)\n", __func__, ret, ADC_VBATMON);
        return ret;
    }
    mdelay(1);
    /* get the temperature voltage value */
    value = k3_adc_get_value(K3_VPH_PWR_ADC_CH, &reserve);
    /* release the resource */
    ret = k3_adc_close_channal(K3_VPH_PWR_ADC_CH);
    if (ret < 0)
    {
        printk(KERN_ERR "%s:close adc channel failed(ret=%d channel_no=%d)\n", __func__, ret, ADC_VBATMON);
        return ret;
    }
    *voltage = value;
//    printk(KERN_ERR "%s:vph-pwr voltage = %d\n", __func__, *voltage);
    return 0;

}


static int k3_get_vchg_state(void)
{
    return 0;   // don't care charger online case, now just read vph pwr voltage
}

struct k3_vph_pwr_vol_vib_iset {
	int vph_voltage;
    int vreg_volue;
};

#define ARRAY_SIZE(arr) (sizeof(arr) / sizeof((arr)[0]))
struct k3_vph_pwr_vol_vib_iset k3_vibrator_table_vbat[] =
{
    {3300, 0xfc},
    {3350, 0xfa},
    {3400, 0xf8},
    {3450, 0xf6},
    {3500, 0xf4},
    {3550, 0xf3},
    {3600, 0xf1},
    {3650, 0xf0},
    {3700, 0xee},
    {3750, 0xed},
    {3800, 0xeb},
    {3850, 0xea},
    {3900, 0xe8},
    {3950, 0xe7},
    {4000, 0xe6},
    {4050, 0xe5},
    {4100, 0xe3},
    {4150, 0xe2},
    {4200, 0xe1},
};

struct k3_vph_pwr_vol_vib_iset k3_vibrator_table_vchg[] =
{
    {3300, 0xf4},
    {3350, 0xf4},
    {3400, 0xf4},
    {3450, 0xf3},
    {3500, 0xf1},
    {3550, 0xf0},
    {3600, 0xee},
    {3650, 0xec},
    {3700, 0xeb},
    {3750, 0xea},
    {3800, 0xe8},
    {3850, 0xe7},
    {3900, 0xe6},
    {3950, 0xe4},
    {4000, 0xe3},
    {4050, 0xe2},
    {4100, 0xe1},
    {4150, 0xe0},
    {4200, 0xe0},
};

static int k3_vibrator_get_iset_value(void)
{
    int iset_val = k3_vibrator_pdata->power;
    int chg_state = k3_get_vchg_state();
    int vph_pwr_vol = 0;
    int array_len = 0;
    int count = 0;
    struct k3_vph_pwr_vol_vib_iset *pArray = NULL;
    if(k3_vibrator_get_vphpwr_vol(&vph_pwr_vol))
    {
        printk(KERN_ERR "%s:get vph-pwr failed, iset_val = %d\n", __func__, iset_val);
        return iset_val;
    }

    if(chg_state)
    {
        pArray = k3_vibrator_table_vchg;
    }
    else
    {
        pArray = k3_vibrator_table_vbat;
    }

    array_len = ARRAY_SIZE(k3_vibrator_table_vchg);
    if(vph_pwr_vol < pArray->vph_voltage)
    {
        iset_val = pArray->vreg_volue;
    }
    else if(vph_pwr_vol > (pArray + array_len - 1)->vph_voltage)
    {
        iset_val = (pArray + array_len - 1)->vreg_volue;
    }
    else
    {
        count = (vph_pwr_vol - pArray->vph_voltage)/((pArray+1)->vph_voltage - pArray->vph_voltage);
        if(count < (array_len - 1))
            iset_val = (pArray + count)->vreg_volue;
        else
            iset_val = (pArray + array_len - 1)->vreg_volue;
    }
//    printk(KERN_ERR "%s: count = %d, iset_val = 0x%02x\n", __func__, count, iset_val);
    return iset_val;
}

static void k3_vibrator_onoff_handler(struct work_struct *data)
{
	struct k3_vibrator_data *pdata = k3_vibrator_pdata;
	int on = pdata->value;
	int ret = 0;
	mutex_lock(&pdata->lock);
	ret = clk_enable(pdata->clk);
	if (ret) {
		pr_err("pmu clock enable failed,ret:%d\n", ret);
		mutex_unlock(&pdata->lock);
		return ;
	}

	if (on) {
        //int iset_val = pdata->power;
        //iset_val = k3_vibrator_get_iset_value();
		//k3_vibrator_reg_write(iset_val, DR2_ISET);
        //k3_vibrator_reg_write(pdata->power, DR2_ISET);
		k3_vibrator_reg_write(pdata->freq | pdata->mode, DR2_CTRL);
	} else {
		k3_vibrator_reg_write(DR2_DISABLE, DR2_CTRL);
	}
	clk_disable(pdata->clk);
	mutex_unlock(&pdata->lock);
	return ;
}


static enum hrtimer_restart k3_vibrator_timer_func(struct hrtimer *timer)
{
	k3_vibrator_onoff(0);

	return HRTIMER_NORESTART;
}

static int k3_vibrator_get_time(struct timed_output_dev *dev)
{
	struct k3_vibrator_data *pdata =
			container_of(dev, struct k3_vibrator_data, dev);
	if (hrtimer_active(&pdata->timer)) {
		ktime_t r = hrtimer_get_remaining(&pdata->timer);
		struct timeval t = ktime_to_timeval(r);
		return t.tv_sec * 1000 + t.tv_usec / 1000;
	} else
		return 0;
}

static void k3_vibrator_enable(struct timed_output_dev *dev, int value)
{
	struct k3_vibrator_data *pdata = container_of(dev, struct k3_vibrator_data, dev);

	if (value < 0) {
		pr_err("error:vibrator_enable value:%d is negative\n", value);
		return;
	}
	/* cancel previous timer */
	if (hrtimer_active(&pdata->timer))
		hrtimer_cancel(&pdata->timer);

	if (value > 0) {
		static int set_count = 0;

		if(time_after(jiffies,g_pre_set_time+60*HZ))
		{
			g_pre_set_time = jiffies;
			set_count = 0;
		}
		if(set_count == 0) 
		{
	        g_iset_val = k3_vibrator_get_iset_value();
			k3_vibrator_reg_write(g_iset_val, DR2_ISET);
		}
		
		set_count = (set_count+1)%50;
		
		if (value < TIMEOUT_MIN)
			value = TIMEOUT_MIN;
		k3_vibrator_set_cfg(pdata, value);
		k3_vibrator_onoff(1);
		hrtimer_start(&pdata->timer,
			ktime_set(value / 1000, (value % 1000) * 1000000),
			HRTIMER_MODE_REL);
	} else {
		k3_vibrator_onoff(0);
	}
}
static int k3_vibrator_probe(struct platform_device *pdev)
{
	struct k3_vibrator_data *p_data;
	struct resource *res;
	int ret = 0;

	p_data = kzalloc(sizeof(struct k3_vibrator_data), GFP_KERNEL);
	if (p_data == NULL) {
		dev_err(&pdev->dev, "failed to allocate vibrator_device\n");
		return -ENOMEM;
	}

	/*get the clock with con_id = "clk_pmuspi"*/
	p_data->clk = clk_get(NULL, "clk_pmuspi");
	if (p_data->clk == NULL) {
		dev_err(&pdev->dev, "pmu clock get failed\n");
		ret = -ENODEV;
		goto err;
	}

	/* get base_addres */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res == NULL) {
		dev_err(&pdev->dev, "failed to find registers\n");
		ret = -ENXIO;
		goto err_clk;
	}
	p_data->k3_vibrator_base = ioremap(res->start, resource_size(res));
	if (p_data->k3_vibrator_base == 0) {
		dev_err(&pdev->dev, "failed to map registers\n");
		ret = -ENXIO;
		goto err_clk;
	}

	/* init timer */
	hrtimer_init(&p_data->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	p_data->timer.function = k3_vibrator_timer_func;

	/* init lock */
	mutex_init(&p_data->lock);

	p_data->plat_data = pdev->dev.platform_data;

	if (NULL == p_data->plat_data) {
		ret = -ENOMEM;
		dev_err(&pdev->dev, "failed get platform_data.\n");
		goto err_remap;
	}

	p_data->mode = p_data->plat_data->mode;
	p_data->freq = p_data->plat_data->low_freq;
	p_data->power = p_data->plat_data->low_power;

	p_data->dev.name = K3_VIBRATOR;
	p_data->dev.get_time = k3_vibrator_get_time;
	p_data->dev.enable = k3_vibrator_enable;

	ret = timed_output_dev_register(&p_data->dev);
	if (ret < 0) {
		dev_err(&pdev->dev, "failed to regist dev\n");
		goto err_remap;
	}
	
	platform_set_drvdata(pdev, p_data);

	k3_vibrator_pdata = p_data;

	/* create a single thread workquene */
	done_queue = create_workqueue("done_queue");
	if (!done_queue) {
		dev_err(&pdev->dev, "failed to creat workqueue\n");
		ret = -ENOMEM;
		goto err_regis;
	}

	//add by wufan w00163571 2012-04-01 for  resolved vibrator freq call adc function
	g_iset_val = p_data->power;
	g_pre_set_time = jiffies ;
	g_iset_val = k3_vibrator_get_iset_value();
	/* Bugfix:there are risk if not enable clk */
	ret = clk_enable(p_data->clk);
	if (ret) {
		pr_err("pmu clock enable failed,ret:%d\n", ret);
		goto err_regis;
	}

	k3_vibrator_reg_write(g_iset_val, DR2_ISET);
	clk_disable(p_data->clk);
	
	return 0;

err_regis:
	timed_output_dev_unregister(&p_data->dev);
err_remap:
	iounmap(p_data->k3_vibrator_base);
err_clk:
	clk_put(p_data->clk);
	p_data->clk = NULL;
err:
	kfree(p_data);
	p_data = NULL;
	return ret;
}

static int k3_vibrator_remove(struct platform_device *pdev)
{
	struct k3_vibrator_data *pdata = platform_get_drvdata(pdev);
	int ret;

	if (pdata == NULL) {
		dev_err(&pdev->dev, "%s:pdata is NULL\n", __func__);
		return -ENODEV;
	}

	if (hrtimer_active(&pdata->timer))
		hrtimer_cancel(&pdata->timer);

	/* Bugfix:there are risk if not enable clk */
	ret = clk_enable(pdata->clk);
	if (ret) {
		dev_err(&pdev->dev, "pmu clock enable failed,ret:%d\n", ret);
	} else {
		k3_vibrator_reg_write(DR2_DISABLE, DR2_CTRL);
		clk_disable(pdata->clk);
	}

	timed_output_dev_unregister(&pdata->dev);

	clk_put(pdata->clk);
	iounmap(pdata->k3_vibrator_base);
	kfree(pdata);
	pdata = NULL;
	destroy_workqueue(done_queue);
	platform_set_drvdata(pdev, NULL);
	return 0;
}

static void k3_vibrator_shutdown(struct platform_device *pdev)
{
	int ret = 0;
	struct k3_vibrator_data *pdata = platform_get_drvdata(pdev);
	if (pdata == NULL) {
		dev_err(&pdev->dev, "%s:pdata is NULL\n", __func__);
		return;
	}
	printk("[%s] +\n", __func__);

	if (hrtimer_active(&pdata->timer))
		hrtimer_cancel(&pdata->timer);


	ret = clk_enable(pdata->clk);
	if (ret) {
		dev_err(&pdev->dev, "pmu clock enable failed,ret:%d\n", ret);
	} else {
		k3_vibrator_reg_write(DR2_DISABLE, DR2_CTRL);
		clk_disable(pdata->clk);
	}

	clk_put(pdata->clk);

	printk("[%s] -\n", __func__);
	return ;
}

#ifdef CONFIG_PM
static int  k3_vibrator_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct k3_vibrator_data *pdata = platform_get_drvdata(pdev);
	int ret;
	if (pdata == NULL) {
		dev_err(&pdev->dev, "%s:pdata is NULL\n", __func__);
		return -ENODEV;
	}
	printk("[%s] +\n", __func__);
	if (!mutex_trylock(&pdata->lock)) {
		dev_err(&pdev->dev, "%s: mutex_trylock.\n", __func__);
		return -EAGAIN;
	}
	if (hrtimer_active(&pdata->timer)) {
		hrtimer_cancel(&pdata->timer);
	}
	ret = clk_enable(pdata->clk);
	if (ret) {
		pr_err("pmu clock enable failed,ret:%d\n", ret);
		mutex_unlock(&pdata->lock);
		return ret;
	}
	k3_vibrator_reg_write(DR2_DISABLE, DR2_CTRL);
	clk_disable(pdata->clk);
	printk("[%s] -\n", __func__);
	return 0;
}

static int k3_vibrator_resume(struct platform_device *pdev)
{
	struct k3_vibrator_data *pdata = platform_get_drvdata(pdev);
	if (pdata == NULL) {
		dev_err(&pdev->dev, "%s:pdata is NULL\n", __func__);
		return -ENODEV;
	}
	printk("[%s] +\n", __func__);
	mutex_unlock(&pdata->lock);
	printk("[%s] -\n", __func__);
	return 0;
}
#endif

static struct platform_driver k3_vibrator_driver = {
	.probe  = k3_vibrator_probe,
	.remove = k3_vibrator_remove,
	.shutdown	= k3_vibrator_shutdown,
#ifdef CONFIG_PM
	.suspend	= k3_vibrator_suspend,
	.resume		= k3_vibrator_resume,
#endif
	.driver = {
		.name   = K3_VIBRATOR,
		.owner  = THIS_MODULE,
	},
};

static int __init k3_vibrator_init(void)
{
	return platform_driver_register(&k3_vibrator_driver);
}

static void __exit k3_vibrator_exit(void)
{
	platform_driver_unregister(&k3_vibrator_driver);
}

module_init(k3_vibrator_init);
module_exit(k3_vibrator_exit);

MODULE_AUTHOR("skf57909");
MODULE_DESCRIPTION(" k3 vibrator driver");
MODULE_LICENSE("GPL");

