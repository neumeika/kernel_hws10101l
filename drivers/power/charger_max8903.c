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
 *  S10 tablet standard power module driver
 *  author: wufan w00163571 2012-08-13
 *
 */

/*=============================================================================
修订历史

问题单号              修改人            日期                           原因
==============================================================================*/

#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/io.h>
#include <linux/gpio.h>
#include <linux/power_supply.h>
#include <linux/platform_device.h>
#include <linux/usb/hiusb_android.h>
#include <linux/power/S10_std_charger.h>
#include <linux/power/S10_std_psy.h>
#include <linux/power/max8903.h>

struct max8903_dev_info
{
    struct max8903_platform_data *pdata;
    bool                          fault;
    bool                          dock_in;
    bool                          usb_in;
    bool                          ac_in;
    struct notifier_block         nb;
};

static struct S10_std_charger_device* charger_max8903 = NULL;

static enum power_supply_property max8903_charger_props[] =
{
    POWER_SUPPLY_PROP_STATUS,/* Charger status output */
    POWER_SUPPLY_PROP_ONLINE,/* External power source */
    POWER_SUPPLY_PROP_TYPE,
    POWER_SUPPLY_PROP_HEALTH,/* Fault or OK */
};

static irqreturn_t max8903_dc_in(int irq, void *data);
int max8903_max_input_current_set (void* dev, int val);

#ifdef RECOGNISE_CHARGER_TYPE_BY_EXTRAL_IC
struct blocking_notifier_head notifier_list_psy;
BLOCKING_NOTIFIER_HEAD(notifier_list_psy);

#define AC_STATUS_DETECT  (1<<0)
#define AC_IRQ_REQUEST    (1<<1)
#define AC_IRQ_FREE       (1<<2)

static int max8903_ac_detect_handle(void* dev, int ac_handle)
{
    struct S10_std_charger_device* charger = (struct S10_std_charger_device*)dev;
    struct max8903_dev_info *dev_info   = charger ? (struct max8903_dev_info *)charger->charger_private_info : NULL;
    struct max8903_platform_data *pdata = dev_info ? dev_info->pdata : NULL;
    int ret = 0;

    power_debug(HW_POWER_CHARGER_IC_DUG, "%s:ac_handle=%d\n", __func__,ac_handle);
    
    if( ac_handle != AC_STATUS_DETECT &&
        ac_handle != AC_IRQ_REQUEST  &&
        ac_handle != AC_IRQ_FREE)
    {
        return -EINVAL; //参数非法，返回
    }

    switch(ac_handle){
    case AC_STATUS_DETECT:
        if (pdata->dok && gpio_is_valid(pdata->dok))
        {
            ret = gpio_get_value(pdata->dok) ? 0 : 1;
        }
        else
        {
            dev_err(dev, "When DC is wired, DOK should be wired."
                    "as well.\n");
            ret = -EIO;
            return ret;
        }
        break;
    case AC_IRQ_REQUEST:
        if (pdata->dc_valid)
		{
			ret = request_threaded_irq(gpio_to_irq(pdata->dok),
									   NULL, max8903_dc_in,
									   IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING,
									   "MAX8903 DC IN", charger);
			if (ret)
			{
				dev_err(dev, "Cannot request irq %d for DC (%d)\n",
						gpio_to_irq(pdata->dok), ret);
                return ret;
			}
		}
        break;
    case AC_IRQ_FREE:
        if (pdata->dc_valid)
        {
            free_irq(gpio_to_irq(pdata->dok), charger);
        }
        break;
    default:
        break;
   }
    return ret;
}

static int max8903_charger_plug_event(struct notifier_block *nb, unsigned long event,
                                      void *_data)
{
    //unsigned long report_event;
    struct S10_std_charger_device* charger = charger_max8903;
    struct max8903_dev_info *dev_info = charger ? (struct max8903_dev_info *)charger->charger_private_info : NULL;
    int dock_charger_online = 0;

    if (IS_ERR_OR_NULL(charger) || IS_ERR_OR_NULL(dev_info))
    {
        return -EINVAL;
    }
    
    power_debug(HW_POWER_CHARGER_IC_DUG, "%s:event=%d\n", __func__,event);
    switch (event)
    {
    case CHG_USB_CDP_PULGIN_EVENT:
        dev_info->usb_in = true;
        charger->charger_online = 1;
        charger->charger_type = POWER_SUPPLY_TYPE_USB_CDP;
        break;
    case CHG_USB_DCP_PULGIN_EVENT:
        dev_info->usb_in = true;
        charger->charger_online = 1;
        charger->charger_type = POWER_SUPPLY_TYPE_USB_DCP;
        break;
    case CHG_USB_SDP_PULGIN_EVENT:
        dev_info->usb_in = true;
        charger->charger_online = 1;
        charger->charger_type = POWER_SUPPLY_TYPE_USB;
        break;
    case CHG_DOCK_PULGIN_EVENT:
        dev_info->dock_in = true;
        //DOCK口状态有变化，检测是否有充电器已插入
        dock_charger_online = max8903_ac_detect_handle(charger, AC_STATUS_DETECT);
        power_debug(HW_POWER_CHARGER_IC_DUG, "%s:dock_charger_online=%d\n", __func__,dock_charger_online);
        if(dock_charger_online)
        {
	        dev_info->ac_in = true;
            charger->charger_online = 1;
            charger->charger_type = POWER_SUPPLY_TYPE_USB_ACA;
        }
        //注册一个DOK中断信号
        max8903_ac_detect_handle(charger, AC_IRQ_REQUEST);
        break;
    case CHG_AC_PULGIN_EVENT:
        dev_info->ac_in = true;
        charger->charger_online = 1;
        charger->charger_type = POWER_SUPPLY_TYPE_USB_ACA;
        break;
    case CHG_CHARGER_PULGOUT_EVENT:
        if(dev_info->dock_in){
            //释放DOK中断信号
            max8903_ac_detect_handle(charger, AC_IRQ_FREE);
            dev_info->dock_in = false;
        }
        dev_info->ac_in   = false;
        dev_info->usb_in  = false;
        charger->charger_online = 0;
        charger->charger_type = POWER_SUPPLY_TYPE_MAINS;
        break;
    case PSY_INFO_CHANGE_EVENT:
        break;
    default:
        break;
    }

#if 0 //注释原因:CEN常开
    if (dev_info->pdata->cen)
    {
        gpio_set_value(dev_info->pdata->cen, dev_info->ac_in || dev_info->usb_in ? 0 : 1);
    }
#endif
    //充电器拨出，设定默认输入电流为500mA
    if(!charger->charger_online){
        charger->charger_type = POWER_SUPPLY_TYPE_USB;
    }
    max8903_max_input_current_set(charger, 0);//设定输入电流

    //通知Monitor监控模块充电器状态的变化
    S10_power_monitor_notifier_call_chain(event, NULL);
    power_debug(HW_POWER_CHARGER_IC_DUG, "%s:End!\n", __func__);
}

#endif

static int max8903_get_charger_health(void)
{
    return POWER_SUPPLY_HEALTH_GOOD;
}

static void max8903_charging_enable(void* dev, int enable_ce)
{
    struct S10_std_charger_device* charger = (struct S10_std_charger_device*)dev;
    struct max8903_dev_info *dev_info   = charger ? (struct max8903_dev_info *)charger->charger_private_info : NULL;
    struct max8903_platform_data *pdata = dev_info ? dev_info->pdata : NULL;

    if (IS_ERR_OR_NULL(charger) || IS_ERR_OR_NULL(dev_info) || IS_ERR_OR_NULL(pdata))
    {
        return;
    }

    if (pdata->cen)
    {
        gpio_set_value(pdata->cen, enable_ce ? 0 : 1);
    }

    //charger->charging_flag = enable_ce ? 1 : 0;
}

int max8903_max_input_current_get(void* dev, int* input_current_limit)
{
    struct S10_std_charger_device* charger = (struct S10_std_charger_device*)dev;
    struct max8903_dev_info *dev_info   = charger ? (struct max8903_dev_info *)charger->charger_private_info : NULL;
    struct max8903_platform_data *pdata = NULL;

    pdata = dev_info ? dev_info->pdata : NULL;

    if (IS_ERR_OR_NULL(charger) || IS_ERR_OR_NULL(dev_info) || IS_ERR_OR_NULL(pdata))
    {
        return -EINVAL;
    }

    switch (charger->charger_type)
    {
    case POWER_SUPPLY_TYPE_USB:
        *input_current_limit = pdata->usb_max_current;
        break;
    case POWER_SUPPLY_TYPE_USB_CDP:
        *input_current_limit = pdata->usb_cdp_max_current;
        break;
    case POWER_SUPPLY_TYPE_USB_DCP:
        *input_current_limit = pdata->usb_dcp_max_current;
        break;
    case POWER_SUPPLY_TYPE_USB_ACA:
        *input_current_limit = pdata->usb_ac_max_current;
        break;
    default:
        *input_current_limit = pdata->usb_max_current;
        break;
    }

    return 0;
}

int max8903_max_input_current_set (void* dev, int val)
{
    struct S10_std_charger_device* charger = (struct S10_std_charger_device*)dev;
    struct max8903_dev_info *dev_info   = charger ? (struct max8903_dev_info *)charger->charger_private_info : NULL;
    struct max8903_platform_data *pdata = NULL;

    pdata = dev_info ? dev_info->pdata : NULL;

    if (IS_ERR_OR_NULL(charger) || IS_ERR_OR_NULL(dev_info) || IS_ERR_OR_NULL(pdata))
    {
        return -EINVAL;
    }

    switch (charger->charger_type)
    {
    case POWER_SUPPLY_TYPE_USB:      // 500ma
        if (pdata->dcm && gpio_is_valid(pdata->dcm))
        {
            gpio_set_value(pdata->dcm, 0);
        }
        else
        {
            if ((pdata->dcm_ctrl0) && (pdata->dcm_ctrl1) && gpio_is_valid(pdata->dcm_ctrl0)
                && gpio_is_valid(pdata->dcm_ctrl1))
            {
                gpio_set_value(pdata->dcm_ctrl0, 0);
                gpio_set_value(pdata->dcm_ctrl1, 0);
            }
        }

        break;
    case POWER_SUPPLY_TYPE_USB_CDP:     // 1500ma
        if (pdata->dcm && gpio_is_valid(pdata->dcm))
        {
            gpio_set_value(pdata->dcm, 0);     // 500ma
        }
        else
        {
            if ((pdata->dcm_ctrl0) && (pdata->dcm_ctrl1) && gpio_is_valid(pdata->dcm_ctrl0)
                && gpio_is_valid(pdata->dcm_ctrl1))
            {
                gpio_set_value(pdata->dcm_ctrl0, 1);
                gpio_set_value(pdata->dcm_ctrl1, 0);
            }
        }

        break;
    case POWER_SUPPLY_TYPE_USB_DCP:      // 2000ma
        if (pdata->dcm && gpio_is_valid(pdata->dcm))
        {
            gpio_set_value(pdata->dcm, 1);
        }
        else
        {
            if ((pdata->dcm_ctrl0) && (pdata->dcm_ctrl1) && gpio_is_valid(pdata->dcm_ctrl0)
                && gpio_is_valid(pdata->dcm_ctrl1))
            {
                gpio_set_value(pdata->dcm_ctrl0, 1);
                gpio_set_value(pdata->dcm_ctrl1, 1);
            }
        }

        break;
    case POWER_SUPPLY_TYPE_USB_ACA:     // 2000ma
        if (pdata->dcm && gpio_is_valid(pdata->dcm))
        {
            gpio_set_value(pdata->dcm, 1);
        }
        else
        {
            if ((pdata->dcm_ctrl0) && (pdata->dcm_ctrl1) && gpio_is_valid(pdata->dcm_ctrl0)
                && gpio_is_valid(pdata->dcm_ctrl1))
            {
                gpio_set_value(pdata->dcm_ctrl0, 1);
                gpio_set_value(pdata->dcm_ctrl1, 1);
            }
        }

        break;
    default:
        if (pdata->dcm && gpio_is_valid(pdata->dcm))
        {
            gpio_set_value(pdata->dcm, 0);
        }
        else
        {
            if ((pdata->dcm_ctrl0) && (pdata->dcm_ctrl1) && gpio_is_valid(pdata->dcm_ctrl0)
                && gpio_is_valid(pdata->dcm_ctrl1))
            {
                gpio_set_value(pdata->dcm_ctrl0, 0);
                gpio_set_value(pdata->dcm_ctrl1, 0);
            }
        }

        break;
    }

    return 0;
}

static int max8903_get_real_property(enum power_supply_property  psp,
                                     union power_supply_propval *val)
{
    struct S10_std_charger_device* charger = charger_max8903;
    struct S10_psy_monitor_dev_info* monitordev_info = NULL;

    switch (psp)
    {
    case POWER_SUPPLY_PROP_STATUS:
        monitordev_info = S10_power_get_monitor_devinfo();
        if (charger->charger_online)
        {
            if ((((POWER_SUPPLY_TYPE_USB == charger->charger_type)
                       || (POWER_SUPPLY_TYPE_USB_CDP == charger->charger_type))
                   && monitordev_info->usb_charging_display_support && monitordev_info->usb_charging_support)
               || (POWER_SUPPLY_TYPE_USB_ACA == charger->charger_type)
               || (POWER_SUPPLY_TYPE_USB_DCP == charger->charger_type))
            {
                val->intval = POWER_SUPPLY_STATUS_CHARGING;
            }
            else
            {
                val->intval = POWER_SUPPLY_STATUS_NOT_CHARGING;
            }
        }
        else
        {
            val->intval = POWER_SUPPLY_STATUS_DISCHARGING;
        }

        break;
    case POWER_SUPPLY_PROP_ONLINE:
        val->intval = charger->charger_online;
        break;
    case POWER_SUPPLY_PROP_HEALTH:
        val->intval = max8903_get_charger_health();
        break;
    case POWER_SUPPLY_PROP_TYPE:
        val->intval = charger->charger_type;
        break;
    default:
        break;
    }
	return 0;
}

static int max8903_get_property(struct power_supply *       psy,
                                enum power_supply_property  psp,
                                union power_supply_propval *val)
{
    struct S10_std_charger_device* charger = charger_max8903;
	int charger_type_ac = 0;

    if (IS_ERR_OR_NULL(charger))
    {
        return -EINVAL;
    }

    switch (psp)
    {
    case POWER_SUPPLY_PROP_STATUS:
        val->intval = charger->charger_report_info.charging_status;
        break;
    case POWER_SUPPLY_PROP_ONLINE:
	    charger_type_ac = ((POWER_SUPPLY_TYPE_USB_ACA == charger->charger_report_info.charger_type)
                           || (POWER_SUPPLY_TYPE_USB_DCP == charger->charger_report_info.charger_type)
                           || (POWER_SUPPLY_TYPE_MAINS == charger->charger_report_info.charger_type));
        if (charger_type_ac)
        {
			val->intval = charger->charger_report_info.charger_online;
			break;
		}
        else
        {
            val->intval = 0;
            break;
        }
    case POWER_SUPPLY_PROP_TYPE:
        val->intval = charger->charger_report_info.charger_type;
        break;
    case POWER_SUPPLY_PROP_HEALTH:
        val->intval = charger->charger_report_info.charger_health;
        break;
    default:
        break;
    }

    return 0;
}

static int max8903_report_get_property_usb(struct power_supply *       psy,
                                    enum power_supply_property  psp,
                                    union power_supply_propval *val)
{
    struct S10_std_charger_device* charger = charger_max8903;
    int charger_type_usb = 0;

    if (IS_ERR_OR_NULL(charger))
    {
        return -EINVAL;
    }

    switch (psp)
    {
    case POWER_SUPPLY_PROP_STATUS:
        val->intval = charger->charger_report_info.charging_status;
        break;
    case POWER_SUPPLY_PROP_ONLINE:
        charger_type_usb = ((POWER_SUPPLY_TYPE_USB == charger->charger_report_info.charger_type)
                            || (POWER_SUPPLY_TYPE_USB_CDP == charger->charger_report_info.charger_type)
                            || (POWER_SUPPLY_TYPE_UPS == charger->charger_report_info.charger_type));

        if (charger_type_usb)
        {
            val->intval = charger->charger_report_info.charger_online;
            break;
        }
        else
        {
            val->intval = 0;
            break;
        }

    case POWER_SUPPLY_PROP_TYPE:
        val->intval = charger->charger_report_info.charger_type;
        break;
    case POWER_SUPPLY_PROP_HEALTH:
        val->intval = charger->charger_report_info.charger_health;
        break;
    default:
        break;
    }

    power_debug(HW_POWER_CHARGER_IC_DUG, "[Power Dug: %s, %s, %d,%d]:  psp = %d \n", __FILE__, __func__, __LINE__, psp,
                val->intval);
    return 0;
}

static irqreturn_t max8903_dc_in(int irq, void *data)
{
    struct S10_std_charger_device* charger = (struct S10_std_charger_device*)data;
    struct max8903_dev_info *dev_info = (struct max8903_dev_info *)charger->charger_private_info;

#ifdef RECOGNISE_CHARGER_TYPE_BY_EXTRAL_IC
    if (dev_info->dock_in)
#endif
    {
        struct max8903_platform_data *pdata = dev_info->pdata;
        unsigned long event;
        bool ac_in;

        ac_in = gpio_get_value(pdata->dok) ? false : true;

        if (ac_in == dev_info->ac_in)
        {
            return IRQ_HANDLED;
        }

        dev_info->ac_in = ac_in;
        if (ac_in)
        {
            event = CHG_AC_PULGIN_EVENT;
            charger->charger_online = 1;
            charger->charger_type = POWER_SUPPLY_TYPE_USB_ACA;
        }
        else
        {
            event = CHG_CHARGER_PULGOUT_EVENT;
            charger->charger_online = 0;
            charger->charger_type = POWER_SUPPLY_TYPE_MAINS;
        }

        /* Do not touch Current-Limit-Mode */
        max8903_max_input_current_set(charger, -1);

        /* Charger Enable / Disable (cen is negated) */
        if (pdata->cen)
        {
            gpio_set_value(pdata->cen, ac_in ? 0 :
                           (dev_info->usb_in ? 0 : 1));
        }

        dev_dbg(charger->dev, "AC Charger %s.\n", ac_in ?
                "Connected" : "Disconnected");

        //notify monitor driver there are plug event
        S10_power_monitor_notifier_call_chain(event, NULL);
    }

    return IRQ_HANDLED;
}

static irqreturn_t max8903_usb_in(int irq, void *data)
{
    struct S10_std_charger_device* charger = (struct S10_std_charger_device*)data;
    struct max8903_dev_info *dev_info = (struct max8903_dev_info *)charger->charger_private_info;

#ifdef RECOGNISE_CHARGER_TYPE_BY_EXTRAL_IC
    if (dev_info->dock_in)
#endif
    {
        struct max8903_platform_data *pdata = dev_info->pdata;
        unsigned long event;
        bool usb_in;

        usb_in = gpio_get_value(pdata->uok) ? false : true;

        if (usb_in == dev_info->usb_in)
        {
            return IRQ_HANDLED;
        }

        dev_info->usb_in = usb_in;

        if (usb_in)
        {
            event = CHG_USB_SDP_PULGIN_EVENT;
            charger->charger_online = 1;
            charger->charger_type = POWER_SUPPLY_TYPE_USB;
        }
        else
        {
            event = CHG_CHARGER_PULGOUT_EVENT;
            charger->charger_online = 0;
            charger->charger_type = POWER_SUPPLY_TYPE_MAINS;
        }

        /* Do not touch Current-Limit-Mode */
        max8903_max_input_current_set(charger, -1);

        /* Charger Enable / Disable (cen is negated) */
#if 0
        if (pdata->cen)
        {
            gpio_set_value(pdata->cen, usb_in ? 0 :
                           (dev_info->ac_in ? 0 : 1));
        }

#endif
        dev_dbg(charger->dev, "USB Charger %s.\n", usb_in ?
                "Connected" : "Disconnected");

        //notify monitor driver plug event was happened 
        S10_power_monitor_notifier_call_chain(event, NULL);
    }

    return IRQ_HANDLED;
}

static irqreturn_t max8903_fault(int irq, void *_data)
{
    //TODO
    return IRQ_HANDLED;
}

int max8903_charger_chip_enable(void *dev, int enable)
{
    return 0;
}

int max8903_max_charging_current_get(void* dev, int* charging_current_limit)
{
    return 0;
}

int max8903_max_charging_current_set(void* dev, int current_limit)
{
    return 0;
}

int max8903_cutoff_current_set(void* dev, int cutoff_current)
{
    return 0;
}

int max8903_cutoff_voltage_set(void* dev, int cutoff_voltage)
{
    return 0;
}

int max8903_firmware_download(void)
{
    return 0;
}

int max8903_plug_notify_call(bool can_schedule, void * dev)
{
    return 0;
}

int max8903_event_notify_call(bool can_schedule, void * dev)
{
    return 0;
}

struct S10_std_charger_ops max8903_private_ops =
{
    .charging_enable                   = max8903_charging_enable,
    .charger_chip_enable               = max8903_charger_chip_enable,
    .charger_get_real_property         = max8903_get_real_property,
    .charging_max_charging_current_get = max8903_max_charging_current_get,
    .charging_max_charging_current_set = max8903_max_charging_current_set,
    .charging_max_input_current_get    = max8903_max_input_current_get,
    .charging_max_input_current_set    = max8903_max_input_current_set,
    .charging_cutoff_current_set       = max8903_cutoff_current_set,
    .charging_cutoff_voltage_set       = max8903_cutoff_voltage_set,
    .charger_firmware_download         = max8903_firmware_download,
    .charging_plug_notify_call         = max8903_plug_notify_call,
    .charging_event_notify_call        = max8903_event_notify_call,
};

static __devinit int max8903_charger_probe(struct platform_device *pdev)
{
    struct S10_std_charger_device* charger;
    struct max8903_dev_info* dev_info;
    struct device *dev = &pdev->dev;
    struct max8903_platform_data *pdata = pdev->dev.platform_data;
    int ret = 0;
	struct S10_psy_monitor_dev_info* monitordev_info = NULL;
//#ifdef RECOGNISE_CHARGER_TYPE_BY_EXTRAL_IC
//    struct notifier_block nb;
//#endif

    printk("%s\n", __func__);
    charger = kzalloc(sizeof(struct S10_std_charger_device), GFP_KERNEL);
    if (charger == NULL)
    {
        dev_err(dev, "Cannot allocate memory.\n");
        return -ENOMEM;
    }

    //分配充电器私有数据空间
    dev_info = kzalloc(sizeof(struct max8903_dev_info), GFP_KERNEL);
    if (dev_info == NULL)
    {
        dev_err(dev, "Cannot allocate memory.\n");
        return -ENOMEM;
    }

    dev_info->pdata = pdata;
    charger->dev = dev;
    charger->charger_private_info = (void*)dev_info;
    
    platform_set_drvdata(pdev, dev_info);

    //初始化充电相关的IO，供后面的代码使用
    if (pdata->io_mux_block_init(pdata))
    {
        ret = -EINVAL;
        goto err_iomux_init;
    }

    //if (pdata->gpio_block_init(pdata))
    //{
     //   ret = -EINVAL;
     //   goto err_gpio_init;
   // }

    //判断MAX8903芯片的各个管脚是否可用
    //DC与USB充电器须可用
    if ((pdata->dc_valid == false) && (pdata->usb_valid == false))
    {
        dev_err(dev, "No valid power sources.\n");
        ret = -EINVAL;
        goto err_config_charger;
    }

    //使能MAX8903，后续不再操作CEN管脚(MAX8903芯片有bug，使系统的电压跌落0.8v左右导致死机)
    //printk("LEON>>1\n");
    if (pdata->cen)
    {
        if (gpio_is_valid(pdata->cen))
        {
	
    	    //printk("LEON>>2\n");
            gpio_set_value(pdata->cen, 0);
        }
        else
        {
            dev_err(dev, "Invalid pin: cen.\n");
            ret = -EINVAL;
            goto err_config_charger;
        }
    }
    
    if (pdata->chg) {
          if (!gpio_is_valid(pdata->chg)) {
                  dev_err(dev, "Invalid pin: chg.\n");
                        ret = -EINVAL;
                        goto err_config_charger;
                }
        }
    
    if (pdata->flt)
    {
        //充电异常处理IO，必须可使用
        if (!gpio_is_valid(pdata->flt))
        {
            dev_err(dev, "Invalid pin: flt.\n");
            ret = -EINVAL;
            goto err_config_charger;
        }
        
        //printk("LEON>>3\n");
        ret = request_threaded_irq(gpio_to_irq(pdata->flt),
                                   NULL, max8903_fault,
                                   IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING,
                                   "MAX8903 Fault", charger);
        if (ret)
        {
            dev_err(dev, "Cannot request irq %d for Fault (%d)\n",
                    gpio_to_irq(pdata->flt), ret);
            goto err_config_charger;
        }
    }

    if (pdata->usus)
    {
        if (!gpio_is_valid(pdata->usus))
        {
            dev_err(dev, "Invalid pin: usus.\n");

            //	ret = -EINVAL;
            //	goto err_config_charger;
        }
    }

    //检查充电器是否已在位，如果已在位，则设置相应充电电流
    //此处不检测，fsa880驱动后于此驱动加载且fsa880会主动检测充电器是否在位
    //如果在位，则发送notifier消息来通知Max8903驱动
#ifdef RECOGNISE_CHARGER_TYPE_BY_EXTRAL_IC
    dev_info->nb.notifier_call = max8903_charger_plug_event;
    ret = blocking_notifier_chain_register(&notifier_list_psy, &(dev_info->nb));
    if (ret)
    {
        goto err_notifier_register;
    }
#endif

    
    //填充S10_std_charger_device结构并向监控模块注册
    strcpy(charger->name, AC_CHARGER);
    
    charger->charger_report_get_property = max8903_get_property;
    charger->charging_done_flag = 1; //充电是否截止标志
    charger->psy_type = POWER_SUPPLY_TYPE_MAINS;/* 注册type为AC的电量上报节点 */
    //此处不再判断充电器在位与否，也不再更新power_supply设备通知上层    
    #if 0
    charger->charger_online = dev_info.ac_in || dev_info.usb_in ? 1 : 0;
    if (dev_info.ac_in)
    {
        charger->charger_type = POWER_SUPPLY_TYPE_USB;
    }
    else if (dev_info.usb_in)
    {
        charger->charger_type = POWER_SUPPLY_TYPE_USB_ACA;
    }
    else
    {
        charger->charger_type = POWER_SUPPLY_TYPE_MAINS;
    }
    #endif

    charger->S10_charger_support_props = max8903_charger_props;
    
    charger->num_properties = ARRAY_SIZE(max8903_charger_props);
   
    charger->ops = &max8903_private_ops;
    charger_max8903 = charger;

	monitordev_info = S10_power_get_monitor_devinfo();
    if (monitordev_info->usb_charging_support)
    {
        charger->charger_report_get_property_usb = max8903_report_get_property_usb;
    }
    else
    {
        charger->charger_report_get_property_usb = NULL;
    }

    ret = charger2monitor_register(charger);
    if (ret)
    {
        power_debug(HW_POWER_CHARGER_IC_ERR, "[Power ERR: %s, %s, %d]\n", __FILE__, __func__, __LINE__);
        goto err_register2monitor;
    }

    return 0;

err_register2monitor:
    charger_max8903 = NULL;
#ifdef RECOGNISE_CHARGER_TYPE_BY_EXTRAL_IC
    blocking_notifier_chain_unregister(&notifier_list_psy, &(dev_info->nb));
err_notifier_register:
    if (pdata->flt)
    {
        free_irq(gpio_to_irq(pdata->flt), charger);
    }
#endif

err_config_charger:
  //  pdata->gpio_block_exit(pdata);
err_gpio_init:
    pdata->io_mux_block_exit(pdata);
err_iomux_init:
    kfree(charger);
    kfree(dev_info);
    return ret;
}

static __devexit int max8903_charger_remove(struct platform_device *pdev)
{
    struct S10_std_charger_device* charger = charger_max8903;
    struct max8903_dev_info *dev_info = platform_get_drvdata(pdev);

    if (charger)
    {
        struct max8903_platform_data *pdata = dev_info->pdata;

#ifdef RECOGNISE_CHARGER_TYPE_BY_EXTRAL_IC
        //struct notifier_block nb;
        dev_info->nb.notifier_call = max8903_charger_plug_event;
        blocking_notifier_chain_unregister(&notifier_list_psy, &dev_info->nb);
#endif

        if (pdata->flt)
        {
            free_irq(gpio_to_irq(pdata->flt), charger);
        }

        pdata->gpio_block_exit(pdata);
        pdata->io_mux_block_exit(pdata);
        kfree(charger);
        kfree(dev_info);
    }

    return 0;
}

static struct platform_driver max8903_charger_driver =
{
    .probe     = max8903_charger_probe,
    .remove    = __devexit_p(max8903_charger_remove),
    .driver    = {
        .name  = MAX8903_NAME,
        .owner = THIS_MODULE,
    },
};

static int __init max8903_charger_init(void)
{
    return platform_driver_register(&max8903_charger_driver);
}

//Note:Max8903 driver must load before BC1.1 driver
fs_initcall(max8903_charger_init);

static void __exit max8903_charger_exit(void)
{
    platform_driver_unregister(&max8903_charger_driver);
}

module_exit(max8903_charger_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("MAX8903 Charger Driver");
MODULE_AUTHOR("S10 Inc");
MODULE_ALIAS("max8903-charger");
