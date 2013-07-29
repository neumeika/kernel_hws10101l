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
 *  
 */
 /*=============================================================================
==============================================================================*/
#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/power_supply.h>
#include <linux/notifier.h>
#include <linux/err.h>
#include <linux/power/S10_std_psy.h>

struct platform_device *monitor_report_dev = NULL;

int power_debug_mask = HW_POWER_BATTERY_IC_ERR | HW_POWER_CHARGER_IC_ERR |
                       HW_POWER_BATTERY_DATA_COLLECT_ERR |
                       HW_POWER_CHARGER_CONTROL_ERR | HW_POWER_PSYINFO_REPORT_ERR
                       | HW_POWER_TESTDEV_ERR | HW_POWER_COUL_REG_DUMP | HW_POWER_CHARGER_REG_DUMP;

/*| HW_POWER_BATTERY_IC_DUG
 | HW_POWER_CHARGER_IC_DUG | HW_POWER_BATTERY_CAPACITY_SMOOTH_DUG
 | HW_POWER_PSYINFO_REPORT_DUG
 | HW_POWER_TESTDEV_DUG;*/

module_param_named(power_debug_mask, power_debug_mask, int, S_IRUGO | S_IWUSR | S_IWGRP);

static const char *S10_psy_supplies_to[] = {
    "MainBattery",
    "ExtraBattery",
    "CoinBattery",
};

#if 0
// 预留定义项
static const char *S10_psy_status_str[] = {
    "unknown",
    "charging",
    "discharging",
    "not charging",
    "full",
};

static const char *S10_psy_charger_phase_str[] = {
    "unknown",
    "none",
    "trickle",
    "fast",
};
#endif

static const char *S10_psy_health_str[] = {
    "unknown",
    "good",
    "overheat",
    "dead",
    "over voltage",
    "unspec failure",
    "cold",
};

#if 0
// 预留定义项
static const char *S10_psy_battery_tech_str[] = {
    "unknown",
    "NiMH",
    "LION",
    "LIPO",
    "LiFe",
    "NiCd",
    "LiMn",
};

static const char *S10_psy_capacity_level_str[] = {
    "unknown",
    "critical",
    "low",
    "normal",
    "high",
    "full",
};
#endif

static int S10_psy_battery_infomation_collect(void);
static int S10_psy_charger_information_collect(void);
static int S10_psy_battery_dect_info_get(struct S10_std_battery_device* battery, struct battery_psy_info* info);
static int S10_psy_battery_infomation_collect(void);
static int S10_psy_charger_information_collect(void);
static int S10_psy_charger_real_info_get(struct charger_psy_info* info);
static int S10_charger_psy_report(void);

struct S10_psy_monitor_dev_info* S10_power_get_monitor_devinfo(void)
{
    if (!monitor_report_dev)
    {
        return NULL;
    }

    return monitor_report_dev->dev.platform_data;
}

EXPORT_SYMBOL_GPL(S10_power_get_monitor_devinfo);

struct S10_psy_reporter* S10_power_get_monitor_reporter(void)
{
    if (!monitor_report_dev)
    {
        return NULL;
    }

    return platform_get_drvdata(monitor_report_dev);
}

EXPORT_SYMBOL_GPL(S10_power_get_monitor_reporter);

static struct platform_device * S10_power_get_monitor_device(void)
{
    return monitor_report_dev;
}

int S10_power_lock_init(enum S10_power_lock_type lock_type, struct S10_power_lock* lock)
{
    if ((lock_type < 0) || (lock_type >= S10_POWER_LOCK_UNDEF))
    {
        return -EINVAL;
    }

    lock->lock_type = lock_type;
    switch (lock_type)
    {
    case S10_POWER_LOCK_LIGHT:
        spin_lock_init(&lock->lock.light_lock);
        break;
    case S10_POWER_LOCK_HEAVY:
        mutex_init(&lock->lock.heavy_lock);
        break;
    case S10_POWER_LOCK_SEMP:
        sema_init(&lock->lock.semp_lock, 1);
        break;
    default:
        break;
    }

    return 0;
}

EXPORT_SYMBOL_GPL(S10_power_lock_init);

void S10_power_lock_deinit(struct S10_power_lock* lock)
{
    switch (lock->lock_type)
    {
    case S10_POWER_LOCK_LIGHT:
        break;
    case S10_POWER_LOCK_HEAVY:
        mutex_destroy(&lock->lock.heavy_lock);
        break;
    case S10_POWER_LOCK_SEMP:
        break;
    default:
        break;
    }

    return;
}

EXPORT_SYMBOL_GPL(S10_power_lock_deinit);

int S10_power_lock_lock(struct S10_power_lock* lock)
{
    switch (lock->lock_type)
    {
    case S10_POWER_LOCK_LIGHT:
        spin_lock_irqsave(&lock->lock.light_lock, lock->irq_flags);
        break;
    case S10_POWER_LOCK_HEAVY:
        mutex_lock(&lock->lock.heavy_lock);
        break;
    case S10_POWER_LOCK_SEMP:
        up(&lock->lock.semp_lock);
        break;
    default:
        break;
    }

    return 0;
}

EXPORT_SYMBOL_GPL(S10_power_lock_lock);

int S10_power_lock_unlock(struct S10_power_lock* lock)
{
    switch (lock->lock_type)
    {
    case S10_POWER_LOCK_LIGHT:
        spin_unlock_irqrestore(&lock->lock.light_lock, lock->irq_flags);
        break;
    case S10_POWER_LOCK_HEAVY:
        mutex_unlock(&lock->lock.heavy_lock);
        break;
    case S10_POWER_LOCK_SEMP:
        down(&lock->lock.semp_lock);
        break;
    default:
        break;
    }

    return 0;
}

EXPORT_SYMBOL_GPL(S10_power_lock_unlock);

struct S10_std_battery_device * S10_power_get_battery_in_list(const char *name)
{
    struct S10_psy_reporter* reporter;
    struct S10_std_battery_device * battery;

    reporter = S10_power_get_monitor_reporter();
    if (IS_ERR_OR_NULL(reporter) || !name)
    {
        return ERR_PTR(-EINVAL);
    }

    S10_power_lock_lock(&reporter->battery_list_lock);
    if (!list_empty(&reporter->battery_list))
    {
        list_for_each_entry(battery, &reporter->battery_list, node)
        {
            if (!strcmp(name, battery->battery_name))
            {
                S10_power_lock_unlock(&reporter->battery_list_lock);
                return battery;
            }
        }
    }

    S10_power_lock_unlock(&reporter->battery_list_lock);
    return ERR_PTR(-ENOENT);
}

EXPORT_SYMBOL_GPL(S10_power_get_battery_in_list);

//通过名字从powersupply_list中找到对应的psy结构
static struct S10_psy_node * get_psy_in_list(const char *name)
{
    struct S10_psy_reporter* reporter;
    struct S10_psy_node* hwpsy;

    reporter = S10_power_get_monitor_reporter();
    if (IS_ERR_OR_NULL(reporter) || !name)
    {
        return ERR_PTR(-EINVAL);
    }

    S10_power_lock_lock(&reporter->psy_list_lock);
    if (!list_empty(&reporter->powersupply_list))
    {
        list_for_each_entry(hwpsy, &reporter->powersupply_list, node)
        {
            if (!strcmp(name, hwpsy->psy.name))
            {
                S10_power_lock_unlock(&reporter->psy_list_lock);
                return hwpsy;
            }
        }
    }

    S10_power_lock_unlock(&reporter->psy_list_lock);
    return ERR_PTR(-ENOENT);
}

#if 0
// 预留接口，备用
static int del_psy_from_list(char *name)
{
    int ret = -EINVAL;
    struct S10_psy_node* hw_psy = NULL;
    struct S10_psy_reporter* reporter = S10_power_get_monitor_reporter();

    if (IS_ERR_OR_NULL(reporter) || !name)
    {
        return -EINVAL;
    }

    hw_psy = get_psy_in_list(name);
    if (IS_ERR_OR_NULL(hw_psy))
    {
        return ret;
    }

    S10_power_lock_lock(&reporter->psy_list_lock);
    list_del(&hw_psy->node);
    S10_power_lock_unlock(&reporter->psy_list_lock);
    return 0;
}
#endif

int charger2monitor_register(struct S10_std_charger_device* charger)
{
    struct S10_psy_monitor_dev_info* monitordev_info = NULL;
    struct S10_psy_reporter* reporter = NULL;
    struct platform_device * pdev = S10_power_get_monitor_device();
    int ret = 0;

    monitordev_info = S10_power_get_monitor_devinfo();
    reporter = S10_power_get_monitor_reporter();
    if (IS_ERR_OR_NULL(monitordev_info) || IS_ERR_OR_NULL(reporter))
    {
        ret = -EINVAL;
        goto err_getinfo;
    }

    reporter->stdcharger = charger;
    reporter->stdcharger->std_psy_ac = kzalloc(sizeof(struct power_supply), GFP_KERNEL);

    if (IS_ERR_OR_NULL(reporter->stdcharger->std_psy_ac))
    {
        ret = -EINVAL;
        goto err_malloc;
    }

    reporter->stdcharger->std_psy_ac->name = charger->name;
    reporter->stdcharger->std_psy_ac->supplied_to = (char**)S10_psy_supplies_to;
    reporter->stdcharger->std_psy_ac->num_supplicants = ARRAY_SIZE(S10_psy_supplies_to);
    reporter->stdcharger->std_psy_ac->type = charger->psy_type;
    reporter->stdcharger->std_psy_ac->properties = charger->S10_charger_support_props;
    reporter->stdcharger->std_psy_ac->num_properties = charger->num_properties;
    reporter->stdcharger->std_psy_ac->get_property = charger->charger_report_get_property;

    //first detect and report
    ret = S10_psy_charger_real_info_get(&charger->charger_real_info);

    S10_power_lock_lock(&charger->data_lock);
    memcpy(&charger->charger_report_info, &charger->charger_real_info, sizeof(struct charger_psy_info));
    S10_power_lock_unlock(&charger->data_lock);

    ret = power_supply_register(&pdev->dev, reporter->stdcharger->std_psy_ac);
    if (ret)
    {
        goto err_psy_ac_register;
    }

    /* 增加USB上报节点 */
    if (monitordev_info->usb_charging_support)
    {
        reporter->stdcharger->std_psy_usb = kzalloc(sizeof(struct power_supply), GFP_KERNEL);
        if (IS_ERR_OR_NULL(reporter->stdcharger->std_psy_usb))
        {
            ret = -EINVAL;
            goto err_malloc;
        }

        reporter->stdcharger->std_psy_usb->name = USB_CHARGER;
        reporter->stdcharger->std_psy_usb->supplied_to = (char**)S10_psy_supplies_to;
        reporter->stdcharger->std_psy_usb->num_supplicants = ARRAY_SIZE(S10_psy_supplies_to);
        reporter->stdcharger->std_psy_usb->type = POWER_SUPPLY_TYPE_USB;

        /* 上报的属性和属性个数以及上报接口修改为USB对应 */
        reporter->stdcharger->std_psy_usb->properties = charger->S10_charger_support_props;
        reporter->stdcharger->std_psy_usb->num_properties = charger->num_properties;
        reporter->stdcharger->std_psy_usb->get_property = charger->charger_report_get_property_usb;

        /* 注册USB上报结构体*/
        ret = power_supply_register(&pdev->dev, reporter->stdcharger->std_psy_usb);
        if (ret)
        {
            goto err_psy_usb_register;
        }
    }

    //	ret = S10_charger_psy_report();
    return ret;

err_psy_usb_register:
    if (!IS_ERR_OR_NULL(charger->std_psy_ac))
    {
        power_supply_unregister((struct power_supply*)&charger->std_psy_ac);
        kfree(&charger->std_psy_ac);
    }

err_psy_ac_register:
    if (!IS_ERR_OR_NULL(charger->std_psy_usb))
    {
        power_supply_unregister((struct power_supply*)&charger->std_psy_usb);
        kfree(&charger->std_psy_usb);
    }

err_malloc:
    if (reporter->stdcharger->std_psy_usb)
    {
        kfree(reporter->stdcharger->std_psy_usb);
        reporter->stdcharger->std_psy_usb = NULL;
    }

    if (reporter->stdcharger->std_psy_ac)
    {
        kfree(reporter->stdcharger->std_psy_ac);
        reporter->stdcharger->std_psy_ac = NULL;
    }
err_getinfo:
    power_debug(HW_POWER_PSYINFO_REPORT_ERR, "%s: function end with err\n", __func__);
    return ret;
}

EXPORT_SYMBOL_GPL(charger2monitor_register);

int charger2monitor_unregister(struct S10_std_charger_device* charger)
{
    struct S10_psy_monitor_dev_info* monitordev_info = NULL;
    struct S10_psy_reporter* reporter = NULL;
    int ret = 0;

    if (IS_ERR_OR_NULL(charger))
    {
        return -EINVAL;
    }

    if (!charger->name)
    {
        return -EINVAL;
    }

    monitordev_info = S10_power_get_monitor_devinfo();
    reporter = S10_power_get_monitor_reporter();
    if (IS_ERR_OR_NULL(monitordev_info) || IS_ERR_OR_NULL(reporter))
    {
        ret = -EINVAL;
    }

    if (IS_ERR_OR_NULL(charger->std_psy_ac))
    {
        return -EINVAL;
    }

    power_supply_unregister(charger->std_psy_ac);
    kfree(&charger->std_psy_ac);

    if (monitordev_info->usb_charging_support)
    {
        if (IS_ERR_OR_NULL(charger->std_psy_usb))
        {
            return -EINVAL;
        }

        power_supply_unregister(charger->std_psy_usb);
        kfree(&charger->std_psy_usb);
    }

    reporter->stdcharger = NULL;
    return ret;
}

EXPORT_SYMBOL_GPL(charger2monitor_unregister);

int battery2monitor_register(struct S10_std_battery_device* battery)
{
    struct S10_psy_monitor_dev_info* monitordev_info = NULL;
    struct S10_psy_reporter* reporter = NULL;
    struct S10_psy_node* hw_psy = NULL;
    struct platform_device * pdev = S10_power_get_monitor_device();
    int ret = 0;

    monitordev_info = S10_power_get_monitor_devinfo();
    reporter = S10_power_get_monitor_reporter();

    power_debug(HW_POWER_PSYINFO_REPORT_DUG, "[Power DUG: %s, %s, %d] enter battery2monitor_register! \n", __FILE__,
                __func__, __LINE__);
    if (IS_ERR_OR_NULL(monitordev_info) || IS_ERR_OR_NULL(reporter))
    {
        ret = -EINVAL;
        goto err_getinfo;
    }

    power_debug(HW_POWER_PSYINFO_REPORT_DUG, "[Power DUG: %s, %s, %d] battery2monitor_register! \n", __FILE__,
                __func__, __LINE__);
    if (battery->battery_psy_visible)
    {
        hw_psy = kzalloc(sizeof(*hw_psy), GFP_KERNEL);
        if (IS_ERR_OR_NULL(hw_psy))
        {
            ret = -EINVAL;
            goto err_malloc;
        }

        hw_psy->psy.name = battery->battery_name;
        hw_psy->psy.supplied_to = (char**)S10_psy_supplies_to;
        hw_psy->psy.num_supplicants = ARRAY_SIZE(S10_psy_supplies_to);
        hw_psy->psy.type = battery->psy_type;
        hw_psy->psy.properties = battery->S10_battery_support_props;
        hw_psy->psy.num_properties = battery->num_properties;
        hw_psy->psy.get_property = battery->battery_report_get_property;

        strcpy(reporter->report_battery_name_array[reporter->report_battery_num], battery->battery_name);
        reporter->report_battery_num++;
        S10_power_lock_lock(&reporter->psy_list_lock);
        list_add_tail(&hw_psy->node, &reporter->powersupply_list);
        S10_power_lock_unlock(&reporter->psy_list_lock);
    }

    power_debug(HW_POWER_PSYINFO_REPORT_DUG, "[Power DUG: %s, %s, %d] battery2monitor_register! \n", __FILE__,
                __func__, __LINE__);
    S10_power_lock_lock(&reporter->battery_list_lock);
    list_add_tail(&battery->node, &reporter->battery_list);
    S10_power_lock_unlock(&reporter->battery_list_lock);
    power_debug(HW_POWER_PSYINFO_REPORT_DUG, "[Power DUG: %s, %s, %d] battery2monitor_register! \n", __FILE__,
                __func__, __LINE__);

    //first detect and report
    ret = S10_psy_battery_dect_info_get(battery, &battery->battery_detect_info);
    S10_power_lock_lock(&battery->data_lock);
    memcpy(&battery->battery_report_info, &battery->battery_detect_info, sizeof(struct battery_psy_info));
    S10_power_lock_unlock(&battery->data_lock);
    if (battery->battery_psy_visible)
    {
        ret = power_supply_register(&pdev->dev, (struct power_supply *)&(hw_psy->psy));
        if (ret)
        {
            goto err_psy_register;
        }

        //	ret = S10_battery_psy_report(battery->battery_name);
    }

    return ret;
err_psy_register:
    list_del(&hw_psy->node);
err_malloc:
    power_debug(HW_POWER_PSYINFO_REPORT_DUG, "[Power DUG: %s, %s, %d] battery2monitor_register! \n", __FILE__,
                __func__, __LINE__);
    if (hw_psy)
    {
        kfree(hw_psy);
        hw_psy = NULL;
    }

err_getinfo:
    power_debug(HW_POWER_PSYINFO_REPORT_ERR, "%s: function end with err\n", __func__);
    return ret;
}

EXPORT_SYMBOL_GPL(battery2monitor_register);

int battery2monitor_unregister(struct S10_std_battery_device* battery)
{
    struct S10_psy_reporter* reporter = NULL;
    struct S10_psy_node * hwpsy;
    struct S10_std_battery_device* pdel_battery;
    int ret = 0;

    reporter = S10_power_get_monitor_reporter();
    if (IS_ERR_OR_NULL(reporter))
    {
        power_debug(HW_POWER_PSYINFO_REPORT_ERR, "%s: NULL Pointer\n", __func__);
        return -EINVAL;
    }

    if (IS_ERR_OR_NULL(battery))
    {
        power_debug(HW_POWER_PSYINFO_REPORT_ERR, "%s: NULL Pointer\n", __func__);
        return -EINVAL;
    }

    if (!battery->battery_name)
    {
        power_debug(HW_POWER_PSYINFO_REPORT_ERR, "%s: Name is null\n", __func__);
        return -EINVAL;
    }

    hwpsy = get_psy_in_list(battery->battery_name);
    if (IS_ERR_OR_NULL(hwpsy))
    {
        power_debug(HW_POWER_PSYINFO_REPORT_ERR, "%s: can't find [%s] psy!\n", __func__, battery->battery_name);
    }
    else
    {
        S10_power_lock_lock(&reporter->psy_list_lock);
        list_del(&hwpsy->node);
        S10_power_lock_unlock(&reporter->psy_list_lock);

        power_supply_unregister(&hwpsy->psy);
        kfree(hwpsy);
    }

    pdel_battery = S10_power_get_battery_in_list(battery->battery_name);
    if (IS_ERR_OR_NULL(pdel_battery))
    {
        power_debug(HW_POWER_PSYINFO_REPORT_ERR, "%s: can't find [%s] battery!\n", __func__, battery->battery_name);
    }
    else
    {
        S10_power_lock_lock(&reporter->battery_list_lock);
        list_del(&pdel_battery->node);
        S10_power_lock_unlock(&reporter->battery_list_lock);
    }

    return ret;
}

EXPORT_SYMBOL_GPL(battery2monitor_unregister);

static int psy_list_destroy(void)
{
    struct S10_psy_reporter* reporter;
    struct S10_psy_node* hwpsy = NULL;
    struct list_head *p = NULL;
    int ret = 0;

    reporter = S10_power_get_monitor_reporter();
    if (IS_ERR_OR_NULL(reporter))
    {
        power_debug(HW_POWER_PSYINFO_REPORT_ERR, "%s: NULL Pointer\n", __func__);
        return -EINVAL;
    }

    S10_power_lock_lock(&reporter->psy_list_lock);
    if (!list_empty(&reporter->powersupply_list))
    {
        list_for_each(p, &reporter->powersupply_list)
        {
            hwpsy = list_entry(p, struct S10_psy_node, node);
            if (!IS_ERR_OR_NULL(hwpsy))
            {
                list_del(&hwpsy->node);
            }
        }
    }

    S10_power_lock_unlock(&reporter->psy_list_lock);

    return ret;
}

static int battery_list_destroy(void)
{
    struct S10_psy_reporter* reporter;
    struct S10_std_battery_device* battery = NULL;
    struct list_head *p = NULL;
    int ret = 0;

    reporter = S10_power_get_monitor_reporter();
    if (IS_ERR_OR_NULL(reporter))
    {
        power_debug(HW_POWER_PSYINFO_REPORT_ERR, "%s: NULL Pointer\n", __func__);
        return -EINVAL;
    }

    S10_power_lock_lock(&reporter->battery_list_lock);
    if (!list_empty(&reporter->battery_list))
    {
        list_for_each(p, &reporter->battery_list)
        {
            battery = list_entry(p, struct S10_std_battery_device, node);
            if (!IS_ERR_OR_NULL(battery))
            {
                S10_power_lock_unlock(&reporter->battery_list_lock);
            }

            ret = battery2monitor_unregister(battery);
            S10_power_lock_lock(&reporter->battery_list_lock);
        }
    }

    S10_power_lock_unlock(&reporter->battery_list_lock);
    return ret;
}

static void fill_charger_psy_info(enum power_supply_property psp, union power_supply_propval val,
                                  struct charger_psy_info* info)
{
    switch (psp)
    {
    case POWER_SUPPLY_PROP_ONLINE:
        info->charger_online = val.intval;
        break;
    case POWER_SUPPLY_PROP_TYPE:
        info->charger_type = val.intval;
        break;

        //		case POWER_SUPPLY_PROP_TYPE:
        //			info->charging_phase = val.intval;
        //			break;
    case POWER_SUPPLY_PROP_STATUS:
        info->charging_status = val.intval;
        break;
    case POWER_SUPPLY_PROP_HEALTH:
        info->charger_health = val.intval;
        break;
    default:
        break;
    }

    return;
}

static int S10_psy_charger_real_info_get(struct charger_psy_info* info)
{
    struct S10_psy_reporter* reporter;
    union power_supply_propval val;
    enum power_supply_property psp;
    int num = 0, count = 0;
    int ret = 0;

    reporter = S10_power_get_monitor_reporter();
    if (IS_ERR_OR_NULL(reporter))
    {
        power_debug(HW_POWER_PSYINFO_REPORT_ERR, "%s: NULL reporter\n", __func__);
        return -EINVAL;
    }

    if ((reporter->stdcharger->S10_charger_support_props)
        && (reporter->stdcharger->ops->charger_get_real_property))
    {
        num = reporter->stdcharger->num_properties;
        while (count < num)
        {
            psp = reporter->stdcharger->S10_charger_support_props[count];
            ret = reporter->stdcharger->ops->charger_get_real_property(psp, &val);
            S10_power_lock_lock(&reporter->stdcharger->data_lock);
            power_debug(HW_POWER_PSYINFO_REPORT_DUG, "%s:  psp%d=%d\n", __func__, (int)psp,val.intval);
            fill_charger_psy_info(psp, val, info);
            S10_power_lock_unlock(&reporter->stdcharger->data_lock);
            count++;
        }
    }
    else
    {
        ret = -ESRCH;
    }

    return ret;
}

/* 预留接口，备用 */
#if 0
static int S10_psy_battery_dect_psp_get(char* battery_name, enum power_supply_property psp,
                                           union power_supply_propval* val)
{
    struct S10_std_battery_device * battery;
    int ret = -EINVAL;

    battery = S10_power_get_battery_in_list(battery_name);
    if (IS_ERR_OR_NULL(battery))
    {
        power_debug(HW_POWER_PSYINFO_REPORT_ERR, "%s: can't find %s battery\n", __func__, battery_name);
        return ret;
    }

    if (battery->battery_detect_get_property)
    {
        ret = battery->battery_detect_get_property(battery_name, psp, val);
    }
    else
    {
        ret = -ESRCH;
    }

    return ret;
}
#endif

static void fill_battery_psy_info(enum power_supply_property psp, union power_supply_propval val,
                                  struct battery_psy_info* info)
{
    switch (psp)
    {
    case POWER_SUPPLY_PROP_STATUS:
        info->battery_status = val.intval;
        break;
    case POWER_SUPPLY_PROP_TECHNOLOGY:
        info->battery_technology = val.intval;
        break;
    case POWER_SUPPLY_PROP_PRESENT:
        info->battery_present = val.intval;
        break;
    case POWER_SUPPLY_PROP_HEALTH:
        info->battery_health = val.intval;
        break;
    case POWER_SUPPLY_PROP_VOLTAGE_NOW:
        info->battery_voltage = val.intval;
        break;
    case POWER_SUPPLY_PROP_CURRENT_NOW:
        info->battery_current = val.intval;
        break;
    case POWER_SUPPLY_PROP_CAPACITY:
        info->battery_capacity = val.intval;
        break;
    case POWER_SUPPLY_PROP_TEMP:
        info->battery_temperature = val.intval;
        break;
    case POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN:
        info->battery_voltage_max_design = val.intval;
        break;
    case POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN:
        info->battery_voltage_min_design = val.intval;
        break;
    case POWER_SUPPLY_PROP_ENERGY_FULL_DESIGN:
        info->battery_capacity_full_design = val.intval;
        break;
    case POWER_SUPPLY_PROP_ENERGY_EMPTY_DESIGN:
        info->battery_capacity_empty_design = val.intval;
        break;
    case POWER_SUPPLY_PROP_CYCLE_COUNT:
        info->battery_cycle_count = val.intval;
        break;
    case POWER_SUPPLY_PROP_TIME_TO_FULL_NOW:
        info->time2full_now = val.intval;
        break;
    case POWER_SUPPLY_PROP_TIME_TO_FULL_AVG:
        info->time2full_avg = val.intval;
        break;
    case POWER_SUPPLY_PROP_TIME_TO_EMPTY_NOW:
        info->time2empty_now = val.intval;
        break;
    case POWER_SUPPLY_PROP_TIME_TO_EMPTY_AVG:
        info->time2empty_avg = val.intval;
        break;
    case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN:
        info->battery_capacity_full_design = val.intval;
        break;
    case POWER_SUPPLY_PROP_CHARGE_FULL:
        info->battery_capacity_full_charge = val.intval;
        break;        
    default:
        break;
    }

    return;
}

static int S10_psy_battery_dect_info_get(struct S10_std_battery_device* battery, struct battery_psy_info* info)
{
    struct S10_psy_reporter* reporter = NULL;
    union power_supply_propval val;
    enum power_supply_property psp;
    int num = 0, count = 0;
    int ret = -EINVAL;

    reporter = S10_power_get_monitor_reporter();
    if (IS_ERR_OR_NULL(reporter))
    {
        power_debug(HW_POWER_PSYINFO_REPORT_ERR, "%s: NULL reporter\n", __func__);
        return ret;
    }

    if (IS_ERR_OR_NULL(battery))
    {
        power_debug(HW_POWER_PSYINFO_REPORT_ERR, "%s: can't find battery\n", __func__);
        return ret;
    }

    if (battery->S10_battery_support_props && battery->battery_detect_get_property)
    {
        num = battery->num_properties;
        while (count < num)
        {
            psp = battery->S10_battery_support_props[count];
            ret = battery->battery_detect_get_property(battery->battery_name, psp, &val);
            power_debug(HW_POWER_PSYINFO_REPORT_DUG, "%s:  psp%d=%d\n", __func__, (int)psp,val.intval);
            S10_power_lock_lock(&battery->data_lock);
            fill_battery_psy_info(psp, val, info);
            S10_power_lock_unlock(&battery->data_lock);
            count++;
        }
    }
    else
    {
        ret = -ESRCH;
    }
    return ret;
}

static ssize_t S10_show_charging_enable(struct device *dev,
                                           struct device_attribute *attr, char *buf)
{
    struct S10_psy_reporter* reporter;
    int val = 0;

    reporter = S10_power_get_monitor_reporter();
    val = reporter->stdcharger->dev_info.charging_enable_status;
    return sprintf(buf, "%d\n", val);
}

static ssize_t S10_store_charging_enable(struct device *dev,
                                            struct device_attribute *attr, const char *buf, size_t count)
{
    struct S10_psy_reporter* reporter;
    int val = 0;

    reporter = S10_power_get_monitor_reporter();

    if ((strict_strtol(buf, 10, (long*)&val) < 0) || (val < 0)
        || (val > 1))
    {
        return -EINVAL;
    }

    if (reporter->stdcharger->ops->charging_enable)
    {
        reporter->stdcharger->ops->charging_enable((void*)reporter->stdcharger, val);
    }

    return count;
}

static ssize_t S10_show_charger_in_max_current(struct device *dev,
                                                  struct device_attribute *attr, char *buf)
{
    struct S10_psy_reporter* reporter;
    int val = 0;

    reporter = S10_power_get_monitor_reporter();
    if (reporter->stdcharger->ops->charging_max_charging_current_get)
    {
        reporter->stdcharger->ops->charging_max_charging_current_get((void*)reporter->stdcharger, &val);
    }

    return sprintf(buf, "%d\n", val);
}

static ssize_t S10_store_charger_in_max_current(struct device *dev,
                                                   struct device_attribute *attr, const char *buf, size_t count)
{
    struct S10_psy_reporter* reporter;
    int val = 0;

    reporter = S10_power_get_monitor_reporter();

    if (strict_strtol(buf, 10, (long*)&val) < 0)
    {
        return -EINVAL;
    }

    if (reporter->stdcharger->ops->charging_max_charging_current_set)
    {
        reporter->stdcharger->ops->charging_max_charging_current_set((void*)reporter->stdcharger, val);
    }

    return count;
}

static ssize_t S10_show_charger_type(struct device *dev,
                                        struct device_attribute *attr, char *buf)
{
    struct S10_psy_reporter* reporter;
    union power_supply_propval val;

    reporter   = S10_power_get_monitor_reporter();
    val.intval = 0;
    if (reporter->stdcharger->ops->charger_get_real_property)
    {
        reporter->stdcharger->ops->charger_get_real_property(POWER_SUPPLY_PROP_TYPE, &val);
    }

    return sprintf(buf, "%d\n", val.intval);
}

static ssize_t S10_show_charging_max_current(struct device *dev,
                                                struct device_attribute *attr, char *buf)
{
    struct S10_psy_reporter* reporter;
    int val = 0;

    reporter = S10_power_get_monitor_reporter();
    if (reporter->stdcharger->ops->charging_max_charging_current_get)
    {
        reporter->stdcharger->ops->charging_max_charging_current_get((void*)reporter->stdcharger, &val);
    }

    return sprintf(buf, "%d\n", val);
}

static ssize_t S10_store_charging_max_current(struct device *dev,
                                                 struct device_attribute *attr, const char *buf, size_t count)
{
    struct S10_psy_reporter* reporter;
    int val = 0;
    int set_current = 0;

    reporter = S10_power_get_monitor_reporter();

    if (strict_strtol(buf, 10, (long*)&val) < 0)
    {
        return -EINVAL;
    }

    if ((val >= 0) && (val <= 3)) //for PT charging current set
    {
        switch (val)
        {
        case 0:
            set_current = 1500;
            break;
        case 1:
            set_current = 500;
            break;
        case 2:
            set_current = 1500;
            break;
        case 3:
            set_current = 2000;
            break;
        default:
            set_current = 500;
            break;
        }
    }
    else
    {
        set_current = val;
    }

    /* 产线电流设置接口，val等于0的时候恢复默认的电流设置，等于其它值时优先使用设定值 */
    S10_power_lock_lock(&reporter->data_lock);
    reporter->factory_charging_current_set_flag = val ? 1 : 0;
    S10_power_lock_unlock(&reporter->data_lock);

    if (reporter->stdcharger->ops->charging_max_charging_current_set)
    {
        reporter->stdcharger->ops->charging_max_charging_current_set((void*)reporter->stdcharger, set_current);
    }

    return count;
}

static ssize_t S10_show_charger_status(struct device *dev,
                                          struct device_attribute *attr, char *buf)
{
    struct S10_psy_reporter* reporter;
    union power_supply_propval val;

    reporter   = S10_power_get_monitor_reporter();
    val.intval = 0;
    if (reporter->stdcharger->ops->charger_get_real_property)
    {
        reporter->stdcharger->ops->charger_get_real_property(POWER_SUPPLY_PROP_STATUS, &val);
    }

    return sprintf(buf, "%d\n", val.intval);
}

static ssize_t S10_show_charger_health(struct device *dev,
                                          struct device_attribute *attr, char *buf)
{
    struct S10_psy_reporter* reporter;
    union power_supply_propval val;

    reporter   = S10_power_get_monitor_reporter();
    val.intval = 0;
    if (reporter->stdcharger->ops->charger_get_real_property)
    {
        reporter->stdcharger->ops->charger_get_real_property(POWER_SUPPLY_PROP_HEALTH, &val);
    }

    return sprintf(buf, "%d\n", val.intval);
}

static ssize_t S10_show_charging_phase(struct device *dev,
                                          struct device_attribute *attr, char *buf)
{
    struct S10_psy_reporter* reporter;
    int val = 0;

    reporter = S10_power_get_monitor_reporter();
    return sprintf(buf, "%d\n", val);
}

static ssize_t S10_show_battery_present(struct device *dev,
                                           struct device_attribute *attr, char *buf)
{
    struct S10_psy_reporter* reporter;
    struct S10_std_battery_device* battery;
    struct S10_psy_node* hwpsy = NULL;
    union power_supply_propval val;
    int icount = 0, ret = 0;

    reporter = S10_power_get_monitor_reporter();
    for (icount = 0; icount < reporter->report_battery_num; icount++)
    {
        battery = S10_power_get_battery_in_list(reporter->report_battery_name_array[icount]);
        if (IS_ERR_OR_NULL(battery))
        {
            continue;
        }

        hwpsy = get_psy_in_list(battery->battery_name);
        if (IS_ERR_OR_NULL(hwpsy))
        {
            power_debug(HW_POWER_PSYINFO_REPORT_ERR, "%s: can't find %s psy\n", __func__, battery->battery_name);
            continue;
        }

        if (battery->battery_detect_get_property)
        {
            battery->battery_detect_get_property(battery->battery_name, POWER_SUPPLY_PROP_PRESENT, &val);
            ret += sprintf(buf, "%s:%s\n", battery->battery_name, val.intval > 0 ? "present" : "not present");
        }
        else
        {
            continue;
        }
    }

    return ret;
}

static ssize_t S10_show_battery_health(struct device *dev,
                                          struct device_attribute *attr, char *buf)
{
    struct S10_psy_reporter* reporter;
    struct S10_std_battery_device* battery;
    struct S10_psy_node* hwpsy = NULL;
    union power_supply_propval val;
    int icount = 0, ret = 0;

    reporter = S10_power_get_monitor_reporter();
    for (icount = 0; icount < reporter->report_battery_num; icount++)
    {
        battery = S10_power_get_battery_in_list(reporter->report_battery_name_array[icount]);
        if (IS_ERR_OR_NULL(battery))
        {
            continue;
        }

        hwpsy = get_psy_in_list(battery->battery_name);
        if (IS_ERR_OR_NULL(hwpsy))
        {
            power_debug(HW_POWER_PSYINFO_REPORT_ERR, "%s: can't find %s psy\n", __func__, battery->battery_name);
            continue;
        }

        if (battery->battery_detect_get_property)
        {
            battery->battery_detect_get_property(battery->battery_name, POWER_SUPPLY_PROP_HEALTH, &val);
            ret += sprintf(buf, "%s:%s\n", battery->battery_name, S10_psy_health_str[val.intval]);
        }
        else
        {
            continue;
        }
    }

    return ret;
}

static ssize_t S10_show_battery_voltage(struct device *dev,
                                           struct device_attribute *attr, char *buf)
{
    struct S10_psy_reporter* reporter;
    struct S10_std_battery_device* battery;
    struct S10_psy_node* hwpsy = NULL;
    union power_supply_propval val;
    int icount = 0, ret = 0;

    reporter = S10_power_get_monitor_reporter();
    for (icount = 0; icount < reporter->report_battery_num; icount++)
    {
        battery = S10_power_get_battery_in_list(reporter->report_battery_name_array[icount]);
        if (IS_ERR_OR_NULL(battery))
        {
            continue;
        }

        hwpsy = get_psy_in_list(battery->battery_name);
        if (IS_ERR_OR_NULL(hwpsy))
        {
            power_debug(HW_POWER_PSYINFO_REPORT_ERR, "%s: can't find %s psy\n", __func__, battery->battery_name);
            continue;
        }

        if (battery->battery_detect_get_property)
        {
            battery->battery_detect_get_property(battery->battery_name, POWER_SUPPLY_PROP_VOLTAGE_NOW, &val);
            ret += sprintf(buf, "%s:%d mv\n", battery->battery_name, val.intval);
        }
        else
        {
            continue;
        }
    }

    return ret;
}

static ssize_t S10_show_battery_current(struct device *dev,
			     struct device_attribute *attr, char *buf)
{
    struct S10_psy_reporter* reporter;
    struct S10_std_battery_device* battery;
    struct S10_psy_node* hwpsy = NULL;
    union power_supply_propval val;
    int icount = 0, ret = 0;

    reporter = S10_power_get_monitor_reporter();
    for (icount = 0; icount < reporter->report_battery_num; icount++)
    {
        battery = S10_power_get_battery_in_list(reporter->report_battery_name_array[icount]);
        if (IS_ERR_OR_NULL(battery))
        {
            continue;
        }

        hwpsy = get_psy_in_list(battery->battery_name);
        if (IS_ERR_OR_NULL(hwpsy))
        {
            power_debug(HW_POWER_PSYINFO_REPORT_ERR, "%s: can't find %s psy\n", __func__, battery->battery_name);
            continue;
        }

        if (battery->battery_detect_get_property)
        {
            battery->battery_detect_get_property(battery->battery_name, POWER_SUPPLY_PROP_CURRENT_NOW, &val);
            ret += sprintf(buf, "%s:%d ma\n", battery->battery_name, val.intval);
        }
        else
        {
            continue;
        }
    }

    return ret;
}

static ssize_t S10_show_battery_temperature(struct device *dev,
                                               struct device_attribute *attr, char *buf)
{
    struct S10_psy_reporter* reporter;
    struct S10_std_battery_device* battery;
    struct S10_psy_node* hwpsy = NULL;
    union power_supply_propval val;
    int icount = 0, ret = 0;

    reporter = S10_power_get_monitor_reporter();
    for (icount = 0; icount < reporter->report_battery_num; icount++)
    {
        battery = S10_power_get_battery_in_list(reporter->report_battery_name_array[icount]);
        if (IS_ERR_OR_NULL(battery))
        {
            continue;
        }

        hwpsy = get_psy_in_list(battery->battery_name);
        if (IS_ERR_OR_NULL(hwpsy))
        {
            power_debug(HW_POWER_PSYINFO_REPORT_ERR, "%s: can't find %s psy\n", __func__, battery->battery_name);
            continue;
        }

        if (battery->battery_report_get_property)
        {
            battery->battery_report_get_property(&hwpsy->psy, POWER_SUPPLY_PROP_TEMP, &val);
            ret += sprintf(buf, "%s:%d \n", battery->battery_name, val.intval);
        }
        else
        {
            continue;
        }
    }

    return ret;
}

static ssize_t  S10_show_battery_capacity_unsmoothed(struct device *dev,
                                                        struct device_attribute *attr, char *buf)
{
    struct S10_psy_reporter* reporter;
    struct S10_std_battery_device* battery;
    struct S10_psy_node* hwpsy = NULL;
    union power_supply_propval val;
    int icount = 0, ret = 0;

    reporter = S10_power_get_monitor_reporter();
    for (icount = 0; icount < reporter->report_battery_num; icount++)
    {
        battery = S10_power_get_battery_in_list(MAIN_BATTERY);
        if (IS_ERR_OR_NULL(battery))
        {
            continue;
        }

        hwpsy = get_psy_in_list(battery->battery_name);
        if (IS_ERR_OR_NULL(hwpsy))
        {
            power_debug(HW_POWER_PSYINFO_REPORT_ERR, "%s: can't find %s psy\n", __func__, battery->battery_name);
            continue;
        }

        if (battery->battery_detect_get_property)
        {
            battery->battery_detect_get_property(battery->battery_name, POWER_SUPPLY_PROP_CAPACITY, &val);
            ret += sprintf(buf, "%d", val.intval);
        }
        else
        {
            continue;
        }
    }

    return ret;
}

static ssize_t S10_show_battery_capacity_smoothed(struct device *dev,
                                                     struct device_attribute *attr, char *buf)
{
    struct S10_psy_reporter* reporter;
    struct S10_std_battery_device* battery;
    struct S10_psy_node* hwpsy = NULL;
    union power_supply_propval val;
    int icount = 0, ret = 0;

    reporter = S10_power_get_monitor_reporter();
    for (icount = 0; icount < reporter->report_battery_num; icount++)
    {
        battery = S10_power_get_battery_in_list(reporter->report_battery_name_array[icount]);
        if (IS_ERR_OR_NULL(battery))
        {
            continue;
        }

        hwpsy = get_psy_in_list(battery->battery_name);
        if (IS_ERR_OR_NULL(hwpsy))
        {
            power_debug(HW_POWER_PSYINFO_REPORT_ERR, "%s: can't find %s psy\n", __func__, battery->battery_name);
            continue;
        }

        if (battery->battery_report_get_property)
        {
            battery->battery_report_get_property(&hwpsy->psy, POWER_SUPPLY_PROP_CAPACITY, &val);

            ret += sprintf(buf, "%s:%d \n", battery->battery_name, val.intval);
        }
        else
        {
            continue;
        }
    }

    return ret;
}

static ssize_t S10_show_factory_charging_full_capacity(struct device *dev,
                                                          struct device_attribute *attr, char *buf)
{
    struct S10_psy_reporter* reporter;
    int capacity = 0, ret = 0;

    reporter = S10_power_get_monitor_reporter();
    S10_power_lock_lock(&reporter->data_lock);
    capacity = reporter->factory_charging_full_capacity;
    S10_power_lock_unlock(&reporter->data_lock);
    ret = sprintf(buf, "%d \n", capacity);
    return ret;
}

static ssize_t S10_store_factory_charging_full_capacity(struct device *dev,
                                                           struct device_attribute *attr, const char *buf, size_t count)
{
    struct S10_psy_reporter* reporter;
    int val = 0;

    reporter = S10_power_get_monitor_reporter();

    if (strict_strtol(buf, 10, (long*)&val) < 0)
    {
        return -EINVAL;
    }

    if ((val > 0) && (val < 100)) //for PT charging current set
    {
        S10_power_lock_lock(&reporter->data_lock);
        reporter->factory_charging_full_capacity = val;
        S10_power_lock_unlock(&reporter->data_lock);
    }

    return count;
}

static ssize_t S10_show_factory_charging_enable(struct device *dev,
                                                   struct device_attribute *attr, char *buf)
{
    struct S10_psy_reporter* reporter;
    int val = 0, ret = 0;

    reporter = S10_power_get_monitor_reporter();
    S10_power_lock_lock(&reporter->data_lock);
    val = reporter->factory_charging_mode_enable;
    S10_power_lock_unlock(&reporter->data_lock);
    ret = sprintf(buf, "%d \n", val);
    return ret;
}

static ssize_t S10_store_factory_charging_enable(struct device *dev,
                                                    struct device_attribute *attr, const char *buf, size_t count)
{
    struct S10_psy_reporter* reporter;
    int val = 0;

    reporter = S10_power_get_monitor_reporter();

    if (strict_strtol(buf, 10, (long*)&val) < 0)
    {
        return -EINVAL;
    }

    S10_power_lock_lock(&reporter->data_lock);
    reporter->factory_charging_mode_enable = val ? 1 : 0;
    S10_power_lock_unlock(&reporter->data_lock);
    return count;
}

static ssize_t S10_show_shutdown_charging_flag(struct device *dev,
                                                  struct device_attribute *attr, char *buf)
{
    struct S10_psy_reporter* reporter;
    int val = 0, ret = 0;

    reporter = S10_power_get_monitor_reporter();
    S10_power_lock_lock(&reporter->data_lock);
    val = reporter->psy_monitor_shutdown_charging_flag;
    S10_power_lock_unlock(&reporter->data_lock);
    ret = sprintf(buf, "%d \n", val);
    return ret;
}

static ssize_t S10_store_shutdown_charging_flag(struct device *dev,
                                                   struct device_attribute *attr, const char *buf, size_t count)
{
    struct S10_psy_reporter* reporter;
    int val = 0;

    reporter = S10_power_get_monitor_reporter();
    if (strict_strtol(buf, 10, (long*)&val) < 0)
    {
        return -EINVAL;
    }

    S10_power_lock_lock(&reporter->data_lock);
    reporter->psy_monitor_shutdown_charging_flag = val ? 1 : 0;
    S10_power_lock_unlock(&reporter->data_lock);

    return count;
}

static ssize_t S10_show_charger_firmware_version(struct device *dev,
                                                    struct device_attribute *attr, char *buf)
{
    return 0;
}

static ssize_t S10_charger_firmware_download(struct device *dev,
                                                struct device_attribute *attr, const char *buf, size_t count)
{
    return count;
}

static ssize_t S10_show_coulometer_firmware_version(struct device *dev,
                                                       struct device_attribute *attr, char *buf)
{
    struct S10_psy_reporter* reporter;
    struct S10_std_battery_device* battery;
    int icount = 0, ret = 0, version = 0;

    reporter = S10_power_get_monitor_reporter();
    for (icount = 0; icount < reporter->report_battery_num; icount++)
    {
        battery = S10_power_get_battery_in_list(reporter->report_battery_name_array[icount]);
        if (IS_ERR_OR_NULL(battery))
        {
            continue;
        }

        if (battery->ops->battery_get_firmware_version)
        {
            version = battery->ops->battery_get_firmware_version();
        }
        else
        {
            continue;
        }
        ret += sprintf(buf, "%x", version);
    }

    return ret;
}

static ssize_t S10_coulometer_firmware_download(struct device *dev,
                                                   struct device_attribute *attr, const char *buf, size_t count)
{
    struct S10_psy_reporter* reporter;
    struct S10_std_battery_device* battery;
    int find_flag = 0;
    int charging_flag = 0; //0 is not charging ,1 is charging.
    unsigned char firmware_path[255];

    reporter = S10_power_get_monitor_reporter();

    if (!list_empty(&reporter->battery_list))
    {
        list_for_each_entry(battery, &reporter->battery_list, node)
        {
            if (battery->firmware_name[0] == '\0')
            {
                continue;
            }

            if (strstr(buf, battery->firmware_name))
            {
                find_flag = 1;
                break;
            }
        }
    }

    if (find_flag)
    {
        if (count > 254)
        {
            power_debug(HW_POWER_PSYINFO_REPORT_DUG, "%s: firmware_path too long [%d]\n", __func__, count);
            return -EINVAL;
        }

        memcpy (firmware_path, buf, count);

        /* replace '\n' with  '\0'  */
        if ((firmware_path[count - 1]) == '\n')
        {
            firmware_path[count - 1] = '\0';
        }
        else
        {
            firmware_path[count] = '\0';
        }

        power_debug(HW_POWER_PSYINFO_REPORT_DUG, "%s: find firmware name [%s] battery \n", __func__, firmware_path);

        charging_flag = reporter->stdcharger->dev_info.charging_enable_status;
        if (1 == charging_flag)
        {
            if (reporter->stdcharger->ops->charging_enable)
            {
                reporter->stdcharger->ops->charging_enable((void*)reporter->stdcharger, (!charging_flag));
            }

            if (battery->ops->battery_firmware_download)
            {
                battery->ops->battery_firmware_download(firmware_path);
            }

            if (reporter->stdcharger->ops->charging_enable)
            {
                reporter->stdcharger->ops->charging_enable((void*)reporter->stdcharger, (charging_flag));
            }
        }
        else
        {
            if (battery->ops->battery_firmware_download)
            {
                battery->ops->battery_firmware_download(firmware_path);
            }
        }
    }
    else
    {
        power_debug(HW_POWER_PSYINFO_REPORT_DUG, "%s: can't find firmware name [%s] battery \n", __func__, buf);
    }

    return count;
}

static ssize_t S10_show_charger_chip_enable(struct device *dev,
                                                       struct device_attribute *attr, char *buf)
{
    int ienable = 0;
    struct S10_psy_reporter* reporter;

    reporter = S10_power_get_monitor_reporter();

    if (reporter->stdcharger->ops->charger_chip_enable)
    {
        ienable = reporter->stdcharger->ops->charger_chip_enable((void*)reporter->stdcharger, 1);
    }

    power_debug(HW_POWER_PSYINFO_REPORT_ERR, "[Power ERR: %s, %s, %d] \n", __FILE__, __func__, __LINE__);
    return sprintf(buf, "%d\n", ienable);
}

static ssize_t S10_show_charger_chip_disable(struct device *dev,
                                                struct device_attribute *attr, char *buf)
{
    int ienable = 0;
    struct S10_psy_reporter* reporter;

    reporter = S10_power_get_monitor_reporter();

    if (reporter->stdcharger->ops->charger_chip_enable)
    {
        ienable = reporter->stdcharger->ops->charger_chip_enable((void*)reporter->stdcharger, 0);
    }

    power_debug(HW_POWER_PSYINFO_REPORT_DUG, "[Power ERR: %s, %s, %d] \n", __FILE__, __func__, __LINE__);
    return sprintf(buf, "%d\n", ienable);
}

static ssize_t S10_show_charger_charge_done(struct device *dev,
                                               struct device_attribute *attr, char *buf)
{
    int done_flag = 0;
    struct S10_psy_reporter* reporter;

    reporter = S10_power_get_monitor_reporter();
    S10_power_lock_lock(&reporter->data_lock);
    if (NULL != reporter)
    {
        done_flag = reporter->stdcharger->charging_done_flag;
    }

    S10_power_lock_unlock(&reporter->data_lock);
    power_debug(HW_POWER_PSYINFO_REPORT_DUG, "[Power ERR: %s, %s, %d] \n", __FILE__, __func__, __LINE__);
    return sprintf(buf, "%d\n", done_flag);
}

static ssize_t S10_store_charger_charge_done(struct device *dev,
                                                struct device_attribute *attr, const char *buf, size_t count)
{
    /* 用于模拟充电芯片充电完成 */
    struct S10_psy_reporter* reporter;
    int val = 0;

    reporter = S10_power_get_monitor_reporter();
    if (strict_strtol(buf, 10, (long*)&val) < 0)
    {
        return -EINVAL;
    }

    S10_power_lock_lock(&reporter->data_lock);
    reporter->stdcharger->charging_done_flag = val ? 1 : 0;
    S10_power_lock_unlock(&reporter->data_lock);
    power_debug(HW_POWER_PSYINFO_REPORT_ERR, "[Power ERR: %s, %s, %d] \n", __FILE__, __func__, __LINE__);
    return count;
}
static ssize_t std_show_fast_charge_enable(struct device *dev,
                                            struct device_attribute *attr, char *buf)
{
    int ret = 0;
    struct S10_psy_reporter* reporter = NULL;

    reporter = S10_power_get_monitor_reporter();

    /* 设置充电模式标记位为快速充电 */
    S10_power_lock_lock(&reporter->data_lock);
    if (NULL != reporter)
    {
        reporter->power_monitor_charge_mode_flag = CHARGE_MODE_FAST;
    }
    S10_power_lock_unlock(&reporter->data_lock);

    if (reporter->stdcharger->ops->charger_chip_enable)
    {
        ret = reporter->stdcharger->ops->charge_mode_set((void*)reporter->stdcharger, CHARGE_MODE_FAST);
    }

    power_debug(HW_POWER_PSYINFO_REPORT_ERR, "[Power ERR: %s, %s, %d] \n", __FILE__, __func__, __LINE__);
    return sprintf(buf, "%d\n", ret);
}

static ssize_t std_show_normal_charge_enable(struct device *dev,
                                            struct device_attribute *attr, char *buf)
{
    int ret = 0;
    struct S10_psy_reporter* reporter = NULL;

    reporter = S10_power_get_monitor_reporter();

    /* 设置充电模式标记位为正常充电 */
    S10_power_lock_lock(&reporter->data_lock);
    if (NULL != reporter)
    {
        reporter->power_monitor_charge_mode_flag = CHARGE_MODE_NORMAL;
    }
    S10_power_lock_unlock(&reporter->data_lock);

    if (reporter->stdcharger->ops->charge_mode_set)
    {
        ret = reporter->stdcharger->ops->charge_mode_set((void*)reporter->stdcharger, CHARGE_MODE_NORMAL);
    }

    power_debug(HW_POWER_PSYINFO_REPORT_ERR, "[Power ERR: %s, %s, %d] \n", __FILE__, __func__, __LINE__);
    return sprintf(buf, "%d\n", ret);
}


static ssize_t battery_aging_degree_show(struct device *dev,
                                         struct device_attribute *attr,
                                         char *buf)
{
    struct S10_psy_reporter* reporter;
    struct S10_std_battery_device* battery;
    struct S10_psy_node* hwpsy = NULL;
    union power_supply_propval val;
    int icount = 0, ret = 0, degree = 0;

    reporter = S10_power_get_monitor_reporter();
    for (icount = 0; icount < reporter->report_battery_num; icount++)
    {
        battery = S10_power_get_battery_in_list(reporter->report_battery_name_array[icount]);
        if (IS_ERR_OR_NULL(battery))
        {
            continue;
        }

        if (battery->ops->battery_aging_detect)
        {
            degree = battery->ops->battery_aging_detect(battery->battery_name);
            power_debug(HW_POWER_BATTERY_IC_DUG,"%s:degree=%d\n",__func__,degree);
            ret += sprintf(buf, "%d\n", degree);
        }
        else
        {
            continue;
        }
    }

    return ret;
}

static DEVICE_ATTR(charger_firmware, S_IWUSR | S_IRUGO,
                   S10_show_charger_firmware_version,
                   S10_charger_firmware_download);
static DEVICE_ATTR(coulometer_firmware, S_IWUSR | S_IRUGO,
                   S10_show_coulometer_firmware_version,
                   S10_coulometer_firmware_download);
static DEVICE_ATTR(charging_enable, S_IWUSR | S_IRUGO,
                   S10_show_charging_enable,
                   S10_store_charging_enable);
static DEVICE_ATTR(charger_max_incurrent, S_IWUSR | S_IRUGO,
                   S10_show_charger_in_max_current,
                   S10_store_charger_in_max_current); //charger input current
static DEVICE_ATTR(charging_max_current, S_IWUSR | S_IRUGO,
                   S10_show_charging_max_current,
                   S10_store_charging_max_current); //charger charging current
static DEVICE_ATTR(charger_type, S_IWUSR | S_IRUGO,
                   S10_show_charger_type,
                   NULL); //charger type
static DEVICE_ATTR(charger_status, S_IWUSR | S_IRUGO,
                   S10_show_charger_status,
                   NULL); //charger status
static DEVICE_ATTR(charger_health, S_IWUSR | S_IRUGO,
                   S10_show_charger_health,
                   NULL); //charger health
static DEVICE_ATTR(charging_phase, S_IWUSR | S_IRUGO,
                   S10_show_charging_phase,
                   NULL); //charging phase

static DEVICE_ATTR(battery_present, S_IWUSR | S_IRUGO,
                   S10_show_battery_present,
                   NULL); //battery present
static DEVICE_ATTR(battery_health, S_IWUSR | S_IRUGO,
                   S10_show_battery_health,
                   NULL); //battery health
static DEVICE_ATTR(battery_voltage, S_IWUSR | S_IRUGO,
                   S10_show_battery_voltage,
                   NULL); //battery voltage
static DEVICE_ATTR(battery_current, S_IWUSR | S_IRUGO,
                   S10_show_battery_current,
                   NULL); //battery current
static DEVICE_ATTR(battery_temperature, S_IWUSR | S_IRUGO,
                   S10_show_battery_temperature,
                   NULL); //battery temperature
static DEVICE_ATTR(battery_capacity_unsmoothed, S_IWUSR | S_IRUGO,
                   S10_show_battery_capacity_unsmoothed,
                   NULL); //battery capacity before smooth
static DEVICE_ATTR(battery_capacity_smoothed, S_IWUSR | S_IRUGO,
                   S10_show_battery_capacity_smoothed,
                   NULL); //battery capacity after smooth
static DEVICE_ATTR(factory_charging_capacity, S_IWUSR | S_IRUGO,
                   S10_show_factory_charging_full_capacity,
                   S10_store_factory_charging_full_capacity); //battery capacity before smooth
static DEVICE_ATTR(factory_charging_enable, S_IWUSR | S_IRUGO,
                   S10_show_factory_charging_enable,
                   S10_store_factory_charging_enable); //battery capacity after smooth
static DEVICE_ATTR(shutdown_charging_enable, S_IWUSR | S_IRUGO,
                   S10_show_shutdown_charging_flag,
                   S10_store_shutdown_charging_flag); //battery capacity after smooth
static DEVICE_ATTR(charger_chip_enable, S_IWUSR | S_IRUGO,
                   S10_show_charger_chip_enable,
                   NULL); //battery capacity after smooth
static DEVICE_ATTR(charger_chip_disable, S_IWUSR | S_IRUGO,
                   S10_show_charger_chip_disable,
                   NULL); //battery capacity after smooth
static DEVICE_ATTR(charger_charge_done, S_IWUSR | S_IRUGO,
                   S10_show_charger_charge_done,
                   S10_store_charger_charge_done); // bq24161 finished charge flag

static DEVICE_ATTR(fast_charge_enable, S_IWUSR | S_IRUGO,
                   std_show_fast_charge_enable, NULL); /* 快速充电使能接口 */
static DEVICE_ATTR(normal_charge_enable, S_IWUSR | S_IRUGO,
                   std_show_normal_charge_enable, NULL); /* 快速充电禁止接口 */
static DEVICE_ATTR(battery_aging_degree, S_IWUSR | S_IRUGO,
                   battery_aging_degree_show, NULL); 

static struct attribute *S10_power_attributes[] =
{
    &dev_attr_charger_firmware.attr,
    &dev_attr_coulometer_firmware.attr,
    &dev_attr_charging_enable.attr,
    &dev_attr_charger_max_incurrent.attr,
    &dev_attr_charging_max_current.attr,
    &dev_attr_charger_type.attr,
    &dev_attr_charger_status.attr,
    &dev_attr_charger_health.attr,
    &dev_attr_charging_phase.attr,
    &dev_attr_battery_present.attr,
    &dev_attr_battery_health.attr,
    &dev_attr_battery_voltage.attr,
    &dev_attr_battery_current.attr,
    &dev_attr_battery_temperature.attr,
    &dev_attr_battery_capacity_unsmoothed.attr,
    &dev_attr_battery_capacity_smoothed.attr,
    &dev_attr_factory_charging_capacity.attr,
    &dev_attr_factory_charging_enable.attr,
    &dev_attr_shutdown_charging_enable.attr,
    &dev_attr_charger_chip_enable.attr,
    &dev_attr_charger_chip_disable.attr,
    &dev_attr_charger_charge_done.attr,
    &dev_attr_fast_charge_enable.attr,
    &dev_attr_normal_charge_enable.attr,
    &dev_attr_battery_aging_degree.attr,
    NULL
};

static mode_t S10_power_sysfs_is_visible(struct kobject *kobj, struct attribute *attr, int idx)
{
    return 1;
}

static const struct attribute_group S10_power_attr_group =
{
    .is_visible = S10_power_sysfs_is_visible,
    .attrs		= S10_power_attributes,
};

struct blocking_notifier_head S10_psy_monitor_notifier_list;
BLOCKING_NOTIFIER_HEAD(S10_psy_monitor_notifier_list);
static int S10_power_monitor_register_notifier(struct notifier_block *nb)
{
    return blocking_notifier_chain_register(&S10_psy_monitor_notifier_list, nb);
}

static int S10_power_monitor_unregister_notifier(struct notifier_block *nb)
{
    return blocking_notifier_chain_register(&S10_psy_monitor_notifier_list, nb);
}

int S10_power_monitor_notifier_call_chain(unsigned long val, void *data)
{
    return blocking_notifier_call_chain(&S10_psy_monitor_notifier_list, val, data);
}

EXPORT_SYMBOL_GPL(S10_power_monitor_notifier_call_chain);

static int S10_psy_monitor_err_detect(void)
{
    return 0;
}

static int S10_psy_battery_infomation_collect(void)
{
    struct S10_psy_monitor_dev_info* monitordev_info;
    struct S10_psy_reporter* reporter;
    struct S10_std_battery_device* battery;
    int ret   = 0;
    int count = 0;

    monitordev_info = S10_power_get_monitor_devinfo();
    reporter = S10_power_get_monitor_reporter();
    if (IS_ERR_OR_NULL(monitordev_info) || IS_ERR_OR_NULL(reporter))
    {
        power_debug(HW_POWER_PSYINFO_REPORT_ERR, "%s: NULL Pointer\n", __func__);
        return -EINVAL;
    }

    for (count = 0; count < reporter->report_battery_num; count++)
    {
        battery = S10_power_get_battery_in_list(reporter->report_battery_name_array[count]);
        if (IS_ERR_OR_NULL(battery))
        {
            continue;
        }

        ret |= S10_psy_battery_dect_info_get(battery, &battery->battery_detect_info);
    }

    return ret;
}

static int S10_psy_charger_information_collect(void)
{
    struct S10_psy_monitor_dev_info* monitordev_info;
    struct S10_psy_reporter* reporter;
    struct S10_std_charger_device* charger;
    int ret = 0;

    monitordev_info = S10_power_get_monitor_devinfo();
    reporter = S10_power_get_monitor_reporter();

    if (IS_ERR_OR_NULL(monitordev_info) || IS_ERR_OR_NULL(reporter))
    {
        power_debug(HW_POWER_PSYINFO_REPORT_ERR, "%s: NULL Pointer\n", __func__);
        return -EINVAL;
    }

    charger = reporter->stdcharger;
    if (IS_ERR_OR_NULL(charger))
    {
        power_debug(HW_POWER_PSYINFO_REPORT_ERR, "%s: NULL Pointer\n", __func__);
        return -EINVAL;
    }

    ret = S10_psy_charger_real_info_get(&charger->charger_real_info);
    return ret;
}

static int S10_psy_monitor_infomation_collect(void)
{
    int ret = 0;

    ret = S10_psy_charger_information_collect();
    power_debug(HW_POWER_PSYINFO_REPORT_DUG, "%s: call charger info collect return %d\n", __func__, ret);

    ret |= S10_psy_battery_infomation_collect();
    power_debug(HW_POWER_PSYINFO_REPORT_DUG, "%s: call battery info collect return %d\n", __func__, ret);

    return ret;
}

/**************************************begin: battery capacity smooth arithmetic *************************************************/

#define BATTERY_CAPACITY_ARRAY_MAX_NUM 5
struct  S10_power_val_array
{
    int val_array[BATTERY_CAPACITY_ARRAY_MAX_NUM];
    int begin_pos;
    int end_pos;
};

struct S10_power_val_array g_capacity_array;

//struct S10_power_val_array g_temp_array;

static void add2array(struct S10_power_val_array* array, int val)
{
    if ((array->end_pos >= array->begin_pos) && (array->end_pos < BATTERY_CAPACITY_ARRAY_MAX_NUM - 1))
    {
        array->val_array[array->end_pos] = val;
        array->end_pos += 1;
    }
    else
    {
        array->val_array[array->end_pos] = val;
        array->begin_pos = (array->begin_pos + 1) % BATTERY_CAPACITY_ARRAY_MAX_NUM;
        array->end_pos = (array->end_pos + 1) % BATTERY_CAPACITY_ARRAY_MAX_NUM;
    }
}

static int check_array_empty(struct S10_power_val_array* array)
{
    if (array->begin_pos == array->end_pos)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

static int get_avg_val_from_array(struct S10_power_val_array* array)
{
    int count = 0, sum = 0, num = 0;

    for (count = array->begin_pos; count != array->end_pos;)
    {
        sum += array->val_array[count];
        num++;
        count = (count + 1) % BATTERY_CAPACITY_ARRAY_MAX_NUM;
    }

    if (num > 0)
    {
        return sum / num;
    }
    else
    {
        return 0;
    }
}

static int get_last_val_from_array(struct S10_power_val_array* array)
{
    int pos = 0;

    if (array->end_pos > 0)
    {
        pos = array->end_pos - 1;
    }
    else
    {
        pos = BATTERY_CAPACITY_ARRAY_MAX_NUM - 1;
    }

    return array->val_array[pos];
}

#define CAPACITY_GAP_TRIGGER 5   // capacity change more than 5% regard as a capacity gap
/* 上报电量稳定的次数 ，到达设定值后，翻转为0重新计数 */
#define BATTERY_INFO_STABLE_MAX_NUM 3
#define BATTERY_INFO_STABLE_MIN_NUM 2

/* 当有跳变时，上报平滑单步的间隔（稳定计数器的翻转次数）*/
#define DALTA_CAPACITY_MIN_REPORT_INTERVAL 0
#define DALTA_CAPACITY_MID_REPORT_INTERVAL 2
#define DALTA_CAPACITY_MAX_REPORT_INTERVAL 10

#define DALTA_VOLTAGE_MIN 100
#define DALTA_VOLTAGE_MID 200
#define DALTA_VOLTAGE_MAX 300

/* 平滑的步进 */
#define CAPACITY_SMOOTH_STEP 1

static int g_low_capacity_step = 0;

static void low_capacity_smooth(int orig_capacity,int * smooth_capacity, int chg_online)
{

    *smooth_capacity = orig_capacity;
    
    if(orig_capacity > 2 || chg_online == 1){
        return ;
    }

    if(g_low_capacity_step <= 1){
        g_low_capacity_step++;
        *smooth_capacity = orig_capacity;
        return ;
    }
        
    *smooth_capacity = (orig_capacity + 1)/2;

    return;
}

static int S10_battery_capacity_and_status_smooth(struct S10_std_charger_device* charger,
                                          struct S10_std_battery_device* battery)
{
    struct S10_psy_reporter* reporter = NULL;
    int avg_capacity = 0, dalta_report_capacity = 0, dalta_capacity = 0, report_capacity = 0;
    int low_capacity = 0, chg_online = 0;
    struct S10_psy_monitor_dev_info* monitordev_info = S10_power_get_monitor_devinfo();
    /*********电池电量平滑开始 *********/
    reporter = S10_power_get_monitor_reporter();
    if (IS_ERR_OR_NULL(reporter) || IS_ERR_OR_NULL(charger) || IS_ERR_OR_NULL(battery))
    {
        power_debug(HW_POWER_PSYINFO_REPORT_ERR, "%s: NULL Pointer\n", __func__);
        return -EINVAL;
    }

    chg_online = charger->charger_real_info.charger_online;
    S10_power_lock_lock(&reporter->data_lock);
    avg_capacity = get_avg_val_from_array(&g_capacity_array);
    report_capacity = get_last_val_from_array(&g_capacity_array);

    if (check_array_empty(&g_capacity_array))
    {
        report_capacity = battery->battery_detect_info.battery_capacity;
        avg_capacity = report_capacity;
        add2array(&g_capacity_array, report_capacity);
        S10_power_lock_unlock(&reporter->data_lock);
        S10_power_lock_lock(&battery->data_lock);

        low_capacity_smooth(report_capacity, &low_capacity, chg_online);
        battery->battery_report_info.battery_capacity = low_capacity;

        S10_power_lock_unlock(&battery->data_lock);
        return 0;
    }

    S10_power_lock_unlock(&reporter->data_lock);
    S10_power_lock_lock(&battery->data_lock);
    dalta_capacity = battery->battery_detect_info.battery_capacity - avg_capacity;

    power_debug(HW_POWER_PSYINFO_REPORT_ERR,
                "%s smoot begin: detc_cp=%d,av_cp=%d,dlt_cp = %d,rp_cp=%d,rp_intval=%d,st_cnt= %d\n",
                battery->battery_name, battery->battery_detect_info.battery_capacity, avg_capacity, dalta_capacity,
                report_capacity,
                battery->battery_smooth_data.dalta_capacity_report_interval,
                battery->battery_smooth_data.capacity_stable_count);
    if (abs(dalta_capacity) <= 1)
    {
        battery->battery_smooth_data.dalta_capacity_report_interval = 0;
        battery->battery_smooth_data.capacity_stable_count = 0;
        report_capacity = battery->battery_detect_info.battery_capacity;
        power_debug(HW_POWER_PSYINFO_REPORT_DUG, "%s: report_capacity = %d \n", __func__,
                    report_capacity);
    }
    else if (abs(dalta_capacity) <= CAPACITY_GAP_TRIGGER)
    {
        if (battery->battery_smooth_data.dalta_capacity_report_interval == 0)
        {
            dalta_report_capacity = (dalta_capacity > 0) ? (CAPACITY_SMOOTH_STEP) : (-CAPACITY_SMOOTH_STEP);
            report_capacity = report_capacity + dalta_report_capacity;
            battery->battery_smooth_data.dalta_capacity_report_interval = DALTA_CAPACITY_MAX_REPORT_INTERVAL;
        }
        else
        {
            battery->battery_smooth_data.dalta_capacity_report_interval--;
        }

        battery->battery_smooth_data.capacity_stable_count = 0;

        power_debug(HW_POWER_PSYINFO_REPORT_DUG, "%s: report_capacity = %d  \n", __func__, report_capacity);
    }
    else
    {
        if (charger->charger_track_info.charger_online != charger->charger_real_info.charger_online)
        {
#if 0
            if (dalta_capacity > 0) // capacity increase
            {
                if (charger->charger_track_info.charger_online == 1)  //plug out charger, discard this collected data
                {
                    battery->battery_smooth_data.dalta_capacity_report_interval = 0;
                    battery->battery_smooth_data.capacity_stable_count = 0;
                }
                else //plug in charger
                {
                    battery->battery_smooth_data.dalta_capacity_report_interval = DALTA_CAPACITY_MID_REPORT_INTERVAL;
                }
            }
            else // capacity decrease
            {
                if (charger->charger_track_info.charger_online == 1)  //plug out charger
                {
                    battery->battery_smooth_data.dalta_capacity_report_interval = DALTA_CAPACITY_MIN_REPORT_INTERVAL;
                }
                else //plug in charger, , discard this collected data
                {
                    battery->battery_smooth_data.dalta_capacity_report_interval = 0;
                    battery->battery_smooth_data.capacity_stable_count = 0;
                }
            }

#else
            battery->battery_smooth_data.dalta_capacity_report_interval = 0;
            battery->battery_smooth_data.capacity_stable_count = 0;
            power_debug(HW_POWER_PSYINFO_REPORT_DUG, "%s: report_capacity = %d  \n", __func__, report_capacity);
#endif
        }
        else
        {
            /* 重新设置上报的间隔数 */
            if ((battery->battery_smooth_data.dalta_capacity_report_interval == 0) && (battery->battery_smooth_data.capacity_stable_count == 0))
            {
                if (abs(dalta_capacity) < CAPACITY_GAP_TRIGGER * 2)
                {
                    battery->battery_smooth_data.dalta_capacity_report_interval = DALTA_CAPACITY_MID_REPORT_INTERVAL;
                }
                else
                {
                    battery->battery_smooth_data.dalta_capacity_report_interval = DALTA_CAPACITY_MIN_REPORT_INTERVAL;
                }

                dalta_report_capacity = 0;
            }
            /* 上报间隔数自减，同时修改上报 */
            else if (battery->battery_smooth_data.capacity_stable_count == BATTERY_INFO_STABLE_MAX_NUM - 1)
            {
                if (battery->battery_smooth_data.dalta_capacity_report_interval == 0)
                {
                    dalta_report_capacity = (dalta_capacity > 0) ? (CAPACITY_SMOOTH_STEP) : (-CAPACITY_SMOOTH_STEP);
                    if (abs(dalta_capacity) < CAPACITY_GAP_TRIGGER * 2)
                    {
                        battery->battery_smooth_data.dalta_capacity_report_interval =
                            DALTA_CAPACITY_MID_REPORT_INTERVAL;
                    }
                    else
                    {
                        battery->battery_smooth_data.dalta_capacity_report_interval =
                            DALTA_CAPACITY_MIN_REPORT_INTERVAL;
                    }
                }
                else
                {
                    battery->battery_smooth_data.dalta_capacity_report_interval--;
                    if (battery->battery_smooth_data.dalta_capacity_report_interval == 0)
                    {
                        dalta_report_capacity = (dalta_capacity > 0) ? (CAPACITY_SMOOTH_STEP) : (-CAPACITY_SMOOTH_STEP);                   
                    }
                }
            }

            battery->battery_smooth_data.capacity_stable_count = (battery->battery_smooth_data.capacity_stable_count
                                                                  + 1) % BATTERY_INFO_STABLE_MAX_NUM;
        }

        report_capacity = report_capacity + dalta_report_capacity;
        power_debug(HW_POWER_PSYINFO_REPORT_DUG,
                    "%s: report_capacity = %d ,dalta_capacity_report_interval = %d,capacity_stable_count= %d\n",
                    __func__,
                    report_capacity, battery->battery_smooth_data.dalta_capacity_report_interval,
                    battery->battery_smooth_data.capacity_stable_count);
    }



    /* 电池在放电状态时，电量不上升 */
    if ((battery->battery_report_info.battery_current < 0) && (dalta_report_capacity > 0))
    {
        report_capacity = report_capacity - dalta_report_capacity;
    }

    /* 电池在充电状态时，电量不下降 */
    if ((battery->battery_report_info.battery_current > 0) && (dalta_report_capacity < 0))
    {
        report_capacity = report_capacity - dalta_report_capacity;
    }

    low_capacity_smooth(report_capacity, &low_capacity, chg_online);
    battery->battery_report_info.battery_capacity = low_capacity;

    power_debug(HW_POWER_PSYINFO_REPORT_ERR,
                "%s smoot end  : detc_cp=%d,av_cp=%d,dlt_cp = %d,rp_cp=%d,rp_intval=%d,st_cnt= %d\n",
                battery->battery_name, battery->battery_detect_info.battery_capacity, avg_capacity, dalta_capacity,
                report_capacity,
                battery->battery_smooth_data.dalta_capacity_report_interval,
                battery->battery_smooth_data.capacity_stable_count);

    power_debug(HW_POWER_PSYINFO_REPORT_DUG, "%s: report_capacity = %d, avg_capacity = %d  5555555555\n", __func__,
                report_capacity, avg_capacity);

    if (report_capacity != avg_capacity)
    {
        S10_power_lock_lock(&reporter->data_lock);
        add2array(&g_capacity_array, report_capacity);
        S10_power_lock_unlock(&reporter->data_lock);
    }
    /*********电池电量平滑结束 *********/

    /*********根据平滑后的电量修正充电状态开始 *********/
    if (!battery->battery_report_info.battery_present)
    {
        battery->battery_report_info.battery_status= POWER_SUPPLY_STATUS_UNKNOWN;
    }
    else if (battery->battery_report_info.battery_capacity< 100)
    {
        S10_power_lock_lock(&charger->data_lock);
        if (charger->charger_real_info.charger_online)
        {
            if ((((POWER_SUPPLY_TYPE_USB == charger->charger_real_info.charger_type)
                  || (POWER_SUPPLY_TYPE_USB_CDP == charger->charger_real_info.charger_type))
                 && monitordev_info->usb_charging_display_support && monitordev_info->usb_charging_support)
                || (POWER_SUPPLY_TYPE_USB_ACA == charger->charger_real_info.charger_type)
                || (POWER_SUPPLY_TYPE_USB_DCP == charger->charger_real_info.charger_type))
            {
                battery->battery_report_info.battery_status = POWER_SUPPLY_STATUS_CHARGING;
            }
            else
            {
                battery->battery_report_info.battery_status = POWER_SUPPLY_STATUS_NOT_CHARGING;
            }
        }
        else
        {
            battery->battery_report_info.battery_status = POWER_SUPPLY_STATUS_DISCHARGING;
        }
        S10_power_lock_unlock(&charger->data_lock);
    }
    else
    {
        battery->battery_report_info.battery_status = POWER_SUPPLY_STATUS_FULL;
    }
    S10_power_lock_unlock(&battery->data_lock);
    
    power_debug(HW_POWER_BATTERY_IC_DUG, "[Power ERR: %s, %s, %d, POWER_SUPPLY_PROP_STATUS val = %d]  \n", __FILE__,
                __func__, __LINE__, battery->battery_report_info.battery_status);

    /*********根据平滑后的电量修正充电状态结束 *********/
    
    return 0;
}

static int S10_battery_smooth(struct S10_std_charger_device* charger, struct S10_std_battery_device* battery)
{
    int ret = -EINVAL;

    power_debug(HW_POWER_PSYINFO_REPORT_DUG, "%s: \n", __func__);
    if (IS_ERR_OR_NULL(charger) || IS_ERR_OR_NULL(battery))
    {
        power_debug(HW_POWER_PSYINFO_REPORT_ERR, "%s: NULL Pointer\n", __func__);
        return ret;
    }

    // just for test
    S10_power_lock_lock(&battery->data_lock);
    memcpy(&battery->battery_track_info, &battery->battery_report_info, sizeof(struct battery_psy_info));
    memcpy(&battery->battery_report_info, &battery->battery_detect_info, sizeof(struct battery_psy_info));
    S10_power_lock_unlock(&battery->data_lock);
    ret = S10_battery_capacity_and_status_smooth(charger, battery);
    return ret;
}

static int S10_psy_battery_info_smooth(void)
{
    struct S10_psy_monitor_dev_info* monitordev_info;
    struct S10_psy_reporter* reporter;
    struct S10_std_charger_device* charger;
    struct S10_std_battery_device* battery;
    int count = 0;
    int ret = -EINVAL;

    monitordev_info = S10_power_get_monitor_devinfo();
    reporter = S10_power_get_monitor_reporter();

    power_debug(HW_POWER_PSYINFO_REPORT_DUG, "%s: \n", __func__);
    if (IS_ERR_OR_NULL(monitordev_info) || IS_ERR_OR_NULL(reporter))
    {
        power_debug(HW_POWER_PSYINFO_REPORT_ERR, "%s: NULL Pointer\n", __func__);
        return ret;
    }

    charger = reporter->stdcharger;
    if (IS_ERR_OR_NULL(charger))
    {
        power_debug(HW_POWER_PSYINFO_REPORT_ERR, "%s: NULL charger\n", __func__);
        return ret;
    }

    for (count = 0; count < reporter->report_battery_num; count++)
    {
        battery = S10_power_get_battery_in_list(reporter->report_battery_name_array[count]);
        if (IS_ERR_OR_NULL(battery))
        {
            continue;
        }

        ret |= S10_battery_smooth(charger, battery);
    }

    return ret;
}

static int S10_psy_charger_info_smooth(void)
{
    struct S10_psy_reporter* reporter;
    struct S10_std_charger_device* charger;
    int ret = 0;

    reporter = S10_power_get_monitor_reporter();
    if (IS_ERR_OR_NULL(reporter))
    {
        power_debug(HW_POWER_PSYINFO_REPORT_ERR, "%s: NULL Pointer\n", __func__);

        //	return ret;
    }

    charger = reporter->stdcharger;
    if (IS_ERR_OR_NULL(charger))
    {
        power_debug(HW_POWER_PSYINFO_REPORT_ERR, "%s: NULL charger\n", __func__);

        //	return ret;
    }

    S10_power_lock_lock(&charger->data_lock);
    memcpy(&charger->charger_track_info, &charger->charger_report_info, sizeof(struct charger_psy_info));
    memcpy(&charger->charger_report_info, &charger->charger_real_info, sizeof(struct charger_psy_info));
    S10_power_lock_unlock(&charger->data_lock);
    return ret;
}

static int S10_monitor_info_smootch(void)
{
    int ret = -EINVAL;

    power_debug(HW_POWER_PSYINFO_REPORT_DUG, "%s: \n", __func__);
    ret  = S10_psy_charger_info_smooth();
    ret |= S10_psy_battery_info_smooth();
    return ret;
}

/*****************************************************************************
 函 数 名  : std_monitor_reg_dump
 功能描述  : 打印充电相关芯片寄存器数值
 输入参数  : void
 输出参数  : 无
 返 回 值  : 无
 调用函数  : 略
 被调函数  : S10_psy_monitor_work
 
 修改历史      :
  1.日    期   : 2012年11月23日
    作    者   : l00220156
    修改内容   : 新生成函数

*****************************************************************************/
static void std_monitor_reg_dump(void)
{
    int icount = 0;
    struct S10_psy_reporter* reporter;
    struct S10_std_battery_device* battery;

    /* 库仑计信息打印开始 */
    if (HW_POWER_COUL_REG_DUMP == (power_debug_mask & HW_POWER_COUL_REG_DUMP))
    {
        power_debug(HW_POWER_PSYINFO_REPORT_DUG, "%s: \n", __func__);
        reporter = S10_power_get_monitor_reporter();
        if (IS_ERR_OR_NULL(reporter))
        {
            power_debug(HW_POWER_PSYINFO_REPORT_ERR, "%s: NULL Pointer\n", __func__);
            return;
        }

        for (icount = 0; icount < reporter->report_battery_num; icount++)
        {
            battery = S10_power_get_battery_in_list(reporter->report_battery_name_array[icount]);
            if (IS_ERR_OR_NULL(battery))
            {
                continue;
            }

            if (battery->ops->battery_reg_dump)
            {
                battery->ops->battery_reg_dump();
            }
            else
            {
                power_debug(HW_POWER_PSYINFO_REPORT_ERR, "%s: battery_reg_dump NULL Pointer\n", __func__);
            }
        }
    }
    else
    {
        return ;
    }
    /* 库仑计信息打印结束 */
}

/**************************************end: battery capacity smooth arithmetic *************************************************/

static enum S10_power_monitor_report_type S10_psy_battery_information_report_filter(
    struct S10_std_battery_device* battery)
{
	int ret = PSY_NOT_REPORT;
    int battery_report_flag = PSY_NOT_REPORT;
    if (IS_ERR_OR_NULL(battery))
    {
        power_debug(HW_POWER_PSYINFO_REPORT_ERR, "%s: NULL Pointer\n", __func__);
        return ret;
    }
    
    battery_report_flag = (abs(battery->battery_track_info.battery_capacity - battery->battery_report_info.battery_capacity) >= DELTA_CAPACITY_REPORT) ||\
                  (abs(battery->battery_track_info.battery_temperature- battery->battery_report_info.battery_temperature) >= DELTA_TEMP_REPORT) ||\
                  (battery->battery_track_info.battery_present != battery->battery_report_info.battery_present) ||\
                  (battery->battery_track_info.battery_health!= battery->battery_report_info.battery_health) ||\
                  (battery->battery_track_info.battery_status!= battery->battery_report_info.battery_status);
    if(battery_report_flag)
        ret = BATTERY_INFO_REPORT;
    return ret;
}

static enum S10_power_monitor_report_type S10_psy_charger_information_report_filter(
    struct S10_std_charger_device* charger)
{
    int ret = PSY_NOT_REPORT;
    int charger_report_flag = PSY_NOT_REPORT;
    charger_report_flag = (charger->charger_track_info.charger_online != charger->charger_real_info.charger_online) ||\
        (charger->charger_track_info.charger_health != charger->charger_real_info.charger_health) ||\
        (charger->charger_track_info.charging_status != charger->charger_real_info.charging_status) ||
        (charger->charger_track_info.charger_type!= charger->charger_real_info.charger_type);
    
    if(charger_report_flag)
    {
        ret = CHARGER_INFO_REPORT;
    }
    return ret;
        
}

#if 0
// 预留接口
static int S10_battery_psy_report(const unsigned char* battery_name)
{
    struct S10_psy_node* hwpsy = NULL;

    power_debug(HW_POWER_PSYINFO_REPORT_ERR, "%s: \n", __func__);
    hwpsy = get_psy_in_list(battery_name);
    if (IS_ERR_OR_NULL(hwpsy))
    {
        return -EINVAL;
    }

    power_supply_changed(&hwpsy->psy);
    return 0;
}
#endif

static int S10_all_battery_psy_report(void)
{
    struct S10_psy_reporter* reporter;
    struct S10_std_battery_device* battery;
    struct S10_psy_node* hwpsy = NULL;
    int count = 0;

    reporter = S10_power_get_monitor_reporter();
    power_debug(HW_POWER_PSYINFO_REPORT_DUG, "%s: \n", __func__);
    if (IS_ERR_OR_NULL(reporter))
    {
        power_debug(HW_POWER_PSYINFO_REPORT_ERR, "%s: NULL Pointer\n", __func__);
        return -EINVAL;
    }

    for (count = 0; count < reporter->report_battery_num; count++)
    {
        battery = S10_power_get_battery_in_list(reporter->report_battery_name_array[count]);
        if (IS_ERR_OR_NULL(battery))
        {
            power_debug(HW_POWER_PSYINFO_REPORT_ERR, "%s: NULL Pointer\n", __func__);
            continue;
        }

        if (BATTERY_INFO_REPORT == S10_psy_battery_information_report_filter(battery))
        {
            hwpsy = get_psy_in_list(reporter->report_battery_name_array[count]);
            if (IS_ERR_OR_NULL(hwpsy))
            {
                power_debug(HW_POWER_PSYINFO_REPORT_ERR, "%s: %s psy can't found \n", __func__,
                            battery->battery_name);
                continue;
            }

            power_debug(HW_POWER_PSYINFO_REPORT_DUG, "%s: power_supply_changed\n", __func__);
            power_supply_changed(&hwpsy->psy);
        }
    }

    return 0;
}

//充电器状态信息上报
static int S10_charger_psy_report(void)
{
    struct S10_psy_reporter* reporter;
    struct S10_std_charger_device* charger;

    struct S10_psy_monitor_dev_info* monitordev_info = NULL;

    power_debug(HW_POWER_PSYINFO_REPORT_DUG, "%s: \n", __func__);

    monitordev_info = S10_power_get_monitor_devinfo();
    reporter = S10_power_get_monitor_reporter();
    if (IS_ERR_OR_NULL(monitordev_info) || IS_ERR_OR_NULL(reporter))
    {
        power_debug(HW_POWER_PSYINFO_REPORT_ERR, "%s: NULL Pointer\n", __func__);
        return -EINVAL;
    }

    charger = reporter->stdcharger;
    if (IS_ERR_OR_NULL(charger))
    {
        power_debug(HW_POWER_PSYINFO_REPORT_ERR, "%s: NULL Pointer\n", __func__);
        return -EINVAL;
    }

    if (CHARGER_INFO_REPORT == S10_psy_charger_information_report_filter(charger))
    {
        if (IS_ERR_OR_NULL(charger->std_psy_ac))
        {
            power_debug(HW_POWER_PSYINFO_REPORT_ERR, "%s: %s psy can't find\n", __func__,
                        reporter->stdcharger->name);
            return -EINVAL;
        }

        power_supply_changed(charger->std_psy_ac);
        power_debug(HW_POWER_PSYINFO_REPORT_DUG, "%s: charger->std_psy_ac \n", __func__);

        if (monitordev_info->usb_charging_support)
        {
            if (IS_ERR_OR_NULL(charger->std_psy_usb))
            {
                power_debug(HW_POWER_PSYINFO_REPORT_ERR, "%s: %s psy can't find\n", __func__,
                            reporter->stdcharger->name);
                return -EINVAL;
            }

            power_supply_changed(charger->std_psy_usb);
            power_debug(HW_POWER_PSYINFO_REPORT_DUG, "%s: charger->std_psy_usb \n", __func__);
        }
    }

    return 0;
}

static void S10_psy_monitor_infomation_report(void)
{
    S10_charger_psy_report();
    S10_all_battery_psy_report();
    return;
}

void S10_psy_monitor_start(unsigned int delay_ms)
{
    struct S10_psy_reporter* reporter;

    reporter = S10_power_get_monitor_reporter();
    if (reporter->psy_monitor_running_flag)
    {
        return;
    }

    queue_delayed_work(reporter->psy_monitor_wq, &reporter->psy_monitor_work, msecs_to_jiffies(delay_ms));
    reporter->psy_monitor_running_flag = 1;
    return;
}

EXPORT_SYMBOL_GPL(S10_psy_monitor_start);

void S10_psy_monitor_stop(void)
{
    struct S10_psy_reporter* reporter;

    reporter = S10_power_get_monitor_reporter();

    if (!reporter->psy_monitor_running_flag)
    {
        return;
    }
    //cancel_work_sync(&reporter->psy_notifier_work);
    cancel_delayed_work_sync(&reporter->psy_notifier_work);
    cancel_delayed_work_sync(&reporter->psy_monitor_work);
    cancel_delayed_work_sync(&reporter->vbat_low_dect_work);
    flush_workqueue(reporter->psy_monitor_wq);
    reporter->psy_monitor_running_flag = 0;
    return;
}

EXPORT_SYMBOL_GPL(S10_psy_monitor_stop);

/*****************************************************************************
 函 数 名  :  S10_psy_repeat_charging_dect
 功能描述  : 复充，当电量低于repeat_charging_capacity时，启动复充
 输入参数  : repeat_charging_capacity，复充电量门限
 输出参数  : 无
 返 回 值  : 无
 调用函数  : 略
 被调函数  : 
 
 修改历史      :
  1.日    期   : 2012年10月10日
    作    者   : l00220156
    修改内容   : 修改函数实现

*****************************************************************************/
static void S10_psy_repeat_charging_dect(int repeat_charging_capacity)
{
    struct S10_psy_reporter* reporter;
    struct S10_std_battery_device * battery;
    struct S10_std_charger_device * charger;
    int find = 0;

    reporter = S10_power_get_monitor_reporter();
    if (IS_ERR_OR_NULL(reporter))
    {
        return ;
    }

    S10_power_lock_lock(&reporter->battery_list_lock);
    if (!list_empty(&reporter->battery_list))
    {
        list_for_each_entry(battery, &reporter->battery_list, node)
        {
            if (battery->battery_repeat_charging_flag)
            {
                find = 1;
                break;
            }
        }
    }

    S10_power_lock_unlock(&reporter->battery_list_lock);

    if (find)
    {
        charger = reporter->stdcharger;
        if ((battery->battery_report_info.battery_status == POWER_SUPPLY_STATUS_FULL)
            && charger->charger_online)
        {
            reporter->psy_repeat_charging_detect_enable = 1;
        }
        
        /* 有过慢充状态，且充电器在位 */
        if (reporter->psy_repeat_charging_detect_enable)
        {
            if (repeat_charging_capacity <= 0)
            {
                repeat_charging_capacity = reporter->dev_info->psy_repeat_charging_capacity;
            }

            if (battery->battery_report_info.battery_capacity <= repeat_charging_capacity)
            {
                reporter->psy_repeat_charging_detect_enable = 0;
                printk("S10_psy_repeat_charging_dect"); 
                charger->ops->charging_enable(charger, 0);
                msleep(1000);
                charger->ops->charging_enable(charger, 1);
            }
        }
    }
}

static void S10_factory_charging_dect(void)
{
    struct S10_psy_reporter* reporter;
    struct S10_std_battery_device * battery;
    struct S10_std_charger_device * charger;
    int find = 0, report_capacity = 0, factory_full_capacity = 0;

    reporter = S10_power_get_monitor_reporter();
    charger = reporter->stdcharger;
    if (reporter->factory_charging_mode_enable && charger->charger_online && !reporter->factory_charging_stop_flag)
    {
        S10_power_lock_lock(&reporter->battery_list_lock);
        if (!list_empty(&reporter->battery_list))
        {
            list_for_each_entry(battery, &reporter->battery_list, node)
            {
                if (battery->battery_repeat_charging_flag)
                {
                    find = 1;
                    break;
                }
            }
        }

        S10_power_lock_unlock(&reporter->battery_list_lock);

        if (find)
        {
            S10_power_lock_lock(&battery->data_lock);
            report_capacity = battery->battery_report_info.battery_capacity;
            S10_power_lock_unlock(&battery->data_lock);
            S10_power_lock_lock(&reporter->data_lock);
            factory_full_capacity = reporter->factory_charging_full_capacity;
            S10_power_lock_unlock(&reporter->data_lock);
            if (report_capacity >= factory_full_capacity)
            {
                charger->ops->charging_enable(charger, 0);
                S10_power_lock_lock(&reporter->data_lock);
                reporter->factory_charging_stop_flag = 1;
                S10_power_lock_unlock(&reporter->data_lock);
            }
        }
    }
}


/*****************************************************************************
 函 数 名  :  S10_charge_tempreture_protect
 功能描述  : 温度保护监测,根据温度等级设定充电的电流,充电状态的修正需结合电量平滑处实现，
 此处对平滑后的状态进行再次修正。
 输入参数  : 无
 输出参数  : 无
 返 回 值  : 无
 调用函数  : 略
 被调函数  : S10_psy_monitor_work

 修改历史      :
  1.日    期   : 2012年10月9日
    作    者   : l00220156
    修改内容   : 新生成函数

*****************************************************************************/
static void S10_charge_tempreture_protect(void)
{

    struct S10_psy_reporter* reporter;
    struct S10_std_battery_device * battery;
    struct S10_std_charger_device * charger;
    
    int icount = 0;
    int iTemperature = 0;
    unsigned int iVoltage  = 0;
    unsigned int iCapacity = 0;
    int factory_current_set_flag = 0;

    power_debug(HW_POWER_PSYINFO_REPORT_DUG, "[Power DUG: %s, %s, %d]  \n", __FILE__, __func__, __LINE__);
    reporter = S10_power_get_monitor_reporter();
    if (IS_ERR_OR_NULL(reporter))
    {
        return ;
    }

    for (icount = 0; icount < reporter->report_battery_num; icount++)
    {
        battery = S10_power_get_battery_in_list(reporter->report_battery_name_array[icount]);
        if (IS_ERR_OR_NULL(battery))
        {
            continue;
        }

        S10_power_lock_lock(&battery->data_lock);
        if (!battery->battery_report_info.battery_present)
        {
            S10_power_lock_unlock(&battery->data_lock);
            continue;
        }

        iTemperature = battery->battery_report_info.battery_temperature;
        iVoltage  = battery->battery_report_info.battery_voltage;
        iCapacity = battery->battery_report_info.battery_capacity;
        S10_power_lock_unlock(&battery->data_lock);

        charger = reporter->stdcharger;
        if (!(reporter->stdcharger->charger_online))
        {
            continue;
        }

        /* 若温度小于0度则不充电 */
        if (iTemperature < THREASHOLD_COLD)
        {
            power_debug(HW_POWER_PSYINFO_REPORT_DUG, "[Power DUG: %s, %s, %d]  \n", __FILE__, __func__, __LINE__);
            S10_power_lock_lock(&battery->data_lock);
            battery->temperature_status = TEMP_LOW;
            battery->battery_report_info.battery_status = POWER_SUPPLY_STATUS_NOT_CHARGING;
            S10_power_lock_unlock(&battery->data_lock);
            reporter->stdcharger->ops->charging_enable((void*)reporter->stdcharger, 0);
            continue;
        }

        /* 若温度大于0度小于10度 */
        if ((iTemperature >= THREASHOLD_COLD) && (iTemperature < THRESHOLD_COOL))
        {
            /* 当电池温度从非正常温度区间(<0度 或 >45度)，恢复到正常温度区间(0-10度) */
            /* 如果不等于正常温度状态,说明之前温度过高过，或过低过，此处应为过低过。*/
            /* 如需加温度过低，再次充电温度设置，请在此处添加。*/
            power_debug(HW_POWER_PSYINFO_REPORT_DUG, "[Power DUG: %s, %s, %d]  \n", __FILE__, __func__, __LINE__);
            if ((TEMP_NOMAL != battery->temperature_status))
            {
                if (iTemperature > (THREASHOLD_COLD + DELTA_TEMP))
                {
                    S10_power_lock_lock(&battery->data_lock);
                    battery->temperature_status = TEMP_NOMAL;
                    S10_power_lock_unlock(&battery->data_lock);

                    /* psy_repeat_charging_detect_enable为1的情况，由复充函数处理，为0说明不在复充阶段内。 */
                    if (0 == reporter->psy_repeat_charging_detect_enable)
                    {
                        reporter->stdcharger->ops->charging_enable((void*)reporter->stdcharger, 1);
                    }
                }
                else
                {
                    S10_power_lock_lock(&battery->data_lock);
                    battery->battery_report_info.battery_status = POWER_SUPPLY_STATUS_NOT_CHARGING;
                    S10_power_lock_unlock(&battery->data_lock);
                }
            }

            /* 设置充电电流为550mA ,但不一定充电 */
            if (charger->ops->charging_max_charging_current_set)
            {
                charger->ops->charging_max_charging_current_set((void*)reporter->stdcharger, 550);
            }

            continue;
        }

        /* 若温度大于10度，小于45度 */
        if ((iTemperature >= THRESHOLD_COOL) && (iTemperature < THRESHOLD_WARM))
        {
            power_debug(HW_POWER_PSYINFO_REPORT_DUG, "[Power DUG: %s, %s, %d]  \n", __FILE__, __func__, __LINE__);

            /* 当电池温度从非正常温度区间(<0度 或 >45度)，恢复到正常温度区间(10-45度) */
            /* 如果不等于正常温度状态,说明之前温度过高过，或过低过，此处应为过高过。*/
            /* 如需加温度过高，再次充电温度设置，请在此处添加,2度的变化门限。*/
            if (TEMP_NOMAL != battery->temperature_status)
            {
                if (iTemperature < (THRESHOLD_WARM - DELTA_TEMP))
                {
                    S10_power_lock_lock(&battery->data_lock);
                    battery->temperature_status = TEMP_NOMAL;
                    S10_power_lock_unlock(&battery->data_lock);

                    if (0 == reporter->psy_repeat_charging_detect_enable)
                    {
                        reporter->stdcharger->ops->charging_enable((void*)reporter->stdcharger, 1);
                    }
                }
                else
                {
                    S10_power_lock_lock(&battery->data_lock);
                    battery->battery_report_info.battery_status = POWER_SUPPLY_STATUS_NOT_CHARGING;
                    S10_power_lock_unlock(&battery->data_lock);
                }
            }

            /* 产线充电电流设置时，不自动改变充电电流值 */
            factory_current_set_flag = reporter->factory_charging_current_set_flag;
            if (0 == factory_current_set_flag)
            {
                power_debug(HW_POWER_PSYINFO_REPORT_DUG, "[Power DUG: %s, %s, %d]  \n", __FILE__, __func__, __LINE__);

                /* 设置充电电流为最大值2000mA，,但不一定充电，设置函数根据充电器类型自己调整为对应最大充电电流 */
                if (reporter->stdcharger->ops->charging_max_charging_current_set)
                {
                    reporter->stdcharger->ops->charging_max_charging_current_set((void*)reporter->stdcharger, 2000);
                }
            }

            continue;
        }

        /* 温度大于45度 ，停止充电 */
        if (iTemperature >= THRESHOLD_WARM)
        {
            power_debug(HW_POWER_PSYINFO_REPORT_DUG, "[Power DUG: %s, %s, %d]  \n", __FILE__, __func__, __LINE__);
            S10_power_lock_lock(&battery->data_lock);
            battery->temperature_status = TEMP_HIGH;
            battery->battery_report_info.battery_status = POWER_SUPPLY_STATUS_NOT_CHARGING;
            S10_power_lock_unlock(&battery->data_lock);
            reporter->stdcharger->ops->charging_enable((void*)reporter->stdcharger, 0);
            continue;
        }
    }

    return;
}

static void S10_psy_monitor_work(struct work_struct *work)
{
    struct S10_psy_reporter* reporter;
    struct S10_psy_monitor_dev_info* monitordev_info = NULL;

    power_debug(HW_POWER_PSYINFO_REPORT_DUG, "%s: \n", __func__);
    reporter = S10_power_get_monitor_reporter();
    monitordev_info = S10_power_get_monitor_devinfo();
    if (IS_ERR_OR_NULL(monitordev_info) || IS_ERR_OR_NULL(reporter))
    {
        if (S10_psy_monitor_err_detect())
        {
            return;
        }

        goto call_next_work;
    }

    if (S10_psy_monitor_infomation_collect())
    {
        if (S10_psy_monitor_err_detect())
        {
            return;
        }

        goto call_next_work;
    }

    std_monitor_reg_dump();
    S10_monitor_info_smootch();
    S10_psy_repeat_charging_dect(-1);
    S10_factory_charging_dect();

    S10_charge_tempreture_protect();
    S10_psy_monitor_infomation_report();
call_next_work:
    queue_delayed_work(reporter->psy_monitor_wq, &reporter->psy_monitor_work, msecs_to_jiffies(1000
                                                                                               * monitordev_info->monitoring_interval));
    return;
}

static void S10_psy_lowbattery_work(struct work_struct *work)
{
    return;
}

static void S10_psy_notifier_event_work(struct work_struct *work)
{
    S10_psy_monitor_infomation_collect();
    S10_monitor_info_smootch();
    S10_charge_tempreture_protect();
    S10_psy_monitor_infomation_report();
}

static int S10_psy_monitor_event(struct notifier_block *nb, unsigned long event,
                                    void *_data)
{
    struct S10_psy_reporter* reporter;

    reporter = S10_power_get_monitor_reporter();

    if (event == CHG_CHARGER_PULGOUT_EVENT)
    {
        reporter->psy_repeat_charging_detect_enable = 0;
    }

    if ((event < S10_PWR_NOTIFY_EVENT_MAX) && (reporter->psy_monitor_running_flag))
    {
        //queue_work(reporter->psy_monitor_wq, &reporter->psy_notifier_work);
        queue_delayed_work(reporter->psy_monitor_wq, &reporter->psy_notifier_work, msecs_to_jiffies(5));
    }

    return 0;
}

static int S10_monitor_first_detect(void)
{
    struct S10_psy_reporter* reporter;
    int ret = -EINVAL;

    reporter = S10_power_get_monitor_reporter();
    if (IS_ERR_OR_NULL(reporter))
    {
        power_debug(HW_POWER_PSYINFO_REPORT_ERR, "%s: NULL Pointer\n", __func__);
        return ret;
    }

    /* begin first detect, inital all data struct */

    /* end  first detect, inital all data struct */
    S10_psy_monitor_start(S10_PWR_MONITOR_FIRST_REPORT_DELAY);
    return 0;
}

static int __devinit S10_psy_monitor_probe(struct platform_device *pdev)
{
    struct S10_psy_monitor_dev_info* monitordev_info = NULL;
    struct S10_psy_reporter* reporter = NULL;
    int ret = -EINVAL;

    monitordev_info = pdev->dev.platform_data;
    if (IS_ERR_OR_NULL(monitordev_info))
    {
        power_debug(HW_POWER_PSYINFO_REPORT_ERR, "%s: get platformdata NULL\n", __func__);
        goto err_devinfo_null;
    }

    reporter = kzalloc(sizeof(*reporter), GFP_KERNEL);
    if (IS_ERR_OR_NULL(reporter))
    {
        power_debug(HW_POWER_PSYINFO_REPORT_ERR, "%s: alloc reporter NULL\n", __func__);
        goto err_reporter_null;
    }

    //	reporter->dev = &pdev->dev;

    INIT_LIST_HEAD(&reporter->battery_list);
    INIT_LIST_HEAD(&reporter->powersupply_list);
    S10_power_lock_init(S10_POWER_LOCK_LIGHT, &reporter->battery_list_lock);
    S10_power_lock_init(S10_POWER_LOCK_LIGHT, &reporter->psy_list_lock);
    S10_power_lock_init(S10_POWER_LOCK_LIGHT, &reporter->data_lock);

    wake_lock_init(&reporter->psy_reporter_wake_lock, WAKE_LOCK_SUSPEND, "S10_psy_monitor.0");

    if (monitordev_info->monitoring_interval == 0)
    {
        monitordev_info->monitoring_interval = S10_PWR_MONITOR_INTERVAL_PSY;
    }

    reporter->dev_info = monitordev_info;

    ret = sysfs_create_group(&pdev->dev.kobj, &S10_power_attr_group);
    if (ret)
    {
        goto err_create_sysfs;
    }

    reporter->report_battery_num = 0;
    reporter->factory_charging_full_capacity = DEFAULT_FACTORY_CHARGING_FULL_CAPACITY;
    reporter->psy_monitor_shutdown_charging_flag = 0;

    reporter->factory_charging_current_set_flag = 0;
    reporter->power_monitor_charge_mode_flag = CHARGE_MODE_NORMAL;
    memset(reporter->report_battery_name_array, 0, S10_REPORT_BATTERY_MAX_NUM * BATTERY_NAME_LEN);

    platform_set_drvdata(pdev, reporter);
    monitor_report_dev = pdev;

    reporter->psy_monitor_wq = create_freezable_workqueue(S10_PWR_MONITOR_WQ_NAME);
    if (IS_ERR_OR_NULL(reporter->psy_monitor_wq))
    {
        goto err_create_monitor_wq;
    }
    //INIT_WORK(&reporter->psy_notifier_work, S10_psy_notifier_event_work);
    INIT_DELAYED_WORK_DEFERRABLE(&reporter->psy_notifier_work,
                                 S10_psy_notifier_event_work);
    INIT_DELAYED_WORK_DEFERRABLE(&reporter->psy_monitor_work,
                                 S10_psy_monitor_work);
    INIT_DELAYED_WORK_DEFERRABLE(&reporter->vbat_low_dect_work,
                                 S10_psy_lowbattery_work);

    reporter->psy_monitor_nb.notifier_call = S10_psy_monitor_event;
    ret = S10_power_monitor_register_notifier(&reporter->psy_monitor_nb);
    if (ret)
    {
        power_debug(HW_POWER_PSYINFO_REPORT_ERR, "%s: register notifier failed!\n", __func__);
        goto err_register_notifier;
    }

    S10_monitor_first_detect();
    power_debug(HW_POWER_PSYINFO_REPORT_DUG, "%s: psy monitor probe succeed!\n", __func__);
    return ret;

err_register_notifier:
    destroy_workqueue(reporter->psy_monitor_wq);
err_create_monitor_wq:
    monitor_report_dev = NULL;
    platform_set_drvdata(pdev, NULL);
    sysfs_remove_group(&pdev->dev.kobj, &S10_power_attr_group);
err_create_sysfs:
    S10_power_lock_deinit(&reporter->battery_list_lock);
    S10_power_lock_deinit(&reporter->psy_list_lock);
    S10_power_lock_deinit(&reporter->data_lock);

    kfree(reporter);
    reporter = NULL;

err_reporter_null:
err_devinfo_null:
    power_debug(HW_POWER_PSYINFO_REPORT_ERR, "%s: probe err with errcode [%d]\n", __func__, ret);
    return ret;
}

static int __devexit S10_psy_monitor_remove(struct platform_device *pdev)
{
    struct S10_psy_reporter* reporter;

    reporter = (struct S10_psy_reporter*)platform_get_drvdata(pdev);

    sysfs_remove_group(&pdev->dev.kobj, &S10_power_attr_group);
    //cancel_work_sync(&reporter->psy_notifier_work);
    cancel_delayed_work(&reporter->psy_notifier_work);
    cancel_delayed_work(&reporter->psy_monitor_work);
    cancel_delayed_work(&reporter->vbat_low_dect_work);
    flush_workqueue(reporter->psy_monitor_wq);
    destroy_workqueue(reporter->psy_monitor_wq);

    S10_power_monitor_unregister_notifier(&reporter->psy_monitor_nb);

    S10_power_lock_deinit(&reporter->battery_list_lock);
    S10_power_lock_deinit(&reporter->psy_list_lock);
    S10_power_lock_deinit(&reporter->data_lock);

    reporter->report_battery_num = 0;
    memset(reporter->report_battery_name_array, 0, S10_REPORT_BATTERY_MAX_NUM * BATTERY_NAME_LEN);
    battery_list_destroy();
    psy_list_destroy();
    reporter->stdcharger = NULL;
    platform_set_drvdata(pdev, NULL);
    monitor_report_dev = NULL;
    kfree(reporter);
    reporter = NULL;
    return 0;
}

static int suspend_array_clear(struct S10_power_val_array* array)
{
    if(array != NULL)
    {
        printk(KERN_NOTICE "%s:capacity array clear!!\n",__func__);
        array->begin_pos = array->end_pos = 0;
    }
}

#ifdef CONFIG_PM

static int S10_psy_monitor_suspend(struct platform_device *pdev, pm_message_t state)
{
    struct S10_psy_reporter* reporter = (struct S10_psy_reporter*)platform_get_drvdata(pdev);

    power_debug(HW_POWER_PSYINFO_REPORT_ERR, "%s: suspend enter\n", __func__);
    //cancel_work_sync(&reporter->psy_notifier_work);
    cancel_delayed_work(&reporter->psy_notifier_work);
    cancel_delayed_work(&reporter->psy_monitor_work);
    cancel_delayed_work(&reporter->vbat_low_dect_work);

    suspend_array_clear(&g_capacity_array); 
    power_debug(HW_POWER_PSYINFO_REPORT_ERR, "%s: suspend exit\n", __func__);
    return 0;
}


static int S10_psy_monitor_resume(struct platform_device *pdev)
{
    struct S10_psy_reporter* reporter = (struct S10_psy_reporter*)platform_get_drvdata(pdev);
    struct S10_std_charger_device* charger = reporter->stdcharger;

    power_debug(HW_POWER_PSYINFO_REPORT_ERR, "%s: resume enter\n", __func__);
    if (charger->charger_online)
    {
        S10_psy_repeat_charging_dect(98);
    }

    queue_delayed_work(reporter->psy_monitor_wq, &reporter->psy_monitor_work, msecs_to_jiffies(1));

    /* 如果为关机充电，且已休眠（充满后），持锁20S，显示充电电量 */
    if (1 == reporter->psy_monitor_shutdown_charging_flag)
    {
        wake_lock_timeout(&reporter->psy_reporter_wake_lock, msecs_to_jiffies(1000 * 20));
    }

    power_debug(HW_POWER_PSYINFO_REPORT_ERR, "%s: resume exit\n", __func__);
    return 0;
}


#else
 #define S10_psy_monitor_suspend NULL
 #define S10_psy_monitor_resume NULL
#endif

static struct platform_driver S10_psy_monitor_driver =
{
    .probe    = S10_psy_monitor_probe,
    .remove   = __devexit_p(S10_psy_monitor_remove),
    .suspend  = S10_psy_monitor_suspend,
    .resume   = S10_psy_monitor_resume,
    .driver   = {
        .name = "S10_psy_monitor",
    },
};

static int __init S10_psy_monitor_init(void)
{
    return platform_driver_register(&S10_psy_monitor_driver);
}

subsys_initcall(S10_psy_monitor_init);

static void __exit S10_psy_monitor_exit(void)
{
    platform_driver_unregister(&S10_psy_monitor_driver);
}

module_exit(S10_psy_monitor_exit);

MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:S10_psy_monitor");
MODULE_AUTHOR("S10 Inc");
