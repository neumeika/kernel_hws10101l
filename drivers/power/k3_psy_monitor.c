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
 * k3 psy monitor driver
 */
 /*=============================================================================
==============================================================================*/
#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/power_supply.h>
#include <linux/wakelock.h>
#include <linux/hkadc/hiadc_hal.h>
#include <linux/hkadc/hiadc_linux.h>
#include <linux/notifier.h>

#include <linux/power/k3_psy_monitor.h>
#include <linux/i2c/k3_i2c_bq275x0.h>

#include <linux/interrupt.h>
#include <linux/irq.h>
#include <mach/irqs.h>

#include <mach/product_feature_sel.h>

struct blocking_notifier_head notifier_list_psy;
BLOCKING_NOTIFIER_HEAD(notifier_list_psy);

#define K3_PSY_BAT_INFO_CLASS  (1 << BATTERY_PROP_TECHNOLOGY|1 << BATTERY_PROP_PRESENT|1 << BATTERY_PROP_TEMPERATURE \
                               | 1 << BATTERY_PROP_VOLTAGE | 1<< BATTERY_PROP_CURRENT | 1 << BATTERY_PROP_CAPACITY \
                               |1 << BATTERY_PROP_HEALTH)
#ifdef BARCELONA_SHOW

#define K3_PSY_BAT_SDP_INFO_CLASS  (1 << BATTERY_PROP_TECHNOLOGY|1 << BATTERY_PROP_PRESENT|1 << BATTERY_PROP_TEMPERATURE \
                               | 1 << BATTERY_PROP_VOLTAGE | 1<< BATTERY_PROP_CURRENT | 1 << BATTERY_PROP_CAPACITY \
                               |1 << BATTERY_PROP_HEALTH | 1 << BATTERY_PROP_SDP_ONLINE)
#define K3_PSY_BAT_DCP_INFO_CLASS  (1 << BATTERY_PROP_TECHNOLOGY|1 << BATTERY_PROP_PRESENT|1 << BATTERY_PROP_TEMPERATURE \
                                   | 1 << BATTERY_PROP_VOLTAGE | 1<< BATTERY_PROP_CURRENT | 1 << BATTERY_PROP_CAPACITY \
                                   |1 << BATTERY_PROP_HEALTH | 1 << BATTERY_PROP_DCP_ONLINE)
#endif
#define K3_PSY_BAT_CAP         (1 << BATTERY_PROP_CAPACITY)
#define K3_PSY_BAT_TEST        (0)

struct k3_charger_supply_report_dev{
   int                          extra_battery_exist; //0:no extra battery, 1: extra battery exist
   int                          bottom_exit;//
   struct mutex                 charger_info_lock;
   struct k3_charger_info       charger_psy_info;
   struct power_supply          charger_psy;
   struct power_supply          ac; 
};

struct k3_battery_supply_report_dev{
    struct mutex                battery_info_lock;
	struct k3_battery_info      battery_psy_info;
	struct power_supply	        battery_psy;
};

struct k3_psy_monitor_device_info {

	unsigned int			        monitoring_interval;

	struct device	                *dev;

    struct mutex                    charger_list_lock;
	struct list_head                charger_list;
    struct mutex                    battery_gauge_list_lock;
	struct list_head                battery_gauge_list;

	struct k3_charger_supply_report_dev current_charger_dev;
	struct k3_battery_supply_report_dev main_battery_dev;
    #ifdef CONFIG_EXTRA_BATTERY
	struct k3_battery_supply_report_dev extra_battery_dev;
    #endif

	struct notifier_block	        nb;

	struct delayed_work		        k3_psy_monitor_work;
	struct delayed_work             vbat_low_int_work;
    struct workqueue_struct         *k3_power_supply_monitor_wq;

};

static int get_prop_batt_info_update(const char *battery_name, int info);

static struct k3_psy_monitor_device_info* k3_psy_dev_info = NULL;
static struct platform_driver k3_psy_monitor_driver;

#define AC_STATUS_DETECT  (1<<0)
#define AC_IRQ_REQUEST    (1<<1)
#define AC_IRQ_FREE       (1<<2)

static int k3_ac_detect_handle(struct k3_psy_monitor_device_info *di, int ac_handle)
{
    struct k3_charger_dev  *hw_chg          = NULL;
    struct list_head       *list            = NULL;
    int ret = 0;

    if(0 == ac_handle){
        return -EINVAL;
    }

    if(!list_empty(&di->charger_list)){
        list = di->charger_list.next;
        hw_chg = list_entry(list, struct k3_charger_dev, charger_list);
    }

    switch(ac_handle){
    case AC_STATUS_DETECT:
        if(hw_chg && hw_chg->get_gpio_value){
            ret = hw_chg->get_gpio_value();
        }
        break;
    case AC_IRQ_REQUEST:
        if(hw_chg && hw_chg->irq_request){
            ret = hw_chg->irq_request();
        }
        break;
    case AC_IRQ_FREE:
        if(hw_chg && hw_chg->irq_free){
            hw_chg->irq_free();
        }
        break;
    default:
        break;
   }
    return ret;
}

static void k3_charger_current_ctl(struct k3_psy_monitor_device_info *di)
{
	struct k3_charger_dev  *hw_chg          = NULL;
    struct list_head       *list            = NULL;
    int  set_current                        = 0;

    set_current = di->current_charger_dev.charger_psy_info.charger_max_charging_current;

    if(!list_empty(&di->charger_list)){
        list = di->charger_list.next;
        hw_chg = list_entry(list, struct k3_charger_dev, charger_list);
    }

    if(hw_chg && hw_chg->charging_current_control){
        hw_chg->charging_current_control(hw_chg,set_current);
    }

}

void k3_charger_set_power(int on)
{
	struct k3_charger_dev  *hw_chg         = NULL;
	struct list_head       *list           = NULL;
	struct k3_psy_monitor_device_info *di  = k3_psy_dev_info;

	if(!list_empty(&di->charger_list)){
	list = di->charger_list.next;
		hw_chg = list_entry(list, struct k3_charger_dev, charger_list);
	}

	if(hw_chg && hw_chg->charging_switch){
		hw_chg->charging_switch(hw_chg,on);
	}
}
EXPORT_SYMBOL_GPL(k3_charger_set_power);

static ssize_t show_charge_en(struct device_driver *dev, char *buf)
{
     struct k3_psy_monitor_device_info* di = k3_psy_dev_info;
     int charge_en = 1;

     printk(KERN_INFO "[%s %d] show_charge_en...... \n", __func__, __LINE__ );

	if (NULL == buf) {
		sprintf(buf, "%s", "show_charge_en Error");
		return -1;
	}

    if (di->main_battery_dev.battery_psy_info.battery_status == POWER_SUPPLY_STATUS_DISCHARGING)
   {
        charge_en = 0;
   }

     printk(KERN_INFO "[%s %d] charge_en [%d]...... \n", __func__, __LINE__ , charge_en);

     return sprintf(buf, "%d\n", charge_en);

}


static ssize_t disable(struct device_driver *driver,const char *buf,size_t count)
{
	int flag = (int)buf[0] -'0';
    struct k3_psy_monitor_device_info* di = k3_psy_dev_info;

	if(!flag)
	{
	    k3_charger_set_power(flag);
		printk(KERN_INFO "[%s %d] close charging...... \n", __func__, __LINE__ );
//        di->main_battery_dev.battery_psy_info.battery_status = POWER_SUPPLY_STATUS_DISCHARGING;
	}
	else
	{
		k3_charger_set_power(flag);
		printk(KERN_INFO "[%s %d] open charging...... \n", __func__, __LINE__ );
//        di->main_battery_dev.battery_psy_info.battery_status = POWER_SUPPLY_STATUS_CHARGING;
	}
	return count;
}


static int k3_psy_atoi(const char *s)
{
	int k = 0;

	k = 0;
	while (*s != '\0' && *s >= '0' && *s <= '9') {
		k = 10 * k + (*s - '0');
		s++;
	}
	return k;
}


static ssize_t show_charging_current(struct device_driver *dev, char *buf)
{
     struct k3_psy_monitor_device_info* di = k3_psy_dev_info;
     int charging_current = 0;

     printk(KERN_INFO "[%s %d] show charging current...... \n", __func__, __LINE__ );

	if (NULL == buf) {
		sprintf(buf, "%s", "show_charging_current Error");
		return -1;
	}

     charging_current = di->current_charger_dev.charger_psy_info.charger_max_charging_current;
     printk(KERN_INFO "[%s %d] charging current [%d]...... \n", __func__, __LINE__ , charging_current);

     return sprintf(buf, "%d\n", charging_current);
}

static ssize_t store_charging_current(struct device_driver *driver,const char *buf,size_t count)
{
    struct k3_psy_monitor_device_info* di = k3_psy_dev_info;
    int charger_level = 0;
    int current_set = 0;

    charger_level = k3_psy_atoi(buf);
    printk(KERN_INFO "[%s %d] set charging current [%d]...... \n",__func__, __LINE__, charger_level);

    if(charger_level < 0 || charger_level > 3)
    {
        printk(KERN_INFO "[%s %d] charger level error...... !\n", __func__, __LINE__ );
        charger_level = 0;
    }

    switch(charger_level){
    case 0:
        break;
    case 1:
        mutex_lock(&di->current_charger_dev.charger_info_lock);
        di->current_charger_dev.charger_psy_info.charger_max_charging_current = 500;
        mutex_unlock(&di->current_charger_dev.charger_info_lock);
        break;
    case 2:
        mutex_lock(&di->current_charger_dev.charger_info_lock);
        di->current_charger_dev.charger_psy_info.charger_max_charging_current = 1500;
        mutex_unlock(&di->current_charger_dev.charger_info_lock);
        break;
    case 3:
        mutex_lock(&di->current_charger_dev.charger_info_lock);
        di->current_charger_dev.charger_psy_info.charger_max_charging_current = 2000;
        mutex_unlock(&di->current_charger_dev.charger_info_lock);
        break;
    default:
        mutex_lock(&di->current_charger_dev.charger_info_lock);
        di->current_charger_dev.charger_psy_info.charger_max_charging_current = 500;
        mutex_unlock(&di->current_charger_dev.charger_info_lock);
        break;
    }
	
    k3_charger_current_ctl(di); 
    return count;
}

//static DRIVER_ATTR(CEN_N, S_IRUGO|S_IWUGO, show_charge_en, disable);
//static DRIVER_ATTR(CURRENT, S_IRUGO|S_IWUGO, show_charging_current, store_charging_current);
static DRIVER_ATTR(CEN_N, S_IRUGO|S_IWUSR|S_IWGRP, show_charge_en, disable);
static DRIVER_ATTR(CURRENT, S_IRUGO|S_IWUSR|S_IWGRP, show_charging_current, store_charging_current);

/*Update charger present status for sysfs*/
static int k3_battery_monitor_charger_event(struct notifier_block *nb, unsigned long event,
											void *_data)
{
	int ret                  = 0;
    int batt_present         = 0;
    int ac_status            = 0;//
	int current_capacity  = 0; 
	int charge_type_usb = 0;

	struct k3_psy_monitor_device_info *di = container_of(nb, struct k3_psy_monitor_device_info, nb);
    PSY_DBG("k3_psy_monitor:%s,di=0x%08x\n",__func__, di);



    batt_present = di->main_battery_dev.battery_psy_info.battery_present;

    PSY_DBG("k3_psy_monitor:%s,event=%x,batt_present=%d, di->current_charger_dev=0x%08x\n", \
                                           __func__, event, batt_present, &(di->current_charger_dev));

    	switch (event) {
        case CHG_USB_CDP_INSERT_EVENT:
                mutex_lock(&di->current_charger_dev.charger_info_lock);
				//di->current_charger_dev.charger_psy_info.charger_type = POWER_SUPPLY_TYPE_USB_CDP;
				di->current_charger_dev.charger_psy_info.charger_max_charging_current = 1500;
                di->current_charger_dev.charger_psy_info.ac_online = 0;
				di->current_charger_dev.charger_psy_info.charger_online = 1;
                mutex_unlock(&di->current_charger_dev.charger_info_lock);
				break;
        case CHG_USB_DCP_INSERT_EVENT:
                mutex_lock(&di->current_charger_dev.charger_info_lock);
				//di->current_charger_dev.charger_psy_info.charger_type = POWER_SUPPLY_TYPE_USB_DCP;
				di->current_charger_dev.charger_psy_info.charger_max_charging_current = 2000;
                di->current_charger_dev.charger_psy_info.ac_online = 1;
				di->current_charger_dev.charger_psy_info.charger_online = 0;
                mutex_unlock(&di->current_charger_dev.charger_info_lock);
                break;
        case CHG_USB_SDP_INSERT_EVENT:
                mutex_lock(&di->current_charger_dev.charger_info_lock);
				charge_type_usb = 1;
				//di->current_charger_dev.charger_psy_info.charger_type = POWER_SUPPLY_TYPE_USB;
                //just for debug(500->2000)
				//di->current_charger_dev.charger_psy_info.charger_max_charging_current = 500;
				//di->current_charger_dev.charger_psy_info.charger_max_charging_current = 2000;
				if(get_product_feature(PROD_FEATURE_SDP_CHARGER_CURRENT)){
	        		di->current_charger_dev.charger_psy_info.charger_max_charging_current = 500;
				}        
				else
				{
	        		di->current_charger_dev.charger_psy_info.charger_max_charging_current = 2000;
				}
				printk("charger_max_charging_current:%d\n",di->current_charger_dev.charger_psy_info.charger_max_charging_current);
				di->current_charger_dev.charger_psy_info.ac_online = 0;
				di->current_charger_dev.charger_psy_info.charger_online = 1;
                mutex_unlock(&di->current_charger_dev.charger_info_lock);
				break;
	    case CHG_BOTTOM_EXIST_EVENT:
                mutex_lock(&di->current_charger_dev.charger_info_lock);
                ac_status = k3_ac_detect_handle(di, AC_STATUS_DETECT);
				//di->current_charger_dev.extra_battery_exist = 1;
                di->current_charger_dev.bottom_exit = 1;
                printk(KERN_EMERG "CHG_BOTTOM_EXIST_EVENT happen!!ac_status=%d\n", ac_status);
                if(!ac_status){
					di->current_charger_dev.charger_psy_info.charger_max_charging_current = 2000;
					di->current_charger_dev.charger_psy_info.charger_online = 1;
                }
                k3_ac_detect_handle(di, AC_IRQ_REQUEST);
                mutex_unlock(&di->current_charger_dev.charger_info_lock);
				break;
        case CHG_AC_INSERT_EVENT:
                printk(KERN_EMERG "CHG_AC_INSERT_EVENT happen!!!\n");
                mutex_lock(&di->current_charger_dev.charger_info_lock);
				//di->current_charger_dev.charger_psy_info.charger_type = POWER_SUPPLY_TYPE_MAINS;
				di->current_charger_dev.charger_psy_info.charger_max_charging_current = 2000;
				di->current_charger_dev.charger_psy_info.charger_online = 0;
				di->current_charger_dev.charger_psy_info.ac_online = 1;
                mutex_unlock(&di->current_charger_dev.charger_info_lock);
                break;
        case CHG_AC_REMOVED_EVENT:
                printk(KERN_EMERG "LEON:CHG_AC_REMOVED_EVENT happen!!!\n");
                mutex_lock(&di->current_charger_dev.charger_info_lock);
				di->current_charger_dev.charger_psy_info.charger_online = 0;
				di->current_charger_dev.charger_psy_info.ac_online = 0;
                mutex_unlock(&di->current_charger_dev.charger_info_lock);
                break;
        case CHG_REMOVED_EVENT:
                if(di->current_charger_dev.bottom_exit){
                     k3_ac_detect_handle(di, AC_IRQ_FREE);
                     di->current_charger_dev.bottom_exit = 0;	
                }
                mutex_lock(&di->current_charger_dev.charger_info_lock);
				if(di->current_charger_dev.extra_battery_exist)
					di->current_charger_dev.extra_battery_exist = 0;
                di->current_charger_dev.charger_psy_info.ac_online = 0;
				di->current_charger_dev.charger_psy_info.charger_online = 0;
				if(get_product_feature(PROD_FEATURE_SDP_CHARGER_CURRENT)){
	        		di->current_charger_dev.charger_psy_info.charger_max_charging_current = 500;
				}        
				else
				{
	        		di->current_charger_dev.charger_psy_info.charger_max_charging_current = 2000;
				}
				printk("charger_max_charging_current:%d\n",di->current_charger_dev.charger_psy_info.charger_max_charging_current);
	            mutex_unlock(&di->current_charger_dev.charger_info_lock);
				break;
		default:
				goto out;
        }
    if(batt_present){
    	k3_charger_current_ctl(di);
    	if(di->current_charger_dev.charger_psy_info.ac_online || 
	        di->current_charger_dev.charger_psy_info.charger_online){
    		//k3_charger_current_ctl(di);
    		k3_charger_set_power(1);
			get_prop_batt_info_update(di->main_battery_dev.battery_psy.name, 1 << BATTERY_PROP_CAPACITY);//update battery capacity info
			current_capacity = di->main_battery_dev.battery_psy_info.battery_capacity;
			if(current_capacity == 100){
				di->main_battery_dev.battery_psy_info.battery_status = POWER_SUPPLY_STATUS_FULL;
			}
			else if(charge_type_usb == 1){
				di->main_battery_dev.battery_psy_info.battery_status = POWER_SUPPLY_STATUS_NOT_CHARGING;
			}
			else{
           		di->main_battery_dev.battery_psy_info.battery_status = POWER_SUPPLY_STATUS_CHARGING;
			}
    	}
    	else {
			charge_type_usb = 0;
    		//k3_charger_set_power(0);
            di->main_battery_dev.battery_psy_info.battery_status = POWER_SUPPLY_STATUS_DISCHARGING;
        }

        #ifdef CONFIG_EXTRA_BATTERY
    	power_supply_changed(&di->extra_battery_dev.battery_psy);
    	#endif

	    power_supply_changed(&di->current_charger_dev.charger_psy);
        power_supply_changed(&di->current_charger_dev.ac);
    }
    else{
        //k3_charger_set_power(0);
      	//  di->current_charger_dev.charger_psy_info.charger_online = 0;
    }

out:
        return ret;
}

int k3_psy_monitor_register_notifier(struct notifier_block *nb,
				unsigned int events)
{
	return blocking_notifier_chain_register(&notifier_list_psy, nb);
}
EXPORT_SYMBOL_GPL(k3_psy_monitor_register_notifier);

int k3_psy_monitor_unregister_notifier(struct notifier_block *nb,
				unsigned int events)
{
	return blocking_notifier_chain_register(&notifier_list_psy, nb);
}
EXPORT_SYMBOL_GPL(k3_psy_monitor_unregister_notifier);


int k3_psy_monitor_register_charger(struct k3_charger_dev* charger)
{
	int ret                                = 0;
	struct k3_psy_monitor_device_info* di  = NULL;

    printk(KERN_EMERG "k3_psy_monitor:k3_psy_monitor_register_charger:%s\n", charger->name);
	if(!charger || !charger->name || !charger->charging_switch || !charger->charging_current_control)
		return -EINVAL;

	di = k3_psy_dev_info;

    mutex_lock(&di->charger_list_lock);
	list_add_tail(&charger->charger_list, &di->charger_list);
    mutex_unlock(&di->charger_list_lock);

	return ret;
}
EXPORT_SYMBOL_GPL(k3_psy_monitor_register_charger);

int k3_psy_monitor_unregister_charger(struct k3_charger_dev* charger)
{
	int ret                                 = -EINVAL;
	struct k3_charger_dev* chg_temp         = NULL;
	struct k3_psy_monitor_device_info* di   = k3_psy_dev_info;
	struct list_head *p                     = NULL;

	// del current charger form the psy monitor charger list
	if(!list_empty(&di->charger_list)){
		list_for_each(p, &di->charger_list){
			chg_temp = list_entry(p, struct k3_charger_dev, charger_list);
			if(!strcmp(charger->name, chg_temp->name)){
				mutex_lock(&di->charger_list_lock);
				list_del(&chg_temp->charger_list);
				mutex_unlock(&di->charger_list_lock);
				ret = 0;
			}
		}
	}

	return ret;
}
EXPORT_SYMBOL_GPL(k3_psy_monitor_unregister_charger);

int k3_psy_monitor_register_battery(struct k3_battery_gauge_dev* battery)
{
	int ret                                 = 0;
	struct k3_psy_monitor_device_info* di   = k3_psy_dev_info;

    PSY_DBG("k3_psy_monitor:k3_psy_monitor_register_battery:%s, battery = 0x%08x\n", battery->name, battery);
	if(!battery || !battery->name || !battery->get_battery_info)
		return -EINVAL;

    ret = battery->get_battery_info(K3_PSY_BAT_INFO_CLASS, &(di->main_battery_dev.battery_psy_info));

	mutex_lock(&di->battery_gauge_list_lock);
	list_add_tail(&battery->battery_list, &di->battery_gauge_list);
	mutex_unlock(&di->battery_gauge_list_lock);

	return ret;
}
EXPORT_SYMBOL_GPL(k3_psy_monitor_register_battery);

int k3_psy_monitor_unregister_battery(struct k3_battery_gauge_dev* battery)
{
	int ret                                 = -EINVAL;
	struct k3_battery_gauge_dev* bat_temp   = NULL;
	struct k3_psy_monitor_device_info* di   = k3_psy_dev_info;
	struct list_head *p                     = NULL;

	// del current battery form the psy monitor battery list
	if(!list_empty(&di->battery_gauge_list)){
		list_for_each(p, &di->battery_gauge_list){
			bat_temp = list_entry(p, struct k3_battery_gauge_dev, battery_list);
			if(!strcmp(battery->name, bat_temp->name)){
				mutex_lock(&di->battery_gauge_list_lock);
				list_del(&bat_temp->battery_list);
				mutex_unlock(&di->battery_gauge_list_lock);
				ret = 0;
			}
		}
	}
	return ret;
}
EXPORT_SYMBOL_GPL(k3_psy_monitor_unregister_battery);

#ifdef CONFIG_BQ275X0_FIRMWARE_UPDATE
void k3_psy_monitor_stop_sync(void)
{
	struct k3_psy_monitor_device_info *di = k3_psy_dev_info;
	cancel_delayed_work(&di->k3_psy_monitor_work);
	flush_scheduled_work();
}
EXPORT_SYMBOL_GPL(k3_psy_monitor_stop_sync);

void k3_psy_monitor_restart_sync(void)
{
	struct k3_psy_monitor_device_info *di = k3_psy_dev_info;
	queue_delayed_work(di->k3_power_supply_monitor_wq,&di->k3_psy_monitor_work, 0);
}
EXPORT_SYMBOL_GPL(k3_psy_monitor_restart_sync);
#endif

static struct k3_battery_gauge_dev* k3_get_battery_from_list(const char* battery_name)
{
	struct k3_battery_gauge_dev* bat_temp   = NULL;
    if(!k3_psy_dev_info)
        return NULL;
	struct k3_psy_monitor_device_info* di   = k3_psy_dev_info;
	struct list_head *p                     = NULL;
    PSY_DBG("%s di = 0x%08x!\n",__func__, di);

    PSY_DBG("%s list_empty begin!\n",__func__);
	if(!list_empty(&di->battery_gauge_list)){
		list_for_each(p,&di->battery_gauge_list){
			bat_temp = list_entry(p, struct k3_battery_gauge_dev, battery_list);
            PSY_DBG("%s bat_temp->name:%s!\n",__func__, bat_temp->name);
			if(!strcmp(battery_name, bat_temp->name))
			{
				return bat_temp;
			}
		}
	}

    PSY_DBG("%s end!\n",__func__);
	return NULL;
}


static int get_prop_batt_info(const char *battery_name)
{
#ifdef BARCELONA_SHOW
    int chg_online                          = 0;
    int chg_type                            = 0;
#endif
    int ret                                      = -EINVAL;
    struct k3_battery_gauge_dev *battery_tmp     = k3_get_battery_from_list(battery_name);
    struct k3_psy_monitor_device_info* di        = k3_psy_dev_info;
    PSY_DBG("%s:battery_tmp = 0x%08x\n", __func__,battery_tmp);

    if(battery_tmp != NULL && battery_tmp->get_battery_info != NULL)
    {
#ifdef BARCELONA_SHOW
        chg_online  = di->current_charger_dev.charger_psy_info.charger_online;
        chg_type    = di->current_charger_dev.charger_psy_info.charger_type;
        if(!chg_online){
            ret = battery_tmp->get_battery_info(K3_PSY_BAT_INFO_CLASS, &(di->main_battery_dev.battery_psy_info));
        }
        else{
            if(chg_type == POWER_SUPPLY_TYPE_USB_DCP){
                ret = battery_tmp->get_battery_info(K3_PSY_BAT_DCP_INFO_CLASS, &(di->main_battery_dev.battery_psy_info));
            }
            else{
                ret = battery_tmp->get_battery_info(K3_PSY_BAT_SDP_INFO_CLASS, &(di->main_battery_dev.battery_psy_info));
            }
        }
#else
        ret = battery_tmp->get_battery_info(K3_PSY_BAT_INFO_CLASS, &(di->main_battery_dev.battery_psy_info));
#endif
    }
    return ret;
}

static int get_prop_batt_info_update(const char *battery_name, int info)
{
    int ret                                      = -EINVAL;
    struct k3_battery_gauge_dev *battery_tmp     = k3_get_battery_from_list(battery_name);
    struct k3_psy_monitor_device_info* di        = k3_psy_dev_info;

    if(battery_tmp != NULL && battery_tmp->get_battery_info != NULL)
    {
        ret = battery_tmp->get_battery_info(info, &(di->main_battery_dev.battery_psy_info));
    }
    return ret;
}

static enum power_supply_property k3_psy_monitor_charger_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
};

static enum power_supply_property k3_psy_monitor_ac_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
};

static enum power_supply_property k3_psy_monitor_battery_props[] = {
    POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_TEMP,
};

#define to_k3_charger_supply_device_info(x) container_of((x), struct k3_charger_supply_report_dev, charger_psy)
#define to_k3_ac_supply_device_info(x) container_of((x), struct k3_charger_supply_report_dev, ac) 
#define to_k3_battery_supply_device_info(x) container_of((x), struct k3_battery_supply_report_dev, battery_psy)

static char *k3_charger_supplies_to[] = {
      "MainBattery",
      "ExtraBattery",
};

static int k3_charger_monitor_get_property(struct power_supply *psy,
					                  enum power_supply_property psp,
					                  union power_supply_propval *val)
{
	struct k3_charger_supply_report_dev* charger_supply_dev  = NULL;

	charger_supply_dev = to_k3_charger_supply_device_info(psy);

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = charger_supply_dev->charger_psy_info.charger_online;
		break;
	case POWER_SUPPLY_PROP_CHARGE_TYPE:
		val->intval = charger_supply_dev->charger_psy_info.charger_type;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}
static int k3_ac_monitor_get_property(struct power_supply *psy,
					                  enum power_supply_property psp,
					                  union power_supply_propval *val)
{
	struct k3_charger_supply_report_dev* charger_supply_dev  = NULL;

	charger_supply_dev = to_k3_ac_supply_device_info(psy);

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = charger_supply_dev->charger_psy_info.ac_online;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int k3_battery_monitor_get_property(struct power_supply *psy,
					                  enum power_supply_property psp,
					                  union power_supply_propval *val)
{
	struct k3_battery_supply_report_dev* battery_supply_dev  = NULL;
    struct k3_battery_gauge_dev* bat_temp                    = NULL;

	battery_supply_dev = to_k3_battery_supply_device_info(psy);
    bat_temp = k3_get_battery_from_list(MAIN_BATTERY);
    PSY_DBG("%s:bat_temp = 0x%08x\n", __func__,bat_temp);
#if 0
    if(bat_temp != NULL && bat_temp->get_battery_info != NULL)
    {
        chg_online = di->current_charger_dev.charger_psy_info.charger_online;
        if(!chg_online){
             bat_temp->get_battery_info(K3_PSY_BAT_INFO_CLASS, &battery_supply_dev->battery_psy_info);
            }
        else{
             bat_temp->get_battery_info(K3_PSY_BAT_CHG_INFO_CLASS, &battery_supply_dev->battery_psy_info);
            }
        }
#endif
	PSY_DBG("k3_psy_monitor:%s psp = %d\n",__func__,psp);
	switch (psp) {
		case POWER_SUPPLY_PROP_STATUS:
			val->intval = battery_supply_dev->battery_psy_info.battery_status;
			break;
		case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		    //TODO:just for BC1.1 driver , i2c conflict
			val->intval = battery_supply_dev->battery_psy_info.battery_voltage * 1000;
			break;
		case POWER_SUPPLY_PROP_CURRENT_NOW:
			val->intval = battery_supply_dev->battery_psy_info.battery_current;

			break;
		case POWER_SUPPLY_PROP_TEMP:
			val->intval = battery_supply_dev->battery_psy_info.battery_temperture;
			break;
		case POWER_SUPPLY_PROP_PRESENT:
			val->intval = battery_supply_dev->battery_psy_info.battery_present;
			break;
		case POWER_SUPPLY_PROP_HEALTH:
			val->intval = battery_supply_dev->battery_psy_info.battery_health;
			break;
		case POWER_SUPPLY_PROP_CAPACITY:
			//TODO:just for BC1.1 driver
			val->intval = battery_supply_dev->battery_psy_info.battery_capacity;
			break;
		case POWER_SUPPLY_PROP_TECHNOLOGY:
			val->intval = battery_supply_dev->battery_psy_info.battery_technology;
			break;
		case POWER_SUPPLY_PROP_TIME_TO_EMPTY_NOW:
			val->intval = battery_supply_dev->battery_psy_info.battery_time2empty;
			break;
		case POWER_SUPPLY_PROP_TIME_TO_FULL_NOW:
			val->intval = battery_supply_dev->battery_psy_info.battery_time2full;
			break;
		default:
			PSY_DBG("%s defualt run.\n", __func__);
			return -EINVAL;
	}

	return 0;
}

#ifdef CONFIG_EXTRA_BATTERY
static int k3_extra_battery_monitor_get_property(struct power_supply *psy,
					enum power_supply_property psp,
					union power_supply_propval *val)
{
	struct k3_battery_supply_report_dev* battery_supply_dev     = NULL;
	struct k3_psy_monitor_device_info* di                       = NULL;

	battery_supply_dev = to_k3_battery_supply_device_info(psy);
    	di = k3_psy_dev_info;

	if(di->current_charger_dev.extra_battery_exist){
		switch (psp) {
			case POWER_SUPPLY_PROP_STATUS:
				val->intval = battery_supply_dev->battery_psy_info.battery_status;
				break;
			case POWER_SUPPLY_PROP_VOLTAGE_NOW:
				val->intval = battery_supply_dev->battery_psy_info.battery_voltage;
				break;
			case POWER_SUPPLY_PROP_CURRENT_NOW:
				val->intval = battery_supply_dev->battery_psy_info.battery_current;
				break;
			case POWER_SUPPLY_PROP_TEMP:
				val->intval = battery_supply_dev->battery_psy_info.battery_temperture;
				break;
			case POWER_SUPPLY_PROP_PRESENT:
				val->intval = battery_supply_dev->battery_psy_info.battery_present;
				break;
			case POWER_SUPPLY_PROP_HEALTH:
				val->intval = battery_supply_dev->battery_psy_info.battery_health;
				break;
			case POWER_SUPPLY_PROP_CAPACITY:

				val->intval = battery_supply_dev->battery_psy_info.battery_capacity;
				break;
			case POWER_SUPPLY_PROP_TECHNOLOGY:
				val->intval = battery_supply_dev->battery_psy_info.battery_technology;
				break;
			case POWER_SUPPLY_PROP_TIME_TO_EMPTY_NOW:
				val->intval = battery_supply_dev->battery_psy_info.battery_time2empty;
				break;
			case POWER_SUPPLY_PROP_TIME_TO_FULL_NOW:
				val->intval = battery_supply_dev->battery_psy_info.battery_time2full;
				break;
			default:
				dev_err(di->dev, "%s defualt run.\n", __func__);
				return -EINVAL;
			}
	}
	else{
			dev_err(di->dev, "%s cannot got extra battery prop,not exit!!\n", __func__);
			return -EINVAL;
		}
		return 0;
	}
#endif

// convert the origen data which went to report to the up level
static int k3_psy_monitor_info_convert(struct k3_psy_monitor_device_info *di)
{
    //TODO:将原始电池数据去抖后存放到di中相应的report字段中去
    PSY_DBG("%s begin!\n",__func__);

	return 0;
}

// monitor k3 psy capacity changer flag
static int k3_psy_monitor_capacity_changed(struct k3_psy_monitor_device_info *di)
{
    int ret = 0;

    int old_main_batt_capacity      = 0;
	int main_batt_capacity          = 0;

	int main_batt_update            = 0;

    int main_batt_online            = 0;
    int old_main_batt_online        = 0;

    old_main_batt_capacity = di->main_battery_dev.battery_psy_info.battery_capacity;

    old_main_batt_online = di->main_battery_dev.battery_psy_info.battery_present;

    //获取电池信息
	ret = get_prop_batt_info(di->main_battery_dev.battery_psy.name);

    if(ret < 0){
        PSY_ERR("%s:get_prop_batt_info error!!\n",__func__);
        goto update_flag;
    }

    main_batt_capacity = di->main_battery_dev.battery_psy_info.battery_capacity;
    main_batt_online = di->main_battery_dev.battery_psy_info.battery_present;

    PSY_DBG("%s:main_batt_capacity = %d\n",__func__,main_batt_capacity);
	if(main_batt_capacity < 0 || main_batt_capacity == NULL){
		PSY_ERR("%s failed, the main battery capacity is error!\n", __func__);
		return 0;
	}
	if ( abs (old_main_batt_capacity - main_batt_capacity) >= 1) {
		if ((di->current_charger_dev.charger_psy_info.charger_online || di->current_charger_dev.charger_psy_info.ac_online ) && main_batt_capacity == 100){
				di->main_battery_dev.battery_psy_info.battery_status = POWER_SUPPLY_STATUS_FULL;
        }
			main_batt_update = 1;
	} else {
			main_batt_update = 0;
	}

    if(old_main_batt_online != main_batt_online)
        main_batt_update = 1;

    PSY_DBG("%s:main_batt_update=%d\n",__func__,main_batt_update);

update_flag:
	return main_batt_update;
}


static void k3_psy_monitor_work(struct work_struct *work)
{
	struct k3_psy_monitor_device_info *di = container_of(work,
	    struct k3_psy_monitor_device_info, k3_psy_monitor_work.work);

    PSY_DBG("k3_psy_monitor:%s begin\n!!!!!\n",__func__);

	schedule_delayed_work(&di->k3_psy_monitor_work,
			msecs_to_jiffies(1000 * di->monitoring_interval));

    if (k3_psy_monitor_capacity_changed(di))
    {
        power_supply_changed(&di->main_battery_dev.battery_psy);
        #ifdef CONFIG_EXTRA_BATTERY
        power_supply_changed(&di->extra_battery_dev.battery_psy);
        #endif
    }
}


static int __devinit k3_psy_monitor_probe(struct platform_device *pdev)
{
	int ret                                         = 0;
	struct k3_psy_monitor_platform_data *pdata  = NULL;
	struct k3_psy_monitor_device_info* di           = NULL;

	PSY_DBG("k3_psy_monitor:k3_psy_monitor_probe!\n");
	di = kzalloc(sizeof(*di), GFP_KERNEL);
	if (!di) {
		dev_dbg(&pdev->dev, "psy_monitor_dev is NULL\n");
		return -ENOMEM;
	}

	pdata = pdev->dev.platform_data;
	if (!pdata) {
		dev_dbg(&pdev->dev, "platform_data not available\n");
		ret = -EINVAL;
		goto err_pdata;
	}


	if (pdata->monitoring_interval == 0) {
		di->monitoring_interval = BATTERY_MONITOR_INTERVAL_PSY;
	} else {
		di->monitoring_interval = pdata->monitoring_interval;
	}

	di->dev = &pdev->dev;

	INIT_LIST_HEAD(&di->charger_list);
	INIT_LIST_HEAD(&di->battery_gauge_list);

	mutex_init(&di->charger_list_lock);
	mutex_init(&di->battery_gauge_list_lock);

	mutex_init(&di->main_battery_dev.battery_info_lock);
    #ifdef CONFIG_EXTRA_BATTERY
	mutex_init(&di->extra_battery_dev.battery_info_lock);
    #endif
	mutex_init(&di->current_charger_dev.charger_info_lock);

	di->current_charger_dev.extra_battery_exist = 0;
    di->current_charger_dev.bottom_exit = 0;//
	di->current_charger_dev.charger_psy_info.charger_online = 0;
    di->current_charger_dev.charger_psy_info.ac_online = 0;

	platform_set_drvdata(pdev, di);

	di->main_battery_dev.battery_psy.name = MAIN_BATTERY;
	di->main_battery_dev.battery_psy.type = POWER_SUPPLY_TYPE_BATTERY;
	di->main_battery_dev.battery_psy.properties = k3_psy_monitor_battery_props;
	di->main_battery_dev.battery_psy.num_properties = ARRAY_SIZE(k3_psy_monitor_battery_props);
	di->main_battery_dev.battery_psy.get_property = k3_battery_monitor_get_property;

    #ifdef CONFIG_EXTRA_BATTERY
	di->extra_battery_dev.battery_psy.name = EXTRA_BATTERY;
	di->extra_battery_dev.battery_psy.type = POWER_SUPPLY_TYPE_BATTERY;
	di->extra_battery_dev.battery_psy.properties = k3_psy_monitor_battery_props;
	di->extra_battery_dev.battery_psy.num_properties = ARRAY_SIZE(k3_psy_monitor_battery_props);
	di->extra_battery_dev.battery_psy.get_property = k3_extra_battery_monitor_get_property;
    #endif

	di->current_charger_dev.charger_psy.name = MAIN_CHARGER;
	di->current_charger_dev.charger_psy.supplied_to = k3_charger_supplies_to;
	di->current_charger_dev.charger_psy.num_supplicants = ARRAY_SIZE(k3_charger_supplies_to);
	di->current_charger_dev.charger_psy.type = POWER_SUPPLY_TYPE_USB;
	di->current_charger_dev.charger_psy.properties = k3_psy_monitor_charger_props;
	di->current_charger_dev.charger_psy.num_properties = ARRAY_SIZE(k3_psy_monitor_charger_props);
	di->current_charger_dev.charger_psy.get_property = k3_charger_monitor_get_property;
    di->current_charger_dev.ac.name = "AC";
	di->current_charger_dev.ac.type = POWER_SUPPLY_TYPE_MAINS;
	di->current_charger_dev.ac.properties = k3_psy_monitor_ac_props;
	di->current_charger_dev.ac.num_properties = ARRAY_SIZE(k3_psy_monitor_ac_props);
	di->current_charger_dev.ac.get_property = k3_ac_monitor_get_property;
	
   	 di->main_battery_dev.battery_psy_info.battery_status = POWER_SUPPLY_STATUS_DISCHARGING;

	INIT_DELAYED_WORK_DEFERRABLE(&di->k3_psy_monitor_work,
				k3_psy_monitor_work);
	schedule_delayed_work(&di->k3_psy_monitor_work, 0);

	ret = power_supply_register(&pdev->dev, &di->main_battery_dev.battery_psy);
	if (ret) {
		PSY_DBG("failed to register main battery power supply\n");
		goto main_batter_failed;
	}

#ifdef CONFIG_EXTRA_BATTERY
	ret = power_supply_register(&pdev->dev, &di->extra_battery_dev.battery_psy);
	if (ret) {
		dev_err(&pdev->dev, "failed to register extra battery power supply\n");
		goto extra_batter_failed;
	}
#endif
	PSY_DBG("k3_psy_monitor:k3_psy_monitor_probe! extra battery power supply end\n");
	ret = power_supply_register(&pdev->dev, &di->current_charger_dev.charger_psy);
	if (ret) {
		PSY_DBG("failed to register main charger power supply\n");
		goto main_charger_failed;
	}

	ret = power_supply_register(&pdev->dev, &di->current_charger_dev.ac);
	if (ret) {
		goto main_ac_failed;
	}

	di->nb.notifier_call = k3_battery_monitor_charger_event;
	ret = k3_psy_monitor_register_notifier(&di->nb, 1);
    if(ret){
        dev_err(&pdev->dev, "failed to register k3_psy_monitor_register_notifier\n");
        goto register_notifier_fail;  
    }

    ret = driver_create_file(&(k3_psy_monitor_driver.driver), &driver_attr_CEN_N);
	if (ret < 0)
	{
		pr_err("failed to create sysfs entry(CEN_N): %d\n", ret);
		goto main_charger_failed;
	}

    ret = driver_create_file(&(k3_psy_monitor_driver.driver), &driver_attr_CURRENT);
	if (ret < 0)
	{
		pr_err("failed to create sysfs entry(current): %d\n", ret);
        goto system_file_failed;
	}

	k3_psy_dev_info = di;

	PSY_DBG("k3_psy_monitor:k3_psy_monitor_probe!end!\n");
	return 0;

system_file_failed:
    driver_remove_file(&(k3_psy_monitor_driver.driver), &driver_attr_CEN_N);
register_notifier_fail:
    power_supply_unregister(&di->current_charger_dev.ac);
main_ac_failed:
	power_supply_unregister(&di->current_charger_dev.charger_psy);
main_charger_failed:
#ifdef CONFIG_EXTRA_BATTERY
	power_supply_unregister(&di->extra_battery_dev.battery_psy);
extra_batter_failed:
#endif
	power_supply_unregister(&di->main_battery_dev.battery_psy);

main_batter_failed:
	cancel_delayed_work(&di->k3_psy_monitor_work);
	platform_set_drvdata(pdev, NULL);
err_pdata:
	mutex_destroy(&di->charger_list_lock);
	mutex_destroy(&di->battery_gauge_list_lock);

	mutex_destroy(&di->main_battery_dev.battery_info_lock);
#ifdef CONFIG_EXTRA_BATTERY
	mutex_destroy(&di->extra_battery_dev.battery_info_lock);
#endif
	mutex_destroy(&di->current_charger_dev.charger_info_lock);

	kfree(di);
	di = NULL;

	return ret;
}

static int __devexit k3_psy_monitor_remove(struct platform_device *pdev)
{
	struct k3_psy_monitor_device_info *di = platform_get_drvdata(pdev);

    driver_remove_file(&(k3_psy_monitor_driver.driver),
			   &driver_attr_CEN_N);
	driver_remove_file(&(k3_psy_monitor_driver.driver),
			   &driver_attr_CURRENT);

	cancel_delayed_work(&di->k3_psy_monitor_work);
	k3_psy_monitor_unregister_notifier(&di->nb, 1);

	flush_scheduled_work();

	mutex_destroy(&di->charger_list_lock);
	mutex_destroy(&di->battery_gauge_list_lock);

	mutex_destroy(&di->main_battery_dev.battery_info_lock);
    #ifdef CONFIG_EXTRA_BATTERY
	mutex_destroy(&di->extra_battery_dev.battery_info_lock);
    #endif
	mutex_destroy(&di->current_charger_dev.charger_info_lock);

	power_supply_unregister(&di->main_battery_dev.battery_psy);
    #ifdef CONFIG_EXTRA_BATTERY
	power_supply_unregister(&di->extra_battery_dev.battery_psy);
    #endif
    power_supply_unregister(&di->current_charger_dev.charger_psy);
	power_supply_unregister(&di->current_charger_dev.ac);   

	platform_set_drvdata(pdev, NULL);

	kfree(di);

	di = NULL;

	return 0;
}

#ifdef CONFIG_PM
static int k3_psy_monitor_suspend(struct platform_device *pdev,
	pm_message_t state)
{
	struct k3_psy_monitor_device_info *di = platform_get_drvdata(pdev);

	cancel_delayed_work(&di->k3_psy_monitor_work);
	return 0;
}

static int k3_psy_monitor_resume(struct platform_device *pdev)
{
	struct k3_psy_monitor_device_info *di = platform_get_drvdata(pdev);

	schedule_delayed_work(&di->k3_psy_monitor_work, 0);

	return 0;
}
#else
#define k3_psy_monitor_suspend	NULL
#define k3_psy_monitor_resume	NULL
#endif

static struct platform_driver k3_psy_monitor_driver = {
	.probe		= k3_psy_monitor_probe,
	.remove		= __devexit_p(k3_psy_monitor_remove),
	.suspend		= k3_psy_monitor_suspend,
	.resume		= k3_psy_monitor_resume,
	.driver		= {
		.name	= "k3_psy_monitor",
	},
};

static int __init k3_psy_monitor_init(void)
{
	return platform_driver_register(&k3_psy_monitor_driver);
}
subsys_initcall(k3_psy_monitor_init);

static void __exit k3_psy_monitor_exit(void)
{
	platform_driver_unregister(&k3_psy_monitor_driver);
}
module_exit(k3_psy_monitor_exit);

MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:k3_psy_monitor");
MODULE_AUTHOR("S10 Inc");

