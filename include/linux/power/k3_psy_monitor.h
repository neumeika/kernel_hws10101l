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
ÐÞ¶©ÀúÊ·

==============================================================================*/
#ifndef _PSY_MONITOR_H
#define _PSY_MONITOR_H

#include <linux/list.h>
#include <linux/workqueue.h>


#define MAIN_BATTERY  "MainBattery"
#define EXTRA_BATTERY  "ExtraBattery"
#define MAIN_CHARGER  "MainCharger"

#define PSY_WQ_NAME  "psy_monitor_wq"

#define BATTERY_MONITOR_INTERVAL_PSY     (60)//(10)

#define	BATTERY_CAPACITY_FULL			(100)
#define	BATTERY_LOW_CAPACITY			(15)

#define CHARGER_MAX_CURRENT		2000
#define CHARGER_MAX_VOLTAGE		12000
#define BATTERY_MAX_VOLTAGE		4200
#define BATTERY_MIN_VOLTAGE		3000

/* INTERFACES FOR BC1.1 DRVIER etc*/
#define	CHG_USB_CDP_INSERT_EVENT		(0x0001)
#define	CHG_USB_DCP_INSERT_EVENT		(0x0002)
#define	CHG_USB_SDP_INSERT_EVENT		(0x0003)
#define CHG_BOTTOM_EXIST_EVENT			(0X0004)
#define CHG_REMOVED_EVENT               (0x0005)
#define CHG_AC_INSERT_EVENT             (0x0006)
#define CHG_AC_REMOVED_EVENT            (0x0007)

enum battery_report_property {
	/* Properties of type `int' */
	BATTERY_PROP_TECHNOLOGY	= 0,
	BATTERY_PROP_PRESENT ,
	BATTERY_PROP_TEMPERATURE ,
	BATTERY_PROP_VOLTAGE ,
	BATTERY_PROP_CURRENT ,
	BATTERY_PROP_CAPACITY ,
	BATTERY_PROP_TIME_TO_EMPTY ,
	BATTERY_PROP_TIME_TO_FULL ,
	BATTERY_PROP_HEALTH,
	BATTERY_PROP_SDP_ONLINE,
	BATTERY_PROP_DCP_ONLINE,
	BATTERY_PROP_MAX
};

#define	BAT_FULL_VOL							(4200)
#define	BAT_SLOW_VOL							(3300)
#define	LOW_VOL									(3500)
#define	HIGH_VOL								(4440)
#define	VOL_STEP								(20)

#define	LOW_CURRENT								(550)
#define	HIGH_CURRENT							(1500)
#define	HIGH_TERM_CURRENT						(350)
#define	CURRENT_STEP							(75)
#define	TERM_CURRENT_OFFSET						(50)
#define	TERM_CURRENT_STEP						(50)


//#define PSY_MONITOR_DEBUG_FLAG
#if defined(PSY_MONITOR_DEBUG_FLAG)
#define	PSY_DBG(format, arg...)		do { printk(KERN_ALERT format, ## arg);  } while (0)
#define	PSY_ERR(format, arg...)		do { printk(KERN_ERR format, ## arg);  } while (0)
#define	PSY_FAT(format, arg...)		do { printk(KERN_CRIT format, ## arg); } while (0)
#else
#define	PSY_DBG(format, arg...)		do { (void)(format); } while (0)
#define	PSY_ERR(format, arg...)		do { (void)(format); } while (0)
#define	PSY_FAT(format, arg...)		do { (void)(format); } while (0)
#endif



struct k3_battery_info
{
	unsigned int 	battery_status;
	unsigned int    battery_technology;
	int 			battery_present;
	unsigned int	battery_capacity;
	int 			battery_voltage;
	s32 			battery_current;
	s32 			battery_temperture;
	int 			battery_health;
	int 			battery_max_voltage;
	int 			battery_min_voltage;
	int 			battery_time2empty;
	int				battery_time2full;
};

struct k3_charger_info
{
	int charger_online;
    int ac_online;
	int charger_valid;
	int charger_type;
	int charger_max_charging_current;
};

struct k3_battery_gauge_dev {
	const char name[100];
    struct list_head battery_list;

	int (*get_battery_info)(int prop, struct k3_battery_info* battery_info);  // prop : 1<<(enum battery_report_property)
};

struct k3_charger_dev{
	const char name[100];
    struct list_head charger_list;

	int (*charging_switch)(void *dev_id, int on_off);
	int (*charging_current_control)(void *dev_id, int level);
    int (*irq_request)(void);
    void (*irq_free)(void);
    int (*get_gpio_value)(void);
};

struct k3_psy_monitor_platform_data {
	unsigned int monitoring_interval;

	unsigned int max_charger_currentmA;
	unsigned int max_charger_voltagemV;

	unsigned int max_bat_voltagemV;
	unsigned int low_bat_voltagemV;
};

int k3_psy_monitor_register_notifier(struct notifier_block *nb, unsigned int events);
int k3_psy_monitor_unregister_notifier(struct notifier_block *nb,unsigned int events);
int k3_psy_monitor_register_charger(struct k3_charger_dev* charger);
int k3_psy_monitor_unregister_charger(struct k3_charger_dev* charger);
int k3_psy_monitor_register_battery(struct k3_battery_gauge_dev* battery);
int k3_psy_monitor_unregister_battery(struct k3_battery_gauge_dev* battery);

//for coulomb firmware update only
#ifdef CONFIG_BQ275X0_FIRMWARE_UPDATE
void k3_charger_set_power(int on);
void k3_psy_monitor_stop_sync(void);
void k3_psy_monitor_restart_sync(void);
#endif

#endif
