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
修订历史

==============================================================================*/

#ifndef _S10_STD_PSY_H_
#define _S10_STD_PSY_H_
#include <linux/power/S10_psy_lock.h>
#include <linux/power/S10_std_charger.h>
#include <linux/power/S10_std_coulometer.h>

#define S10_PWR_MONITOR_FIRST_REPORT_DELAY (300) // 300ms

#define S10_PWR_MONITOR_INTERVAL_PSY (30) //(10)

#define S10_PWR_MONITOR_WQ_NAME "S10_pwr_monitor_wq"

#define S10_REPORT_BATTERY_MAX_NUM (5)

#define BATTERY_LOW_TRIGGER (10)

#define DEFAULT_FACTORY_CHARGING_FULL_CAPACITY (60)

/* 过滤函数中使用，电量变化1%时，上报电池充电状态 */
#define DELTA_CAPACITY_REPORT (1)
/* 过滤函数中使用，温度变化0.5摄氏度时(单位0.1度)，上报电池充电状态 */
#define DELTA_TEMP_REPORT (5)

#define AC_CHARGER "ChargerAC"
#define USB_CHARGER "ChargerUSB"
#define MAIN_BATTERY "MainBattery"
#define EXTRA_BATTERY "ExtraBattery"

struct S10_psy_monitor_dev_info
{
    unsigned int monitoring_interval;
    unsigned int usb_charging_support;
    unsigned int usb_charging_display_support;
    unsigned int dock_charging_support;
    unsigned int dock_charging_display_support;
    unsigned int coin_battery_support;
    unsigned int coin_battery_display_support;
    unsigned int psy_repeat_charging_capacity;
};

struct S10_psy_node
{
    struct power_supply psy;
    struct list_head    node;
};

struct S10_psy_reporter
{
    struct S10_psy_monitor_dev_info*     dev_info;

    struct S10_power_lock battery_list_lock;
    struct list_head         battery_list;
    struct S10_power_lock psy_list_lock;
    struct list_head         powersupply_list;

    struct S10_power_lock data_lock;
    struct wake_lock         psy_reporter_wake_lock;

    unsigned int                            report_battery_num;
    unsigned char                           report_battery_name_array[S10_REPORT_BATTERY_MAX_NUM][BATTERY_NAME_LEN];
    struct S10_std_charger_device*       stdcharger;

    unsigned int                            psy_monitor_running_flag;
    unsigned int                            psy_monitor_shutdown_charging_flag;
    unsigned int                            psy_repeat_charging_detect_enable;
    unsigned int                            factory_charging_mode_enable;
    unsigned int                            factory_charging_stop_flag;
    unsigned int                            factory_charging_full_capacity;
    unsigned int                            factory_charging_current_set_flag;
    unsigned int                            power_monitor_charge_mode_flag;
    
    struct notifier_block                   psy_monitor_nb;
    //struct work_struct                      psy_notifier_work;
    struct delayed_work                     psy_notifier_work;
    struct delayed_work                     psy_monitor_work;
    struct delayed_work                     vbat_low_dect_work;
    struct workqueue_struct*                psy_monitor_wq;
};

enum S10_power_notify_event_type
{
    CHG_USB_CDP_PULGIN_EVENT = 1,
    CHG_USB_DCP_PULGIN_EVENT,
    CHG_USB_SDP_PULGIN_EVENT,
    CHG_DOCK_PULGIN_EVENT,
    CHG_AC_PULGIN_EVENT,
    CHG_CHARGER_PULGOUT_EVENT,
    PSY_INFO_CHANGE_EVENT,
    S10_PWR_NOTIFY_EVENT_MAX,
};

enum S10_power_monitor_report_type
{
    PSY_NOT_REPORT,
    BATTERY_INFO_REPORT,
    CHARGER_INFO_REPORT,
    PSY_REPORT_ALL,
};

/* 充电模式配置 */
enum std_power_monitor_charge_mode
{
    CHARGE_MODE_DEFAULT,/* 默认充电配置 */
    CHARGE_MODE_NORMAL,/* 标准充电配置 */
    CHARGE_MODE_FAST,/* 快速充电配置 */
};
#define HW_POWER_BATTERY_IC_DUG 1
#define HW_POWER_BATTERY_IC_ERR 1 << 1
#define HW_POWER_CHARGER_IC_DUG 1 << 2
#define HW_POWER_CHARGER_IC_ERR 1 << 3
#define HW_POWER_BATTERY_DATA_COLLECT_ERR 1 << 4
#define HW_POWER_CHARGER_CONTROL_ERR 1 << 5
#define HW_POWER_BATTERY_CAPACITY_SMOOTH_DUG 1 << 6
#define HW_POWER_PSYINFO_REPORT_DUG 1 << 7
#define HW_POWER_PSYINFO_REPORT_ERR 1 << 8
#define HW_POWER_TESTDEV_DUG 1 << 9
#define HW_POWER_TESTDEV_ERR 1 << 10

#define HW_POWER_COUL_REG_DUMP 1 << 11
#define HW_POWER_CHARGER_REG_DUMP 1 << 12
extern int power_debug_mask;
#define power_debug(cause, fmt, ...) \
    if (power_debug_mask & cause) \
        printk(KERN_INFO pr_fmt(fmt), ## __VA_ARGS__)

extern struct blocking_notifier_head notifier_S10_psy;

struct S10_psy_monitor_dev_info* S10_power_get_monitor_devinfo(void);
struct S10_psy_reporter*			S10_power_get_monitor_reporter(void);

int									charger2monitor_register(struct S10_std_charger_device* dev);
int									charger2monitor_unregister(struct S10_std_charger_device* dev);
int									battery2monitor_register(struct S10_std_battery_device* dev);
int									battery2monitor_unregister(struct S10_std_battery_device* dev);

int									S10_power_monitor_notifier_call_chain(unsigned long val, void *data);

struct S10_std_battery_device *	S10_power_get_battery_in_list(const char *name);

void								S10_psy_monitor_start(unsigned int delay_ms);
void								S10_psy_monitor_stop(void);
#endif
