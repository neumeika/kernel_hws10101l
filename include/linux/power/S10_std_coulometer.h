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

#ifndef _S10_STD_COULOMETER_H_
#define _S10_STD_COULOMETER_H_

#include <linux/power_supply.h>
#include <linux/power/S10_psy_lock.h>

#define BATTERY_NAME_LEN (30)

/* 充电低温保护，0度 */
#define	THREASHOLD_COLD		(0)
/* 10度温度保护，0-10度时充电电流有限制 */
#define	THRESHOLD_COOL	    (100)
/* 高温温度保护，45度 */
#define THRESHOLD_WARM      (450) 
/* 温度保护取消的温度变化值,2度 */
#define	DELTA_TEMP	    (20)

// 电池温度状态
enum battery_temperature_status
{
	/* 温度高于45C */
	TEMP_HIGH,
	/* 温度在0-45度之间 */
	TEMP_NOMAL,
	/* 温度低于 0度 */
	TEMP_LOW,
};
struct battery_psy_info
{
    int battery_status;
    int battery_technology;
    int battery_id;
    int battery_present;
    int battery_health;
    int battery_voltage;
    int battery_current;
    int battery_capacity;
    int battery_temperature;
    int battery_voltage_max_design;
    int battery_voltage_min_design;
    int battery_capacity_full_design;
    int battery_capacity_empty_design;

    int battery_capacity_full_charge;
    int battery_cycle_count;
    int time2full_now;
    int time2full_avg;
    int time2empty_now;
    int time2empty_avg;
    int battery_cycle1; 
    int battery_cycle2; 
};

struct S10_std_battery_private_ops
{
    int  (*battery_type_get)(void* dev, int* type);
    int  (*battery_firmware_download)(unsigned char* firmware_path);
    int  (*battery_get_firmware_version)(void);
    void (*battery_reg_dump)(void);
    int  (*battery_aging_detect)(char* name);
};

struct S10_std_battery_smooth_data
{
    unsigned int dalta_capacity_report_interval;
    unsigned int capacity_stable_count;
};

struct S10_std_battery_device
{
    struct device       *                 dev;
    struct list_head                      node;
    char                                  battery_name[BATTERY_NAME_LEN];
    char                                  firmware_name[BATTERY_NAME_LEN];
    int                                   battery_psy_visible;
    int                                   battery_dev_type; //  main battery; extral battery; corn battery
    int                                   battery_repeat_charging_flag;
    enum battery_temperature_status       temperature_status;
    enum power_supply_type                psy_type; // battery; ups; main; usb; usb dcp; usb cdp; usb aca
    enum power_supply_property *          S10_battery_support_props;
    unsigned int                          num_properties;
    struct battery_psy_info               battery_report_info;
    struct battery_psy_info               battery_detect_info;
    struct battery_psy_info               battery_track_info;
    struct S10_std_battery_smooth_data battery_smooth_data;
    struct S10_power_lock              data_lock;

    int (*battery_report_get_property)(struct power_supply *psy,
                                       enum power_supply_property psp,
                                       union power_supply_propval *val);
    int (*battery_detect_get_property)(char* name,
                                       enum power_supply_property psp,
                                       union power_supply_propval *val);
    struct S10_std_battery_private_ops* ops;
};

#endif
