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
#ifndef _S10_STD_CHARGER_H_
#define _S10_STD_CHARGER_H_

#include <linux/power/S10_psy_lock.h>
#include <linux/power_supply.h>

struct charger_psy_info
{
    int charger_online;
    int charger_type;
    int charging_phase;
    int charging_status;
    int charger_health;
};

union charger_max_current_set
{
    int  max_current_real;
    int* currentArray;
};

struct S10_std_charger_dev_info
{
    int                           charger_valid;
    int                           charging_cut_off_current;
    int                           charging_cut_off_voltage;
    int                           charging_enable_status;
    union charger_max_current_set max_in_current_set;
    union charger_max_current_set max_charging_current_set;
};

struct S10_std_charger_ops
{
    int (*charging_enable)(void *dev, int enable);

    int (*charger_chip_enable)(void *dev, int enable);

    int (*charge_mode_set)(void *dev, int charge_mode);
    int (*charger_get_real_property)(enum power_supply_property psp,
                                     union power_supply_propval *val);

    int (*charging_max_charging_current_get)(void* dev, int* charging_current_limit);
    int (*charging_max_charging_current_set)(void* dev, int current_limit);
    int (*charging_max_input_current_get)(void* dev, int* input_current_limit);
    int (*charging_max_input_current_set)(void* dev, int current_limit);

    int (*charging_cutoff_current_set)(void* dev, int cutoff_current);
    int (*charging_cutoff_voltage_set)(void* dev, int cutoff_voltage);

    int (*charger_firmware_download)(void);

    int (*charging_plug_notify_call)(bool can_schedule, void * dev);    // charger plug in & out notify call
    int (*charging_event_notify_call)(bool can_schedule, void * dev);
};

struct charger_plug_extral_notifier
{
    unsigned long         event;
    struct notifier_block charger_plug_notifier;
};

struct S10_std_charger_device
{
    char                 name[100];
    struct device       *dev;
    struct list_head     node;

    bool software_control_charging;            //软件控制充电
    bool software_control_repeat_charging;     //软件控制电池复充
    bool software_control_charging_over;
    bool software_control_charging_accord_temp;

    bool charger_temp_protect_enable;

    int                                 charging_done_flag;// 正在充电标记，充满后为1.

    int                                 charger_type;
    int                                 charger_online;
    struct charger_plug_extral_notifier plug_extral_notifier;

    enum power_supply_type      psy_type; // battery; ups; main; usb; usb dcp; usb cdp; usb aca
    enum power_supply_property *S10_charger_support_props;
    unsigned int                num_properties;
    int                         (*charger_report_get_property)(struct power_supply *psy,
                                                               enum power_supply_property psp,
                                                               union power_supply_propval *val);

    int (*charger_report_get_property_usb)(struct power_supply *psy,
                                           enum power_supply_property psp,
                                           union power_supply_propval *val);
    struct power_supply*               std_psy_ac;
    struct power_supply*               std_psy_usb;
    int                                factory_charging_stop_limit;
    struct charger_psy_info            charger_report_info;
    struct charger_psy_info            charger_real_info;
    struct charger_psy_info            charger_track_info;
    struct S10_power_lock           data_lock;
    struct S10_std_charger_dev_info dev_info;
    struct S10_std_charger_ops*     ops;
    void*                              charger_private_info;
};

#endif
