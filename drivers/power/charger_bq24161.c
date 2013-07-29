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
==============================================================================*/
#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/io.h>
#include <linux/power_supply.h>
#include <linux/usb/hiusb_android.h>
#include <linux/power/S10_std_psy.h>
#include <linux/power/S10_std_charger.h>
#include <linux/power/bq24161.h>
#include <linux/wakelock.h>
#include <linux/timer.h>
#include <linux/timex.h>
#include <linux/rtc.h>
#include <linux/err.h>
#include <linux/kthread.h>
// 器件version
#define BQ24161_VERSION (0x02)
// 寄存器个数
#define TOTAL_REG_NUM (8)
// 定义寄存器
#define REG_STATUS_CONTROL (0x00)
#define REG_BATTERY_AND_SUPPLY_STATUS (0x01)
#define REG_CONTROL_REGISTER (0x02)
#define REG_BATTERY_VOLTAGE (0x03)
#define REG_BATTERY_CURRENT (0x05)
#define REG_DPPM_VOLTAGE (0x06)
#define REG_SAFETY_TIMER (0x07)

// Vender/Part/Revision Register
#define REG_PART_REVISION (0x04)
#define BQ24161_VERSION_MSK (0x18)

#define TIMER_RST (1 << 7)

// 设置充电电压时的移位
#define VOLTAGE_SHIFT (2)

// Safety Timer/NTC Monitor Register
// Timer slowed by 2x when in thermal regulation
#define ENABLE_TMR_PIN (1 << 7)

// 6 hour fast charge
#define TMR_X_6 (1 << 5)

// 7 hour fast charge
#define TMR_X_9 (1 << 6)

// TS function enabled
#define ENABLE_TS_PIN (1 << 3)

// Enable no battery operation
#define EN_NOBATOP (1 << 0)

/* USB input VIN-DPM voltage shift bit */
#define USB_INPUT_DPPM_SHIFT (3)

// 定义器件地址
#define I2C_ADDR_BQ24161 (0x6B)
#define BQ24161_GPIO_074 (GPIO_9_2)

// 充电基准电压、上限电压，电压步长
#define LOW_VOL (3500)
#define HIGH_VOL (4440)
#define VOL_STEP (20)

// 充电基准电流、上限电流，电流步长
// 截止基准电流、截止电流步长
#define LOW_CURRENT (550)
#define HIGH_CURRENT (2500)
#define HIGH_TERM_CURRENT (350)
#define CURRENT_STEP (75)
#define TERM_CURRENT_OFFSET (50)
#define TERM_CURRENT_STEP (50)


//动态电源管理基准电压、上限电压、步长
#define LOW_DPPM_VOLTAGE (4200)
#define HIGH_DPPM_VOLTAGE (4760)
#define DPPM_VOLTAGE_STEP (80)

#define BAT_FULL_VOL (4200)
#define BAT_SLOW_VOL (3300)

//喂狗时间间隔20秒
#define	KICK_WATCHDOG_TIME					(20)

/* 踢狗超时寄存器数值 */
#define REG0_WTD_TIMEOUT (0x03)
#define REG0_BATT_FAULT (0x07)
#define REG3_VOLT_DEFAULT (0x14)

/* 使能位翻转的产生延时ms */
#define ENABLE_TIME_DELAY (100)

static struct i2c_client * g_bq24161_i2c_client = NULL;
static struct S10_std_charger_device* charger_bq24161 = NULL;
static struct task_struct *k3wdt_kick_task = NULL;

struct bq24161_dev_info
{
    int                           charging_sleep_flag;
    struct bq24161_platform_data* pdata;
    struct wake_lock              charger_wake_lock;
};

static enum power_supply_property bq24161_support_props[] =
{
    //POWER_SUPPLY_PROP_STATUS,
    POWER_SUPPLY_PROP_ONLINE,
    POWER_SUPPLY_PROP_TYPE,
    //POWER_SUPPLY_PROP_HEALTH,
};
/* 适配器接入在DCIN，输入电流限制 */
#define AC_IN_LOW_CUR (1500)
#define AC_IN_HIGH_CUR (2500)
/* 设置DCIN，输入电流限制的移位 */
#define AC_IN_SHIFT (1)
#define AC_IN_MASK (0xFD)
/* 设置DC IN 输入限制电流的寄存器值*/
#define DC_IN_1500MA (0x0)
#define DC_IN_2500MA (0x1)

/* 适配器接入在USB IN，输入电流限制 */
#define USB_IN_LOW_CUR (500)
#define USB_IN_HIGH_CUR (1500)
/* 设置USB IN，输入电流限制的移位 */
#define USB_IN_SHIFT (4)
#define USB_IN_MASK (0x8F)
/* 设置USB IN 输入限制电流的寄存器值*/
#define USB_IN_500MA (0x2)
#define USB_IN_1500MA (0x5)

/* 定义USB快充参数 */
#define USB_CHARGE_MODE_FAST_DPM (4520)
#define USB_CHARGE_MODE_FAST_IN (1500)
#define USB_CHARGE_MODE_FAST_CUR (1500)

/* 定义AC快充参数 */
#ifdef CONFIG_BQ24161_USB_INPUT
#define AC_CHARGE_MODE_FAST_DPM (4440)
#define AC_CHARGE_MODE_FAST_IN (1500)
#define AC_CHARGE_MODE_FAST_CUR (2000)
#else
#define AC_CHARGE_MODE_FAST_DPM (4440)
#define AC_CHARGE_MODE_FAST_IN (2500)
#define AC_CHARGE_MODE_FAST_CUR (2000)
#endif

enum watchdog_time_out
{
    /* 踢狗正常 */
    WTD_NORMAL,

    /* 踢狗超时 */
    WTD_TIME_OUT,

    /* I2C读写错误 */
    WTD_RW_ERRO,
};
static int g_timeout_flag = 0;/* 踢狗超时标记，超时置1，默认值为0 */
static void bq24161_charger_charging_set(struct S10_std_charger_device* charger, int enable_flag);
static int	bq24161_kick_watchdog_work_func(void *data);
int bq24161_kickdog_thread_start(void)
{
    int ret = 0;

    if (NULL == k3wdt_kick_task) // 为空才初始化，并调度。
    {
        k3wdt_kick_task = kthread_create(bq24161_kick_watchdog_work_func, NULL, "bq24161_wdt_kick");
        if (IS_ERR(k3wdt_kick_task))
        {
            power_debug(HW_POWER_CHARGER_IC_ERR, "Unable to start kernel thread./n");
            ret = PTR_ERR(k3wdt_kick_task);
            k3wdt_kick_task = NULL;
        }

        wake_up_process(k3wdt_kick_task);
    }

    return ret;
}

int bq24161_kickdog_thread_stop(void)
{
    int ret = 0;

    if (NULL != k3wdt_kick_task)
    {
        kthread_stop(k3wdt_kick_task);
    }

    k3wdt_kick_task = NULL;

    return ret;
}

struct i2c_client* bq24161_get_i2c_client(void)
{
    return g_bq24161_i2c_client;
}

static int bq24161_write_reg(struct i2c_client *client, u8 value, u8 reg)
{
    int ret = 0;

    ret = i2c_smbus_write_byte_data(client, reg, value);
    if (ret < 0)
    {
        power_debug(HW_POWER_CHARGER_IC_ERR, "[Power ERR: %s, %s, %d]\n", __FILE__, __func__, __LINE__);
    }

    return ret;
}

static int bq24161_read_reg(struct i2c_client *client, u8 *value, u8 reg)
{
    int ret = 0;

    ret = i2c_smbus_read_byte_data(client, reg);
    if (ret < 0)
    {
        power_debug(HW_POWER_CHARGER_IC_ERR, "[Power ERR: %s, %s, %d]\n", __FILE__, __func__, __LINE__);
    }
    else
    {
        *value = ret;
        ret = 0;
    }

    return ret;
}

static int bq24161_write_byte(struct i2c_client *client, u8 value, u8 reg)
{
    return bq24161_write_reg(client, value, reg);
}

static int bq24161_read_byte(struct i2c_client *client, u8 *value, u8 reg)
{
    return bq24161_read_reg(client, value, reg);
}

static void bq24161_config_tmr_rest_watchdog(void)
{
    struct i2c_client * client = bq24161_get_i2c_client();

    bq24161_write_byte(client, TIMER_RST, REG_STATUS_CONTROL);
    return;
}

static int bq24161_charging_done_check(void)
{
    struct i2c_client * client = bq24161_get_i2c_client();
    struct S10_psy_reporter* reporter = S10_power_get_monitor_reporter();
    u8 val = 0;

    bq24161_read_byte(client, &val, REG_STATUS_CONTROL);
    val &= 0x70;
    if (val == 0x50)
    {
        S10_power_lock_lock(&reporter->data_lock);
        reporter->stdcharger->charging_done_flag = 1;
        S10_power_lock_unlock(&reporter->data_lock);
        return 1;
    }
    else
    {
        return 0;
    }
}

static void bq24161_enable_charge(int enable_ce)
{
    u8 value = 0;
    struct i2c_client * client = bq24161_get_i2c_client();

    //获取已经在芯片内部设置的属性，加上电流配置的参数一起写到寄存器
    if (bq24161_read_byte(client, &value, REG_CONTROL_REGISTER) < 0)
    {
        power_debug(HW_POWER_CHARGER_IC_ERR, "[Power ERR: %s, %s, %d]\n", __FILE__, __func__, __LINE__);
        return;
    }

    value = (value & 0x7d) | ((!enable_ce) << 1) | (0x50);
    bq24161_write_byte(client, value, REG_CONTROL_REGISTER);

    return;
}

#if defined (CONFIG_SUPPORT_MICRO_USB_PORT)
static int k3_bq24161_hz_mode(int hz_mode)
{
    u8 value = 0;

    //获取已经在芯片内部设置的属性，加上电流配置的参数一起写到寄存器
    if (bq24161_read_byte(g_bq24161_i2c_client, &value, REG_CONTROL_REGISTER) < 0)
    {
        printk("[Err: %d]\n", __LINE__);
        return -1;
    }

    dev_dbg(&g_bq24161_i2c_client->dev, "LINK>>%s,control reg=%d\n", __func__, value);
    value = (value & 0x7e) | (hz_mode);
    bq24161_write_byte(g_bq24161_i2c_client, value, REG_CONTROL_REGISTER);

    bq24161_read_byte(g_bq24161_i2c_client, &value, REG_CONTROL_REGISTER);
    dev_dbg(&g_bq24161_i2c_client->dev, "LINK>>%s,control reg=%d\n", __func__, value);

    return 0;
}

#endif
#ifdef CONFIG_BQ24161_USB_INPUT
static void bq24161_config_cur_in_limit(unsigned int config_current)
{
    unsigned int currentemA = 0;
    u8 value_reg_old = 0;
    u8 value_reg_cur = 0;

    struct i2c_client* client = bq24161_get_i2c_client();

    /* 当硬件电源接在USB IN引脚时，适配器输出电流限制，0.5A 和1.5A两档 */
    currentemA = config_current;
    if (currentemA >= USB_IN_HIGH_CUR)
    {
        value_reg_cur = USB_IN_1500MA;
    }
    else
    {
        value_reg_cur = USB_IN_500MA;
    }

    /* 获取已经在芯片内部设置的属性，加上电流配置的参数一起写到寄存器 */
    if (bq24161_read_byte(client, &value_reg_old, REG_CONTROL_REGISTER) < 0)
    {
        power_debug(HW_POWER_CHARGER_IC_ERR, "[Power ERR: %s, %s, %d]\n", __FILE__, __func__, __LINE__);
        return;
    }

    value_reg_cur = (value_reg_cur << USB_IN_SHIFT) | (value_reg_old & USB_IN_MASK); /*  保留高1位和低4位配置，0x1000,1111 */

    bq24161_write_byte(client, value_reg_cur, REG_CONTROL_REGISTER);
    power_debug(HW_POWER_CHARGER_IC_DUG, "[Power Dug: %s, %s, %d]: bq24161_config_cur_in_limit VALUE %d \n", __FILE__,
                __func__, __LINE__, value_reg_cur);
    return;
}
#else
static void bq24161_config_cur_in_limit(unsigned int config_current)
{
    unsigned int currentemA = 0;
    u8 value_reg_old = 0;
    u8 value_reg_cur = 0;

    struct i2c_client* client = bq24161_get_i2c_client();

    /* 当硬件电源接在DCIN引脚时，适配器输出电流限制，1.5A,和2.5A两档 */
    currentemA = config_current;
    if (currentemA >= AC_IN_HIGH_CUR)
    {
        value_reg_cur = DC_IN_2500MA;
    }
    else
    {
        value_reg_cur = DC_IN_1500MA;
    }

    /* 获取已经在芯片内部设置的属性，加上电流配置的参数一起写到寄存器 */
    if (bq24161_read_byte(client, &value_reg_old, REG_BATTERY_VOLTAGE) < 0)
    {
        power_debug(HW_POWER_CHARGER_IC_ERR, "[Power ERR: %s, %s, %d]\n", __FILE__, __func__, __LINE__);
        return;
    }

    value_reg_cur = (value_reg_cur << AC_IN_SHIFT) | (value_reg_old & AC_IN_MASK); /* 保留高6位和低1位配置，0x1111,1101 */

    bq24161_write_byte(client, value_reg_cur, REG_BATTERY_VOLTAGE);
    power_debug(HW_POWER_CHARGER_IC_DUG, "[Power Dug: %s, %s, %d]: bq24161_config_ac_in_limit VALUE %d \n", __FILE__,
                __func__, __LINE__, value_reg_cur);
    return;
}
#endif

static void bq24161_config_voltage_reg(unsigned int config_voltage)
{
    unsigned int voltagemV = 0;
    u8 Voreg = 0;
    u8 value_reg_old = 0;

    struct i2c_client * client = bq24161_get_i2c_client();

    // 充电电压设置，取值区间0-63，基准电压3.5V，步长20mV, 最高可设置4.76V
    voltagemV = config_voltage;
    if (voltagemV < LOW_VOL)
    {
        voltagemV = LOW_VOL;
    }
    else if (voltagemV > HIGH_VOL)
    {
        voltagemV = HIGH_VOL;
    }

    Voreg = (voltagemV - LOW_VOL) / VOL_STEP;

    /* 获取已经在芯片内部设置的属性，加上电流配置的参数一起写到寄存器 */
    if (bq24161_read_byte(client, &value_reg_old, REG_BATTERY_VOLTAGE) < 0)
    {
        power_debug(HW_POWER_CHARGER_IC_ERR, "[Power ERR: %s, %s, %d]\n", __FILE__, __func__, __LINE__);
        return;
    }

    Voreg = (Voreg << VOLTAGE_SHIFT) | (value_reg_old & 0x03);/* 保留低2位配置，0x0000,0011。 */
    bq24161_write_byte(client, Voreg, REG_BATTERY_VOLTAGE);

    return;
}

static void bq24161_config_charging_current(unsigned int config_current)
{
    unsigned int currentmA = config_current;
    u8 Vichrg = 0;
    u8 value = 0;

    struct i2c_client * client = bq24161_get_i2c_client();

    //获取已经在芯片内部设置的属性，加上电流配置的参数一起写到寄存器
    if (bq24161_read_byte(client, &value, REG_BATTERY_CURRENT) < 0)
    {
        return;
    }

    // 充电电流设置，取值区间:0-31, 基准电流550mA, 步长75mA ，最大值2875mA
    if (currentmA < LOW_CURRENT)
    {
        currentmA = LOW_CURRENT;
    }
    else if (currentmA > HIGH_CURRENT)
    {
        currentmA = HIGH_CURRENT;
    }

    Vichrg = (currentmA - LOW_CURRENT) / CURRENT_STEP;

    power_debug(HW_POWER_CHARGER_IC_DUG, "[Power Dug: %s, %s, %d]: bq24161_config_charge_current %d  Vichrg%d \n",
                __FILE__, __func__, __LINE__, value, Vichrg);

    value = (value & 0x07) | (Vichrg << 3);
    power_debug(HW_POWER_CHARGER_IC_DUG, "[Power Dug: %s, %s, %d]: bq24161_config_charge_current VALUE %d \n", __FILE__,
                __func__, __LINE__, value);

    bq24161_write_byte(client, value, REG_BATTERY_CURRENT);
    return;
}

static void bq24161_config_input_current(unsigned int config_current)
{}

//配置截止电流
static void bq24161_config_term_current(unsigned int config_term_current)
{
    unsigned int term_currentmA = config_term_current;
    u8 Viterm = 0;
    u8 value = 0;
    struct i2c_client * client = bq24161_get_i2c_client();

    //获取已经在芯片内部设置的属性，加上电流配置的参数一起写到寄存器
    if (bq24161_read_byte(client, &value, REG_BATTERY_CURRENT) < 0)
    {
        power_debug(HW_POWER_CHARGER_IC_ERR, "[Power ERR: %s, %s, %d]\n", __FILE__, __func__, __LINE__);
        return;
    }

    // 充电截止电流设置，取值区间:0-7, 基准截止电流50mA, 步长50mA ，最大值400mA
    if (term_currentmA > HIGH_TERM_CURRENT)
    {
        term_currentmA = HIGH_TERM_CURRENT;
    }

    Viterm = (term_currentmA - TERM_CURRENT_OFFSET) / TERM_CURRENT_STEP;

    value = (value & 0xf8) | Viterm;
    bq24161_write_byte(client, value, REG_BATTERY_CURRENT);
    return;
}

void bq24161_config_dppm_voltage_reg(unsigned int dppm_voltagemV)
{
    u8 VmregIn  = 0;
    u8 VmregUsb = 0;
    u8 Vmreg = 0;
    struct i2c_client * client = bq24161_get_i2c_client();

    // 充电电压设置，取值区间0-7，基准电压4.2V，步长80mV, 最高可设置4.76V
    if (dppm_voltagemV < LOW_DPPM_VOLTAGE)
    {
        dppm_voltagemV = LOW_DPPM_VOLTAGE;
    }
    else if (dppm_voltagemV > HIGH_DPPM_VOLTAGE)
    {
        dppm_voltagemV = HIGH_DPPM_VOLTAGE;
    }

    VmregIn  = (dppm_voltagemV - LOW_DPPM_VOLTAGE) / DPPM_VOLTAGE_STEP;
    VmregUsb = VmregIn; // set USB and IN dppm as the same //nick
    Vmreg = (VmregUsb << USB_INPUT_DPPM_SHIFT) | VmregIn;

    bq24161_write_byte(client, Vmreg, REG_DPPM_VOLTAGE);
    return;
}

void bq24161_config_safety_reg(void)
{
    u8 value = 0;
    struct i2c_client * client = bq24161_get_i2c_client();

    bq24161_read_byte(client, &value, REG_SAFETY_TIMER);

    /*  不设定安全时间，坏电池可能会一直充  */
    value = (value & 0x7F) | TMR_X_6 | TMR_X_9;

    bq24161_write_byte(client, value, REG_SAFETY_TIMER);
    return;
}

void bq24161_open_inner_fet(void)
{
    u8 en_nobatop = 0;
    struct i2c_client * client = bq24161_get_i2c_client();

    bq24161_read_byte(client, &en_nobatop, REG_BATTERY_AND_SUPPLY_STATUS);

    en_nobatop = en_nobatop & (~EN_NOBATOP);

    bq24161_write_byte(client, en_nobatop, REG_BATTERY_AND_SUPPLY_STATUS);
    return;
}

void bq24161_charging_init(void)
{
    int charger_type = 0;
    int charge_mode_flag = 0;
    struct S10_psy_reporter* reporter = S10_power_get_monitor_reporter();
    struct S10_std_charger_device* charger = charger_bq24161;
    struct bq24161_dev_info* dev_info   = charger ? (struct bq24161_dev_info*)charger->charger_private_info : NULL;
    struct bq24161_platform_data* pdata = NULL;

    pdata = dev_info ? dev_info->pdata : NULL;
    if (IS_ERR_OR_NULL(charger) || IS_ERR_OR_NULL(dev_info) || IS_ERR_OR_NULL(pdata) || IS_ERR_OR_NULL(reporter))
    {
        return;
    }

    k3_bq24161_hz_mode(0);/* 关机时可能被设置为高阻态 */
    bq24161_config_voltage_reg(pdata->max_charger_voltagemV);
    bq24161_config_term_current(pdata->termination_currentmA);

    S10_power_lock_lock(&reporter->data_lock);
    charge_mode_flag = reporter->power_monitor_charge_mode_flag;
    S10_power_lock_unlock(&reporter->data_lock);

    S10_power_lock_lock(&charger->data_lock);
    charger_type = charger->charger_type;
    S10_power_lock_unlock(&charger->data_lock);
    power_debug(HW_POWER_CHARGER_IC_DUG, "[: %s, %s, %d,type=%d,mode=%d]\n", __FILE__, __func__, __LINE__,
                charger_type, charge_mode_flag);

    /* 根据适配器的类型设置DPM.和DCIN电流值 */
    switch (charger_type)
    {
    case POWER_SUPPLY_TYPE_USB:
    case POWER_SUPPLY_TYPE_USB_CDP:
    case POWER_SUPPLY_TYPE_UPS:
    {
        if (CHARGE_MODE_NORMAL == charge_mode_flag)
        {
            bq24161_config_dppm_voltage_reg(pdata->chargerUSB_mode_normal_DPM);
            bq24161_config_cur_in_limit(pdata->usb_input_max_currentmA );
        }
        else if (CHARGE_MODE_FAST == charge_mode_flag)
        {
            bq24161_config_dppm_voltage_reg(USB_CHARGE_MODE_FAST_DPM);
            bq24161_config_cur_in_limit(USB_CHARGE_MODE_FAST_IN );
        }
        else
        {
            bq24161_config_dppm_voltage_reg(pdata->chargerUSB_mode_normal_DPM);
            bq24161_config_cur_in_limit(pdata->usb_input_max_currentmA );
        }
    }
        break;
    case POWER_SUPPLY_TYPE_USB_ACA:
    case POWER_SUPPLY_TYPE_USB_DCP:
    case POWER_SUPPLY_TYPE_MAINS:
    {
        if (CHARGE_MODE_NORMAL == charge_mode_flag)
        {
            bq24161_config_dppm_voltage_reg(pdata->chargerAC_mode_normal_DPM);
            bq24161_config_cur_in_limit(pdata->usb_ac_input_max_currentmA );
        }
        else if (CHARGE_MODE_FAST == charge_mode_flag)
        {
            bq24161_config_dppm_voltage_reg(AC_CHARGE_MODE_FAST_DPM);
            bq24161_config_cur_in_limit(AC_CHARGE_MODE_FAST_IN);
        }
        else
        {
            bq24161_config_dppm_voltage_reg(pdata->chargerAC_mode_normal_DPM);
            bq24161_config_cur_in_limit(pdata->usb_ac_input_max_currentmA );
        }
    }
        break;
    default:
    {
        bq24161_config_dppm_voltage_reg(pdata->chargerAC_mode_normal_DPM);
        bq24161_config_cur_in_limit(pdata->usb_ac_input_max_currentmA );
    }
        break;
    }

    bq24161_config_safety_reg();
    bq24161_open_inner_fet();
}

/*****************************************************************************
 函 数 名  : int bq24161_charger_chip_enable(void *dev, int enable)
 功能描述  : 充电芯片置为有效状态，或者高阻状态。
 输入参数  : dev设备名称；enable设置的参数，大于0为有有效状态，小于或等于0为高阻状态。
 输出参数  : 无
 返 回 值  : 无
 调用函数  : 略
 被调函数  : 略
 
 修改历史      :
  1.日    期   : 2012年10月11日
    作    者   : l00220156
    修改内容   : 新生成函数

*****************************************************************************/
int bq24161_charger_chip_enable(void *dev, int enable)
{
    int iret = 0;
    int isetval = 0;

    isetval = enable;

    if ( isetval <=0 )
    {
        isetval = 1;
    }
    else
    {
        isetval = 0;
    }
    
    iret = k3_bq24161_hz_mode(isetval); 
    return iret;
}

int bq24161_charging_enable(void *dev, int enable)
{
    struct S10_std_charger_device* charger = (struct S10_std_charger_device*)dev;
    struct bq24161_dev_info* dev_info = (struct bq24161_dev_info*)charger->charger_private_info;
    struct S10_psy_reporter* reporter = S10_power_get_monitor_reporter();

    if ((enable != 0) && (enable != 1))
    {
        return 0;
    }
    
    charger->dev_info.charging_enable_status = enable;

    if (enable)
    {
        bq24161_charging_init();

        bq24161_enable_charge(!enable);
        mdelay(ENABLE_TIME_DELAY);
        bq24161_enable_charge(enable);
        charger->charging_done_flag = 0;

        if (!wake_lock_active(&dev_info->charger_wake_lock))
        {
            wake_lock(&dev_info->charger_wake_lock);
        }

        /* 插入适配器时调用踢狗进程。看门狗超时，不重复调用踢狗进程 */
        if (WTD_NORMAL == g_timeout_flag)
        {
            bq24161_kickdog_thread_start();
        }
    }
    else
    {
        bq24161_enable_charge(enable);
        /*  停止，关机充电或者USB拔出的时侯，禁止充电，进入休眠状态，不踢狗，不持锁 */
        if ((reporter->psy_monitor_shutdown_charging_flag)
           || (!charger->charger_online))
        {
            bq24161_kickdog_thread_stop();
            if (wake_lock_active(&dev_info->charger_wake_lock))
            {
                wake_unlock(&dev_info->charger_wake_lock);
            }
        }
    }

    power_debug(HW_POWER_CHARGER_IC_ERR,
                "[Power ERR: %s, %s, %d]: enable = %d,  charger->charging_flag = %d, psy_monitor_shutdown_charging_flag=%d\n",
                __FILE__, __func__, __LINE__, enable, charger->charging_done_flag,
                reporter->psy_monitor_shutdown_charging_flag);
    return 0;
}

/*****************************************************************************
 函 数 名  : bq24161_charge_mode_set
 功能描述  :  设置充电器的充电模式
 输入参数  : void *dev
             int charge_mode
 输出参数  : 无
 返 回 值  : 设置成功返回0，错误返回-1
 调用函数  : 略
 被调函数  : 监测设备接口实现

 修改历史      :
  1.日    期   : 2012年11月27日
    作    者   : l00220156
    修改内容   : 新生成函数

*****************************************************************************/
int bq24161_charge_mode_set(void *dev, int charge_mode)
{
    int charge_mode_flag = 0;
    int charger_type = 0;

    struct S10_psy_reporter* reporter = S10_power_get_monitor_reporter();
    struct S10_std_charger_device* charger = charger_bq24161;
    struct bq24161_dev_info* dev_info   = charger ? (struct bq24161_dev_info*)charger->charger_private_info : NULL;
    struct bq24161_platform_data* pdata = NULL;

    pdata = dev_info ? dev_info->pdata : NULL;
    if (IS_ERR_OR_NULL(charger) || IS_ERR_OR_NULL(dev_info) || IS_ERR_OR_NULL(pdata) || IS_ERR_OR_NULL(reporter))
    {
        return -EINVAL;
    }

    S10_power_lock_lock(&reporter->data_lock);
    charge_mode_flag = reporter->power_monitor_charge_mode_flag;
    S10_power_lock_unlock(&reporter->data_lock);

    S10_power_lock_lock(&charger->data_lock);
    charger_type = charger->charger_type;
    S10_power_lock_unlock(&charger->data_lock);
    power_debug(HW_POWER_CHARGER_IC_DUG, "[: %s, %s, %d,type=%d,mode=%d]\n", __FILE__, __func__, __LINE__,
                charger_type, charge_mode_flag);

    /* 根据适配器的类型设置DPM.和DCIN电流值,充电电流值 */
    switch (charger_type)
    {
    case POWER_SUPPLY_TYPE_USB:
    case POWER_SUPPLY_TYPE_USB_CDP:
    case POWER_SUPPLY_TYPE_UPS:
    {
        if (CHARGE_MODE_NORMAL == charge_mode_flag)
        {
            bq24161_config_dppm_voltage_reg(pdata->chargerUSB_mode_normal_DPM);
            bq24161_config_cur_in_limit(pdata->usb_dcp_input_max_currentmA );
            bq24161_config_charging_current(pdata->usb_charging_max_currentmA);
        }
        else if (CHARGE_MODE_FAST == charge_mode_flag)
        {
            bq24161_config_dppm_voltage_reg(USB_CHARGE_MODE_FAST_DPM);
            bq24161_config_cur_in_limit(USB_CHARGE_MODE_FAST_IN );
            bq24161_config_charging_current(USB_CHARGE_MODE_FAST_CUR);
        }
        else
        {
            bq24161_config_dppm_voltage_reg(pdata->chargerUSB_mode_normal_DPM);
            bq24161_config_cur_in_limit(pdata->usb_dcp_input_max_currentmA );
            bq24161_config_charging_current(pdata->usb_charging_max_currentmA);
        }
    }
        break;
    case POWER_SUPPLY_TYPE_USB_ACA:
    case POWER_SUPPLY_TYPE_USB_DCP:
    case POWER_SUPPLY_TYPE_MAINS:
    {
        if (CHARGE_MODE_NORMAL == charge_mode_flag)
        {
            bq24161_config_dppm_voltage_reg(pdata->chargerAC_mode_normal_DPM);
            bq24161_config_cur_in_limit(pdata->usb_ac_input_max_currentmA );
            bq24161_config_charging_current(pdata->usb_ac_charging_max_currentmA);
        }
        else if (CHARGE_MODE_FAST == charge_mode_flag)
        {
            bq24161_config_dppm_voltage_reg(AC_CHARGE_MODE_FAST_DPM);
            bq24161_config_cur_in_limit(AC_CHARGE_MODE_FAST_IN);
            bq24161_config_charging_current(AC_CHARGE_MODE_FAST_CUR);
        }
        else
        {
            bq24161_config_dppm_voltage_reg(pdata->chargerAC_mode_normal_DPM);
            bq24161_config_cur_in_limit(pdata->usb_ac_input_max_currentmA );
            bq24161_config_charging_current(pdata->usb_charging_max_currentmA);
        }
    }
        break;
    default:
    {
        bq24161_config_dppm_voltage_reg(pdata->chargerAC_mode_normal_DPM);
        bq24161_config_cur_in_limit(pdata->usb_ac_input_max_currentmA );
        bq24161_config_charging_current(pdata->usb_ac_charging_max_currentmA);
    }
        break;
    }

    return 0;
}

int bq24161_get_real_property(enum power_supply_property  psp,
                              union power_supply_propval *val)
{
    struct S10_std_charger_device* charger = charger_bq24161;
    struct S10_psy_monitor_dev_info* monitordev_info = NULL;

    switch (psp)
    {
    case POWER_SUPPLY_PROP_ONLINE:
        S10_power_lock_lock(&charger->data_lock);
        val->intval = charger->charger_online;
        S10_power_lock_unlock(&charger->data_lock);
        power_debug(HW_POWER_CHARGER_IC_DUG, "[Power Dug: %s, %s, %d]: bq24161 online:%d \n", __FILE__, __func__,
                    __LINE__, val->intval);

        break;
    case POWER_SUPPLY_PROP_TYPE:
        S10_power_lock_lock(&charger->data_lock);
        val->intval = charger->charger_type;
        S10_power_lock_unlock(&charger->data_lock);
        break;
    case POWER_SUPPLY_PROP_STATUS:
        monitordev_info = S10_power_get_monitor_devinfo();
        S10_power_lock_lock(&charger->data_lock);
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

        S10_power_lock_unlock(&charger->data_lock);
        break;
    case POWER_SUPPLY_PROP_HEALTH:
        val->intval = POWER_SUPPLY_HEALTH_GOOD;
        break;
    default:
        break;
    }

    return 0;
}

int bq24161_max_input_current_get(void* dev, int* input_current_limit)
{
    struct S10_std_charger_device* charger = (struct S10_std_charger_device*)dev;
    struct bq24161_dev_info* dev_info   = charger ? (struct bq24161_dev_info*)charger->charger_private_info : NULL;
    struct bq24161_platform_data *pdata = NULL;

    pdata = dev_info ? dev_info->pdata : NULL;
    if (IS_ERR_OR_NULL(charger) || IS_ERR_OR_NULL(dev_info) || IS_ERR_OR_NULL(pdata))
    {
        return -EINVAL;
    }

    switch (charger->charger_type)
    {
    case POWER_SUPPLY_TYPE_USB:
        *input_current_limit = pdata->usb_input_max_currentmA;
        break;
    case POWER_SUPPLY_TYPE_USB_CDP:
        *input_current_limit = pdata->usb_cdp_input_max_currentmA;
        break;
    case POWER_SUPPLY_TYPE_USB_DCP:
        *input_current_limit = pdata->usb_dcp_input_max_currentmA;
        break;
    case POWER_SUPPLY_TYPE_USB_ACA:
        *input_current_limit = pdata->usb_ac_input_max_currentmA;
        break;
    default:
        *input_current_limit = pdata->usb_input_max_currentmA;
        break;
    }

    return 0;
}

int bq24161_max_input_current_set (void* dev, int val)
{
    struct S10_std_charger_device* charger = (struct S10_std_charger_device*)dev;
    struct bq24161_dev_info* dev_info   = charger ? (struct bq24161_dev_info*)charger->charger_private_info : NULL;
    struct bq24161_platform_data *pdata = NULL;
    int input_current_limit = 0;

    pdata = dev_info ? dev_info->pdata : NULL;

    if (IS_ERR_OR_NULL(charger) || IS_ERR_OR_NULL(dev_info) || IS_ERR_OR_NULL(pdata))
    {
        return -EINVAL;
    }

    if (val > 0)
    {
        input_current_limit = val;
    }
    else
    {
        switch (charger->charger_type)
        {
        case POWER_SUPPLY_TYPE_USB:
            input_current_limit = pdata->usb_input_max_currentmA;
            break;
        case POWER_SUPPLY_TYPE_USB_CDP:
            input_current_limit = pdata->usb_cdp_input_max_currentmA;
            break;
        case POWER_SUPPLY_TYPE_USB_DCP:
            input_current_limit = pdata->usb_dcp_input_max_currentmA;
            break;
        case POWER_SUPPLY_TYPE_USB_ACA:
            input_current_limit = pdata->usb_ac_input_max_currentmA;
            break;
        default:
            input_current_limit = pdata->usb_input_max_currentmA;
            break;
        }
    }

    bq24161_config_input_current(input_current_limit);
    return 0;
}

int bq24161_max_charging_current_set (void* dev, int val)
{
    int charger_type = 0;
    int charge_mode_flag = 0;
    struct S10_psy_reporter* reporter = S10_power_get_monitor_reporter();
    struct S10_std_charger_device* charger = (struct S10_std_charger_device*)dev;
    struct bq24161_dev_info* dev_info   = charger ? (struct bq24161_dev_info*)charger->charger_private_info : NULL;
    struct bq24161_platform_data *pdata = NULL;
    int max_charging_current_limit = 0;

    pdata = dev_info ? dev_info->pdata : NULL;

    if (IS_ERR_OR_NULL(charger) || IS_ERR_OR_NULL(dev_info) || IS_ERR_OR_NULL(pdata))
    {
        return -EINVAL;
    }

    S10_power_lock_lock(&reporter->data_lock);
    charge_mode_flag = reporter->power_monitor_charge_mode_flag;
    S10_power_lock_unlock(&reporter->data_lock);

    S10_power_lock_lock(&charger->data_lock);
    charger_type = charger->charger_type;
    S10_power_lock_unlock(&charger->data_lock);

    switch (charger_type)
    {
    case POWER_SUPPLY_TYPE_USB:
    case POWER_SUPPLY_TYPE_USB_CDP:
    case POWER_SUPPLY_TYPE_UPS:
    {
        if (CHARGE_MODE_NORMAL == charge_mode_flag)
        {
            max_charging_current_limit = pdata->usb_charging_max_currentmA;
        }
        else if (CHARGE_MODE_FAST == charge_mode_flag)
        {
            max_charging_current_limit = USB_CHARGE_MODE_FAST_CUR;
        }
        else
        {
            max_charging_current_limit = pdata->usb_charging_max_currentmA;
        }
    }
        break;
    case POWER_SUPPLY_TYPE_USB_ACA:
    case POWER_SUPPLY_TYPE_USB_DCP:
    case POWER_SUPPLY_TYPE_MAINS:
        if (CHARGE_MODE_NORMAL == charge_mode_flag)
        {
            max_charging_current_limit = pdata->usb_ac_charging_max_currentmA;
        }
        else if (CHARGE_MODE_FAST == charge_mode_flag)
        {
            max_charging_current_limit = AC_CHARGE_MODE_FAST_CUR;
        }
        else
        {
            max_charging_current_limit = pdata->usb_ac_charging_max_currentmA;
        }

        break;
    default:
        max_charging_current_limit = pdata->usb_charging_max_currentmA;
        break;
    }

    /* 如果需要设置的充电电流值大于充电器类型允许的最大值，充电器允许值优先，以保护充电器/适配器 */
    if (val > max_charging_current_limit)
    {
        val = max_charging_current_limit;
    }

    /* 保存当前设定的最大充电电流值，供产线查询功能使用 */
    charger->dev_info.max_charging_current_set.max_current_real = val;

    bq24161_config_charging_current(val);
    return 0;
}

int bq24161_max_charging_current_get(void* dev, int* max_charging_current_limit)
{
    int charger_type = 0;
    int charge_mode_flag = 0;
    struct S10_psy_reporter* reporter = S10_power_get_monitor_reporter();
    struct S10_std_charger_device* charger = (struct S10_std_charger_device*)dev;
    struct bq24161_dev_info* dev_info   = charger ? (struct bq24161_dev_info*)charger->charger_private_info : NULL;
    struct bq24161_platform_data *pdata = NULL;
    int max_charging_current_set = 0;

    pdata = dev_info ? dev_info->pdata : NULL;
    if (IS_ERR_OR_NULL(charger) || IS_ERR_OR_NULL(dev_info) || IS_ERR_OR_NULL(pdata))
    {
        return -EINVAL;
    }

    max_charging_current_set = charger->dev_info.max_charging_current_set.max_current_real;

    S10_power_lock_lock(&reporter->data_lock);
    charge_mode_flag = reporter->power_monitor_charge_mode_flag;
    S10_power_lock_unlock(&reporter->data_lock);

    S10_power_lock_lock(&charger->data_lock);
    charger_type = charger->charger_type;
    S10_power_lock_unlock(&charger->data_lock);

    switch (charger_type)
    {
    case POWER_SUPPLY_TYPE_USB:
    case POWER_SUPPLY_TYPE_USB_CDP:
    case POWER_SUPPLY_TYPE_UPS:
    {
        if (CHARGE_MODE_NORMAL == charge_mode_flag)
        {
            *max_charging_current_limit = pdata->usb_charging_max_currentmA;
        }
        else if (CHARGE_MODE_FAST == charge_mode_flag)
        {
            *max_charging_current_limit = USB_CHARGE_MODE_FAST_CUR;
        }
        else
        {
            *max_charging_current_limit = pdata->usb_charging_max_currentmA;
        }
    }
        break;
    case POWER_SUPPLY_TYPE_USB_ACA:
    case POWER_SUPPLY_TYPE_USB_DCP:
    case POWER_SUPPLY_TYPE_MAINS:
        if (CHARGE_MODE_NORMAL == charge_mode_flag)
        {
            *max_charging_current_limit = pdata->usb_ac_charging_max_currentmA;
        }
        else if (CHARGE_MODE_FAST == charge_mode_flag)
        {
            *max_charging_current_limit = AC_CHARGE_MODE_FAST_CUR;
        }
        else
        {
            *max_charging_current_limit = pdata->usb_ac_charging_max_currentmA;
        }

        break;
    default:
        *max_charging_current_limit = pdata->usb_charging_max_currentmA;
        break;
    }

    if (max_charging_current_set > (*max_charging_current_limit))
    {
        *max_charging_current_limit = max_charging_current_set;
    }

    return 0;
}

int bq24161_cutoff_current_set (void* dev, int cutoff_current)
{
    struct S10_std_charger_device* charger = (struct S10_std_charger_device*)dev;

    charger->dev_info.charging_cut_off_current = cutoff_current;
    bq24161_config_term_current(cutoff_current);
    return 0;
}

int bq24161_cutoff_voltage_set(void* dev, int cutoff_voltage)
{
    struct S10_std_charger_device* charger = (struct S10_std_charger_device*)dev;

    charger->dev_info.charging_cut_off_voltage = cutoff_voltage;
    return 0;
}

#if 0
static void detect_dc_cable(void)
{
    unsigned int reg_value = 0;
    long int event = 0;

    reg_value = gpio_request(GPIO_2_4, "24161_charging_inout");
    if (reg_value)
    {
        printk("bq24161 request gpio20 failed!\n");
        return reg_value;
    }

    reg_value = gpio_get_value(GPIO_2_4);

    gpio_free(GPIO_2_4);

    dev_info(charger_bq24161->dev, "LINK>>%s,reg_value=%d\n", reg_value);
    if (reg_value)
    {
        event = CHG_AC_PULGIN_EVENT;
    }
    else
    {
        event = CHG_AC_PULGOUT_EVENT;
    }

    dev_info(charger_bq24161->dev, "LINK>>%s,event = %d\n", __func__, event);
    S10_power_monitor_notifier_call_chain(event, NULL);
}

static irqreturn_t dc_in_interrupt(int irq, void *_di)
{
    detect_dc_cable();
    return IRQ_HANDLED;
}

static irqreturn_t dc_out_interrupt(int irq, void *_di)
{
    detect_dc_cable();
    return IRQ_HANDLED;
}

static int dc_request_irq()
{
    int ret;

    ret = request_threaded_irq(gpio_to_irq(GPIO_2_4),
                               NULL, dc_in_interrupt,
                               IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING,
                               "BQ24161_DCIN", NULL);
    if (ret)
    {
        printk(KERN_ERR "Cannot request irq %d for DC (%d)\n", gpio_to_irq(gpio_to_irq(GPIO_2_4)), ret);
    }

    return ret;
}

#endif

int bq24161_firmware_download(void)
{
    return 0;
}

int bq24161_plug_notify_call(bool can_schedule, void * dev)   // charger plug in & out notify call
{
    return 0;
}

int bq24161_event_notify_call(bool can_schedule, void * dev)
{
    return 0;
}
struct S10_std_charger_ops bq24161_private_ops =
{
    .charging_enable                   = bq24161_charging_enable,
    .charger_chip_enable               = bq24161_charger_chip_enable,
    .charge_mode_set                   = bq24161_charge_mode_set,
    .charger_get_real_property         = bq24161_get_real_property,
    .charging_max_charging_current_get = bq24161_max_charging_current_get,
    .charging_max_charging_current_set = bq24161_max_charging_current_set,
    .charging_max_input_current_get    = bq24161_max_input_current_get,
    .charging_max_input_current_set    = bq24161_max_input_current_set,
    .charging_cutoff_current_set       = bq24161_cutoff_current_set,
    .charging_cutoff_voltage_set       = bq24161_cutoff_voltage_set,
    .charger_firmware_download         = bq24161_firmware_download,
    .charging_plug_notify_call         = bq24161_plug_notify_call,
    .charging_event_notify_call        = bq24161_event_notify_call,
};
int bq24161_report_get_property(struct power_supply *       psy,
                                enum power_supply_property  psp,
                                union power_supply_propval *val)
{
    struct S10_std_charger_device* charger = charger_bq24161;

    int charger_type_ac = 0;

    if (IS_ERR_OR_NULL(charger))
    {
        return -EINVAL;
    }

    S10_power_lock_lock(&charger->data_lock);
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

    S10_power_lock_unlock(&charger->data_lock);
    power_debug(HW_POWER_CHARGER_IC_DUG, "[Power Dug: %s, %s, %d,%d]:  psp = %d \n", __FILE__, __func__, __LINE__, psp,
                val->intval);

    return 0;
}

int bq24161_report_get_property_usb(struct power_supply *       psy,
                                    enum power_supply_property  psp,
                                    union power_supply_propval *val)
{
    struct S10_std_charger_device* charger = charger_bq24161;
    int charger_type_usb = 0;

    if (IS_ERR_OR_NULL(charger))
    {
        return -EINVAL;
    }

    S10_power_lock_lock(&charger->data_lock);
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

    S10_power_lock_unlock(&charger->data_lock);

    power_debug(HW_POWER_CHARGER_IC_DUG, "[Power Dug: %s, %s, %d,%d]:  psp = %d \n", __FILE__, __func__, __LINE__, psp,
                val->intval);
    return 0;
}
/*****************************************************************************
 函 数 名  : charger_bq24161_log_print
 功能描述  :  打印充电芯片寄存器值
*****************************************************************************/
static void charger_bq24161_reg_dump(void)
{
    int i = 0;
    u8 value = 0;
    struct i2c_client* client = NULL;
    struct timex txc;
    struct rtc_time tm;

    if (HW_POWER_CHARGER_REG_DUMP == (power_debug_mask & HW_POWER_CHARGER_REG_DUMP))
    {
        do_gettimeofday(&(txc.time));
        rtc_time_to_tm(txc.time.tv_sec, &tm);
        printk("UTC time :%d-%02d-%02d %02d:%02d:%02d \n", tm.tm_year + 1900, tm.tm_mon + 1,
               tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec);

        client = bq24161_get_i2c_client();
        if (client != NULL)
        {
            printk("bq24161 ");
            for (i = 0; i < TOTAL_REG_NUM; i++)
            {
                bq24161_read_reg(client, &value, i);
                printk("reg%d:%x;", i, value);
            }

            printk("\n");
        }
        else
        {
            printk("erro :client NULL!\n");
        }
    }
}

/*****************************************************************************
 函 数 名  : watchdog_timeout_check
 功能描述  :  看门狗踢狗，超时检测
 输入参数  : 无
 输出参数  : 无
 返 回 值  : WTD_RW_ERRO读寄存器错误；WTD_NORMAL未超时，WTD_TIME_OUT超时。
 调用函数  : 略
 被调函数  : bq24161_kick_watchdog_work_func

 修改历史      :
  1.日    期   : 2012年11月26日
    作    者   : l00220156
    修改内容   : 新生成函数

*****************************************************************************/
static int watchdog_timeout_check(void)
{
    u8 value_reg3 = 0, value_reg0 = 0;
    int iret = 0;
    int time_fault = 0;
    struct i2c_client* client = NULL;

    client = bq24161_get_i2c_client();
    if (NULL == client)
    {
        power_debug(HW_POWER_CHARGER_IC_ERR, "[Power ERR: %s, %s, %d]\n", __FILE__, __func__, __LINE__);
        return WTD_RW_ERRO;
    }

    iret = bq24161_read_reg(client, &value_reg3, REG_BATTERY_VOLTAGE);
    if (iret < 0)
    {
        power_debug(HW_POWER_CHARGER_IC_ERR, "[Power ERR: %s, %s, %d]\n", __FILE__, __func__, __LINE__);
        return WTD_RW_ERRO;
    }

    iret = bq24161_read_reg(client, &value_reg0, REG_STATUS_CONTROL);
    if (iret < 0)
    {
        power_debug(HW_POWER_CHARGER_IC_ERR, "[Power ERR: %s, %s, %d]\n", __FILE__, __func__, __LINE__);
        return WTD_RW_ERRO;
    }

    /* 通过0号寄存器的低三位和电池电压设置寄存器判断是否喂狗超时 */
    time_fault = (REG3_VOLT_DEFAULT == value_reg3) || (REG0_BATT_FAULT == (value_reg0 & REG0_BATT_FAULT))
                 || (REG0_WTD_TIMEOUT == (value_reg0 & REG0_WTD_TIMEOUT));
    if (WTD_TIME_OUT == time_fault)
    {
        power_debug(HW_POWER_CHARGER_IC_ERR, "[Power ERR: %s, %s, %d] watchdog timeout!!\n", __FILE__, __func__,
                    __LINE__);
        return WTD_TIME_OUT;
    }

    return WTD_NORMAL;
}
static int bq24161_kick_watchdog_work_func(void *data)
{
    int done_flag = 0;
    struct bq24161_dev_info* dev_info = NULL;
    struct S10_psy_reporter* reporter = NULL;
    struct S10_std_charger_device* charger = NULL;

    dev_info = (struct bq24161_dev_info*)charger_bq24161->charger_private_info;
    reporter = S10_power_get_monitor_reporter();
    charger = charger_bq24161;

    while (1)
    {
        g_timeout_flag = WTD_NORMAL; /* 踢狗超时标记位复位 */

        bq24161_config_tmr_rest_watchdog();
        charger_bq24161_reg_dump();

        g_timeout_flag = watchdog_timeout_check();

        switch (g_timeout_flag)
        {
        case WTD_TIME_OUT:
            bq24161_charger_charging_set(charger, charger->charger_online);
            g_timeout_flag = WTD_NORMAL;
            break;
        case WTD_RW_ERRO:
            power_debug(HW_POWER_CHARGER_IC_ERR, "[WTD_RW_ERRO: %s, %s, %d]\n", __FILE__, __func__, __LINE__);
            break;
        case WTD_NORMAL:
            break;
        default:
            break;
        }


        bq24161_charging_done_check();
        S10_power_lock_lock(&reporter->data_lock);
        done_flag = reporter->stdcharger->charging_done_flag;
        S10_power_lock_unlock(&reporter->data_lock);

        if (done_flag)
        {
            /* 关机充电标志判断 */
            if (reporter->psy_monitor_shutdown_charging_flag)
            {
                bq24161_enable_charge(0);/* 配合关机充电，不吃VBUS电 */

                if (wake_lock_active(&dev_info->charger_wake_lock))
                {
                    wake_unlock(&dev_info->charger_wake_lock);
                }

                k3wdt_kick_task = NULL;
                do_exit(-1);
            }
        }

        /* 进程执行控制 */
        if (kthread_should_stop())
        {
            power_debug(HW_POWER_CHARGER_IC_ERR, "exit kick dog thread\n");
            break;
        }
		set_current_state(TASK_INTERRUPTIBLE);
        schedule_timeout(KICK_WATCHDOG_TIME * HZ);
        set_current_state(TASK_RUNNING);
        /* 进程执行控制 */
    }
    return 0;
}

/* 根据充电器类型，设定充电电流的大小 */
static void bq24161_charger_charging_set(struct S10_std_charger_device* charger, int enable_flag)
{
    int charging_current = 0;
    int charge_mode_flag = 0;
    int charger_type = 0;
    struct S10_psy_reporter* reporter = S10_power_get_monitor_reporter();
    struct bq24161_dev_info* dev_info   = charger ? (struct bq24161_dev_info*)charger->charger_private_info : NULL;
    struct bq24161_platform_data* pdata = NULL;

    pdata = dev_info ? dev_info->pdata : NULL;
    if (IS_ERR_OR_NULL(charger) || IS_ERR_OR_NULL(dev_info) || IS_ERR_OR_NULL(pdata) || IS_ERR_OR_NULL(reporter))
    {
        return;
    }

    S10_power_lock_lock(&reporter->data_lock);
    charge_mode_flag = reporter->power_monitor_charge_mode_flag;
    S10_power_lock_unlock(&reporter->data_lock);

    S10_power_lock_lock(&charger->data_lock);
    charger_type = charger->charger_type;
    S10_power_lock_unlock(&charger->data_lock);
    power_debug(HW_POWER_CHARGER_IC_DUG, "[: %s, %s, %d,type=%d,mode=%d]\n", __FILE__, __func__, __LINE__,
                charger_type, charge_mode_flag);
    if (enable_flag)
    {
        switch (charger_type)

        {
        case POWER_SUPPLY_TYPE_USB:
        case POWER_SUPPLY_TYPE_USB_CDP:
        case POWER_SUPPLY_TYPE_UPS:
        {
            if (CHARGE_MODE_NORMAL == charge_mode_flag)
            {
                charging_current = pdata->usb_charging_max_currentmA;
            }
            else if (CHARGE_MODE_FAST == charge_mode_flag)
            {
                charging_current = USB_CHARGE_MODE_FAST_CUR;
            }
            else
            {
                charging_current = pdata->usb_charging_max_currentmA;
            }
        }
            break;
        case POWER_SUPPLY_TYPE_USB_ACA:
        case POWER_SUPPLY_TYPE_USB_DCP:
        case POWER_SUPPLY_TYPE_MAINS:
            if (CHARGE_MODE_NORMAL == charge_mode_flag)
            {
                charging_current = pdata->usb_ac_charging_max_currentmA;
            }
            else if (CHARGE_MODE_FAST == charge_mode_flag)
            {
                charging_current = AC_CHARGE_MODE_FAST_CUR;
            }
            else
            {
                charging_current = pdata->usb_ac_charging_max_currentmA;
            }

            break;
        default:
            charging_current = pdata->usb_charging_max_currentmA;
            break;
        }

        bq24161_config_charging_current(charging_current);
    }

    bq24161_charging_enable((void *)charger, enable_flag);
}

static void bq24161_charger_dect(void)
{
    unsigned long event = 0;

    struct S10_std_charger_device* charger = charger_bq24161;

    if (ioread32(PMU_STATUS0_ADDR) & (VBUS4P5_D10 | VBUS_COMP_VBAT_D20))

    {
        S10_power_lock_lock(&charger->data_lock);

        charger->charger_online = 1;

        S10_power_lock_unlock(&charger->data_lock);
    }
    else

    {
        S10_power_lock_lock(&charger->data_lock);

        charger->charger_online = 0;

        S10_power_lock_unlock(&charger->data_lock);
    }

    switch (charger->plug_extral_notifier.event)

    {
    case CHARGER_TYPE_USB:
#if defined(CONFIG_SUPPORT_MICRO_USB_PORT)     
    case CHARGER_TYPE_MHL_POWER:
#endif
        event = CHG_USB_SDP_PULGIN_EVENT;

        S10_power_lock_lock(&charger->data_lock);

        charger->charger_type = POWER_SUPPLY_TYPE_USB;

        S10_power_lock_unlock(&charger->data_lock);

        bq24161_charger_charging_set(charger, 1);

        break;

    case CHARGER_TYPE_NON_STANDARD:

        event = CHG_USB_CDP_PULGIN_EVENT;

        S10_power_lock_lock(&charger->data_lock);

        charger->charger_type = POWER_SUPPLY_TYPE_USB_CDP;

        S10_power_lock_unlock(&charger->data_lock);

        bq24161_charger_charging_set(charger, 1);

        break;

    case CHARGER_TYPE_BC_USB:

        event = CHG_USB_DCP_PULGIN_EVENT;

        S10_power_lock_lock(&charger->data_lock);

        charger->charger_type = POWER_SUPPLY_TYPE_USB_DCP;

        S10_power_lock_unlock(&charger->data_lock);

        bq24161_charger_charging_set(charger, 1);

        break;

    case CHARGER_TYPE_STANDARD:

        event = CHG_AC_PULGIN_EVENT;

        S10_power_lock_lock(&charger->data_lock);

        charger->charger_type = POWER_SUPPLY_TYPE_USB_ACA;

        S10_power_lock_unlock(&charger->data_lock);

        bq24161_charger_charging_set(charger, 1);

        break;
#if defined (CONFIG_SUPPORT_MICRO_USB_PORT)
    case CHARGER_TYPE_USB_OTG_LINE:
    case CHARGER_TYPE_MHL_NO_POWER:

        S10_power_lock_lock(&charger->data_lock);
        charger->charger_online = 0;
        S10_power_lock_unlock(&charger->data_lock);
        
        k3_bq24161_hz_mode(1);

        break;

    case CHARGER_USB_OTG_LINE_REMOVED:
    case CHARGER_TYPE_MHL_REMOVED:
        S10_power_lock_lock(&charger->data_lock);
        charger->charger_online = 0;
        S10_power_lock_unlock(&charger->data_lock);
        
        k3_bq24161_hz_mode(0);

        break;
#endif

    case CHARGER_REMOVED:

        event = CHG_CHARGER_PULGOUT_EVENT;

        bq24161_charger_charging_set(charger, 0);

        break;

    default:

        event = S10_PWR_NOTIFY_EVENT_MAX;

        break;
    }

    //inform monitor charger status has changed!

    S10_power_monitor_notifier_call_chain(event, NULL);

    return;
}

static int bq24161_plug_notifier_call(struct notifier_block *nb,

                                      unsigned long event, void *data)
{
    struct S10_std_charger_device* charger = charger_bq24161;

    charger->plug_extral_notifier.event = event;

    bq24161_charger_dect();

    return NOTIFY_OK;
}

/************************************************************************************************
* 函数名称	：k3_bq24161_charger_probe
* 功能描述	：充电IC驱动初始化
* 输入参数	：
* 输出参数	：
* 返 回 值	：
************************************************************************************************/

static int __devinit bq24161_charger_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    u8 read_reg = 0;

    u8 value = 0;

    int ret = 0;

    int i = 0;

    enum usb_charger_type plugin_stat = CHARGER_REMOVED;

    struct bq24161_platform_data *pdata = NULL;

    struct bq24161_dev_info* dev_info = NULL;
    struct S10_std_charger_device* charger = NULL;

    struct S10_psy_monitor_dev_info* monitordev_info = NULL;

    unsigned long event;

    g_bq24161_i2c_client = client;

    pdata = client->dev.platform_data;
    if (!pdata)
    {
        dev_err(&client->dev, "pdata is NULL!\n");
        return -EINVAL;
    }

    dev_info = kzalloc(sizeof(struct bq24161_dev_info), GFP_KERNEL);
    if (!dev_info)
    {
        ret = -ENOMEM;
        goto EXIT;
    }

    dev_info->pdata = pdata;

    //register bq275x0 i2c client to battery report driver
    charger = kzalloc(sizeof(struct S10_std_charger_device), GFP_KERNEL);
    if (!charger)
    {
        ret = -ENOMEM;
        goto EXIT;
    }

    charger_bq24161 = charger;
    dev_info->charging_sleep_flag = 0;
    charger->charger_private_info = (void *)dev_info;

    ret = bq24161_read_reg(client, &read_reg, REG_PART_REVISION);
    if (ret < 0)
    {
        power_debug(HW_POWER_CHARGER_IC_ERR, "[Power ERR: %s, %s, %d]\n", __FILE__, __func__, __LINE__);
        ret = -EINVAL;
        goto err_i2c;
    }

    if (((read_reg & BQ24161_VERSION_MSK) == 0x00) && (client->addr == I2C_ADDR_BQ24161))
    {
        power_debug(HW_POWER_CHARGER_IC_DUG, "[Power DUG: %s, %s, %d]: bqchip_version: 0x02\n", __FILE__, __func__,
                    __LINE__);
    }
    else
    {
        power_debug(HW_POWER_CHARGER_IC_ERR, "[Power ERR: %s, %s, %d]\n", __FILE__, __func__, __LINE__);
        ret = -EINVAL;
        goto err_i2c;
    }

    ret = pdata->bq24161_io_block_config(1);
    if (ret)
    {
        goto err_io;
    }

    wake_lock_init(&dev_info->charger_wake_lock, WAKE_LOCK_SUSPEND, "bq24161");

    S10_power_lock_init(S10_POWER_LOCK_LIGHT, &charger->data_lock);



    strcpy(charger->name, AC_CHARGER);
    charger->dev = &(client->dev);

    charger->psy_type = POWER_SUPPLY_TYPE_MAINS;/* 注册type为AC的电量上报节点 */
    charger->charging_done_flag = 0;
    charger->software_control_charging = 1;
    charger->software_control_charging_over = 0;
    charger->software_control_charging_accord_temp = 0;
    charger->software_control_repeat_charging = 1;
    charger->charger_report_get_property  = bq24161_report_get_property;
    charger->S10_charger_support_props = bq24161_support_props;
    charger->num_properties = ARRAY_SIZE(bq24161_support_props);
    charger->ops = &bq24161_private_ops;

    /* 如果是支持USB充电，则需添加 USB 上报节点对应的上报函数，上报节点在检测设备中会根据设置创建 */
    monitordev_info = S10_power_get_monitor_devinfo();
    if (monitordev_info->usb_charging_support)
    {
        charger->charger_report_get_property_usb = bq24161_report_get_property_usb;
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

    charger->plug_extral_notifier.charger_plug_notifier.notifier_call = bq24161_plug_notifier_call;

#if defined (CONFIG_SUPPORT_MICRO_USB_PORT)
    ret = hiusb_charger_registe_notifier(&charger->plug_extral_notifier.charger_plug_notifier);
    if (ret < 0)
    {
        power_debug(HW_POWER_CHARGER_IC_ERR, "[Power ERR: %s, %s, %d] hiusb notifier register failed!\n", __FILE__,
                    __func__, __LINE__);
        goto err_register2notifier;
    }

    //detect_dc_cable();

    plugin_stat = get_charger_name();
    power_debug(HW_POWER_CHARGER_IC_DUG, "[Power DUG: %s, %s, %d] plugin_stat = %d \n", __FILE__, __func__, __LINE__,
                plugin_stat);
    if (CHARGER_TYPE_USB == plugin_stat)
    {
        event = CHG_USB_SDP_PULGIN_EVENT;
        charger->charger_type   = POWER_SUPPLY_TYPE_USB;
        charger->charger_online = 1;
    }
    else if (CHARGER_TYPE_NON_STANDARD == plugin_stat)
    {
        event = CHG_USB_CDP_PULGIN_EVENT;
        charger->charger_type   = POWER_SUPPLY_TYPE_USB_CDP;
        charger->charger_online = 1;
    }
    else if (CHARGER_TYPE_BC_USB == plugin_stat)
    {
        event = CHG_USB_DCP_PULGIN_EVENT;
        charger->charger_type   = POWER_SUPPLY_TYPE_USB_DCP;
        charger->charger_online = 1;
    }
    else if (CHARGER_TYPE_STANDARD == plugin_stat)
    {
        event = CHG_AC_PULGIN_EVENT;
        charger->charger_type   = POWER_SUPPLY_TYPE_USB_ACA;
        charger->charger_online = 1;
    }
    else
    {
        event = CHG_CHARGER_PULGOUT_EVENT;
        charger->charger_type   = POWER_SUPPLY_TYPE_MAINS;
        charger->charger_online = 0;
    }

    bq24161_charger_charging_set(charger, charger->charger_online);
    S10_power_monitor_notifier_call_chain(event, NULL);
#endif

    for (i = 0; i < 8; i++)
    {
        bq24161_read_reg(client, &value, i);
        power_debug(HW_POWER_CHARGER_IC_DUG, "[Power DUG: %s, %s, %d]: reg%d:%x \n", __FILE__, __func__, __LINE__, i,
                    value);
    }

    power_debug(HW_POWER_CHARGER_IC_ERR, "[Power DUG: %s, %s, %d] probe OK! \n", __FILE__, __func__, __LINE__);

    return 0;

err_register2notifier:
    charger2monitor_unregister(charger);
err_register2monitor:
    wake_lock_destroy(&dev_info->charger_wake_lock);
    S10_power_lock_deinit(&charger->data_lock);
    pdata->bq24161_io_block_config(0);
err_io:
err_i2c:
    kfree(charger);
    charger_bq24161 = NULL;
EXIT:
    kfree(dev_info);
    g_bq24161_i2c_client = NULL;
    power_debug(HW_POWER_CHARGER_IC_ERR, "[Power DUG: %s, %s, %d] probe failed, ret = %d !\n", __FILE__, __func__,
                __LINE__, ret);
    return ret;
}

/************************************************************************************************
* 函数名称	：bq24161_charger_remove
* 功能描述	: 驱动卸载处理函数
* 输入参数	：
* 输出参数	：
* 返 回 值	：
************************************************************************************************/
static int __devexit bq24161_charger_remove(struct i2c_client *client)
{
    struct S10_std_charger_device* charger = charger_bq24161;
    struct bq24161_dev_info * dev_info  = (struct bq24161_dev_info *)charger->charger_private_info;
    struct bq24161_platform_data *pdata = dev_info->pdata;

#if defined (CONFIG_SUPPORT_MICRO_USB_PORT)
    hiusb_charger_unregiste_notifier(&charger->plug_extral_notifier.charger_plug_notifier);
#endif

    charger2monitor_unregister(charger);

    bq24161_kickdog_thread_stop();


    wake_lock_destroy(&dev_info->charger_wake_lock);
    S10_power_lock_deinit(&charger->data_lock);
    pdata->bq24161_io_block_config(0);

    kfree(dev_info);
    kfree(charger);
    charger_bq24161 = NULL;
    g_bq24161_i2c_client = NULL;
    power_debug(HW_POWER_CHARGER_IC_DUG, "[Power Dug: %s, %s, %d]\n", __FILE__, __func__, __LINE__);
    return 0;
}

/************************************************************************************************
* 函数名称	：bq24161_charger_shutdown
* 功能描述	: 深度休眠接口
* 输入参数	：
* 输出参数	：
* 返 回 值	：
************************************************************************************************/
static void bq24161_charger_shutdown(struct i2c_client *client)
{
    k3_bq24161_hz_mode(1);/* 关机后立即停止充电，配合关机后,充电器在位时系统重启 */
    power_debug(HW_POWER_CHARGER_IC_DUG, "[Power Dug: %s, %s, %d]\n", __FILE__, __func__, __LINE__);
    return;
}

static const struct i2c_device_id bq24161_id[] =
{
    { "bq24161_charger", 0 },
    {},
};

#ifdef CONFIG_PM

/************************************************************************************************
* 函数名称	：k3_bq24161_charger_suspend
* 功能描述	: 休眠接口
* 输入参数	：
* 输出参数	：
* 返 回 值	：
************************************************************************************************/
static int bq24161_charger_suspend(struct i2c_client *client, pm_message_t state)
{
    power_debug(HW_POWER_CHARGER_IC_DUG, "[Power Dug: %s, %s, %d]\n", __FILE__, __func__, __LINE__);
    return 0;
}

/************************************************************************************************
* 函数名称	：k3_bq24161_charger_resume
* 功能描述	: 唤醒休眠接口
* 输入参数	：
* 输出参数	：
* 返 回 值	：
************************************************************************************************/
static int bq24161_charger_resume(struct i2c_client *client)
{
    power_debug(HW_POWER_CHARGER_IC_DUG, "[Power Dug: %s, %s, %d]\n", __FILE__, __func__, __LINE__);
    return 0;
}

#else
 #define bq24161_charger_suspend NULL
 #define bq24161_charger_resume NULL
#endif

MODULE_DEVICE_TABLE(i2c, bq24161);

static struct i2c_driver bq24161_charger_driver =
{
    .probe     = bq24161_charger_probe,
    .remove    = __devexit_p(bq24161_charger_remove),
    .suspend   = bq24161_charger_suspend,
    .resume    = bq24161_charger_resume,
    .shutdown  = bq24161_charger_shutdown,
    .id_table  = bq24161_id,

    .driver    = {
        .owner = THIS_MODULE,
        .name  = "bq24161_charger",
    },
};

/************************************************************************************************
* 函数名称	：k3_bq24161_charger_init
* 功能描述	: 加载充电IC驱动
* 输入参数	：
* 输出参数	：
* 返 回 值	：
************************************************************************************************/
static int __init bq24161_charger_init(void)
{
    return i2c_add_driver(&bq24161_charger_driver);
}

/************************************************************************************************
* 函数名称	：bq24161_charger_exit
* 功能描述	: 卸载充电IC驱动
* 输入参数	：
* 输出参数	：
* 返 回 值	：
************************************************************************************************/
static void __exit bq24161_charger_exit(void)
{
    i2c_del_driver(&bq24161_charger_driver);
    return;
}

module_init(bq24161_charger_init);
module_exit(bq24161_charger_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("S10 Inc");
