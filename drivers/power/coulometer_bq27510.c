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
=============================================================================*/

#include <linux/device.h>
#include <linux/module.h>
#include <linux/param.h>
#include <linux/workqueue.h>
#include <linux/delay.h>
#include <linux/power_supply.h>
#include <linux/idr.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>

//#include <asm/irq.h>
#include <linux/uaccess.h>
#include <linux/vmalloc.h>
#include <linux/fs.h>
#include <linux/fcntl.h>
#include <linux/ctype.h>
#include <linux/slab.h>

//#include <mach/gpio.h>
#include <linux/power/S10_std_psy.h>
#include <linux/power/S10_std_coulometer.h>
#include <linux/power/bq27510.h>
#include <mach/product_feature_sel.h> 

#define BQ275x0_FIRMWARE_DOWNLOAD_MODE_I2C_ADDR (0x0B)

// 电量计三种状态: 正常工作模式，固件下载模式，中间模式
#define BQ275x0_NORMAL_MODE (0x0)
#define BQ275x0_UPDATE_FIRMWARE_MODE (0x1)
#define BQ275x0_LOCK_MODE (0x2)

#define BQ275x0_REG_CLASS_ID 82

#define CONST_NUM_TEMP 2730

// 定义电量计寄存器
#define     BQ275x0_REG_CTRS 0x0000
#define     BQ275x0_REG_FIRMWARE_ID 0x0008
#define     BQ275x0_REG_FIRMWARE_VERSION 0x0039
#define     BQ275x0_REG_CTRL 0x00
#define     BQ275x0_REG_TEMP 0x06
#define     BQ275x0_REG_VOLT 0x08
#define     BQ275x0_REG_FLAGS 0x0a
#define     BQ275x0_REG_RCAP 0x10
#define     BQ275x0_REG_FCC 0x12
#define     BQ275x0_REG_AI 0x14
#define     BQ275x0_REG_TTE 0x16
#define     BQ275x0_REG_TTF 0x18
#define     BQ275x0_REG_SI 0x1a
#define     BQ275x0_REG_CC 0x2a  
#define     BQ275x0_REG_SOC 0x2C
#define     BQ275x0_REG_DFCLS 0x3e
#define     BQ275x0_REG_FLASH 0x40
#define     BQ275x0_REG_QMAX 0x42
#define     BQ275x0_REG_QMAX1 0x43
#define     BQ275x0_FLAG_FC (1 << 9)      /* Full-charged bit */
#define     BQ275x0_FLAG_DET (1 << 3)
#define     BQ275x0_FLAG_OTC (1 << 15)     /* Over-Temperature-Charge bit */
#define     BQ275x0_FLAG_OTD (1 << 14)     /* Over-Temperature-Discharge bit */
#define     BQ275x0_REG_DFBLK 0x3f
#define     BQ275x0_REG_DFDCNTL 0x61
#define     DCM_BATTERY_CYCLE1  1500
#define     DCM_BATTERY_CYCLE2  3500

#define     NORMAL_BATTERY_CYCLE1  500
#define     NORMAL_BATTERY_CYCLE2  1000
enum batt_age
{
    LEVEL1 = 1,
    LEVEL2,
    LEVEL3,
};
enum bq27510_reg
{
    REG_VOL = 0,
    REG_CUR,
    REG_CAP,
    REG_TMP,
    REG_FCC,
    REG_NUM_MAX,
};
static int g_bq27510_reg_value[REG_NUM_MAX] = {0};
static int bq275x0_work_mode = BQ275x0_NORMAL_MODE;

//下载固件后的5秒钟不可以访问库仑计
static unsigned long locked_timeout_jiffies = 0;
static struct i2c_client * g_bq275x0_i2c_client = NULL;
static struct S10_std_battery_device* battery_bq275x0 = NULL;

#ifdef CONFIG_BQ275X0_FIRMWARE_UPDATE

/* added for Firmware upgrade begine */
 #define BSP_UPGRADE_FIRMWARE_BQFS_CMD "upgradebqfs"
 #define BSP_UPGRADE_FIRMWARE_DFFS_CMD "upgradedffs"

/*库仑计的完整的firmware，包含可执行镜像及数据*/
 #define BSP_UPGRADE_FIRMWARE_BQFS_NAME "/system/etc/coulometer/bq27510_pro.bqfs"

/*库仑计的的数据信息*/
 #define BSP_UPGRADE_FIRMWARE_DFFS_NAME "/system/etc/coulometer/bq27510_pro.dffs"

 #define BSP_FIRMWARE_FILE_SIZE (400 * 1024)
 #define BSP_I2C_MAX_TRANSFER_LEN (128)
 #define BSP_MAX_ASC_PER_LINE (400)
 #define BSP_ENTER_ROM_MODE_CMD (0x00)
 #define BSP_ENTER_ROM_MODE_DATA (0x0F00)
 #define BSP_FIRMWARE_DOWNLOAD_MODE (0xDDDDDDDD)
 #define BSP_NORMAL_MODE (0x00)
/* added for Firmware upgrade end */
#endif

enum power_supply_property bq27510_support_props[] =
{
    POWER_SUPPLY_PROP_STATUS,
    POWER_SUPPLY_PROP_TECHNOLOGY,
    POWER_SUPPLY_PROP_PRESENT,
    POWER_SUPPLY_PROP_HEALTH,
    POWER_SUPPLY_PROP_VOLTAGE_NOW,
    POWER_SUPPLY_PROP_CURRENT_NOW,
    POWER_SUPPLY_PROP_CAPACITY,
    POWER_SUPPLY_PROP_TEMP,
    POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN,
    POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN,
    POWER_SUPPLY_PROP_ENERGY_FULL_DESIGN,
    POWER_SUPPLY_PROP_ENERGY_EMPTY_DESIGN,
    POWER_SUPPLY_PROP_CYCLE_COUNT,
    POWER_SUPPLY_PROP_TIME_TO_FULL_NOW,
    POWER_SUPPLY_PROP_TIME_TO_FULL_AVG,
    POWER_SUPPLY_PROP_TIME_TO_EMPTY_NOW,
    POWER_SUPPLY_PROP_TIME_TO_EMPTY_AVG,

    POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
    POWER_SUPPLY_PROP_CHARGE_FULL,
};

struct i2c_client* bq275x0_get_i2c_client(void)
{
    return g_bq275x0_i2c_client;
}

void bq275x0_set_normal_mode(void)
{
    struct i2c_client* client = bq275x0_get_i2c_client();

    bq275x0_work_mode = BQ275x0_NORMAL_MODE;
    if (client)
    {
        client->addr = BQ275X0_I2C_ADDR;
    }

    return;
}

static bool bq275x0_check_normal_mode(void)
{
    bool ret = false;

    switch (bq275x0_work_mode)
    {
    case BQ275x0_UPDATE_FIRMWARE_MODE:

        // 库仑计处于下载状态，此时不能读取电量、电压等信息
        ret = false;
        break;
    case BQ275x0_NORMAL_MODE:
        ret = true;
        break;
    case BQ275x0_LOCK_MODE:

        //下载固件后的5秒是不可访问的中间态
        if (time_is_before_jiffies(locked_timeout_jiffies))
        {
            bq275x0_work_mode = BQ275x0_NORMAL_MODE;
            ret = true;
        }
        else
        {
            ret = false;
        }

        break;
    default:
        ret = false;
        break;
    }

    power_debug(HW_POWER_BATTERY_IC_DUG, "[Power DUG: %s, %s, %d]  mode = %d !\n", __FILE__, __func__, __LINE__,
                bq275x0_work_mode);
    return ret;
}

static int bq275x0_i2c_word_read(struct i2c_client *client, u8 reg)
{
    int err = 0;

    if (bq275x0_check_normal_mode())
    {
        err = i2c_smbus_read_word_data(client, reg);
        if (err < 0)
        {
            power_debug(HW_POWER_BATTERY_IC_ERR, "[Power ERR: %s, %s, %d]  read failed return %d !\n", __FILE__,
                        __func__, __LINE__, err);
        }
    }
    else
    {
        err = -1;
    }

    return err;
}

/* added for Firmware upgrade begin */
static int bq275x0_i2c_word_write(struct i2c_client *client, u8 reg, u16 value)
{
    int err = 0;

    err = i2c_smbus_write_word_data(client, reg, value);
    if (err < 0)
    {
        power_debug(HW_POWER_BATTERY_IC_ERR, "[Power ERR: %s, %s, %d]  write failed return %d !\n", __FILE__, __func__,
                    __LINE__, err);
    }

    return err;
}

static int bq275x0_i2c_bytes_write(struct i2c_client *client, u8 reg, u8 *pBuf, u16 len)
{
    int i2c_ret = 0;
    int i = 0;
    int j = 0;
    u8 *p = pBuf;

    for (i = 0; i < len; i += I2C_SMBUS_BLOCK_MAX)
    {
        j = ((len - i) > I2C_SMBUS_BLOCK_MAX) ? I2C_SMBUS_BLOCK_MAX : (len - i);
        i2c_ret = i2c_smbus_write_i2c_block_data(client, reg + i, j, p + i);
        if (i2c_ret < 0)
        {
            power_debug(HW_POWER_BATTERY_IC_ERR, "[Power ERR: %s, %s, %d]  write failed return %d !\n", __FILE__,
                        __func__, __LINE__, i2c_ret);
        }
    }

    return i2c_ret;
}

static int bq275x0_i2c_bytes_read(struct i2c_client *client, u8 reg, u8 *pBuf, u16 len)
{
    int ret = 0;
    int i = 0;
    int j = 0;
    u8 *p = pBuf;

    for (i = 0; i < len; i += I2C_SMBUS_BLOCK_MAX)
    {
        j = ((len - i) > I2C_SMBUS_BLOCK_MAX) ? I2C_SMBUS_BLOCK_MAX : (len - i);

        ret = i2c_smbus_read_i2c_block_data(client, reg + i, j, p + i);
        if (ret < 0)
        {
            power_debug(HW_POWER_BATTERY_IC_ERR, "[Power ERR: %s, %s, %d]  read failed return %d !\n", __FILE__,
                        __func__, __LINE__, ret);
        }
    }

    return ret;
}

static int bq275x0_i2c_bytes_read_and_compare(struct i2c_client *client, u8 reg, u8 *pSrcBuf, u8 *pDstBuf, u16 len)
{
    int ret = 0;

    ret = bq275x0_i2c_bytes_read(client, reg, pSrcBuf, len);
    if (ret < 0)
    {
        power_debug(HW_POWER_BATTERY_IC_ERR, "[Power ERR: %s, %s, %d]  read failed return %d !\n", __FILE__, __func__,
                    __LINE__, ret);
        return ret;
    }

    ret = strncmp(pDstBuf, pSrcBuf, len);
    return ret;
}

/* added for Firmware upgrade end */

int bq275x0_battery_temperature(struct i2c_client *client, int* value)
{
    int ret  = 0;
    int data = 0;

    data = bq275x0_i2c_word_read(client, BQ275x0_REG_TEMP);
    if (data < 0)
    {
        ret = data;
        power_debug(HW_POWER_BATTERY_IC_ERR, "[Power ERR: %s, %s, %d]  read failed return %d !\n", __FILE__, __func__,
                    __LINE__, data);
    }
    else
    {
        *value = (data - CONST_NUM_TEMP);
        power_debug(HW_POWER_BATTERY_IC_DUG, "[Power DUG: %s, %s, %d]  temp =  %d !\n", __FILE__, __func__, __LINE__,
                    *value);
    }

    return ret;
}

int bq275x0_battery_voltage(struct i2c_client *client, int* value)
{
    int ret  = 0;
    int data = 0;

    data = bq275x0_i2c_word_read(client, BQ275x0_REG_VOLT);
    if (data < 0)
    {
        ret = data;
        power_debug(HW_POWER_BATTERY_IC_ERR, "[Power ERR: %s, %s, %d]  read failed return %d !\n", __FILE__, __func__,
                    __LINE__, data);
    }
    else
    {
        /*adapt android upper layer unit: uV*/
        *value = data * 1000;
        power_debug(HW_POWER_BATTERY_IC_DUG, "[Power DUG: %s, %s, %d]  voltage =  %d !\n", __FILE__, __func__, __LINE__,
                    *value);
    }

    return ret;
}

int bq275x0_battery_current(struct i2c_client *client, int * value)
{
    int data = 0;
    short nCurr = 0;

    data = bq275x0_i2c_word_read(client, BQ275x0_REG_AI);

    nCurr  = (signed short)data;
    *value = nCurr;
    power_debug(HW_POWER_BATTERY_IC_DUG, "[Power DUG: %s, %s, %d]  current =  %d !\n", __FILE__, __func__, __LINE__,
                *value);

    return 0;
}

int bq275x0_remaining_capacity(struct i2c_client *client, int* value)
{
    int ret  = 0;
    int data = 0;

    data = bq275x0_i2c_word_read(client, BQ275x0_REG_RCAP);
    if (data < 0)
    {
        ret = data;
        power_debug(HW_POWER_BATTERY_IC_ERR, "[Power ERR: %s, %s, %d]  read failed return %d !\n", __FILE__, __func__,
                    __LINE__, data);
    }
    else
    {
        *value = data;
        power_debug(HW_POWER_BATTERY_IC_DUG, "[Power DUG: %s, %s, %d]  remaining capacity =  %d !\n", __FILE__,
                    __func__, __LINE__, *value);
    }

    return ret;
}

int bq275x0_full_charge_capacity(struct i2c_client *client, int* value)
{
    int ret  = 0;
    int data = 0;

    data = bq275x0_i2c_word_read(client, BQ275x0_REG_FCC);
    if (data < 0)
    {
        ret = data;
        power_debug(HW_POWER_BATTERY_IC_ERR, "[Power ERR: %s, %s, %d]  read failed return %d !\n", __FILE__, __func__,
                    __LINE__, data);
    }
    else
    {
        *value = data;
        power_debug(HW_POWER_BATTERY_IC_DUG, "[Power DUG: %s, %s, %d]  full charge capacity =  %d !\n", __FILE__,
                    __func__, __LINE__, *value);
    }

    return ret;
}

int bq275x0_battery_cyclecount(struct i2c_client *client, int* value)
{
    int ret  = 0;
    int data = 0;

    data = bq275x0_i2c_word_read(client, BQ275x0_REG_CC);
    if (data < 0)
    {
        ret = data;
        power_debug(HW_POWER_BATTERY_IC_ERR, "[Power ERR: %s, %s, %d]  read failed return %d !\n", __FILE__, __func__,
                    __LINE__, data);
    }
    else
    {
        *value = data;
        power_debug(HW_POWER_BATTERY_IC_DUG, "[Power DUG: %s, %s, %d]  cyclecount =  %d !\n", __FILE__, __func__,
                    __LINE__, *value);
    }

    return ret;
}

int bq275x0_battery_capacity(struct i2c_client *client, int* value)
{
    int ret  = 0;
    int data = 0;

    data = bq275x0_i2c_word_read(client, BQ275x0_REG_SOC);
    if (data < 0)
    {
        ret = data;
        power_debug(HW_POWER_BATTERY_IC_ERR, "[Power ERR: %s, %s, %d]  read failed return %d !\n", __FILE__, __func__,
                    __LINE__, data);
    }
    else
    {
        *value = data;
        power_debug(HW_POWER_BATTERY_IC_DUG, "[Power DUG: %s, %s, %d]  capacity =  %d !\n", __FILE__, __func__,
                    __LINE__, *value);
    }

    return ret;
}

static int bq275x0_battery_tte(struct i2c_client *client, int* value)
{
    int ret  = 0;
    int data = 0;

    data = bq275x0_i2c_word_read(client, BQ275x0_REG_TTE);
    if (data < 0)
    {
        ret = data;
        power_debug(HW_POWER_BATTERY_IC_ERR, "[Power ERR: %s, %s, %d]  read failed return %d !\n", __FILE__, __func__,
                    __LINE__, data);
    }
    else
    {
        *value = data;
        power_debug(HW_POWER_BATTERY_IC_DUG, "[Power DUG: %s, %s, %d]  tte =  %d !\n", __FILE__, __func__, __LINE__,
                    *value);
    }

    return ret;
}

static int bq275x0_battery_ttf(struct i2c_client *client, int* value)
{
    int ret  = 0;
    int data = 0;

    data = bq275x0_i2c_word_read(client, BQ275x0_REG_TTF);
    if (data < 0)
    {
        ret = data;
        power_debug(HW_POWER_BATTERY_IC_ERR, "[Power ERR: %s, %s, %d]  read failed return %d !\n", __FILE__, __func__,
                    __LINE__, data);
    }
    else
    {
        *value = data;
        power_debug(HW_POWER_BATTERY_IC_DUG, "[Power DUG: %s, %s, %d]  ttf =  %d !\n", __FILE__, __func__, __LINE__,
                    *value);
    }

    return ret;
}

static int bq275x0_battery_present(struct i2c_client *client, int* value)
{
    int ret  = 0;
    int data = 0;

    data = bq275x0_i2c_word_read(client, BQ275x0_REG_FLAGS);

    if (data < 0)
    {
        ret = data;
        power_debug(HW_POWER_BATTERY_IC_ERR, "[Power ERR: %s, %s, %d]  read failed return %d !\n", __FILE__, __func__,
                    __LINE__, data);
    }
    else
    {
        *value = !!(data & BQ275x0_FLAG_DET);
        power_debug(HW_POWER_BATTERY_IC_DUG, "[Power DUG: %s, %s, %d]  present =  %d !\n", __FILE__, __func__, __LINE__,
                    *value);
    }

    return ret;
}

int bq275x0_battery_health(struct i2c_client *client, int* value)
{
    int ret  = 0;
    int data = 0;

    data = bq275x0_i2c_word_read(client, BQ275x0_REG_FLAGS);
    if (data < 0)
    {
        ret = data;
        *value = POWER_SUPPLY_HEALTH_UNKNOWN;
        power_debug(HW_POWER_BATTERY_IC_ERR, "[Power ERR: %s, %s, %d]  read failed return %d !\n", __FILE__, __func__,
                    __LINE__, data);
    }
    else
    {
        power_debug(HW_POWER_BATTERY_IC_DUG, "[Power DUG: %s, %s, %d]  data =  %d !\n", __FILE__, __func__, __LINE__,
                    data);
        if (data & (BQ275x0_FLAG_OTD))
        {
            *value = POWER_SUPPLY_HEALTH_OVERHEAT;
        }
        else
        {
            *value = POWER_SUPPLY_HEALTH_GOOD;
        }

        power_debug(HW_POWER_BATTERY_IC_DUG, "[Power DUG: %s, %s, %d]  health =  %d !\n", __FILE__, __func__, __LINE__,
                    *value);
    }

    return ret;
}

#ifdef CONFIG_BQ275X0_FIRMWARE_UPDATE

/* added for Firmware upgrade begin */
static int bq275x0_atoi(const char *s)
{
    int k = 0;

    k = 0;
    while ((*s != '\0') && (*s >= '0') && (*s <= '9'))
    {
        k = 10 * k + (*s - '0');
        s++;
    }

    return k;
}

static unsigned long bq275x0_strtoul(const char *cp, unsigned int base)
{
    unsigned long result = 0, value;

    while (isxdigit(*cp) && (value = isdigit(*cp) ? *cp - '0' : (islower(*cp)
                                                                 ? toupper(*cp) : *cp) - 'A' + 10) < base)
    {
        result = result * base + value;
        cp++;
    }

    return result;
}

/*Parse the bqfs/dffs file's addr and data*/
static int bq275x0_firmware_program(struct i2c_client *client, const unsigned char *pgm_data, unsigned int filelen)
{
    unsigned int i = 0, j = 0, ulDelay = 0, ulReadNum = 0;
    unsigned int ulCounter = 0, ulLineLen = 0;
    unsigned char temp = 0;
    unsigned char *p_cur;
    unsigned char pBuf[BSP_MAX_ASC_PER_LINE] = { 0 };
    unsigned char p_src[BSP_I2C_MAX_TRANSFER_LEN] = { 0 };
    unsigned char p_dst[BSP_I2C_MAX_TRANSFER_LEN] = { 0 };
    unsigned char ucTmpBuf[16] = { 0 };

bq275x0_firmware_program_begin:
    if (ulCounter > 10)
    {
        return -1;
    }

    p_cur = (unsigned char *)pgm_data;

    while (1)
    {
        if ((p_cur - pgm_data) >= filelen)
        {
            power_debug(HW_POWER_BATTERY_IC_DUG, "[Power DUG: %s, %s, %d]  Download success!\n", __FILE__, __func__,
                        __LINE__);
            break;
        }

        while ((*p_cur == '\r') || (*p_cur == '\n'))
        {
            p_cur++;
        }

        i = 0;
        ulLineLen = 0;

        memset(p_src, 0x00, sizeof(p_src));
        memset(p_dst, 0x00, sizeof(p_dst));
        memset(pBuf, 0x00, sizeof(pBuf));

        while (i < BSP_MAX_ASC_PER_LINE)
        {
            temp = *p_cur++;
            i++;
            if (('\r' == temp) || ('\n' == temp))
            {
                break;
            }

            if (' ' != temp)
            {
                pBuf[ulLineLen++] = temp;
            }
        }

        p_src[0] = pBuf[0];
        p_src[1] = pBuf[1];

        if (('W' == p_src[0]) || ('C' == p_src[0]))
        {
            for (i = 2, j = 0; i < ulLineLen; i += 2, j++)
            {
                memset(ucTmpBuf, 0x00, sizeof(ucTmpBuf));
                memcpy(ucTmpBuf, pBuf + i, 2);
                p_src[2 + j] = bq275x0_strtoul(ucTmpBuf, 16);
            }

            temp = (ulLineLen - 2) / 2;
            ulLineLen = temp + 2;
        }
        else if ('X' == p_src[0])
        {
            memset(ucTmpBuf, 0x00, sizeof(ucTmpBuf));
            memcpy(ucTmpBuf, pBuf + 2, ulLineLen - 2);
            ulDelay = bq275x0_atoi(ucTmpBuf);
        }
        else if ('R' == p_src[0])
        {
            memset(ucTmpBuf, 0x00, sizeof(ucTmpBuf));
            memcpy(ucTmpBuf, pBuf + 2, 2);
            p_src[2] = bq275x0_strtoul(ucTmpBuf, 16);
            memset(ucTmpBuf, 0x00, sizeof(ucTmpBuf));
            memcpy(ucTmpBuf, pBuf + 4, 2);
            p_src[3] = bq275x0_strtoul(ucTmpBuf, 16);
            memset(ucTmpBuf, 0x00, sizeof(ucTmpBuf));
            memcpy(ucTmpBuf, pBuf + 6, ulLineLen - 6);
            ulReadNum = bq275x0_atoi(ucTmpBuf);
        }

        if (':' == p_src[1])
        {
            switch (p_src[0])
            {
            case 'W':
 #if 0
                /* firmware upgrade */
                printk("W: ");
                for (i = 0; i < ulLineLen - 4; i++)
                {
                    printk("%x ", p_src[4 + i]);
                }

                printk(KERN_ERR "\n");
 #endif
                if (bq275x0_i2c_bytes_write(client, p_src[3], &p_src[4], ulLineLen - 4) < 0)
                {
                    power_debug(HW_POWER_BATTERY_IC_ERR, "[Power ERR: %s, %s, %d]  i2c write faile len = %d!\n",
                                __FILE__, __func__, __LINE__, ulLineLen - 4);
                }

                break;
            case 'R':
                if (bq275x0_i2c_bytes_read(client, p_src[3], p_dst, ulReadNum) < 0)
                {
                    power_debug(HW_POWER_BATTERY_IC_ERR, "[Power ERR: %s, %s, %d]  i2c read failed!\n", __FILE__,
                                __func__, __LINE__);
                }

                break;
            case 'C':
                if (bq275x0_i2c_bytes_read_and_compare(client, p_src[3], p_dst, &p_src[4], ulLineLen - 4))
                {
                    ulCounter++;
                    power_debug(HW_POWER_BATTERY_IC_ERR, "[Power ERR: %s, %s, %d]  i2c read and compare failed!\n",
                                __FILE__, __func__, __LINE__);
                    goto bq275x0_firmware_program_begin;
                }

                break;
            case 'X':
                mdelay(ulDelay);
                break;
            default:
                return 0;
            }
        }
    }

    return 0;
}

static int bq275x0_firmware_data_download(struct i2c_client *client, const unsigned char *pgm_data, unsigned int len)
{
    int iRet;

    bq275x0_work_mode = BQ275x0_UPDATE_FIRMWARE_MODE;

    /*Enter Rom Mode */
    iRet = bq275x0_i2c_word_write(client, BSP_ENTER_ROM_MODE_CMD, BSP_ENTER_ROM_MODE_DATA);
    if (0 != iRet)
    {
        power_debug(HW_POWER_BATTERY_IC_ERR, "[Power ERR: %s, %s, %d]  i2c write failed !\n", __FILE__, __func__,
                    __LINE__);
    }

    mdelay(10);

    /*change i2c addr*/
    g_bq275x0_i2c_client->addr = BQ275x0_FIRMWARE_DOWNLOAD_MODE_I2C_ADDR;

    /*program bqfs*/
    iRet = bq275x0_firmware_program(client, pgm_data, len);
    if (0 != iRet)
    {
        power_debug(HW_POWER_BATTERY_IC_ERR, "[Power ERR: %s, %s, %d]  firmware download failed!\n", __FILE__, __func__,
                    __LINE__);
    }

    /*change i2c addr*/
    g_bq275x0_i2c_client->addr = BQ275X0_I2C_ADDR;
    locked_timeout_jiffies = jiffies + msecs_to_jiffies(5000);
    bq275x0_work_mode = BQ275x0_LOCK_MODE;

    return iRet;
}

static int bq275x0_update_firmware(struct i2c_client *client, const char *pFilePath)
{
    char *buf;
    struct file *filp;
    struct inode *inode = NULL;
    mm_segment_t oldfs;
    unsigned int length;
    int ret = 0;

    /* open file */
    oldfs = get_fs();
    set_fs(KERNEL_DS);
    filp = filp_open(pFilePath, O_RDONLY, S_IRUSR);
    if (IS_ERR(filp))
    {
        power_debug(HW_POWER_BATTERY_IC_ERR, "[Power ERR: %s, %s, %d]  \n", __FILE__, __func__, __LINE__);
        set_fs(oldfs);
        return -1;
    }

    if (!filp->f_op)
    {
        power_debug(HW_POWER_BATTERY_IC_ERR, "[Power ERR: %s, %s, %d]  \n", __FILE__, __func__, __LINE__);
        filp_close(filp, NULL);
        set_fs(oldfs);
        return -1;
    }

    inode = filp->f_path.dentry->d_inode;
    if (!inode)
    {
        power_debug(HW_POWER_BATTERY_IC_ERR, "[Power ERR: %s, %s, %d]  \n", __FILE__, __func__, __LINE__);
        filp_close(filp, NULL);
        set_fs(oldfs);
        return -1;
    }

    /* file's size */
    length = i_size_read(inode->i_mapping->host);
    power_debug(HW_POWER_BATTERY_IC_DUG, "[Power DUG: %s, %s, %d]  length = %d \n", __FILE__, __func__, __LINE__,
                length);
    if (!((length > 0) && (length < BSP_FIRMWARE_FILE_SIZE)))
    {
        power_debug(HW_POWER_BATTERY_IC_ERR, "[Power ERR: %s, %s, %d]  \n", __FILE__, __func__, __LINE__);
        filp_close(filp, NULL);
        set_fs(oldfs);
        return -1;
    }

    /* allocation buff size */
    /* buf size if even */
    buf = vmalloc(length + (length % 2));
    if (!buf)
    {
        power_debug(HW_POWER_BATTERY_IC_ERR, "[Power ERR: %s, %s, %d]  \n", __FILE__, __func__, __LINE__);
        filp_close(filp, NULL);
        set_fs(oldfs);
        return -1;
    }

    /* read data */
    if (filp->f_op->read(filp, buf, length, &filp->f_pos) != length)
    {
        power_debug(HW_POWER_BATTERY_IC_ERR, "[Power ERR: %s, %s, %d]  \n", __FILE__, __func__, __LINE__);
        filp_close(filp, NULL);
        filp_close(filp, NULL);
        set_fs(oldfs);
        vfree(buf);
        return -1;
    }

    ret = bq275x0_firmware_data_download(client, (const char *)buf, length);

    filp_close(filp, NULL);
    set_fs(oldfs);
    vfree(buf);

    return ret;
}

#endif

int report_get_property(struct power_supply *       psy,
                        enum power_supply_property  psp,
                        union power_supply_propval *val)
{
    struct S10_std_battery_device * battery = NULL;
    int ret = 0;

    battery = S10_power_get_battery_in_list(psy->name);
    if (IS_ERR_OR_NULL(battery))
    {
        power_debug(HW_POWER_BATTERY_IC_ERR, "[Power ERR: %s, %s, %d]  \n", __FILE__, __func__, __LINE__);
        ret = -EINVAL;
        goto err_report;
    }

    switch (psp)
    {
    case POWER_SUPPLY_PROP_STATUS:
        power_debug(HW_POWER_BATTERY_IC_DUG,
                    "[Power ERR: %s, %s, %d, POWER_SUPPLY_PROP_STATUS real val = %d, report val = %d]  \n", __FILE__,
                    __func__,
                    __LINE__, battery->battery_detect_info.battery_status, battery->battery_report_info.battery_status);

        val->intval = battery->battery_report_info.battery_status;
        break;
    case POWER_SUPPLY_PROP_TECHNOLOGY:
        val->intval = battery->battery_report_info.battery_technology;
        break;
    case POWER_SUPPLY_PROP_PRESENT:
        val->intval = battery->battery_report_info.battery_present;
        break;
    case POWER_SUPPLY_PROP_HEALTH:
        val->intval = battery->battery_report_info.battery_health;
        break;
    case POWER_SUPPLY_PROP_VOLTAGE_NOW:
        val->intval = battery->battery_report_info.battery_voltage;
        break;
    case POWER_SUPPLY_PROP_CURRENT_NOW:
        val->intval = battery->battery_report_info.battery_current;
        break;
    case POWER_SUPPLY_PROP_CAPACITY:
        val->intval = battery->battery_report_info.battery_capacity;
        break;
    case POWER_SUPPLY_PROP_TEMP:
        val->intval = battery->battery_report_info.battery_temperature;
        break;
    case POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN:
        val->intval = battery->battery_report_info.battery_voltage_max_design;
        break;
    case POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN:
        val->intval = battery->battery_report_info.battery_voltage_min_design;
        break;
    case POWER_SUPPLY_PROP_ENERGY_FULL_DESIGN:
        val->intval = battery->battery_report_info.battery_capacity_full_design;
        break;
    case POWER_SUPPLY_PROP_ENERGY_EMPTY_DESIGN:
        val->intval = battery->battery_report_info.battery_capacity_empty_design;
        break;
    case POWER_SUPPLY_PROP_CYCLE_COUNT:
        val->intval = battery->battery_report_info.battery_cycle_count;
        break;
    case POWER_SUPPLY_PROP_TIME_TO_FULL_NOW:
        val->intval = battery->battery_report_info.time2full_now;
        break;
    case POWER_SUPPLY_PROP_TIME_TO_FULL_AVG:
        val->intval = battery->battery_report_info.time2full_avg;
        break;
    case POWER_SUPPLY_PROP_TIME_TO_EMPTY_NOW:
        val->intval = battery->battery_report_info.time2empty_now;
        break;
    case POWER_SUPPLY_PROP_TIME_TO_EMPTY_AVG:
        val->intval = battery->battery_report_info.time2empty_avg;
        break;

    case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN: /* 设计的电池满充电量，单位: uAh */
        val->intval = battery->battery_report_info.battery_capacity_full_design;
        break;
    case POWER_SUPPLY_PROP_CHARGE_FULL:/* 库仑计学习的电池满电量，单位uAh */
        val->intval = battery->battery_report_info.battery_capacity_full_charge;
        break;
    default:
        ret = -EINVAL;
        goto err_report;
    }

    power_debug(HW_POWER_BATTERY_IC_DUG, "[Power ERR: %s, %s, %d]  psp = %d, val = %d, ret = %d\n", __FILE__, __func__,
                __LINE__, psp, val->intval, ret);
    return 0;

err_report:
    switch (psp)
    {
    case POWER_SUPPLY_PROP_STATUS:
        val->intval = POWER_SUPPLY_STATUS_UNKNOWN;
        break;
    case POWER_SUPPLY_PROP_HEALTH:
        val->intval = POWER_SUPPLY_HEALTH_UNKNOWN;
        break;
    case POWER_SUPPLY_PROP_TECHNOLOGY:
        val->intval = POWER_SUPPLY_TECHNOLOGY_UNKNOWN;
        break;
    case POWER_SUPPLY_PROP_CAPACITY_LEVEL:
        val->intval = POWER_SUPPLY_CAPACITY_LEVEL_UNKNOWN;
        break;
    default:
        val->intval = -1;
        break;
    }

    return ret;
}

int bq275x0_get_property(char*                       name,
                         enum power_supply_property  psp,
                         union power_supply_propval *val)
{
    int ret  = 0;
    int data = 0;
    struct i2c_client* client = bq275x0_get_i2c_client();

    struct S10_std_battery_device * battery = NULL;

    battery = S10_power_get_battery_in_list(name);

    if (IS_ERR_OR_NULL(battery) || IS_ERR_OR_NULL(client))
    {
        power_debug(HW_POWER_BATTERY_IC_ERR, "[Power ERR: %s, %s, %d]  \n", __FILE__, __func__, __LINE__);
        ret = -EINVAL;
        goto err_report;
    }

    switch (psp)
    {
    case POWER_SUPPLY_PROP_STATUS:
    {
        struct S10_psy_reporter* reporter = S10_power_get_monitor_reporter();
        struct S10_std_charger_device* charger;
        struct S10_psy_monitor_dev_info* monitordev_info = S10_power_get_monitor_devinfo();
        int present = 0, capacity = 0;

        power_debug(HW_POWER_BATTERY_IC_DUG, "[Power ERR: %s, %s, %d, POWER_SUPPLY_PROP_STATUS ]  \n", __FILE__,
                    __func__, __LINE__);
        if (IS_ERR_OR_NULL(monitordev_info) || IS_ERR_OR_NULL(reporter))
        {
            val->intval = 0;
            break;
        }

        power_debug(HW_POWER_BATTERY_IC_DUG, "[Power ERR: %s, %s, %d, POWER_SUPPLY_PROP_STATUS ]  \n", __FILE__,
                    __func__, __LINE__);
        charger = reporter->stdcharger;
        if (IS_ERR_OR_NULL(charger))
        {
            val->intval = 0;
            break;
        }

        power_debug(HW_POWER_BATTERY_IC_DUG, "[Power ERR: %s, %s, %d, POWER_SUPPLY_PROP_STATUS ]  \n",
                    __FILE__, __func__, __LINE__);
        bq275x0_battery_present(client, &present);
        bq275x0_battery_capacity(client, &capacity);

        if (!present)
        {
            val->intval = POWER_SUPPLY_STATUS_UNKNOWN;
        }
        else if (capacity < 100)
        {
            if (charger->charger_real_info.charger_online)
            {
                if ((((POWER_SUPPLY_TYPE_USB == charger->charger_real_info.charger_type)
                      || (POWER_SUPPLY_TYPE_USB_CDP == charger->charger_real_info.charger_type))
                     && monitordev_info->usb_charging_display_support && monitordev_info->usb_charging_support)
                    || (POWER_SUPPLY_TYPE_USB_ACA == charger->charger_real_info.charger_type)
                    || (POWER_SUPPLY_TYPE_USB_DCP == charger->charger_real_info.charger_type))
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
        }
        else
        {
            val->intval = POWER_SUPPLY_STATUS_FULL;
        }

        power_debug(HW_POWER_BATTERY_IC_DUG, "[Power ERR: %s, %s, %d, POWER_SUPPLY_PROP_STATUS val = %d]  \n", __FILE__,
                    __func__, __LINE__, val->intval);
    }
        break;
    case POWER_SUPPLY_PROP_TECHNOLOGY:
        val->intval = battery->battery_detect_info.battery_technology;
        break;
    case POWER_SUPPLY_PROP_PRESENT:
        ret = bq275x0_battery_present(client, &data);
        val->intval = data;
        break;
    case POWER_SUPPLY_PROP_HEALTH:
        ret = bq275x0_battery_health(client, &data);
        val->intval = data;
        break;
    case POWER_SUPPLY_PROP_VOLTAGE_NOW:
        ret = bq275x0_battery_voltage(client, &data);
        if (ret >= 0)
        {
            val->intval = data;
            g_bq27510_reg_value[REG_VOL] = data;
        }

        break;
    case POWER_SUPPLY_PROP_CURRENT_NOW:
        ret = bq275x0_battery_current(client, &data);
        val->intval = data;
        g_bq27510_reg_value[REG_CUR] = data;
        break;
    case POWER_SUPPLY_PROP_CAPACITY:
        ret = bq275x0_battery_capacity(client, &data);
        if (ret >= 0)
        {
            val->intval = data;
            g_bq27510_reg_value[REG_CAP] = data;
        }
        break;
    case POWER_SUPPLY_PROP_TEMP:
        ret = bq275x0_battery_temperature(client, &data);
        if (ret >= 0)
        {
            val->intval = data;
            g_bq27510_reg_value[REG_TMP] = data;
        }
        break;
    case POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN:
        val->intval = battery->battery_detect_info.battery_voltage_max_design;
        break;
    case POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN:
        val->intval = battery->battery_detect_info.battery_voltage_min_design;
        break;
    case POWER_SUPPLY_PROP_ENERGY_FULL_DESIGN:
        val->intval = battery->battery_detect_info.battery_capacity_full_design;
        break;
    case POWER_SUPPLY_PROP_ENERGY_EMPTY_DESIGN:
        val->intval = battery->battery_report_info.battery_capacity_empty_design;
        break;
    case POWER_SUPPLY_PROP_CYCLE_COUNT:
        break;
    case POWER_SUPPLY_PROP_TIME_TO_FULL_NOW:
        ret = bq275x0_battery_ttf(client, &data);
        if (ret >= 0)
        {
            val->intval = data;
        }

        break;
    case POWER_SUPPLY_PROP_TIME_TO_FULL_AVG:
        break;
    case POWER_SUPPLY_PROP_TIME_TO_EMPTY_NOW:
        ret = bq275x0_battery_tte(client, &data);
        if (ret >= 0)
        {
            val->intval = data;
        }

        break;
    case POWER_SUPPLY_PROP_TIME_TO_EMPTY_AVG:
        break;

    case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN: /* 设计的电池满充电量，单位: uAh */
        val->intval = battery->battery_detect_info.battery_capacity_full_design;
        break;
    case POWER_SUPPLY_PROP_CHARGE_FULL:/* 库仑计学习的电池满电量，单位uAh */
        ret = bq275x0_full_charge_capacity(client, &data);
        if (ret >= 0)
        {
            val->intval = data;
            g_bq27510_reg_value[REG_FCC] = data;
        }
        break;
    default:
        ret = -EINVAL;
        goto err_report;
    }

    return 0;

err_report:
    switch (psp)
    {
    case POWER_SUPPLY_PROP_STATUS:
        val->intval = POWER_SUPPLY_STATUS_UNKNOWN;
        break;
    case POWER_SUPPLY_PROP_HEALTH:
        val->intval = POWER_SUPPLY_HEALTH_UNKNOWN;
        break;
    case POWER_SUPPLY_PROP_TECHNOLOGY:
        val->intval = POWER_SUPPLY_TECHNOLOGY_UNKNOWN;
        break;
    case POWER_SUPPLY_PROP_CAPACITY_LEVEL:
        val->intval = POWER_SUPPLY_CAPACITY_LEVEL_UNKNOWN;
        break;
    default:
        val->intval = -1;
        break;
    }

    return ret;
}

int bq275x0_type_get(void* dev, int* type)
{
    *type = POWER_SUPPLY_TECHNOLOGY_LION;
    return 0;
}

#ifdef CONFIG_BQ275X0_FIRMWARE_UPDATE
int bq275x0_firmware_download(unsigned char* firmware_path)
{
    struct i2c_client* client = bq275x0_get_i2c_client();

    return bq275x0_update_firmware(client, firmware_path);

    //client = g_bq275x0_i2c_client;
    i2c_smbus_write_word_data(g_bq275x0_i2c_client, 0x00, 0x0041);
}

int bq275x0_get_firmware_version(void)
{
    int ret = 0, version = 0;
    struct i2c_client *client = g_bq275x0_i2c_client;
    int firmware_id = 0;
    int child_ver = 0;
 #ifdef CONFIG_BQ275X0_FIRMWARE_UPDATE
    /* added for Firmware upgrade begin*/
    i2c_smbus_write_word_data(client, 0x00, 0x0008);

    mdelay(2);
    /* 读固件大版本号 */
    firmware_id = i2c_smbus_read_word_data(client, 0x00);
    if (firmware_id < 0)
    {
        power_debug(HW_POWER_BATTERY_IC_ERR, "[Power ERR: %s, %s, %d] Coulometer Damaged or Firmware Error! \n",
                    __FILE__, __func__, __LINE__);
    }
    else
    {
        power_debug(HW_POWER_BATTERY_IC_DUG, "[Power DUG: %s, %s, %d] Firmware version=%04x! \n", __FILE__, __func__,
                    __LINE__, firmware_id);
    }

    /* 读固件小版本号 */
    mdelay(2);
    ret = i2c_smbus_write_word_data(client, BQ275x0_REG_DFCLS, BQ275x0_REG_FIRMWARE_VERSION);
    if (ret < 0)
    {
        power_debug(HW_POWER_BATTERY_IC_ERR, "[Power DUG: %s, %s, %d] Firmware child version=%04x! \n", __FILE__,
                    __func__,
                    __LINE__, ret);
    }

    mdelay(2);
    child_ver = i2c_smbus_read_byte_data(client, BQ275x0_REG_FLASH);
    if (child_ver < 0)
    {
        power_debug(HW_POWER_BATTERY_IC_ERR, "[Power DUG: %s, %s, %d] Firmware child version=%04x! \n", __FILE__,
                    __func__,
                    __LINE__, child_ver);
    }
    else
    {
        power_debug(HW_POWER_BATTERY_IC_DUG, "[Power DUG: %s, %s, %d] Firmware version=%04x! \n", __FILE__, __func__,
                    __LINE__, child_ver);
    }

    version = (firmware_id << 16) | child_ver;
    /*added for Firmware upgrade end*/
 #endif
    return version;
}

#endif

/*****************************************************************************
 函 数 名  : show_control_status
 功能描述  : 读取control_status寄存器数值
 输入参数  : void
 输出参数  : 无
 返 回 值  : 无
 调用函数  : 略
 被调函数  : 略

 修改历史      :
  1.日    期   : 2012年11月23日
    作    者   : l00220156
    修改内容   : 新生成函数

*****************************************************************************/
static int show_control_status(void)
{
    int data = 0;
    int ret = 0;

    ret = i2c_smbus_write_word_data(g_bq275x0_i2c_client, 0x00, BQ275x0_REG_CTRS);
    if (ret < 0)
    {
        power_debug(HW_POWER_BATTERY_IC_ERR, "[Power DUG: %s, %d, %d] \n", __func__, __LINE__, ret);
        return -1;
    }

    data = i2c_smbus_read_word_data(g_bq275x0_i2c_client, 0x00);
    if (data < 0)
    {
        power_debug(HW_POWER_BATTERY_IC_ERR, "control_status register can't be accessed...\n");
        return -1;
    }

    return data;
}

/*****************************************************************************
 函 数 名  : bq275x0_battery_flag
 功能描述  :  返回gas gauge status register数值
 输入参数  : void
 输出参数  : 无
 返 回 值  : 正确返回寄存器数值，错误时返回-1
 调用函数  : 略
 被调函数  : 略
 
 修改历史      :
  1.日    期   : 2012年11月23日
    作    者   : l00220156
    修改内容   : 新生成函数

*****************************************************************************/
static int bq275x0_battery_flag(void)
{
    int data = 0;

    data = i2c_smbus_read_word_data(g_bq275x0_i2c_client, BQ275x0_REG_FLAGS);
    if (data < 0)
    {
        power_debug(HW_POWER_BATTERY_IC_ERR, "status register can't be accessed...\n");
        return -1;
    }

    return data;
}

/*****************************************************************************
 函 数 名  : static void show_bq275x0_battery_info(void)
 功能描述  : 打印输出充电电池状态信息
 输入参数  : void
 输出参数  : 无
 返 回 值  : 无
 调用函数  : 略
 被调函数  : coulometer_bq27510_reg_dump

 修改历史      :
  1.日    期   : 2012年11月23日
    作    者   : l00220156
    修改内容   : 新生成函数

*****************************************************************************/
static void show_bq275x0_battery_info(void)
{
    int status_reg = 0;
    int flag_reg = 0;

    status_reg = show_control_status();
    if (status_reg < 0)
    {
        power_debug(HW_POWER_BATTERY_IC_ERR, "[Power DUG: %s, %d, %d] \n", __func__, __LINE__, status_reg);
        return;
    }

    flag_reg = bq275x0_battery_flag();
    if (flag_reg < 0)
    {
        power_debug(HW_POWER_BATTERY_IC_ERR, "[Power DUG: %s, %d, %d] \n", __func__, __LINE__, flag_reg);
        return;
    }

    power_debug(HW_POWER_BATTERY_IC_ERR,
                "BQ275X0_LOGS: CONTROL_STATUS:0x%x;Flags:0x%x;vol:%d;cur:%d;cap:%d;tmp:%d;fcc:%d\n",
                status_reg,
                flag_reg,
                g_bq27510_reg_value[REG_VOL],
                g_bq27510_reg_value[REG_CUR],
                g_bq27510_reg_value[REG_CAP],
                g_bq27510_reg_value[REG_TMP],
                g_bq27510_reg_value[REG_FCC]
    );
}

/*****************************************************************************
 函 数 名  : get_bq275x0_block_value
 功能描述  : 获取库仑计82、91-94数据块内容
 输入参数  : 数据块ID
 输出参数  : 成功与否
 返 回 值  : 1、0
 调用函数  : 略
 被调函数  : coulometer_bq27510_reg_dump

 修改历史      :
  1.日    期   : 2012年11月23日
    作    者   : l00220156
    修改内容   : 新生成函数

*****************************************************************************/
static int get_bq275x0_block_value(char block_id)
{
    unsigned char data[32];
    int i;
    int ilen = 19;
    struct i2c_client* client = bq275x0_get_i2c_client();

    data[0] = block_id;

    // 判断电量计是否处于工作模式
    if (false == bq275x0_check_normal_mode())
    {
        power_debug(HW_POWER_BATTERY_IC_ERR, "[Power DUG: %s, %d] MODE erro!! \n", __func__, __LINE__);
        return -1;
    }

    printk("Block[%d]: ", data[0]);

    if (bq275x0_i2c_bytes_write(client, BQ275x0_REG_DFCLS, data, 1))
    {
        power_debug(HW_POWER_BATTERY_IC_ERR, "[Power DUG: %s, %d] write erro!! \n", __func__, __LINE__);
        return -1;
    }

    mdelay(2);

    data[0] = 0x00;
    if (bq275x0_i2c_bytes_write(client, BQ275x0_REG_DFBLK, data, 1))
    {
        power_debug(HW_POWER_BATTERY_IC_ERR, "[Power DUG: %s, %d] write erro!! \n", __func__, __LINE__);
        return -1;
    }

    mdelay(2);

    data[0] = 0x00;
    if (bq275x0_i2c_bytes_write(client, BQ275x0_REG_DFDCNTL, data, 1))
    {
        power_debug(HW_POWER_BATTERY_IC_ERR, "[Power DUG: %s, %d] write erro!! \n", __func__, __LINE__);
        return -1;
    }

    mdelay(2);

    bq275x0_i2c_bytes_read(client, 0x60, data, 1);
    mdelay(2);

    bq275x0_i2c_bytes_read(client, BQ275x0_REG_FLASH, data, 32);

    for (i = 0; i < ilen; i++)
    {
        printk("0x%02x ", data[i]);
    }

    printk("\n");

    return 0;
}

/*****************************************************************************
 函 数 名  : coulometer_bq27510_reg_dump
 功能描述  :  打印库仑计状态数据
 输入参数  : void
 输出参数  : 无
 返 回 值  : 无
 调用函数  : 略
 被调函数  : 略
 
 修改历史      :
  1.日    期   : 2012年11月23日
    作    者   : l00220156
    修改内容   : 新生成函数

*****************************************************************************/
static void coulometer_bq27510_reg_dump(void)
{
    int iret = 0;

    show_bq275x0_battery_info();
    iret = get_bq275x0_block_value(82); // BQ27510 DATA FLASH ID 82
    if (iret < 0)
    {
        power_debug(HW_POWER_BATTERY_IC_ERR, "[Power DUG: %s, %d, %d] \n", __func__, __LINE__, iret);
        return;
    }

    iret = get_bq275x0_block_value(91); // read BQ27510 DATA FLASH ID 91
    if (iret < 0)
    {
        power_debug(HW_POWER_BATTERY_IC_ERR, "[Power DUG: %s, %d, %d] \n", __func__, __LINE__, iret);
        return;
    }
}

static int bq275x0_battery_aging_detect(char* name)
{
    int ret = 0;
    int cyclecount;
    int c1,c2;
    struct i2c_client* client = bq275x0_get_i2c_client();
    struct S10_std_battery_device * battery = NULL;

    battery = S10_power_get_battery_in_list(name);

    if (IS_ERR_OR_NULL(battery) || IS_ERR_OR_NULL(client))
    {
        power_debug(HW_POWER_BATTERY_IC_ERR, "[Power ERR: %s, %s, %d]  \n", __FILE__, __func__, __LINE__);
        ret = -EINVAL;
        return ret;
    }

    c1 = battery->battery_detect_info.battery_cycle1;
    c2 = battery->battery_detect_info.battery_cycle2;
    if(c1 <= 0 || c2 <= 0){
        ret = -EINVAL;
        return ret;
    }
    
    ret = bq275x0_battery_cyclecount(client, &cyclecount);
    if(ret < 0){
        power_debug(HW_POWER_BATTERY_IC_ERR, "[Power ERR: %s, %s, %d]  \n", __FILE__, __func__, __LINE__);
        return ret;
    }

    power_debug(HW_POWER_BATTERY_IC_DUG, "%s:cyclecount=%d,c1=%d,c2=%d\n",__func__, cyclecount,c1,c2);

    //80% or more of its original capacity
    if(cyclecount <= c1){
        ret = LEVEL1;
    }
    //At least 50% but less than 80％ of its original capacity
    else if(cyclecount > c1 && cyclecount <= c2){
              
        ret = LEVEL2;
    }
    //Below 50% of its original capacity
    else {
        ret = LEVEL3;
    }
    
    return ret;
}

struct S10_std_battery_private_ops bq275x0_private_ops =
{
    .battery_type_get             = bq275x0_type_get,
    .battery_reg_dump             = coulometer_bq27510_reg_dump,
#ifdef CONFIG_BQ275X0_FIRMWARE_UPDATE
    .battery_firmware_download    = bq275x0_firmware_download,
    .battery_get_firmware_version = bq275x0_get_firmware_version,
#endif
    .battery_aging_detect         = bq275x0_battery_aging_detect, 
};
static int bq275x0_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    struct bq27510_platform_data *pdata = NULL;
    struct S10_std_battery_device* battery = NULL;
    int ret = 0;

    if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
    {
        power_debug(HW_POWER_BATTERY_IC_ERR, "[Power ERR: %s, %s, %d]  \n", __FILE__, __func__, __LINE__);
        ret = -EIO;
        goto err_check_i2c_func;
    }

    pdata = client->dev.platform_data;
    if (!pdata)
    {
        dev_err(&client->dev, "pdata is NULL!\n");
        return -EINVAL;
    }

    power_debug(HW_POWER_BATTERY_IC_DUG, "[Power DUG: %s, %s, %d] ! \n", __FILE__, __func__, __LINE__);

#if 0
    // check bq275x0 exist
    value = i2c_smbus_read_word_data(client, BQ275x0_REG_VOLT);
    if (value < 0)
    {
        power_debug(HW_POWER_BATTERY_IC_ERR, "[Power ERR: %s, %s, %d] bq27510 is not on this i2c bus! \n", __FILE__,
                    __func__, __LINE__);
        ret = -EIO;
        goto err_check_chip_exist;
    }
#endif


    //register bq275x0 i2c client to battery report driver
    battery = kzalloc(sizeof(struct S10_std_battery_device), GFP_KERNEL);
    if (!battery)
    {
        ret = -ENOMEM;
        goto err_alloc_battery_failed;
    }

    battery_bq275x0 = battery;
#ifdef CONFIG_BQ275X0_FIRMWARE_UPDATE
    /* added for Firmware upgrade begin//BQ275X0_FIRMWARE_UPDATE default is Y*/
    i2c_smbus_write_word_data(client, 0x00, 0x0008);

    mdelay(2);

    ret = i2c_smbus_read_word_data(client, 0x00);
    if (ret < 0)
    {
        power_debug(HW_POWER_BATTERY_IC_ERR, "[Power ERR: %s, %s, %d] Coulometer Damaged or Firmware Error! \n",
                    __FILE__, __func__, __LINE__);
    }
    else
    {
        power_debug(HW_POWER_BATTERY_IC_DUG, "[Power DUG: %s, %s, %d] Firmware version=%04x! \n", __FILE__, __func__,
                    __LINE__, ret);
    }

    /*added for Firmware upgrade end*/
#endif

    S10_power_lock_init(S10_POWER_LOCK_LIGHT, &battery->data_lock);
    g_bq275x0_i2c_client = client;
    strcpy(battery->battery_name, MAIN_BATTERY);
    strcpy(battery->firmware_name, "bq27510");
    battery->dev = &client->dev;
    battery->battery_psy_visible = 1;
    battery->battery_repeat_charging_flag = 1;
    battery->psy_type = POWER_SUPPLY_TYPE_BATTERY;
    battery->temperature_status = TEMP_NOMAL;
    battery->S10_battery_support_props = bq27510_support_props;
    battery->num_properties = ARRAY_SIZE(bq27510_support_props);
    battery->battery_report_get_property = report_get_property;
    battery->battery_detect_get_property = bq275x0_get_property;
    battery->ops = &bq275x0_private_ops;
    battery->battery_detect_info.battery_technology = pdata->battery_technology;
    battery->battery_detect_info.battery_voltage_max_design    = pdata->voltage_max_design;
    battery->battery_detect_info.battery_voltage_min_design    = pdata->voltage_min_design;
    battery->battery_detect_info.battery_capacity_full_design  = pdata->energy_full_design;
    battery->battery_detect_info.battery_capacity_empty_design = pdata->energy_empty_design;

    if(get_product_feature(PROD_FEATURE_DETECT_DCM_HARDWARE)){
        battery->battery_detect_info.battery_cycle1 = DCM_BATTERY_CYCLE1;
        battery->battery_detect_info.battery_cycle2 = DCM_BATTERY_CYCLE2;
    }else{
        battery->battery_detect_info.battery_cycle1 = NORMAL_BATTERY_CYCLE1;
        battery->battery_detect_info.battery_cycle2 = NORMAL_BATTERY_CYCLE2;
    }
 
    power_debug(HW_POWER_BATTERY_IC_DUG, "[Power DUG: %s, %s, %d] before battery2monitor_register! \n", __FILE__,
                __func__, __LINE__);
    ret = battery2monitor_register(battery);
    if (ret)
    {
        goto err_register2monitor;
    }

    power_debug(HW_POWER_BATTERY_IC_DUG, "[Power DUG: %s, %s, %d] probe OK! \n", __FILE__, __func__, __LINE__);
    return 0;

err_register2monitor:
    S10_power_lock_deinit(&battery->data_lock);
    g_bq275x0_i2c_client = NULL;
    if (battery)
    {
        kfree(battery);
        battery_bq275x0 = NULL;
    }

err_alloc_battery_failed:
err_check_i2c_func:
    return ret;
}

static int bq275x0_remove(struct i2c_client *client)
{
    if (battery_bq275x0)
    {
        S10_power_lock_deinit(&battery_bq275x0->data_lock);
        kfree(battery_bq275x0);
        battery_bq275x0 = NULL;
    }

    //资源释放
    return 0;
}

#ifdef CONFIG_PM
static int bq275x0_suspend(struct i2c_client *client,
                           pm_message_t       state)
{
    printk("[%s] +\n", __func__);
    printk("[%s] -\n", __func__);

    return 0;
}

static int bq275x0_resume(struct i2c_client *client)
{
    printk("[%s] +\n", __func__);
    printk("[%s] -\n", __func__);

    return 0;
}

#else
 #define bq275x0_suspend NULL
 #define bq275x0_resume NULL
#endif

static const struct i2c_device_id bq275x0_id[] =
{
    {BQ275X0_NAME, 0 },
    { }
};

static struct i2c_driver bq275x0_driver =
{
    .probe    = bq275x0_probe,
    .remove   = bq275x0_remove,
    .suspend  = bq275x0_suspend,
    .resume   = bq275x0_resume,
    .id_table = bq275x0_id,
    .driver   = {
        .name = BQ275X0_NAME,
    },
};

static int __devinit bq275x0_init(void)
{
    return i2c_add_driver(&bq275x0_driver);
}

static void __exit bq275x0_exit (void)
{
    i2c_del_driver(&bq275x0_driver);
}

fs_initcall(bq275x0_init);
module_exit(bq275x0_exit);

MODULE_AUTHOR("Huawei");
MODULE_DESCRIPTION("I2C bq275x0 Driver");
MODULE_LICENSE("GPL");
