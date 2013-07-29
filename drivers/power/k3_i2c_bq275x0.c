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
 * bq275x0 battery guage driver
==============================================================================*/
#include <linux/device.h>
#include <linux/mux.h>
#include <linux/module.h>
#include <linux/param.h>
#include <linux/jiffies.h>
#include <linux/workqueue.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/idr.h>
#include <linux/i2c.h>
#include <asm/unaligned.h>
#include <linux/interrupt.h>
#include <asm/irq.h>
#include <linux/module.h>
#include <linux/io.h>
#include <linux/uaccess.h>
#include <linux/vmalloc.h>
#include <linux/fs.h>
#include <linux/fcntl.h>
#include <linux/ctype.h>
#include <linux/slab.h>
#include <mach/gpio.h>
#include <linux/mutex.h>



#include <linux/power_supply.h>


#include <linux/i2c/k3_i2c_bq275x0.h>
#include <linux/power/k3_psy_monitor.h>

#if 0
#ifdef CONFIG_BQ275X0_FIRMWARE_UPDATE
#include <linux/i2c/k3_bq275x0_firmware.h>
#endif
#endif

#define BQ275x0_REG_TEMP	0x06
#define BQ275x0_REG_VOLT	0x08
#define BQ275x0_REG_FLAGS   0x0a
#define BQ275x0_REG_TTE		0x16		/*Time to Empty*/
#define BQ275x0_REG_TTF		0x18		/*Time to Full*/
#define BQ275x0_REG_SOC		0x2C		/* State-of-Charge */
#define BQ275x0_REG_AI		0x14		/*Average Current*/

#define BQ275x0_REG_RCAP    0x10
#define BQ275x0_REG_FCC     0x12

#define BQ275x0_FLAG_FC		1<<9		/* Full-charged bit */
#define BQ275x0_FLAG_DET	1<<3

/* Over-Temperature-Charge bit */
#define BQ275x0_FLAG_OTC			            (1<<15)
/* Over-Temperature-Discharge bit */
#define BQ275x0_FLAG_OTD    		            (1<<14)

#define CONST_NUM_TEMP		                     2730
#if 0
#define BQ275x0_FIRMWARE_DOWNLOAD_MODE          0xDDDDDDDD
#define BQ275x0_NORMAL_MODE                     0x0
#define BQ275x0_FIRMWARE_DOWNLOAD_MODE_I2C_ADDR               0x0B
#define BQ275x0_NORMAL_MODE_I2C_ADDR            0x55
#endif
#ifdef CONFIG_BQ275X0_FIRMWARE_UPDATE
/* added for Firmware upgrade begin */
static DEFINE_MUTEX(k3_bq275x0_battery_mutex);
/* added for Firmware upgrade end */
#endif

static int bq275x0_work_mode = BQ275x0_NORMAL_MODE;
static struct i2c_client * g_bq275x0_i2c_client = NULL;
static struct k3_battery_gauge_dev* battery_bq275x0 = NULL;

static struct i2c_driver bq275x0_driver;

#ifdef BARCELONA_SHOW
static const capacity_map capacitymap_for_Barcelona[] =
{
/* pro battery volt -> capacity data */
    {3300, 0},
    {3400, 1},
    {3500, 1},
    {3600, 3},
    {3700, 16},
    {3800, 49},
    {3900, 68},
    {4000, 86},
    {4100, 99},
    {4200, 100}
};

static const capacity_map capacitymap_SDP_for_Barcelona[] =
{
/* pro battery volt -> capacity data */
    {3450, 0},
    {3550, 1},
    {3650, 1},
    {3750, 3},
    {3850, 16},
    {3900, 49},
    {4010, 68},
    {4100, 86},
    {4150, 99},
    {4200, 100}
};

static const capacity_map capacitymap_DCP_for_Barcelona[] =
{
/* pro battery volt -> capacity data */
    {3500, 0},
    {3600, 1},
    {3700, 1},
    {3800, 3},
    {3900, 16},
    {4000, 49},
    {4050, 68},
    {4100, 86},
    {4190, 99},
    {4200, 100}
};



capacity_result_status battery_capacity_voltage_map(
                        const capacity_map *mappts,
                        unsigned int    mapsize,
                        int             input,
                        int             *output
)
{
    unsigned int idx = 0;

    if((NULL == mappts) || (NULL == output)){
        return CAPACITY_RESULT_INVALID;
    }

    while(idx < mapsize){
        if(mappts[idx].x > input){
            break;
        }
        else{
            idx++;
        }
    }

    if(idx == 0){
        *output = mappts[0].y;
    }
    else if (idx == mapsize){
        *output = mappts[mapsize-1].y;
    }
    else{
        *output = (
                  ( (int)
                   (
                    (mappts[idx].y - mappts[idx-1].y)
                     * (input - mappts[idx-1].x)
                   )
                   / (mappts[idx].x - mappts[idx-1].x)
               )
               + mappts[idx-1].y
             );
    }
    return CAPACITY_RESULT_VALID;
}

#endif /* BARCELONA_SHOW */

struct i2c_client* bq275x0_get_i2c_client(void)
{
	return g_bq275x0_i2c_client;
}

static bool bq275x0_check_normal_mode(void)
{
	return (bq275x0_work_mode == BQ275x0_NORMAL_MODE);
}

void bq275x0_set_normal_mode(void)
{
	struct i2c_client* client = bq275x0_get_i2c_client();
	bq275x0_work_mode = BQ275x0_NORMAL_MODE;
	if(client)
		client->addr = BQ275x0_FIRMWARE_DOWNLOAD_MODE_I2C_ADDR;
}

#if 0
void bq275x0_set_firmware_download_mode(void)
{
	struct i2c_client* client = bq275x0_get_i2c_client();
	bq275x0_work_mode = BQ275x0_NORMAL_MODE;
	if(client)
		client->addr = BQ275x0_FIRMWARE_DOWNLOAD_MODE_I2C_ADDR;
}
#endif

static int bq275x0_i2c_read_word(struct i2c_client *client,u8 reg)
{
	int err = 0;
	if(bq275x0_check_normal_mode()){
        BQ275x0_DBG("k3_i2c_bq275x0:bq275x0_i2c_read_word:mode current!!\n");
        err = i2c_smbus_read_word_data(client,reg);
       }
	else
		err = -1;

    BQ275x0_DBG("k3_i2c_bq275x0:bq275x0_i2c_read_word:err:%d\n", err);
	return err;
}

#ifdef CONFIG_BQ275X0_FIRMWARE_UPDATE
/* added for Firmware upgrade begin */
static int k3_bq275x0_i2c_word_write(struct i2c_client *client, u8 reg, u16 value)
{
	int err = 0;

	mutex_lock(&k3_bq275x0_battery_mutex);

	err = i2c_smbus_write_word_data(client, reg, value);
	if (err < 0)  {
		BQ275x0_ERR("[%s,%d] i2c_smbus_write_word_data failed\n", __FUNCTION__, __LINE__);
	}

	mutex_unlock(&k3_bq275x0_battery_mutex);

	return err;
}

static int k3_bq275x0_i2c_bytes_write(struct i2c_client *client, u8 reg, u8 *pBuf, u16 len)
{
	int i2c_ret 	= 0;
	int i 			= 0;
	int j 			= 0;
	u8 *p		= pBuf;

	mutex_lock(&k3_bq275x0_battery_mutex);

	for (i = 0; i < len; i += I2C_SMBUS_BLOCK_MAX) {

		j = ((len - i) > I2C_SMBUS_BLOCK_MAX) ? I2C_SMBUS_BLOCK_MAX : (len - i);

		i2c_ret = i2c_smbus_write_i2c_block_data(client, reg+i, j, p+i);
		if (i2c_ret < 0)  {
			BQ275x0_ERR("[%s,%d] i2c_transfer failed\n", __FUNCTION__, __LINE__);
		}
	}

	mutex_unlock(&k3_bq275x0_battery_mutex);

	return i2c_ret;
}

static int k3_bq275x0_i2c_bytes_read(struct i2c_client *client, u8 reg, u8 *pBuf, u16 len)
{
	int i2c_ret 	= 0;
	int i 			= 0;
	int j 			= 0;
	u8 *p		= pBuf;

	mutex_lock(&k3_bq275x0_battery_mutex);

	for (i = 0; i < len; i += I2C_SMBUS_BLOCK_MAX) {
		j = ((len - i) > I2C_SMBUS_BLOCK_MAX) ? I2C_SMBUS_BLOCK_MAX : (len - i);

		i2c_ret = i2c_smbus_read_i2c_block_data(client, reg+i, j, p+i);
		if (i2c_ret < 0) {
			BQ275x0_ERR("[%s,%d] i2c_transfer failed\n", __FUNCTION__, __LINE__);
		}
	}

	mutex_unlock(&k3_bq275x0_battery_mutex);

	return i2c_ret;
}

static int k3_bq275x0_i2c_bytes_read_and_compare(struct i2c_client *client, u8 reg, u8 *pSrcBuf, u8 *pDstBuf, u16 len)
{
	int i2c_ret = 0;

	i2c_ret = k3_bq275x0_i2c_bytes_read(client, reg, pSrcBuf, len);
	if (i2c_ret < 0) {
		BQ275x0_ERR("[%s,%d] bq275x0_i2c_bytes_read failed\n", __FUNCTION__, __LINE__);
		return i2c_ret;
	}

	i2c_ret = strncmp(pDstBuf, pSrcBuf, len);

	return i2c_ret;
}
/* added for Firmware upgrade end */
#endif

int bq275x0_battery_temperature(struct i2c_client *client, int* value)
{
	int ret = 0;
    int data = 0;

	data = bq275x0_i2c_read_word(client,BQ275x0_REG_TEMP);
	if(data < 0)
	{
		ret = data;
		BQ275x0_ERR("[%s,%d] get temperature failed\n",__FUNCTION__,__LINE__);
	}
	else
	{
		*value = (data-CONST_NUM_TEMP);
		BQ275x0_DBG("[%s,%d] read temperature '%d' Celsius\n",__FUNCTION__,__LINE__,*value);
	}
	return ret;
}

int bq275x0_battery_voltage(struct i2c_client *client, int* value)
{
	int ret = 0;
    int data = 0;

	data = bq275x0_i2c_read_word(client,BQ275x0_REG_VOLT);
	if(data < 0)
	{
		ret = data;
		BQ275x0_ERR("[%s,%d] get voltage failed\n",__FUNCTION__,__LINE__);
	}
	else
	{
		*value = data;
		BQ275x0_DBG("[%s,%d] read voltage '%d' mV\n",__FUNCTION__,__LINE__,*value);
	}
	return ret;
}

int bq275x0_battery_current(struct i2c_client *client, short * value)
{
	int     data    = 0;
    short   nCurr           = 0;

	data = bq275x0_i2c_read_word(client,BQ275x0_REG_AI);

    nCurr  =  (signed short)data;
	*value = nCurr;
    BQ275x0_DBG("[%s,%d] read current '%d' mA\n",__FUNCTION__,__LINE__, nCurr);

	return 0;
}

int bq275x0_remaining_capacity(struct i2c_client *client, int* value)
{
	int ret = 0;
    int data = 0;
	data = bq275x0_i2c_read_word(client,BQ275x0_REG_RCAP);
	if(data < 0)
	{
		ret = data;
		BQ275x0_ERR("[%s,%d] get remaining capacity failed\n",__FUNCTION__,__LINE__);
	}
	else
	{
		*value = data;
		BQ275x0_DBG("[%s,%d] remaining capacity  = %d mAH\n",__FUNCTION__,__LINE__,data);
	}
	return ret;
}

int bq275x0_full_charge_capacity(struct i2c_client *client, int* value)
{
	int ret = 0;
    int data = 0;
	data = bq275x0_i2c_read_word(client,BQ275x0_REG_FCC);
	if(data < 0)
	{
		ret = data;
		BQ275x0_ERR("[%s,%d] get full charge capacity failed\n",__FUNCTION__,__LINE__);
	}
	else
	{
		*value = data;
		BQ275x0_DBG("[%s,%d] full charge capacity  = %d mAH\n",__FUNCTION__,__LINE__,data);
	}
	return ret;
}

#ifdef BARCELONA_SHOW

int bq275x0_battery_SDP_capacity(struct i2c_client *client, int* value)
{
    int voltage_now = 0;
    int ret = 0;
    #if 0
    int data = 0;
    #endif

    //printk(KERN_INFO "LEON:%s11111111111\n", __func__);
    ret = bq275x0_battery_voltage(client, &voltage_now);
    //printk(KERN_EMERG "LEON:%s get battery voltage=%d\n",__FUNCTION__,voltage_now);
    if(!ret){
        battery_capacity_voltage_map(&capacitymap_SDP_for_Barcelona,
                                sizeof(capacitymap_SDP_for_Barcelona)/sizeof(capacitymap_SDP_for_Barcelona[0]),
                                voltage_now,
                                value);
    //    printk(KERN_EMERG "LEON:%s get battery chg capacity=%d\n", __FUNCTION__,*value);
    }

    return ret;
}



int bq275x0_battery_capacity(struct i2c_client *client, int* value)
{
    int voltage_now = 0;
    int ret = 0;
    #if 0
    int data = 0;
    #endif
    //printk(KERN_INFO "LEON:%s2222222222222222\n", __func__);

    ret = bq275x0_battery_voltage(client, &voltage_now);
    //printk(KERN_EMERG "LEON:%s get battery voltage=%d\n",__FUNCTION__,voltage_now);
    if(!ret){
        #if 1
        battery_capacity_voltage_map(&capacitymap_for_Barcelona,
                                sizeof(capacitymap_for_Barcelona)/sizeof(capacitymap_for_Barcelona[0]),
                                voltage_now,
                                value);
        #endif
        #if 0
        data = calc_capacity_from_voltage(voltage_now);
        *value = data;
        #endif
     //   printk(KERN_EMERG "LEON:%s get battery capacity=%d\n", __FUNCTION__,*value);
    }

    return ret;
}

int bq275x0_battery_DCP_capacity(struct i2c_client *client, int* value)
{
    int voltage_now = 0;
    int ret = 0;
    #if 0
    int data = 0;
    #endif

    //printk(KERN_INFO "LEON:%s33333333333\n", __func__);
    ret = bq275x0_battery_voltage(client, &voltage_now);
    //printk(KERN_EMERG "LEON:%s get battery voltage=%d\n",__FUNCTION__,voltage_now);
    if(!ret){
        battery_capacity_voltage_map(&capacitymap_DCP_for_Barcelona,
                                sizeof(capacitymap_DCP_for_Barcelona)/sizeof(capacitymap_DCP_for_Barcelona[0]),
                                voltage_now,
                                value);
     //   printk(KERN_EMERG "LEON:%s get battery chg capacity=%d\n", __FUNCTION__,*value);
    }

    return ret;
}

#else
int bq275x0_battery_capacity(struct i2c_client *client, int* value)
{
	int ret = 0;
    int data = 0;

#if 0
	bq275x0_remaining_capacity(di);
	bq275x0_full_charge_capacity(di);
#endif

	data = bq275x0_i2c_read_word(client,BQ275x0_REG_SOC);
	if(data < 0)
	{
		ret = data;
		BQ275x0_ERR("[%s,%d] get battery capacity failed\n",__FUNCTION__,__LINE__);
	}
	else
	{
		*value = data;
		BQ275x0_DBG("[%s,%d] battery capacity  = %d mAH\n",__FUNCTION__,__LINE__,data);
	}
	return ret;

}
#endif /* BARCELONA_SHOW */
/*
 * Return the battery Time to Empty
 */
int bq275x0_battery_tte(struct i2c_client *client, int* value)
{
	int ret = 0;
    int data = 0;
	data = bq275x0_i2c_read_word(client,BQ275x0_REG_TTE);
	if(data < 0)
	{
		ret = data;
		BQ275x0_ERR("[%s,%d] get tte failed\n",__FUNCTION__,__LINE__);
	}
	else
	{
		*value = data;
		BQ275x0_DBG("[%s,%d] tte result = %d minutes\n",__FUNCTION__,__LINE__,data);
	}
	return ret;
}

/*
 * Return the battery Time to Full
 */
int bq275x0_battery_ttf(struct i2c_client *client, int* value)
{
	int ret = 0;
    int data = 0;
	data = bq275x0_i2c_read_word(client,BQ275x0_REG_TTF);
	if(data < 0)
	{
		ret = data;
		BQ275x0_ERR("[%s,%d] get ttf failed\n",__FUNCTION__,__LINE__);
	}
	else
	{
		*value = data;
		BQ275x0_DBG("[%s,%d] ttf result = %d minutes\n",__FUNCTION__,__LINE__,data);
	}
	return ret;
}

#if 1 
static int bq275x0_battery_present(struct i2c_client *client, int* value)
{
        int ret = 0;
        int data = 0;
        data = bq275x0_i2c_read_word(client,BQ275x0_REG_FLAGS);

        if(data < 0)
        {
            ret = data;
            BQ275x0_ERR("[%s,%d] get battery present failed\n",__FUNCTION__,__LINE__);
        }
        else
        {
            *value = data;
            BQ275x0_DBG("[%s,%d] battery present result = %d\n",__FUNCTION__,__LINE__,data);
        }

        BQ275x0_DBG("read battery present result = 0x%d \n",!!(data & BQ275x0_FLAG_DET));
        return !!(data & BQ275x0_FLAG_DET);

}
#endif

int bq275x0_battery_health(struct i2c_client *client, int* value)
{
    int data = 0;
    int status = 0;

    data = bq275x0_i2c_read_word(client,BQ275x0_REG_FLAGS);

    if(data < 0)
    {
        status = POWER_SUPPLY_HEALTH_UNKNOWN;
        BQ275x0_ERR("[%s,%d] get battery health failed\n",__FUNCTION__,__LINE__);
    }
    else
    {
        *value = data;
        BQ275x0_DBG("[%s,%d] battery health result = %d\n",__FUNCTION__,__LINE__,data);

        //if (data & (BQ275x0_FLAG_OTC | BQ275x0_FLAG_OTD))
        if (data & (BQ275x0_FLAG_OTD))
            status = POWER_SUPPLY_HEALTH_OVERHEAT;
        else
            status = POWER_SUPPLY_HEALTH_GOOD;

    }

	return status;
}

#define MAX_UP_POINT		4
#define MAX_DOWN_POINT	4
static int old_battery_capacity = -1;
static int old_charging = 0;
static int charging = 0;
#define SMOOTH_ADJUST	1
#define MAX_SMOOTH_STEP	4
static int smooth_step = 0;

static int smooth_capacity(int new_cap)
{
	int difference;
	int result = 0;

	//if((10 > new_cap)&&(3 <= new_cap)){
	//	new_cap = 2;
	//}
	
	if(charging == 0){
		if((0 == smooth_step)&&(9 == new_cap%10)&&(90 > new_cap)){
			smooth_step = 1;
		}
	}
	else{
		if((0 == smooth_step)&&(0 == new_cap%10)&&(91 > new_cap)){
			smooth_step = 1;
		}
	}

    if(new_cap < 15){
		new_cap = (new_cap + 1)/2;
	}
		
	if((20 > new_cap)&&(15 <= new_cap)){
		new_cap = new_cap - (9 - 20/10);
	}
	if((90 > new_cap)&&(20 <= new_cap))
	new_cap = new_cap - (9 - new_cap/10);
	
	//printk("old_battery_capacity %d \n smooth_step %d  middle_num is %d\n",old_battery_capacity,smooth_step,new_cap);

	if(-1 == old_battery_capacity){
		old_battery_capacity = new_cap;
	}
		
	if(charging == 0){		//NO_charging
		difference = old_battery_capacity - new_cap;
		if(0 > difference){
			return old_battery_capacity;
		}

		if(0 != smooth_step){
			if(0 != difference){
			result = old_battery_capacity - SMOOTH_ADJUST;
			old_battery_capacity = result;
			}
			return old_battery_capacity;
		}
		
		if(MAX_DOWN_POINT <= difference){
			result = old_battery_capacity - MAX_DOWN_POINT;
			old_battery_capacity = result;
			return result;
		}
	}
	else{		//charging
		difference = new_cap - old_battery_capacity;
		if(0 > difference){
		return old_battery_capacity;
		}
		
		if(0 != smooth_step){
			if(0 != difference){
			result = old_battery_capacity + SMOOTH_ADJUST;
			old_battery_capacity = result;
			}
			return old_battery_capacity;
		}

		if(MAX_UP_POINT <= difference){
			result = old_battery_capacity + MAX_DOWN_POINT;
			old_battery_capacity = result;
			return result;
		}
	}
	
	return new_cap;
}

static void sort_data(int* data, int len)
{  
	int i = 0;
	int j = 0;
	int temp =0;  

	for (i = 0; i < len; i++) {  
		for (j = i+1; j < len; j++) {  
			if(data[i]>data[j]){  
				temp = data[j];  
				data[j] = data[i];  
				data[i] = temp;  
			}  
		}  
	}  
}

static int bq275x0_get_battery_info(int prop, struct k3_battery_info* battery_info)
{
	int ret = 0;
	int value = 0;
    short value1 = 0;
    
	int i = 0;
    int health = 0;

	int tmp = 0;
	short value_c = 0;

#ifdef BARCELONA_SHOW
    int chg_online = 0;
    int chg_type = 0;
#endif
	struct i2c_client *client = g_bq275x0_i2c_client;

    BQ275x0_DBG("%s:prop:%d\n",__func__, prop);

    if(0 == prop){
          return -EINVAL;
    }

//battery capacity jump begin
    bq275x0_battery_current(g_bq275x0_i2c_client,&value_c);
	old_charging = charging;
	if(0 > smooth_step){
		smooth_step = 0;
	}
	if(0 > value_c){ //No_charging
		charging = 0;
	}
	else{		//charging
		charging = 1;
	}
	
	if(0 == smooth_step){
		if(old_charging != charging)
			smooth_step = MAX_SMOOTH_STEP;
	}


	
#ifdef BARCELONA_SHOW
    if(prop & (1<<BATTERY_PROP_DCP_ONLINE)){
        chg_online = 1;
        chg_type = POWER_SUPPLY_TYPE_USB_DCP;
    }
    else if(prop & (1<<BATTERY_PROP_SDP_ONLINE)){
        chg_online = 1;
        chg_type = POWER_SUPPLY_TYPE_USB;
    }
#endif
	while(i < BATTERY_PROP_MAX)
	{
		if(prop & (1<<i))
		{

		    BQ275x0_DBG("%s:i:%d\n",__func__, i);

			switch(i)
			{
				case BATTERY_PROP_TECHNOLOGY:
				battery_info->battery_technology = POWER_SUPPLY_TECHNOLOGY_LION;
				break;
				case BATTERY_PROP_PRESENT:
			     battery_info->battery_present = 1; 
                //TODO:battery 是否在位须实现,目前用电压来判断且有魔鬼数字
                ret = bq275x0_battery_present(client,&value);
               //ret = bq275x0_battery_voltage(client,&value);
               //if(!ret && value > 2300){
				if(ret){
			     battery_info->battery_present = 1;
                }
               else{
			   		ret = bq275x0_battery_voltage(client,&value);
					if(value < 2300){
                    	battery_info->battery_present = 0;
					}
                }
				break;
				case BATTERY_PROP_TEMPERATURE:
				ret = bq275x0_battery_temperature(client,&value);
				if(!ret)
				{
					battery_info->battery_temperture = value;
				}
				break;
				case BATTERY_PROP_VOLTAGE:
				ret = bq275x0_battery_voltage(client,&value);
				if(!ret)
				{
					battery_info->battery_voltage = value;
				}
				break;
				case BATTERY_PROP_CURRENT:
				ret = bq275x0_battery_current(client,&value1);
				if(!ret)
				{
					battery_info->battery_current = value1;
				}
				break;
				case BATTERY_PROP_CAPACITY:
#ifdef BARCELONA_SHOW
                if(!chg_online){
				    ret = bq275x0_battery_capacity(client,&value);
                }
                else{
                    if(chg_type == POWER_SUPPLY_TYPE_USB_DCP){
                        ret = bq275x0_battery_DCP_capacity(client,&value);
                    }
                    else{
                        ret = bq275x0_battery_SDP_capacity(client,&value);
                    }
                }
#else
                ret = bq275x0_battery_capacity(client,&value);
#endif
				if(!ret)
				{										
			      		 if(value >= 0 && value <= 100){
			        		tmp = value;
			
					         old_battery_capacity = smooth_capacity(value);
					         if(0 != smooth_step){
						           smooth_step--;
				        	 }
				   		 	 BQ275x0_DBG("smooth_capacity ver 2.2 charging is %d  new_cap is %d  modified_cap is %d\n",charging,tmp,old_battery_capacity);
			      		 }

					//battery_info->battery_capacity = value;
					battery_info->battery_capacity = old_battery_capacity;
				}
				break;
				case BATTERY_PROP_TIME_TO_EMPTY:
				ret = bq275x0_battery_tte(client,&value);
				if(!ret)
				{
					battery_info->battery_time2empty = value;
				}
				break;
				case BATTERY_PROP_TIME_TO_FULL:
				ret = bq275x0_battery_ttf(client,&value);
				if(!ret)
				{
					battery_info->battery_time2full = value;
				}
				break;
                case BATTERY_PROP_HEALTH:
				health = bq275x0_battery_health(client,&value);
		      	battery_info->battery_health = health;
                break;
                default:
				//ret = -EINVAL;
				break;
			}

            msleep(5);
		}
        i++;
	}
	return ret;
}

#ifdef CONFIG_BQ275X0_FIRMWARE_UPDATE
#if 0
extern ssize_t bq275x0_attr_show(struct device_driver *driver, char *buf);
extern ssize_t bq275x0_attr_store(struct device_driver *driver,const char *buf, size_t count);

static DRIVER_ATTR(state, 0664, bq275x0_attr_show, bq275x0_attr_store);
#endif

/* added for Firmware upgrade begin */
static int k3_bq275x0_atoi(const char *s)
{
	int k = 0;

	k = 0;
	while (*s != '\0' && *s >= '0' && *s <= '9') {
		k = 10 * k + (*s - '0');
		s++;
	}
	return k;
}

static unsigned long k3_bq275x0_strtoul(const char *cp, unsigned int base)
{
	unsigned long result = 0, value;

	while (isxdigit(*cp) && (value = isdigit(*cp) ? *cp-'0' : (islower(*cp)
	    ? toupper(*cp) : *cp)-'A'+10) < base) {
		result = result*base + value;
		cp++;
	}

	return result;
}

/*Parse the bqfs/dffs file's addr and data*/
static int k3_bq275x0_firmware_program(struct i2c_client *client, const unsigned char *pgm_data, unsigned int filelen)
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
	if (ulCounter > 10) {
		return -1;
	}

	p_cur = (unsigned char *)pgm_data;

	while (1) {
		if ((p_cur - pgm_data) >= filelen) {
			printk("Download success\n");
			break;
		}

		while (*p_cur == '\r' || *p_cur == '\n') {
			p_cur++;
		}

		i = 0;
		ulLineLen = 0;

		memset(p_src, 0x00, sizeof(p_src));
		memset(p_dst, 0x00, sizeof(p_dst));
		memset(pBuf, 0x00, sizeof(pBuf));

		while (i < BSP_MAX_ASC_PER_LINE) {
			temp = *p_cur++;
			i++;
			if (('\r' == temp) || ('\n' == temp)) {
				break;
			}
			if (' ' != temp) {
				pBuf[ulLineLen++] = temp;
			}
		}

		p_src[0] = pBuf[0];
		p_src[1] = pBuf[1];

		if (('W' == p_src[0]) || ('C' == p_src[0])) {
			for (i = 2, j = 0; i < ulLineLen; i += 2, j++) {
				memset(ucTmpBuf, 0x00, sizeof(ucTmpBuf));
				memcpy(ucTmpBuf, pBuf+i, 2);
				p_src[2+j] = k3_bq275x0_strtoul(ucTmpBuf, 16);
			}
			temp = (ulLineLen - 2)/2;
			ulLineLen = temp + 2;
		} else if ('X' == p_src[0]) {
			memset(ucTmpBuf, 0x00, sizeof(ucTmpBuf));
			memcpy(ucTmpBuf, pBuf+2, ulLineLen-2);
			ulDelay = k3_bq275x0_atoi(ucTmpBuf);
		} else if ('R' == p_src[0]) {
		memset(ucTmpBuf, 0x00, sizeof(ucTmpBuf));
		memcpy(ucTmpBuf, pBuf+2, 2);
		p_src[2] = k3_bq275x0_strtoul(ucTmpBuf, 16);
		memset(ucTmpBuf, 0x00, sizeof(ucTmpBuf));
		memcpy(ucTmpBuf, pBuf+4, 2);
		p_src[3] = k3_bq275x0_strtoul(ucTmpBuf, 16);
		memset(ucTmpBuf, 0x00, sizeof(ucTmpBuf));
		memcpy(ucTmpBuf, pBuf+6, ulLineLen-6);
		ulReadNum = k3_bq275x0_atoi(ucTmpBuf);
		}

		if (':' == p_src[1]) {
			switch (p_src[0]) {
			case 'W':
#if 0
				/* firmware upgrade */
				printk("W: ");
				for (i = 0; i < ulLineLen-4; i++) {
					printk("%x ", p_src[4+i]);
				}
				printk(KERN_ERR "\n");
#endif
				if (k3_bq275x0_i2c_bytes_write(client, p_src[3], &p_src[4], ulLineLen-4) < 0) {
					printk(KERN_ERR "[%s,%d] bq275x0_i2c_bytes_write failed len=%d\n", __FUNCTION__, __LINE__, ulLineLen-4);
				}
				break;
			case 'R':
				if (k3_bq275x0_i2c_bytes_read(client, p_src[3], p_dst, ulReadNum) < 0) {
					printk(KERN_ERR "[%s,%d] bq275x0_i2c_bytes_read failed\n", __FUNCTION__, __LINE__);
				}
				break;
			case 'C':
				if (k3_bq275x0_i2c_bytes_read_and_compare(client, p_src[3], p_dst, &p_src[4], ulLineLen-4)) {
					ulCounter++;
					printk(KERN_ERR "[%s,%d] bq275x0_i2c_bytes_read_and_compare failed\n", __FUNCTION__, __LINE__);
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

static int k3_bq275x0_firmware_download(struct i2c_client *client, const unsigned char *pgm_data, unsigned int len)
{
	int iRet;

	/*Enter Rom Mode */
	iRet = k3_bq275x0_i2c_word_write(client, BSP_ENTER_ROM_MODE_CMD, BSP_ENTER_ROM_MODE_DATA);
	if (0 != iRet) {
		printk(KERN_ERR "[%s,%d] bq275x0_i2c_word_write failed\n", __FUNCTION__, __LINE__);
	}

	mdelay(10);

	/*change i2c addr*/
	g_bq275x0_i2c_client->addr = BQ275x0_FIRMWARE_DOWNLOAD_MODE_I2C_ADDR;

	/*program bqfs*/
	iRet = k3_bq275x0_firmware_program(client, pgm_data, len);
	if (0 != iRet) {
		printk(KERN_ERR "[%s,%d] bq275x0_firmware_program failed\n", __FUNCTION__, __LINE__);
	}

	/*change i2c addr*/
	g_bq275x0_i2c_client->addr = BQ275X0_I2C_ADDR;

	return iRet;
}

static int k3_bq275x0_update_firmware(struct i2c_client *client, const char *pFilePath)
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
	if (IS_ERR(filp)) {
		printk(KERN_ERR "[%s,%d] filp_open failed\n", __FUNCTION__, __LINE__);
		set_fs(oldfs);
		return -1;
	}

	if (!filp->f_op) {
		printk(KERN_ERR "[%s,%d] File Operation Method Error\n", __FUNCTION__, __LINE__);
		filp_close(filp, NULL);
		set_fs(oldfs);
		return -1;
	}

	inode = filp->f_path.dentry->d_inode;
	if (!inode) {
		printk(KERN_ERR "[%s,%d] Get inode from filp failed\n", __FUNCTION__, __LINE__);
		filp_close(filp, NULL);
		set_fs(oldfs);
		return -1;
	}

	/* file's size */
	length = i_size_read(inode->i_mapping->host);
	printk("bq275x0 firmware image size is %d \n", length);
	if (!(length > 0 && length < BSP_FIRMWARE_FILE_SIZE)) {
		printk(KERN_ERR "[%s,%d] Get file size error\n", __FUNCTION__, __LINE__);
		filp_close(filp, NULL);
		set_fs(oldfs);
		return -1;
	}

	/* allocation buff size */
	/* buf size if even */
	buf = vmalloc(length+(length%2));
	if (!buf) {
		printk(KERN_ERR "[%s,%d] Alloctation memory failed\n", __FUNCTION__, __LINE__);
		filp_close(filp, NULL);
		set_fs(oldfs);
		return -1;
	}

	/* read data */
	if (filp->f_op->read(filp, buf, length, &filp->f_pos) != length) {
		printk(KERN_ERR "[%s,%d] File read error\n", __FUNCTION__, __LINE__);
		filp_close(filp, NULL);
		filp_close(filp, NULL);
		set_fs(oldfs);
		vfree(buf);
		return -1;
	}

	ret = k3_bq275x0_firmware_download(client, (const char *)buf, length);

	filp_close(filp, NULL);
	set_fs(oldfs);
	vfree(buf);

	return ret;
}

/*Firmware upgrade sysfs store interface*/
static ssize_t k3_bq275x0_attr_store(struct device_driver *driver, const char *buf, size_t count)
{
	int iRet = 0;
	unsigned char path_image[255];

	if (NULL == buf || count > 255 || count == 0 || strnchr(buf, count, 0x20))
		return -1;

	memcpy (path_image, buf,  count);
	/* replace '\n' with  '\0'  */
	if ((path_image[count-1]) == '\n')
		path_image[count-1] = '\0';
	else
		path_image[count] = '\0';

	 /*enter firmware bqfs download*/
	 bq275x0_work_mode = BSP_FIRMWARE_DOWNLOAD_MODE;
	 iRet = k3_bq275x0_update_firmware(g_bq275x0_i2c_client, path_image);
	 bq275x0_work_mode = BSP_NORMAL_MODE;

	 dev_info(&g_bq275x0_i2c_client->dev, "k3_bq275x0_update_firmware return value is %d\n", iRet);
	/* added for shutdown system in charging, begin */
	/* begin: added for refresh Qmax*/
	//i2c_smbus_write_word_data(g_bq275x0_i2c_client, 0x00, 0x0021);
	i2c_smbus_write_word_data(g_bq275x0_i2c_client, 0x00, 0x0041);
	/* end: added for refresh Qmax*/
	/* added for shutdown system in charging, end */

	return iRet;
}

/* Firmware upgrade sysfs show interface*/
static ssize_t k3_bq275x0_attr_show(struct device_driver *driver, char *buf)
{
	int iRet = 0;

	if (NULL == buf) {
		sprintf(buf, "%s", "k3_bq275x0_attr_show Error");
		return -1;
	}

	mutex_lock(&k3_bq275x0_battery_mutex);
	i2c_smbus_write_word_data(g_bq275x0_i2c_client, 0x00, 0x0008);
	mdelay(2);
	iRet = i2c_smbus_read_word_data(g_bq275x0_i2c_client, 0x00);
	mutex_unlock(&k3_bq275x0_battery_mutex);
	if (iRet < 0) {
		return sprintf(buf, "%s", "Coulometer Damaged or Firmware Error");
	} else {
		return sprintf(buf, "%x", iRet);
	}
}

/*define a sysfs interface for firmware upgrade*/
//static DRIVER_ATTR(state, S_IRUGO|S_IWUGO, k3_bq275x0_attr_show, k3_bq275x0_attr_store);
static DRIVER_ATTR(state, S_IRUGO|S_IWUSR|S_IWGRP, k3_bq275x0_attr_show, k3_bq275x0_attr_store);

/* added for Firmware upgrade end */
#endif

static int bq275x0_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    int ret                                       = 0;
    int value                                       = 0;
    //int retval = 0;
    //int num = 0;
	struct bq275x0_i2c_platform_data*platdata     = NULL;
	struct battery_report_i2c_dev* dev_id         = NULL;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "%s: failed to check i2c function!\n", __func__);
		ret = -EIO;
		goto err_check_i2c_func;
	}

#if defined(CONFIG_HARDWARE_OLD) 
   // check bq275x0 exist
    value = i2c_smbus_read_word_data(client,BQ275x0_REG_VOLT);
    if(value < 0)
    {
        dev_err(&client->dev, "%s: failed register bq275x0 on i2c bus!\n", __func__);
		ret = -EIO;
        goto err_check_chip_exist;
    }
#endif

	platdata = client->dev.platform_data;
	if (!platdata) {
		ret = -EINVAL;
		goto err_get_platform_data;
	}

	dev_id = kzalloc(sizeof(struct battery_report_i2c_dev), GFP_KERNEL);
	if (!dev_id) {
		ret = -ENOMEM;
		goto err_alloc_data_failed;
	}

	dev_id ->client = client;
	dev_id ->client->dev.init_name = BQ275X0_NAME;

	i2c_set_clientdata(client, dev_id);


	//register bq275x0 i2c client to battery report driver
	battery_bq275x0 = kzalloc(sizeof(struct k3_battery_gauge_dev), GFP_KERNEL);
	if (!battery_bq275x0) {
		ret = -ENOMEM;
		goto err_alloc_battery_failed;
	}


#ifdef CONFIG_BQ275X0_FIRMWARE_UPDATE
	/* added for Firmware upgrade begin*/
	i2c_smbus_write_word_data(client, 0x00, 0x0008);

	mdelay(2);

	ret = i2c_smbus_read_word_data(client, 0x00);
	if (ret < 0) {
		dev_err(&client->dev, "[%s,%d] Coulometer Damaged or Firmware Error\n", __FUNCTION__, __LINE__);
	} else {
		dev_info(&client->dev, "Normal Mode and read Firmware version=%04x\n", ret);
	}

	ret = driver_create_file(&(bq275x0_driver.driver), &driver_attr_state);
       if (0 != ret) {
		printk(KERN_ERR "failed to create sysfs entry(state): %d\n", ret);
		ret =  -ENOENT;
		goto err_create_file_failed;
	}
	/*added for Firmware upgrade end*/
#endif

	g_bq275x0_i2c_client = client;
	strcpy(battery_bq275x0->name,MAIN_BATTERY);
	battery_bq275x0->get_battery_info = bq275x0_get_battery_info;


#if 0//#if defined(BQ275X0_DEBUG_FLAG) //
    mdelay(200);
    for(num = 0; num <= 5; num++){
        ret = bq275x0_battery_capacity(client,&value);
        BQ275x0_DBG("bq275x0_probe bq275x0_battery_capacity:ret=%d,value=%d!!\n",ret, value);
        mdelay(2);
        ret = bq275x0_battery_voltage(client,&value);
        BQ275x0_DBG("bq275x0_probe bq275x0_battery_voltage: ret=%d, value=%d!!\n", ret, value);
        mdelay(2);
    }
#endif

	k3_psy_monitor_register_battery(battery_bq275x0);

    BQ275x0_DBG("%s end!!\n", __func__);

	return 0;

#ifdef CONFIG_BQ275X0_FIRMWARE_UPDATE
err_create_file_failed:
	kfree(battery_bq275x0);
	battery_bq275x0 = NULL;
#endif
err_alloc_battery_failed:
	i2c_set_clientdata(client, NULL);
	kfree(dev_id);
	dev_id = NULL;
err_alloc_data_failed:
err_get_platform_data:
#if defined(CONFIG_HARDWARE_OLD) 
err_check_chip_exist:  //
#endif
err_check_i2c_func:
	return ret;
}

static int bq275x0_remove(struct i2c_client *client)
{

	struct battery_report_i2c_dev* dev_id = i2c_get_clientdata(client);

	//	unregister_report_i2c_dev(client);

#ifdef CONFIG_BQ275X0_FIRMWARE_UPDATE
	driver_remove_file(&(bq275x0_driver.driver), &driver_attr_state);
#endif

	k3_psy_monitor_unregister_battery(battery_bq275x0);

	if(dev_id){
		kfree(dev_id);
	}
	if(battery_bq275x0){
		kfree(battery_bq275x0);
        battery_bq275x0 = NULL;
	}
	//资源释放
	return 0;
}

#ifdef CONFIG_PM
static int k3_bq27510_charger_suspend(struct i2c_client *client,
        pm_message_t state)
{
        printk("[%s] +\n", __func__);

	    old_battery_capacity = -1;
       	smooth_step = 0;
        
        printk("[%s] -\n", __func__);

        return 0;
}

static int k3_bq27510_charger_resume(struct i2c_client *client)
{
        printk("[%s] +\n", __func__);
        printk("[%s] -\n", __func__);

        return 0;
}
#else
#define k3_bq27510_charger_suspend      NULL
#define k3_bq27510_charger_resume       NULL
#endif

static const struct i2c_device_id bq275x0_id[] = {
	{BQ275X0_NAME, 0 },
	{ }
};
static struct i2c_driver bq275x0_driver = {
	.probe		= bq275x0_probe,
	.remove		= bq275x0_remove,
    .suspend    = k3_bq27510_charger_suspend,
    .resume     = k3_bq27510_charger_resume,
	.id_table	= bq275x0_id,
	.driver = {
		.name	= BQ275X0_NAME,
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

MODULE_AUTHOR("Hisilicon K3");
MODULE_DESCRIPTION("I2C bq275x0 Driver");
MODULE_LICENSE("GPL");
