/* drivers/input/touchscreen/atmel_mXT1386.c - ATMEL Touch driver
 *
 * Copyright (C) 2011 Huawei Corporation.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/module.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/earlysuspend.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/jiffies.h>
#include <linux/stat.h>
#include <linux/slab.h>
#include <linux/mux.h>
#include <linux/regulator/consumer.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
#include <linux/atmel_mxt1386.h>
#include <linux/ctype.h>
extern int tp_probe_status;
//the first tp sample 
//int and reset is high active
#define ATMEL_BUG 0

#define ATMEL_EN_SYSFS
#define ATMEL_I2C_RETRY_TIMES (2)
#define TPTYPE_MXT1386			"ATMEL_MXT1386"
#ifdef ATMEL_EN_SYSFS
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/vmalloc.h>
#endif
struct atmel_ts_data {
	struct i2c_client *client;
	struct input_dev *input_dev;
	struct workqueue_struct *atmel_wq;
	struct work_struct work;
	struct delayed_work calibrate_dwork;
	struct early_suspend early_suspend;
	struct info_id_t *id;
	struct object_t *object_table;
	uint8_t finger_count;
	uint16_t abs_x_min;
	uint16_t abs_x_max;
	uint16_t abs_y_min;
	uint16_t abs_y_max;
	uint8_t abs_area_min;
	uint8_t abs_area_max;
	uint8_t abs_pressure_min;
	uint8_t abs_pressure_max;
	uint8_t debug_log_level;
	struct atmel_finger_data finger_data[MAX_NUM];
	uint8_t finger_type;
	uint8_t finger_support;
	uint16_t finger_pressed;
	uint8_t face_suppression;
	uint8_t noise_status[2];
	uint8_t calibration_confirm;
	uint64_t timestamp;
	struct atmel_config_data config_setting;
	uint8_t GCAF_sample;
	uint8_t *GCAF_level;
	uint8_t noisethr;
	uint8_t noisethr_config;
	uint8_t diag_command;
	uint8_t *ATCH_EXT;
	int pre_data[11];
#ifdef ATMEL_EN_SYSFS
	struct device dev;
#endif
	struct report_id_map *report_id_map;

};
#define is_valid_data_space(c)	(c == ' ' || c == '\t')
#define is_valid_space(c)			(is_valid_data_space(c) \
									|| c == 10 || c == 13 )
#define is_valid_alphanumeric(c)	((c >= '0' && c <= '9') \
									|| (c >= 'a' && c <= 'f') \
									|| (c >= 'A' && c <= 'F'))
#define is_valid_annotation(c)	(c == '#')
#define is_valid_data(c)			(is_valid_space(c) \
									|| is_valid_alphanumeric(c) \
									|| is_valid_annotation(c))
#define char_to_numeric(c)		((c >= '0' && c <= '9') ? (c-'0'):\
									((c >= 'a' && c <= 'f') ? (c-'a'+10):\
									((c >= 'A' && c <= 'F') ? (c-'A'+10) : -1)))
#define MXT1386_MAX_OBJECT_NUM					32
#define MXT1386_MAX_OBJECT_CONFIGDATA_NUM		128
u8 t1386_config_data[MXT1386_MAX_OBJECT_NUM][MXT1386_MAX_OBJECT_CONFIGDATA_NUM];

static struct atmel_ts_data *private_ts = NULL;
static struct i2c_client *private_client = NULL;
static int i2c_update_firmware(const char * file, size_t count);
static irqreturn_t atmel_ts_irq_handler(int irq, void *dev_id);
static int t1386_update_firmware(struct atmel_ts_data *mxt);
#ifdef CONFIG_HAS_EARLYSUSPEND
static void atmel_ts_early_suspend(struct early_suspend *h);
static void atmel_ts_late_resume(struct early_suspend *h);
#endif

static int i2c_atmel_read(struct i2c_client *client, uint16_t address, uint8_t *data, uint8_t length)
{
	int retry = 0;
	uint8_t addr[2] = {0};

	struct i2c_msg msg[] = {
		{
			.addr = client->addr,
			.flags = 0,
			.len = 2,
			.buf = addr,
		},
		{
			.addr = client->addr,
			.flags = I2C_M_RD,
			.len = length,
			.buf = data,
		}
	};
	addr[0] = address & 0xFF;
	addr[1] = (address >> 8) & 0xFF;

	for (retry = 0; retry < ATMEL_I2C_RETRY_TIMES; retry++) {
		if (i2c_transfer(client->adapter, msg, 2) == 2)
			break;
		mdelay(10);
	}
    
	if (retry == ATMEL_I2C_RETRY_TIMES) {
		dev_err(&client->dev,  "k3ts, %s: i2c_read_block retry over %d\n", __func__,
			ATMEL_I2C_RETRY_TIMES);
		return -EIO;
	}
	return 0;
}

static int i2c_atmel_write(struct i2c_client *client, uint16_t address, uint8_t *data, uint8_t length)
{
	int retry = 0;
    int loop_i = 0;
	uint8_t buf[length + 2];

	struct i2c_msg msg[] = {
		{
			.addr = client->addr,
			.flags = 0,
			.len = length + 2,
			.buf = buf,
		}
	};

	buf[0] = address & 0xFF;
	buf[1] = (address >> 8) & 0xFF;

	for (loop_i = 0; loop_i < length; loop_i++) {
		buf[loop_i + 2] = data[loop_i];
    }    

	for (retry = 0; retry < ATMEL_I2C_RETRY_TIMES; retry++) {
		if (i2c_transfer(client->adapter, msg, 1) == 1) {
			break;
        }    
		mdelay(10);
	}

	if (retry == ATMEL_I2C_RETRY_TIMES) {
		dev_err(&client->dev, "k3ts, %s: i2c_write_block retry over %d\n", __func__,
			ATMEL_I2C_RETRY_TIMES);
		return -EIO;
	}
	return 0;

}

static int i2c_atmel_write_byte_data(struct i2c_client *client, uint16_t address, uint8_t value)
{
	i2c_atmel_write(client, address, &value, 1);
	return 0;
}

static uint16_t get_object_address(struct atmel_ts_data *ts, uint8_t object_type)
{
	uint8_t loop_i = 0;
	for (loop_i = 0; loop_i < ts->id->num_declared_objects; loop_i++) {
		if (ts->object_table[loop_i].object_type == object_type) {
			return ts->object_table[loop_i].i2c_address;
        }    
	}
	return 0;
}
static uint8_t get_object_size(struct atmel_ts_data *ts, uint8_t object_type)
{
	uint8_t loop_i = 0;
	for (loop_i = 0; loop_i < ts->id->num_declared_objects; loop_i++) {
		if (ts->object_table[loop_i].object_type == object_type) {
			return ts->object_table[loop_i].size;
        }    
	}
	return 0;
}

static uint8_t get_rid(struct atmel_ts_data *ts, uint8_t object_type)
{
	uint8_t loop_i = 0;
	for (loop_i = 0; loop_i < ts->id->num_declared_objects; loop_i++) {
		if (ts->object_table[loop_i].object_type == object_type) {
			return ts->object_table[loop_i].report_ids;
        }    
	}
	return 0;
}

static uint8_t report_id_to_object(struct atmel_ts_data *ts, uint8_t id)
{
    return ts->report_id_map[id].object;
}

static int read_info_msg(struct i2c_client *client,uint8_t *data)
{
	int ret = 0;
    ret = i2c_atmel_read(client, ADDR_INFO_BLOCK, data, ID_BLOCK_SIZE);	
    return ret;
}
static int hw_reset_chip(void)
{
	struct atmel_i2c_platform_data *pdata = private_ts->client->dev.platform_data;
	return (!!ATMEL_BUG) ? pdata->tp_gpio_reset("atmel") : pdata->tp_gpio_reset("atmel_new");
}

static int sw_reset_chip(void)
{
    i2c_atmel_write_byte_data(private_ts->client,
        get_object_address(private_ts, GEN_COMMANDPROCESSOR_T6) +
        T6_CFG_RESET, RESET_CHIP);
    msleep(300);
	return 1;
}

static int backup_to_nv(void)
{
	int ret = 0;
    ret = i2c_atmel_write_byte_data(private_ts->client,
        get_object_address(private_ts, GEN_COMMANDPROCESSOR_T6) + T6_CFG_BACKUPNV,
        BACKUP_CHIP);

	return ret;
}

static int config_enable_mxt1386(void)
{
	int ret = -1;
	struct atmel_i2c_platform_data *pdata = private_ts->client->dev.platform_data;
	
	ret = i2c_atmel_write(private_ts->client,
		get_object_address(private_ts, GEN_POWERCONFIG_T7),
		pdata->config_T7,
		get_object_size(private_ts, GEN_POWERCONFIG_T7));
	if (ret < 0) {
		printk(KERN_ERR "write T7 config failed");
	}
	return ret;
}
#ifdef ATMEL_EN_SYSFS
static ssize_t atmel_gpio_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int ret = 0;
	struct atmel_ts_data *ts_data = NULL;
	struct atmel_i2c_platform_data *pdata = NULL;

	ts_data = private_ts;
	pdata = ts_data->client->dev.platform_data;

	ret = gpio_get_value(pdata->gpio_irq);

	sprintf(buf, "GPIO_TP_INT_N=%d\n", ret);
	ret = strlen(buf) + 1;
	return ret;
}
static DEVICE_ATTR(gpio, S_IRUGO, atmel_gpio_show, NULL);

static ssize_t atmel_diag_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct atmel_ts_data *ts_data = NULL;
	size_t count = 0;
	uint8_t data[T37_PAGE_SIZE] = {0};
	uint8_t loop_i = 0;
    uint8_t loop_j = 0;
	int16_t rawdata = 0;
	int x = 0;
    int y = 0;
    
	ts_data = private_ts;
	memset(data, 0x0, sizeof(data));

	if (ts_data->diag_command != T6_CFG_DIAG_CMD_DELTAS &&
		ts_data->diag_command != T6_CFG_DIAG_CMD_REF) {
		return count;
    }    

	i2c_atmel_write_byte_data(ts_data->client,
		get_object_address(ts_data, GEN_COMMANDPROCESSOR_T6) + T6_CFG_DIAG,
		ts_data->diag_command);

	x = T28_CFG_MODE0_X + ts_data->config_setting.config_T28[T28_CFG_MODE];
	y = T28_CFG_MODE0_Y - ts_data->config_setting.config_T28[T28_CFG_MODE];
	count += sprintf(buf, "Channel: %d * %d\n", x, y);

	for (loop_i = 0; loop_i < 4; loop_i++) {
		for (loop_j = 0;
			!(data[T37_MODE] == ts_data->diag_command && data[T37_PAGE] == loop_i) && loop_j < 10; loop_j++) {
			msleep(5);
			i2c_atmel_read(ts_data->client,
				get_object_address(ts_data, DIAGNOSTIC_T37), data, 2);
		}
		if (loop_j == 10) {
			dev_err(dev, "k3ts, %s: Diag data not ready\n", __func__);
		}
        
		i2c_atmel_read(ts_data->client,
			get_object_address(ts_data, DIAGNOSTIC_T37) +
			T37_DATA, data, T37_PAGE_SIZE);
        
		for (loop_j = 0; loop_j < T37_PAGE_SIZE - 1; loop_j += 2) {
			if ((loop_i * 64 + loop_j / 2) >= (x * y)) {
				count += sprintf(buf + count, "\n");
				return count;
			} else {
				rawdata = data[loop_j+1] << 8 | data[loop_j];
				count += sprintf(buf + count, "%5d", rawdata);
				if (((loop_i * 64 + loop_j / 2) % y) == (y - 1)) {
					count += sprintf(buf + count, "\n");
                }    
			}
		}
		i2c_atmel_write_byte_data(ts_data->client,
			get_object_address(ts_data, GEN_COMMANDPROCESSOR_T6) +
			T6_CFG_DIAG, T6_CFG_DIAG_CMD_PAGEUP);

	}

	return count;
}

static ssize_t atmel_diag_dump(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct atmel_ts_data *ts_data = NULL;
	ts_data = private_ts;
	if (buf[0] == '1') {
		ts_data->diag_command = T6_CFG_DIAG_CMD_DELTAS;
    }    
	if (buf[0] == '2') {
		ts_data->diag_command = T6_CFG_DIAG_CMD_REF;
	}
	return count;
}

static DEVICE_ATTR(diag, (S_IWUSR|S_IRUGO),
	atmel_diag_show, atmel_diag_dump);
#define firmware_attr(_name) \
static struct kobj_attribute _name##_attr = {	\
	.attr	= {				\
		.name = __stringify(_name),	\
		.mode = 0664,			\
	},					\
	.show	= _name##_show,			\
	.store	= _name##_store,		\
}

static ssize_t update_firmware_show(struct kobject *kobj, 
		struct kobj_attribute *attr,char *buf);
static ssize_t update_firmware_store(struct kobject *kobj, 
		struct kobj_attribute *attr, const char *buf, size_t count);
firmware_attr(update_firmware);
static ssize_t firmware_version_show(struct kobject *kobj, 
		struct kobj_attribute *attr,char *buf);
static ssize_t firmware_version_store(struct kobject *kobj, 
		struct kobj_attribute *attr, const char *buf, size_t count);
firmware_attr(firmware_version);
static ssize_t firmware_file_version_show(struct kobject *kobj, 
		struct kobj_attribute *attr,char *buf);
static ssize_t firmware_file_version_store(struct kobject *kobj, 
		struct kobj_attribute *attr, const char *buf, size_t count);
firmware_attr(firmware_file_version);
static ssize_t firmware_tptype_show(struct kobject *kobj, 
		struct kobj_attribute *attr,char *buf);
static ssize_t firmware_tptype_store(struct kobject *kobj, 
		struct kobj_attribute *attr, const char *buf, size_t count);
firmware_attr(firmware_tptype);
static ssize_t tp_control_show(struct kobject *kobj, 
		struct kobj_attribute *attr,char *buf);
static ssize_t tp_control_store(struct kobject *kobj, 
		struct kobj_attribute *attr, const char *buf, size_t count);
firmware_attr(tp_control);

#define FLAG_RR 		0x00
#define FLAG_WR 		0x01
#define FLAG_RPX 		0x02
#define FLAG_RPY 		0x03
#define FLAG_RNIT 		0x04
#define FLAG_HRST 		0x05
#define FLAG_SRST 		0x06
#define FLAG_EINT 		0x07
#define FLAG_DINT 		0x08
#define FLAG_RTA 		0x09
#define FLAG_PRST 		0x0a 
#define FLAG_POFF 		0x0b 
#define FLAG_EFW 		0x0c 
#define FLAG_BR 		0x0d 
#define FLAG_ROA 		0x0e 
#define FLAG_ROS 		0x0f
#define FLAG_RRP       0x10

static ssize_t ts_debug_show(struct kobject *kobj, 
		struct kobj_attribute *attr,char *buf);
static ssize_t ts_debug_store(struct kobject *kobj, 
		struct kobj_attribute *attr, const char *buf, size_t count);
firmware_attr(ts_debug);

#define VERSION_OFFSET  		0
#define VERSION_CHECK_ADDR 	44u
static ssize_t update_firmware_show(struct kobject *kobj, 
		struct kobj_attribute *attr, char *buf)
{
	return 0;
}
static ssize_t update_firmware_store(struct kobject *kobj, 
		struct kobj_attribute *attr, const char *buf, size_t count)
{
	int ret = -1;
	printk("%s: start mxt1386 firmware update download\n", __func__);

	disable_irq(private_client->irq);	
	ret = t1386_update_firmware(private_ts);
	enable_irq(private_client->irq);

	if( 0 != ret ){
	 	printk("Update firmware failed!\n");
	} else {
	 	printk("Update firmware success!\n");
	}
	
	return count;
}
 
static ssize_t firmware_version_show(struct kobject *kobj, 
			struct kobj_attribute *attr, char *buf)
{
	u16 obj_addr, obj_size;
	int error = -1;
	u8 data;

	obj_addr = get_object_address(private_ts, USERDATA_T38);
	obj_size = get_object_size(private_ts, USERDATA_T38);
	
	error = i2c_atmel_read(private_client, obj_addr, &data, 1);
	if (error) {
		printk("%s: start mxt1386 firmware update download\n", __func__);
		return 0;
	}
	return sprintf(buf, "%03d", data);
}

static ssize_t firmware_version_store(struct kobject *kobj, 
			struct kobj_attribute *attr, const char *buf, size_t count)
{
    return 0;
}

static ssize_t firmware_file_version_show(struct kobject *kobj, 
			struct kobj_attribute *attr,char *buf)
{
	int i;
	
	for (i = 0; i < MXT1386_MAX_OBJECT_NUM; i++){
		if(USERDATA_T38 == t1386_config_data[i][0] ){
			return sprintf(buf, "%04d", t1386_config_data[i][2]);
		}
	}
	
	return 1;
}

static ssize_t firmware_file_version_store(struct kobject *kobj, 
			struct kobj_attribute *attr, const char *buf, size_t count)
{
	int ret = -1;
	
	disable_irq(private_client->irq); 
	ret = i2c_update_firmware(buf, count);
	if(ret != 0){
		memset(t1386_config_data, 0, sizeof(t1386_config_data));
	}
	enable_irq(private_client->irq);
	return 1;
}
static ssize_t tp_control_show(struct kobject *kobj, 
			struct kobj_attribute *attr,char *buf)
{
    return 0;
}

static ssize_t tp_control_store(struct kobject *kobj, 
			struct kobj_attribute *attr, const char *buf, size_t count)
{
	return 0;
}
  
static ssize_t firmware_tptype_show(struct kobject *kobj, 
			struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%s", TPTYPE_MXT1386);
}
static ssize_t firmware_tptype_store(struct kobject *kobj, 
			struct kobj_attribute *attr, const char *buf, size_t count)
{
	return 0;
}

static ssize_t ts_debug_show(struct kobject *kobj, 
		struct kobj_attribute *attr, char *buf)
{
	return 0;
}

static int x_report = 0, y_report = 0;
static ssize_t ts_debug_store(struct kobject *kobj, 
			struct kobj_attribute *attr, const char *buf, size_t count)
{
	uint8_t value = 0;
	int ret = -1;
	struct atmel_i2c_platform_data *pdata = private_ts->client->dev.platform_data;
	
	if(buf[0] == FLAG_RR){
		if (i2c_atmel_read(private_client, 
			get_object_address(private_ts, buf[1]) + buf[2], &value, 1) < 0) {
			return 0;
		} else {
			return value;
		}
	}else if(buf[0] == FLAG_WR){
		if (i2c_atmel_write_byte_data(private_client,
			get_object_address(private_ts, buf[1]) + buf[2], buf[3]) < 0) {
			return 0;
		} else {
			return 1;
		}
	}else if(buf[0] == FLAG_RPX){
		return x_report;
	}else if(buf[0] == FLAG_RPY){
		return y_report;
	}else if(buf[0] == FLAG_RNIT){
		return pdata->tp_gpio_get_value();
	}else if(buf[0] == FLAG_HRST){
		return hw_reset_chip();
	}else  if(buf[0] == FLAG_SRST){
		return sw_reset_chip();
	}else if(buf[0] == FLAG_EINT){
		ret = request_irq(private_client->irq, atmel_ts_irq_handler, 
				((!!ATMEL_BUG) ? IRQF_TRIGGER_HIGH : IRQF_TRIGGER_LOW),
	            private_client->name, private_ts);
	    if (ret) {
			dev_err(&private_client->dev, "k3ts, %s: request_irq failed\n", __func__);
			return 0;
		}
		config_enable_mxt1386();
		return 1;
	}else if(buf[0] == FLAG_DINT){
		free_irq(private_client->irq, private_ts);
		return 1;
	}else if(buf[0] == FLAG_PRST){
		if (pdata->set_power(1, "atmel") < 0) {
			return 0;
		} else {
			return 1;
		}
	}else if(buf[0] == FLAG_POFF){
		if (pdata->set_power(0, "atmel") < 0) {
			return 0;
		} else {
			return 1;
		}
	}else if(buf[0] == FLAG_BR){
		backup_to_nv();
		return 1;
	}else if(buf[0] == FLAG_ROA){
		if(0 == get_object_address(private_ts, buf[1]))
			return 0;
	    else 
			return 1;
	}else if(buf[0] == FLAG_ROS){
		return get_object_size(private_ts, buf[1]);
	}else {
		return 0;
	}
}
static int set_config_data(u8 *buf, uint16_t length)
{
	int k, i = 0, j = 0, line_num = 1, data_num = 0;

  	memset(t1386_config_data, 0, sizeof(t1386_config_data));
  	do {
	  	/* if a line is begin with annotation '#',ignore it */
		while (is_valid_annotation(buf[i])) {
			do {
				i++;
			} while ((!(buf[i]==0x0d && buf[i+1] ==0x0a)) && (i<length));
			if (i < length) {
				i += 2;
				line_num++;
			}
		}
		j = i;
		k = 0;
      	/* if a line is begin with non annotation '#',process it */
		while ((!(buf[i]==0x0d &&buf[i+1] ==0x0a)) && (i<length)) {
			/* a valid  space char in a line,ignore it */
			while((is_valid_data_space(buf[i])) && (i<length)){
				i++;
			}

			/* valid data char in a line,process it */		   
			if((i >= length) || ((buf[i] == 0x0d) && (buf[i+1] = 0x0a))){
				break;
			} else if (is_valid_alphanumeric(buf[i]) 
					&& is_valid_alphanumeric(buf[i+1])) {
				if(((i+1 < length-1) && is_valid_alphanumeric(buf[i]) 
					&& is_valid_alphanumeric(buf[i+1]) 
					&& (is_valid_space(buf[i+2]) 
					||  ((buf[i+2] == 0x0d)&&(buf[i+3] == 0x0a))))
					|| ((i+1 == length-1) && is_valid_alphanumeric(buf[i]) 
					&& is_valid_alphanumeric(buf[i+1]))) {
					t1386_config_data[data_num][k++] = (char_to_numeric(buf[i])<<4) 
							+  char_to_numeric(buf[i+1]);
					i += 2;
				} else {
					printk("mxt1386 firmware config data [line_num=%d,colum_num=%d] "
							"format erro: buf[%d] = %x, %x, %x\n",
							line_num, i-j+1, i, buf[i], buf[i+1], buf[i+2]);
					return -1;	
				}	      
			}else{
				/* invalid data char in a line,exit directly */
				printk("mxt1386 firmware config data [line_num=%d,colum_num=%d] "
						"format erro: buf[%d] = %x, %x, %x\n",
						line_num, i-j+1, i, buf[i], buf[i+1], buf[i+2]);
				return -1;	
			}
		}
		
		if (i < length) {
			i += 2;
			line_num++;
		}
	
		if (k > 0) {
			if (t1386_config_data[data_num][1] != k-2) {
				printk("object[%d] config data num erro:require_num=%d but actual_num=%d!\n",
					t1386_config_data[data_num][0], t1386_config_data[data_num][1], k-2);
				return -1;	
			}
			data_num++;
		}
	
	}while(i < length); 

	return 0;
}
static int t1386_power_config(struct atmel_ts_data *mxt, u8 *data)
{
	struct i2c_client *client = mxt->client;

	u16 obj_addr, obj_size;
	int error;

	obj_addr = get_object_address(mxt, GEN_POWERCONFIG_T7);
	obj_size = get_object_size(mxt, GEN_POWERCONFIG_T7);

	/* power_config initial values are got from platform_data */
	error = i2c_atmel_write(client, obj_addr, data, obj_size);
	if (error < 0) {
		dev_err(&client->dev, "mxt_write_byte failed!\n");
		return -EIO;
	}
	return 0;
}

static int t1386_acquisition_config(struct atmel_ts_data *mxt, u8 *data)
{
	struct i2c_client *client = mxt->client;
	u16 obj_addr, obj_size;
	int error;

	obj_addr = get_object_address(mxt, GEN_ACQUISITIONCONFIG_T8);
	obj_size = get_object_size(mxt, GEN_ACQUISITIONCONFIG_T8);
	
	/* acquisition_config initial values are got from platform_data */
	error = i2c_atmel_write(client, obj_addr, data, obj_size);
	if (error < 0) {
		dev_err(&client->dev, "mxt_write_byte failed!\n");
		return -EIO;
	}
	return 0;
}

static int t1386_multitouch_config(struct atmel_ts_data *mxt, u8 *data)
{
	struct i2c_client *client = mxt->client;
	u16 obj_addr, obj_size;
	int error;

	obj_addr = get_object_address(mxt, TOUCH_MULTITOUCHSCREEN_T9);
	obj_size = get_object_size(mxt, TOUCH_MULTITOUCHSCREEN_T9);
	
	/* multitouch_config initial values are got from platform_data */
	error = i2c_atmel_write(client, obj_addr, data, obj_size);
	if (error < 0) {
		dev_err(&client->dev, "mxt_write_byte failed!\n");
		return -EIO;
	}
	return 0;
}

static int t1386_keyarray_config(struct atmel_ts_data *mxt, u8 *data)
{
	struct i2c_client *client = mxt->client;
	u16 obj_addr, obj_size;
	int error;

	obj_addr = get_object_address(mxt, TOUCH_KEYARRAY_T15);
	obj_size = get_object_size(mxt, TOUCH_KEYARRAY_T15);
	
	/* keyarray_config initial values are got from platform_data */
	error = i2c_atmel_write(client, obj_addr, data, obj_size);
	if (error < 0) {
		dev_err(&client->dev, "mxt_write_byte failed!\n");
		return -EIO;
	}
	return 0;
}

static int t1386_com_config(struct atmel_ts_data *mxt, u8 *data)
{
	struct i2c_client *client = mxt->client;
	u16 obj_addr, obj_size;
	int error;

	obj_addr = get_object_address(mxt, SPT_COMCONFIG_T18);
	obj_size = get_object_size(mxt, SPT_COMCONFIG_T18);
	
	/* com_config initial values are got from platform_data */
	error = i2c_atmel_write(client, obj_addr, data, obj_size);
	if (error < 0) {
		dev_err(&client->dev, "mxt_write_byte failed!\n");
		return -EIO;
	}
	return 0;
}

static int t1386_noise_suppression_config(struct atmel_ts_data *mxt, 
	u8 *data)
{
	struct i2c_client *client = mxt->client;
	u16 obj_addr, obj_size;
	int error;

	obj_addr = get_object_address(mxt, PROCG_NOISESUPPRESSION_T22);
	obj_size = get_object_size(mxt, PROCG_NOISESUPPRESSION_T22);
	
	/* noise_suppression_config initial values are got from platform_data */
 	error = i2c_atmel_write(client, obj_addr, data, obj_size);
	if (error < 0) {
		dev_err(&client->dev, "mxt_write_byte failed!\n");
		return -EIO;
	}
	return 0;
}

static int t1386_onetouch_gesture_config(struct atmel_ts_data *mxt, u8 *data)
{
	struct i2c_client *client = mxt->client;
	u16 obj_addr, obj_size;
	int error;

	obj_addr = get_object_address(mxt, PROCI_ONETOUCHGESTUREPROCESSOR_T24);
	obj_size = get_object_size(mxt, PROCI_ONETOUCHGESTUREPROCESSOR_T24);
	
	/* touchscreen_config initial values are got from platform_data */
	error = i2c_atmel_write(client, obj_addr, data, obj_size);
	if (error < 0) {
		dev_err(&client->dev, "mxt_write_byte failed!\n");
		return -EIO;
	}
	return 0;
}

static int t1386_selftest_config(struct atmel_ts_data *mxt, u8 *data)
{
	struct i2c_client *client = mxt->client;
	u16 obj_addr, obj_size;
	int error;

	obj_addr = get_object_address(mxt, SPT_SELFTEST_T25);
	obj_size = get_object_size(mxt, SPT_SELFTEST_T25);
	
	/* selftest_config initial values are got from platform_data */
	error = i2c_atmel_write(client, obj_addr, data, obj_size);
	if (error < 0) {
		dev_err(&client->dev, "mxt_write_byte failed!\n");
		return -EIO;
	}
	return 0;
}

static int t1386_twotouch_gesture_config(struct atmel_ts_data *mxt, u8 *data)
{
	struct i2c_client *client = mxt->client;
	u16 obj_addr, obj_size;
	int error;

	obj_addr = get_object_address(mxt, PROCI_TWOTOUCHGESTUREPROCESSOR_T27);
	obj_size = get_object_size(mxt, PROCI_TWOTOUCHGESTUREPROCESSOR_T27);
	
	/* twotouch_config initial values are got from platform_data */
	error = i2c_atmel_write(client, obj_addr, data, obj_size);
	if (error < 0) {
		dev_err(&client->dev, "mxt_write_byte failed!\n");
		return -EIO;
	}
	return 0;
}

static int t1386_cte_config(struct atmel_ts_data *mxt, u8 *data)
{
	struct i2c_client *client = mxt->client;
	u16 obj_addr, obj_size;
	int error;

	obj_addr = get_object_address(mxt, SPT_CTECONFIG_T28);
	obj_size = get_object_size(mxt, SPT_CTECONFIG_T28);
	
	/* cte_config initial values are got from platform_data */
 	error = i2c_atmel_write(client, obj_addr, data, obj_size);
	if (error < 0) {
		dev_err(&client->dev, "mxt_write_byte failed!\n");
		return -EIO;
	}

	return 0;
}

static int t1386_userdata_config(struct atmel_ts_data *mxt, u8 *data)
{
	struct i2c_client *client = mxt->client;
	u16 obj_addr, obj_size;
	int error;

	obj_addr = get_object_address(mxt, USERDATA_T38);
	obj_size = get_object_size(mxt, USERDATA_T38);
	
	/* userdata_config initial values are got from platform_data */
 	error = i2c_atmel_write(client, obj_addr, data, obj_size);
	if (error < 0) {
		dev_err(&client->dev, "mxt_write_byte failed!\n");
		return -EIO;
	}

	return 0;
}

static int t1386_gripsuppression_config(struct atmel_ts_data *mxt,
	u8 *data)
{
	struct i2c_client *client = mxt->client;
	u16 obj_addr, obj_size;
	int error;
	
	obj_addr = get_object_address(mxt, PROCI_GRIPSUPPRESSION_T40);
	obj_size = get_object_size(mxt, PROCI_GRIPSUPPRESSION_T40);
	
	/* gripsupression_config initial values are got from platform_data */
	error = i2c_atmel_write(client, obj_addr, data, obj_size);
	if (error < 0) {
		dev_err(&client->dev, "mxt_write_byte failed!\n");
		return -EIO;
	}

	return 0;
}

static int t1386_update_firmware(struct atmel_ts_data *mxt)
{
	int i;
	
	for (i = 0; i < MXT1386_MAX_OBJECT_NUM; i++) {
		if (GEN_POWERCONFIG_T7 == t1386_config_data[i][0]) {
			if (t1386_power_config(mxt, t1386_config_data[i] + 2) < 0) {
				printk(KERN_ERR "%s: power config fail\n", __func__);
				return -1;
			}
		}

		if (GEN_ACQUISITIONCONFIG_T8 == t1386_config_data[i][0]) {
			if (t1386_acquisition_config(mxt, t1386_config_data[i] + 2) < 0) {
				printk(KERN_ERR "%s: acquisition config fail\n", __func__);
				return -1;
			}
		}

		if (TOUCH_MULTITOUCHSCREEN_T9 == t1386_config_data[i][0]) {
			if (t1386_multitouch_config(mxt, t1386_config_data[i] + 2) < 0) {
				printk(KERN_ERR "%s: multitouch config fail\n", __func__);
				return -1;
			}
		}

		if (TOUCH_KEYARRAY_T15 == t1386_config_data[i][0]) {
			if (t1386_keyarray_config(mxt, t1386_config_data[i] + 2) < 0) {
				printk(KERN_ERR "%s: keyarray config fail\n", __func__);
				return -1;
			}
		}

		if (SPT_COMCONFIG_T18 == t1386_config_data[i][0]) {
			if (t1386_com_config(mxt, t1386_config_data[i] + 2) < 0) {
				printk(KERN_ERR "%s: com config fail\n", __func__);
				return -1;
			}
		}

		if (PROCG_NOISESUPPRESSION_T22 == t1386_config_data[i][0]) {
			if (t1386_noise_suppression_config(mxt, t1386_config_data[i] + 2) < 0) {
				printk(KERN_ERR "%s: noise_suppression config fail\n", __func__);
				return -1;
			}
		}
		
		if (PROCI_ONETOUCHGESTUREPROCESSOR_T24 == t1386_config_data[i][0]) {
			if (t1386_onetouch_gesture_config(mxt, t1386_config_data[i] + 2) < 0) {
				printk(KERN_ERR "%s: onetouch_gesture config fail\n", __func__);
				return -1;
			}
		}

		if (SPT_SELFTEST_T25 == t1386_config_data[i][0]) {
			if (t1386_selftest_config(mxt, t1386_config_data[i] + 2) < 0) {
				printk(KERN_ERR "%s: selftest config fail\n", __func__);
				return -1;
			}
		}

		if (PROCI_TWOTOUCHGESTUREPROCESSOR_T27 == t1386_config_data[i][0]) {
			if (t1386_twotouch_gesture_config(mxt, t1386_config_data[i] + 2) < 0) {
				printk(KERN_ERR "%s: twotouch_gesture config fail\n", __func__);
				return -1;
			}
		}
		
		if (SPT_CTECONFIG_T28 == t1386_config_data[i][0]) {
			if (t1386_cte_config(mxt, t1386_config_data[i] + 2) < 0) {
				printk(KERN_ERR "%s: cte config fail\n", __func__);
				return -1;
			}
		}

		if (USERDATA_T38 == t1386_config_data[i][0]) {
			if (t1386_userdata_config(mxt, t1386_config_data[i] + 2) < 0) {
				printk(KERN_ERR "%s: userdata config fail\n", __func__);
				return -1;
			}
		}

		if (PROCI_GRIPSUPPRESSION_T40 == t1386_config_data[i][0]) {
			if (t1386_gripsuppression_config(mxt, t1386_config_data[i] + 2) < 0) {
				printk(KERN_ERR "%s: gripsuppression config fail\n", __func__);
				return -1;
			}
		}
	}

	/* backup to nv memory */
	backup_to_nv();
	
	/* forces a reset of the chipset */
	sw_reset_chip();
	/* mxt1386 need 250ms */
	msleep(250);

	return 0;
}
static int i2c_update_firmware(const char * file, size_t count) 
{
	u8 *buf;
	struct file *filp;
	struct inode *inode = NULL;
	mm_segment_t oldfs;
	uint16_t length;
	int ret = 0;
	unsigned char path_image[255];

     if(count > 255 || count == 0 || strnchr(file, count, 0x20))
     	return -1;     
     
     memcpy (path_image, file,  count);
     /* replace '\n' with  '\0'  */ 
     if((path_image[count-1]) == '\n')
     	path_image[count-1] = '\0'; 
     else
 		path_image[count] = '\0';
	
	
	/* open file */
	oldfs = get_fs();
	set_fs(KERNEL_DS);
	filp = filp_open(path_image, O_RDONLY, S_IRUSR);
	if (IS_ERR(filp)) {
		printk("%s: file %s filp_open error\n", __func__,path_image);
		set_fs(oldfs);
		return -1;
	}
	
	if (!filp->f_op) {
		printk("%s: File Operation Method Error\n", __func__);
		filp_close(filp, NULL);
		set_fs(oldfs);
		return -1;
	}
	
	inode = filp->f_path.dentry->d_inode;
	if (!inode) {
		printk("%s: Get inode from filp failed\n", __func__);
		filp_close(filp, NULL);
		set_fs(oldfs);
		return -1;
	}
	
	/* file's size */
	length = i_size_read(inode->i_mapping->host);
	printk("%s: mxt1386 image file is %s and data size is %d Bytes\n",
			__func__,path_image,length);
	if (!( length > 0 && length < 64*1024 )){
		printk("%s: file size error\n", __func__);
		filp_close(filp, NULL);
		set_fs(oldfs);
		return -1;
	}
	
	/* allocation buff size */
	buf = vmalloc(length+(length%2));	/* buf size if even */
	if (!buf) {
		printk("%s: alloctation memory failed\n", __func__);
		filp_close(filp, NULL);
		set_fs(oldfs);
		return -1;
	}
	
	/* read data */
	if (filp->f_op->read(filp, (char *)buf, length, &filp->f_pos) != length) {
		printk("%s: file read error\n", __func__);
		filp_close(filp, NULL);
		set_fs(oldfs);
		vfree(buf);
		return -1;
	}

	ret = set_config_data(buf, length);
	
	filp_close(filp, NULL);
	set_fs(oldfs);
	vfree(buf);
	return ret;
}

static struct kobject *android_touch_kobj = NULL;

static int atmel_touch_sysfs_init(void)
{
	int ret = 0;
	
	android_touch_kobj = kobject_create_and_add("touchscreen", NULL);
	if (android_touch_kobj == NULL) {
		pr_err("k3ts, %s: subsystem_register failed\n", __func__);
		ret = -ENOMEM;
		return ret;
	}
	
	ret = sysfs_create_file(android_touch_kobj, &dev_attr_gpio.attr);
	if (ret) {
		pr_err("k3ts, gpio %s: sysfs_create_file failed\n", __func__);
		goto sysfs_create_bin_file_fail1;
	}

	ret = sysfs_create_file(android_touch_kobj, &dev_attr_diag.attr);
	if (ret) {
		pr_err("k3ts, diag %s: sysfs_create_file failed\n", __func__);
		goto sysfs_create_bin_file_fail2;
	}  

	ret = sysfs_create_file(android_touch_kobj, &update_firmware_attr.attr);
	if (ret) {
		pr_err("k3ts, update_firmware %s: sysfs_create_file failed\n", __func__);
		goto sysfs_create_bin_file_fail3;
	} 

	ret = sysfs_create_file(android_touch_kobj, &firmware_version_attr.attr);
	if (ret) {
		pr_err("k3ts, firmware_version %s: sysfs_create_file failed\n", __func__);
		goto sysfs_create_bin_file_fail4;
	} 
	
	ret = sysfs_create_file(android_touch_kobj, &firmware_file_version_attr.attr);
	if (ret) {
		pr_err("k3ts, firmware_file_version %s: sysfs_create_file failed\n", __func__);
		goto sysfs_create_bin_file_fail5;
	} 
	
	ret = sysfs_create_file(android_touch_kobj, &tp_control_attr.attr);
	if (ret) {
		pr_err("k3ts, tp_control %s: sysfs_create_file failed\n", __func__);
		goto sysfs_create_bin_file_fail6;
	}
	
	ret = sysfs_create_file(android_touch_kobj, &firmware_tptype_attr.attr);
	if (ret) {
		pr_err("k3ts, firmware_tptype %s: sysfs_create_file failed\n", __func__);
		goto sysfs_create_bin_file_fail7;
	}
	
	ret = sysfs_create_file(android_touch_kobj, &ts_debug_attr.attr);
	if (ret) {
		pr_err("k3ts, ts_debug %s: sysfs_create_file failed\n", __func__);
		goto sysfs_create_bin_file_fail8;
	}

	return 0;

sysfs_create_bin_file_fail8:
	sysfs_remove_file(android_touch_kobj, &firmware_tptype_attr.attr);
sysfs_create_bin_file_fail7:
	sysfs_remove_file(android_touch_kobj, &tp_control_attr.attr);
sysfs_create_bin_file_fail6:
	sysfs_remove_file(android_touch_kobj, &firmware_file_version_attr.attr);
sysfs_create_bin_file_fail5:
	sysfs_remove_file(android_touch_kobj, &firmware_version_attr.attr);
sysfs_create_bin_file_fail4:
	sysfs_remove_file(android_touch_kobj, &update_firmware_attr.attr);
sysfs_create_bin_file_fail3:
	sysfs_remove_file(android_touch_kobj, &dev_attr_diag.attr);
sysfs_create_bin_file_fail2:
	sysfs_remove_file(android_touch_kobj, &dev_attr_gpio.attr);
sysfs_create_bin_file_fail1:	
	kobject_del(android_touch_kobj);
	return ret;
}

static void atmel_touch_sysfs_deinit(void)
{
	sysfs_remove_file(android_touch_kobj, &dev_attr_gpio.attr);
    sysfs_remove_file(android_touch_kobj, &dev_attr_diag.attr);
	sysfs_remove_file(android_touch_kobj, &update_firmware_attr.attr);
	sysfs_remove_file(android_touch_kobj, &firmware_version_attr.attr);
	sysfs_remove_file(android_touch_kobj, &firmware_file_version_attr.attr);
    sysfs_remove_file(android_touch_kobj, &tp_control_attr.attr);
	sysfs_remove_file(android_touch_kobj, &firmware_tptype_attr.attr);
	sysfs_remove_file(android_touch_kobj, &ts_debug_attr.attr);
	kobject_del(android_touch_kobj);
}
#endif
static void msg_process_multitouch(struct atmel_ts_data *ts, uint8_t *data)
{
	u8 status = data[T9_MSG_STATUS];
    static int prev_id = -1;
    int i = 0;
    int idx = 0;
	u16 xpos = 0xFFFF;
	u16 ypos = 0xFFFF;
	u8  touch_size = 255;
	int bChangeUpDn = 0;

	if (ts == NULL) {
		printk(KERN_ERR "[%s %d] ts is NULL", __func__, __LINE__);
        return ;
    }    

	if (status & T9_MSG_STATUS_SUPPRESS) {
		/* Touch has been suppressed by grip/face detection */
		input_mt_sync(ts->input_dev);
		input_sync(ts->input_dev);
	} else {
		xpos = ((data[T9_MSG_XPOSMSB] ) << 4) + (data[T9_MSG_XYPOSLSB] >> 4);
		ypos = ((data[T9_MSG_YPOSMSB] )<< 4) + (data[T9_MSG_XYPOSLSB] & 0x0F);
		idx = data[MSG_RID] - ts->finger_type;
		if (idx < 0 || (idx >= 10)) {
			printk(KERN_ERR "[%s %d] finger index error", __func__, __LINE__);
	        return ;
		}

		if (status & T9_MSG_STATUS_DETECT) {
			/*
			 * TODO: more precise touch size calculation?
			 * mXT1386 reports the number of touched nodes,
			 * so the exact value for touch ellipse major
			 * axis length would be 2*sqrt(touch_size/pi)
			 * (assuming round touch shape).
			 */
			touch_size = data[T9_MSG_TCHAREA];
			touch_size = touch_size >> 1;
			if (!touch_size) {
				touch_size = 1;
			}
            
            if (status & T9_MSG_STATUS_PRESS) {
            	bChangeUpDn= 1;
            }
        
            if ((status & T9_MSG_STATUS_PRESS)
			    || (status & T9_MSG_STATUS_MOVE)
			    || (status & T9_MSG_STATUS_AMP)){	
	            ts->finger_data[idx].x = xpos;
				ts->finger_data[idx].y = ypos;
				ts->finger_data[idx].w = touch_size;
	            ts->finger_data[idx].z = data[T9_MSG_TCHAMPLITUDE];    
			}
		}else if (status & T9_MSG_STATUS_RELEASE) {
			/* The previously reported touch has been removed.*/
			bChangeUpDn = 1;
			ts->finger_data[idx].w = 0;
		#ifdef ATMEL_EN_SYSFS 
			x_report = 0;
			y_report = 0;
		#endif
		}
	}

	if ((prev_id >= idx) || bChangeUpDn) {
		for (i = 0; i < MAX_NUM; i++) {
	        if(ts->finger_data[i].w == -1){
	          	continue;
	        }

			if(ts->finger_data[i].w == 0){
				input_mt_sync(ts->input_dev);
	            ts->finger_data[i].w= -1;
				continue;
	        }

		#ifdef ATMEL_EN_SYSFS 
			x_report = ts->finger_data[i].x;
			y_report = ts->finger_data[i].y;
		#endif
			input_report_abs(ts->input_dev, ABS_MT_POSITION_X, ts->finger_data[i].x);
			input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, ts->finger_data[i].y);
			input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, ts->finger_data[i].w);
			input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR,  ts->finger_data[i].w);
			input_mt_sync(ts->input_dev);
		}

		input_sync(ts->input_dev);
	}
	prev_id = idx;
}

static int process_message(struct atmel_ts_data *ts, uint8_t *message, uint8_t object)
{
	struct i2c_client *client = ts->client;
	int status = message[1];

	switch (object) {
		case GEN_COMMANDPROCESSOR_T6:
            if (status & T6_MSG_STATUS_COMSERR) {
                dev_err(&client->dev,
                    "maXTouch checksum error\n");
            }

            if (status & T6_MSG_STATUS_CFGERR) {
                dev_err(&client->dev,
                    "maXTouch configuration error\n");
				hw_reset_chip();
            }

            if (status & T6_MSG_STATUS_CAL) {
                dev_info(&client->dev,
                    "maXTouch calibration in progress\n");
				memset(ts->finger_data, 0, sizeof(ts->finger_data));
                input_mt_sync(ts->input_dev);
                input_sync(ts->input_dev);
            }

            if (status & T6_MSG_STATUS_SIGERR) {
                dev_err(&client->dev,
                    "maXTouch acquisition error\n");
                hw_reset_chip();
            }
            
            if (status & T6_MSG_STATUS_OFL) {
                dev_err(&client->dev,
                    "maXTouch cycle overflow\n");
				sw_reset_chip();
            }
            
            if (status & T6_MSG_STATUS_RESET) {
                dev_info(&client->dev,
                    "maXTouch chip reset\n");
            }
			break;

        case TOUCH_MULTITOUCHSCREEN_T9:
            msg_process_multitouch(ts, message);
            break;

        case PROCG_NOISESUPPRESSION_T22:
            if (status & T22_MSG_STATUS_FHCHG) {
                dev_info(&client->dev,
                    "maXTouch: Freq changed\n");
            }
            
            if (status & T22_MSG_STATUS_GCAFERR) {
                dev_info(&client->dev,
                    "maXTouch: High noise level\n");
            }
            
            if (status & T22_MSG_STATUS_FHERR) {
                    dev_info(&client->dev,
                    "maXTouch: Freq changed - "
                    "Noise level too high\n");
            }
            break;
    }    

	return 0;
}

static void atmel_ts_work_func(struct work_struct *work)
{
	int ret = 0;
	struct atmel_ts_data *ts = container_of(work, struct atmel_ts_data, work);
	uint8_t data[ID_BLOCK_SIZE] = {0};
	int object = 0;
    
	ret = i2c_atmel_read(ts->client, get_object_address(ts,
		GEN_MESSAGEPROCESSOR_T5), data, ID_BLOCK_SIZE);

    object = report_id_to_object(ts, data[MSG_RID]);
    process_message(ts, data, object);
	
	enable_irq(ts->client->irq);
}

//close the anti-touch
static void atmel_calibrate_worker(struct work_struct *work)
{
	int ret = 0;
    uint8_t anti_config[] = {255, 1, 0, 0};
    printk(KERN_ERR "[%s %d] close the anti-touch", __func__, __LINE__);
	ret = i2c_atmel_write(private_ts->client,
		get_object_address(private_ts, GEN_ACQUISITIONCONFIG_T8) + 6,
		anti_config,
		sizeof(anti_config));
    if (ret < 0) {
		printk(KERN_ERR "[%s %d] write anti_touch config failed", __func__, __LINE__);
    }    
}

static void atmel_config_antitouch(void)
{
	int ret = 0;
	ret = i2c_atmel_write(private_ts->client,
		get_object_address(private_ts, GEN_ACQUISITIONCONFIG_T8) + 6,
		private_ts->ATCH_EXT, 4);
    if (ret < 0) {
		printk(KERN_ERR "[%s %d] write anti_touch config failed", __func__, __LINE__);
    }    
}

static irqreturn_t atmel_ts_irq_handler(int irq, void *dev_id)
{
	struct atmel_ts_data *ts = dev_id;

	disable_irq_nosync(ts->client->irq);
	queue_work(ts->atmel_wq, &ts->work);
	return IRQ_HANDLED;
}

static int read_object_table(struct atmel_ts_data *ts)
{
	uint8_t i = 0, type_count = 0;
	uint8_t data[OBJECT_TABLE_ELEMENT_SIZE] = {0};
    int ret = 0;
    int report_id = 1;
    int object_report_id = 0;
    int object_instance = 0;
	
	memset(data, 0x0, sizeof(data));
	ts->object_table = kzalloc(sizeof(struct object_t)*ts->id->num_declared_objects, GFP_KERNEL);
	if (ts->object_table == NULL) {
		dev_err(&ts->client->dev, "k3ts, %s: allocate object_table failed\n", __func__);
		return -ENOMEM;
	}

	type_count = 0;
	for (i = 0; i < ts->id->num_declared_objects; i++) {
		i2c_atmel_read(ts->client, i * OBJECT_TABLE_ELEMENT_SIZE + ADDR_OBJECT_TABLE, data, OBJECT_TABLE_ELEMENT_SIZE);
		ts->object_table[i].object_type = data[OBJ_TABLE_TYPE];
		ts->object_table[i].i2c_address =
			data[OBJ_TABLE_LSB] | data[OBJ_TABLE_MSB] << 8;
		ts->object_table[i].size = data[OBJ_TABLE_SIZE] + 1;
		ts->object_table[i].instances = data[OBJ_TABLE_INSTANCES] + 1;
		ts->object_table[i].num_report_ids = data[OBJ_TABLE_RIDS];
		if (data[OBJ_TABLE_RIDS]) {
			ts->object_table[i].report_ids = type_count + 1;
			type_count += data[OBJ_TABLE_RIDS] * ts->object_table[i].instances;
		}
		if (data[OBJ_TABLE_TYPE] == TOUCH_MULTITOUCHSCREEN_T9) {
			ts->finger_type = ts->object_table[i].report_ids;
        }
	}

	ts->report_id_map = kzalloc(sizeof(struct report_id_map) * type_count + 1, GFP_KERNEL);
    if (ts->report_id_map == NULL) {
		printk(KERN_ERR "cannot allocate memory for report_id_map");
        ret = -ENOMEM;
        goto err_msg_alloc;
    }    

    report_id = 1;
	for (i = 0; i < ts->id->num_declared_objects; i++) {
        for (object_instance = 0; object_instance < ts->object_table[i].instances; 
        	object_instance++) {
			for (object_report_id = 0; object_report_id < ts->object_table[i].num_report_ids; 
	        	 object_report_id++) {
	            ts->report_id_map[report_id].object = ts->object_table[i].object_type;
	            ts->report_id_map[report_id].instance = ts->object_table[i].instances;
	            report_id++;
	        }
        }
    }    

	return 0;

err_msg_alloc:
    kfree(ts->report_id_map);
    return ret;
}

static int write_object_config(struct atmel_ts_data *ts, struct atmel_i2c_platform_data *pdata)
{
	int ret = 0;
    
	ret = i2c_atmel_write(ts->client,
		get_object_address(ts, GEN_POWERCONFIG_T7),
		pdata->config_T7,
		get_object_size(ts, GEN_POWERCONFIG_T7));
	if (ret < 0) {
		printk(KERN_ERR "write T7 config failed");
    }    
    
	ret = i2c_atmel_write(ts->client,
		get_object_address(ts, GEN_ACQUISITIONCONFIG_T8),
		pdata->config_T8,
		get_object_size(ts, GEN_ACQUISITIONCONFIG_T8));    
	if (ret < 0) {
		printk(KERN_ERR "write T8 config failed");
    }    

	ret = i2c_atmel_write(ts->client,
		get_object_address(ts, TOUCH_MULTITOUCHSCREEN_T9),
		pdata->config_T9,
		get_object_size(ts, TOUCH_MULTITOUCHSCREEN_T9));
	if (ret < 0) {
		printk(KERN_ERR "write T9 config failed");
    }    
    
	ret = i2c_atmel_write(ts->client,
		get_object_address(ts, TOUCH_KEYARRAY_T15),
		pdata->config_T15,
		get_object_size(ts, TOUCH_KEYARRAY_T15));
	if (ret < 0) {
		printk(KERN_ERR "write T15 config failed");
    }    

    ret = i2c_atmel_write(ts->client,
        get_object_address(ts, SPT_COMCONFIG_T18),
        pdata->config_T18,
        get_object_size(ts, SPT_COMCONFIG_T18));
	if (ret < 0) {
		printk(KERN_ERR "write T18 config failed");
    }    

	ret = i2c_atmel_write(ts->client,
		get_object_address(ts, PROCG_NOISESUPPRESSION_T22),
		pdata->config_T22,
		get_object_size(ts, PROCG_NOISESUPPRESSION_T22));
	if (ret < 0) {
		printk(KERN_ERR "write T22 config failed");
    }    

	ret = i2c_atmel_write(ts->client,
		get_object_address(ts, PROCI_ONETOUCHGESTUREPROCESSOR_T24),
		pdata->config_T24,
		get_object_size(ts, PROCI_ONETOUCHGESTUREPROCESSOR_T24));
	if (ret < 0) {
		printk(KERN_ERR "write T24 config failed");
    }    

	ret = i2c_atmel_write(ts->client,
		get_object_address(ts, SPT_SELFTEST_T25),
		pdata->config_T25,
		get_object_size(ts, SPT_SELFTEST_T25));
	if (ret < 0) {
		printk(KERN_ERR "write T25 config failed");
    }    

	ret = i2c_atmel_write(ts->client,
		get_object_address(ts, PROCI_TWOTOUCHGESTUREPROCESSOR_T27),
		pdata->config_T27,
		get_object_size(ts, PROCI_TWOTOUCHGESTUREPROCESSOR_T27));
	if (ret < 0) {
		printk(KERN_ERR "write T27 config failed");
    }    

	ret = i2c_atmel_write(ts->client,
		get_object_address(ts, SPT_CTECONFIG_T28),
		pdata->config_T28,
		get_object_size(ts, SPT_CTECONFIG_T28));
	if (ret < 0) {
		printk(KERN_ERR "write T28 config failed");
    }    

	ret = i2c_atmel_write(ts->client,
		get_object_address(ts, PROCI_GRIPSUPPRESSION_T40),
		pdata->config_T40,
		get_object_size(ts, PROCI_GRIPSUPPRESSION_T40));
	if (ret < 0) {
		printk(KERN_ERR "write T40 config failed");
    }    

	backup_to_nv();
	return 0;
}

static int __devinit atmel_ts_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	struct atmel_ts_data *ts = NULL;
	struct atmel_i2c_platform_data *pdata = NULL;
	int ret = 0, intr = 0;
	uint8_t loop_i = 0;
	uint8_t data[16] = {0};
    if(1 == tp_probe_status){
	    printk("================================\n");
	    printk("The TP has be probe.Return -1 here.\n");
		printk("%s,tp_probe_status = %d.\n",__func__,tp_probe_status);
		printk("================================\n");
		return -1;
	}
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "k3ts, %s: need I2C_FUNC_I2C\n", __func__);
		ret = -ENODEV;
		goto err_check_functionality_failed;
	}

	ts = kzalloc(sizeof(struct atmel_ts_data), GFP_KERNEL);
	if (ts == NULL) {
		dev_err(&client->dev, "k3ts, %s: allocate atmel_ts_data failed\n", __func__);
		ret = -ENOMEM;
		goto err_alloc_data_failed;
	}

	private_ts = ts;
	private_client = client;

	ts->atmel_wq = create_singlethread_workqueue("atmel_wq");
	if (!ts->atmel_wq) {
		dev_err(&client->dev, "k3ts, %s: create workqueue failed\n", __func__);
		ret = -ENOMEM;
		goto err_cread_wq_failed;
	}

	INIT_WORK(&ts->work, atmel_ts_work_func);
	INIT_DELAYED_WORK(&ts->calibrate_dwork, atmel_calibrate_worker);
	
	ts->client = client;
	i2c_set_clientdata(client, ts);
	pdata = client->dev.platform_data;

	if (!pdata) {
		dev_err(&client->dev, "k3ts, %s: platform data is null\n", __func__);
		ret = -ENOMEM;
		goto err_config_gpio_failed;
	}

	intr = pdata->gpio_irq;
	client->irq = gpio_to_irq(intr);

    if (pdata->gpio_config) {
        ret = pdata->gpio_config(NORMAL);
        if (ret < 0) {
            dev_err(&client->dev, "failed to set gpio mode. error %d\n", ret);
            goto err_config_gpio_failed;
        }
    }

	if (pdata->tp_gpio_request) {
		ret = pdata->tp_gpio_request(1);
        if (ret < 0) {
            dev_err(&client->dev, "failed to request gpio. error %d\n", ret);
            goto err_gpio_request;
        }    
    }    

	if (pdata->set_power) {
        ret = (!!ATMEL_BUG) ? pdata->set_power(1, "atmel") : pdata->set_power(1, "atmel_new");
        if (ret < 0) {
			dev_err(&client->dev, "failed to set power. error %d\n", ret);
            goto err_set_power;
        }    
    }    

	/* read message */
    for (loop_i = 0; loop_i < ATMEL_I2C_RETRY_TIMES; loop_i++) {
		ret = read_info_msg(client, data);
		if (ret >= 0) {
            printk(KERN_ERR "i2c communications is success! i2c addr is 0x%x.", ts->client->addr);
			break;
        }
        
		if ((ret < 0) && (loop_i == (ATMEL_I2C_RETRY_TIMES -1))) {
			dev_err(&client->dev,  "k3ts, %s: No Atmel chip inside\n", __func__);
			goto err_detect_failed;
		}
        
        msleep(30);
    }
	ts->id = kzalloc(sizeof(struct info_id_t), GFP_KERNEL);
	if (ts->id == NULL) {
		dev_err(&client->dev, "k3ts, %s: allocate info_id_t failed\n", __func__);
		goto err_alloc_failed;
	}

	ts->id->family_id = data[INFO_BLK_FID];
	ts->id->variant_id = data[INFO_BLK_VID];
	ts->id->version = data[INFO_BLK_VER];
	ts->id->build = data[INFO_BLK_BUILD];
	ts->id->matrix_x_size = data[INFO_BLK_XSIZE];
	ts->id->matrix_y_size = data[INFO_BLK_YSIZE];
	ts->id->num_declared_objects = data[INFO_BLK_OBJS];

	printk(KERN_INFO
		"k3ts, %s: info block: 0x%2.2X 0x%2.2X 0x%2.2X 0x%2.2X 0x%2.2X 0x%2.2X 0x%2.2X\n", __func__,
		ts->id->family_id, ts->id->variant_id,
		ts->id->version, ts->id->build,
		ts->id->matrix_x_size, ts->id->matrix_y_size,
		ts->id->num_declared_objects);

	/* Read object table. */
	ret = read_object_table(ts);
	if (ret < 0) {
		goto err_read_table_failed;
	}

	/* write config */
    write_object_config(ts, pdata);

	ts->finger_support = pdata->config_T9[T9_CFG_NUMTOUCH];
	ts->abs_x_min = pdata->abs_x_min;
	ts->abs_x_max = pdata->abs_x_max;
	ts->abs_y_min = pdata->abs_y_min;
	ts->abs_y_max = pdata->abs_y_max;
	ts->abs_area_min = pdata->abs_area_min;
	ts->abs_area_max = pdata->abs_area_max;
	ts->abs_pressure_min = pdata->abs_pressure_min;
	ts->abs_pressure_max = pdata->abs_pressure_max;
	ts->GCAF_level = pdata->GCAF_level;
	ts->ATCH_EXT = &pdata->config_T8[6];

	ts->config_setting.config_T7
		= ts->config_setting.config_T7
		= pdata->config_T7;
	ts->config_setting.config_T8
		= ts->config_setting.config_T8
		= pdata->config_T8;
	ts->config_setting.config_T9 = pdata->config_T9;
	ts->config_setting.config_T22 = pdata->config_T22;
	ts->config_setting.config_T28 = pdata->config_T28;  
    
	ts->input_dev = input_allocate_device();
	if (ts->input_dev == NULL) {
		ret = -ENOMEM;
		dev_err(&client->dev, "k3ts, %s: Failed to allocate input device\n", __func__);
		goto err_input_dev_alloc_failed;
	}
    
	ts->input_dev->name = "atmel";
	set_bit(EV_SYN, ts->input_dev->evbit);
	set_bit(EV_ABS, ts->input_dev->evbit);
	set_bit(EV_KEY, ts->input_dev->evbit);
	set_bit(INPUT_PROP_DIRECT, ts->input_dev->propbit);
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_X,
				ts->abs_x_min, ts->abs_x_max, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_Y,
				ts->abs_y_min, ts->abs_y_max, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_TOUCH_MAJOR,
				ts->abs_area_min, ts->abs_area_max, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_WIDTH_MAJOR, 
				ts->abs_area_min, ts->abs_area_max, 0, 0);

	ret = input_register_device(ts->input_dev);
	if (ret) {
		dev_err(&client->dev,
			"k3ts, %s: atmel_ts_probe: Unable to register %s input device\n", __func__,
			ts->input_dev->name);
		goto err_input_register_device_failed;
	}

	ret = request_irq(client->irq, atmel_ts_irq_handler, 
				((!!ATMEL_BUG) ? IRQF_TRIGGER_HIGH : IRQF_TRIGGER_LOW),
	            client->name, ts);
    if (ret) {
		dev_err(&client->dev, "k3ts, %s: request_irq failed\n", __func__);
		goto err_input_register_device_failed;
	}
    
#ifdef CONFIG_HAS_EARLYSUSPEND
	ts->early_suspend.level = EARLY_SUSPEND_LEVEL_STOP_DRAWING - 1;
	ts->early_suspend.suspend = atmel_ts_early_suspend;
	ts->early_suspend.resume = atmel_ts_late_resume;
	register_early_suspend(&ts->early_suspend);
#endif

#ifdef ATMEL_EN_SYSFS
	atmel_touch_sysfs_init();
#endif
	schedule_delayed_work(&ts->calibrate_dwork, msecs_to_jiffies(50000));
	dev_info(&client->dev, "k3ts, %s: probe %s successfully\n", __func__,
			ts->input_dev->name);
    // make sure tp_probe_status =1,then do not probe other tp.
	tp_probe_status = 1;
	printk("================================\n");
	printk("%s,tp_probe_status = %d.\n",__func__,tp_probe_status);
	printk("================================\n");

	return 0;

err_input_register_device_failed:
	input_free_device(ts->input_dev);
err_input_dev_alloc_failed:
err_read_table_failed:
	kfree(ts->id);
err_alloc_failed:
err_detect_failed:
    if (pdata->set_power) {
		if (ATMEL_BUG) {
    	    pdata->set_power(0, "atmel");
        } else {
	        pdata->set_power(0, "atmel_new");
        }
    }    
err_set_power:
    if (pdata->tp_gpio_request) {
        pdata->tp_gpio_request(0);
    }    
err_gpio_request:
    if (pdata->gpio_config) {
        pdata->gpio_config(LOWPOWER);
    }
err_config_gpio_failed:
	destroy_workqueue(ts->atmel_wq);
err_cread_wq_failed:
	kfree(ts);
err_alloc_data_failed:
err_check_functionality_failed:
	return ret;
}

static int atmel_ts_remove(struct i2c_client *client)
{
	struct atmel_ts_data *ts = i2c_get_clientdata(client);

#ifdef ATMEL_EN_SYSFS
	atmel_touch_sysfs_deinit();
#endif

	unregister_early_suspend(&ts->early_suspend);
	free_irq(client->irq, ts);

	destroy_workqueue(ts->atmel_wq);
	input_unregister_device(ts->input_dev);
	kfree(ts);

	return 0;
}

static int atmel_ts_suspend(struct i2c_client *client, pm_message_t mesg)
{
	int ret;
	struct atmel_ts_data *ts = i2c_get_clientdata(client);

	disable_irq(client->irq);

	ret = cancel_work_sync(&ts->work);
	ret = cancel_delayed_work_sync(&ts->calibrate_dwork);

	atmel_config_antitouch();

	i2c_atmel_write_byte_data(client,
		get_object_address(ts, GEN_POWERCONFIG_T7) + T7_CFG_IDLEACQINT, 0x0);
	i2c_atmel_write_byte_data(client,
		get_object_address(ts, GEN_POWERCONFIG_T7) + T7_CFG_ACTVACQINT, 0x0);
    
	return 0;
}

static int atmel_ts_resume(struct i2c_client *client)
{
	struct atmel_ts_data *ts = i2c_get_clientdata(client);

	/* Do a hardware reset */
	hw_reset_chip();
	
	schedule_delayed_work(&ts->calibrate_dwork, msecs_to_jiffies(50000));

	i2c_atmel_write(ts->client,
		get_object_address(ts, GEN_POWERCONFIG_T7),
		ts->config_setting.config_T7,
		get_object_size(ts, GEN_POWERCONFIG_T7));
    
	enable_irq(client->irq);

	return 0;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void atmel_ts_early_suspend(struct early_suspend *h)
{
	struct atmel_ts_data *ts;
	ts = container_of(h, struct atmel_ts_data, early_suspend);
	atmel_ts_suspend(ts->client, PMSG_SUSPEND);
}

static void atmel_ts_late_resume(struct early_suspend *h)
{
	struct atmel_ts_data *ts;
	ts = container_of(h, struct atmel_ts_data, early_suspend);
	atmel_ts_resume(ts->client);
}
#endif
static const struct i2c_device_id atml_ts_i2c_id[] = {
	{ ATMEL_MXT1386_NAME, 0 },
	{ ATMEL_MXT1386_NAME_I2C1, 0 },
	{ }
};

static struct i2c_driver atmel_ts_driver = {
	.id_table = atml_ts_i2c_id,
	.probe = atmel_ts_probe,
	.remove = atmel_ts_remove,
#ifndef CONFIG_HAS_EARLYSUSPEND
	.suspend = atmel_ts_suspend,
	.resume = atmel_ts_resume,
#endif
	.driver = {
			.name = ATMEL_MXT1386_NAME,
	},
};

static int __devinit atmel_ts_init(void)
{
	pr_info("k3ts, %s\n", __func__);
	return i2c_add_driver(&atmel_ts_driver);
}

static void __exit atmel_ts_exit(void)
{
	i2c_del_driver(&atmel_ts_driver);
}

module_init(atmel_ts_init);
module_exit(atmel_ts_exit);

MODULE_DESCRIPTION("ATMEL Touch driver");
MODULE_LICENSE("GPL");
