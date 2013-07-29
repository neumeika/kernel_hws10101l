/*
 * Copyright (c) 2011 Synaptics Incorporated
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */
#include <linux/kernel.h>
#include <linux/mutex.h>
#include <linux/rmi.h>
#include <linux/slab.h>
#include <linux/version.h>
#include "rmi_driver.h"
#include <linux/delay.h>

#define SYNA
#define ENABLE_DELAY
//#define ENABLE_SHOW_FLASH_PROGRESS
#define SAME_PATH

#ifdef CONFIG_UPDATE_RMI4_FIRMWARE   //syna
#include <linux/sysfs.h>
#include <linux/gpio.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/vmalloc.h>
struct rmi_function_container *rmi_fc = NULL;
#define CRC_CHECK

#ifdef CRC_CHECK
#define UPDATE_FAIL    "Failed"
#define UPDATE_SUCCESS "Successful"
#define FAIL   -1
#define SUCCESS 1
static int update_flag = 0;
#define DELAY_COUNT 50
#endif
#endif

/* define fn $34 commands */
#define WRITE_FW_BLOCK            0x2
#define ERASE_ALL                 0x3
#define READ_CONFIG_BLOCK         0x5
#define WRITE_CONFIG_BLOCK        0x6
#define ERASE_CONFIG              0x7
#define ENABLE_FLASH_PROG         0xf

#define STATUS_IN_PROGRESS        0xff
#define STATUS_IDLE		  0x80

#define PDT_START_SCAN_LOCATION	0x00e9
#define PDT_END_SCAN_LOCATION	0x0005

#define BLK_SZ_OFF	3
#define IMG_BLK_CNT_OFF	5
#define CFG_BLK_CNT_OFF	7

#define BLK_NUM_OFF 2


#if LINUX_VERSION_CODE > KERNEL_VERSION(2, 6, 32)
#define KERNEL_VERSION_ABOVE_2_6_32 1
#endif

/* data specific to fn $34 that needs to be kept around */
struct rmi_fn_34_data {
	unsigned char status;
	unsigned char cmd;
	unsigned short bootloaderid;
	unsigned short blocksize;
	unsigned short imageblockcount;
	unsigned short configblockcount;
	unsigned short blocknum;
	bool inflashprogmode;
	struct mutex attn_mutex;
    #ifdef SYNA
    u8 product_id[RMI_PRODUCT_ID_LENGTH+1];
    #endif
};

static ssize_t rmi_fn_34_status_show(struct device *dev,
				     struct device_attribute *attr, char *buf);


static ssize_t rmi_fn_34_status_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t count);

static ssize_t rmi_fn_34_cmd_show(struct device *dev,
				  struct device_attribute *attr, char *buf);

static ssize_t rmi_fn_34_cmd_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count);

#ifdef KERNEL_VERSION_ABOVE_2_6_32
static ssize_t rmi_fn_34_data_read(struct file *data_file, struct kobject *kobj,
				   struct bin_attribute *attributes,
				   char *buf, loff_t pos, size_t count);

static ssize_t rmi_fn_34_data_write(struct file *data_file,
				    struct kobject *kobj,
				    struct bin_attribute *attributes, char *buf,
				    loff_t pos, size_t count);
#else
static ssize_t rmi_fn_34_data_read(struct kobject *kobj,
				   struct bin_attribute *attributes,
				   char *buf, loff_t pos, size_t count);

static ssize_t rmi_fn_34_data_write(struct kobject *kobj,
				    struct bin_attribute *attributes, char *buf,
				    loff_t pos, size_t count);
#endif

static ssize_t rmi_fn_34_bootloaderid_show(struct device *dev,
					   struct device_attribute *attr,
					   char *buf);

static ssize_t rmi_fn_34_bootloaderid_store(struct device *dev,
					    struct device_attribute *attr,
					    const char *buf, size_t count);

static ssize_t rmi_fn_34_blocksize_show(struct device *dev,
					struct device_attribute *attr,
					char *buf);

static ssize_t rmi_fn_34_imageblockcount_show(struct device *dev,
					      struct device_attribute *attr,
					      char *buf);

static ssize_t rmi_fn_34_configblockcount_show(struct device *dev,
					       struct device_attribute *attr,
					       char *buf);

static ssize_t rmi_fn_34_blocknum_show(struct device *dev,
				       struct device_attribute *attr,
				       char *buf);

static ssize_t rmi_fn_34_blocknum_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count);

static ssize_t rmi_fn_34_rescanPDT_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count);

#ifdef SYNA
#ifndef SAME_PATH
static ssize_t rmi_fn_34_productid_show(struct device *dev,
					struct device_attribute *attr,
					char *buf);
#endif
#endif
static ssize_t rmi_fn_34_read_pdt(struct rmi_function_container *fc);

static int rmi_f34_alloc_memory(struct rmi_function_container *fc);

static void rmi_f34_free_memory(struct rmi_function_container *fc);

static int rmi_f34_initialize(struct rmi_function_container *fc);

static int rmi_f34_config(struct rmi_function_container *fc);

static int rmi_f34_reset(struct rmi_function_container *fc);

static int rmi_f34_create_sysfs(struct rmi_function_container *fc);


#ifdef CONFIG_UPDATE_RMI4_FIRMWARE
#define CONFIG_ID_LEN		4
/* data specific to fn $34 that needs to be kept around */

#define READ_MEM_OK                 1u
#define READ_MEM_FAILED             2u
#define WRITE_MEM_OK                1u
#define WRITE_MEM_FAILED            2u
#define OBJECT_NOT_FOUND            0u

#define F01_VERSION_QUERY_OFFSET   	3
#define F01_PRODUCTID_QUERY_OFFSET	11
#define F01_PRODUCTID_SIZE			10

static int power_reset_enable = 0;
static int is_upgrade_firmware = 0;

#define firmware_attr(_name) \
static struct kobj_attribute _name##_attr = {	\
	.attr	= {				\
		.name = __stringify(_name),	\
		.mode = 0664,			\
	},					\
	.show	= _name##_show,			\
	.store	= _name##_store,		\
}

extern irqreturn_t rmi_i2c_irq_thread(int irq, void *p);

static int ts_firmware_file(void);
static int i2c_update_firmware(struct rmi_function_container *fc,
			const char * file) ;

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

#ifdef CONFIG_DEBUG_RMI4_FIRMWARE
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

extern int rmi_f11_getX_report(void);
extern int rmi_f11_getY_report(void);

#endif

extern struct rmi_device_platform_data *rmi_i2c_get_platform_data(void);
static int ts_firmware_file(void)
{
	int ret = -1;
	struct kobject *kobject_ts = NULL;
	kobject_ts = kobject_create_and_add("touchscreen", NULL);
	if (!kobject_ts) {
	 	printk("create kobjetct error!\n");
	 	return ret;
	}

	ret = sysfs_create_file(kobject_ts, &update_firmware_attr.attr);
	if (ret) {
	 	kobject_put(kobject_ts);
	 	printk("create file error\n");
	 	return ret;
	}

	ret = sysfs_create_file(kobject_ts, &firmware_version_attr.attr);
	if (ret) {
		sysfs_remove_file(kobject_ts, &update_firmware_attr.attr);
	 	kobject_put(kobject_ts);
	 	printk("create file error\n");
	 	return ret;
	}

	ret = sysfs_create_file(kobject_ts, &firmware_file_version_attr.attr);
	if (ret) {
		sysfs_remove_file(kobject_ts, &update_firmware_attr.attr);
		sysfs_remove_file(kobject_ts, &firmware_version_attr.attr);
	 	kobject_put(kobject_ts);
	 	printk("create firmware file version error\n");
	 	return ret;
	}

	ret = sysfs_create_file(kobject_ts, &tp_control_attr.attr);
	if (ret) {
		sysfs_remove_file(kobject_ts, &update_firmware_attr.attr);
		sysfs_remove_file(kobject_ts, &firmware_version_attr.attr);
		sysfs_remove_file(kobject_ts, &firmware_file_version_attr.attr);
		kobject_put(kobject_ts);
		printk("create file error\n");
		return ret;
	}

	ret = sysfs_create_file(kobject_ts, &firmware_tptype_attr.attr);
	if (ret) {
		sysfs_remove_file(kobject_ts, &update_firmware_attr.attr);
		sysfs_remove_file(kobject_ts, &firmware_version_attr.attr);
		sysfs_remove_file(kobject_ts, &firmware_file_version_attr.attr);
		sysfs_remove_file(kobject_ts, &tp_control_attr.attr);
	 	kobject_put(kobject_ts);
	 	printk("create tptype file error\n");
	 	return ret;
	}

#ifdef CONFIG_DEBUG_RMI4_FIRMWARE
	ret = sysfs_create_file(kobject_ts, &ts_debug_attr.attr);
	if (ret) {
		sysfs_remove_file(kobject_ts, &update_firmware_attr.attr);
		sysfs_remove_file(kobject_ts, &firmware_version_attr.attr);
		sysfs_remove_file(kobject_ts, &firmware_file_version_attr.attr);
		sysfs_remove_file(kobject_ts, &tp_control_attr.attr);
		sysfs_remove_file(kobject_ts, &ts_debug_attr.attr);
	 	kobject_put(kobject_ts);
	 	printk("create debug_ts file error\n");
	 	return ret;
	}
#endif

	return ret;
}
static ssize_t update_firmware_show(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
    #ifdef CRC_CHECK
    if(SUCCESS == update_flag){
        return snprintf(buf, PAGE_SIZE, "%s",UPDATE_SUCCESS);
    }
    else{
      printk("%s: update_flag = %d\n", __func__,update_flag);
      return snprintf(buf, PAGE_SIZE, "%s",UPDATE_FAIL);
    } 
    #endif

}
static ssize_t update_firmware_store(struct kobject *kobj,
		struct kobj_attribute *attr, const char *buf, size_t count)
{
    //dev_err(&rmi_fc->dev, "%s: Couldn't create fw update sys file\r\n", __func__);
    //printk("%s: update_firmware_store\n", __func__);
	int ret = -1;
	unsigned char path_image[255];
	struct rmi_device_platform_data *rmi_ts = NULL;

	if(count > 255 || count == 0 || strnchr(buf, count, 0x20))
	{
        printk("%s: count > 255 || count == 0 || strnchr(buf, count, 0x20)\n", __func__);
	 	return ret;
	}

	memcpy (path_image, buf, count);

    printk("%s: memcpy\n", __func__);
	/* replace '\n' with  '\0' */
	if((path_image[count-1]) == '\n')
		path_image[count-1] = '\0';
	else
		path_image[count] = '\0';

	power_reset_enable = 0;
	is_upgrade_firmware = 1;

    printk("%s: ready to get rmi_i2c_get_platform_data\n", __func__);

	rmi_ts = rmi_i2c_get_platform_data();
    //printk("%s: get rmi_i2c_get_platform_data\n", __func__);
	if (rmi_ts == NULL) {
		printk("Update firmware failed!\n");
		return ret;
	}

	disable_irq(gpio_to_irq(rmi_ts->attn_gpio));
    printk("%s: [SYNA] ==== Disable interrupt ====\n", __func__);
	ret = i2c_update_firmware(rmi_fc, path_image);
    printk("%s: [SYNA] ==== Enable interrupt ====\n", __func__);
	enable_irq(gpio_to_irq(rmi_ts->attn_gpio));
	if( ret < 0 ){
        #ifdef CRC_CHECK
        update_flag = FAIL;
        #endif
	 	printk("Update firmware failed!\n");
	} else {
	    #ifdef CRC_CHECK
	    update_flag = SUCCESS;
        #endif
	 	printk("Update firmware success!\n");
	 	
	}

	power_reset_enable = 1;
	is_upgrade_firmware = 0;

	return count;
}

static ssize_t firmware_version_show(struct kobject *kobj,
			struct kobj_attribute *attr, char *buf)
{
	struct rmi_device *rmi_dev = NULL;
	int i, error = 0;
	char values[CONFIG_ID_LEN + 1] = {0};

	rmi_dev = rmi_fc->rmi_dev;
	error = rmi_read_block(rmi_dev, rmi_fc->fd.control_base_addr, values, CONFIG_ID_LEN);
	if (error < 0) {
		printk(KERN_ERR "%s: read version fail\r\n", __func__);
		return error;
	}
	//just for firmware version is TXXX or NXXX.
	#if 0
	for (i = 0; i < CONFIG_ID_LEN-1; i++) {
		if ((values[i] < '0') || (values[i] > '9')) {
			values[i] = '0';
		}
	}
	#endif
    if (values[0] == 0) {
        memset(values, '0', sizeof(values));
    }
    values[CONFIG_ID_LEN ] = 0;
	return sprintf(buf, "%s", values);
}
static ssize_t firmware_version_store(struct kobject *kobj,
			struct kobj_attribute *attr, const char *buf, size_t count)
{
    return 0;
}

static ssize_t firmware_file_version_show(struct kobject *kobj,
			struct kobj_attribute *attr,char *buf)
{
 	return 0;
}
static ssize_t firmware_file_version_store(struct kobject *kobj,
			struct kobj_attribute *attr, const char *buf, size_t count)
{
    return 0;
}
static ssize_t tp_control_show(struct kobject *kobj,
			struct kobj_attribute *attr,char *buf)
{
    return 0;
}

static ssize_t tp_control_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
      int on = -1;
      struct rmi_device_platform_data *rmi_ts = rmi_i2c_get_platform_data(); 
      struct rmi_fn_34_data *instance_data = rmi_fc->data;
      sscanf(buf,"%x",&on);
      if (instance_data->inflashprogmode == false) {
	    rmi_ts->set_power(on,"synaptics");
      }

      return count;
}

#if 0
static ssize_t firmware_tptype_show(struct kobject *kobj,
			struct kobj_attribute *attr, char *buf)
{
	struct rmi_function_container *f01_fc = NULL;
	struct rmi_device *rmi_dev = NULL;
	struct rmi_driver_data *driver_data = NULL;
	int error = 0;
	u8 tptype[F01_PRODUCTID_SIZE+1] = {0};

	rmi_dev = rmi_fc->rmi_dev;
	driver_data = rmi_get_driverdata(rmi_dev);
	f01_fc = driver_data->f01_container;

	error = rmi_read_block(f01_fc->rmi_dev, f01_fc->fd.query_base_addr
			+ F01_PRODUCTID_QUERY_OFFSET, tptype, F01_PRODUCTID_SIZE);
	if (error < 0) {
		printk(KERN_ERR "%s: read product type fail\r\n", __func__);
		return error;
	}

	return sprintf(buf, "%s", tptype) ;
}
#endif
static ssize_t firmware_tptype_store(struct kobject *kobj,
			struct kobj_attribute *attr, const char *buf, size_t count)
{
	return 0;
}

#ifdef CONFIG_DEBUG_RMI4_FIRMWARE
static ssize_t ts_debug_show(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	return 0;
}

static ssize_t ts_debug_store(struct kobject *kobj,
			struct kobj_attribute *attr, const char *buf, size_t count)
{
	u8 value = 0;
	int ret = -1, irq_flags;
	struct rmi_device_platform_data *rmi_ts = rmi_i2c_get_platform_data();

	if(buf[0] == FLAG_RR){
		if (rmi_read(rmi_fc->rmi_dev, (u16)buf[1], &value) < 0) {
			return 0;
		} else {
			return value;
		}
	}else if(buf[0] == FLAG_WR){
		if (rmi_write(rmi_fc->rmi_dev, (u16)buf[1], buf[2]) < 0) {
			return 0;
		} else {
			return 1;
		}
	}else if(buf[0] == FLAG_RPX){
		return rmi_f11_getX_report();
	}else if(buf[0] == FLAG_RPY){
		return rmi_f11_getY_report();
	}else if(buf[0] == FLAG_RNIT){
		return rmi_ts->tp_gpio_get_value();
	}else if(buf[0] == FLAG_HRST){
		return rmi_ts->tp_gpio_reset("synaptics");
	}else  if(buf[0] == FLAG_SRST){
		/* Issue a reset command */
		if (rmi_write(rmi_fc->rmi_dev, rmi_fc->fd.command_base_addr, 0x01) < 0) {
			return 0;
		}
		mdelay(300);
		rmi_read(rmi_fc->rmi_dev, rmi_fc->fd.data_base_addr, &value);
		return 1;
	}else if(buf[0] == FLAG_EINT){
		irq_flags = (rmi_ts->attn_polarity == RMI_ATTN_ACTIVE_HIGH) ?
			IRQF_TRIGGER_RISING : IRQF_TRIGGER_FALLING;
		ret = request_threaded_irq(gpio_to_irq(rmi_ts->attn_gpio), NULL, rmi_i2c_irq_thread,
			irq_flags, "rmi4", rmi_fc->rmi_dev->phys);
		return 1;
	}else if(buf[0] == FLAG_DINT){
		free_irq(gpio_to_irq(rmi_ts->attn_gpio), rmi_fc->rmi_dev->phys);
		return 1;
	}else if(buf[0] == FLAG_PRST){
		if (rmi_ts->set_power(1, "synaptics") < 0) {
			return 0;
		} else {
			return 1;
		}
	}else if(buf[0] == FLAG_POFF){
		if (rmi_ts->set_power(0, "synaptics") < 0) {
			return 0;
		} else {
			return 1;
		}
	}else if(buf[0] == FLAG_EFW){
		return 0;
	}else {
		return 0;
	}
}
#endif

#ifdef CONFIG_UPDATE_RMI4_FIRMWARE
static unsigned long extract_long(const unsigned char *ptr_data)
{
	return((unsigned long)ptr_data[0] +
		 (unsigned long)(ptr_data[1] << 8) +
		 (unsigned long)(ptr_data[2] << 16) +
		 (unsigned long)(ptr_data[3] << 24));
}

static int rmi_check_firmware(struct rmi_function_container *fc,
			const unsigned char *pgm_data)
{
	unsigned long checkSumCode = 0;
	unsigned long m_firmwareImgSize = 0;
	unsigned long m_configImgSize = 0;
	unsigned short m_bootloadImgID = 0;
	unsigned char m_firmwareImgVersion = 0;
	unsigned short bootloader_id = 0;
	unsigned short ui_block_count = 0;
	unsigned short conf_block_count = 0;
	unsigned short fw_block_size = 0;

	struct rmi_fn_34_data *instance_data = fc->data;

	/* extract firmware data from firmware file */
	checkSumCode = extract_long(&(pgm_data[0]));
	m_bootloadImgID = (unsigned short)pgm_data[4]
					+ (unsigned short)(pgm_data[5] << 8);
	m_firmwareImgVersion = pgm_data[7];
	m_firmwareImgSize = extract_long(&(pgm_data[8]));
	m_configImgSize = extract_long(&(pgm_data[12]));

 	/* read some item from instance data */
	ui_block_count = instance_data->imageblockcount;
	conf_block_count = instance_data->configblockcount;
	fw_block_size = instance_data->blocksize;
	bootloader_id = instance_data->bootloaderid;

	/* return the compared result */
	return (m_firmwareImgVersion != 0 || bootloader_id == m_bootloadImgID) ? 0 : -EINVAL;
}

#ifdef CONFIG_UPDATE_RMI4_FIRMWARE   //fixme //syna
static int rmi_wait_attn(struct rmi_function_container *fc, int udelay)
{
	struct rmi_function_container *f01_fc = NULL;
	struct rmi_device *rmi_dev = NULL;
	struct rmi_driver_data *driver_data = NULL;
	int loop_count = 0;
	u8 status = 0, retval = 0;
      u8 f34_status = 0, f01_status = 0;
	u16 data_base_addr = fc->fd.data_base_addr;
	struct rmi_fn_34_data *instance_data = fc->data;

	rmi_dev = fc->rmi_dev;
	driver_data = rmi_get_driverdata(rmi_dev);
	f01_fc = driver_data->f01_container;

#if 0
	do {
		mdelay(udelay);
		rmi_read(fc->rmi_dev,
			  data_base_addr + instance_data->blocksize +
			  BLK_NUM_OFF, &status);
         dev_err(&fc->dev, "%s : status = % d\n", __func__,status);

		/* Clear the attention assertion by reading
		 * the interrupt status register */
		rmi_read(f01_fc->rmi_dev, f01_fc->fd.data_base_addr + 1, &retval);
	} while ((loop_count++ < 0x20) && (status != 0x80));
#else
	do {
		 retval = rmi_read(fc->rmi_dev,
			  data_base_addr + instance_data->blocksize +
			  BLK_NUM_OFF, &f34_status);
		 
		if ((f34_status == 0x80) && (retval >= 0))
		{
			/* Clear the attention assertion by reading the interrupt status register */		
			retval = rmi_read(f01_fc->rmi_dev, f01_fc->fd.data_base_addr + 1, &status);			
			break;
		}
		else
		{
			dev_err(&fc->dev, "%s : F$34 status = 0x%X, ret = %d\n", __func__, f34_status, retval);
		}
		mdelay(udelay);	
		} while (loop_count++ < 0x20);
#endif

	if (loop_count >= 0x20) {
		printk(KERN_ERR "%s: timeout, status = %02x, blocksize = %d\n",
				__func__, status, instance_data->blocksize);
		return -EINVAL;
	}
	return 0;
}
static int rmi_enable_program(struct rmi_function_container *fc)
{
	struct rmi_fn_34_data *instance_data = fc->data;
	u16 data_base_addr = fc->fd.data_base_addr;
	u16 query_dase_addr = fc->fd.query_base_addr;

	struct rmi_function_container *f01_fc = NULL;
	struct rmi_device *rmi_dev = NULL;
	struct rmi_driver_data *driver_data = NULL;
	u8 data[2] = {0};
	u8 status = 0;
	int error = 0;
    int count = 0;
	rmi_dev = fc->rmi_dev;
	driver_data = rmi_get_driverdata(rmi_dev);
	f01_fc = driver_data->f01_container;

    dev_err(&fc->dev, "%s : [SYNA] rmi_enable_program\n", __func__);

    #if 0   //SYNA //FIXME
	/* Check flash mode and status */
	if ((instance_data->inflashprogmode)
		|| (STATUS_IDLE != instance_data->status)) {
		printk(KERN_ERR "%s: Check flash mode and status\n", __func__);
		return -EINVAL;
	}
    #endif
    
    #if 1
    if (instance_data->inflashprogmode)
    {
        dev_info(&fc->dev, "%s : Already in flash programming mode\n", __func__);
        return 0;
    }
    #endif
    #if 0
    if (STATUS_IDLE != instance_data->status)
    {
        dev_err(&fc->dev, "%s : Check flash mode and status\n", __func__);
        return -EINVAL;
    }
    #endif
    

    dev_err(&fc->dev, "%s : [SYNA] Check flash mode and status success\n", __func__);

	/* Read and write bootload ID, which acts as a key to Enable the operation */
	error = rmi_read_block(fc->rmi_dev, query_dase_addr, &data[0], 2);
	if (error < 0) {
		printk(KERN_ERR "%s: Could not read bootloader ID\n", __func__);
		return error;
	}
    dev_err(&fc->dev, "%s : [SYNA] read bootloader ID\n", __func__);
	error = rmi_write_block(fc->rmi_dev,
			data_base_addr + BLK_NUM_OFF, &data[0], 2);
	if (error < 0) {
		printk(KERN_ERR "%s: Could not writeback bootloader ID\n", __func__);
		return error;
	}
     dev_err(&fc->dev, "%s : [SYNA] write bootloader ID\n", __func__);

	/* Make sure Reflash is not already enabled */
	do {
		mdelay(10);
		rmi_read(fc->rmi_dev,
				data_base_addr + instance_data->blocksize + BLK_NUM_OFF,
				&status);
        #ifdef CRC_CHECK
        dev_err(&fc->dev, "%s : status = %d\n", __func__,status);  
        #endif
	} while ((count++ < DELAY_COUNT) && ((status & 0x0f) != 0x00));
    
	if (count >= DELAY_COUNT) {
		dev_err(&fc->dev, "%s: count = %02d ,status = 0x%02x\n",__func__,count,status);
		return -EINVAL;
	}
    
    dev_err(&fc->dev, "%s : [SYNA] Make sure Reflash is not already enabled???\n", __func__);

	/* Clear ATTN */
	error = rmi_read(f01_fc->rmi_dev, f01_fc->fd.data_base_addr, &status);
	if (error < 0) {
		printk(KERN_ERR "%s: Clear ATTN fail\n", __func__);
		return error;
	}
	dev_err(&fc->dev, "%s : [SYNA] Clear attn???\n", __func__);
    dev_err(&fc->dev, "%s : [SYNA] Device status 0x%x\n", __func__, status);
	if ((status & 0x40) == 0) {
		/* Write the "Enable Flash Programming command to F34 Control register
		 * Wait for ATTN and then clear the ATTN. */
		u8 s_data = 0xf;
        dev_err(&fc->dev, "%s : [SYNA] write enable flash command\n", __func__);
		error = rmi_write(fc->rmi_dev,
					data_base_addr + instance_data->blocksize +
					BLK_NUM_OFF, ENABLE_FLASH_PROG);
        dev_err(&fc->dev, "%s : [SYNA] write enable flash command finish\n", __func__);
		if (error < 0) {
			printk(KERN_ERR "%s: Could not write enable command to 0x%04x\n",
				__func__, data_base_addr + instance_data->blocksize + BLK_NUM_OFF);
			return error;
		}

		mdelay(300);
		rmi_read(f01_fc->rmi_dev, f01_fc->fd.data_base_addr + 1, &status);

        dev_err(&fc->dev, "%s : [SYNA] ready to re-scan PDT\n", __func__);
		/* Scan the PDT again to ensure all register offsets are correct */
		rmi_fn_34_read_pdt(fc);
		dev_err(&fc->dev, "%s : [SYNA] re-scan PDT Finished\n", __func__);
		/* Read the "Program Enabled" bit of the F34 Control register,
		 * and proceed only if the bit is set. */
		rmi_read(fc->rmi_dev,
					data_base_addr + instance_data->blocksize +
					BLK_NUM_OFF, &s_data);
		/* In practice, if uData!=0x80 happens for multiple counts, it indicates
		 * reflash is failed to be enabled, and program should quit */

        dev_err(&fc->dev, "%s : [SYNA] check f34 status 0x%x\n", __func__, s_data);
        if (s_data != 0x80) {
			error = rmi_wait_attn(fc, 20);
			if (error < 0) {
                dev_err(&fc->dev, "%s : error = 0x%x\n", __func__, error);
				return error;
			}
		}
	}

	/* set enable mode */
	instance_data->inflashprogmode = true;
	/* set status to indicate we are in progress */
	instance_data->status = STATUS_IN_PROGRESS;
	return 0;
}
static int rmi_disable_program(struct rmi_function_container *fc)
{
	struct rmi_function_container *f01_fc = NULL;
	struct rmi_device *rmi_dev = NULL;
	struct rmi_driver_data *driver_data = NULL;
	struct rmi_fn_34_data *instance_data = fc->data;
	int error = 0;
	u8 status = 0;
    int count = 0;

	rmi_dev = fc->rmi_dev;
	driver_data = rmi_get_driverdata(rmi_dev);
	f01_fc = driver_data->f01_container;

	/* Issue a reset command */
	error = rmi_write(f01_fc->rmi_dev, f01_fc->fd.command_base_addr, 0x01);
	if (error < 0) {
		printk(KERN_ERR "%s: write reset command fail\r\n", __func__);
		return error;
	}
	mdelay(300);
	rmi_read(f01_fc->rmi_dev, f01_fc->fd.data_base_addr, &status);

	/* Sanity check that the reflash process is still enabled */
	do {
		mdelay(10);
		rmi_read(fc->rmi_dev, fc->fd.data_base_addr
				+ instance_data->blocksize + BLK_NUM_OFF, &status);
        #ifdef CRC_CHECK
        dev_err(&fc->dev, "%s : [1]status = %d\n", __func__,status);
        #endif
	} while ((count++ < DELAY_COUNT) && ((status & 0x0f) != 0x00));
    if (count >= DELAY_COUNT) {
		dev_err(&fc->dev, "%s: [1]count = %02d ,status = 0x%02x\n",__func__,count,status);
		return -EINVAL;
	}
    //You must clear the count.
    count = 0;
    
	rmi_read(f01_fc->rmi_dev, f01_fc->fd.data_base_addr + 1, &status);

	rmi_fn_34_read_pdt(fc);

	/* Check if the "Program Enabled" bit in F01 data register is cleared
	 * Reflash is completed, and the image passes testing when the bit is cleared */
	do {
		mdelay(10);
		rmi_read(f01_fc->rmi_dev, f01_fc->fd.data_base_addr, &status);
        #ifdef CRC_CHECK
        dev_err(&fc->dev, "%s : [2]status = %d\n", __func__,status);
        #endif
	} while ((count++ < DELAY_COUNT) && ((status & 0x40) != 0));
    if (count >= DELAY_COUNT) {
		dev_err(&fc->dev, "%s: [2]count = %02d ,status = 0x%02x\n",__func__,count,status);
		return -EINVAL;
	}

	/* Rescan PDT the update any changed register offsets */
	rmi_fn_34_read_pdt(fc);

	/* set disable mode */
	instance_data->inflashprogmode = false;
	/* set status to indicate we are in progress */
	instance_data->status = STATUS_IDLE;
	return 0;
}
#endif
static int rmi_write_image(struct rmi_function_container *fc,
			unsigned char type_cmd, const unsigned char *pgm_data)
{
	struct rmi_fn_34_data *instance_data = fc->data;
	u16 data_base_addr = fc->fd.data_base_addr;
	unsigned short blocks_cnt = 0;
	unsigned short block_index = 0;
	unsigned short block_size = 0;
	const unsigned char * p_data = NULL;
	u8 data[2] = {0};
	int error = 0;
    u8 status = 0;

	block_size = instance_data->blocksize;
	if (type_cmd == WRITE_FW_BLOCK) {
		blocks_cnt = instance_data->imageblockcount;
	} else if (type_cmd == WRITE_CONFIG_BLOCK) {
		blocks_cnt = instance_data->configblockcount;
	} else {
		printk(KERN_ERR "%s: Command type error\r\n", __func__);
		return -EINVAL;
	}

	printk("%s: block_size=%d,block_cnt=%d\r\n",
			__func__,block_size,blocks_cnt);
	p_data = pgm_data;
	for(block_index = 0; block_index < blocks_cnt; block_index++){

        #ifdef ENABLE_SHOW_FLASH_PROGRESS
            printk(KERN_ERR "Flash progress %d/%d\r\n", block_index, blocks_cnt);
        #endif
        /* Write Block Number */
        if ((type_cmd == WRITE_CONFIG_BLOCK) ||
            ((type_cmd == WRITE_FW_BLOCK) && (block_index == 0)))
        {
            #ifdef ENABLE_SYNA_DEBUG
            printk(KERN_ERR "[SYNA] write block number %d\n", block_index);
            #endif
            hstoba(data, block_index);
    		error = rmi_write_block(fc->rmi_dev, data_base_addr, &data[0], 2);
    		if(error < 0){
    			printk(KERN_ERR "%s: write block number error\n", __func__);
    			return error;
    		}
        }

		/* Write the data block - only if the count is non-zero */
		if (block_size) {
            #ifdef ENABLE_SYNA_DEBUG
            printk(KERN_ERR "[SYNA] start to rmi_write_block\n");
            #endif
			error = rmi_write_block(fc->rmi_dev,
					data_base_addr + BLK_NUM_OFF,
					(unsigned char *)p_data, block_size);

			if (error < 0) {
				printk(KERN_ERR "%s : Could not write block data "
					"to 0x%x\n", __func__,
					data_base_addr + BLK_NUM_OFF);
				return error;
			}
            #ifdef ENABLE_SYNA_DEBUG
            printk(KERN_ERR "[SYNA] rmi_write_block sucess\n");
            #endif
		}

		p_data += block_size;

		/* Issue Write Firmware or configuration Block command */
        #ifdef ENABLE_SYNA_DEBUG
        printk(KERN_ERR "[SYNA] %s: issue write command\n", __func__);
        #endif
		error = rmi_write(fc->rmi_dev,
				data_base_addr + instance_data->blocksize +
				BLK_NUM_OFF, type_cmd);

		if (error < 0) {
			printk(KERN_ERR "%s: issue write command error\n", __func__);
			return error;
		}
        #ifdef ENABLE_SYNA_DEBUG
        printk(KERN_ERR "[SYNA] %s: issue write command success\n", __func__);
        #endif

		/* Wait ATTN. Read Flash Command register and check error */
		if(rmi_wait_attn(fc, 12) != 0) {
			printk(KERN_ERR "%s: wait ATTN. fail\n", __func__);
			return error;
		}
        #ifdef ENABLE_SYNA_DEBUG
        printk(KERN_ERR "[SYNA] %s: attntion is asserted\n", __func__);
        #endif
        /* Read device status */
        error = rmi_read(fc->rmi_dev, 0x12, &status);   //syna fixed me
        if (error < 0) {
            printk(KERN_ERR "%s: Clear ATTN fail\n", __func__);
            return error;
        }
        #ifdef ENABLE_SYNA_DEBUG
        printk(KERN_ERR "[SYNA] flash status 0x%2X\n", status);
        #endif
        if (status != 0x80)
        {
           printk(KERN_ERR "[SYNA] Error! flash status 0x%2X\n", status);
        }
	}

	return 0;
}
static int rmi_program_config(struct rmi_function_container *fc,
			const unsigned char *pgm_data)
{
	struct rmi_fn_34_data *instance_data = fc->data;
	u16 data_base_addr = fc->fd.data_base_addr;
	int error = 0;
	u8 status = 0;
	unsigned short block_size = 0;
	unsigned short ui_blocks = 0;

	struct rmi_function_container *f01_fc = NULL;
	struct rmi_device *rmi_dev = NULL;
	struct rmi_driver_data *driver_data = NULL;

	rmi_dev = fc->rmi_dev;
	driver_data = rmi_get_driverdata(rmi_dev);
	f01_fc = driver_data->f01_container;

	block_size = instance_data->blocksize;
	ui_blocks = instance_data->imageblockcount;

	/* Make sure it is in bootloader mode */
	error = rmi_read(f01_fc->rmi_dev, f01_fc->fd.data_base_addr, &status);
	if (error < 0) {
		printk(KERN_ERR "%s: Clear ATTN fail\n", __func__);
		return error;
	}
	if ((status & 0x40) == 0) {
		printk(KERN_ERR "%s: status error, not in bootloader mode\n", __func__);
		return -EINVAL;
	}

 	error = rmi_write_image(fc, WRITE_CONFIG_BLOCK, pgm_data + ui_blocks * block_size);
	if (error < 0) {
		printk(KERN_ERR "%s: write configure image error\n", __func__);
        return error;
	}

	error = rmi_read(fc->rmi_dev,
			data_base_addr + instance_data->blocksize +
			BLK_NUM_OFF, &status);
	if (error < 0) {
		printk(KERN_ERR "%s: read status fail\r\n", __func__);
		return error;
	}

	return ((status & 0xf0) == 0x80) ? 0 : -EINVAL;
}

static int rmi_program_firmware(struct rmi_function_container *fc,
			const unsigned char *pgm_data)
{
	struct rmi_fn_34_data *instance_data = fc->data;
	u16 data_base_addr = fc->fd.data_base_addr;
	u16 query_dase_addr = fc->fd.query_base_addr;
	u8 data[2] = {0};
	u8 status = 0;
	int error = 0;
	struct rmi_function_container *f01_fc = NULL;
	struct rmi_device *rmi_dev = NULL;
	struct rmi_driver_data *driver_data = NULL;

	rmi_dev = fc->rmi_dev;
	driver_data = rmi_get_driverdata(rmi_dev);
	f01_fc = driver_data->f01_container;

    #ifdef ENABLE_SYNA_DEBUG
    printk(KERN_ERR "[SYNA] ready to read/write BootID before erase all %s\n", __func__);
    #endif
	/* Read and write bootload ID, which acts as a key to Enable the operation */
	error = rmi_read_block(fc->rmi_dev, query_dase_addr, &data[0], 2);
	if (error < 0) {
		printk(KERN_ERR "%s: Could not read bootloader ID\n", __func__);
		return error;
	}
	error = rmi_write_block(fc->rmi_dev,
			data_base_addr + BLK_NUM_OFF, &data[0], 2);
	if (error < 0) {
		printk(KERN_ERR "%s: Could not writeback bootloader ID\n", __func__);
		return error;
	}

	/* Make sure it is in bootloader mode */
	error = rmi_read(f01_fc->rmi_dev, f01_fc->fd.data_base_addr, &status);
	if (error < 0) {
		printk(KERN_ERR "%s: Clear ATTN fail\n", __func__);
		return error;
	}
	if ((status & 0x40) == 0) {
		printk(KERN_ERR "%s: status error, not in bootloader mode\n", __func__);
		return -EINVAL;
	}

	/* issue erase all command */
	error = rmi_write(fc->rmi_dev,
			data_base_addr + instance_data->blocksize +
			BLK_NUM_OFF, ERASE_ALL);
	if (error < 0) {
		printk(KERN_ERR "%s: erase firmware error\n", __func__);
		return -EINVAL;
	}
    #ifdef ENABLE_SYNA_DEBUG
    printk(KERN_ERR "[SYNA] erase all command %s\n", __func__);
    #endif
	mdelay(500);

    error = rmi_wait_attn(fc, 50);

    if (error < 0) {
		printk(KERN_ERR "%s: rmi_wait_attn - status error\n", __func__);
		return error;
	}
    #ifdef ENABLE_SYNA_DEBUG
    else
    {
        printk(KERN_ERR "%s Erase all finish, attn is asserted\n", __func__);
    }
    #endif

	rmi_read(f01_fc->rmi_dev, f01_fc->fd.data_base_addr + 1, &status);

	/* Read the "Program Enabled" bit of the F34 Control register,
	 * and proceed only if the bit is set. */
	rmi_read(fc->rmi_dev,
				data_base_addr + instance_data->blocksize +
				BLK_NUM_OFF, &status);
	/* In practice, if uData!=0x80 happens for multiple counts, it indicates
	 * reflash is failed to be enabled, and program should quit */
	 #if 0  //syna
	if (status != 0x80) {
		error = rmi_wait_attn(fc, 20);
		if (error < 0) {
			printk(KERN_ERR "%s: rmi_wait_attn - status error\n", __func__);
			return error;
		}
	}
    #else
	if (status != 0x80)
    {
	    printk(KERN_ERR "%s: rmi_wait_attn - status error\n", __func__);
		return error;
	}

    #endif

	/* Make sure it is in bootloader mode */
	error = rmi_read(f01_fc->rmi_dev, f01_fc->fd.data_base_addr, &status);
	if (error < 0) {
		printk(KERN_ERR "%s: Clear ATTN fail\n", __func__);
		return error;
	}
	if ((status & 0x40) == 0) {
		printk(KERN_ERR "%s: status error, not in bootloader mode\n", __func__);
		return -EINVAL;
	}

	/* write firmware */
    printk(KERN_ERR "[SYNA] 4 start to flash fw %s\n", __func__);
	error = rmi_write_image(fc, WRITE_FW_BLOCK, pgm_data);
	if (error < 0) {
		printk(KERN_ERR "%s: write UI firmware error\n", __func__);
		return error;
	}
    printk(KERN_ERR "[SYNA]  flash fw success %s\n", __func__);

	/* read status */
	error = rmi_read(fc->rmi_dev,
			  data_base_addr + instance_data->blocksize +
			  BLK_NUM_OFF, &status);
	if (error < 0) {
		printk(KERN_ERR "%s: check firmware status error\n", __func__);
		return error;
	}

	return ((status & 0xf0) == 0x80) ? 0 : -EINVAL;
}
static ssize_t rmi_firmware_download(struct rmi_function_container *fc,
			unsigned char *buf)
{
	struct rmi_fn_34_data *instance_data = NULL;
	u16 data_base_addr = 0;
	int error = 0;

	instance_data = fc->data;
	data_base_addr = fc->fd.data_base_addr;

	error = rmi_fn_34_read_pdt(fc);
    printk(KERN_ERR "[SYNA] 6 %s\n", __func__);
	if (error < 0) {
		printk(KERN_ERR "%s : Read PDT fail\r\n", __func__);
		return -EINVAL;
	}

	error = rmi_enable_program(fc);
    printk(KERN_ERR "[SYNA] 14 %s\n", __func__);
	if (error < 0) {
		printk(KERN_ERR "%s : Enable program status fail\r\n", __func__);
		return -EINVAL;
	}

	error = rmi_check_firmware(fc, buf);
    printk(KERN_ERR "[SYNA] 8 %s\n", __func__);
	if (error < 0) {
		printk(KERN_ERR "%s : Check firmware fail\r\n", __func__);
		goto data_write_fail;
	}

      printk(KERN_ERR "=== Flash UI FW Start===\n");
	error = rmi_program_firmware(fc, buf + 0x100);
      printk(KERN_ERR "=== Flash UI FW Finished===\n");
	if (error < 0) {
		printk(KERN_ERR "%s : Program firmware fail\r\n", __func__);
		goto data_write_fail;
	}

      printk(KERN_ERR "=== Flash Config Start===\n");
	error = rmi_program_config(fc, buf + 0x100);
      printk(KERN_ERR "=== Flash Config Finished===\n");
	if (error < 0) {
		printk(KERN_ERR "%s : Program config fail\r\n", __func__);
		goto data_write_fail;
	}

	error = rmi_disable_program(fc);
	if (error < 0) {
		printk(KERN_ERR "%s : Disable program status fail\r\n", __func__);
	}

	return error;

data_write_fail:
	rmi_disable_program(fc);
	return error;
}

static int i2c_update_firmware(struct rmi_function_container *fc,
			const char * file)
{
	void *buf = NULL;
	struct file *filp = NULL;
	struct inode *inode = NULL;
	mm_segment_t oldfs;
	uint16_t	length = 0;
	int ret = 0;

    /* open file */
	oldfs = get_fs();
	set_fs(KERNEL_DS);
	filp = filp_open(file, O_RDONLY, S_IRUSR);
	if (IS_ERR(filp)) {
		printk("%s: file %s filp_open error\n", __func__,file);
		set_fs(oldfs);
		return -EINVAL;
	}

	if (!filp->f_op) {
		printk("%s: File Operation Method Error\n", __func__);
		filp_close(filp, NULL);
		set_fs(oldfs);
		return -EINVAL;
	}

	inode = filp->f_path.dentry->d_inode;
	if (!inode) {
		printk("%s: Get inode from filp failed\n", __func__);
		filp_close(filp, NULL);
		set_fs(oldfs);
		return -EINVAL;
	}

	/* file's size */
	length = i_size_read(inode->i_mapping->host);
	printk("%s: rmi image file is %s and data size is %d Bytes\n",
			__func__,file,length);
	if (!( length > 0 && length < 64*1024 )){
		printk("%s: file size error\n", __func__);
		filp_close(filp, NULL);
		set_fs(oldfs);
		return -EINVAL;
	}

	/* allocation buff size */
	buf = vmalloc(length+(length%2));	/* buf size if even */
	if (!buf) {
		printk("%s: alloctation memory failed\n", __func__);
		filp_close(filp, NULL);
		set_fs(oldfs);
		return -EINVAL;
	}

	/* read data */
	if (filp->f_op->read(filp, (char *)buf, length, &filp->f_pos) != length) {
		printk("%s: file read error\n", __func__);
		filp_close(filp, NULL);
		set_fs(oldfs);
		vfree(buf);
		return -EINVAL;
	}

	ret = rmi_firmware_download(fc, buf);

	filp_close(filp, NULL);
	set_fs(oldfs);
	vfree(buf);
	return ret;

}
#endif

#endif



static struct device_attribute attrs[] = {
	__ATTR(status, RMI_RW_ATTR,
	       rmi_fn_34_status_show, rmi_fn_34_status_store),

	/* Also, sysfs will need to have a file set up to distinguish
	 * between commands - like Config write/read, Image write/verify. */
	__ATTR(cmd, RMI_RW_ATTR,
	       rmi_fn_34_cmd_show, rmi_fn_34_cmd_store),
	__ATTR(bootloaderid, RMI_RW_ATTR,
	       rmi_fn_34_bootloaderid_show, rmi_fn_34_bootloaderid_store),
	__ATTR(blocksize, RMI_RO_ATTR,
	       rmi_fn_34_blocksize_show, rmi_store_error),
	__ATTR(imageblockcount, RMI_RO_ATTR,
	       rmi_fn_34_imageblockcount_show, rmi_store_error),
	__ATTR(configblockcount, RMI_RO_ATTR,
	       rmi_fn_34_configblockcount_show, rmi_store_error),
	__ATTR(blocknum, RMI_RW_ATTR,
	       rmi_fn_34_blocknum_show, rmi_fn_34_blocknum_store),
	__ATTR(rescanPDT, RMI_WO_ATTR,
	       rmi_show_error, rmi_fn_34_rescanPDT_store),
#ifdef SYNA
#ifndef SAME_PATH
    __ATTR(productinfo, RMI_RW_ATTR,
       rmi_fn_34_productid_show, rmi_store_error),
#endif
#endif
};
struct bin_attribute dev_attr_data = {
	.attr = {
		 .name = "data",
		 .mode = 0664},
	.size = 0,
	.read = rmi_fn_34_data_read,
	.write = rmi_fn_34_data_write,
};

static int rmi_f34_init(struct rmi_function_container *fc)
{
	int retval;

	dev_info(&fc->dev, "Intializing f34 values.");

#ifdef CONFIG_UPDATE_RMI4_FIRMWARE
	/* init gloable function container */
	rmi_fc = fc;
#endif

	/* init instance data, fill in values and create any sysfs files */
	retval = rmi_f34_alloc_memory(fc);
	if (retval < 0)
		goto exit_free_data;

	retval = rmi_f34_initialize(fc);
	if (retval < 0)
		goto exit_free_data;

	retval = rmi_f34_create_sysfs(fc);
	if (retval < 0)
		goto exit_free_data;

#ifdef CONFIG_UPDATE_RMI4_FIRMWARE
	retval = ts_firmware_file();
	if (retval) {
		dev_err(&fc->dev, "%s: Couldn't create fw update sys file\r\n", __func__);
		goto exit_free_data;
	}
#endif
    return 0;

exit_free_data:
	rmi_f34_free_memory(fc);

	return retval;
}

static int rmi_f34_alloc_memory(struct rmi_function_container *fc)
{
	struct rmi_fn_34_data *f34;

	f34 = kzalloc(sizeof(struct rmi_fn_34_data), GFP_KERNEL);
	if (!f34) {
		dev_err(&fc->dev, "Failed to allocate rmi_fn_34_data.\n");
		return -ENOMEM;
	}
	fc->data = f34;

	return 0;
}

static void rmi_f34_free_memory(struct rmi_function_container *fc)
{
	kfree(fc->data);
	fc->data = NULL;
}

static int rmi_f34_initialize(struct rmi_function_container *fc)
{
	struct rmi_device *rmi_dev = fc->rmi_dev;
	struct rmi_device_platform_data *pdata;
	int retval = 0;
	struct rmi_fn_34_data *f34 = fc->data;
	u16 query_base_addr;
	u16 control_base_addr;
	unsigned char buf[2];

	pdata = to_rmi_platform_data(rmi_dev);
	dev_dbg(&fc->dev, "Initializing F34 values for %s.\n",
		pdata->sensor_name);

	mutex_init(&f34->attn_mutex);

	/* get the Bootloader ID and Block Size. */
	query_base_addr = fc->fd.query_base_addr;
	control_base_addr = fc->fd.control_base_addr;

	retval = rmi_read_block(fc->rmi_dev, query_base_addr, buf,
			ARRAY_SIZE(buf));

	if (retval < 0) {
		dev_err(&fc->dev, "Could not read bootloaderid from 0x%04x.\n",
			query_base_addr);
		return retval;
	}

	batohs(&f34->bootloaderid, buf);

	retval = rmi_read_block(fc->rmi_dev, query_base_addr + BLK_SZ_OFF, buf,
			ARRAY_SIZE(buf));

	if (retval < 0) {
		dev_err(&fc->dev, "Could not read block size from 0x%04x, "
			"error=%d.\n", query_base_addr + BLK_SZ_OFF, retval);
		return retval;
	}
	batohs(&f34->blocksize, buf);

	/* Get firmware image block count and store it in the instance data */
	retval = rmi_read_block(fc->rmi_dev, query_base_addr + IMG_BLK_CNT_OFF,
			buf, ARRAY_SIZE(buf));

	if (retval < 0) {
		dev_err(&fc->dev, "Couldn't read image block count from 0x%x, "
			"error=%d.\n", query_base_addr + IMG_BLK_CNT_OFF,
			retval);
		return retval;
	}
	batohs(&f34->imageblockcount, buf);

	/* Get config block count and store it in the instance data */
	retval = rmi_read_block(fc->rmi_dev, query_base_addr + 7, buf,
			ARRAY_SIZE(buf));

	if (retval < 0) {
		dev_err(&fc->dev, "Couldn't read config block count from 0x%x, "
			"error=%d.\n", query_base_addr + CFG_BLK_CNT_OFF,
			retval);
		return retval;
	}
	batohs(&f34->configblockcount, buf);
#if 1
	f34->status = STATUS_IDLE;
#endif

//Go into bootloader mode
#if 0 //def SYNA
    int error = 0;
    error = rmi_enable_program(fc);
    if (error < 0) {
        printk(KERN_ERR "%s : Disable program status fail (%d)\n", __func__, error);
    }

	struct rmi_driver_data *driver_data = NULL;
	rmi_dev = fc->rmi_dev;
	driver_data = rmi_get_driverdata(rmi_dev);

    u16 base_address = driver_data->f01_container->fd.query_base_addr;
    u16 offset = 11;
    //f01_base
    retval = rmi_read_block(rmi_dev,
    base_address + offset,
    f34->product_id, ARRAY_SIZE(f34->product_id));

    dev_err(&fc->dev, "%s : [SYNA] F01 Query base 0x%X\n", __func__, base_address);

    if (retval < 0) {
        dev_err(&fc->dev, "Failed to read Bootloader product ID.\n");
        return retval;
    }
    f34->product_id[RMI_PRODUCT_ID_LENGTH] = '\0';

    dev_err(&fc->dev, "[SYNA] Product ID 1: %s\n", &f34->product_id);

    error = rmi_disable_program(fc);
    mdelay(500);    //fixme
    dev_err(&fc->dev, "%s : [SYNA] Disable flash programming\r\n", __func__);

    if (error < 0) {
        printk(KERN_ERR "%s : Disable program status fail\r\n", __func__);
    }
    //leave bootloader mode
#endif

	return 0;
}

static int rmi_f34_create_sysfs(struct rmi_function_container *fc)
{
	int attr_count = 0;
	int rc;

	dev_dbg(&fc->dev, "Creating sysfs files.");

	/* We need a sysfs file for the image/config block to write or read.
	 * Set up sysfs bin file for binary data block. Since the image is
	 * already in our format there is no need to convert the data for
	 * endianess. */
	rc = sysfs_create_bin_file(&fc->dev.kobj,
				&dev_attr_data);
	if (rc < 0) {
		dev_err(&fc->dev, "Failed to create sysfs file for F34 data "
		     "(error = %d).\n", rc);
		return -ENODEV;
	}

	/* Set up sysfs device attributes. */
	for (attr_count = 0; attr_count < ARRAY_SIZE(attrs); attr_count++) {
		if (sysfs_create_file
		    (&fc->dev.kobj, &attrs[attr_count].attr) < 0) {
			dev_err(&fc->dev, "Failed to create sysfs file for %s.",
			     attrs[attr_count].attr.name);
			rc = -ENODEV;
			goto err_remove_sysfs;
		}
	}

	return 0;

err_remove_sysfs:
	sysfs_remove_bin_file(&fc->dev.kobj, &dev_attr_data);

	for (attr_count--; attr_count >= 0; attr_count--)
		sysfs_remove_file(&fc->dev.kobj,
				  &attrs[attr_count].attr);
	return rc;
}

static int rmi_f34_config(struct rmi_function_container *fc)
{
	/* for this function we should do nothing here */
	return 0;
}


static int rmi_f34_reset(struct rmi_function_container *fc)
{
	struct  rmi_fn_34_data  *instance_data = fc->data;

	instance_data->status = ECONNRESET;

	return 0;
}

static void rmi_f34_remove(struct rmi_function_container *fc)
{
	int attr_count;

	sysfs_remove_bin_file(&fc->dev.kobj,
						  &dev_attr_data);

	for (attr_count = 0; attr_count < ARRAY_SIZE(attrs); attr_count++)
		sysfs_remove_file(&fc->dev.kobj,
				  &attrs[attr_count].attr);

	rmi_f34_free_memory(fc);
}

static int f34_read_status(struct rmi_function_container *fc)
{
	struct rmi_fn_34_data *instance_data = fc->data;
	u16 data_base_addr = fc->fd.data_base_addr;
	u8 status;
	int retval;

	if (instance_data->status == ECONNRESET)
		return instance_data->status;

	/* Read the Fn $34 status from F34_Flash_Data3 to see the previous
	 * commands status. F34_Flash_Data3 will be the address after the
	 * 2 block number registers plus blocksize Data registers.
	 *  inform user space - through a sysfs param. */
	retval = rmi_read(fc->rmi_dev,
			  data_base_addr + instance_data->blocksize +
			  BLK_NUM_OFF, &status);

	if (retval < 0) {
		dev_err(&fc->dev, "Could not read status from 0x%x\n",
		       data_base_addr + instance_data->blocksize + BLK_NUM_OFF);
		status = 0xff;	/* failure */
	}

	/* set a sysfs value that the user mode can read - only
	 * upper 4 bits are the status. successful is $80, anything
	 * else is failure */
	instance_data->status = status & 0xf0;

	/* put mode into Flash Prog Mode when we successfully do
	 * an Enable Flash Prog cmd. */
	if ((instance_data->status == STATUS_IDLE) &&
		(instance_data->cmd == ENABLE_FLASH_PROG))
		instance_data->inflashprogmode = true;

	return retval;
}

int rmi_f34_attention(struct rmi_function_container *fc, u8 *irq_bits)
{
    #if 0
    dev_err(&fc->dev, "[SYNA] rmi_f34_attention is asserted\n");
    #endif
    int retval;
	struct rmi_fn_34_data *data = fc->data;

	mutex_lock(&data->attn_mutex);
	retval = f34_read_status(fc);
	mutex_unlock(&data->attn_mutex);
	return retval;
}

static struct rmi_function_handler function_handler = {
	.func = 0x34,
	.init = rmi_f34_init,
	.config = rmi_f34_config,
	.reset = rmi_f34_reset,
	.attention = rmi_f34_attention,
	.remove = rmi_f34_remove
};

static ssize_t rmi_fn_34_bootloaderid_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct rmi_function_container *fc;
	struct rmi_fn_34_data *instance_data;

	fc = to_rmi_function_container(dev);
	instance_data = fc->data;

	return snprintf(buf, PAGE_SIZE, "%u\n", instance_data->bootloaderid);
}

static ssize_t rmi_fn_34_bootloaderid_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf,
					size_t count)
{
	int error;
	unsigned long val;
	unsigned char data[2];
	struct rmi_function_container *fc;
	struct rmi_fn_34_data *instance_data;
	u16 data_base_addr;

	fc = to_rmi_function_container(dev);
	instance_data = fc->data;

	/* need to convert the string data to an actual value */
	error = strict_strtoul(buf, 10, &val);

	if (error)
		return error;

	instance_data->bootloaderid = val;

	/* Write the Bootloader ID key data back to the first two Block
	 * Data registers (F34_Flash_Data2.0 and F34_Flash_Data2.1). */
	hstoba(data, (unsigned short)val);
	data_base_addr = fc->fd.data_base_addr;

	error = rmi_write_block(fc->rmi_dev,
				data_base_addr + BLK_NUM_OFF,
				data,
				ARRAY_SIZE(data));

	if (error < 0) {
		dev_err(dev, "%s : Could not write bootloader id to 0x%x\n",
		       __func__, data_base_addr + BLK_NUM_OFF);
		return error;
	}

	return count;
}

static ssize_t rmi_fn_34_blocksize_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct rmi_function_container *fc;
	struct rmi_fn_34_data *instance_data;

	fc = to_rmi_function_container(dev);
	instance_data = fc->data;

	return snprintf(buf, PAGE_SIZE, "%u\n", instance_data->blocksize);
}

static ssize_t rmi_fn_34_imageblockcount_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct rmi_function_container *fc;
	struct rmi_fn_34_data *instance_data;

	fc = to_rmi_function_container(dev);
	instance_data = fc->data;

	return snprintf(buf, PAGE_SIZE, "%u\n",
			instance_data->imageblockcount);
}

static ssize_t rmi_fn_34_configblockcount_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct rmi_function_container *fc;
	struct rmi_fn_34_data *instance_data;

	fc = to_rmi_function_container(dev);
	instance_data = fc->data;

	return snprintf(buf, PAGE_SIZE, "%u\n",
			instance_data->configblockcount);
}

static ssize_t rmi_fn_34_status_show(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	struct rmi_function_container *fc;
	struct rmi_fn_34_data *instance_data;
	int retval;

	fc = to_rmi_function_container(dev);
	instance_data = fc->data;

	mutex_lock(&instance_data->attn_mutex);
	retval = f34_read_status(fc);
	mutex_unlock(&instance_data->attn_mutex);

	if (retval < 0)
		return retval;

	return snprintf(buf, PAGE_SIZE, "%u\n", instance_data->status);
}


static ssize_t rmi_fn_34_status_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct rmi_function_container *fc;
	struct rmi_fn_34_data *instance_data;

	fc = to_rmi_function_container(dev);
	instance_data = fc->data;

	instance_data->status = 0;

	return 0;
}


static ssize_t rmi_fn_34_cmd_show(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	struct rmi_function_container *fc;
	struct rmi_fn_34_data *instance_data;

	fc = to_rmi_function_container(dev);
	instance_data = fc->data;

	return snprintf(buf, PAGE_SIZE, "%u\n", instance_data->cmd);
}

static ssize_t rmi_fn_34_cmd_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf,
				size_t count)
{
	struct rmi_function_container *fc;
	struct rmi_fn_34_data *instance_data;
	unsigned long val;
	u16 data_base_addr;
	int error;

	fc = to_rmi_function_container(dev);
	instance_data = fc->data;
	data_base_addr = fc->fd.data_base_addr;

	/* need to convert the string data to an actual value */
	error = strict_strtoul(buf, 10, &val);
	if (error)
		return error;

	/* make sure we are in Flash Prog mode for all cmds except the
	 * Enable Flash Programming cmd - otherwise we are in error */
	if ((val != ENABLE_FLASH_PROG) && !instance_data->inflashprogmode) {
		dev_err(dev, "%s: CANNOT SEND CMD %d TO SENSOR - "
			"NOT IN FLASH PROG MODE\n"
			, __func__, data_base_addr);
		return -EINVAL;
	}

	instance_data->cmd = val;

	/* Validate command value and (if necessary) write it to the command
	 * register.
	 */
	switch (instance_data->cmd) {
	case ENABLE_FLASH_PROG:
	case ERASE_ALL:
	case ERASE_CONFIG:
	case WRITE_FW_BLOCK:
	case READ_CONFIG_BLOCK:
	case WRITE_CONFIG_BLOCK:
		/* Reset the status to indicate we are in progress on a cmd. */
		/* The status will change when the ATTN interrupt happens
		   and the status of the cmd that was issued is read from
		   the F34_Flash_Data3 register - result should be 0x80 for
		   success - any other value indicates an error */

		/* Issue the command to the device. */
		error = rmi_write(fc->rmi_dev,
				data_base_addr + instance_data->blocksize +
				BLK_NUM_OFF, instance_data->cmd);

		if (error < 0) {
			dev_err(dev, "%s: Could not write command 0x%02x "
				"to 0x%04x\n", __func__, instance_data->cmd,
				data_base_addr + instance_data->blocksize +
				BLK_NUM_OFF);
			return error;
		}

		if (instance_data->cmd == ENABLE_FLASH_PROG)
			instance_data->inflashprogmode = true;

		/* set status to indicate we are in progress */
		instance_data->status = STATUS_IN_PROGRESS;
		break;
	default:
		dev_dbg(dev, "%s: RMI4 function $34 - "
				"unknown command 0x%02lx.\n", __func__, val);
		count = -EINVAL;
		break;
	}

	return count;
}

static ssize_t rmi_fn_34_blocknum_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct rmi_function_container *fc;
	struct rmi_fn_34_data *instance_data;

	fc = to_rmi_function_container(dev);
	instance_data = fc->data;

	return snprintf(buf, PAGE_SIZE, "%u\n", instance_data->blocknum);
}

static ssize_t rmi_fn_34_blocknum_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf,
					size_t count)
{
	int error;
	unsigned long val;
	unsigned char data[2];
	struct rmi_function_container *fc;
	struct rmi_fn_34_data *instance_data;
	u16 data_base_addr;

	fc = to_rmi_function_container(dev);
	instance_data = fc->data;
	data_base_addr = fc->fd.data_base_addr;

	/* need to convert the string data to an actual value */
	error = strict_strtoul(buf, 10, &val);

	if (error)
		return error;

	instance_data->blocknum = val;

	/* Write the Block Number data back to the first two Block
	 * Data registers (F34_Flash_Data_0 and F34_Flash_Data_1). */
	hstoba(data, (unsigned short)val);

	error = rmi_write_block(fc->rmi_dev,
				data_base_addr,
				data,
				ARRAY_SIZE(data));

	if (error < 0) {
		dev_err(dev, "%s : Could not write block number %u to 0x%x\n",
		       __func__, instance_data->blocknum, data_base_addr);
		return error;
	}

	return count;
}

static ssize_t rmi_fn_34_rescanPDT_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{
	struct rmi_function_container *fc;
	struct rmi_fn_34_data *instance_data;
	struct rmi_device *rmi_dev;
	struct rmi_driver_data *driver_data;
	struct pdt_entry pdt_entry;
	bool fn01found = false;
	bool fn34found = false;
	unsigned int rescan;
	int irq_count = 0;
	int retval = 0;
	int i;

	/* Rescan of the PDT is needed since issuing the Flash Enable cmd
	 * the device registers for Fn$01 and Fn$34 moving around because
	 * of the change from Bootloader mode to Flash Programming mode
	 * may change to a different PDT with only Fn$01 and Fn$34 that
	 * could have addresses for query, control, data, command registers
	 * that differ from the PDT scan done at device initialization. */

	fc = to_rmi_function_container(dev);
	instance_data = fc->data;
	rmi_dev = fc->rmi_dev;
	driver_data = rmi_get_driverdata(rmi_dev);

	/* Make sure we are only in Flash Programming mode  - DON'T
	 * ALLOW THIS IN UI MODE. */
	if (instance_data->cmd != ENABLE_FLASH_PROG) {
		dev_err(dev, "%s: NOT IN FLASH PROG MODE - CAN'T RESCAN PDT.\n"
				, __func__);
		return -EINVAL;
	}

	/* The only good value to write to this is 1, we allow 0, but with
	 * no effect (this is consistent with the way the command bit works. */
	if (sscanf(buf, "%u", &rescan) != 1)
		return -EINVAL;
	if (rescan < 0 || rescan > 1)
		return -EINVAL;

	/* 0 has no effect, so we skip it entirely. */
	if (rescan) {
		/* rescan the PDT - filling in Fn01 and Fn34 addresses -
		 * this is only temporary - the device will need to be reset
		 * to return the PDT to the normal values. */

		/* mini-parse the PDT - we only have to get Fn$01 and Fn$34 and
		   since we are Flash Programming mode we only have page 0. */
		for (i = PDT_START_SCAN_LOCATION; i >= PDT_END_SCAN_LOCATION;
			i -= sizeof(pdt_entry)) {
			retval = rmi_read_block(rmi_dev, i, (u8 *)&pdt_entry,
					       sizeof(pdt_entry));
			if (retval != sizeof(pdt_entry)) {
				dev_err(dev, "%s: err frm rmi_read_block pdt "
					"entry data from PDT, "
					"error = %d.", __func__, retval);
				return retval;
			}

			if ((pdt_entry.function_number == 0x00) ||
				(pdt_entry.function_number == 0xff))
				break;

			dev_dbg(dev, "%s: Found F%.2X\n",
				__func__, pdt_entry.function_number);

			/* f01 found - just fill in the new addresses in
			 * the existing fc. */
			if (pdt_entry.function_number == 0x01) {
				struct rmi_function_container *f01_fc =
					driver_data->f01_container;
				fn01found = true;
				f01_fc->fd.query_base_addr =
					pdt_entry.query_base_addr;
				f01_fc->fd.command_base_addr =
				  pdt_entry.command_base_addr;
				f01_fc->fd.control_base_addr =
				  pdt_entry.control_base_addr;
				f01_fc->fd.data_base_addr =
				  pdt_entry.data_base_addr;
				f01_fc->fd.function_number =
				  pdt_entry.function_number;
				f01_fc->fd.interrupt_source_count =
				  pdt_entry.interrupt_source_count;
				f01_fc->num_of_irqs =
				  pdt_entry.interrupt_source_count;
				f01_fc->irq_pos = irq_count;

				irq_count += f01_fc->num_of_irqs;

				if (fn34found)
					break;
			}

			/* f34 found - just fill in the new addresses in
			 * the existing fc. */
			if (pdt_entry.function_number == 0x34) {
				fn34found = true;
				fc->fd.query_base_addr =
				  pdt_entry.query_base_addr;
				fc->fd.command_base_addr =
				  pdt_entry.command_base_addr;
				fc->fd.control_base_addr =
				  pdt_entry.control_base_addr;
				fc->fd.data_base_addr =
				  pdt_entry.data_base_addr;
				fc->fd.function_number =
				  pdt_entry.function_number;
				fc->fd.interrupt_source_count =
				  pdt_entry.interrupt_source_count;
				fc->num_of_irqs =
				  pdt_entry.interrupt_source_count;
				fc->irq_pos = irq_count;

				irq_count += fc->num_of_irqs;

				if (fn01found)
					break;
			}

		}

		if (!fn01found || !fn34found) {
			dev_err(dev, "%s: failed to find fn$01 or fn$34 trying "
				"to do rescan PDT.\n"
				, __func__);
			return -EINVAL;
		}
	}

	return count;
}

#ifdef KERNEL_VERSION_ABOVE_2_6_32
static ssize_t rmi_fn_34_data_read(struct file *data_file,
				struct kobject *kobj,
				struct bin_attribute *attributes,
				char *buf,
				loff_t pos,
				size_t count)
#else
static ssize_t rmi_fn_34_data_read(struct kobject *kobj,
				struct bin_attribute *attributes,
				char *buf,
				loff_t pos,
				size_t count)
#endif
{
	struct device *dev = container_of(kobj, struct device, kobj);
	struct rmi_function_container *fc;
	struct rmi_fn_34_data *instance_data;
	u16 data_base_addr;
	int error;

	fc = to_rmi_function_container(dev);
	instance_data = fc->data;

	data_base_addr = fc->fd.data_base_addr;

	if (count != instance_data->blocksize) {
		dev_err(dev,
			"%s : Incorrect F34 block size %d. "
			"Expected size %d.\n",
			__func__, count, instance_data->blocksize);
		return -EINVAL;
	}

	/* Read the data from flash into buf.  The app layer will be blocked
	 * at reading from the sysfs file.  When we return the count (or
	 * error if we fail) the app will resume. */
	error = rmi_read_block(fc->rmi_dev, data_base_addr + BLK_NUM_OFF,
			(unsigned char *)buf, count);

	if (error < 0) {
		dev_err(dev, "%s : Could not read data from 0x%04x\n",
		       __func__, data_base_addr + BLK_NUM_OFF);
		return error;
	}

	return count;
}

#ifdef KERNEL_VERSION_ABOVE_2_6_32
static ssize_t rmi_fn_34_data_write(struct file *data_file,
				struct kobject *kobj,
				struct bin_attribute *attributes,
				char *buf,
				loff_t pos,
				size_t count)
#else
static ssize_t rmi_fn_34_data_write(struct kobject *kobj,
				struct bin_attribute *attributes,
				char *buf,
				loff_t pos,
				size_t count)
#endif
{
	struct device *dev = container_of(kobj, struct device, kobj);
	struct rmi_function_container *fc;
	struct rmi_fn_34_data *instance_data;
	u16 data_base_addr;
	int error;

	fc = to_rmi_function_container(dev);
	instance_data = fc->data;

	data_base_addr = fc->fd.data_base_addr;

	/* Write the data from buf to flash. The app layer will be
	 * blocked at writing to the sysfs file.  When we return the
	 * count (or error if we fail) the app will resume. */

	if (count != instance_data->blocksize) {
		dev_err(dev,
			"%s : Incorrect F34 block size %d. "
			"Expected size %d.\n",
			__func__, count, instance_data->blocksize);
		return -EINVAL;
	}

	/* Write the data block - only if the count is non-zero  */
	if (count) {
		error = rmi_write_block(fc->rmi_dev,
				data_base_addr + BLK_NUM_OFF,
				(unsigned char *)buf,
				count);

		if (error < 0) {
			dev_err(dev, "%s : Could not write block data "
				"to 0x%x\n", __func__,
				data_base_addr + BLK_NUM_OFF);
			return error;
		}
	}

	return count;
}
#ifdef SYNA
#ifndef SAME_PATH
static ssize_t rmi_fn_34_productid_show(struct device *dev,
					  struct device_attribute *attr,
					  char *buf)
#else
static ssize_t firmware_tptype_show(struct kobject *kobj,
            struct kobj_attribute *attr,char *buf)
#endif
{
   #if 0
#ifdef S10_PRODUCT_S10
   struct rmi_fn_34_data *instance_data = NULL;
    struct rmi_device *rmi_dev = rmi_fc->rmi_dev;
    struct rmi_driver_data *driver_data = NULL;
    driver_data = rmi_get_driverdata(rmi_dev);
    //u8 irq_status;
    u16 data_base_addr = 0;
	int error = 0;
    u16 base_address;
    u16 offset = 11;
	
	instance_data = rmi_fc->data;
	//data_base_addr = rmi_fc->fd.data_base_addr;

    //enable flash program
    error = rmi_enable_program(rmi_fc);
    if (error < 0) {
        printk(KERN_ERR "%s : Enable program status fail (%d)\n", __func__, error);
    }

    //re-scan PDT
	error = rmi_fn_34_read_pdt(rmi_fc);
    printk(KERN_ERR "[SYNA] 6 %s\n", __func__);
	if (error < 0) {
		printk(KERN_ERR "%s : Read PDT fail\r\n", __func__);
		return -EINVAL;
	}

    //f01_base
    base_address = driver_data->f01_container->fd.query_base_addr;
    printk(KERN_ERR "[SYNA] F$01 query base address 0x%X\n", base_address);
    error = rmi_read_block(rmi_dev,
    base_address + offset,
    instance_data->product_id, ARRAY_SIZE(instance_data->product_id));

    dev_err(&rmi_fc->dev, "%s : [SYNA] F01 Query base 0x%X\n", __func__, base_address);

    if (error < 0) {
        dev_err(&rmi_fc->dev, "Failed to read Bootloader product ID.\n");
        return error;
    }
    instance_data->product_id[RMI_PRODUCT_ID_LENGTH] = '\0';

    dev_err(&rmi_fc->dev, "[SYNA] Product ID: %s\n", &instance_data->product_id);

    error = rmi_disable_program(rmi_fc);
    dev_err(&rmi_fc->dev, "%s : [SYNA] Disable flash programming\r\n", __func__);

    if (error < 0) {
        printk(KERN_ERR "%s : Disable program status fail\r\n", __func__);
    }
	#endif
	struct rmi_device *rmi_dev = NULL;
	struct rmi_driver_data *driver_data = NULL;
	int error = 0;
	rmi_dev = rmi_fc->rmi_dev;
	driver_data = rmi_get_driverdata(rmi_dev);
	return sprintf(buf, "%s",driver_data->product_id) ;
  //return snprintf(buf, PAGE_SIZE, "%s\n", instance_data->product_id);
  //return snprintf(buf, PAGE_SIZE, "%s", instance_data->product_id);
#else
	struct rmi_device *rmi_dev = NULL;
	struct rmi_driver_data *driver_data = NULL;
	int error = 0;
	rmi_dev = rmi_fc->rmi_dev;
	driver_data = rmi_get_driverdata(rmi_dev);
	return sprintf(buf, "%s",driver_data->product_id) ;
#endif
}
#endif
static ssize_t rmi_fn_34_read_pdt(struct rmi_function_container *fc)
{
	struct rmi_fn_34_data *instance_data = fc->data;
	struct rmi_device *rmi_dev = NULL;
	struct rmi_driver_data *driver_data = NULL;
	struct pdt_entry pdt_entry;
	bool fn01found = false;
	bool fn34found = false;
	int irq_count = 0;
	int retval = 0;
	int i;

	/* Rescan of the PDT is needed since issuing the Flash Enable cmd
	 * the device registers for Fn$01 and Fn$34 moving around because
	 * of the change from Bootloader mode to Flash Programming mode
	 * may change to a different PDT with only Fn$01 and Fn$34 that
	 * could have addresses for query, control, data, command registers
	 * that differ from the PDT scan done at device initialization. */

	instance_data = fc->data;
	rmi_dev = fc->rmi_dev;
	driver_data = rmi_get_driverdata(rmi_dev);

	/* mini-parse the PDT - we only have to get Fn$01 and Fn$34 and
	   since we are Flash Programming mode we only have page 0. */
	for (i = PDT_START_SCAN_LOCATION; i >= PDT_END_SCAN_LOCATION;
		i -= sizeof(pdt_entry)) {
		retval = rmi_read_block(rmi_dev, i, (u8 *)&pdt_entry,
				       sizeof(pdt_entry));
		if (retval != sizeof(pdt_entry)) {
			printk(KERN_ERR "%s: err frm rmi_read_block pdt "
				"entry data from PDT, "
				"error = %d.", __func__, retval);
			return retval;
		}

		if ((pdt_entry.function_number == 0x00) ||
			(pdt_entry.function_number == 0xff))
			break;

		printk("%s: Found F%.2X\n",
			__func__, pdt_entry.function_number);

		/* f01 found - just fill in the new addresses in
		 * the existing fc. */
		if (pdt_entry.function_number == 0x01) {
			struct rmi_function_container *f01_fc =
				driver_data->f01_container;
			fn01found = true;
			f01_fc->fd.query_base_addr =
				pdt_entry.query_base_addr;
			f01_fc->fd.command_base_addr =
			  pdt_entry.command_base_addr;
			f01_fc->fd.control_base_addr =
			  pdt_entry.control_base_addr;
			f01_fc->fd.data_base_addr =
			  pdt_entry.data_base_addr;
			f01_fc->fd.function_number =
			  pdt_entry.function_number;
			f01_fc->fd.interrupt_source_count =
			  pdt_entry.interrupt_source_count;
			f01_fc->num_of_irqs =
			  pdt_entry.interrupt_source_count;
			f01_fc->irq_pos = irq_count;

			irq_count += f01_fc->num_of_irqs;

            if (fn34found)
				break;
		}

		/* f34 found - just fill in the new addresses in
		 * the existing fc. */
		if (pdt_entry.function_number == 0x34) {
			fn34found = true;
			fc->fd.query_base_addr =
			  pdt_entry.query_base_addr;
			fc->fd.command_base_addr =
			  pdt_entry.command_base_addr;
			fc->fd.control_base_addr =
			  pdt_entry.control_base_addr;
			fc->fd.data_base_addr =
			  pdt_entry.data_base_addr;
			fc->fd.function_number =
			  pdt_entry.function_number;
			fc->fd.interrupt_source_count =
			  pdt_entry.interrupt_source_count;
			fc->num_of_irqs =
			  pdt_entry.interrupt_source_count;
			fc->irq_pos = irq_count;

			irq_count += fc->num_of_irqs;

			if (fn01found)
				break;
		}

	}

	if (!fn01found || !fn34found) {
		printk(KERN_ERR "%s: failed to find fn$01 or fn$34 trying "
			"to do rescan PDT.\n"
			, __func__);
		return -EINVAL;
	}

	return 0;
}

static int __init rmi_f34_module_init(void)
{
	int error;

	error = rmi_register_function_driver(&function_handler);
	if (error < 0) {
		pr_err("%s : register failed !\n", __func__);
		return error;
	}

	return 0;
}

static void rmi_f34_module_exit(void)
{
	rmi_unregister_function_driver(&function_handler);
}

module_init(rmi_f34_module_init);
module_exit(rmi_f34_module_exit);

MODULE_AUTHOR("William Manson <wmanson@synaptics.com");
MODULE_DESCRIPTION("RMI F34 module");
MODULE_LICENSE("GPL");

