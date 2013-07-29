/*
 *  mma8452.c - Linux kernel modules for 3-Axis Orientation/Motion
 *  Detection Sensor
 *
 *  Copyright (C) 2010-2011 Freescale Semiconductor, Inc. All Rights Reserved.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */
/*############################################################
  History
  Problem NO.         Name        Time         Reason

############################################################*/

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/pm.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/err.h>
#include <linux/input-polldev.h>
#include "gs_mma8452.h"
#include <linux/board_sensors.h>

#include	<mach/gpio.h>
#include	<asm/io.h>
#include	<linux/mux.h>
#define GPIO_BOLCK_NAME "block_gsensor"
struct iomux_block *gpio_block;
struct block_config *gpio_block_config;
static struct mma8452_status mma_status;
static struct input_polled_dev *mma8452_idev;
static struct i2c_client *mma8452_i2c_client;
struct mutex lock;
static int16_t accl8452_data[3] = {0, 0, 0};
static int ACCHAL[8][3][3] = {
	{{ 0, -1,  0}, { 1,  0,	0}, {0, 0,	1},},
	{{-1,  0,  0}, { 0, -1,	0}, {0, 0,	1},},
	{{ 0,  1,  0}, {-1,  0,	0}, {0, 0,	1},},
	{{ 1,  0,  0}, { 0,  1,	0}, {0, 0,	1},},

	{{ 0, -1,  0}, {-1,  0,	0}, {0, 0,  -1},},
	{{-1,  0,  0}, { 0,  1,	0}, {0, 0,  -1},},
	{{ 0,  1,  0}, { 1,  0,	0}, {0, 0,  -1},},
	{{ 1,  0,  0}, { 0, -1,	0}, {0, 0,  -1},},
};

static DEFINE_MUTEX(mma8452_lock);
static int mma8452_adjust_position(short *x, short *y, short *z)
{
	short rawdata[3], data[3];
	int i, j;
	int position = mma_status.position ;
	if (position < 0 || position > 7)
		position = 0;
	rawdata[0] = *x ; rawdata[1] = *y ; rawdata[2] = *z ;
	for (i = 0; i < 3 ; i++) {
		data[i] = 0;
		for (j = 0; j < 3; j++) {
			data[i] += rawdata[j] * ACCHAL[position][i][j];
		}
	}
	*x = data[0];
	*y = data[1];
	*z = data[2];
	return 0;
}

static int mma8452_change_mode(struct i2c_client *client, int mode)
{
	int result;

	/* change ODR to 200HZ */
	mma_status.ctl_reg1 = 0x11;
	result = i2c_smbus_write_byte_data(client, MMA8452_CTRL_REG1, mma_status.ctl_reg1);
	if (result < 0)
		goto out;

	mma_status.mode = mode;
	result = i2c_smbus_write_byte_data(client, MMA8452_XYZ_DATA_CFG, mma_status.mode);
	if (result < 0)
		goto out;
	mma_status.active = MMA_STANDBY;
	msleep(MODE_CHANGE_DELAY_MS);

	return 0;
out:
	dev_err(&client->dev, "error when init mma8452:(%d)", result);
	return result;
}

/***************************************************************
*
* read sensor data from mma8452
*
***************************************************************/
static int mma8452_read_data(short *x, short *y, short *z)
{

	u8 tmp_data[MMA8452_BUF_SIZE];
	int ret;

	ret = i2c_smbus_read_i2c_block_data(mma8452_i2c_client,
					    MMA8452_OUT_X_MSB, MMA8452_BUF_SIZE, tmp_data);
	if (ret < MMA8452_BUF_SIZE) {
		dev_err(&mma8452_i2c_client->dev, "i2c block read failed\n");
		return -EIO;
	}

	*x = ((tmp_data[0] << 8) & 0xff00) | tmp_data[1];
	*y = ((tmp_data[2] << 8) & 0xff00) | tmp_data[3];
	*z = ((tmp_data[4] << 8) & 0xff00) | tmp_data[5];

	*x = (short)(*x) >> 4;
	*y = (short)(*y) >> 4;
	*z = (short)(*z) >> 4;


	if (mma_status.mode == MODE_4G) {
		(*x) = (*x) << 1;
		(*y) = (*y) << 1;
		(*z) = (*z) << 1;
	} else if (mma_status.mode == MODE_8G) {
		(*x) = (*x) << 2;
		(*y) = (*y) << 2;
		(*z) = (*z) << 2;
	}

	return 0;
}

static void report_abs(void)
{
	short x, y, z;
	int result;
	int retry = MMA8452_RETRY_COUNT;

	mutex_lock(&mma8452_lock);
	if (mma_status.active == MMA_STANDBY)
		goto out;

	/* wait for the data ready */
	do {
		result = i2c_smbus_read_byte_data(mma8452_i2c_client, MMA8452_STATUS);
		if (result & MMA8452_STATUS_ZYXDR) {
			break;
		} else {
			retry--;
			msleep(1);
		}
	} while (retry > 0);

	if (retry == 0) {
		dev_err(&mma8452_i2c_client->dev, "%s retry over %d\n", __func__, MMA8452_RETRY_COUNT);
		goto out;
	}

	if (mma8452_read_data(&x, &y, &z) != 0)
		goto out;

	mma8452_adjust_position(&x, &y, &z);

	accl8452_data[0] = x;
	accl8452_data[1] = y;
	accl8452_data[2] = z;

	input_report_abs(mma8452_idev->input, ABS_X, x);
	input_report_abs(mma8452_idev->input, ABS_Y, y);
	input_report_abs(mma8452_idev->input, ABS_Z, z);
	input_sync(mma8452_idev->input);

out:
	mutex_unlock(&mma8452_lock);
}

static void mma8452_dev_poll(struct input_polled_dev *dev)
{
	report_abs();
}

static ssize_t mma8452_enable_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct i2c_client *client;
	u8 val;
	int enable;

	mutex_lock(&mma8452_lock);
	client = mma8452_i2c_client;
	val = i2c_smbus_read_byte_data(client, MMA8452_CTRL_REG1);
	if ((val & 0x01) && mma_status.active == MMA_ACTIVED)
		enable = 1;
	else
		enable = 0;
	mutex_unlock(&mma8452_lock);
	return sprintf(buf, "%d\n", enable);
}

static ssize_t mma8452_enable_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t count)
{
	struct i2c_client *client;
	int ret;
	unsigned long enable;
	u8 val = 0;
	enable = simple_strtoul(buf, NULL, 10);
	printk("%s: enable=%ld\n", __func__, enable);
	mutex_lock(&mma8452_lock);
	client = mma8452_i2c_client;
	enable = (enable > 0) ? 1 : 0;
	if (enable && mma_status.active == MMA_STANDBY) {
		val = i2c_smbus_read_byte_data(client, MMA8452_CTRL_REG1);
		ret = i2c_smbus_write_byte_data(client, MMA8452_CTRL_REG1, val | 0x01);
		if (ret < 0) {
			dev_err(dev, "mma8452_enable_store enable write error.\n");
			goto err;
		}

		mma_status.active = MMA_ACTIVED;
		dev_dbg(dev, "mma enable setting active\n");

	} else if (enable == 0 && mma_status.active == MMA_ACTIVED) {
		val = i2c_smbus_read_byte_data(client, MMA8452_CTRL_REG1);
		ret = i2c_smbus_write_byte_data(client, MMA8452_CTRL_REG1, val & 0xFE);
		if (ret < 0) {
			dev_err(dev, "mma8452_enable_store disable write error.\n");
			goto err;
		}

		mma_status.active = MMA_STANDBY;
		dev_dbg(dev, "mma enable setting inactive\n");
	}

err:
	mutex_unlock(&mma8452_lock);
	return count;
}
static ssize_t mma8452_position_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int position = 0;
	mutex_lock(&mma8452_lock);
	position = mma_status.position ;
	mutex_unlock(&mma8452_lock);
	return sprintf(buf, "%d\n", position);
}

static ssize_t mma8452_position_store(struct device *dev, struct device_attribute *attr,
					const char *buf, size_t count)
{
	int  position;
	position = simple_strtoul(buf, NULL, 10);
	mutex_lock(&mma8452_lock);
	mma_status.position = position;
	mutex_unlock(&mma8452_lock);
	return count;
}

static ssize_t mma8452_get_polling_rate(struct device *dev,
				struct device_attribute *attr, char *buf)
{
#if 0
	int val;
	struct input_polled_dev *mma8452 = dev_get_drvdata(dev);
	mutex_lock(&lock);
	val = mma8452->poll_interval;
	mutex_unlock(&lock);
	return sprintf(buf, "%d\n", val);
#endif
	return 0;
}

static ssize_t mma8452_set_polling_rate(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	/*no interface to set the polling rate*/
	return size;
}

static ssize_t mma8452_get_accl_data(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	mutex_lock(&mma8452_lock);

	*((int16_t *)&buf[0]) = accl8452_data[0];
	*((int16_t *)&buf[2]) = accl8452_data[1];
	*((int16_t *)&buf[4]) = accl8452_data[2];

	mutex_unlock(&mma8452_lock);

	dev_dbg(dev, "%s: buf1=%d,buf2=%d,buf3=%d\n", __func__, buf[0], buf[1], buf[2]);

	return ACCL_DATA_SIZE;
}

static ssize_t mma8452_get_accl_info(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	ssize_t count;
	count = sprintf(buf, "FREESCALE MMA8452");
	return count;
}

static struct device_attribute attributes[] = {
		__ATTR(enable, S_IWUSR | S_IRUGO,
					mma8452_enable_show, mma8452_enable_store),
		__ATTR(position, S_IWUSR | S_IRUGO,
					mma8452_position_show, mma8452_position_store),
		__ATTR(pollrate_ms, 0664, mma8452_get_polling_rate, mma8452_set_polling_rate),
		__ATTR(accl_data, 0664, mma8452_get_accl_data, NULL),
		__ATTR(accl_info, 0664, mma8452_get_accl_info, NULL),
};

static int create_sysfs_interfaces(struct device *dev)
{
	int i;
	for (i = 0; i < ARRAY_SIZE(attributes); i++)
		if (device_create_file(dev, attributes + i))
			goto error;
	return 0;

error:
	for ( ; i >= 0; i--)
		device_remove_file(dev, attributes + i);
	dev_err(dev, "%s:Unable to create interface\n", __func__);
	return -1;
}

static int remove_sysfs_interfaces(struct device *dev)
{
	int i;
	for (i = 0; i < ARRAY_SIZE(attributes); i++)
		device_remove_file(dev, attributes + i);
	return 0;
}

static int mma8452_config_gpio(struct device *dev)
{
	int ret = 0;

	/* get gpio block*/
	gpio_block = iomux_get_block(GPIO_BOLCK_NAME);
	if (IS_ERR(gpio_block)) {
		dev_err(dev, "%s: failed to get gpio block\n", __func__);
		ret = -EINVAL;
		return ret;
	}

	/* get gpio block config*/
	gpio_block_config = iomux_get_blockconfig(GPIO_BOLCK_NAME);
	if (IS_ERR(gpio_block_config)) {
		dev_err(dev, "%s: failed to get gpio block config\n", __func__);
		ret = -EINVAL;
		goto err_block_config;
	}

	/* config gpio work mode*/
	ret = blockmux_set(gpio_block, gpio_block_config, LOWPOWER);
	if (ret) {
		dev_err(dev, "%s: failed to config gpio\n", __func__);
		ret = -EINVAL;
		goto err_mux_set;
	}

	return ret;

err_mux_set:
	if (gpio_block_config)
		gpio_block_config = NULL;
err_block_config:
	if (gpio_block)
		gpio_block = NULL;

	return ret;

}

static int __devinit mma8452_probe(struct i2c_client *client,
				   const struct i2c_device_id *id)
{
	int result, client_id;
	struct input_dev *idev;
	struct mma8452_acc_platform_data *devpd = client->dev.platform_data;

	if (!devpd) {
		dev_err(&client->dev, "No platfrom data!\n");
		result = -ENODEV;
		goto err_get_pdata_fail;
	}

	mma8452_i2c_client = client;

	if (!i2c_check_functionality(client->adapter,
			I2C_FUNC_SMBUS_BYTE | I2C_FUNC_SMBUS_BYTE_DATA)) {
		dev_err(&client->dev, "client not i2c capable\n");
		result = -ENODEV;
		goto err_out;
	}

	client_id = i2c_smbus_read_byte_data(client, MMA8452_WHO_AM_I);

	if (client_id != MMA8452_ID && client_id != MMA8452_ID) {
		dev_err(&client->dev,
			"read chip ID 0x%x is not equal to 0x%x or 0x%x!\n",
			client_id, MMA8452_ID, MMA8452_ID);
		result = -EINVAL;
		goto err_out;
	}

	dev_info(&client->dev, "Read mma8452 chip ok, ID is 0x%x\n", client_id);

	result =  i2c_smbus_write_byte_data(client, MMA8452_CTRL_REG3, 0x2);
	if (result < 0) {
		dev_err(&client->dev, "%s: failed to selects the polarity of the interrupt signal\n", __func__);
		goto err_out;
	}

	result = mma8452_config_gpio(&client->dev);
	if (result) {
		dev_err(&client->dev, "%s: failed to config gpio lowpower mode\n", __func__);
		goto err_out;
	}

	/* Initialize the MMA8452 chip */
	result = mma8452_change_mode(client, MODE_2G);
	if (result) {
		dev_err(&client->dev,
			"error when init mma8452 chip:(%d)\n", result);
		goto err_out;
	}

	mma8452_idev = input_allocate_polled_device();
	if (!mma8452_idev) {
		result = -ENOMEM;
		dev_err(&client->dev, "alloc poll device failed!\n");
		goto err_alloc_poll_device;
	}
	mma8452_idev->poll = mma8452_dev_poll;
	mma8452_idev->poll_interval = POLL_INTERVAL;
	mma8452_idev->poll_interval_min = POLL_INTERVAL_MIN;
	mma8452_idev->poll_interval_max = POLL_INTERVAL_MAX;
	idev = mma8452_idev->input;
	idev->name = ACCL_INPUT_DEV_NAME;
	idev->id.bustype = BUS_I2C;
	idev->evbit[0] = BIT_MASK(EV_ABS);

	input_set_abs_params(idev, ABS_X, -8192, 8191, INPUT_FUZZ, INPUT_FLAT);
	input_set_abs_params(idev, ABS_Y, -8192, 8191, INPUT_FUZZ, INPUT_FLAT);
	input_set_abs_params(idev, ABS_Z, -8192, 8191, INPUT_FUZZ, INPUT_FLAT);
	idev->dev.parent = &client->dev;
	result = input_register_polled_device(mma8452_idev);
	if (result) {
		dev_err(&client->dev, "register poll device failed!\n");
		goto err_register_polled_device;
	}

	result = create_sysfs_interfaces(&client->dev);
	if (result < 0) {
		dev_err(&client->dev, "create device file failed!\n");
		result = -EINVAL;
		goto err_create_sysfs;
	}

#ifdef CONFIG_S10_SENSORS_INPUT_INFO
	result = set_sensor_input(ACC, mma8452_idev->input->dev.kobj.name);
	if (result) {
		remove_sysfs_interfaces(&client->dev);
		dev_err(&client->dev, "%s set_sensor_input failed!\n", __func__);
		goto err_create_sysfs;
	}
#endif
	mma_status.position = devpd->config_mxc_mma_position;

	return 0;

err_create_sysfs:
err_register_polled_device:
	input_free_polled_device(mma8452_idev);
err_alloc_poll_device:
err_out:
	mma8452_i2c_client = NULL;
err_get_pdata_fail:
	return result;
}

static int mma8452_stop_chip(struct i2c_client *client)
{
	int ret = 0;

	if (mma_status.active == MMA_ACTIVED) {
		mma_status.ctl_reg1 = i2c_smbus_read_byte_data(client, MMA8452_CTRL_REG1);
		ret = i2c_smbus_write_byte_data(client, MMA8452_CTRL_REG1,
						mma_status.ctl_reg1 & 0xFE);
		if (ret < 0) {
			dev_err(&client->dev, "%s, i2c_smbus_write_byte_data failed!\n",
					__func__);
		}
	}

	return ret;
}

static int __devexit mma8452_remove(struct i2c_client *client)
{
	int ret;
	ret = mma8452_stop_chip(client);
	if (ret < 0)
		dev_err(&client->dev, "mma8452 stop failed\n");

	remove_sysfs_interfaces(&client->dev);
	input_free_polled_device(mma8452_idev);

	return ret;
}

#ifdef CONFIG_PM_SLEEP
static int mma8452_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	printk("[%s] +\n", __func__);
	mutex_lock(&mma8452_lock);
	if (mma8452_stop_chip(client))
		dev_err(&client->dev, "mma8452 stop failed\n");
	mutex_unlock(&mma8452_lock);
	printk("[%s] -\n", __func__);
	return 0;
}

static int mma8452_resume(struct device *dev)
{
	int ret = 0;
	struct i2c_client *client = to_i2c_client(dev);
	printk("[%s] +\n", __func__);
	mutex_lock(&mma8452_lock);
	if (mma_status.active == MMA_ACTIVED) {
		ret = i2c_smbus_write_byte_data(client, MMA8452_CTRL_REG1,
						mma_status.ctl_reg1);
		if (ret < 0) {
			dev_err(dev, "%s, i2c_smbus_write_byte_data failed!\n",
					__func__);
		}
	}
	mutex_unlock(&mma8452_lock);
	printk("[%s] -\n", __func__);
	return 0;
}
#endif

/* zkf55108 2011/11/3 add begin */
static void mma8452_shutdown(struct i2c_client *client)
{
	printk("[%s] +\n", __func__);

	if (mma8452_stop_chip(client))
		dev_err(&client->dev, "mma8452 stop failed\n");

	printk("[%s],-\n", __func__);

}
/* zkf55108 2011/11/3 add end */

static const struct i2c_device_id mma8452_id[] = {
	{MMA8452_ACC_DEV_NAME, 0},
};
MODULE_DEVICE_TABLE(i2c, mma8452_id);

static SIMPLE_DEV_PM_OPS(mma8452_pm_ops, mma8452_suspend, mma8452_resume);
static struct i2c_driver mma8452_driver = {
	.driver = {
		   .name = MMA8452_ACC_DEV_NAME,
		   .owner = THIS_MODULE,
		   .pm = &mma8452_pm_ops,
		   },
	.probe = mma8452_probe,
	.remove = __devexit_p(mma8452_remove),
	/* zkf55108 2011/11/3 add begin */
	.shutdown = mma8452_shutdown,
	/* zkf55108 2011/11/3 add end */
	.id_table = mma8452_id,
};

static int __init mma8452_init(void)
{
	return i2c_add_driver(&mma8452_driver);
}

static void __exit mma8452_exit(void)
{
	i2c_del_driver(&mma8452_driver);
}

module_init(mma8452_init);
module_exit(mma8452_exit);

MODULE_AUTHOR("Chen Gang <gang.chen@freescale.com>");
MODULE_DESCRIPTION("MMA8452 3-Axis Orientation/Motion Detection Sensor driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.3.2");
