/*
 *  drivers/switch/switch_usb_class.c
 *
 * Copyright (C) 2008 Google, Inc.
 * Author: Jake.Chen
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
#include <linux/types.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/err.h>
#include <linux/switch_usb.h>
#include <mach/gpio.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/interrupt.h>

#include <linux/delay.h>
#include <linux/wait.h>
#include <linux/time.h>
#include <linux/wakelock.h>
#include <linux/usb/hiusb_android.h>

struct class *switch_usb_class;
static struct wake_lock usbsw_wakelock;

static ssize_t usb_state_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	struct switch_usb_dev *sdev = (struct switch_usb_dev *)
		dev_get_drvdata(dev);

	return sprintf(buf, "%d\n", sdev->state);
}

static ssize_t usb_state_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct switch_usb_dev *sdev = (struct switch_usb_dev *)
				dev_get_drvdata(dev);
	int state;

	sscanf(buf, "%d", &state);
	if (switch_usb_set_state(sdev, state) < 0) {
		dev_err(sdev->dev, "%s: switch_usb_set_state err\n", __func__);
		return -1;
	}

	return size;
}

/*
* usw_en_gpio          usw_ctrl_gpio
*   0                   0               USB OFF
*   0                   1               USB TO AP
*   1                   0               SB TO MODEM
*   1                   1               USB TO MHL
*/
int switch_usb_set_state(struct switch_usb_dev *sdev, int state)
{
	int ret = 0;
	struct usb_switch_platform_data *pdata = sdev->pdata;
	struct usb_switch_data *switch_data = (struct usb_switch_data *)sdev;
	BUG_ON(pdata == NULL);
	BUG_ON(switch_data == NULL);
#ifndef CONFIG_FSA880_I2C
	ret = gpio_request(pdata->usw_ctrl_gpio, "usb_sw_ctrl");
	if (ret < 0) {
		dev_err(sdev->dev, "%s: gpio_request %d return" 
			"err: %d.\n", __func__, pdata->usw_ctrl_gpio, ret);
		goto err_request_usw_ctrl_gpio;
	}

	ret = gpio_request(pdata->usw_en_gpio, "usb_sw_en");
	if (ret < 0) {
		dev_err(sdev->dev, "%s: gpio_request %d return" 
			"err: %d.\n", __func__, pdata->usw_en_gpio, ret);
		goto err_request_en_gpio;
	}
#endif
	if (sdev->state != state) {
		sdev->state = state;

		msleep(500);
		if ((state == USB_TO_MODEM)
			&& pdata && pdata->wait_for_cp_ready) {
			wake_lock(&usbsw_wakelock);
			if (pdata->wait_for_cp_ready() < 0) {
				ret = -EIO;
				goto exit_func;
			}
			/*if (hiusb_do_usb_disconnect() < 0) {
				dev_err(sdev->dev, "%s: hiusb_do_usb_disconnect"
				" return err\n", __func__);
			}*/
			msleep(20);
		}

		if ((state == USB_TO_AP) && pdata && pdata->simul_cp_usb_plug) {
			wake_unlock(&usbsw_wakelock);
			pdata->simul_cp_usb_plug(1);
			msleep(100);
		}
#ifndef CONFIG_FSA880_I2C
		ret = gpio_direction_output(pdata->usw_ctrl_gpio, GPIO_LOW);
		if (ret < 0) {
			dev_err(sdev->dev, "%s: gpio_direction_output %d return" 
				"err: %d.\n", __func__, pdata->usw_ctrl_gpio, ret);
			goto err_request_usw_ctrl_gpio_input;
		}

		ret = gpio_direction_output(pdata->usw_en_gpio, GPIO_LOW);
		if (ret < 0) {
			dev_err(sdev->dev, "%s: gpio_direction_output %d return" 
				"err: %d.\n", __func__, pdata->usw_en_gpio, ret);
			goto err_request_usw_en_gpio_input;
		}
#endif
		switch (state) {
		case USB_TO_AP:
			dev_info(sdev->dev, "%s: USB_TO_AP\n", __func__);
			msleep(1000);

#ifndef CONFIG_FSA880_I2C
			gpio_set_value(pdata->usw_ctrl_gpio, GPIO_HI);
			gpio_set_value(pdata->usw_en_gpio, GPIO_LOW);
#endif
			break;
		case USB_TO_MODEM:
			dev_info(sdev->dev, "%s: USB_TO_MODEM\n", __func__);
			msleep(1000);

			if (pdata && pdata->simul_cp_usb_plug)
				pdata->simul_cp_usb_plug(0);
			msleep(100);

#ifndef CONFIG_FSA880_I2C
			gpio_set_value(pdata->usw_ctrl_gpio, GPIO_LOW);
			gpio_set_value(pdata->usw_en_gpio, GPIO_HI);			
#endif

			enable_irq(switch_data->irq);
			break;
		case USB_OFF:
			dev_info(sdev->dev, "%s: USB_OFF\n", __func__);
#ifndef CONFIG_FSA880_I2C
			gpio_set_value(pdata->usw_en_gpio, GPIO_LOW);
			gpio_set_value(pdata->usw_ctrl_gpio, GPIO_LOW);
#endif
			break;
		default:
			dev_info(sdev->dev, "%s: state[%d] is overrun",
				__func__, sdev->state);
			ret = -1;
			break;
		}
	} else {
		dev_info(sdev->dev, "%s: swstate[%d] is not changed, new "
			"swstate[%d]\n", __func__, sdev->state, state);
		ret = -1;
	}
exit_func:
#ifndef CONFIG_FSA880_I2C
err_request_usw_en_gpio_input:
err_request_usw_ctrl_gpio_input:
	gpio_free(pdata->usw_en_gpio);
err_request_en_gpio:
	gpio_free(pdata->usw_ctrl_gpio);
err_request_usw_ctrl_gpio:
#endif
	return ret;
}
EXPORT_SYMBOL_GPL(switch_usb_set_state);

static DEVICE_ATTR(swstate, S_IRUGO | S_IWUSR, usb_state_show, usb_state_store);


static int create_switch_usb_class(void)
{
	if (!switch_usb_class) {
		switch_usb_class = class_create(THIS_MODULE, "usbswitch");
		if (IS_ERR(switch_usb_class))
			return PTR_ERR(switch_usb_class);
	}

	/* This wakelock will be used to arrest system sleeping when USB is in L0 state */
	wake_lock_init(&usbsw_wakelock, WAKE_LOCK_SUSPEND, "usb_switch");
	return 0;
}

int switch_usb_dev_register(struct switch_usb_dev *sdev)
{
	int ret;

	if (!switch_usb_class) {
		ret = create_switch_usb_class();
		if (ret < 0)
			return ret;
	}

	sdev->dev = device_create(switch_usb_class, NULL,
		MKDEV(0, 0), NULL, sdev->name);
	if (IS_ERR(sdev->dev))
		return PTR_ERR(sdev->dev);

	ret = device_create_file(sdev->dev, &dev_attr_swstate);
	if (ret < 0)
		goto err_create_file;

	dev_set_drvdata(sdev->dev, sdev);
	sdev->state = USB_TO_AP;
	return 0;

err_create_file:
	device_destroy(switch_usb_class, MKDEV(0, 0));
	dev_err(sdev->dev, "switch_usb: Failed to register driver %s\n",
		sdev->name);

	return ret;
}
EXPORT_SYMBOL_GPL(switch_usb_dev_register);

void switch_usb_dev_unregister(struct switch_usb_dev *sdev)
{
	device_remove_file(sdev->dev, &dev_attr_swstate);
	device_destroy(switch_usb_class, MKDEV(0, 0));
	dev_set_drvdata(sdev->dev, NULL);
}
EXPORT_SYMBOL_GPL(switch_usb_dev_unregister);

static int __init switch_usb_class_init(void)
{
	return create_switch_usb_class();
}

static void __exit switch_usb_class_exit(void)
{
	wake_lock_destroy(&usbsw_wakelock);
	class_destroy(switch_usb_class);
}

module_init(switch_usb_class_init);
module_exit(switch_usb_class_exit);

MODULE_AUTHOR("Jake.Chen");
MODULE_DESCRIPTION("Switch usb class driver");
MODULE_LICENSE("GPL");
