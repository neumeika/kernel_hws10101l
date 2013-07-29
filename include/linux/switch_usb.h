/*
 *  Switch_usb class driver
 *
 * Copyright (C) 2011 Google, Inc.
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

#ifndef __LINUX_SWITCH_USB_H__
#define __LINUX_SWITCH_USB_H__

#define GPIO_HI		                        1
#define	GPIO_LOW	                        0

#define USB_TO_AP		                0
#define USB_TO_MODEM	                        1
#define USB_OFF		                        2


struct usb_switch_platform_data {
	const char      *name;
	unsigned        usw_en_gpio;
	unsigned        usw_ctrl_gpio;
	unsigned        usw_int_gpio;
	int		(*wait_for_cp_ready) (void);
	int		(*simul_cp_usb_plug) (int direction);
	unsigned long   irq_flags;
};

struct switch_usb_dev {
	const char	*name;
	struct device	*dev;
	int		state;
	struct usb_switch_platform_data *pdata;
};

struct usb_switch_data {
	struct switch_usb_dev   sdev;
	unsigned                usw_en_gpio;
	unsigned                usw_ctrl_gpio;
	unsigned                usw_int_gpio;
	int                     irq;
	struct work_struct      work;
};


extern int switch_usb_dev_register(struct switch_usb_dev *sdev);
extern void switch_usb_dev_unregister(struct switch_usb_dev *sdev);
extern int  switch_usb_set_state(struct switch_usb_dev *sdev, int state);

#endif /* __LINUX_SWITCH_H__ */
