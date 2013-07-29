/*include/linux/power/micro_usb_power_control.h - platform data structure*/
#ifndef _MICRO_USB_POWER_CONTROL_H_
#define _MICRO_USB_POWER_CONTROL_H_

#include <linux/gpio.h>

#define USB_CURRENT_WARNING_NAME  "usb_current_warning"
#define USB_CURRENT_WARNING_WQ_NAME  "usb_current_warning_wq"
#define USB_ID_DETECT_NAME        "usb_id_detect"

struct micro_usb_power_control_platform_data{
	char io_block_name[20];
	int usb_ap5v_en_gpio;
	int modem_usb_control_gpio;
	int usb_current_warning_gpio;
	int irq_type;
	int usb_id_detect_gpio;
	int usb_id_detect_irq_type;
	int usb_vbus5v_en_gpio;
	int (*io_mux_block_init)(struct micro_usb_power_control_platform_data* pdata);
	int (*io_mux_block_exit)(struct micro_usb_power_control_platform_data* pdata);
	int (*init_gpio)(void);
	void (*exit_gpio)(void);
};

struct micro_usb_power_control_device{
	int ap5v_en_gpio;
	int modem_usb_ctrl_gpio;
	struct work_struct  current_warning_work;
	char *current_warning_name;
	int usb_current_warning_irq;
	struct work_struct  usb_id_detect_work;
	char *usb_id_detect_name;
	int usb_id_detect_irq;
	int out5v_en_gpio;
	struct workqueue_struct *micro_usb_power_wq;
	struct device dev;
};
void open_usb_ap5v(void);
void close_usb_ap5v(void);
void open_usb_out5v(void);
void close_usb_out5v(void);
#endif




