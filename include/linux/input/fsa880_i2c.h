/*
 * include/linux/input/fsa880_i2c.h - platform data structure
 *

 *
 */

#ifndef _LINUX_FSA880_I2C_H
#define _LINUX_FSA880_I2C_H
#include <linux/workqueue.h>
#include <linux/i2c.h>


#define FAIRCHILD_FSA880_NAME   "fairchild_fsa880"
#define FSA880_INIT_NAME  "fsa880-bc11"
#define FSA880_WQ_NAME  "fsa880_wq"

#define FAIRCHILD_FSA880_I2C_ADDR  0x25

#define  CURRENT_WARNING_NAME  "current_warning"

struct fairchild_fsa880 {
	struct i2c_client *client;
	struct work_struct  work;
	struct work_struct  current_warning_work;
	char *current_warning_name;
	struct workqueue_struct *fsa880_wq;
	int use_irq;
	
};

struct fairchild_fsa880_platform_data 
{
	int intr_gpio;
	int switch_out5v_gpio;
	int switch_ap5v_gpio;
	int switch_usb_id_gpio;
	int usb_current_warning_gpio;
	int balong_use_gpio;
	int	irq_type;      
	int current_warning_irq_type;
	char io_block_name[20];
	int (*io_mux_block_init)(struct fairchild_fsa880_platform_data* pdata);
	int (*io_mux_block_exit)(struct fairchild_fsa880_platform_data* pdata);
	int  (*init_gpio)(void);
	void (*exit_gpio)(void);
};

int fsa880_switch_ap_or_modem(int);


#endif /* _LINUX_FSA880_I2C_H */
