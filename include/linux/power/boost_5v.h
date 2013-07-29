/*include/linux/power/boost_5v.h - platform data structure*/
#ifndef _BOOST_5V_H_
#define _BOOST_5V_H_

#include <linux/gpio.h>
#include <linux/mutex.h>

struct boost_5v_platdata{
	char io_block_name[20];
	int switch_boost_gpio;
	int (*io_mux_block_init)(struct boost_5v_platdata* pdata);
	int (*io_mux_block_exit)(struct boost_5v_platdata* pdata);
	int (*init_gpio)(unsigned gpio);
	void (*exit_gpio)(unsigned gpio);
};

struct boost_5v_device{
	int gpio_num;
	struct spinlock power_lock;
	int use_gpio_moduel_num;

	struct kobject *kobj_boost;
};
int request_5v_boost(int on,int bit_map);
enum Boost_5v_request_module
{
	BOOST_5V_USB      = 0x1 << 0,
	BOOST_5V_AUDIO    = 0x1 << 1,
	BOOST_5V_TP       = 0x1 << 2,
	BOOST_5V_SELF     = 0x1 << 3,	
	BOOST_5V_TEST     = 0x1 << 7,	
};

#define BOOST_5V_CHECK(v)   ((BOOST_5V_USB==v) \
							||(BOOST_5V_AUDIO==v) \
							||(BOOST_5V_TP==v) \
							||(BOOST_5V_SELF==v) \
							||(BOOST_5V_TEST==v))
#endif




