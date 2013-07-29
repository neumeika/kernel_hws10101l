#ifndef _BALONG_POWER_H
#define _BALONG_POWER_H

#include <linux/platform_device.h>
#include <linux/gpio.h>

enum balong_gpio_index {
	BALONG_RESET,
	BALONG_POWER_ON,
	BALONG_PMU_RESET,
	BALONG_HOST_ACTIVE,
	BALONG_HOST_WAKEUP,
	BALONG_SLAVE_WAKEUP,
	BALONG_POWER_IND,
	BALONG_RESET_IND,	
#ifdef CONFIG_BALONG_FLASHLESS_POWER
    BALONG_SUSPEND_REQUEST,
    BALONG_SLAVE_ACTIVE,
#endif
	
	BALONG_GPIO_NUM
};

#if defined(CONFIG_BALONG_FLASHLESS_POWER) || defined(CONFIG_BALONG_POWER)
struct balong_reset_ind_reg {
    resource_size_t base_addr;
    resource_size_t gpioic;
};
#endif

struct balong_power_plat_data {
    bool                    flashless;
    struct gpio             gpios[BALONG_GPIO_NUM];
    struct platform_device* echi_device;
#if defined(CONFIG_BALONG_FLASHLESS_POWER) || defined(CONFIG_BALONG_POWER)
    char                   *block_name;
    struct balong_reset_ind_reg *reset_reg;
#endif
};


/* The state that can be set through FS interface */
#define POWER_SET_DEBUGOFF     	0    /* Only turn off the modem, for debug USE */
#define POWER_SET_DEBUGON      	1    /* Only turn on the modem, for debug USE */
#define POWER_SET_OFF       	2    
#define POWER_SET_ON       	3
#define POWER_SET_HSIC_RESET     4   /* when finish download modem image reset HSIC */
#define POWER_SET_DL_FINISH     4
#define POWER_SET_ACTIVE	5

/* For usb to call for bus suspend/resume */
extern void balong_power_runtime_idle(void);
extern void balong_power_runtime_resume(void);


#endif /*_balong_POWER_H*/

