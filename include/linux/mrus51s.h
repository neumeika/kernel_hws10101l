#ifndef __MRUS51S_H_
#define __MRUS51S_H_

#include <linux/wakelock.h>

struct mrus51s_platform_data_struct {
    int    (*gpio_config) (int mode);
    int    irq_gpio;

    struct wake_lock mrus_wake_lock;
    struct workqueue_struct  *mrus51s_wq;
    struct work_struct work;
    struct timer_list  timer_detect;//timer to schdule the task to send uevent;
    struct device    *dev;
};

#endif
