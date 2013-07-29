#ifndef _K3V2_GPIO_KEY_H_
#define _K3V2_GPIO_KEY_H_

struct s10_gpio_key_platdata {
    int vol_up;
    int vol_down;
	int (*init) (void);
    int (*exit) (void);
};

#endif
