#ifndef __GPIO_I2C_ADPT_H
#define __GPIO_I2C_ADPT_H
#include <linux/i2c.h>
#include <linux/mutex.h>

#define GPIO_I2C_ACK			1
#define GPIO_I2C_NACK			0
#define GPIO_I2C_SUCCESS		1
#define GPIO_I2C_FAIL			0

struct gpio_i2c_dev {
	struct device 			*dev;
	struct mutex 			mlock;
	struct i2c_adapter 	adap_gpio;
	struct gpio_i2c_adpt_platform_data	*pdata;
};

struct gpio_i2c_adpt_platform_data {
	int dely_usec;
	unsigned int gpio_i2c_clk;
	unsigned int gpio_i2c_dat;
};

#define GPIO_I2C_ADTP_NAME	"adpt_gpio_i2c"
#define GPIO_I2C_SCL		(15)
#define GPIO_I2C_SDA		(14)

#endif
