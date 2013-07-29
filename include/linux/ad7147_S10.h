#ifndef __AD7147_H__
#define __AD7147_H__

#include <linux/regulator/consumer.h>
#include <linux/i2c.h>

struct ad7147_data {
	struct i2c_client *client;
	atomic_t sensor_status;
	struct regulator* ldo15_3v3;   
	struct iomux_block  *piomux_block;
	struct block_config *pblock_config;
	struct mutex mutex;
};

struct ad7147_platform_data {
	void (*ad7147_gpio_config) (int );
	u32 ad7147_int_gpio;
};    

#define AD7147_NV_NUMBER      301
#define AD7147_NV_NAME        "TSAR"
#define AD7147_NV_VALID_SIZE  4

#endif
