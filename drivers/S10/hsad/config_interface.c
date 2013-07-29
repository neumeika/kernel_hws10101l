
#include <linux/types.h>
#include <linux/gpio.h>
#include <linux/string.h>
#include <linux/mm.h>
#include <linux/slab.h>

#include <hsad/config_interface.h>
#include <hsad/configdata.h>
#include <hsad/config_mgr.h>
#include <hsad/config_general_struct.h>
#include <hsad/config_debugfs.h>

#define MAX_KEY_LENGTH 48
#define MAX_SENSOR_LAYOUT_LEN 10

bool product_type(char *pname)
{
	char product_name[MAX_KEY_LENGTH];
	memset(product_name, 0, sizeof(product_name));
	get_hw_config_string("product/name", product_name, MAX_SENSOR_LAYOUT_LEN, NULL);
	if (strstr(product_name, pname))
		return true;
	else
		return false;
}

int get_touchscreen_type(void)
{
	unsigned int type = 0;

	bool ret = get_hw_config_int("touchscreen/touchscreen_type", &type, NULL);
	HW_CONFIG_DEBUG("hsad: touchscreen_type = %d\n", type);
	if (ret == true) {
		return type;
	}

	return -1;
}

int get_usbphy_tune(void)
{
	unsigned int value = 0;

	bool ret = get_hw_config_int("usbphy/usbphy_type", &value, NULL);
	HW_CONFIG_DEBUG("hsad: usbphy_type = %d\n", value);
	if (ret == true) {
		return value;
	}

	return -1;
}

int get_board_type(void)
{
	unsigned int type = 0;

	bool ret = get_hw_config_int("board/board_type", &type, NULL);
	HW_CONFIG_DEBUG("hsad: board_type = %d\n", type);
	if (ret == true) {
		return type;
	}

	return -1;
}

int get_sensor_type(void)
{
	unsigned int type = 0;

	bool ret = get_hw_config_int("sensor/sensor_type", &type, NULL);
	HW_CONFIG_DEBUG("hsad: sensor_type = %d\n", type);
	if (ret == true) {
		return type;
	}

	return -1;
}

int get_sensor_timing_type(void)
{
	unsigned int type = 0;

	bool ret = get_hw_config_int("camera/camera_timing_type", &type, NULL);
	HW_CONFIG_DEBUG("hsad: camera_timing_type = %d\n", type);
	if (ret == true) {
		return type;
	}

	return -1;
}

int get_iomux_type(void)
{
	unsigned int type = 0;

	bool ret = get_hw_config_int("iomux/iomux_type", &type, NULL);
	HW_CONFIG_DEBUG("hsad: iomux_type = %d\n", type);
	if (ret == true) {
		return type;
	}

	return -1;
}

/* gpio get gpio struct */
#ifdef CONFIG_S10_GPIO_UNITE
struct gpio_config_type *get_gpio_config_table(void)
{
	struct board_id_general_struct *gpios_ptr = get_board_id_general_struct(GPIO_MODULE_NAME);
	struct gpio_config_type *gpio_ptr;

	if (NULL == gpios_ptr) {
		HW_CONFIG_DEBUG(" can not find  module:gpio\n");
		return NULL;
	}

	gpio_ptr = (struct gpio_config_type *)gpios_ptr->data_array.gpio_ptr;

    if (NULL != gpio_ptr) {
		return gpio_ptr;
	} else {
		HW_CONFIG_DEBUG(" return NULL\n");
		return NULL;
	}
}

/*gpio get number by name*/
int get_gpio_num_by_name(char *name)
{
    int min = 0;
    int max = NR_GPIO_IRQS - 1;
    int result;
    int new_cursor;
	struct gpio_config_type *gpio_ptr = get_gpio_config_table();

    if (NULL == gpio_ptr) {
		HW_CONFIG_DEBUG(" get gpio struct failed.\n");
		return -EFAULT;
    }

	while (min <= max) {
		new_cursor = (min+max)/2;

		if (!(strcmp((gpio_ptr + new_cursor)->name, ""))) {
			result = 1;
		} else {
			result = strcmp((gpio_ptr+new_cursor)->name, name);
		}

		if (0 == result) {
			/*found it, just return*/
			return (gpio_ptr+new_cursor)->gpio_number;
		} else if (result > 0) {
			/* key is smaller, update max*/
			max = new_cursor-1;
		} else if (result < 0) {
			/* key is bigger, update min*/
			min = new_cursor+1;
		}
	}

	return -EFAULT;
}

struct pm_gpio_cfg_t *get_pm_gpio_config_table(void)
{
	struct board_id_general_struct *pm_gpios_ptr = get_board_id_general_struct(PM_GPIO_MODULE_NAME);
	struct pm_gpio_cfg_t *pm_gpio_ptr;

	if (NULL == pm_gpios_ptr) {
		HW_CONFIG_DEBUG(" can not find  module:pm gpio\n");
		return NULL;
	}

	pm_gpio_ptr = (struct pm_gpio_cfg_t *)pm_gpios_ptr->data_array.pm_gpio_ptr;

    if (NULL != pm_gpio_ptr) {
		return pm_gpio_ptr;
	} else {
		HW_CONFIG_DEBUG(" return NULL\n");
		return NULL;
	}
}

int get_pm_gpio_num_by_name(char *name)
{
	int min = 0;
    int max = PM8921_GPIO_NUM - 1;
    int result;
    int new_cursor;
    struct pm_gpio_cfg_t *pm_gpio_ptr = get_pm_gpio_config_table();

	if (NULL == pm_gpio_ptr) {
		HW_CONFIG_DEBUG(" get pm gpio config table failed.\n");
		return -EFAULT;
    }

    while (min <= max) {
		new_cursor = (min + max) / 2;

		if (!(strcmp((pm_gpio_ptr + new_cursor)->name, ""))) {
			result = 1;
		} else {
			result = strcmp((pm_gpio_ptr + new_cursor)->name, name);
		}

		if (0 == result) {
			/*found it, just return*/
			return (pm_gpio_ptr+new_cursor)->gpio_number;
		} else if (result > 0) {
			/* key is smaller, update max*/
			max = new_cursor-1;
		} else if (result < 0) {
			/* key is bigger, update min*/
			min = new_cursor+1;
		}
	}

	return -EFAULT;
}
#endif

#ifdef CONFIG_HW_POWER_TREE
struct hw_config_power_tree *get_power_config_table(void)
{
	struct board_id_general_struct *power_general_struct = get_board_id_general_struct(POWER_MODULE_NAME);
	struct hw_config_power_tree *power_ptr = NULL;

	if (NULL == power_general_struct) {
		HW_CONFIG_DEBUG("Can not find module:regulator\n");
		return NULL;
	}

	power_ptr = (struct hw_config_power_tree *)power_general_struct->data_array.power_tree_ptr;

	if (NULL == power_ptr) {
		HW_CONFIG_DEBUG("hw_config_power_tree return  NULL\n");
	}
	return power_ptr;
}
#endif

int get_hsd_invert(void)
{
	unsigned int type = 0;

	bool ret = get_hw_config_int("audio/hsd_invert", &type, NULL);
	HW_CONFIG_DEBUG("hsad: hsd_invert = %d\n", type);
	if (ret == true) {
		return type;
	}

	return -1;
}

int get_hs_keys(void)
{
	unsigned int type = 0;

	bool ret = get_hw_config_int("audio/hs_keys", &type, NULL);
	HW_CONFIG_DEBUG("hsad: hs_keys = %d\n", type);
	if (ret == true) {
		return type;
	}

	return -1;
}
