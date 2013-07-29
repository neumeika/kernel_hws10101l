#ifndef _LINUX_MAX8903_H
#define _LINUX_MAX8903_H


#define CHARGING_CURRENT_LEVEL0  500
#define CHARGING_CURRENT_LEVEL1  1500
#define CHARGING_CURRENT_LEVEL2 2000

#define DRIVER_NAME "max8903_charger"


//#define MAX8903_DEBUG_FLAG
#if defined(MAX8903_DEBUG_FLAG)
#define	MAX8903_DBG(format, arg...)		do { printk(KERN_ALERT format, ## arg);  } while (0)
#define	MAX8903_ERR(format, arg...)		do { printk(KERN_ERR format, ## arg);  } while (0)
#define	MAX8903_FAT(format, arg...)		do { printk(KERN_CRIT format, ## arg); } while (0)
#else
#define	MAX8903_DBG(format, arg...)		do { (void)(format); } while (0)
#define	MAX8903_ERR(format, arg...)		do { (void)(format); } while (0)
#define	MAX8903_FAT(format, arg...)		do { (void)(format); } while (0)
#endif

struct max8903_platform_data
{
    int gpio_dok;//
	int switch_gpio;
    int fat_intr_gpio;
    int ctr0_gpio;
    int ctr1_gpio;
	int	irq_type;
	char io_block_name[20];
    int (*io_mux_block_init)(struct max8903_platform_data* pdata);
    int (*io_mux_block_exit)(struct max8903_platform_data* pdata);
	int  (*init_gpio)(unsigned gpio, int level);
};

#endif /* _LINUX_MAX8903_H */
