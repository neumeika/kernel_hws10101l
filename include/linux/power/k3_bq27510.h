#ifndef __K3_BQ27510_BATTERY_H
#define __K3_BQ27510_BATTERY_H

#include <linux/power_supply.h>
#include <linux/mux.h>

#define I2C_ADDR_BQ27510			(0x55)

#define	DRIVER_VERSION				"1.0.0"

#define	BQ27510_REG_TEMP			(0x06)
#define	BQ27510_REG_VOLT			(0x08)
#define	BQ27510_REG_FLAGS			(0x0a)
/*Time to Empty*/
#define	BQ27510_REG_TTE				(0x16)
/*Time to Full*/
#define	BQ27510_REG_TTF				(0x18)
/* State-of-Charge */
#define	BQ27510_REG_SOC				(0x2C)
/*Average Current*/
#define	BQ27510_REG_AI				(0x14)

/* both words and bytes are LSB*/
/* Full-charged bit */
#define	BQ27510_FLAG_FC				(1<<9)
#define	BQ27510_FLAG_DET			(1<<3)
/* Over-Temperature-Charge bit */
#define	BQ27510_FLAG_OTC			(1<<15)
/* Over-Temperature-Discharge bit */
#define 	BQ27510_FLAG_OTD		(1<<14)
/* State-of-Charge-Threshold 1 bit */
#define	BQ27510_FLAG_SOC1			(1<<2)
/* State-of-Charge-Threshold Final bit */
#define	BQ27510_FLAG_SOCF			(1<<1)
/* Discharging detected bit */
#define	BQ27510_FLAG_DSG			(1<<0)

#define	CONST_NUM_10				(10)
#define	CONST_NUM_2730				(2730)

#define	NORMAL_CAPACITY_LEVEL	(95)

#define	ERROR_TIME					(8)
/***********ADD BY WANGYUE BEGIN**********/
/*gpio_170*/
#define ADCSENSOR_PIN				"g28"
/***********ADD BY WANGYUE END**********/

/* added for Firmware upgrade begine */
#define	BSP_UPGRADE_FIRMWARE_BQFS_CMD       "upgradebqfs"
#define	BSP_UPGRADE_FIRMWARE_DFFS_CMD       "upgradedffs"
/*库仑计的完整的firmware，包含可执行镜像及数据*/
#define	BSP_UPGRADE_FIRMWARE_BQFS_NAME      "/system/etc/coulometer/bq27510_pro.bqfs"
/*库仑计的的数据信息*/
#define	BSP_UPGRADE_FIRMWARE_DFFS_NAME      "/system/etc/coulometer/bq27510_pro.dffs"

#define	BSP_ROM_MODE_I2C_ADDR				(0x0B)
#define	BSP_NORMAL_MODE_I2C_ADDR			(0x55)
#define	BSP_FIRMWARE_FILE_SIZE				(400*1024)
#define	BSP_I2C_MAX_TRANSFER_LEN			(128)
#define	BSP_MAX_ASC_PER_LINE				(400)
#define	BSP_ENTER_ROM_MODE_CMD				(0x00)
#define	BSP_ENTER_ROM_MODE_DATA				(0x0F00)
#define	BSP_FIRMWARE_DOWNLOAD_MODE			(0xDDDDDDDD)
#define	BSP_NORMAL_MODE						(0x00)
/* added for Firmware upgrade end */

#define	BAT_VOL_3500						(3500000)
#define	BAT_VOL_3600						(3600000)
#define	BAT_VOL_3700						(3700000)
#define	BAT_VOL_3800						(3800000)
#define	BAT_VOL_3900						(3900000)

#define	BQ27510_ERR(format, arg...)		do { printk(KERN_ERR format, ## arg);  } while (0)
#define	BQ27510_INFO(format, arg...)		do { printk(format);} while (0)

/*#define BQ27510_DEBUG_FLAG*/
#if defined(BQ27510_DEBUG_FLAG)
#define	BQ27510_DBG(format, arg...)		do { printk(KERN_ALERT format, ## arg);  } while (0)
#define	BQ27510_FAT(format, arg...)		do { printk(KERN_CRIT format, ## arg); } while (0)
#else
#define	BQ27510_DBG(format, arg...)		do { (void)(format); } while (0)
#define	BQ27510_FAT(format, arg...)		do { (void)(format); } while (0)
#endif

 struct k3_bq27510_device_info {
	struct device 		*dev;
	int					id;
	struct power_supply	bat;
	struct i2c_client	*client;

	struct delayed_work	notifier_work;
	struct iomux_pin 	*pin;
};

/*external functions*/
extern int k3_bq27510_battery_temperature(struct k3_bq27510_device_info *di);
extern int k3_bq27510_battery_voltage(struct k3_bq27510_device_info *di);
extern short k3_bq27510_battery_current(struct k3_bq27510_device_info *di);
extern int k3_bq27510_battery_tte(struct k3_bq27510_device_info *di);
extern int k3_bq27510_battery_ttf(struct k3_bq27510_device_info *di);
extern int is_k3_bq27510_battery_full(struct k3_bq27510_device_info *di);
extern int is_k3_bq27510_battery_exist(struct k3_bq27510_device_info *di);
extern int k3_bq27510_battery_status(struct k3_bq27510_device_info *di);
extern int k3_bq27510_battery_health(struct k3_bq27510_device_info *di);
extern int k3_bq27510_battery_capacity(struct k3_bq27510_device_info *di);
extern int k3_bq27510_battery_capacity_level(struct k3_bq27510_device_info *di);
extern int k3_bq27510_battery_technology(struct k3_bq27510_device_info *di);

#endif


