/* ***************************************************************************************
* Filename:	k3_bq27510.c
*
* Discription:  this file monitor the battery state such as the capacity, voltage,current,
*		temperature and send the information to the battery monitor.
*
* Copyright: 	(C) 2011 S10.
*
* revision history: 1.0
******************************************************************************************/
#include <linux/module.h>
#include <linux/param.h>
#include <linux/jiffies.h>
#include <linux/workqueue.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/idr.h>
#include <linux/i2c.h>
#include <asm/unaligned.h>
#include <linux/interrupt.h>
#include <asm/irq.h>
#include <linux/module.h>
#include <linux/io.h>
#include <linux/uaccess.h>
#include <linux/vmalloc.h>
#include <linux/fs.h>
#include <linux/fcntl.h>
#include <linux/ctype.h>
#include <linux/slab.h>
#include <mach/gpio.h>
#include <linux/mutex.h>
#include <linux/power/k3_bq24161.h>
#include <linux/power/k3_bq27510.h>

#define RETRY_TIME	5
#define SLEEP_TIME	5

static DEFINE_IDR(k3_bq27510_battery_id);
static DEFINE_MUTEX(k3_bq27510_battery_mutex);

struct i2c_client *g_battery_measure_by_bq27510_i2c_client;
struct k3_bq27510_device_info *g_battery_measure_by_bq27510_device;

/*extern a notifier list for battery low notification*/
extern struct blocking_notifier_head notifier_list_bat;
static struct i2c_driver k3_bq27510_battery_driver;

static unsigned int gBq27510DownloadFirmwareFlag = BSP_NORMAL_MODE;

/*
 * Common code for bq27510 devices
 */

static int k3_bq27510_i2c_read_word(struct k3_bq27510_device_info *di, u8 reg)
{
	int err = 0;
	int i = 0;

	if (BSP_FIRMWARE_DOWNLOAD_MODE == gBq27510DownloadFirmwareFlag) {
		return -1;
	}

	mutex_lock(&k3_bq27510_battery_mutex);

	for (i = 0; i < RETRY_TIME; i++) {
		err = i2c_smbus_read_word_data(di->client, reg);
		if (err < 0) {
			BQ27510_ERR("[%s,%d] i2c_smbus_read_byte_data failed\n", __FUNCTION__, __LINE__);
		} else {
			break;
		}
		msleep(SLEEP_TIME);
	}

	mutex_unlock(&k3_bq27510_battery_mutex);

	return err;
}

/* added for Firmware upgrade begine */
static int k3_bq27510_i2c_word_write(struct i2c_client *client, u8 reg, u16 value)
{
	int err = 0;
	int i = 0;

	mutex_lock(&k3_bq27510_battery_mutex);

	for (i = 0; i < RETRY_TIME; i++) {
		err = i2c_smbus_write_word_data(client, reg, value);
		if (err < 0)  {
			BQ27510_ERR("[%s,%d] i2c_smbus_write_word_data failed\n", __FUNCTION__, __LINE__);
		} else {
			break;
		}
		msleep(SLEEP_TIME);
	}

	mutex_unlock(&k3_bq27510_battery_mutex);

	return err;
}

static int k3_bq27510_i2c_bytes_write(struct i2c_client *client, u8 reg, u8 *pBuf, u16 len)
{
	int i2c_ret 	= 0;
	int i 			= 0;
	int j 			= 0;
	u8 *p		= pBuf;

	mutex_lock(&k3_bq27510_battery_mutex);

	for (i = 0; i < len; i += I2C_SMBUS_BLOCK_MAX) {

		j = ((len - i) > I2C_SMBUS_BLOCK_MAX) ? I2C_SMBUS_BLOCK_MAX : (len - i);

		i2c_ret = i2c_smbus_write_i2c_block_data(client, reg+i, j, p+i);
		if (i2c_ret < 0)  {
			BQ27510_ERR("[%s,%d] i2c_transfer failed\n", __FUNCTION__, __LINE__);
		}
	}

	mutex_unlock(&k3_bq27510_battery_mutex);

	return i2c_ret;
}

static int k3_bq27510_i2c_bytes_read(struct i2c_client *client, u8 reg, u8 *pBuf, u16 len)
{
	int i2c_ret 	= 0;
	int i 			= 0;
	int j 			= 0;
	u8 *p		= pBuf;

	mutex_lock(&k3_bq27510_battery_mutex);

	for (i = 0; i < len; i += I2C_SMBUS_BLOCK_MAX) {
		j = ((len - i) > I2C_SMBUS_BLOCK_MAX) ? I2C_SMBUS_BLOCK_MAX : (len - i);

		i2c_ret = i2c_smbus_read_i2c_block_data(client, reg+i, j, p+i);
		if (i2c_ret < 0) {
			BQ27510_ERR("[%s,%d] i2c_transfer failed\n", __FUNCTION__, __LINE__);
		}
	}

	mutex_unlock(&k3_bq27510_battery_mutex);

	return i2c_ret;
}

static int k3_bq27510_i2c_bytes_read_and_compare(struct i2c_client *client, u8 reg, u8 *pSrcBuf, u8 *pDstBuf, u16 len)
{
	int i2c_ret = 0;

	i2c_ret = k3_bq27510_i2c_bytes_read(client, reg, pSrcBuf, len);
	if (i2c_ret < 0) {
		BQ27510_ERR("[%s,%d] bq27510_i2c_bytes_read failed\n", __FUNCTION__, __LINE__);
		return i2c_ret;
	}

	i2c_ret = strncmp(pDstBuf, pSrcBuf, len);

	return i2c_ret;
}
/* added for Firmware upgrade end */

/*
 * Return the battery temperature in Celcius degrees
 * Or < 0 if something fails.
 */
int k3_bq27510_battery_temperature(struct k3_bq27510_device_info *di)
{
	int data = 0;

	data = k3_bq27510_i2c_read_word(di, BQ27510_REG_TEMP);
	if (data < 0) {
		BQ27510_DBG("k3_bq27510_battery_temperature failed!!\n");
		return 0;
	}

	data = (data-CONST_NUM_2730)/CONST_NUM_10;

	BQ27510_DBG("read temperature result = %d Celsius\n", data);

	/*adapt android upper layer unit: 10 x degrees*/
	return data * 10;
}

/*
 * Return the battery Voltage in milivolts
 * Or < 0 if something fails.
 */
int k3_bq27510_battery_voltage(struct k3_bq27510_device_info *di)
{
	int data = 0;

	data = k3_bq27510_i2c_read_word(di, BQ27510_REG_VOLT);
	if (data < 0) {
		BQ27510_DBG("k3_bq27510_battery_voltage failed!!\n");
		return 0;
	}

	BQ27510_DBG("read voltage result = %d mVolts\n", data);

	/*adapt android upper layer unit: uV*/
	return data * 1000;
}

/*
 * Return the battery average current
 * Note that current can be negative signed as well
 * Or 0 if something fails.
 */
short k3_bq27510_battery_current(struct k3_bq27510_device_info *di)
{
	int	data 		= 0;
	short	nCurr 		= 0;

	data = k3_bq27510_i2c_read_word(di, BQ27510_REG_AI);
	if (data < 0) {
		BQ27510_DBG("k3_bq27510_battery_current failed!!\n");
		return 0;
	}

	nCurr = (signed short)data;

	BQ27510_DBG("read current result = %d mA\n", nCurr);

	return nCurr;
}

/*
 * Return the battery Relative State-of-Charge
 * The reture value is 0 - 100%
 * Or < 0 if something fails.
 */
int k3_bq27510_battery_capacity(struct k3_bq27510_device_info *di)
{
	int data 				= 0;
	int time 				= ERROR_TIME;

	data = k3_bq27510_i2c_read_word(di, BQ27510_REG_SOC);

	while ((data < 0) && (time > 0)) {
		BQ27510_DBG("k3_bq27510_battery_capacity failed!!\n");
		data = k3_bq27510_i2c_read_word(di, BQ27510_REG_SOC);
		time--;
		msleep(1);
	}

	BQ27510_DBG("read soc result = %d Hundred Percents\n", data);

	if (data < 0)
		data = 0;

	return data;
}

/*
 * Return the battery Time to Empty
 * Or < 0 if something fails
 */
int k3_bq27510_battery_tte(struct k3_bq27510_device_info *di)
{
	int data = 0;

	data = k3_bq27510_i2c_read_word(di, BQ27510_REG_TTE);
	if (data < 0) {
		BQ27510_DBG("k3_bq27510_battery_tte failed!!\n");
		return 0;
	}

	BQ27510_DBG("read tte result = %d minutes\n", data);

	return data;
}

/*
 * Return the battery Time to Full
 * Or < 0 if something fails
 */
int k3_bq27510_battery_ttf(struct k3_bq27510_device_info *di)
{
	int data = 0;

	data = k3_bq27510_i2c_read_word(di, BQ27510_REG_TTF);
	if (data < 0) {
		BQ27510_DBG("k3_bq27510_battery_ttf failed!!\n");
		return 0;
	}

	BQ27510_DBG("read ttf result = %d minutes\n", data);

	return data;
}

/*
 * Return whether the battery charging is full
 *
 */
int is_k3_bq27510_battery_full(struct k3_bq27510_device_info *di)
{
	int data = 0;

	data = k3_bq27510_i2c_read_word(di, BQ27510_REG_FLAGS);
	if (data < 0) {
		BQ27510_DBG("is_k3_bq27510_battery_full failed!!\n");
		return 0;
	}

	BQ27510_DBG("read flags result = 0x%x \n", data);

	return(data & BQ27510_FLAG_FC);
}

/*
 * Return whether the battery is discharging now
 *
 */
int is_k3_bq27510_battery_discharging(struct k3_bq27510_device_info *di)
{
	int data = 0;

	data = k3_bq27510_i2c_read_word(di, BQ27510_REG_FLAGS);
	if (data < 0) {
		BQ27510_DBG("is_k3_bq27510_battery_discharging failed!!\n");
		return 0;
	}

	BQ27510_DBG("read flags result = 0x%x \n", data);

	return (data & BQ27510_FLAG_DSG);
}

/*===========================================================================
  Function:       k3_bq27510_battery_status
  Description:    get the battery status of battery
  Calls:
  Called By:
  Input:          struct k3_bq27510_device_info *
  Output:         none
  Return:         0->"Unknown", 1->"Charging", 2->"Discharging", 3->"Not charging", 4->"Full"
  Others:         none
===========================================================================*/
int k3_bq27510_battery_status(struct k3_bq27510_device_info *di)
{
	int data 			= 0;
	int status 			= 0;
	short m_current 		= 0;

	/*Use the current direction to decide charging or discharging*/
	m_current = k3_bq27510_battery_current(di);

	data = k3_bq27510_i2c_read_word(di, BQ27510_REG_FLAGS);
	if (data < 0) {
		BQ27510_DBG("k3_bq27510_battery_status failed!!\n");
		return 0;
	}

	BQ27510_DBG("read status result = %d minutes\n", data);

	if (data & BQ27510_FLAG_FC)
		status = POWER_SUPPLY_STATUS_FULL;
	else if ((data & BQ27510_FLAG_DSG) && m_current < 0)
		status = POWER_SUPPLY_STATUS_DISCHARGING;
	else if (m_current > 0)
		status = POWER_SUPPLY_STATUS_CHARGING;
	else if (0 == m_current)
		status = POWER_SUPPLY_STATUS_NOT_CHARGING;

	return status;
}

/*===========================================================================
  Function:       k3_bq27510_battery_health
  Description:    get the health status of battery
  Calls:
  Called By:
  Input:          struct k3_bq27510_device_info *
  Output:         none
  Return:         0->"Unknown", 1->"Good", 2->"Overheat", 3->"Dead", 4->"Over voltage",
			5->"Unspecified failure", 6->"Cold",
  Others:         none
===========================================================================*/
int k3_bq27510_battery_health(struct k3_bq27510_device_info *di)
{
	int data 	= 0;
	int status 	= 0;

	data = k3_bq27510_i2c_read_word(di, BQ27510_REG_FLAGS);
	BQ27510_DBG("read health result = %d \n", data);

	if (data & (BQ27510_FLAG_OTC | BQ27510_FLAG_OTD))
		status = POWER_SUPPLY_HEALTH_OVERHEAT;
	else
		status = POWER_SUPPLY_HEALTH_GOOD;

	return status;
}

/*===========================================================================
  Function:       k3_bq27510_battery_capacity_level
  Description:    get the capacity level of battery
  Calls:
  Called By:
  Input:          struct k3_bq27510_device_info *
  Output:         none
  Return:         capacity percent
  Others:         none
===========================================================================*/
int k3_bq27510_battery_capacity_level(struct k3_bq27510_device_info *di)
{
	int data 		= 0;
	int data_capacity 	= 0;
	int status 		= 0;

	data = k3_bq27510_i2c_read_word(di, BQ27510_REG_FLAGS);
	data_capacity = k3_bq27510_battery_capacity(di);

	BQ27510_DBG("read capactiylevel result = %d \n", data);

	if (data & BQ27510_FLAG_SOCF)
		status = POWER_SUPPLY_CAPACITY_LEVEL_CRITICAL;
	else if (data & BQ27510_FLAG_SOC1)
		status = POWER_SUPPLY_CAPACITY_LEVEL_LOW;
	else if (data & BQ27510_FLAG_FC)
		status = POWER_SUPPLY_CAPACITY_LEVEL_FULL;
	else if (data_capacity > NORMAL_CAPACITY_LEVEL)
		status = POWER_SUPPLY_CAPACITY_LEVEL_HIGH;
	else
		status = POWER_SUPPLY_CAPACITY_LEVEL_NORMAL;

	return status;
}

/*===========================================================================
  Function:       k3_bq27510_battery_technology
  Description:    get the battery technology
  Calls:
  Called By:
  Input:           struct k3_bq27510_device_info *
  Output:         none
  Return:         value of battery technology
  Others:         none
===========================================================================*/
int k3_bq27510_battery_technology(struct k3_bq27510_device_info *di)
{
	/*Default technology is "Li-poly"*/
	int data = POWER_SUPPLY_TECHNOLOGY_LIPO;

	return data;
}

/*===========================================================================
  Function:       k3_is_bq27510_battery_exist
  Description:    get the status of battery exist
  Calls:
  Called By:
  Input:          struct k3_bq27510_device_info *
  Output:         none
  Return:         1->battery present; 0->battery not present.
  Others:         none
===========================================================================*/
int is_k3_bq27510_battery_exist(struct k3_bq27510_device_info *di)
{
	int data = 0;

	data = k3_bq27510_i2c_read_word(di, BQ27510_REG_FLAGS);
	BQ27510_DBG("is_exist flags result = 0x%x \n", data);

	if (data < 0) {
		BQ27510_DBG("is_k3_bq27510_battery_exist failed!!\n");
		return 0;
	}

	return !!(data & BQ27510_FLAG_DET);
}

EXPORT_SYMBOL_GPL(k3_bq27510_battery_temperature);
EXPORT_SYMBOL_GPL(k3_bq27510_battery_voltage);
EXPORT_SYMBOL_GPL(k3_bq27510_battery_current);
EXPORT_SYMBOL_GPL(k3_bq27510_battery_tte);
EXPORT_SYMBOL_GPL(k3_bq27510_battery_ttf);
EXPORT_SYMBOL_GPL(is_k3_bq27510_battery_full);
EXPORT_SYMBOL_GPL(is_k3_bq27510_battery_exist);
EXPORT_SYMBOL_GPL(k3_bq27510_battery_status);
EXPORT_SYMBOL_GPL(k3_bq27510_battery_health);
EXPORT_SYMBOL_GPL(k3_bq27510_battery_capacity);
EXPORT_SYMBOL_GPL(k3_bq27510_battery_capacity_level);

/*
 * Return the battery Control
 * Or < 0 if something fails
 */

#define to_k3_bq27510_device_info(x) container_of((x), \
				struct k3_bq27510_device_info, bat);


/* added for Firmware upgrade begine */
static int k3_bq27510_atoi(const char *s)
{
	int k = 0;

	k = 0;
	while (*s != '\0' && *s >= '0' && *s <= '9') {
		k = 10 * k + (*s - '0');
		s++;
	}
	return k;
}

static unsigned long k3_bq27510_strtoul(const char *cp, unsigned int base)
{
	unsigned long result = 0, value;

	while (isxdigit(*cp) && (value = isdigit(*cp) ? *cp-'0' : (islower(*cp)
	    ? toupper(*cp) : *cp)-'A'+10) < base) {
		result = result*base + value;
		cp++;
	}

	return result;
}

/*Parse the bqfs/dffs file's addr and data*/
static int k3_bq27510_firmware_program(struct i2c_client *client, const unsigned char *pgm_data, unsigned int filelen)
{
	unsigned int i = 0, j = 0, ulDelay = 0, ulReadNum = 0;
	unsigned int ulCounter = 0, ulLineLen = 0;
	unsigned char temp = 0;
	unsigned char *p_cur;
	unsigned char pBuf[BSP_MAX_ASC_PER_LINE] = { 0 };
	unsigned char p_src[BSP_I2C_MAX_TRANSFER_LEN] = { 0 };
	unsigned char p_dst[BSP_I2C_MAX_TRANSFER_LEN] = { 0 };
	unsigned char ucTmpBuf[16] = { 0 };

bq27510_firmware_program_begin:
	if (ulCounter > 10) {
		return -1;
	}

	p_cur = (unsigned char *)pgm_data;

	while (1) {
		if ((p_cur - pgm_data) >= filelen) {
			printk("Download success\n");
			break;
		}

		while (*p_cur == '\r' || *p_cur == '\n') {
			p_cur++;
		}

		i = 0;
		ulLineLen = 0;

		memset(p_src, 0x00, sizeof(p_src));
		memset(p_dst, 0x00, sizeof(p_dst));
		memset(pBuf, 0x00, sizeof(pBuf));

		while (i < BSP_MAX_ASC_PER_LINE) {
			temp = *p_cur++;
			i++;
			if (('\r' == temp) || ('\n' == temp)) {
				break;
			}
			if (' ' != temp) {
				pBuf[ulLineLen++] = temp;
			}
		}

		p_src[0] = pBuf[0];
		p_src[1] = pBuf[1];

		if (('W' == p_src[0]) || ('C' == p_src[0])) {
			for (i = 2, j = 0; i < ulLineLen; i += 2, j++) {
				memset(ucTmpBuf, 0x00, sizeof(ucTmpBuf));
				memcpy(ucTmpBuf, pBuf+i, 2);
				p_src[2+j] = k3_bq27510_strtoul(ucTmpBuf, 16);
			}
			temp = (ulLineLen - 2)/2;
			ulLineLen = temp + 2;
		} else if ('X' == p_src[0]) {
			memset(ucTmpBuf, 0x00, sizeof(ucTmpBuf));
			memcpy(ucTmpBuf, pBuf+2, ulLineLen-2);
			ulDelay = k3_bq27510_atoi(ucTmpBuf);
		} else if ('R' == p_src[0]) {
		memset(ucTmpBuf, 0x00, sizeof(ucTmpBuf));
		memcpy(ucTmpBuf, pBuf+2, 2);
		p_src[2] = k3_bq27510_strtoul(ucTmpBuf, 16);
		memset(ucTmpBuf, 0x00, sizeof(ucTmpBuf));
		memcpy(ucTmpBuf, pBuf+4, 2);
		p_src[3] = k3_bq27510_strtoul(ucTmpBuf, 16);
		memset(ucTmpBuf, 0x00, sizeof(ucTmpBuf));
		memcpy(ucTmpBuf, pBuf+6, ulLineLen-6);
		ulReadNum = k3_bq27510_atoi(ucTmpBuf);
		}

		if (':' == p_src[1]) {
			switch (p_src[0]) {
			case 'W':
#if 0
				/* firmware upgrade */
				printk("W: ");
				for (i = 0; i < ulLineLen-4; i++) {
					printk("%x ", p_src[4+i]);
				}
				printk(KERN_ERR "\n");
#endif
				if (k3_bq27510_i2c_bytes_write(client, p_src[3], &p_src[4], ulLineLen-4) < 0) {
					printk(KERN_ERR "[%s,%d] bq27510_i2c_bytes_write failed len=%d\n", __FUNCTION__, __LINE__, ulLineLen-4);
				}
				break;
			case 'R':
				if (k3_bq27510_i2c_bytes_read(client, p_src[3], p_dst, ulReadNum) < 0) {
					printk(KERN_ERR "[%s,%d] bq27510_i2c_bytes_read failed\n", __FUNCTION__, __LINE__);
				}
				break;
			case 'C':
				if (k3_bq27510_i2c_bytes_read_and_compare(client, p_src[3], p_dst, &p_src[4], ulLineLen-4)) {
					ulCounter++;
					printk(KERN_ERR "[%s,%d] bq27510_i2c_bytes_read_and_compare failed\n", __FUNCTION__, __LINE__);
					goto bq27510_firmware_program_begin;
				}
				break;
			case 'X':
				msleep(ulDelay);
				break;
			default:
				return 0;
			}
		}
	}
	return 0;
}

static int k3_bq27510_firmware_download(struct i2c_client *client, const unsigned char *pgm_data, unsigned int len)
{
	int iRet;

	/*Enter Rom Mode */
	iRet = k3_bq27510_i2c_word_write(client, BSP_ENTER_ROM_MODE_CMD, BSP_ENTER_ROM_MODE_DATA);
	if (0 != iRet) {
		printk(KERN_ERR "[%s,%d] bq27510_i2c_word_write failed\n", __FUNCTION__, __LINE__);
	}

	msleep(10);

	/*change i2c addr*/
	g_battery_measure_by_bq27510_i2c_client->addr = BSP_ROM_MODE_I2C_ADDR;

	/*program bqfs*/
	iRet = k3_bq27510_firmware_program(client, pgm_data, len);
	if (0 != iRet) {
		printk(KERN_ERR "[%s,%d] bq27510_firmware_program failed\n", __FUNCTION__, __LINE__);
	}

	/*change i2c addr*/
	g_battery_measure_by_bq27510_i2c_client->addr = BSP_NORMAL_MODE_I2C_ADDR;

	return iRet;
}

static int k3_bq27510_update_firmware(struct i2c_client *client, const char *pFilePath)
{
	char *buf;
	struct file *filp;
	struct inode *inode = NULL;
	mm_segment_t oldfs;
	unsigned int length;
	int ret = 0;

	/* open file */
	oldfs = get_fs();
	set_fs(KERNEL_DS);
	filp = filp_open(pFilePath, O_RDONLY, S_IRUSR);
	if (IS_ERR(filp)) {
		printk(KERN_ERR "[%s,%d] filp_open failed\n", __FUNCTION__, __LINE__);
		set_fs(oldfs);
		return -1;
	}

	if (!filp->f_op) {
		printk(KERN_ERR "[%s,%d] File Operation Method Error\n", __FUNCTION__, __LINE__);
		filp_close(filp, NULL);
		set_fs(oldfs);
		return -1;
	}

	inode = filp->f_path.dentry->d_inode;
	if (!inode) {
		printk(KERN_ERR "[%s,%d] Get inode from filp failed\n", __FUNCTION__, __LINE__);
		filp_close(filp, NULL);
		set_fs(oldfs);
		return -1;
	}

	/* file's size */
	length = i_size_read(inode->i_mapping->host);
	printk("bq27510 firmware image size is %d \n", length);
	if (!(length > 0 && length < BSP_FIRMWARE_FILE_SIZE)) {
		printk(KERN_ERR "[%s,%d] Get file size error\n", __FUNCTION__, __LINE__);
		filp_close(filp, NULL);
		set_fs(oldfs);
		return -1;
	}

	/* allocation buff size */
	/* buf size if even */
	buf = vmalloc(length+(length%2));
	if (!buf) {
		printk(KERN_ERR "[%s,%d] Alloctation memory failed\n", __FUNCTION__, __LINE__);
		filp_close(filp, NULL);
		set_fs(oldfs);
		return -1;
	}

	/* read data */
	if (filp->f_op->read(filp, buf, length, &filp->f_pos) != length) {
		printk(KERN_ERR "[%s,%d] File read error\n", __FUNCTION__, __LINE__);
		filp_close(filp, NULL);
		filp_close(filp, NULL);
		set_fs(oldfs);
		vfree(buf);
		return -1;
	}

	ret = k3_bq27510_firmware_download(client, (const char *)buf, length);

	filp_close(filp, NULL);
	set_fs(oldfs);
	vfree(buf);

	return ret;
}

/*Firmware upgrade sysfs store interface*/
static ssize_t k3_bq27510_attr_store(struct device_driver *driver, const char *buf, size_t count)
{
	int iRet = 0;
	unsigned char path_image[255];

	if (NULL == buf || count >= 255 || count == 0 || strnchr(buf, count, 0x20))
		return -1;

	memcpy (path_image, buf,  count);
	/* replace '\n' with  '\0'  */
	if ((path_image[count-1]) == '\n')
		path_image[count-1] = '\0';
	else
		path_image[count] = '\0';

	 /*enter firmware bqfs download*/
	 gBq27510DownloadFirmwareFlag = BSP_FIRMWARE_DOWNLOAD_MODE;
	 iRet = k3_bq27510_update_firmware(g_battery_measure_by_bq27510_i2c_client, path_image);
	 gBq27510DownloadFirmwareFlag = BSP_NORMAL_MODE;

	/* added for shutdown system in charging, begin */
	/* begin: added for refresh Qmax*/
	i2c_smbus_write_word_data(g_battery_measure_by_bq27510_i2c_client, 0x00, 0x0021);
	/* end: added for refresh Qmax*/
	/* added for shutdown system in charging, end */

	return iRet;
}

/* Firmware upgrade sysfs show interface*/
static ssize_t k3_bq27510_attr_show(struct device_driver *driver, char *buf)
{
	int iRet = 0;

	if (NULL == buf) {
		BQ27510_ERR("k3_bq27510_attr_show Error\n");
		return -1;
	}

	mutex_lock(&k3_bq27510_battery_mutex);
	i2c_smbus_write_word_data(g_battery_measure_by_bq27510_i2c_client, 0x00, 0x0008);
	msleep(2);
	iRet = i2c_smbus_read_word_data(g_battery_measure_by_bq27510_i2c_client, 0x00);
	mutex_unlock(&k3_bq27510_battery_mutex);
	if (iRet < 0) {
		return sprintf(buf, "%s", "Coulometer Damaged or Firmware Error");
	} else {
		return sprintf(buf, "%x", iRet);
	}
}

static ssize_t k3_bq27510_show_voltage(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int val = 0;
	struct k3_bq27510_device_info *di = dev_get_drvdata(dev);

	if (NULL == buf) {
		BQ27510_ERR("k3_bq27510_show_voltage Error\n");
		return -1;
	}
	val = k3_bq27510_i2c_read_word(di, BQ27510_REG_VOLT);

	if (val < 0) {
		BQ27510_ERR("get voltage is Error\n");
		return -1;
	}
	val *=1000;
	return sprintf(buf, "%d\n", val);
}

static DEVICE_ATTR(voltage, S_IWUSR | S_IRUGO, k3_bq27510_show_voltage, NULL);

/*define a sysfs interface for firmware upgrade*/
static DRIVER_ATTR(state, 0664, k3_bq27510_attr_show, k3_bq27510_attr_store);
/* added for Firmware upgrade end */

static struct attribute *k3_bq27150_attributes[] = {
	&dev_attr_voltage.attr,
	NULL,
};

static const struct attribute_group k3_bq27150_attr_group = {
	.attrs = k3_bq27150_attributes,
};


/*
 * Use BAT_LOW not BAT_GD. When battery capacity is below SOC1, BAT_LOW PIN will pull up and cause a
 * interrput, this is the interrput callback.
 */
static irqreturn_t k3_bq27510_abnormal_status_interrupt(int irq, void *_di)
{
	struct k3_bq27510_device_info *di = _di;

	schedule_delayed_work(&di->notifier_work, 0);

	return IRQ_HANDLED;
}

/*===========================================================================
  Function:       interrupt_notifier_work
  Description:    send a notifier event to sysfs
  Calls:
  Called By:
  Input:          struct work_struct *
  Output:         none
  Return:         none
  Others:         none
===========================================================================*/
static void interrupt_notifier_work(struct work_struct *work)
{
	struct k3_bq27510_device_info *di = container_of(work,
		struct k3_bq27510_device_info, notifier_work.work);

	long int  events = BQ24161_EVENT_UNKOWN;;

	if (!is_k3_bq27510_battery_exist(di)) {
		BQ27510_DBG("NO BATTERY!");
		return;
	}

	BQ27510_INFO("throw interrupt: low capacity\n");
	blocking_notifier_call_chain(&notifier_list_bat, events, NULL);
}

static int k3_bq27510_battery_probe(struct i2c_client *client,
				 const struct i2c_device_id *id)
{
	int retval 	= 0;
	int ret 	= 0;
	int num 	= 0;
	char *name 	= NULL;
	struct k3_bq27510_device_info *di = NULL;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		BQ27510_ERR("[%s,%d]: need I2C_FUNC_I2C\n", __FUNCTION__, __LINE__);
		return -ENODEV;
	}

	/* added for Firmware upgrade begin*/

	i2c_smbus_write_word_data(client, 0x00, 0x0008);

	msleep(2);

	retval = i2c_smbus_read_word_data(client, 0x00);
	if (retval < 0) {
		dev_err(&client->dev, "[%s,%d] Coulometer Damaged or Firmware Error\n", __FUNCTION__, __LINE__);
	} else {
		dev_info(&client->dev, "Normal Mode and read Firmware version=%04x\n", retval);
	}

	retval = driver_create_file(&(k3_bq27510_battery_driver.driver), &driver_attr_state);
       if (0 != retval) {
		BQ27510_ERR("failed to create sysfs entry(state): %d\n", retval);
		retval =  -ENOENT;
		goto batt_failed_0;
	}
	/*added for Firmware upgrade end*/

	/* Get new ID for the new battery device */
	retval = idr_pre_get(&k3_bq27510_battery_id, GFP_KERNEL);
	if (retval == 0) {
		BQ27510_ERR("bq27510 idr_pre_get failed!!\n");
		retval = -ENOMEM;
		goto batt_failed_0;
	}

	mutex_lock(&k3_bq27510_battery_mutex);
	retval = idr_get_new(&k3_bq27510_battery_id, client, &num);
	mutex_unlock(&k3_bq27510_battery_mutex);
	if (retval < 0) {
		BQ27510_ERR("bq27510 idr_get_new failed!!\n");
		retval =  -ENOENT;
		goto batt_failed_1;
	}

	name = kasprintf(GFP_KERNEL, "bq27510-%d", num);
	if (!name) {
		BQ27510_ERR("failed to allocate device name\n");
		retval = -ENOMEM;
		goto batt_failed_1;
	}

	di = kzalloc(sizeof(*di), GFP_KERNEL);
	if (!di) {
		BQ27510_ERR("failed to allocate device info data\n");
		retval = -ENOMEM;
		goto batt_failed_2;
	}

	di->id = num;

	i2c_set_clientdata(client, di);
	di->dev = &client->dev;
	di->bat.name = name;
	di->client = client;

	/*k3_bq27510_powersupply_init(di);*/

	INIT_DELAYED_WORK(&di->notifier_work,
				interrupt_notifier_work);

	/**********ADD BY 00186176 begin****************/

#ifdef CONFIG_GPIO_BAT
	di->pin = iomux_get_pin(ADCSENSOR_PIN);
	if (!di->pin) {
		BQ27510_ERR("could not iomux_get_pin");
		retval = -ENOMEM;
		goto batt_failed_3;
	}

	ret = pinmux_setpullupdown(di->pin, PULLUP);
	if (ret < 0) {
		BQ27510_ERR("could not pinmux_setpullupdown");
		retval = -ENOMEM;
		goto batt_failed_3;
	}
#endif

	if (gpio_request(client->irq, "Batteryctl") < 0) {
		BQ27510_ERR("Batteryctl GPIO request failed\n");
		retval = -ENOMEM;
		goto batt_failed_3;
	}

	gpio_direction_input(client->irq);

	/* request battery_low interruption */
	ret = request_irq(gpio_to_irq(client->irq), k3_bq27510_abnormal_status_interrupt, IRQF_TRIGGER_FALLING | IRQF_NO_SUSPEND,
				"bq27510_irq_ctrl", di);

	/**********ADD BY 00186176 END****************/

	if (ret) {
		BQ27510_ERR("could not request irq %d, status %d\n", gpio_to_irq(client->irq), ret);
		retval = -ENOMEM;
		goto batt_failed_4;
	}

	g_battery_measure_by_bq27510_i2c_client = client;
	g_battery_measure_by_bq27510_device = di;

	retval = sysfs_create_group(&client->dev.kobj, &k3_bq27150_attr_group);

	if (retval) {
		BQ27510_ERR("[%s,%d]:\n ", __FUNCTION__, __LINE__);
		retval =  -ENOENT;
		goto batt_failed_5;
	}

	BQ27510_DBG("bq27510 support ver. %s enabled\n", DRIVER_VERSION);

	return 0;

	/**********ADD BY 00186176 begin****************/
batt_failed_5:
	free_irq(gpio_to_irq(di->client->irq), di);
batt_failed_4:
	gpio_free(client->irq);
batt_failed_3:
	/**********ADD BY 00186176 END****************/
	kfree(di);
	di = NULL;
batt_failed_2:
	kfree(name);
	name = NULL;
batt_failed_1:
	mutex_lock(&k3_bq27510_battery_mutex);
	idr_remove_all(&k3_bq27510_battery_id);
	idr_destroy(&k3_bq27510_battery_id);
	mutex_unlock(&k3_bq27510_battery_mutex);
batt_failed_0:
	return retval;
}

static int k3_bq27510_battery_remove(struct i2c_client *client)
{
	struct k3_bq27510_device_info *di = i2c_get_clientdata(client);

	sysfs_remove_group(&client->dev.kobj, &k3_bq27150_attr_group);

	free_irq(gpio_to_irq(di->client->irq), di);

	/**********ADD BY 00186176 begin****************/
	gpio_free(di->client->irq);
	/**********ADD BY 00186176 END****************/

	/*power_supply_unregister(&di->bat);*/

	kfree(di->bat.name);
	g_battery_measure_by_bq27510_i2c_client = NULL;

	cancel_delayed_work(&di->notifier_work);

	mutex_lock(&k3_bq27510_battery_mutex);
	idr_remove_all(&k3_bq27510_battery_id);
	idr_destroy(&k3_bq27510_battery_id);
	mutex_unlock(&k3_bq27510_battery_mutex);

	kfree(di);

	return 0;
}

/**********ADD BY 00186176 begin****************/
static void k3_bq27510_battery_shutdown(struct i2c_client *client)
{
	struct k3_bq27510_device_info *di = i2c_get_clientdata(client);

	printk("[%s] +\n", __func__);
	free_irq(gpio_to_irq(di->client->irq), di);
	cancel_delayed_work(&di->notifier_work);
	printk("[%s] -\n", __func__);

	return;
}
/**********ADD BY 00186176 END****************/

/*
 * Module stuff
 */

#ifdef CONFIG_PM
static int k3_bq27510_charger_suspend(struct i2c_client *client,
	pm_message_t state)
{
	printk("[%s] +\n", __func__);
	printk("[%s] -\n", __func__);

	return 0;
}

static int k3_bq27510_charger_resume(struct i2c_client *client)
{
	printk("[%s] +\n", __func__);
	printk("[%s] -\n", __func__);

	return 0;
}
#else
#define k3_bq24161_charger_suspend	NULL
#define k3_bq24161_charger_resume	NULL
#endif

static const struct i2c_device_id k3_bq27510_id[] = {
	{"bq27510-battery", 0},
	{},
};

static struct i2c_driver k3_bq27510_battery_driver = {
	.driver = {
		.name = "bq27510-battery",
	},
	.probe = k3_bq27510_battery_probe,
	.remove = k3_bq27510_battery_remove,
	.shutdown = k3_bq27510_battery_shutdown,
	.suspend = k3_bq27510_charger_suspend,
	.resume	= k3_bq27510_charger_resume,
	.id_table = k3_bq27510_id,
};

static int __init k3_bq27510_battery_init(void)
{
	int ret = 0;

	ret = i2c_add_driver(&k3_bq27510_battery_driver);
	if (ret)
		BQ27510_ERR("Unable to register BQ27510 driver\n");

	return ret;
}

fs_initcall(k3_bq27510_battery_init);

static void __exit k3_bq27510_battery_exit(void)
{
	i2c_del_driver(&k3_bq27510_battery_driver);
}
module_exit(k3_bq27510_battery_exit);

MODULE_AUTHOR("S10");
MODULE_DESCRIPTION("BQ27510 battery monitor driver");
MODULE_LICENSE("GPL");
