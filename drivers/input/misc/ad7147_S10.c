
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/miscdevice.h>
#include <linux/slab.h>
#include <linux/workqueue.h>
#include <linux/ctype.h>
#include <linux/mux.h>
#include <linux/ad7147_S10.h>
#include "AD7147RegMap.h"
#include <linux/mtd/nve_interface.h>
#include <linux/hrtimer.h>
#define TRIGERR_THRESHOLD    (800)

//寄存器地址
#define BANK1_ADDR (0x000)
#define BANK2_ADDR (0x080)

#define POWER_UP_INTERRUPT (0x0001)
#define STAGE_CAL_EN_VALUE (0x0001)
#define DELAY_TIME (1000)

#define EVENT_NO_CHANGE (0)
#define EVENT_CHANGED   (1)

#define TOUCH 	(1)
#define NOTOUCH (0)
#define TOUCH_TO_NOTOUCH (200)

static void ad7147_work_func(struct work_struct * work);
static DECLARE_DELAYED_WORK(ad7147_work, ad7147_work_func);
static int ad7147_log(void);

static void ad7147_log_work_func(struct work_struct * work);
static DECLARE_DELAYED_WORK(ad7147_log_work, ad7147_log_work_func);

unsigned short bank2_reg[] = {
//  0x080   0x081   0x082   0x083   0x084   0x085   0x086   0x087
    0x3FEF, 0x1FFF, 0x0c00, 0x0000, 0x640, 0x0b30, 0x0ae8, 0x7d0,
};

//Bank1 register (0x000--0x007)
unsigned short bank1_reg_init[] = {
//  0x000   0x001(具体值后面设置)   0x002   0x003   0x004   0x005   0x006   0x007
    0x0002, 0xf001, 0x0ff3, 0x03ff, 0xffff, 0x0001, 0x0001, 0x0000,
};


static struct nve_info_user nve_sar_info;
static void read_nv_work_func(struct work_struct * work);
static DECLARE_DELAYED_WORK(read_nv_work, read_nv_work_func);
static int ad7147_write_nv(int value);
static int ad7147_read_nv(void);
static int ad7147_nv_value = 0;
static int sar_debug = 0; //0: close, 1: open
static int enable_uevent = 1; // enable uevent, 0: disable uevent

#ifndef ARRAY_SIZE
 #define ARRAY_SIZE(x) (sizeof(x) / sizeof((x)[0]))
#endif

static struct ad7147_data *ad7147_data_global = NULL;

static int ad7147_i2c_write(struct i2c_client *client, unsigned short reg, unsigned short data)
{
    int ret   = 0;
    u8 *_reg  = (u8 *)&reg;
    u8 *_data = (u8 *)&data;

    u8 tx[4] = {
        _reg[1],
        _reg[0],
        _data[1],
        _data[0]
    };

    ret = i2c_master_send(client, tx, 4);
    if (ret < 0)
    {
        dev_err(&client->dev, "I2C write error %d\n", ret);
    }

    return ret;
}

static int ad7147_i2c_read(struct i2c_client *client, unsigned short reg, volatile unsigned short *data)
{
    int ret   = 0;
    u8 *_reg  = (u8 *)&reg;
    u8 *_data = (u8 *)data;

    u8 tx[2] = {
        _reg[1],
        _reg[0]
    };
    u8 rx[2];

    ret = i2c_master_send(client, tx, 2);
    if (ret >= 0)
    {
        ret = i2c_master_recv(client,rx,2);
    }

    if (unlikely(ret < 0))
    {
        dev_err(&client->dev, "I2C read error %d\n", ret);
    }
    else
    {
        _data[0] = rx[1];
        _data[1] = rx[0];
    }

    return ret;
}

static int ad7147_clear_irq(struct i2c_client *client)
{
    int ret = 0;
    unsigned short result = 0;

    ret = ad7147_i2c_read(client, STAGE_LOW_LIMIT_INT, &result);
    if (ret < 0)
    {
        printk(KERN_ERR "[%s]: read STAGE_LOW_LIMIT_INT register  0x%x err %d", __func__, STAGE_LOW_LIMIT_INT, ret);
    }

    ad7147_i2c_read(client, STAGE_HIGH_LIMIT_INT, &result);
    if (ret < 0)
    {
        printk(KERN_ERR "[%s]: read STAGE_HIGH_LIMIT_INT register  0x%x err %d", __func__, STAGE_HIGH_LIMIT_INT, ret);
    }

    ad7147_i2c_read(client, STAGE_COMPLETE_LIMIT_INT, &result);
    if (ret < 0)
    {
        printk(KERN_ERR "[%s]: read STAGE_COMPLETE_LIMIT_INT register  0x%x err %d", __func__, STAGE_COMPLETE_LIMIT_INT,ret);
    }

    return ret;
}

static int ForceCalibration(struct i2c_client *client)
{
    int ret = -1;
    unsigned short result = 0;

    ret = ad7147_i2c_read(client, AMB_COMP_CTRL0, &result);
    if (ret < 0)
    {
        printk(KERN_ERR "[%s]: read AMB_COMP_CTRL0 register  0x%x err %d", __func__, AMB_COMP_CTRL0, ret);
    }

    result |= 0x4000;

    ret = ad7147_i2c_write(client, AMB_COMP_CTRL0, result);
    if (ret < 0)
    {
        printk(KERN_ERR "[%s]: write AMB_COMP_CTRL0 register  0x%x err %d", __func__, AMB_COMP_CTRL0, ret);
    }

    return ret;
}

static int ad7147_send_uevent(int status)
{
    int ret = -1;
    static int sar_status = -1;

    char *touch[2] = {"SAR_STATE=TOUCH", NULL};
    char *notouch[2] = {"SAR_STATE=NOTOUCH", NULL};
    char **uevent_envp = NULL;

    if (!enable_uevent)
        return 0;

    if (sar_status != status) {
        sar_status = status;
    } else {
    	printk(KERN_ERR "[%s] status is the same. status %d\n", __func__, sar_status);
        return 0;;
    }

    if (status == TOUCH) {
    	uevent_envp = touch;
    } else {
    	uevent_envp = notouch;
    }

    ret = kobject_uevent_env(&ad7147_data_global->client->dev.kobj, KOBJ_CHANGE, uevent_envp);
	printk(KERN_ERR "send %s uevent.\n", status ? "touch" : "notouch");

    return ret;
}
static int ad7147_interrupt_func(void)
{
    int status = -1;
    unsigned short cdc0_value = 0;
    unsigned short h_th = 0;
    unsigned short int_low_status  = 0;
    unsigned short int_high_status = 0;
    unsigned short int_com_status = 0;

    if (sar_debug) {
        printk(KERN_ERR "@@@@@@@@@@@@@@@@\n");
	ad7147_log();
    }
	
    mutex_lock(&ad7147_data_global->mutex);
    ad7147_i2c_read(ad7147_data_global->client, STAGE_LOW_LIMIT_INT, &int_low_status);
    ad7147_i2c_read(ad7147_data_global->client, STAGE_HIGH_LIMIT_INT, &int_high_status);
    ad7147_i2c_read(ad7147_data_global->client, STAGE_COMPLETE_LIMIT_INT, &int_com_status);
    if (sar_debug) {
	    printk(KERN_ERR "INT_LOW is: 0x%x\nINT_HIGH is: 0x%x\nINT_COM is: 0x%x\n", int_low_status, int_high_status,int_com_status);
    }
    if ((int_low_status & POWER_UP_INTERRUPT) != 0x0) {
		if (sar_debug) {
	        printk(KERN_ERR "ForceCalibration**********\n");
		}
        ForceCalibration(ad7147_data_global->client);
    } else {
        status = (int_high_status & POWER_UP_INTERRUPT ? TOUCH : NOTOUCH);

        if (sar_debug) {
	        printk(KERN_ERR "%s\n", status ? "touch" : "notouch");
	}

        if (status == NOTOUCH) {
            ad7147_i2c_read(ad7147_data_global->client, ADCRESULT_S0, &cdc0_value);
            ad7147_i2c_read(ad7147_data_global->client, STAGE0_UPP_THRES, &h_th);
            if (sar_debug) {
	            printk(KERN_ERR "h_th(%d) - cdc0_value(%d) = %d\n", h_th, cdc0_value, h_th-cdc0_value);
	    }
            if (h_th - cdc0_value < TOUCH_TO_NOTOUCH) {
                // if hard interrupt do nothing here, start the work thread to adjust touch or notouch
                // event, because the hard interrupt may not occur again when the hand move very slowly.
                mutex_unlock(&ad7147_data_global->mutex);
                return EVENT_NO_CHANGE;
            }
        }

	ad7147_send_uevent(status);
    }
    mutex_unlock(&ad7147_data_global->mutex);
    return EVENT_CHANGED;
}

static irqreturn_t ad7147_interrupt_handler(int irq, void *dev_id)
{
        schedule_work(&ad7147_work.work);
	return IRQ_HANDLED;
}
static int ad7147_reg_init(struct ad7147_data * data)
{
    int ret = -1;
    int len = -1;
    int i = 0;

    //写BANK2寄存器
    len = ARRAY_SIZE(bank2_reg);
    for (i = 0; i < len; i++)
    {
        ret = ad7147_i2c_write(data->client, BANK2_ADDR + i, bank2_reg[i]);
        if (ret < 0)
        {
            printk(KERN_ERR "[%s]: write bank2 register  0x%x err %d", __func__, BANK2_ADDR + i, ret);
        }
    }

    //写Bank1寄存器,保持0x001为缺省值
    len = ARRAY_SIZE(bank1_reg_init);

    //reg 0
    ret = ad7147_i2c_write(data->client, BANK1_ADDR, bank1_reg_init[0]);
    if (ret < 0)
    {
        printk(KERN_ERR "[%s]: write bank1 register 0x%x err %d", __func__, BANK1_ADDR + i, ret);
    }

    //reg 2-7
    for (i = 2; i < len; i++)
    {
        ret = ad7147_i2c_write(data->client, BANK1_ADDR + i, bank1_reg_init[i]);
        if (ret < 0)
        {
            printk(KERN_ERR "[%s]: write bank1 register 0x%x err %d", __func__, BANK1_ADDR + i, ret);
        }
    }

    //reg 1,enable cdc channel
    ad7147_i2c_write(data->client, BANK1_ADDR + 1, (bank1_reg_init[1] | STAGE_CAL_EN_VALUE));
    //clear interrupts
    ret = ad7147_clear_irq(data->client);

    ret = ForceCalibration(ad7147_data_global->client);

    return ret;
}

static int ad7147_hw_detect(struct ad7147_data * data)
{
    unsigned short status = 0;
    int ret = -1;

    ret = ad7147_i2c_read(data->client, DEVID, &status);

    if (ret < 0)
    {
        printk(KERN_ERR "fail to read DEVID, read ID is %04x\n", status);
        return -ENODEV;
    }
    printk(KERN_ERR "DEVID read ID is %04x\n", status);

    switch (status & 0xFFF0)
    {
    case 0x1490:
        return 0;
    default:
        printk(KERN_ERR "fail to detect AD714X captouch, read ID is %04x\n", status);
        return -ENODEV;
    }
}

//需要在board-hi6421-regulator.h文件中添加对应的
//REGULATOR_SUPPLY结构,dev_name应该和device中name是一样的
#define AD7147_VCC_VOLTAGE 3300000
static int ad7147_pwr_init(struct ad7147_data * data, struct ad7147_platform_data * platformd_data)
{
    int ret = -1;

    struct ad7147_data *ad7147_data = data;
    struct ad7147_platform_data *ad7147_platform_data = platformd_data;

    ad7147_data->ldo15_3v3 = regulator_get(&ad7147_data->client->dev, "ad7147-vcc");
    if (IS_ERR(ad7147_data->ldo15_3v3))
    {
        printk(KERN_ERR "cannot get ad7147 vdrive");
        return -1;
    }

    ret = regulator_set_voltage(ad7147_data->ldo15_3v3, AD7147_VCC_VOLTAGE, AD7147_VCC_VOLTAGE);
    if (ret < 0)
    {
        printk(KERN_ERR "set ad7147 vdrive error");
        return -1;
    }

    ret = regulator_enable(ad7147_data->ldo15_3v3);
    if (ret < 0)
    {
        printk(KERN_ERR "enable ad7147 vdrive error");
        return -1;
    }
    ret = gpio_request(ad7147_platform_data->ad7147_int_gpio, "ad7147_int_gpio");
    if (ret < 0)
    {
        printk(KERN_ERR "request ad7147 irq gpio failed. ret %d", ret);
        return -1;
    }

    ret = gpio_direction_input(ad7147_platform_data->ad7147_int_gpio);
    if (ret < 0)
    {
        gpio_free(ad7147_platform_data->ad7147_int_gpio);
        printk(KERN_ERR "set ad7147 gpio direction failed");
        return -1;
    }

    return ret;	
}

static ssize_t status_show(struct device *          dev,
                           struct device_attribute *attr,
                           char *                   buf)
{
    unsigned short status = 0;
    int i = 0;

    if (sar_debug) {
	    schedule_delayed_work(&ad7147_log_work, msecs_to_jiffies(DELAY_TIME));
    }
    mutex_lock(&ad7147_data_global->mutex);
    ad7147_i2c_read(ad7147_data_global->client, 0x17, &status);
    printk(KERN_ERR "DEVICE ID is %d", status);

    for (i = 0; i < 0x09F; i++)
    {
        ad7147_i2c_read(ad7147_data_global->client, i, &status);
        printk(KERN_ERR "0x%x:  0x%x", i, status);
        if ((i % 8) == 0)
        {
            printk(KERN_ERR "\n\n");
        }
    }
    mutex_unlock(&ad7147_data_global->mutex);
    return sprintf(buf, "%d", atomic_read(&ad7147_data_global->sensor_status));
}

static ssize_t status_store(struct device *dev,
                              struct device_attribute *attr,
                              const char *buf, size_t count)
{
    if(!strncmp(buf,"logon", 5)) {
		sar_debug = 1;
    } else {
		sar_debug = 0;
    }

    sprintf(buf, "debug is %s\n", sar_debug ? "on" : "off");

    return count;
}

static DEVICE_ATTR(status, 0644, status_show, status_store);

static ssize_t enable_uevent_store(struct device *dev,
                              struct device_attribute *attr,
                              const char *buf, size_t count)
{
    enable_uevent = simple_strtol(buf, NULL, 0);

    if (enable_uevent) {
        printk(KERN_INFO "enable uevent\n");
    } else {
        printk(KERN_INFO "disable uevent\n");
    }

    sprintf(buf, "enable uevent %s\n", enable_uevent ? "on" : "off");

    return count;
}

static DEVICE_ATTR(enable_uevent, 0664, NULL, enable_uevent_store);

static ssize_t register_store(struct device *dev,
                              struct device_attribute *attr,
                              const char *buf, size_t count)
{
    int addr = -1;
    unsigned short w_value = -1;
    unsigned short r_value = 0;
    int i   = 0;
    int len = 0;
    int j = 0;
    char tmp[25] = {0};

    //0: read
    //1: write
    int rw = 0;

    struct i2c_client *client = to_i2c_client(dev);

    if (count > 25)
    {
        printk(KERN_ERR "input is too long");
        return -1;
    }

    if (buf[0] == 'r')
    {
        rw = 0;
    }
    else if (buf[0] == 'w')
    {
        rw = 1;
    }

    for (i = 0, j = 0; i < count; i++)
    {
        printk(KERN_ERR "buf[%d] %c", i, buf[i]);
        if (buf[i] == ',')
        {
            len = i;
            break;
        }
        else if (isxdigit(buf[i]) || (buf[i] == 'x') || (buf[i] == 'X'))
        {
            tmp[j++] = buf[i];
        }
    }

    addr = simple_strtol(tmp, NULL, 16);
    printk(KERN_ERR "tmp %s", tmp);
    w_value = simple_strtol(buf + len + 1, NULL, 16);
    printk(KERN_ERR "buf_len %s", buf + len);
    printk(KERN_ERR "buf is %s\n", buf);
    printk(KERN_ERR "addr 0x%x, w_value 0x%x", addr, w_value);
    mutex_lock(&ad7147_data_global->mutex);
    if (rw == 0)
    {
        ad7147_i2c_read(client, addr, &r_value);
        printk(KERN_ERR "read register 0x%x old value 0x%x\n", addr, r_value);
    }
    else if (rw == 1)
    {
        ad7147_i2c_write(client, addr, w_value);
        printk(KERN_ERR "write addr 0x%x value 0x%x\n", addr, w_value);
        ad7147_i2c_read(client, addr, &r_value);
        printk(KERN_ERR "read register 0x%x old value 0x%x\n", addr, r_value);
    }
    mutex_unlock(&ad7147_data_global->mutex);
    return count;
}

static DEVICE_ATTR(register, 0664, NULL, register_store);

static int ad7147_log(void)
{
    unsigned short cdc0 = 0;
    unsigned short low_int_status  = 0;
    unsigned short high_int_status = 0;
    unsigned short complete_int_status = 0;
    unsigned short status = 0;
    unsigned short ambinet, max_avg, h_th, min_avg, l_th;
    int ret = 0;
    int i = 0;

    mutex_lock(&ad7147_data_global->mutex);
    for (i = 0; i < 2; i++)
    {
        ret = ad7147_i2c_read(ad7147_data_global->client, ADCRESULT_S0 + i, &cdc0);
        if (ret < 0)
        {
            printk(KERN_ERR "[%s %d]: read cdc0 error\n", __func__, __LINE__);
        }

        printk(KERN_ERR "cdc%d is %d", i, cdc0);
    }

    ret = ad7147_i2c_read(ad7147_data_global->client, PROX_STAT_REG, &status);
    printk(KERN_ERR "PROX_STAT_REG %d\n", status);
    ret = ad7147_i2c_read(ad7147_data_global->client, STAGE_LOW_LIMIT_INT, &low_int_status);
    ret = ad7147_i2c_read(ad7147_data_global->client, STAGE_HIGH_LIMIT_INT, &high_int_status);
    ret = ad7147_i2c_read(ad7147_data_global->client, STAGE_COMPLETE_LIMIT_INT, &complete_int_status);
    printk(KERN_ERR "INT8 is 0x%x    --INT9 is 0x%x    --INT10 is 0x%x\n", low_int_status, high_int_status,
           complete_int_status);
    ret = ad7147_i2c_read(ad7147_data_global->client, 0xf1, &ambinet);
    ret = ad7147_i2c_read(ad7147_data_global->client, 0xf9, &max_avg);
    ret = ad7147_i2c_read(ad7147_data_global->client, 0xfa, &h_th);
    ret = ad7147_i2c_read(ad7147_data_global->client, 0x100, &min_avg);
    ret = ad7147_i2c_read(ad7147_data_global->client, 0x101, &l_th);
    mutex_unlock(&ad7147_data_global->mutex);
    printk(KERN_ERR "Ambinet: %d\n", ambinet);
    printk(KERN_ERR "max_avg: %d\n", max_avg);
    printk(KERN_ERR "h_th   : %d\n", h_th);
    printk(KERN_ERR "min_avg: %d\n", min_avg);
    printk(KERN_ERR "l_th   : %d\n", l_th);

    printk(KERN_ERR "\n\n\n");

	return 0;
}

static void ad7147_log_work_func(struct work_struct * work)
{
    ad7147_log();
    schedule_delayed_work(&ad7147_log_work, msecs_to_jiffies(DELAY_TIME));
}

static void ad7147_work_func(struct work_struct * work)
{
    if (ad7147_interrupt_func() == EVENT_NO_CHANGE) {
        // if the hard interrupt did not enable the touch or notouch event,
        // start this work to adjust the touch or notouch event every 1s.
        schedule_delayed_work(&ad7147_work, msecs_to_jiffies(DELAY_TIME));
    } 
}

static ssize_t ad7147_get_cdcval(struct device *          dev,
                                 struct device_attribute *attr,
                                 char *                   buf)
{
    unsigned short status = 0;
    int ret = -1;
    mutex_lock(&ad7147_data_global->mutex);
    ret = ad7147_i2c_read(ad7147_data_global->client, ADCRESULT_S0, &status);
    mutex_unlock(&ad7147_data_global->mutex);
    if (ret < 0)
    {
        printk(KERN_ERR "read  ad7147 ADCRESULT_S0 fail\n");
        return ret;
    }

    printk(KERN_ERR "ADCRESULT_S0 val is :%d\n", status);

    ret = sprintf(buf, "%d\n", status);

    return ret;
}

static DEVICE_ATTR(getcdc, 0444, ad7147_get_cdcval, NULL);

static int int2byte(int res, char *byte, int length)
{
	if (length < 4) {
		printk(KERN_ERR "[%s] length(%d) is too short\n", __func__, length);
		return -1;
	} else {
	    byte[0] = (res & 0xff);
	    byte[1] = ((res >> 8) & 0xff);
	    byte[2] = ((res >> 16) & 0xff);
	    byte[3] = ((res >> 24));
	}

    return 0;
}

static int byte2int(char *res)
{
    int target = (res[0] & 0xff) | ((res[1] << 8) & 0xff00) | ((res[2] << 24) >> 8) | (res[3] << 24);
    return target;
}


static int ad7147_read_nv(void)
{
	int ret = -1;

	memset(&nve_sar_info, 0, sizeof(struct nve_info_user));
	nve_sar_info.nv_number = AD7147_NV_NUMBER;
	strcpy(nve_sar_info.nv_name, AD7147_NV_NAME);
	nve_sar_info.valid_size = AD7147_NV_VALID_SIZE;
	nve_sar_info.nv_operation = NV_READ;

	ret = nve_direct_access(&nve_sar_info);
	if (ret) {
	    pr_err("%s: read nv partion failed\n", __func__);
	    return 0;
	}

    return byte2int(nve_sar_info.nv_data);
}

static int ad7147_write_nv(int value)
{
	int ret = -1;

	memset(&nve_sar_info, 0, sizeof(struct nve_info_user));
	nve_sar_info.nv_number = AD7147_NV_NUMBER;
	strcpy(nve_sar_info.nv_name, AD7147_NV_NAME);
	nve_sar_info.valid_size = AD7147_NV_VALID_SIZE;
	nve_sar_info.nv_operation = NV_WRITE;
	int2byte(value, nve_sar_info.nv_data, sizeof(nve_sar_info.nv_data));

	ret = nve_direct_access(&nve_sar_info);
	if (ret) {
	    pr_warning("%s: write nv partion failed\n", __func__);
	    return 0;
	}

}

static void read_nv_work_func(struct work_struct * work)
{

    unsigned short status = 0;
    int ret = -1;
    mutex_lock(&ad7147_data_global->mutex);
    ret = ad7147_i2c_read(ad7147_data_global->client, ADCRESULT_S0, &status);
    mutex_unlock(&ad7147_data_global->mutex);
    if (ret < 0)
    {
        printk(KERN_ERR "read  ad7147 ADCRESULT_S0 fail\n");
    }

    ad7147_nv_value = ad7147_read_nv();
    printk(KERN_ERR "[%s] get value from nv %d. cdc0 %d\n", __func__, ad7147_nv_value, status);

    if ((status - ad7147_nv_value) > TRIGERR_THRESHOLD) {
        ad7147_send_uevent(TOUCH);
    }

    enable_irq(ad7147_data_global->client->irq);
}


static int __devinit ad7147_probe(struct i2c_client *client, const struct i2c_device_id *devid)
{
    struct ad7147_data *ad7147_data = NULL;
    struct ad7147_platform_data *ad7147_platform_data = NULL;
    int status = -1;

    printk(KERN_ERR "[%s]: ad7147 probe into!", __func__);

    if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
    {
        goto err_kzalloc;
    }

    ad7147_data = kzalloc(sizeof(struct ad7147_data), GFP_KERNEL);
    if (ad7147_data == NULL){
        printk(KERN_ERR "failed to allocate memory for ad7147_data");
        goto err_kzalloc;
    }

    ad7147_platform_data = client->dev.platform_data;

    ad7147_data->client = client;
    ad7147_data->client->dev.init_name = "ad7147";
    ad7147_data_global = ad7147_data;
    i2c_set_clientdata(client, ad7147_data);

    //gpio init
    ad7147_platform_data->ad7147_gpio_config(1);

    //power supply init
    status = ad7147_pwr_init(ad7147_data, ad7147_platform_data);
    if (status < 0)
    {
        printk(KERN_ERR "ad7147 pwr init failed");
        goto error_init;
    }
    printk(KERN_ERR "ad7147 pwr init success");

    //detect device ID
    status = ad7147_hw_detect(ad7147_data);
    if (status < 0)
    {
        printk(KERN_ERR "ad7147 detect failed");
        goto error_init;
    }
    printk(KERN_ERR "ad7147 detect sucdess");

    //register init
    status = ad7147_reg_init(ad7147_data);
    if (status < 0)
    {
        printk(KERN_ERR "ad7147 reg init failed");
        goto error_init;
    }
    printk(KERN_ERR "ad7147 reg init success");
    mutex_init(&ad7147_data_global->mutex);

    status = device_create_file(&ad7147_data->client->dev, &dev_attr_status);
    if (status < 0)
    {
        printk(KERN_ERR "Failed to device_create_file status\n");
        goto error_init0;
    }

    status = device_create_file(&ad7147_data->client->dev, &dev_attr_register);
    if (status < 0)
    {
        printk(KERN_ERR "Failed to device_create_file register\n");
        goto error_init1;
    }

    status = device_create_file(&ad7147_data->client->dev, &dev_attr_getcdc);
    if (status < 0)
    {
        printk(KERN_ERR "Failed to device_create_file getcdc\n");
        goto error_init2;
    }
	
    status = device_create_file(&ad7147_data->client->dev, &dev_attr_enable_uevent);
    if (status < 0)
    {
        printk(KERN_ERR "Failed to device_create_file enable_uevent\n");
        // if error, still go ahead because enable/disable uevent is just for test program.
    }
   
    ad7147_data->client->irq = gpio_to_irq(ad7147_platform_data->ad7147_int_gpio);
   
    status = request_threaded_irq(ad7147_data->client->irq, ad7147_interrupt_handler,NULL,
                                  IRQF_TRIGGER_FALLING, "ad7147", ad7147_data);
   
    disable_irq(ad7147_data->client->irq);

    if (status < 0)
    {
        printk(KERN_ERR "can't allocate irq %d\n", client->irq);
        goto error_irq;
    }

    schedule_delayed_work(&read_nv_work, msecs_to_jiffies(50000));
    printk(KERN_ERR "[%s]: ad7147 probe success!", __func__);

    return 0;

error_irq:
error_init2:
    device_remove_file(&ad7147_data->client->dev, &dev_attr_getcdc);
error_init1:
    device_remove_file(&ad7147_data->client->dev, &dev_attr_register);
error_init0:
    device_remove_file(&ad7147_data->client->dev, &dev_attr_status);
error_init:
    kfree(ad7147_data);
    ad7147_data = NULL;
err_kzalloc:
    printk(KERN_ERR "[%s]: ad7147 probe error", __func__);
    return -1;
}

static int ad7147_remove(struct i2c_client *client)
{
    struct ad7147_data *ad7147_data = i2c_get_clientdata(client);
    struct ad7147_platform_data *pdata = client->dev.platform_data;

    free_irq(ad7147_data->client->irq, NULL);

   
    device_remove_file(&ad7147_data->client->dev, &dev_attr_enable_uevent);
    device_remove_file(&ad7147_data->client->dev, &dev_attr_getcdc);
    device_remove_file(&ad7147_data->client->dev, &dev_attr_register);
    device_remove_file(&ad7147_data->client->dev, &dev_attr_status);

    gpio_free(pdata->ad7147_int_gpio);
    kfree(ad7147_data);
    ad7147_data = NULL;

    return 0;
}
#ifdef CONFIG_PM
static int ad7147_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
        volatile unsigned short reg = 0;
		
	disable_irq(client->irq);
	cancel_delayed_work_sync(&ad7147_work);
	
	mutex_lock(&ad7147_data_global->mutex);
	ad7147_i2c_read(client, PWR_CONTROL,&reg);
	ad7147_i2c_write(client, PWR_CONTROL, reg|0x03);
	mutex_unlock(&ad7147_data_global->mutex);

	return 0;
}

static int ad7147_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	volatile unsigned short reg = 0;
	
	mutex_lock(&ad7147_data_global->mutex);
	ad7147_i2c_read(client, PWR_CONTROL,&reg);
    ad7147_i2c_write(client, PWR_CONTROL, reg&0xFFFFFFFC);	
    ad7147_clear_irq(client);
	mutex_unlock(&ad7147_data_global->mutex);
	
	enable_irq(client->irq);
	return 0;
}

static const struct dev_pm_ops ad7147_dev_pm_ops = {
	.suspend = ad7147_suspend,
	.resume  = ad7147_resume,
};
#endif
static const struct i2c_device_id ad7147_id[] =
{
    {"ad7147", 0},
    {},
};
MODULE_DEVICE_TABLE(i2c, ad7147_id);

static struct i2c_driver ad7147_driver =
{
    .driver   = {
        .name = "ad7147",
    #ifdef CONFIG_PM
    .pm   = &ad7147_dev_pm_ops,
    #endif
    },
    .probe    = ad7147_probe,
    .remove   = __devexit_p(ad7147_remove),
    .id_table = ad7147_id,
};

static int __init ad7147_init(void)
{
    return i2c_add_driver(&ad7147_driver);
}

static void __exit ad7147_exit(void)
{
    i2c_del_driver(&ad7147_driver);
}

module_init(ad7147_init);
module_exit(ad7147_exit);

