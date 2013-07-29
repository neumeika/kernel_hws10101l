/*
 *  apds990x.c - Linux kernel modules for ambient light + proximity sensor
 *
 *  Copyright (C) 2010 Lee Kai Koon <kai-koon.lee@avagotech.com>
 *  Copyright (C) 2010 Avago Technologies
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/input.h>
#include "apds990x.h"
#include <linux/board_sensors.h>
/* zkf55108 2011/10/26 modified begin */
#include <linux/gpio.h>
#include <mach/gpio.h>
#include <asm/io.h>
#include <linux/mux.h>
#include <linux/wakelock.h>
#define DEBUG 1
#define CLOSESENSOR_PIN "aa28"/*gpio_126*/
/* zkf55108 2011/10/26 modified end */
static struct wake_lock wlock;

/* Change History
 *
 * 1.0.1	Functions apds990x_show_rev(), apds990x_show_id() and apds990x_show_status()
 *			have missing CMD_BYTE in the i2c_smbus_read_byte_data(). APDS-990x needs
 *			CMD_BYTE for i2c write/read byte transaction.
 *
 *
 * 1.0.2	Include PS switching threshold level when interrupt occurred
 *
 *
 * 1.0.3	Implemented ISR and delay_work, correct PS threshold storing
 *
 * 1.0.4	Added Input Report Event
 */

/*
 * Management functions
 */

static int apds990x_gpio_init(enum pull_updown new_mode, char *pin_name);


static int apds990x_set_command(struct i2c_client *client, int command)
{
	struct apds990x_data *data = i2c_get_clientdata(client);
	int ret;
	int clearInt;

	if (command == 0)
		clearInt = CMD_CLR_PS_INT;
	else if (command == 1)
		clearInt = CMD_CLR_ALS_INT;
	else
		clearInt = CMD_CLR_PS_ALS_INT;

	mutex_lock(&data->update_lock);
	ret = i2c_smbus_write_byte(client, clearInt);
	mutex_unlock(&data->update_lock);

	return ret;
}

static int apds990x_set_enable(struct i2c_client *client, int enable)
{
	struct apds990x_data *data = i2c_get_clientdata(client);
	int ret;

	mutex_lock(&data->update_lock);
	ret = i2c_smbus_write_byte_data(client,
		CMD_BYTE | APDS990x_ENABLE_REG, enable);
	mutex_unlock(&data->update_lock);

	data->enable = enable;

	return ret;
}

static int apds990x_set_atime(struct i2c_client *client, int atime)
{
	struct apds990x_data *data = i2c_get_clientdata(client);
	int ret;

	mutex_lock(&data->update_lock);
	ret = i2c_smbus_write_byte_data(client,
		CMD_BYTE | APDS990x_ATIME_REG, atime);
	mutex_unlock(&data->update_lock);

	data->atime = atime;

	return ret;
}

static int apds990x_set_ptime(struct i2c_client *client, int ptime)
{
	struct apds990x_data *data = i2c_get_clientdata(client);
	int ret;

	mutex_lock(&data->update_lock);
	ret = i2c_smbus_write_byte_data(client,
		CMD_BYTE | APDS990x_PTIME_REG, ptime);
	mutex_unlock(&data->update_lock);

	data->ptime = ptime;

	return ret;
}

static int apds990x_set_wtime(struct i2c_client *client, int wtime)
{
	struct apds990x_data *data = i2c_get_clientdata(client);
	int ret;

	mutex_lock(&data->update_lock);
	ret = i2c_smbus_write_byte_data(client,
		CMD_BYTE | APDS990x_WTIME_REG, wtime);
	mutex_unlock(&data->update_lock);

	data->wtime = wtime;

	return ret;
}

static int apds990x_set_ailt(struct i2c_client *client, int threshold)
{
	struct apds990x_data *data = i2c_get_clientdata(client);
	int ret;

	mutex_lock(&data->update_lock);
	ret = i2c_smbus_write_word_data(client,
		CMD_WORD | APDS990x_AILTL_REG, threshold);
	mutex_unlock(&data->update_lock);

	data->ailt = threshold;

	return ret;
}

static int apds990x_set_aiht(struct i2c_client *client, int threshold)
{
	struct apds990x_data *data = i2c_get_clientdata(client);
	int ret;

	mutex_lock(&data->update_lock);
	ret = i2c_smbus_write_word_data(client,
		CMD_WORD | APDS990x_AIHTL_REG, threshold);
	mutex_unlock(&data->update_lock);

	data->aiht = threshold;

	return ret;
}

static int apds990x_set_pilt(struct i2c_client *client, int threshold)
{
	struct apds990x_data *data = i2c_get_clientdata(client);
	int ret;

	mutex_lock(&data->update_lock);
	ret = i2c_smbus_write_word_data(client,
		CMD_WORD | APDS990x_PILTL_REG, threshold);
	mutex_unlock(&data->update_lock);

	data->pilt = threshold;

	return ret;
}

static int apds990x_set_piht(struct i2c_client *client, int threshold)
{
	struct apds990x_data *data = i2c_get_clientdata(client);
	int ret;

	mutex_lock(&data->update_lock);
	ret = i2c_smbus_write_word_data(client,
		CMD_WORD | APDS990x_PIHTL_REG, threshold);
	mutex_unlock(&data->update_lock);

	data->piht = threshold;

	return ret;
}

static int apds990x_set_pers(struct i2c_client *client, int pers)
{
	struct apds990x_data *data = i2c_get_clientdata(client);
	int ret;

	mutex_lock(&data->update_lock);
	ret = i2c_smbus_write_byte_data(client,
		CMD_BYTE | APDS990x_PERS_REG, pers);
	mutex_unlock(&data->update_lock);

	data->pers = pers;

	return ret;
}

static int apds990x_set_config(struct i2c_client *client, int config)
{
	struct apds990x_data *data = i2c_get_clientdata(client);
	int ret;

	mutex_lock(&data->update_lock);
	ret = i2c_smbus_write_byte_data(client,
		CMD_BYTE | APDS990x_CONFIG_REG, config);
	mutex_unlock(&data->update_lock);

	data->config = config;

	return ret;
}

static int apds990x_set_ppcount(struct i2c_client *client, int ppcount)
{
	struct apds990x_data *data = i2c_get_clientdata(client);
	int ret;

	mutex_lock(&data->update_lock);
	ret = i2c_smbus_write_byte_data(client,
		CMD_BYTE | APDS990x_PPCOUNT_REG, ppcount);
	mutex_unlock(&data->update_lock);

	data->ppcount = ppcount;

	return ret;
}

static int apds990x_set_control(struct i2c_client *client, int control)
{
	struct apds990x_data *data = i2c_get_clientdata(client);
	int ret;

	mutex_lock(&data->update_lock);
	ret = i2c_smbus_write_byte_data(client,
		CMD_BYTE | APDS990x_CONTROL_REG, control);
	mutex_unlock(&data->update_lock);

	data->control = control;

	/* obtain ALS gain value */
	if ((data->control & 0x03) == 0x00) /* 1X Gain */
		data->als_gain = 1;
	else if ((data->control & 0x03) == 0x01) /* 8X Gain */
		data->als_gain = 8;
	else if ((data->control & 0x03) == 0x02) /* 16X Gain */
		data->als_gain = 16;
	else  /* 120X Gain */
		data->als_gain = 120;

	return ret;
}

static int LuxCalculation(struct i2c_client *client, int cdata, int irdata)
{
	struct apds990x_data *data = i2c_get_clientdata(client);
	int luxValue = 0;

	int IAC1 = 0;
	int IAC2 = 0;
	int IAC = 0;
	int GA = 48;			/* 0.48 without glass window */
	int COE_B = 223;		/* 2.23 without glass window */
	int COE_C = 70;			/* 0.70 without glass window */
	int COE_D = 142;		/* 1.42 without glass window */
	int DF = 52;

	/* re-adjust COE_B to avoid 2 decimal point */
	IAC1 = (cdata - (COE_B * irdata) / 100);

	/* re-adjust COE_C and COE_D to void 2 decimal point */
	IAC2 = ((COE_C * cdata) / 100 - (COE_D*irdata) / 100);


	if (IAC1 > IAC2)
		IAC = IAC1;
	else
		IAC = IAC2;

	/* modify FAE */
    /* luxValue = ((IAC*GA*DF)/100)/(((272*(256-data->atime))/100) *data->als_gain); */
	luxValue = SCALE * ((IAC * GA * DF) / 100) /
		(((272 * (256 - data->atime)) / 100) * data->als_gain);

	return luxValue;
}

static void apds990x_change_ps_threshold(struct i2c_client *client)
{
	struct apds990x_data *data = i2c_get_clientdata(client);

	data->ps_data =	i2c_smbus_read_word_data(client,
		CMD_WORD|APDS990x_PDATAL_REG);

	if ((data->ps_data > data->pilt) && (data->ps_data >= data->piht)) {
		/* far-to-near detected */
		data->ps_detection = 1;

		/*input_report_abs(data->input_dev_ps, ABS_DISTANCE, 1);
			FAR-to-NEAR detection */
		input_report_abs(data->input_dev_ps, ABS_DISTANCE, 0);

		input_sync(data->input_dev_ps);

		i2c_smbus_write_word_data(client,
			CMD_WORD | APDS990x_PILTL_REG, data->ps_hysteresis_threshold);
		i2c_smbus_write_word_data(client,
			CMD_WORD | APDS990x_PIHTL_REG, 1023);

		data->pilt = data->ps_hysteresis_threshold;
		data->piht = 1023;

		dev_dbg(&client->dev, "far-to-near detected\n");
	} else if ((data->ps_data <= data->pilt) && (data->ps_data < data->piht)) {
		/* near-to-far detected */
		data->ps_detection = 0;

		/*input_report_abs(data->input_dev_ps, ABS_DISTANCE, 0);*/
		/* NEAR-to-FAR detection */
		input_report_abs(data->input_dev_ps, ABS_DISTANCE, 1);

		input_sync(data->input_dev_ps);

		i2c_smbus_write_word_data(client,
			CMD_WORD | APDS990x_PILTL_REG, 0);
		i2c_smbus_write_word_data(client,
			CMD_WORD | APDS990x_PIHTL_REG, data->ps_threshold);

		data->pilt = 0;
		data->piht = data->ps_threshold;

		dev_dbg(&client->dev, "near-to-far detected\n");
	}
}

static void apds990x_change_als_threshold(struct i2c_client *client)
{
	struct apds990x_data *data = i2c_get_clientdata(client);
	int cdata, irdata;
	int luxValue = 0;

	cdata = i2c_smbus_read_word_data(client, CMD_WORD | APDS990x_CDATAL_REG);
	irdata = i2c_smbus_read_word_data(client, CMD_WORD | APDS990x_IRDATAL_REG);

	luxValue = LuxCalculation(client, cdata, irdata);

	luxValue = luxValue > 0 ? luxValue : 0;
	luxValue = luxValue < 10000 ? luxValue : 10000;

	/* check PS under sunlight */
	if ((data->ps_detection == 1) &&
			(cdata > (75 * (1024 * (256 - data->atime))) / 100)) {
		/*PS was previously in far-to-near condition*/
		/* need to inform input event as there will be no interrupt from the PS */
		/* NEAR-to-FAR detection */
		input_report_abs(data->input_dev_ps, ABS_DISTANCE, 1);

		input_sync(data->input_dev_ps);

		i2c_smbus_write_word_data(client,
			CMD_WORD | APDS990x_PILTL_REG, 0);
		i2c_smbus_write_word_data(client,
			CMD_WORD | APDS990x_PIHTL_REG, data->ps_threshold);

		data->pilt = 0;
		data->piht = data->ps_threshold;

		data->ps_detection = 0;	/* near-to-far detected */

		dev_dbg(&client->dev, "apds_990x_proximity_handler = FAR\n");
	}


	input_report_abs(data->input_dev_als, ABS_MISC, luxValue); /* report the lux level */
	input_sync(data->input_dev_als);

	data->als_data = cdata;

	data->als_threshold_l = (data->als_data *
		(100 - APDS990x_ALS_THRESHOLD_HSYTERESIS)) / 100;
	data->als_threshold_h = (data->als_data *
		(100 + APDS990x_ALS_THRESHOLD_HSYTERESIS)) / 100;

	if (data->als_threshold_h >= 65535)
		data->als_threshold_h = 65535;

	i2c_smbus_write_word_data(client,
		CMD_WORD | APDS990x_AILTL_REG, data->als_threshold_l);

	i2c_smbus_write_word_data(client,
		CMD_WORD | APDS990x_AIHTL_REG, data->als_threshold_h);
}

static void apds990x_reschedule_work(struct apds990x_data *data,
					  unsigned long delay)
{
	/* unsigned long flags; */
    /* dev_dbg(&client->dev, "reschedule_work enter\n");*/
	/* spin_lock_irqsave(&data->update_lock, flags);*/

	/*
	 * If work is already scheduled then subsequent schedules will not
	 * change the scheduled time that's why we have to cancel it first.
	 */
	__cancel_delayed_work(&data->dwork);
	schedule_delayed_work(&data->dwork, delay);

	/* spin_unlock_irqrestore(&data->update_lock, flags); */
}

/* ALS polling routine */
static void apds990x_als_polling_work_handler(struct work_struct *work)
{
	struct apds990x_data *data = container_of(work,
		struct apds990x_data, als_dwork.work);
	struct i2c_client *client = data->client;
	int cdata, irdata, pdata;
	int luxValue = 0;

	cdata = i2c_smbus_read_word_data(client, CMD_WORD | APDS990x_CDATAL_REG);
	irdata = i2c_smbus_read_word_data(client, CMD_WORD | APDS990x_IRDATAL_REG);
	pdata = i2c_smbus_read_word_data(client, CMD_WORD | APDS990x_PDATAL_REG);

	luxValue = LuxCalculation(client, cdata, irdata);

	luxValue = luxValue > 0 ? luxValue : 0;
	luxValue = luxValue < 10000 ? luxValue : 10000;

	dev_dbg(&client->dev, "%s: lux = %d cdata = %x irdata = %x pdata = %x \n",
			__func__, luxValue, cdata, irdata, pdata);

	/* check PS under sunlight */
	if ((data->ps_detection == 1) && (cdata > (75 * (1024 * (256 - data->atime))) / 100)) {

		/* PS was previously in far-to-near condition*/
		/* need to inform input event as there will be no interrupt from the PS */

		/* NEAR-to-FAR detection */
		input_report_abs(data->input_dev_ps, ABS_DISTANCE, 1);
		input_sync(data->input_dev_ps);

		i2c_smbus_write_word_data(client,
			CMD_WORD | APDS990x_PILTL_REG, 0);
		i2c_smbus_write_word_data(client,
			CMD_WORD | APDS990x_PIHTL_REG, data->ps_threshold);

		data->pilt = 0;
		data->piht = data->ps_threshold;

		data->ps_detection = 0;	/* near-to-far detected */

		dev_dbg(&client->dev, "apds_990x_proximity_handler = FAR\n");
	}

    input_report_abs(data->input_dev_als, ABS_MISC, luxValue); /* report the lux level */
    input_sync(data->input_dev_als);

	/* restart timer */
	schedule_delayed_work(&data->als_dwork,
		msecs_to_jiffies(data->als_poll_delay));
}

/* PS interrupt routine */
static void apds990x_work_handler(struct work_struct *work)
{
	struct apds990x_data *data = container_of(work,
		struct apds990x_data, dwork.work);
	struct i2c_client *client = data->client;
	int	status;
	int cdata;


	status = i2c_smbus_read_byte_data(client, CMD_BYTE | APDS990x_STATUS_REG);

	/* disable 990x's ADC first */
	i2c_smbus_write_byte_data(client, CMD_BYTE | APDS990x_ENABLE_REG, 1);

	dev_dbg(&client->dev, "status = %x\n", status);

	if ((status & data->enable & 0x30) == 0x30) {
		/* both PS and ALS are interrupted */
		apds990x_change_als_threshold(client);

		cdata = i2c_smbus_read_word_data(client, CMD_WORD | APDS990x_CDATAL_REG);
		if (cdata < (75 * (1024 * (256 - data->atime))) / 100)
			apds990x_change_ps_threshold(client);
		else {
			if (data->ps_detection == 1) {
				apds990x_change_ps_threshold(client);
			} else {
				dev_dbg(&client->dev, "Triggered by background ambient noise\n");
			}
		}

		apds990x_set_command(client, 2);	/* 2 = CMD_CLR_PS_ALS_INT */
	} else if ((status & data->enable & 0x20) == 0x20) {
		/* only PS is interrupted */

		/* check if this is triggered by background ambient noise */
		cdata = i2c_smbus_read_word_data(client,
			CMD_WORD | APDS990x_CDATAL_REG);
		if (cdata < (75 * (1024 * (256 - data->atime))) / 100)
			apds990x_change_ps_threshold(client);
		else {
			if (data->ps_detection == 1) {
				apds990x_change_ps_threshold(client);
			} else {
				dev_dbg(&client->dev, "Triggered by background ambient noise\n");
			}
		}

		apds990x_set_command(client, 0);	/* 0 = CMD_CLR_PS_INT */
	} else if ((status & data->enable & 0x10) == 0x10) {
		/* only ALS is interrupted */
		apds990x_change_als_threshold(client);

		apds990x_set_command(client, 1);	/* 1 = CMD_CLR_ALS_INT */
	}

	i2c_smbus_write_byte_data(client,
		CMD_BYTE | APDS990x_ENABLE_REG, data->enable);
    enable_irq(client->irq);
	wake_unlock(&wlock);

}

/* assume this is ISR */
static irqreturn_t apds990x_interrupt(int vec, void *info)
{
	struct i2c_client *client = (struct i2c_client *)info;
	struct apds990x_data *data = i2c_get_clientdata(client);
	wake_lock(&wlock);
    disable_irq_nosync(client->irq);
	apds990x_reschedule_work(data, 0);

	return IRQ_HANDLED;
}

/*
 * SysFS support
 */

static ssize_t apds990x_show_enable_ps_sensor(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct apds990x_data *data = i2c_get_clientdata(client);

	return sprintf(buf, "%d\n", data->enable_ps_sensor);
}

static ssize_t apds990x_store_enable_ps_sensor(struct device *dev,
				struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct apds990x_data *data = i2c_get_clientdata(client);
	unsigned long val = simple_strtoul(buf, NULL, 10);
	unsigned long flags;
	int err = -1;
	printk("[%s] val=%ld\n", __func__, val);

	dev_dbg(dev, "%s: enable ps senosr ( %ld)\n", __func__, val);

	if ((val != 0) && (val != 1)) {
		dev_dbg(dev, "%s:store unvalid value=%ld\n", __func__, val);
		return count;
	}

	if (data->ps_lock == 0) {
		dev_dbg(dev, "%s:ps lock value=%u\n", __func__, data->ps_lock);
		return count;
	}
	if (val == 1) {
		err = apds990x_gpio_init(PULLUP, CLOSESENSOR_PIN);
		if (err < 0) {
			dev_err(&client->dev, "%s: failed to init gpio\n", __func__);
			return err;
		}
		/*turn on p sensor
		first status is FAR */
		input_report_abs(data->input_dev_ps, ABS_DISTANCE, 1);
		input_sync(data->input_dev_ps);
		if (data->enable_ps_sensor == 0) {

			data->enable_ps_sensor = 1;

			apds990x_set_enable(client, 0); /* Power Off */
			apds990x_set_atime(client, 0xf6); /* 27.2ms */
			apds990x_set_ptime(client, 0xff); /* 2.72ms */

			/*apds990x_set_ppcount(client, 8);*/ /* 8-pulse */
			apds990x_set_ppcount(client, 5); /* 5-pulse */
			/*modify FAE */
			/*apds990x_set_control(client, 0x20); */ /* 100mA, IR-diode, 1X PGAIN, 1X AGAIN */

			/* 50mA, IR-diode, 1X PGAIN, 1X AGAIN */
			apds990x_set_control(client, 0x62);
			/* init threshold for proximity */
			apds990x_set_pilt(client, 0);
			apds990x_set_piht(client, APDS990x_PS_DETECTION_THRESHOLD);

			data->ps_threshold = APDS990x_PS_DETECTION_THRESHOLD;
			data->ps_hysteresis_threshold = APDS990x_PS_HSYTERESIS_THRESHOLD;

			apds990x_set_ailt(client, 0);
			apds990x_set_aiht(client, 0xffff);

			apds990x_set_pers(client, 0x33); /* 3 persistence */

			if (data->enable_als_sensor == 0) {

				/* we need this polling timer routine for sunlight canellation */
				spin_lock_irqsave(&data->update_lock.wait_lock, flags);

				/*
				 * If work is already scheduled then subsequent schedules will not
				 * change the scheduled time that's why we have to cancel it first.
				 */
				__cancel_delayed_work(&data->als_dwork);
				schedule_delayed_work(&data->als_dwork,
					msecs_to_jiffies(data->als_poll_delay));	/* 100ms */

				spin_unlock_irqrestore(&data->update_lock.wait_lock, flags);
			}

			/* enable PS interrupt, enable ps and als function */
			apds990x_set_enable(client, 0x27);
		}
	} else {
		/*turn off p sensor - kk 25 Apr 2011 we can't turn off the entire sensor, the light sensor may be needed by HAL*/
		data->enable_ps_sensor = 0;
		if (data->enable_als_sensor) {

			/* reconfigute light sensor setting */
			apds990x_set_enable(client, 0); /* Power Off */

			apds990x_set_atime(client, data->als_atime);  /* previous als poll delay */

			apds990x_set_ailt(client, 0);
			apds990x_set_aiht(client, 0xffff);

			/* modify FAE */
			/* apds990x_set_control(client, 0x20);*/ /* 100mA, IR-diode, 1X PGAIN, 1X AGAIN */

			/* 50mA, IR-diode, 1X PGAIN, 16X AGAIN */
			apds990x_set_control(client, 0x62);
			apds990x_set_pers(client, 0x33); /* 3 persistence */

			apds990x_set_enable(client, 0x3);	 /* only enable light sensor */

			spin_lock_irqsave(&data->update_lock.wait_lock, flags);

			/*
			 * If work is already scheduled then subsequent schedules will not
			 * change the scheduled time that's why we have to cancel it first.
			 */
			__cancel_delayed_work(&data->als_dwork);
			schedule_delayed_work(&data->als_dwork, msecs_to_jiffies(data->als_poll_delay));	/* 100ms */

			spin_unlock_irqrestore(&data->update_lock.wait_lock, flags);

		} else {
			apds990x_set_enable(client, 0);

			spin_lock_irqsave(&data->update_lock.wait_lock, flags);

			/*
			 * If work is already scheduled then subsequent schedules will not
			 * change the scheduled time that's why we have to cancel it first.
			 */
			__cancel_delayed_work(&data->als_dwork);

			spin_unlock_irqrestore(&data->update_lock.wait_lock, flags);
		}
		err = apds990x_gpio_init(PULLUP, CLOSESENSOR_PIN);
		if (err < 0) {
			dev_err(&client->dev, "%s: failed to PULLUP gpio\n",
				__func__);
			return err;
		}

	}


	return count;
}

static DEVICE_ATTR(enable_ps_sensor, 0664,
				   apds990x_show_enable_ps_sensor, apds990x_store_enable_ps_sensor);

static ssize_t apds990x_show_enable_als_sensor(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct apds990x_data *data = i2c_get_clientdata(client);

	return sprintf(buf, "%d\n", data->enable_als_sensor);
}

static ssize_t apds990x_store_enable_als_sensor(struct device *dev,
				struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct apds990x_data *data = i2c_get_clientdata(client);
	unsigned long val = simple_strtoul(buf, NULL, 10);
	unsigned long flags;

	dev_dbg(dev, "%s: enable als sensor ( %ld)\n",
		__func__, val);

	if ((val != 0) && (val != 1)) {
		dev_dbg(dev, "%s: enable als sensor=%ld\n",
			__func__, val);
		return count;
	}
	printk("[%s] val=%ld\n", __func__, val);
	if (val == 1) {
		/* turn on light sensor */
		if (data->enable_als_sensor == 0) {

			data->enable_als_sensor = 1;

			apds990x_set_enable(client, 0); /* Power Off */

			apds990x_set_ailt(client, 0);
			apds990x_set_aiht(client, 0xffff);
			/* modify 1020 */
			/* apds990x_set_control(client, 0x20); */ /* 100mA, IR-diode, 1X PGAIN, 1X AGAIN */
			apds990x_set_control(client, 0x62); /* 50mA, IR-diode, 1X PGAIN, 16X AGAIN */
			apds990x_set_pers(client, 0x33); /* 3 persistence */

			if (data->enable_ps_sensor) {
				apds990x_set_atime(client, 0xf6); /* 27.2ms */
				apds990x_set_ptime(client, 0xff); /* 2.72ms */

			/*	apds990x_set_ppcount(client, 8);*/ /* 8-pulse */
				apds990x_set_ppcount(client, 5); /* 5-pulse */

				/* if prox sensor was activated previously */
				apds990x_set_enable(client, 0x27);
			} else {
				apds990x_set_atime(client, data->als_atime);  /* 100.64ms */
				apds990x_set_enable(client, 0x3);	 /* only enable light sensor */
			}

			spin_lock_irqsave(&data->update_lock.wait_lock, flags);

			/*
			 * If work is already scheduled then subsequent schedules will not
			 * change the scheduled time that's why we have to cancel it first.
			 */
			__cancel_delayed_work(&data->als_dwork);
			schedule_delayed_work(&data->als_dwork,
				msecs_to_jiffies(data->als_poll_delay));

			spin_unlock_irqrestore(&data->update_lock.wait_lock, flags);
		}
	} else {
		/*turn off light sensor
		what if the p sensor is active?*/
		data->enable_als_sensor = 0;
		if (data->enable_ps_sensor) {
			apds990x_set_enable(client, 0); /* Power Off */
			apds990x_set_atime(client, 0xf6); /* 27.2ms */
			apds990x_set_ptime(client, 0xff); /* 2.72ms */
			/*apds990x_set_ppcount(client, 8);*/ /* 8-pulse */
			apds990x_set_ppcount(client, 5); /* 5-pulse */
			/* modify */
			/* apds990x_set_control(client, 0x20); */ /* 100mA, IR-diode, 1X PGAIN, 1X AGAIN */

			/* 50mA, IR-diode, 1X PGAIN, 16X AGAIN */
			apds990x_set_control(client, 0x62);

			apds990x_set_piht(client, 0);
			apds990x_set_piht(client, APDS990x_PS_DETECTION_THRESHOLD);

			apds990x_set_ailt(client, 0);
			apds990x_set_aiht(client, 0xffff);

			/* 3 persistence */
			apds990x_set_pers(client, 0x33);

			/* only enable prox sensor with interrupt */
			apds990x_set_enable(client, 0x27);
		} else {
			apds990x_set_enable(client, 0);
		}


		spin_lock_irqsave(&data->update_lock.wait_lock, flags);

		/*
		 * If work is already scheduled then subsequent schedules will not
		 * change the scheduled time that's why we have to cancel it first.
		 */
		__cancel_delayed_work(&data->als_dwork);

		spin_unlock_irqrestore(&data->update_lock.wait_lock, flags);
	}

	return count;
}

static DEVICE_ATTR(enable_als_sensor, 0664,
				   apds990x_show_enable_als_sensor, apds990x_store_enable_als_sensor);

static ssize_t apds990x_show_als_poll_delay(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct apds990x_data *data = i2c_get_clientdata(client);

	/* return in micro-second */
	return sprintf(buf, "%d\n", data->als_poll_delay*1000);
}

static ssize_t apds990x_store_als_poll_delay(struct device *dev,
					struct device_attribute *attr, const char *buf, size_t count)
{

	struct i2c_client *client = to_i2c_client(dev);
	struct apds990x_data *data = i2c_get_clientdata(client);
	unsigned long val = simple_strtoul(buf, NULL, 10);
	int ret;
	int poll_delay = 0;
	/*unsigned long flags;*/
	printk("[%s] val=%ld\n", __func__, val);

	if (val < 5000)
		val = 5000;	/* minimum 5ms*/

	data->als_poll_delay = val / 1000;	/* convert us => ms */

	/* the minimum is 2.72ms = 2720 us, maximum is 696.32ms */
	poll_delay = 256 - (val / 2720);

	if (poll_delay < 0)
		data->als_atime = 0;
	else
		data->als_atime = poll_delay;

	if (data->enable_ps_sensor != 1) {
	ret = apds990x_set_atime(client, data->als_atime);
	if (ret < 0)
		return ret;
	}

	/* we need this polling timer routine for sunlight canellation */
	/*spin_lock_irqsave(&data->update_lock.wait_lock, flags);*/

	/*
	 * If work is already scheduled then subsequent schedules will not
	 * change the scheduled time that's why we have to cancel it first.
	 */
	/*__cancel_delayed_work(&data->als_dwork);
	schedule_delayed_work(&data->als_dwork, msecs_to_jiffies(data->als_poll_delay));*/	/* 100ms */

	/*spin_unlock_irqrestore(&data->update_lock.wait_lock, flags); */

	return count;

}

static DEVICE_ATTR(als_poll_delay, S_IWUSR | S_IRUGO,
				   apds990x_show_als_poll_delay, apds990x_store_als_poll_delay);

static ssize_t apds990x_show_ps_enable(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct apds990x_data *data = i2c_get_clientdata(client);

	return sprintf(buf, "%d\n", data->ps_lock);
}
static ssize_t apds990x_store_ps_enable(struct device *dev,
				struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct apds990x_data *data = i2c_get_clientdata(client);
	unsigned long val = simple_strtoul(buf, NULL, 10);
	printk("[%s] val=%lu\n", __func__, val);

	if (val != 0) {
		if (data->enable_ps_sensor == 0) {
			data->ps_lock = 1;
		} else {/*ps opened */
			dev_dbg(dev, "%s:store ps lock value=%u\n", __func__, data->ps_lock);
		}
	} else {
		apds990x_store_enable_ps_sensor(dev,
				attr, buf, count);
		data->ps_lock = 0;

		input_report_abs(data->input_dev_ps, ABS_DISTANCE, 1);
		input_sync(data->input_dev_ps);
	}
	return count;
}

static DEVICE_ATTR(ps_enable, 0664,
				   apds990x_show_ps_enable, apds990x_store_ps_enable);
static struct attribute *apds990x_attributes[] = {
	&dev_attr_enable_ps_sensor.attr,
	&dev_attr_enable_als_sensor.attr,
	&dev_attr_als_poll_delay.attr,
	&dev_attr_ps_enable.attr,
	NULL
};


static const struct attribute_group apds990x_attr_group = {
	.attrs = apds990x_attributes,
};

/*
 * Initialization function
 */

static int apds990x_init_client(struct i2c_client *client)
{
	struct apds990x_data *data = i2c_get_clientdata(client);
	int err;
	int id;

	err = apds990x_set_enable(client, 0);

	if (err < 0) {
		dev_err(&client->dev, "%s, apds990x_set_enable failed\n",
				__func__);
		return err;
	}

	id = i2c_smbus_read_byte_data(client, CMD_BYTE | APDS990x_ID_REG);
	if (id == 0x20) {
		dev_dbg(&client->dev, "APDS-9901\n");
	} else if (id == 0x29) {
		dev_dbg(&client->dev, "APDS-990x\n");
	} else {
		dev_dbg(&client->dev, "Neither APDS-9901 nor APDS-9901\n");
		return -EIO;
	}
	dev_info(&client->dev, "Read apds990x chip ok, ID is 0x%x \n", id);
	apds990x_set_atime(client, 0xDB);	/* 100.64ms ALS integration time*/
	apds990x_set_ptime(client, 0xFF);	/* 2.72ms Prox integration time*/
	apds990x_set_wtime(client, 0xFF);	/* 2.72ms Wait time*/

    apds990x_set_ppcount(client, 0x05);	/* 5-Pulse for proximity*/

	apds990x_set_config(client, 0);		/* no long wait */

	apds990x_set_control(client, 0x62);	/* 50mA, IR-diode, 1X PGAIN, 16X AGAIN*/

	apds990x_set_pilt(client, 0);		/* init threshold for proximity*/
	apds990x_set_piht(client, APDS990x_PS_DETECTION_THRESHOLD);

	data->ps_threshold =
		APDS990x_PS_DETECTION_THRESHOLD;
	data->ps_hysteresis_threshold =
		APDS990x_PS_HSYTERESIS_THRESHOLD;

	/* init threshold for als*/
	apds990x_set_ailt(client, 0);
	apds990x_set_aiht(client, 0xFFFF);

	/* 2 consecutive Interrupt persistence*/
	apds990x_set_pers(client, 0x22);

	/* sensor is in disabled mode but all the configurations are preset*/

	return 0;
}
/* zkf55108 2011/10/27 add begin */
static int apds990x_gpio_init(enum pull_updown new_mode, char *pin_name)
{
	int ret = 0;
	struct  iomux_pin *pinTemp;

	pinTemp = iomux_get_pin(pin_name);
	if (!pinTemp) {
		ret = -EINVAL;
		goto err;
	}
	ret = pinmux_setpullupdown(pinTemp, new_mode);
	if (ret < 0)
		goto err;
	return 0;

err:
	return ret;
}
/* zkf55108 2011/10/27 add end */

/*
 * I2C init/probing/exit functions
 */

static struct i2c_driver apds990x_driver;
static int __devinit apds990x_probe(struct i2c_client *client,
				   const struct i2c_device_id *id)
{
	struct apds990x_data *data;
	int err = 0;

	if (!i2c_check_functionality(client->adapter,
		I2C_FUNC_SMBUS_BYTE)) {
		err = -ENODEV;
		goto exit;
	}

	data = kzalloc(sizeof(struct apds990x_data), GFP_KERNEL);
	if (NULL == data) {
		err = -ENOMEM;
		goto exit;
	}
	data->client = client;
	i2c_set_clientdata(client, data);

	data->enable = 0;	/* default mode is standard */
	data->ps_lock = 1; /* default ps can be opened*/
	data->ps_threshold = 0;
	data->ps_hysteresis_threshold = 0;
	data->ps_detection = 0;	/* default to no detection */
	data->enable_als_sensor = 0;	/* default to 0 */
	data->enable_ps_sensor = 0;	/* default to 0 */
	/*data->als_poll_delay = 100;*/	/* default to 100ms*/
	data->als_poll_delay = 500;	/* default to 500ms*/
	data->als_atime	= 0xdb;			/* work in conjuction with als_poll_delay*/

	dev_dbg(&client->dev, "enable = %x\n", data->enable);

	mutex_init(&data->update_lock);
	INIT_DELAYED_WORK(&data->dwork, apds990x_work_handler);
	INIT_DELAYED_WORK(&data->als_dwork, apds990x_als_polling_work_handler);
	/* Initialize the APDS990x chip */
	err = apds990x_init_client(client);
	if (err)
		goto exit_kfree;

	err = apds990x_gpio_init(PULLUP, CLOSESENSOR_PIN);
	if (err < 0) {
		dev_err(&client->dev, "%s: gpio PULLUP failed\n",
			__func__);
		goto exit_kfree;
	}
	if (client->irq >= 0) {
		err = gpio_request(irq_to_gpio(client->irq), "gpio_apds990x");
		if (err) {
			dev_err(&client->dev, "%s: failed to request gpio int1 for irq\n",
				__func__);
			goto exit_kfree;
		}

		gpio_direction_input(irq_to_gpio(client->irq));

	/* Initialize wakelock*/
	wake_lock_init(&wlock, WAKE_LOCK_SUSPEND, "adps990x");

	err = request_irq(client->irq, apds990x_interrupt,
			IRQF_DISABLED | IRQ_TYPE_LEVEL_LOW | IRQF_NO_SUSPEND,
						APDS990x_DRV_NAME, (void *)client);
		if (err) {
			dev_err(&client->dev, "%s  allocate APDS990x_IRQ failed.\n",
				__func__);
			goto exit_gpio_request;
		}
	}

	dev_dbg(&client->dev, "%s interrupt is hooked\n", __func__);



	/* Register to Input Device */
	data->input_dev_als = input_allocate_device();
	if (!data->input_dev_als) {
		err = -ENOMEM;
		dev_err(&client->dev, "%s, Failed to allocate input device als\n",
			__func__);
		goto exit_free_irq;
	}

	data->input_dev_ps = input_allocate_device();
	if (!data->input_dev_ps) {
		err = -ENOMEM;
		dev_err(&client->dev, "%s, Failed to allocate input device ps\n",
			__func__);
		goto exit_free_dev_als;
	}

    data->input_dev_als->name = LIGHT_INPUT_DEV_NAME;
    data->input_dev_als->id.bustype = BUS_I2C;
	data->input_dev_als->dev.parent = &data->client->dev;
    data->input_dev_ps->name = PROX_INPUT_DEV_NAME;
    data->input_dev_ps->id.bustype = BUS_I2C;
	data->input_dev_ps->dev.parent = &data->client->dev;

	set_bit(EV_ABS, data->input_dev_als->evbit);
	set_bit(EV_ABS, data->input_dev_ps->evbit);

	input_set_abs_params(data->input_dev_als, ABS_MISC, 0, 10000, 0, 0);
	input_set_abs_params(data->input_dev_ps, ABS_DISTANCE, 0, 1, 0, 0);

	err = input_register_device(data->input_dev_als);
	if (err) {
		dev_err(&client->dev, "%s, Unable to register input device als: %s\n",
		       __func__, data->input_dev_als->name);
		goto exit_free_dev_ps;
	}

	err = input_register_device(data->input_dev_ps);
	if (err) {
		dev_err(&client->dev, "%s,Unable to register input device ps: %s\n",
		       __func__, data->input_dev_ps->name);
		goto exit_unregister_dev_als;
	}

	/* Register sysfs hooks */
	err = sysfs_create_group(&client->dev.kobj, &apds990x_attr_group);
	if (err) {
		dev_err(&client->dev, "%s, sysfs_create_group failed\n", __func__);
		goto exit_unregister_dev_ps;
	}

#ifdef CONFIG_S10_SENSORS_INPUT_INFO
	err = set_sensor_input(ALS, data->input_dev_als->dev.kobj.name);
	if (err) {
		sysfs_remove_group(&client->dev.kobj, &apds990x_attr_group);
		dev_err(&client->dev, "%s, set_sensor_input als error: %s\n",
		       __func__, data->input_dev_als->name);
		goto exit_unregister_dev_ps;
	}

	err = set_sensor_input(PS, data->input_dev_ps->dev.kobj.name);
	if (err) {
		sysfs_remove_group(&client->dev.kobj, &apds990x_attr_group);
		dev_err(&client->dev, "%s,set_sensor_input ps error: %s\n",
		       __func__, data->input_dev_ps->name);
		goto exit_unregister_dev_ps;
	}
#endif
	dev_dbg(&client->dev, "%s support ver. %s enabled\n",
		__func__, DRIVER_VERSION);

	return 0;

exit_unregister_dev_ps:
	input_unregister_device(data->input_dev_ps);
exit_unregister_dev_als:
	input_unregister_device(data->input_dev_als);
exit_free_dev_ps:
	input_free_device(data->input_dev_ps);
exit_free_dev_als:
	input_free_device(data->input_dev_als);
exit_free_irq:
	if (client->irq >= 0)
	free_irq(client->irq, client);
exit_gpio_request:
	wake_lock_destroy(&wlock);
	if (client->irq >= 0)
	gpio_free(irq_to_gpio(client->irq));
exit_kfree:
	i2c_set_clientdata(client, NULL);
	kfree(data);
	data = NULL;
exit:
	return err;
}

static int __devexit apds990x_remove(struct i2c_client *client)
{
	struct apds990x_data *data = i2c_get_clientdata(client);

	input_unregister_device(data->input_dev_als);
	input_unregister_device(data->input_dev_ps);

	input_free_device(data->input_dev_als);
	input_free_device(data->input_dev_ps);

	if (client->irq >= 0) {
		free_irq(client->irq, client);
		gpio_free(irq_to_gpio(client->irq));
	}

	sysfs_remove_group(&client->dev.kobj, &apds990x_attr_group);
	/* Power down the device */
	apds990x_set_enable(client, 0);
	if (apds990x_gpio_init(PULLUP, CLOSESENSOR_PIN) < 0)
		dev_err(&client->dev, "%s: failed to set gpio pullup mode\n",
			__func__);
	i2c_set_clientdata(client, NULL);
	kfree(data);
	data = NULL;

	return 0;
}

/* zkf55108 2011/11/3 add begin */
static void apds990x_shutdown(struct i2c_client *client)
{
	struct apds990x_data *data = i2c_get_clientdata(client);

	printk("[%s] +\n", __func__);

	if (client->irq >= 0) {
		free_irq(client->irq, client);
	}

	__cancel_delayed_work(&data->dwork);
	__cancel_delayed_work(&data->als_dwork);

	/* Power down the device */
	apds990x_set_enable(client, 0);
	if (apds990x_gpio_init(PULLUP, CLOSESENSOR_PIN) < 0)
		dev_err(&client->dev, "%s: failed to set gpio pullup mode\n",
			__func__);
	wake_lock_destroy(&wlock);
	printk("[%s] -\n", __func__);
}

/* zkf55108 2011/11/3 add end */

#ifdef CONFIG_PM

static int apds990x_suspend(struct i2c_client *client, pm_message_t mesg)
{

    struct apds990x_data *data = i2c_get_clientdata(client);
    unsigned long flags;
	printk("[%s] +\n", __func__);
    dev_dbg(&client->dev, "%s: apds990x try to suspend\n",
		__func__);

	if (!data->enable_ps_sensor && data->enable_als_sensor) {
			spin_lock_irqsave(&data->update_lock.wait_lock, flags);
			__cancel_delayed_work(&data->als_dwork);
			spin_unlock_irqrestore(&data->update_lock.wait_lock, flags);
			i2c_smbus_write_byte_data(client, CMD_BYTE | APDS990x_ENABLE_REG, 0);
	}


	printk("[%s] -\n", __func__);
	return 0;
}

static int apds990x_resume(struct i2c_client *client)
{
	struct apds990x_data *data = i2c_get_clientdata(client);
	unsigned long flags;
	printk("[%s] +\n", __func__);
	dev_dbg(&client->dev, "%s: apds990x try to resume\n", __func__);
    if (!data->enable_ps_sensor && data->enable_als_sensor) {
		i2c_smbus_write_byte_data(client,
		CMD_BYTE | APDS990x_ENABLE_REG, data->enable);

		spin_lock_irqsave(&data->update_lock.wait_lock, flags);

		__cancel_delayed_work(&data->als_dwork);
		schedule_delayed_work(&data->als_dwork,
			msecs_to_jiffies(data->als_poll_delay));

		spin_unlock_irqrestore(&data->update_lock.wait_lock, flags);
	}

	printk("[%s] -\n", __func__);

	return 0;
}

#else

#define apds990x_suspend	NULL
#define apds990x_resume		NULL

#endif /* CONFIG_PM */

static const struct i2c_device_id apds990x_id[] = {
	{ "apds990x", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, apds990x_id);

static struct i2c_driver apds990x_driver = {
	.driver = {
		.name	= APDS990x_DRV_NAME,
		.owner	= THIS_MODULE,
	},
	.suspend = apds990x_suspend,
	.resume	= apds990x_resume,
	.probe	= apds990x_probe,
	.remove	= __devexit_p(apds990x_remove),
	/* zkf55108 2011/10/27 add begin */
	.shutdown = apds990x_shutdown,
	/* zkf55108 2011/10/27 add end */
	.id_table = apds990x_id,
};

static int __init apds990x_init(void)
{
	return i2c_add_driver(&apds990x_driver);
}

static void __exit apds990x_exit(void)
{
	i2c_del_driver(&apds990x_driver);
}

MODULE_AUTHOR("Lee Kai Koon <kai-koon.lee@avagotech.com>");
MODULE_DESCRIPTION("APDS990x ambient light + proximity sensor driver");
MODULE_LICENSE("GPL");
MODULE_VERSION(DRIVER_VERSION);

module_init(apds990x_init);
module_exit(apds990x_exit);

