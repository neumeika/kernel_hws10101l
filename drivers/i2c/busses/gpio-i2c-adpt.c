/* drivers/i2c/busses/gpio-i2c-adpt.c
 *
 * Copyright (C) 2007 Google, Inc.
 * Copyright (c) 2009, Code Aurora Forum. All rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/clk.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/timer.h>
#include <linux/i2c/gpio-i2c-adpt.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/mutex.h>

static void
gpio_i2c_start(struct gpio_i2c_dev *dev)
{
	int dely_usec = dev->pdata->dely_usec;
	unsigned int gpio_i2c_dat = dev->pdata->gpio_i2c_dat;
	unsigned int gpio_i2c_clk = dev->pdata->gpio_i2c_clk;

	gpio_direction_output(gpio_i2c_dat,1);
	gpio_direction_output(gpio_i2c_clk,1);
	udelay(dely_usec);

	gpio_set_value(gpio_i2c_dat,0);
	udelay(dely_usec);

	gpio_set_value(gpio_i2c_clk,0);
	udelay(dely_usec);
}

static void
gpio_i2c_stop(struct gpio_i2c_dev *dev)
{
	int dely_usec = dev->pdata->dely_usec;
	unsigned int gpio_i2c_dat = dev->pdata->gpio_i2c_dat;
	unsigned int gpio_i2c_clk = dev->pdata->gpio_i2c_clk;

	gpio_direction_output(gpio_i2c_dat,0);
	udelay(dely_usec);
	gpio_direction_output(gpio_i2c_clk,1);
	udelay(dely_usec);
	gpio_set_value(gpio_i2c_dat,1);
	udelay(dely_usec);
}

static unsigned int
gpio_i2c_byte_write(struct gpio_i2c_dev	*dev, unsigned char data)
{
	unsigned char i = 0;
	unsigned int retAck = 0;
	int dely_usec = dev->pdata->dely_usec;
	unsigned int gpio_i2c_dat = dev->pdata->gpio_i2c_dat;
	unsigned int gpio_i2c_clk = dev->pdata->gpio_i2c_clk;

	for(i=0; i<8; i++) {
		if((0x80 >> i) & data) {
			gpio_set_value(gpio_i2c_dat,1);
		} else {
			gpio_set_value(gpio_i2c_dat,0);
		}
		udelay(dely_usec);
		gpio_set_value(gpio_i2c_clk,1);
		udelay(dely_usec);
		gpio_set_value(gpio_i2c_clk,0);

        if (i == 7) {
            gpio_direction_input(gpio_i2c_dat);             /*set GPIO input, get ACK*/
        }

        udelay(dely_usec);
   }

	i=0;
	retAck=gpio_get_value(gpio_i2c_dat);
	while (retAck) {
		udelay(dely_usec/8);
		i++;
		if(i>40) {
			break;
        }
		retAck=gpio_get_value(gpio_i2c_dat);
	}

	gpio_set_value(gpio_i2c_clk,1);
    udelay(dely_usec);
	gpio_set_value(gpio_i2c_clk,0);
	udelay(dely_usec);

	gpio_direction_output(gpio_i2c_dat,1);			/*set GPIO output, release SDA*/
	udelay(dely_usec);
	retAck = ((retAck == 0) ? GPIO_I2C_SUCCESS:GPIO_I2C_FAIL);

	return retAck;
}

static unsigned char
gpio_i2c_byte_read(struct gpio_i2c_dev *dev, unsigned char ack)
{
	unsigned char i = 0;
	unsigned char retVal = 0x00;
	int dely_usec = dev->pdata->dely_usec;
	unsigned int gpio_i2c_dat = dev->pdata->gpio_i2c_dat;
	unsigned int gpio_i2c_clk = dev->pdata->gpio_i2c_clk;

	gpio_direction_input(gpio_i2c_dat);				/*set GPIO input, get 8-bit DATA*/
	udelay(dely_usec);

	for(i=0; i<8; i++)
	{
		gpio_set_value(gpio_i2c_clk,1);
		udelay(dely_usec);

		retVal <<= 1;
		if(gpio_get_value(gpio_i2c_dat))
		{
			retVal  |= 0x01;
		}

		gpio_set_value(gpio_i2c_clk,0);
		udelay(dely_usec);
	}

	if(ack)
	{
		gpio_direction_output(gpio_i2c_dat,0);		/*set GPIO output, send ACK*/
	}
	else
	{
		gpio_direction_output(gpio_i2c_dat,1);		/*set GPIO output, send NACK*/
	}
	udelay(dely_usec);

	gpio_set_value(gpio_i2c_clk,1);
	udelay(dely_usec);

	gpio_set_value(gpio_i2c_clk,0);
	udelay(dely_usec);

	gpio_set_value(gpio_i2c_dat,1);				/*release SDA*/
	udelay(dely_usec);

	return retVal;
}

static int
gpio_i2c_adpt_xfer(struct i2c_adapter *adap, struct i2c_msg msgs[], int num)
{
	unsigned char  addr;
	unsigned int i=0;
	int rem = num;
	unsigned int retVal=0;
	struct gpio_i2c_dev	*dev = i2c_get_adapdata(adap);

	addr = msgs->addr << 1;

	mutex_lock(&dev->mlock);
	while(rem) {
		if (msgs->flags & I2C_M_RD) { //read command

			addr |=1;
			//start
			gpio_i2c_start(dev);

			//write addr
			retVal = gpio_i2c_byte_write(dev,addr);
			if (!retVal) {				/*write read address fail*/
				dev_err(dev->dev,"[%s,%d]i2c write byte fail, addr = %x, err = %d\n"
							,__FUNCTION__,__LINE__,addr,retVal);
				goto out_err;
			}

			for (i=0;i<msgs->len;i++) {
				if(i == msgs->len-1) {				/*read last byte send NACK*/
					msgs->buf[i] = gpio_i2c_byte_read(dev,GPIO_I2C_NACK);
					dev_dbg(dev->dev,"[%s,%d] i2c read return NACK\n",__FUNCTION__,__LINE__);
				} else	{ 	 						/*else send ACK*/
					msgs->buf[i] = gpio_i2c_byte_read(dev,GPIO_I2C_ACK);
					dev_dbg(dev->dev,"[%s,%d] i2c read return ACK\n",__FUNCTION__,__LINE__);
				}
			}

					gpio_i2c_stop(dev);
		} else { //write comman

            //start
			gpio_i2c_start(dev);

			//write addr
			retVal = gpio_i2c_byte_write(dev,addr);
			if(!retVal)	{					/*write write address fail*/
				dev_err(dev->dev,"[%s,%d]i2c write byte fail, addr = %x, err = %d\n"
							,__FUNCTION__,__LINE__,addr,retVal);
				goto out_err;
			}

			for(i=0;i<msgs->len;i++) {
				retVal = gpio_i2c_byte_write(dev,msgs->buf[i]);
				if(!retVal) {
					dev_err(dev->dev,"[%s,%d]i2c write byte fail, data[%d] = %x, err = %d\n"
							,__FUNCTION__,__LINE__,i,msgs->buf[i],retVal);
					goto out_err;
				}
			}

			gpio_i2c_stop(dev);
		}

		msgs++;
		rem--;
	}
	mutex_unlock(&dev->mlock);
	return 0;

out_err:
	gpio_i2c_stop(dev);
	mutex_unlock(&dev->mlock);
	return -1;
}

static u32
gpio_i2c_adpt_func(struct i2c_adapter *adap)
{
	return I2C_FUNC_I2C;
}

static const struct i2c_algorithm gpio_i2c_adpt_algo = {
	.master_xfer	= gpio_i2c_adpt_xfer,
	.functionality	= gpio_i2c_adpt_func,
};

static int
gpio_i2c_adpt_probe(struct platform_device *pdev)
{
	int ret;
	struct gpio_i2c_dev	*dev;
	struct gpio_i2c_adpt_platform_data *pdata;


	printk(KERN_ERR "[%s %d]\n", __func__, __LINE__);

	pdata = pdev->dev.platform_data;
	if(!pdata) {
		dev_err(&pdev->dev, "[%s,%d]: platform data not initialized\n",__FUNCTION__,__LINE__);
		return -ENOSYS;
	}

	if(pdata->dely_usec <2 ||pdata->dely_usec >50)  {
		dev_err(&pdev->dev, "[%s,%d]: clock frequency not supported\n",__FUNCTION__,__LINE__);
		return  -EIO;
	}

	dev = kzalloc(sizeof(struct gpio_i2c_dev),GFP_KERNEL);
	if (!dev) {
		dev_err(&pdev->dev, "[%s,%d]: not enough memory\n",__FUNCTION__,__LINE__);
		return -ENOMEM;
	}

	dev->dev = &pdev->dev;
	dev->pdata = pdata;
	platform_set_drvdata(pdev,dev);

	i2c_set_adapdata(&dev->adap_gpio, dev);
	dev->adap_gpio.algo = &gpio_i2c_adpt_algo;
	strlcpy(dev->adap_gpio.name,
		"GPIO I2C adapter",
		sizeof(dev->adap_gpio.name));

	dev->adap_gpio.nr = pdev->id;
	ret = i2c_add_numbered_adapter(&dev->adap_gpio);
	if(ret) {
		dev_err(&pdev->dev, "[%s,%d]: Add GPIO I2C adapter failed, err = %d\n",__FUNCTION__,__LINE__,ret);
		kfree(dev);
		return -EIO;
	}

	mutex_init(&dev->mlock);

	gpio_request(pdata->gpio_i2c_clk, "gpio_i2c_clk");
	if (ret) {
		dev_err(&pdev->dev, "%s: unable to request gpio %d\n",
			__FUNCTION__,  pdata->gpio_i2c_clk);
		goto out;
	}

	gpio_request(pdata->gpio_i2c_dat, "gpio_i2c_dat");
	if (ret) {
		dev_err(&pdev->dev, "%s: unable to request gpio %d\n",
			__FUNCTION__, pdata->gpio_i2c_dat);
		goto reg_disable;
	}

	gpio_export(pdata->gpio_i2c_clk, true);
	gpio_export(pdata->gpio_i2c_dat, true);

	gpio_direction_input(pdata->gpio_i2c_clk);
	msleep(1);
	gpio_direction_input(pdata->gpio_i2c_dat);
	msleep(1);

    printk(KERN_ERR "[%s %d]\n", __func__, __LINE__);

	goto out;

reg_disable:
	gpio_free(pdata->gpio_i2c_clk);

out:
	printk(KERN_INFO "[%s %d] i2c_gpio return=%d \n",__func__, __LINE__ ,ret);
	return ret;
}

static int
gpio_i2c_adpt_remove(struct platform_device *pdev)
{
	struct gpio_i2c_dev	*dev = platform_get_drvdata(pdev);

	struct gpio_i2c_adpt_platform_data *pdata;

	pdata = pdev->dev.platform_data;
	if(pdata) {
		gpio_free(pdata->gpio_i2c_clk);
		gpio_free(pdata->gpio_i2c_dat);
	}

	mutex_destroy(&dev->mlock);
	i2c_del_adapter(&dev->adap_gpio);

	platform_set_drvdata(pdev, NULL);
	return 0;
}

static struct platform_driver gpio_i2c_adpt_driver = {
	.probe = gpio_i2c_adpt_probe,
	.remove = gpio_i2c_adpt_remove,
	.driver		= {
		.name	= GPIO_I2C_ADTP_NAME,
		.owner	= THIS_MODULE,
	},
};

static int __init
gpio_i2c_adpt_init_driver(void)
{
	return platform_driver_register(&gpio_i2c_adpt_driver);
}
subsys_initcall(gpio_i2c_adpt_init_driver);

static void __exit
gpio_i2c_adpt_exit_driver(void)
{
	platform_driver_unregister(&gpio_i2c_adpt_driver);
}
module_exit(gpio_i2c_adpt_exit_driver);
