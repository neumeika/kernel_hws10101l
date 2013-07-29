/* 
 * micro_usb_power_control.c
 *	
 */

#include <linux/module.h>
#include <linux/input.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/power/micro_usb_power_control.h>
#include <linux/interrupt.h>
#include <mach/product_feature_sel.h>
#include <linux/usb/hiusb_android.h>

#define DRIVER_NAME    "micro_usb_power_control"
static int usb_current_warning_irq_num = 0;

static struct micro_usb_power_control_device *usb_power_control_dev;

enum gpio_level{
GPIO_OUTPUT_LOW,
GPIO_OUTPUT_HIGH,
};

void open_usb_out5v(void)
{
	gpio_direction_output(usb_power_control_dev->out5v_en_gpio, GPIO_OUTPUT_HIGH);
}
 EXPORT_SYMBOL(open_usb_out5v);

void close_usb_out5v(void)
{
	gpio_direction_output(usb_power_control_dev->out5v_en_gpio, GPIO_OUTPUT_LOW);
}
 EXPORT_SYMBOL(close_usb_out5v);

 void open_usb_ap5v(void)
 {
 	usb_current_warning_irq_num = 0;
 	gpio_direction_output(usb_power_control_dev->ap5v_en_gpio, GPIO_OUTPUT_HIGH);
 }
 EXPORT_SYMBOL(open_usb_ap5v);

 void close_usb_ap5v(void)
 {
 	gpio_direction_output(usb_power_control_dev->ap5v_en_gpio, GPIO_OUTPUT_LOW);
 }
 EXPORT_SYMBOL(close_usb_ap5v);


static unsigned int  ap5v_en_flag = 0;
static ssize_t ap5v_attr_show(struct device *dev, struct device_attribute *attr, 
		char *buf)
 {
     return sprintf(buf, "%d", ap5v_en_flag);
 }

static ssize_t ap5v_attr_store(struct device *dev, 
		struct device_attribute *attr, const char *buf, size_t count)
{
    int iret = 0;
	
    printk(KERN_INFO " ap5v_attr_store\n");

    if(NULL != buf) 
    {   
        iret = sscanf(buf,"%d", & ap5v_en_flag);
    }

	if(-1 == iret) 
	{
		printk(KERN_ERR "sscanf ap5v_en_flag error\n");
		return iret;
	} 
	else
	{
	    if(0 == ap5v_en_flag)		
	    {
			close_usb_ap5v();
	    }
		else
		{
			open_usb_ap5v();
		}

		return count;
	}
}

static DEVICE_ATTR(ap5v_control, S_IRUSR | S_IWUSR , ap5v_attr_show, ap5v_attr_store);

static unsigned int  modem_usb_control_flag = 0;
static ssize_t modem_usb_control_attr_show(struct device *dev, 
		struct device_attribute *attr, char *buf)
 {
     return sprintf(buf, "%d", modem_usb_control_flag);
 }

static ssize_t modem_usb_control_attr_store(struct device *dev, 
		struct device_attribute *attr, const char *buf, size_t count)
{
    int iret = 0;
	
    printk(KERN_INFO " modem_usb_control_attr_store\n");

    if(NULL != buf) 
    {   
        iret = sscanf(buf,"%d", & modem_usb_control_flag);
    }

	if(-1 == iret) 
	{
		printk(KERN_ERR "sscanf modem_usb_control_flag error\n");
		return iret;
	} 
	else
	{
		if(0 == modem_usb_control_flag)		
	    {
	    	gpio_direction_output(usb_power_control_dev->modem_usb_ctrl_gpio, GPIO_OUTPUT_LOW);			
	    }
		else
		{
			gpio_direction_output(usb_power_control_dev->modem_usb_ctrl_gpio, GPIO_OUTPUT_HIGH);	
		}
		return count;
	}
}

static DEVICE_ATTR(modem_usb_control, S_IRUSR | S_IWUSR , modem_usb_control_attr_show, modem_usb_control_attr_store);

 static struct attribute* micro_usb_power_attributes[] = {
         & dev_attr_ap5v_control.attr,
		 & dev_attr_modem_usb_control.attr,
         NULL
 };

 static struct attribute_group micro_usb_power_defattr_group = {
         .attrs = micro_usb_power_attributes,
 };

 static void current_warning_work_func(struct work_struct *work)
{
	struct micro_usb_power_control_device *dev = container_of(work, struct micro_usb_power_control_device, current_warning_work);
	usb_current_warning_irq_num ++;
	if(usb_current_warning_irq_num > 2)
	{
		printk(KERN_INFO " micro usb:Detect current warning\n");
		usb_current_warning_irq_num = 2;
		kobject_uevent(&dev->dev.kobj,KOBJ_CHANGE);		
	}
	else
	{		
        	printk(KERN_INFO "micro usb:Accessory is plug out or there may be a shake\n");
	}
	enable_irq(dev->usb_current_warning_irq);	
}

irqreturn_t current_warning_irq_handler(int irq, void *dev_id)
{
	struct micro_usb_power_control_device *dev = (struct micro_usb_power_control_device *)dev_id;
	disable_irq_nosync(dev->usb_current_warning_irq);
	queue_work(dev->micro_usb_power_wq, &dev->current_warning_work);
	return IRQ_HANDLED;
}

 static void usb_id_detect_work_func(struct work_struct *work)
{
	struct micro_usb_power_control_device *dev = container_of(work, struct micro_usb_power_control_device, usb_id_detect_work);
	k3v2_otg_id_status_change(ID_FALL);
	enable_irq(dev->usb_id_detect_irq);	
}


irqreturn_t usb_id_detect_irq_handler(int irq, void *dev_id)
{
	struct micro_usb_power_control_device *dev = (struct micro_usb_power_control_device *)dev_id;
	disable_irq_nosync(dev->usb_id_detect_irq);
	printk(KERN_INFO "usb_id_detect_irq_handler in\n");
	queue_work(dev->micro_usb_power_wq, &dev->usb_id_detect_work);
	return IRQ_HANDLED;
}

static int __devinit micro_usb_power_control_probe(struct platform_device *pdev)
{
	struct micro_usb_power_control_platform_data *pdata = NULL;
	struct micro_usb_power_control_device *dev = NULL;

	int ret = 0;

	pdata = pdev->dev.platform_data;

	printk(KERN_INFO " micro_usb_power_control_probe in\n");

	if(!pdata)
	{
		printk(KERN_ERR "micro_usb_power_control pdata is NULL,retrun -EINVAL\n");
		return -EINVAL;
	}
	if(pdata->io_mux_block_init)
	{
		ret = pdata->io_mux_block_init(pdata);
		if(ret)
		{
			printk(KERN_ERR " micro_usb_power_control io_mux_block init fail 0x%x\n",ret);
			goto io_mux_block_init_fail;
		}
	}
	
	dev = kzalloc(sizeof(struct micro_usb_power_control_device), GFP_KERNEL);
	if (!dev) 
	{
		dev_err(&pdev->dev, "Error: No memory\n");
		ret =  -ENOMEM;
		goto err_alloc_data_failed;
	}
    dev->dev = pdev->dev;
    if (sysfs_create_group(&dev->dev.kobj, &micro_usb_power_defattr_group))
	{
		dev_err(&pdev->dev, "Error: Sysfs create group fail\n");
		ret = -ENOMEM;
		goto create_group_fail;
    }
	dev->micro_usb_power_wq = create_singlethread_workqueue(USB_CURRENT_WARNING_WQ_NAME);
	if (!dev->micro_usb_power_wq) 
	{
		dev_err(&pdev->dev, "Error: Create usb current warning workqueue fail\n");
		ret = -ENOMEM;
		goto err_creat_workqueue;
	}

	INIT_WORK(&dev->current_warning_work, current_warning_work_func);
	dev->current_warning_name = USB_CURRENT_WARNING_NAME;
	dev->usb_current_warning_irq = gpio_to_irq(pdata->usb_current_warning_gpio);


	ret = gpio_request(pdata->usb_current_warning_gpio, dev->current_warning_name);
	if (ret < 0) 
	{
		dev_err(&pdev->dev, "Error: Request current warning gpio fail\n");
		ret = -EIO;
		goto err_request_current_warning_irq;
	}

	gpio_direction_input(pdata->usb_current_warning_gpio);	

	ret = request_irq(dev->usb_current_warning_irq, current_warning_irq_handler,
			pdata->irq_type, dev->current_warning_name, dev);
	if (!ret) 
	{
		printk("Request current warning irq success\n");
	}
	else 
	{
		dev_err(&pdev->dev, "Error: Request current warning irq fail\n");
		goto request_current_warning_irq_fail;
	}

	if(get_product_feature(PROD_FEATURE_DETECT_MHL_CHIP))
	{

		INIT_WORK(&dev->usb_id_detect_work, usb_id_detect_work_func);
		dev->usb_id_detect_name = USB_ID_DETECT_NAME;
		dev->usb_id_detect_irq = gpio_to_irq(pdata->usb_id_detect_gpio);


		ret = gpio_request(pdata->usb_id_detect_gpio, dev->usb_id_detect_name);
		if (ret < 0) 
		{
			printk("Error: Request current warning gpio fail\n");
			dev_err(&pdev->dev, "Error: Request current warning gpio fail\n");
			ret = -EIO;
			goto err_request_usb_id_detect_irq;
		}

		gpio_direction_input(pdata->usb_id_detect_gpio);	

		ret = request_irq(dev->usb_id_detect_irq, usb_id_detect_irq_handler,
			pdata->usb_id_detect_irq_type, dev->usb_id_detect_name, dev);
		if (!ret) 
		{
			printk("Request usb id detect irq success\n");
		}
		else 
		{
			dev_err(&pdev->dev, "Error: Request usb id detect irq fail\n");
			goto request_usb_id_detect_irq_fail;
		}
	}

	/*set ap5v gpio high*/
	if(pdata->init_gpio)
	{
		ret = pdata->init_gpio();
		if(ret<0)
		{
			printk(KERN_ERR " micro_usb_power_control request gpio fail 0x%x\n",ret);
			goto init_gpio_fail;				
		}
	}
	dev->modem_usb_ctrl_gpio = pdata->modem_usb_control_gpio;
	dev->ap5v_en_gpio = pdata->usb_ap5v_en_gpio;	
	dev->out5v_en_gpio = pdata->usb_vbus5v_en_gpio;
	usb_power_control_dev = dev;
	return 0;

init_gpio_fail:
	if(get_product_feature(PROD_FEATURE_DETECT_MHL_CHIP))
	{
		free_irq(pdata->usb_id_detect_gpio, dev);
	}
request_usb_id_detect_irq_fail:
	if(get_product_feature(PROD_FEATURE_DETECT_MHL_CHIP))
	{
		gpio_free(pdata->usb_id_detect_gpio);
	}
err_request_usb_id_detect_irq:	
	free_irq(pdata->usb_current_warning_gpio, dev);
request_current_warning_irq_fail:
	gpio_free(pdata->usb_current_warning_gpio);
err_request_current_warning_irq:
	destroy_workqueue(dev->micro_usb_power_wq);
err_creat_workqueue:	
	sysfs_remove_group(&dev->dev.kobj,&micro_usb_power_defattr_group);
create_group_fail:	
	kfree(dev);	
err_alloc_data_failed:
	pdata->io_mux_block_exit(pdata);
io_mux_block_init_fail:
	return ret;	
	
}

static int micro_usb_power_control_remove(struct platform_device *pdev)
{
	struct micro_usb_power_control_platform_data *pdata = NULL;
	pdata = pdev->dev.platform_data;
	if(pdata->exit_gpio)
	{
		pdata->exit_gpio();
	}
	if(get_product_feature(PROD_FEATURE_DETECT_MHL_CHIP))
	{
		free_irq(pdata->usb_id_detect_gpio, usb_power_control_dev);
		gpio_free(pdata->usb_id_detect_gpio);
	}
	free_irq(pdata->usb_current_warning_gpio, usb_power_control_dev);
	gpio_free(pdata->usb_current_warning_gpio);
    if(usb_power_control_dev->micro_usb_power_wq)
    {
	    destroy_workqueue(usb_power_control_dev->micro_usb_power_wq);
    }
	sysfs_remove_group(&usb_power_control_dev->dev.kobj,&micro_usb_power_defattr_group);
	kfree(usb_power_control_dev);
	if(pdata->io_mux_block_exit)
	{
		pdata->io_mux_block_exit(pdata);
	}
	usb_power_control_dev = NULL;
	return 0;
}


static int micro_usb_power_control_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct micro_usb_power_control_platform_data *pdata = NULL;
	pdata = pdev->dev.platform_data;
	if(pdata->io_mux_block_exit)
	{
		pdata->io_mux_block_exit(pdata);
	}
	return 0;
}

static int micro_usb_power_control_resume(struct platform_device *pdev)
{
	struct micro_usb_power_control_platform_data *pdata = NULL;
	pdata = pdev->dev.platform_data;
	if(pdata->io_mux_block_init)
	{
		pdata->io_mux_block_init(pdata);
	}
	return 0;
}


static struct platform_driver  micro_usb_power_control_driver = {
	.probe = micro_usb_power_control_probe,
	.remove	= micro_usb_power_control_remove,
	.suspend = micro_usb_power_control_suspend,
	.resume = micro_usb_power_control_resume,
	.driver = {
		.name = DRIVER_NAME,
		.owner = THIS_MODULE,
	}
};
static int __init  micro_usb_power_control_init(void)
{
	return platform_driver_register(&micro_usb_power_control_driver);
}

static void __exit micro_usb_power_control_exit(void)
{
	platform_driver_unregister(&micro_usb_power_control_driver);
}

fs_initcall(micro_usb_power_control_init);
module_exit(micro_usb_power_control_exit);

MODULE_AUTHOR("S10 Inc");
MODULE_DESCRIPTION("Micro_usb_power_control Driver");
MODULE_LICENSE("GPL");
