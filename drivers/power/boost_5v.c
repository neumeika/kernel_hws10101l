/* 
 * boost_5v.c
 *	
 */

#include <linux/module.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/gpio.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <mach/gpio.h>
#include <linux/mutex.h>
#include <linux/power/boost_5v.h>

#include <linux/syscore_ops.h>

#define DRIVER_NAME    "boost_5v"

static  struct  boost_5v_device *g_boost_5v_dev;

#define BOOST_5V_OUT_OPEN  1
#define BOOST_5V_OUT_CLOSE   0
static unsigned int g_boost_5v_bitmap = 0;
/*the function request_5v_boost can not be transfered in irq function*/
int request_5v_boost(int on, int bit_map)
{
    unsigned int pre_5v_bit_map;
    struct boost_5v_device *dev = g_boost_5v_dev;
    unsigned long flags;
    spin_lock_irqsave(&dev->power_lock,flags);
    pre_5v_bit_map = g_boost_5v_bitmap;
    if(!BOOST_5V_CHECK(bit_map))
    {
  	    printk(KERN_ERR " boost 5v request insert a erro module variable\n");
  	    spin_unlock_irqrestore(&dev->power_lock,flags);
        return -EINVAL;
    }
	
    //printk(KERN_INFO " boost 5v out:%d from module:%d ,pre state is %x\n",on,bit_map,pre_5v_bit_map);
    if(on)
    {
    	g_boost_5v_bitmap |= bit_map;
        if((0 == pre_5v_bit_map) && (g_boost_5v_bitmap != 0))
        {
        	if(!gpio_get_value(g_boost_5v_dev->gpio_num))
        	{
			    gpio_direction_output(dev->gpio_num,BOOST_5V_OUT_OPEN);
			    //printk(KERN_INFO "boost 5v out open\n");
		    }
	    }
    }
    else
    {
    	g_boost_5v_bitmap &= ~bit_map;
        if((0 == g_boost_5v_bitmap) && (g_boost_5v_bitmap != pre_5v_bit_map))
        {
        	if(gpio_get_value(g_boost_5v_dev->gpio_num))
        	{
			    gpio_direction_output(dev->gpio_num,BOOST_5V_OUT_CLOSE);
			    //printk(KERN_INFO " boost 5v out close\n");
		    }
    	}
    }
    spin_unlock_irqrestore(&dev->power_lock,flags);
    return 0;
}
EXPORT_SYMBOL(request_5v_boost);



static unsigned int  boost5vEnFlag = 0;
static ssize_t boost5v_attr_show(struct kobject *kobj, struct kobj_attribute *attr,
        char *buf)
 {
     return sprintf(buf, "%d", boost5vEnFlag);
 }

static ssize_t boost5v_attr_store(struct kobject *kobj, struct kobj_attribute *attr,
			 const char *buf, size_t count)
{
    int iret = 0;
    int rc = 0;
	
    printk(KERN_INFO " boost5v_attr_store\n");

    if(NULL != buf) 
    {   
        iret = sscanf(buf,"%d", & boost5vEnFlag);
    }

	if(-1 == iret) 
	{
		printk(KERN_ERR "sscanf boost5vEnFlag error\n");
		return iret;
	} 
	else
	{
		rc = request_5v_boost(boost5vEnFlag,BOOST_5V_TEST);
		return count;
	}
}

/* Define the device attributes */
 static struct kobj_attribute boost5v_attribute =
         __ATTR(switch, 0664, boost5v_attr_show, boost5v_attr_store);

 static struct attribute* boost5v_attributes[] = {
         & boost5v_attribute.attr,
         NULL
 };

 static struct attribute_group boost5v_defattr_group = {
         .attrs = boost5v_attributes,
 };

static int boost5v_suspend(void)
{
	request_5v_boost(0,BOOST_5V_SELF);

	return 0;
}
static void boost5v_resume(void)
{
	request_5v_boost(1,BOOST_5V_SELF);

	return;
}

static struct syscore_ops boost5v_syscore_ops = {
	.suspend	= boost5v_suspend,
	.resume		= boost5v_resume,
};


static int __init boost_5v_probe(struct platform_device *pdev)
{
	struct boost_5v_platdata *pdata = NULL;
	struct boost_5v_device   *dev = NULL;
	int ret = 0;

	pdata = pdev->dev.platform_data;

	printk(KERN_INFO " boost_5v_probe in\n");

	if(!pdata)
	{
		return -EINVAL;
	}
	if(pdata->io_mux_block_init)
	{
		ret = pdata->io_mux_block_init(pdata);
		if(ret)
		{
			printk(KERN_ERR " io_mux_block init fail 0x%x\n",ret);
			goto io_mux_block_init_fail;
		}
	}
	
	dev = kzalloc(sizeof(struct boost_5v_device), GFP_KERNEL);
	if (!dev) 
	{
		dev_err(&pdev->dev, "Error: No memory\n");
		ret =  -ENOMEM;
		goto err_alloc_data_failed;
	}


	dev->kobj_boost = kobject_create_and_add("boost5v_en", NULL);
    	if (dev->kobj_boost  == NULL) 
	{
		printk(KERN_ERR " create and add kobject fail 0x%x\n",ret);
        	ret = -ENOMEM;
		goto dev_kobj_err;
    	}
    	if (sysfs_create_group(dev->kobj_boost , &boost5v_defattr_group))
	{
		printk(KERN_ERR " sysfs create group fail 0x%x\n",ret);
        	ret = -ENOMEM;
		goto create_group_fail;
    	}

	//dev->use_gpio_moduel_num = 0;
	spin_lock_init(&dev->power_lock);
	/*set boost 5v gpio high to close VREG_5V*/
	if(pdata->init_gpio)
	{
		ret = pdata->init_gpio(pdata->switch_boost_gpio);
		if(ret<0)
		{
			printk(KERN_ERR " boost_5v request switch boost gpio fail 0x%x\n",ret);
			goto init_gpio_fail;				
		}
	}

	g_boost_5v_dev = dev;
	dev->gpio_num = pdata->switch_boost_gpio;
	if(!gpio_get_value(dev->gpio_num))
	{
		printk(KERN_ERR "%s: open 5v power early at init!\n", __func__);
		gpio_direction_output(g_boost_5v_dev->gpio_num,BOOST_5V_OUT_OPEN);
	}	
	register_syscore_ops(&boost5v_syscore_ops);
	return 0;
init_gpio_fail:
	sysfs_remove_group(dev->kobj_boost,&boost5v_defattr_group);
create_group_fail:
	kobject_put(dev->kobj_boost);	
dev_kobj_err:	
	kfree(dev);	
err_alloc_data_failed:
	pdata->io_mux_block_exit(pdata);
io_mux_block_init_fail:
	return ret;	
	
}

static int  boost_5v_remove(struct platform_device *pdev)
{
	struct boost_5v_platdata *pdata = NULL;
	pdata = pdev->dev.platform_data;
	if(pdata->exit_gpio)
	{
		pdata->exit_gpio(pdata->switch_boost_gpio);
	}
	sysfs_remove_group(g_boost_5v_dev->kobj_boost,&boost5v_defattr_group);
	kobject_put(g_boost_5v_dev->kobj_boost);	
	kfree(g_boost_5v_dev);
	if(pdata->io_mux_block_exit)
	{
		pdata->io_mux_block_exit(pdata);
	}
	g_boost_5v_dev = NULL;
	return 0;
}


static struct platform_driver  boost_5v_driver = {
	.probe		=boost_5v_probe,
	.remove		= boost_5v_remove,
	.driver = {
		.name = DRIVER_NAME,
		.owner = THIS_MODULE,
	},
};
static int __init  boost_5v_init(void)
{
	return platform_driver_register(&boost_5v_driver);
}

static void __exit boost_5v_exit(void)
{
	platform_driver_unregister(&boost_5v_driver);
}

/*boost 5v module is used by TP/AUDIO/FSA880,so the inital priority of it  is higher than others,we use fs_initcall*/
fs_initcall(boost_5v_init);
module_exit(boost_5v_exit);

MODULE_AUTHOR("Hisilicon K3");
MODULE_DESCRIPTION("Boost_5v Driver");
MODULE_LICENSE("GPL");
