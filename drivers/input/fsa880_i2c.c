/* 
 * kernel/drivers/input/fsa880_i2c.c
 *	
 */
/*=============================================================================


==============================================================================*/
#include <linux/module.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/gpio.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <mach/gpio.h>
#include <linux/input/fsa880_i2c.h>
#include <linux/usb/hiusb_android.h>

#ifdef CONFIG_BOOST_5V
#include <linux/power/boost_5v.h>
#endif
#include <linux/delay.h>
#ifdef CONFIG_PSY_MONITOR
#include <linux/notifier.h>
//#include <linux/power/k3_psy_monitor.h>//reomove s10 ics charging code (out of use)
#include <linux/power/S10_std_psy.h>
#endif
#include <mach/product_feature_sel.h>

#include <mach/gpio.h>

#include <linux/switch.h>
enum {
	NO_DEVICE	    = 0,
	DOCK_AUDIO_ANLG	= (1 << 0),
};
static struct switch_dev sdev;

/* fairchild fsa880's  Register: */
#define  DEVICE_ID       0x01
#define  CONTROL_REG    0x02
#define  INT_REG           0x03
#define  ADC_REG          0x07
#define  CHARGER_TYPE_REG     0x0A
#define  ACCESSORY_REG           0x0B
#define  MANUAL_SW1_REG       0x13
#define  MANUAL_SW2_REG       0x14
#define  RESET_REG                   0x1B



#ifdef CONFIG_PSY_MONITOR
extern struct blocking_notifier_head notifier_list_psy;
#endif

static struct  i2c_client *g_fsa880_client;

#define  CLR_INT_MASK           0x0   
#define  ACCESSORY_BIT         0x8f

#define  CHANGE_TO_MODEM         0x6C
#define  CHANGE_TO_AP  0x24
#define  DEFALT_SWITCH_USB_PORT  0x0
#define  RESET_VALUE  0x1

enum  Charger_type{
	NO_CHARGER = 0x0,
	CHARGER_SDP =  0x04,
	CHARGER_CAR_KIT1  = 0x10,
	CHARGER_CDP = 0x20,
	CHARGER_DCP = 0x40,
};

enum Accessory_type{
	RESISTOR_121K = 0x15,
	RESISTOR_150K = 0x16,
	RESISTOR_200K_POW = 0x17,
	RESISTOR_255K = 0x18,
	RESISTOR_301K = 0x19,
	RESISTOR_365K = 0x1A,
	RESISTOR_442K_POW = 0x1B,
	RESISTOR_523K = 0x1C,
	RESISTOR_619K = 0x1D,
	RESISTOR_1000K = 0x1E,
};
enum Detect_device{
	DEVICE_PLUG_OUT,
	SDP_CHARGER,
	CDP_CHARGER,
	DCP_CHARGER,
	CAR_KIT1_CHARGER,
	KEYPAD_BASE,
	DOCKING_VGA_LINE,
	USB_DEVICE_LINE,
	EQUIPMENT_TEST_BOARD,
	DOCKING_HDMI_LINE,
	TABLETOP_BASE,
};

enum Detect_device_class{
	PLUG_OUT = 0x04,
	HID_AS_HOST = 0x0b,
	HID_AS_DEVICE = 0x0a,
	VGA_HDMI_MHL = 0x08,
};

enum Boost_5v_out_level{
BOOST_5V_LOW,
BOOST_5V_HIGH,
};
static int boost_5v_is_out = BOOST_5V_LOW;

static int usb_out_current_warning_irq_num = 0;

enum Gpio_level{
GPIO_LEVEL_LOW,
GPIO_LEVEL_HIGH,
};
static int balong_gpio = GPIO_LEVEL_LOW;

static int g_accessory_status = DEVICE_PLUG_OUT;


static void fsa880_request_boost_5v(int on)
{
	if(on)
	{
		if(boost_5v_is_out == BOOST_5V_HIGH)
		{
			return;
		}
		else
		{
#ifdef CONFIG_BOOST_5V
			request_5v_boost(on,BOOST_5V_USB);
#endif
			boost_5v_is_out = BOOST_5V_HIGH;
		}		
	}
	else
	{
		if(boost_5v_is_out == BOOST_5V_LOW)
		{
			return;
		}
		else
		{
#ifdef CONFIG_BOOST_5V
			request_5v_boost(on,BOOST_5V_USB);
#endif
			boost_5v_is_out = BOOST_5V_LOW;
		}		
	}	
}

static void set_gpio_for_vbus_and_id(int device_class)
{
	int dev_class;
	int out_5v_gpio;
	struct fairchild_fsa880_platform_data *pdata = NULL;
	dev_class = device_class;
	pdata = g_fsa880_client->dev.platform_data;
	if(get_product_feature(PROD_FEATURE_USB_OUT_5V_SUPPLY))
	{
		out_5v_gpio = !((dev_class&0x04)>>2);
	}
	else
	{
		out_5v_gpio = (dev_class&0x04)>>2;
	}

	fsa880_request_boost_5v((dev_class&0x08)>>3);
	gpio_direction_output(pdata->switch_out5v_gpio,out_5v_gpio);
	gpio_direction_output(pdata->switch_ap5v_gpio,(dev_class&0x02)>>1);
	gpio_direction_output(pdata->switch_usb_id_gpio,dev_class&0x01);
	return;
}

static void  process_power_and_usb_id(int detect_status)
{
	switch(detect_status)
	{
		case DEVICE_PLUG_OUT:
			set_gpio_for_vbus_and_id(PLUG_OUT);
			if((g_accessory_status == SDP_CHARGER)||(g_accessory_status == CDP_CHARGER))
			{
				k3v2_otg_id_status_change(USB_DISCONNECT);
			}
			break;
		
		case SDP_CHARGER:
		case CDP_CHARGER:
			set_gpio_for_vbus_and_id(HID_AS_DEVICE);
			k3v2_otg_id_status_change(USB_CONNECT);
			break;

		case EQUIPMENT_TEST_BOARD:
		case KEYPAD_BASE:
		case USB_DEVICE_LINE:
		case TABLETOP_BASE:
			set_gpio_for_vbus_and_id(HID_AS_HOST);
				k3v2_otg_id_status_change(ID_FALL);
			break;

		case DOCKING_VGA_LINE:
		case DOCKING_HDMI_LINE:
			set_gpio_for_vbus_and_id(VGA_HDMI_MHL);
			break;

		default:
			break;		
		return;
	}
}


static void detect_charger_or_accessory_in(struct i2c_client *client)
{
	struct fairchild_fsa880_platform_data *platdata = NULL;
       int ret;
#ifdef CONFIG_PSY_MONITOR
       long int event = 0;
#endif
	platdata = client->dev.platform_data;


	ret = i2c_smbus_read_byte_data(client,ACCESSORY_REG);
	printk(KERN_INFO " fsa880 ACCESSORY_REG:0x%x\n",ret);
	if(ret&ACCESSORY_BIT)
	{
		dev_info(&client->dev, "%s: Accessory is in\n", __func__);
			
		ret = i2c_smbus_read_byte_data(client,ADC_REG);
		printk(KERN_INFO " fsa880 ADC_REG:%x\n",ret);
		switch(ret)
		{
			/*keypad base*/
			case  RESISTOR_121K:
				process_power_and_usb_id(KEYPAD_BASE);
				g_accessory_status = KEYPAD_BASE;
//				/*send message to tell PM*/
#ifdef CONFIG_PSY_MONITOR
				event = CHG_DOCK_PULGIN_EVENT;//CHG_BOTTOM_EXIST_EVENT;
				blocking_notifier_call_chain(&notifier_list_psy, event, NULL);
#endif
				break;

			/* 30 pin docking  to VGA*/
			case RESISTOR_150K:
				process_power_and_usb_id(DOCKING_VGA_LINE);
				g_accessory_status = DOCKING_VGA_LINE;
#ifdef CONFIG_PSY_MONITOR
				event = CHG_DOCK_PULGIN_EVENT;//CHG_BOTTOM_EXIST_EVENT;
				blocking_notifier_call_chain(&notifier_list_psy, event, NULL);
#endif
				break;
			
			/* 30 pin docking  to usb A port*/
			case RESISTOR_301K:
				process_power_and_usb_id(USB_DEVICE_LINE);
				g_accessory_status = USB_DEVICE_LINE;			
				break;
			case RESISTOR_523K:
				 process_power_and_usb_id(EQUIPMENT_TEST_BOARD);
				 g_accessory_status = EQUIPMENT_TEST_BOARD;
                 switch_set_state(&sdev, DOCK_AUDIO_ANLG);
#ifdef CONFIG_PSY_MONITOR
				event = CHG_DOCK_PULGIN_EVENT;//CHG_BOTTOM_EXIST_EVENT;
				blocking_notifier_call_chain(&notifier_list_psy, event, NULL);
#endif
				 break;
			
			/* 30 pin docking to HDMI*/
			case RESISTOR_619K:
				process_power_and_usb_id(DOCKING_HDMI_LINE);
			    g_accessory_status = DOCKING_HDMI_LINE;
#ifdef CONFIG_PSY_MONITOR
				event = CHG_DOCK_PULGIN_EVENT;//CHG_BOTTOM_EXIST_EVENT;
				blocking_notifier_call_chain(&notifier_list_psy, event, NULL);
#endif
				break;
				
			/* tabletop base*/				
			case RESISTOR_1000K:
				process_power_and_usb_id(TABLETOP_BASE);
				g_accessory_status = TABLETOP_BASE;
                switch_set_state(&sdev, DOCK_AUDIO_ANLG);
				break;
				
			default:
				break;
			}
	}
	else
	{
		ret = i2c_smbus_read_byte_data(client,CHARGER_TYPE_REG);
		printk(KERN_INFO " fsa880 CHARGER_TYPE_REG:0x%x\n",ret);
		switch(ret){
       		case CHARGER_SDP:
				process_power_and_usb_id(SDP_CHARGER);
				g_accessory_status = SDP_CHARGER;
#ifdef CONFIG_PSY_MONITOR
                		event =CHG_USB_SDP_PULGIN_EVENT;//CHG_USB_SDP_INSERT_EVENT;
#endif

				ret = i2c_smbus_read_byte_data(client,MANUAL_SW1_REG);	
				if(ret == CHANGE_TO_MODEM)
				{
					gpio_direction_output(BALONG_USE_GPIO, 1);	
					balong_gpio = GPIO_LEVEL_HIGH;	
				}
				break;

			case  CHARGER_CDP:
				process_power_and_usb_id(CDP_CHARGER);
				g_accessory_status = CDP_CHARGER;
#ifdef CONFIG_PSY_MONITOR
                		event = CHG_USB_CDP_PULGIN_EVENT;//CHG_USB_CDP_INSERT_EVENT;
#endif	
				ret = i2c_smbus_read_byte_data(client,MANUAL_SW1_REG);	
				if(ret == CHANGE_TO_MODEM)
				{
					gpio_direction_output(BALONG_USE_GPIO, 1);	
					balong_gpio = GPIO_LEVEL_HIGH;
				}		
				break;
		
			case  CHARGER_DCP:				
				process_power_and_usb_id(DCP_CHARGER);	
				g_accessory_status = DCP_CHARGER;
#ifdef CONFIG_PSY_MONITOR
                 		event = CHG_USB_DCP_PULGIN_EVENT;//CHG_USB_DCP_INSERT_EVENT;
#endif				
				break;
		
		default:
			break;	
		}
#ifdef CONFIG_PSY_MONITOR
       	blocking_notifier_call_chain(&notifier_list_psy, event, NULL);
#endif
	}
	return;
}

/*
 **************************************************************************
 * FunctionName: fsa880_switch_ap_or_modem;
 * Description : if we input 0,fsa880 will switch to modem,else switch to ap
 * Input       : int value;
 * Output      : NA;
 * ReturnValue : int ret;
 * Other       : NA;
 **************************************************************************
 */
extern void usb_gadget_control(int on);
int fsa880_switch_ap_or_modem(int value)
{
	int ret=0;
	int register_value = 0;

	if(value)
	{
		register_value = CHANGE_TO_AP;
	}
	else
	{
		register_value = CHANGE_TO_MODEM;
	}

	printk(KERN_INFO " fsa880_switch_ap_or_modem 0x%x\n",value);
	ret = i2c_smbus_write_byte_data(g_fsa880_client, MANUAL_SW1_REG, 0);	
	if(ret<0)
	{
		dev_err(&g_fsa880_client->dev, "%s: failed to write SW1 register to zero!\n", __func__);
	}
	else
	{

		if(register_value == CHANGE_TO_MODEM) {
			usb_gadget_control(0);  //disable usb gadget
			mdelay(100);
			if((g_accessory_status == SDP_CHARGER)||(g_accessory_status == CDP_CHARGER))
			{
				gpio_direction_output(BALONG_USE_GPIO, 1);	
				set_gpio_for_vbus_and_id(PLUG_OUT);	
				k3v2_otg_id_status_change(USB_DISCONNECT);
			}
		}

		ret = i2c_smbus_write_byte_data(g_fsa880_client, MANUAL_SW1_REG, register_value);
		if(ret<0)
		{
			dev_err(&g_fsa880_client->dev, "%s: failed to write SW1 register!\n", __func__);
		}
		
		if(register_value == CHANGE_TO_AP) {
			if((g_accessory_status == SDP_CHARGER)||(g_accessory_status == CDP_CHARGER))
			{
				set_gpio_for_vbus_and_id(HID_AS_DEVICE);
				k3v2_otg_id_status_change(USB_CONNECT);
			}
			gpio_direction_output(BALONG_USE_GPIO, 0);
			mdelay(100);
			usb_gadget_control(1);  //enable usb gadget
		}
	}
	return ret;
}

EXPORT_SYMBOL(fsa880_switch_ap_or_modem);

static void fairchild_fsa880_work_func(struct work_struct *work)
{

	int ret = 0;
#ifdef CONFIG_PSY_MONITOR
       long int event = 0;
#endif
	struct fairchild_fsa880 *dev = container_of(work, struct fairchild_fsa880, work);

	/*read the irq Interrupt Status*/
	ret = i2c_smbus_read_byte_data(dev->client, INT_REG);
	printk(KERN_INFO "fsa880 interrupt 0x%x\n",ret);

	if(ret==0x01)   //charger or accessory  attach
	{		
		detect_charger_or_accessory_in(dev->client);
		kobject_uevent(&dev->client->dev.kobj, KOBJ_CHANGE);
	}
	else if(ret==0x02)    //detach
	{
		gpio_direction_output(BALONG_USE_GPIO, 0);	
		balong_gpio = GPIO_LEVEL_LOW;	

		usb_out_current_warning_irq_num = 0;
#ifdef CONFIG_PSY_MONITOR
	    	event = CHG_CHARGER_PULGOUT_EVENT;//CHG_REMOVED_EVENT;
	    	blocking_notifier_call_chain(&notifier_list_psy, event, NULL);
#endif
		process_power_and_usb_id(DEVICE_PLUG_OUT);
        if (g_accessory_status == EQUIPMENT_TEST_BOARD || g_accessory_status == TABLETOP_BASE)
            switch_set_state(&sdev, NO_DEVICE);
		g_accessory_status = DEVICE_PLUG_OUT;
		kobject_uevent(&dev->client->dev.kobj, KOBJ_CHANGE);
	}
	else
	{
		dev_err(&dev->client->dev, "%s: unknown interrupt!!!\n", __func__);
	}
		enable_irq(dev->client->irq);
	return;
}

static void current_warning_work_func(struct work_struct *work)
{
	struct fairchild_fsa880 *dev = container_of(work, struct fairchild_fsa880, current_warning_work);
	usb_out_current_warning_irq_num ++;
	if(usb_out_current_warning_irq_num > 2)
	{
		printk(KERN_INFO " fsa880:Detect current warning\n");
		usb_out_current_warning_irq_num = 2;
		kobject_uevent(&dev->client->dev.kobj,KOBJ_OFFLINE);		
	}
	else
	{		
        	printk(KERN_INFO " fsa880:Accessory is plug out or there may be a shake\n");
	}
	enable_irq(dev->use_irq);	
}

irqreturn_t current_warning_irq_handler(int irq, void *dev_id)
{
	struct fairchild_fsa880 *dev = (struct fairchild_fsa880 *)dev_id;
	disable_irq_nosync(dev->use_irq);
	queue_work(dev->fsa880_wq, &dev->current_warning_work);
	return IRQ_HANDLED;
}

irqreturn_t fairchild_fsa880_irq_handler(int irq, void *dev_id)
{
	struct fairchild_fsa880 *dev = (struct fairchild_fsa880 *)dev_id;
	disable_irq_nosync(dev->client->irq);
	queue_work(dev->fsa880_wq, &dev->work);
	return IRQ_HANDLED;
}


static unsigned int  switch_to_ap = 1;
static ssize_t switch_ap_modem_show(struct device_driver *driver,char *buf)
{
    if(NULL == buf)
    {
        return -1;
    }

	return sprintf(buf, "%d", switch_to_ap);

}

static ssize_t switch_ap_modem_store(struct device_driver *driver,const char *buf,size_t count)
{
	int iret = 0;
	
	 if(NULL != buf) 
        {   
              iret = sscanf(buf,"%d", &switch_to_ap);
        }
	 if(-1 == iret) 
	 {
        	printk(KERN_ERR "sscanf switch_to_ap error\n");
        	return iret;
        }
	 else
	 {
		if(switch_to_ap)
		{
			fsa880_switch_ap_or_modem(1);	
		}
		else
		{
			fsa880_switch_ap_or_modem(0);	
		}
		return count;
	 }
}
static DRIVER_ATTR(switch_state, S_IRUSR | S_IWUSR, switch_ap_modem_show, switch_ap_modem_store);


/*test code start*/
static unsigned int  addr_offset = 0;

static ssize_t address_attr_show(struct device *dev, struct device_attribute *attr, char *buf)
 {
 	return sprintf(buf, "%d", addr_offset);
 }

static ssize_t address_attr_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int iret = 0;

	printk(KERN_INFO "  address_attr_store\n");

	if(NULL != buf) 
	{   
		iret = sscanf(buf,"%d", &addr_offset);
	}

	if(-1 == iret) 
	{
		printk(KERN_ERR "sscanf address_attr_store error\n");
		return iret;
	} 
	else 
	{
		return count;
	}
}

static DEVICE_ATTR(addr, S_IRUSR | S_IWUSR, address_attr_show, address_attr_store);


static unsigned int  register_value = 0;
static ssize_t reg_attr_show(struct device *dev, struct device_attribute *attr, char *buf)
 {
	if(addr_offset != 0)
	{
		register_value = i2c_smbus_read_byte_data(g_fsa880_client,addr_offset);	
	}
	return sprintf(buf, "%d", register_value);
 }

static ssize_t reg_attr_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int iret = 0;
	int rc = 0;
	
	printk(KERN_INFO " reg_attr_store\n");

	if(NULL != buf) 
	{   
		iret = sscanf(buf,"%d", &register_value);
	}

	if(-1 == iret) 
	{
		printk(KERN_ERR "sscanf reg_attr_store error\n");
		return iret;
	} 
	else 
	{
		rc = i2c_smbus_write_byte_data(g_fsa880_client, addr_offset, register_value);	 
		if(rc<0)
		{
			printk(KERN_ERR "sscanf reg_attr_store error\n");   
		}
	return count;
	}
}


static DEVICE_ATTR(reg, S_IRUSR | S_IWUSR, reg_attr_show, reg_attr_store);


static ssize_t accessory_status_attr_show(struct device *dev, struct device_attribute *attr, char *buf)
 {
	return sprintf(buf, "%d", g_accessory_status);
 }


static DEVICE_ATTR(accessory_status, S_IRUSR , accessory_status_attr_show, NULL);


/* Define the device attributes */
 static struct attribute* addr_reg_attributes[] = {
         &dev_attr_addr.attr,
	  &dev_attr_reg.attr,
	  &dev_attr_accessory_status.attr,
         NULL
 };

 static struct attribute_group addr_reg_defattr_group = {
         .attrs = addr_reg_attributes,
 };
/*test code end*/

static ssize_t dock_print_name(struct switch_dev *sdev, char *buf)
{
	return sprintf(buf, "usb_audio\n");
}

static int dock_configure_hsed(void)
{
    int rc;

	sdev.name = "usb_audio";
    sdev.print_name = dock_print_name;
    rc = switch_dev_register(&sdev);
    if (rc) {
        pr_err("%s(%u):error registering switch device %d\n", __FUNCTION__, __LINE__, rc);
    }

    return rc;
}

static int fairchild_fsa880_probe(struct i2c_client *client, const struct i2c_device_id *id)
{  
	struct fairchild_fsa880_platform_data *platdata = NULL;
	struct fairchild_fsa880 *dev = NULL;
	int ret = 0;

	g_fsa880_client = client;

      printk(KERN_INFO " fairchild_fsa880_probe\n");

    dock_configure_hsed();

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) 
	{
		dev_err(&client->dev, "%s: failed to check i2c function!\n", __func__);
		ret = -EIO;
		goto err_check_i2c_func;
	}

	platdata = client->dev.platform_data;
	if (!platdata) 
	{
		dev_err(&client->dev, "%s: the platform data is NULL!\n", __func__);
		ret = -ENOMEM;
		goto err_get_platform_data;
	}

	if(platdata->io_mux_block_init)
	{
		ret = platdata->io_mux_block_init(platdata);
		if(ret)
			goto err_init_io_block_failed;
	}

	dev = kzalloc(sizeof(struct fairchild_fsa880), GFP_KERNEL);
	if (!dev) 
	{
		dev_err(&client->dev, "%s: failed to kzalloc fairchild_fsa880!\n", __func__);
		ret = -ENOMEM;
		goto err_alloc_data_failed;
	}

	dev->fsa880_wq = create_singlethread_workqueue(FSA880_WQ_NAME);
	if (!dev->fsa880_wq) 
	{
		dev_err(&client->dev, "%s: failed to create workqueue\n", __func__);
		ret = -ENOMEM;
		goto err_creat_workqueue;
	}

	INIT_WORK(&dev->work, fairchild_fsa880_work_func);
	
	dev->client = client;
	dev->client->dev.init_name = FSA880_INIT_NAME;
	dev->client->irq = gpio_to_irq(platdata->intr_gpio);
	i2c_set_clientdata(client, dev);
	INIT_WORK(&dev->current_warning_work, current_warning_work_func);
	dev->current_warning_name = CURRENT_WARNING_NAME;
	dev->use_irq = gpio_to_irq(platdata->usb_current_warning_gpio);

	/* Register sysfs hooks */
	ret = sysfs_create_group(&client->dev.kobj, &addr_reg_defattr_group);
	if(ret)
	{
		goto exit_free;
	}


	if(platdata->init_gpio)
	{
		ret = platdata->init_gpio();
		if (ret < 0) 
		{
			dev_err(&client->dev, "%s: failed to init  gpio\n", __func__);
			goto init_gpio_fail;
		}
	}

	/*software reset for fsa880*/
	ret = i2c_smbus_write_byte_data(client, RESET_REG, RESET_VALUE);
	if(ret<0)
	{
		dev_err(&client->dev, "%s: failed to write reset register!!\n", __func__);
	}
	
	/*default switch the DP/DM  to AP*/ 
	ret = i2c_smbus_read_byte_data(client,MANUAL_SW1_REG);	
       printk(KERN_INFO " fsa880 MANUAL_SW1_REG 0x%x\n",ret);
	if(ret==DEFALT_SWITCH_USB_PORT)
	{
		ret = i2c_smbus_write_byte_data(client, MANUAL_SW1_REG, CHANGE_TO_AP);
		if(ret<0)
		{
			dev_err(&client->dev, "%s: failed to write SW1 register,switch to ap fail\n", __func__);
		}
	}
	else
	{
		ret = fsa880_switch_ap_or_modem(1);   
		if(ret<0)
		{
			dev_err(&client->dev, "%s: failed to write SW1 register,switch to ap failed\n", __func__);
		}
	}


	/* request gpio pin for irq */
	ret = gpio_request(platdata->intr_gpio, client->name);
	if (ret < 0) 
	{
		dev_err(&client->dev, "%s: failed to request gpio for irq\n", __func__);
		ret = -EIO;
		goto err_request_irq_pin;
	}

	gpio_direction_input(platdata->intr_gpio);

	/* set irq handler */
	ret = request_irq(dev->client->irq, fairchild_fsa880_irq_handler,
			platdata->irq_type, client->name, dev);
	if (!ret) 
	{
		dev_info(&client->dev, "%s: BC1.1 device %s work in interrupt mode\n", 
				__func__, client->name);
		enable_irq_wake(dev->client->irq);
	}
	else 
	{
		dev_info(&client->dev, "%s: BC1.1 device request irq fail\n", __func__);
		goto request_irq_fail;
	}

	ret = gpio_request(platdata->usb_current_warning_gpio, dev->current_warning_name);
	if (ret < 0) 
	{
		dev_err(&client->dev, "%s: failed to request gpio for current warning irq\n", __func__);
		ret = -EIO;
		goto err_request_current_warning_irq;
	}

	gpio_direction_input(platdata->usb_current_warning_gpio);	

	ret = request_irq(dev->use_irq, current_warning_irq_handler,
			platdata->current_warning_irq_type, dev->current_warning_name, dev);
	if (!ret) 
	{
		dev_info(&client->dev, "%s: usb out 5v current %s work in interrupt mode\n", 
				__func__, dev->current_warning_name);
	}
	else 
	{
		dev_info(&client->dev, "%s:  usb out 5v current :%s  request irq fail\n", __func__,dev->current_warning_name);
		goto request_current_warning_irq_fail;
	}

	/* set Manual switch;Clear  interrupt  mask bit ,so interrupt is enable*/
	ret = i2c_smbus_write_byte_data(client, CONTROL_REG, CLR_INT_MASK);
	if(ret<0)
	{
		ret = i2c_smbus_write_byte_data(client, CONTROL_REG, CLR_INT_MASK);
		if(ret<0)
		{
			dev_err(&client->dev, "%s: failed to write control register!\n", __func__);
			goto write_i2c_fail;
		}
	}




	return 0;

write_i2c_fail:
	free_irq(platdata->usb_current_warning_gpio, dev);
request_current_warning_irq_fail:
	gpio_free(platdata->usb_current_warning_gpio);
err_request_current_warning_irq:
	free_irq(platdata->intr_gpio, dev);
request_irq_fail:
	gpio_free(platdata->intr_gpio);
err_request_irq_pin:
	platdata->exit_gpio();
init_gpio_fail:	
	sysfs_remove_group(&client->dev.kobj,&addr_reg_defattr_group);
exit_free:
	destroy_workqueue(dev->fsa880_wq);
err_creat_workqueue:
	kfree(dev);
err_alloc_data_failed:
	platdata->io_mux_block_exit(platdata);
err_init_io_block_failed:
err_get_platform_data:
err_check_i2c_func:
	return ret;
}


static int fairchild_fsa880_remove(struct i2c_client *client)
{
	struct fairchild_fsa880 *dev = i2c_get_clientdata(client);
	struct fairchild_fsa880_platform_data *platdata = client->dev.platform_data;

	printk(KERN_INFO " fairchild_fsa880_remove\n");

    switch_dev_unregister(&sdev);

	free_irq(platdata->usb_current_warning_gpio, dev);
	gpio_free(platdata->usb_current_warning_gpio);

	free_irq(platdata->intr_gpio, dev);

	gpio_free(platdata->intr_gpio);

	if(platdata->exit_gpio)
	{
		platdata->exit_gpio();
	}

	sysfs_remove_group(&client->dev.kobj,&addr_reg_defattr_group);
	if (dev->fsa880_wq)
	{
		destroy_workqueue(dev->fsa880_wq);
	}
	
	kfree(dev);
	if(platdata->io_mux_block_exit)
	{
		platdata->io_mux_block_exit(platdata);
	}

	return 0;
}


static int fairchild_fsa880_suspend(struct i2c_client *client,pm_message_t mesg)
{
	struct fairchild_fsa880_platform_data *platdata = client->dev.platform_data;
	struct fairchild_fsa880 *dev = i2c_get_clientdata(client);
       disable_irq(dev->use_irq);
	cancel_work_sync(&dev->current_warning_work);
	cancel_work_sync(&dev->work);
	usb_out_current_warning_irq_num = 0;

	balong_gpio = gpio_get_value(platdata->balong_use_gpio);
	gpio_direction_input(platdata->balong_use_gpio);
	gpio_direction_input(platdata->switch_ap5v_gpio);
	gpio_direction_input(platdata->switch_usb_id_gpio);
	if(platdata->io_mux_block_exit)
	{
		platdata->io_mux_block_exit(platdata);
	}		
	return 0;
}

static int fairchild_fsa880_resume(struct i2c_client *client)
{
	struct fairchild_fsa880_platform_data *platdata = client->dev.platform_data;
	int ret = 0;
	struct fairchild_fsa880 *dev = i2c_get_clientdata(client);
       enable_irq(dev->use_irq);
	if(platdata->io_mux_block_init)
	{
		ret = platdata->io_mux_block_init(platdata);
		if(ret)
		{
			printk(KERN_ERR " fsa880 resume io_mux_block init fail :0x%x\n",ret);	
		}
			
	}

	gpio_direction_output(platdata->balong_use_gpio, balong_gpio);
	if(!((g_accessory_status == SDP_CHARGER)||(g_accessory_status == CDP_CHARGER)))
	{
		process_power_and_usb_id(g_accessory_status);
	}
	
	return ret;
}


static const struct i2c_device_id fairchild_fsa880_id[] = {
	{ FAIRCHILD_FSA880_NAME, 0 },
	{ }
};

static struct i2c_driver fairchild_fsa880_driver = {
	.probe		= fairchild_fsa880_probe,
	.remove		= fairchild_fsa880_remove,
	.suspend        = fairchild_fsa880_suspend,
	.resume        = fairchild_fsa880_resume,
	.id_table	= fairchild_fsa880_id,
	.driver = {
		.name	= FAIRCHILD_FSA880_NAME,
	},
};
static int __devinit fairchild_fsa880_init(void)
{
	int retval = 0;
	int error;
	retval = i2c_add_driver(&fairchild_fsa880_driver);
	if (retval < 0) 
	{
		printk(KERN_ERR "%s retval=%d\n", __func__, retval);
		return retval;
	}
	error = driver_create_file(&fairchild_fsa880_driver.driver, &driver_attr_switch_state);
	return retval;
}

static void __exit fairchild_fsa880_exit(void)
{
	i2c_del_driver(&fairchild_fsa880_driver);
}

module_init(fairchild_fsa880_init);
module_exit(fairchild_fsa880_exit);
MODULE_AUTHOR("S10 Inc");
MODULE_DESCRIPTION("Fairchild FSA880 Driver");
MODULE_LICENSE("GPL");
