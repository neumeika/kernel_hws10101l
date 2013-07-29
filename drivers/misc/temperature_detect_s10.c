#include <linux/module.h> 
#include <linux/kernel.h>
#include <linux/hkadc/hiadc_hal.h>
#include <mach/gpio.h>
#include <mach/k3_version.h>

static struct kobject *kobj = NULL;
/* save the temperature value by adc channel */
static int hw_temperature = 0;
static int temp_debug_mask = 1;
/* we can control the print switch by /sys/module/temperature_detect_s10/parameters/temperature_dbg_switch */
module_param_named(temperature_dbg_switch, temp_debug_mask, int, S_IRUGO | S_IWUSR | S_IWGRP);

#define TEMPERATURE_DBG(x...) do {\
if (temp_debug_mask) \
    printk(KERN_DEBUG x);\
} while (0);

/* get the temperature voltage value by adc chennel number */
static int  hw_temperature_get(int channel_no)
{
    unsigned char reserve = 0;
    int value = 0;    
    int ret = 0;

    /* open temperature channel */
    ret = k3_adc_open_channel(channel_no);
    if (ret < 0)
    {
        printk(KERN_ERR "%s:open adc channel failed(ret=%d channel_no=%d)\n", __func__, ret, channel_no);
    }
    /* get the temperature voltage value */
    value = k3_adc_get_value(channel_no, &reserve);
    /* release the resource */
    ret = k3_adc_close_channal(channel_no);
    if (ret < 0)
    {
        printk(KERN_ERR "%s:close adc channel failed(ret=%d channel_no=%d)\n", __func__, ret, channel_no);
    }
    return value;
}
static void hw_temp_detect_start(void)
{
    int temp_mv_value = 0;

    temp_mv_value = hw_temperature_get(ADC_RTMP);
    hw_temperature = temp_mv_value;
    TEMPERATURE_DBG("%s:temperature vol is %d\n", __func__, temp_mv_value);
}

static ssize_t hw_temperature_attr_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
    /* If the user accesses the node of '/sys/temperature/hw_temperature', we will get the value again */
    hw_temp_detect_start();
    return sprintf(buf, "%d\n", hw_temperature);
}
static struct kobj_attribute hw_temp_attribute =
     __ATTR(hw_temperature, 0444, hw_temperature_attr_show, NULL);

static struct attribute* temperature_attributes[] =
{
     &hw_temp_attribute.attr,
     NULL
};
static struct attribute_group temperature_attr_group =
{
     .attrs = temperature_attributes,
};

static int __init hw_temp_detect_init(void)
{
   int ret=0;

   TEMPERATURE_DBG("%s:start........\n", __func__);
   
   kobj = kobject_create_and_add("temperature", NULL);
   if (kobj == NULL) 
   {
       printk(KERN_ERR "%s:create kobject failed\n", __func__);
       return -1;
   }
    
   if (sysfs_create_group(kobj, &temperature_attr_group)) 
   {
       printk(KERN_ERR "%s:create sysfs group failed\n", __func__);
       kobject_put(kobj);
       return -1;
   } 
  
   hw_temp_detect_start();
   return ret;
}

static void __exit hw_temp_detect_exit (void)
{
    sysfs_remove_group(kobj, &temperature_attr_group);
}
late_initcall(hw_temp_detect_init);
module_exit(hw_temp_detect_exit);

MODULE_DESCRIPTION("hardware temperature detect for s10");
MODULE_LICENSE("GPL");
