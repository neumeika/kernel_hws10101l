#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/pwm.h>
#include <mach/platform.h>
#include <mach/gpio.h>

#include "k3_fb.h"
#include "k3_fb_def.h"
#include "mipi_dsi.h"
#include "mipi_reg.h"
#include <linux/i2c/gpio-i2c-adpt.h>

#define PWM_LEVEL 		(100)
static int gpio_cm = 0;
static int gpio_cabc = 0;
static struct k3_fb_panel_data qimei_panel_data;
struct k3_fb_data_type *k3fd_backlight = NULL;

#define SUSPEND_TO_RESUME	(500)
unsigned long jiffies_old = 0;

int qimei_pwm_on(struct k3_fb_data_type *k3fd)
{
	struct k3_panel_info *pinfo = NULL;

	BUG_ON(k3fd == NULL);
	pinfo = &(k3fd->panel_info);

	gpio_direction_output(pinfo->gpio_led_en, 1);
	mdelay(20);	
	/* backlight on */
	PWM_IOMUX_SET(&(k3fd->panel_info), NORMAL);
	pwm_set_backlight(k3fd->bl_level, &(k3fd->panel_info));

	return 0;
}

int qimei_pwm_off(struct k3_fb_data_type *k3fd)
{
	struct k3_panel_info *pinfo = NULL;

	BUG_ON(k3fd == NULL);
	pinfo = &(k3fd->panel_info);

	/* backlight off */
	pwm_set_backlight(0, &(k3fd->panel_info));    
	gpio_direction_output(k3fd->panel_info.gpio_pwm0, 0);
	mdelay(2);
	gpio_direction_output(pinfo->gpio_led_en, 0);	
	mdelay(2);
	gpio_direction_input(k3fd->panel_info.gpio_pwm0);
	PWM_IOMUX_SET(&(k3fd->panel_info), LOWPOWER);

	return 0;
}

static void qimei_disp_on(struct k3_fb_data_type *k3fd)
{
	u32 edc_base = 0;
	struct k3_panel_info *pinfo = NULL;
	int try_times = 0;

	BUG_ON(k3fd == NULL);
	edc_base = k3fd->edc_base;
	pinfo = &(k3fd->panel_info);

	/* LCD */
	LCD_IOMUX_SET(pinfo, NORMAL);

	/* lcd gpio */
	gpio_direction_input(pinfo->gpio_lcd_id1);
	mdelay(1);

	while (time_is_after_eq_jiffies(jiffies_old + msecs_to_jiffies(SUSPEND_TO_RESUME))) {
		mdelay(10);
		if (try_times > 50) {
			break;	
        }
        try_times++;
    }
   
	gpio_direction_output(pinfo->gpio_avdd, 1);
	mdelay(1);
}

static void qimei_disp_off(struct k3_fb_data_type *k3fd)
{
	u32 edc_base = 0;
	struct k3_panel_info *pinfo = NULL;

	BUG_ON(k3fd == NULL);
	edc_base = k3fd->edc_base;
	pinfo = &(k3fd->panel_info);

	gpio_direction_output(pinfo->gpio_avdd, 0);
	mdelay(1);
	LCD_IOMUX_SET(pinfo, LOWPOWER);

    jiffies_old = jiffies;
}

static int mipi_qimei_panel_on(struct platform_device *pdev)
{
	struct k3_fb_data_type *k3fd = NULL;

	BUG_ON(pdev == NULL);

	k3fd = (struct k3_fb_data_type *)platform_get_drvdata(pdev);
	BUG_ON(k3fd == NULL);

	if (!k3fd->panel_info.display_on) {
		/* lcd display on */
		qimei_disp_on(k3fd);
		k3fd->panel_info.display_on = true;
	}

	return 0;
}

static int mipi_qimei_panel_off(struct platform_device *pdev)
{
	struct k3_fb_data_type *k3fd = NULL;

	BUG_ON(pdev == NULL);

	k3fd = (struct k3_fb_data_type *)platform_get_drvdata(pdev);
	BUG_ON(k3fd == NULL);

	if (k3fd->panel_info.display_on) {
		k3fd->panel_info.display_on = false;
		/* lcd display off */
		qimei_disp_off(k3fd);
	}

	return 0;
}

static int mipi_qimei_panel_remove(struct platform_device *pdev)
{
	struct k3_fb_data_type *k3fd = NULL;
	
	pr_info("k3fb, %s: enter!\n", __func__);

	BUG_ON(pdev == NULL);

	k3fd = (struct k3_fb_data_type *)platform_get_drvdata(pdev);
	BUG_ON(k3fd == NULL);

	if (k3fd->panel_info.bl_set_type & BL_SET_BY_PWM) {
		PWM_CLK_PUT(&(k3fd->panel_info));
	}

	pr_info("k3fb, %s: exit!\n", __func__);

	return 0;
}

static int mipi_qimei_panel_set_backlight(struct platform_device *pdev)
{
	struct k3_fb_data_type *k3fd = NULL;

	BUG_ON(pdev == NULL);

	k3fd = (struct k3_fb_data_type *)platform_get_drvdata(pdev);
	BUG_ON(k3fd == NULL);

	if (k3fd->panel_info.bl_set_type & BL_SET_BY_PWM) {
		return pwm_set_backlight(k3fd->bl_level, &(k3fd->panel_info));
	} else {
		return 0;
	}
}

static int lcd_gpio_request(struct k3_panel_info *pinfo)
{
	if (gpio_request(pinfo->gpio_led_en, GPIO_LED_EN_NAME) != 0) { 
		pr_err("k3fb, %s: failed to request gpio led enable.\n", __func__); 
	}
    
	if (gpio_request(pinfo->gpio_avdd, GPIO_LCD_AVDD_NAME) != 0) { 
		pr_err("k3fb, %s:  failed to request gpio lcd avdd.\n", __func__); 
	}
    
	if (gpio_request(pinfo->gpio_lcd_id1, GPIO_LCD_ID1_NAME) != 0) { 
		pr_err("k3fb, %s: failed to request gpio_lcd_id1.\n", __func__); 
	}

	if (gpio_request(pinfo->gpio_pwm0, GPIO_LCD_PWM0_NAME) != 0) { 
		pr_err("k3fb, %s: failed to request GPIO_LCD_PWM0_.\n", __func__); 
	}

	if (gpio_request(gpio_cm, "gpio_lcd_cm") != 0) { 
		pr_err("k3fb, %s: failed to request GPIO_LCD_CM.\n", __func__);
	}

	if (gpio_request(gpio_cabc, "gpio_lcd_cabc") != 0) { 
		pr_err("k3fb, %s: failed to request GPIO_LCD_CABC.\n", __func__); 
	}

    //default to open cm and cabc
	gpio_direction_output(gpio_cm, 0);
    gpio_direction_output(gpio_cabc, 0);

    return 0;
}

static int mipi_qimei_panel_set_fastboot(struct platform_device *pdev)
{
	struct k3_fb_data_type *k3fd = NULL;

	BUG_ON(pdev == NULL);

	k3fd = (struct k3_fb_data_type *)platform_get_drvdata(pdev);   
	BUG_ON(k3fd == NULL);
    k3fd_backlight = k3fd;
	LCD_IOMUX_SET(&(k3fd->panel_info), NORMAL);   
    PWM_IOMUX_SET(&(k3fd->panel_info), NORMAL);
    
	lcd_gpio_request(&(k3fd->panel_info));

	k3fd->panel_info.display_on = true;

	return 0;
}

static int mipi_qimei_panel_set_cabc(struct platform_device *pdev, int value)
{
    //gpio_direction_output(gpio_cabc, !value);
	return 0;
}

static struct k3_panel_info qimei_panel_info = {0};
static struct k3_fb_panel_data qimei_panel_data = {
	.panel_info = &qimei_panel_info,
	.on = mipi_qimei_panel_on,
	.off = mipi_qimei_panel_off,
	.remove = mipi_qimei_panel_remove,
	.set_backlight = mipi_qimei_panel_set_backlight,
	.set_fastboot = mipi_qimei_panel_set_fastboot,
	.set_cabc = mipi_qimei_panel_set_cabc,
};

static ssize_t qimei_lcd_info_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "Qimei_HJ101IA_01F_10 10.1'TFT 1280 x 800\n");
}


static  ssize_t set_cm_function(struct device *dev, struct device_attribute *attr,
			 const char *buf, size_t count)
{
	int status = 0;	
	if (buf == NULL) {
		return count;	
	} 	

	sscanf(buf, "%d", &status);
	if ((status != 0) && (status != 1)) {
		printk(KERN_ERR "[%s %d] Must be 1 or 0\n", __func__, __LINE__);	
	} else {
        gpio_direction_output(gpio_cm, !status);
	}
	return count;
}

static  ssize_t set_cabc_function(struct device *dev, struct device_attribute *attr,
			 const char *buf, size_t count)
{
	int status = 0;	
	if (buf == NULL) {
		return count;	
	}
    
	sscanf(buf, "%d", &status);	
	if ((status != 0) && (status != 1)) {
		printk(KERN_ERR "[%s %d] Must be 1 or 0\n", __func__, __LINE__);	
	} else {
        gpio_direction_output(gpio_cabc, !status);
	}
    
	return count;
}

static  ssize_t set_backlight_status(struct device *dev, struct device_attribute *attr,
			 const char *buf, size_t count)
{
	static int backlight_status = -1;
	int status = 0;

	status = simple_strtoul(buf, NULL, 0);

	if ((status != 0) && (status != 1)) {
		printk(KERN_ERR "[%s %d] Must be 1 or 0\n", __func__, __LINE__);
	} else if (status != backlight_status) {
		printk(KERN_ERR "[%s %d] status %d, backlight_status %d\n", __func__, __LINE__, status, backlight_status);
		if (1 == status) {
            qimei_pwm_on(k3fd_backlight);
		} else if (0 == status) {
            qimei_pwm_off(k3fd_backlight);
		}

		backlight_status = status;
	}

	return count;
}

static DEVICE_ATTR(lcd_info, 0444, qimei_lcd_info_show, NULL);
static DEVICE_ATTR(set_cm_function, 0644, NULL, set_cm_function);
static DEVICE_ATTR(set_cabc_function, 0644, NULL, set_cabc_function);
static DEVICE_ATTR(set_backlight_status, 0664, NULL, set_backlight_status);

static int qimei_sysfs_init(struct platform_device *pdev)
{
	int status = 0;

    status = device_create_file(&pdev->dev, &dev_attr_lcd_info);
    if (status < 0) {
        printk(KERN_ERR "Failed to device_create_file  dev_attr_lcd_info\n");
    }

    status = device_create_file(&pdev->dev, &dev_attr_set_cm_function);
    if (status < 0) {
        printk(KERN_ERR "Failed to device_create_file dev_attr_set_cm_function\n");
    }

    status = device_create_file(&pdev->dev, &dev_attr_set_cabc_function);
    if (status < 0) {
        printk(KERN_ERR "Failed to device_create_file dev_attr_set_cabc_function\n");
    }

    status = device_create_file(&pdev->dev, &dev_attr_set_backlight_status);
    if (status < 0) {
        printk(KERN_ERR "Failed to device_create_file dev_attr_set_cabc_function\n");
    }
    
	return 0;
}


static int __devinit qimei_probe(struct platform_device *pdev)
{
	struct k3_panel_info *pinfo = NULL;
	struct resource *res = NULL;

	pinfo = qimei_panel_data.panel_info;

	/* init lcd panel info */
	pinfo->display_on = false;
	pinfo->xres = 1280;
	pinfo->yres = 800;
	pinfo->width = 217;
	pinfo->height = 136;
	pinfo->type = PANEL_MIPI_VIDEO;
	pinfo->orientation = LCD_LANDSCAPE;
	pinfo->bpp = EDC_OUT_RGB_888;
	pinfo->s3d_frm = EDC_FRM_FMT_2D;
	pinfo->bgr_fmt = EDC_RGB;
	pinfo->bl_set_type = BL_SET_BY_PWM;
	pinfo->bl_max = PWM_LEVEL;
	pinfo->bl_min = 1;

    pinfo->frc_enable = 0;
    pinfo->esd_enable = 1;
    pinfo->sbl_enable = 1;

	pinfo->sbl.bl_max = 0x64;
	pinfo->sbl.cal_a = 0x18;    
	pinfo->sbl.cal_b = 0xd8;
	pinfo->sbl.str_limit = 0x50;
    
	pinfo->ldi.h_back_porch = 50;//32;
	pinfo->ldi.h_front_porch = 186;//607;//445;
	pinfo->ldi.h_pulse_width = 5;//32;
	pinfo->ldi.v_back_porch = 10;//15;
	pinfo->ldi.v_front_porch = 21;//15;
	pinfo->ldi.v_pulse_width = 2;//5;

    pinfo->ldi.hsync_plr = 1;
	pinfo->ldi.vsync_plr = 1;
	pinfo->ldi.pixelclk_plr = 0;
	pinfo->ldi.data_en_plr = 0;
	pinfo->ldi.disp_mode = LDI_DISP_MODE_NOT_3D_FBF;

	/* Note: must init here */
    
	pinfo->frame_rate = 60;//50;
	pinfo->clk_rate = 76000000;//150000000;

    pinfo->mipi.lane_nums = DSI_4_LANES;
	pinfo->mipi.color_mode = DSI_24BITS_1;
	pinfo->mipi.vc = 0;

	pinfo->mipi.dsi_bit_clk = 241;  /* clock lane(p/n) */

	/* lcd iomux */
	LCD_IOMUX_GET(pinfo);
	/* lcd resource */
	LCD_RESOURCE(pdev, pinfo, res);

    res   = platform_get_resource_byname((pdev), IORESOURCE_IO, GPIO_CM_NAME);
    gpio_cm = res->start;
    res = platform_get_resource_byname((pdev), IORESOURCE_IO, GPIO_CABC_NAME);
   	gpio_cabc = res->start;

	if (pinfo->bl_set_type & BL_SET_BY_PWM) {
		/* pwm clock*/
		PWM_CLK_GET(pinfo);
		/* pwm iomux */
		PWM_IOMUX_GET(pinfo);
		/* pwm resource */
		PWM_RESOUTCE(pdev, pinfo, res);
	}

	/* alloc panel device data */
	if (platform_device_add_data(pdev, &qimei_panel_data,
			sizeof(struct k3_fb_panel_data))) {
		pr_err("k3fb, %s: platform_device_add_data failed!\n", __func__);
		platform_device_put(pdev);
		return -ENOMEM;
	}

	k3_fb_add_device(pdev);
    qimei_sysfs_init(pdev);
	return 0;
}

static struct platform_driver this_driver = {
	.probe = qimei_probe,
	.remove = NULL,
	.suspend = NULL,
	.resume = NULL,
	.shutdown = NULL,
	.driver = {
		.name = "mipi_qimei_HJ101IA_01F_10",
	},
};

static int __init mipi_qimei_panel_init(void)
{
	int ret = 0;

	ret = platform_driver_register(&this_driver);
	if (ret) {
		pr_err("k3fb, %s not able to register the driver\n", __func__);
		return ret;
	}

	return ret;
}

module_init(mipi_qimei_panel_init);
