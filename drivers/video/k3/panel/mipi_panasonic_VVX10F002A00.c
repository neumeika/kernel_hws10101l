/* Copyright (c) 2008-2011, Hisilicon Tech. Co., Ltd. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials provided
 *       with the distribution.
 *     * Neither the name of Code Aurora Forum, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
 * IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

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

#ifdef CONFIG_LCD_FIRMWARE_UPDATE_S10
#include "lcd_firmware.h"
struct i2c_client *g_lcd_client = NULL;
#endif

struct k3_fb_data_type *k3fd_hdmi = NULL;

#include <linux/i2c/gpio-i2c-adpt.h>

/*#define BOARD_HI4511MMU1_VER_B*/

#define PWM_LEVEL 100

atomic_t cm_on = ATOMIC_INIT(1);
atomic_t cabc_on = ATOMIC_INIT(1);
static int lcd_reset = 0;
static int set_cm_status(int value);
static int set_cabc_status(int value);
#define CONTROL_CABC	(0)

static int lcd_type = PANASONIC_LCD;
int get_lcd_type(void)
{
	return lcd_type;
}

static struct k3_fb_panel_data panasonic_panel_data;

static ssize_t panasonic_lcd_info_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int ret = 0;
	struct k3_panel_info *pinfo;
	pinfo = panasonic_panel_data.panel_info;
    if (get_lcd_type() == TRULY_LCD) {
		sprintf(buf, "TRULY 10.1'TFT %d x %d\n",
			pinfo->xres, pinfo->yres);
    } else {
		sprintf(buf, "Panasonic_VVX10F002A00 10.1'TFT %d x %d\n",
			pinfo->xres, pinfo->yres);
    }
	ret = strlen(buf) + 1;
	return ret;
}

static DEVICE_ATTR(lcd_info, S_IRUGO, panasonic_lcd_info_show, NULL);

static struct attribute *panasonic_attrs[] = {
	&dev_attr_lcd_info.attr,
	NULL,
};

static struct attribute_group panasonic_attr_group = {
	.attrs = panasonic_attrs,
};

static int panasonic_sysfs_init(struct platform_device *pdev)
{
	int ret;
	ret = sysfs_create_group(&pdev->dev.kobj, &panasonic_attr_group);
	if (ret) {
		k3fb_loge("create sysfs file failed!\n");
		return ret;
	}
	return 0;
}

static void panasonic_sysfs_deinit(struct platform_device *pdev)
{
	sysfs_remove_group(&pdev->dev.kobj, &panasonic_attr_group);
}

int panasonic_pwm_on(struct k3_fb_data_type *k3fd)
{
	struct k3_panel_info *pinfo = NULL;

	BUG_ON(k3fd == NULL);
	pinfo = &(k3fd->panel_info);

	if (lcd_reset == 1) {
		
    	if (get_lcd_type() == TRULY_LCD) {
			set_cabc_status(atomic_read(&cabc_on));
            set_cm_status(atomic_read(&cm_on));
        } else {
			if (!atomic_read(&cabc_on)) {
				set_cabc_status(0);
	        }
	        
			if (!atomic_read(&cm_on)) {
	            set_cm_status(0);
	        }
        }

		lcd_reset = 0;
    }    
	PWM_IOMUX_SET(&(k3fd->panel_info), NORMAL);

	gpio_direction_output(pinfo->gpio_led_en, 1);
	mdelay(2);	
	
	gpio_direction_input(k3fd->panel_info.gpio_pwm1);
	mdelay(2);
	pwm_set_backlight(k3fd->bl_level, &(k3fd->panel_info));

	return 0;
}

int panasonic_pwm_off(struct k3_fb_data_type *k3fd)
{
	struct k3_panel_info *pinfo = NULL;

	BUG_ON(k3fd == NULL);
	pinfo = &(k3fd->panel_info);

	gpio_direction_output(pinfo->gpio_led_en, 0);	
	mdelay(2);

	/* backlight off */
	pwm_set_backlight(0, &(k3fd->panel_info));
	gpio_direction_output(k3fd->panel_info.gpio_pwm0, 0);
	mdelay(2);
	gpio_direction_input(k3fd->panel_info.gpio_pwm1);
	mdelay(2);
	gpio_direction_input(k3fd->panel_info.gpio_pwm0);
	PWM_IOMUX_SET(&(k3fd->panel_info), LOWPOWER);

	return 0;
}

static void panasonic_disp_on(struct k3_fb_data_type *k3fd)
{
	u32 edc_base = 0;
	struct k3_panel_info *pinfo = NULL;

	BUG_ON(k3fd == NULL);
	edc_base = k3fd->edc_base;
	pinfo = &(k3fd->panel_info);
	/* lcd vcc */
	//LCD_VCC_ENABLE(pinfo);

	/* LCD */
	LCD_IOMUX_SET(pinfo, NORMAL);

	/* lcd gpio */
	gpio_direction_input(pinfo->gpio_lcd_id0);
	mdelay(2);
	gpio_direction_input(pinfo->gpio_lcd_id1);
	mdelay(2);

	gpio_direction_output(pinfo->gpio_avdd, 1);
	mdelay(2);
	lcd_reset = 1;    
}

static void panasonic_disp_off(struct k3_fb_data_type *k3fd)
{
	u32 edc_base = 0;
	struct k3_panel_info *pinfo = NULL;

	BUG_ON(k3fd == NULL);
	edc_base = k3fd->edc_base;
	pinfo = &(k3fd->panel_info);

	/* LCD */
	gpio_direction_input(pinfo->gpio_lcd_id0);
	mdelay(2);
	gpio_direction_input(pinfo->gpio_lcd_id1);
	
	mdelay(2);

	
	gpio_direction_output(pinfo->gpio_avdd, 0);
	mdelay(2);
	lcd_reset = 0;
	LCD_IOMUX_SET(pinfo, LOWPOWER);

	/* lcd vcc */
	//LCD_VCC_DISABLE(pinfo);
}

static int mipi_panasonic_panel_on(struct platform_device *pdev)
{
	struct k3_fb_data_type *k3fd = NULL;

	BUG_ON(pdev == NULL);

	k3fd = (struct k3_fb_data_type *)platform_get_drvdata(pdev);
	BUG_ON(k3fd == NULL);

	if (!k3fd->panel_info.display_on) {
		/* lcd display on */
		panasonic_disp_on(k3fd);
		k3fd->panel_info.display_on = true;

		
	}

	return 0;
}

static int mipi_panasonic_panel_off(struct platform_device *pdev)
{
	struct k3_fb_data_type *k3fd = NULL;

	BUG_ON(pdev == NULL);

	k3fd = (struct k3_fb_data_type *)platform_get_drvdata(pdev);
	BUG_ON(k3fd == NULL);

	if (k3fd->panel_info.display_on) {
		k3fd->panel_info.display_on = false;

		
		/* lcd display off */
		panasonic_disp_off(k3fd);
	}

	return 0;
}

static int mipi_panasonic_panel_remove(struct platform_device *pdev)
{
	struct k3_fb_data_type *k3fd = NULL;
	
	BUG_ON(pdev == NULL);
	k3fd = (struct k3_fb_data_type *)platform_get_drvdata(pdev);
	BUG_ON(k3fd == NULL);

	k3fb_logi("index=%d, enter!\n", k3fd->index);

	if (k3fd->panel_info.bl_set_type & BL_SET_BY_PWM) {
		PWM_CLK_PUT(&(k3fd->panel_info));
	}
	LCD_VCC_PUT(&(k3fd->panel_info));

	panasonic_sysfs_deinit(pdev);

	k3fb_logi("index=%d, exit!\n", k3fd->index);

	return 0;
}

static int mipi_panasonic_panel_set_backlight(struct platform_device *pdev)
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

static int mipi_panasonic_panel_set_fastboot(struct platform_device *pdev)
{
	struct k3_fb_data_type *k3fd = NULL;

	BUG_ON(pdev == NULL);

	k3fd = (struct k3_fb_data_type *)platform_get_drvdata(pdev);
	BUG_ON(k3fd == NULL);

	LCD_VCC_ENABLE(&(k3fd->panel_info));
	LCD_IOMUX_SET(&(k3fd->panel_info), NORMAL);
	LCD_GPIO_REQUEST(&(k3fd->panel_info));

	if (k3fd->panel_info.bl_set_type & BL_SET_BY_PWM) {
		PWM_IOMUX_SET(&(k3fd->panel_info), NORMAL);
		PWM_GPIO_REQUEST(&(k3fd->panel_info));
	}

	k3fd->panel_info.display_on = true;

	return 0;
}

static u32 g_cm_cabc_status = 0x1A;

static char cmd_begin[] = {
	0xF3,
	0xA0,
};

static char cmd_cabc_cm_status[] = {
	0x0E,
	0x1A,
};

static struct dsi_cmd_desc panasonic_parameter_begin = {
	.dtype = 	DTYPE_GEN_WRITE2,
	.vc        =  0,
	.wait    =  30,
	.waittype = WAIT_TYPE_US,
	.dlen         = sizeof(cmd_begin),
	.payload    = cmd_begin,
};

static struct dsi_cmd_desc panasonic_parameter = {
	.dtype = 	DTYPE_GEN_WRITE2,
	.vc        =  0,
	.wait    =  30,
	.waittype = WAIT_TYPE_US,
	.dlen         = sizeof(cmd_cabc_cm_status),
	.payload    = cmd_cabc_cm_status,
};

static struct dsi_cmd_desc panasonic_parameter_end = {
	.dtype = 	DTYPE_GEN_WRITE,
	.vc        =  0,
	.wait    =  30,
	.waittype = WAIT_TYPE_US,
	.dlen         = 0,
	.payload    = 0,
};

void  set_value(u32 *value, u32 b_val, u8 bw, u8 bs)
{
	u32 mask = (1 << bw) - 1;
	u32 tmp = *value;
	tmp &= ~(mask << bs);
	*value = tmp | (b_val & mask) << bs;
}
static int set_panasonic_function(u32 status)
{
	cmd_cabc_cm_status[1] = status;

	mipi_dsi_swrite(&panasonic_parameter_begin, k3fd_hdmi->edc_base);

	mipi_dsi_swrite(&panasonic_parameter, k3fd_hdmi->edc_base);

	mipi_dsi_swrite(&panasonic_parameter_end, k3fd_hdmi->edc_base);
	
	return 0;
}

#define CABC_MIPI	(0x15)
#define CM_MIPI		(0x16)
#define CM_CABC_ON  (0x55)
#define CM_CABC_OFF (0xAA)
static char truly_cabc_cm_status[] = {
	CABC_MIPI,
	CM_CABC_ON,
};

static struct dsi_cmd_desc truly_parameter = {
	.dtype = 	DTYPE_GEN_WRITE2,
	.vc        =  0,
	.wait    =  30,
	.waittype = WAIT_TYPE_US,
	.dlen         = sizeof(truly_cabc_cm_status),
	.payload    = truly_cabc_cm_status,
};

static int set_truly_function(u32  name, u32 status)
{
	if ((name != CABC_MIPI) && (name != CM_MIPI)) {
		printk(KERN_ERR "cmd is not valid\n");
        return -1;
    }

	if ((status!= CM_CABC_ON) && (status != CM_CABC_OFF)) {
		printk(KERN_ERR "status is not valid\n");
        return -1;
    }
    
   	truly_cabc_cm_status[0] = name;
    truly_cabc_cm_status[1] = status;
	mipi_dsi_swrite(&truly_parameter, k3fd_hdmi->edc_base);

	return 0;
}

static int mipi_panasonic_panel_set_cabc(struct platform_device *pdev, int value)
{
    set_cabc_status(value);

	return 0;
}

static struct k3_panel_info panasonic_panel_info = {0};
static struct k3_fb_panel_data panasonic_panel_data = {
	.panel_info = &panasonic_panel_info,
	.on = mipi_panasonic_panel_on,
	.off = mipi_panasonic_panel_off,
	.remove = mipi_panasonic_panel_remove,
	.set_backlight = mipi_panasonic_panel_set_backlight,
	.set_fastboot = mipi_panasonic_panel_set_fastboot,
	.set_cabc = mipi_panasonic_panel_set_cabc,
};

static int __devinit panasonic_probe(struct platform_device *pdev)
{
	struct k3_panel_info *pinfo = NULL;
	struct resource *res = NULL;
	int id0 = 0;
    int id1 = 0;

	pinfo = panasonic_panel_data.panel_info;

	/* init lcd panel info */
	pinfo->display_on = false;
	pinfo->xres = 1920;
	pinfo->yres = 1200;
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

	pinfo->frc_enable = 1;
	pinfo->esd_enable = 1;
	pinfo->sbl_enable = 1;

	pinfo->sbl.bl_max = 0x64;
	pinfo->sbl.cal_a = 0x18;
	pinfo->sbl.cal_b = 0xd8;
	pinfo->sbl.str_limit = 0x50;
	pinfo->ldi.h_back_porch = 20;//32;

#if defined(CONFIG_OVERLAY_COMPOSE)
	pinfo->ldi.h_front_porch = 25;
#else
	pinfo->ldi.h_front_porch = 38;//607;//445;
#endif
	pinfo->ldi.h_pulse_width = 2;//32;

#if defined(CONFIG_OVERLAY_COMPOSE)
	pinfo->ldi.v_back_porch = 12;
#else
	pinfo->ldi.v_back_porch = 4;//15;
#endif
	pinfo->ldi.v_front_porch = 6;//15;
	pinfo->ldi.v_pulse_width = 2;//5;
	pinfo->ldi.hsync_plr = 1;
	pinfo->ldi.vsync_plr = 1;
	pinfo->ldi.pixelclk_plr = 0;
	pinfo->ldi.data_en_plr = 0;
	pinfo->ldi.disp_mode = LDI_DISP_MODE_NOT_3D_FBF;

	/* Note: must init here */
	pinfo->frame_rate = 60;//50;
	/*pinfo->clk_rate = LCD_GET_CLK_RATE(pinfo);*/
	pinfo->clk_rate = 144000000;//150000000;
	pinfo->mipi.lane_nums = DSI_4_LANES;
	pinfo->mipi.color_mode = DSI_24BITS_1;
	pinfo->mipi.vc = 0;
	pinfo->mipi.dsi_bit_clk = 468;  /* clock lane(p/n) */

	/* lcd vcc */
	LCD_VCC_GET(pdev, pinfo);
	LCDIO_SET_VOLTAGE(pinfo, 1800000, 1800000);

	/* lcd iomux */
	LCD_IOMUX_GET(pinfo);
	/* lcd resource */
	LCD_RESOURCE(pdev, pinfo, res);

	if (pinfo->bl_set_type & BL_SET_BY_PWM) {
		/* pwm clock*/
		PWM_CLK_GET(pinfo);
		/* pwm iomux */
		PWM_IOMUX_GET(pinfo);
		/* pwm resource */
		PWM_RESOUTCE(pdev, pinfo, res);
	}

	/* alloc panel device data */
	if (platform_device_add_data(pdev, &panasonic_panel_data,
			sizeof(struct k3_fb_panel_data))) {
		k3fb_loge("failed to platform_device_add_data!\n");
		platform_device_put(pdev);
		return -ENOMEM;
	}

	k3_fb_add_device(pdev);

	panasonic_sysfs_init(pdev);

	gpio_direction_input(pinfo->gpio_lcd_id0);
	gpio_direction_input(pinfo->gpio_lcd_id1);
    id0 = gpio_get_value(pinfo->gpio_lcd_id0);
    id1 = gpio_get_value(pinfo->gpio_lcd_id1);
    if ((id0 == 1) && (id1 == 1)) {
		lcd_reset = 1;
		lcd_type = TRULY_LCD;
        set_truly_function(CABC_MIPI, CM_CABC_ON);
        set_truly_function(CM_MIPI, CM_CABC_ON);
        lcd_reset = 0;        
        pinfo->frc_enable = 0;
    } else {
		lcd_type = PANASONIC_LCD;
    }

	
	return 0;
}

static struct platform_driver this_driver = {
	.probe = panasonic_probe,
	.remove = NULL,
	.suspend = NULL,
	.resume = NULL,
	.shutdown = NULL,
	.driver = {
		.name = "mipi_panasonic_VVX10F002A00",
	},
};


static int set_cm_status(int value)
{
	int status = 0;

    printk(KERN_INFO "[%s %d] lcd_reset %d, cm_on %d, set cm %s",
        	__func__, __LINE__, lcd_reset, atomic_read(&cm_on), value ?"on" :"off");

	if (atomic_read(&cm_on) == value && (lcd_reset == 0)) {
		return 0;
    }


	if (get_lcd_type() == TRULY_LCD) {
		if (value == 0) {
			status = CM_CABC_OFF;
	    } else {
			status = CM_CABC_ON;
	    }
	    
		set_truly_function(CM_MIPI, status);
	} else {
		if (value == 0) {
			set_value(&g_cm_cabc_status, 0x00, 1, 4);
			set_panasonic_function(g_cm_cabc_status);	
	    } else {
			set_value(&g_cm_cabc_status, 0x01, 1, 4);
			set_panasonic_function(g_cm_cabc_status);				
	    }
     }
    atomic_set(&cm_on, value);

    return 0;
}


static int set_cabc_status(int value)
{
	
#if CONTROL_CABC
		int status = 0;
	    printk(KERN_INFO "[%s %d] lcd_reset %d, cabc_on %d, set cabc %s",
	        	__func__, __LINE__, lcd_reset, atomic_read(&cabc_on), value ? "on" :"off");


		if (atomic_read(&cabc_on) == value && (lcd_reset == 0)) {
			return 0;
	    }

    	if (get_lcd_type() == TRULY_LCD) {
	        if (value == 0) {
	            status = CM_CABC_OFF;
	        } else {
	            status = CM_CABC_ON;
	        }
	        
	        set_truly_function(CABC_MIPI, status);
		} else {
            if (value == 0) {
                set_value(&g_cm_cabc_status, 0x00, 1, 3);
                set_panasonic_function(g_cm_cabc_status);   
            } else {
                set_value(&g_cm_cabc_status, 0x01, 1, 3);
                set_panasonic_function(g_cm_cabc_status);               
            }
		}

	    atomic_set(&cabc_on, value);
#endif   
		return 0;

}

static  ssize_t set_cm_function(struct device_driver *drv, const char *buf, size_t count)
{
	int status = 0;	
	if (buf == NULL) {
		return count;	
	} 	

	sscanf(buf, "%d", &status);
	if ((status != 0) && (status != 1)) {
		printk(KERN_ERR "[%s %d] Must be 1 or 0\n", __func__, __LINE__);	
	} else {
   		set_cm_status(status);
	}
	return count;
}

static  ssize_t set_cabc_function(struct device_driver *drv, const char *buf, size_t count)
{
	int status = 0;	
	if (buf == NULL) {
		return count;	
	}
    
	sscanf(buf, "%d", &status);	
	if ((status != 0) && (status != 1)) {
		printk(KERN_ERR "[%s %d] Must be 1 or 0\n", __func__, __LINE__);	
	} else {
	   	set_cabc_status(status);
	}
	return count;
}

static DRIVER_ATTR(set_cm_function, 0644, NULL, set_cm_function);
static DRIVER_ATTR(set_cabc_function, 0644, NULL, set_cabc_function);

static  ssize_t set_backlight_status(struct device_driver *drv, const char *buf, size_t count)
{
	static int backlight_status = -1;
	int status = 0;

	status = simple_strtoul(buf, NULL, 0);

	if ((status != 0) && (status != 1)) {
		printk(KERN_ERR "[%s %d] Must be 1 or 0\n", __func__, __LINE__);
	} else if (status != backlight_status) {
		printk(KERN_ERR "[%s %d] status %d, backlight_status %d\n", __func__, __LINE__, status, backlight_status);

		if (1 == status) {
			panasonic_pwm_on(k3fd_hdmi);
		} else if (0 == status) {
			panasonic_pwm_off(k3fd_hdmi);
		}

		backlight_status = status;
	}

	return count;
}

static DRIVER_ATTR(set_backlight_status, 0664, NULL, set_backlight_status);

#ifdef CONFIG_LCD_FIRMWARE_UPDATE_S10
static int lcd_panasonic_i2c_write(uint8_t address, uint8_t *data, unsigned short length)
{
	struct i2c_client *client = g_lcd_client;
	struct i2c_msg msg;
	uint8_t *buf = data;
    int i = 0;
    uint8_t buf2[3] = {0};
    int ret = 0;

	for (i = 0; i < length; i++) {
		buf2[0] = i * 2;
        buf2[1] = buf[i * 2];
        buf2[2] = buf[i*2 + 1];
        msg.addr = address;
        msg.flags = 0;
        msg.len = 3;
        msg.buf = buf2;
        ret = i2c_transfer(client->adapter, &msg, 1);

        if (ret < 0) {
            printk(KERN_ERR "[%s %d]  msg.addr = 0x%x,  len = %d,buf[0] = 0x%x, buf[1] = 0x%x, buf[2]  = 0x%x \n", __func__, __LINE__, msg.addr, msg.len,     msg.buf[0],msg.buf[1], msg.buf[2]);
            return -1;
        }

        msleep(5);
   }

	return 0;
}

static int update_firmware(void)
{
	uint8_t i = 0;
    uint8_t address = 0;

	if (g_lcd_client != NULL) {
		address = g_lcd_client->addr;
    }

	printk(KERN_ERR "[%s %d] start to update firmware\n", __func__, __LINE__);

    for (i = 0; i < 8; i++) {
		if (lcd_panasonic_i2c_write(address + i, lcd_firmware_data[i], 128) < 0) {
            printk(KERN_ERR "[%s %d] update failed. \n", __func__, __LINE__);
			return -1;
        }
    }

	printk(KERN_ERR "[%s %d] update success\n", __func__, __LINE__);

    return 0;
}

static  ssize_t lcd_update_firmware(struct device_driver *drv, const char *buf, size_t count)
{

	int ret = 0;

	if (buf == NULL) {
		return count;
	}

	if (strcmp(buf, "updatelcdfirmwarefors10")) {
		gpio_direction_output(GPIO_I2C_SCL, 1);
		msleep(1);
		gpio_direction_output(GPIO_I2C_SDA, 1);
		msleep(1);
		ret = update_firmware();
		gpio_direction_input(GPIO_I2C_SCL);
		msleep(1);
		gpio_direction_input(GPIO_I2C_SDA);
		msleep(1);
	} else {
		printk(KERN_ERR "[%s %d] nothing.\n", __func__, __LINE__);
	}

	if (ret < 0) {
		printk(KERN_ERR "[%s %d] update firmware failed\n", __func__, __LINE__);
    } else {
		printk(KERN_ERR "[%s %d] update firmware success\n", __func__, __LINE__);
    }

	return count;
}

static DRIVER_ATTR(lcd_update_firmware, 0664, NULL, lcd_update_firmware);

static int lcd_panasonic_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	g_lcd_client = client;
    printk(KERN_ERR "[%s]\n", __func__);

    return 0;
}

static const struct i2c_device_id lcd_panasonic_id[] = {
	{"lcd_panasonic", 0},
    {},
};

static struct i2c_driver lcd_i2c_driver = {
	.driver	=	{
		.name = "lcd_panasonic",
        .owner = THIS_MODULE,
    },
	.probe = lcd_panasonic_probe,
	.id_table = lcd_panasonic_id,
};
#endif

static int __init mipi_panasonic_panel_init(void)
{
	int ret = 0;

	ret = platform_driver_register(&this_driver);
	if (ret) {
		k3fb_loge("not able to register the driver, error=%d!\n", ret);
		return ret;
	}

	if(driver_create_file(&(this_driver.driver), &driver_attr_set_cm_function) < 0) 	
	{
		pr_err("failed to create sysfs entry(set_cm_function): \n");	
	}	
	if(driver_create_file(&(this_driver.driver), &driver_attr_set_cabc_function) < 0) 	
	{
		pr_err("failed to create sysfs entry(set_cabc_function): \n");	
	}


	if(driver_create_file(&(this_driver.driver), &driver_attr_set_backlight_status) < 0)
	{
		pr_err("failed to create sysfs entry(set_backlight_status): \n");
	}

#ifdef CONFIG_LCD_FIRMWARE_UPDATE_S10
	ret = i2c_add_driver(&lcd_i2c_driver);
	if (ret) {
		printk(KERN_ERR "%s: failed to add i2c driver.\n", __func__);
    }

	if (driver_create_file(&(this_driver.driver), &driver_attr_lcd_update_firmware)) {
		printk(KERN_ERR "failed to create sysfs entry(lcd_update_firmware): \n");
    }
#endif
	return ret;
}

module_init(mipi_panasonic_panel_init);
