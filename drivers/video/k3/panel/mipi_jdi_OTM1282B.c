/* Copyright (c) 2008-2011, Hisilicon Tech. Co., Ltd. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *	 * Redistributions of source code must retain the above copyright
 *	   notice, this list of conditions and the following disclaimer.
 *	 * Redistributions in binary form must reproduce the above
 *	   copyright notice, this list of conditions and the following
 *	   disclaimer in the documentation and/or other materials provided
 *	   with the distribution.
 *	 * Neither the name of Code Aurora Forum, Inc. nor the names of its
 *	   contributors may be used to endorse or promote products derived
 *	   from this software without specific prior written permission.
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
#include <mach/boardid.h>

#include "k3_fb.h"
#include "k3_fb_def.h"
#include "mipi_dsi.h"
#include "mipi_reg.h"
#include <linux/lcd_tuning.h>

#define PWM_LEVEL 100


/*----------------Power ON Sequence(sleep mode to Normal mode)---------------------*/

static char soft_reset[] = {
	0x01,
};

static char bl_level_0[] = {
	0x51,
	0x00,
};

static char bl_level[] = {
	0x51,
	0xd0,
};

static char bl_enable[] = {
	0x53,
	0x2C,
};

static char bl_mode[] = {
	0x55,
	0x01,
};

static char te_enable[] = {
	0x35,
	0x00,
};


static char exit_sleep[] = {
	0x11,
};

static char normal_display_on[] = {
	0x13,
};

static char all_pixels_off[] = {
	0x22,
};

static char display_on[] = {
	0x29,
};

static char display_off[] = {
	0x28,
};

static char enter_sleep[] = {
	0x10,
};

static char get_chip_id[] = {
	0xDA,
};

static char lcd_disp_x[] = {
	0x2A,
	0x00, 0x00,0x02,0xCF
};

static char lcd_disp_y[] = {
	0x2B,
	0x00, 0x00,0x04,0xFF
};

static struct dsi_cmd_desc jdi_backlight_cmds[] = {
	{DTYPE_DCS_WRITE1, 0, 100, WAIT_TYPE_US,
		sizeof(bl_level), bl_level},
	{DTYPE_DCS_WRITE1, 0, 100, WAIT_TYPE_US,
		sizeof(bl_enable), bl_enable},
};

static struct dsi_cmd_desc jdi_video_on_cmds[] = {
	{DTYPE_DCS_WRITE, 0, 10, WAIT_TYPE_MS,
		sizeof(te_enable), te_enable},
	{DTYPE_DCS_WRITE, 0, 200, WAIT_TYPE_MS,
		sizeof(all_pixels_off), all_pixels_off},	
	{DTYPE_DCS_WRITE, 0, 200, WAIT_TYPE_MS,
		sizeof(normal_display_on), normal_display_on},	
	{DTYPE_DCS_WRITE, 0, 200, WAIT_TYPE_MS,
		sizeof(exit_sleep), exit_sleep},
	{DTYPE_DCS_WRITE, 0, 10, WAIT_TYPE_MS,
		sizeof(display_on), display_on},
	{DTYPE_DCS_WRITE1, 0, 100, WAIT_TYPE_US,
		sizeof(bl_level), bl_level},
	{DTYPE_DCS_WRITE1, 0, 100, WAIT_TYPE_US,
		sizeof(bl_enable), bl_enable},
};

static struct dsi_cmd_desc jdi_display_off_cmds[] = {
	{DTYPE_DCS_WRITE1, 0, 100, WAIT_TYPE_US,
		sizeof(bl_level_0), bl_level_0},		
	{DTYPE_DCS_WRITE, 0, 200, WAIT_TYPE_MS,
		sizeof(all_pixels_off), all_pixels_off},	
	{DTYPE_DCS_WRITE, 0, 30, WAIT_TYPE_US,
		sizeof(display_off), display_off},
	{DTYPE_DCS_WRITE, 0, 120, WAIT_TYPE_MS,
		sizeof(enter_sleep), enter_sleep}
};

static struct dsi_cmd_desc jdi_get_chip_id_cmds[] = {
	{DTYPE_DCS_WRITE, 0, 120, WAIT_TYPE_MS,
		sizeof(get_chip_id), get_chip_id}
};

static struct k3_fb_panel_data jdi_panel_data;


/******************************************************************************/

/*y=pow(x,0.6),x=[0,255]*/
static u32 square_point_six(u32 x)
{
	unsigned long t = x * x * x;
	int i = 0, j = 255, k = 0;
	unsigned long t0 = 0;
	while (j - i > 1) {
		k = (i + j) / 2;
			t0 = k * k * k * k * k;
		if(t0 < t)
			i = k;
		else if (t0 > t)
			j = k;
		else
			return k;
	}
	return k;
}

static struct lcd_tuning_dev *p_tuning_dev = NULL;
static int cabc_mode = 1;	 /* allow application to set cabc mode to ui mode */

static int jdi_set_cabc(struct lcd_tuning_dev *ltd, enum tft_cabc cabc);

static ssize_t jdi_lcd_info_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	int ret = 0;
	struct k3_panel_info *pinfo = NULL;

	pinfo = jdi_panel_data.panel_info;
	sprintf(buf, "jdi_OTM1282B 4.7' CMD TFT %d x %d\n",
		pinfo->xres, pinfo->yres);
	ret = strlen(buf) + 1;

	return ret;
}

static ssize_t show_cabc_mode(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", cabc_mode);
}

static ssize_t store_cabc_mode(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	int ret = 0;
	unsigned long val = 0;

	ret = strict_strtoul(buf, 0, &val);
	if (ret)
		return ret;

	if(val == 1) {
		/* allow application to set cabc mode to ui mode */
		cabc_mode =1;
		jdi_set_cabc(p_tuning_dev, CABC_UI);
	} else if (val == 2) {
		/* force cabc mode to video mode */
		cabc_mode =2;
		jdi_set_cabc(p_tuning_dev, CABC_VID);
	}

	return sprintf(buf, "%d\n", cabc_mode);
}

static DEVICE_ATTR(lcd_info, S_IRUGO, jdi_lcd_info_show, NULL);
static DEVICE_ATTR(cabc_mode, 0644, show_cabc_mode, store_cabc_mode);

static struct attribute *jdi_attrs[] = {
	&dev_attr_lcd_info,
	&dev_attr_cabc_mode,
	NULL,
};

static struct attribute_group jdi_attr_group = {
	.attrs = jdi_attrs,
};

static int jdi_sysfs_init(struct platform_device *pdev)
{
	int ret;
	ret = sysfs_create_group(&pdev->dev.kobj, &jdi_attr_group);
	if (ret) {
		k3fb_loge("create sysfs file failed!\n");
		return ret;
	}
	return 0;
}

static void jdi_sysfs_deinit(struct platform_device *pdev)
{
	sysfs_remove_group(&pdev->dev.kobj, &jdi_attr_group);
}

static int jdi_set_gamma(struct lcd_tuning_dev *ltd, enum lcd_gamma gamma)
{
	int ret = 0;
	struct platform_device *pdev = NULL;
	struct k3_fb_data_type *k3fd = NULL;
	u32 edc_base = 0;

	BUG_ON(ltd == NULL);
	pdev = (struct platform_device *)(ltd->data);
	k3fd = (struct k3_fb_data_type *)platform_get_drvdata(pdev);
	BUG_ON(k3fd == NULL);

	edc_base = k3fd->edc_base;

#if 0
	switch (gamma) {
	case GAMMA25:
		mipi_dsi_cmds_tx(cmi_gamma25_cmds, \
			ARRAY_SIZE(cmi_gamma25_cmds), edc_base);
		break;
	case GAMMA22:
		mipi_dsi_cmds_tx(cmi_gamma22_cmds, \
			ARRAY_SIZE(cmi_gamma22_cmds), edc_base);
		break;
	default:
		ret = -1;
		break;
	}
#endif

	return ret;
}

static int jdi_set_cabc(struct lcd_tuning_dev *ltd, enum  tft_cabc cabc)
{
	int ret = 0;
	struct platform_device *pdev = NULL;
	struct k3_fb_data_type *k3fd = NULL;
	u32 edc_base = 0;

	BUG_ON(ltd == NULL);
	pdev = (struct platform_device *)(ltd->data);
	k3fd = (struct k3_fb_data_type *)platform_get_drvdata(pdev);
	BUG_ON(k3fd == NULL);

	edc_base = k3fd->edc_base;

	if(cabc_mode==2)
		cabc=CABC_VID;

#if 0
	switch (cabc) {
	case CABC_UI:
		cabcUiData3[1] = (k3fd->bl_level >> 4) & 0x0f;
		cabcUiData3[2] = k3fd->bl_level & 0x0f;
		mipi_dsi_cmds_tx(cmi_cabc_ui_cmds, \
			ARRAY_SIZE(cmi_cabc_ui_cmds), edc_base);
		isCabcVidMode = FALSE_MIPI;
		break;
	case CABC_VID:
		cabcVidData3[1] = (k3fd->bl_level >> 4) & 0x0f;
		cabcVidData3[2] = k3fd->bl_level & 0x0f;
		mipi_dsi_cmds_tx(cmi_cabc_vid_cmds, \
			ARRAY_SIZE(cmi_cabc_vid_cmds), edc_base);
		isCabcVidMode = TRUE_MIPI;
		break;
	case CABC_OFF:
		break;
	default:
		ret = -1;
		break;
	}
#endif

	return ret;
}

static struct lcd_tuning_ops sp_tuning_ops = {
	.set_gamma = jdi_set_gamma,
	.set_cabc = jdi_set_cabc,
};


/*******************************************************************************/

static int jdi_pwm_on(struct k3_fb_data_type *k3fd)
{
	BUG_ON(k3fd == NULL);

	/* backlight on */
	PWM_IOMUX_SET(&(k3fd->panel_info), NORMAL);
	PWM_GPIO_REQUEST(&(k3fd->panel_info));
	gpio_direction_input(k3fd->panel_info.gpio_pwm1);
	mdelay(1);
	pwm_set_backlight(k3fd->bl_level, &(k3fd->panel_info));

	return 0;
}

static int jdi_pwm_off(struct k3_fb_data_type *k3fd)
{
	BUG_ON(k3fd == NULL);

	pwm_set_backlight(0, &(k3fd->panel_info));
	gpio_direction_output(k3fd->panel_info.gpio_pwm0, 0);
	mdelay(1);
	gpio_direction_input(k3fd->panel_info.gpio_pwm1);
	mdelay(1);
	PWM_GPIO_FREE(&(k3fd->panel_info));
	PWM_IOMUX_SET(&(k3fd->panel_info), LOWPOWER);

	return 0;
}

static void jdi_disp_on(struct k3_fb_data_type *k3fd)
{
	u32 edc_base = 0;
	struct k3_panel_info *pinfo = NULL;

	BUG_ON(k3fd == NULL);
	edc_base = k3fd->edc_base;
	pinfo = &(k3fd->panel_info);

	LCD_IOMUX_SET(pinfo, NORMAL);
	LCD_GPIO_REQUEST(pinfo);

	gpio_direction_output(pinfo->gpio_power, 1);
	mdelay(3);
	gpio_direction_output(pinfo->gpio_reset, 1);
	mdelay(3);
	gpio_direction_output(pinfo->gpio_reset, 0);
	mdelay(10);
	gpio_direction_output(pinfo->gpio_reset, 1);	
	mdelay(120);

	mipi_dsi_cmds_tx(jdi_video_on_cmds, \
		ARRAY_SIZE(jdi_video_on_cmds), edc_base);
}

static void jdi_disp_off(struct k3_fb_data_type *k3fd)
{
	u32 edc_base = 0;
	struct k3_panel_info *pinfo = NULL;

	BUG_ON(k3fd == NULL);
	edc_base = k3fd->edc_base;
	pinfo = &(k3fd->panel_info);

	mipi_dsi_cmds_tx(jdi_display_off_cmds,
		ARRAY_SIZE(jdi_display_off_cmds), edc_base);

	gpio_direction_output(pinfo->gpio_power, 0);
	mdelay(1);
	gpio_direction_output(pinfo->gpio_reset, 0);
	mdelay(1);
	LCD_GPIO_FREE(pinfo);
	LCD_IOMUX_SET(pinfo, LOWPOWER);
	LCD_VCC_DISABLE(pinfo);
}

static int mipi_jdi_panel_on(struct platform_device *pdev)
{
	struct k3_fb_data_type *k3fd = NULL;
	struct k3_panel_info *pinfo = NULL;

	BUG_ON(pdev == NULL);
	
	k3fd = (struct k3_fb_data_type *)platform_get_drvdata(pdev);
	BUG_ON(k3fd == NULL);

	pinfo = &(k3fd->panel_info);
	if (pinfo->lcd_init_step == LCD_INIT_POWER_ON) {
		LCD_VCC_ENABLE(pinfo);
		pinfo->lcd_init_step = LCD_INIT_SEND_SEQUENCE;
		return 0;
	}

	if (!k3fd->panel_info.display_on) {
		/* lcd display on */
		jdi_disp_on(k3fd);
		k3fd->panel_info.display_on = true;
		if (k3fd->panel_info.bl_set_type & BL_SET_BY_PWM) {
			/* backlight on */
			jdi_pwm_on(k3fd);
		}
	}

	return 0;
}

static int mipi_jdi_panel_off(struct platform_device *pdev)
{
	struct k3_fb_data_type *k3fd = NULL;

	BUG_ON(pdev == NULL);

	k3fd = (struct k3_fb_data_type *)platform_get_drvdata(pdev);
	BUG_ON(k3fd == NULL);

	if (k3fd->panel_info.display_on) {
		k3fd->panel_info.display_on = false;
		if (k3fd->panel_info.bl_set_type & BL_SET_BY_PWM) {
			/* backlight off */
			jdi_pwm_off(k3fd);
		}
		/* lcd display off */
		jdi_disp_off(k3fd);
	}

	return 0;
}

static int mipi_jdi_panel_remove(struct platform_device *pdev)
{
	struct k3_fb_data_type *k3fd = NULL;

	BUG_ON(pdev == NULL);

	k3fd = (struct k3_fb_data_type *)platform_get_drvdata(pdev);
	/*BUG_ON(k3fd == NULL);*/
	if (!k3fd) {
		return 0;
	}

	if (k3fd->panel_info.bl_set_type & BL_SET_BY_PWM) {
		PWM_CLK_PUT(&(k3fd->panel_info));
	}
	LCD_VCC_PUT(&(k3fd->panel_info));

	jdi_sysfs_deinit(pdev);

	return 0;
}

static int mipi_jdi_panel_set_backlight(struct platform_device *pdev)
{
	struct k3_fb_data_type *k3fd = NULL;
	u32 edc_base = 0;
	int level = 0;

	BUG_ON(pdev == NULL);
	k3fd = (struct k3_fb_data_type *)platform_get_drvdata(pdev);
	BUG_ON(k3fd == NULL);
	edc_base = k3fd->edc_base;

#if 0
	/*Our eyes are more sensitive to small brightness.
	So we adjust the brightness of lcd following iphone4 */
	level = (k3fd->bl_level * pow_point_six(k3fd->bl_level) * 100) / 2779;  //Y=(X/255)^1.6*255
	if (level > 255)
		level = 255;

	if (k3fd->panel_info.bl_set_type & BL_SET_BY_PWM) {
		return pwm_set_backlight(level, &(k3fd->panel_info));
	} else {
		outp32(edc_base + MIPIDSI_GEN_HDR_OFFSET,
			(level << 16) | (0x51 << 8) | 0x15);
	}
#endif

	return 0;

}

static int mipi_jdi_panel_set_fastboot(struct platform_device *pdev)
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

static int mipi_jdi_panel_set_cabc(struct platform_device *pdev, int value)
{
	u32 edc_base = 0;
	struct k3_fb_data_type *k3fd = NULL;

	BUG_ON(pdev == NULL);
	k3fd = (struct k3_fb_data_type *)platform_get_drvdata(pdev);
	BUG_ON(k3fd == NULL);
	edc_base = k3fd->edc_base;

#if 0
	if (value) {
		outp32(edc_base + MIPIDSI_GEN_HDR_OFFSET, 0x0dbb23);
	} else {
		outp32(edc_base + MIPIDSI_GEN_HDR_OFFSET, 0x0cbb23);
	}
#endif

	return 0;
}

static struct k3_panel_info jdi_panel_info = {0};
static struct k3_fb_panel_data jdi_panel_data = {
	.panel_info = &jdi_panel_info,
	.on = mipi_jdi_panel_on,
	.off = mipi_jdi_panel_off,
	.remove = mipi_jdi_panel_remove,
	.set_backlight = mipi_jdi_panel_set_backlight,
	.set_fastboot = mipi_jdi_panel_set_fastboot,
	.set_cabc = mipi_jdi_panel_set_cabc,
};

static int __devinit jdi_probe(struct platform_device *pdev)
{
	struct k3_panel_info *pinfo = NULL;
	struct resource *res = NULL;
	struct platform_device *reg_pdev;
	struct lcd_tuning_dev *ltd;
	struct lcd_properities lcd_props;

	pinfo = jdi_panel_data.panel_info;
	/* init lcd panel info */
	pinfo->display_on = false;
	pinfo->xres = 720;
	pinfo->yres = 1280;
	pinfo->type = PANEL_MIPI_CMD;
	pinfo->orientation = LCD_PORTRAIT;
	pinfo->bpp = EDC_OUT_RGB_888;
	pinfo->s3d_frm = EDC_FRM_FMT_2D;
	pinfo->bgr_fmt = EDC_RGB;
	pinfo->bl_set_type = BL_SET_BY_MIPI;
	pinfo->bl_max = PWM_LEVEL;
	pinfo->bl_min = 1;

	pinfo->frc_enable = 0;
	pinfo->esd_enable = 1;
	pinfo->sbl_enable = 0;

	pinfo->sbl.bl_max = 0xff;
	pinfo->sbl.cal_a = 0x0f;
	pinfo->sbl.str_limit = 0x40;

	pinfo->ldi.h_back_porch = 16;
	pinfo->ldi.h_front_porch = 16;
	pinfo->ldi.h_pulse_width = 2;
	pinfo->ldi.v_back_porch = 2;
	pinfo->ldi.v_front_porch = 2;
	pinfo->ldi.v_pulse_width = 2;

	pinfo->ldi.hsync_plr = 1;
	pinfo->ldi.vsync_plr = 0;
	pinfo->ldi.pixelclk_plr = 1;
	pinfo->ldi.data_en_plr = 0;

	pinfo->ldi.disp_mode = LDI_DISP_MODE_NOT_3D_FBF;

	/* Note: must init here */
	pinfo->frame_rate = 60;
	pinfo->clk_rate = 76000000;

	pinfo->mipi.lane_nums = DSI_4_LANES;
	pinfo->mipi.color_mode = DSI_24BITS_1;
	pinfo->mipi.vc = 0;
	pinfo->mipi.dsi_bit_clk = 260;

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
	if (platform_device_add_data(pdev, &jdi_panel_data,
		sizeof(struct k3_fb_panel_data))) {
		k3fb_loge("platform_device_add_data failed!\n");
		platform_device_put(pdev);
		return -ENOMEM;
	}

	reg_pdev = k3_fb_add_device(pdev);

	/* for cabc */
	lcd_props.type = TFT;
	lcd_props.default_gamma = GAMMA25;
	ltd = lcd_tuning_dev_register(&lcd_props, &sp_tuning_ops, (void *)reg_pdev);
	p_tuning_dev=ltd;
	if (IS_ERR(ltd)) {
		k3fb_loge("lcd_tuning_dev_register failed!\n");
		return -1;
	}

	jdi_sysfs_init(pdev);

	return 0;
}

static struct platform_driver this_driver = {
	.probe = jdi_probe,
	.remove = NULL,
	.suspend = NULL,
	.resume = NULL,
	.shutdown = NULL,
	.driver = {
		.name = "mipi_jdi_OTM1282B",
	},
};

static int __init mipi_jdi_panel_init(void)
{
	int ret = 0;

	ret = platform_driver_register(&this_driver);
	if (ret) {
		k3fb_loge("not able to register the driver\n");
		return ret;
	}

	return ret;
}

module_init(mipi_jdi_panel_init);
