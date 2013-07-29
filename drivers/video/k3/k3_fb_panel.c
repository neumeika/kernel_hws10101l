/* Copyright (c) 2008-2011, Hisilicon Tech. Co., Ltd. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 *
 */

#include <linux/clk.h>
#include <linux/pwm.h>
#include <mach/gpio.h>

#include "k3_fb_panel.h"
#include "edc_overlay.h"


void set_reg(u32 addr, u32 val, u8 bw, u8 bs)
{
	u32 mask = (1 << bw) - 1;
	u32 tmp = inp32(addr);
	tmp &= ~(mask << bs);
	outp32(addr, tmp | ((val & mask) << bs));
}

int panel_next_on(struct platform_device *pdev)
{
	int ret = 0;
	struct k3_fb_panel_data *pdata = NULL;
	struct k3_fb_panel_data *next_pdata = NULL;
	struct platform_device *next_pdev = NULL;

	BUG_ON(pdev == NULL);

	pdata = (struct k3_fb_panel_data *)pdev->dev.platform_data;
	if (pdata) {
		next_pdev = pdata->next;
		if (next_pdev) {
			next_pdata = (struct k3_fb_panel_data *)next_pdev->dev.platform_data;
			if ((next_pdata) && (next_pdata->on))
				ret = next_pdata->on(next_pdev);
		}
	}

	return ret;
}

int panel_next_off(struct platform_device *pdev)
{
	int ret = 0;
	struct k3_fb_panel_data *pdata = NULL;
	struct k3_fb_panel_data *next_pdata = NULL;
	struct platform_device *next_pdev = NULL;

	BUG_ON(pdev == NULL);

	pdata = (struct k3_fb_panel_data *)pdev->dev.platform_data;
	if (pdata) {
		next_pdev = pdata->next;
		if (next_pdev) {
			next_pdata = (struct k3_fb_panel_data *)next_pdev->dev.platform_data;
			if ((next_pdata) && (next_pdata->off))
				ret = next_pdata->off(next_pdev);
		}
	}

	return ret;
}

int panel_next_remove(struct platform_device *pdev)
{
	int ret = 0;
	struct k3_fb_panel_data *pdata = NULL;
	struct k3_fb_panel_data *next_pdata = NULL;
	struct platform_device *next_pdev = NULL;

	BUG_ON(pdev == NULL);

	pdata = (struct k3_fb_panel_data *)pdev->dev.platform_data;
	if (pdata) {
		next_pdev = pdata->next;
		if (next_pdev) {
			next_pdata = (struct k3_fb_panel_data *)next_pdev->dev.platform_data;
			if ((next_pdata) && (next_pdata->remove))
				ret = next_pdata->remove(next_pdev);
		}
	}

	return ret;
}

int panel_next_set_backlight(struct platform_device *pdev)
{
	int ret = 0;
	struct k3_fb_panel_data *pdata = NULL;
	struct k3_fb_panel_data *next_pdata = NULL;
	struct platform_device *next_pdev = NULL;

	BUG_ON(pdev == NULL);

	pdata = (struct k3_fb_panel_data *)pdev->dev.platform_data;
	if (pdata) {
		next_pdev = pdata->next;
		if (next_pdev) {
			next_pdata = (struct k3_fb_panel_data *)next_pdev->dev.platform_data;
			if ((next_pdata) && (next_pdata->set_backlight))
				ret = next_pdata->set_backlight(next_pdev);
		}
	}

	return ret;
}

int panel_next_set_timing(struct platform_device *pdev)
{
	int ret = 0;
	struct k3_fb_panel_data *pdata = NULL;
	struct k3_fb_panel_data *next_pdata = NULL;
	struct platform_device *next_pdev = NULL;

	BUG_ON(pdev == NULL);

	pdata = (struct k3_fb_panel_data *)pdev->dev.platform_data;
	if (pdata) {
		next_pdev = pdata->next;
		if (next_pdev) {
			next_pdata = (struct k3_fb_panel_data *)next_pdev->dev.platform_data;
			if ((next_pdata) && (next_pdata->set_timing))
				ret = next_pdata->set_timing(next_pdev);
		}
	}

	return ret;
}

int panel_next_set_frc(struct platform_device *pdev, int target_fps)
{
	int ret = 0;
	struct k3_fb_panel_data *pdata = NULL;
	struct k3_fb_panel_data *next_pdata = NULL;
	struct platform_device *next_pdev = NULL;

	BUG_ON(pdev == NULL);

	pdata = (struct k3_fb_panel_data *)pdev->dev.platform_data;
	if (pdata) {
		next_pdev = pdata->next;
		if (next_pdev) {
			next_pdata = (struct k3_fb_panel_data *)next_pdev->dev.platform_data;
			if ((next_pdata) && (next_pdata->set_frc))
				ret = next_pdata->set_frc(next_pdev, target_fps);
		}
	}

	return ret;
}

int panel_next_check_esd(struct platform_device *pdev)
{
	int ret = 0;
	struct k3_fb_panel_data *pdata = NULL;
	struct k3_fb_panel_data *next_pdata = NULL;
	struct platform_device *next_pdev = NULL;

	BUG_ON(pdev == NULL);

	pdata = (struct k3_fb_panel_data *)pdev->dev.platform_data;
	if (pdata) {
		next_pdev = pdata->next;
		if (next_pdev) {
			next_pdata = (struct k3_fb_panel_data *)next_pdev->dev.platform_data;
			if ((next_pdata) && (next_pdata->check_esd))
				ret = next_pdata->check_esd(next_pdev);
		}
	}

	return ret;
}

struct platform_device *k3_fb_device_alloc(struct k3_fb_panel_data *pdata,
	u32 type, u32 index, u32 *graphic_ch)
{
	struct platform_device *this_dev = NULL;
	char dev_name[16] = {0};

	BUG_ON(pdata == NULL);

	switch (type) {
	case PANEL_HDMI:
		snprintf(dev_name, sizeof(dev_name), DEV_NAME_LDI);
		*graphic_ch = OVERLAY_PIPE_EDC1_CH1;
		break;
	case PANEL_DP:
		snprintf(dev_name, sizeof(dev_name), DEV_NAME_DP);
		*graphic_ch = OVERLAY_PIPE_EDC1_CH1;
		break;
	case PANEL_LDI:
		snprintf(dev_name, sizeof(dev_name), DEV_NAME_LDI);
		*graphic_ch = OVERLAY_PIPE_EDC0_CH2;
		break;
	case PANEL_MIPI_VIDEO:
	case PANEL_MIPI_CMD:
		snprintf(dev_name, sizeof(dev_name), DEV_NAME_MIPIDSI);
		*graphic_ch = OVERLAY_PIPE_EDC0_CH2;
		break;
	default:
		k3fb_loge("invalid panel type = %d!\n", type);
		return NULL;
	}

	pdata->next = NULL;

	this_dev = platform_device_alloc(dev_name, index + 1);
	if (this_dev) {
		if (platform_device_add_data(this_dev, pdata, sizeof(struct k3_fb_panel_data))) {
			k3fb_loge("failed to platform_device_add_data!\n");
			platform_device_put(this_dev);
			return NULL;
		}
	}

	return this_dev;
}

void  k3_fb_device_free(struct platform_device *pdev)
{
	BUG_ON(pdev == NULL);
	platform_device_put(pdev);
}


/******************************************************************************/

int LCD_VCC_GET(struct platform_device *pdev, struct k3_panel_info *pinfo)
{
	BUG_ON(pdev == NULL);
	BUG_ON(pinfo == NULL);

	pinfo->lcdio_vcc = regulator_get(&pdev->dev, VCC_LCDIO_NAME);
	if (IS_ERR(pinfo->lcdio_vcc)) {
		k3fb_loge("failed to get lcdio-vcc regulator!\n");
		return PTR_ERR((pinfo)->lcdio_vcc);
	}
	
#if !defined(CONFIG_LCD_PANASONIC_VVX10F002A00) && !defined(CONFIG_LCD_QIMEI_HJ101IA_01F_10)
	pinfo->lcdanalog_vcc = regulator_get(&pdev->dev, VCC_LCDANALOG_NAME);
	if (IS_ERR(pinfo->lcdanalog_vcc)) {
		k3fb_loge("failed to get lcdanalog-vcc regulator!\n");
		return PTR_ERR(pinfo->lcdanalog_vcc);
	}
#endif
	return 0;
}

void LCDIO_SET_VOLTAGE(struct k3_panel_info *pinfo, u32 min_uV, u32 max_uV)
{
	BUG_ON(pinfo == NULL);

	if (regulator_set_voltage(pinfo->lcdio_vcc, min_uV, max_uV) != 0) {
		k3fb_loge("failed to set lcdio vcc!\n");
	}
}

void LCD_VCC_PUT(struct k3_panel_info *pinfo)
{
	BUG_ON(pinfo == NULL);

	if (!IS_ERR(pinfo->lcdio_vcc)) {
		regulator_put(pinfo->lcdio_vcc);
	}
	
#if !defined(CONFIG_LCD_PANASONIC_VVX10F002A00) && !defined(CONFIG_LCD_QIMEI_HJ101IA_01F_10)
	if (!IS_ERR(pinfo->lcdanalog_vcc)) {
		regulator_put(pinfo->lcdanalog_vcc);
	}
#endif
}

void LCD_VCC_ENABLE(struct k3_panel_info *pinfo)
{
	BUG_ON(pinfo == NULL);

	if (!IS_ERR(pinfo->lcdio_vcc)) {
		if (regulator_enable(pinfo->lcdio_vcc) != 0) {
			k3fb_loge("failed to enable lcdio-vcc regulator!\n");
		}
	}
	
#if !defined(CONFIG_LCD_PANASONIC_VVX10F002A00) && !defined(CONFIG_LCD_QIMEI_HJ101IA_01F_10)
	if (!IS_ERR(pinfo->lcdanalog_vcc)) {
		if (regulator_enable(pinfo->lcdanalog_vcc) != 0) {
			k3fb_loge("failed to enable lcdanalog-vcc regulator!\n");
		}
	}
#endif
}

void LCD_VCC_DISABLE(struct k3_panel_info *pinfo)
{
	BUG_ON(pinfo == NULL);

#if !defined(CONFIG_LCD_PANASONIC_VVX10F002A00) && !defined(CONFIG_LCD_QIMEI_HJ101IA_01F_10)
	if (!IS_ERR(pinfo->lcdanalog_vcc)) {
		if (regulator_disable(pinfo->lcdanalog_vcc) != 0) {
			k3fb_loge("failed to disable lcdanalog-vcc regulator!\n");
		}
	}
#endif
	
	if (!IS_ERR(pinfo->lcdio_vcc)) {
		if (regulator_disable(pinfo->lcdio_vcc) != 0) {
			k3fb_loge("failed to disable lcdio-vcc regulator!\n");
		}
	}
}

int LCD_IOMUX_GET(struct k3_panel_info *pinfo)
{
	BUG_ON(pinfo == NULL);

	pinfo->lcd_block = iomux_get_block(IOMUX_LCD_NAME);
	if (!pinfo->lcd_block) {
		k3fb_loge("failed to get iomux_lcd!\n");
		return PTR_ERR(pinfo->lcd_block);
	}
	
	pinfo->lcd_block_config = iomux_get_blockconfig(IOMUX_LCD_NAME);
	if (!pinfo->lcd_block_config) {
		k3fb_loge("failed to get iomux_lcd config!\n");
		return PTR_ERR(pinfo->lcd_block_config);
	}

	return 0;
}

void LCD_IOMUX_SET(struct k3_panel_info *pinfo, int mode)
{
	BUG_ON(pinfo == NULL);

	if (blockmux_set(pinfo->lcd_block, pinfo->lcd_block_config, mode) != 0) {
		k3fb_loge("failed to set iomux_lcd normal mode!\n");
	}
}

#if defined(CONFIG_LCD_PANASONIC_VVX10F002A00) || defined(CONFIG_LCD_QIMEI_HJ101IA_01F_10)
int LCD_RESOURCE(struct platform_device *pdev, struct k3_panel_info *pinfo, 
	struct resource *res)
{
	BUG_ON(pdev == NULL);
	BUG_ON(pinfo == NULL);

	res = platform_get_resource_byname(pdev, IORESOURCE_IO, GPIO_LCD_AVDD_NAME);
	if (!res) {
		k3fb_loge("failed to get gpio avdd resource!\n");
		return -ENXIO;
	}
	
	pinfo->gpio_avdd = res->start;
	if (!gpio_is_valid(pinfo->gpio_avdd)) {
		k3fb_loge("failed to get gpio reset resource!\n");
		return -ENXIO;
	}
	
	res = platform_get_resource_byname(pdev, IORESOURCE_IO, GPIO_LED_EN_NAME);
	if (!res) {
		k3fb_loge("failed to get gpio led enable resource!\n");
		return -ENXIO;
	}
	
	pinfo->gpio_led_en = res->start;
	if (!gpio_is_valid(pinfo->gpio_led_en)) {
		k3fb_loge("failed to get gpio power resource!\n");
		return -ENXIO;
	}

	res = platform_get_resource_byname(pdev, IORESOURCE_IO, GPIO_LCD_ID0_NAME);
	if (!res) {
		k3fb_loge("failed to get gpio_lcd_id0 resource!\n");
		return -ENXIO;
	}
	
	pinfo->gpio_lcd_id0 = res->start;
	if (!(gpio_is_valid(pinfo->gpio_lcd_id0))) {
		k3fb_loge("gpio_lcd_id0 is invalid!\n");
		return -ENXIO;
	}
	
	res = platform_get_resource_byname(pdev, IORESOURCE_IO, GPIO_LCD_ID1_NAME);
	if (!res) {
		k3fb_loge("failed to get gpio_lcd_id1 resource.\n");
		return -ENXIO;
	}
	
	pinfo->gpio_lcd_id1 = res->start;
	if (!(gpio_is_valid(pinfo->gpio_lcd_id1))) {
		k3fb_loge("gpio_lcd_id1 is invalid!\n");
		return -ENXIO;
	}

	return 0;
}

void LCD_GPIO_REQUEST(struct k3_panel_info *pinfo)
{
	BUG_ON(pinfo == NULL);

	if (gpio_request(pinfo->gpio_led_en, GPIO_LED_EN_NAME) != 0) {
		k3fb_loge("failed to request gpio led enable!\n");
	}

	if (gpio_request(pinfo->gpio_avdd, GPIO_LCD_AVDD_NAME) != 0) {
		k3fb_loge("failed to request gpio lcd avdd!\n");
	}

	if (gpio_request(pinfo->gpio_lcd_id0, GPIO_LCD_ID0_NAME) != 0) {
		k3fb_loge("failed to request gpio_lcd_id0!\n");
	}

	if (gpio_request(pinfo->gpio_lcd_id1, GPIO_LCD_ID1_NAME) != 0) {
		k3fb_loge("failed to request gpio_lcd_id1!\n");
	}
}

void LCD_GPIO_FREE(struct k3_panel_info *pinfo)
{
	BUG_ON(pinfo == NULL);

	if (gpio_is_valid(pinfo->gpio_avdd)) {
		gpio_free(pinfo->gpio_avdd);
	}
	
	if (gpio_is_valid(pinfo->gpio_led_en)) {
		gpio_free(pinfo->gpio_led_en);
	}
	
	if (gpio_is_valid(pinfo->gpio_lcd_id0)) {
		gpio_free(pinfo->gpio_lcd_id0);
	}
	
	if (gpio_is_valid(pinfo->gpio_lcd_id1)) {
		gpio_free(pinfo->gpio_lcd_id1);
	}
}
#else
int LCD_RESOURCE(struct platform_device *pdev, struct k3_panel_info *pinfo, 
	struct resource *res)
{
	BUG_ON(pdev == NULL);
	BUG_ON(pinfo == NULL);

	res = platform_get_resource_byname(pdev, IORESOURCE_IO, GPIO_LCD_RESET_NAME);
	if (!res) {
		k3fb_loge("failed to get gpio reset resource!\n");
		return -ENXIO;
	}
	
	pinfo->gpio_reset = res->start;
	if (!gpio_is_valid(pinfo->gpio_reset)) {
		k3fb_loge("failed to get gpio reset resource!\n");
		return -ENXIO;
	}
	
	res = platform_get_resource_byname(pdev, IORESOURCE_IO, GPIO_LCD_POWER_NAME);
	if (!res) {
		k3fb_loge("failed to get gpio power resource!\n");
		return -ENXIO;
	}
	
	pinfo->gpio_power = res->start;
	if (!gpio_is_valid(pinfo->gpio_power)) {
		k3fb_loge("failed to get gpio power resource!\n");
		return -ENXIO;
	}

	res = platform_get_resource_byname(pdev, IORESOURCE_IO, GPIO_LCD_ID0_NAME);
	if (!res) {
		k3fb_loge("failed to get gpio_lcd_id0 resource!\n");
		return -ENXIO;
	}
	
	pinfo->gpio_lcd_id0 = res->start;
	if (!(gpio_is_valid(pinfo->gpio_lcd_id0))) {
		k3fb_loge("gpio_lcd_id0 is invalid!\n");
		return -ENXIO;
	}
	
	res = platform_get_resource_byname(pdev, IORESOURCE_IO, GPIO_LCD_ID1_NAME);
	if (!res) {
		k3fb_loge("failed to get gpio_lcd_id1 resource.\n");
		return -ENXIO;
	}
	
	pinfo->gpio_lcd_id1 = res->start;
	if (!(gpio_is_valid(pinfo->gpio_lcd_id1))) {
		k3fb_loge("gpio_lcd_id1 is invalid!\n");
		return -ENXIO;
	}

	return 0;
}

void LCD_GPIO_REQUEST(struct k3_panel_info *pinfo)
{
	BUG_ON(pinfo == NULL);

	if (gpio_request(pinfo->gpio_power, GPIO_LCD_POWER_NAME) != 0) {
		k3fb_loge("failed to request gpio power!\n");
	}

	if (gpio_request(pinfo->gpio_reset, GPIO_LCD_RESET_NAME) != 0) {
		k3fb_loge("failed to request gpio reset!\n");
	}

	if (gpio_request(pinfo->gpio_lcd_id0, GPIO_LCD_ID0_NAME) != 0) {
		k3fb_loge("failed to request gpio_lcd_id0!\n");
	}

	if (gpio_request(pinfo->gpio_lcd_id1, GPIO_LCD_ID1_NAME) != 0) {
		k3fb_loge("failed to request gpio_lcd_id1!\n");
	}
}

void LCD_GPIO_FREE(struct k3_panel_info *pinfo)
{
	BUG_ON(pinfo == NULL);

	if (gpio_is_valid(pinfo->gpio_reset)) {
		gpio_free(pinfo->gpio_reset);
	}
	
	if (gpio_is_valid(pinfo->gpio_power)) {
		gpio_free(pinfo->gpio_power);
	}
	
	if (gpio_is_valid(pinfo->gpio_lcd_id0)) {
		gpio_free(pinfo->gpio_lcd_id0);
	}
	
	if (gpio_is_valid(pinfo->gpio_lcd_id1)) {
		gpio_free(pinfo->gpio_lcd_id1);
	}
}
#endif
int PWM_CLK_GET(struct k3_panel_info *pinfo)
{
	BUG_ON(pinfo == NULL);

	pinfo->pwm_clk = clk_get(NULL, CLK_PWM0_NAME);
	if (IS_ERR(pinfo->pwm_clk)) {
		k3fb_loge("failed to get pwm0 clk!\n");
		return PTR_ERR(pinfo->pwm_clk);
	}
	
	if (clk_set_rate(pinfo->pwm_clk, DEFAULT_PWM_CLK_RATE) != 0) {
		k3fb_loge("failed to set pwm clk rate!\n");
	}

	return 0;
}

void PWM_CLK_PUT(struct k3_panel_info *pinfo)
{
	BUG_ON(pinfo == NULL);

	if (!IS_ERR(pinfo->pwm_clk)) {
		clk_put(pinfo->pwm_clk);
	}
}

int PWM_IOMUX_GET(struct k3_panel_info *pinfo)
{
	BUG_ON(pinfo == NULL);

	pinfo->pwm_block = iomux_get_block(IOMUX_PWM_NAME);
	if (!pinfo->pwm_block) {
		k3fb_loge("failed to get iomux_pwm!\n");
		return PTR_ERR(pinfo->pwm_block);
	}
	
	pinfo->pwm_block_config = iomux_get_blockconfig(IOMUX_PWM_NAME);
	if (!pinfo->pwm_block_config) {
		k3fb_loge("failed to get iomux_pwm config!\n");
		return PTR_ERR(pinfo->pwm_block_config);
	}

	return 0;
}

void PWM_IOMUX_SET(struct k3_panel_info *pinfo, int mode)
{
	BUG_ON(pinfo == NULL);

	if (blockmux_set(pinfo->pwm_block, pinfo->pwm_block_config, (mode)) != 0) {
		k3fb_loge("failed to set iomux pwm normal mode!\n");
	}
}

int PWM_RESOUTCE(struct platform_device *pdev, struct k3_panel_info *pinfo, 
	struct resource *res)
{
	BUG_ON(pdev == NULL);
	BUG_ON(pinfo == NULL);

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM,  REG_BASE_PWM0_NAME);
	if (!res) {
		k3fb_loge("failed to get pwm0 resource!\n");
		return -ENXIO;
	}
	
	pinfo->pwm_base = res->start;
	res = platform_get_resource_byname(pdev, IORESOURCE_IO, GPIO_LCD_PWM0_NAME);
	if (!res) {
		k3fb_loge("failed to get gpio pwm0 resource!\n");
		return -ENXIO;
	}
	
	pinfo->gpio_pwm0 = res->start;
	if (!(gpio_is_valid(pinfo->gpio_pwm0))) {
		k3fb_loge("gpio pwm0 is invalid!\n");
		return -ENXIO;
	}
	
	res = platform_get_resource_byname(pdev, IORESOURCE_IO, GPIO_LCD_PWM1_NAME);
	if (!res) {
		k3fb_loge("failed to get gpio pwm1 resource!\n");
		return -ENXIO;
	}
	
	pinfo->gpio_pwm1 = res->start;
	if (!(gpio_is_valid(pinfo->gpio_pwm1))) {
		k3fb_loge("gpio pwm1 is invalid!\n");
		return -ENXIO;
	}

	return 0;
}

void PWM_GPIO_REQUEST(struct k3_panel_info *pinfo)
{
	BUG_ON(pinfo == NULL);

	if (gpio_request(pinfo->gpio_pwm0, GPIO_LCD_PWM0_NAME) != 0) {
		k3fb_loge("failed to request gpio pwm0!\n");
	}
	
	if (gpio_request(pinfo->gpio_pwm1, GPIO_LCD_PWM1_NAME) != 0) {
		k3fb_loge("failed to request gpio pwm1!\n");
	}
}

void PWM_GPIO_FREE(struct k3_panel_info *pinfo)
{
	BUG_ON(pinfo == NULL);

	if (gpio_is_valid(pinfo->gpio_pwm0)) {
		gpio_free(pinfo->gpio_pwm0);
	}
	
	if (gpio_is_valid(pinfo->gpio_pwm1)) {
		gpio_free(pinfo->gpio_pwm1);
	}
}

int LCD_GET_CLK_RATE(struct k3_panel_info *pinfo)
{
	return 0;
}
