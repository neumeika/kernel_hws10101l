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

#ifndef K3_FB_PANEL_H
#define K3_FB_PANEL_H

#include <linux/regulator/consumer.h>
#include <linux/mux.h>
#include "clock.h"

#include "k3_fb_def.h"

/* panel type list */
#define PANEL_NO			0xffff	/* No Panel */
#define PANEL_LDI			1	/* internal LCDC type */
#define PANEL_HDMI		2	/* HDMI TV */
#define PANEL_MIPI_VIDEO	3	/* MIPI */
#define PANEL_MIPI_CMD	4	/* MIPI */
#define PANEL_DP			5	/* Display Port */
#define PANEL_MIPI2RGB	6	/* MIPI to RGB */

/* device name */
#define DEV_NAME_LDI				"ldi"
#define DEV_NAME_HDMI			"hdmi"
#define DEV_NAME_DP				"displayport"
#define DEV_NAME_MIPI2RGB		"mipi2rgb"
#define DEV_NAME_MIPIDSI			"mipi_dsi"
#define DEV_NAME_FB				"k3_fb"
#define DEV_NAME_LCD_BKL		"lcd_backlight0"

/* LCD init step */
enum {
	LCD_INIT_POWER_ON = 0,
	LCD_INIT_SEND_SEQUENCE,
};

typedef struct panel_id_s {
	u32 id;
	u32 type;
} panel_id_type;

struct ldi_panel_info {
	u32 h_back_porch;
	u32 h_front_porch;
	u32 h_pulse_width;
	u32 v_back_porch;
	u32 v_front_porch;
	u32 v_pulse_width;

	u8 hsync_plr;
	u8 vsync_plr;
	u8 pixelclk_plr;
	u8 data_en_plr;
	u8 disp_mode;
};

/* DSI PHY configuration */
struct mipi_dsi_phy_ctrl {
	u32 burst_mode;
	u32 lane_byte_clk;
	u32 clk_division;
	u32 lp2hs_time;
	u32 hs2lp_time;
	u32 hsfreqrange;
	u32 pll_unlocking_filter;
	u32 cp_current;
	u32 lpf_ctrl;
	u32 n_pll;
	u32 m_pll_1;
	u32 m_pll_2;
	u32 factors_effective;
};

struct mipi_panel_info {
	u8 vc;
	u8 lane_nums;
	u8 color_mode;
	u32 dsi_bit_clk; /* clock lane(p/n) */
	/*u32 dsi_pclk_rate;*/
};

struct sbl_panel_info{
	u32 bl_max;
	u32 cal_a;
	u32 cal_b;
	u32 str_limit;
};

struct k3_panel_info {
	u32 type;
	u32 xres;
	u32 yres;
	u32 width;
	u32 height;
	u8 bpp;
	u8 orientation;
	u8 s3d_frm;
	u8 bgr_fmt;
	u8 frame_rate;
	u8 bl_set_type;
	u32 clk_rate;

	volatile bool display_on;
	u8 lcd_init_step;

	u8 sbl_enable;
	u8 frc_enable;
	u8 esd_enable;

	struct ldi_panel_info ldi;
	struct mipi_panel_info mipi;
	struct sbl_panel_info sbl;

#if defined(CONFIG_LCD_PANASONIC_VVX10F002A00) \
 || defined(CONFIG_LCD_QIMEI_HJ101IA_01F_10) \
 || defined(CONFIG_LCD_BOE_EK79016A)
	u32 gpio_avdd;
	u32 gpio_led_en;
#endif
	u32 gpio_reset;
	u32 gpio_power;
	u32 gpio_lcd_id0;
	u32 gpio_lcd_id1;
	struct regulator *lcdio_vcc;
	struct regulator *lcdanalog_vcc;
	struct iomux_block *lcd_block;
	struct block_config *lcd_block_config;

	u32 bl_max;
	u32 bl_min;
	u32 gpio_pwm0;
	u32 gpio_pwm1;
	u32 pwm_base;
	struct clk *pwm_clk;
	struct iomux_block *pwm_block;
	struct block_config *pwm_block_config;
};

struct k3_fb_data_type;

struct k3_fb_panel_data {
	struct k3_panel_info *panel_info;

	/* function entry chain */
	int (*on) (struct platform_device *pdev);
	int (*off) (struct platform_device *pdev);
	int (*remove) (struct platform_device *pdev);
	int (*set_backlight) (struct platform_device *pdev);
	int (*set_timing) (struct platform_device *pdev);
	int (*set_fastboot) (struct platform_device *pdev);
	int (*set_frc)(struct platform_device *pdev, int target_fps);
	int (*set_cabc)(struct platform_device *pdev, int value);
	int (*check_esd) (struct platform_device *pdev);
	struct platform_device *next;
};


/*******************************************************************************
** FUNCTIONS PROTOTYPES
*/

void set_reg(u32 addr, u32 val, u8 bw, u8 bs);

struct platform_device *k3_fb_device_alloc(struct k3_fb_panel_data *pdata,
	u32 type, u32 index, u32 *graphic_ch);
void k3_fb_device_free(struct platform_device *pdev);

int panel_next_on(struct platform_device *pdev);
int panel_next_off(struct platform_device *pdev);
int panel_next_remove(struct platform_device *pdev);
int panel_next_set_backlight(struct platform_device *pdev);
int panel_next_set_timing(struct platform_device *pdev);
int panel_next_set_frc(struct platform_device *pdev, int target_fps);
int panel_next_check_esd(struct platform_device *pdev);
int pwm_set_backlight(int bl_lvl, struct k3_panel_info *pinfo);


/******************************************************************************/

#define CLK_PWM0_NAME	"clk_pwm0"
#define CLK_MIPI_DPHY0_NAME	"clk_dphy0"
#define CLK_MIPI_DPHY1_NAME	"clk_dphy2"
#define CLK_LDI0_NAME	"clk_ldi0"
#define CLK_LDI1_NAME	"clk_ldi1"
#define CLK_EDC0_NAME	"clk_edc0"
#define CLK_EDC0_RST_NAME	"clk_edc0_rst"
#define CLK_EDC1_NAME	"clk_edc1"
#define CLK_G2D_NAME	"clk_g2d"

#define IRQ_LDI0_NAME	"irq_ldi0"
#define IRQ_LDI1_NAME	"irq_ldi1"
#define IRQ_EDC0_NAME	"irq_edc0"
#define IRQ_EDC1_NAME	"irq_edc1"

#define REG_BASE_PWM0_NAME	"reg_base_pwm0"
#define REG_BASE_EDC0_NAME	"reg_base_edc0"
#define REG_BASE_EDC1_NAME	"reg_base_edc1"

#define VCC_EDC1_NAME		"vcc_edc1"
#define VCC_LCDIO_NAME		"lcdio-vcc"
#define VCC_LCDANALOG_NAME	"lcdanalog-vcc"

#if defined(CONFIG_LCD_PANASONIC_VVX10F002A00) \
 || defined(CONFIG_LCD_QIMEI_HJ101IA_01F_10) \
 || defined(CONFIG_LCD_BOE_EK79016A)
#define GPIO_LCD_AVDD_NAME	     "gpio_lcd_avdd"
#define GPIO_LED_EN_NAME	     "gpio_led_enable"
#else
#define GPIO_LCD_RESET_NAME	"gpio_lcd_reset"
#define GPIO_LCD_POWER_NAME	"gpio_lcd_power"
#endif

#if defined(CONFIG_LCD_QIMEI_HJ101IA_01F_10)
#define GPIO_CM_NAME			 "gpio_cm"
#define GPIO_CABC_NAME			 "gpio_cabc"
#endif

#if defined(CONFIG_LCD_BOE_EK79016A)
#undef  GPIO_CABC0_NAME
#define GPIO_CABC0_NAME     "gpio_cabc0"
#undef  GPIO_CABC1_NAME
#define GPIO_CABC1_NAME     "gpio_cabc1"
#endif

#define GPIO_LCD_PWM0_NAME	"gpio_pwm0"
#define GPIO_LCD_PWM1_NAME	"gpio_pwm1"
#define GPIO_LCD_ID0_NAME	"gpio_lcd_id0"
#define GPIO_LCD_ID1_NAME	"gpio_lcd_id1"

#define IOMUX_LCD_NAME	"block_lcd"
#define IOMUX_PWM_NAME	"block_pwm"

//#define DEFAULT_PWM_CLK_RATE	(13 * 1000000)
#define DEFAULT_PWM_CLK_RATE	(32000)


int LCD_VCC_GET(struct platform_device *pdev, struct k3_panel_info *pinfo);
void LCDIO_SET_VOLTAGE(struct k3_panel_info *pinfo, u32 min_uV, u32 max_uV);
void LCD_VCC_PUT(struct k3_panel_info *pinfo);
void LCD_VCC_ENABLE(struct k3_panel_info *pinfo);
void LCD_VCC_DISABLE(struct k3_panel_info *pinfo);

int LCD_IOMUX_GET(struct k3_panel_info *pinfo);
void LCD_IOMUX_SET(struct k3_panel_info *pinfo, int mode);

int LCD_RESOURCE(struct platform_device *pdev, struct k3_panel_info *pinfo, 
	struct resource *res);

void LCD_GPIO_REQUEST(struct k3_panel_info *pinfo);
void 	LCD_GPIO_FREE(struct k3_panel_info *pinfo);

int LCD_GET_CLK_RATE(struct k3_panel_info *pinfo);

int PWM_CLK_GET(struct k3_panel_info *pinfo);
void PWM_CLK_PUT(struct k3_panel_info *pinfo);

int PWM_IOMUX_GET(struct k3_panel_info *pinfo);
void PWM_IOMUX_SET(struct k3_panel_info *pinfo, int mode);

int PWM_RESOUTCE(struct platform_device *pdev, struct k3_panel_info *pinfo, 
	struct resource *res);

void PWM_GPIO_REQUEST(struct k3_panel_info *pinfo);
void PWM_GPIO_FREE(struct k3_panel_info *pinfo);


enum LCD_TYPE {
    PANASONIC_LCD   = 0,
    TRULY_LCD    	= 1,
};
int get_lcd_type(void);

#endif /* K3_FB_PANEL_H */
