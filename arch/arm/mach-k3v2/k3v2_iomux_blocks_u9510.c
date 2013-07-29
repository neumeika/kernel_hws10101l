#include "iomux.h"
#include "k3v2_iomux_pins.h"
#include "k3v2_iomux_blocks.h"

/*gpio for  vbusdrv pins :GPIO_082*/
struct  iomux_pin *vbusdrv_pins[] = {
	&r26, NULL,
};

/*gpio for  vbusdrv pins for cs board :GPIO_082*/
struct  iomux_pin *vbusdrv_cs_pins[] = {
	&r26_cs, NULL,
};



IOMUX_BLOCK(block_vbusdrv, "block_vbusdrv", NORMAL, vbusdrv_pins)
IOMUX_BLOCK(block_vbusdrv_cs, "block_vbusdrv", NORMAL, vbusdrv_cs_pins)

/*hdmi pins:
 *HDMI_SCL, HDMI_SDA, HDMI_CEC, HDMI_HPD
 */
enum pull_updown hdmi_pullud_array2_u9510[] = {NOPULL, NOPULL, PULLDOWN, PULLDOWN, -INVALID,};
struct block_config hdmi_config_u9510[] = {
	[NORMAL] = {hdmi_func_normal, hdmi_pullud_array1, hdmi_drv_array1},
	[GPIO] = {hdmi_func_gpio, hdmi_pullud_array2_u9510, hdmi_drv_array1},
	[LOWPOWER] = {hdmi_func_gpio, hdmi_pullud_array2_u9510, hdmi_drv_array1},
};

/*kpc key pad  3X3:keypad_in0~keypad_in2, keypad_out0~keypad_out2*/
enum pull_updown kpc_pullud_array2_u9510[] = {PULLUP, PULLUP, PULLDOWN, \
	PULLDOWN, PULLDOWN, PULLDOWN, -INVALID,};
struct block_config kpc_config_u9510[] = {
	[NORMAL] = {kpc_func_array2, kpc_pullud_array2_u9510, kpc_drv_array1},
	[GPIO] = {kpc_func_array2, kpc_pullud_array1, kpc_drv_array1},
	[LOWPOWER] = {kpc_func_array2, kpc_pullud_array2_u9510, kpc_drv_array1},
};

/*gpio for  vbusdrv pins :GPIO_082*/
enum lowlayer_func vbusdrv_func_array1[] = {FUNC1, -INVALID,};
enum lowlayer_func vbusdrv_func_array2[] = {FUNC0, -INVALID,};
enum pull_updown vbusdrv_pullud_array1[] = {PULLUP,  -INVALID,};
enum pull_updown vbusdrv_pullud_array2[] = {NOPULL,  -INVALID,};
enum drive_strength vbusdrv_drv_array1[] = {LEVEL2,  -INVALID,};
struct block_config vbusdrv_config[] = {
	[NORMAL] = {vbusdrv_func_array1, vbusdrv_pullud_array1, vbusdrv_drv_array1},
	[GPIO] = {vbusdrv_func_array2, vbusdrv_pullud_array2, vbusdrv_drv_array1},
	[LOWPOWER] = {vbusdrv_func_array2, vbusdrv_pullud_array2, vbusdrv_drv_array1},
};

/*gpio for  vbusdrv pins for cs :GPIO_082*/
enum lowlayer_func vbusdrv_cs_func_array1[] = {FUNC1, -INVALID,};
enum lowlayer_func vbusdrv_cs_func_array2[] = {FUNC0, -INVALID,};
enum pull_updown vbusdrv_cs_pullud_array1[] = {PULLUP,  -INVALID,};
enum pull_updown vbusdrv_cs_pullud_array2[] = {NOPULL,  -INVALID,};
enum drive_strength vbusdrv_cs_drv_array1[] = {LEVEL2,  -INVALID,};
struct block_config vbusdrv_cs_config[] = {
	[NORMAL] = {vbusdrv_cs_func_array1, vbusdrv_cs_pullud_array1, vbusdrv_cs_drv_array1},
	[GPIO] = {vbusdrv_cs_func_array2, vbusdrv_cs_pullud_array2, vbusdrv_cs_drv_array1},
	[LOWPOWER] = {vbusdrv_cs_func_array2, vbusdrv_cs_pullud_array2, vbusdrv_cs_drv_array1},
};


struct block_table block_config_phone_u9510_es[] = {
	BLOCK_CONFIG("block_i2c0", &block_i2c0, i2c0_config)
	BLOCK_CONFIG("block_i2c1", &block_i2c1, i2c1_config)
	BLOCK_CONFIG("block_i2c2", &block_i2c2, i2c2_config)
	BLOCK_CONFIG("block_i2c3", &block_i2c3, i2c3_config)
	BLOCK_CONFIG("block_spi0", &block_spi0, spi0_config)
	BLOCK_CONFIG("block_spi1", &block_spi1, spi1_config)
	BLOCK_CONFIG("block_uart0", &block_uart0, uart0_config)
	BLOCK_CONFIG("block_uart1", &block_uart1, uart1_config)
	BLOCK_CONFIG("block_uart2", &block_uart2, uart2_config)
	BLOCK_CONFIG("block_uart3", &block_uart3, uart3_config)
	BLOCK_CONFIG("block_uart4", &block_uart4, uart4_config)
	BLOCK_CONFIG("block_kpc", &block_kpc, kpc_config_u9510)
	BLOCK_CONFIG("block_emmc", &block_emmc, emmc_config)
	BLOCK_CONFIG("block_sd", &block_sd, sd_config)
	BLOCK_CONFIG("block_nand", &block_nand, nand_config)
	BLOCK_CONFIG("block_sdio", &block_sdio, sdio_config)
	BLOCK_CONFIG("block_btpm", &block_btpm, btpm_config)
	BLOCK_CONFIG("block_btpwr", &block_btpwr, btpwr_config)
	BLOCK_CONFIG("block_gps_cellguide", &block_gps_cellguide, gps_cellguide_config)
	BLOCK_CONFIG("block_gps_boardcom", &block_gps_boardcom, gps_boardcom_config)
	BLOCK_CONFIG("block_ts", &block_ts, ts_config)
	BLOCK_CONFIG("block_lcd", &block_lcd, lcd_config)
	BLOCK_CONFIG("block_vbusdrv", &block_vbusdrv, vbusdrv_config)
	BLOCK_CONFIG("block_pwm", &block_pwm, pwm_config)
	BLOCK_CONFIG("block_hdmi", &block_hdmi, hdmi_config_u9510)
	BLOCK_CONFIG("block_wifi", &block_wifi, wifi_config)
	/*the following blocks are defined for camera*/
	BLOCK_CONFIG("block_isp_dvp", &block_isp_dvp, isp_dvp_config)
	BLOCK_CONFIG("block_isp_i2c", &block_isp_i2c, isp_i2c_config)
	BLOCK_CONFIG("block_isp_reset", &block_isp_reset, isp_reset_config)
	BLOCK_CONFIG("block_isp", &block_isp, isp_config)
	BLOCK_CONFIG("block_isp_flash", &block_isp_flash, isp_flash_config)
	BLOCK_CONFIG("block_charger", &block_charger, charger_config)
	BLOCK_CONFIG("block_gsensor", &block_gsensor, gsensor_config)
	BLOCK_CONFIG("block_audio_es305", &block_audio_es305, audio_es305_config)
	BLOCK_CONFIG("block_audio_spk", &block_audio_spk, audio_spk_config)
	BLOCK_CONFIG("block_audio_eph", &block_audio_eph, audio_eph_config)
	BLOCK_CONFIG("block_modem", &block_modem, modem_config)
	{NULL, NULL, NULL},
	/*
	 *TODO:
	 *there are only several blocks table defined here
	 */
};

struct block_table block_config_phone_u9510_cs[] = {
	BLOCK_CONFIG("block_i2c0", &block_i2c0, i2c0_config)
	BLOCK_CONFIG("block_i2c1", &block_i2c1, i2c1_config)
	BLOCK_CONFIG("block_i2c2", &block_i2c2_cs, i2c2_cs_config)
	BLOCK_CONFIG("block_i2c3", &block_i2c3, i2c3_config)
	BLOCK_CONFIG("block_spi0_cs", &block_spi0_cs, spi0_cs_config)
	BLOCK_CONFIG("block_spi1", &block_spi1_cs, spi1_cs_config)
	BLOCK_CONFIG("block_uart0", &block_uart0, uart0_config)
	BLOCK_CONFIG("block_uart1", &block_uart1, uart1_config)
	BLOCK_CONFIG("block_uart2", &block_uart2, uart2_config)
	BLOCK_CONFIG("block_uart3", &block_uart3, uart3_config)
	BLOCK_CONFIG("block_uart4", &block_uart4, uart4_config)
	BLOCK_CONFIG("block_kpc", &block_kpc, kpc_config_u9510)
	BLOCK_CONFIG("block_emmc", &block_emmc, emmc_config)
	BLOCK_CONFIG("block_sd", &block_sd, sd_config)
	BLOCK_CONFIG("block_nand", &block_nand, nand_config)
	BLOCK_CONFIG("block_sdio", &block_sdio, sdio_config)
	BLOCK_CONFIG("block_btpm", &block_btpm_cs, btpm_cs_config)
	BLOCK_CONFIG("block_btpwr", &block_btpwr_cs, btpwr_cs_config)
	BLOCK_CONFIG("block_gps_cellguide", &block_gps_cellguide, gps_cellguide_config)
	BLOCK_CONFIG("block_gps_boardcom", &block_gps_boardcom, gps_boardcom_config)
	BLOCK_CONFIG("block_ts", &block_ts, ts_config)
	BLOCK_CONFIG("block_lcd", &block_lcd, lcd_config)
	BLOCK_CONFIG("block_vbusdrv", &block_vbusdrv_cs, vbusdrv_cs_config)
	BLOCK_CONFIG("block_pwm", &block_pwm, pwm_config)
	BLOCK_CONFIG("block_hdmi", &block_hdmi, hdmi_config_u9510)
	BLOCK_CONFIG("block_wifi", &block_wifi, wifi_config)
	/*the following blocks are defined for camera*/
	BLOCK_CONFIG("block_isp_dvp", &block_isp_dvp, isp_dvp_config)
	BLOCK_CONFIG("block_isp_i2c", &block_isp_i2c, isp_i2c_config)
	BLOCK_CONFIG("block_isp_reset", &block_isp_reset, isp_reset_config)
	BLOCK_CONFIG("block_isp", &block_isp, isp_config)
	BLOCK_CONFIG("block_isp_flash", &block_isp_flash_cs, isp_flash_cs_config)
	BLOCK_CONFIG("block_charger", &block_charger, charger_config)
	BLOCK_CONFIG("block_gsensor", &block_gsensor, gsensor_config)
	BLOCK_CONFIG("block_audio_es305", &block_audio_es305, audio_es305_config)
	BLOCK_CONFIG("block_audio_spk", &block_audio_spk, audio_spk_config)
	BLOCK_CONFIG("block_audio_eph", &block_audio_eph, audio_eph_config)
	BLOCK_CONFIG("block_modem", &block_modem, modem_config)
	{NULL, NULL, NULL},
	/*
	 *TODO:
	 *there are only several blocks table defined here
	 */
};

