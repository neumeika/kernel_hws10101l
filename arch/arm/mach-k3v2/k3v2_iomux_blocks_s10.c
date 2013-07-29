#include "iomux.h"
#include "k3v2_iomux_pins.h"
#include "k3v2_iomux_blocks.h"
#include <mach/k3v2_s10_iomux_blocks_name.h>

#if defined(CONFIG_GPIO_I2C_ADPT)
struct  iomux_pin *gpio_i2c_pins[] = {
	&u2, &v1, NULL,
};
#endif

/////////////////////////////////////////////////////////////////////////////////////////////
/*spi0_cs:SPI0_CS0_N,SPI0_CS1_N,SPI0_CS2_N,SPI0_CS3_N*/
#if 0
struct  iomux_pin *spi0_cs_pins_s10[] = {
	&l29, &m21, &n28, &m23, NULL,
};
#else
struct  iomux_pin *spi0_cs_pins_s10[] = {
	&l29, &m21, NULL,
};
#endif

/*kpc key pad  3X3:keypad_in0~keypad_in2, keypad_out0~keypad_out2*/
#if 0
struct  iomux_pin *kpc_pins_s10[] = {
	&aj21, &ah21, &af22, &aj24, &ah24, &ah25, NULL,
};
#else
struct  iomux_pin *kpc_pins_s10[] = {
	&aj21, &ah21, NULL,
};
#endif

/*pcm*/

struct  iomux_pin *pcm_pins_s10_es[] = {
	&u26, &t28, &t26, NULL,
};

struct  iomux_pin *pcm_pins_s10_cs[] = {
	&u26_cs, &t28_cs, &t26_cs, NULL,
};

/*gps for cellguide*/
#if 0
struct  iomux_pin *gps_cellguide_pins_s10[] = {
	&m26, &l25, &j28, &l28, &k28, &k26, &m28, &m23, NULL,
};
#else
struct  iomux_pin *gps_cellguide_pins_s10[] = {
	&m26, &l25, &j28, &l28, &k28, &k26, &m28, NULL,
};
#endif

struct  iomux_pin *ts_pins_s10_es[] = {
	&aa23, &v29, &g19, NULL,
};

struct  iomux_pin *ts_pins_s10_cs[] = {
	&aa23, &v29, &g19_cs, NULL,
};


/*pwm pins:
 *PWM_OUT0, PWM_OUT1
 */
#if 0
struct  iomux_pin *pwm_pins_s10[] = {
	&d28, &b27, NULL,
};
#else
struct  iomux_pin *pwm_pins_s10[] = {
	&d28, NULL,
};
#endif

/*hdmi pins:
 *HDMI_SCL, HDMI_SDA, HDMI_CEC, HDMI_CEC
 */
#if 0
struct  iomux_pin *hdmi_pins_s10[] = {
	&e12, &d12, &e11, &d11, NULL,
};
#else
struct  iomux_pin *hdmi_pins_s10[] = {
	&e12, &d12, &d11, NULL,
};
#endif

/*charger pins:
 *EFUSE_PGM, ISP_GPIO5
 */
#if 0
struct  iomux_pin *charger_pins_s10[] = {
	&ac25, &a20, NULL,
};
#else
struct  iomux_pin *charger_pins_s10_es[] = {
//	&y4, &g28, &a20, &aa4, &u1, &ac25, &ab2, NULL,
	&y4, &a20, &aa4, &u1, &ac25, &ab2, NULL,
};

struct  iomux_pin *charger_pins_s10_cs[] = {
	&y4, &g28_cs, &a20, &aa4, &u1, &ac25, &ab2, NULL,
};

#if defined(CONFIG_SUPPORT_MICRO_USB_PORT)
/*GPIO145,GPIO116,GPIO11,GPIO21,GPIO82*/
struct iomux_pin  *micro_usb_pins_link[] = {
	&aa25, &e1, &w1, &aa2,&r26, NULL,			//wuxinxian
};
#endif

#if defined(CONFIG_FSA880_I2C)
struct  iomux_pin *bc11_pins_s10_es[] = {
	&ab5, &r26, &aa25, &aa2, &w1, &e1, NULL,
};

struct  iomux_pin *bc11_pins_s10_cs[] = {
	&ab5, &r26_cs, &aa25, &aa2, &w1, &e1, NULL,
};
#endif

struct  iomux_pin *boost_5v_pins_s10[] = {
	&ad26, NULL,
};


struct  iomux_pin *battery_gauge_pins_s10[] = {
	&g28, NULL,
};
#endif


/* af23->GPIO143_SDCARD_PWREN, a28->GPIO083_SD_DETECT*/
struct  iomux_pin *sd_ctrl_pins_s10_es[] = {
    &af23, &a28, NULL,
};

struct  iomux_pin *sd_ctrl_pins_s10_cs[] = {
    &af23, &a28_cs, NULL,
};

/* u26->GPIO079_AP_WAKEUP_MHL, t28->GPIO080_MHL_INT, t26->GPIO081_MHL_RST_N */
struct  iomux_pin *mhl_pins_s10_es[] = {
	&u26, &t28, &t26, NULL,
};

struct  iomux_pin *mhl_pins_s10_cs[] = {
	&u26_cs, &t28_cs, &t26_cs, NULL,
};

struct  iomux_pin *camera_pwr_pins_s10_es[] = {
	&g25, &h25, &v2, &w2, NULL,
};

struct  iomux_pin *camera_pwr_pins_s10_cs[] = {
	&g25_cs, &h25_cs, &v2, &w2, NULL,
};

struct  iomux_pin *main_camera_pins_s10[] = {
	&n28, NULL,
};

struct  iomux_pin *slave_camera_pins_s10[] = {
	&m23, NULL,
};

struct  iomux_pin *mr_sensor_pins_s10[] = {
	&aa28, NULL,
};

struct  iomux_pin *compass_pins_s10[] = {
	&ac28, NULL,
};

struct  iomux_pin *gyroscope_pins_s10[] = {
	&d15, &ab26, NULL,
};

struct  iomux_pin *capacitor_sensor_pins_s10[] = {
	&ae23, NULL,
};

struct  iomux_pin *sim_dect_pins_s10[] = {
	&ab4, NULL,
};

struct  iomux_pin *extral_dcdc_pins_s10[] = {
	&h25, NULL,
};

struct  iomux_pin *dock_pins_s10_es[] = {
	&h26, NULL,
};

struct  iomux_pin *dock_pins_s10_cs[] = {
	&h26_cs, NULL,
};

struct  iomux_pin *audio_earphone_pins_s10[] = {
	&b26, NULL,
};

#if 0
struct  iomux_pin *modem_ap_ctrl_pins_s10[] = {
	&d1, &f5, &e1, &e21, &d2, &l25, &e2, NULL,
};
#else
struct  iomux_pin *modem_ap_ctrl_pins_s10[] = {
	&g1, &f4, &f2, &f1, &g2, &d1_cs, &d2_cs, &e2,  NULL,
};
#endif

struct  iomux_pin *usb_switch_ctrl_pins_s10[] = {
	&f5,NULL,
};

struct  iomux_pin *minor_version_ctrl_pins_s10[] = {
	&u4, &u5, NULL,
};

#define IOMUX_BLOCK(_iomux_block, _block_name, _block_func, _pins)   \
struct iomux_block _iomux_block = {\
	.block_name  = _block_name,\
	.block_func   =  _block_func,\
	.pins = _pins,\
	.ops = &iomux_block_ops,\
	.init = 0, \
};

/*define blocks*/
#if defined(CONFIG_GPIO_I2C_ADPT)
IOMUX_BLOCK(block_gpio_i2c, IO_GPIO_I2C_BLOCK_NAME, NORMAL, gpio_i2c_pins)
#endif
IOMUX_BLOCK(block_spi0_cs_s10, IO_SPI0_CS_BLOCK_NAME, NORMAL, spi0_cs_pins_s10)  // "block_spi0_cs"
IOMUX_BLOCK(block_kpc_s10, IO_KPC_BLOCK_NAME, NORMAL, kpc_pins_s10)  // "block_kpc"
IOMUX_BLOCK(block_pcm_s10_es, IO_PCM_BLOCK_NAME, NORMAL, pcm_pins_s10_es)  // "block_pcm"
IOMUX_BLOCK(block_pcm_s10_cs, IO_PCM_BLOCK_NAME, NORMAL, pcm_pins_s10_cs)  // "block_pcm"
IOMUX_BLOCK(block_gps_cellguide_s10, IO_GPS_CELLGUIDE_BLOCK_NAME, NORMAL, gps_cellguide_pins_s10)  // "block_gps_cellguide"
IOMUX_BLOCK(block_ts_s10_es, IO_TS_BLOCK_NAME, NORMAL, ts_pins_s10_es)    // "block_ts"
IOMUX_BLOCK(block_ts_s10_cs, IO_TS_BLOCK_NAME, NORMAL, ts_pins_s10_cs)    // "block_ts"
IOMUX_BLOCK(block_pwm_s10, IO_PWM_BLOCK_NAME, NORMAL, pwm_pins_s10)  // "block_pwm"
IOMUX_BLOCK(block_hdmi_s10, IO_HDMI_BLOCK_NAME, NORMAL, hdmi_pins_s10)    // "block_hdmi"
IOMUX_BLOCK(block_charger_s10_es, IO_CHARGER_BLOCK_NAME, NORMAL, charger_pins_s10_es)  // "block_charger"
IOMUX_BLOCK(block_charger_s10_cs, IO_CHARGER_BLOCK_NAME, NORMAL, charger_pins_s10_cs)  // "block_charger"
#if defined(CONFIG_SUPPORT_MICRO_USB_PORT)
IOMUX_BLOCK(block_micro_usb_link, IO_MICRO_USB, NORMAL, micro_usb_pins_link)    // "block_gpio_micro_usb"
#endif

#if defined(CONFIG_FSA880_I2C)
IOMUX_BLOCK(block_bc11_s10_es, IO_BC11_BLOCK_NAME, NORMAL, bc11_pins_s10_es)    // "block_bc11"
IOMUX_BLOCK(block_bc11_s10_cs, IO_BC11_BLOCK_NAME, NORMAL, bc11_pins_s10_cs)    // "block_bc11"
#endif
IOMUX_BLOCK(block_boost_5v_s10, IO_BOOST_5V_BLOCK_NAME, NORMAL, boost_5v_pins_s10)    // "block_boost_5v"
IOMUX_BLOCK(block_battery_gauge_s10, IO_BATTERY_GAUGE_BLOCK_NAME, NORMAL, battery_gauge_pins_s10)  // "block_battery_gauge"
IOMUX_BLOCK(block_sd_ctrl_s10_es, IO_SD_CTRL_BLOCK_NAME, NORMAL, sd_ctrl_pins_s10_es)  // "block_sd_ctrl"
IOMUX_BLOCK(block_sd_ctrl_s10_cs, IO_SD_CTRL_BLOCK_NAME, NORMAL, sd_ctrl_pins_s10_cs)  // "block_sd_ctrl"
IOMUX_BLOCK(block_mhl_s10_es, IO_MHL_BLOCK_NAME, NORMAL, mhl_pins_s10_es)  // "block_mhl"
IOMUX_BLOCK(block_mhl_s10_cs, IO_MHL_BLOCK_NAME, NORMAL, mhl_pins_s10_cs)  // "block_mhl"
IOMUX_BLOCK(block_camera_pwr_s10_es, IO_CAMERA_PWR_BLOCK_NAME, NORMAL, camera_pwr_pins_s10_es)    // "block_camera_pwr"
IOMUX_BLOCK(block_camera_pwr_s10_cs, IO_CAMERA_PWR_BLOCK_NAME, NORMAL, camera_pwr_pins_s10_cs)    // "block_camera_pwr"
IOMUX_BLOCK(block_main_camera_s10, IO_MAIN_CAMERA_BLOCK_NAME, NORMAL, main_camera_pins_s10)  // "block_main_camera"
IOMUX_BLOCK(block_slave_camera_s10, IO_SLAVE_CAMERA_BLOCK_NAME, NORMAL, slave_camera_pins_s10)    // "block_slave_camera"
IOMUX_BLOCK(block_mr_sensor_s10, IO_MR_SENSOR_BLOCK_NAME, NORMAL, mr_sensor_pins_s10)  // "block_mr_sensor"
IOMUX_BLOCK(block_compass_s10, IO_COMPASS_BLOCK_NAME, NORMAL, compass_pins_s10)  // "block_compass"
IOMUX_BLOCK(block_gyroscope_s10, IO_GYROSCOPE_BLOCK_NAME, NORMAL, gyroscope_pins_s10)  // "block_gyroscope"
IOMUX_BLOCK(block_capacitor_sensor_s10, IO_CAPACITOR_SENSOR_BLOCK_NAME, NORMAL, capacitor_sensor_pins_s10)    // "block_capacitor_sensor"
IOMUX_BLOCK(block_sim_dect_s10, IO_SIM_DECT_BLOCK_NAME, NORMAL, sim_dect_pins_s10)    // "block_sim_dect"
IOMUX_BLOCK(block_extral_dcdc_s10, IO_EXTRAL_DCDC_BLOCK_NAME, NORMAL, extral_dcdc_pins_s10)  // "block_extral_dcdc"
IOMUX_BLOCK(block_dock_s10_es, IO_DOCK_BLOCK_NAME, NORMAL, dock_pins_s10_es)    // "block_dock"
IOMUX_BLOCK(block_dock_s10_cs, IO_DOCK_BLOCK_NAME, NORMAL, dock_pins_s10_cs)    // "block_dock"
IOMUX_BLOCK(block_audio_earphone_s10, IO_AUDIO_EARPHONE_BLOCK_NAME, NORMAL, audio_earphone_pins_s10)    // "block_audio_earphone"
IOMUX_BLOCK(block_moden_ap_ctrl_s10, IO_MODEM_AP_CTRL_BLOCK_NAME, NORMAL, modem_ap_ctrl_pins_s10)  // "block_moden_ap_ctrl"
IOMUX_BLOCK(block_minor_version_s10, IO_MINOR_VERSION_BLOCK_NAME, NORMAL, minor_version_ctrl_pins_s10)  // "block_minor_version"
IOMUX_BLOCK(block_usb_switch_ctrl_s10, IO_USB_SWITCH_CTRL_BLOCK_NAME, NORMAL, usb_switch_ctrl_pins_s10)  // "block_usb_switch_ctrl"


/*gpio_i2c:gpio_i2c_SCL,gpio_i2c_SDA*/
#if defined(CONFIG_GPIO_I2C_ADPT)
enum lowlayer_func gpio_i2c_func_array1[] = {FUNC1, FUNC1, -INVALID,};
enum lowlayer_func gpio_i2c_func_array2[] = {FUNC1, FUNC1, -INVALID,};
enum pull_updown gpio_i2c_pullud_array1[] = {PULLUP, PULLUP, -INVALID,};
enum drive_strength gpio_i2c_drv_array1[] = {LEVEL4, LEVEL4,  -INVALID,};
struct block_config gpio_i2c_config[] = {
	[NORMAL] = {gpio_i2c_func_array1, gpio_i2c_pullud_array1, gpio_i2c_drv_array1},
	[GPIO] = {gpio_i2c_func_array2, gpio_i2c_pullud_array1, gpio_i2c_drv_array1},
	[LOWPOWER] = {gpio_i2c_func_array2, gpio_i2c_pullud_array1, gpio_i2c_drv_array1},
};
#endif

/*spi0_cs:SPI0_CS0_N,SPI0_CS1_N,SPI0_CS2_N,SPI0_CS3_N*/
#if 0
enum lowlayer_func spi0_cs_func_array1_s10[] = {FUNC0, FUNC0, FUNC0, FUNC0,  -INVALID,};
enum lowlayer_func spi0_cs_func_array2_s10[] = {FUNC1, FUNC1, FUNC1, FUNC1,  -INVALID,};
enum pull_updown spi0_cs_pullud_array1_s10[] = {PULLDOWN, PULLDOWN, NOPULL, NOPULL, -INVALID,};
enum drive_strength spi0_cs_drv_array1_s10[] = {LEVEL2, LEVEL2, LEVEL2, LEVEL2, -INVALID,};
#else
enum lowlayer_func spi0_cs_func_array1_s10[] = {FUNC0, FUNC0, -INVALID,};
enum lowlayer_func spi0_cs_func_array2_s10[] = {FUNC1, FUNC1, -INVALID,};
enum pull_updown spi0_cs_pullud_array1_s10[] = {PULLDOWN, PULLDOWN, -INVALID,};
enum drive_strength spi0_cs_drv_array1_s10[] = {LEVEL2, LEVEL2, -INVALID,};

#endif
struct block_config spi0_cs_config_s10[] = {
	[NORMAL] = {spi0_cs_func_array1_s10, spi0_cs_pullud_array1_s10, spi0_cs_drv_array1_s10},
	[GPIO] = {spi0_cs_func_array2_s10, spi0_cs_pullud_array1_s10, spi0_cs_drv_array1_s10},
	[LOWPOWER] = {spi0_cs_func_array2_s10, spi0_cs_pullud_array1_s10, spi0_cs_drv_array1_s10},
};


/*uart2:onewire, usim_rst*/
enum lowlayer_func uart2_func_array1_s10[] = {FUNC2, FUNC2, -INVALID,};
enum lowlayer_func uart2_func_array2_s10[] = {FUNC1, FUNC1, -INVALID,};
enum pull_updown uart2_pullud_array1_s10[] = {NOPULL, NOPULL, -INVALID,};
enum pull_updown uart2_pullud_array2_s10[] = {PULLDOWN, PULLDOWN, -INVALID,};
enum drive_strength uart2_drv_array1_s10[] = {RESERVE, RESERVE, -INVALID,};
struct block_config uart2_config_s10[] = {
	[NORMAL] = {uart2_func_array1_s10, uart2_pullud_array1_s10, uart2_drv_array1_s10},
	[GPIO] = {uart2_func_array2_s10, uart2_pullud_array1_s10, uart2_drv_array1_s10},
	[LOWPOWER] = {uart2_func_array2_s10, uart2_pullud_array2_s10, uart2_drv_array1_s10},
};

/*kpc key pad  3X3:keypad_in0~keypad_in2, keypad_out0~keypad_out2*/
#if 0
enum lowlayer_func kpc_func_array1_s10[] = {FUNC0, FUNC0, FUNC0, FUNC0, FUNC0, FUNC0, -INVALID,};
enum lowlayer_func kpc_func_array2_s10[] = {FUNC1, FUNC1, FUNC1, FUNC1, FUNC1, FUNC1, -INVALID,};
enum pull_updown kpc_pullud_array1_s10[] = {PULLUP, PULLUP, PULLUP, PULLUP, PULLUP, PULLUP, -INVALID,};
enum pull_updown kpc_pullud_array2_s10[] = {PULLDOWN, PULLDOWN, PULLDOWN, \
	PULLDOWN, PULLDOWN, PULLDOWN, -INVALID,};
enum drive_strength kpc_drv_array1_s10[] = {RESERVE, RESERVE, RESERVE, RESERVE, RESERVE, RESERVE, -INVALID,};
#else
enum lowlayer_func kpc_func_array1_s10[] = {FUNC1, FUNC1, -INVALID,};
enum lowlayer_func kpc_func_array2_s10[] = {FUNC1, FUNC1, -INVALID,};
enum pull_updown kpc_pullud_array1_s10[] = {PULLUP, PULLUP, -INVALID,};
enum pull_updown kpc_pullud_array2_s10[] = {PULLUP, PULLUP, -INVALID,};
enum drive_strength kpc_drv_array1_s10[] = {RESERVE, RESERVE, -INVALID,};
#endif

struct block_config kpc_config_s10[] = {
	[NORMAL] = {kpc_func_array1_s10, kpc_pullud_array2_s10, kpc_drv_array1_s10},
	[GPIO] = {kpc_func_array2_s10, kpc_pullud_array1_s10, kpc_drv_array1_s10},
	[LOWPOWER] = {kpc_func_array2_s10, kpc_pullud_array2_s10, kpc_drv_array1_s10},
};


/*bt, g28 is used as adc interrupt pin
 *BT_SPI_CLK,BT_SPI_CS_N,BT_SPI_DATA,BT_ENABLE_RM
 */
/*btpm config*/
enum lowlayer_func btpm_func_gpio_s10[] = {FUNC1, FUNC1, -INVALID,};
enum pull_updown btpm_pullud_normal_s10[] = {PULLUP, NOPULL, -INVALID,};
//enum pull_updown btpm_pullud_gpio_s10[] = {PULLDOWN, PULLDOWN, -INVALID,};
enum pull_updown btpm_pullud_gpio_s10[] = {PULLDOWN, NOPULL, -INVALID,};
enum drive_strength btpm_drv_default_s10[] = {LEVEL2, LEVEL2, -INVALID,};
struct block_config btpm_config_s10[] = {
	[NORMAL] = {btpm_func_gpio_s10, btpm_pullud_normal_s10, btpm_drv_default_s10},
	[GPIO] = {btpm_func_gpio_s10, btpm_pullud_gpio_s10, btpm_drv_default_s10},
	[LOWPOWER] = {btpm_func_gpio_s10, btpm_pullud_gpio_s10, btpm_drv_default_s10},
};


/*pcm config*/
enum lowlayer_func pcm_func_gpio_s10_es[] = {FUNC1, FUNC1, FUNC1, -INVALID,};
enum pull_updown pcm_pullud_gpio_s10_es[] = {PULLDOWN, PULLDOWN, NOPULL,  -INVALID,};
enum drive_strength pcm_drv_default_s10_es[] = {LEVEL2, LEVEL2, LEVEL2, -INVALID,};
struct block_config pcm_config_s10_es[] = {
	[NORMAL] = {pcm_func_gpio_s10_es, pcm_pullud_gpio_s10_es, pcm_drv_default_s10_es},
	[GPIO] = {pcm_func_gpio_s10_es, pcm_pullud_gpio_s10_es, pcm_drv_default_s10_es},
	[LOWPOWER] = {pcm_func_gpio_s10_es, pcm_pullud_gpio_s10_es, pcm_drv_default_s10_es},
};

enum lowlayer_func pcm_func_gpio_s10_cs[] = {FUNC1, FUNC1, FUNC1, -INVALID,};
enum pull_updown pcm_pullud_gpio_s10_cs[] = {PULLDOWN, PULLDOWN, NOPULL,  -INVALID,};
enum drive_strength pcm_drv_default_s10_cs[] = {LEVEL2, LEVEL2, LEVEL2, -INVALID,};
struct block_config pcm_config_s10_cs[] = {
	[NORMAL] = {pcm_func_gpio_s10_cs, pcm_pullud_gpio_s10_cs, pcm_drv_default_s10_cs},
	[GPIO] = {pcm_func_gpio_s10_cs, pcm_pullud_gpio_s10_cs, pcm_drv_default_s10_cs},
	[LOWPOWER] = {pcm_func_gpio_s10_cs, pcm_pullud_gpio_s10_cs, pcm_drv_default_s10_cs},
};

/*gps config for cellguide*/
#if 0
enum lowlayer_func gps_func_cellguide_s10[] = {FUNC0, FUNC0, FUNC0, FUNC0, FUNC0, FUNC0, FUNC0, FUNC2, -INVALID,};
enum lowlayer_func gps_func_gpio_s10[] = {FUNC1, FUNC1, FUNC1, FUNC1, FUNC1, FUNC1, FUNC1, FUNC1, -INVALID,};
enum pull_updown gps_pullud_cellguide_s10[] = {PULLDOWN, PULLDOWN, NOPULL, PULLUP, PULLUP, PULLUP, PULLUP, PULLDOWN, -INVALID,};
enum pull_updown gps_pullud_gpio_s10[] = {PULLDOWN, PULLDOWN, PULLDOWN, PULLDOWN, PULLDOWN, PULLDOWN, \
	PULLDOWN, PULLDOWN, PULLDOWN, -INVALID,};
enum drive_strength gps_drv_cellguide_s10[] = {RESERVE, RESERVE, RESERVE, LEVEL1, RESERVE, LEVEL1, LEVEL1, LEVEL0, -INVALID,};
enum drive_strength gps_drv_array1_s10[] = {RESERVE, RESERVE, RESERVE, LEVEL1, RESERVE, LEVEL1, LEVEL1, LEVEL1, -INVALID,};
enum drive_strength gps_drv_array2_s10[] = {RESERVE, RESERVE, RESERVE, LEVEL0, RESERVE, LEVEL0, LEVEL0, LEVEL0, -INVALID,};
#else
enum lowlayer_func gps_func_cellguide_s10[] = {FUNC0, FUNC0, FUNC0, FUNC0, FUNC0, FUNC0, FUNC0, -INVALID,};
enum lowlayer_func gps_func_gpio_s10[] = {FUNC1, FUNC1, FUNC1, FUNC1, FUNC1, FUNC1, FUNC1, -INVALID,};
enum pull_updown gps_pullud_cellguide_s10[] = {PULLDOWN, PULLDOWN, NOPULL, PULLUP, PULLUP, PULLUP, PULLUP, -INVALID,};
enum pull_updown gps_pullud_gpio_s10[] = {PULLDOWN, PULLDOWN, PULLDOWN, PULLDOWN, PULLDOWN, PULLDOWN, PULLDOWN, -INVALID,};
enum drive_strength gps_drv_cellguide_s10[] = {RESERVE, RESERVE, RESERVE, LEVEL1, RESERVE, LEVEL1, LEVEL1, -INVALID,};
enum drive_strength gps_drv_array1_s10[] = {RESERVE, RESERVE, RESERVE, LEVEL1, RESERVE, LEVEL1, LEVEL1, -INVALID,};
enum drive_strength gps_drv_array2_s10[] = {RESERVE, RESERVE, RESERVE, LEVEL0, RESERVE, LEVEL0, LEVEL0, -INVALID,};
#endif
struct block_config gps_cellguide_config_s10[] = {
	[NORMAL] = {gps_func_cellguide_s10, gps_pullud_cellguide_s10, gps_drv_cellguide_s10},
	[GPIO] = {gps_func_gpio_s10, gps_pullud_gpio_s10, gps_drv_array1_s10},
	[LOWPOWER] = {gps_func_gpio_s10, gps_pullud_gpio_s10, gps_drv_array2_s10},
};



/*touch screen:GPIO_156,GPIO_157*/
enum lowlayer_func ts_func_array1_s10_es[] = {RESERVE, RESERVE, FUNC1, -INVALID,};
enum pull_updown ts_pullud_array1_s10_es[] = {PULLDOWN, PULLDOWN, NOPULL, -INVALID,};
enum drive_strength ts_drv_array1_s10_es[] = {RESERVE, RESERVE, LEVEL2 -INVALID,};
struct block_config ts_config_s10_es[] = {
	[NORMAL] = {ts_func_array1_s10_es, ts_pullud_array1_s10_es, ts_drv_array1_s10_es},
	[GPIO] = {ts_func_array1_s10_es, ts_pullud_array1_s10_es, ts_drv_array1_s10_es},
	[LOWPOWER] = {ts_func_array1_s10_es, ts_pullud_array1_s10_es, ts_drv_array1_s10_es},
};

enum lowlayer_func ts_func_array1_s10_cs[] = {RESERVE, RESERVE, FUNC1, -INVALID,};
enum pull_updown ts_pullud_array1_s10_cs[] = {PULLDOWN, PULLDOWN, NOPULL, -INVALID,};
enum drive_strength ts_drv_array1_s10_cs[] = {RESERVE, RESERVE, LEVEL2 -INVALID,};
struct block_config ts_config_s10_cs[] = {
	[NORMAL] = {ts_func_array1_s10_cs, ts_pullud_array1_s10_cs, ts_drv_array1_s10_cs},
	[GPIO] = {ts_func_array1_s10_cs, ts_pullud_array1_s10_cs, ts_drv_array1_s10_cs},
	[LOWPOWER] = {ts_func_array1_s10_cs, ts_pullud_array1_s10_cs, ts_drv_array1_s10_cs},
};

#if 0
enum lowlayer_func pwm_func_normal_s10[] = {FUNC0, FUNC1, -INVALID,};
enum lowlayer_func pwm_func_gpio_s10[] = {FUNC1, FUNC1, -INVALID,};
enum pull_updown pwm_pullud_array1_s10[] = {NOPULL, PULLDOWN, -INVALID,};
enum drive_strength pwm_drv_array1_s10[] = {LEVEL2, LEVEL2, -INVALID,};
#else
enum lowlayer_func pwm_func_normal_s10[] = {FUNC0, -INVALID,};
enum lowlayer_func pwm_func_gpio_s10[] = {FUNC1, -INVALID,};
enum pull_updown pwm_pullud_array1_s10[] = {NOPULL, -INVALID,};
enum pull_updown pwm_pullud_array2_s10[] = {PULLDOWN, -INVALID,};
enum drive_strength pwm_drv_array1_s10[] = {LEVEL2, -INVALID,};
#endif
struct block_config pwm_config_s10[] = {
	[NORMAL] = {pwm_func_normal_s10, pwm_pullud_array1_s10, pwm_drv_array1_s10},
	[GPIO] = {pwm_func_gpio_s10, pwm_pullud_array1_s10, pwm_drv_array1_s10},
	[LOWPOWER] = {pwm_func_gpio_s10, pwm_pullud_array2_s10, pwm_drv_array1_s10},
};

/*hdmi pins:
 *HDMI_SCL, HDMI_SDA, HDMI_CEC, HDMI_CEC
 */
#if 0
enum lowlayer_func hdmi_func_normal_s10[] = {FUNC0, FUNC0, FUNC0, FUNC0, -INVALID,};
enum lowlayer_func hdmi_func_gpio_s10[] = {FUNC1, FUNC1, FUNC1, FUNC1, -INVALID,};
enum pull_updown hdmi_pullud_array1_s10[] = {NOPULL, NOPULL, NOPULL, NOPULL, -INVALID,};
enum pull_updown hdmi_pullud_array2_s10[] = {PULLDOWN, PULLDOWN, PULLDOWN, PULLDOWN, -INVALID,};
enum drive_strength hdmi_drv_array1_s10[] = {RESERVE, RESERVE, RESERVE, RESERVE, -INVALID,};
#else
enum lowlayer_func hdmi_func_normal_s10[] = {FUNC0, FUNC0, FUNC0, -INVALID,};
enum lowlayer_func hdmi_func_gpio_s10[] = {FUNC1, FUNC1, FUNC1, -INVALID,};
enum pull_updown hdmi_pullud_array1_s10[] = {NOPULL, NOPULL, NOPULL, -INVALID,};
enum pull_updown hdmi_pullud_array2_s10[] = {PULLDOWN, PULLDOWN, PULLDOWN, -INVALID,};
enum drive_strength hdmi_drv_array1_s10[] = {RESERVE, RESERVE, RESERVE, -INVALID,};
#endif
struct block_config hdmi_config_s10[] = {
	[NORMAL] = {hdmi_func_normal_s10, hdmi_pullud_array1_s10, hdmi_drv_array1_s10},
	[GPIO] = {hdmi_func_gpio_s10, hdmi_pullud_array2_s10, hdmi_drv_array1_s10},
	[LOWPOWER] = {hdmi_func_gpio_s10, hdmi_pullud_array2_s10, hdmi_drv_array1_s10},
};

/*isp pins:
 *isp_cclk0, isp_cclk2, isp_gpio0,
 *isp_gpio1, isp_gpio2, isp_gpio3, isp_gpio4, isp_gpio6,
 *and isp_gpio8, isp_gpio9 are used as gpio,don't need to mux
 */
enum lowlayer_func isp_func_normal_s10[] = {FUNC0, FUNC0, FUNC0, FUNC0, FUNC0, FUNC0, FUNC0,\
	FUNC0, -INVALID,};
enum lowlayer_func isp_func_gpio_s10[] = {FUNC1, FUNC1, FUNC1, FUNC1, FUNC1, FUNC1, FUNC1, FUNC1, -INVALID,};
enum pull_updown isp_pullud_array1_s10[] = {NOPULL, NOPULL, NOPULL, NOPULL, NOPULL, \
	NOPULL, NOPULL, NOPULL, -INVALID,};
//enum pull_updown isp_pullud_array2_s10[] = {NOPULL, NOPULL, NOPULL, \
//	NOPULL, NOPULL, PULLDOWN, PULLDOWN, NOPULL, -INVALID,};
enum pull_updown isp_pullud_array2_s10[] = {NOPULL, NOPULL, NOPULL, \
	NOPULL,PULLDOWN, PULLDOWN, PULLDOWN, NOPULL, -INVALID,};//gpio_71 PD
enum drive_strength isp_drv_array1_s10[] = {LEVEL2, LEVEL2, RESERVE, RESERVE, \
	RESERVE, RESERVE, RESERVE, RESERVE, -INVALID,};
struct block_config isp_config_s10[] = {
	[NORMAL] = {isp_func_normal_s10, isp_pullud_array1_s10, isp_drv_array1_s10},
	[GPIO] = {isp_func_gpio_s10, isp_pullud_array2_s10, isp_drv_array1_s10},
	[LOWPOWER] = {isp_func_gpio_s10, isp_pullud_array2_s10, isp_drv_array1_s10},
};

/*charger pins:
 *EFUSE_PGM, ISP_GPIO5
 */
#if 0
enum lowlayer_func charger_func_normal_s10[] = {FUNC0, FUNC1, -INVALID,};
enum lowlayer_func charger_func_gpio_s10[] = {FUNC0, FUNC1, -INVALID,};
enum pull_updown charger_pullud_array1_s10[] = {PULLUP, PULLUP, -INVALID,};
enum drive_strength charger_drv_array1_s10[] = {RESERVE, RESERVE, -INVALID,};
#else
/* y4, g28, a20, aa4, u1, ac25, ab2, */
enum lowlayer_func charger_func_normal_s10_es[] = {FUNC1,FUNC1, FUNC1, FUNC1, FUNC0, FUNC1, -INVALID,};
enum lowlayer_func charger_func_gpio_s10_es[] = {FUNC1,FUNC1, FUNC1, FUNC1, FUNC0, FUNC1, -INVALID,};
enum pull_updown charger_pullud_array1_s10_es[] = {PULLUP,PULLDOWN, PULLDOWN, NOPULL, PULLDOWN, PULLUP, -INVALID,};
enum pull_updown charger_pullud_array2_s10_es[] = {PULLDOWN,PULLDOWN, PULLUP, PULLUP, PULLUP, PULLDOWN, -INVALID,};
//enum drive_strength charger_drv_array1_s10[] = {LEVEL0, LEVEL2, LEVEL2, LEVEL0, LEVEL0, LEVEL2, LEVEL0, -INVALID,};
enum drive_strength charger_drv_array1_s10_es[] = {LEVEL4, LEVEL2, LEVEL0, LEVEL0, LEVEL2, LEVEL4, -INVALID,};
#endif
struct block_config charger_config_s10_es[] = {
	[NORMAL] = {charger_func_normal_s10_es, charger_pullud_array1_s10_es, charger_drv_array1_s10_es},
	[GPIO] = {charger_func_gpio_s10_es, charger_pullud_array1_s10_es, charger_drv_array1_s10_es},
	[LOWPOWER] = {charger_func_gpio_s10_es, charger_pullud_array2_s10_es, charger_drv_array1_s10_es},
};

/* y4, g28, a20, aa4, u1, ac25, ab2, */
enum lowlayer_func charger_func_normal_s10_cs[] = {FUNC1, FUNC0, FUNC1, FUNC1, FUNC1, FUNC0, FUNC1, -INVALID,};
enum lowlayer_func charger_func_gpio_s10_cs[] = {FUNC1, FUNC0, FUNC1, FUNC1, FUNC1, FUNC0, FUNC1, -INVALID,};
enum pull_updown charger_pullud_array1_s10_cs[] = {PULLUP, PULLDOWN, PULLDOWN, PULLDOWN, NOPULL, PULLDOWN, PULLUP, -INVALID,};
enum pull_updown charger_pullud_array2_s10_cs[] = {PULLDOWN, PULLUP, PULLDOWN, PULLUP, PULLUP, PULLUP, PULLDOWN, -INVALID,};
enum drive_strength charger_drv_array1_s10_cs[] = {LEVEL4, LEVEL2, LEVEL2, LEVEL0, LEVEL0, LEVEL2, LEVEL4, -INVALID,};
struct block_config charger_config_s10_cs[] = {
	[NORMAL] = {charger_func_normal_s10_cs, charger_pullud_array1_s10_cs, charger_drv_array1_s10_cs},
	[GPIO] = {charger_func_gpio_s10_cs, charger_pullud_array1_s10_cs, charger_drv_array1_s10_cs},
	[LOWPOWER] = {charger_func_gpio_s10_cs, charger_pullud_array2_s10_cs, charger_drv_array1_s10_cs},
};

#if defined(CONFIG_SUPPORT_MICRO_USB_PORT)
//wuxinxian>>
/*GPIO145,GPIO116,GPIO11,GPIO21,GPIO82*/
enum lowlayer_func micro_usb_func_gpio_link[] = {FUNC1, FUNC1, FUNC1, FUNC1, FUNC0,-INVALID,};
enum pull_updown micro_usb_pullud_array1_link[] = {PULLUP, PULLDOWN, NOPULL, NOPULL, PULLDOWN,-INVALID,};
enum pull_updown micro_usb_pullud_array2_link[] = {PULLDOWN, NOPULL, PULLDOWN, NOPULL, PULLDOWN,-INVALID,};
enum drive_strength micro_usb_drv_array1_link[] = {LEVEL2, LEVEL2, LEVEL2, LEVEL2,LEVEL2, -INVALID,};
//wuxinxian<<
struct block_config micro_usb_config_link[] = {
	[NORMAL] = {micro_usb_func_gpio_link, micro_usb_pullud_array1_link, micro_usb_drv_array1_link},
	[GPIO]	= {micro_usb_func_gpio_link, micro_usb_pullud_array1_link, micro_usb_drv_array1_link},
	[LOWPOWER] = {micro_usb_func_gpio_link,micro_usb_pullud_array2_link,micro_usb_drv_array1_link},
};
#endif

#if defined(CONFIG_FSA880_I2C)
/* ab5, r26, aa25, aa2 */
enum lowlayer_func bc11_func_gpio_s10_es[] = {FUNC1, FUNC1, FUNC1, FUNC1, FUNC1,FUNC1, -INVALID,};
enum pull_updown bc11_pullud_array1_s10_es[] = {NOPULL, PULLDOWN, PULLDOWN, PULLDOWN, PULLUP,PULLDOWN, -INVALID,};
enum pull_updown bc11_pullud_array2_s10_es[] = {PULLUP, PULLDOWN, PULLDOWN, PULLDOWN, NOPULL,NOPULL, -INVALID,};
enum drive_strength bc11_drv_array1_s10_es[] = {LEVEL2, LEVEL0, LEVEL2, LEVEL0, LEVEL2,LEVEL2, -INVALID,};
struct block_config bc11_config_s10_es[] = {
	[NORMAL] = {bc11_func_gpio_s10_es, bc11_pullud_array1_s10_es, bc11_drv_array1_s10_es},
	[GPIO] = {bc11_func_gpio_s10_es, bc11_pullud_array1_s10_es, bc11_drv_array1_s10_es},
	[LOWPOWER] = {bc11_func_gpio_s10_es, bc11_pullud_array2_s10_es, bc11_drv_array1_s10_es},
};

/* ab5, r26, aa25, aa2 */
enum lowlayer_func bc11_func_gpio_s10_cs[] = {FUNC1, FUNC0, FUNC1, FUNC1, FUNC1, FUNC1, -INVALID,};
enum pull_updown bc11_pullud_array1_s10_cs[] = {NOPULL, PULLDOWN, PULLDOWN, PULLDOWN, NOPULL, PULLDOWN, -INVALID,};
enum pull_updown bc11_pullud_array2_s10_cs[] = {PULLUP, PULLDOWN, PULLDOWN, PULLDOWN, NOPULL, NOPULL, -INVALID,};
enum drive_strength bc11_drv_array1_s10_cs[] = {LEVEL2, LEVEL0, LEVEL2, LEVEL0, LEVEL2, LEVEL2, -INVALID,};
struct block_config bc11_config_s10_cs[] = {
	[NORMAL] = {bc11_func_gpio_s10_cs, bc11_pullud_array1_s10_cs, bc11_drv_array1_s10_cs},
	[GPIO] = {bc11_func_gpio_s10_cs, bc11_pullud_array1_s10_cs, bc11_drv_array1_s10_cs},
	[LOWPOWER] = {bc11_func_gpio_s10_cs, bc11_pullud_array2_s10_cs, bc11_drv_array1_s10_cs},
};
#endif

/* ad26 */
enum lowlayer_func boost_5v_func_gpio_s10[] = {FUNC0, -INVALID,};
enum pull_updown boost_5v_pullud_array1_s10[] = {PULLDOWN, -INVALID,};
enum drive_strength boost_5v_drv_array1_s10[] = {LEVEL2, -INVALID,};
struct block_config boost_5v_config_s10[] = {
	[NORMAL] = {boost_5v_func_gpio_s10, boost_5v_pullud_array1_s10, boost_5v_drv_array1_s10},
	[GPIO] = {boost_5v_func_gpio_s10, boost_5v_pullud_array1_s10, boost_5v_drv_array1_s10},
	[LOWPOWER] = {boost_5v_func_gpio_s10, boost_5v_pullud_array1_s10, boost_5v_drv_array1_s10},
};


/* g28 */
enum lowlayer_func battery_gauge_func_gpio_s10[] = {FUNC0, -INVALID,};
enum pull_updown battery_gauge_pullud_array1_s10[] = {PULLDOWN, -INVALID,};
enum drive_strength battery_gauge_drv_array1_s10[] = {LEVEL2, -INVALID,};
struct block_config battery_gauge_config_s10[] = {
	[NORMAL] = {battery_gauge_func_gpio_s10, battery_gauge_pullud_array1_s10, battery_gauge_drv_array1_s10},
	[GPIO] = {battery_gauge_func_gpio_s10, battery_gauge_pullud_array1_s10, battery_gauge_drv_array1_s10},
	[LOWPOWER] = {battery_gauge_func_gpio_s10, battery_gauge_pullud_array1_s10, battery_gauge_drv_array1_s10},
};



/* af23->GPIO143_SDCARD_PWREN, a28->GPIO083_SD_DETECT*/
enum lowlayer_func sd_ctrl_func_gpio_s10_es[] = {FUNC1, FUNC1, -INVALID,};
enum pull_updown sd_ctrl_pullud_array1_s10_es[] = {PULLUP, PULLUP, -INVALID,};
enum drive_strength sd_ctrl_drv_array1_s10_es[] = {LEVEL2, LEVEL0, -INVALID,};
struct block_config sd_ctrl_config_s10_es[] = {
	[NORMAL] = {sd_ctrl_func_gpio_s10_es, sd_ctrl_pullud_array1_s10_es, sd_ctrl_drv_array1_s10_es},
	[GPIO] = {sd_ctrl_func_gpio_s10_es, sd_ctrl_pullud_array1_s10_es, sd_ctrl_drv_array1_s10_es},
	[LOWPOWER] = {sd_ctrl_func_gpio_s10_es, sd_ctrl_pullud_array1_s10_es, sd_ctrl_drv_array1_s10_es},
};

/* af23->GPIO143_SDCARD_PWREN, a28->GPIO083_SD_DETECT*/
enum lowlayer_func sd_ctrl_func_gpio_s10_cs[] = {FUNC1, FUNC1, -INVALID,};
enum pull_updown sd_ctrl_pullud_array1_s10_cs[] = {PULLUP, PULLUP, -INVALID,};
enum drive_strength sd_ctrl_drv_array1_s10_cs[] = {LEVEL2, LEVEL0, -INVALID,};
struct block_config sd_ctrl_config_s10_cs[] = {
	[NORMAL] = {sd_ctrl_func_gpio_s10_cs, sd_ctrl_pullud_array1_s10_cs, sd_ctrl_drv_array1_s10_cs},
	[GPIO] = {sd_ctrl_func_gpio_s10_cs, sd_ctrl_pullud_array1_s10_cs, sd_ctrl_drv_array1_s10_cs},
	[LOWPOWER] = {sd_ctrl_func_gpio_s10_cs, sd_ctrl_pullud_array1_s10_cs, sd_ctrl_drv_array1_s10_cs},
};

/* u26->GPIO079_AP_WAKEUP_MHL, t28->GPIO080_MHL_INT, t26->GPIO081_MHL_RST_N */
enum lowlayer_func mhl_func_gpio_s10_es[] = {FUNC0, FUNC0, FUNC0, -INVALID,};
enum pull_updown mhl_pullud_array1_s10_es[] = {PULLDOWN, PULLUP, PULLUP, -INVALID,};
enum drive_strength mhl_drv_array1_s10_es[] = {LEVEL2, LEVEL0, LEVEL0, -INVALID,};
struct block_config mhl_config_s10_es[] = {
	[NORMAL] = {mhl_func_gpio_s10_es, mhl_pullud_array1_s10_es, mhl_drv_array1_s10_es},
	[GPIO] = {mhl_func_gpio_s10_es, mhl_pullud_array1_s10_es, mhl_drv_array1_s10_es},
	[LOWPOWER] = {mhl_func_gpio_s10_es, mhl_pullud_array1_s10_es, mhl_drv_array1_s10_es},
};

enum lowlayer_func mhl_func_gpio_s10_cs[] = {FUNC0, FUNC0, FUNC0, -INVALID,};
enum pull_updown mhl_pullud_array1_s10_cs[] = {PULLDOWN, PULLUP, PULLUP, -INVALID,};
enum drive_strength mhl_drv_array1_s10_cs[] = {LEVEL2, LEVEL0, LEVEL0, -INVALID,};
struct block_config mhl_config_s10_cs[] = {
	[NORMAL] = {mhl_func_gpio_s10_cs, mhl_pullud_array1_s10_cs, mhl_drv_array1_s10_cs},
	[GPIO] = {mhl_func_gpio_s10_cs, mhl_pullud_array1_s10_cs, mhl_drv_array1_s10_cs},
	[LOWPOWER] = {mhl_func_gpio_s10_cs, mhl_pullud_array1_s10_cs, mhl_drv_array1_s10_cs},
};

/*g25, h25, v2, w2*/
enum lowlayer_func camera_pwr_func_gpio_s10_es[] = {FUNC0, FUNC0, FUNC1, FUNC1, -INVALID,};
enum pull_updown camera_pwr_pullud_array1_s10_es[] = {PULLDOWN, PULLDOWN, PULLDOWN, PULLDOWN, -INVALID,};
enum drive_strength camera_pwr_drv_array1_s10_es[] = {LEVEL2, LEVEL2, LEVEL0, LEVEL0, -INVALID,};
struct block_config camera_pwr_config_s10_es[] = {
	[NORMAL] = {camera_pwr_func_gpio_s10_es, camera_pwr_pullud_array1_s10_es, camera_pwr_drv_array1_s10_es},
	[GPIO] = {camera_pwr_func_gpio_s10_es, camera_pwr_pullud_array1_s10_es, camera_pwr_drv_array1_s10_es},
	[LOWPOWER] = {camera_pwr_func_gpio_s10_es, camera_pwr_pullud_array1_s10_es, camera_pwr_drv_array1_s10_es},
};

/*g25, h25, v2, w2*/
enum lowlayer_func camera_pwr_func_gpio_s10_cs[] = {FUNC0, FUNC0, FUNC1, FUNC1, -INVALID,};
enum pull_updown camera_pwr_pullud_array1_s10_cs[] = {PULLDOWN, PULLDOWN, PULLDOWN, PULLDOWN, -INVALID,};
enum drive_strength camera_pwr_drv_array1_s10_cs[] = {LEVEL2, LEVEL2, LEVEL0, LEVEL0, -INVALID,};
struct block_config camera_pwr_config_s10_cs[] = {
	[NORMAL] = {camera_pwr_func_gpio_s10_cs, camera_pwr_pullud_array1_s10_cs, camera_pwr_drv_array1_s10_cs},
	[GPIO] = {camera_pwr_func_gpio_s10_cs, camera_pwr_pullud_array1_s10_cs, camera_pwr_drv_array1_s10_cs},
	[LOWPOWER] = {camera_pwr_func_gpio_s10_cs, camera_pwr_pullud_array1_s10_cs, camera_pwr_drv_array1_s10_cs},
};

/*n28 */
enum lowlayer_func main_camera_func_gpio_s10[] = {FUNC1, -INVALID,};
enum pull_updown main_camera_pullud_array1_s10[] = {PULLDOWN, -INVALID,};
enum drive_strength main_camera_drv_array1_s10[] = {LEVEL0, -INVALID,};
struct block_config main_camera_config_s10[] = {
	[NORMAL] = {main_camera_func_gpio_s10, main_camera_pullud_array1_s10, main_camera_drv_array1_s10},
	[GPIO] = {main_camera_func_gpio_s10, main_camera_pullud_array1_s10, main_camera_drv_array1_s10},
	[LOWPOWER] = {main_camera_func_gpio_s10, main_camera_pullud_array1_s10, main_camera_drv_array1_s10},
};

/*m23 */
enum lowlayer_func slave_camera_func_gpio_s10[] = {FUNC1, -INVALID,};
enum pull_updown slave_camera_pullud_array1_s10[] = {PULLDOWN, -INVALID,};
enum drive_strength slave_camera_drv_array1_s10[] = {LEVEL0, -INVALID,};
struct block_config slave_camera_config_s10[] = {
	[NORMAL] = {slave_camera_func_gpio_s10, slave_camera_pullud_array1_s10, slave_camera_drv_array1_s10},
	[GPIO] = {slave_camera_func_gpio_s10, slave_camera_pullud_array1_s10, slave_camera_drv_array1_s10},
	[LOWPOWER] = {slave_camera_func_gpio_s10, slave_camera_pullud_array1_s10, slave_camera_drv_array1_s10},
};


/*aa28 */
enum lowlayer_func mr_sensor_func_gpio_s10[] = {FUNC1, -INVALID,};
enum pull_updown mr_sensor_pullud_array1_s10[] = {PULLDOWN, -INVALID,};
enum drive_strength mr_sensor_drv_array1_s10[] = {LEVEL2, -INVALID,};
struct block_config mr_sensor_config_s10[] = {
	[NORMAL] = {mr_sensor_func_gpio_s10, mr_sensor_pullud_array1_s10, mr_sensor_drv_array1_s10},
	[GPIO] = {mr_sensor_func_gpio_s10, mr_sensor_pullud_array1_s10, mr_sensor_drv_array1_s10},
	[LOWPOWER] = {mr_sensor_func_gpio_s10, mr_sensor_pullud_array1_s10, mr_sensor_drv_array1_s10},
};


/*ac28 */
enum lowlayer_func compass_func_gpio_s10[] = {FUNC1, -INVALID,};
enum pull_updown compass_pullud_array1_s10[] = {PULLDOWN, -INVALID,};
enum drive_strength compass_drv_array1_s10[] = {LEVEL2, -INVALID,};
struct block_config compass_config_s10[] = {
	[NORMAL] = {compass_func_gpio_s10, compass_pullud_array1_s10, compass_drv_array1_s10},
	[GPIO] = {compass_func_gpio_s10, compass_pullud_array1_s10, compass_drv_array1_s10},
	[LOWPOWER] = {compass_func_gpio_s10, compass_pullud_array1_s10, compass_drv_array1_s10},
};


/*d15, ab26 */
enum lowlayer_func gyroscope_func_gpio_s10[] = {FUNC1, FUNC1, -INVALID,};
enum pull_updown gyroscope_pullud_array1_s10[] = {PULLDOWN, PULLDOWN, -INVALID,};
enum drive_strength gyroscope_drv_array1_s10[] = {LEVEL2, LEVEL2, -INVALID,};
struct block_config gyroscope_config_s10[] = {
	[NORMAL] = {gyroscope_func_gpio_s10, gyroscope_pullud_array1_s10, gyroscope_drv_array1_s10},
	[GPIO] = {gyroscope_func_gpio_s10, gyroscope_pullud_array1_s10, gyroscope_drv_array1_s10},
	[LOWPOWER] = {gyroscope_func_gpio_s10, gyroscope_pullud_array1_s10, gyroscope_drv_array1_s10},
};


/*ae23 */
enum lowlayer_func capacitor_sensor_func_gpio_s10[] = {FUNC1, -INVALID,};
enum pull_updown capacitor_sensor_pullud_array1_s10[] = {PULLUP, -INVALID,};
enum drive_strength capacitor_sensor_drv_array1_s10[] = {LEVEL2, -INVALID,};
struct block_config capacitor_sensor_config_s10[] = {
	[NORMAL] = {capacitor_sensor_func_gpio_s10, capacitor_sensor_pullud_array1_s10, capacitor_sensor_drv_array1_s10},
	[GPIO] = {capacitor_sensor_func_gpio_s10, capacitor_sensor_pullud_array1_s10, capacitor_sensor_drv_array1_s10},
	[LOWPOWER] = {capacitor_sensor_func_gpio_s10, capacitor_sensor_pullud_array1_s10, capacitor_sensor_drv_array1_s10},
};

/*ab4 */
enum lowlayer_func sim_dect_func_gpio_s10[] = {FUNC1, -INVALID,};
enum pull_updown sim_dect_pullud_array1_s10[] = {NOPULL, -INVALID,};
enum pull_updown sim_dect_pullud_array2_s10[] = {PULLUP, -INVALID,};
enum drive_strength sim_dect_drv_array1_s10[] = {LEVEL0, -INVALID,};
struct block_config sim_dect_config_s10[] = {
	[NORMAL] = {sim_dect_func_gpio_s10, sim_dect_pullud_array1_s10, sim_dect_drv_array1_s10},
	[GPIO] = {sim_dect_func_gpio_s10, sim_dect_pullud_array1_s10, sim_dect_drv_array1_s10},
	[LOWPOWER] = {sim_dect_func_gpio_s10, sim_dect_pullud_array2_s10, sim_dect_drv_array1_s10},
};

/*h25 */
enum lowlayer_func extral_dcdc_func_gpio_s10[] = {FUNC0, -INVALID,};
enum pull_updown extral_dcdc_pullud_array1_s10[] = {PULLDOWN, -INVALID,};
enum drive_strength extral_dcdc_drv_array1_s10[] = {LEVEL2, -INVALID,};
struct block_config extral_dcdc_config_s10[] = {
	[NORMAL] = {extral_dcdc_func_gpio_s10, extral_dcdc_pullud_array1_s10, extral_dcdc_drv_array1_s10},
	[GPIO] = {extral_dcdc_func_gpio_s10, extral_dcdc_pullud_array1_s10, extral_dcdc_drv_array1_s10},
	[LOWPOWER] = {extral_dcdc_func_gpio_s10, extral_dcdc_pullud_array1_s10, extral_dcdc_drv_array1_s10},
};

/*h26 */
enum lowlayer_func dock_func_gpio_s10_es[] = {FUNC0, -INVALID,};
enum pull_updown dock_pullud_array1_s10_es[] = {PULLDOWN, -INVALID,};
enum pull_updown dock_pullud_array2_s10_es[] = {PULLUP, -INVALID,};
enum drive_strength dock_drv_array1_s10_es[] = {LEVEL2, -INVALID,};
struct block_config dock_config_s10_es[] = {
	[NORMAL] = {dock_func_gpio_s10_es, dock_pullud_array1_s10_es, dock_drv_array1_s10_es},
	[GPIO] = {dock_func_gpio_s10_es, dock_pullud_array1_s10_es, dock_drv_array1_s10_es},
	[LOWPOWER] = {dock_func_gpio_s10_es, dock_pullud_array2_s10_es, dock_drv_array1_s10_es},
};

/*h26_cs */
enum lowlayer_func dock_func_gpio_s10_cs[] = {FUNC0, -INVALID,};
enum pull_updown dock_pullud_array1_s10_cs[] = {PULLDOWN, -INVALID,};
enum pull_updown dock_pullud_array2_s10_cs[] = {PULLUP, -INVALID,};
enum drive_strength dock_drv_array1_s10_cs[] = {LEVEL2, -INVALID,};
struct block_config dock_config_s10_cs[] = {
	[NORMAL] = {dock_func_gpio_s10_cs, dock_pullud_array1_s10_cs, dock_drv_array1_s10_cs},
	[GPIO] = {dock_func_gpio_s10_cs, dock_pullud_array1_s10_cs, dock_drv_array1_s10_cs},
	[LOWPOWER] = {dock_func_gpio_s10_cs, dock_pullud_array2_s10_cs, dock_drv_array1_s10_cs},
};

/*a27 */
enum lowlayer_func audio_spk_func_gpio_s10[] = {FUNC1, -INVALID,};
enum pull_updown audio_spk_pullud_array1_s10[] = {PULLDOWN, -INVALID,};
enum drive_strength audio_spk_drv_array1_s10[] = {LEVEL2, -INVALID,};
struct block_config audio_spk_config_s10[] = {
	[NORMAL] = {audio_spk_func_gpio_s10, audio_spk_pullud_array1_s10, audio_spk_drv_array1_s10},
	[GPIO] = {audio_spk_func_gpio_s10, audio_spk_pullud_array1_s10, audio_spk_drv_array1_s10},
	[LOWPOWER] = {audio_spk_func_gpio_s10, audio_spk_pullud_array1_s10, audio_spk_drv_array1_s10},
};


/*b26 */
enum lowlayer_func audio_earphone_func_gpio_s10[] = {FUNC1, -INVALID,};
enum pull_updown audio_earphone_pullud_array1_s10[] = {PULLDOWN, -INVALID,};
enum drive_strength audio_earphone_drv_array1_s10[] = {LEVEL2, -INVALID,};
struct block_config audio_earphone_config_s10[] = {
	[NORMAL] = {audio_spk_func_gpio_s10, audio_spk_pullud_array1_s10, audio_spk_drv_array1_s10},
	[GPIO] = {audio_spk_func_gpio_s10, audio_spk_pullud_array1_s10, audio_spk_drv_array1_s10},
	[LOWPOWER] = {audio_spk_func_gpio_s10, audio_spk_pullud_array1_s10, audio_spk_drv_array1_s10},
};

#if 0
/* d1, f5, e1, e21, d2, l25, e2 */
enum lowlayer_func modem_ap_ctrl_func_gpio[] = {FUNC1, FUNC1, FUNC1, FUNC1, FUNC1, FUNC1, FUNC1, -INVALID,};
enum pull_updown modem_ap_ctrl_pullud_array1[] = {PULLDOWN, PULLDOWN, PULLDOWN, PULLDOWN, PULLDOWN, PULLDOWN, PULLDOWN, -INVALID,};
enum drive_strength modem_ap_ctrl_drv_array1[] = {LEVEL0, LEVEL0, LEVEL0, LEVEL2, LEVEL0, LEVEL2, LEVEL0, -INVALID,};
struct block_config modem_ap_ctrl_config[] = {
	[NORMAL] = {modem_ap_ctrl_func_gpio, modem_ap_ctrl_pullud_array1, modem_ap_ctrl_drv_array1},
	[GPIO] = {modem_ap_ctrl_func_gpio, modem_ap_ctrl_pullud_array1, modem_ap_ctrl_drv_array1},
	[LOWPOWER] = {modem_ap_ctrl_func_gpio, modem_ap_ctrl_pullud_array1, modem_ap_ctrl_drv_array1},
};
#else
/* g1, f4, f2, f1, g2, d1, d2, e2, e1 */
enum lowlayer_func modem_ap_ctrl_func_gpio_s10[] = {FUNC1, FUNC1, FUNC1, FUNC1, FUNC1, FUNC1, \
                                                FUNC1, FUNC1,  -INVALID,};
enum pull_updown modem_ap_ctrl_pullud_array1_s10[] = {NOPULL, PULLDOWN, PULLDOWN, NOPULL, NOPULL, NOPULL, \
                                                PULLDOWN, PULLDOWN,  -INVALID,};
enum drive_strength modem_ap_ctrl_drv_array1_s10[] = {LEVEL0, LEVEL0, LEVEL0, LEVEL0, LEVEL0, LEVEL0,  \
                                                  LEVEL0, LEVEL0,  -INVALID,};
struct block_config modem_ap_ctrl_config_s10[] = {
	[NORMAL] = {modem_ap_ctrl_func_gpio_s10, modem_ap_ctrl_pullud_array1_s10, modem_ap_ctrl_drv_array1_s10},
	[GPIO] = {modem_ap_ctrl_func_gpio_s10, modem_ap_ctrl_pullud_array1_s10, modem_ap_ctrl_drv_array1_s10},
	[LOWPOWER] = {modem_ap_ctrl_func_gpio_s10, modem_ap_ctrl_pullud_array1_s10, modem_ap_ctrl_drv_array1_s10},
};
#endif

enum lowlayer_func usb_switch_ctrl_func_gpio_s10[] = {FUNC1, -INVALID,};
enum pull_updown usb_switch_ctrl_pullud_array1_s10[] = {NOPULL, -INVALID,};													
enum drive_strength usb_switch_ctrl_drv_array1_s10[] = {LEVEL0, -INVALID,};
struct block_config usb_switch_ctrl_config_s10[] = {
	[NORMAL] = {usb_switch_ctrl_func_gpio_s10, usb_switch_ctrl_pullud_array1_s10, usb_switch_ctrl_drv_array1_s10},
	[GPIO] = {usb_switch_ctrl_func_gpio_s10, usb_switch_ctrl_pullud_array1_s10, usb_switch_ctrl_drv_array1_s10},
	[LOWPOWER] = {usb_switch_ctrl_func_gpio_s10, usb_switch_ctrl_pullud_array1_s10, usb_switch_ctrl_drv_array1_s10},
};

/* u4, u5 */
enum lowlayer_func minor_version_func_gpio_s10[] = {FUNC1, FUNC1, -INVALID,};
enum pull_updown minor_version_pullud_array1_s10[] = {NOPULL, NOPULL, -INVALID,};
enum pull_updown minor_version_pullud_array2_s10[] = {PULLDOWN, PULLDOWN, -INVALID,};
enum drive_strength minor_version_drv_array1_s10[] = {LEVEL0, LEVEL0, -INVALID,};
struct block_config minor_version_config_s10[] = {
	[NORMAL] = {minor_version_func_gpio_s10, minor_version_pullud_array1_s10, minor_version_drv_array1_s10},
	[GPIO] = {minor_version_func_gpio_s10, minor_version_pullud_array1_s10, minor_version_drv_array1_s10},
	[LOWPOWER] = {minor_version_func_gpio_s10, minor_version_pullud_array2_s10, minor_version_drv_array1_s10},
};
////////////////////////////////////////////////////////////////////////////////////////////////

struct block_table block_config_pad_s10_es[] = {
	BLOCK_CONFIG(IO_I2C0_BLOCK_NAME, &block_i2c0, i2c0_config)    // "block_i2c0"
	BLOCK_CONFIG(IO_I2C1_BLOCK_NAME, &block_i2c1, i2c1_config)    // "block_i2c1"
	//BLOCK_CONFIG(IO_I2C2_BLOCK_NAME, &block_i2c2_cs, i2c2_cs_config)
	//BLOCK_CONFIG(IO_I2C3_BLOCK_NAME, &block_i2c3, i2c3_config)
	BLOCK_CONFIG(IO_SPI0_BLOCK_NAME, &block_spi0, spi0_config)    // "block_spi0"
	BLOCK_CONFIG(IO_SPI0_CS_BLOCK_NAME, &block_spi0_cs_s10, spi0_cs_config_s10)  // "block_spi0_cs"
	BLOCK_CONFIG(IO_SPI1_BLOCK_NAME, &block_spi1, spi1_config)    // "block_spi1"
	BLOCK_CONFIG(IO_UART0_BLOCK_NAME, &block_uart0, uart0_config)  // "block_uart0"
	BLOCK_CONFIG(IO_UART1_BLOCK_NAME, &block_uart1, uart1_config)  // "block_uart1"
	BLOCK_CONFIG(IO_UART2_BLOCK_NAME, &block_uart2, uart2_config_s10)  // "block_uart2"
	BLOCK_CONFIG(IO_UART3_BLOCK_NAME, &block_uart3, uart3_config)  // "block_uart3"
	BLOCK_CONFIG(IO_UART4_BLOCK_NAME, &block_uart4, uart4_config)  // "block_uart4"
	BLOCK_CONFIG(IO_KPC_BLOCK_NAME, &block_kpc_s10, kpc_config_s10)  // "block_kpc"
	BLOCK_CONFIG(IO_EMMC_BLOCK_NAME, &block_emmc, emmc_config)    // "block_emmc"
	BLOCK_CONFIG(IO_SD_BLOCK_NAME, &block_sd, sd_config)    // "block_sd"
//	BLOCK_CONFIG(IO_NAND_BLOCK_NAME, &block_nand, nand_config)    // "block_nand"
	BLOCK_CONFIG(IO_SDIO_BLOCK_NAME, &block_sdio, sdio_config)    // "block_sdio"
	BLOCK_CONFIG(IO_BTPM_BLOCK_NAME, &block_btpm, btpm_config_s10)    // "block_btpm"
	BLOCK_CONFIG(IO_BTPWR_BLOCK_NAME, &block_btpwr, btpwr_config)  // "block_btpwr"
	BLOCK_CONFIG(IO_PCM_BLOCK_NAME, &block_pcm_s10_es, pcm_config_s10_es)  // "block_pcm"
	BLOCK_CONFIG(IO_GPS_CELLGUIDE_BLOCK_NAME, &block_gps_cellguide_s10, gps_cellguide_config_s10)  // "block_gps_cellguide"
	BLOCK_CONFIG(IO_GPS_BOARDCOM_BLOCK_NAME, &block_gps_boardcom, gps_boardcom_config)    // "block_gps_boardcom"
	BLOCK_CONFIG(IO_TS_BLOCK_NAME, &block_ts_s10_es, ts_config_s10_es)    // "block_ts"
	BLOCK_CONFIG(IO_LCD_BLOCK_NAME, &block_lcd, lcd_config)  // "block_lcd"
	BLOCK_CONFIG(IO_PWM_BLOCK_NAME, &block_pwm_s10, pwm_config_s10)  // "block_pwm"
	BLOCK_CONFIG(IO_HDMI_BLOCK_NAME, &block_hdmi_s10, hdmi_config_s10)    // "block_hdmi"
	BLOCK_CONFIG(IO_WIFI_BLOCK_NAME, &block_wifi, wifi_config)    // "block_wifi"
	/*the following blocks are defined for camera*/
	BLOCK_CONFIG(IO_ISP_DVP_BLOCK_NAME, &block_isp_dvp, isp_dvp_config)  // "block_isp_dvp"
	BLOCK_CONFIG(IO_ISP_I2C_BLOCK_NAME, &block_isp_i2c, isp_i2c_config)  // "block_isp_i2c"
	BLOCK_CONFIG(IO_ISP_RESET_BLOCK_NAME, &block_isp_reset, isp_reset_config)  // "block_isp_reset"
	BLOCK_CONFIG(IO_ISP_BLOCK_NAME, &block_isp, isp_config_s10)  // "block_isp"
	BLOCK_CONFIG(IO_ISP_FLASH_BLOCK_NAME, &block_isp_flash, isp_flash_config)  // "block_isp_flash"
	BLOCK_CONFIG(IO_CHARGER_BLOCK_NAME, &block_charger_s10_es, charger_config_s10_es)  // "block_charger"
	BLOCK_CONFIG(IO_GSENSOR_BLOCK_NAME, &block_gsensor, gsensor_config)  // "block_gsensor"
	/*
	 *TODO:
	 *there are only several blocks table defined here
	 */
#if defined(CONFIG_SUPPORT_MICRO_USB_PORT)
	BLOCK_CONFIG(IO_MICRO_USB, &block_micro_usb_link, micro_usb_config_link)
#endif

#if defined(CONFIG_FSA880_I2C)
    BLOCK_CONFIG(IO_BC11_BLOCK_NAME, &block_bc11_s10_es, bc11_config_s10_es)    // "block_bc11"
#endif
    BLOCK_CONFIG(IO_BOOST_5V_BLOCK_NAME, &block_boost_5v_s10, boost_5v_config_s10)    // "block_boost_5v"

    BLOCK_CONFIG(IO_BATTERY_GAUGE_BLOCK_NAME, &block_battery_gauge_s10, battery_gauge_config_s10)  // "block_battery_gauge"
    BLOCK_CONFIG(IO_SD_CTRL_BLOCK_NAME, &block_sd_ctrl_s10_es, sd_ctrl_config_s10_es)  // "block_sd_ctrl"
    BLOCK_CONFIG(IO_MHL_BLOCK_NAME, &block_mhl_s10_es, mhl_config_s10_es)  // "block_mhl"
    BLOCK_CONFIG(IO_CAMERA_PWR_BLOCK_NAME, &block_camera_pwr_s10_es, camera_pwr_config_s10_es)    // "block_camera_pwr"
    BLOCK_CONFIG(IO_MAIN_CAMERA_BLOCK_NAME, &block_main_camera_s10, main_camera_config_s10)  // "block_main_camera"
    BLOCK_CONFIG(IO_SLAVE_CAMERA_BLOCK_NAME, &block_slave_camera_s10, slave_camera_config_s10)    // "block_slave_camera"
    BLOCK_CONFIG(IO_MR_SENSOR_BLOCK_NAME, &block_mr_sensor_s10, mr_sensor_config_s10)  // "block_mr_sensor"
    BLOCK_CONFIG(IO_COMPASS_BLOCK_NAME, &block_compass_s10, compass_config_s10)  // "block_compass"
    BLOCK_CONFIG(IO_GYROSCOPE_BLOCK_NAME, &block_gyroscope_s10, gyroscope_config_s10)  // "block_gyroscope"
    BLOCK_CONFIG(IO_CAPACITOR_SENSOR_BLOCK_NAME, &block_capacitor_sensor_s10, capacitor_sensor_config_s10)    // "block_capacitor_sensor"
    BLOCK_CONFIG(IO_SIM_DECT_BLOCK_NAME, &block_sim_dect_s10, sim_dect_config_s10)    // "block_sim_dect"
    BLOCK_CONFIG(IO_EXTRAL_DCDC_BLOCK_NAME, &block_extral_dcdc_s10, extral_dcdc_config_s10)  // "block_extral_dcdc"
    BLOCK_CONFIG(IO_DOCK_BLOCK_NAME, &block_dock_s10_es, dock_config_s10_es)    // "block_dock"
    BLOCK_CONFIG(IO_AUDIO_SPK_BLOCK_NAME, &block_audio_spk, audio_spk_config_s10)  // "block_audio_spk"
    BLOCK_CONFIG(IO_AUDIO_EARPHONE_BLOCK_NAME, &block_audio_earphone_s10, audio_earphone_config_s10)    // "block_audio_earphone"
//    BLOCK_CONFIG(IO_MODEM_AP_CTRL_BLOCK_NAME, &block_moden_ap_ctrl_s10, modem_ap_ctrl_config_s10)  // "block_moden_ap_ctrl"
    BLOCK_CONFIG(IO_MINOR_VERSION_BLOCK_NAME, &block_minor_version_s10, minor_version_config_s10)  // "block_minor_version"
    BLOCK_CONFIG(IO_USB_SWITCH_CTRL_BLOCK_NAME, &block_usb_switch_ctrl_s10, usb_switch_ctrl_config_s10)  // "block_moden_ap_ctrl"	
	{NULL, NULL, NULL},
};

struct block_table block_config_pad_s10_cs[] = {
	BLOCK_CONFIG(IO_I2C0_BLOCK_NAME, &block_i2c0, i2c0_config)    // "block_i2c0"
	BLOCK_CONFIG(IO_I2C1_BLOCK_NAME, &block_i2c1, i2c1_config)    // "block_i2c1"
	BLOCK_CONFIG(IO_I2C2_BLOCK_NAME, &block_i2c2_cs, i2c2_cs_config)
	BLOCK_CONFIG(IO_I2C3_BLOCK_NAME, &block_i2c3, i2c3_config)
#if defined(CONFIG_GPIO_I2C_ADPT)
	BLOCK_CONFIG(IO_GPIO_I2C_BLOCK_NAME, &block_gpio_i2c, gpio_i2c_config)
#endif
	BLOCK_CONFIG(IO_SPI0_BLOCK_NAME, &block_spi0, spi0_config)    // "block_spi0"
	BLOCK_CONFIG(IO_SPI0_CS_BLOCK_NAME, &block_spi0_cs_s10, spi0_cs_config_s10)  // "block_spi0_cs"
	BLOCK_CONFIG(IO_SPI1_BLOCK_NAME, &block_spi1_cs, spi1_cs_config)    // "block_spi1"
	BLOCK_CONFIG(IO_UART0_BLOCK_NAME, &block_uart0, uart0_config)  // "block_uart0"
	BLOCK_CONFIG(IO_UART1_BLOCK_NAME, &block_uart1, uart1_config)  // "block_uart1"
	BLOCK_CONFIG(IO_UART2_BLOCK_NAME, &block_uart2, uart2_config_s10)  // "block_uart2"
	BLOCK_CONFIG(IO_UART3_BLOCK_NAME, &block_uart3, uart3_config)  // "block_uart3"
	BLOCK_CONFIG(IO_UART4_BLOCK_NAME, &block_uart4, uart4_config)  // "block_uart4"
	BLOCK_CONFIG(IO_KPC_BLOCK_NAME, &block_kpc_s10, kpc_config_s10)  // "block_kpc"
	BLOCK_CONFIG(IO_EMMC_BLOCK_NAME, &block_emmc, emmc_config)    // "block_emmc"
	BLOCK_CONFIG(IO_SD_BLOCK_NAME, &block_sd, sd_config)    // "block_sd"
//	BLOCK_CONFIG(IO_NAND_BLOCK_NAME, &block_nand, nand_config)    // "block_nand"
	BLOCK_CONFIG(IO_SDIO_BLOCK_NAME, &block_sdio, sdio_config)    // "block_sdio"
	BLOCK_CONFIG(IO_BTPM_BLOCK_NAME, &block_btpm, btpm_config_s10)    // "block_btpm"
	BLOCK_CONFIG(IO_BTPWR_BLOCK_NAME, &block_btpwr, btpwr_config)  // "block_btpwr"
	BLOCK_CONFIG(IO_PCM_BLOCK_NAME, &block_pcm_s10_cs, pcm_config_s10_cs)  // "block_pcm"
	BLOCK_CONFIG(IO_GPS_CELLGUIDE_BLOCK_NAME, &block_gps_cellguide_s10, gps_cellguide_config_s10)  // "block_gps_cellguide"
	BLOCK_CONFIG(IO_GPS_BOARDCOM_BLOCK_NAME, &block_gps_boardcom, gps_boardcom_config)    // "block_gps_boardcom"
	BLOCK_CONFIG(IO_TS_BLOCK_NAME, &block_ts_s10_cs, ts_config_s10_cs)    // "block_ts"
	BLOCK_CONFIG(IO_LCD_BLOCK_NAME, &block_lcd, lcd_config)  // "block_lcd"
	BLOCK_CONFIG(IO_PWM_BLOCK_NAME, &block_pwm_s10, pwm_config_s10)  // "block_pwm"
	BLOCK_CONFIG(IO_HDMI_BLOCK_NAME, &block_hdmi_s10, hdmi_config_s10)    // "block_hdmi"
	BLOCK_CONFIG(IO_WIFI_BLOCK_NAME, &block_wifi, wifi_config)    // "block_wifi"
	/*the following blocks are defined for camera*/
	BLOCK_CONFIG(IO_ISP_DVP_BLOCK_NAME, &block_isp_dvp, isp_dvp_config)  // "block_isp_dvp"
	BLOCK_CONFIG(IO_ISP_I2C_BLOCK_NAME, &block_isp_i2c, isp_i2c_config)  // "block_isp_i2c"
	BLOCK_CONFIG(IO_ISP_RESET_BLOCK_NAME, &block_isp_reset, isp_reset_config)  // "block_isp_reset"
	BLOCK_CONFIG(IO_ISP_BLOCK_NAME, &block_isp, isp_config_s10)  // "block_isp"
	BLOCK_CONFIG(IO_ISP_FLASH_BLOCK_NAME, &block_isp_flash, isp_flash_config)  // "block_isp_flash"
	BLOCK_CONFIG(IO_CHARGER_BLOCK_NAME, &block_charger_s10_cs, charger_config_s10_cs)  // "block_charger"
	BLOCK_CONFIG(IO_GSENSOR_BLOCK_NAME, &block_gsensor, gsensor_config)  // "block_gsensor"
	/*
	 *TODO:
	 *there are only several blocks table defined here
	 */
#if defined(CONFIG_SUPPORT_MICRO_USB_PORT)
	BLOCK_CONFIG(IO_MICRO_USB, &block_micro_usb_link, micro_usb_config_link)
#endif

#if defined(CONFIG_FSA880_I2C)
    BLOCK_CONFIG(IO_BC11_BLOCK_NAME, &block_bc11_s10_cs, bc11_config_s10_cs)    // "block_bc11"
#endif
    BLOCK_CONFIG(IO_BOOST_5V_BLOCK_NAME, &block_boost_5v_s10, boost_5v_config_s10)    // "block_boost_5v"

    BLOCK_CONFIG(IO_BATTERY_GAUGE_BLOCK_NAME, &block_battery_gauge_s10, battery_gauge_config_s10)  // "block_battery_gauge"
    BLOCK_CONFIG(IO_SD_CTRL_BLOCK_NAME, &block_sd_ctrl_s10_cs, sd_ctrl_config_s10_cs)  // "block_sd_ctrl"
    BLOCK_CONFIG(IO_MHL_BLOCK_NAME, &block_mhl_s10_cs, mhl_config_s10_cs)  // "block_mhl"
    BLOCK_CONFIG(IO_CAMERA_PWR_BLOCK_NAME, &block_camera_pwr_s10_cs, camera_pwr_config_s10_cs)    // "block_camera_pwr"
    BLOCK_CONFIG(IO_MAIN_CAMERA_BLOCK_NAME, &block_main_camera_s10, main_camera_config_s10)  // "block_main_camera"
    BLOCK_CONFIG(IO_SLAVE_CAMERA_BLOCK_NAME, &block_slave_camera_s10, slave_camera_config_s10)    // "block_slave_camera"
    BLOCK_CONFIG(IO_MR_SENSOR_BLOCK_NAME, &block_mr_sensor_s10, mr_sensor_config_s10)  // "block_mr_sensor"
    BLOCK_CONFIG(IO_COMPASS_BLOCK_NAME, &block_compass_s10, compass_config_s10)  // "block_compass"
    BLOCK_CONFIG(IO_GYROSCOPE_BLOCK_NAME, &block_gyroscope_s10, gyroscope_config_s10)  // "block_gyroscope"
    BLOCK_CONFIG(IO_CAPACITOR_SENSOR_BLOCK_NAME, &block_capacitor_sensor_s10, capacitor_sensor_config_s10)    // "block_capacitor_sensor"
    BLOCK_CONFIG(IO_SIM_DECT_BLOCK_NAME, &block_sim_dect_s10, sim_dect_config_s10)    // "block_sim_dect"
    BLOCK_CONFIG(IO_EXTRAL_DCDC_BLOCK_NAME, &block_extral_dcdc_s10, extral_dcdc_config_s10)  // "block_extral_dcdc"
    BLOCK_CONFIG(IO_DOCK_BLOCK_NAME, &block_dock_s10_cs, dock_config_s10_cs)    // "block_dock"
    BLOCK_CONFIG(IO_AUDIO_SPK_BLOCK_NAME, &block_audio_spk, audio_spk_config_s10)  // "block_audio_spk"
    BLOCK_CONFIG(IO_AUDIO_EARPHONE_BLOCK_NAME, &block_audio_earphone_s10, audio_earphone_config_s10)    // "block_audio_earphone"
    BLOCK_CONFIG(IO_MODEM_AP_CTRL_BLOCK_NAME, &block_moden_ap_ctrl_s10, modem_ap_ctrl_config_s10)  // "block_moden_ap_ctrl"
    BLOCK_CONFIG(IO_MINOR_VERSION_BLOCK_NAME, &block_minor_version_s10, minor_version_config_s10)  // "block_minor_version"
    BLOCK_CONFIG(IO_USB_SWITCH_CTRL_BLOCK_NAME, &block_usb_switch_ctrl_s10, usb_switch_ctrl_config_s10)  // "block_moden_ap_ctrl"	
	{NULL, NULL, NULL},
};