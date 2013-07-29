#ifndef __MACH_K3V2_IOMUX_BLOCKS_H
#define __MACH_K3V2_IOMUX_BLOCKS_H

#include "iomux.h"
#include "k3v2_iomux_pins.h"
#include <hsad/config_interface.h>

extern struct iomux_ops iomux_block_ops;

/*the bins array which every block contained*/
/*i2c0 pins :I2C0_SCL,I2C0_SDA*/
struct  iomux_pin *i2c0_pins[] = {
	&p25, &r25, NULL,
};

/*i2c1:I2C1_SCL,I2C1_SDA*/
struct  iomux_pin *i2c1_pins[] = {
	&n25, &p26, NULL, };

/*i2c2 pins :I2C2_SCL,I2C2_SDA*/
struct  iomux_pin *i2c2_pins[] = {
	&d20, &d22, NULL,
};


/*i2c2  for cs pins :I2C2_SCL,I2C2_SDA*/
struct  iomux_pin *i2c2_cs_pins[] = {
	&d20_cs, &d22_cs, NULL,
};

/*i2c3:I2C3_SCL,I2C3_SDA*/
struct  iomux_pin *i2c3_pins[] = {
	&d24, &a23, NULL,
};

/*spi0:SPI0_CLK,SPI0_DI,SPI0_DI*/
struct  iomux_pin *spi0_pins[] = {
	&n29, &n26, &m29, NULL,
};

/*spi0_cs:SPI0_CS0_N,SPI0_CS1_N,SPI0_CS2_N,SPI0_CS3_N*/
struct  iomux_pin *spi0_cs_pins[] = {
	&l29, &m21, &n28, &m23, NULL,
};

/*spi1:SPI1_CLK,SPI1_CS_N,SPI1_DI,SPI1_DO*/
struct  iomux_pin *spi1_pins[] = {
	&d1, &e1, &d2, &e2, NULL,
};

/*spi1_cs:SPI1_CLK,SPI1_CS_N,SPI1_DI,SPI1_DO*/
struct  iomux_pin *spi1_cs_pins[] = {
	&d1_cs, &e1, &d2_cs, &e2, NULL,
};

/*uart0:UART0_RXD,UART0_TXD.and UART0_CTS_N,UART0_RTS_N are used by asp*/
struct  iomux_pin *uart0_pins[] = {
	/*&a27, &b26,*/ &a26, &a25, NULL,
};

/*uart1:UART1_CTS_N,UART1_RTS_N,UART1_RXD,UART1_TXD*/
struct  iomux_pin *uart1_pins[] = {
	&ae28, &af29, &af28, &ag28, NULL,
};

/*onewire, usim_rst,
 * usim_clk and usim_data is used as sensor pins
 */
struct  iomux_pin *uart2_pins[] = {
	&ac26, /*&ac28, &aa28,*/ &aa29, NULL,
};

/*uart3:UART3_CTS_N,UART3_RTS_N,UART3_RXD,UART3_TXD*/
struct  iomux_pin *uart3_pins[] = {
	&l28, &k28, &k26, &m28, NULL,
};

/*uart4:UART4_CTS_N, UART4_RXD, UART4_RTS_N,UART4_TXD*/
struct  iomux_pin *uart4_pins[] = {
	&g29, &l23, &k29, &j23, NULL,
};

/*kpc key pad  3X3:keypad_in0~keypad_in2, keypad_out0~keypad_out2*/
struct  iomux_pin *kpc_pins[] = {
	&aj21, &ah21, &af22, &aj24, &ah24, &ah25, NULL,
};


/*emmc pins:
 *nand_cs3_n,nand_busy2_n,nand_busy3_n,nand_data8,nand_data9,nand_data10,nand_data11,
 *nand_data12,nand_data13,nand_data14,nand_data15,emmc_cmd,emmc_clk
 */
struct  iomux_pin *emmc_pins[] = {
	&ab1, &ac5, &ac1, &ac4, &ae2, &ad2,\
    &ad4, &ad5, &ae4, &ae1, NULL,
};

/*
 *sd_clk, sd_cmd, sd_data0, sd_data1, sd_data2, sd_data3
 */
struct  iomux_pin *sd_pins[] = {
	&ag2, &af5, &ae5, &af4, &af6, &ae6, NULL,
};

/*nand pins:
 *nand_ale,nand_busy0_n,nand_busy1_n,nand_busy2_n,nand_busy3_n,nand_cle,nand_cs0_n,
 *nand_cs1_n,nand_cs2_n,nand_cs3_n,nand_data0,nand_data1,nand_data2,nand_data3,nand_data4,
 *nand_data5,nand_data6,nand_data7,nand_data8,nand_data9,nand_data10,nand_data11,
 *nand_data12,nand_data13,nand_data14,nand_data15,nand_re_n,nand_we_n
 */
struct  iomux_pin *nand_pins[] = {
	&v2, &v5, &w4, &v4, &w5, &w2, &u5, &u4, &u2, &v1, &u1, &aa2, &ab2, &y4, &aa4, \
	&aa5, &ab4, &ab5, &ab1, &ac5, &ac1, &ac4, &ae2, &ad2, &ad4, &ad5, &y5, &w1, NULL,
};

/*sdio:SDIO1_CLK,SDIO1_CMD,SDIO1_DATA0,SDIO1_DATA1,SDIO1_DATA2,SDIO1_DATA3*/
struct  iomux_pin *sdio_pins[] = {
	&e25, &e28, &e29, &g26, &d29, &e26, NULL,
};

/*bt, g28 is used as adc interrupt pin
 *BT_SPI_CLK,BT_SPI_CS_N,BT_SPI_DATA,BT_ENABLE_RM
 */
/*btpm*/
struct  iomux_pin *btpm_pins[] = {
	&j26, &k25, NULL,
};

/*btpm for cs*/
struct  iomux_pin *btpm_cs_pins[] = {
	&j26_cs, &k25_cs, NULL,
};

/*btpwr*/
struct  iomux_pin *btpwr_pins[] = {
	&j25, &h28, NULL,
};

/*btpwr for cs*/
struct  iomux_pin *btpwr_cs_pins[] = {
	&j25_cs, &h28_cs, NULL,
};

/*gps for cellguide*/
struct  iomux_pin *gps_cellguide_pins[] = {
	&m26, &l25, &j28, &l28, &k28, &k26, &m28, &m23, NULL,
};

/*gps for boardcom*/
struct  iomux_pin *gps_boardcom_pins[] = {
	&j28, &m26, NULL,
};

/*touch screen:GPIO_156,GPIO_157*/
struct  iomux_pin *ts_pins[] = {
	&aa23, &v29, NULL,
};

/*lcd pins:
 *KEYPAD_OUT6,KEYPAD_OUT7,EFUSE_CSB,RFTCXO_PWR
 */
#if defined(CONFIG_LCD_PANASONIC_VVX10F002A00)
struct  iomux_pin *lcd_pins[] = {
	&aj27, &af25, &af26, &k23, NULL,
};

/*pwm pins:
 *PWM_OUT0, PWM_OUT1
 */
struct  iomux_pin *pwm_pins[] = {
	&d28, &b27, NULL,
};

#elif defined(CONFIG_LCD_QIMEI_HJ101IA_01F_10)
struct  iomux_pin *lcd_pins[] = {
	&af25, &af26, &b27, &v5, &w4, NULL,
};

struct  iomux_pin *pwm_pins[] = {
	&d28, NULL,
};
#endif

/*hdmi pins:
 *HDMI_SCL, HDMI_SDA, HDMI_CEC, HDMI_CEC
 */
struct  iomux_pin *hdmi_pins[] = {
	&e12, &d12, &e11, &d11, NULL,
};

/*wifi pins:
 *EFUSE_SEL, EFUSE_SCLK
 */
struct  iomux_pin *wifi_pins[] = {
	&ah27, &ad25, NULL,
};

/*dvp pins:
 *cam_data0, cam_data1, cam_data2, cam_data3, cam_data4, cam_data5, cam_data6, cam_data7,
 *cam_hysnc, cam_pclk, cam_vsync.
 *d15(cam_data8) is used as other pin
 *a14(cam_data9) is used as gsensor pin
 */
struct  iomux_pin *isp_dvp[] = {
	&e16, &b18, &b17, &a17, &d17, &e15, &b16, &d16, /*&d15, &a14,*/ &d13, &e13, &b14, NULL,
};

/*i2c isp pins:
 *isp_scl0, isp_sda0.
 */
struct  iomux_pin *isp_i2c_pins[] = {
	&d23, &b23, NULL,
};

/*isp_reset
 *isp_resetb0, isp_resetb1
 */
 struct  iomux_pin *isp_reset_pins[] = {
	&g20, &e22, NULL,
};

/*isp pins:
 *isp_cclk0, isp_cclk2, isp_gpio0,
 *isp_gpio1, isp_gpio2, isp_gpio3, isp_gpio4,  isp_gpio6,
 *and isp_gpio8, isp_gpio9 are used as gpio,don't need to mux
 *isp_gpio5 is used as gpio by charger
 */
struct  iomux_pin *isp_pins[] = {
	&a22, &e18, &b21, &g18, &d18, &b20, &a21, /*&a20,*/ &b19, NULL,
};

/*isp flash light:isp_strobe0, isp_strobe1, isp_gpio7*/
struct  iomux_pin *isp_flash_pins[] = {
	&e20, &d21, &e17, NULL,
};

/*isp flash light for cs:isp_strobe0, isp_strobe1, isp_gpio7*/
struct  iomux_pin *isp_flash_cs_pins[] = {
	&e20, &d21, &e17_cs, NULL,
};

/*charger pins:
 *EFUSE_PGM, ISP_GPIO5
 */
struct  iomux_pin *charger_pins[] = {
	&ac25, &a20, NULL,
};

/*gsensor pins:
 *CAM_DATA9, TBC_DOWN,
 */
struct  iomux_pin *gsensor_pins[] = {
	&a14, &ab25, NULL,
};

/*
 *audio es305 pins:
 *USIM_RST,ONEWIRE
 */
struct  iomux_pin *audio_es305_pins[] = {
	&aa29, &ac26, NULL,
};

/*
 *audio speaker pin:
 *UART0_CTS_N
 */
struct  iomux_pin *audio_spk_pins[] = {
	&a27, NULL,
};

/*
 *audio earphone pin:
 *UART0_RTS_N
 */
struct  iomux_pin *audio_eph_pins[] = {
	&b26, NULL,
};

/*
 *modem pins:
 *SDIO0_CLK(gpio_094), SDIO0_CMD(gpio_095), SDIO0_DATA0(gpio_096), SDIO0_DATA1(gpio_097),
 *SDIO0_DATA2(gpio_098), SPI1_CLK(gpio_113), SPI1_DI(gpio_114), SPI1_DO(gpio_115)
 */
struct  iomux_pin *modem_pins[] = {
	&g1, &f4, &f2, &f1, &g2, &d1, &d2, &e2, NULL,
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
IOMUX_BLOCK(block_i2c0, "block_i2c0", NORMAL, i2c0_pins)
IOMUX_BLOCK(block_i2c1, "block_i2c1", NORMAL, i2c1_pins)
IOMUX_BLOCK(block_i2c2, "block_i2c2", NORMAL, i2c2_pins)
IOMUX_BLOCK(block_i2c2_cs, "block_i2c2", NORMAL, i2c2_cs_pins)
IOMUX_BLOCK(block_i2c3, "block_i2c3", NORMAL, i2c3_pins)
IOMUX_BLOCK(block_spi0, "block_spi0", NORMAL, spi0_pins)
IOMUX_BLOCK(block_spi0_cs, "block_spi0_cs", NORMAL, spi0_cs_pins)
IOMUX_BLOCK(block_spi1, "block_spi1", NORMAL, spi1_pins)
IOMUX_BLOCK(block_spi1_cs, "block_spi1", NORMAL, spi1_cs_pins)
IOMUX_BLOCK(block_uart0, "block_uart0", NORMAL, uart0_pins)
IOMUX_BLOCK(block_uart1, "block_uart1", NORMAL, uart1_pins)
IOMUX_BLOCK(block_uart2, "block_uart2", NORMAL, uart2_pins)
IOMUX_BLOCK(block_uart3, "block_uart3", NORMAL, uart3_pins)
IOMUX_BLOCK(block_uart4, "block_uart4", NORMAL, uart4_pins)
IOMUX_BLOCK(block_kpc, "block_kpc", NORMAL, kpc_pins)
IOMUX_BLOCK(block_emmc, "block_emmc", NORMAL, emmc_pins)
IOMUX_BLOCK(block_nand, "block_nand", NORMAL, nand_pins)
IOMUX_BLOCK(block_sd, "block_sd", NORMAL, sd_pins)
IOMUX_BLOCK(block_sdio, "block_sdio", NORMAL, sdio_pins)
IOMUX_BLOCK(block_btpm, "block_btpm", NORMAL, btpm_pins)
IOMUX_BLOCK(block_btpm_cs, "block_btpm", NORMAL, btpm_cs_pins)
IOMUX_BLOCK(block_btpwr, "block_btwr", NORMAL, btpwr_pins)
IOMUX_BLOCK(block_btpwr_cs, "block_btwr", NORMAL, btpwr_cs_pins)
IOMUX_BLOCK(block_gps_cellguide, "block_gps_cellguide", NORMAL, gps_cellguide_pins)
IOMUX_BLOCK(block_gps_boardcom, "block_gps_boardcom", NORMAL, gps_boardcom_pins)
IOMUX_BLOCK(block_ts, "block_ts", NORMAL, ts_pins)
IOMUX_BLOCK(block_lcd, "block_lcd", NORMAL, lcd_pins)
IOMUX_BLOCK(block_pwm, "block_pwm", NORMAL, pwm_pins)
IOMUX_BLOCK(block_hdmi, "block_hdmi", NORMAL, hdmi_pins)
IOMUX_BLOCK(block_wifi, "block_wifi", NORMAL, wifi_pins)
IOMUX_BLOCK(block_isp_dvp, "block_isp_dvp", NORMAL, isp_dvp)
IOMUX_BLOCK(block_isp_i2c, "block_isp_i2c", NORMAL, isp_i2c_pins)
IOMUX_BLOCK(block_isp_reset, "block_isp_reset", NORMAL, isp_reset_pins)
IOMUX_BLOCK(block_isp, "block_isp", NORMAL, isp_pins)
IOMUX_BLOCK(block_isp_flash, "block_isp_flash", NORMAL, isp_flash_pins)
IOMUX_BLOCK(block_isp_flash_cs, "block_isp_flash", NORMAL, isp_flash_cs_pins)
IOMUX_BLOCK(block_charger, "block_charger", NORMAL, charger_pins)
IOMUX_BLOCK(block_gsensor, "block_gsensor", NORMAL, gsensor_pins)
IOMUX_BLOCK(block_audio_es305, "block_audio_es305", NORMAL, audio_es305_pins)
IOMUX_BLOCK(block_audio_spk, "block_audio_spk", NORMAL, audio_spk_pins)
IOMUX_BLOCK(block_audio_eph, "block_audio_eph", NORMAL, audio_eph_pins)
IOMUX_BLOCK(block_modem, "block_modem", NORMAL, modem_pins)

/*define config value of each block*/
/*i2c0 pins :I2C0_SCL,I2C0_SDA*/
enum lowlayer_func i2c0_func_array1[] = {FUNC0, FUNC0, -INVALID,};
enum lowlayer_func i2c0_func_array2[] = {FUNC1, FUNC1, -INVALID,};
enum pull_updown i2c0_pullud_array1[] = {NOPULL, NOPULL, -INVALID,};
enum drive_strength i2c0_drv_array1[] = {LEVEL2, LEVEL2,  -INVALID,};
struct block_config i2c0_config[] = {
	[NORMAL] = {i2c0_func_array1, i2c0_pullud_array1, i2c0_drv_array1},
	[GPIO] = {i2c0_func_array2, i2c0_pullud_array1, i2c0_drv_array1},
	[LOWPOWER] = {i2c0_func_array2, i2c0_pullud_array1, i2c0_drv_array1},
};

/*i2c1:I2C1_SCL,I2C1_SDA*/
enum lowlayer_func i2c1_func_array1[] = {FUNC0, FUNC0, -INVALID,};
enum lowlayer_func i2c1_func_array2[] = {FUNC1, FUNC1, -INVALID,};
enum pull_updown i2c1_pullud_array1[] = {NOPULL, NOPULL, -INVALID,};
enum drive_strength i2c1_drv_array1[] = {LEVEL2, LEVEL2,  -INVALID,};
struct block_config i2c1_config[] = {
	[NORMAL] = {i2c1_func_array1, i2c1_pullud_array1, i2c1_drv_array1},
	[GPIO] = {i2c1_func_array2, i2c1_pullud_array1, i2c1_drv_array1},
	[LOWPOWER] = {i2c1_func_array2, i2c1_pullud_array1, i2c1_drv_array1},
};

/*i2c2 pins :I2C2_SCL,I2C2_SDA*/
enum lowlayer_func i2c2_func_array1[] = {FUNC0, FUNC0, -INVALID,};
enum lowlayer_func i2c2_func_array2[] = {FUNC1, FUNC1, -INVALID,};
enum pull_updown i2c2_pullud_array1[] = {NOPULL, NOPULL, -INVALID,};
enum drive_strength i2c2_drv_array1[] = {LEVEL2, LEVEL2,  -INVALID,};
struct block_config i2c2_config[] = {
	[NORMAL] = {i2c2_func_array1, i2c2_pullud_array1, i2c2_drv_array1},
	[GPIO] = {i2c2_func_array2, i2c2_pullud_array1, i2c2_drv_array1},
	[LOWPOWER] = {i2c2_func_array2, i2c2_pullud_array1, i2c0_drv_array1},
};

/*i2c2 pins :I2C2_SCL,I2C2_SDA*/
enum lowlayer_func i2c2_func_array1_cs[] = {FUNC0, FUNC0, -INVALID,};
enum lowlayer_func i2c2_func_array2_cs[] = {FUNC1, FUNC1, -INVALID,};
enum pull_updown i2c2_pullud_array1_cs[] = {NOPULL, NOPULL, -INVALID,};
enum drive_strength i2c2_drv_array1_cs[] = {LEVEL2, LEVEL2,  -INVALID,};
struct block_config i2c2_cs_config[] = {
	[NORMAL] = {i2c2_func_array1_cs, i2c2_pullud_array1_cs, i2c2_drv_array1_cs},
	[GPIO] = {i2c2_func_array2_cs, i2c2_pullud_array1_cs, i2c2_drv_array1_cs},
	[LOWPOWER] = {i2c2_func_array2_cs, i2c2_pullud_array1_cs, i2c2_drv_array1_cs},
};

/*i2c1:I2C3_SCL,I2C3_SDA*/
enum lowlayer_func i2c3_func_array1[] = {FUNC2, FUNC2, -INVALID,};
enum lowlayer_func i2c3_func_array2[] = {FUNC1, FUNC1, -INVALID,};
enum pull_updown i2c3_pullud_array1[] = {NOPULL, NOPULL, -INVALID,};
enum drive_strength i2c3_drv_array1[] = {LEVEL2, LEVEL2,  -INVALID,};
struct block_config i2c3_config[] = {
	[NORMAL] = {i2c3_func_array1, i2c3_pullud_array1, i2c3_drv_array1},
	[GPIO] = {i2c3_func_array2, i2c3_pullud_array1, i2c3_drv_array1},
	[LOWPOWER] = {i2c3_func_array2, i2c3_pullud_array1, i2c3_drv_array1},
};

/*spi0:SPI0_CLK,SPI0_DI,SPI0_DI*/
enum lowlayer_func spi0_func_array1[] = {FUNC0, FUNC0, FUNC0,  -INVALID,};
enum lowlayer_func spi0_func_array2[] = {FUNC1, FUNC1, FUNC1, -INVALID,};
enum pull_updown spi0_pullud_array1[] = {PULLDOWN, PULLDOWN, PULLDOWN,  -INVALID,};
enum drive_strength spi0_drv_array1[] = {LEVEL2, LEVEL2, LEVEL2, -INVALID,};
struct block_config spi0_config[] = {
	[NORMAL] = {spi0_func_array1, spi0_pullud_array1, spi0_drv_array1},
	[GPIO] = {spi0_func_array2, spi0_pullud_array1, spi0_drv_array1},
	[LOWPOWER] = {spi0_func_array2, spi0_pullud_array1, spi0_drv_array1},
};

/*spi0_cs:SPI0_CS0_N,SPI0_CS1_N,SPI0_CS2_N,SPI0_CS3_N*/
enum lowlayer_func spi0_cs_func_array1[] = {FUNC0, FUNC0, FUNC0, FUNC0,  -INVALID,};
enum lowlayer_func spi0_cs_func_array2[] = {FUNC1, FUNC1, FUNC1, FUNC1,  -INVALID,};
enum pull_updown spi0_cs_pullud_array1[] = {PULLDOWN, PULLDOWN, NOPULL, NOPULL, -INVALID,};
enum drive_strength spi0_cs_drv_array1[] = {LEVEL2, LEVEL2, LEVEL2, LEVEL2, -INVALID,};
struct block_config spi0_cs_config[] = {
	[NORMAL] = {spi0_cs_func_array1, spi0_cs_pullud_array1, spi0_cs_drv_array1},
	[GPIO] = {spi0_cs_func_array2, spi0_cs_pullud_array1, spi0_cs_drv_array1},
	[LOWPOWER] = {spi0_cs_func_array2, spi0_cs_pullud_array1, spi0_cs_drv_array1},
};

/*spi1:SPI1_CLK,SPI1_CS_N,SPI1_DI,SPI1_DO*/
enum lowlayer_func spi1_func_array1[] = {FUNC0, FUNC0, FUNC0, FUNC0, -INVALID,};
enum lowlayer_func spi1_func_array2[] = {FUNC1, FUNC1, FUNC1, FUNC1, -INVALID,};
enum pull_updown spi1_pullud_array1[] = {PULLUP, PULLDOWN, NOPULL, NOPULL, -INVALID,};
enum pull_updown spi1_pullud_array2[] = {PULLUP, PULLDOWN, NOPULL, NOPULL, -INVALID,};
enum drive_strength spi1_drv_array1[] = {LEVEL2, LEVEL2, LEVEL2, LEVEL2, -INVALID,};
struct block_config spi1_config[] = {
	[NORMAL] = {spi1_func_array1, spi1_pullud_array1, spi1_drv_array1},
	[GPIO] = {spi1_func_array2, spi1_pullud_array2, spi1_drv_array1},
	[LOWPOWER] = {spi1_func_array2, spi1_pullud_array2, spi1_drv_array1},
};


/*spi1_cs:SPI1_CLK,SPI1_CS_N,SPI1_DI,SPI1_DO*/
enum lowlayer_func spi1_cs_func_array1[] = {FUNC0, FUNC0, FUNC0, FUNC0, -INVALID,};
enum lowlayer_func spi1_cs_func_array2[] = {FUNC1, FUNC1, FUNC1, FUNC1, -INVALID,};
enum pull_updown spi1_cs_pullud_array1[] = {PULLUP, PULLDOWN, NOPULL, NOPULL, -INVALID,};
enum pull_updown spi1_cs_pullud_array2[] = {PULLUP, PULLDOWN, NOPULL, NOPULL, -INVALID,};
enum drive_strength spi1_cs_drv_array1[] = {LEVEL2, LEVEL2, LEVEL2, LEVEL2, -INVALID,};
struct block_config spi1_cs_config[] = {
	[NORMAL] = {spi1_cs_func_array1, spi1_cs_pullud_array1, spi1_cs_drv_array1},
	[GPIO] = {spi1_cs_func_array2, spi1_cs_pullud_array2, spi1_cs_drv_array1},
	[LOWPOWER] = {spi1_cs_func_array2, spi1_cs_pullud_array2, spi1_cs_drv_array1},
};


/*uart0:UART0_RXD,UART0_TXD.and UART0_CTS_N,UART0_RTS_N are used by asp*/
enum lowlayer_func uart0_func_array1[] = {FUNC0, FUNC0, -INVALID,};
enum lowlayer_func uart0_func_array2[] = {FUNC1, FUNC1, -INVALID,};
enum pull_updown uart0_pullud_array1[] = {NOPULL, NOPULL, -INVALID,};
enum pull_updown uart0_pullud_array2[] = {PULLDOWN, PULLDOWN, -INVALID,};
enum drive_strength uart0_drv_array1[] = {RESERVE, RESERVE, -INVALID,};
struct block_config uart0_config[] = {
	[NORMAL] = {uart0_func_array1, uart0_pullud_array1, uart0_drv_array1},
	[GPIO] = {uart0_func_array2, uart0_pullud_array2, uart0_drv_array1},
	[LOWPOWER] = {uart0_func_array2, uart0_pullud_array2, uart0_drv_array1},
};

/*uart1:UART1_CTS_N,UART1_RTS_N,UART1_RXD,UART1_TXD*/
enum lowlayer_func uart1_func_array1[] = {FUNC0, FUNC0, FUNC0, FUNC0, -INVALID,};
enum lowlayer_func uart1_func_array2[] = {FUNC1, FUNC1, FUNC1, FUNC1, -INVALID,};
enum pull_updown uart1_pullud_array1[] = {NOPULL, NOPULL, NOPULL, NOPULL, -INVALID,};
enum pull_updown uart1_pullud_array2[] = {PULLDOWN, PULLDOWN, PULLDOWN, PULLDOWN, -INVALID,};
enum drive_strength uart1_drv_array1[] = {RESERVE, RESERVE, RESERVE, RESERVE, -INVALID,};
struct block_config uart1_config[] = {
	[NORMAL] = {uart1_func_array1, uart1_pullud_array1, uart1_drv_array1},
	[GPIO] = {uart1_func_array2, uart1_pullud_array2, uart1_drv_array1},
	[LOWPOWER] = {uart1_func_array2, uart1_pullud_array2, uart1_drv_array1},
};

/*uart2:onewire, usim_rst*/
enum lowlayer_func uart2_func_array1[] = {FUNC2, FUNC2, -INVALID,};
enum lowlayer_func uart2_func_array2[] = {FUNC1, FUNC1, -INVALID,};
enum pull_updown uart2_pullud_array1[] = {NOPULL, NOPULL, -INVALID,};
enum pull_updown uart2_pullud_array2[] = {PULLDOWN, NOPULL, -INVALID,};
enum drive_strength uart2_drv_array1[] = {RESERVE, RESERVE, -INVALID,};
struct block_config uart2_config[] = {
	[NORMAL] = {uart2_func_array1, uart2_pullud_array1, uart2_drv_array1},
	[GPIO] = {uart2_func_array2, uart2_pullud_array1, uart2_drv_array1},
	[LOWPOWER] = {uart2_func_array2, uart2_pullud_array2, uart2_drv_array1},
};

/*uart3:UART3_CTS_N,UART3_RTS_N,UART3_RXD,UART3_TXD*/
enum lowlayer_func uart3_func_array1[] = {FUNC2, FUNC2, FUNC2, FUNC2, -INVALID,};
enum lowlayer_func uart3_func_array2[] = {FUNC1, FUNC1, FUNC1, FUNC1, -INVALID,};
enum pull_updown uart3_pullud_array1[] = {NOPULL, NOPULL, NOPULL, NOPULL, -INVALID,};
enum pull_updown uart3_pullud_array2[] = {PULLDOWN, PULLDOWN, PULLDOWN, PULLDOWN, -INVALID,};
enum drive_strength uart3_drv_array1[] = {LEVEL2, RESERVE, LEVEL2, LEVEL2, -INVALID,};
struct block_config uart3_config[] = {
	[NORMAL] = {uart3_func_array1, uart3_pullud_array1, uart3_drv_array1},
	[GPIO] = {uart3_func_array2, uart3_pullud_array1, uart3_drv_array1},
	[LOWPOWER] = {uart3_func_array2, uart3_pullud_array2, uart3_drv_array1},
};

/*uart4:UART4_CTS_N, UART4_RXD, UART4_RTS_N,UART4_TXD*/
enum lowlayer_func uart4_func_array1[] = {FUNC2, FUNC2, FUNC2, FUNC2, -INVALID,};/*this will be changed in v110*/
enum lowlayer_func uart4_func_array2[] = {FUNC1, FUNC1, FUNC1, FUNC1, -INVALID,};
enum pull_updown uart4_pullud_array1[] = {NOPULL, NOPULL, NOPULL, NOPULL, -INVALID,};
enum pull_updown uart4_pullud_array2[] = {NOPULL, NOPULL, NOPULL, NOPULL, -INVALID,};/*these must be NOPULL to solve wake problem of BT*/
enum drive_strength uart4_drv_array1[] = {RESERVE, RESERVE, RESERVE, RESERVE, -INVALID,};
struct block_config uart4_config[] = {
	[NORMAL] = {uart4_func_array1, uart4_pullud_array1, uart4_drv_array1},
	[GPIO] = {uart4_func_array2, uart4_pullud_array2, uart4_drv_array1},
	[LOWPOWER] = {uart4_func_array2, uart4_pullud_array2, uart4_drv_array1},
};

/*kpc key pad  3X3:keypad_in0~keypad_in2, keypad_out0~keypad_out2*/
enum lowlayer_func kpc_func_array1[] = {FUNC0, FUNC0, FUNC0, FUNC0, FUNC0, FUNC0, -INVALID,};
enum lowlayer_func kpc_func_array2[] = {FUNC1, FUNC1, FUNC1, FUNC1, FUNC1, FUNC1, -INVALID,};
enum pull_updown kpc_pullud_array1[] = {PULLUP, PULLUP, PULLUP, PULLUP, PULLUP, PULLUP, -INVALID,};
enum pull_updown kpc_pullud_array2[] = {PULLDOWN, PULLDOWN, PULLDOWN, \
	PULLDOWN, PULLDOWN, PULLDOWN, -INVALID,};
enum drive_strength kpc_drv_array1[] = {RESERVE, RESERVE, RESERVE, RESERVE, RESERVE, RESERVE, -INVALID,};
struct block_config kpc_config[] = {
	[NORMAL] = {kpc_func_array1, kpc_pullud_array2, kpc_drv_array1},
	[GPIO] = {kpc_func_array2, kpc_pullud_array1, kpc_drv_array1},
	[LOWPOWER] = {kpc_func_array2, kpc_pullud_array2, kpc_drv_array1},
};

/*emmc pins:
 *nand_cs3_n,nand_busy2_n,nand_busy3_n,nand_data8,nand_data9,nand_data10,nand_data11,
 *nand_data12,nand_data13,nand_data14,nand_data15,emmc_cmd,emmc_clk
 */
enum lowlayer_func emmc_func_array1[] = {FUNC1, FUNC1, FUNC1, \
		FUNC1, FUNC1, FUNC1, FUNC1, FUNC1, FUNC0, FUNC0, -INVALID,};
enum lowlayer_func emmc_func_array2[] = {FUNC2, FUNC2, FUNC2, \
		FUNC2, FUNC2, FUNC2, FUNC2, FUNC2, FUNC2, FUNC2, -INVALID,};
enum pull_updown emmc_pullud_array1[] = {PULLUP, PULLUP, PULLUP, \
		PULLUP, PULLUP, PULLUP, PULLUP, PULLUP, PULLUP, NOPULL, -INVALID,};
enum pull_updown emmc_pullud_array2[] = {PULLDOWN, PULLDOWN, \
	PULLDOWN, PULLDOWN, PULLDOWN, PULLDOWN, PULLDOWN, PULLDOWN,  PULLDOWN, PULLDOWN, -INVALID,};
enum pull_updown emmc_pullud_array3[] = {NOPULL, NOPULL,\
	NOPULL, NOPULL, NOPULL, NOPULL, NOPULL, NOPULL,  NOPULL, NOPULL, -INVALID,};
enum drive_strength emmc_drv_array1[] = {LEVEL1, LEVEL1, \
	LEVEL1, LEVEL1, LEVEL1, LEVEL1, LEVEL1, LEVEL1, LEVEL1, LEVEL1, -INVALID,};
enum drive_strength emmc_drv_array2[] = {LEVEL2, LEVEL2, LEVEL2, LEVEL2, \
	LEVEL2, LEVEL2, LEVEL2, LEVEL2, LEVEL2, LEVEL2, -INVALID,};
enum drive_strength emmc_drv_array3[] = {LEVEL0, LEVEL0, LEVEL0, LEVEL0, \
	LEVEL0, LEVEL0, LEVEL0, LEVEL0, LEVEL0, LEVEL0, -INVALID,};
struct block_config emmc_config[] = {
	[NORMAL] = {emmc_func_array2, emmc_pullud_array1, emmc_drv_array2},
	[GPIO] = {emmc_func_array1, emmc_pullud_array3, emmc_drv_array1},
	[LOWPOWER] = {emmc_func_array1, emmc_pullud_array2, emmc_drv_array3},
};

/*
  *sd_clk, sd_cmd, sd_data0, sd_data1, sd_data2, sd_data3
  */
enum lowlayer_func sd_func_array1[] = {FUNC0, FUNC0, FUNC0, FUNC0, FUNC0, FUNC0, -INVALID,};
enum lowlayer_func sd_func_array2[] = {FUNC1, FUNC1, FUNC1, FUNC1, FUNC1, FUNC1, -INVALID,};
enum pull_updown sd_pullud_array1[] = {NOPULL, PULLUP, PULLUP, PULLUP, PULLUP, PULLUP, -INVALID,};
enum pull_updown sd_pullud_array2[] = {PULLDOWN, PULLDOWN, PULLDOWN, \
	PULLDOWN, PULLDOWN, PULLDOWN, -INVALID,};
enum drive_strength sd_drv_array1[] = {LEVEL2, LEVEL3, LEVEL3, LEVEL3, LEVEL3, LEVEL3, -INVALID,};
struct block_config sd_config[] = {
	[NORMAL] = {sd_func_array1, sd_pullud_array1, sd_drv_array1},
	[GPIO] = {sd_func_array2, sd_pullud_array2, sd_drv_array1},
	[LOWPOWER] = {sd_func_array2, sd_pullud_array2, sd_drv_array1},
};

/*nand pins:
 *nand_ale,nand_busy0_n,nand_busy1_n,nand_busy2_n,nand_busy3_n,
 *nand_cle,nand_cs0_n,nand_cs1_n,nand_cs2_n,nand_cs3_n,nand_data0,
 *nand_data1,nand_data2,nand_data3,nand_data4,nand_data5,nand_data6,
 *nand_data7,nand_data8,nand_data9,nand_data10,nand_data11,
 *nand_data12,nand_data13,nand_data14,nand_data15,nand_re_n,nand_we_n
 */
enum lowlayer_func nand_func_normal[] = {FUNC0, FUNC0, FUNC0, FUNC0, FUNC0, FUNC0, FUNC0, FUNC0, FUNC0, FUNC0, FUNC0, \
FUNC0, FUNC0, FUNC0, FUNC0, FUNC0, FUNC0, FUNC0, FUNC0, FUNC0, FUNC0, FUNC0, FUNC0, FUNC0, FUNC0, FUNC0, FUNC0, -INVALID,};
enum lowlayer_func nand_func_gpio[] = {FUNC1, FUNC1, FUNC1, FUNC1, FUNC1, FUNC1, FUNC1, FUNC1, FUNC1, FUNC1, FUNC1, \
FUNC1, FUNC1, FUNC1, FUNC1, FUNC1, FUNC1, FUNC1, FUNC1, FUNC1, FUNC1, FUNC1, FUNC1, FUNC1, FUNC1, FUNC1, FUNC1, -INVALID,};
enum pull_updown nand_pullud_array1[] = {
	NOPULL, NOPULL, NOPULL, NOPULL, NOPULL, NOPULL, NOPULL, NOPULL, NOPULL, NOPULL, NOPULL, NOPULL, NOPULL, NOPULL, NOPULL,
	NOPULL, NOPULL, NOPULL, NOPULL, NOPULL, NOPULL, NOPULL, NOPULL, NOPULL, NOPULL, NOPULL, NOPULL, NOPULL, -INVALID,};
enum pull_updown nand_pullud_array2[] = {
	PULLDOWN, PULLDOWN, PULLDOWN, PULLDOWN, PULLDOWN, PULLDOWN, PULLDOWN, PULLDOWN, PULLDOWN, PULLDOWN, PULLDOWN, PULLDOWN, \
	PULLDOWN, PULLDOWN, PULLDOWN, PULLDOWN, PULLDOWN, PULLDOWN, PULLDOWN, PULLDOWN, PULLDOWN, PULLDOWN, PULLDOWN, \
	PULLDOWN, PULLDOWN, PULLDOWN, PULLDOWN, PULLDOWN, -INVALID,};
enum drive_strength nand_drv_array1[] = {
	LEVEL2, RESERVE, RESERVE, RESERVE, RESERVE, LEVEL2, LEVEL2, LEVEL2, LEVEL2, LEVEL2, LEVEL2, LEVEL2, LEVEL2, LEVEL2, LEVEL2,
	LEVEL2, LEVEL2, LEVEL2, LEVEL2, LEVEL2, LEVEL2, LEVEL2, LEVEL2, LEVEL2, LEVEL2, LEVEL2, LEVEL2, LEVEL2, -INVALID,};
struct block_config nand_config[] = {
	[NORMAL] = {nand_func_normal, nand_pullud_array1, nand_drv_array1},
	[GPIO] = {nand_func_gpio, nand_pullud_array2, nand_drv_array1},
	[LOWPOWER] = {nand_func_gpio, nand_pullud_array2, nand_drv_array1},
};

/*sdio:SDIO1_CLK,SDIO1_CMD,SDIO1_DATA0,SDIO1_DATA1,SDIO1_DATA2,SDIO1_DATA3*/
enum lowlayer_func sdio_func_array1[] = {FUNC0, FUNC0, FUNC0, FUNC0, FUNC0, FUNC0, -INVALID,};
enum lowlayer_func sdio_func_array2[] = {FUNC1, FUNC1, FUNC1, FUNC1, FUNC1, FUNC1, -INVALID,};
enum pull_updown sdio_pullud_array1[] = {NOPULL, PULLUP, PULLUP, PULLUP, PULLUP, PULLUP, -INVALID,};
enum pull_updown sdio_pullud_array2[] = {PULLDOWN, PULLDOWN, PULLDOWN, PULLDOWN, PULLDOWN, PULLDOWN, -INVALID,};
enum pull_updown sdio_pullud_array3[] = {NOPULL, NOPULL, NOPULL, NOPULL, NOPULL, NOPULL, -INVALID,};
enum drive_strength sdio_drv_array1[] = {LEVEL1, LEVEL1, LEVEL1, LEVEL1, LEVEL1, LEVEL1, -INVALID,};
enum drive_strength sdio_drv_array2[] = {LEVEL2, LEVEL2, LEVEL2, LEVEL2, LEVEL2, LEVEL2, -INVALID,};
enum drive_strength sdio_drv_array3[] = {LEVEL0, LEVEL0, LEVEL0, LEVEL0, LEVEL0, LEVEL0, -INVALID,};
struct block_config sdio_config[] = {
	[NORMAL] = {sdio_func_array1, sdio_pullud_array1, sdio_drv_array2},
	[GPIO] = {sdio_func_array2, sdio_pullud_array3, sdio_drv_array1},
	[LOWPOWER] = {sdio_func_array2, sdio_pullud_array2, sdio_drv_array3},
};

/*bt, g28 is used as adc interrupt pin
 *BT_SPI_CLK,BT_SPI_CS_N,BT_SPI_DATA,BT_ENABLE_RM
 */
/*btpm config*/
enum lowlayer_func btpm_func_gpio[] = {FUNC1, FUNC1, -INVALID,};
enum pull_updown btpm_pullud_normal[] = {PULLUP, NOPULL, -INVALID,};
enum pull_updown btpm_pullud_gpio[] = {PULLDOWN, NOPULL, -INVALID,};
enum drive_strength btpm_drv_default[] = {LEVEL2, LEVEL2, -INVALID,};
struct block_config btpm_config[] = {
	[NORMAL] = {btpm_func_gpio, btpm_pullud_normal, btpm_drv_default},
	[GPIO] = {btpm_func_gpio, btpm_pullud_gpio, btpm_drv_default},
	[LOWPOWER] = {btpm_func_gpio, btpm_pullud_gpio, btpm_drv_default},
};

/*btpm config*/
enum lowlayer_func btpm_func_gpio_cs[] = {FUNC1, FUNC1, -INVALID,};
enum pull_updown btpm_pullud_normal_cs[] = {PULLUP, NOPULL, -INVALID,};
enum pull_updown btpm_pullud_gpio_cs[] = {PULLDOWN, NOPULL, -INVALID,};
enum drive_strength btpm_drv_default_cs[] = {LEVEL2, LEVEL2, -INVALID,};
struct block_config btpm_cs_config[] = {
	[NORMAL] = {btpm_func_gpio_cs, btpm_pullud_normal_cs, btpm_drv_default_cs},
	[GPIO] = {btpm_func_gpio_cs, btpm_pullud_gpio_cs, btpm_drv_default_cs},
	[LOWPOWER] = {btpm_func_gpio_cs, btpm_pullud_gpio_cs, btpm_drv_default_cs},
};

/*btpwr config*/
enum lowlayer_func btpwr_func_gpio[] = {FUNC1, FUNC1, -INVALID,};
enum pull_updown btpwr_pullud_normal[] = {NOPULL, NOPULL, -INVALID,};
enum pull_updown btpwr_pullud_gpio[] = {NOPULL, NOPULL,  -INVALID,};
enum drive_strength btpwr_drv_default[] = {LEVEL2, LEVEL2, -INVALID,};
struct block_config btpwr_config[] = {
	[NORMAL] = {btpwr_func_gpio, btpwr_pullud_normal, btpwr_drv_default},
	[GPIO] = {btpwr_func_gpio, btpwr_pullud_gpio, btpwr_drv_default},
	[LOWPOWER] = {btpwr_func_gpio, btpwr_pullud_gpio, btpwr_drv_default},
};

/*btpwr for cs config*/
enum lowlayer_func btpwr_func_gpio_cs[] = {FUNC1, FUNC1, -INVALID,};
enum pull_updown btpwr_pullud_normal_cs[] = {NOPULL, NOPULL, -INVALID,};
enum pull_updown btpwr_pullud_gpio_cs[] = {NOPULL, NOPULL,  -INVALID,};
enum drive_strength btpwr_drv_default_cs[] = {LEVEL2, LEVEL2, -INVALID,};
struct block_config btpwr_cs_config[] = {
	[NORMAL] = {btpwr_func_gpio_cs, btpwr_pullud_normal_cs, btpwr_drv_default_cs},
	[GPIO] = {btpwr_func_gpio_cs, btpwr_pullud_gpio_cs, btpwr_drv_default_cs},
	[LOWPOWER] = {btpwr_func_gpio_cs, btpwr_pullud_gpio_cs, btpwr_drv_default_cs},
};

/*gps config for cellguide*/
enum lowlayer_func gps_func_cellguide[] = {FUNC0, FUNC0, FUNC0, FUNC0, FUNC0, FUNC0, FUNC0, FUNC2, -INVALID,};
enum lowlayer_func gps_func_gpio[] = {FUNC1, FUNC1, FUNC1, FUNC1, FUNC1, FUNC1, FUNC1, FUNC1, -INVALID,};
enum pull_updown gps_pullud_cellguide[] = {PULLDOWN, PULLDOWN, NOPULL, PULLUP, PULLUP, PULLUP, PULLUP, PULLDOWN, -INVALID,};
enum pull_updown gps_pullud_gpio[] = {PULLDOWN, PULLDOWN, PULLDOWN, PULLDOWN, PULLDOWN, PULLDOWN, \
	PULLDOWN, PULLDOWN, PULLDOWN, -INVALID,};
enum drive_strength gps_drv_cellguide[] = {RESERVE, RESERVE, RESERVE, LEVEL1, RESERVE, LEVEL1, LEVEL1, LEVEL0, -INVALID,};
enum drive_strength gps_drv_array1[] = {RESERVE, RESERVE, RESERVE, LEVEL1, RESERVE, LEVEL1, LEVEL1, LEVEL1, -INVALID,};
enum drive_strength gps_drv_array2[] = {RESERVE, RESERVE, RESERVE, LEVEL0, RESERVE, LEVEL0, LEVEL0, LEVEL0, -INVALID,};
struct block_config gps_cellguide_config[] = {
	[NORMAL] = {gps_func_cellguide, gps_pullud_cellguide, gps_drv_cellguide},
	[GPIO] = {gps_func_gpio, gps_pullud_gpio, gps_drv_array1},
	[LOWPOWER] = {gps_func_gpio, gps_pullud_gpio, gps_drv_array2},
};

/*gps config for boardcom*/
enum lowlayer_func gps_func_boardcom[] = {FUNC1, FUNC1, -INVALID,};
enum lowlayer_func baordcom_func_gpio[] = {FUNC1, FUNC1, -INVALID,};
enum pull_updown gps_pullud_boardcom[] = {NOPULL, NOPULL,  -INVALID,};
enum pull_updown baordcom_pullud_gpio[] = {NOPULL, NOPULL, -INVALID,};
enum drive_strength gps_drv_boardcom[] = {RESERVE, RESERVE, -INVALID,};
struct block_config gps_boardcom_config[] = {
	[NORMAL] = {gps_func_boardcom, gps_pullud_boardcom, gps_drv_boardcom},
	[GPIO] = {baordcom_func_gpio, gps_pullud_boardcom, gps_drv_boardcom},
	[LOWPOWER] = {baordcom_func_gpio, baordcom_pullud_gpio, gps_drv_boardcom},
};

/*touch screen:GPIO_156, GPIO_157, there are no iomg*/
enum lowlayer_func ts_func_array1[] = {RESERVE, RESERVE, -INVALID,};
enum pull_updown ts_pullud_array1[] = {NOPULL, PULLUP, -INVALID,};
enum drive_strength ts_drv_array1[] = {RESERVE, RESERVE,  -INVALID,};
struct block_config ts_config[] = {
	[NORMAL] = {ts_func_array1, ts_pullud_array1, ts_drv_array1},
	[GPIO] = {ts_func_array1, ts_pullud_array1, ts_drv_array1},
	[LOWPOWER] = {ts_func_array1, ts_pullud_array1, ts_drv_array1},
};

/*lcd pins:
 *KEYPAD_OUT6, KEYPAD_OUT7, EFUSE_CSB, RFTCXO_PWR
 */
#if defined(CONFIG_LCD_PANASONIC_VVX10F002A00)
enum lowlayer_func lcd_func_array1[] = {FUNC1, FUNC1, FUNC0, FUNC1, -INVALID,};
enum pull_updown lcd_pullud_array1[] = {NOPULL, NOPULL, NOPULL, NOPULL, -INVALID,};
enum drive_strength lcd_drv_array1[] = {RESERVE, RESERVE, RESERVE, RESERVE, -INVALID,};
struct block_config lcd_config[] = {
	[NORMAL] = {lcd_func_array1, lcd_pullud_array1, lcd_drv_array1},
	[GPIO] = {lcd_func_array1, lcd_pullud_array1, lcd_drv_array1},
	[LOWPOWER] = {lcd_func_array1, lcd_pullud_array1, lcd_drv_array1},
};

/*pwm pins:
 *PWM_OUT0, PWM_OUT1
 */
enum lowlayer_func pwm_func_normal[] = {FUNC0, FUNC1, -INVALID,};
enum lowlayer_func pwm_func_gpio[] = {FUNC1, FUNC1, -INVALID,};
enum pull_updown pwm_pullud_array1[] = {NOPULL, PULLDOWN, -INVALID,};
enum drive_strength pwm_drv_array1[] = {LEVEL2, LEVEL2, -INVALID,};
struct block_config pwm_config[] = {
	[NORMAL] = {pwm_func_normal, pwm_pullud_array1, pwm_drv_array1},
	[GPIO] = {pwm_func_gpio, pwm_pullud_array1, pwm_drv_array1},
	[LOWPOWER] = {pwm_func_gpio, pwm_pullud_array1, pwm_drv_array1},
};

#elif defined(CONFIG_LCD_QIMEI_HJ101IA_01F_10)
enum lowlayer_func lcd_func_array1[] = {FUNC1, FUNC0, FUNC1, FUNC1, FUNC1, -INVALID,};
enum pull_updown lcd_pullud_array1[] = {NOPULL, NOPULL, NOPULL, NOPULL, NOPULL,-INVALID,};
enum drive_strength lcd_drv_array1[] = {RESERVE, RESERVE, RESERVE, RESERVE, RESERVE, -INVALID,};
struct block_config lcd_config[] = {
	[NORMAL] = {lcd_func_array1, lcd_pullud_array1, lcd_drv_array1},
	[GPIO] = {lcd_func_array1, lcd_pullud_array1, lcd_drv_array1},
	[LOWPOWER] = {lcd_func_array1, lcd_pullud_array1, lcd_drv_array1},
};

enum lowlayer_func pwm_func_normal[] = {FUNC0, -INVALID,};
enum lowlayer_func pwm_func_gpio[] = {FUNC1, -INVALID,};
enum pull_updown pwm_pullud_array1[] = {NOPULL, -INVALID,};
enum drive_strength pwm_drv_array1[] = {LEVEL2, -INVALID,};
struct block_config pwm_config[] = {
	[NORMAL] = {pwm_func_normal, pwm_pullud_array1, pwm_drv_array1},
	[GPIO] = {pwm_func_gpio, pwm_pullud_array1, pwm_drv_array1},
	[LOWPOWER] = {pwm_func_gpio, pwm_pullud_array1, pwm_drv_array1},
};
#endif

/*hdmi pins:
 *HDMI_SCL, HDMI_SDA, HDMI_CEC, HDMI_HPD
 */
enum lowlayer_func hdmi_func_normal[] = {FUNC0, FUNC0, FUNC0, FUNC0, -INVALID,};
enum lowlayer_func hdmi_func_gpio[] = {FUNC1, FUNC1, FUNC1, FUNC1, -INVALID,};
enum pull_updown hdmi_pullud_array1[] = {NOPULL, NOPULL, NOPULL, NOPULL, -INVALID,};
enum pull_updown hdmi_pullud_array2[] = {NOPULL, NOPULL, NOPULL, PULLDOWN, -INVALID,};
enum drive_strength hdmi_drv_array1[] = {RESERVE, RESERVE, RESERVE, RESERVE, -INVALID,};
struct block_config hdmi_config[] = {
	[NORMAL] = {hdmi_func_normal, hdmi_pullud_array1, hdmi_drv_array1},
	[GPIO] = {hdmi_func_gpio, hdmi_pullud_array2, hdmi_drv_array1},
	[LOWPOWER] = {hdmi_func_gpio, hdmi_pullud_array2, hdmi_drv_array1},
};

/*wifi pins:
 *EFUSE_SEL, EFUSE_SCLK
 */
enum lowlayer_func wifi_func_normal[] = {FUNC0, FUNC0, -INVALID,};
enum lowlayer_func wifi_func_gpio[] = {FUNC0, FUNC0, -INVALID,};
enum pull_updown wifi_pullud_array1[] = {NOPULL, PULLDOWN, -INVALID,};
enum drive_strength wifi_drv_array1[] = {RESERVE, RESERVE, -INVALID,};
struct block_config wifi_config[] = {
	[NORMAL] = {wifi_func_normal, wifi_pullud_array1, wifi_drv_array1},
	[GPIO] = {wifi_func_gpio, wifi_pullud_array1, wifi_drv_array1},
	[LOWPOWER] = {wifi_func_gpio, wifi_pullud_array1, wifi_drv_array1},
};

/*dvp pins:
 *cam_data0, cam_data1, cam_data2, cam_data3, cam_data4, cam_data5, cam_data6, cam_data7,
 *cam_hysnc, cam_pclk, cam_vsync.
 */
enum lowlayer_func isp_dvp_func_normal[] = {FUNC0, FUNC0, FUNC0, FUNC0, FUNC0, FUNC0, FUNC0, FUNC0,\
	FUNC0, FUNC0, FUNC0, -INVALID,};
enum lowlayer_func isp_dvp_func_gpio[] = {FUNC1, FUNC1, FUNC1, FUNC1, FUNC1, FUNC1, FUNC1, FUNC1, \
	FUNC1, FUNC1, FUNC1, -INVALID,};
enum pull_updown isp_dvp_pullud_array1[] = {NOPULL, NOPULL,  NOPULL, NOPULL, NOPULL, \
	NOPULL, NOPULL, NOPULL, NOPULL, NOPULL, NOPULL, -INVALID,};
enum pull_updown isp_dvp_pullud_array2[] = {PULLDOWN, PULLDOWN,  PULLDOWN, PULLDOWN, PULLDOWN, \
	PULLDOWN, PULLDOWN, PULLDOWN, PULLDOWN, PULLDOWN, PULLDOWN, -INVALID,};
enum drive_strength isp_dvp_drv_array1[] = {RESERVE, RESERVE, RESERVE, RESERVE, RESERVE, RESERVE, \
	RESERVE, RESERVE, RESERVE, RESERVE, RESERVE, -INVALID,};
struct block_config isp_dvp_config[] = {
	[NORMAL] = {isp_dvp_func_normal, isp_dvp_pullud_array1, isp_dvp_drv_array1},
	[GPIO] = {isp_dvp_func_gpio, isp_dvp_pullud_array2, isp_dvp_drv_array1},
	[LOWPOWER] = {isp_dvp_func_gpio, isp_dvp_pullud_array2, isp_dvp_drv_array1},
};

/*i2c isp pins:
 *isp_scl0, isp_sda0.
 */
enum lowlayer_func isp_i2c_func_normal[] = {FUNC0, FUNC0, -INVALID,};
enum lowlayer_func isp_i2c_func_gpio[] = {FUNC1, FUNC1, -INVALID,};
enum pull_updown isp_i2c_pullud_array1[] = {PULLUP, PULLUP, -INVALID,};
enum pull_updown isp_i2c_pullud_array2[] = {PULLDOWN, PULLDOWN, -INVALID,};
enum drive_strength isp_i2c_drv_array1[] = {RESERVE, RESERVE, -INVALID,};
struct block_config isp_i2c_config[] = {
	[NORMAL] = {isp_i2c_func_normal, isp_i2c_pullud_array1, isp_i2c_drv_array1},
	[GPIO] = {isp_i2c_func_gpio, isp_i2c_pullud_array2, isp_i2c_drv_array1},
	[LOWPOWER] = {isp_i2c_func_gpio, isp_i2c_pullud_array2, isp_i2c_drv_array1},
};

/*isp_reset
 *isp_resetb0, isp_resetb1
 */
enum lowlayer_func isp_reset_func_normal[] = {FUNC0, FUNC0, -INVALID,};
enum lowlayer_func isp_reset_func_gpio[] = {FUNC1, FUNC1, -INVALID,};
enum pull_updown isp_reset_pullud_array1[] = {NOPULL, NOPULL, -INVALID,};
enum drive_strength isp_reset_drv_array1[] = {RESERVE, RESERVE,  -INVALID,};
struct block_config isp_reset_config[] = {
	[NORMAL] = {isp_reset_func_normal, isp_reset_pullud_array1, isp_reset_drv_array1},
	[GPIO] = {isp_reset_func_gpio, isp_reset_pullud_array1, isp_reset_drv_array1},
	[LOWPOWER] = {isp_reset_func_gpio, isp_reset_pullud_array1, isp_reset_drv_array1},
};

/*isp pins:
 *isp_cclk0, isp_cclk2, isp_gpio0,
 *isp_gpio1, isp_gpio2, isp_gpio3, isp_gpio4, isp_gpio6,
 *and isp_gpio8, isp_gpio9 are used as gpio,don't need to mux
 */
enum lowlayer_func isp_func_normal[] = {FUNC0, FUNC0, FUNC0, FUNC0, FUNC0, FUNC0, FUNC0,\
	FUNC0, -INVALID,};
enum lowlayer_func isp_func_gpio[] = {FUNC1, FUNC1, FUNC1, FUNC1, FUNC1, FUNC1, FUNC1, FUNC0, -INVALID,};
enum pull_updown isp_pullud_array1[] = {NOPULL, NOPULL, NOPULL, NOPULL, NOPULL, \
	NOPULL, NOPULL, NOPULL, -INVALID,};
enum pull_updown isp_pullud_array2[] = {NOPULL, NOPULL, NOPULL, \
	NOPULL, NOPULL, NOPULL, NOPULL, NOPULL, -INVALID,};
enum drive_strength isp_drv_array1[] = {LEVEL0, LEVEL0, RESERVE, RESERVE, \
	RESERVE, RESERVE, RESERVE, RESERVE, -INVALID,};
struct block_config isp_config[] = {
	[NORMAL] = {isp_func_normal, isp_pullud_array1, isp_drv_array1},
	[GPIO] = {isp_func_gpio, isp_pullud_array2, isp_drv_array1},
	[LOWPOWER] = {isp_func_gpio, isp_pullud_array2, isp_drv_array1},
};

/*isp flash light:isp_strobe0, isp_strobe1, isp_gpio7*/
enum lowlayer_func isp_flash_func_normal[] = {FUNC1, FUNC1, FUNC1, -INVALID,};
enum lowlayer_func isp_flash_func_gpio[] = {FUNC1, FUNC1, FUNC1, -INVALID,};
enum pull_updown isp_flash_pullud_array1[] = {NOPULL, NOPULL, NOPULL, -INVALID,};
enum drive_strength isp_flash_drv_array1[] = {RESERVE, RESERVE, RESERVE, -INVALID,};
struct block_config isp_flash_config[] = {
	[NORMAL] = {isp_flash_func_normal, isp_flash_pullud_array1, isp_flash_drv_array1},
	[GPIO] = {isp_flash_func_gpio, isp_flash_pullud_array1, isp_flash_drv_array1},
	[LOWPOWER] = {isp_flash_func_gpio, isp_flash_pullud_array1, isp_flash_drv_array1},
};

/*isp flash light:isp_strobe0, isp_strobe1, isp_gpio7*/
enum lowlayer_func isp_flash_func_normal_cs[] = {FUNC1, FUNC1, FUNC1, -INVALID,};
enum lowlayer_func isp_flash_func_gpio_cs[] = {FUNC1, FUNC1, FUNC1, -INVALID,};
enum pull_updown isp_flash_pullud_array1_cs[] = {NOPULL, NOPULL, NOPULL, -INVALID,};
enum drive_strength isp_flash_drv_array1_cs[] = {RESERVE, RESERVE, RESERVE, -INVALID,};
struct block_config isp_flash_cs_config[] = {
	[NORMAL] = {isp_flash_func_normal_cs, isp_flash_pullud_array1_cs, isp_flash_drv_array1_cs},
	[GPIO] = {isp_flash_func_gpio_cs, isp_flash_pullud_array1_cs, isp_flash_drv_array1_cs},
	[LOWPOWER] = {isp_flash_func_gpio_cs, isp_flash_pullud_array1_cs, isp_flash_drv_array1_cs},
};

/*charger pins:
 *EFUSE_PGM, ISP_GPIO5
 */
enum lowlayer_func charger_func_normal[] = {FUNC0, FUNC1, -INVALID,};
enum lowlayer_func charger_func_gpio[] = {FUNC0, FUNC1, -INVALID,};
enum pull_updown charger_pullud_array1[] = {PULLUP, NOPULL, -INVALID,};
enum drive_strength charger_drv_array1[] = {RESERVE, RESERVE, -INVALID,};
struct block_config charger_config[] = {
	[NORMAL] = {charger_func_normal, charger_pullud_array1, charger_drv_array1},
	[GPIO] = {charger_func_gpio, charger_pullud_array1, charger_drv_array1},
	[LOWPOWER] = {charger_func_gpio, charger_pullud_array1, charger_drv_array1},
};

/*gsensor pins:
 *CAM_DATA9, TBC_DOWN,
 */
enum lowlayer_func gsensor_func_normal[] = {FUNC1, FUNC1, -INVALID,};
enum pull_updown gsensor_pullud_array1[] = {PULLDOWN, PULLDOWN, -INVALID,};
enum drive_strength gsensor_drv_array1[] = {RESERVE, RESERVE, -INVALID,};
struct block_config gsensor_config[] = {
	[NORMAL] = {gsensor_func_normal, gsensor_pullud_array1, gsensor_drv_array1},
	[GPIO] = {gsensor_func_normal, gsensor_pullud_array1, gsensor_drv_array1},
	[LOWPOWER] = {gsensor_func_normal, gsensor_pullud_array1, gsensor_drv_array1},
};

/*
 *audio es305 pins:
 *USIM_RST,ONEWIRE
 */
enum lowlayer_func audio_es305_func_normal[] = {FUNC1, FUNC1, -INVALID,};
enum pull_updown audio_es305_pullud_array1[] = {NOPULL, NOPULL, -INVALID,};
enum pull_updown audio_es305_pullud_array2[] = {NOPULL, NOPULL, -INVALID,};
enum drive_strength audio_es305_drv_array1[] = {RESERVE, RESERVE, -INVALID,};
struct block_config audio_es305_config[] = {
       [NORMAL] = {audio_es305_func_normal, audio_es305_pullud_array1, audio_es305_drv_array1},
       [GPIO] = {audio_es305_func_normal, audio_es305_pullud_array2, audio_es305_drv_array1},
       [LOWPOWER] = {audio_es305_func_normal, audio_es305_pullud_array2, audio_es305_drv_array1},
};

/*
 *audio speaker pins:
 *UART0_CTS_N
 */
enum lowlayer_func audio_spk_func_normal[] = {FUNC1, -INVALID,};
enum pull_updown audio_spk_pullud_array1[] = {NOPULL, -INVALID,};
enum drive_strength audio_spk_drv_array1[] = {RESERVE, -INVALID,};
struct block_config audio_spk_config[] = {
       [NORMAL] = {audio_spk_func_normal, audio_spk_pullud_array1, audio_spk_drv_array1},
       [GPIO] = {audio_spk_func_normal, audio_spk_pullud_array1, audio_spk_drv_array1},
       [LOWPOWER] = {audio_spk_func_normal, audio_spk_pullud_array1, audio_spk_drv_array1},
};

/*
 *audio earphone pins:
 *UART0_CTS_N
 */
enum lowlayer_func audio_eph_func_normal[] = {FUNC1, -INVALID,};
enum pull_updown audio_eph_pullud_array1[] = {NOPULL, -INVALID,};
enum drive_strength audio_eph_drv_array1[] = {RESERVE, -INVALID,};
struct block_config audio_eph_config[] = {
       [NORMAL] = {audio_eph_func_normal, audio_eph_pullud_array1, audio_eph_drv_array1},
       [GPIO] = {audio_eph_func_normal, audio_eph_pullud_array1, audio_eph_drv_array1},
       [LOWPOWER] = {audio_eph_func_normal, audio_eph_pullud_array1, audio_eph_drv_array1},
};

/*
 *modem pins:
 *SDIO0_CLK(gpio_094), SDIO0_CMD(gpio_095), SDIO0_DATA0(gpio_096), SDIO0_DATA1(gpio_097),
 *SDIO0_DATA2(gpio_098), SPI1_CLK(gpio_113), SPI1_DI(gpio_114), SPI1_DO(gpio_115)
 */
enum lowlayer_func modem_func_gpio[] = {FUNC1, FUNC1, FUNC1, FUNC1, FUNC1, FUNC1, FUNC1,\
	FUNC1, -INVALID,};
enum pull_updown modem_pullud_array1[] = {NOPULL, NOPULL, NOPULL, NOPULL, NOPULL, \
	NOPULL, NOPULL, NOPULL, -INVALID,};
enum pull_updown modem_pullud_array2[] = {NOPULL, NOPULL, NOPULL, \
	NOPULL, PULLDOWN, PULLDOWN, NOPULL, NOPULL, -INVALID,};
enum drive_strength modem_drv_array1[] = {LEVEL2, LEVEL2, LEVEL2, LEVEL2, \
	LEVEL2, LEVEL2, LEVEL2, LEVEL2, -INVALID,};
struct block_config modem_config[] = {
	[NORMAL] = {modem_func_gpio, modem_pullud_array1, modem_drv_array1},
	[GPIO] = {modem_func_gpio, modem_pullud_array2, modem_drv_array1},
	[LOWPOWER] = {modem_func_gpio, modem_pullud_array2, modem_drv_array1},
};
/*TODO:every config of blocks*/
#define BLOCK_CONFIG(_name, _block, _configarray) \
{\
	.name = _name,\
	.block = _block,\
	.config_array = _configarray,\
},

struct block_table block_config_table_es[] = {
	BLOCK_CONFIG("block_i2c0", &block_i2c0, i2c0_config)
	BLOCK_CONFIG("block_i2c1", &block_i2c1, i2c1_config)
	BLOCK_CONFIG("block_spi0", &block_spi0, spi0_config)
	BLOCK_CONFIG("block_spi1", &block_spi1, spi1_config)
	BLOCK_CONFIG("block_uart0", &block_uart0, uart0_config)
	BLOCK_CONFIG("block_uart1", &block_uart1, uart1_config)
	BLOCK_CONFIG("block_uart2", &block_uart2, uart2_config)
	BLOCK_CONFIG("block_uart3", &block_uart3, uart3_config)
	BLOCK_CONFIG("block_uart4", &block_uart4, uart4_config)
	BLOCK_CONFIG("block_kpc", &block_kpc, kpc_config)
	BLOCK_CONFIG("block_emmc", &block_emmc, emmc_config)
	BLOCK_CONFIG("block_sd", &block_sd, sd_config)
	BLOCK_CONFIG("block_nand", &block_nand, nand_config)
	BLOCK_CONFIG("block_sdio", &block_sdio, sdio_config)
	BLOCK_CONFIG("block_btpm", &block_btpm, btpm_config)
	BLOCK_CONFIG("block_btpwr", &block_btpwr, btpwr_config)
	BLOCK_CONFIG("block_btpwr", &block_btpwr_cs, btpwr_config)
	BLOCK_CONFIG("block_gps_cellguide", &block_gps_cellguide, gps_cellguide_config)
	BLOCK_CONFIG("block_gps_boardcom", &block_gps_boardcom, gps_boardcom_config)
	BLOCK_CONFIG("block_ts", &block_ts, ts_config)
	BLOCK_CONFIG("block_lcd", &block_lcd, lcd_config)
	BLOCK_CONFIG("block_pwm", &block_pwm, pwm_config)
	BLOCK_CONFIG("block_hdmi", &block_hdmi, hdmi_config)
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

struct block_table block_config_table_cs[] = {
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
	BLOCK_CONFIG("block_kpc", &block_kpc, kpc_config)
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
	BLOCK_CONFIG("block_pwm", &block_pwm, pwm_config)
	BLOCK_CONFIG("block_hdmi", &block_hdmi, hdmi_config)
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

//extern struct block_table block_config_phone_u9510_es[];
//extern struct block_table block_config_phone_u9510_cs[];
extern struct block_table block_config_pad_s10_es[];
extern struct block_table block_config_pad_s10_cs[];

struct block_table *block_config_tables[] = {
	[E_IOMUX_PALTFORM_ES] = block_config_table_es,
	[E_IOMUX_PALTFORM_CS] = block_config_table_cs,
//	[E_IOMUX_PHONE_ES] = block_config_phone_u9510_es,
//	[E_IOMUX_PHONE_CS] = block_config_phone_u9510_cs,
	[E_IOMUX_PAD_ES] = block_config_pad_s10_es,
	[E_IOMUX_PAD_CS] = block_config_pad_s10_cs,
	[E_IOMUX_MAX] = NULL,
};
#endif
