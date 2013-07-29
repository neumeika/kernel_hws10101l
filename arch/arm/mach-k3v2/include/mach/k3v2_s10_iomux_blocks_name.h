#ifndef __MACH_K3V2_S10_IOMUX_BLOCKS_NAME_H
#define __MACH_K3V2_S10_IOMUX_BLOCKS_NAME_H


#define  IO_I2C0_BLOCK_NAME                 "block_i2c0"
#define  IO_I2C1_BLOCK_NAME                 "block_i2c1"
#define  IO_I2C2_BLOCK_NAME                 "block_i2c2"
#define  IO_I2C3_BLOCK_NAME                 "block_i2c3"

#if defined(CONFIG_GPIO_I2C_ADPT)
#define IO_GPIO_I2C_BLOCK_NAME				"block_gpio_i2c"
#endif

#define  IO_SPI0_BLOCK_NAME                 "block_spi0"
#define  IO_SPI0_CS_BLOCK_NAME              "block_spi0_cs"
#define  IO_SPI1_BLOCK_NAME                 "block_spi1"
#define  IO_UART0_BLOCK_NAME                "block_uart0"
#define  IO_UART1_BLOCK_NAME                "block_uart1"
#define  IO_UART2_BLOCK_NAME                "block_uart2"
#define  IO_UART3_BLOCK_NAME                "block_uart3"
#define  IO_UART4_BLOCK_NAME                "block_uart4"
#define  IO_KPC_BLOCK_NAME                  "block_kpc"
#define  IO_EMMC_BLOCK_NAME                 "block_emmc"
#define  IO_SD_BLOCK_NAME                   "block_sd"
#define  IO_NAND_BLOCK_NAME                 "block_nand"
#define  IO_SDIO_BLOCK_NAME                 "block_sdio"
#define  IO_BTPM_BLOCK_NAME                 "block_btpm"
#define  IO_BTPWR_BLOCK_NAME                "block_btpwr"
#define  IO_PCM_BLOCK_NAME                  "block_pcm"
#define  IO_GPS_CELLGUIDE_BLOCK_NAME        "block_gps_cellguide"
#define  IO_GPS_BOARDCOM_BLOCK_NAME         "block_gps_boardcom"
#define  IO_TS_BLOCK_NAME                   "block_ts"
#define  IO_LCD_BLOCK_NAME                  "block_lcd"
#define  IO_PWM_BLOCK_NAME                  "block_pwm"
#define  IO_HDMI_BLOCK_NAME                 "block_hdmi"
#define  IO_WIFI_BLOCK_NAME                 "block_wifi"
#define  IO_ISP_DVP_BLOCK_NAME              "block_isp_dvp"
#define  IO_ISP_I2C_BLOCK_NAME              "block_isp_i2c"
#define  IO_ISP_RESET_BLOCK_NAME            "block_isp_reset"
#define  IO_ISP_BLOCK_NAME                  "block_isp"
#define  IO_ISP_FLASH_BLOCK_NAME            "block_isp_flash"
#define  IO_CHARGER_BLOCK_NAME              "block_charger"
#define  IO_GSENSOR_BLOCK_NAME              "block_gsensor"

#if defined(CONFIG_SUPPORT_MICRO_USB_PORT)
#define  IO_MICRO_USB                       "block_micro_usb"
#define  IO_VBUS_DRV                        "block_vbus_drv"
#endif

#if defined(CONFIG_FSA880_I2C)
#define  IO_BC11_BLOCK_NAME                 "block_bc11"
#endif

#define  IO_BOOST_5V_BLOCK_NAME             "block_boost_5v"

#define  IO_BATTERY_GAUGE_BLOCK_NAME        "block_battery_gauge"
#define  IO_SD_CTRL_BLOCK_NAME              "block_sd_ctrl"
#define  IO_MHL_BLOCK_NAME                  "block_mhl"
#define  IO_CAMERA_PWR_BLOCK_NAME           "block_camera_pwr"
#define  IO_MAIN_CAMERA_BLOCK_NAME          "block_main_camera"
#define  IO_SLAVE_CAMERA_BLOCK_NAME         "block_slave_camera"
#define  IO_MR_SENSOR_BLOCK_NAME            "block_mr_sensor"
#define  IO_COMPASS_BLOCK_NAME              "block_compass"
#define  IO_GYROSCOPE_BLOCK_NAME            "block_gyroscope"
#define  IO_CAPACITOR_SENSOR_BLOCK_NAME     "block_capacitor_sensor"
#define  IO_SIM_DECT_BLOCK_NAME             "block_sim_dect"

#define  IO_EXTRAL_DCDC_BLOCK_NAME          "block_extral_dcdc"
#define  IO_DOCK_BLOCK_NAME                 "block_dock"
#define  IO_AUDIO_SPK_BLOCK_NAME            "block_audio_spk"
#define  IO_AUDIO_EARPHONE_BLOCK_NAME       "block_audio_eph"
#define  IO_MODEM_AP_CTRL_BLOCK_NAME        "block_moden_ap_ctrl"
#define  IO_MINOR_VERSION_BLOCK_NAME        "block_minor_version"  //
#define IO_USB_SWITCH_CTRL_BLOCK_NAME	"usb_switch_ctrl"

#endif
