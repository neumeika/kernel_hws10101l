/*
 * arch/arm/mach-k3v2/board-tc45msu3.c
 *
 * Copyright (C) 2010 Google, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/serial_8250.h>
#include <linux/clk.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#include <linux/dma-mapping.h>
#include <linux/pda_power.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/sysdev.h>
#include <linux/amba/bus.h>
#include <linux/io.h>
#include <linux/rmi.h>
#include <linux/delay.h>
#include <linux/mux.h>

#if 1 //#ifdef CONFIG_TOUCHSCREEN_SYNAPTICS_I2C_RMI4_K3V2
#include <linux/synaptics_i2c_rmi4.h>
#endif

#ifdef CONFIG_BATTERY_K3_BQ24161
#include <linux/power/k3_bq24161.h>
#endif
#ifdef CONFIG_BATTERY_K3_BQ27510
#include <linux/power/k3_bq27510.h>
#endif
#ifdef CONFIG_BATTERY_K3
#include <linux/power/k3_battery_monitor.h>
#endif
#include <linux/mhl/mhl.h>
#ifdef CONFIG_PSY_MONITOR
//#include <linux/power/k3_psy_monitor.h>//reomove s10 ics charging code (out of use) 
#include <linux/power/S10_std_psy.h>
#endif
#ifdef CONFIG_PSY_MAX8903
//#include <linux/power/k3_max8903_charger.h>//reomove s10 ics charging code (out of use)
#include <linux/power/max8903.h>
#endif
#ifdef CONFIG_PSY_BQ275X0
#include <linux/power/bq27510.h>
 #include <linux/power/S10_std_coulometer.h>
#endif
#ifdef CONFIG_PSY_BQ24161
 #include <linux/power/bq24161.h>
#endif
#include <linux/hkadc/hiadc_hal.h>
#include <asm/setup.h>
#include <asm/mach-types.h>
#include <asm/system.h>
#include <asm/mach/arch.h>
#include <mach/hardware.h>
#include <mach/system.h>
#include <mach/irqs.h>
#include <mach/io.h>
#include <mach/gpio.h>
#include <mach/early-debug.h>
#include <mach/hisi_mem.h>
#include <mach/k3_keypad.h>
#include <mach/boardid.h>
#if defined(CONFIG_TOUCHSCREEN_MXT224E) || defined(CONFIG_TOUCHSCREEN_MXT224E_MODULE)
#include <mach/atmel_mXT224E.h>
#elif defined(CONFIG_TOUCHSCREEN_MXT1386) || defined(CONFIG_TOUCHSCREEN_MXT1386_MODULE)
#include <linux/atmel_mxt1386.h>
#endif
#include <mach/tps61310.h>

#include "board.h"
#include "clock.h"
#include "k3v2_clocks_init_data.h"
#include <mach/sound/tpa2028_spk_l.h>
#include <mach/sound/tpa2028_spk_r.h>
#include <mach/sound/tpa6132.h>
#include <mach/k3_version.h>
#ifdef CONFIG_AUDIENCE
#include <mach/sound/es305.h>
#endif

#include <linux/switch_usb.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/irq.h>
#ifdef CONFIG_FSA880_I2C
#include <linux/input/fsa880_i2c.h>
#endif

#ifdef CONFIG_BOOST_5V
#include <linux/power/boost_5v.h>
#endif

#if defined(CONFIG_SUPPORT_MICRO_USB_PORT)
#include <linux/power/micro_usb_power_control.h>
#endif

#if defined(CONFIG_FSA880_I2C) && defined(CONFIG_SUPPORT_MICRO_USB_PORT)
#error  sorry,you can not define CONFIG_FSA880_I2C and CONFIG_SUPPORT_MICRO_USB_PORT at the same time!
#endif

#include <hsad/config_debugfs.h>
#include <hsad/config_interface.h>
#ifdef CONFIG_LEDS_K3_6421
/* skf57909 2011/11/8  add begin */
#include <linux/led/k3-leds.h>
#endif

#ifdef CONFIG_ANDROID_K3_VIBRATOR
#include <linux/vibrator/k3_vibrator.h>
#endif

#include <linux/ad7147_S10.h>

#include <mach/s10_board_info.h>
#if S10_HWID_L3H(S10, S10101, A)
#include <mach/k3v2_gpio_key.h>
#endif

#ifdef CONFIG_EXTRAL_DYNAMIC_DCDC
#include <mach/extral_dynamic_dcdc.h>
#endif

#include <mach/product_feature_sel.h>

#include <mach/k3v2_s10_iomux_blocks_name.h>

extern int iomux_block_set_work_mode(char* block_name);
extern int iomux_block_set_lowpower_mode(char* block_name);
extern int iomux_block_set_gpio_mode(char* block_name);
extern int iomux_blocks_set_work_mode(char** block_name_array, unsigned int array_size);
extern int iomux_blocks_set_lowpower_mode(char** block_name_array, unsigned int array_size);
extern int iomux_blocks_set_gpio_mode(char** block_name_array, unsigned int array_size);

#if defined(CONFIG_TOUCHSCREEN_RMI4) || defined(CONFIG_TOUCHSCREEN_RMI4_MODULE)
#include <linux/rmi.h>
#endif
#ifdef CONFIG_DEBUG_FS
#include <linux/debugfs.h>
#endif

/*Begin:Added by g00124340 2011/09/19  for for bluetooth */
#define	GPIO_BT_EN					(GPIO_21_1)
#define	GPIO_BT_RST					(GPIO_21_0)
#define	GPIO_HOST_WAKEUP			(GPIO_20_6)
#define	GPIO_DEV_WAKEUP				(GPIO_20_7)

#define	REGULATOR_DEV_BLUETOOTH_NAME	"bt-io"

/*End:Added	by g00124340 2011/09/19	*/
/* skf57909 2011/11/8  add end */
#if defined(CONFIG_LCD_QIMEI_HJ101IA_01F_10)
#define GPIO_LCD_AVDD  (003)
#define GPIO_LED_EN     (150)
#elif defined(CONFIG_LCD_PANASONIC_VVX10F002A00)
#define GPIO_LCD_AVDD  (003)
#define GPIO_LED_EN  (171)
#else
#define GPIO_LCD_RESET  (003)
#define GPIO_LCD_POWER  (171)
#endif

#define GPIO_LCD_ID0	(135)
#define GPIO_LCD_ID1	(136)
#define GPIO_PWM0   (149)
#define GPIO_PWM1   (150)
#if defined(CONFIG_LCD_PANASONIC_VVX10F002A00) || defined(CONFIG_LCD_QIMEI_HJ101IA_01F_10)
#define GPIO_LCD_AVDD_NAME	     "gpio_lcd_avdd"
#define GPIO_LED_EN_NAME	     "gpio_led_enable"
#else
#define GPIO_LCD_POWER_NAME "gpio_lcd_power"
#define GPIO_LCD_RESET_NAME "gpio_lcd_reset"
#endif
#define GPIO_LCD_ID0_NAME "gpio_lcd_id0"
#define GPIO_LCD_ID1_NAME "gpio_lcd_id1"
#define GPIO_PWM0_NAME   "gpio_pwm0"
#define GPIO_PWM1_NAME   "gpio_pwm1"
#define REG_BASE_PWM0_NAME  "reg_base_pwm0"
#define REGULATOR_DEV_LCD_NAME  "k3_dev_lcd"
#define REGULATOR_DEV_EDC_NAME  "k3_dev_edc"

#if defined(CONFIG_LCD_QIMEI_HJ101IA_01F_10)
#define GPIO_CM  				 (16)
#define GPIO_CABC 				 (17)
#define GPIO_CM_NAME			 "gpio_cm"
#define GPIO_CABC_NAME			 "gpio_cabc"
#endif

/*
#define PLATFORM_DEVICE_LCD_NAME "ldi_samsung_LMS350DF04"
#define PLATFORM_DEVICE_LCD_NAME "mipi_samsung_S6E39A"
#define PLATFORM_DEVICE_LCD_NAME "mipi_sharp_LS035B3SX"
#define PLATFORM_DEVICE_LCD_NAME "mipi_toshiba_MDW70"
*/

#ifdef CONFIG_LCD_TOSHIBA_MDW70
#define PLATFORM_DEVICE_LCD_NAME "mipi_toshiba_MDW70_V001"
#elif  defined(CONFIG_LCD_PANASONIC_VVX10F002A00)
#define PLATFORM_DEVICE_LCD_NAME "mipi_panasonic_VVX10F002A00"
#elif  defined(CONFIG_LCD_QIMEI_HJ101IA_01F_10)
#define PLATFORM_DEVICE_LCD_NAME "mipi_qimei_HJ101IA_01F_10"
#elif defined(CONFIG_LCD_CMI_OTM1280A)
#define PLATFORM_DEVICE_LCD_NAME "mipi_cmi_OTM1280A"
#elif defined(CONFIG_LCD_SAMSUNG_LMS350DF04)
#define PLATFORM_DEVICE_LCD_NAME "ldi_samsung_LMS350DF04"
#elif defined(CONFIG_LCD_SAMSUNG_S6E39A)
#define PLATFORM_DEVICE_LCD_NAME "mipi_samsung_S6E39A"
#elif defined(CONFIG_LCD_SHARP_LS035B3SX)
#define PLATFORM_DEVICE_LCD_NAME "mipi_sharp_LS035B3SX"
#elif defined(CONFIG_LCD_CMI_PT045TN07)
#define PLATFORM_DEVICE_LCD_NAME "mipi_cmi_PT045TN07"
#elif defined(CONFIG_LCD_JDI_OTM1282B)
#define PLATFORM_DEVICE_LCD_NAME "mipi_jdi_OTM1282B"
#else
#error "PLATFORM_DEVICE_LCD_NAME not defined"
#endif

/* Begin: Added by d59977 for BCM GPS */
#define GPIO_GPS_BCM_EN    (GPIO_18_7)
#define GPIO_GPS_BCM_RET   (GPIO_19_0)
/* End: Added by d59977 for BCM GPS */

/* Begin: Added by d59977 for BCM GPS */
#define GPIO_GPS_BCM_EN_NAME    "gpio_gps_bcm_enable"
#define GPIO_GPS_BCM_RET_NAME   "gpio_gps_bcm_rest"
/* End: Added by d59977 for BCM GPS */

static struct resource k3_adc_resources = {
	.start	= REG_BASE_PMUSPI,
	.end	= REG_BASE_PMUSPI + REG_PMUSPI_IOSIZE - 1,
	.flags	= IORESOURCE_MEM,
};

static struct adc_data hi6421_adc_table[] = {
	{
		.ch = ADC_ADCIN1,
		.vol = ADC_VOLTAGE_MOD3,
		.clock = HKADC_CK_SEL_TWO,
		.buffer = HKADC_BUFF_SEL_YES,
		.parameter = ADC_PARAMETER1,
	},
	{
		.ch = ADC_ADCIN2,
		.vol = ADC_VOLTAGE_MOD3,
		.clock = HKADC_CK_SEL_TWO,
		.buffer = HKADC_BUFF_SEL_YES,
		.parameter = ADC_PARAMETER1,
	},
	{
		.ch = ADC_ADCIN3,
		.vol = ADC_VOLTAGE_MOD3,
		.clock = HKADC_CK_SEL_TWO,
		.buffer = HKADC_BUFF_SEL_YES,
		.parameter = ADC_PARAMETER1,
	},
	{
		.ch = ADC_NC0,
		.vol = ADC_VOLTAGE_MOD1,
		.clock = HKADC_CK_SEL_TWO,
		.buffer = HKADC_BUFF_SEL_YES,
		.parameter = ADC_PARAMETER1,
	},
	{
		.ch = ADC_VBATMON,
		.vol = ADC_VOLTAGE_MOD1,
		.clock = HKADC_CK_SEL_TWO,
		.buffer = HKADC_BUFF_SEL_YES,
		.parameter = ADC_PARAMETER2,
	},
	{
		.ch = ADC_VCOINMON,
		.vol = ADC_VOLTAGE_MOD1,
		.clock = HKADC_CK_SEL_TWO,
		.buffer = HKADC_BUFF_SEL_YES,
		.parameter = ADC_PARAMETER2,
	},
	{
		.ch = ADC_RTMP,
		.vol = ADC_VOLTAGE_MOD3,
		.clock = HKADC_CK_SEL_TWO,
		.buffer = HKADC_BUFF_SEL_YES,
		.parameter = ADC_PARAMETER1,
	},
	{
		.ch = ADC_PB_DETECT,
		.vol = ADC_VOLTAGE_MOD2,
		.clock = HKADC_CK_SEL_TWO,
		.buffer = HKADC_BUFF_SEL_YES,
		.parameter = ADC_PARAMETER1,
	},
	{
		.ch = ADC_NC1,
		.vol = ADC_VOLTAGE_MOD1,
		.clock = HKADC_CK_SEL_TWO,
		.buffer = HKADC_BUFF_SEL_YES,
		.parameter = ADC_PARAMETER1,
	},
	{
		.ch = ADC_NC2,
		.vol = ADC_VOLTAGE_MOD1,
		.clock = HKADC_CK_SEL_TWO,
		.buffer = HKADC_BUFF_SEL_YES,
		.parameter = ADC_PARAMETER1,
	},
	{
		.ch = ADC_500K,
		.vol = ADC_VOLTAGE_MOD2,
		.clock = HKADC_CK_SEL_TWO,
		.buffer = HKADC_BUFF_SEL_YES,
		.parameter = ADC_PARAMETER1,
	},
};

static struct adc_dataEx hi6421_adc_tableEx = {
	.data = hi6421_adc_table,
	.sum = ARRAY_SIZE(hi6421_adc_table),
};

static struct platform_device hisik3_adc_device = {
	.name    = "k3adc",
	.id    = 0,
	.dev    = {
		.platform_data = &hi6421_adc_tableEx,
		.init_name = "hkadc",
	},
	.num_resources    = 1,
	.resource    =  &k3_adc_resources,
};

static struct platform_device hisik3_device_hwmon = {
	.name		= "k3-hwmon",
	.id		= -1,
};

#ifdef CONFIG_LEDS_K3_6421
/* skf57909 2011/11/8 led add begin */
/*k3_led begin*/
static struct k3_led_platform_data hi6421_leds = {
	.leds_size = K3_LEDS_MAX,
	.leds = {
		[0] = {
			.name = "red",
			.brightness = LED_OFF,
			.delay_on = 0,
			.delay_off = 0,
			.default_trigger = "timer",
		},
		[1] = {
			.name = "green",
			.brightness = LED_OFF,
			.delay_on = 0,
			.delay_off = 0,
			.default_trigger = "timer",
		},
		[2] {
			.name = "blue",
			.brightness = LED_OFF,
			.delay_on = 0,
			.delay_off = 0,
			.default_trigger = "timer",
		},
	},
};

static struct k3_led_platform_data hi6421_leds_phone = {
	.leds_size = K3_LEDS_MAX,
	.leds = {
		[0] = {
			.name = "green",
			.brightness = LED_OFF,
			.delay_on = 0,
			.delay_off = 0,
			.default_trigger = "timer",
		},
		[1] = {
			.name = "red",
			.brightness = LED_OFF,
			.delay_on = 0,
			.delay_off = 0,
			.default_trigger = "timer",
		},
		[2] {
			.name = "blue",
			.brightness = LED_OFF,
			.delay_on = 0,
			.delay_off = 0,
			.default_trigger = "timer",
		},
	},
};

static struct resource hi6421_led_resources = {
	.start		= REG_BASE_PMUSPI,
	.end			= REG_BASE_PMUSPI + REG_PMUSPI_IOSIZE - 1,
	.flags		= IORESOURCE_MEM,

};
static struct platform_device hi6421_led_device = {
	.name		= K3_LEDS,
	.id			= 0,
	.dev = {
		.platform_data = &hi6421_leds,
		.init_name = "hkled",
	},
	.num_resources		= 1,
	.resource       =  &hi6421_led_resources,
};
/*k3_led end*/
#endif

/* skf57909 2011/11/8 led add end */

#ifdef CONFIG_ANDROID_K3_VIBRATOR
/* skf57909 2011/11/8 vibrator add begin */
static struct k3_vibrator_platform_data hi6421_vibrator = {
	.low_freq  = PERIOD,
	.low_power = ISET_POWER,
	.mode  = SET_MODE,
	.high_freq = PERIOD_QUICK,
	.high_power = ISET_POWERSTRONG,
};

/*vibrator  begin*/
static struct resource hi6421_vibrator_resources = {
	.start		= REG_BASE_PMUSPI,
	.end		= REG_BASE_PMUSPI + REG_PMUSPI_IOSIZE - 1,
	.flags		= IORESOURCE_MEM,

};
static struct platform_device hi6421_vibrator_device = {
	.name		= K3_VIBRATOR,
	.id			= 0,
	.dev = {
		.platform_data = &hi6421_vibrator,
		.init_name = "hkvibrator",
	},
	.num_resources		= 1,
	.resource       =  &hi6421_vibrator_resources,
};
#endif

/*vibrator end*/
/* skf57909 2011/11/8 vibrator add end */

static struct resource hi6421_irq_resources[] = {
	{
		.start		= REG_BASE_PMUSPI,
		.end		= REG_BASE_PMUSPI + REG_PMUSPI_IOSIZE - 1,
		.flags		= IORESOURCE_MEM,
	},
	{
		.start		= IRQ_GPIO159,
		.end		= IRQ_GPIO159,
		.flags		= IORESOURCE_IRQ,
	},
};

static struct platform_device hisik3_hi6421_irq_device = {
	.name		= "hi6421-irq",
	.id			= 0,
	.dev.platform_data	= NULL,
	.num_resources		= ARRAY_SIZE(hi6421_irq_resources),
	.resource       =  hi6421_irq_resources,
};

#if defined(CONFIG_LCD_QIMEI_HJ101IA_01F_10)
static struct resource k3_lcd_resources[] = {
	{
		.name = GPIO_LCD_AVDD_NAME,
		.start = GPIO_LCD_AVDD,
		.end = GPIO_LCD_AVDD,
		.flags = IORESOURCE_IO,
	},
	{
		.name = GPIO_LED_EN_NAME,
		.start = GPIO_LED_EN,
		.end = GPIO_LED_EN,
		.flags = IORESOURCE_IO,
	},
	{
		.name = GPIO_LCD_ID0_NAME,
		.start = GPIO_LCD_ID0,
		.end = GPIO_LCD_ID0,
		.flags = IORESOURCE_IO,
	},
	{
		.name = GPIO_LCD_ID1_NAME,
		.start = GPIO_LCD_ID1,
		.end = GPIO_LCD_ID1,
		.flags = IORESOURCE_IO,
	},
    {
        .name = GPIO_CM_NAME,
        .start = GPIO_CM,
        .end = GPIO_CM,
        .flags = IORESOURCE_IO,
    },
    {
        .name = GPIO_CABC_NAME,
        .start = GPIO_CABC,
        .end = GPIO_CABC,
        .flags = IORESOURCE_IO,
    },
	{
		.name = GPIO_PWM0_NAME,
		.start = GPIO_PWM0,
		.end = GPIO_PWM0,
		.flags = IORESOURCE_IO,
	}, 
	{
		.name = GPIO_PWM1_NAME,
		.start = GPIO_PWM1,
		.end = GPIO_PWM1,
		.flags = IORESOURCE_IO,
	}, 
	{
		.name = REG_BASE_PWM0_NAME,
		.start = REG_BASE_PWM0,
		.end = REG_BASE_PWM0 + REG_PWM0_IOSIZE-1,
		.flags = IORESOURCE_MEM,
	},  
};

#else
static struct resource k3_lcd_resources[] = {
#if defined(CONFIG_LCD_PANASONIC_VVX10F002A00) || defined(CONFIG_LCD_QIMEI_HJ101IA_01F_10)
	[0] = {
		.name = GPIO_LCD_AVDD_NAME,
		.start = GPIO_LCD_AVDD,
		.end = GPIO_LCD_AVDD,
		.flags = IORESOURCE_IO,
	},
	[1] = {
		.name = GPIO_LED_EN_NAME,
		.start = GPIO_LED_EN,
		.end = GPIO_LED_EN,
		.flags = IORESOURCE_IO,
	},
#else
	[0] = {
		.name = GPIO_LCD_RESET_NAME,
		.start = GPIO_LCD_RESET,
		.end = GPIO_LCD_RESET,
		.flags = IORESOURCE_IO,
	},
	[1] = {
		.name = GPIO_LCD_POWER_NAME,
		.start = GPIO_LCD_POWER,
		.end = GPIO_LCD_POWER,
		.flags = IORESOURCE_IO,
	},
#endif
	[2] = {
		.name = GPIO_LCD_ID0_NAME,
		.start = GPIO_LCD_ID0,
		.end = GPIO_LCD_ID0,
		.flags = IORESOURCE_IO,
	},
	[3] = {
		.name = GPIO_LCD_ID1_NAME,
		.start = GPIO_LCD_ID1,
		.end = GPIO_LCD_ID1,
		.flags = IORESOURCE_IO,
	},
	[4] = {
		.name = GPIO_PWM0_NAME,
		.start = GPIO_PWM0,
		.end = GPIO_PWM0,
		.flags = IORESOURCE_IO,
	}, 
	[5] = {
		.name = GPIO_PWM1_NAME,
		.start = GPIO_PWM1,
		.end = GPIO_PWM1,
		.flags = IORESOURCE_IO,
	}, 
	[6] = {
		.name = REG_BASE_PWM0_NAME,
		.start = REG_BASE_PWM0,
		.end = REG_BASE_PWM0 + REG_PWM0_IOSIZE-1,
		.flags = IORESOURCE_MEM,
	},  
};
#endif

static struct platform_device k3_lcd_device = {
	.name = PLATFORM_DEVICE_LCD_NAME,
	.id	= 1,
	.dev = {
		.init_name = REGULATOR_DEV_LCD_NAME,
	},
	.num_resources = ARRAY_SIZE(k3_lcd_resources),
	.resource = k3_lcd_resources,
};


/* USB switch device define */
#define CONFIG_SWITCH_USB_TTY_NAME   "/dev/ttyUSB0" 

#define USB_SWITCH_CALL_ISR_PROC    0
#define USB_SWITCH_SET_VBUS_VALID   1
#define USB_SWITCH_SET_VBUS_INVALID 2

#define USB_SWITCH_CONTROL_GPIO     144
#define USB_SWITCH_EN_GPIO          174
#define USB_SWITCH_INTERRUPT_GPIO   99

static int send_at_cmd_to_cp(int atcmd)
{
	int ret = 0;
	char buf[32] = "AT+USBSWITCH=";
	mm_segment_t old_fs;
	struct file *filp;

	sprintf(buf+strlen(buf), "%d\r", atcmd);

	filp = filp_open(CONFIG_SWITCH_USB_TTY_NAME, O_WRONLY, 0755);
	if (IS_ERR(filp)) {
		pr_err("send_at_cmd_to_cp: failed to open %s with error: %ld\n",
			CONFIG_SWITCH_USB_TTY_NAME, PTR_ERR(filp));
		return -EIO;
	}

	old_fs = get_fs();
	set_fs(KERNEL_DS);
	if (filp->f_op != NULL && filp->f_op->write != NULL) {
		ret = filp->f_op->write(filp, buf, strlen(buf), 0);
		if (ret < 0) {
			pr_err("%s: failed to write %s with ret: %d\n",
				__func__, CONFIG_SWITCH_USB_TTY_NAME, ret);
		}
	}
	set_fs(old_fs);
	filp_close(filp, NULL);
	return ret;
}
#ifdef CONFIG_FSA880_I2C
extern int fsa880_switch_ap_or_modem(int value); 
#endif
#define USB_SWITCH_TO_AP 1
#define USB_SWITCH_TO_MODEM 0

int wait_for_cp_ready(void)
{
	struct file *filp;
	int ret = 0;
	const int MAX_RETRY_TIMES = 30;
	int retry_time = 0;
	pr_info("wait_for_cp_ready entry.\n");

	for (retry_time = 0; retry_time < MAX_RETRY_TIMES; retry_time++) {	
		filp = filp_open(CONFIG_SWITCH_USB_TTY_NAME, O_WRONLY, 0755);
		if (IS_ERR(filp)) {
			pr_err("%s: failed to open %s witch error: %ld"
			", retry time: %d.\n", __func__,
			CONFIG_SWITCH_USB_TTY_NAME, PTR_ERR(filp), retry_time);
			msleep(1000);
		} else {
			filp_close(filp, NULL);
			break;
		}
	}
	if (retry_time == MAX_RETRY_TIMES) {
		pr_err("%s: failed to open %s retry 30 times.\n",
			__func__, CONFIG_SWITCH_USB_TTY_NAME);
		ret = -EIO;
	}
	return ret;
}

int simulate_cp_usb_plug(int direction)
{
	int ret = 0;
	pr_info("simulate_cp_usb_plug, direction: %d\n", direction);

	if (direction == 0) {
		/* switch ap usb to cp usb */
		ret = send_at_cmd_to_cp(USB_SWITCH_SET_VBUS_VALID);
		if (ret < 0)
			goto Exit;
		mdelay(100);
		ret = send_at_cmd_to_cp(USB_SWITCH_CALL_ISR_PROC);
		if (ret < 0)
			goto Exit;

#ifdef CONFIG_FSA880_I2C
		ret = fsa880_switch_ap_or_modem(USB_SWITCH_TO_MODEM);
		if(ret < 0) {
			pr_err("%s,%d: switch usb to modem fail\n", __func__, __LINE__);
			goto Exit;		
		}
#endif
	} else if (direction == 1) {
		/* switch cp usb to ap usb */
		ret = send_at_cmd_to_cp(USB_SWITCH_SET_VBUS_INVALID);
		if (ret < 0)
			goto Exit;
		mdelay(100);
		ret = send_at_cmd_to_cp(USB_SWITCH_CALL_ISR_PROC);
		if(ret < 0)
			goto Exit;	
#ifdef CONFIG_FSA880_I2C
		ret = fsa880_switch_ap_or_modem(USB_SWITCH_TO_AP);
		if(ret < 0) {
			pr_err("%s,%d: switch usb to ap fail\n", __func__, __LINE__);
			goto Exit;		
		}
#endif
	}
Exit:
	return ret;
}

static struct usb_switch_platform_data usw_plat_data = {
		.name           = "usbsw",
		.usw_ctrl_gpio  = USB_SWITCH_CONTROL_GPIO,
		.usw_en_gpio    = USB_SWITCH_EN_GPIO,
		.usw_int_gpio   = USB_SWITCH_INTERRUPT_GPIO,
		.irq_flags      = IRQ_TYPE_EDGE_RISING,
		.wait_for_cp_ready = &wait_for_cp_ready,
		.simul_cp_usb_plug = &simulate_cp_usb_plug,
};

static struct platform_device usb_switch_device = {
	.name   = "switch-usb",
	.dev    = {
		.init_name = "switch-usb",
		.platform_data = &usw_plat_data,
	},
};



#if defined(CONFIG_MRUS51S) 
#include <linux/mux.h>
#include <linux/mrus51s.h>
int mrus51s_gpio_config(int mode)
{
	int ret = 0;
	if (mode == NORMAL) {	
		ret = iomux_block_set_work_mode(IO_MR_SENSOR_BLOCK_NAME);
		if (ret < 0) {
			printk(KERN_ERR "set mrus51s iomux to work mode failed\n");
			return ret;
		}    
	}
	else if (mode == LOWPOWER) {
		ret = iomux_block_set_lowpower_mode(IO_MR_SENSOR_BLOCK_NAME);
		if (ret < 0) {
			printk(KERN_ERR "set mrus51s iomux to low power mode failed\n");
			return ret;
		}    
	}   
	return ret;
}

struct mrus51s_platform_data_struct mrus51s_platform_data = {
	.gpio_config = mrus51s_gpio_config,
	.irq_gpio    = MRSENSOR_MRUS51S_IO,
};

struct platform_device mrus51s = {
	.name = "mrus51s",
	.dev  = {
		.init_name = "mrus51s",
		.platform_data = &mrus51s_platform_data,
	},
	.id = -1,
};
#endif

#if defined(CONFIG_SIM_DETECT)
#include <linux/mux.h>
#include <linux/sim_detect.h>

int sim_gpio_config(int mode)
{
	int ret = 0;
	if (mode == NORMAL) {
		ret = iomux_block_set_work_mode(IO_SIM_DECT_BLOCK_NAME);
		if (ret < 0) {
			printk(KERN_ERR "set sim card iomux to work mode failed\n");
			return ret;
		}
	}
	else if (mode == LOWPOWER) {
		ret = iomux_block_set_lowpower_mode(IO_SIM_DECT_BLOCK_NAME);
		if (ret < 0) {
			printk(KERN_ERR "set sim card iomux to low power mode failed\n");
			return ret;
		}
	}
	return ret;
}

static struct sim_detect_platform_data sim_pdata = {
		.det_gpio_config		= sim_gpio_config,
		.det_irq_gpio   = SIM_INT_IO,
};

static struct platform_device sim_detect_device = {
		.name	= "sim_detect",
		.dev  = {
			.init_name     = "sim_detect",
			.platform_data = &sim_pdata,
		},
		.id		= -1,
};
#endif

/* Begin: Added by d59977 for BCM GPS */
static struct resource k3_gps_bcm_resources[] = {
	[0] = {
	.name  = GPIO_GPS_BCM_EN_NAME,
	.start = GPIO_GPS_BCM_EN,
	.end   = GPIO_GPS_BCM_EN,
	.flags = IORESOURCE_IO,
	},
	[1] = {
	.name  = GPIO_GPS_BCM_RET_NAME,
	.start = GPIO_GPS_BCM_RET,
	.end   = GPIO_GPS_BCM_RET,
	.flags = IORESOURCE_IO,
	},
};

static struct platform_device k3_gps_bcm_device = {
	.name = "k3_gps_bcm_47511",
	.id	= 1,
	.dev = {
		.init_name = "gps_bcm_47511",
	},
	.num_resources = ARRAY_SIZE(k3_gps_bcm_resources),
	.resource = k3_gps_bcm_resources,
};
/* End: Added by d59977 for BCM GPS */

/*Begin:Added by g00124340 2011/09/19  for for bluetooth */

static struct resource bluepower_resources[] = {
	{
		.name	= "bt_gpio_enable",
		.start	= GPIO_BT_EN,
		.end	= GPIO_BT_EN,
		.flags	= IORESOURCE_IO,
	},
	{
		.name	= "bt_gpio_rst",
		.start	= GPIO_BT_RST,
		.end	= GPIO_BT_RST,
		.flags	= IORESOURCE_IO,
	},
};

static struct platform_device btbcm_device = {
	.name =	"bt_power",
	.dev  =	{
		.platform_data = NULL,
		.init_name = REGULATOR_DEV_BLUETOOTH_NAME,
	},
	.id	= -1,
	.num_resources	= ARRAY_SIZE(bluepower_resources),
	.resource	= bluepower_resources,

};

static struct resource bluesleep_resources[] = {
	{
		.name	= "gpio_host_wake",
		.start	= GPIO_HOST_WAKEUP,
		.end	= GPIO_HOST_WAKEUP,
		.flags	= IORESOURCE_IO,
	},
	{
		.name	= "gpio_ext_wake",
		.start	= GPIO_DEV_WAKEUP,
		.end	= GPIO_DEV_WAKEUP,
		.flags	= IORESOURCE_IO,
	},
};

static struct platform_device bcm_bluesleep_device = {
	.name =	"bluesleep",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(bluesleep_resources),
	.resource	= bluesleep_resources,
};
/*End:Added	by g00124340 2011/09/19	*/
/*  camera resources */
static struct resource hisik3_camera_resources[] = {
	{
		.name		= "isp_base",
		.start		= REG_BASE_ISP,
		.end		= REG_BASE_ISP + REG_ISP_IOSIZE - 1,
		.flags		= IORESOURCE_MEM,
	},
	{
		.name		= "isp_irq",
		.start		= IRQ_ISP,
		.end		= IRQ_ISP,
		.flags		= IORESOURCE_IRQ,
	},
	{
		.name		= "csi0_irq",
		.start		= IRQ_MIPICSI0,
		.end		= IRQ_MIPICSI0,
		.flags		= IORESOURCE_IRQ,
	},
	{
		.name		= "csi1_irq",
		.start		= IRQ_MIPICSI1,
		.end		= IRQ_MIPICSI1,
		.flags		= IORESOURCE_IRQ,
	}
};

static struct platform_device hisik3_camera_device = {
	.id	= 0,
	.name	= "k3-camera-v4l2",
	.dev = {
		.init_name = "camera",
	},
	.resource	= hisik3_camera_resources,
	.num_resources	= ARRAY_SIZE(hisik3_camera_resources),
};

static struct platform_device hisik3_fake_camera_device = {
	.id	= 1,
	.name	= "k3-fake-camera-v4l2",
	.resource	= 0,
	.num_resources	= 0,
	/*
	.dev = {
		.release = camera_platform_release,
	}
	,*/
};

/* Keypad device and platform data start, use KPC realizing keypad. */
static const uint32_t default_keymap[] = {
	/*row, col, key*/
#if 0
	/* used for truly platform.*/
	KEY(0, 0, KEY_MENU),
	KEY(1, 0, KEY_SEND),
	KEY(2, 0, KEY_VOLUMEUP),
	
	KEY(0, 1, KEY_HOME),
	KEY(1, 1, KEY_END),
	KEY(2, 1, KEY_VOLUMEDOWN),
	
	KEY(0, 2, KEY_CAMERA_FOCUS),
	KEY(1, 2, KEY_CAMERA),
	KEY(2, 2, DPAD_CENTER),
#endif

	/*row, col, key*/
	/* used for debug only.*/
	KEY(0, 0, KEY_MENU),
	KEY(0, 1, KEY_BACK),
        
	KEY(1, 0, KEY_LEFT),
	KEY(1, 1, KEY_RIGHT),
	
	KEY(2, 0, KEY_UP),
	
	KEY(2, 1, KEY_DOWN),
	KEY(2, 2, DPAD_CENTER),
	
	KEY(0, 2, KEY_CAMERA_FOCUS),
	KEY(1, 2, KEY_CAMERA),
	
	/* TODO: add your keys below*/

	/*Used for software function, not physical connection!*/
	
};

static struct matrix_keymap_data hisik3_keymap_data = {
	.keymap = default_keymap,
	.keymap_size = ARRAY_SIZE(default_keymap),
};
static uint16_t long_func_key1[] = {KEY_BACK};
static uint16_t long_func_key2[] = {DPAD_CENTER, KEY_VOLUMEDOWN};

static struct keypad_remap_item remap_items[] = {
	{KEY_HOME, 1, 1000/*ms*/, long_func_key1},
	/*{KEY_A, 2, 500, long_func_key2},*/
	/*TODO: add your remap_item here*/
};

static struct keypad_remap keypad_long_remap = {
	.count = ARRAY_SIZE(remap_items),
	.items = remap_items,
};

static struct resource hisik3_keypad_resources[] = {
	[0] = {
		.start = REG_BASE_KPC,
		.end = REG_BASE_KPC + REG_KPC_IOSIZE - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_KPC,
		.end = IRQ_KPC,
		.flags = IORESOURCE_IRQ,
	},
};

static struct k3v2_keypad_platdata hisik3_keypad_platdata = {
	.keymap_data = &hisik3_keymap_data,
	.keypad_remap = &keypad_long_remap,
	.rows = 8,
	.cols = 8,
	.row_shift = 3,
};

static struct platform_device hisik3_keypad_device = {
	.name = "k3_keypad",
	.id = -1,
	.num_resources = ARRAY_SIZE(hisik3_keypad_resources),
	.resource = hisik3_keypad_resources,
	.dev.platform_data = &hisik3_keypad_platdata,
};

/* Keypad device and platform data start, use GPIO realizing keypad. */

static struct resource hisik3_gpio_keypad_resources[] = {
	[0] = {
		.start = REG_BASE_GPIO18,
		.end = REG_BASE_GPIO18 + REG_GPIO18_IOSIZE - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_GPIO(GPIO_17_1),
		.end = IRQ_GPIO(GPIO_17_1),
		.flags = IORESOURCE_IRQ,
	},
	[2] = {
		.start = IRQ_GPIO(GPIO_17_2),
		.end = IRQ_GPIO(GPIO_17_2),
		.flags = IORESOURCE_IRQ,
	},
};

static void s10_gpio_version_init(void)
{
    iomux_block_set_gpio_mode(IO_MINOR_VERSION_BLOCK_NAME);
}
static void s10_gpio_version_exit(void)
{
    iomux_block_set_lowpower_mode(IO_MINOR_VERSION_BLOCK_NAME);
}

static struct s10_gpio_version_platdata s10_gpio_version_data = {
    .gpio1 = VERSION_1_GPIO,
    .gpio2 = VERSION_2_GPIO,
    .init = s10_gpio_version_init,
    .exit = s10_gpio_version_exit,
};

static struct platform_device hisik3_gpio_version_device = {
    .name = "k3v2_version",
    .id = -1,
    .dev = {
        .platform_data = &s10_gpio_version_data,
	 .init_name = "s10_gpio_version",
    },
};

#if S10_HWID_L3H(S10, S10101, A)
static int s10_gpio_key_init(void)
{
   return iomux_block_set_work_mode(IO_KPC_BLOCK_NAME);
}

static int s10_gpio_key_exit(void)
{
    return iomux_block_set_lowpower_mode(IO_KPC_BLOCK_NAME);
}

static struct s10_gpio_key_platdata s10_gpio_key_data = {
    .vol_up 	= VOL_UP,
    .vol_down 	= VOL_DOWN,
    .init = s10_gpio_key_init,
    .exit = s10_gpio_key_exit,
};
#endif

static struct platform_device hisik3_gpio_keypad_device = {
	.name = "k3v2_gpio_key",
	.id = -1,
    #if S10_HWID_L3H(S10, S10101, A)
    .dev = {
		.platform_data = &s10_gpio_key_data,
		.init_name = "s10_gpio_key",
	},
    #endif
	.num_resources = ARRAY_SIZE(hisik3_gpio_keypad_resources),
	.resource = hisik3_gpio_keypad_resources,
};

/*power-key*/
static struct resource hisik3_power_key_resources[] = {
	[0] = {
		.start = REG_BASE_PMUSPI,
		.end = REG_BASE_PMUSPI + REG_PMUSPI_IOSIZE - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_POWER_KEY_PRESS,
		.end = IRQ_POWER_KEY_PRESS,
		.flags = IORESOURCE_IRQ,
	},
	[2] = {
		.start = IRQ_POWER_KEY_RELEASE,
		.end = IRQ_POWER_KEY_RELEASE,
		.flags = IORESOURCE_IRQ,
	},
	[3] = {
		.start = IRQ_POWER_KEY_LONG_PRESS_1S,
		.end = IRQ_POWER_KEY_LONG_PRESS_1S,
		.flags = IORESOURCE_IRQ,
	},
};

static struct platform_device hisik3_power_key_device = {
	.name = "k3v2_power_key",
	.id = -1,
	.num_resources = ARRAY_SIZE(hisik3_power_key_resources),
	.resource = hisik3_power_key_resources,
};

/*watchdog added by s00212129*/
static struct resource  hisik3_watchdog_resources[] = {
	[0] = {
		.start = REG_BASE_WD,
		.end = REG_BASE_WD + REG_WD_IOSIZE -1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_WDOG,
		.end   = IRQ_WDOG,
		.flags = IORESOURCE_IRQ,
	},
};

static struct platform_device  hisik3_watchdog_device = {
	.name = "k3v2_watchdog",
	.id = -1,
	.num_resources = ARRAY_SIZE(hisik3_watchdog_resources),
	.resource = hisik3_watchdog_resources,
};
 /*end watchdog*/

/*Keypad backlight*/
static struct platform_device hisik3_keypad_backlight_device = {
	.name = "keyboard-backlight",
};

#ifdef CONFIG_BOOST_5V
static  int  boost_5v_init_gpio(unsigned gpio)
{
	int ret =0;
	ret = gpio_request(gpio, "boost_5v_gpio");
	if (ret < 0) 
	{
		printk(KERN_ERR " boost_5v control gpio  request fail 0x%x\n",ret);
		return ret;
	}
	ret = gpio_direction_output(gpio,0);
	if(ret)
	{
		printk(KERN_ERR " boost_5v control gpio  output fail 0x%x\n",ret);
		gpio_free(gpio);
	}
	return ret;	
}

static void boost_5v_exit_gpio(unsigned gpio)
{
	gpio_free(gpio);
}


static int boost_5v_io_block_init(struct boost_5v_platdata* pdata)
{
	if(pdata && pdata->io_block_name)
	{
		return iomux_block_set_work_mode(pdata->io_block_name);
	}
	return -EINVAL;
}

static int boost_5v_io_block_exit(struct boost_5v_platdata* pdata)
{
	if(pdata && pdata->io_block_name)
	{
		return iomux_block_set_lowpower_mode(pdata->io_block_name);
	}
	return -EINVAL;
}

static struct boost_5v_platdata  hisik3_boost_5v_platdata = {
	.switch_boost_gpio =  SWITCH_BOOST_5V_GPIO,
	.init_gpio               =  boost_5v_init_gpio,
	.exit_gpio              =  boost_5v_exit_gpio,
	.io_block_name      =  IO_BOOST_5V_BLOCK_NAME,
	.io_mux_block_init  =  boost_5v_io_block_init,
	.io_mux_block_exit =  boost_5v_io_block_exit,
};

static struct platform_device  hisik3_boost_5v_device = {
	.name = "boost_5v",
	.id = -1,
	.dev.platform_data = &hisik3_boost_5v_platdata,
};
#endif

#if defined(CONFIG_SUPPORT_MICRO_USB_PORT)
static int micro_usb_init_gpio(void)
{
	int ret = 0;
	ret = gpio_request(USB_AP_5V_EN_GPIO, NULL);
	if (ret < 0) 
	{
		printk(KERN_ERR " micro usb power control module request usb_ap_en_gpio fail 0x%x\n",ret);
		return ret;
	}
	gpio_direction_output(USB_AP_5V_EN_GPIO, 1);
//wuxinxian>>
	ret = gpio_request(MODEM_USB_CTRL_GPIO, NULL);
	if (ret < 0) 
	{
		printk(KERN_ERR " micro usb power control module request usb_ap_en_gpio fail 0x%x\n",ret);
		gpio_free(USB_AP_5V_EN_GPIO);
		return ret;
	}
	gpio_direction_output(MODEM_USB_CTRL_GPIO, 0);
	ret = gpio_request(USB_VBUS_5V_EN_GPIO, NULL);
	if(ret < 0)
	{
		printk(KERN_ERR " micro usb power control module request usb_out5v_en_gpio fail 0x%x\n",ret);
		gpio_free(MODEM_USB_CTRL_GPIO);
		gpio_free(USB_AP_5V_EN_GPIO);
		return ret;
	}
	gpio_direction_output(USB_VBUS_5V_EN_GPIO, 0);
//wuxinxian<<	
	return ret;
}

static void micro_usb_exit_gpio(void)
{
	gpio_free(USB_AP_5V_EN_GPIO);
	gpio_free(MODEM_USB_CTRL_GPIO);
	gpio_free(USB_VBUS_5V_EN_GPIO);
}

static int micro_usb_io_block_init(struct micro_usb_power_control_platform_data *pdata)
{
	if(pdata && pdata->io_block_name)
	{
		return iomux_block_set_work_mode(pdata->io_block_name);
	}
	return -EINVAL;
}

static int micro_usb_io_block_exit(struct micro_usb_power_control_platform_data *pdata)
{
	if(pdata && pdata->io_block_name)
	{
		return iomux_block_set_lowpower_mode(pdata->io_block_name);
	}
	return -EINVAL;
}

static struct micro_usb_power_control_platform_data  micro_usb_power_control_platdata = {
	.usb_ap5v_en_gpio =  USB_AP_5V_EN_GPIO,
	.modem_usb_control_gpio = MODEM_USB_CTRL_GPIO,
	.usb_current_warning_gpio = USB_CURRENT_WARNING_GPIO,
	.irq_type = IRQF_TRIGGER_FALLING,
	.usb_id_detect_gpio      =  USB_ID_DETECT_GPIO,
	.usb_id_detect_irq_type = IRQF_TRIGGER_FALLING |IRQF_NO_SUSPEND,
	.usb_vbus5v_en_gpio = USB_VBUS_5V_EN_GPIO,
	.init_gpio = micro_usb_init_gpio,
	.exit_gpio = micro_usb_exit_gpio,
	.io_block_name = IO_MICRO_USB,
	.io_mux_block_init = micro_usb_io_block_init,
	.io_mux_block_exit = micro_usb_io_block_exit,
};
static struct platform_device micro_usb_power_control_device = {
	.name = "micro_usb_power_control",
	.id = -1,
	.dev.platform_data = &micro_usb_power_control_platdata,
};
#endif

#ifdef CONFIG_FSA880_I2C
static int fairchild_fsa880_init_gpio(void)
{
	int ret =0;
	ret = gpio_request(SWITCH_OUT5V_GPIO, NULL);
	if (ret < 0) 
	{
		printk(KERN_ERR " fsa880 request switch out_5v fail 0x%x\n",ret);
		return ret;
	}
		if(get_product_feature(PROD_FEATURE_USB_OUT_5V_SUPPLY)) 
		{
			gpio_direction_output(SWITCH_OUT5V_GPIO, 0);
		}
		else
		{
			gpio_direction_output(SWITCH_OUT5V_GPIO, 1);
		}
	
	ret = gpio_request(SWITCH_AP5V_GPIO, NULL);
	if (ret < 0) 
	{
		printk(KERN_ERR " fsa880 request switch ap_5v fail 0x%x\n",ret);
		gpio_free(SWITCH_OUT5V_GPIO);
		return ret;
	}
	gpio_direction_output(SWITCH_AP5V_GPIO, 0);	

	ret = gpio_request(SWITCH_USB_ID_GPIO, NULL);
	if (ret < 0) 
	{
		printk(KERN_ERR " fsa880 request switch usb id gpio  fail 0x%x\n",ret);
		gpio_free(SWITCH_OUT5V_GPIO);
		gpio_free(SWITCH_AP5V_GPIO);
		return ret;
	}
	gpio_direction_output(SWITCH_USB_ID_GPIO, 0);	

	ret = gpio_request(BALONG_USE_GPIO, NULL);
	if (ret < 0) 
	{
		printk(KERN_ERR " fsa880 request  balong use gpio  fail 0x%x\n",ret);
		gpio_free(SWITCH_OUT5V_GPIO);
		gpio_free(SWITCH_AP5V_GPIO);
		gpio_free(SWITCH_USB_ID_GPIO);
		return ret;
	}
	gpio_direction_output(BALONG_USE_GPIO, 0);	
				
	return ret;
}

static void fairchild_fsa880_exit_gpio(void)
{
	gpio_free(SWITCH_OUT5V_GPIO);
	gpio_free(SWITCH_AP5V_GPIO);
	gpio_free(SWITCH_USB_ID_GPIO);
	gpio_free(BALONG_USE_GPIO);
}

static int fsa880_io_block_init(struct fairchild_fsa880_platform_data* pdata)
{
	if(pdata && pdata->io_block_name)
	{
		return iomux_block_set_work_mode(pdata->io_block_name);
	}
	return -EINVAL;
}

static int fsa880_io_block_exit(struct fairchild_fsa880_platform_data* pdata)
{
	if(pdata && pdata->io_block_name)
	{
		return iomux_block_set_lowpower_mode(pdata->io_block_name);
	}
	return -EINVAL;
}


static struct fairchild_fsa880_platform_data  fsa880_platform_data =
{
	.intr_gpio                    =  FSA880_INTR_GPIO,
	.switch_out5v_gpio      =  SWITCH_OUT5V_GPIO,
	.switch_ap5v_gpio       =  SWITCH_AP5V_GPIO,
	.switch_usb_id_gpio     =  SWITCH_USB_ID_GPIO,
	.usb_current_warning_gpio = USB_CURRENT_WARNING_GPIO,
	.balong_use_gpio        =  BALONG_USE_GPIO,
	.irq_type                     =  IRQF_TRIGGER_FALLING|IRQF_NO_SUSPEND,	
	.current_warning_irq_type = IRQF_TRIGGER_FALLING,
	.io_block_name	      =  IO_BC11_BLOCK_NAME,
	.io_mux_block_init       =  fsa880_io_block_init,
	.io_mux_block_exit      =  fsa880_io_block_exit,
	.init_gpio                     =  fairchild_fsa880_init_gpio,
	.exit_gpio                    =  fairchild_fsa880_exit_gpio,
};
#endif

/* TouchScreen start*/
#if defined(CONFIG_TOUCHSCREEN_RMI4) || defined(CONFIG_TOUCHSCREEN_RMI4_MODULE) \
 || defined(CONFIG_TOUCHSCREEN_MXT1386) || defined(CONFIG_TOUCHSCREEN_MXT1386_MODULE)
static int tp_gpio_config(int mode)
{
	int ret = 0;
    if (mode == NORMAL) {
		ret = iomux_block_set_work_mode(IO_TS_BLOCK_NAME);
		if (ret < 0) {
			printk(KERN_ERR "[%s]: set tp gpio NORMAL error. ret %d",__func__ ,ret);
            return -1;
	    }    
    } else if (mode == LOWPOWER) {    
	    ret = iomux_block_set_lowpower_mode(IO_TS_BLOCK_NAME);
	    if (ret < 0) {
	        printk(KERN_ERR "[%s]: set tp gpio LOWPOWER error. ret %d",__func__ ,ret);
	        return -1;
	    }    
    } else {
		printk(KERN_ERR "do not support the mode %d", mode);
        return -1;
    }
    return 0;
    
}

static int tp_gpio_request(int on)
{
	int ret = 0;
    if (on == 1) {
       
	    ret = gpio_request(GPIO_7_5, "tp_1v8_3v3");
	    if (ret < 0) {
	            printk(KERN_ERR "%s: gpio request GPIO_061 error\n", __func__);
	            goto error_request_power;
	        }
	    
	    ret = gpio_request(GPIO_19_4, "tp_reset");
	    if (ret) {
	        printk(KERN_ERR "%s: failed to request gpio for reset\n", __func__);
	        goto err_request_reset_pin;
	    }
	    
	    
	    ret = gpio_request(GPIO_19_5, "tp_int");
	    if (ret < 0) {
	        printk(KERN_ERR "%s: failed to request gpio for irq\n", __func__);
	        goto err_request_irq_pin;
	    }
	} else if (on == 0) {
	    gpio_free(GPIO_19_5);
	    gpio_free(GPIO_19_4);
	    gpio_free(GPIO_7_5);
	} else {
		printk(KERN_ERR "the argument is error\n");
    }

    return 0;
    
err_request_irq_pin:
    gpio_free(GPIO_19_4);
err_request_reset_pin:
    gpio_free(GPIO_7_5);
error_request_power:  
    //gpio_free(GPIO_0_6);  //no request no free.
    return ret;
}

#define GPIO_CTP_INT  GPIO_19_5
#define GPIO_CTP_RST  GPIO_19_4
static int tp_gpio_get_value(void)
{
	return gpio_get_value(GPIO_CTP_INT);
}
extern int tp_probe_status;
static int tp_gpio_reset(const char *name)
{
	if (!strcmp(name, "atmel")) {	
	    gpio_direction_output(GPIO_CTP_RST, 0);
	    msleep(100);
	    gpio_direction_output(GPIO_CTP_RST, 1);
	    msleep(100);
	    gpio_direction_output(GPIO_CTP_RST, 0);
    } else {    	    
	    gpio_direction_output(GPIO_CTP_RST, 0);
	    msleep(5);
	    gpio_direction_output(GPIO_CTP_RST, 1);
	}
    //If tp is found,tp_probe_status = 1,or tp_probe_status = 0;
    if(!tp_probe_status){
        printk(KERN_ERR "%s,Delay here for start!\n",__func__);
        msleep(100);
    }
	return 1;
}
static int tp_set_power(int on, const char* name )
{
	int ret = 0;

	if ((on != 0) && (on != 1)) {
		printk(KERN_ERR "[%s] the argument is error\n", __func__);
        return -1;
    }    

#ifdef CONFIG_BOOST_5V   
   request_5v_boost(on,BOOST_5V_TP);
#endif
   //set GPIO_61 high,apply the 1v8,3v3 
   //hardware bug. keep the 1v8 on
   ret = gpio_direction_output(GPIO_7_5, on);
   if (ret < 0) {
       printk(KERN_ERR "%s: set GPIO_061 output 1 error\n", __func__);
   }

    if (on) {
	    //wait the voltage stably 
	    //msleep(100);
	    if(!tp_probe_status){
            printk(KERN_ERR "%s,Delay here for start!\n",__func__);
            msleep(100);
        }
	    tp_gpio_reset(name);

		//set gpio_int to input
	    gpio_direction_input(GPIO_19_5);
    }
	//msleep(100);
    return 0;
}
#endif

#if defined(CONFIG_TOUCHSCREEN_SYNAPTICS_I2C_RMI4_K3V2) \
    || defined(CONFIG_TOUCHSCREEN_SYNAPTICS_I2C_RMI4_K3V2_MODULE)
static struct synaptics_rmi4_platform_data synaptics_ts_platform_data = {
	.irq				= GPIO_19_5,
	.irq_flag			= IRQF_TRIGGER_LOW,
	.flip_flags			= SYNAPTICS_NO_FLIP,
	.x_res				= LCD_X_720P,
	.y_res				= LCD_Y_720P,
	.y_all				= LCD_Y_ALL_720P,
	.fuzz_x				= 0,
	.fuzz_y				= 0,
	.fuzz_p				= 0,
	.fuzz_w				= 0,
	.reset_pin			= GPIO_19_4,
};

static ssize_t synaptics_virtual_keys_show(struct kobject *kobj,
			       struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf,
		__stringify(EV_KEY) ":" __stringify(KEY_BACK)   ":125:1380:160:90"
		":" __stringify(EV_KEY) ":" __stringify(KEY_HOMEPAGE)   ":360:1380:160:90"
		":" __stringify(EV_KEY) ":" __stringify(KEY_MENU) ":598:1380:160:90"
		"\n");
}

static struct kobj_attribute synaptics_virtual_keys_attr = {
	.attr = {
		.name = "virtualkeys.synaptics",
		.mode = S_IRUGO,
	},
	.show = &synaptics_virtual_keys_show,
};

static struct attribute *synaptics_properties_attrs[] = {
	&synaptics_virtual_keys_attr.attr,
	NULL
};

static struct attribute_group synaptics_properties_attr_group = {
	.attrs = synaptics_properties_attrs,
};
	
static void __init synaptics_virtual_keys_init(void)
{
	struct kobject *properties_kobj;
	int ret = 0;

	properties_kobj = kobject_create_and_add("board_properties", NULL);
	if (properties_kobj)
		ret = sysfs_create_group(properties_kobj,
			&synaptics_properties_attr_group);
	if (!properties_kobj || ret)
		pr_err("%s: failed to create board_properties!\n", __func__);
}

#elif defined(CONFIG_TOUCHSCREEN_RMI4) || defined(CONFIG_TOUCHSCREEN_RMI4_MODULE)
static struct rmi_device_platform_data rmi4_pdata = {
	.driver_name        = "rmi_generic",
	.attn_gpio				= GPIO_19_5,
	.attn_polarity = RMI_ATTN_ACTIVE_LOW,
    //.gpio_config 		= NULL,
    //.f11_ctrl 		= NULL,
    //.button_map 		= NULL,
	.reset_pin			= GPIO_19_4,
	.gpio_config		= tp_gpio_config,
	.set_power          = tp_set_power,
	.tp_gpio_request	= tp_gpio_request,
	.tp_gpio_get_value  = tp_gpio_get_value,
	.tp_gpio_reset 	    = tp_gpio_reset,
	.reset_delay_ms     = 50,
};

#endif
/* TouchScreen end*/

#if defined(CONFIG_TOUCHSCREEN_MXT224E) || defined(CONFIG_TOUCHSCREEN_MXT224E_MODULE)
/* Atmel mXT224E Touchscreen start*/
static struct atmel_i2c_platform_data atmel_tp_platform_data = {
	.version = 0x10,
	.source = 0,
	.abs_x_min = 0,
	.abs_x_max = 719,
	.abs_y_min = 0,
	.abs_y_max = 1279,
	.abs_pressure_min = 0,
	/*.abs_pressure_max = 15,*/
	.abs_pressure_max = 255,
	.abs_width_min = 0,
	.abs_width_max = 255,
	/*.abs_area_min = 0,*/
	/*.abs_area_max = 255,*/
	.gpio_irq = GPIO_19_5,
	.gpio_reset = GPIO_19_4,
	.power = NULL,
	.config_T6 = {
		0, 0, 0, 0, 0,
		0
	},
	.config_T7 = {
		32, 12, 5
	},
	.config_T8 = {
		24, 0, 5, 5, 0,
		0, 5, 60, 10, 192
	},
	.config_T9 = {
		139, 0, 0, 19, 11,
		0, 32, 60, 1, 3,
		0, 5, 2, 47, 10,
		15, 22, 10, 106, 5,/*XRANGE = 1386*/
		207, 2, 0, 0, 2,/* YRANGE = 719*/
		2, 161, 40, 183, 64,
		30, 20, 0, 0, 0
	},
	.config_T15 = {
		0, 0, 0, 0, 0,
		0, 0, 0, 0, 0,
		0
	},
	.config_T19 = {
		0, 0, 0, 0, 0,
		0, 0, 0, 0, 0,
		0, 0, 0, 0, 0,
		0
	},
	.config_T23 = {
		0, 0, 0, 0, 0,
		0, 0, 0, 0, 0,
		0, 0, 0, 0, 0
	},
	.config_T25 = {
		0, 0, 0, 0, 0,
		0, 0, 0, 0, 0,
		0, 0, 0, 0
	},
	.config_T40 = {
		0, 0, 0, 0, 0
	},
	.config_T42 = {
		3, 40, 60, 80, 128,
		0, 0, 0
	},
	.config_T46 = {
		0, 3, 63, 63, 0,
		0, 0, 0, 0
	},
	.config_T47 = {
		0, 20, 50, 5, 2,
		40, 40, 180, 0, 100
	},
	.config_T48 = {
		1, 4, 66, 1, 0,
		0, 0, 0, 7, 6,
		0, 0, 0, 6, 6,
		0, 0, 63, 6, 64,
		10, 0, 20, 5, 0,
		38, 0, 20, 0, 0,
		0, 0, 0, 0, 0,
		40, 2, 5, 2, 32,
		10, 12, 20, 241, 251,
		0, 0, 191, 40, 183,
		64, 30, 15, 0
	},
	.object_crc = {
		0xFD, 0x3B, 0x8D
	},/*CRC*/
	.cable_config = {
		70, 30, 32, 32
	},
	.cable_config_T7 = {
		32, 16, 25
	},
	.cable_config_T8 = {
		24, 0, 5, 5, 0,
		0, 5, 60, 10, 192
	},
	.cable_config_T9 = {
		139, 0, 0, 19, 11,
		0, 32, 60, 2, 3,
		0, 5, 2, 64, 10,
		12, 20, 10, 106, 5,/*XRANGE = 1386*/
		207, 2, 0, 0, 2,/* YRANGE = 719*/
		2, 161, 40, 183, 64,
		30, 20, 0, 0, 0
	},
	.cable_config_T46 = {
		0, 3, 40, 40, 0,
		0, 0, 0, 0
	},

	.cable_config_T48 = {
		1, 128, 114, 1, 0,
		0, 0, 0, 7, 6,
		0, 0, 0, 6, 6,
		0, 0, 63, 6, 64,
		10, 0, 20, 5, 0,
		38, 0, 20, 0, 0,
		0, 0, 0, 0, 0,
		40, 2, 5, 2, 32,
		10, 12, 20, 241, 251,
		0, 0, 191, 40, 183,
		64, 30, 15, 0
	},
	.noise_config = {
		70, 3, 35
	},
	.filter_level = {
		0, 0, 539, 539
	},
	.GCAF_level = {
		8, 16, 24, 32, 40
	},
	.ATCH_NOR = {
		0, 0, 5, 60, 10,
		192
	},
	.ATCH_NOR_20S = {
		0, 0, 255, 1, 0,
		0
	},
};
#elif defined(CONFIG_TOUCHSCREEN_MXT1386) || defined(CONFIG_TOUCHSCREEN_MXT1386_MODULE)
static struct atmel_i2c_platform_data atmel_tp_platform_data = {
    .version = 0x10,
	.abs_x_min = 0x00,
	.abs_x_max = 1919, 
	.abs_y_min = 0x00,
	.abs_y_max = 1199, 
	.abs_area_min = 0x00,
	.abs_area_max = 0xff,
	.abs_pressure_min = 0,
	.abs_pressure_max = 0xF,
	.gpio_irq = GPIO_19_5,
    .gpio_config  = tp_gpio_config,
    .set_power    = tp_set_power,
    .tp_gpio_request = tp_gpio_request,
    .tp_gpio_get_value = tp_gpio_get_value,
    .tp_gpio_reset 	   = tp_gpio_reset,
	.config_T7 = {255, 255, 65},
	.config_T8 = {10, 0, 20, 10, 0, 0, 1, 50, 48, 40},
	.config_T9 = {
		143, 0, 0, 33, 42,
		0, 16, 55, 3, 1,  //switch x and y
		0, 40, 5, 48, 10,  /*the max finger is 10*/
		0, 0, 0, 175,4,   /*XRANGE = 1199*/
		127,7, 0, 0, 0,   /*YRANGE = 1919*/
		0, 64, 0, 64, 0,
		15, 18, 0, 0
	},
	.config_T15 = {	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
    .config_T18 = { 0, 0},
	.config_T22 = {5,  0, 0, 0, 0, 0, 0, 0, 25, 0, 0, 0, 10, 20, 35,  50, 0},
	.config_T24 = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
	.config_T25 = {	0, 0, 0, 0, 0, 0, 0, 0, 0, 0,0, 0, 0, 0	},
	.config_T27 = {0, 0, 0, 0, 0, 0, 0},
	.config_T28 = { 0, 0, 0, 0, 0, 0},
	.config_T40 = {	0, 0, 0, 0, 0},
	.object_crc = {
		0x41, 0x0e, 0x90
	},/*CRC*/
	.cable_config = {
		70, 30, 32, 32
	},
	.cable_config_T7 = {
		48, 255, 25
	},
	.cable_config_T8 = {
		39, 0, 20, 20, 0,
		0, 255, 1, 0, 0
	},
	.cable_config_T9 = {
		139, 0, 0, 33, 42, 
		0, 32, 60, 2, 3,
		0, 5, 2, 1, 10,   
		10, 10, 10, 127, 7, /*XRANGE = 1919*/
		175, 4, 0, 0, 0,	/*YRANGE = 1199*/
		0, 128, 0, 128, 0,
		0, 10, 0, 0 
	},
	.cable_config_T22 = {5,  0, 0, 0, 0, 0, 0, 0, 25, 0, 0, 0, 10, 20, 35,  50, 0},
	.cable_config_T28 = { 0, 0, 0, 8, 8, 60},
	.GCAF_level = {
		8, 16, 24, 32, 40
	},
};

#endif

/* Atmel mXT224E Touchscreen end*/

/* Audience */

#ifdef CONFIG_AUDIENCE
static struct es305_platform_data audience_platform_data = {
	.gpio_es305_wakeup  = GPIO_16_0,/* 128 */
	.gpio_es305_reset   = GPIO_18_1,/* 145 */
};
#endif
/* TPA2028_SPK_L */
static struct tpa2028_l_platform_data tpa2028_l_pdata = {
	.gpio_tpa2028_en    = GPIO_14_5,/* 117 */
};

/* TPA2028_SPK_R */
static struct tpa2028_r_platform_data tpa2028_r_pdata = {
    //.gpio_tpa2028_en    = GPIO_14_5,/* 117 */
};

/* TPA6132 */
static struct tpa6132_platform_data tpa6132_pdata = {
	.gpio_tpa6132_en    = GPIO_14_6,/* 118 */
};

static struct platform_device tpa6132_device = {
	.name    = TPA6132_NAME,
	.id      = 0,
	.dev     = {
		.platform_data = &tpa6132_pdata,
	},
};

#ifdef CONFIG_HIK3_CAMERA_FLASH
static struct tps61310_platform_data tps61310_platform_data = 
{
	.reset_pin			= GPIO_9_4,
	.strobe0			= GPIO_8_1,
	.strobe1			= GPIO_8_2,
};
#endif
void ad7147_gpio_config(int value)
{
    char * io_mux_block_ad7147 = IO_CAPACITOR_SENSOR_BLOCK_NAME;
	int ret = 0;
    ret = iomux_block_set_work_mode(io_mux_block_ad7147);

    if (ret < 0) {
		printk(KERN_ERR "[%s %d]: set gpio error\n", __func__, __LINE__);
    }    

}

static struct ad7147_platform_data ad7147_platform_data = 
{
	.ad7147_gpio_config = ad7147_gpio_config,
	.ad7147_int_gpio = AD7147_INT_GPIO,
};
static struct platform_device boardid_dev ={
    .name    = "boardid_dev",
    .id      = 0,
};

#ifdef CONFIG_BATTERY_K3
static int hi6421_batt_table[] = {
	/* adc code for temperature in degree C */
	929, 925, /* -2 ,-1 */
	920, 917, 912, 908, 904, 899, 895, 890, 885, 880, /* 00 - 09 */
	875, 869, 864, 858, 853, 847, 841, 835, 829, 823, /* 10 - 19 */
	816, 810, 804, 797, 790, 783, 776, 769, 762, 755, /* 20 - 29 */
	748, 740, 732, 725, 718, 710, 703, 695, 687, 679, /* 30 - 39 */
	671, 663, 655, 647, 639, 631, 623, 615, 607, 599, /* 40 - 49 */
	591, 583, 575, 567, 559, 551, 543, 535, 527, 519, /* 50 - 59 */
	511, 504, 496 /* 60 - 62 */
};

static struct k3_battery_monitor_platform_data hi6421_bci_data = {
	.termination_currentmA		=CIN_LIMIT_100,
	.monitoring_interval		= MONITOR_TIME,
	.max_charger_currentmA		= HIGH_CURRENT,
	.max_charger_voltagemV		= HIGH_VOL,
	.max_bat_voltagemV		= BAT_FULL_VOL,
	.low_bat_voltagemV		= BAT_SLOW_VOL,
	.battery_tmp_tbl		= hi6421_batt_table,
	.tblsize			= ARRAY_SIZE(hi6421_batt_table),
};

static struct resource hisik3_battery_resources[] = {
	[0] = {
		.start  = IRQ_VBATLOW_RISING,
		.name   = NULL,
		.flags  = IORESOURCE_IRQ,
	} ,
};

static struct platform_device hisik3_battery_monitor = {
	.name	= "k3_battery_monitor",
	.id	= 1,
	.resource	= hisik3_battery_resources,
	.num_resources	= ARRAY_SIZE(hisik3_battery_resources),
	.dev = {
		.platform_data	= &hi6421_bci_data,
	},
};
#endif


#ifdef CONFIG_PSY_MAX8903
static int max8903_io_block_init(struct max8903_platform_data* pdata)
{
    if(pdata->io_block_name){
        return iomux_block_set_work_mode(pdata->io_block_name);
    }
    return -EINVAL;
}

static int max8903_io_block_exit(struct max8903_platform_data* pdata)
{
    if(pdata->io_block_name){
        return iomux_block_set_lowpower_mode(pdata->io_block_name);
    }
    return -EINVAL;
}
static struct max8903_platform_data max8903_platform_data =
{
    .cen                    = MAX8903_CHARGING_EN,    /* Charger Enable input */
    .dok                    = MAX8903_DOK,            /* DC(Adapter) Power OK output */
    .uok                    = NULL,                   /* USB Power OK output */
    .chg                    = MAX8903_CHG,            /* Charger status output */
    .flt                    = MAX8903_CHARGING_FAT_INTR,    /* Fault output */
    .dcm                    = 0,    /* Current-Limit Mode input (1: DC, 2: USB) */
    .usus                   = 0,   /* USB Suspend Input (1: suspended) */
    .dc_valid               = 1,
    .usb_valid              = 0,
    .dcm_ctrl0              = MAX8903_CHARGING_CTRL0,
    .dcm_ctrl1              = MAX8903_CHARGING_CTRL1,
    .usb_max_current        = CHARGING_CURRENT_LEVEL0,
    .usb_cdp_max_current    = CHARGING_CURRENT_LEVEL1,
    .usb_dcp_max_current    = CHARGING_CURRENT_LEVEL2,
    .usb_ac_max_current     = CHARGING_CURRENT_LEVEL2,
    .io_block_name          = IO_CHARGER_BLOCK_NAME,
    .io_mux_block_init      = max8903_io_block_init,
    .io_mux_block_exit      = max8903_io_block_init,
    .gpio_block_init        = NULL,
    .gpio_block_exit        = NULL,
};

static struct platform_device k3_max8903_device = {
	.name	= "max8903_charger",
	.id	= 0,
	.dev	= {
		.platform_data = &max8903_platform_data,
		.init_name = "max8903_charger",
	},
};
#endif

//#if defined(S10_PRODUCT_LINK)
#ifdef CONFIG_PSY_MONITOR
static struct S10_psy_monitor_dev_info psy_monitor_dev_info =
{
    .monitoring_interval           = S10_PWR_MONITOR_INTERVAL_PSY,
    .psy_repeat_charging_capacity  =                              97,
    .usb_charging_support          =                               1,
    .usb_charging_display_support  =                               0,
    .dock_charging_support         =                               1,
    .dock_charging_display_support =                               1,
    .coin_battery_support          =                               0,
    .coin_battery_display_support  =                               0,
};
struct resource k3_psy_resources[] =
{
    [0] = {
        .start = IRQ_VBATLOW_RISING,
        .name  = NULL,
        .flags = IORESOURCE_IRQ,
    },
};

static struct platform_device S10_psy_monitor_device =
{
    .name              = "S10_psy_monitor",
    .dev               = {
        .platform_data = &psy_monitor_dev_info,
    },
};
#endif

#ifdef CONFIG_PSY_BQ275X0
static struct bq27510_platform_data bq275x0_data =
{
    .irq_gpio            = GPIO_21_2,
    .irq_flag            = IRQF_TRIGGER_LOW,
    .battery_technology  = POWER_SUPPLY_TECHNOLOGY_LION,
    .voltage_max_design  =                         4200,
    .voltage_min_design  =                         3300,
    .energy_full_design  =                         6500,
    .energy_empty_design =                            0,
};
#endif

#ifdef CONFIG_PSY_BQ24161
int bq24161_io_set(int mode)
{
    struct iomux_block *block_tmp   = NULL;
    struct block_config *config_tmp = NULL;
    int ret = 0;

    block_tmp = iomux_get_block("block_charger");
    if (!block_tmp)
    {
        pr_err("%s: iomux_get_block(%s) error.\n",
               __func__, "block_charger");
        return -1;
    }

    config_tmp = iomux_get_blockconfig("block_charger");
    if (!config_tmp)
    {
        pr_err("%s: iomux_get_blockconfig(%s) error.\n",
               __func__, "block_charger");
        return -1;
    }

    if (mode)
    {
        ret = blockmux_set(block_tmp, config_tmp, NORMAL);
    }
    else
    {
        ret = blockmux_set(block_tmp, config_tmp, LOWPOWER);
    }

    return ret;
}
static struct bq24161_platform_data bq24161_data =
{
#ifdef CONFIG_BQ24161_USB_INPUT
    .usb_input_max_currentmA		=			500,
    .usb_cdp_input_max_currentmA	=			500,
#else
    .usb_input_max_currentmA		=			1500,
    .usb_cdp_input_max_currentmA	=			1500,
#endif
    .usb_dcp_input_max_currentmA	=			1500,
    .usb_ac_input_max_currentmA		=			1500,

    .usb_charging_max_currentmA		=            550,
    .usb_cdp_charging_max_currentmA =            550,
    .usb_dcp_charging_max_currentmA =			2000,
    .usb_ac_charging_max_currentmA	=			2000,

    .max_charger_voltagemV			=			4200,
    .chargerUSB_mode_normal_DPM		=			4600,
    .chargerAC_mode_normal_DPM		=			4200,
    .termination_currentmA			=            200,
    .enbale_gpio					= GPIO_9_2,
    .bq24161_io_block_config		= bq24161_io_set,
};
#endif

#ifdef CONFIG_BATTERY_K3_BQ24161
static struct k3_bq24161_platform_data k3_bq24161_data = 
{
	.max_charger_currentmA = 1000,
	.max_charger_voltagemV = 4200,
	.gpio = BQ24161_GPIO_074,
};
#endif

static struct mhl_platform_data k3_mhl_data =
{
	.gpio_reset 	= MHL_GPIO_RESET,
	.gpio_wake_up	= MHL_GPIO_WAKE_UP,
	.gpio_int	= MHL_GPIO_INT,
#ifdef CONFIG_MHL_USB_SHARE
	.gpio_switch_1	= MHL_GPIO_SWITCH_1,
	.gpio_switch_2	= MHL_GPIO_SWITCH_2,
//	.gpio_dcdc	= MHL_GPIO_DCDC_MODE,
#endif
};

#ifdef CONFIG_EXTRAL_DYNAMIC_DCDC
static struct extral_dynamic_dcdc_platform_data tps6236x_platform_date ={
    .enable_gpio    = EXTRAL_DYNAMIC_DCDC_EN,
    .regulator_data = &extral_dynamic_dcdc_regulator,
};
#endif
/* please add i2c bus 0 devices here */
static struct i2c_board_info hisik3_i2c_bus0_devs[]= {
	/*TODO: add your device here*/
#ifdef CONFIG_TPA2028_SPK_L    
	/* TPA2028 FOR SPEAKER LEFT */
	{
		.type			= TPA2028_L_NAME,
		.addr			= TPA2028_I2C_ADDR,
		.flags 			= true,
		.platform_data 	= &tpa2028_l_pdata,
	},
#endif	
	/* camera tps61310 light */
#ifdef CONFIG_HIK3_CAMERA_FLASH	
	{
		.type			= K3_FLASH_NAME, 
		.addr			= K3_FLASH_I2C_ADDR,
		.platform_data		= &tps61310_platform_data,
	},
#endif	
	#ifdef CONFIG_PSY_BQ275X0
	{
          .type           = BQ275X0_NAME,
          .addr           = BQ275X0_I2C_ADDR,
		  .platform_data = &bq275x0_data,
	},
	#endif

#if defined(CONFIG_AD7147_S10) || defined(CONFIG_AD7147_S10_MODULE)         
      {
        .type           = "ad7147",
        .addr           = 0x2c,       
		.platform_data		= &ad7147_platform_data,
	},
#endif    
    //#ifdef CONFIG_EXTRAL_DYNAMIC_DCDC
    //{
        //.type			= EXTRAL_DYNAMIC_DCDC_NAME,
        //.addr           = EXTRAL_DYNAMIC_DCDC_I2C_ADDR,
        //.platform_data      = &tps6236x_platform_date,
    //},
    //#endif
};
/* please add i2c bus 1 devices here */
static struct i2c_board_info hisik3_i2c_bus1_devs[]= {
	/* Synaptics Touchscreen*/
#if defined(CONFIG_TOUCHSCREEN_SYNAPTICS_I2C_RMI4_K3V2) \
        || defined(CONFIG_TOUCHSCREEN_SYNAPTICS_I2C_RMI4_K3V2_MODULE)    
	{
		.type		= SYNAPTICS_RMI4_NAME,
		.addr		= SYNAPTICS_RMI4_I2C_ADDR,
		/* Multi-touch support*/
		.flags 		= true,
		.platform_data 	= &synaptics_ts_platform_data,
	},
#elif defined(CONFIG_TOUCHSCREEN_RMI4) || defined(CONFIG_TOUCHSCREEN_RMI4_MODULE)
#ifdef S10_PRODUCT_S10
	{
	    .type       = "rmi_i2c1",
	    .addr       = 0x70,
	    /* Multi-touch support*/
	    .platform_data  = &rmi4_pdata,
	},
#endif	    
#endif
#ifdef CONFIG_BATTERY_K3_BQ24161
	{
		.type			= "k3_bq24161_charger",
		.addr			= I2C_ADDR_BQ24161,
//#if defined(S10_PRODUCT_LINK)
//		.platform_data = &std_bq24161_data,
//#else
		.platform_data 	= &k3_bq24161_data,
//#endif
		.irq				= GPIO_0_5,
	},
	#endif
#if defined(CONFIG_HARDWARE_OLD) 	
	#ifdef CONFIG_PSY_BQ275X0 
	{
          .type           = BQ275X0_NAME,
          .addr           = BQ275X0_I2C_ADDR,
          .platform_data  = &bq275x0_platform_data,
	},
	#endif
#endif

#ifdef CONFIG_PSY_BQ24161
    {
        .type = BQ24161_NAME,
        .addr = BQ24161_I2C_ADDR,
        .platform_data = &bq24161_data,
        .irq = GPIO_0_5,
    },
#endif

#ifdef CONFIG_BATTERY_K3_BQ27510
	{
		.type			= "k3-bq27510-battery",
		.addr			= I2C_ADDR_BQ27510,
		.platform_data 	= NULL,
		.irq				= GPIO_21_2,
	},
#endif
	/* Atmel mXT224E touchscreen*/
#if defined(CONFIG_TOUCHSCREEN_MXT224E) || defined(CONFIG_TOUCHSCREEN_MXT224E_MODULE)
	/* Atmel mXT224E touchscreen*/
#ifdef S10_PRODUCT_S10	
	{
		.type			= ATMEL_MXT224E_NAME,
		.addr			= 0x4A,
		.platform_data	= &atmel_tp_platform_data,
	},
#endif		
#elif defined(CONFIG_TOUCHSCREEN_MXT1386) || defined(CONFIG_TOUCHSCREEN_MXT1386_MODULE)
#ifdef S10_PRODUCT_S10	
	{
	    .type           = ATMEL_MXT1386_NAME_I2C1,
	    .addr           = 0x4C, //0x4C,0x4D, 0x5A, 0x5B
	    .platform_data  = &atmel_tp_platform_data,
	},
#endif

#endif
	{
		.type			= "mhl_Sii9244_page0",
		.addr			= MHL_SII9244_PAGE0_ADDR,
		.platform_data 		= &k3_mhl_data,
	},
	{
		.type			= "mhl_Sii9244_page1",
		.addr			= MHL_SII9244_PAGE1_ADDR,
		.platform_data 		= NULL,
	},
	{
		.type			= "mhl_Sii9244_page2",
		.addr			= MHL_SII9244_PAGE2_ADDR,
		.platform_data 		= NULL,
	},
	{
		.type			= "mhl_Sii9244_cbus",
		.addr			= MHL_SII9244_CBUS_ADDR,
		.platform_data 		= NULL,
	},
#ifdef CONFIG_FSA880_I2C
	{
		.type			= FAIRCHILD_FSA880_NAME,
		.addr			= FAIRCHILD_FSA880_I2C_ADDR,
		.flags 			= true, 	
		.platform_data 	= &fsa880_platform_data,	
	},
#endif
#ifdef CONFIG_TPA2028_L_LITE2        
    /* TPA2028 FOR SPEAKER LEFT */
    {
        .type           = TPA2028_L_NAME,
        .addr           = TPA2028_I2C_ADDR,
        .flags          = true,
        .platform_data  = &tpa2028_l_pdata,
    },
#endif	

#ifdef CONFIG_TPA2028_SPK_R  
	/* TPA2028 FOR SPEAKER RIGHT */
	{
		.type			= TPA2028_R_NAME,
		.addr			= TPA2028_I2C_ADDR,
		.flags 			= true,
		.platform_data 	= &tpa2028_r_pdata,
	},
#endif	
	/*TODO: add your device here*/
};
static struct i2c_board_info hisik3_i2c_tp_devs[]= {
	/* Synaptics Touchscreen*/
    {
        .type       = "rmi_i2c",
        .addr       = 0x70,
        .platform_data  = &rmi4_pdata,
    },
#ifdef S10_PRODUCT_S10	

    {
        .type           = ATMEL_MXT1386_NAME,
        .addr           = 0x4C, //0x4C,0x4D, 0x5A, 0x5B
        .platform_data  = &atmel_tp_platform_data,
    },
#endif
	/*TODO: add your device here*/
};

#if defined(CONFIG_GPIO_I2C_ADPT)
static struct i2c_board_info gpio_i2c_devs[]= {
	{
		.type			= "lcd_panasonic",
		.addr			= 0xA0 >> 1,
	},
};
#endif
static struct i2c_board_info hisik3_i2c_gpudcdc_devs[]= {
    #ifdef CONFIG_EXTRAL_DYNAMIC_DCDC
    {
        .type			= EXTRAL_DYNAMIC_DCDC_NAME,
        .addr           = EXTRAL_DYNAMIC_DCDC_I2C_ADDR,
        .platform_data      = &tps6236x_platform_date,
    },
    #endif
#ifdef CONFIG_AUDIENCE 
	{
        .type			= ES305_NAME,
		.addr			= 0x3E,
		.flags 			= true,
		.platform_data 	= &audience_platform_data,
	},
#endif
};
static struct i2c_board_info hisik3_i2c_bus3_devs[]= {
    /*TODO: add your device here*/    
#ifdef CONFIG_AUDIENCE
	{
        .type			= ES305_NAME,
		.addr			= 0x3E,
		.flags 			= true,
		.platform_data 	= &audience_platform_data,
	},
#endif
	 
};

/* please add platform device in the struct.*/
static struct platform_device *k3v2oem1_public_dev[] __initdata = {
	&hisik3_hi6421_irq_device,
	&hisik3_adc_device,
/* skf57909 2011/11/8  add begin */
#ifdef CONFIG_LEDS_K3_6421	
	&hi6421_led_device,
#endif

#ifdef CONFIG_ANDROID_K3_VIBRATOR	
	&hi6421_vibrator_device,
#endif
/* skf57909 2011/11/8  add end */
	&hisik3_camera_device,
	&hisik3_fake_camera_device,
	&hisik3_device_hwmon,
#if S10_HWID_L3H(S10, S10101, A)
    &hisik3_gpio_keypad_device,
#else
	&hisik3_keypad_device,
#endif
	&hisik3_keypad_backlight_device,
	&k3_lcd_device, 
	&k3_gps_bcm_device, 
#ifdef CONFIG_BATTERY_K3
	&hisik3_battery_monitor,
#endif
#ifdef CONFIG_PSY_MONITOR
	//&k3_psy_monitor_device,//reomove s10 ics charging code (out of use)
	&S10_psy_monitor_device,
#endif
/*Begin:Added by g00124340 2011/09/19  for for bluetooth */
	&btbcm_device,
	&bcm_bluesleep_device,
/*End:Added	by g00124340 2011/09/19	*/
	&hisik3_power_key_device,
	&tpa6132_device,
	&usb_switch_device,
    &hisik3_gpio_version_device,
#ifdef CONFIG_BOOST_5V
	&hisik3_boost_5v_device,
#endif
#ifdef CONFIG_PSY_MAX8903
	&k3_max8903_device,
#endif

	&boardid_dev,
#if defined(CONFIG_MRUS51S) 
	&mrus51s,
#endif
#if defined(CONFIG_SIM_DETECT)
	&sim_detect_device,
#endif
/*added by s00212129 for watchdog 2012/2/29 */
	&hisik3_watchdog_device,
#if defined(CONFIG_SUPPORT_MICRO_USB_PORT)
	&micro_usb_power_control_device,
#endif
};

static void k3v2_i2c_devices_init(void)
{
	/* Register devices on I2C Bus0 and Bus1*/
	i2c_register_board_info(0, hisik3_i2c_bus0_devs,
					ARRAY_SIZE(hisik3_i2c_bus0_devs));
	i2c_register_board_info(1, hisik3_i2c_bus1_devs,
					ARRAY_SIZE(hisik3_i2c_bus1_devs));
	i2c_register_board_info(2, hisik3_i2c_tp_devs,
					ARRAY_SIZE(hisik3_i2c_tp_devs));
        if(get_product_feature(PROD_FEATURE_GPU_DCDC_SUPPLY)){
           i2c_register_board_info(3, hisik3_i2c_gpudcdc_devs,
                ARRAY_SIZE(hisik3_i2c_gpudcdc_devs));
		 }else{
#ifdef CONFIG_I2C3_LITE2
        	i2c_register_board_info(3, hisik3_i2c_bus3_devs,
              	ARRAY_SIZE(hisik3_i2c_bus3_devs));
#endif
		}
#if defined(CONFIG_GPIO_I2C_ADPT)
	i2c_register_board_info(4, gpio_i2c_devs,
					ARRAY_SIZE(gpio_i2c_devs));
#endif

}


static void __init k3v2oem1_init(void)
{
	unsigned int  index = 0;
	unsigned int  board_type;

	edb_trace(1);
	k3v2_common_init();
	/* 
	 * providing two ways of realizing keypad, one is KPC, the other is GPIO.
	 * depending on current boardid, use corresponding register device.
	 * boardid=0 means board, choose hisik3_keypad_device which is realized by KPC.
	 * boardid=1 means phone, choose hisi_gpiokeypad_device which is realized by GPIO.
	 */
	board_type = get_board_type();
	printk("get_board_type = %d\n",board_type);
	switch (board_type) {
	case E_BOARD_TYPE_U9510:
#ifdef CONFIG_LEDS_K3_6421
		hi6421_led_device.dev.platform_data = &hi6421_leds_phone;
#endif
		for( index =0; index <  ARRAY_SIZE(k3v2oem1_public_dev); index++ ) {
			if ( (struct platform_device *)(&hisik3_keypad_device) == (struct platform_device *)(k3v2oem1_public_dev[index]) ) {
				k3v2oem1_public_dev[index] = &hisik3_gpio_keypad_device;
				break;
			}
		}
		break;
	case E_BOARD_TYPE_PLATFORM:
		break;
	case E_BOARD_TYPE_S10:
	case E_BOARD_TYPE_LINK_S10:

		printk("wanglin test board -- gpiokey\n");
		for( index =0; index <  ARRAY_SIZE(k3v2oem1_public_dev); index++ ) {
			if ( (struct platform_device *)(&hisik3_keypad_device) == (struct platform_device *)(k3v2oem1_public_dev[index]) ) {
				k3v2oem1_public_dev[index] = &hisik3_gpio_keypad_device;
				break;
			}
		}
		break;
	default:
		break;
	}
	platform_add_devices(k3v2oem1_public_dev, ARRAY_SIZE(k3v2oem1_public_dev));

	k3v2_i2c_devices_init();
#if defined(CONFIG_TOUCHSCREEN_SYNAPTICS_I2C_RMI4_K3V2) \
            || defined(CONFIG_TOUCHSCREEN_SYNAPTICS_I2C_RMI4_K3V2_MODULE)    
	synaptics_virtual_keys_init();
#endif
	
#ifdef CONFIG_DEBUG_FS
	config_debugfs_init();
#endif
}

static void __init k3v2_early_init(void)
{
	int chip_id = 0;
	k3v2_init_clock();
	chip_id = get_chipid();
	if (chip_id == CS_CHIP_ID) {
		k3v2_clk_init_from_table(common_clk_init_table_cs);
	} else if (chip_id == DI_CHIP_ID) {
		k3v2_clk_init_from_table(common_clk_init_table_es);
	}
}

#if 0
static void k3v2_mem_setup(void)
{
	unsigned long reserved_size;

	printk(KERN_INFO "k3v2_mem_setup\n");

	/*
	   Memory reserved for Graphic/ Dcode/EnCode
	*/
	reserved_size = hisi_get_reserve_mem_size();

	/*
	 * Memory configuration with SPARSEMEM enabled on  (see
	 * asm/mach/memory.h for more information).
	 */
	arm_add_memory(PLAT_PHYS_OFFSET, (HISI_BASE_MEMORY_SIZE - reserved_size));

	return;
}

/*
 * k3v2_mem=size1@start1[,size2@start2][,...]
 * size means memory size which larger than 512M
 */
static int __init early_k3v2_mem(char *p)
{
	unsigned long size;
	phys_addr_t start;
	char *endp = NULL;
	char *ep = NULL;

	k3v2_mem_setup();

	printk(KERN_INFO "k3v2_mem = %s\n", p);

	start = PLAT_PHYS_OFFSET + HISI_BASE_MEMORY_SIZE;
	while (*p != '\0') {
		size  = memparse(p, &endp);
		if (*endp == '@')
			start = memparse(endp + 1, &ep);

		/* oem ec1 1G memory based */
		if ((start == SZ_512M)) {
			if (size < SZ_512M)
				size = 0;
			else
				size -= SZ_512M;
		}

		arm_add_memory(start, size);

		printk(KERN_INFO "early_k3v2_mem start 0x%x size 0x%lx\n", start, size);

		if (*ep == ',')
			p = ep + 1;
		else
			break;

		printk(KERN_INFO "k3v2_mem = %s\n", p);
	}

	return 0;
}
early_param("k3v2_mem", early_k3v2_mem);
#endif

static void __init k3v2_map_io(void)
{
	printk("k3v2oem1 map io\n");
	k3v2_map_common_io();
}

MACHINE_START(K3V2OEM1, "k3v2oem1")
	.boot_params	= PLAT_PHYS_OFFSET + 0x00000100,
	.init_irq       = k3v2_gic_init_irq,
	.init_machine   = k3v2oem1_init,
	.map_io         = k3v2_map_io,
	.timer          = &k3v2_timer,
	.init_early 	= k3v2_early_init,
MACHINE_END
