/*
 *  mt9v113 camera driver source file
 *
 *  CopyRight (C) Hisilicon Co., Ltd.
 *	Author :
 *  Version:  1.2
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/ctype.h>
#include <linux/types.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/clk.h>
#include <linux/videodev2.h>
#include <linux/time.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/slab.h>
#include <linux/mm.h>
#include <linux/fb.h>
#include <linux/fs.h>
#include <linux/io.h>
#include <linux/unistd.h>
#include <linux/uaccess.h>
#include <asm/div64.h>
#include <mach/hisi_mem.h>
#include "mach/hardware.h"
#include <mach/gpio.h>
#include "../isp/sensor_common.h"
#include "mt9v113.h"
/*#include "../isp/k3_isp_io.h"*/
#include <asm/bug.h>
#include <linux/device.h>

#define LOG_TAG "MT9V113"
//#define DEBUG_DEBUG 0
#include "../isp/cam_log.h"
#include "../isp/cam_dbg.h"

#define MT9V113_BYD_SLAVE_ADDRESS     0x7a
#define MT9V113_SUNNY_SLAVE_ADDRESS     0x7a
#define MT9V113_CHIP_ID       (0x2280)

#define MT9V113_CAM_MODULE_SKIPFRAME     4


#define MT9V113_NO_FLIP	0x00
#define MT9V113_H_FLIP	       0x01
#define MT9V113_V_FLIP	       0x02
#define MT9V113_HV_FLIP	0x03

#define MT9V113_EXPOSURE_REG	0x3012
#define MT9V113_GAIN_REG		0x305e


#define MT9V113_HFLIP		((1 << CAMERA_H_FLIP) |\
					(1 << CAMERA_V_FLIP))

#define MT9V113_VFLIP		(\
					(1 << CAMERA_NO_FLIP) | \
					(1 << CAMERA_H_FLIP) \
				)

static camera_capability 	mt9v113_cap[] = {
	{V4L2_CID_AUTO_WHITE_BALANCE, THIS_AUTO_WHITE_BALANCE},
	{V4L2_CID_DO_WHITE_BALANCE, THIS_WHITE_BALANCE},
	{V4L2_CID_EFFECT, THIS_EFFECT},
	{V4L2_CID_HFLIP, MT9V113_HFLIP},
	{V4L2_CID_VFLIP, MT9V113_VFLIP},
};

const struct isp_reg_t isp_init_regs_mt9v113[] = {
    {0x63022,0x98},
    {0x63c12,0x04},
    //{0x63b34,0x03},
    //{0x6502f,0x21},
};

/* Fixme: 50/60Hz anti-banding params should be calibrated. */
static framesize_s mt9v113_framesizes[] = {
	{0, 0, 640, 480, 640, 480, 30, 30, 0 , 0, 0x100, VIEW_FULL, RESOLUTION_4_3, false, {mt9v113_framesize_full, ARRAY_SIZE(mt9v113_framesize_full)}},
    {0, 0, 640, 480, 640, 480, 30, 30, 0 , 0, 0x100, VIEW_FULL, RESOLUTION_4_3, false, {mt9v113_framesize_full, ARRAY_SIZE(mt9v113_framesize_full)}},
};

/******************************************/
typedef struct _sensor_module{
	char  *sensor_name;
	u16   chip_id;
	u32   slave_addr;
	u32   gpio_14_0_val;
	struct _sensor_reg_t *preg;
	u32   reg_size;
} sensor_module;

static sensor_module mt9v113_sensor_module[] = {
	{"mt9v113_byd", MT9V113_CHIP_ID, MT9V113_BYD_SLAVE_ADDRESS, 1, mt9v113_byd_init_regs, ARRAY_SIZE(mt9v113_byd_init_regs)},
	{"mt9v113_sunny", MT9V113_CHIP_ID, MT9V113_SUNNY_SLAVE_ADDRESS, 0, mt9v113_sunny_init_regs, ARRAY_SIZE(mt9v113_sunny_init_regs)},
};
static int camera_index = -1;
/******************************************/

static camera_sensor mt9v113_sensor;
static bool sensor_inited = false;
static camera_effects sensor_effect = CAMERA_EFFECT_NONE;
static camera_white_balance sensor_awb_mode = CAMERA_WHITEBALANCE_AUTO;
static void mt9v113_set_default(void);
static void mt9v113_change_frame_rate(camera_frame_rate_mode fps_mode);
static void mt9v113_update_frame_rate(void);
/*
 **************************************************************************
 * FunctionName: mt9v113_read_reg;
 * Description : read mt9v113_byd reg by i2c;
 * Input       : NA;
 * Output      : NA;
 * ReturnValue : NA;
 * Other       : NA;
 **************************************************************************
*/
static int mt9v113_read_reg(u16 reg, u16 *val)
{
	return k3_ispio_read_reg(mt9v113_sensor.i2c_config.index,
				 mt9v113_sensor.i2c_config.addr, reg, val,mt9v113_sensor.i2c_config.val_bits);
}

/*
 **************************************************************************
 * FunctionName: mt9v113_write_reg;
 * Description : write mt9v113_byd reg by i2c;
 * Input       : NA;
 * Output      : NA;
 * ReturnValue : NA;
 * Other       : NA;
 **************************************************************************
*/
static int mt9v113_write_reg(u16 reg, u16 val, i2c_length length, u8 mask)
{
	return k3_ispio_write_reg(mt9v113_sensor.i2c_config.index,
			mt9v113_sensor.i2c_config.addr, reg, val, length, mask);
}

/*
 **************************************************************************
 * FunctionName: mt9v113_write_seq;
 * Description : NA;
 * Input       : NA;
 * Output      : NA;
 * ReturnValue : NA;
 * Other       : NA;
 **************************************************************************
 */
static int mt9v113_write_seq(const struct _sensor_reg_t *seq, u32 size, u8 mask)
{
	print_debug("Enter %s, seq[%#x], size=%d", __func__, (int)seq, size);
	return k3_ispio_write_seq(mt9v113_sensor.i2c_config.index,
			mt9v113_sensor.i2c_config.addr, seq, size,mt9v113_sensor.i2c_config.val_bits, mask);
}

/*
 **************************************************************************
 * FunctionName: mt9v113_write_isp_seq;
 * Description : NA;
 * Input       : NA;
 * Output      : NA;
 * ReturnValue : NA;
 * Other       : NA;
 **************************************************************************
 */
static void mt9v113_write_isp_seq(const struct isp_reg_t *seq, u32 size)
{
	print_debug("Enter %s, seq[%#x], size=%d", __func__, (int)seq, size);
	k3_ispio_write_isp_seq(seq, size);
}


/*
 **************************************************************************
 * FunctionName: mt9v113_enum_frame_intervals;
 * Description : NA;
 * Input       : NA;
 * Output      : NA;
 * ReturnValue : NA;
 * Other       : NA;
 **************************************************************************
*/
static int mt9v113_enum_frame_intervals(struct v4l2_frmivalenum *fi)
{
	assert(fi);

	print_debug("enter %s", __func__);
	if (fi->index >= CAMERA_MAX_FRAMERATE) {
		return -EINVAL;
	}

	fi->type = V4L2_FRMIVAL_TYPE_DISCRETE;
	fi->discrete.numerator = 1;
	fi->discrete.denominator = (fi->index+1);
	return 0;
}


/*
 **************************************************************************
 * FunctionName: mt9v113_get_format;
 * Description : NA;
 * Input       : NA;
 * Output      : NA;
 * ReturnValue : NA;
 * Other       : NA;
 **************************************************************************
*/
static int mt9v113_get_format(struct v4l2_fmtdesc *fmt)
{
	if (fmt->type == V4L2_BUF_TYPE_VIDEO_OVERLAY) {
		fmt->pixelformat = mt9v113_sensor.fmt[STATE_PREVIEW];
	} else {
		fmt->pixelformat = mt9v113_sensor.fmt[STATE_CAPTURE];
	}
	return 0;
}

/*
 **************************************************************************
 * FunctionName: mt9v113_enum_framesizes;
 * Description : NA;
 * Input       : NA;
 * Output      : NA;
 * ReturnValue : NA;
 * Other       : NA;
 **************************************************************************
*/
static int mt9v113_enum_framesizes(struct v4l2_frmsizeenum *framesizes)
{
	u32 max_index = ARRAY_SIZE(camera_framesizes) - 1;
	u32 this_max_index = ARRAY_SIZE(mt9v113_framesizes) - 1;

	assert(framesizes);

	print_debug("enter %s; ", __func__);

	if (framesizes->index > max_index) {
		print_error("framesizes->index = %d error", framesizes->index);
		return -EINVAL;
	}

	if ((camera_framesizes[framesizes->index].width > mt9v113_framesizes[this_max_index].width)
		|| (camera_framesizes[framesizes->index].height > mt9v113_framesizes[this_max_index].height)) {
		print_error("framesizes->index = %d error", framesizes->index);
		return -EINVAL;
	}

	framesizes->type = V4L2_FRMSIZE_TYPE_DISCRETE;
	framesizes->discrete.width = mt9v113_framesizes[this_max_index].width;
	framesizes->discrete.height = mt9v113_framesizes[this_max_index].height;
	return 0;
}

/*
 **************************************************************************
 * FunctionName: mt9v113_try_framesizes;
 * Description : NA;
 * Input       : NA;
 * Output      : NA;
 * ReturnValue : NA;
 * Other       : NA;
 **************************************************************************
*/
static int mt9v113_try_framesizes(struct v4l2_frmsizeenum *framesizes)
{
	int max_index = ARRAY_SIZE(mt9v113_framesizes) - 1;

	assert(framesizes);

	print_debug("Enter Function:%s  ", __func__);


	if ((framesizes->discrete.width <= mt9v113_framesizes[max_index].width)
	    && (framesizes->discrete.height <= mt9v113_framesizes[max_index].height)) {
		print_debug("===========width = %d", framesizes->discrete.width);
		print_debug("===========height = %d", framesizes->discrete.height);
		return 0;
	}

	print_error("frame size too large, [%d,%d]",
		    framesizes->discrete.width, framesizes->discrete.height);
	return -EINVAL;
}

/*
 **************************************************************************
 * FunctionName: mt9v113_set_framesizes;
 * Description : NA;
 * Input       : flag: if 1, set framesize to sensor,
 *					   if 0, only store framesize to camera_interface;
 * Output      : NA;
 * ReturnValue : NA;
 * Other       : NA;
 **************************************************************************
*/
static int mt9v113_set_framesizes(camera_state state,
				 struct v4l2_frmsize_discrete *fs, int flag, camera_setting_view_type view_type)
{
	int i = 0;
	bool match = false;

	assert(fs);

	print_debug("Enter Function:%s State(%d), flag=%d, width=%d, height=%d",
		    __func__, state, flag, fs->width, fs->height);

	for (i = 0; i < ARRAY_SIZE(mt9v113_framesizes); i++) {
		if ((mt9v113_framesizes[i].width >= fs->width)
		    && (mt9v113_framesizes[i].height >= fs->height)
		    && (mt9v113_framesizes[i].width * fs->height
		    == mt9v113_framesizes[i].height * fs->width)) {
			fs->width = mt9v113_framesizes[i].width;
			fs->height = mt9v113_framesizes[i].height;
			match = true;
			break;
		}
	}
	
	if (false == match) {
		for (i = 0; i < ARRAY_SIZE(mt9v113_framesizes); i++) {
			if ((mt9v113_framesizes[i].width >= fs->width)
			    && (mt9v113_framesizes[i].height >= fs->height)) {
				fs->width = mt9v113_framesizes[i].width;
				fs->height = mt9v113_framesizes[i].height;
				break;
			}
		}
	}
	if (i >= ARRAY_SIZE(mt9v113_framesizes)) {
		print_error("request resolution larger than sensor's max resolution");
		return -EINVAL;
	}

	if (state == STATE_PREVIEW) {
		mt9v113_sensor.preview_frmsize_index = i;
	} else {
		mt9v113_sensor.capture_frmsize_index = i;
	}

	return 0;
}

/*
 **************************************************************************
 * FunctionName: mt9v113_get_framesizes;
 * Description : NA;
 * Input       : NA;
 * Output      : NA;
 * ReturnValue : NA;
 * Other       : NA;
 **************************************************************************
*/
static int mt9v113_get_framesizes(camera_state state,
				 struct v4l2_frmsize_discrete *fs)
{
	int frmsize_index;

	assert(fs);

	if (state == STATE_PREVIEW) {
		frmsize_index = mt9v113_sensor.preview_frmsize_index;
	} else if (state == STATE_CAPTURE) {
		frmsize_index = mt9v113_sensor.capture_frmsize_index;
	} else {
		return -EINVAL;
	}
	fs->width = mt9v113_framesizes[frmsize_index].width;
	fs->height = mt9v113_framesizes[frmsize_index].height;

	return 0;
}

void mt9v113_set_effect(camera_effects effect)
{
	print_debug("%s, effect:%d", __func__, effect);
	if (false == sensor_inited) {
		sensor_effect = effect;
		return;
	}

    switch (effect) {
    	case CAMERA_EFFECT_NONE:
            ////1.2.0 Normal
            mt9v113_write_reg(0x098C, 0x2759, I2C_16BIT, 0x00); // MCU_ADDRESS [MODE_SPEC_EFFECTS_A]
            mt9v113_write_reg(0x0990, 0x6440, I2C_16BIT, 0x00); // MCU_DATA_0
            mt9v113_write_reg(0x098C, 0x275B, I2C_16BIT, 0x00); // MCU_ADDRESS [MODE_SPEC_EFFECTS_B]
            mt9v113_write_reg(0x0990, 0x6440, I2C_16BIT, 0x00); // MCU_DATA_0
            mt9v113_write_reg(0x098C, 0xA103, I2C_16BIT, 0x00); // MCU_ADDRESS [SEQ_CMD]
            mt9v113_write_reg(0x0990, 0x0005, I2C_16BIT, 0x00); // MCU_DATA_0
    		break;
            
    	case CAMERA_EFFECT_MONO:
            ////1.2.1 B&W
            mt9v113_write_reg(0x098C, 0x2759, I2C_16BIT, 0x00); // MCU_ADDRESS [MODE_SPEC_EFFECTS_A]
            mt9v113_write_reg(0x0990, 0x6441, I2C_16BIT, 0x00); // MCU_DATA_0
            mt9v113_write_reg(0x098C, 0x275B, I2C_16BIT, 0x00); // MCU_ADDRESS [MODE_SPEC_EFFECTS_B]
            mt9v113_write_reg(0x0990, 0x6441, I2C_16BIT, 0x00); // MCU_DATA_0
            mt9v113_write_reg(0x098C, 0xA103, I2C_16BIT, 0x00); // MCU_ADDRESS [SEQ_CMD]
            mt9v113_write_reg(0x0990, 0x0005, I2C_16BIT, 0x00); // MCU_DATA_0
    		break;
            
    	case CAMERA_EFFECT_SEPIA:
            ////1.2.2 Sepia
            mt9v113_write_reg(0x098C, 0x2759, I2C_16BIT, 0x00); // MCU_ADDRESS [MODE_SPEC_EFFECTS_A]
            mt9v113_write_reg(0x0990, 0x6442, I2C_16BIT, 0x00); // MCU_DATA_0
            mt9v113_write_reg(0x098C, 0x275B, I2C_16BIT, 0x00); // MCU_ADDRESS [MODE_SPEC_EFFECTS_B]
            mt9v113_write_reg(0x0990, 0x6442, I2C_16BIT, 0x00); // MCU_DATA_0
            mt9v113_write_reg(0x098C, 0xA103, I2C_16BIT, 0x00); // MCU_ADDRESS [SEQ_CMD]
            mt9v113_write_reg(0x0990, 0x0005, I2C_16BIT, 0x00); // MCU_DATA_0
    		break;
            
    	case CAMERA_EFFECT_NEGATIVE:
            ////1.2.3 Negative
            mt9v113_write_reg(0x098C, 0x2759, I2C_16BIT, 0x00); // MCU_ADDRESS [MODE_SPEC_EFFECTS_A]
            mt9v113_write_reg(0x0990, 0x6443, I2C_16BIT, 0x00); // MCU_DATA_0
            mt9v113_write_reg(0x098C, 0x275B, I2C_16BIT, 0x00); // MCU_ADDRESS [MODE_SPEC_EFFECTS_B]
            mt9v113_write_reg(0x0990, 0x6443, I2C_16BIT, 0x00); // MCU_DATA_0
            mt9v113_write_reg(0x098C, 0xA103, I2C_16BIT, 0x00); // MCU_ADDRESS [SEQ_CMD]
            mt9v113_write_reg(0x0990, 0x0005, I2C_16BIT, 0x00); // MCU_DATA_0
    		break;
            
    	case CAMERA_EFFECT_SOLARIZE:
    		break;

        default:
    		print_error("%s, Unsupport effect:%d", __func__, effect);
    		return;
    	}

	sensor_effect = effect;
}


void mt9v113_set_awb(camera_white_balance awb_mode)
{
	if (false == sensor_inited) {
		sensor_awb_mode = awb_mode;
		return;
	}

	switch (awb_mode) {
	case CAMERA_WHITEBALANCE_AUTO:
		print_info("Auto");
		//Auto:
        mt9v113_write_reg(0x098C, 0xA34A, I2C_16BIT, 0x00); // MCU_ADDRESS [AWB_GAIN_MIN]
        mt9v113_write_reg(0x0990, 0x0090, I2C_16BIT, 0x00); // MCU_DATA_0
        mt9v113_write_reg(0x098C, 0xA34B, I2C_16BIT, 0x00); // MCU_ADDRESS [AWB_GAIN_MAX]
        mt9v113_write_reg(0x0990, 0x00FF, I2C_16BIT, 0x00); // MCU_DATA_0
        mt9v113_write_reg(0x098C, 0xA34C, I2C_16BIT, 0x00); // MCU_ADDRESS [AWB_GAINMIN_B]
        mt9v113_write_reg(0x0990, 0x0075, I2C_16BIT, 0x00); // MCU_DATA_0
        mt9v113_write_reg(0x098C, 0xA34D, I2C_16BIT, 0x00); // MCU_ADDRESS [AWB_GAINMAX_B]
        mt9v113_write_reg(0x0990, 0x00EF, I2C_16BIT, 0x00); // MCU_DATA_0
        mt9v113_write_reg(0x098C, 0xA351, I2C_16BIT, 0x00); // MCU_ADDRESS [AWB_CCM_POSITION_MIN]
        mt9v113_write_reg(0x0990, 0x0000, I2C_16BIT, 0x00); // MCU_DATA_0
        mt9v113_write_reg(0x098C, 0xA352, I2C_16BIT, 0x00); // MCU_ADDRESS [AWB_CCM_POSITION_MAX]
        mt9v113_write_reg(0x0990, 0x007F, I2C_16BIT, 0x00); // MCU_DATA_0
        mt9v113_write_reg(0x098C, 0xA103, I2C_16BIT, 0x00); // MCU_ADDRESS [SEQ_CMD]
        mt9v113_write_reg(0x0990, 0x0005, I2C_16BIT, 0x00); // MCU_DATA_0
		mdelay(50);
		break;
        
	case CAMERA_WHITEBALANCE_INCANDESCENT:
		print_info("Incandescent");
		//Incandescent:
        mt9v113_write_reg(0x098C, 0xA34A, I2C_16BIT, 0x00); // MCU_ADDRESS [AWB_GAIN_MIN]
        mt9v113_write_reg(0x0990, 0x008D, I2C_16BIT, 0x00); // MCU_DATA_0
        mt9v113_write_reg(0x098C, 0xA34B, I2C_16BIT, 0x00); // MCU_ADDRESS [AWB_GAIN_MAX]
        mt9v113_write_reg(0x0990, 0x008D, I2C_16BIT, 0x00); // MCU_DATA_0
        mt9v113_write_reg(0x098C, 0xA34C, I2C_16BIT, 0x00); // MCU_ADDRESS [AWB_GAINMIN_B]
        mt9v113_write_reg(0x0990, 0x0088, I2C_16BIT, 0x00); // MCU_DATA_0
        mt9v113_write_reg(0x098C, 0xA34D, I2C_16BIT, 0x00); // MCU_ADDRESS [AWB_GAINMAX_B]
        mt9v113_write_reg(0x0990, 0x0088, I2C_16BIT, 0x00); // MCU_DATA_0
        mt9v113_write_reg(0x098C, 0xA351, I2C_16BIT, 0x00); // MCU_ADDRESS [AWB_CCM_POSITION_MIN]
        mt9v113_write_reg(0x0990, 0x0000, I2C_16BIT, 0x00); // MCU_DATA_0
        mt9v113_write_reg(0x098C, 0xA352, I2C_16BIT, 0x00); // MCU_ADDRESS [AWB_CCM_POSITION_MAX]
        mt9v113_write_reg(0x0990, 0x0000, I2C_16BIT, 0x00); // MCU_DATA_0
        mt9v113_write_reg(0x098C, 0xA103, I2C_16BIT, 0x00); // MCU_ADDRESS [SEQ_CMD]
        mt9v113_write_reg(0x0990, 0x0005, I2C_16BIT, 0x00); // MCU_DATA_0                                                      
        mdelay(50);
		break;
        
	case CAMERA_WHITEBALANCE_FLUORESCENT:
		print_info("Fluorescent");
		//Fluorescent:
        mt9v113_write_reg(0x098C, 0xA34A, I2C_16BIT, 0x00); // MCU_ADDRESS [AWB_GAIN_MIN]
        mt9v113_write_reg(0x0990, 0x00A6, I2C_16BIT, 0x00); // MCU_DATA_0
        mt9v113_write_reg(0x098C, 0xA34B, I2C_16BIT, 0x00); // MCU_ADDRESS [AWB_GAIN_MAX]
        mt9v113_write_reg(0x0990, 0x00A6, I2C_16BIT, 0x00); // MCU_DATA_0
        mt9v113_write_reg(0x098C, 0xA34C, I2C_16BIT, 0x00); // MCU_ADDRESS [AWB_GAINMIN_B]
        mt9v113_write_reg(0x0990, 0x0080, I2C_16BIT, 0x00); // MCU_DATA_0
        mt9v113_write_reg(0x098C, 0xA34D, I2C_16BIT, 0x00); // MCU_ADDRESS [AWB_GAINMAX_B]
        mt9v113_write_reg(0x0990, 0x0080, I2C_16BIT, 0x00); // MCU_DATA_0
        mt9v113_write_reg(0x098C, 0xA351, I2C_16BIT, 0x00); // MCU_ADDRESS [AWB_CCM_POSITION_MIN]
        mt9v113_write_reg(0x0990, 0x0035, I2C_16BIT, 0x00); // MCU_DATA_0
        mt9v113_write_reg(0x098C, 0xA352, I2C_16BIT, 0x00); // MCU_ADDRESS [AWB_CCM_POSITION_MAX]
        mt9v113_write_reg(0x0990, 0x0035, I2C_16BIT, 0x00); // MCU_DATA_0
        mt9v113_write_reg(0x098C, 0xA103, I2C_16BIT, 0x00); // MCU_ADDRESS [SEQ_CMD]
        mt9v113_write_reg(0x0990, 0x0005, I2C_16BIT, 0x00); // MCU_DATA_0                         
		mdelay(50);
		break;
        
	case CAMERA_WHITEBALANCE_DAYLIGHT:
		print_info("Sunny");
		//Sunny:
        mt9v113_write_reg(0x098C, 0xA34A, I2C_16BIT, 0x00); // MCU_ADDRESS [AWB_GAIN_MIN]
        mt9v113_write_reg(0x0990, 0x00CA, I2C_16BIT, 0x00); // MCU_DATA_0
        mt9v113_write_reg(0x098C, 0xA34B, I2C_16BIT, 0x00); // MCU_ADDRESS [AWB_GAIN_MAX]
        mt9v113_write_reg(0x0990, 0x00CA, I2C_16BIT, 0x00); // MCU_DATA_0
        mt9v113_write_reg(0x098C, 0xA34C, I2C_16BIT, 0x00); // MCU_ADDRESS [AWB_GAINMIN_B]
        mt9v113_write_reg(0x0990, 0x007B, I2C_16BIT, 0x00); // MCU_DATA_0
        mt9v113_write_reg(0x098C, 0xA34D, I2C_16BIT, 0x00); // MCU_ADDRESS [AWB_GAINMAX_B]
        mt9v113_write_reg(0x0990, 0x007B, I2C_16BIT, 0x00); // MCU_DATA_0
        mt9v113_write_reg(0x098C, 0xA351, I2C_16BIT, 0x00); // MCU_ADDRESS [AWB_CCM_POSITION_MIN]
        mt9v113_write_reg(0x0990, 0x007F, I2C_16BIT, 0x00); // MCU_DATA_0
        mt9v113_write_reg(0x098C, 0xA352, I2C_16BIT, 0x00); // MCU_ADDRESS [AWB_CCM_POSITION_MAX]
        mt9v113_write_reg(0x0990, 0x007F, I2C_16BIT, 0x00); // MCU_DATA_0
        mt9v113_write_reg(0x098C, 0xA103, I2C_16BIT, 0x00); // MCU_ADDRESS [SEQ_CMD]
        mt9v113_write_reg(0x0990, 0x0005, I2C_16BIT, 0x00); // MCU_DATA_0                      
		mdelay(50);
		break;
        
	case CAMERA_WHITEBALANCE_CLOUDY_DAYLIGHT:
		print_info("Cloudy");
		//Cloudy:
        mt9v113_write_reg(0x098C, 0xA34A, I2C_16BIT, 0x00); // MCU_ADDRESS [AWB_GAIN_MIN]
        mt9v113_write_reg(0x0990, 0x00D0, I2C_16BIT, 0x00); // MCU_DATA_0
        mt9v113_write_reg(0x098C, 0xA34B, I2C_16BIT, 0x00); // MCU_ADDRESS [AWB_GAIN_MAX]
        mt9v113_write_reg(0x0990, 0x00D0, I2C_16BIT, 0x00); // MCU_DATA_0
        mt9v113_write_reg(0x098C, 0xA34C, I2C_16BIT, 0x00); // MCU_ADDRESS [AWB_GAINMIN_B]
        mt9v113_write_reg(0x0990, 0x0070, I2C_16BIT, 0x00); // MCU_DATA_0
        mt9v113_write_reg(0x098C, 0xA34D, I2C_16BIT, 0x00); // MCU_ADDRESS [AWB_GAINMAX_B]
        mt9v113_write_reg(0x0990, 0x0070, I2C_16BIT, 0x00); // MCU_DATA_0
        mt9v113_write_reg(0x098C, 0xA351, I2C_16BIT, 0x00); // MCU_ADDRESS [AWB_CCM_POSITION_MIN]
        mt9v113_write_reg(0x0990, 0x007F, I2C_16BIT, 0x00); // MCU_DATA_0
        mt9v113_write_reg(0x098C, 0xA352, I2C_16BIT, 0x00); // MCU_ADDRESS [AWB_CCM_POSITION_MAX]
        mt9v113_write_reg(0x0990, 0x007F, I2C_16BIT, 0x00); // MCU_DATA_0
        mt9v113_write_reg(0x098C, 0xA103, I2C_16BIT, 0x00); // MCU_ADDRESS [SEQ_CMD]
        mt9v113_write_reg(0x0990, 0x0005, I2C_16BIT, 0x00); // MCU_DATA_0

		mdelay(50);
		break;
        
	default:
		print_error("%s, Unsupport awb_mode:%d", __func__, awb_mode);
		return;
	}
	sensor_awb_mode = awb_mode;

}


/*
 **************************************************************************
 * FunctionName: mt9v113_init_reg;
 * Description : download initial seq for sensor init;
 * Input       : NA;
 * Output      : NA;
 * ReturnValue : NA;
 * Other       : NA;
 **************************************************************************
*/
static int mt9v113_init_reg(void)
{
    int index;
    int size = 0;
	int i = 0;
    u16 id = 0;
    
	print_info("Enter Function:%s  Line:%d, initsize=%d",
	    __func__, __LINE__, sizeof(mt9v113_sunny_init_regs));

    msleep(10);
    /* Read the Model ID of the sensor */
    mt9v113_read_reg(0x0000, &id);
    print_info("Function:%s============>mt9v113 product id:0x%x", __func__, id);
     
    index = camera_index;
    size = mt9v113_sensor_module[index].reg_size;
    for (i = 0; i < size; i++) {
        if (0 == mt9v113_sensor_module[index].preg[i].subaddr) {
            msleep(mt9v113_sensor_module[index].preg[i].value);
        } else {
            switch (mt9v113_sensor_module[index].preg[i].size) {
            case 1:
                mt9v113_write_reg(mt9v113_sensor_module[index].preg[i].subaddr, mt9v113_sensor_module[index].preg[i].value & 0xff, I2C_8BIT, 0x00);
                break;
            case 2:
                mt9v113_write_reg(mt9v113_sensor_module[index].preg[i].subaddr, mt9v113_sensor_module[index].preg[i].value & 0xffff, I2C_16BIT, 0x00);
                break;
            case 4:
                mt9v113_write_reg(mt9v113_sensor_module[index].preg[i].subaddr, (mt9v113_sensor_module[index].preg[i].value >> 16) & 0xffff, I2C_16BIT, 0x00);
                mt9v113_write_reg(mt9v113_sensor_module[index].preg[i].subaddr + 2, mt9v113_sensor_module[index].preg[i].value & 0xffff, I2C_16BIT, 0x00);
                break;
            default:
                print_error("%s, Unsupport reg size:%d", __func__, mt9v113_sensor_module[index].preg[i].size);
                return -EFAULT;
            }
        }
    }

    if (0 != k3_ispio_init_csi(mt9v113_sensor.mipi_index,
				 mt9v113_sensor.mipi_lane_count, mt9v113_sensor.lane_clk)) {
		print_error("fail to init csi");
		return -EFAULT;
	}

	msleep(100);
	
	sensor_inited = true;
    
	if (CAMERA_EFFECT_NONE != sensor_effect)
		mt9v113_set_effect(sensor_effect);
	mt9v113_set_awb(sensor_awb_mode);
	
	return 0;
}


static int mt9v113_get_capability(u32 id, u32 *value)
{
	int i;
	for (i = 0; i < sizeof(mt9v113_cap) / sizeof(mt9v113_cap[0]); ++i) {
		if (id == mt9v113_cap[i].id) {
			*value = mt9v113_cap[i].value;
			break;
		}
	}
	return 0;
}

static int mt9v113_update_flip(u16 width, u16 height)
{
    //sensor have already vflip and hflip in init data, so no need to flip
    u16 reg = 0x0025;  // 180  
	u8 new_flip = ((mt9v113_sensor.vflip << 1) | mt9v113_sensor.hflip);
	print_debug("enter %s", __func__);

    if (mt9v113_sensor.old_flip != new_flip) {
        if((0 != mt9v113_sensor.hflip) && (0 == mt9v113_sensor.vflip)) {          
            reg = 0x0027; // HORIZONTAL
        } else if ((0 != mt9v113_sensor.vflip) && (0 == mt9v113_sensor.hflip)) {           
            reg = 0x0024; // Vertical
        } else if((0 != mt9v113_sensor.vflip) && (0 != mt9v113_sensor.hflip)) {            
            reg = 0x0026; // NORMAL 
        }

        mt9v113_write_reg(0x098C, 0x2717, I2C_16BIT, 0x00);  // MCU_ADDRESS [MODE_SENSOR_READ_MODE_A]
        mt9v113_write_reg(0x0990, reg, I2C_16BIT, 0x00);  // MCU_DATA_0
        mt9v113_write_reg(0x098C, 0x272D, I2C_16BIT, 0x00);  // MCU_ADDRESS [MODE_SENSOR_READ_MODE_B]
        mt9v113_write_reg(0x0990, reg, I2C_16BIT, 0x00);  // MCU_DATA_0
        mt9v113_write_reg(0x098C, 0xA103, I2C_16BIT, 0x00);  // MCU_ADDRESS [SEQ_CMD]
        mt9v113_write_reg(0x0990, 0x0006, I2C_16BIT, 0x00);  // MCU_DATA_0
        
        mt9v113_sensor.old_flip = new_flip;
        msleep(250);
    }

	return 0;
}

static u32 mt9v113_get_gain(void)
{
	u16 gain_reg_val = 0;
	u32 digital_gain, analog_gain;
	mt9v113_read_reg(MT9V113_GAIN_REG, &gain_reg_val);
	digital_gain = (((gain_reg_val >> 15) & 0x01) * 8)
		+ (((gain_reg_val >> 14) & 0x01) * 4)
		+ (((gain_reg_val >> 13) & 0x01) * 2)
		+ ((gain_reg_val >> 12) & 0x01);
	analog_gain = (1 << ((gain_reg_val >> 6) & 0x03))
		* (gain_reg_val & 0x3f);
	return digital_gain * analog_gain;
}

static u32 mt9v113_get_exposure(void)
{
	u16 exposure = 0;
	mt9v113_read_reg(MT9V113_EXPOSURE_REG, &exposure);
	return (exposure * 33);
}

/*
 **************************************************************************
 * FunctionName: mt9v113_framesize_switch;
 * Description : switch frame size, used by preview and capture
 * Input       : NA;
 * Output      : NA;
 * ReturnValue : NA;
 * Other       : NA;
 **************************************************************************
*/
static int mt9v113_framesize_switch(camera_state state)
{
	u32 size = 0;
	u8 next_frmsize_index;

	if (state == STATE_PREVIEW)
		next_frmsize_index = mt9v113_sensor.preview_frmsize_index;
	else
		next_frmsize_index = mt9v113_sensor.capture_frmsize_index;

	print_info("Enter Function:%s frm index=%d", __func__, next_frmsize_index);
	/*using for update frame rate*/	
	mt9v113_update_frame_rate();
	switch (next_frmsize_index) {

	case 0:
		size = ARRAY_SIZE(mt9v113_framesize_full);
		if (0 != mt9v113_write_seq(mt9v113_framesize_full, size, 0x00)) {
			print_error("fail to init mt9v113_byd sensor");
			return -ETIME;
		}
		break;

	default:
		print_error("fail to init mt9v113_byd sensor");
		return -ETIME;
		break;
	}
    mt9v113_update_flip(mt9v113_framesizes[next_frmsize_index].width, mt9v113_framesizes[next_frmsize_index].height);
	return 0;
}

/*
 **************************************************************************
 * FunctionName: mt9v113_stream_on;
 * Description : download preview seq for sensor preview;
 * Input       : NA;
 * Output      : NA;
 * ReturnValue : NA;
 * Other       : NA;
 **************************************************************************
*/
static int mt9v113_stream_on(camera_state state)
{
	print_debug("Enter Function:%s ", __func__);
	return mt9v113_framesize_switch(state);
}


/*
 **************************************************************************
 * FunctionName: mt9v113_find_sensor;
 * Description : NA;
 * Input       : NA;
 * Output      : NA;
 * ReturnValue : NA;
 * Other       : NA;
 **************************************************************************
 */
static int mt9v113_find_sensor(sensor_module fix_sensor, int index)
{
	u16 id = 0;
    
	mt9v113_sensor.i2c_config.addr = fix_sensor.slave_addr;

	mt9v113_read_reg(0x0000, &id);
	if (fix_sensor.chip_id == id) {
		camera_index = index;
		strcpy(mt9v113_sensor.info.name, fix_sensor.sensor_name);
		print_info("mt9v113 module slave address=%d, %s verify successfully.\n", fix_sensor.slave_addr, mt9v113_sensor.info.name);
		return 1;
	} else {
		print_info("============>mt9v113 module slave address=%d, %s verify fail. chip_id[%#x], id[%#x]\n",
			fix_sensor.slave_addr, mt9v113_sensor.info.name, fix_sensor.chip_id, id);
		return 0;
	}
}


/***************************************************************************
* FunctionName: mt9v113_check_sensor;
* Description : NA;
* Input       : NA;
* Output      : NA;
* ReturnValue : NA;
* Other       : NA;
***************************************************************************/
static int mt9v113_check_sensor(void)
{
    int index;
    unsigned short id = 0;

    //[MIPI 640x4840 ExtClk=24MHz Op_Pix_Clk=28MHz]
    //XMClk=24000000
    mt9v113_write_reg(0x0018, 0x4028, I2C_16BIT, 0x00);    // STANDBY_CONTROL
    msleep(10);
    mt9v113_write_reg(0x001A, 0x0011, I2C_16BIT, 0x00);    // RESET_AND_MISC_CONTROL
    msleep(10);
    mt9v113_write_reg(0x001A, 0x0010, I2C_16BIT, 0x00);    // RESET_AND_MISC_CONTROL
    msleep(100);
    mt9v113_write_reg(0x0018, 0x402E, I2C_16BIT, 0x00);    // STANDBY_CONTROL FOR MIPI
    msleep(10);
    mt9v113_write_reg(0x31E0, 0x0001, I2C_16BIT, 0x00);    // RESERVED_CORE_31E0

    //[For MIPI]
    mt9v113_write_reg(0x001A, 0x0018, I2C_16BIT, 0x00);    // RESET_AND_MISC_CONTROL
    mt9v113_write_reg(0x3400, 0x7A28, I2C_16BIT, 0x00);    // MIPI_CONTROL
    mt9v113_write_reg(0x321C, 0x0003, I2C_16BIT, 0x00);    // OFIFO_CONTROL_STATUS
    mt9v113_write_reg(0x001E, 0x0777, I2C_16BIT, 0x00);    // PAD_SLEW
    mt9v113_write_reg(0x0016, 0x42DF, I2C_16BIT, 0x00);    // CLOCKS_CONTROL

    //[PLL Setting]
    mt9v113_write_reg(0x0014, 0x2145, I2C_16BIT, 0x00);    // PLL_CONTROL
    mt9v113_write_reg(0x0014, 0x2145, I2C_16BIT, 0x00);    // PLL_CONTROL
    mt9v113_write_reg(0x0010, 0x021C, I2C_16BIT, 0x00);    // PLL_DIVIDERS
    mt9v113_write_reg(0x0012, 0x0000, I2C_16BIT, 0x00);    // PLL_P_DIVIDERS
    mt9v113_write_reg(0x0014, 0x244B, I2C_16BIT, 0x00);    // PLL_CONTROL
    mt9v113_write_reg(0x0014, 0x304B, I2C_16BIT, 0x00);    // PLL_CONTROL
    mt9v113_write_reg(0x0014, 0xB04A, I2C_16BIT, 0x00);    // PLL_CONTROL    
    /*
     * after sensor is initialized ,we distinguish the model by CAM_ID gpio:
     * sunny model's GPIO[1] connects DGND, byd model's GPIO[1] connects to DOVDD
     * bit[9] of register 0x1070 is the value of GPIO[1] signal, so we read
     * the value of 0x1070 and move right 9 bits to get the GPIO[1] value
     */
    mt9v113_write_reg(0x098C, 0x1070, I2C_16BIT, 0x00);
    mt9v113_read_reg(0x0990, &id);
    id = (id & 0x0200) >> 9;
       
    if (id == 0) {
        index = 1;           
    } else {
        index = 0;          
    } 
        
    if (mt9v113_find_sensor(mt9v113_sensor_module[index], index)) {
        return 0;
    } else {
        print_error("Invalid product id, Could not load sensor mt9v113");
        return -EFAULT;
    }  
}


/****************************************************************************
* FunctionName: mt9v113_check_sensor;
* Description : NA;
* Input       : NA;
* Output      : NA;
* ReturnValue : NA;
* Other       : NA;
***************************************************************************/
int mt9v113_power(camera_power_state power)
{
	int ret = 0;

	if (power == POWER_ON) {
        k3_ispgpio_reset_sensor(mt9v113_sensor.sensor_index, POWER_ON,
					mt9v113_sensor.power_conf.reset_valid);
     
        k3_ispldo_power_sensor(power,"camera-vcc");         // 1.8V 数字电压
        //udelay(20);
        k3_ispldo_power_sensor(power,"sec-cameralog-vcc");  //前置模拟电压 2.8v          
        k3_ispio_ioconfig(&mt9v113_sensor, power);
        ret = camera_power_core_ldo(power);                 // 1.2V核电压
        k3_ispldo_power_sensor(power,"pri-cameralog-vcc");  // 2.8V 模拟电压
	} else {
        k3_ispio_deinit_csi(mt9v113_sensor.mipi_index);
        k3_ispldo_power_sensor(power,"pri-cameralog-vcc");  
        camera_power_core_ldo(power);
        udelay(200);
        k3_ispio_ioconfig(&mt9v113_sensor, power);         
        k3_ispldo_power_sensor(power,"sec-cameralog-vcc");
        udelay(10);
        k3_ispldo_power_sensor(power,"camera-vcc");     
		sensor_inited = false;
	}
	return ret;
}
/*
 * Here gain is in unit 1/16 of sensor gain,
 * y36721 todo, temporarily if sensor gain=0x10, ISO is 100
 * in fact we need calibrate an ISO-ET-gain table.
 */
u32 mt9v113_gain_to_iso(int gain)
{
	return (gain * 100) / 0x10;
}

u32 mt9v113_iso_to_gain(int iso)
{
	return (iso * 0x10) / 100;
}

void mt9v113_set_gain(u32 gain)
{
}

void mt9v113_set_exposure(u32 exposure)
{
}

/*
 **************************************************************************
 * FunctionName: mt9v113_reset;
 * Description : NA;
 * Input       : NA;
 * Output      : NA;
 * ReturnValue : NA;
 * Other       : NA;
 **************************************************************************
*/
static int mt9v113_reset( camera_power_state power_state)
{
	print_debug("%s  ", __func__);

	if (POWER_ON == power_state) {
        k3_isp_io_enable_mclk(MCLK_ENABLE, mt9v113_sensor.sensor_index);
        k3_ispgpio_reset_sensor(mt9v113_sensor.sensor_index, POWER_OFF,
					mt9v113_sensor.power_conf.reset_valid);
        udelay(400);  
        k3_ispgpio_reset_sensor(mt9v113_sensor.sensor_index, POWER_ON,
                    mt9v113_sensor.power_conf.reset_valid);
	} else {
        k3_ispgpio_reset_sensor(mt9v113_sensor.sensor_index, power_state,
                  mt9v113_sensor.power_conf.reset_valid);
        k3_isp_io_enable_mclk(MCLK_DISABLE, mt9v113_sensor.sensor_index);
	}

	return 0;
}

/*
 **************************************************************************
 * FunctionName: mt9v113_init;
 * Description : mt9v113 init function;
 * Input       : NA;
 * Output      : NA;
 * ReturnValue : Error code indicating success or failure;
 * Other       : NA;
 **************************************************************************
*/
static int mt9v113_init(void)
{
	print_debug("%s  ", __func__);
	if (mt9v113_sensor.owner && !try_module_get(mt9v113_sensor.owner)) {
		print_error("%s: try_module_get fail", __func__);
		return -ENOENT;
	}

	k3_ispio_power_init("pri-cameralog-vcc", LDO_VOLTAGE_28V, LDO_VOLTAGE_28V);	/*analog 2.85V*/
	k3_ispio_power_init("camera-vcc", LDO_VOLTAGE_18V, LDO_VOLTAGE_18V);	/*IO 1.8V*/
	k3_ispio_power_init("sec-cameralog-vcc", LDO_VOLTAGE_28V, LDO_VOLTAGE_28V);	/*analog 2.85V*/

	return 0;
}

/*
 **************************************************************************
 * FunctionName: mt9v113_exit;
 * Description : mt9v113 exit function;
 * Input       : NA;
 * Output      : NA;
 * ReturnValue : NA;
 * Other       : NA;
 **************************************************************************
*/
static void mt9v113_exit(void)
{
	print_debug("enter %s", __func__);

	k3_ispio_power_deinit();

	if (mt9v113_sensor.owner) {
		module_put(mt9v113_sensor.owner);
	}
	print_debug("exit %s", __func__);
}

/*
 **************************************************************************
 * FunctionName: mt9v113_shut_down;
 * Description : mt9v113 shut down function;
 * Input       : NA;
 * Output      : NA;
 * ReturnValue : NA;
 * Other       : NA;
 **************************************************************************
*/
static void mt9v113_shut_down(void)
{
	print_debug("enter %s", __func__);
	k3_ispgpio_reset_sensor(CAMERA_SENSOR_SECONDARY , POWER_OFF, mt9v113_sensor.power_conf.reset_valid);
}

static int mt9v113_set_hflip(int flip)
{
	print_debug("enter %s flip=%d", __func__, flip);
	mt9v113_sensor.hflip = flip;
	return 0;
}

static int mt9v113_get_hflip(void)
{
	print_debug("enter %s", __func__);
	return mt9v113_sensor.hflip;
}

static int mt9v113_set_vflip(int flip)
{
	print_debug("enter %s flip=%d", __func__, flip);

	mt9v113_sensor.vflip = flip;
	return 0;
}

static int mt9v113_get_vflip(void)
{
	print_debug("enter %s", __func__);
	return mt9v113_sensor.vflip;
}


/*
 **************************************************************************
 * FunctionName: mt9v113_set_default;
 * Description : init mt9v113_sensor;
 * Input       : NA;
 * Output      : NA;
 * ReturnValue : NA;
 * Other       : NA;
 **************************************************************************
*/
static void mt9v113_set_default(void)
{
	mt9v113_sensor.init = mt9v113_init;
	mt9v113_sensor.exit = mt9v113_exit;
	mt9v113_sensor.shut_down = mt9v113_shut_down;
	mt9v113_sensor.reset = mt9v113_reset;
	mt9v113_sensor.check_sensor = mt9v113_check_sensor;
	mt9v113_sensor.power = mt9v113_power;
	mt9v113_sensor.init_reg = mt9v113_init_reg;
	mt9v113_sensor.stream_on = mt9v113_stream_on;

	mt9v113_sensor.get_format = mt9v113_get_format;
	mt9v113_sensor.set_flash = NULL;
	mt9v113_sensor.get_flash = NULL;
	mt9v113_sensor.set_scene = NULL;
	mt9v113_sensor.get_scene = NULL;

	mt9v113_sensor.enum_framesizes = mt9v113_enum_framesizes;
	mt9v113_sensor.try_framesizes = mt9v113_try_framesizes;
	mt9v113_sensor.set_framesizes = mt9v113_set_framesizes;
	mt9v113_sensor.get_framesizes = mt9v113_get_framesizes;

	mt9v113_sensor.set_hflip = mt9v113_set_hflip;
	mt9v113_sensor.get_hflip = mt9v113_get_hflip;
	mt9v113_sensor.set_vflip = mt9v113_set_vflip;
	mt9v113_sensor.get_vflip = mt9v113_get_vflip;
	mt9v113_sensor.update_flip = mt9v113_update_flip;

	mt9v113_sensor.get_gain = mt9v113_get_gain;
	mt9v113_sensor.get_exposure = mt9v113_get_exposure;

	mt9v113_sensor.enum_frame_intervals = mt9v113_enum_frame_intervals;
	mt9v113_sensor.try_frame_intervals = NULL;
	mt9v113_sensor.set_frame_intervals = NULL;
	mt9v113_sensor.get_frame_intervals = NULL;

	mt9v113_sensor.get_capability = mt9v113_get_capability;

	strcpy(mt9v113_sensor.info.name,"mt9v113");//mt9v113_sensor.name = "mt9v113";
	mt9v113_sensor.interface_type = MIPI2;
	mt9v113_sensor.mipi_lane_count = CSI_LINES_1;
	mt9v113_sensor.mipi_index = CSI_INDEX_1;
	mt9v113_sensor.sensor_index = CAMERA_SENSOR_SECONDARY;
	mt9v113_sensor.skip_frames = MT9V113_CAM_MODULE_SKIPFRAME;

	mt9v113_sensor.power_conf.pd_valid = LOW_VALID;
	mt9v113_sensor.power_conf.reset_valid = LOW_VALID;
	mt9v113_sensor.power_conf.vcmpd_valid = LOW_VALID;

	mt9v113_sensor.i2c_config.index = I2C_PRIMARY;
	mt9v113_sensor.i2c_config.speed = I2C_SPEED_400;
	mt9v113_sensor.i2c_config.addr = MT9V113_SUNNY_SLAVE_ADDRESS;
	mt9v113_sensor.i2c_config.addr_bits = 16;
	mt9v113_sensor.i2c_config.val_bits = I2C_16BIT;

	mt9v113_sensor.preview_frmsize_index = 0;
	mt9v113_sensor.capture_frmsize_index = 0;
	mt9v113_sensor.frmsize_list = mt9v113_framesizes;
	mt9v113_sensor.fmt[STATE_PREVIEW] = V4L2_PIX_FMT_YUYV;
	mt9v113_sensor.fmt[STATE_CAPTURE] = V4L2_PIX_FMT_YUYV;

	mt9v113_sensor.aec_addr[0] = 0;
	mt9v113_sensor.aec_addr[1] = 0;
	mt9v113_sensor.agc_addr[0] = 0;
	mt9v113_sensor.agc_addr[1] = 0;

	mt9v113_sensor.set_gain = NULL;
	mt9v113_sensor.set_exposure = NULL;

	mt9v113_sensor.set_vts = NULL;
	mt9v113_sensor.get_vts_reg_addr = NULL;

	mt9v113_sensor.sensor_gain_to_iso = NULL;
	mt9v113_sensor.sensor_iso_to_gain = NULL;

	mt9v113_sensor.isp_location = CAMERA_USE_SENSORISP;
	mt9v113_sensor.sensor_tune_ops = (isp_tune_ops *)kmalloc(sizeof(isp_tune_ops), GFP_KERNEL);

	if (mt9v113_sensor.sensor_tune_ops == NULL) {
		print_error("failed to kmalloc isp_tune_ops");
		return -ENOMEM;
	}

	mt9v113_sensor.af_enable = 0;

	mt9v113_sensor.image_setting.lensc_param = NULL;//mt9v113_byd_lensc_param;
	mt9v113_sensor.image_setting.ccm_param = NULL;//mt9v113_byd_ccm_param;
	mt9v113_sensor.image_setting.awb_param = NULL;//mt9v113_byd_awb_param;


	mt9v113_sensor.fps_max = 30;
	mt9v113_sensor.fps_min = 16;
	mt9v113_sensor.fps = 25;

	mt9v113_sensor.owner = THIS_MODULE;

	mt9v113_sensor.info.facing = CAMERA_FACING_BACK;
	mt9v113_sensor.info.orientation = 270;
	mt9v113_sensor.lane_clk = CLK_400M;
	mt9v113_sensor.hflip = 0;
	mt9v113_sensor.vflip = 0;
	mt9v113_sensor.old_flip = 0;
	mt9v113_sensor.sensor_tune_ops->set_effect = mt9v113_set_effect;
	mt9v113_sensor.sensor_tune_ops->set_awb	= mt9v113_set_awb;
	mt9v113_sensor.sensor_tune_ops->set_anti_banding = NULL; 
}

/*
 **************************************************************************
 * FunctionName: mt9v113_module_init;
 * Description : NA;
 * Input       : NA;
 * Output      : NA;
 * ReturnValue : NA;
 * Other       : NA;
 **************************************************************************
 */
static __init int mt9v113_module_init(void)
{
	mt9v113_set_default();
	return register_camera_sensor(mt9v113_sensor.sensor_index, &mt9v113_sensor);
}

/*
 **************************************************************************
 * FunctionName: mt9v113_module_exit;
 * Description : NA;
 * Input       : NA;
 * Output      : NA;
 * ReturnValue : NA;
 * Other       : NA;
 **************************************************************************
 */
static void __exit mt9v113_module_exit(void)
{
	if (mt9v113_sensor.sensor_tune_ops) {
		kfree(mt9v113_sensor.sensor_tune_ops);
		mt9v113_sensor.sensor_tune_ops = NULL;
	}
	unregister_camera_sensor(mt9v113_sensor.sensor_index, &mt9v113_sensor);
}
/*
 **************************************************************************
 * FunctionName: mt9v113_change_frame_rate;
 * Description : NA;
 * Input       : NA;
 * Output      : NA;
 * ReturnValue : NA;
 * Other       : NA;
 **************************************************************************
 */
static void mt9v113_change_frame_rate(camera_frame_rate_mode fps_mode)
{
	if (fps_mode == CAMERA_FRAME_RATE_FIX_MAX){
		/*Set max frame rate*/
		mt9v113_write_reg(0xC88C, 0x1D99, I2C_16BIT, 0);
		/*Set min frame rate*/
		mt9v113_write_reg(0xC88E, 0x1D99, I2C_16BIT, 0);
	} else {
		/*Set max frame rate*/
		mt9v113_write_reg(0xC88C, 0x1D99, I2C_16BIT, 0);
		/*Set min frame rate*/
		mt9v113_write_reg(0xC88E, 0x0A00, I2C_16BIT, 0);
	}
	
	mt9v113_write_reg(0x098E, 0xDC00, I2C_16BIT, 0);
	/*Change-Config*/
	mt9v113_write_reg(0xDC00, 0x28, I2C_8BIT, 0);	
	/*Set COMMAND_REGISTER to (HOST_COMMAND_OK | HOST_COMMAND_1)*/
	mt9v113_write_reg(0x0080, 0x8002, I2C_16BIT, 0);	
	/*Delay 3ms*/
	mt9v113_write_reg(0x0000, 3, I2C_16BIT, 0);	
}
/*
 **************************************************************************
 * FunctionName: mt9v113_update_frame_rate;
 * Description : NA;
 * Input       : NA;
 * Output      : NA;
 * ReturnValue : NA;
 * Other       : NA;
 **************************************************************************
 */
static void mt9v113_update_frame_rate(void){
	if (mt9v113_sensor.fps_max == mt9v113_sensor.fps_min) {
		mt9v113_change_frame_rate(CAMERA_FRAME_RATE_FIX_MAX);
	} else {
		mt9v113_change_frame_rate(CAMERA_FRAME_RATE_AUTO);
	}
}
MODULE_AUTHOR("Hisilicon");
module_init(mt9v113_module_init);
module_exit(mt9v113_module_exit);
MODULE_LICENSE("GPL");

/********************************** END **********************************************/
