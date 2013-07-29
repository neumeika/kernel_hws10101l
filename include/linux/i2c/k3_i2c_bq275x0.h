
/*
 * Copyright 2010 S10 Tech. Co., Ltd.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Code Aurora nor
 *       the names of its contributors may be used to endorse or promote
 *       products derived from this software without specific prior written
 *       permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NON-INFRINGEMENT ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
/*
 * bq275x0 battery guage driver
 *
 */
#ifndef __BQ275x0_BATTERY_H
#define __BQ275x0_BATTERY_H

//#include <linux/battery_info.h>

#define BQ275X0_NAME  "bq275x0-i2c1"
#define BQ275X0_I2C_ADDR 							(0x55)
#define BQ275x0_FIRMWARE_DOWNLOAD_MODE_I2C_ADDR     (0x0B)
#define BQ275x0_NORMAL_MODE                         (0x0)
#define BQ275X0_INTR_GPIO  GPIO_21_2  //battery low interrupt

//#define BQ275X0_DEBUG_FLAG
#if defined(BQ275X0_DEBUG_FLAG)
#define BQ275x0_DBG(format,arg...)		do { printk(KERN_ALERT format, ## arg);  } while (0)
#define BQ275x0_ERR(format,arg...)		do { printk(KERN_ERR format, ## arg);  } while (0)
#define BQ275x0_FAT(format,arg...)		do { printk(KERN_CRIT format, ## arg); } while (0)
#else
#define BQ275x0_DBG(format,arg...)		do { (void)(format); } while (0)
#define BQ275x0_ERR(format,arg...)		do { (void)(format); } while (0)
#define BQ275x0_FAT(format,arg...)		do { (void)(format); } while (0)
#endif

#if 0
#define	BSP_NORMAL_MODE_I2C_ADDR			(0x55)
#define	BSP_ROM_MODE_I2C_ADDR		    (0x0B)
#define BSP_FIRMWARE_FILE_SIZE              (400*1024)
#define BSP_I2C_MAX_TRANSFER_LEN            128
#define BSP_MAX_ASC_PER_LINE                400
#define BSP_ENTER_ROM_MODE_CMD              0x00
#define BSP_ENTER_ROM_MODE_DATA             0x0F00

#define BSP_UPGRADE_FIRMWARE_BQFS_CMD       "upgradebqfs"
#define BSP_UPGRADE_FIRMWARE_DFFS_CMD       "upgradedffs"

#define BSP_UPGRADE_FIRMWARE_BQFS_NAME      "/system/etc/coulometer/bq27510_pro.bqfs"
#define BSP_UPGRADE_FIRMWARE_DFFS_NAME      "/system/etc/coulometer/bq27510_pro.dffs"
#endif

#ifdef CONFIG_BQ275X0_FIRMWARE_UPDATE
/* added for Firmware upgrade begine */
#define	BSP_UPGRADE_FIRMWARE_BQFS_CMD       "upgradebqfs"
#define	BSP_UPGRADE_FIRMWARE_DFFS_CMD       "upgradedffs"
/*库仑计的完整的firmware，包含可执行镜像及数据*/
#define	BSP_UPGRADE_FIRMWARE_BQFS_NAME      "/system/etc/coulometer/bq27510_pro.bqfs"
/*库仑计的的数据信息*/
#define	BSP_UPGRADE_FIRMWARE_DFFS_NAME      "/system/etc/coulometer/bq27510_pro.dffs"

#define	BSP_FIRMWARE_FILE_SIZE				(400*1024)
#define	BSP_I2C_MAX_TRANSFER_LEN			(128)
#define	BSP_MAX_ASC_PER_LINE				(400)
#define	BSP_ENTER_ROM_MODE_CMD				(0x00)
#define	BSP_ENTER_ROM_MODE_DATA				(0x0F00)
#define	BSP_FIRMWARE_DOWNLOAD_MODE			(0xDDDDDDDD)
#define	BSP_NORMAL_MODE						(0x00)
/* added for Firmware upgrade end */
#endif

//#define COULOMETER_VERSION  0xA6

//#define BARCELONA_SHOW
#ifdef BARCELONA_SHOW
typedef struct
{
    int x;
    int y;
}capacity_map;

typedef enum{
    CAPACITY_RESULT_INVALID,
    CAPACITY_RESULT_VALID,
}capacity_result_status;
#endif /* BARCELONA_SHOW */

struct bq275x0_i2c_platform_data {
	int irq_gpio;
	int irq_flag;
};

struct battery_report_i2c_dev {
    struct device      *dev;
    int                id;
    struct i2c_client  *client;
};

#endif
