/* include/linux/input/kxtik.h - KXTIK accelerometer driver
 *
 * Copyright (C) 2011 Kionix, Inc.
 * Written by Kuching Tan <kuchingtan@kionix.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#ifndef __KXTIK_H__
#define __KXTIK_H__


/* CTRL_REG1: set resolution, g-range, data ready enable */
/* Output resolution: 8-bit valid or 12-bit valid */
#define RES_8BIT        0
#define RES_12BIT       (1 << 6)
    
/* Output g-range: +/-2g, 4g, or 8g */
#define KXTIK_G_2G        0
#define KXTIK_G_4G        (1 << 3)
#define KXTIK_G_8G        (1 << 4)
    

/* DATA_CTRL_REG: controls the output data rate of the part */
#define ODR12_5F        0
#define ODR25F          1
#define ODR50F          2
#define ODR100F         3
#define ODR200F         4
#define ODR400F         5
#define ODR800F         6

#define ACCL_DATA_SIZE 6

#define KXTIK_DEVICE_MAP 7
#define KXTIK_MAP_X ((KXTIK_DEVICE_MAP-1)%2)
#define KXTIK_MAP_Y (KXTIK_DEVICE_MAP%2)
#define KXTIK_NEG_X (((KXTIK_DEVICE_MAP+2)/2)%2)
#define KXTIK_NEG_Y (((KXTIK_DEVICE_MAP+5)/4)%2)
#define KXTIK_NEG_Z ((KXTIK_DEVICE_MAP-1)/4)

#define KX_G_MAX            8000
/* OUTPUT REGISTERS */
#define KX_XOUT_L           0x06
#define KX_WHO_AM_I         0x0F
/* CONTROL REGISTERS */
#define KX_INT_REL          0x1A
#define KX_CTRL_REG1        0x1B
#define KX_INT_CTRL1        0x1E
#define KX_DATA_CTRL        0x21
/* CONTROL REGISTER 1 BITS */
#define KX_PC1_OFF            0x7F
#define KX_PC1_ON            (1 << 7)
/* INPUT_ABS CONSTANTS */
#define KX_FUZZ            3
#define KX_FLAT            3
/* RESUME STATE INDICES */
#define KX_RES_DATA_CTRL        0
#define KX_RES_CTRL_REG1        1
#define KX_RES_INT_CTRL1        2
#define KX_RESUME_ENTRIES       3

struct kxtik_platform_data {
    unsigned int min_interval;    /* minimum poll interval (in milli-seconds) */
    unsigned int poll_interval;   /* desired poll interval (in milli-seconds) */
    
    u8 device_map;

    /*
     * By default, x is axis 0, y is axis 1, z is axis 2; these can be
     * changed to account for sensor orientation within the host device.
     */
    u8 axis_map_x;
    u8 axis_map_y;
    u8 axis_map_z;

    /*
     * Each axis can be negated to account for sensor orientation within
     * the host device.
     */
    bool negate_x;
    bool negate_y;
    bool negate_z;

    
    u8 res_12bit;
    
    u8 g_range;

    
    int *init_flag;

    int (*init)(void);
    void (*exit)(void);
    int (*power_on)(void);
    int (*power_off)(void);
};
#endif  /* __KXTIK_H__ */
