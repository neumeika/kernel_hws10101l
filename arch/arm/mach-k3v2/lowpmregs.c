/*
 *	linux/arch/arm/mach-k3v2/irq.c
 *
 * Copyright (C) 2011 Hisilicon
 * License terms: GNU General Public License (GPL) version 2
 *
 */
/*=============================================================================
ÐÞ¶©ÀúÊ·
==============================================================================*/
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/io.h>
#include <asm/hardware/gic.h>
#include <mach/io.h>
#include <mach/platform.h>
#include "board.h"
#include <mach/platform.h>
#include <linux/platform_device.h>
#include <linux/console.h>
#include <mach/early-debug.h>
#include <asm/hardware/arm_timer.h>
#include <mach/boardid.h>

#include <linux/regulator/consumer.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
#include <hsad/config_interface.h>
#include <linux/ipps.h>

static void __iomem *g_ioc_addr;
static void __iomem *g_gpio_addr;
static void __iomem *g_sc_addr;
static void __iomem *g_pctrl_addr;
static void __iomem *g_pmuspi_addr;

#ifdef	CONFIG_LOWPM_DEBUG

static struct ipps_client ipps_client;

struct iocfg_lp {
	unsigned int uiomg_off;
	int iomg_val;
	unsigned int uiocg_off;
	int iocg_val;			/*-1: need not to set*/
	unsigned int ugpiog;    /*gpio grpup*/
	unsigned int ugpio_bit; /*gpio bit*/
	int gpio_dir;			/*1: out,  0: in*/
	int gpio_val;			/*1: high, 0:low*/
};


#define LOW_POWER(imo, imv, ico, icv, gpiog, gpiob, gpiod, gpiov) \
	{				\
		.uiomg_off = imo,  .iomg_val = imv,   \
		.uiocg_off = ico,  .iocg_val  = icv,   \
		.ugpiog    = gpiog, .ugpio_bit = gpiob, \
		.gpio_dir  = gpiod, .gpio_val  = gpiov, \
	}

static struct iocfg_lp iocfg_lookups[] = {
		/*gpio_002 gpio out low iomg000 iocg006*/
		LOW_POWER(0x0, 0x0, 0x24, 0x0, 0, 2, 0x1, 0x0),

		/*gpio_003 gpio out low iomg000 iocg007*/
		LOW_POWER(0x0, 0x0, 0x28, 0x0, 0, 3, 0x1, 0x0),

		/*gpio_004 gpio in low  iomg000 iocg008*/
		LOW_POWER(0x0, 0x0, 0x2C, 0x2, 0, 4, 0, 0x0),

		/*gpio_005 gpio in high iomg000 iocg009*/
		LOW_POWER(0x0, 0x0, 0x30, 0x1, 0, 5, 0, 0x0),

		/*gpio_006 gpio out low iomg001 iocg010*/
		LOW_POWER(0x4, 0x0, 0x34, 0x0,  0, 6, 1, 0x0),

		/*gpio_007 gpio in low  iomg002 iocg011*/
		LOW_POWER(0x8, 0x0, 0x38, 0x2, 0, 7, 0, 0x0),

		/*gpio_008 gpio in low iomg003 iocg012*/
		LOW_POWER(0x0C, 0x01, 0x3C, 0x2, 1, 0, 0, 0x0),

		/*gpio_009 gpio in low iomg003 iocg013*/
		LOW_POWER(0x0C, 0x01, 0x40, 0x2, 1, 1, 0, 0x0),

		/*gpio_010 gpio in low iomg003 iocg014*/
		LOW_POWER(0x0C, 0x01, 0x44, 0x2, 1, 2, 0, 0x0),

		/*gpio_011 gpio in low iomg003 iocg015*/
		LOW_POWER(0x0C, 0x01, 0x48, 0x2, 1, 3, 0, 0x0),

		/*gpio_012 gpio in low iomg003 iocg016*/
		LOW_POWER(0x0C, 0x01, 0x4C, 0x2, 1, 4, 0, 0x0),

		/*gpio_013 gpio in low iomg004 iocg017*/
		LOW_POWER(0x10, 0x01, 0x50, 0x2, 1, 5, 0, 0x0),

		/*gpio_014 gpio in low iomg005 iocg018*/
		LOW_POWER(0x14, 0x01, 0x54, 0x2, 1, 6, 0, 0x0),

		/*gpio_015 gpio in low iomg006 iocg019*/
		LOW_POWER(0x18, 0x01, 0x58, 0x2, 1, 7, 0, 0x0),

		/*gpio_016 gpio in low iomg094 iocg020*/
		LOW_POWER(0x1C, 0x01, 0x5C, 0x2, 2, 0, 0, 0x0),

		/*gpio_017 gpio in low iomg007 iocg021*/
		LOW_POWER(0x20, 0x01, 0x60, 0x2, 2, 1, 0, 0x0),

		/*gpio_018 gpio in low iomg008 iocg022*/
		LOW_POWER(0x20, 0x01, 0x64, 0x2, 2, 2, 0, 0x0),

		/*gpio_019 gpio in low iomg009 iocg023*/
		LOW_POWER(0x20, 0x01, 0x68, 0x2, 2, 3, 0, 0x0),

		/*gpio_020 gpio in low iomg003 iocg024*/
		LOW_POWER(0x0C, 0x01, 0x6C, 0x2, 2, 4, 0, 0x0),

		/*gpio_021 gpio in low iomg003 iocg025*/
		LOW_POWER(0x0C, 0x01, 0x70, 0x2, 2, 5, 0, 0x0),

		/*gpio_022 gpio in low iomg003 iocg026*/
		LOW_POWER(0x0C, 0x01, 0x74, 0x2, 2, 6, 0, 0x0),

		/*gpio_023 gpio in low iomg003 iocg027*/
		LOW_POWER(0x0C, 0x01, 0x78, 0x2, 2, 7, 0, 0x0),

		/*gpio_024 gpio in low iomg003 iocg028*/
		LOW_POWER(0x0C, 0x01, 0x7C, 0x2, 3, 0, 0, 0x0),

		/*gpio_025 gpio in low iomg003 iocg029*/
		LOW_POWER(0x0C, 0x01, 0x80, 0x2, 3, 1, 0, 0x0),

		/*gpio_026 gpio in low iomg003 iocg030*/
		LOW_POWER(0x0C, 0x01, 0x84, 0x2, 3, 2, 0, 0x0),

		/*gpio_027 gpio in low iomg003 iocg031*/
		LOW_POWER(0x0C, 0x01, 0x88, 0x2, 3, 3, 0, 0x0),

		/*gpio_028 gpio in low iomg010 iocg032*/
		LOW_POWER(0x2C, 0x01, 0x8C, 0x2, 3, 4, 0, 0x0),

		/*gpio_029 gpio in low iomg010 iocg033*/
		LOW_POWER(0x2C, 0x01, 0x90, 0x2, 3, 5, 0, 0x0),

		/*gpio_030 gpio in low iomg010 iocg034*/
		LOW_POWER(0x2C, 0x01, 0x94, 0x2, 3, 6, 0, 0x0),

		/*gpio_031 gpio in low iomg010 iocg035*/
		LOW_POWER(0x2C, 0x01, 0x98, 0x2, 3, 7, 0, 0x0),

		/*gpio_032 gpio in low iomg010 iocg036*/
		LOW_POWER(0x2C, 0x01, 0x9C, 0x2, 4, 0, 0, 0x0),

		/*gpio_033 gpio in low iomg010 iocg037*/
		LOW_POWER(0x2C, 0x01, 0xA0, 0x2, 4, 1, 0, 0x0),

		/*gpio_034 gpio in low iomg010 iocg038*/
		LOW_POWER(0x2C, 0x01, 0xA4, 0x2, 4, 2, 0, 0x0),

		/*gpio_035 gpio in low iomg010 iocg039*/
		LOW_POWER(0x2C, 0x01, 0xA8, 0x2, 4, 3, 0, 0x0),

		/*gpio_036 gpio in low iomg012 iocg040*/
		LOW_POWER(0x30, 0x00, 0xAC, 0x2, 4, 4, 0, 0x0),

		/*gpio_037 gpio in low iomg012 iocg041*/
		LOW_POWER(0x30, 0x00, 0xB0, 0x2, 4, 5, 0, 0x0),

		/*gpio_038 gpio in nopull iomg013 iocg054*/
		LOW_POWER(0x34, 0x01, 0xB4, 0x0, 4, 6, 0, 0x0),

		/*gpio_039 gpio in nopull iomg013 iocg055*/
		LOW_POWER(0x34, 0x01, 0xB8, 0x0, 4, 7, 0, 0x0),

		/*gpio_040 gpio in pulldown (phone) iomg014 iocg056*/
		LOW_POWER(0x38, 0x01, 0xBC, 0x2, 5, 0, 0, 0x0),

		/*gpio_041 gpio in pulldown iomg015 iocg057*/
		LOW_POWER(0x3C, 0x01, 0xC0, 0x2, 5, 1, 0, 0x0),

		/*gpio_042 gpio in low iomg016 iocg058*/
		LOW_POWER(0x40, 0x01, 0xC4, 0x2, 5, 2, 0, 0x0),

		/*gpio_043 gpio in low iomg016 iocg059*/
		LOW_POWER(0x40, 0x01, 0xC8, 0x2, 5, 3, 0, 0x0),

		/*gpio_044 gpio in low iomg016 iocg060*/
		LOW_POWER(0x40, 0x01, 0xCC, 0x2, 5, 4, 0, 0x0),

		/*gpio_045 gpio in low iomg016 iocg061*/
		LOW_POWER(0x40, 0x01, 0xD0, 0x2, 5, 5, 0, 0x0),

		/*gpio_046 gpio in low iomg016 iocg062*/
		LOW_POWER(0x40, 0x01, 0xD4, 0x2, 5, 6, 0, 0x0),

		/*gpio_047 gpio in low iomg016 iocg063*/
		LOW_POWER(0x40, 0x01, 0xD8, 0x2, 5, 7, 0, 0x0),

		/*gpio_048 gpio in low iomg016 iocg064*/
		LOW_POWER(0x40, 0x01, 0xDC, 0x2, 6, 0, 0, 0x0),

		/*gpio_049 gpio in low iomg016 iocg065*/
		LOW_POWER(0x40, 0x01, 0xE0, 0x2, 6, 1, 0, 0x0),

		/*gpio_050 gpio in low iomg017 iocg066*/
		LOW_POWER(0x44, 0x01, 0xE4, 0x2, 6, 2, 0, 0x0),

		/*gpio_051 gpio in low iomg017 iocg067*/
		LOW_POWER(0x44, 0x01, 0xE8, 0x2, 6, 3, 0, 0x0),

		/*gpio_052 gpio in low iomg018 iocg068*/
		LOW_POWER(0x48, 0x01, 0xEC, 0x2, 6, 4, 0, 0x0),

		/*gpio_053 gpio in low iomg018 iocg069*/
		LOW_POWER(0x48, 0x01, 0xF0, 0x2, 6, 5, 0, 0x0),

		/*gpio_054 gpio in low iomg018 iocg070*/
		LOW_POWER(0x48, 0x01, 0xF4, 0x2, 6, 6, 0, 0x0),

		/*gpio_055 gpio in low iomg019 iocg071*/
		LOW_POWER(0x4C, 0x01, 0xF8, 0x0, 6, 7, 0, 0x0),

		/*gpio_056 gpio in low iomg019 iocg072*/
		LOW_POWER(0x4C, 0x01, 0xFC, 0x0, 7, 0, 0, 0x0),

		/*gpio_057 gpio in none iomg020 iocg073*/
		LOW_POWER(0x50, 0x01, 0x100, 0x0, 7, 1, 0, 0x0),

		/*gpio_058 gpio in none iomg021 iocg074*/
		LOW_POWER(0x54, 0x01, 0x104, 0x0, 7, 2, 0, 0x0),

		/*gpio_059 gpio in low iomg022 iocg075*/
		LOW_POWER(0x58, 0x01, 0x108, 0x0, 7, 3, 1, 0x0),

		/*gpio_060 gpio in low iomg023 iocg076*/
		LOW_POWER(0x5C, 0x01, 0x10C, 0x0, 7, 4, 1, 0x0),

		/*gpio_061 gpio out high iomg024 iocg077*/
		//LOW_POWER(0x60, 0x01, 0x110, 0x0, 7, 5, 1, 0x1),
		LOW_POWER(0x60, 0x01, 0x110, 0x0, 7, 5, 1, 0x0),//gpio out low
		/*gpio_062 gpio in low iomg025 iocg078*/
		LOW_POWER(0x64, 0x01, 0x114, 0x2, 7, 6, 0, 0x0),

		/*gpio_063 gpio in low iomg026 iocg079*/
		LOW_POWER(0x68, 0x01, 0x118, 0x2, 7, 7, 0, 0x0),

		/*gpio_064 gpio in low iomg027 iocg080*/
		LOW_POWER(0x6C, 0x01, 0x11C, 0x2, 8, 0, 0, 0x0),

		/*gpio_065 gpio out low iomg028 iocg081*/
		LOW_POWER(0x70, 0x01, 0x120, 0x0, 8, 1, 1, 0x0),

		/*gpio_066 gpio out low iomg029 iocg082*/
		LOW_POWER(0x74, 0x01, 0x124, 0x0, 8, 2, 1, 0x0),

		/*gpio_067 gpio out low iomg030 iocg083*/
		LOW_POWER(0x78, 0x01, 0x128, 0x0, 8, 3, 1, 0x0),

		/*gpio_068 gpio out low iomg031 iocg084*/
		LOW_POWER(0x7C, 0x01, 0x12C, 0x0, 8, 4, 1, 0x0),

		/*gpio_069 gpio out low iomg032 iocg085*/
		LOW_POWER(0x80, 0x01, 0x130, 0x0, 8, 5, 1, 0x0),

		/*gpio_070 gpio out low iomg033 iocg086*/
		LOW_POWER(0x84, 0x01, 0x134, 0x0, 8, 6, 1, 0x0),

		/*gpio_071 gpio out low iomg034 iocg087*/
		LOW_POWER(0x88, 0x01, 0x138, 0x0, 8, 7, 1, 0x0),

		/*gpio_072 gpio in low iomg035 iocg088*/
		LOW_POWER(0x8C, 0x01, 0x13C, 0x0, 9, 0, 1, 0x0),

		/*gpio_073 gpio in low iomg036 iocg089*/
		LOW_POWER(0x90, 0x01, 0x140, 0x0, 9, 1, 1, 0x0),

		/*gpio_074 gpio out low iomg037 iocg090*/
		LOW_POWER(0x94, 0x01, 0x144, 0x0, 9, 2, 1, 0x1),

		/*gpio_075 isp_gpio out low iomg038 iocg091*/
		LOW_POWER(0x98, 0x00, 0x148, 0x0, 9, 3, 1, 0x0),

		/*gpio_076 gpio in low iomg039 iocg092*/
		LOW_POWER(0x9C, 0x01, 0x14C, 0x0, 9, 4, 1, 0x0),

		/*gpio_077 gpio in low iomg040 iocg093*/
		LOW_POWER(0xA0, 0x01, 0x150, 0x2, 9, 5, 0, 0x0),

		/*gpio_078 gpio out low iomg041 iocg094*/
		LOW_POWER(0xA4, 0x01, 0x154, 0x0, 9, 6, 1, 0x0),

		/*gpio_079 gpio out low iomg043 iocg098*/
		LOW_POWER(0xAC, 0x01, 0x168, 0x0, 9, 7, 1, 0x0),

		/*gpio_080 gpio in high iomg043 iocg099*/
		LOW_POWER(0xAC, 0x01, 0x16C, 0x1, 10, 0, 0, 0x0),

		/*gpio_081 gpio out high iomg043 iocg100*/
		LOW_POWER(0xAC, 0x01, 0x170, 0x0, 10, 1, 1, 0x1),

		/*gpio_082 gpio out low iomg043 iocg101*/
		LOW_POWER(0xAC, 0x01, 0x174, 0x0, 10, 2, 1, 0x0),

		/*gpio_083 gpio in nopull(phone) iomg044 iocg102*/
		LOW_POWER(0xB0, 0x01, 0x178, 0x0, 10, 3, 0, 0x0),

		/*gpio_084 gpio in none iomg045 iocg103*/
		LOW_POWER(0xB4, 0x01, 0x17C, 0x0, 10, 4, 0, 0x0),

		/*gpio_085 gpio in low iomg046 iocg104*/
		LOW_POWER(0xB8, 0x01, 0x180, 0x0, 10, 5, 0, 0x0),

		/*gpio_086 gpio in low iomg046 iocg105*/
		LOW_POWER(0xB8, 0x01, 0x184, 0x0, 10, 6, 0, 0x0),

		/*gpio_087 gpio in low iomg046 iocg106*/
		LOW_POWER(0xB8, 0x01, 0x188, 0x0, 10, 7, 0, 0x0),

		/*gpio_088 gpio in low iomg047 iocg107*/
		LOW_POWER(0xBC, 0x01, 0x18C, 0x2, 11, 0, 0, 0x0),

		/*gpio_089 gpio in low iomg047 iocg108*/
		LOW_POWER(0xBC, 0x01, 0x190, 0x2, 11, 1, 0, 0x0),

		/*gpio_090 gpio in low iomg047 iocg109*/
		LOW_POWER(0xBC, 0x01, 0x194, 0x2, 11, 2, 0, 0x0),

		/*gpio_091 gpio in low iomg047 iocg110*/
		LOW_POWER(0xBC, 0x01, 0x198, 0x2, 11, 3, 0, 0x0),

		/*gpio_092 gpio in low iomg047 iocg111*/
		LOW_POWER(0xBC, 0x01, 0x19C, 0x2, 11, 4, 0, 0x0),

		/*gpio_093 gpio in low iomg048 iocg112*/
		LOW_POWER(0xC0, 0x01, 0x1A0, 0x2, 11, 5, 0, 0x0),

		/*gpio_094 gpio in none iomg049 iocg113*/
		LOW_POWER(0xC4, 0x01, 0x1A4, 0x0, 11, 6, 0, 0x0),

		/*gpio_095 gpio out low iomg049 iocg114*/
		LOW_POWER(0xC4, 0x01, 0x1A8, 0x0, 11, 7, 1, 0x0),

		/*gpio_096 gpio out low iomg049 iocg115*/
		//LOW_POWER(0xC4, 0x01, 0x1AC, 0x0, 12, 0, 1, 0x0),
		LOW_POWER(0xC4, 0x01, 0x1AC, 0x1, 12, 0, 0, 0x0),
		/*gpio_097 gpio out low iomg050 iocg116*/
		LOW_POWER(0xC8, 0x01, 0x1B0, 0x0, 12, 1, 1, 0x0),

		/*gpio_098 gpio in low iomg049 iocg117*/
		LOW_POWER(0xC4, 0x01, 0x1B4, 0x2, 12, 2, 0, 0x0),

		/*gpio_099 gpio in low iomg049 iocg118*/
		LOW_POWER(0xC4, 0x01, 0x1B8, 0x2, 12, 3, 0, 0x0),

		/*gpio_100 gpio in low iomg051 iocg119*/
		LOW_POWER(0xCC, 0x01, 0x1BC, 0x2, 12, 4, 0, 0x0),

		/*gpio_101 gpio in low iomg051 iocg120*/
		LOW_POWER(0xCC, 0x01, 0x1C0, 0x2, 12, 5, 0, 0x0),

		/*gpio_102 gpio in low iomg051 iocg121*/
		LOW_POWER(0xCC, 0x01, 0x1C4, 0x2, 12, 6, 0, 0x0),

		/*gpio_103 gpio in low iomg052 iocg122*/
		LOW_POWER(0xD0, 0x01, 0x1C8, 0x2, 12, 7, 0, 0x0),

		/*gpio_104 gpio in low iomg051 iocg123*/
		LOW_POWER(0xCC, 0x01, 0x1CC, 0x2, 13, 0, 0, 0x0),

		/*gpio_105 gpio in low iomg051 iocg124*/
		LOW_POWER(0xCC, 0x01, 0x1D0, 0x2, 13, 1, 0, 0x0),

		/*gpio_106 gpio in low iomg053 iocg125*/
		LOW_POWER(0xD4, 0x01, 0x1D4, 0x2, 13, 2, 0, 0x0),

		/*gpio_107 gpio in low iomg053 iocg126*/
		LOW_POWER(0xD4, 0x01, 0x1D8, 0x2, 13, 3, 0, 0x0),

		/*gpio_108 gpio in low iomg053 iocg127*/
		LOW_POWER(0xD4, 0x01, 0x1DC, 0x2, 13, 4, 0, 0x0),

		/*gpio_109 gpio in low iomg054 iocg128*/
		LOW_POWER(0xD8, 0x01, 0x1E0, 0x2, 13, 5, 0, 0x0),

		/*gpio_110 gpio in low iomg055 iocg129*/
		LOW_POWER(0xDC, 0x01, 0x1E4, 0x2, 13, 6, 0, 0x0),

		/*gpio_111 gpio in none iomg056 iocg130*/
		//LOW_POWER(0xE0, 0x01, 0x1E8, 0x0, 13, 7, 0, 0x0),
		LOW_POWER(0xE0, 0x01, 0x1E8, 0x2, 13, 7, 0, 0x0),

		/*gpio_112 gpio in none iomg057 iocg131*/
		//LOW_POWER(0xE4, 0x01, 0x1EC, 0x0, 14, 0, 0, 0x0),
		LOW_POWER(0xE4, 0x01, 0x1EC, 0x2, 14, 0, 0, 0x0),

		/*gpio_113 gpio in low iomg058 iocg132*/
		LOW_POWER(0xE8, 0x01, 0x1F0, 0x2, 14, 1, 0, 0x0),

		/*gpio_114 gpio out low iomg058 iocg133*/
		LOW_POWER(0xE8, 0x01, 0x1F4, 0x0, 14, 2, 1, 0x0),

		/*gpio_115 gpio out low iomg058 iocg134*/
		LOW_POWER(0xE8, 0x01, 0x1F8, 0x0, 14, 3, 1, 0x0),

		/*gpio_116 gpio in low iomg095 iocg135*/
		LOW_POWER(0xEC, 0x01, 0x1FC, 0x2, 14, 4, 0, 0x0),

		/*gpio_117 gpio out low iomg059 iocg136*/
		//LOW_POWER(0xF0, 0x01, 0x200, 0x0, 14, 5, 1, 0x0),
		LOW_POWER(0xF0, 0x01, 0x200, 0x2, 14, 5, 0, 0x0),

		/*gpio_118 gpio out low iomg059 iocg137*/
		LOW_POWER(0xF0, 0x01, 0x204, 0x0, 14, 6, 1, 0x0),

		/*gpio_119 gpio in low iomg060 iocg138*/
		LOW_POWER(0xF4, 0x01, 0x208, 0x2, 14, 7, 0, 0x0),

		/*gpio_120 gpio in low iomg060 iocg139*/
		LOW_POWER(0xF4, 0x01, 0x20C, 0x2, 15, 0, 0, 0x0),

		/*gpio_121 gpio in low iomg061 iocg140*/
		LOW_POWER(0xF8, 0x01, 0x210, 0x2, 15, 1, 0, 0x0),

		/*gpio_122 gpio in low iomg061 iocg141*/
		LOW_POWER(0xF8, 0x01, 0x214, 0x2, 15, 2, 0, 0x0),

		/*gpio_123 gpio in low iomg062 iocg142*/
		LOW_POWER(0xFC, 0x01, 0x218, 0x2, 15, 3, 0, 0x0),

		/*gpio_124 gpio in low iomg062 iocg143*/
		LOW_POWER(0xFC, 0x01, 0x21C, 0x2, 15, 4, 0, 0x0),

		/*gpio_125 gpio in low iomg063 iocg144*/
		LOW_POWER(0x100, 0x01, 0x220, 0x2, 15, 5, 0, 0x0),

		/*gpio_126 gpio in pullup iomg063 iocg145*/
		LOW_POWER(0x100, 0x01, 0x224, -1, 15, 6, 0, 0x0),

		/*gpio_127 gpio in pullup iomg096 iocg146*/
		LOW_POWER(0x104, 0x01, 0x228, 0x2, 15, 7, 0, 0x0),

		/*gpio_128 gpio in pulldown iomg064 iocg147*/
		LOW_POWER(0x108, 0x01, 0x22C, 0x2, 16, 0, 0, 0x0),

		/*gpio_129 gpio in low iomg065 iocg148*/
		LOW_POWER(0x10C, 0x01, 0x230, 0x2, 16, 1, 0, 0x0),

		/*gpio_130 gpio in low iomg066 iocg149*/
		LOW_POWER(0x110, 0x01, 0x234, 0x2, 16, 2, 0, 0x0),

		/*gpio_131 gpio in low iomg067 iocg150*/
		LOW_POWER(0x114, 0x01, 0x238, 0x2, 16, 3, 0, 0x0),

		/*gpio_132 gpio in low iomg068 iocg151*/
		LOW_POWER(0x118, 0x01, 0x23C, 0x2, 16, 4, 0, 0x0),

		/*gpio_133 gpio out low iomg069 iocg152*/
		LOW_POWER(0x11C, 0x01, 0x240, 0x0, 16, 5, 1, 0x0),

		/*gpio_134 gpio out low iomg070 iocg153*/
		LOW_POWER(0x120, 0x01, 0x244, 0x0, 16, 6, 1, 0x0),

		/*gpio_135 gpio in none iomg071 iocg154*/
		LOW_POWER(0x124, 0x01, 0x248, 0x0, 16, 7, 0, 0x0),

		/*gpio_136 gpio in none iomg072 iocg155*/
		LOW_POWER(0x128, 0x01, 0x24C, 0x0, 17, 0, 0, 0x0),

		/*gpio_137 gpio in pullup iomg073 iocg156*/
		LOW_POWER(0x12C, 0x01, 0x250, 0x1, 17, 1, 0, 0x0),

		/*gpio_138 gpio in pullup iomg074 iocg157*/
		LOW_POWER(0x130, 0x01, 0x254, 0x1, 17, 2, 0, 0x0),

		/*gpio_139 gpio in low iomg075 iocg158*/
		LOW_POWER(0x134, 0x01, 0x258, 0x2, 17, 3, 0, 0x0),

		/*gpio_140 gpio in low iomg076 iocg159*/
		LOW_POWER(0x138, 0x01, 0x25C, 0x2, 17, 4, 0, 0x0),

		/*gpio_141 gpio in pulldown iomg077 iocg160*/
		LOW_POWER(0x13C, 0x01, 0x260, 0x2, 17, 5, 0, 0x0),

		/*gpio_142 gpio in pulldown iomg078 iocg161*/
		LOW_POWER(0x140, 0x01, 0x264, 0x2, 17, 6, 0, 0x0),

		/*gpio_143 gpio in pullup iomg079 iocg162*/
		LOW_POWER(0x144, 0x01, 0x268, 0x1, 17, 7, 0, 0x0),

		/*gpio_144 gpio out high iomg080 iocg163*/
		LOW_POWER(0x148, 0x01, 0x26C, 0x0, 18, 0, 1, 0x1),

		/*gpio_145 gpio out high iomg081 iocg164*//*fixed*/
		//LOW_POWER(0x14C, 0x01, 0x270, 0x0, 18, 1, 1, 0x1),
        LOW_POWER(0x14C, 0x01, 0x270, 0x2, 18, 1, 0, 0x0),//gpio in low

		/*gpio_146 gpio out high iomg081 iocg165*/
		LOW_POWER(0x14C, 0x01, 0x274, 0x0, 18, 2, 0, 0x0),

		/*gpio_147 gpio in low iomg097 iocg166*/
		LOW_POWER(0x150, 0x01, 0x278, 0x2, 18, 3, 0, 0x0),

		/*gpio_148 gpio in low iomg097 iocg167*/
		LOW_POWER(0x150, 0x01, 0x27C, 0x2, 18, 4, 0, 0x0),

		/*gpio_149 gpio out low iomg082 iocg168*/
		LOW_POWER(0x154, 0x01, 0x280, 0, 18, 5, 1, 0x0),

		/*gpio_150 gpio in low iomg083 iocg169*/
		LOW_POWER(0x158, 0x01, 0x284, 0x2, 18, 6, 0, 0x0),

		/*gpio_151 gpio out low iomg084 iocg170*/
		LOW_POWER(0x15C, 0x01, 0x288, 0, 18, 7, 1, 0x0),

		/*gpio_152 gpio out high iomg084 iocg171*/
		LOW_POWER(0x15C, 0x01, 0x28C, 0, 19, 0, 1, 0x1),

		/*gpio_153 gpio in low iomg084 iocg172*/
		LOW_POWER(0x15C, 0x01, 0x290, 0, 19, 1, 1, 0x0),

		/*gpio_154 gpio in low iomg085 iocg173*/
		LOW_POWER(0x160, 0x01, 0x294, 0x2, 19, 2, 0, 0x0),

		/*gpio_155 gpio in low iomg085 iocg174*/
		LOW_POWER(0x160, 0x01, 0x298, 0x2, 19, 3, 0, 0x0),
		/*gpio_156 gpio out high iomg--- iocg000*/
	//	LOW_POWER(0x160, -1, 0x00C, 0, 19, 4, 1, 0x1), // dir: out;  val: high
		LOW_POWER(0x160, -1, 0x00C, 0x2, 19, 4, 0, 0x0), // dir: in;  pull: down; val: low

		/*gpio_157 gpio in pullup iomg--- iocg001*/
	//	LOW_POWER(0x160, -1, 0x010, 0x1, 19, 5, 0, 0x0),  // dir: in; pull: up
		LOW_POWER(0x160, -1, 0x010, 0x2, 19, 5, 0, 0x0), // dir: in; pull: down

		/*gpio_158 gpio out high iomg--- iocg002*/
		LOW_POWER(0x160, -1, 0x014, 0, 19, 6, 1, 0x1),

		/*gpio_159 gpio in high iomg--- iocg003*/
		LOW_POWER(0x160, -1, 0x018, 0x1, 19, 7, 0, 0x0),

		/*gpio_160 gpio in low iomg086 iocg175*/
		LOW_POWER(0x164, 0x01, 0x29C, 0x2, 20, 0, 0, 0x0),

		/*gpio_161 gpio in low iomg086 iocg176*/
		LOW_POWER(0x164, 0x01, 0x2A0, 0x2, 20, 1, 0, 0x0),

		/*gpio_162 gpio in low iomg087 iocg177*/
		LOW_POWER(0x168, 0x01, 0x2A4, 0x2, 20, 2, 0, 0x0),

		/*gpio_163 gpio in low iomg087 iocg178*/
		LOW_POWER(0x168, 0x01, 0x2A8, 0x2, 20, 3, 0, 0x0),

		/*gpio_164 gpio in low iomg088 iocg179*/
		LOW_POWER(0x16C, 0x01, 0x2AC, 0x2, 20, 4, 0, 0x0),

		/*gpio_165 gpio in low iomg093 iocg180*/
		LOW_POWER(0x170, 0x01, 0x2B0, 0x2, 20, 5, 0, 0x0),

		/*gpio_166 gpio in low iomg089 iocg181*/
		LOW_POWER(0x174, 0x01, 0x2B4, 0x2, 20, 6, 0, 0x0),

		/*gpio_167 gpio out low iomg089 iocg182*/  /*fix me*/
		LOW_POWER(0x174, 0x01, 0x2B8, 0x0, 20, 7, 1, 0x0),

		/*gpio_168 gpio out low iomg089 iocg183*/   /*fix me*/
		LOW_POWER(0x174, 0x01, 0x2BC, 0x0, 21, 0, 1, 0x0),

		/*gpio_169 gpio out low iomg089 iocg184*/
		LOW_POWER(0x174, 0x01, 0x2C0, 0x0, 21, 1, 1, 0x0),

		/*gpio_170 gpio in high iomg089 iocg185*/
		LOW_POWER(0x174, 0x01, 0x2C4, 0x1, 21, 2, 0, 0x0),

		/*gpio_171 gpio out low iomg090 iocg186*/
		LOW_POWER(0x178, 0x01, 0x2C8, 0x0, 21, 3, 1, 0x0),

		/*gpio_172 gpio out low iomg091 iocg187*/
		LOW_POWER(0x17C, 0x01, 0x2CC, 0x0, 21, 4, 1, 0x0),

		/*gpio_173 gpio out low iomg091 iocg188*/
		LOW_POWER(0x17C, 0x01, 0x2D0, 0x0, 21, 5, 1, 0x0),

		/*gpio_174 gpio out low iomg091 iocg189*/
	//	LOW_POWER(0x17C, 0x01, 0x2D4, 0x0, 21, 6, 1, 0x0),
		LOW_POWER(0x17C, 0x01, 0x2D4, 0x1, 21, 6, 0, 0x0),   //modified to dir:in, pullup  for fix mhl suspend consume about 0.3ma current.

		/*gpio_175 gpio in high iomg092 iocg190*/
		LOW_POWER(0x180, 0x01, 0x2D8, 0x2, 21, 7, 0, 0x0),
};

#define IOMG_REG(x)			(g_ioc_addr + (iocfg_lookups[x].uiomg_off))
#define IOCG_REG(x)			(g_ioc_addr + 0x800 + (iocfg_lookups[x].uiocg_off))

#define GPIO_DIR(x)			(g_gpio_addr + (x)*0x1000 + 0x400)
#define GPIO_DATA(x)		(g_gpio_addr + (x)*0x1000 + 0x3FC)
#define GPIO_BIT(x, y)		((x) << (y))
#define GPIO_IS_SET(x)		(((uregv)>>(x))&0x1)

#define DEBG_SUSPEND_PRINTK		(1<<0)
#define DEBG_SUSPEND_IO_SHOW	(1<<1)
#define DEBG_SUSPEND_PMU_SHOW	(1<<2)
#define DEBG_SUSPEND_IO_SET		(1<<3)
#define DEBG_SUSPEND_PMU_SET	(1<<4)
#define DEBG_SUSPEND_IO_S_SET	(1<<5)
#define DEBG_SUSPEND_RTC_EN		(1<<6)
#define DEBG_SUSPEND_TIMER_EN	(1<<7)
#define DEBG_SUSPEND_WAKELOCK	(1<<8)
#define DEBG_SUSPEND_AUDIO		(1<<9)

static int g_suspended;
//static unsigned g_usavedcfg= (DEBG_SUSPEND_IO_SET | DEBG_SUSPEND_PMU_SET );
static unsigned g_usavedcfg;
static unsigned g_utimer_inms;
static unsigned g_urtc_ins;

#define GPIO_GROUP_NUM  22
#define IO_MGREG_NUM      96   //  (0xFC80_3180 - 0xFC80_3000 )/4 

unsigned int gpio_dir_array[GPIO_GROUP_NUM] = {0};
unsigned int gpio_data_array[GPIO_GROUP_NUM] = {0};
unsigned int iomg_array[IO_MGREG_NUM] = {0};

struct iocg_struct {

	unsigned int iocg_off;
	unsigned int value;    

};
struct iocg_struct iocg_array[] = 
{
{0x024, 0}, 
{0x028, 0}, 
{0x02C, 0}, 
{0x030, 0}, 
{0x034, 0}, 
{0x038, 0}, 
{0x03C, 0}, 
{0x040, 0}, 
{0x044, 0}, 
{0x048, 0}, 
{0x04C, 0}, 
{0x050, 0}, 
{0x054, 0}, 
{0x058, 0}, 
{0x05C, 0}, 
{0x060, 0}, 
{0x064, 0}, 
{0x068, 0}, 
{0x06C, 0}, 
{0x070, 0}, 
{0x074, 0}, 
{0x078, 0}, 
{0x07C, 0}, 
{0x080, 0}, 
{0x084, 0}, 
{0x088, 0}, 
{0x08C, 0}, 
{0x090, 0}, 
{0x094, 0}, 
{0x098, 0}, 
{0x09C, 0}, 
{0x0A0, 0}, 
{0x0A4, 0}, 
{0x0A8, 0}, 
{0x0AC, 0}, 
{0x0B0, 0}, 
{0x0B4, 0}, 
{0x0B8, 0}, 
{0x0BC, 0},  //
{0x0C0, 0}, 
{0x0C4, 0}, 
{0x0C8, 0}, 
{0x0CC, 0}, 
{0x0D0, 0}, 
{0x0D4, 0}, 
{0x0D8, 0}, 
{0x0DC, 0}, 
{0x0E0, 0}, 
{0x0E4, 0}, 
{0x0E8, 0}, 
{0x0EC, 0}, 
{0x0F0, 0}, 
{0x0F4, 0}, 
{0x0F8, 0}, 
{0x0FC, 0}, 
{0x100, 0}, 
{0x104, 0}, 
{0x108, 0}, 
{0x110, 0}, 
{0x114, 0}, 
{0x118, 0}, 
{0x11C, 0}, 
{0x120, 0}, 
{0x124, 0}, 
{0x128, 0}, 
{0x12C, 0}, 
{0x130, 0}, 
{0x134, 0}, 
{0x138, 0}, 
{0x13C, 0}, 
{0x140, 0}, 
{0x144, 0}, 
{0x148, 0}, 
{0x14C, 0}, 
{0x150, 0}, 
{0x154, 0}, 
{0x168, 0}, 
{0x16C, 0}, 
{0x170, 0}, 
{0x174, 0}, 
{0x178, 0},  //
{0x17C, 0}, 
{0x180, 0}, 
{0x184, 0}, 
{0x188, 0}, 
{0x18C, 0}, 
{0x190, 0}, 
{0x194, 0}, 
{0x198, 0}, 
{0x19C, 0}, 
{0x1A0, 0}, 
{0x1A4, 0}, 
{0x1A8, 0}, 
{0x1AC, 0}, 
{0x1B0, 0}, 
{0x1B4, 0}, 
{0x1B8, 0}, 
{0x1BC, 0}, 
{0x1C0, 0}, 
{0x1C4, 0}, 
{0x1C8, 0}, 
{0x1CC, 0}, 
{0x1D0, 0}, 
{0x1D4, 0}, 
{0x1D8, 0}, 
{0x1DC, 0}, 
{0x1E0, 0}, 
{0x1E4, 0}, 
{0x1E8, 0}, 
{0x1EC, 0}, 
{0x1F0, 0}, 
{0x1F4, 0}, 
{0x1F8, 0}, 
{0x1FC, 0}, 
{0x200, 0}, 
{0x204, 0}, 
{0x208, 0}, 
{0x20C, 0}, 
{0x210, 0}, 
{0x214, 0}, 
{0x218, 0}, 
{0x21C, 0}, 
{0x220, 0}, 
{0x224, 0}, 
{0x228, 0}, 
{0x22C, 0}, 
{0x230, 0},  //
{0x234, 0}, 
{0x238, 0}, 
{0x23C, 0},  //
{0x240, 0}, 
{0x244, 0}, 
{0x248, 0}, 
{0x24C, 0}, 
{0x250, 0},  //
{0x254, 0}, 
{0x258, 0}, 
{0x25C, 0},  //
{0x260, 0}, 
{0x264, 0}, 
{0x268, 0}, 
{0x26C, 0}, 
{0x270, 0},  //
{0x274, 0},  //
{0x278, 0}, 
{0x27C, 0}, 
{0x280, 0}, 
{0x284, 0}, 
{0x288, 0}, 
{0x28C, 0}, 
{0x290, 0}, 
{0x294, 0}, 
{0x298, 0}, 
{0x00C, 0}, 
{0x010, 0}, 
{0x014, 0}, 
{0x018, 0}, 
{0x29C, 0}, 
{0x2A0, 0}, 
{0x2A4, 0}, 
{0x2A8, 0}, 
{0x2AC, 0}, 
{0x2B0, 0}, 
{0x2B4, 0}, 
{0x2B8, 0}, 
{0x2BC, 0}, 
{0x2C0, 0}, 
{0x2C4, 0}, 
{0x2C8, 0}, 
{0x2CC, 0}, 
{0x2D0, 0}, 
{0x2D4, 0}, 
{0x2D8, 0},
};

#define IOMG_WF_REG(x)			(g_ioc_addr + x*4)
#define IOCG_WF_REG(x)			(g_ioc_addr + 0x800 + (iocg_array[x].iocg_off))
void storeiostatus(void)
{
	int i = 0;
	int iocg_len = sizeof(iocg_array)/sizeof(iocg_array[0]);
	printk(KERN_ERR "%s: enter iocg_len = %d\n",__func__,iocg_len);
	
//	if (!(g_usavedcfg & DEBG_SUSPEND_IO_SET))
//		return;
	//store gpio dir and data reg value
	for(i = 0; i < GPIO_GROUP_NUM; i++)
	{
		gpio_dir_array[i] = readl(GPIO_DIR(i));
		gpio_data_array[i] = readl(GPIO_DATA(i));
	}
	
	//store io mg reg value
	for(i = 0; i < IO_MGREG_NUM; i++)
	{
		iomg_array[i] = readl(IOMG_WF_REG(i));
	}
	#if 1
	//store io cg reg value
	for(i = 0; i < iocg_len; i++)
	{
		iocg_array[i].value = readl(IOCG_WF_REG(i));;
	}
	#endif
	printk(KERN_ERR "%s: exit\n",__func__);
}

void restoreiostatus(void)
{
	int i = 0;	
	int iocg_len = sizeof(iocg_array)/sizeof(iocg_array[0]);
	printk(KERN_ERR "%s: enter iocg_len = %d\n",__func__,iocg_len);
//	if (!(g_usavedcfg & DEBG_SUSPEND_IO_SET))
//		return;
	
	//restore io mg reg value
	for(i = 0; i < IO_MGREG_NUM; i++)
	{
		writel(iomg_array[i], IOMG_WF_REG(i));
	}
	#if 1
	//restore io cg reg value
	for(i = 0; i < iocg_len; i++)
	{
		writel(iocg_array[i].value , IOCG_WF_REG(i));
	}
	#endif

	//restore gpio dir and data reg value
	for(i = 0; i < GPIO_GROUP_NUM; i++)
	{
		writel(gpio_dir_array[i], GPIO_DIR(i));
		writel(gpio_data_array[i] , GPIO_DATA(i));
	}
	
	printk(KERN_ERR "%s: exit\n",__func__);
}



/*****************************************************************
* function: setiolowpower
* description:
*  set all io to low power state.
******************************************************************/
void setiolowpower(void)
{
	int i = 0;
	int ilen = sizeof(iocfg_lookups)/sizeof(iocfg_lookups[0]);
	unsigned int uregv = 0;

	if (!(g_usavedcfg & DEBG_SUSPEND_IO_SET))
		return;

	pr_info("[%s] %d enter.\n", __func__, __LINE__);

	for (i = 0; i < ilen; i++) {

		uregv = ((iocfg_lookups[i].ugpiog<<3)+iocfg_lookups[i].ugpio_bit);

		/*uart0 suspend printk*/
		if ((0 == console_suspend_enabled)
			&& ((uregv >= 117) && (uregv <= 120)))
			continue;

		if (E_BOARD_TYPE_PLATFORM == get_board_type()) {
			/*oem board*/
			if ((uregv == 40) || (uregv == 83))
				continue;

			if ((uregv >= 129) && (uregv <= 132))
				continue;

			if ((uregv >= 137) && (uregv <= 140))
				continue;
		} else {
			if ((uregv == 145) || (uregv == 146))
				continue;
		}

		uregv = readl(IOMG_REG(i));
		if (iocfg_lookups[i].iomg_val != -1) {
			if ((uregv&0x1) == iocfg_lookups[i].iomg_val)
				writel(uregv, IOMG_REG(i));
			else
				writel(iocfg_lookups[i].iomg_val, IOMG_REG(i));
		}

		uregv = readl(IOCG_REG(i));
		if (iocfg_lookups[i].iocg_val != -1) {
			if ((uregv&0x3) == iocfg_lookups[i].iocg_val)
				writel(uregv, IOCG_REG(i));
			else
				writel(iocfg_lookups[i].iocg_val, IOCG_REG(i));
		}

		uregv = readl(GPIO_DIR(iocfg_lookups[i].ugpiog));
		uregv &= ~GPIO_BIT(1, iocfg_lookups[i].ugpio_bit);
		uregv |= GPIO_BIT(iocfg_lookups[i].gpio_dir, iocfg_lookups[i].ugpio_bit);
		writel(uregv, GPIO_DIR(iocfg_lookups[i].ugpiog));

		uregv = readl(GPIO_DIR(iocfg_lookups[i].ugpiog));
		uregv = readl(GPIO_DATA(iocfg_lookups[i].ugpiog));
		uregv &= ~GPIO_BIT(1, iocfg_lookups[i].ugpio_bit);
		uregv |= GPIO_BIT(iocfg_lookups[i].gpio_val, iocfg_lookups[i].ugpio_bit);
		writel(uregv, GPIO_DATA(iocfg_lookups[i].ugpiog));

	}

	pr_info("[%s] %d leave.\n", __func__, __LINE__);
}

/*****************************************************************
* function: setiomuxvalue
* description:
*  set specified io to low power state.
******************************************************************/
void setiomuxvalue(int igpio)
{
	unsigned uregv = 0;

	if (!(g_usavedcfg & DEBG_SUSPEND_IO_S_SET))
		return;

	uregv = readl(IOMG_REG(igpio));
	if (iocfg_lookups[igpio].iomg_val != -1) {
		if ((uregv&iocfg_lookups[igpio].iomg_val) == iocfg_lookups[igpio].iomg_val)
			writel(uregv, IOMG_REG(igpio));
		else
			writel(iocfg_lookups[igpio].iomg_val, IOMG_REG(igpio));
	}

	uregv = readl(IOCG_REG(igpio));
	if (iocfg_lookups[igpio].iocg_val != -1) {
		if ((uregv&iocfg_lookups[igpio].iocg_val) == iocfg_lookups[igpio].iocg_val)
			writel(uregv, IOCG_REG(igpio));
		else
			writel(iocfg_lookups[igpio].iocg_val, IOCG_REG(igpio));
	}


	if (iocfg_lookups[igpio].gpio_dir != -1) {
		uregv = readl(GPIO_DIR(iocfg_lookups[igpio].ugpiog));
		uregv |= GPIO_BIT(iocfg_lookups[igpio].gpio_dir, iocfg_lookups[igpio].ugpio_bit);
		writel(uregv, GPIO_DIR(iocfg_lookups[igpio].ugpiog));

		uregv = readl(GPIO_DATA(iocfg_lookups[igpio].ugpiog));
		uregv |= GPIO_BIT(iocfg_lookups[igpio].gpio_val, iocfg_lookups[igpio].ugpio_bit);
		writel(uregv, GPIO_DATA(iocfg_lookups[igpio].ugpiog));
	}
}

/*****************************************************************
* function: ioshowstatus
* description:
*  show io status.
******************************************************************/
void ioshowstatus(int check)
{
	int i = 0;
	int ilen = sizeof(iocfg_lookups)/sizeof(iocfg_lookups[0]);
	unsigned int uregv = 0;
	int iflg = 0;

	if (!(g_usavedcfg & DEBG_SUSPEND_IO_SHOW))
		return;

	pr_info("[%s] %d enter.\n", __func__, __LINE__);

	for (i = 0; i < ilen; i++) {

		iflg = 0;

		printk("GPIO_%02d_%d (%03d) ",
			iocfg_lookups[i].ugpiog, iocfg_lookups[i].ugpio_bit,
			((iocfg_lookups[i].ugpiog<<3)+iocfg_lookups[i].ugpio_bit));

		uregv = readl(IOMG_REG(i));
		printk("IOMG=0x%02X ", uregv);

		if (check == 1) {
			if ((uregv == iocfg_lookups[i].iomg_val)
				|| (-1 == iocfg_lookups[i].iomg_val))
				printk("(0x%02X) ", (unsigned char)uregv);
			else {
				iflg = 1;
				printk("(0x%02X) ", (unsigned char)iocfg_lookups[i].iomg_val);
			}
		}

		uregv = readl(IOCG_REG(i));
		printk("IOCG=0x%02X ", uregv);

		if (check == 1) {
			if (((uregv & 0x3) == iocfg_lookups[i].iocg_val)
				|| (-1 == iocfg_lookups[i].iocg_val))
				printk("(0x%02X) ", (unsigned char)uregv);
			else {
				iflg = 1;
				printk("(0x%02X) ", (unsigned char)iocfg_lookups[i].iocg_val);
			}
		}

		uregv = readl(GPIO_DIR(iocfg_lookups[i].ugpiog));
		printk("DIR=0x%02X ", GPIO_IS_SET(iocfg_lookups[i].ugpio_bit));

		if (check == 1) {
			if ((uregv & GPIO_BIT(1, iocfg_lookups[i].ugpio_bit))
				== (GPIO_BIT(iocfg_lookups[i].gpio_dir, iocfg_lookups[i].ugpio_bit)))
				printk("(0x%02X) ", GPIO_IS_SET(iocfg_lookups[i].ugpio_bit));
			else {
				iflg = 1;
				printk("(0x%02X) ", (unsigned char)iocfg_lookups[i].gpio_dir);
			}
		}

		uregv = readl(GPIO_DATA(iocfg_lookups[i].ugpiog));
		printk("VAL=0x%02X ", GPIO_IS_SET(iocfg_lookups[i].ugpio_bit));

		if (check == 1) {
			if (((uregv & GPIO_BIT(1, iocfg_lookups[i].ugpio_bit))
				== GPIO_BIT(iocfg_lookups[i].gpio_val, iocfg_lookups[i].ugpio_bit))
				|| (uregv & GPIO_BIT(iocfg_lookups[i].iocg_val, iocfg_lookups[i].ugpio_bit)))
				printk("(0x%02X) ", GPIO_IS_SET(iocfg_lookups[i].ugpio_bit));
			else {
				iflg = 1;
				printk("(0x%02X) ", (unsigned char)iocfg_lookups[i].gpio_val);
			}
		}

		if (iflg == 1)
			printk("e");

		printk("\n");
	}

	pr_info("[%s] %d leave.\n", __func__, __LINE__);
}

#endif

/*****************************************************************
* function:    pmuspi_enable
* description:
*  enable pmu clk.
******************************************************************/
static void pmuspi_enable(void)
{
	/*set clk div*/
	writel(0xFF0003, (g_pctrl_addr + 0x8));

	/*enable clk*/
	writel(1<<1, (g_sc_addr + 0x40));

	/*undo reset*/
	writel(1<<1, (g_sc_addr + 0x9C));
}

/*****************************************************************
* function:    pmuspi_disable
* description:
*  disable pmu clk.
******************************************************************/
static void pmuspi_disable(void)
{
	/*reset*/
	writel(1<<1, (g_sc_addr + 0x98));

	/*disable clk*/
	writel(1<<1, (g_sc_addr + 0x44));
}

#define PMUSPI_REG(x) (g_pmuspi_addr + ((x)<<2))

struct pmuregs {
	unsigned char ucoffset;
	char cval;
	char old_val;
	char cmask;
};

#define PMU_LOW(x, y, z) { .ucoffset = (x), .cval = (y), .cmask = (z), .old_val = 0,}

static struct pmuregs pmuregs_lookups[] = {
	/*close LDO0 */
	PMU_LOW(0x20, 0x00, 0x10),

	/*w 35 0, emmc rst2_n output low.*/
	PMU_LOW(0x35, 0x00, 0x01),

	/*close ldo13*/
	PMU_LOW(0x2D, 0x30, 0x30),

	/*w 25 31  LDO5 ECO mode*/
	/*PMU_LOW(0x25, 0x20, 0x20),*/

	/*w 26 34  LDO6 ECO mode*/
	/*PMU_LOW(0x26, 0x20, 0x20),*/

	/*w 4e 20 close over temperature protect*/
	PMU_LOW(0x4E, 0x00, 0x01),

	/*w 52 00 close backup battery charging*/
	PMU_LOW(0x52, 0x00, 0x01),

	/*w 14 11  BUCK4 Sleep*/
	PMU_LOW(0x14, 0x11, 0x11),

	/*w 16 11  BUCK5 Sleep*/
	PMU_LOW(0x16, 0x11, 0x11),

	/*w 8f 08 sleep*/
	/*PMU_LOW(0x8F, 0x08, 0x07),*/
};


/*****************************************************************
* function: pmulowpower
* description:
*  configure pmu low power state.
******************************************************************/
void pmulowpower(int isuspend)
{
	int i = 0;
	int ilen = ARRAY_SIZE(pmuregs_lookups);
	unsigned uregv = 0;

	pr_info("[%s] %d enter.\n", __func__, __LINE__);

	pmuspi_enable();

	if (1 == isuspend) {
		for (i = 0; i < ilen; i++) {
			uregv = readl(PMUSPI_REG(pmuregs_lookups[i].ucoffset));
			pmuregs_lookups[i].old_val = uregv;
			uregv &= ~pmuregs_lookups[i].cmask;
			uregv |= pmuregs_lookups[i].cval;

#ifdef CONFIG_DEBUG_FS
			if (g_usavedcfg & DEBG_SUSPEND_PMU_SHOW)
				pr_info("[%s] %d %02d reg_%02x=%02x old=%02x.\n", __func__,
				__LINE__, i, pmuregs_lookups[i].ucoffset,
				uregv, pmuregs_lookups[i].old_val);
#endif
			writel(uregv, PMUSPI_REG(pmuregs_lookups[i].ucoffset));
		}

	} else {
		for (i = (ilen - 1); i >= 0; i--) {
			uregv = readl(PMUSPI_REG(pmuregs_lookups[i].ucoffset));
			uregv &= ~pmuregs_lookups[i].cmask;
			uregv |= pmuregs_lookups[i].old_val;
			writel(uregv, PMUSPI_REG(pmuregs_lookups[i].ucoffset));
		}
	}

	if (1 == isuspend)
		pmuspi_disable();

	pr_info("[%s] %d leave.\n", __func__, __LINE__);
}

#ifdef CONFIG_LOWPM_DEBUG

#include <linux/wakelock.h>

static struct wake_lock lowpm_wake_lock;

/****************************************
*function: debuguart_reinit
*description:
*  reinit debug uart.
*****************************************/
void debuguart_reinit(void)
{
	unsigned int usctrl_base = IO_ADDRESS(REG_BASE_SCTRL);
	unsigned int uuart_base  = IO_ADDRESS(REG_EDB_UART);
	unsigned int io_base = IO_ADDRESS(REG_BASE_IOC);
	unsigned int uregv = 0;

	/* Config necessary IOMG configuration */
	writel(0, (io_base+0xF4));

	/* config necessary IOCG configuration */
	writel(0, (io_base+0xA08));
	writel(0, (io_base+0xA0C));

	/*disable clk*/
	uregv = 0x10000;
	writel(uregv, (usctrl_base + 0x44));

	/*select 26MHz clock*/
	uregv = (1<<23);
	writel(uregv, (usctrl_base + 0x100));

	/*@ enable clk*/
	uregv = 0x10000;
	writel(uregv, (usctrl_base + 0x40));

	/*@;disable recieve and send*/
	uregv = 0x0;
	writel(uregv, (uuart_base + 0x30));

	/*@;enable FIFO*/
	uregv = 0x70;
	writel(uregv, (uuart_base + 0x2c));

	/*@;set baudrate*/
	uregv = 0xE;
	writel(uregv, (uuart_base + 0x24));

	uregv = 0x7;
	writel(uregv, (uuart_base + 0x28));

	/*@;clear buffer*/
	uregv = readl(uuart_base);

	/*@;enable FIFO*/
	uregv = 0x70;
	writel(uregv, (uuart_base + 0x2C));

	/*@;set FIFO depth*/
	uregv = 0x10A;
	writel(uregv, (uuart_base + 0x34));

	uregv = 0x50;
	writel(uregv, (uuart_base + 0x38));

	/*@;enable uart trans*/
	uregv = 0xF01;
	writel(uregv, (uuart_base + 0x30));
}

static struct pmuregs pmuregs_lookups_all[] = {

	/*w c9 FF close adc, all pga*/
	PMU_LOW(0xC9, 0xFF, 0xFF),

	/*w ca FE AVERF 500K,close mix,micbias,pll,ibias*/
	PMU_LOW(0xCA, 0xFE, 0xFF),

	/*w cb FF close DAC,OUTMIX,DAC MIX*/
	PMU_LOW(0xCB, 0xFF, 0xFF),

	/*w cc FF close all output*/
	PMU_LOW(0xCC, 0xFF, 0xFF),

	/*w 20 00 close LDO0*/
	PMU_LOW(0x20, 0x00, 0x10),

	/*w 21 00 close LDO1*/
	PMU_LOW(0x21, 0x00, 0x10),

	/*w 22 31 LDO2 ECO mode*/
	/*PMU_LOW(0x22, 0x20, 0x20),*/

	/*w 23 00 close LDO3*/
	/*PMU_LOW(0x23, 0x00, 0x10),*/

	/*w 24 00 close LDO4*/
	PMU_LOW(0x24, 0x00, 0x10),

	/*w 25 31 LDO5 ECO mode*/
	PMU_LOW(0x25, 0x20, 0x20),

	/*w 26 34 LDO6 ECO mode*/
	PMU_LOW(0x26, 0x20, 0x20),

	/*w 27 3x LDO7 ECO*/
	PMU_LOW(0x27, 0x30, 0x30),

	/*w 28 00 close LDO8*/
	PMU_LOW(0x28, 0x00, 0x10),

	/*close ldo13*/
	PMU_LOW(0x2D, 0x00, 0x10),

	/*w 35 0, emmc rst2_n output low.*/
	PMU_LOW(0x35, 0x00, 0x01),

	/*diable ldo3 hardware config.*/
	PMU_LOW(0x4C, 0x08, 0x08),

	/*w 4e 20 close over temperature protect*/
	PMU_LOW(0x4E, 0x00, 0x01),

	/*w 52 00 close backup battery charging*/
	PMU_LOW(0x52, 0x00, 0x01),

	/*w 53 00 disable culon*/
	PMU_LOW(0x53, 0x00, 0x02),

	/*w 0c 00 diable software control BUCK0*/
	PMU_LOW(0x0C, 0x00, 0xFF),

	/*w 0e 00 diable software control BUCK1*/
	PMU_LOW(0x0E, 0x00, 0xFF),

	/*w 10 00  diable software control BUCK2*/
	PMU_LOW(0x10, 0x00, 0xFF),

	/*w 12 00  diable software control BUCK3*/
	PMU_LOW(0x12, 0x00, 0xFF),

	/*w 14 11  BUCK4 Sleep*/
	PMU_LOW(0x14, 0x11, 0x11),

	/*w 16 11  BUCK5 Sleep*/
	PMU_LOW(0x16, 0x11, 0x11),

	/*w 8f 08 sleep*/
	PMU_LOW(0x8F, 0x08, 0x07),
};



/*****************************************************************
* function: pmulowpower
* description:
*  configure pmu low power state.
******************************************************************/
void pmulowpowerall(int isuspend)
{
	int i = 0;
	int ilen = ARRAY_SIZE(pmuregs_lookups_all);
	unsigned uregv = 0;

	if (!(g_usavedcfg & DEBG_SUSPEND_PMU_SET))
		return;

	pr_info("[%s] %d enter.\n", __func__, __LINE__);

	if (g_suspended == 1)
		pmuspi_enable();

	if (1 == isuspend) {
		for (i = 0; i < ilen; i++) {
			/*audio relative*/
			if (!(DEBG_SUSPEND_AUDIO & g_usavedcfg))
				if ((pmuregs_lookups_all[i].ucoffset == 0xC9)
					|| (pmuregs_lookups_all[i].ucoffset == 0xCA)
					|| (pmuregs_lookups_all[i].ucoffset == 0xCB)
					|| (pmuregs_lookups_all[i].ucoffset == 0xCC))
					continue;

			uregv = readl(PMUSPI_REG(pmuregs_lookups_all[i].ucoffset));
			pmuregs_lookups_all[i].old_val = uregv;
			uregv &= ~pmuregs_lookups_all[i].cmask;
			uregv |= pmuregs_lookups_all[i].cval;
			pr_info("[%s] %d %02d reg_%02x=%02x old=%02x.\n", __func__, __LINE__,
				i, pmuregs_lookups_all[i].ucoffset, uregv, pmuregs_lookups_all[i].old_val);
			writel(uregv, PMUSPI_REG(pmuregs_lookups_all[i].ucoffset));
		}
	} else {
		for (i = (ilen - 1); i >= 0; i--) {

			/*audio relative*/
			if (!(DEBG_SUSPEND_AUDIO & g_usavedcfg))
				if ((pmuregs_lookups_all[i].ucoffset == 0xC9)
					|| (pmuregs_lookups_all[i].ucoffset == 0xCA)
					|| (pmuregs_lookups_all[i].ucoffset == 0xCB)
					|| (pmuregs_lookups_all[i].ucoffset == 0xCC))
					continue;

			uregv = readl(PMUSPI_REG(pmuregs_lookups_all[i].ucoffset));
			uregv &= ~pmuregs_lookups_all[i].cmask;
			uregv |= pmuregs_lookups_all[i].old_val;
			writel(uregv, PMUSPI_REG(pmuregs_lookups_all[i].ucoffset));
		}
	}

	if ((g_suspended == 1) && (1 == isuspend))
		pmuspi_disable();

	pr_info("[%s] %d leave.\n", __func__, __LINE__);
}


/*****************************************************************
* function: pmulowpower_show
* description:
*  show pmu status.
******************************************************************/
void pmulowpower_show(int check)
{
	int i = 7;
	int ilen = ARRAY_SIZE(pmuregs_lookups_all);
	int index = 0;
	unsigned char uregv;

	if (!(g_usavedcfg & DEBG_SUSPEND_PMU_SHOW))
		return;

	pr_info("[%s] %d enter.\n", __func__, __LINE__);

	if (g_suspended == 1)
		pmuspi_enable();

	for (i = 7; i < 0xFF; i++) {
		uregv = readl(PMUSPI_REG(i));
		printk("PMU 0x%02X=0x%02X", i, uregv);

		if ((check == 1)
			&& (i == pmuregs_lookups_all[index].ucoffset)
			&& (index < ilen)) {
			printk(" (0x%02X)", pmuregs_lookups_all[index].cval);
			index++;
		}

		printk("\n");
	}

	if (g_suspended == 1)
		pmuspi_disable();

	pr_info("[%s] %d leave.\n", __func__, __LINE__);
}

void rtc_enable(void)
{
	unsigned uregv = 0;

	void __iomem * urtc_base = (void __iomem *)IO_ADDRESS(REG_BASE_RTC);

	if (!(g_usavedcfg&DEBG_SUSPEND_RTC_EN))
		return;

#if 0
	printk("%s %x\n, %x\n, %x\n, %x\n, %x\n, %x\n, %x\n, %x\n",
		__func__, readl(urtc_base),
		readl(urtc_base+0x4),
		readl(urtc_base+0x8),
		readl(urtc_base+0xc),
		readl(urtc_base+0x10),
		readl(urtc_base+0x14),
		readl(urtc_base+0x18),
		readl(urtc_base+0x1c));
#endif

	/*enable rtc*/
	/*writel(0x1, (urtc_base+0xC));*/

	/*initial LR*/
	/*writel(0x1, (urtc_base+0x8));*/

	/*enable interrupt*/
	writel(0x1, (urtc_base+0x10));

	/*clear intr*/
	writel(0x1, (urtc_base+0x1C));

	/*read current value*/
	uregv = readl(urtc_base);

	/*set cmp value*/
	uregv += g_urtc_ins;
	writel(uregv, (urtc_base+0x4));
}

/**Timer0 wakeup function**/
#define GT_CLK_TIMER1					(1<<3)
#define GT_PCLK_TIMER1					(1<<2)
#define GT_CLK_TIMER0					(1<<1)
#define GT_PCLK_TIMER0					(1<<0)

#define SCCTRL_TIMEREN1OV				(1<<18)
#define SCCTRL_TIMEREN1SEL				(1<<17)
#define SCCTRL_TIMEREN0OV				(1<<16)
#define SCCTRL_TIMEREN0SEL				(1<<15)
#define SCCTRL_TIMEFORCEHIGH			(1<<8)

#define IP_RST_TIMER0					(1<<0)

#define TIMEREN_BIT						(1<<7)
#define TIMERMODE_BIT					(1<<6)
#define TIMEINT_BIT						(1<<5)
#define TIMERSIZE_BIT					(1<<1)


void timer0_0_enable(void)
{
	unsigned uregv = 0;
	void __iomem * usctrl_base = (void __iomem *)IO_ADDRESS(REG_BASE_SCTRL);
	void __iomem * utimer0_base = (void __iomem *)IO_ADDRESS(REG_BASE_TIMER0);

	if (!(g_usavedcfg&DEBG_SUSPEND_TIMER_EN))
		return;

	/*clear timer intr */
	uregv = 1;
	writel(uregv, (utimer0_base+TIMER_INTCLR));

	/*disable timer*/
	uregv = 0;
	writel(uregv, (utimer0_base+TIMER_CTRL));

	/*reset timer0*/
	writel(IP_RST_TIMER0, (usctrl_base + 0x80));

	/*disable pclk_timer0, clk_timer0*/
	uregv = GT_PCLK_TIMER0|GT_CLK_TIMER0;
	writel(uregv, (usctrl_base+0x24));

	uregv  = readl(usctrl_base);
	/*printk("%s, %d, sysctrl=%x\r\n", __func__, __LINE__, uregv);*/

	/*timer0_0 select 32.768KHz.*/
	uregv = readl(usctrl_base);
	uregv &= ~(SCCTRL_TIMEREN0SEL|SCCTRL_TIMEREN0OV);
	uregv |= SCCTRL_TIMEFORCEHIGH;
	writel(uregv, usctrl_base);

	uregv  = readl(usctrl_base);
	/*printk("%s, %d, sysctrl=%x\r\n", __func__, __LINE__, uregv);*/

	/*enable pclk_timer0, clk_timer0*/
	uregv = GT_PCLK_TIMER0|GT_CLK_TIMER0;
	writel(uregv, (usctrl_base+0x20));

	/*undo reset*/
	writel(IP_RST_TIMER0, (usctrl_base + 0x84));

	/*read timer value*/
	uregv = readl((utimer0_base + TIMER_VALUE));
	/*printk("%s, %d, timerv=%d\r\n", __func__, __LINE__, uregv);*/

	/*set load. */
	uregv = 32 * g_utimer_inms;
	writel(uregv, utimer0_base);
	writel(uregv, utimer0_base+TIMER_BGLOAD);

	uregv = readl((utimer0_base + TIMER_VALUE));
	/*printk("%s, %d, timerv=%d\r\n", __func__, __LINE__, uregv);*/

	/*bit 7:timer enable
	*bit 6: timer mode period
	*bit 1:timer size 32bit */
	uregv = TIMEREN_BIT|TIMERMODE_BIT|TIMERSIZE_BIT|TIMEINT_BIT;
	writel(uregv, (utimer0_base+TIMER_CTRL));

	uregv = readl((utimer0_base + TIMER_VALUE));
	/*printk("%s, %d, timerv=%d\r\n", __func__, __LINE__, uregv);*/

}

void timer0_0_disable(void)
{
	unsigned uregv = 0;

	void __iomem * utimer0_base = (void __iomem *)IO_ADDRESS(REG_BASE_TIMER0);
	void __iomem * usctrl_base = (void __iomem *)IO_ADDRESS(REG_BASE_SCTRL);

	if (!(g_usavedcfg&DEBG_SUSPEND_TIMER_EN))
		return;

	uregv = readl((utimer0_base + 0x4));
	/*printk("%s, %d, timerv=%d\r\n", __func__, __LINE__, uregv);*/

	/*clear timer intr */
	uregv = 1;
	writel(uregv, (utimer0_base+0xc));

	uregv  = readl(usctrl_base);
	/*printk("%s, %d, sysctrl=%x\r\n", __func__, __LINE__, uregv);*/

	uregv = readl(usctrl_base);
	uregv &= ~(SCCTRL_TIMEREN0OV|SCCTRL_TIMEFORCEHIGH);
	uregv |= SCCTRL_TIMEREN0SEL;
	writel(uregv, usctrl_base);

	uregv  = readl(usctrl_base);
	/*printk("%s, %d, sysctrl=%x\r\n", __func__, __LINE__, uregv);*/

#if 0
	/*disable pclk_timer0, clk_timer0*/
	uregv = GT_PCLK_TIMER0|GT_CLK_TIMER0;
	writel(uregv, (usctrl_base+0x24));
#endif
}

void set_wakelock(int iflag)
{
	if ((1 == iflag) && (0 == wake_lock_active(&lowpm_wake_lock)))
		wake_lock(&lowpm_wake_lock);
	else if ((0 == iflag) && (0 != wake_lock_active(&lowpm_wake_lock)))
		wake_unlock(&lowpm_wake_lock);
}

#ifdef CONFIG_DEBUG_FS

#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include <linux/uaccess.h>

#define MX_BUF_LEN		1024
char g_ctemp[MX_BUF_LEN] = {0};

/*****************************************************************
* function:    dbg_cfg_open
* description:
*  adapt to the interface.
******************************************************************/
static int dbg_cfg_open(struct inode *inode, struct file *filp)
{
	filp->private_data = inode->i_private;
	return 0;
}

/*****************************************************************
* function:    dbg_cfg_read
* description:
*  show the debug cfg for user.
******************************************************************/
static ssize_t dbg_cfg_read(struct file *filp, char __user *buffer,
	size_t count, loff_t *ppos)
{
	if (*ppos >= MX_BUF_LEN)
		return 0;

	if (*ppos + count > MX_BUF_LEN)
		count = MX_BUF_LEN - *ppos;

	memset(g_ctemp, 0, MX_BUF_LEN);

	sprintf(g_ctemp,
		"0x1<<0 enable suspend console\n"
		"0x1<<1 ENABLE IO STATUS SHOW\n"
		"0x1<<2 ENABLE PMU STATUS SHOW\n"
		"0x1<<3 ENABLE IO SET\n"
		"0x1<<4 ENABLE PMU SET\n"
		"0x1<<5 ENABLE SINGLE IO SET\n"
		"0x1<<6 ENABLE 1s RTC wakeup\n"
		"0x1<<7 ENABLE 500ms TIMER wakeup\n"
		"0x1<<8 ENABLE a wakelock\n"
		"g_usavedcfg=%x\n", g_usavedcfg);

	if (copy_to_user(buffer, g_ctemp + *ppos, count))
		return -EFAULT;

	*ppos += count;
	return count;
}

/*****************************************************************
* function:    dbg_cfg_write
* description:
*  recieve the configuer of the user.
******************************************************************/
static ssize_t dbg_cfg_write(struct file *filp, const char __user *buffer,
	size_t count, loff_t *ppos)
{
	int index = 0;

	memset(g_ctemp, 0, MX_BUF_LEN);

	if (copy_from_user(g_ctemp, buffer, count)) {
		pr_info("error!\n");
		return -EFAULT;
	}

	if (sscanf(g_ctemp, "%d", &index))
		g_usavedcfg = index;
	else
		pr_info("ERRR~\n");

	pr_info("%s %d, g_usavedcfg=%x\n", __func__, __LINE__, g_usavedcfg);

	/*suspend print enable*/
	if (DEBG_SUSPEND_PRINTK & g_usavedcfg)
		console_suspend_enabled = 0;
	else
		console_suspend_enabled = 1;

	if (DEBG_SUSPEND_WAKELOCK & g_usavedcfg)
		set_wakelock(1);
	else
		set_wakelock(0);

	*ppos += count;

	return count;
}

const struct file_operations dbg_cfg_fops = {
	.owner	= THIS_MODULE,
	.open	= dbg_cfg_open,
	.read	= dbg_cfg_read,
	.write	= dbg_cfg_write,
};

/*****************************************************************
* function:    dbg_timer_open
* description:
*  adapt to the interface.
******************************************************************/
static int dbg_timer_open(struct inode *inode, struct file *filp)
{
	filp->private_data = inode->i_private;
	return 0;
}

/*****************************************************************
* function:    dbg_timer_read
* description:
*  show the debug cfg for user.
******************************************************************/
static ssize_t dbg_timer_read(struct file *filp, char __user *buffer,
	size_t count, loff_t *ppos)
{
	if (*ppos >= MX_BUF_LEN)
		return 0;

	if (*ppos + count > MX_BUF_LEN)
		count = MX_BUF_LEN - *ppos;

	memset(g_ctemp, 0, MX_BUF_LEN);

	sprintf(g_ctemp, "ENABLE %dms TIMER wakeup\n", g_utimer_inms);

	if (copy_to_user(buffer, g_ctemp + *ppos, count))
		return -EFAULT;

	*ppos += count;
	return count;
}

/*****************************************************************
* function:    dbg_timer_write
* description:
*  recieve the configuer of the user.
******************************************************************/
static ssize_t dbg_timer_write(struct file *filp, const char __user *buffer,
	size_t count, loff_t *ppos)
{
	int index = 0;

	memset(g_ctemp, 0, MX_BUF_LEN);

	if (copy_from_user(g_ctemp, buffer, count)) {
		pr_info("error!\n");
		return -EFAULT;
	}

	if (sscanf(g_ctemp, "%d", &index))
		g_utimer_inms = index;
	else
		pr_info("ERRR~\n");

	pr_info("%s %d, g_utimer_inms=%x\n", __func__, __LINE__, g_utimer_inms);

	*ppos += count;

	return count;
}

const struct file_operations dbg_timer_fops = {
	.owner	= THIS_MODULE,
	.open	= dbg_timer_open,
	.read	= dbg_timer_read,
	.write	= dbg_timer_write,
};


/*****************************************************************
* function:    dbg_timer_open
* description:
*  adapt to the interface.
******************************************************************/
static int dbg_rtc_open(struct inode *inode, struct file *filp)
{
	filp->private_data = inode->i_private;
	return 0;
}

/*****************************************************************
* function:    dbg_timer_read
* description:
*  show the debug cfg for user.
******************************************************************/
static ssize_t dbg_rtc_read(struct file *filp, char __user *buffer,
	size_t count, loff_t *ppos)
{
	if (*ppos >= MX_BUF_LEN)
		return 0;

	if (*ppos + count > MX_BUF_LEN)
		count = MX_BUF_LEN - *ppos;

	memset(g_ctemp, 0, MX_BUF_LEN);

	sprintf(g_ctemp, "ENABLE %dms rtc wakeup\n", g_urtc_ins);

	if (copy_to_user(buffer, g_ctemp + *ppos, count))
		return -EFAULT;

	*ppos += count;
	return count;
}

/*****************************************************************
* function:    dbg_timer_write
* description:
*  recieve the configuer of the user.
******************************************************************/
static ssize_t dbg_rtc_write(struct file *filp, const char __user *buffer,
	size_t count, loff_t *ppos)
{
	int index = 0;

	memset(g_ctemp, 0, MX_BUF_LEN);

	if (copy_from_user(g_ctemp, buffer, count)) {
		pr_info("error!\n");
		return -EFAULT;
	}

	if (sscanf(g_ctemp, "%d", &index))
		g_urtc_ins = index;
	else
		pr_info("ERRR~\n");

	pr_info("%s %d, g_urtc_ins=%x\n", __func__, __LINE__, g_urtc_ins);

	*ppos += count;

	return count;
}

const struct file_operations dbg_rtc_fops = {
	.owner	= THIS_MODULE,
	.open	= dbg_rtc_open,
	.read	= dbg_rtc_read,
	.write	= dbg_rtc_write,
};

/*****************************************************************
* function:    dbg_iomux_open
* description:
*  adapt to the interface.
******************************************************************/
static int dbg_iomux_open(struct inode *inode, struct file *filp)
{
	filp->private_data = inode->i_private;
	return 0;
}

/*****************************************************************
* function:    dbg_iomux_read
* description:
*  print out he io status on the COM.
******************************************************************/
static ssize_t dbg_iomux_read(struct file *filp, char __user *buffer,
	size_t count, loff_t *ppos)
{
	char temp[32] = {0};

	if (*ppos >= 32)
		return 0;

	if (*ppos + count > 32)
		count = 32 - *ppos;

	if (copy_to_user(buffer, temp + *ppos, count))
		return -EFAULT;

	ioshowstatus(1);

	*ppos += count;
	return count;
}

/*****************************************************************
* function:    dbg_iomux_write
* description:
*  recieve the configuer of the user.
******************************************************************/
static ssize_t dbg_iomux_write(struct file *filp, const char __user *buffer,
	size_t count, loff_t *ppos)
{
	char tmp[128] = {0};
	int index = 0;

    if (count > 128) {
		pr_info("error! buffer size big than internal buffer\n");
		return -EFAULT;
    }

	if (copy_from_user(tmp, buffer, count)) {
		pr_info("error!\n");
		return -EFAULT;
	}

	if (sscanf(tmp, "%d", &index)) {
		pr_info("%s, %d, gpio=%d\n", __func__, __LINE__, index);
		setiomuxvalue(index);
	} else {
		pr_info("ERRR~\n");
	}

	*ppos += count;

	return count;
}

const struct file_operations dbg_iomux_fops = {
	.owner	= THIS_MODULE,
	.open	= dbg_iomux_open,
	.read	= dbg_iomux_read,
	.write	= dbg_iomux_write,
};


/*****************************************************************
* function:    dbg_pmu_show
* description:
*  show the pmu status.
******************************************************************/
static int dbg_pmu_show(struct seq_file *s, void *unused)
{
	pmulowpower_show(1);

	return 0;
}

/*****************************************************************
* function:    dbg_pmu_open
* description:
*  adapt to the interface.
******************************************************************/
static int dbg_pmu_open(struct inode *inode, struct file *file)
{
	return single_open(file, dbg_pmu_show, &inode->i_private);
}

static const struct file_operations debug_pmu_fops = {
	.open		= dbg_pmu_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

/**********************cpufreq adjust begin*****************************/

#ifdef CONFIG_IPPS_SUPPORT

struct ipps_param gdbgipps_param;

/****************cpu begin**********************/

/*****************************************************************
* function:    dbg_cpu_max
* description:
*  adapt to the interface.
******************************************************************/
static int dbg_cpumax_open(struct inode *inode, struct file *filp)
{
	filp->private_data = inode->i_private;
	return 0;
}

/*****************************************************************
* function:    dbg_iomux_read
* description:
*  print out he io status on the COM.
******************************************************************/
static ssize_t dbg_cpumax_read(struct file *filp, char __user *buffer,
	size_t count, loff_t *ppos)
{
	if (*ppos >= MX_BUF_LEN)
		return 0;

	if (*ppos + count > MX_BUF_LEN)
		count = MX_BUF_LEN - *ppos;

	memset(g_ctemp, 0, MX_BUF_LEN);

	sprintf(g_ctemp, "cpumax=%d\n", gdbgipps_param.cpu.max_freq);

	if (copy_to_user(buffer, g_ctemp + *ppos, count))
		return -EFAULT;

	*ppos += count;
	return count;
}

/*****************************************************************
* function:    dbg_
* description:
*  recieve the configuer of the user.
******************************************************************/
static ssize_t dbg_cpumax_write(struct file *filp, const char __user *buffer,
	size_t count, loff_t *ppos)
{
	char tmp[128] = {0};
	int index = 0;

	if (count > 128) {
		pr_info("error! buffer size big than internal buffer\n");
		return -EFAULT;
	}

	if (copy_from_user(tmp, buffer, count)) {
		pr_info("error!\n");
		return -EFAULT;
	}

	if (sscanf(tmp, "%d", &index)) {
		//pr_info("%s, %d, cpumax=%d\n", __func__, __LINE__, index);
		gdbgipps_param.cpu.max_freq = index;
		ipps_set_parameter(&ipps_client, IPPS_OBJ_CPU, &gdbgipps_param);
	} else {
		pr_info("ERRR~\n");
	}

	*ppos += count;

	return count;
}

const struct file_operations dbg_cpumax_fops = {
	.owner	= THIS_MODULE,
	.open	= dbg_cpumax_open,
	.read	= dbg_cpumax_read,
	.write	= dbg_cpumax_write,
};

/*****************************************************************
* function:    dbg_cpumin
* description:
*  adapt to the interface.
******************************************************************/
static int dbg_cpumin_open(struct inode *inode, struct file *filp)
{
	filp->private_data = inode->i_private;
	return 0;
}

/*****************************************************************
* function:    dbg_cpumin_read
* description:
*  print out he io status on the COM.
******************************************************************/
static ssize_t dbg_cpumin_read(struct file *filp, char __user *buffer,
	size_t count, loff_t *ppos)
{
	if (*ppos >= MX_BUF_LEN)
		return 0;

	if (*ppos + count > MX_BUF_LEN)
		count = MX_BUF_LEN - *ppos;

	memset(g_ctemp, 0, MX_BUF_LEN);

	sprintf(g_ctemp, "cpumin=%d\n", gdbgipps_param.cpu.min_freq);

	if (copy_to_user(buffer, g_ctemp + *ppos, count))
		return -EFAULT;

	*ppos += count;
	return count;
}

/*****************************************************************
* function:    dbg_cpumin_write
* description:
*  recieve the configuer of the user.
******************************************************************/
static ssize_t dbg_cpumin_write(struct file *filp, const char __user *buffer,
	size_t count, loff_t *ppos)
{
	char tmp[128] = {0};
	int index = 0;

	if (count > 128) {
		pr_info("error! buffer size big than internal buffer\n");
		return -EFAULT;
	}

	if (copy_from_user(tmp, buffer, count)) {
		pr_info("error!\n");
		return -EFAULT;
	}

	if (sscanf(tmp, "%d", &index)) {
		//pr_info("%s, %d, cpumin=%d\n", __func__, __LINE__, index);
		gdbgipps_param.cpu.min_freq = index;
		ipps_set_parameter(&ipps_client, IPPS_OBJ_CPU, &gdbgipps_param);
	} else {
		pr_info("ERRR~\n");
	}

	*ppos += count;

	return count;
}

const struct file_operations dbg_cpumin_fops = {
	.owner	= THIS_MODULE,
	.open	= dbg_cpumin_open,
	.read	= dbg_cpumin_read,
	.write	= dbg_cpumin_write,
};

/*****************************************************************
* function:    dbg_cpusafe
* description:
*  adapt to the interface.
******************************************************************/
static int dbg_cpusafe_open(struct inode *inode, struct file *filp)
{
	filp->private_data = inode->i_private;
	return 0;
}

/*****************************************************************
* function:    dbg_cpumin_read
* description:
*  print out he io status on the COM.
******************************************************************/
static ssize_t dbg_cpusafe_read(struct file *filp, char __user *buffer,
	size_t count, loff_t *ppos)
{
	if (*ppos >= MX_BUF_LEN)
		return 0;

	if (*ppos + count > MX_BUF_LEN)
		count = MX_BUF_LEN - *ppos;

	memset(g_ctemp, 0, MX_BUF_LEN);

	sprintf(g_ctemp, "cpusafe=%d\n", gdbgipps_param.cpu.safe_freq);

	if (copy_to_user(buffer, g_ctemp + *ppos, count))
		return -EFAULT;

	*ppos += count;
	return count;
}

/*****************************************************************
* function:    dbg_cpumin_write
* description:
*  recieve the configuer of the user.
******************************************************************/
static ssize_t dbg_cpusafe_write(struct file *filp, const char __user *buffer,
	size_t count, loff_t *ppos)
{
	char tmp[128] = {0};
	int index = 0;

	if (count > 128) {
		pr_info("error! buffer size big than internal buffer\n");
		return -EFAULT;
	}

	if (copy_from_user(tmp, buffer, count)) {
		pr_info("error!\n");
		return -EFAULT;
	}

	if (sscanf(tmp, "%d", &index)) {
		//pr_info("%s, %d, cpusafe=%d\n", __func__, __LINE__, index);
		gdbgipps_param.cpu.safe_freq = index;
		ipps_set_parameter(&ipps_client, IPPS_OBJ_CPU, &gdbgipps_param);
	} else {
		pr_info("ERRR~\n");
	}

	*ppos += count;

	return count;
}

const struct file_operations dbg_cpusafe_fops = {
	.owner	= THIS_MODULE,
	.open	= dbg_cpusafe_open,
	.read	= dbg_cpusafe_read,
	.write	= dbg_cpusafe_write,
};
/****************cpu end**********************/

/****************gpu begin**********************/
/*****************************************************************
* function:    dbg_gpumax
* description:
*  adapt to the interface.
******************************************************************/
static int dbg_gpumax_open(struct inode *inode, struct file *filp)
{
	filp->private_data = inode->i_private;
	return 0;
}

/*****************************************************************
* function:    dbg_iomux_read
* description:
*  print out he io status on the COM.
******************************************************************/
static ssize_t dbg_gpumax_read(struct file *filp, char __user *buffer,
	size_t count, loff_t *ppos)
{
	if (*ppos >= MX_BUF_LEN)
		return 0;

	if (*ppos + count > MX_BUF_LEN)
		count = MX_BUF_LEN - *ppos;

	memset(g_ctemp, 0, MX_BUF_LEN);

	sprintf(g_ctemp, "gpumax=%d\n", gdbgipps_param.gpu.max_freq);

	if (copy_to_user(buffer, g_ctemp + *ppos, count))
		return -EFAULT;

	*ppos += count;
	return count;
}

/*****************************************************************
* function:    dbg_gpumax
* description:
*  recieve the configuer of the user.
******************************************************************/
static ssize_t dbg_gpumax_write(struct file *filp, const char __user *buffer,
	size_t count, loff_t *ppos)
{
	char tmp[128] = {0};
	int index = 0;

	if (count > 128) {
		pr_info("error! buffer size big than internal buffer\n");
		return -EFAULT;
	}

	if (copy_from_user(tmp, buffer, count)) {
		pr_info("error!\n");
		return -EFAULT;
	}

	if (sscanf(tmp, "%d", &index)) {
		//pr_info("%s, %d, gpumax=%d\n", __func__, __LINE__, index);
		gdbgipps_param.gpu.max_freq = index;
		ipps_set_parameter(&ipps_client, IPPS_OBJ_GPU, &gdbgipps_param);
	} else {
		pr_info("ERRR~\n");
	}

	*ppos += count;

	return count;
}

const struct file_operations dbg_gpumax_fops = {
	.owner	= THIS_MODULE,
	.open	= dbg_gpumax_open,
	.read	= dbg_gpumax_read,
	.write	= dbg_gpumax_write,
};

/*****************************************************************
* function:    dbg_gpumin
* description:
*  adapt to the interface.
******************************************************************/
static int dbg_gpumin_open(struct inode *inode, struct file *filp)
{
	filp->private_data = inode->i_private;
	return 0;
}

/*****************************************************************
* function:    dbg_cpumin_read
* description:
*  print out he io status on the COM.
******************************************************************/
static ssize_t dbg_gpumin_read(struct file *filp, char __user *buffer,
	size_t count, loff_t *ppos)
{
	if (*ppos >= MX_BUF_LEN)
		return 0;

	if (*ppos + count > MX_BUF_LEN)
		count = MX_BUF_LEN - *ppos;

	memset(g_ctemp, 0, MX_BUF_LEN);

	sprintf(g_ctemp, "gpumin=%d\n", gdbgipps_param.gpu.min_freq);

	if (copy_to_user(buffer, g_ctemp + *ppos, count))
		return -EFAULT;

	*ppos += count;
	return count;
}

/*****************************************************************
* function:    dbg_cpumin_write
* description:
*  recieve the configuer of the user.
******************************************************************/
static ssize_t dbg_gpumin_write(struct file *filp, const char __user *buffer,
	size_t count, loff_t *ppos)
{
	char tmp[128] = {0};
	int index = 0;

	if (count > 128) {
		pr_info("error! buffer size big than internal buffer\n");
		return -EFAULT;
	}

	if (copy_from_user(tmp, buffer, count)) {
		pr_info("error!\n");
		return -EFAULT;
	}

	if (sscanf(tmp, "%d", &index)) {
		//pr_info("%s, %d, gpumin=%d\n", __func__, __LINE__, index);
		gdbgipps_param.gpu.min_freq = index;
		ipps_set_parameter(&ipps_client, IPPS_OBJ_GPU, &gdbgipps_param);
	} else {
		pr_info("ERRR~\n");
	}

	*ppos += count;

	return count;
}

const struct file_operations dbg_gpumin_fops = {
	.owner	= THIS_MODULE,
	.open	= dbg_gpumin_open,
	.read	= dbg_gpumin_read,
	.write	= dbg_gpumin_write,
};

/*****************************************************************
* function:    dbg_gpusafe
* description:
*  adapt to the interface.
******************************************************************/
static int dbg_gpusafe_open(struct inode *inode, struct file *filp)
{
	filp->private_data = inode->i_private;
	return 0;
}

/*****************************************************************
* function:    dbg_cpumin_read
* description:
*  print out he io status on the COM.
******************************************************************/
static ssize_t dbg_gpusafe_read(struct file *filp, char __user *buffer,
	size_t count, loff_t *ppos)
{
	if (*ppos >= MX_BUF_LEN)
		return 0;

	if (*ppos + count > MX_BUF_LEN)
		count = MX_BUF_LEN - *ppos;

	memset(g_ctemp, 0, MX_BUF_LEN);

	sprintf(g_ctemp, "gpusafe=%d\n", gdbgipps_param.gpu.safe_freq);

	if (copy_to_user(buffer, g_ctemp + *ppos, count))
		return -EFAULT;

	*ppos += count;
	return count;
}

/*****************************************************************
* function:    dbg_cpumin_write
* description:
*  recieve the configuer of the user.
******************************************************************/
static ssize_t dbg_gpusafe_write(struct file *filp, const char __user *buffer,
	size_t count, loff_t *ppos)
{
	char tmp[128] = {0};
	int index = 0;

	if (count > 128) {
		pr_info("error! buffer size big than internal buffer\n");
		return -EFAULT;
	}

	if (copy_from_user(tmp, buffer, count)) {
		pr_info("error!\n");
		return -EFAULT;
	}

	if (sscanf(tmp, "%d", &index)) {
		//pr_info("%s, %d, gpusafe=%d\n", __func__, __LINE__, index);
		gdbgipps_param.gpu.safe_freq = index;
		ipps_set_parameter(&ipps_client, IPPS_OBJ_GPU, &gdbgipps_param);
	} else {
		pr_info("ERRR~\n");
	}

	*ppos += count;

	return count;
}

const struct file_operations dbg_gpusafe_fops = {
	.owner	= THIS_MODULE,
	.open	= dbg_gpusafe_open,
	.read	= dbg_gpusafe_read,
	.write	= dbg_gpusafe_write,
};
/****************gpu end************************/

/****************ddr begin**********************/
/*****************************************************************
* function:    dbg_ddrmax
* description:
*  adapt to the interface.
******************************************************************/
static int dbg_ddrmax_open(struct inode *inode, struct file *filp)
{
	filp->private_data = inode->i_private;
	return 0;
}

/*****************************************************************
* function:    dbg_iomux_read
* description:
*  print out he io status on the COM.
******************************************************************/
static ssize_t dbg_ddrmax_read(struct file *filp, char __user *buffer,
	size_t count, loff_t *ppos)
{
	if (*ppos >= MX_BUF_LEN)
		return 0;

	if (*ppos + count > MX_BUF_LEN)
		count = MX_BUF_LEN - *ppos;

	memset(g_ctemp, 0, MX_BUF_LEN);

	sprintf(g_ctemp, "ddrmax=%d\n", gdbgipps_param.ddr.max_freq);

	if (copy_to_user(buffer, g_ctemp + *ppos, count))
		return -EFAULT;

	*ppos += count;
	return count;
}

/*****************************************************************
* function:    dbg_gpumax
* description:
*  recieve the configuer of the user.
******************************************************************/
static ssize_t dbg_ddrmax_write(struct file *filp, const char __user *buffer,
	size_t count, loff_t *ppos)
{
	char tmp[128] = {0};
	int index = 0;

	if (count > 128) {
		pr_info("error! buffer size big than internal buffer\n");
		return -EFAULT;
	}

	if (copy_from_user(tmp, buffer, count)) {
		pr_info("error!\n");
		return -EFAULT;
	}

	if (sscanf(tmp, "%d", &index)) {
		//pr_info("%s, %d, ddrmax=%d\n", __func__, __LINE__, index);
		gdbgipps_param.ddr.max_freq = index;
		ipps_set_parameter(&ipps_client, IPPS_OBJ_DDR, &gdbgipps_param);
	} else {
		pr_info("ERRR~\n");
	}

	*ppos += count;

	return count;
}

const struct file_operations dbg_ddrmax_fops = {
	.owner	= THIS_MODULE,
	.open	= dbg_ddrmax_open,
	.read	= dbg_ddrmax_read,
	.write	= dbg_ddrmax_write,
};

/*****************************************************************
* function:    dbg_ddrmin
* description:
*  adapt to the interface.
******************************************************************/
static int dbg_ddrmin_open(struct inode *inode, struct file *filp)
{
	filp->private_data = inode->i_private;
	return 0;
}

/*****************************************************************
* function:    dbg_cpumin_read
* description:
*  print out he io status on the COM.
******************************************************************/
static ssize_t dbg_ddrmin_read(struct file *filp, char __user *buffer,
	size_t count, loff_t *ppos)
{
	if (*ppos >= MX_BUF_LEN)
		return 0;

	if (*ppos + count > MX_BUF_LEN)
		count = MX_BUF_LEN - *ppos;

	memset(g_ctemp, 0, MX_BUF_LEN);

	sprintf(g_ctemp, "ddrmin=%d\n", gdbgipps_param.ddr.min_freq);

	if (copy_to_user(buffer, g_ctemp + *ppos, count))
		return -EFAULT;

	*ppos += count;
	return count;
}

/*****************************************************************
* function:    dbg_cpumin_write
* description:
*  recieve the configuer of the user.
******************************************************************/
static ssize_t dbg_ddrmin_write(struct file *filp, const char __user *buffer,
	size_t count, loff_t *ppos)
{
	char tmp[128] = {0};
	int index = 0;

	if (count > 128) {
		pr_info("error! buffer size big than internal buffer\n");
		return -EFAULT;
	}

	if (copy_from_user(tmp, buffer, count)) {
		pr_info("error!\n");
		return -EFAULT;
	}

	if (sscanf(tmp, "%d", &index)) {
		//pr_info("%s, %d, ddrmin=%d\n", __func__, __LINE__, index);
		gdbgipps_param.ddr.min_freq = index;
		ipps_set_parameter(&ipps_client, IPPS_OBJ_DDR, &gdbgipps_param);
	} else {
		pr_info("ERRR~\n");
	}

	*ppos += count;

	return count;
}

const struct file_operations dbg_ddrmin_fops = {
	.owner	= THIS_MODULE,
	.open	= dbg_ddrmin_open,
	.read	= dbg_ddrmin_read,
	.write	= dbg_ddrmin_write,
};

/*****************************************************************
* function:    dbg_ddrsafe
* description:
*  adapt to the interface.
******************************************************************/
static int dbg_ddrsafe_open(struct inode *inode, struct file *filp)
{
	filp->private_data = inode->i_private;
	return 0;
}

/*****************************************************************
* function:    dbg_cpumin_read
* description:
*  print out he io status on the COM.
******************************************************************/
static ssize_t dbg_ddrsafe_read(struct file *filp, char __user *buffer,
	size_t count, loff_t *ppos)
{
	if (*ppos >= MX_BUF_LEN)
		return 0;

	if (*ppos + count > MX_BUF_LEN)
		count = MX_BUF_LEN - *ppos;

	memset(g_ctemp, 0, MX_BUF_LEN);

	sprintf(g_ctemp, "ddrsafe=%d\n", gdbgipps_param.ddr.safe_freq);

	if (copy_to_user(buffer, g_ctemp + *ppos, count))
		return -EFAULT;

	*ppos += count;
	return count;
}

/*****************************************************************
* function:    dbg_cpumin_write
* description:
*  recieve the configuer of the user.
******************************************************************/
static ssize_t dbg_ddrsafe_write(struct file *filp, const char __user *buffer,
	size_t count, loff_t *ppos)
{
	char tmp[128] = {0};
	int index = 0;

	if (count > 128) {
		pr_info("error! buffer size big than internal buffer\n");
		return -EFAULT;
	}

	if (copy_from_user(tmp, buffer, count)) {
		pr_info("error!\n");
		return -EFAULT;
	}

	if (sscanf(tmp, "%d", &index)) {
		//pr_info("%s, %d, ddrsafe=%d\n", __func__, __LINE__, index);
		gdbgipps_param.ddr.safe_freq = index;
		ipps_set_parameter(&ipps_client, IPPS_OBJ_DDR, &gdbgipps_param);
	} else {
		pr_info("ERRR~\n");
	}

	*ppos += count;

	return count;
}

const struct file_operations dbg_ddrsafe_fops = {
	.owner	= THIS_MODULE,
	.open	= dbg_ddrsafe_open,
	.read	= dbg_ddrsafe_read,
	.write	= dbg_ddrsafe_write,
};

/****************ddr end************************/

#endif

/**********************cpufreq adjust end****************************/

/*****************************************************************
* function:    lowpm_test_probe
* description:
*  driver interface.
******************************************************************/
static int lowpm_test_probe(struct platform_device *pdev)
{
	int status = 0;
	struct dentry *pdentry;

	pr_info("[%s] %d enter.\n", __func__, __LINE__);

	g_ioc_addr = (void __iomem *) IO_ADDRESS(REG_BASE_IOC);
	g_gpio_addr = (void __iomem *) IO_ADDRESS(REG_BASE_GPIO0);
	g_sc_addr = (void __iomem *) IO_ADDRESS(REG_BASE_SCTRL);
	g_pctrl_addr = (void __iomem *) IO_ADDRESS(REG_BASE_PCTRL);
	g_pmuspi_addr = (void __iomem *) IO_ADDRESS(REG_BASE_PMUSPI);

	g_suspended = 0;

	/*default timer0 wakeup time 500ms*/
	g_utimer_inms = 200;

	/*default rtc wakeup time in 1s*/
	g_urtc_ins = 1;

	wake_lock_init(&lowpm_wake_lock, WAKE_LOCK_SUSPEND, "lowpm_test");

	pdentry = debugfs_create_dir("lowpm_test", NULL);
	if (!pdentry) {
		pr_info("%s %d error can not create debugfs lowpm_test.\n", __func__, __LINE__);
		return -ENOMEM;
	}

	(void) debugfs_create_file("pmu", S_IRUSR, pdentry, NULL, &debug_pmu_fops);

	(void) debugfs_create_file("io", S_IRUSR, pdentry, NULL, &dbg_iomux_fops);

	(void) debugfs_create_file("cfg", S_IRUSR, pdentry, NULL, &dbg_cfg_fops);

	(void) debugfs_create_file("timer", S_IRUSR, pdentry, NULL, &dbg_timer_fops);

	(void) debugfs_create_file("rtc", S_IRUSR, pdentry, NULL, &dbg_rtc_fops);

#ifdef CONFIG_IPPS_SUPPORT
	(void) debugfs_create_file("cpumax", S_IRUSR, pdentry, NULL, &dbg_cpumax_fops);
	(void) debugfs_create_file("cpumin", S_IRUSR, pdentry, NULL, &dbg_cpumin_fops);
	(void) debugfs_create_file("cpusafe", S_IRUSR, pdentry, NULL, &dbg_cpusafe_fops);

	(void) debugfs_create_file("gpumax", S_IRUSR, pdentry, NULL, &dbg_gpumax_fops);
	(void) debugfs_create_file("gpumin", S_IRUSR, pdentry, NULL, &dbg_gpumin_fops);
	(void) debugfs_create_file("gpusafe", S_IRUSR, pdentry, NULL, &dbg_gpusafe_fops);

	(void) debugfs_create_file("ddrmax", S_IRUSR, pdentry, NULL, &dbg_ddrmax_fops);
	(void) debugfs_create_file("ddrmin", S_IRUSR, pdentry, NULL, &dbg_ddrmin_fops);
	(void) debugfs_create_file("ddrsafe", S_IRUSR, pdentry, NULL, &dbg_ddrsafe_fops);
#endif

	pr_info("[%s] %d leave.\n", __func__, __LINE__);

	return status;
}

/*****************************************************************
* function:    lowpm_test_remove
* description:
*  driver interface.
******************************************************************/
static int lowpm_test_remove(struct platform_device *pdev)
{
	return 0;
}

#ifdef CONFIG_PM

static int lowpm_test_suspend(struct platform_device *pdev,
	pm_message_t state)
{
	g_suspended = 1;
	return 0;
}

static int lowpm_test_resume(struct platform_device *pdev)
{
	g_suspended = 0;
	return 0;
}
#else
#define lowpm_test_suspend	NULL
#define lowpm_test_resume	NULL
#endif

#define MODULE_NAME		"lowpm_test"

static struct platform_driver lowpm_test_drv = {
	.probe		= lowpm_test_probe,
	.remove		= __devexit_p(lowpm_test_remove),
	.suspend	= lowpm_test_suspend,
	.resume		= lowpm_test_resume,
	.driver = {
		.name	= MODULE_NAME,
		.owner	= THIS_MODULE,
	},
};

static struct platform_device lowpm_test_device = {
	.id		= 0,
	.name	= MODULE_NAME,
};

#ifdef CONFIG_IPPS_SUPPORT
static void ippsclient_add(struct ipps_device *device)
{
}

static void ippsclient_remove(struct ipps_device *device)
{
}

static struct ipps_client ipps_client = {
	.name   = "lowpmreg",
	.add    = ippsclient_add,
	.remove = ippsclient_remove
};
#endif

static int __init lowpmreg_init(void)
{
	int ret = 0;
	pr_info("[%s] %d enter.\n", __func__, __LINE__);

	ret = platform_driver_register(&lowpm_test_drv);
	if (0 != ret)
		pr_info("%s, %d, err=%x\n", __func__, __LINE__, ret);

	ret = platform_device_register(&lowpm_test_device);
	if (0 != ret)
		pr_info("%s, %d, err=%x\n", __func__, __LINE__, ret);

#ifdef CONFIG_IPPS_SUPPORT
	ret = ipps_register_client(&ipps_client);
#endif

	pr_info("[%s] %d leave.\n", __func__, __LINE__);

	return ret;
}

static void __exit lowpmreg_exit(void)
{
	pr_info("%s %d enter.\n", __func__, __LINE__);

	platform_driver_unregister(&lowpm_test_drv);

	platform_device_unregister(&lowpm_test_device);

#ifdef CONFIG_IPPS_SUPPORT
	ipps_unregister_client(&ipps_client);
#endif

	pr_info("%s %d leave.\n", __func__, __LINE__);
}
#endif /*CONFIG_DEBUG_FS*/

#else

static int __init lowpmreg_init(void)
{
	g_ioc_addr		= (void __iomem *) IO_ADDRESS(REG_BASE_IOC);
	g_gpio_addr		= (void __iomem *) IO_ADDRESS(REG_BASE_GPIO0);
	g_sc_addr		= (void __iomem *) IO_ADDRESS(REG_BASE_SCTRL);
	g_pctrl_addr	= (void __iomem *) IO_ADDRESS(REG_BASE_PCTRL);
	g_pmuspi_addr	= (void __iomem *) IO_ADDRESS(REG_BASE_PMUSPI);
	return 0;
}

static void __exit lowpmreg_exit(void)
{
}

#endif

module_init(lowpmreg_init);
module_exit(lowpmreg_exit);

MODULE_LICENSE("GPL");
