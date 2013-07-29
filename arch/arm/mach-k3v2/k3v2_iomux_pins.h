#ifndef __MACH_K3V2_IOMUX_PINS_H
#define __MACH_K3V2_IOMUX_PINS_H
#include <mach/platform.h>
#include <mach/io.h>
#include <linux/mux.h>
#include "iomux.h"

/*board id can not be used precompile*/
#define V110_VERSION

extern struct iomux_ops iomux_pin_ops;

#define IOMUX_IOMG(_iomg, _iomg_name, _iomg_reg, _func_array)\
struct iomux_iomg _iomg = {\
	.name = _iomg_name,\
	.iomg_reg = (void __iomem	*)(IO_ADDRESS(REG_BASE_IOC) + _iomg_reg),\
	.regValue = _func_array,\
};

#define IOMUX_IOCG(_iocg, _iocg_name, _iocg_reg, _iocg_pud_mask, _iocg_drvstrength_mask)    \
struct iomux_iocg _iocg = {\
	.name = _iocg_name,\
	.iocg_reg = (void __iomem	*)(IO_ADDRESS(REG_BASE_IOC) + _iocg_reg),\
	.iocg_pullupdown_mask = _iocg_pud_mask,\
	.iocg_drivestrength_mask = _iocg_drvstrength_mask,\
};

#define IOMUX_PIN(_iomux_pin, _iomux_pin_name, _pin_func, _pin_pull_updown, \
_pin_drive_strength, _pin_iomg, _pin_iocg)    \
struct  iomux_pin _iomux_pin = {\
	.pin_name = _iomux_pin_name,\
	.pin_func  = _pin_func,\
	.pin_pull_updown = _pin_pull_updown,\
	.pin_drive_strength = _pin_drive_strength,\
	.ops  =  &iomux_pin_ops,\
	.pin_iomg = _pin_iomg,\
	.pin_iocg  = _pin_iocg,\
	.init = 0, \
};

#define PIN_TABLE(_pinname, _iomux_pin)	\
{\
	.pinname = _pinname,\
	.iomux_pin = _iomux_pin,\
}

/*define the iomg*/
int func_array1[] = {0, 1, RESERVE};
int func_array2[] = {0, 1, 2, RESERVE};
int func_array3[] = {0, 1, 2, 3, RESERVE};
int func_array4[] = {0, 1, 2, 3, 4, 5, 6, RESERVE};

IOMUX_IOMG(iomg0, "iomg0", 0x000, func_array1)
IOMUX_IOMG(iomg1, "iomg1", 0x004, func_array2)
IOMUX_IOMG(iomg2, "iomg2", 0x008, func_array1)
IOMUX_IOMG(iomg3, "iomg3", 0x00C, func_array2)
IOMUX_IOMG(iomg4, "iomg4", 0x010, func_array2)
IOMUX_IOMG(iomg5, "iomg5", 0x014, func_array2)
IOMUX_IOMG(iomg6, "iomg6", 0x018, func_array2)
IOMUX_IOMG(iomg7, "iomg7", 0x020, func_array3)
IOMUX_IOMG(iomg8, "iomg8", 0x024, func_array3)
IOMUX_IOMG(iomg9, "iomg9", 0x028, func_array3)
IOMUX_IOMG(iomg10, "iomg10", 0x02C, func_array3)
IOMUX_IOMG(iomg12, "iomg12", 0x030, func_array3)
IOMUX_IOMG(iomg13, "iomg13", 0x034, func_array3)
IOMUX_IOMG(iomg14, "iomg14", 0x038, func_array3)
IOMUX_IOMG(iomg15, "iomg15", 0x03C, func_array3)
IOMUX_IOMG(iomg16, "iomg16", 0x040, func_array4)
IOMUX_IOMG(iomg17, "iomg17", 0x044, func_array4)
IOMUX_IOMG(iomg18, "iomg18", 0x048, func_array4)
IOMUX_IOMG(iomg19, "iomg19", 0x04C, func_array4)
IOMUX_IOMG(iomg20, "iomg20", 0x050, func_array4)/*func2:i2c3_scl*/
IOMUX_IOMG(iomg21, "iomg21", 0x054, func_array2)/*func2:i2c3_sda*/
IOMUX_IOMG(iomg22, "iomg22", 0x058, func_array1)
IOMUX_IOMG(iomg23, "iomg23", 0x05C, func_array1)
IOMUX_IOMG(iomg24, "iomg24", 0x060, func_array3)
IOMUX_IOMG(iomg25, "iomg25", 0x064, func_array3)
IOMUX_IOMG(iomg26, "iomg26", 0x068, func_array2)/*func0:I2C2_SCL*/
IOMUX_IOMG(iomg27, "iomg27", 0x06C, func_array2)/*func0:I2C2_SDA*/
IOMUX_IOMG(iomg28, "iomg28", 0x070, func_array1)
IOMUX_IOMG(iomg29, "iomg29", 0x074, func_array1)
IOMUX_IOMG(iomg30, "iomg30", 0x078, func_array1)
IOMUX_IOMG(iomg31, "iomg31", 0x07C, func_array1)
IOMUX_IOMG(iomg32, "iomg32", 0x080, func_array1)
IOMUX_IOMG(iomg33, "iomg33", 0x084, func_array1)
IOMUX_IOMG(iomg34, "iomg34", 0x088, func_array1)
IOMUX_IOMG(iomg35, "iomg35", 0x08C, func_array2)/*func2:DSI0_TE0*/
IOMUX_IOMG(iomg36, "iomg36", 0x090, func_array2)/*func2:DSI1_TE0*/
IOMUX_IOMG(iomg37, "iomg37", 0x094, func_array1)
IOMUX_IOMG(iomg38, "iomg38", 0x098, func_array2)
IOMUX_IOMG(iomg39, "iomg39", 0x09C, func_array3)
IOMUX_IOMG(iomg40, "iomg40", 0x0A0, func_array2)
IOMUX_IOMG(iomg41, "iomg41", 0x0A4, func_array3)
IOMUX_IOMG(iomg42, "iomg42", 0x0A8, func_array1)
IOMUX_IOMG(iomg43, "iomg43", 0x0AC, func_array1)
IOMUX_IOMG(iomg44, "iomg44", 0x0B0, func_array1)
IOMUX_IOMG(iomg45, "iomg45", 0x0B4, func_array1)
IOMUX_IOMG(iomg46, "iomg46", 0x0B8, func_array1)
IOMUX_IOMG(iomg47, "iomg47", 0x0BC, func_array1)
IOMUX_IOMG(iomg48, "iomg48", 0x0C0, func_array1)
IOMUX_IOMG(iomg49, "iomg49", 0x0C4, func_array2)/*func2:hsi*/
IOMUX_IOMG(iomg50, "iomg50", 0x0C8, func_array2)/*func2:hsi*/
IOMUX_IOMG(iomg51, "iomg51", 0x0CC, func_array1)
IOMUX_IOMG(iomg52, "iomg52", 0x0D0, func_array1)
IOMUX_IOMG(iomg53, "iomg53", 0x0D4, func_array1)
IOMUX_IOMG(iomg54, "iomg54", 0x0D8, func_array1)
IOMUX_IOMG(iomg55, "iomg55", 0x0DC, func_array1)
IOMUX_IOMG(iomg56, "iomg56", 0x0E0, func_array1)
IOMUX_IOMG(iomg57, "iomg57", 0x0E4, func_array2)
IOMUX_IOMG(iomg58, "iomg58", 0x0E8, func_array1)
IOMUX_IOMG(iomg59, "iomg59", 0x0F0, func_array1)
IOMUX_IOMG(iomg60, "iomg60", 0x0F4, func_array1)
IOMUX_IOMG(iomg61, "iomg61", 0x0F8, func_array2)
IOMUX_IOMG(iomg62, "iomg62", 0x0FC, func_array2)
IOMUX_IOMG(iomg63, "iomg63", 0x100, func_array2)
IOMUX_IOMG(iomg64, "iomg64", 0x108, func_array2)
IOMUX_IOMG(iomg65, "iomg65", 0x10C, func_array3)
IOMUX_IOMG(iomg66, "iomg66", 0x110, func_array3)
IOMUX_IOMG(iomg67, "iomg67", 0x114, func_array3)
IOMUX_IOMG(iomg68, "iomg68", 0x118, func_array3)
IOMUX_IOMG(iomg69, "iomg69", 0x11C, func_array3)
IOMUX_IOMG(iomg70, "iomg70", 0x120, func_array3)
IOMUX_IOMG(iomg71, "iomg71", 0x124, func_array3)
IOMUX_IOMG(iomg72, "iomg72", 0x128, func_array3)
IOMUX_IOMG(iomg73, "iomg73", 0x12C, func_array3)
IOMUX_IOMG(iomg74, "iomg74", 0x130, func_array3)
IOMUX_IOMG(iomg75, "iomg75", 0x134, func_array3)
IOMUX_IOMG(iomg76, "iomg76", 0x138, func_array3)
IOMUX_IOMG(iomg77, "iomg77", 0x13C, func_array3)
IOMUX_IOMG(iomg78, "iomg78", 0x140, func_array3)
IOMUX_IOMG(iomg79, "iomg79", 0x144, func_array3)
IOMUX_IOMG(iomg80, "iomg80", 0x148, func_array3)
IOMUX_IOMG(iomg81, "iomg81", 0x14C, func_array3)
IOMUX_IOMG(iomg82, "iomg82", 0x154, func_array1)
IOMUX_IOMG(iomg83, "iomg83", 0x158, func_array1)
IOMUX_IOMG(iomg84, "iomg84", 0x15C, func_array1)
IOMUX_IOMG(iomg85, "iomg85", 0x160, func_array2)
IOMUX_IOMG(iomg86, "iomg86", 0x164, func_array2)
IOMUX_IOMG(iomg87, "iomg87", 0x168, func_array2)/*func0:uart4,func1:gpio*/
IOMUX_IOMG(iomg88, "iomg88", 0x16C, func_array2)/*func0:uart4,func1:gpio*/
/*there are no iomg89 and iomg91 in hi3620v110*/
IOMUX_IOMG(iomg89, "iomg89", 0x174, func_array1)
IOMUX_IOMG(iomg91, "iomg91", 0x17C, func_array1)
IOMUX_IOMG(iomg90, "iomg90", 0x178, func_array1)
IOMUX_IOMG(iomg92, "iomg92", 0x180, func_array1)
IOMUX_IOMG(iomg93, "iomg93", 0x170, func_array2)/*func0:uart4,func1:gpio*/
IOMUX_IOMG(iomg94, "iomg94", 0x01C, func_array3)
IOMUX_IOMG(iomg95, "iomg95", 0x0EC, func_array1)
IOMUX_IOMG(iomg96, "iomg96", 0x104, func_array2)
IOMUX_IOMG(iomg97, "iomg97", 0x150, func_array3)
IOMUX_IOMG(iomg98, "iomg98", 0x184, func_array2)/*func0:spi,func1:gpio,func2:hsi*/

/*define iocg*/
IOMUX_IOCG(iocg191, "iocg191", 0x800, 0x0000, 0x00F0)
IOMUX_IOCG(iocg192, "iocg192", 0x804, 0x0003, 0x00F0)
IOMUX_IOCG(iocg193, "iocg193", 0x808, 0x0000, 0x00F0)
IOMUX_IOCG(iocg0, "iocg0", 0x80C, 0x0003, 0x0000)
IOMUX_IOCG(iocg1, "iocg1", 0x810, 0x0003, 0x0000)
IOMUX_IOCG(iocg2, "iocg2", 0x814, 0x0003, 0x0000)
IOMUX_IOCG(iocg3, "iocg3", 0x818, 0x0003, 0x0000)
IOMUX_IOCG(iocg4, "iocg4", 0x81C, 0x0001, 0x0000)
IOMUX_IOCG(iocg5, "iocg5", 0x820, 0x0001, 0x0000)
IOMUX_IOCG(iocg6, "iocg6", 0x824, 0x0003, 0x0000)
IOMUX_IOCG(iocg7, "iocg7", 0x828, 0x0003, 0x0000)
IOMUX_IOCG(iocg8, "iocg8", 0x82C, 0x0003, 0x0000)
IOMUX_IOCG(iocg9, "iocg9", 0x830, 0x0003, 0x0000)
IOMUX_IOCG(iocg10, "iocg10", 0x834, 0x0003, 0x0000)
IOMUX_IOCG(iocg11, "iocg11", 0x838, 0x0003, 0x0000)
IOMUX_IOCG(iocg12, "iocg12", 0x83C, 0x0003, 0x00F0)
IOMUX_IOCG(iocg13, "iocg13", 0x840, 0x0003, 0x00F0)
IOMUX_IOCG(iocg14, "iocg14", 0x844, 0x0003, 0x00F0)
IOMUX_IOCG(iocg15, "iocg15", 0x848, 0x0003, 0x00F0)
IOMUX_IOCG(iocg16, "iocg16", 0x84C, 0x0003, 0x00F0)
IOMUX_IOCG(iocg17, "iocg17", 0x850, 0x0003, 0x00F0)
IOMUX_IOCG(iocg18, "iocg18", 0x854, 0x0003, 0x00F0)
IOMUX_IOCG(iocg19, "iocg19", 0x858, 0x0003, 0x00F0)
IOMUX_IOCG(iocg20, "iocg20", 0x85C, 0x0003, 0x0000)
IOMUX_IOCG(iocg21, "iocg21", 0x860, 0x0003, 0x0000)
IOMUX_IOCG(iocg22, "iocg22", 0x864, 0x0003, 0x0000)
IOMUX_IOCG(iocg23, "iocg23", 0x868, 0x0003, 0x0000)
IOMUX_IOCG(iocg24, "iocg24", 0x86C, 0x0003, 0x00F0)
IOMUX_IOCG(iocg25, "iocg25", 0x870, 0x0003, 0x00F0)
IOMUX_IOCG(iocg26, "iocg26", 0x874, 0x0003, 0x00F0)
IOMUX_IOCG(iocg27, "iocg27", 0x878, 0x0003, 0x00F0)
IOMUX_IOCG(iocg28, "iocg28", 0x87C, 0x0003, 0x00F0)
IOMUX_IOCG(iocg29, "iocg29", 0x880, 0x0003, 0x00F0)
IOMUX_IOCG(iocg30, "iocg30", 0x884, 0x0003, 0x00F0)
IOMUX_IOCG(iocg31, "iocg31", 0x888, 0x0003, 0x00F0)
IOMUX_IOCG(iocg32, "iocg32", 0x88C, 0x0003, 0x00F0)
IOMUX_IOCG(iocg33, "iocg33", 0x890, 0x0003, 0x00F0)
IOMUX_IOCG(iocg34, "iocg34", 0x894, 0x0003, 0x00F0)
IOMUX_IOCG(iocg35, "iocg35", 0x898, 0x0003, 0x00F0)
IOMUX_IOCG(iocg36, "iocg36", 0x89C, 0x0003, 0x00F0)
IOMUX_IOCG(iocg37, "iocg37", 0x8A0, 0x0003, 0x00F0)
IOMUX_IOCG(iocg38, "iocg38", 0x8A4, 0x0003, 0x00F0)
IOMUX_IOCG(iocg39, "iocg39", 0x8A8, 0x0003, 0x00F0)
IOMUX_IOCG(iocg40, "iocg40", 0x8AC, 0x0003, 0x00F0)
IOMUX_IOCG(iocg41, "iocg41", 0x8B0, 0x0003, 0x00F0)
IOMUX_IOCG(iocg54, "iocg54", 0x8B4, 0x0003, 0x0000)
IOMUX_IOCG(iocg55, "iocg55", 0x8B8, 0x0003, 0x0000)
IOMUX_IOCG(iocg56, "iocg56", 0x8BC, 0x0003, 0x0000)
IOMUX_IOCG(iocg57, "iocg57", 0x8C0, 0x0003, 0x0000)
IOMUX_IOCG(iocg58, "iocg58", 0x8C4, 0x0003, 0x0000)
IOMUX_IOCG(iocg59, "iocg59", 0x8C8, 0x0003, 0x0000)
IOMUX_IOCG(iocg60, "iocg60", 0x8CC, 0x0003, 0x0000)
IOMUX_IOCG(iocg61, "iocg61", 0x8D0, 0x0003, 0x0000)
IOMUX_IOCG(iocg62, "iocg62", 0x8D4, 0x0003, 0x0000)
IOMUX_IOCG(iocg63, "iocg63", 0x8D8, 0x0003, 0x0000)
IOMUX_IOCG(iocg64, "iocg64", 0x8DC, 0x0003, 0x0000)
IOMUX_IOCG(iocg65, "iocg65", 0x8E0, 0x0003, 0x0000)
IOMUX_IOCG(iocg66, "iocg66", 0x8E4, 0x0003, 0x0000)
IOMUX_IOCG(iocg67, "iocg67", 0x8E8, 0x0003, 0x0000)
IOMUX_IOCG(iocg68, "iocg68", 0x8EC, 0x0003, 0x0000)
IOMUX_IOCG(iocg69, "iocg69", 0x8F0, 0x0003, 0x0000)
IOMUX_IOCG(iocg70, "iocg70", 0x8F4, 0x0003, 0x0000)
IOMUX_IOCG(iocg71, "iocg71", 0x8F8, 0x0003, 0x0000)
IOMUX_IOCG(iocg72, "iocg72", 0x8FC, 0x0003, 0x0000)
IOMUX_IOCG(iocg73, "iocg73", 0x900, 0x0003, 0x0000)
IOMUX_IOCG(iocg74, "iocg74", 0x904, 0x0003, 0x0000)
IOMUX_IOCG(iocg75, "iocg75", 0x908, 0x0003, 0x0000)
IOMUX_IOCG(iocg76, "iocg76", 0x90C, 0x0003, 0x0000)
IOMUX_IOCG(iocg77, "iocg77", 0x910, 0x0003, 0x0000)
IOMUX_IOCG(iocg78, "iocg78", 0x914, 0x0003, 0x0000)
IOMUX_IOCG(iocg79, "iocg79", 0x918, 0x0003, 0x0000)
IOMUX_IOCG(iocg80, "iocg80", 0x91C, 0x0003, 0x0000)
IOMUX_IOCG(iocg81, "iocg81", 0x920, 0x0003, 0x0000)
IOMUX_IOCG(iocg82, "iocg82", 0x924, 0x0003, 0x0000)
IOMUX_IOCG(iocg83, "iocg83", 0x928, 0x0003, 0x00F0)
IOMUX_IOCG(iocg84, "iocg84", 0x92C, 0x0003, 0x00F0)
IOMUX_IOCG(iocg85, "iocg85", 0x930, 0x0003, 0x0000)
IOMUX_IOCG(iocg86, "iocg86", 0x934, 0x0003, 0x0000)
IOMUX_IOCG(iocg87, "iocg87", 0x938, 0x0003, 0x0000)
IOMUX_IOCG(iocg88, "iocg88", 0x93C, 0x0003, 0x0000)
IOMUX_IOCG(iocg89, "iocg89", 0x940, 0x0003, 0x0000)
IOMUX_IOCG(iocg90, "iocg90", 0x944, 0x0003, 0x0000)
IOMUX_IOCG(iocg91, "iocg91", 0x948, 0x0003, 0x0000)
IOMUX_IOCG(iocg92, "iocg92", 0x94C, 0x0003, 0x0000)
IOMUX_IOCG(iocg93, "iocg93", 0x950, 0x0003, 0x0000)
IOMUX_IOCG(iocg94, "iocg94", 0x954, 0x0003, 0x00F0)
IOMUX_IOCG(iocg95, "iocg95", 0x958, 0x0002, 0x0000)
IOMUX_IOCG(iocg194, "iocg194", 0x95C, 0x0000, 0x00F0)
IOMUX_IOCG(iocg96, "iocg96", 0x960, 0x0002, 0x0000)
IOMUX_IOCG(iocg97, "iocg97", 0x964, 0x0002, 0x0000)
IOMUX_IOCG(iocg98, "iocg98", 0x968, 0x0003, 0x00F0)
IOMUX_IOCG(iocg99, "iocg99", 0x96C, 0x0003, 0x00F0)
IOMUX_IOCG(iocg100, "iocg100", 0x970, 0x0003, 0x00F0)
IOMUX_IOCG(iocg101, "iocg101", 0x974, 0x0003, 0x00F0)
IOMUX_IOCG(iocg102, "iocg102", 0x978, 0x0003, 0x00F0)
IOMUX_IOCG(iocg103, "iocg103", 0x97C, 0x0003, 0x0000)
IOMUX_IOCG(iocg104, "iocg104", 0x980, 0x0003, 0x0000)
IOMUX_IOCG(iocg105, "iocg105", 0x984, 0x0003, 0x0000)
IOMUX_IOCG(iocg106, "iocg106", 0x988, 0x0003, 0x0000)
IOMUX_IOCG(iocg107, "iocg107", 0x98C, 0x0003, 0x00F0)
IOMUX_IOCG(iocg108, "iocg108", 0x990, 0x0003, 0x00F0)
IOMUX_IOCG(iocg109, "iocg109", 0x994, 0x0003, 0x00F0)
IOMUX_IOCG(iocg110, "iocg110", 0x998, 0x0003, 0x00F0)
IOMUX_IOCG(iocg111, "iocg111", 0x99C, 0x0003, 0x00F0)
IOMUX_IOCG(iocg112, "iocg112", 0x9A0, 0x0003, 0x00F0)
IOMUX_IOCG(iocg113, "iocg113", 0x9A4, 0x0003, 0x00F0)
IOMUX_IOCG(iocg114, "iocg114", 0x9A8, 0x0003, 0x00F0)
IOMUX_IOCG(iocg115, "iocg115", 0x9AC, 0x0003, 0x00F0)
IOMUX_IOCG(iocg116, "iocg116", 0x9B0, 0x0003, 0x00F0)
IOMUX_IOCG(iocg117, "iocg117", 0x9B4, 0x0003, 0x00F0)
IOMUX_IOCG(iocg118, "iocg118", 0x9B8, 0x0003, 0x00F0)
IOMUX_IOCG(iocg119, "iocg119", 0x9BC, 0x0003, 0x00F0)
IOMUX_IOCG(iocg120, "iocg120", 0x9C0, 0x0003, 0x00F0)
IOMUX_IOCG(iocg121, "iocg121", 0x9C4, 0x0003, 0x00F0)
IOMUX_IOCG(iocg122, "iocg122", 0x9C8, 0x0003, 0x00F0)
IOMUX_IOCG(iocg123, "iocg123", 0x9CC, 0x0003, 0x00F0)
IOMUX_IOCG(iocg124, "iocg124", 0x9D0, 0x0003, 0x00F0)
IOMUX_IOCG(iocg125, "iocg125", 0x9D4, 0x0003, 0x00F0)
IOMUX_IOCG(iocg126, "iocg126", 0x9D8, 0x0003, 0x00F0)
IOMUX_IOCG(iocg127, "iocg127", 0x9DC, 0x0003, 0x00F0)
IOMUX_IOCG(iocg128, "iocg128", 0x9E0, 0x0003, 0x00F0)
IOMUX_IOCG(iocg129, "iocg129", 0x9E4, 0x0003, 0x00F0)
IOMUX_IOCG(iocg130, "iocg130", 0x9E8, 0x0003, 0x00F0)
IOMUX_IOCG(iocg131, "iocg131", 0x9EC, 0x0003, 0x00F0)
IOMUX_IOCG(iocg132, "iocg132", 0x9F0, 0x0003, 0x00F0)
IOMUX_IOCG(iocg133, "iocg133", 0x9F4, 0x0003, 0x00F0)
IOMUX_IOCG(iocg134, "iocg134", 0x9F8, 0x0003, 0x00F0)
IOMUX_IOCG(iocg135, "iocg135", 0x9FC, 0x0003, 0x00F0)
IOMUX_IOCG(iocg136, "iocg136", 0xA00, 0x0003, 0x0000)
IOMUX_IOCG(iocg137, "iocg137", 0xA04, 0x0003, 0x0000)
IOMUX_IOCG(iocg138, "iocg138", 0xA08, 0x0003, 0x0000)
IOMUX_IOCG(iocg139, "iocg139", 0xA0C, 0x0003, 0x0000)
IOMUX_IOCG(iocg140, "iocg140", 0xA10, 0x0003, 0x0000)
IOMUX_IOCG(iocg141, "iocg141", 0xA14, 0x0003, 0x0000)
IOMUX_IOCG(iocg142, "iocg142", 0xA18, 0x0003, 0x0000)
IOMUX_IOCG(iocg143, "iocg143", 0xA1C, 0x0003, 0x0000)
IOMUX_IOCG(iocg144, "iocg144", 0xA20, 0x0003, 0x0000)
IOMUX_IOCG(iocg145, "iocg145", 0xA24, 0x0003, 0x0000)
IOMUX_IOCG(iocg146, "iocg146", 0xA28, 0x0003, 0x0000)
IOMUX_IOCG(iocg147, "iocg147", 0xA2C, 0x0003, 0x0000)
IOMUX_IOCG(iocg148, "iocg148", 0xA30, 0x0003, 0x0000)
IOMUX_IOCG(iocg149, "iocg149", 0xA34, 0x0003, 0x0000)
IOMUX_IOCG(iocg150, "iocg150", 0xA38, 0x0003, 0x0000)
IOMUX_IOCG(iocg151, "iocg151", 0xA3C, 0x0003, 0x0000)
IOMUX_IOCG(iocg152, "iocg152", 0xA40, 0x0003, 0x0000)
IOMUX_IOCG(iocg153, "iocg153", 0xA44, 0x0003, 0x0000)
IOMUX_IOCG(iocg154, "iocg154", 0xA48, 0x0003, 0x0000)
IOMUX_IOCG(iocg155, "iocg155", 0xA4C, 0x0003, 0x0000)
IOMUX_IOCG(iocg156, "iocg156", 0xA50, 0x0003, 0x0000)
IOMUX_IOCG(iocg157, "iocg157", 0xA5C, 0x0003, 0x0000)
IOMUX_IOCG(iocg158, "iocg158", 0xA58, 0x0003, 0x0000)
IOMUX_IOCG(iocg159, "iocg159", 0xA5C, 0x0003, 0x0000)
IOMUX_IOCG(iocg160, "iocg160", 0xA60, 0x0003, 0x0000)
IOMUX_IOCG(iocg161, "iocg161", 0xA64, 0x0003, 0x0000)
IOMUX_IOCG(iocg162, "iocg162", 0xA68, 0x0003, 0x0000)
IOMUX_IOCG(iocg163, "iocg163", 0xA6C, 0x0003, 0x0000)
IOMUX_IOCG(iocg164, "iocg164", 0xA70, 0x0003, 0x0000)
IOMUX_IOCG(iocg165, "iocg165", 0xA74, 0x0003, 0x0000)
IOMUX_IOCG(iocg166, "iocg166", 0xA78, 0x0003, 0x0000)
IOMUX_IOCG(iocg167, "iocg167", 0xA7C, 0x0003, 0x0000)
IOMUX_IOCG(iocg168, "iocg168", 0xA80, 0x0003, 0x00F0)
IOMUX_IOCG(iocg169, "iocg169", 0xA84, 0x0003, 0x00F0)
IOMUX_IOCG(iocg170, "iocg170", 0xA88, 0x0003, 0x0000)
IOMUX_IOCG(iocg171, "iocg171", 0xA8C, 0x0003, 0x0000)
IOMUX_IOCG(iocg172, "iocg172", 0xA90, 0x0003, 0x0000)
IOMUX_IOCG(iocg173, "iocg173", 0xA94, 0x0003, 0x00F0)
IOMUX_IOCG(iocg174, "iocg174", 0xA98, 0x0003, 0x0000)
IOMUX_IOCG(iocg175, "iocg175", 0xA9C, 0x0003, 0x00F0)
IOMUX_IOCG(iocg176, "iocg176", 0xAA0, 0x0003, 0x00F0)
IOMUX_IOCG(iocg177, "iocg177", 0xAA4, 0x0003, 0x0000)
IOMUX_IOCG(iocg178, "iocg178", 0xAA8, 0x0003, 0x0000)
IOMUX_IOCG(iocg179, "iocg179", 0xAAC, 0x0003, 0x0000)
IOMUX_IOCG(iocg180, "iocg180", 0xAB0, 0x0003, 0x0000)
IOMUX_IOCG(iocg181, "iocg181", 0xAB4, 0x0003, 0x00F0)
IOMUX_IOCG(iocg182, "iocg182", 0xAB8, 0x0003, 0x00F0)
IOMUX_IOCG(iocg183, "iocg183", 0xABC, 0x0003, 0x00F0)
IOMUX_IOCG(iocg184, "iocg184", 0xAC0, 0x0003, 0x00F0)
IOMUX_IOCG(iocg185, "iocg185", 0xAC4, 0x0003, 0x0000)
IOMUX_IOCG(iocg186, "iocg186", 0xAC8, 0x0003, 0x0000)
IOMUX_IOCG(iocg187, "iocg187", 0xACC, 0x0003, 0x0000)
IOMUX_IOCG(iocg188, "iocg188", 0xAD0, 0x0003, 0x0000)
IOMUX_IOCG(iocg189, "iocg189", 0xAD4, 0x0003, 0x0000)
IOMUX_IOCG(iocg190, "iocg190", 0xAD8, 0x0003, 0x00F0)

/*define pins*/
/*system bins*/
IOMUX_PIN(ah16, "boot_mode0", RESERVE, PULLUP, RESERVE, NULL, &iocg4)
IOMUX_PIN(ae18, "boot_mode1", RESERVE, PULLUP, RESERVE, NULL, &iocg5)
IOMUX_PIN(ae26, "efuse_dout", FUNC0, PULLDOWN, RESERVE, &iomg2, &iocg11)
IOMUX_PIN(a19, "isp_gpio8", FUNC1, PULLDOWN, RESERVE, &iomg40, &iocg93)
IOMUX_PIN(g16, "isp_gpio9", FUNC1, PULLDOWN, LEVEL2, &iomg41, &iocg94)
IOMUX_PIN(ah17, "clk_out0", FUNC0, PULLDOWN, LEVEL2, &iomg92, &iocg190)
IOMUX_PIN(af25, "keypad_out7", FUNC0, NOPULL, RESERVE, &iomg72, &iocg155)
IOMUX_PIN(aa21, "gpio_158", RESERVE, PULLUP, RESERVE, NULL, &iocg2)
IOMUX_PIN(u29, "gpio_159", RESERVE, PULLUP, RESERVE, NULL, &iocg3)
IOMUX_PIN(u21, "pmu_spi_clk", RESERVE, RESERVE, LEVEL2, NULL, &iocg191)
IOMUX_PIN(r21, "pmu_spi_data", RESERVE, PULLDOWN, LEVEL2, NULL, &iocg192)
IOMUX_PIN(r23, "pmu_spi_cs_n", RESERVE, RESERVE, LEVEL2, NULL, &iocg193)

/*bt pins*/
IOMUX_PIN(g28_cs, "bt_26M_in", RESERVE, PULLDOWN, LEVEL2, NULL, &iocg185)
IOMUX_PIN(h28_cs, "bt_enable_rm", RESERVE, PULLDOWN, LEVEL2, NULL, &iocg184)
IOMUX_PIN(j26_cs, "bt_spi_clk", RESERVE, PULLDOWN, LEVEL2, NULL, &iocg181)
IOMUX_PIN(k25_cs, "bt_spi_csn", RESERVE, PULLDOWN, LEVEL2, NULL, &iocg182)
IOMUX_PIN(j25_cs, "bt_spi_data", RESERVE, PULLDOWN, LEVEL2, NULL, &iocg183)

IOMUX_PIN(g25_cs, "coex_btactive", RESERVE, PULLDOWN, RESERVE, NULL, &iocg188)
IOMUX_PIN(h25_cs, "coex_btpriority", RESERVE, PULLDOWN, RESERVE, NULL, &iocg187)
IOMUX_PIN(h26_cs, "coex_wlactive", RESERVE, PULLDOWN, RESERVE, NULL, &iocg189)

IOMUX_PIN(g28, "bt_26M_in", FUNC1, PULLDOWN, LEVEL2, &iomg89, &iocg185)
IOMUX_PIN(h28, "bt_enable_rm", FUNC1, PULLDOWN, LEVEL2, &iomg89, &iocg184)
IOMUX_PIN(j26, "bt_spi_clk", FUNC1, PULLDOWN, LEVEL2, &iomg89, &iocg181)
IOMUX_PIN(k25, "bt_spi_csn", FUNC1, PULLDOWN, LEVEL2, &iomg89, &iocg182)
IOMUX_PIN(j25, "bt_spi_data", FUNC1, PULLDOWN, LEVEL2, &iomg89, &iocg183)

IOMUX_PIN(g25, "coex_btactive", FUNC1, PULLDOWN, LEVEL2, &iomg91, &iocg188)
IOMUX_PIN(h25, "coex_btpriority", FUNC1, PULLDOWN, LEVEL2, &iomg91, &iocg187)
IOMUX_PIN(h26, "coex_wlactive", FUNC1, PULLDOWN, LEVEL2, &iomg91, &iocg189)

IOMUX_PIN(g29, "bt_bdata1", FUNC1, PULLDOWN, LEVEL2, &iomg87, &iocg177)/*0:uart 1:gpio*/
IOMUX_PIN(l23, "bt_bpktctl", FUNC1, PULLDOWN, LEVEL2, &iomg88, &iocg179)/*0:uart 1:gpio*/
IOMUX_PIN(k29, "bt_brclk", FUNC1, PULLDOWN, LEVEL2, &iomg87, &iocg178)/*0:uart 1:gpio*/
IOMUX_PIN(j23, "rf_rst_n", FUNC1, PULLDOWN, LEVEL2, &iomg93, &iocg180)/*0:uart 1:gpio*/
IOMUX_PIN(k23, "rftcxo_pwr", FUNC1, PULLDOWN, LEVEL2, &iomg90, &iocg186)

/*cam pins*/
IOMUX_PIN(e16, "cam_data0", FUNC1, PULLDOWN, LEVEL2, &iomg16, &iocg58)
IOMUX_PIN(b18, "cam_data1", FUNC1, PULLDOWN, LEVEL2, &iomg16, &iocg59)
IOMUX_PIN(b17, "cam_data2", FUNC1, PULLDOWN, LEVEL2, &iomg16, &iocg60)
IOMUX_PIN(a17, "cam_data3", FUNC1, PULLDOWN, LEVEL2, &iomg16, &iocg61)
IOMUX_PIN(d17, "cam_data4", FUNC1, PULLDOWN, LEVEL2, &iomg16, &iocg62)
IOMUX_PIN(e15, "cam_data5", FUNC1, PULLDOWN, LEVEL2, &iomg16, &iocg63)
IOMUX_PIN(b16, "cam_data6", FUNC1, PULLDOWN, LEVEL2, &iomg16, &iocg64)
IOMUX_PIN(d16, "cam_data7", FUNC1, PULLDOWN, LEVEL2, &iomg16, &iocg65)
IOMUX_PIN(d15, "cam_data8", FUNC1, PULLDOWN, LEVEL2, &iomg17, &iocg66)
IOMUX_PIN(a14, "cam_data9", FUNC1, PULLDOWN, LEVEL2, &iomg17, &iocg67)
IOMUX_PIN(d13, "cam_hysnc", FUNC1, PULLDOWN, LEVEL2, &iomg18, &iocg68)
IOMUX_PIN(e13, "cam_pclk", FUNC1, PULLDOWN, LEVEL2, &iomg18, &iocg69)
IOMUX_PIN(b14, "cam_vsync", FUNC1, PULLDOWN, LEVEL2, &iomg18, &iocg70)
IOMUX_PIN(a22, "isp_cclk0", FUNC1, PULLDOWN, LEVEL2, &iomg30, &iocg83)
IOMUX_PIN(e18, "isp_cclk2", FUNC1, PULLDOWN, LEVEL2, &iomg31, &iocg84)

IOMUX_PIN(g19_cs, "isp_fsin0", FUNC1, PULLDOWN, LEVEL2,  &iomg24, &iocg77)/*don't need to set driver strenght*/
IOMUX_PIN(d20_cs, "i2c2_scl", FUNC1, PULLDOWN, LEVEL2,  &iomg26, &iocg79)/*don't need to set driver strenght*/
IOMUX_PIN(e21_cs, "isp_fsin1", FUNC1, PULLDOWN, LEVEL2,  &iomg25, &iocg78)/*don't need to set driver strenght*/
IOMUX_PIN(d22_cs, "i2c2_sda", FUNC1, PULLDOWN, LEVEL2,  &iomg27, &iocg80)/*don't need to set driver strenght*/

IOMUX_PIN(g19, "isp_fsin0_i", FUNC1, PULLDOWN, LEVEL2,  &iomg24, &iocg77)/*don't need to set driver strenght*/
IOMUX_PIN(d20, "isp_fsin0_o", FUNC1, PULLDOWN, LEVEL2,  &iomg26, &iocg79)/*don't need to set driver strenght*/
IOMUX_PIN(e21, "isp_fsin1_i", FUNC1, PULLDOWN, LEVEL2,  &iomg25, &iocg78)/*don't need to set driver strenght*/
IOMUX_PIN(d22, "isp_fsn1_o", FUNC1, PULLDOWN, LEVEL2,  &iomg27, &iocg80)/*don't need to set driver strenght*/

IOMUX_PIN(b21, "isp_gpio0", FUNC1, PULLDOWN, LEVEL2, &iomg32, &iocg85)/*don't need to set driver strenght*/
IOMUX_PIN(g18, "isp_gpio1", FUNC1, PULLDOWN, LEVEL2, &iomg33, &iocg86)/*don't need to set driver strenght*/
IOMUX_PIN(d18, "isp_gpio2", FUNC1, PULLDOWN, LEVEL2, &iomg34, &iocg87)/*don't need to set driver strenght*/
IOMUX_PIN(b20, "isp_gpio3", FUNC1, PULLDOWN, LEVEL2, &iomg35, &iocg88)/*don't need to set driver strenght*/
IOMUX_PIN(a21, "isp_gpio4", FUNC1, PULLDOWN, LEVEL2, &iomg36, &iocg89)/*don't need to set driver strenght*/
IOMUX_PIN(a20, "isp_gpio5", FUNC1, PULLDOWN, LEVEL2, &iomg37, &iocg90)/*don't need to set driver strenght*/
IOMUX_PIN(b19, "isp_gpio6", FUNC1, PULLDOWN, LEVEL2, &iomg38, &iocg91)/*don't need to set driver strenght*/

IOMUX_PIN(e17_cs, "isp_gpio7", FUNC1, PULLDOWN, LEVEL2, &iomg39, &iocg92)/*the default value change form func3 to func1(gpio)*/

IOMUX_PIN(e17, "isp_gpio7", FUNC3, PULLDOWN, LEVEL2, &iomg39, &iocg92)/*the default value change form func3 to func1(gpio)*/

IOMUX_PIN(g20, "isp_resetb0", FUNC1, PULLDOWN, LEVEL2, &iomg22, &iocg75)/*don't need to set driver strenght*/
IOMUX_PIN(e22, "isp_resetb1", FUNC1, PULLDOWN, LEVEL2, &iomg23, &iocg76)/*don't need to set driver strenght*/
IOMUX_PIN(d23, "isp_scl0", FUNC1, PULLDOWN, LEVEL2, &iomg19, &iocg71)/*don't need to set driver strenght*/
IOMUX_PIN(d24, "isp_scl1", FUNC1, PULLDOWN, LEVEL2, &iomg20, &iocg73)/*don't need to set driver strenght*/
IOMUX_PIN(b23, "isp_sda0", FUNC1, PULLDOWN, LEVEL2, &iomg19, &iocg72)/*don't need to set driver strenght*/
IOMUX_PIN(a23, "isp_sda1", FUNC1, PULLDOWN, LEVEL2, &iomg21, &iocg74)/*don't need to set driver strenght*/
IOMUX_PIN(e20, "isp_strobe0", FUNC1, PULLDOWN, LEVEL2,  &iomg28, &iocg81)/*don't need to set driver strenght*/
IOMUX_PIN(d21, "isp_strobe1", FUNC1, PULLDOWN, LEVEL2,  &iomg29, &iocg82)/*don't need to set driver strenght*/

/*periph*/
IOMUX_PIN(p25, "i2c0_scl", FUNC1, PULLDOWN, LEVEL2, &iomg45, &iocg103)
IOMUX_PIN(r25, "i2c0_sda", FUNC1, PULLDOWN, LEVEL2, &iomg45, &iocg104)
IOMUX_PIN(n25, "i2c1_scl", FUNC1, PULLDOWN, LEVEL2, &iomg46, &iocg105)
IOMUX_PIN(p26, "i2c1_sda", FUNC1, PULLDOWN, LEVEL2, &iomg46, &iocg106)
IOMUX_PIN(ac26, "onewire", FUNC1, PULLDOWN, LEVEL2, &iomg64, &iocg147)
IOMUX_PIN(d28, "pwm_out0", FUNC1, PULLDOWN, LEVEL2, &iomg82, &iocg168)
IOMUX_PIN(b27, "pwm_out1", FUNC1, PULLDOWN, LEVEL2, &iomg83, &iocg169)
IOMUX_PIN(n29, "spi0_clk", FUNC1, PULLDOWN, LEVEL2, &iomg53, &iocg125)
IOMUX_PIN(l29, "spi0_cs0_n", FUNC1, PULLDOWN, LEVEL2, &iomg54, &iocg128)
IOMUX_PIN(m21, "spi0_cs1_n", FUNC1, PULLDOWN, LEVEL2, &iomg55, &iocg129)
IOMUX_PIN(n28, "spi0_cs2_n", FUNC1, PULLDOWN, LEVEL2, &iomg56, &iocg130)
IOMUX_PIN(m23, "spi0_cs3_n", FUNC1, PULLDOWN, LEVEL2, &iomg57, &iocg131)
IOMUX_PIN(n26, "spi0_di", FUNC1, PULLDOWN, LEVEL2, &iomg53, &iocg126)
IOMUX_PIN(m29, "spi0_do", FUNC1, PULLDOWN, LEVEL2, &iomg53, &iocg127)
IOMUX_PIN(a27, "uart0_cts_n", FUNC1, PULLDOWN, LEVEL2, &iomg59, &iocg136)
IOMUX_PIN(b26, "uart0_rts_n", FUNC1, PULLDOWN, LEVEL2, &iomg59, &iocg137)
IOMUX_PIN(a26, "uart0_rxd", FUNC1, PULLDOWN, LEVEL2, &iomg60, &iocg138)
IOMUX_PIN(a25, "uart0_txd", FUNC1, PULLDOWN, LEVEL2, &iomg60, &iocg139)
IOMUX_PIN(ac28, "usim_clk", FUNC1, PULLDOWN, LEVEL2, &iomg63, &iocg144)
IOMUX_PIN(aa28, "usim_data", FUNC1, PULLDOWN, LEVEL2, &iomg63, &iocg145)
IOMUX_PIN(aa29, "usim_rst", FUNC1, PULLDOWN, LEVEL2, &iomg96, &iocg146)

/*gpio*/
IOMUX_PIN(af26, "efuse_csb", FUNC0, PULLDOWN, LEVEL2, &iomg0, &iocg7)
IOMUX_PIN(ad26, "efuse_din", FUNC0, PULLDOWN, LEVEL2, &iomg1, &iocg10)
IOMUX_PIN(ac25, "efuse_pgm", FUNC0, PULLDOWN, LEVEL2, &iomg0, &iocg9)
IOMUX_PIN(ad25, "efuse_sclk", FUNC0, PULLDOWN, LEVEL2, &iomg0, &iocg8)
IOMUX_PIN(ah27, "efuse_sel", FUNC0, PULLDOWN, LEVEL2, &iomg0, &iocg6)
IOMUX_PIN(aa23, "gpio_156", RESERVE, PULLDOWN, LEVEL2, NULL, &iocg0)
IOMUX_PIN(v29, "gpio_157", RESERVE, PULLDOWN, LEVEL2, NULL, &iocg1)

/*nvm*/
IOMUX_PIN(ae1, "emmc_clk", FUNC0, PULLDOWN, LEVEL2, &iomg12, &iocg41)
IOMUX_PIN(ae4, "emmc_cmd", FUNC0, PULLDOWN, LEVEL2, &iomg12, &iocg40)
IOMUX_PIN(v2, "nand_ale", FUNC0, NOPULL, LEVEL2, &iomg3, &iocg12)
IOMUX_PIN(w2, "nand_cle", FUNC0, NOPULL, LEVEL2, &iomg3, &iocg13)
IOMUX_PIN(y5, "nand_re_n", FUNC0, NOPULL, LEVEL2, &iomg3, &iocg14)
IOMUX_PIN(w1, "nand_we_n", FUNC0, NOPULL, LEVEL2, &iomg3, &iocg15)
IOMUX_PIN(u5, "nand_cs0_n", FUNC0, NOPULL, LEVEL2, &iomg3, &iocg16)
IOMUX_PIN(u4, "nand_cs1_n", FUNC0, NOPULL, LEVEL2, &iomg4, &iocg17)
IOMUX_PIN(u2, "nand_cs2_n", FUNC0, NOPULL, LEVEL2, &iomg5, &iocg18)
IOMUX_PIN(v1, "nand_cs3_n", FUNC0, NOPULL, LEVEL2, &iomg6, &iocg19)
IOMUX_PIN(v5, "nand_busy0_n", FUNC0, PULLUP, LEVEL2, &iomg94, &iocg20)
IOMUX_PIN(w4, "nand_busy1_n", FUNC0, PULLUP, LEVEL2, &iomg7, &iocg21)
IOMUX_PIN(v4, "nand_busy2_n", FUNC0, PULLUP, LEVEL2, &iomg8, &iocg22)
IOMUX_PIN(w5, "nand_busy3_n", FUNC0, PULLUP, LEVEL2, &iomg9, &iocg23)
IOMUX_PIN(u1, "nand_data0", FUNC0, PULLDOWN, LEVEL2, &iomg3, &iocg24)
IOMUX_PIN(aa2, "nand_data1", FUNC0, PULLDOWN, LEVEL2, &iomg3, &iocg25)
IOMUX_PIN(ab2, "nand_data2", FUNC0, PULLDOWN, LEVEL2, &iomg3, &iocg26)
IOMUX_PIN(y4, "nand_data3", FUNC0, PULLDOWN, LEVEL2, &iomg3, &iocg27)
IOMUX_PIN(aa4, "nand_data4", FUNC0, PULLDOWN, LEVEL2, &iomg3, &iocg28)
IOMUX_PIN(aa5, "nand_data5", FUNC0, PULLDOWN, LEVEL2, &iomg3, &iocg29)
IOMUX_PIN(ab4, "nand_data6", FUNC0, PULLDOWN, LEVEL2, &iomg3, &iocg30)
IOMUX_PIN(ab5, "nand_data7", FUNC0, PULLDOWN, LEVEL2, &iomg3, &iocg31)
IOMUX_PIN(ab1, "nand_data8", FUNC0, PULLDOWN, LEVEL2, &iomg10, &iocg32)
IOMUX_PIN(ac5, "nand_data9", FUNC0, PULLDOWN, LEVEL2, &iomg10, &iocg33)
IOMUX_PIN(ac1, "nand_data10", FUNC0, PULLDOWN, LEVEL2, &iomg10, &iocg34)
IOMUX_PIN(ac4, "nand_data11", FUNC0, PULLDOWN, LEVEL2, &iomg10, &iocg35)
IOMUX_PIN(ae2, "nand_data12", FUNC0, PULLDOWN, LEVEL2, &iomg10, &iocg36)
IOMUX_PIN(ad2, "nand_data13", FUNC0, PULLDOWN, LEVEL2, &iomg10, &iocg37)
IOMUX_PIN(ad4, "nand_data14", FUNC0, PULLDOWN, LEVEL2, &iomg10, &iocg38)
IOMUX_PIN(ad5, "nand_data15", FUNC0, PULLDOWN, LEVEL2, &iomg10, &iocg39)

/*gps*/
IOMUX_PIN(m26, "gps_acqclk", FUNC1, PULLDOWN, LEVEL2, &iomg84, &iocg171)/*nodrv*/
IOMUX_PIN(l25, "gps_pd", FUNC1, PULLDOWN, LEVEL2, &iomg84, &iocg172)
IOMUX_PIN(j28, "gps_sign", FUNC1, PULLDOWN, LEVEL2, &iomg84, &iocg170)/*nodrv*/
IOMUX_PIN(l28, "gps_spi_clk", FUNC1, PULLDOWN, LEVEL2, &iomg85, &iocg173)
IOMUX_PIN(k28, "gps_spi_di", FUNC1, PULLDOWN, LEVEL2, &iomg85, &iocg174)
IOMUX_PIN(k26, "gps_spi_do", FUNC1, PULLDOWN, LEVEL2, &iomg86, &iocg175)
IOMUX_PIN(m28, "ps_spi_en_n", FUNC1, PULLDOWN, LEVEL2, &iomg86, &iocg176)

/*hdmi*/
IOMUX_PIN(e11, "hdmi_cec", FUNC1, PULLDOWN, LEVEL2, &iomg14, &iocg56)
/*no iomg iocg*/
IOMUX_PIN(d11, "hdmi_hpd", FUNC1, PULLDOWN, LEVEL2, &iomg15, &iocg57)
IOMUX_PIN(e12, "hdmi_scl", FUNC1, PULLDOWN, LEVEL2, &iomg13, &iocg54)
IOMUX_PIN(d12, "hdmi_sda", FUNC1, PULLDOWN, LEVEL2, &iomg13, &iocg55)

/*audio*/
IOMUX_PIN(t25, "i2s_di", RESERVE, PULLDOWN, LEVEL2, NULL, &iocg95)
IOMUX_PIN(p21, "i2s_do", FUNC0, RESERVE, LEVEL2, &iomg42, &iocg194)
IOMUX_PIN(p23, "i2s_xclk", RESERVE, PULLDOWN, LEVEL2, NULL, &iocg96)
IOMUX_PIN(n23, "i2s_xfs", RESERVE, PULLDOWN, LEVEL2, NULL, &iocg97)

IOMUX_PIN(u26_cs, "pcm_di", FUNC0, PULLDOWN, LEVEL2, NULL, &iocg98)
IOMUX_PIN(t28_cs, "pcm_do", FUNC0, PULLDOWN, LEVEL2, NULL, &iocg99)
IOMUX_PIN(t26_cs, "pcm_xclk", FUNC0, PULLDOWN, LEVEL2, NULL, &iocg100)
IOMUX_PIN(r26_cs, "usb1_drv_vbus", FUNC0, PULLDOWN, LEVEL2, &iomg43, &iocg101)/*func 1:usb1_drv_vbus*/
IOMUX_PIN(a28_cs, "spdif", FUNC0, PULLDOWN, LEVEL2, &iomg44, &iocg102)

IOMUX_PIN(u26, "pcm_di", FUNC1, PULLDOWN, LEVEL2, &iomg43, &iocg98)
IOMUX_PIN(t28, "pcm_do", FUNC1, PULLDOWN, LEVEL2, &iomg43, &iocg99)
IOMUX_PIN(t26, "pcm_xclk", FUNC1, PULLDOWN, LEVEL2, &iomg43, &iocg100)
IOMUX_PIN(r26, "usb1_drv_vbus", FUNC1, PULLDOWN, LEVEL2, &iomg43, &iocg101)/*func 1:usb1_drv_vbus*/
IOMUX_PIN(a28, "spdif", FUNC1, PULLDOWN, LEVEL2, &iomg44, &iocg102)

/*keypad*/
IOMUX_PIN(aj21, "keypad_in0", FUNC1, PULLDOWN, LEVEL2, &iomg73, &iocg156)
IOMUX_PIN(ah21, "keypad_in1", FUNC1, PULLDOWN, LEVEL2, &iomg74, &iocg157)
IOMUX_PIN(af22, "keypad_in2", FUNC1, PULLDOWN, LEVEL2, &iomg75, &iocg158)
IOMUX_PIN(aj23, "keypad_in3", FUNC1, PULLDOWN, LEVEL2, &iomg76, &iocg159)
IOMUX_PIN(ae22, "keypad_in4", FUNC1, PULLUP, LEVEL2, &iomg77, &iocg160)
IOMUX_PIN(ah23, "keypad_in5", FUNC1, PULLUP, LEVEL2, &iomg78, &iocg161)
IOMUX_PIN(af23, "keypad_in6", FUNC1, PULLUP, LEVEL2, &iomg79, &iocg162)
IOMUX_PIN(ae23, "keypad_in7", FUNC1, PULLUP, LEVEL2, &iomg80, &iocg163)
IOMUX_PIN(aj24, "keypad_out0", FUNC1, PULLDOWN, LEVEL2, &iomg65, &iocg148)
IOMUX_PIN(ah24, "keypad_out1", FUNC1, PULLDOWN, LEVEL2, &iomg66, &iocg149)
IOMUX_PIN(ah25, "keypad_out2", FUNC1, PULLDOWN, LEVEL2, &iomg67, &iocg150)
IOMUX_PIN(aj25, "keypad_out3", FUNC1, PULLDOWN, LEVEL2, &iomg68, &iocg151)
IOMUX_PIN(aj26, "keypad_out4", FUNC1, PULLDOWN, LEVEL2, &iomg69, &iocg152)
IOMUX_PIN(ae25, "keypad_out5", FUNC0, PULLDOWN, LEVEL2, &iomg70, &iocg153)
IOMUX_PIN(aj27, "keypad_out6", FUNC1, PULLDOWN, LEVEL2, &iomg71, &iocg154)
IOMUX_PIN(ab25, "tbc_down", FUNC1, PULLDOWN, LEVEL2, &iomg97, &iocg167)
IOMUX_PIN(aa25, "tbc_left", FUNC1, PULLDOWN, LEVEL2, &iomg81, &iocg164)
IOMUX_PIN(ab28, "tbc_right", FUNC1, PULLDOWN, LEVEL2, &iomg81, &iocg165)
IOMUX_PIN(ab26, "tbc_up", FUNC1, PULLDOWN, LEVEL2, &iomg97, &iocg166)

/*sd*/
IOMUX_PIN(ag2, "sd_clk", FUNC1, PULLDOWN, LEVEL2, &iomg47, &iocg107)
IOMUX_PIN(af5, "sd_cmd", FUNC1, PULLDOWN, LEVEL2, &iomg47, &iocg108)
IOMUX_PIN(ae5, "sd_data0", FUNC1, PULLDOWN, LEVEL2, &iomg47, &iocg109)
IOMUX_PIN(af4, "sd_data1", FUNC1, PULLDOWN, LEVEL2, &iomg47, &iocg110)
IOMUX_PIN(af6, "sd_data2", FUNC1, PULLDOWN, LEVEL2, &iomg47, &iocg111)
IOMUX_PIN(ae6, "sd_data3", FUNC1, PULLDOWN, LEVEL2, &iomg48, &iocg112)

/*modem*/
IOMUX_PIN(g1, "sdio0_clk", FUNC1, PULLDOWN, LEVEL2, &iomg49, &iocg113)
IOMUX_PIN(f4, "sdio0_cmd", FUNC1, PULLDOWN, LEVEL2, &iomg49, &iocg114)
IOMUX_PIN(f2, "sdio0_data0", FUNC1, PULLDOWN, LEVEL2, &iomg49, &iocg115)
IOMUX_PIN(f1, "sdio0_data1", FUNC1, PULLDOWN, LEVEL2, &iomg50, &iocg116)
IOMUX_PIN(g2, "sdio0_data2", FUNC1, PULLDOWN, LEVEL2, &iomg49, &iocg117)
IOMUX_PIN(f5, "sdio0_data3", FUNC1, PULLDOWN, LEVEL2, &iomg49, &iocg118)
IOMUX_PIN(d1, "spi1_clk", FUNC1, PULLDOWN, LEVEL2, &iomg58, &iocg132)
IOMUX_PIN(e1, "spi1_cs_n", FUNC1, PULLDOWN, LEVEL2, &iomg95, &iocg135)
IOMUX_PIN(d2, "spi1_di", FUNC1, PULLDOWN, LEVEL2, &iomg58, &iocg133)
IOMUX_PIN(e2, "spi1_do", FUNC1, PULLDOWN, LEVEL2, &iomg58, &iocg134)
IOMUX_PIN(ae28, "uart1_cts_n", FUNC1, PULLDOWN, LEVEL2, &iomg61, &iocg140)
IOMUX_PIN(af29, "uart1_rts_n", FUNC1, PULLDOWN, LEVEL2, &iomg61, &iocg141)
IOMUX_PIN(af28, "uart1_rxd", FUNC1, PULLDOWN, LEVEL2, &iomg62, &iocg142)
IOMUX_PIN(ag28, "uart1_txd", FUNC1, PULLDOWN, LEVEL2, &iomg62, &iocg143)
IOMUX_PIN(d1_cs, "spi1_clk", FUNC1, PULLDOWN, LEVEL2, &iomg98, &iocg132)
IOMUX_PIN(d2_cs, "spi1_di", FUNC1, PULLDOWN, LEVEL2, &iomg98, &iocg133)

/*wifi*/
IOMUX_PIN(e25, "sdio1_clk", FUNC1, PULLDOWN, LEVEL2, &iomg51, &iocg119)
IOMUX_PIN(e28, "sdio1_cmd", FUNC1, PULLDOWN, LEVEL2, &iomg51, &iocg120)
IOMUX_PIN(e29, "sdio1_data0", FUNC1, PULLDOWN, LEVEL2, &iomg51, &iocg121)
IOMUX_PIN(g26, "sdio1_data1", FUNC1, PULLDOWN, LEVEL2, &iomg52, &iocg122)
IOMUX_PIN(d29, "sdio1_data2", FUNC1, PULLDOWN, LEVEL2, &iomg51, &iocg123)
IOMUX_PIN(e26, "sdio1_data3", FUNC1, PULLDOWN, LEVEL2, &iomg51, &iocg124)

struct iomux_pin_table pins_table[] = {
	PIN_TABLE("aa23", &aa23), PIN_TABLE("v29", &v29), PIN_TABLE("aa21", &aa21), PIN_TABLE("u29", &u29),\
	PIN_TABLE("ah16", &ah16), PIN_TABLE("ae18", &ae18), PIN_TABLE("ah27", &ah27), PIN_TABLE("af26", &af26),\
	PIN_TABLE("ad25", &ad25), PIN_TABLE("ac25", &ac25), PIN_TABLE("ad26", &ad26), PIN_TABLE("ae26", &ae26),\
	PIN_TABLE("v2", &v2), PIN_TABLE("w2", &w2), PIN_TABLE("y5", &y5), PIN_TABLE("w1", &w1), PIN_TABLE("u5", &u5),\
	PIN_TABLE("u4", &u4), PIN_TABLE("u2", &u2), PIN_TABLE("v1", &v1), PIN_TABLE("v5", &v5), PIN_TABLE("w4", &w4),\
	PIN_TABLE("v4", &v4), PIN_TABLE("w5", &w5), PIN_TABLE("u1", &u1), PIN_TABLE("aa2", &aa2), PIN_TABLE("ab2", &ab2),\
	PIN_TABLE("y4", &y4), PIN_TABLE("aa4", &aa4), PIN_TABLE("aa5", &aa5), PIN_TABLE("ab4", &ab4), PIN_TABLE("ab5", &ab5),\
	PIN_TABLE("ab1", &ab1), PIN_TABLE("ac5", &ac5), PIN_TABLE("ac1", &ac1), PIN_TABLE("ac4", &ac4), \
	PIN_TABLE("ae2", &ae2), PIN_TABLE("ad2", &ad2), PIN_TABLE("ad4", &ad4), PIN_TABLE("ad5", &ad5), PIN_TABLE("ae4", &ae4), \
	PIN_TABLE("ae1", &ae1), PIN_TABLE("e12", &e12), PIN_TABLE("d12", &d12), \
	PIN_TABLE("e11", &e11), PIN_TABLE("d11", &d11), PIN_TABLE("e16", &e16), PIN_TABLE("b18", &b18), PIN_TABLE("b17", &b17), \
	PIN_TABLE("a17", &a17), PIN_TABLE("d17", &d17), PIN_TABLE("e15", &e15), PIN_TABLE("b16", &b16), PIN_TABLE("d16", &d16), \
	PIN_TABLE("d15", &d15), PIN_TABLE("a14", &a14), PIN_TABLE("d13", &d13), PIN_TABLE("e13", &e13), PIN_TABLE("b14", &b14), \
	PIN_TABLE("d23", &d23), PIN_TABLE("d24", &d24), PIN_TABLE("b23", &b23), PIN_TABLE("a23", &a23), PIN_TABLE("g20", &g20), \
	PIN_TABLE("e22", &e22), PIN_TABLE("g19", &g19), PIN_TABLE("e21", &e21), PIN_TABLE("d20", &d20), PIN_TABLE("d22", &d22), \
	PIN_TABLE("e20", &e20), PIN_TABLE("d21", &d21), PIN_TABLE("a22", &a22), PIN_TABLE("e18", &e18), PIN_TABLE("b21", &b21), \
	PIN_TABLE("g18", &g18), PIN_TABLE("d18", &d18), PIN_TABLE("b20", &b20), PIN_TABLE("a21", &a21), PIN_TABLE("a20", &a20), \
	PIN_TABLE("b19", &b19), PIN_TABLE("e17", &e17), PIN_TABLE("a19", &a19), PIN_TABLE("g16", &g16), PIN_TABLE("t25", &t25), \
	PIN_TABLE("p23", &p23), PIN_TABLE("n23", &n23), PIN_TABLE("u26", &u26), PIN_TABLE("t28", &t28), PIN_TABLE("t26", &t26), \
	PIN_TABLE("a28", &a28), PIN_TABLE("r26", &r26), PIN_TABLE("p25", &p25), PIN_TABLE("r25", &r25), PIN_TABLE("n25", &n25), \
	PIN_TABLE("p26", &p26), PIN_TABLE("ag2", &ag2), PIN_TABLE("af5", &af5), PIN_TABLE("ae5", &ae5), PIN_TABLE("af4", &af4), \
	PIN_TABLE("af6", &af6), PIN_TABLE("ae6", &ae6), PIN_TABLE("g1", &g1), PIN_TABLE("f4", &f4), PIN_TABLE("f2", &f2), \
	PIN_TABLE("f1", &f1), PIN_TABLE("g2", &g2), PIN_TABLE("f5", &f5), PIN_TABLE("e25", &e25), PIN_TABLE("e28", &e28), \
	PIN_TABLE("e29", &e29), PIN_TABLE("g26", &g26), PIN_TABLE("d29", &d29), PIN_TABLE("e26", &e26), PIN_TABLE("n29", &n29), \
	PIN_TABLE("n26", &n26), PIN_TABLE("m29", &m29), PIN_TABLE("l29", &l29), PIN_TABLE("m21", &m21), PIN_TABLE("n28", &n28), \
	PIN_TABLE("m23", &m23), PIN_TABLE("d1", &d1), PIN_TABLE("e1", &e1), PIN_TABLE("d2", &d2), PIN_TABLE("e2", &e2), \
	PIN_TABLE("a27", &a27), PIN_TABLE("b26", &b26), PIN_TABLE("a26", &a26), PIN_TABLE("a25", &a25), PIN_TABLE("ae28", &ae28), \
	PIN_TABLE("af29", &af29), PIN_TABLE("af28", &af28), PIN_TABLE("ag28", &ag28), PIN_TABLE("ac28", &ac28), PIN_TABLE("aa28", &aa28), \
	PIN_TABLE("aa29", &aa29), PIN_TABLE("ac26", &ac26), PIN_TABLE("aj24", &aj24), PIN_TABLE("ah24", &ah24), PIN_TABLE("ah25", &ah25), \
	PIN_TABLE("aj25", &aj25), PIN_TABLE("ae25", &ae25), PIN_TABLE("aj26", &aj26), PIN_TABLE("e25", &e25), PIN_TABLE("af25", &af25), \
	PIN_TABLE("aj21", &aj21), PIN_TABLE("ah21", &ah21), PIN_TABLE("af22", &af22), PIN_TABLE("aj23", &aj23), PIN_TABLE("ae22", &ae22), \
	PIN_TABLE("ah23", &ah23), PIN_TABLE("af23", &af23), PIN_TABLE("ae23", &ae23), PIN_TABLE("aa25", &aa25), PIN_TABLE("ab28", &ab28), \
	PIN_TABLE("ab26", &ab26), PIN_TABLE("ab25", &ab25), PIN_TABLE("d28", &d28), PIN_TABLE("b27", &b27), PIN_TABLE("j28", &j28), \
	PIN_TABLE("m26", &m26), PIN_TABLE("l25", &l25), PIN_TABLE("l28", &l28), PIN_TABLE("k28", &k28), PIN_TABLE("k26", &k26), \
	PIN_TABLE("m28", &m28), PIN_TABLE("g29", &g29), PIN_TABLE("k29", &k29), PIN_TABLE("l23", &l23), PIN_TABLE("j23", &j23), \
	PIN_TABLE("j26", &j26), PIN_TABLE("k25", &k25), PIN_TABLE("j25", &j25), PIN_TABLE("h28", &h28), PIN_TABLE("g28", &g28), \
	PIN_TABLE("k23", &k23), PIN_TABLE("h25", &h25), PIN_TABLE("g25", &g25), PIN_TABLE("h26", &h26), PIN_TABLE("ah17", &ah17), \
	PIN_TABLE("u21", &u21), PIN_TABLE("r21", &r21), PIN_TABLE("r23", &r23), PIN_TABLE("p21", &p21), PIN_TABLE(NULL, NULL), \
};

struct iomux_pin_table pins_table_cs[] = {
	PIN_TABLE("aa23", &aa23), PIN_TABLE("v29", &v29), PIN_TABLE("aa21", &aa21), PIN_TABLE("u29", &u29),\
	PIN_TABLE("ah16", &ah16), PIN_TABLE("ae18", &ae18), PIN_TABLE("ah27", &ah27), PIN_TABLE("af26", &af26),\
	PIN_TABLE("ad25", &ad25), PIN_TABLE("ac25", &ac25), PIN_TABLE("ad26", &ad26), PIN_TABLE("ae26", &ae26),\
	PIN_TABLE("v2", &v2), PIN_TABLE("w2", &w2), PIN_TABLE("y5", &y5), PIN_TABLE("w1", &w1), PIN_TABLE("u5", &u5),\
	PIN_TABLE("u4", &u4), PIN_TABLE("u2", &u2), PIN_TABLE("v1", &v1), PIN_TABLE("v5", &v5), PIN_TABLE("w4", &w4),\
	PIN_TABLE("v4", &v4), PIN_TABLE("w5", &w5), PIN_TABLE("u1", &u1), PIN_TABLE("aa2", &aa2), PIN_TABLE("ab2", &ab2),\
	PIN_TABLE("y4", &y4), PIN_TABLE("aa4", &aa4), PIN_TABLE("aa5", &aa5), PIN_TABLE("ab4", &ab4), PIN_TABLE("ab5", &ab5),\
	PIN_TABLE("ab1", &ab1), PIN_TABLE("ac5", &ac5), PIN_TABLE("ac1", &ac1), PIN_TABLE("ac4", &ac4), \
	PIN_TABLE("ae2", &ae2), PIN_TABLE("ad2", &ad2), PIN_TABLE("ad4", &ad4), PIN_TABLE("ad5", &ad5), PIN_TABLE("ae4", &ae4), \
	PIN_TABLE("ae1", &ae1), PIN_TABLE("e12", &e12), PIN_TABLE("d12", &d12), \
	PIN_TABLE("e11", &e11), PIN_TABLE("d11", &d11), PIN_TABLE("e16", &e16), PIN_TABLE("b18", &b18), PIN_TABLE("b17", &b17), \
	PIN_TABLE("a17", &a17), PIN_TABLE("d17", &d17), PIN_TABLE("e15", &e15), PIN_TABLE("b16", &b16), PIN_TABLE("d16", &d16), \
	PIN_TABLE("d15", &d15), PIN_TABLE("a14", &a14), PIN_TABLE("d13", &d13), PIN_TABLE("e13", &e13), PIN_TABLE("b14", &b14), \
	PIN_TABLE("d23", &d23), PIN_TABLE("d24", &d24), PIN_TABLE("b23", &b23), PIN_TABLE("a23", &a23), PIN_TABLE("g20", &g20), \
	PIN_TABLE("e22", &e22), PIN_TABLE("g19", &g19_cs), PIN_TABLE("e21", &e21_cs), PIN_TABLE("d20", &d20_cs), PIN_TABLE("d22", &d22_cs), \
	PIN_TABLE("e20", &e20), PIN_TABLE("d21", &d21), PIN_TABLE("a22", &a22), PIN_TABLE("e18", &e18), PIN_TABLE("b21", &b21), \
	PIN_TABLE("g18", &g18), PIN_TABLE("d18", &d18), PIN_TABLE("b20", &b20), PIN_TABLE("a21", &a21), PIN_TABLE("a20", &a20), \
	PIN_TABLE("b19", &b19), PIN_TABLE("e17", &e17_cs), PIN_TABLE("a19", &a19), PIN_TABLE("g16", &g16), PIN_TABLE("t25", &t25), \
	PIN_TABLE("p23", &p23), PIN_TABLE("n23", &n23), PIN_TABLE("u26", &u26_cs), PIN_TABLE("t28", &t28_cs), PIN_TABLE("t26", &t26_cs), \
	PIN_TABLE("a28", &a28_cs), PIN_TABLE("r26", &r26_cs), PIN_TABLE("p25", &p25), PIN_TABLE("r25", &r25), PIN_TABLE("n25", &n25), \
	PIN_TABLE("p26", &p26), PIN_TABLE("ag2", &ag2), PIN_TABLE("af5", &af5), PIN_TABLE("ae5", &ae5), PIN_TABLE("af4", &af4), \
	PIN_TABLE("af6", &af6), PIN_TABLE("ae6", &ae6), PIN_TABLE("g1", &g1), PIN_TABLE("f4", &f4), PIN_TABLE("f2", &f2), \
	PIN_TABLE("f1", &f1), PIN_TABLE("g2", &g2), PIN_TABLE("f5", &f5), PIN_TABLE("e25", &e25), PIN_TABLE("e28", &e28), \
	PIN_TABLE("e29", &e29), PIN_TABLE("g26", &g26), PIN_TABLE("d29", &d29), PIN_TABLE("e26", &e26), PIN_TABLE("n29", &n29), \
	PIN_TABLE("n26", &n26), PIN_TABLE("m29", &m29), PIN_TABLE("l29", &l29), PIN_TABLE("m21", &m21), PIN_TABLE("n28", &n28), \
	PIN_TABLE("m23", &m23), PIN_TABLE("d1", &d1_cs), PIN_TABLE("e1", &e1), PIN_TABLE("d2", &d2_cs), PIN_TABLE("e2", &e2), \
	PIN_TABLE("a27", &a27), PIN_TABLE("b26", &b26), PIN_TABLE("a26", &a26), PIN_TABLE("a25", &a25), PIN_TABLE("ae28", &ae28), \
	PIN_TABLE("af29", &af29), PIN_TABLE("af28", &af28), PIN_TABLE("ag28", &ag28), PIN_TABLE("ac28", &ac28), PIN_TABLE("aa28", &aa28), \
	PIN_TABLE("aa29", &aa29), PIN_TABLE("ac26", &ac26), PIN_TABLE("aj24", &aj24), PIN_TABLE("ah24", &ah24), PIN_TABLE("ah25", &ah25), \
	PIN_TABLE("aj25", &aj25), PIN_TABLE("ae25", &ae25), PIN_TABLE("aj26", &aj26), PIN_TABLE("e25", &e25), PIN_TABLE("af25", &af25), \
	PIN_TABLE("aj21", &aj21), PIN_TABLE("ah21", &ah21), PIN_TABLE("af22", &af22), PIN_TABLE("aj23", &aj23), PIN_TABLE("ae22", &ae22), \
	PIN_TABLE("ah23", &ah23), PIN_TABLE("af23", &af23), PIN_TABLE("ae23", &ae23), PIN_TABLE("aa25", &aa25), PIN_TABLE("ab28", &ab28), \
	PIN_TABLE("ab26", &ab26), PIN_TABLE("ab25", &ab25), PIN_TABLE("d28", &d28), PIN_TABLE("b27", &b27), PIN_TABLE("j28", &j28), \
	PIN_TABLE("m26", &m26), PIN_TABLE("l25", &l25), PIN_TABLE("l28", &l28), PIN_TABLE("k28", &k28), PIN_TABLE("k26", &k26), \
	PIN_TABLE("m28", &m28), PIN_TABLE("g29", &g29), PIN_TABLE("k29", &k29), PIN_TABLE("l23", &l23), PIN_TABLE("j23", &j23), \
	PIN_TABLE("j26", &j26_cs), PIN_TABLE("k25", &k25_cs), PIN_TABLE("j25", &j25_cs), PIN_TABLE("h28", &h28_cs), PIN_TABLE("g28", &g28_cs), \
	PIN_TABLE("k23", &k23), PIN_TABLE("h25", &h25_cs), PIN_TABLE("g25", &g25_cs), PIN_TABLE("h26", &h26_cs), PIN_TABLE("ah17", &ah17), \
	PIN_TABLE("u21", &u21), PIN_TABLE("r21", &r21), PIN_TABLE("r23", &r23), PIN_TABLE("p21", &p21), PIN_TABLE(NULL, NULL), \
};

#endif
