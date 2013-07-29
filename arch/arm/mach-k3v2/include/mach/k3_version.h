#ifndef __MACH_K3_VERSION_H
#define __MACH_K3_VERSION_H

struct s10_gpio_version_platdata
{
    int gpio1;
    int gpio2;
    void (*init) (void);
    void (*exit) (void);
};

struct adc_mv_to_versionidx_map_type
{
   int mV1;
   int mV2;
};
#define DDR_PHYSIZE_LEN 16

#define HW_VERSION_LEN 32
#define VERSION_NUM 64
#define VOL_TOLERANCE 30
#define VERSION_GET_RETRY 5

#endif 
