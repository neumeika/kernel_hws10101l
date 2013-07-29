#ifndef ___BQ27510_BATTERY_H
#define __BQ27510_BATTERY_H

#define BQ275X0_NAME "bq27510-battery"

#define BQ275X0_I2C_ADDR (0x55)

#define DRIVER_VERSION "1.0.0"

struct bq27510_platform_data
{
    int irq_gpio;
    int irq_flag;
    int battery_technology;
    int voltage_max_design;
    int voltage_min_design;
    int energy_full_design;
    int energy_empty_design;
};

#endif
