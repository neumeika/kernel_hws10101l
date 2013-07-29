#ifndef _LINUX_BQ24161_H
#define _LINUX_BQ24161_H

#define BQ24161_NAME "bq24161_charger"
#define   BQ24161_I2C_ADDR (0x6B)

struct bq24161_platform_data
{
    int usb_input_max_currentmA;
    int usb_cdp_input_max_currentmA;
    int usb_dcp_input_max_currentmA;
    int usb_ac_input_max_currentmA;
    int usb_charging_max_currentmA;
    int usb_cdp_charging_max_currentmA;
    int usb_dcp_charging_max_currentmA;
    int usb_ac_charging_max_currentmA;
    int max_charger_voltagemV;
    int chargerUSB_mode_normal_DPM;
    int chargerAC_mode_normal_DPM;
    int termination_currentmA;
    int enbale_gpio;
    int (*bq24161_io_block_config)(int mode);
};

#endif
