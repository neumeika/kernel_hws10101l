#ifndef _LINUX_MAX8903_H
#define _LINUX_MAX8903_H

#ifdef CONFIG_FSA880_I2C
 #define RECOGNISE_CHARGER_TYPE_BY_EXTRAL_IC
#else
 #undef  RECOGNISE_CHARGER_TYPE_BY_EXTRAL_IC
#endif

#define MAX8903_NAME "max8903_charger"

#define CHARGING_CURRENT_LEVEL0 500
#define CHARGING_CURRENT_LEVEL1 1500
#define CHARGING_CURRENT_LEVEL2 2000

#ifdef RECOGNISE_CHARGER_TYPE_BY_EXTRAL_IC
 #define CHG_USB_CDP_INSERT_EVENT (0x0001)
 #define CHG_USB_DCP_INSERT_EVENT (0x0002)
 #define CHG_USB_SDP_INSERT_EVENT (0x0003)
 #define     CHG_BOTTOM_EXIST_EVENT (0X0004)
 #define     CHG_REMOVED_EVENT (0x0005)
 #define     CHG_AC_INSERT_EVENT (0x0006)
 #define     CHG_AC_REMOVED_EVENT (0x0007)
#endif

struct max8903_platform_data
{
    /*
     * GPIOs
     * cen, chg, flt, and usus are optional.
     * dok, dcm, and uok are not optional depending on the status of
     * dc_valid and usb_valid.
     */
    int cen;    /* Charger Enable input */
    int dok;    /* DC(Adapter) Power OK output */
    int uok;    /* USB Power OK output */
    int chg;    /* Charger status output */
    int flt;    /* Fault output */
    int dcm;    /* Current-Limit Mode input (1: DC, 2: USB) */
    int usus;   /* USB Suspend Input (1: suspended) */

    /*
     * DC(Adapter/TA) is wired
     * When dc_valid is true,
     *	dok and dcm should be valid.
     *
     * At least one of dc_valid or usb_valid should be true.
     */
    bool dc_valid;

    /*
     * USB is wired
     * When usb_valid is true,
     *	uok should be valid.
     */
    bool usb_valid;

    int dcm_ctrl0;  // s10 use dcm_ctrl0 and dcm_ctrl1 control dcm output value
    int dcm_ctrl1;  // s10 use dcm_ctrl0 and dcm_ctrl1 control dcm output value

    int usb_max_current;
    int usb_cdp_max_current;
    int usb_dcp_max_current;
    int usb_ac_max_current;
    char io_block_name[20];
    int (*io_mux_block_init)(struct max8903_platform_data *pdata);
    int (*io_mux_block_exit)(struct max8903_platform_data *pdata);
    int (*gpio_block_init)(struct max8903_platform_data *pdata);
    int (*gpio_block_exit)(struct max8903_platform_data *pdata);
};

#endif
