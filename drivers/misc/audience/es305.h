#ifndef AUDIENCE_ES305_H
#define AUDIENCE_ES305_H

#include <linux/ioctl.h>

#define ES305_NAME  "audience_es305"

struct es305_platform_data {
    uint32_t gpio_es305_wakeup;
    uint32_t gpio_es305_reset;
};

struct es305_img {
    unsigned char *buf;
    unsigned img_size;
};

/* path ids */
enum ES305_PATHID {
    ES305_PATH_NO_INIT              = 0,
    ES305_PATH_BYPASS               = 1,
    ES305_PATH_NB_CALL_RECEIVER     = 2,
    ES305_PATH_NB_CALL_HEADSET      = 3,
    ES305_PATH_NB_CALL_SPEAKER      = 4,
    ES305_PATH_NB_CALL_BT           = 5,
    ES305_PATH_WB_CALL_RECEIVER     = 6,
    ES305_PATH_WB_CALL_HEADSET      = 7,
    ES305_PATH_WB_CALL_SPEAKER      = 8,
    ES305_PATH_WB_CALL_BT           = 9,
    ES305_PATH_NB_VOIP_RECEIVER     = 10,
    ES305_PATH_NB_VOIP_HEADSET      = 11,
    ES305_PATH_NB_VOIP_SPEAKER      = 12,
    ES305_PATH_NB_VOIP_BT           = 13,
    ES305_PATH_WB_VOIP_RECEIVER     = 14,
    ES305_PATH_WB_VOIP_HEADSET      = 15,
    ES305_PATH_WB_VOIP_SPEAKER      = 16,
    ES305_PATH_WB_VOIP_BT           = 17,
};

/* max size of firmware */
#define ES305_MAX_FW_SIZE ( 100 * 1024 ) /* 100K */

/* IOCTLs for Audience ES305 */
#define ES305_IOCTL_MAGIC 'u'

#define ES305_CMD_RESET    _IO(ES305_IOCTL_MAGIC,  0x00)
#define ES305_CMD_SYNC     _IO(ES305_IOCTL_MAGIC,  0x01)
#define ES305_CMD_SLEEP    _IO(ES305_IOCTL_MAGIC,  0x02)

#define ES305_I2C_DOWNLOAD _IOW(ES305_IOCTL_MAGIC, 0x03, struct es305_img*)
#define ES305_SET_PATHID   _IOW(ES305_IOCTL_MAGIC, 0x04, enum ES305_PATHID)
#define ES305_SET_NS       _IOW(ES305_IOCTL_MAGIC, 0x05, unsigned char)

/* For Diag */
#define ES305_WRITE_MSG    _IOW(ES305_IOCTL_MAGIC, 0x10, unsigned)
#define ES305_READ_DATA    _IOR(ES305_IOCTL_MAGIC, 0x11, unsigned)

/* commands */
/* sync polling mode */
#define AUDIENCE_MSG_SYNC               ( 0x80000000 )

/* sleep mode */
#define AUDIENCE_MSG_SLEEP_MODE         ( 0x80100001 )

/* reset */
#define AUDIENCE_MSG_RESET_IMMEDIATE    ( 0x80020000 )
#define AUDIENCE_MSG_RESET_DELAYED      ( 0x80020001 )

/* bootload initiate */
#define AUDIENCE_MSG_BOOTLOADINITIATE   ( 0x80030000 )

#define AUDIENCE_MSG_BOOT_BYTE1         ( 0x00 )
#define AUDIENCE_MSG_BOOT_BYTE2         ( 0x01 )
#define AUDIENCE_MSG_BOOT_ACK           ( 0x01 )

/******************************************************************************/
/* device parameter */
#define AUDIENCE_MSG_GETDEVICEPARM      ( 0x800B )
#define AUDIENCE_MSG_SETDEVICEPARMID    ( 0x800C )
#define AUDIENCE_MSG_SETDEVICEPARM      ( 0x800D )

/* port ids */
#define AUDIENCE_PORT_A                 ( 0x0A )
#define AUDIENCE_PORT_B                 ( 0x0B )
#define AUDIENCE_PORT_C                 ( 0x0C )
#define AUDIENCE_PORT_D                 ( 0x0D )

/* parameters */
#define WORD_LENGTH                     ( 0x00 )
#define DEL_FROM_FS_TX                  ( 0x02 )
#define DEL_FROM_FS_RX                  ( 0x03 )
#define LATCH_EDGE                      ( 0x04 )
#define ENDIANNESS                      ( 0x05 )
#define TRISTATE                        ( 0x06 )
#define PORT_MODE                       ( 0x07 )

/* parameters values */
#define WORDLENGTH_16BIT                ( 0x0F ) /* default */
#define WORDLENGTH_24BIT                ( 0x17 )

#define LATCH_EDGE_TX_F_RX_R            ( 0x00 )
#define LATCH_EDGE_TX_R_RX_F            ( 0x03 ) /* default */

#define ENDIANNESS_LITTLE               ( 0x00 )
#define ENDIANNESS_BIG                  ( 0x01 ) /* default */

#define TRISTATE_DISABLE                ( 0x00 ) /* default */
#define TRISTATE_ENABLE                 ( 0x01 )

/* algorithm algorithmmeter */
#define AUDIENCE_MSG_GETALGORITHMPARM   ( 0x8016 )
#define AUDIENCE_MSG_SETALGORITHMPARMID ( 0x8017 )
#define AUDIENCE_MSG_SETALGORITHMPARM   ( 0x8018 )

/******************************************************************************/

/* general definitions */
#define AUDIENCE_RESET_STABLE_TIME      ( 1  )  /* ms / keep 4 cycles in clk */
#define AUDIENCE_SW_RESET_TIME          ( 20 )  /* ms */
#define AUDIENCE_I2C_ACCESS_DELAY_TIME  ( 50 )  /* ms */
#define AUDIENCE_BOOT_ACK_TIME          ( 1  )  /* ms */
#define AUDIENCE_SLEEP_TIME             ( 20 )  /* ms */
#define AUDIENCE_BOOT_SYNC_TIME         ( 120 ) /* ms */

#define AUDIENCE_SYNC_TIME              (  1000 )  /* us */
#define AUDIENCE_POLLING_TIME           ( 20000 )  /* us */
#define AUDIENCE_WAKEUP_H_TIME          (  1000 )  /* us */
#define AUDIENCE_WAKEUP_L_TIME          ( 30000 )  /* us */

#define AUDIENCE_ACK_ERROR              ( 0xFF )
#define AUDIENCE_ACK_NONE               ( 0x00 )

#define RETRY_TIMES                     ( 5 )
#define POLLING_RETRY_TIMES             ( 3 )
#define ES305_CMD_FIFO_DEPTH            ( 128 )

#define AUDIENCE_SW_RESET_OK            ( 0x81 )
#define AUDIENCE_SW_RESET_ERROR         ( 0x82 )

#endif //AUDIENCE_ES305_H