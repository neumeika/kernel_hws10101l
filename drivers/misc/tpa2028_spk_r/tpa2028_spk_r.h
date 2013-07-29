#ifndef TPA2028_SPK_R_H
#define TPA2028_SPK_R_H

#define TPA2028_R_NAME      "tpa2028_r"
#define TPA2028_I2C_ADDR    ( 0x58 )

#define REG_FUNCTION_CTL    (1)
#define REG_ATTACK_CTL      (2)
#define REG_RELEASE_CTL     (3)
#define REG_HOLD_CTL        (4)
#define REG_FIXGAIN_CTL     (5)
#define REG_AGC_CTL1        (6)
#define REG_AGC_CTL2        (7)
#define MAX_18_RATIO_4_1    (0x2)
#define LIMIT_LEVEL_7_5     (0x3c)
#define TPA2028_IOCTL_MAGIC 'u'

#define TPA2028_ENABLE      _IOW(TPA2028_IOCTL_MAGIC, 0xC0, unsigned)
#define TPA2028_DISABLE     _IOW(TPA2028_IOCTL_MAGIC, 0xC1, unsigned)
#define TPA2028_SET_REG     _IOW(TPA2028_IOCTL_MAGIC, 0xC2, unsigned)
#define TPA2028_GET_REG     _IOR(TPA2028_IOCTL_MAGIC, 0xC3, unsigned)
#define TPA2028_EN_PWR      _IOR(TPA2028_IOCTL_MAGIC, 0xC4, unsigned)
#define TPA2028_DIS_PWR     _IOR(TPA2028_IOCTL_MAGIC, 0xC5, unsigned)

struct tpa2028_r_platform_data {
    uint32_t gpio_tpa2028_en;
};

#endif /* TPA2028_SPK_R_H */