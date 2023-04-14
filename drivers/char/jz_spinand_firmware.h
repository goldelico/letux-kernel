#ifndef __JZ_SPINAND_FIRMWARE_H
#define __JZ_SPINAND_FIRMWARE_H
#include <linux/types.h>
#include <linux/ioctl.h>
#include "fmw.h"

#define JZ_SFC_NAND      "ingenic-sfc"
#define SN_BUF_LEN       512
#define MAC_BUF_LEN      512
#define LICENSE_BUF_LEN  CONFIG_LICENSE_FLASH_SIZE


#define SN_GET_CONFIG       _IOR('S', 0x0, uint32_t)
#define SN_READ             _IOR('S', 0x1, uint32_t)
#define MAC_GET_CONFIG      _IOR('S', 0x2, uint32_t)
#define MAC_READ            _IOR('S', 0x3, uint32_t)
#define LICENSE_GET_CONFIG  _IOR('S', 0x4, uint32_t)
#define LICENSE_READ        _IOR('S', 0x5, uint32_t)
#define LICENSE_WRITE       _IOR('S', 0x6, uint32_t)
#define LICENSE_MAX_LEN     _IOR('S', 0x7, uint32_t)

struct sn_config {
    uint32_t sn_len;
    uint32_t crc_val;
};

struct mac_config {
    uint32_t mac_len;
    uint32_t crc_val;
};

struct license_config {
    uint32_t license_len;
    uint32_t crc_val;
};

struct fmw_dev {

    uint8_t minor;
    uint8_t nr_devs;

    struct class *class;
    struct cdev cdev;
    struct device *dev;

    uint8_t *buf;

    /*mac*/
    struct mac_config mac_config;

    /*sn*/
    struct sn_config sn_config;

    /*license*/
    struct license_config license_config;
};

extern struct bus_type platform_bus_type;
extern struct sfc_flash *fmw_flash;

#endif


