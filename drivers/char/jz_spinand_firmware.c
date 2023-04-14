/**
 * 兼容x1830平台，便于应用集成
 */

#include <linux/init.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>
#include <asm/io.h>
#include <linux/cdev.h>

#include "jz_spinand_firmware.h"
#include "sfc_flash.h"


static int fmw_open(struct inode *inode, struct file *filp)
{
    struct cdev *cdev = inode->i_cdev;
    struct fmw_dev *fmw = container_of(cdev, struct fmw_dev, cdev);

    filp->private_data = fmw;

    if(!driver_find(JZ_SFC_NAND, &platform_bus_type)) {
        printk(KERN_ERR "%s %s %d: %s driver module init failed!\n",
            __FILE__, __func__, __LINE__, JZ_SFC_NAND);
        return -ENODEV;
    }

    return 0;
}

static int32_t fmw_read(uint32_t flash_off, void *buf, uint32_t len)
{
    struct mtd_info *mtd = &fmw_flash->mtd;
    uint32_t retlen = 0;
    int ret;

    ret = mtd->_read(mtd, flash_off, len, &retlen, buf);
    return ret;
}

static int32_t fmw_write(uint32_t flash_off, void *buf, uint32_t len)
{
    struct mtd_info *mtd = &fmw_flash->mtd;
    uint32_t retlen = 0;
    int ret;

    ret = mtd->_write(mtd, flash_off, len, &retlen, buf);
    return ret;
}

static int32_t fwm_erase(uint32_t flash_off, uint32_t flash_size)
{
    int i;
    struct mtd_info *mtd = &fmw_flash->mtd;
    struct erase_info info = {
        .addr = flash_off,
        .len  = mtd->erasesize,
    };

    for (i = 0; i < flash_size / mtd->erasesize; i++) {
        mtd->_erase(mtd, &info);
        info.addr += info.len;
    }
    return 0;
}

static int32_t fwm_buf_compare(uint8_t *wbuf, uint8_t *rbuf, uint32_t len)
{
    int ret = memcmp(wbuf, rbuf, len);
    if (ret) {
        printk(KERN_ERR "compare err wbuf do not equal to rbuf.\n");
        return -EIO;
    }
    return 0;
}

static int32_t sn_read_config(struct fmw_dev *fmw)
{

    struct mtd_info *mtd = &fmw_flash->mtd;
    int i,ret;
    int32_t flash_off = CONFIG_MAC_FLASH_SIZE;

    for (i = 0; i < CONFIG_SN_FLASH_SIZE / mtd->erasesize; i++) {
        ret  = fmw_read(mtd->size + flash_off, &fmw->sn_config, sizeof(fmw->sn_config));
        if(ret >= 0 && fmw->sn_config.sn_len != 0 && fmw->sn_config.crc_val != 0)
            break;
        flash_off += mtd->erasesize;
    }

    if(fmw->sn_config.sn_len == -1 ||
        fmw->sn_config.crc_val == -1 ||
        fmw->sn_config.sn_len >= SN_BUF_LEN) {
        printk(KERN_ERR "%s %s %d: flash don`t save sn value!\n",
            __FILE__, __func__, __LINE__);
        return -ENODATA;
    }

    if(i == CONFIG_SN_FLASH_SIZE / mtd->erasesize) {
        printk(KERN_ERR "%s %s %d:flash all blocks ecc error!\n",
            __FILE__, __func__, __LINE__);
            return -EIO;
    }

    return 0;
}

static int32_t sn_read(struct fmw_dev *fmw)
{
    struct mtd_info *mtd = &fmw_flash->mtd;
    uint32_t flash_off = mtd->size + CONFIG_MAC_FLASH_SIZE;
    uint32_t blocksize = mtd->erasesize;
    int32_t ret = 0;
    uint8_t i;

    if(sn_read_config(fmw)) {
        printk(KERN_ERR "%s %s %d:read sn config failed!\n",
            __FILE__, __func__, __LINE__);
        return -EIO;
    }

    for (i = 0; i < CONFIG_SN_FLASH_SIZE / blocksize; i++) {
        ret = fmw_read(flash_off + sizeof(fmw->sn_config), fmw->buf, fmw->sn_config.sn_len);
        if(ret < 0) {
            flash_off += blocksize;
            continue;
        }

        if(local_crc32(0xffffffff, fmw->buf, fmw->sn_config.sn_len) == fmw->sn_config.crc_val)
            break;
        flash_off += blocksize;
    }
    if(i == CONFIG_SN_FLASH_SIZE / blocksize) {
        printk(KERN_ERR "%s %s %d: sn flash all blocks ecc error!\n",
            __FILE__, __func__, __LINE__);
            return -EIO;
    }

    return 0;
}

static int32_t mac_read_config(struct fmw_dev *fmw)
{
    struct mtd_info *mtd = &fmw_flash->mtd;
    int i,ret;
    int32_t flash_off = 0;

    for (i = 0; i < CONFIG_MAC_FLASH_SIZE / mtd->erasesize; i++) {
        ret = fmw_read(mtd->size + flash_off, &fmw->mac_config, sizeof(fmw->mac_config));
        if(ret >= 0 && fmw->mac_config.mac_len != 0 && fmw->mac_config.crc_val != 0)
                break;
        flash_off += mtd->erasesize;

    }

    if(fmw->mac_config.mac_len == -1 ||
        fmw->mac_config.crc_val == -1 ||
        fmw->mac_config.mac_len >= MAC_BUF_LEN) {
        printk(KERN_ERR "%s %s %d: flash don`t save mac value!\n",
            __FILE__, __func__, __LINE__);
        return -ENODATA;
    }

    if(i == CONFIG_MAC_FLASH_SIZE / mtd->erasesize) {
        printk(KERN_ERR "%s %s %d:flash all blocks ecc error!\n",
            __FILE__, __func__, __LINE__);
            return -EIO;
    }

    return 0;
}

static int32_t mac_read(struct fmw_dev *fmw)
{
    struct mtd_info *mtd = &fmw_flash->mtd;
    uint32_t flash_off = mtd->size;
    uint32_t blocksize = mtd->erasesize;
    int32_t ret = 0;
    uint8_t i;

    if(mac_read_config(fmw)) {
        printk(KERN_ERR "%s %s %d:read mac config failed!\n",
            __FILE__, __func__, __LINE__);
        return -EIO;
    }

    for (i = 0; i < CONFIG_MAC_FLASH_SIZE / blocksize; i++) {
        ret = fmw_read(flash_off + sizeof(fmw->mac_config), fmw->buf, fmw->mac_config.mac_len);
        if(ret < 0) {
            flash_off += blocksize;
            continue;
        }

        if(local_crc32(0xffffffff, fmw->buf, fmw->mac_config.mac_len) == fmw->mac_config.crc_val)
            break;
        flash_off += blocksize;
    }

    if(i == CONFIG_MAC_FLASH_SIZE / blocksize) {
        printk(KERN_ERR "%s %s %d: mac flash all blocks ecc error!\n",
            __FILE__, __func__, __LINE__);
            return -EIO;
    }

    return 0;
}

static int32_t license_read_config(struct fmw_dev *fmw)
{
    struct mtd_info *mtd = &fmw_flash->mtd;
    int i, ret;
    int32_t flash_off = CONFIG_MAC_FLASH_SIZE + CONFIG_SN_FLASH_SIZE;

    for (i = 0; i < CONFIG_LICENSE_FLASH_SIZE / mtd->erasesize; i++) {
        ret = fmw_read(mtd->size + flash_off, &fmw->license_config, sizeof(fmw->license_config));
        if (ret >= 0 && fmw->license_config.license_len != 0 && fmw->license_config.crc_val != 0) {
            break;
        }
        flash_off += mtd->erasesize;
    }
    if (fmw->license_config.license_len == -1 ||
        fmw->license_config.crc_val == -1 ||
        fmw->license_config.license_len >= LICENSE_BUF_LEN) {
            pr_err("%s %s %d: flash don't save license value!\n",
                    __FILE__, __func__, __LINE__);
            return -ENODATA;
    }

    if (i == CONFIG_LICENSE_FLASH_SIZE / mtd->erasesize) {
        pr_err("%s %s %d:flash all blocks ecc error!\n",
                __FILE__, __func__, __LINE__);
        return -EIO;
    }
    return 0;
}

static int32_t license_read(struct fmw_dev *fmw)
{
    struct mtd_info *mtd = &fmw_flash->mtd;
    uint32_t flash_off = mtd->size + CONFIG_MAC_FLASH_SIZE + CONFIG_SN_FLASH_SIZE;
    uint32_t blocksize = mtd->erasesize;
    int32_t ret = 0;
    uint32_t i;

    if (license_read_config(fmw)) {
        pr_err("%s %s %d:license config failed\n",
                __FILE__, __func__, __LINE__);
        return -EIO;
    }

    for (i = 0; i < CONFIG_LICENSE_FLASH_SIZE / blocksize; i++) {
        ret = fmw_read(flash_off + sizeof(fmw->license_config), fmw->buf, fmw->license_config.license_len);
        if (ret < 0) {
            flash_off += blocksize;
            continue;
        }
        if(local_crc32(0xffffffff, fmw->buf, fmw->license_config.license_len) == fmw->license_config.crc_val)
            break;
        flash_off += blocksize;
    }

    if(i == CONFIG_LICENSE_FLASH_SIZE / blocksize) {
        printk(KERN_ERR "%s %s %d: license flash all blocks ecc error!\n",
            __FILE__, __func__, __LINE__);
        return -EIO;
    }

    return 0;
}

static int32_t license_write(struct fmw_dev *fmw)
{
    int i;
    uint8_t errcount = 0;
    struct mtd_info *mtd = &fmw_flash->mtd;
    uint32_t flash_offs = mtd->size + CONFIG_MAC_FLASH_SIZE + CONFIG_SN_FLASH_SIZE;
    uint32_t flash_size = CONFIG_LICENSE_FLASH_SIZE;
    int32_t ret = 0;
    uint8_t *rbuf;

    void *buf = kzalloc(sizeof(fmw->license_config) + fmw->license_config.license_len, GFP_KERNEL);
    if(!buf) {
        printk(KERN_ERR "alloc mem failed!\n");
        return -ENOMEM;
    }

    memcpy(buf, &fmw->license_config, sizeof(fmw->license_config));
    memcpy(buf + sizeof(fmw->license_config), fmw->buf, fmw->license_config.license_len);

    fwm_erase(flash_offs, flash_size);
    for (i = 0; i < flash_size / mtd->erasesize; i++) {
        ret = fmw_write(flash_offs, buf, sizeof(fmw->license_config) + fmw->license_config.license_len);
        if(ret) {
            printk(KERN_ERR "%s %s %d:write data failed! errcount = %d\n",
                    __FILE__, __func__, __LINE__, errcount++);
            flash_offs += mtd->erasesize;
            continue;
        }

        rbuf = kzalloc(sizeof(fmw->license_config) + fmw->license_config.license_len, GFP_KERNEL);
        if (rbuf) {
            ret = fmw_read(flash_offs, rbuf, sizeof(fmw->license_config) + fmw->license_config.license_len);
            if (!ret) {
                if ((ret = fwm_buf_compare(buf, rbuf, sizeof(fmw->license_config) + fmw->license_config.license_len))) {
                    printk(KERN_ERR "%s %s %d: buf compare err!\n", __FILE__, __func__, __LINE__);
                }
            }
            kfree(rbuf);
        }
    }
    if(errcount == flash_size / mtd->erasesize) {
        printk(KERN_ERR "all blk write failed!\n");
        kfree(buf);
        return -EIO;
    }

    kfree(buf);
    return ret;
}


static long fmw_ioctl(struct file *filp, unsigned int cmd, unsigned long args)
{
    struct fmw_dev *fmw = filp->private_data;
    uint32_t max_len;
    int32_t ret = 0;

    switch(cmd) {
        case SN_GET_CONFIG:
            if((ret = sn_read_config(fmw))) {
                printk(KERN_ERR "%s %s %d: sn get config failed, ret = %d\n",
                    __FILE__, __func__, __LINE__, ret);
                break;
            }
            copy_to_user((void *)args, &fmw->sn_config, sizeof(fmw->sn_config));
            break;
        case SN_READ:
            if((ret = sn_read(fmw))) {
                printk(KERN_ERR "%s %s %d: sn_read failed, ret = %d\n",
                    __FILE__, __func__, __LINE__, ret);
                break;
            }
            copy_to_user((void *)args, fmw->buf, fmw->sn_config.sn_len);
            break;

        case MAC_GET_CONFIG:
            if((ret = mac_read_config(fmw))) {
                printk(KERN_ERR "%s %s %d: mac get config failed, ret = %d\n",
                    __FILE__, __func__, __LINE__, ret);
                break;
            }
            copy_to_user((void *)args, &fmw->mac_config, sizeof(fmw->mac_config));
            break;
        case MAC_READ:
            if((ret = mac_read(fmw))) {
                printk(KERN_ERR "%s %s %d: mac_read failed, ret = %d\n",
                    __FILE__, __func__, __LINE__, ret);
                break;
            }
            copy_to_user((void *)args, fmw->buf, fmw->mac_config.mac_len);
            break;

        case LICENSE_GET_CONFIG:
            if (license_read_config(fmw)) {
                pr_err("%s %s %d:license get config failed, ret = %d\n",
                       __FILE__, __func__, __LINE__, ret);
                break;
            }
            copy_to_user((void *)args, &fmw->license_config, sizeof(fmw->license_config));
            break;
        case LICENSE_MAX_LEN:
            max_len = LICENSE_BUF_LEN;
            copy_to_user((void *)args, &max_len, sizeof(uint32_t));
            break;
        case LICENSE_READ:
            if (license_read(fmw)) {
                pr_err("%s %s %d:license read failed, ret = %d\n",
                     __FILE__, __func__, __LINE__, ret);
                break;
            }
            copy_to_user((void *)args, fmw->buf, fmw->license_config.license_len);
            break;
        case LICENSE_WRITE:
            if((copy_from_user(&fmw->license_config, (void *)args, sizeof(fmw->license_config)))) {
                printk(KERN_ERR "%s %s %d: get license_config error, ret = %d\n",
                    __FILE__, __func__, __LINE__, ret);
                break;
            }
            fmw->buf = kzalloc(fmw->license_config.license_len, GFP_KERNEL);
            if (!fmw->buf) {
                printk(KERN_ERR "%s %s %d: alloc fmw buf error.\n", __FILE__, __func__, __LINE__);
                return -ENOMEM;
            }
            if((ret = copy_from_user(fmw->buf, (char*)args+sizeof(struct license_config), fmw->license_config.license_len))) {
                printk(KERN_ERR "%s %s %d: get license data error, ret = %d\n",
                    __FILE__, __func__, __LINE__, ret);
                break;
            }
            if((ret = license_write(fmw))) {
                printk(KERN_ERR "%s %s %d: license write failed, ret = %d\n",
                    __FILE__, __func__, __LINE__, ret);
                break;
            }
            break;

        default:
            printk(KERN_ERR "%s %s %d: don`t support this cmd!\n", __FILE__, __func__, __LINE__);
            ret = -EINVAL;
    }
    return ret;
}

static int fmw_close (struct inode * inode, struct file *filp)
{
    return 0;

}

static struct file_operations fmw_ops = {

    .owner = THIS_MODULE,
    .open = fmw_open,
    .release = fmw_close,
    .unlocked_ioctl = fmw_ioctl,
};

static int __init fmw_init(void)
{
    dev_t dev;
    int32_t ret = 0;

    struct fmw_dev *fmw = kzalloc(sizeof(*fmw), GFP_KERNEL);
    if(!fmw) {
        printk(KERN_ERR "%s %s %d :alloc sn failed!\n",
            __FILE__, __func__, __LINE__);
        return -ENOMEM;
    }

    fmw->buf = kzalloc(SN_BUF_LEN, GFP_KERNEL);
    if(!fmw->buf) {
        printk(KERN_ERR "%s %s %d: alloc sn buf failed!\n", __FILE__, __func__, __LINE__);
        kfree(fmw);
        return -ENOMEM;
    }

    fmw->class = class_create(THIS_MODULE, "jz-spinand-sn");
    fmw->minor = 0;
    fmw->nr_devs = 1;

    if(alloc_chrdev_region(&dev, fmw->minor, fmw->nr_devs, "jz-spinand-fmw")) {

        printk(KERN_ERR "%s %s %d: alloc chrdev failed!\n",
            __FILE__, __func__, __LINE__);
        ret = -ENODEV;
        goto failed;
    }

    cdev_init(&fmw->cdev, &fmw_ops);
    fmw->cdev.owner = THIS_MODULE;
    cdev_add(&fmw->cdev, dev, fmw->nr_devs);

    fmw->dev = device_create(fmw->class, NULL, dev, NULL, "jz-spinand-fmw");
    if(IS_ERR_OR_NULL(fmw->dev)) {
        printk(KERN_ERR "%s %s %d :creates a device and register sysfs failed !\n",
            __FILE__, __func__, __LINE__);
        ret = -ENODEV;
        goto free_all;
    }

    return 0;

free_all:
    unregister_chrdev_region(dev, fmw->nr_devs);
failed:
    kfree(fmw->buf);
    kfree(fmw);
    return ret;
}

module_init(fmw_init);
MODULE_AUTHOR("zhangronghua<rhzhang@ingenic.com>");
MODULE_DESCRIPTION("sn speace read driver");
MODULE_LICENSE("GPL");
