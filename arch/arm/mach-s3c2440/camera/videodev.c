/*
 * Video capture interface for Linux Character Device Driver.
 *              based on  
 *              Alan Cox, <alan@redhat.com> video4linux 
 *
 * Author:      SW.LEE <hitchcar@samsung.com>        
 *              2004 (C) Samsung Electronics 
 *              Modified for S3C2440/S3C24A0 Interface
 *
 * This file is released under the GPLv2
 */


#include <linux/module.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/smp_lock.h>
#include <linux/mm.h>
#include <linux/string.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/kmod.h>
#include <linux/slab.h>
/* #include <linux/devfs_fs_kernel.h> */
#include <linux/miscdevice.h>
#include <asm/uaccess.h>
#include <asm/system.h>
#include <asm/semaphore.h>



#define CONFIG_VIDEO_V4L1_COMPAT
#include <linux/videodev.h>
#include "camif.h"
#include "miscdevice.h"


static DECLARE_MUTEX(videodev_lock);

const char *fimc_version = "$Id: videodev.c,v 1.1.1.1 2004/04/27 03:52:50 swlee Exp $";

#define VIDEO_NAME              "video4linux"


#define VIDEO_NUM_DEVICES	2
static struct video_device *video_device[VIDEO_NUM_DEVICES];

static inline struct video_device * get_vd(int nr)
{
	if ( nr == CODEC_MINOR)
		return video_device[0];
	else {
		assert ( nr & PREVIEW_MINOR);
		return video_device[1];
	}
}

static inline void set_vd ( struct video_device * vd, int nr)
{
	if ( nr == CODEC_MINOR)
		video_device[0] = vd;
	else {
		assert ( nr & PREVIEW_MINOR);
		video_device[1] = vd;
	}
}

static inline int video_release(struct inode *inode, struct file *f)
{
	int minor = MINOR(inode->i_rdev);
	struct video_device *vfd;

	vfd = get_vd(minor);
#if 1 /* needed until all drivers are fixed */
	if (!vfd->release)
		return 0;
#endif
	vfd->release(vfd);
	return 0;
}

struct video_device* video_devdata(struct file *file)
{
	return video_device[iminor(file->f_dentry->d_inode)];
}


/*
 *	Open a video device.
 */
static int video_open(struct inode *inode, struct file *file)
{
	int minor = MINOR(inode->i_rdev);
	int err = 0;
	struct video_device *vfl;
	struct file_operations const *old_fops;
	
	down(&videodev_lock);

	vfl = get_vd(minor);

	old_fops = file->f_op;
	file->f_op = fops_get(vfl->fops);
	if(file->f_op->open)
		err = file->f_op->open(inode,file);
	if (err) {
		fops_put(file->f_op);
		file->f_op = fops_get(old_fops);
	}
	fops_put(old_fops);
	up(&videodev_lock);
	return err;
}

/*
 * open/release helper functions -- handle exclusive opens
 */
extern int video_exclusive_open(struct inode *inode, struct file *file)
{
	struct  video_device *vfl = get_vd(MINOR(inode->i_rdev));
	int retval = 0;

	mutex_lock(&vfl->lock);
	if (vfl->users) {
		retval = -EBUSY;
	} else {
		vfl->users++;
	}
	mutex_unlock(&vfl->lock);
	return retval;
}

extern int video_exclusive_release(struct inode *inode, struct file *file)
{
	struct  video_device *vfl = get_vd(MINOR(inode->i_rdev));
	vfl->users--;
	return 0;
}

int
video_usercopy(struct inode *inode, struct file *file,
	       unsigned int cmd, unsigned long arg,
	       int (*func)(struct inode *inode, struct file *file,
			   unsigned int cmd, void *arg))
{
	char	sbuf[128];
	void    *mbuf = NULL;
	void	*parg = NULL;
	int	err  = -EINVAL;

	//	cmd = video_fix_command(cmd);

	/*  Copy arguments into temp kernel buffer  */
	switch (_IOC_DIR(cmd)) {
	case _IOC_NONE:
		parg = (void *)arg;
		break;
	case _IOC_READ:
	case _IOC_WRITE:
	case (_IOC_WRITE | _IOC_READ):
		if (_IOC_SIZE(cmd) <= sizeof(sbuf)) {
			parg = sbuf;
		} else {
			/* too big to allocate from stack */
			mbuf = kmalloc(_IOC_SIZE(cmd),GFP_KERNEL);
			if (NULL == mbuf)
				return -ENOMEM;
			parg = mbuf;
		}
		
		err = -EFAULT;
		if (_IOC_DIR(cmd) & _IOC_WRITE)
			if (copy_from_user(parg, (void *)arg, _IOC_SIZE(cmd)))
				goto out;
		break;
	}

	/* call driver */
	err = func(inode, file, cmd, parg);
	if (err == -ENOIOCTLCMD)
		err = -EINVAL;
	if (err < 0)
		goto out;

	/*  Copy results into user buffer  */
	switch (_IOC_DIR(cmd))
	{
	case _IOC_READ:
	case (_IOC_WRITE | _IOC_READ):
		if (copy_to_user((void *)arg, parg, _IOC_SIZE(cmd)))
			err = -EFAULT;
		break;
	}

out:
	if (mbuf)
		kfree(mbuf);
	return err;
}


static struct file_operations video_fops=
{
	.owner		= THIS_MODULE,
	.llseek		= no_llseek,
	.open		= video_open,
	.release        = video_release,
};

static struct miscdevice codec_dev = {
	minor: CODEC_MINOR,
	name : "codec",
	fops : &video_fops
};

static struct miscdevice preview_dev = {
	minor: PREVIEW_MINOR,
	name : "preview",
	fops : &video_fops
};


/**
 *	video_register_device - register video4linux devices
 *	@vfd:  video device structure we want to register
 *	@type: type of device to register
 *	@nr:   minor number 
 *	
 *	Zero is returned on success.
 *      type : ignored.
 *      nr : 
 *           0 Codec index
 *           1 Preview index
 */
int video_register_device(struct video_device *vfd, int type, int nr)
{
	int ret=0;

	/* pick a minor number */
	down(&videodev_lock);
	set_vd (vfd, nr);
	vfd->minor=nr;
	up(&videodev_lock);

	switch (vfd->minor) {
		case CODEC_MINOR:
			ret = misc_register(&codec_dev);
			if (ret) {
				printk(KERN_ERR 
						"can't misc_register : codec on minor=%d\n", CODEC_MINOR);
				panic(" Give me misc codec \n");
			}
			break;
		case PREVIEW_MINOR:
			ret = misc_register(&preview_dev);
			if (ret) {
				printk(KERN_ERR 
						"can't misc_register (preview) on minor=%d\n", PREVIEW_MINOR);
				panic(" Give me misc codec \n");
			}
			break;
	}

#if 0 /* needed until all drivers are fixed */
	if (!vfd->release)
		printk(KERN_WARNING "videodev: \"%s\" has no release callback. "
				"Please fix your driver for proper sysfs support, see "
				"http://lwn.net/Articles/36850/\n", vfd->name);
#endif
	return 0;
}

/**
 *	video_unregister_device - unregister a video4linux device
 *	@vfd: the device to unregister
 *
 *	This unregisters the passed device and deassigns the minor
 *	number. Future open calls will be met with errors.
 */
 
void video_unregister_device(struct video_device *vfd)
{
	down(&videodev_lock);

	if(get_vd(vfd->minor)!=vfd)
		panic("videodev: bad unregister");

	if (vfd->minor== CODEC_MINOR)
		misc_deregister(&codec_dev);
	else
		misc_deregister(&preview_dev);
	set_vd (NULL, vfd->minor);
	up(&videodev_lock);
}


/*
 *	Initialise video for linux
 */
 
static int __init videodev_init(void)
{
//	printk(KERN_INFO "FIMC2.0 Built:"__DATE__" "__TIME__"\n%s\n",fimc_version);
	return 0;
}

static void __exit videodev_exit(void)
{
}

module_init(videodev_init)
module_exit(videodev_exit)

EXPORT_SYMBOL(video_register_device);
EXPORT_SYMBOL(fimc_version);
EXPORT_SYMBOL(video_unregister_device);
EXPORT_SYMBOL(video_usercopy);
EXPORT_SYMBOL(video_exclusive_open);
EXPORT_SYMBOL(video_exclusive_release);


MODULE_AUTHOR("SW.LEE <hitchcar@sec.samsung.com>");
MODULE_DESCRIPTION("VideoDev For FIMC2.0 MISC Drivers");
MODULE_LICENSE("GPL");


/*
 * Local variables:
 * c-basic-offset: 8
 * End:
 */
