//#ifndef __LINUX_S3C_VIDEODEV_H
//#define __LINUX_S3C_VIDEODEV_H

#include <linux/types.h>
#include <linux/version.h>
#include <media/v4l2-dev.h>

#if 0
struct video_device
{
	/* device info */
	//	struct device *dev;
	char name[32];
	int type;       /* v4l1 */
	int type2;      /* v4l2 */
	int hardware;
	int minor;

	/* device ops + callbacks */
	struct file_operations *fops;
	void (*release)(struct video_device *vfd);


#if 1 /* to be removed in 2.7.x */
	/* obsolete -- fops->owner is used instead */
	struct module *owner;
	/* dev->driver_data will be used instead some day.
	 * Use the video_{get|set}_drvdata() helper functions,
	 * so the switch over will be transparent for you.
	 * Or use {pci|usb}_{get|set}_drvdata() directly. */
	void *priv;
#endif

	/* for videodev.c intenal usage -- please don't touch */
	int users;                     /* video_exclusive_{open|close} ... */
	struct semaphore lock;         /* ... helper function uses these   */
	char devfs_name[64];           /* devfs */
	//	struct class_device class_dev; /* sysfs */
};

#define VIDEO_MAJOR	81

#define VFL_TYPE_GRABBER	0


extern int video_register_device(struct video_device *, int type, int nr);
extern void video_unregister_device(struct video_device *);
extern struct video_device* video_devdata(struct file*);



struct video_picture
{
        __u16   brightness;
        __u16   hue;
        __u16   colour;
        __u16   contrast;
        __u16   whiteness;      /* Black and white only */
        __u16   depth;          /* Capture depth */
        __u16   palette;        /* Palette in use */
#define VIDEO_PALETTE_GREY      1       /* Linear greyscale */
#define VIDEO_PALETTE_HI240     2       /* High 240 cube (BT848) */
#define VIDEO_PALETTE_RGB565    3       /* 565 16 bit RGB */
#define VIDEO_PALETTE_RGB24     4       /* 24bit RGB */
#define VIDEO_PALETTE_RGB32     5       /* 32bit RGB */
#define VIDEO_PALETTE_RGB555    6       /* 555 15bit RGB */
#define VIDEO_PALETTE_YUV422    7       /* YUV422 capture */
#define VIDEO_PALETTE_YUYV      8
#define VIDEO_PALETTE_UYVY      9       /* The great thing about standards is ... */
#define VIDEO_PALETTE_YUV420    10
#define VIDEO_PALETTE_YUV411    11      /* YUV411 capture */
#define VIDEO_PALETTE_RAW       12      /* RAW capture (BT848) */
#define VIDEO_PALETTE_YUV422P   13      /* YUV 4:2:2 Planar */
#define VIDEO_PALETTE_YUV411P   14      /* YUV 4:1:1 Planar */
#define VIDEO_PALETTE_YUV420P   15      /* YUV 4:2:0 Planar */
#define VIDEO_PALETTE_YUV410P   16      /* YUV 4:1:0 Planar */
#define VIDEO_PALETTE_PLANAR    13      /* start of planar entries */
#define VIDEO_PALETTE_COMPONENT 7       /* start of component entries */
};

extern int video_exclusive_open(struct inode *inode, struct file *file);
extern int video_exclusive_release(struct inode *inode, struct file *file);
extern int video_usercopy(struct inode *inode, struct file *file,
                          unsigned int cmd, unsigned long arg,
                          int (*func)(struct inode *inode, struct file *file,
                                      unsigned int cmd, void *arg));



                                                                                                          
#define VID_TYPE_CAPTURE        1       /* Can capture */
#define VID_TYPE_CLIPPING       32      /* Can clip */
#define VID_TYPE_FRAMERAM       64      /* Uses the frame buffer memory */
#define VID_TYPE_SCALES         128     /* Scalable */
#define VID_TYPE_SUBCAPTURE     512     /* Can capture subareas of the image */



#endif
//#endif 

#define VID_HARDWARE_SAMSUNG_FIMC  255

/*
 * Local variables:
 * c-basic-offset: 8
 * End:
 */
