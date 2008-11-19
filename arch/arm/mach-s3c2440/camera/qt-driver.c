/*
 * SW.LEE <hitchcar@samsung.com>
 *   
 * This file is subject to the terms and conditions of the GNU General Public
 * License 2. See the file COPYING in the main directory of this archive
 * for more details.
 */

#include <linux/version.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/fs.h>
#include <linux/kernel.h>
#include <linux/major.h>
#include <linux/slab.h>
#include <linux/poll.h>
#include <linux/signal.h>
#include <linux/ioport.h>
#include <linux/sched.h>
#include <linux/types.h>
#include <linux/interrupt.h>
#include <linux/kmod.h>
#include <linux/vmalloc.h>
#include <linux/init.h>
#include <asm/io.h>
#include <asm/page.h>
#include <asm/irq.h>
#include <asm/semaphore.h>
#include <linux/miscdevice.h>
                                                                                                           
//#define SW_DEBUG
                                                                                                           
#include "camif.h"
#include "videodev.h"
#include "miscdevice.h"
#include "cam_reg.h"
#include "sensor.h"
#include "userapp.h"
                                                                                                           

/************************* Sharp Zarus API **************************
* refering to Camera Driver API for SL-5000D/SL-5600 revision 1.00 
*   				April 11, 2002.
  SW.LEE <hitchcar@sec.samsung.com>
   		I want to use Sharp Camera Application. 
*
*/

#define READ_MODE_STATUS	0x1
#define READ_MODE_IMAGE		0x0
#define CAPTURE_SPEED
#define	H_FLIP	
#define V_FLIP
typedef enum sharp_readmode 
{
        IMAGE = 0, STATUS = 1,
        FASTER = 0, BETTER = 2,
        XNOFLIP = 0, XFLIP = 4,
        YNOFLIP = 0, YFLIP = 8,
        AUTOMATICFLIP = -1
} ReadMode_t;


static struct sharp_param_t {
	ReadMode_t readMode;
	char CameraStatus[4];
} sharp_param = { STATUS, {'s','m','c','A'}};


camif_param_t qt_parm = { 640,480,240,320,16,0};
	
static void setReadMode(const char *b,size_t count)
{
	int i =   *(b+2) - 48 ;
	if ( 4 == count ) {
		i = (*(b+3) - 48)  + i * 10;
	}

	//	DPRINTK(" setReadMode %s conversion value %d \n",b , i); 
	if ( i & STATUS ) { 
	//	DPRINTK(" STATUS MODE \n");
		sharp_param.readMode = i;
	}
	else  {
	//	DPRINTK(" IMAGE MODE \n");
		sharp_param.readMode = i;
	}
}




extern ssize_t camif_p_read(struct file *, char *, size_t , loff_t *);

ssize_t z_read(struct file *f, char *buf, size_t count, loff_t *pos)
{
	size_t end;

	if (sharp_param.readMode & STATUS ) {
		buf[0] = sharp_param.CameraStatus[0];
		buf[1] = sharp_param.CameraStatus[1];
		buf[2] = sharp_param.CameraStatus[2];
		buf[3] = sharp_param.CameraStatus[3];
		end = 4;
		return end;
	} 
	else {	/* Image ReadMode */
		/*
		if (( sharp_param.readMode & (BETTER|X  FLIP|YFLIP)))
			DPRINTK("  Not Supporting BETTER|XFLIP|YFLIP\n");
		*/
		return camif_p_read(f,buf,count,pos);
	}
}

static void z_config(camif_cfg_t *cfg,int x, int y)
{
	cfg->target_x = x;
	cfg->target_y = y;
	cfg->fmt = CAMIF_RGB16;
	if (camif_dynamic_open(cfg)) {
		panic(" Eror Happens \n");
	}
}


ssize_t z_write(struct file *f, const char *b, size_t c, loff_t *pos)
{
	int array[5];
	int zoom = 1;
	camif_cfg_t *cfg;

	cfg = get_camif(MINOR(f->f_dentry->d_inode->i_rdev));
//	DPRINTK(" param %s count %d \n",b, c );

	switch(*b) {
	case 'M':	
		setReadMode(b, c);
		break;
	case 'B':	/* Clear the latch flag of shutter button */
		DPRINTK("  clear latch flag of camera's shutter button\n");
		sharp_param.CameraStatus[0]='s';
		break;
	case 'Y':	/* I don't know how to set Shutter pressed */	
		DPRINTK("  set latch flag n");
		sharp_param.CameraStatus[0]='S';
		break;
	case 'S':	/* Camera Image Resolution */
	case 'R':	/* Donot support Rotation */
	DPRINTK(" param %s count %d \n",b, c );
		get_options((char *)(b+2), 5, array);
		if ( array[3] == 512 ) zoom = 2;
		z_config(cfg, array[1] * zoom , array[2] * zoom );
		camif_4fsm_start(cfg);
		break;
	case 'C':
	DPRINTK(" param %s count %d \n",b, c );
		DPRINTK("  Start the camera to capture \n");
		sharp_param.CameraStatus[2]='C';
		camif_4fsm_start(cfg);
		break;
	default:
		printk("Unexpected  param %s count %d \n",b, c );
	}

	return c;
}

