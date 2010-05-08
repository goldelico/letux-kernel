/*
 * linux/drivers/char/jzchar/minipc-misc.c
 *
 * Power off handling.
 *
 * Copyright unknown
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 *
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/ioport.h>
#include <linux/fcntl.h>
#include <linux/miscdevice.h>
#include <linux/init.h>
#include <linux/poll.h>
#include <linux/spinlock.h>
#include <linux/proc_fs.h>
#include <linux/reboot.h>
#include <linux/input.h>
#include <linux/interrupt.h>

#include <linux/rtc.h>			/* get the user-level API */

#include <asm/reboot.h>
#include <asm/system.h>
#include <asm/jzsoc.h>

// should be included somewhere...

#define MCU_QUERY_BAT   _IOR('p', 0x11, unsigned long) /*query mcu-lpc915 of the baterry value*/
#define SCHEDULE_POWEROFF _IOR('p', 0x12, unsigned long) /*edwin*/

extern void (*_machine_restart)(char *command);
extern void (*_machine_halt)(void);
extern void (*pm_power_off)(void);
#define _machine_power_off pm_power_off	// renamed from 2.4 to 2.6 (?)

#if 1
#define MINIPC_CHAR_MAJOR   239

#define MINIPC_CHAR_POWER_SWITCH 0

const static char power_down_cmd[] = "powerdown";
#define POWER_DOWN_CMD_LEN (sizeof(power_down_cmd)-1)

static wait_queue_head_t wait_power_sw;

static int power_off_pending = 0;
static void pic_shutdown (void);

/*edwin for comunicate with the mcu for bat status*/
#define MCU_DEV_ADDR (0x50>>1)
#define I2C_CLK      10000
#define BAT_STAT_REG    0xDB    
#define BAT_CHARGE_REG  0XD9
static int get_battery_stat(unsigned char *data, unsigned char reg)
{
	int nr;

	i2c_open();

	i2c_setclk(I2C_CLK);
	nr = i2c_read(MCU_DEV_ADDR, data, reg, 1);
	i2c_close();
	return nr;
}
/**/
int mcu_exist(void)
{
	int nr;
	unsigned char data;

	i2c_open();

	i2c_setclk(I2C_CLK);	
	nr=i2c_read(MCU_DEV_ADDR, &data, BAT_CHARGE_REG, 1);
	i2c_close();
	return nr;     
}

static int minipc_char_open(struct inode *inode, struct file *filp)
{
	unsigned char minor = MINOR(inode->i_cdev->dev);
        filp->private_data = (void*)(unsigned long)minor;
	return 0;
}

static int minipc_char_release(struct inode *inode, struct file *filp)
{
	return 0;
}

static int minipc_char_ioctl(struct inode *inode, struct file *filp, unsigned int cmd, unsigned long arg)
{
     unsigned char data;
     int usr_time;
     time_t timeout;
     if( cmd == MCU_QUERY_BAT)
     {	  
	  if(get_battery_stat(&data, BAT_CHARGE_REG) >= 0){
	       copy_to_user((char *)arg, &data, sizeof(data));
	       return sizeof(data);
	  }
     }
     if(cmd == SCHEDULE_POWEROFF){
	  copy_from_user(&usr_time, (int *)arg, sizeof(usr_time));
	  if(usr_time<=0)
	       return -EINVAL;
	  timeout = usr_time*100;//(jiffies=usr_time*1000(ms)/10(ms))
	  set_current_state(TASK_UNINTERRUPTIBLE);
	  printk("slepp %d jiffies before shutdown\n", (int) timeout);
	  schedule_timeout(timeout);
	  current->state = TASK_RUNNING;
	  pic_shutdown();
     }
	return -EINVAL;
}

static ssize_t minipc_char_read(struct file *filp, char *buf, size_t count, loff_t *ppos)
{
        unsigned char minor = (unsigned long)filp->private_data;
	unsigned char data;

        if (MINIPC_CHAR_POWER_SWITCH == minor)
        {
	     /*
                int err = count;
                while (1)
                {
                        interruptible_sleep_on(&wait_power_sw);

                        if (signal_pending(current))
                        {
                                err = -EAGAIN;
                                break;
                        }
                        if (power_off_pending)
                            break;
                        schedule();
                }
                return err;
	     */
	     if(get_battery_stat(&data, BAT_STAT_REG) < 0)
		  return -EBUSY;
	     copy_to_user(buf, &data, sizeof(data));
	     return sizeof(data);
	     
        }
        return -EINVAL;
}

static ssize_t minipc_char_write(struct file *filp, const char *buf, size_t count, loff_t *ppos)
{
        unsigned char minor = (unsigned long)filp->private_data;

        if (MINIPC_CHAR_POWER_SWITCH == minor && count)
        {
                unsigned char tmp[POWER_DOWN_CMD_LEN];

                if (count >= POWER_DOWN_CMD_LEN)// && copy_from_user(tmp, buf, POWER_DOWN_CMD_LEN))
                {
		     copy_from_user(tmp, buf, POWER_DOWN_CMD_LEN);//edwin		     
                        if (0 == memcmp (tmp, power_down_cmd, POWER_DOWN_CMD_LEN))
                        {
                                pic_shutdown(); 
                        }
                }
                return -EINVAL;
        }
	return 0;
}

static unsigned int minipc_char_poll(struct file* filp, poll_table* wait)
{
        unsigned char minor = (unsigned long)filp->private_data;

        if (MINIPC_CHAR_POWER_SWITCH == minor)
        {
                poll_wait(filp, &wait_power_sw, wait);
                return 0;
        }
        return -EINVAL;
}

static struct file_operations minipc_char_fops = {
	open:	minipc_char_open,
	read:	minipc_char_read,
	write:	minipc_char_write,
	ioctl:	minipc_char_ioctl,
	release:minipc_char_release,
        poll:	minipc_char_poll,
};

#define PIC_GPIO (65)   //wjx 2007.12.10 89 -> 65
#define PIC_IRQ  (IRQ_GPIO_0 + PIC_GPIO)

#define LPC915 1
#ifdef LPC915 //edwin
#define SHUTDOWN_REG 0xD8
static void pic_shutdown (void)
{
	int nr;
	unsigned char data = 1;
	printk("pic_shutdown\n");
#if 0	
	while(i2c_open()== 0)
	     break;
#else
	i2c_open();		//wjx
#endif	     
	i2c_setclk(I2C_CLK);
	nr = i2c_write(MCU_DEV_ADDR, &data, SHUTDOWN_REG, 1);	
	i2c_close();
}
#else
static void pic_shutdown (void)
{
        unsigned long flags;

        printk ("MiniPC Power Off!\n");
        mdelay (200);
        local_irq_save(flags);
        __gpio_as_output(PIC_GPIO);
        __gpio_clear_pin(PIC_GPIO);
        mdelay(200);
        __gpio_set_pin(PIC_GPIO);
        mdelay(50);
        __gpio_as_input(PIC_GPIO);
        local_irq_restore(flags);
}
#endif

static irqreturn_t pic_irq(int irq, void *dev_id)
{
        power_off_pending ++;
        wake_up_interruptible (&wait_power_sw);
	msleep(500);
		return IRQ_HANDLED;
}

static int power_down_read_proc (char *page, char **start, off_t off,
			  int count, int *eof, void *data)
{
	int len = 0;

	len += sprintf (page+len, "Write me '%s' to power down.\n", power_down_cmd);
        return len;
}

static int power_down_write_proc(struct file *file, const char *buffer, unsigned long count, void *data)
{
        if (count >= POWER_DOWN_CMD_LEN)
        {
                if (0 == memcmp (power_down_cmd, buffer, POWER_DOWN_CMD_LEN))
                        pic_shutdown ();
        }
        return count;
}

static struct proc_dir_entry *proc_powerdown;

static void (*_old_machine_power_off)(void);

static int __init pic_init(void)
{
        int retval;

        __gpio_as_irq(PIC_GPIO, 2); //falling-edge 

        init_waitqueue_head(&wait_power_sw);

        proc_powerdown = create_proc_entry("powerdown", 0644, NULL);
        if (proc_powerdown) {
                proc_powerdown->read_proc = power_down_read_proc;
                proc_powerdown->write_proc = power_down_write_proc;
                proc_powerdown->data = NULL;
        }

        register_chrdev(MINIPC_CHAR_MAJOR, "MiniPC", &minipc_char_fops);

 //       retval = request_irq (PIC_IRQ, pic_irq, 0, "Power switch", NULL);
	retval=0;
		if (retval)
                printk ("Cannot get Power SW irq %d\n", retval);
        else
        {
                _old_machine_power_off = _machine_power_off;
                _machine_power_off = pic_shutdown;
        }
        printk ("MiniPC Power button initialized.\n");
        return retval;
}

static void __exit pic_exit(void)
{
        free_irq(PIC_IRQ, NULL);
        if (proc_powerdown)
                remove_proc_entry("powerdown", NULL);
        if (_old_machine_power_off)
                _machine_power_off = _old_machine_power_off;

        unregister_chrdev(MINIPC_CHAR_MAJOR, "MiniPC");
}


module_init(pic_init);
module_exit(pic_exit);

#else

// this is all disabled up to EOF
// I think we can remove it to cleanup the code

#define PIC_DEVADDR (0x31)

extern void i2c_open(void);
extern void i2c_close(void);
extern int i2c_read(unsigned char device, unsigned char *buf,
		     unsigned char address, int count);
extern int i2c_write(unsigned char device, unsigned char *buf,
		      unsigned char address, int count);

extern int i2c_read_na(unsigned char device, unsigned char *buf, int count);
extern int i2c_write_na(unsigned char device, unsigned char *buf, int count);
extern void i2c_setclk(unsigned int i2cclk);


#define I2C_FRAME_LEN 2


static unsigned char i2c_recv[I2C_FRAME_LEN];
static unsigned long recv_tick;

static unsigned char i2c_send[I2C_FRAME_LEN];


static void read_pic(void)
{
    int nr;
    unsigned char buff[I2C_FRAME_LEN];
    
    i2c_open ();
    //wjx i2c_setclk(1000);
    nr = i2c_read_na (PIC_DEVADDR, buff, I2C_FRAME_LEN);
    i2c_close();
    
    if (nr == I2C_FRAME_LEN)
    {
        memcpy (i2c_recv, buff, I2C_FRAME_LEN);
        recv_tick = jiffies;
    }
}

static void write_pic(void)
{
    int nr;
    
    i2c_open();
    //wjx i2c_setclk(1000);
    nr = i2c_write_na(PIC_DEVADDR, i2c_send, I2C_FRAME_LEN);
    i2c_close();
}

/* RTC */
extern spinlock_t rtc_lock;
static int rtc_status = 0;
static unsigned int epoch = 1900;

static void get_rtc_time(struct rtc_time* rtc_tm)
{
    
}

static int set_rtc_time(struct rtc_time* rtc_tm)
{
    return 0;
}

static int rtc_ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg)
{
        struct rtc_time wtime, rtc_tm;

        switch (cmd)
        {
        case RTC_RD_TIME:
                get_rtc_time(&wtime);
                break;
        case RTC_SET_TIME:
                if (!capable(CAP_SYS_TIME))
                        return -EACCES;
                if (copy_from_user(&rtc_tm, (struct rtc_time*)arg, sizeof(struct rtc_time)))
                        return -EFAULT;
                return set_rtc_time(&rtc_tm);
        case RTC_EPOCH_READ:
                return put_user(epoch, (unsigned long*)arg);
        case RTC_EPOCH_SET:
                if (arg < 1900)
                        return -EINVAL;
                if (!capable(CAP_SYS_TIME))
                        return -EACCES;
                epoch = arg;
                return 0;
        case RTC_ALM_READ:
        case RTC_ALM_SET:
        default:
                return -EINVAL;
        }

        return copy_to_user((void*)arg, &wtime, sizeof(wtime))?-EFAULT:0;
}


static int rtc_open(struct inode *inode, struct file *file)
{
    spin_lock_irq(&rtc_lock);

    if(rtc_status)
        goto out_busy;

    rtc_status = 1;
    
    spin_unlock_irq (&rtc_lock);
    return 0;
    
out_busy:
    return -EBUSY;    
}

static int rtc_release(struct inode *inode, struct file *file)
{
    return 0;
}

static struct file_operations rtc_fops = {
	owner:		THIS_MODULE,
	llseek:		no_llseek,
	ioctl:		rtc_ioctl,
	open:		rtc_open,
	release:	rtc_release,
};

#define RTC_MINOR 135
static struct miscdevice rtc_dev = {
    RTC_MINOR,
    "rtc",
    &rtc_fops,
};


static int __init pic_init(void)
{
    int r;

    r = misc_register(&rtc_dev);
    
    return 0;
}

static void __exit pic_exit(void)
{
    misc_deregister(&rtc_dev);
}


module_init(pic_init);
module_exit(pic_exit);
#endif
