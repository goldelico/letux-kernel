/*
 * cst3xx Touchscreen Controller Driver
 */

#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/of_gpio.h>
#include <linux/pm_qos.h>
#include <linux/slab.h>
#include <linux/types.h>
#include "hyn_cst3xx.h"
#include "hyn_cst3xx_RS659_fw.h"
#include <soc/gpio.h>


#define HYN_DEBUG_EN  1

struct st1615_ts_finger {
	u16 x;
	u16 y;
	u8 t;
	bool is_valid;
};


static int cst3xx_into_program_mode(struct i2c_client * client);
static int cst3xx_erase_program_area(struct i2c_client * client);
static void cst3xx_reset_ic(struct cst3xx_ts_data *ts,unsigned int ms);
static int cst3xx_write_program_data(struct i2c_client * client,const unsigned char *pdata);

static int cst3xx_check_checksum(struct i2c_client * client);

static int cst3xx_exit_program_mode(struct i2c_client * client);

static int cst3xx_update_firmware(struct cst3xx_ts_data *ts, const unsigned char *pdata);

static int cst3xx_firmware_info(struct i2c_client * client);

static int cst3xx_update_judge( unsigned char *pdata, int strict);

static int cst3xx_boot_update_fw(struct cst3xx_ts_data *ts,  unsigned char *pdata);

static int cst3xx_i2c_read(struct i2c_client *client, unsigned char *buf, int len);
static int cst3xx_i2c_read_register(struct i2c_client *client, unsigned char *buf, int len);
static int cst3xx_i2c_write(struct i2c_client *client, unsigned char *buf, int len);
static int cst3xx_i2c_test(struct i2c_client *client);
static void cst3xx_reset(struct cst3xx_ts_data *ts) ;
extern  void close_vcc3v3Power(void);
extern  void open_vcc3v3Power(void);

static unsigned char *pcst3xx_update_firmware = (unsigned char *)cst3_fw ; //the updating firmware
static unsigned int   g_cst3xx_ic_version  = 0;
static unsigned int   g_cst3xx_ic_checksum = 0;
static unsigned int   g_cst3xx_ic_checkcode =0;
static unsigned int   g_cst3xx_ic_project_id  = 0;
static unsigned int   g_cst3xx_ic_type  = 0;

static unsigned char  report_flag = 0;
static unsigned int x_temp_pos = 0;
static unsigned int y_temp_pos = 0;

#define HYN_DRAW_POINT

//#define USE_ANALOG_I2C     1 //i2c analog -->defconfig i2c.
//#define USE_HIGHEST_I2CA  1// x1000 highest i2c
#define USE_LOWEST_I2CA   1 // x1000 lowest i2c
//#define HYN_UPDATE_FIRMWARE_POWERON_ENABLE
#define HYN_UPDATE_FIRMWARE_ENABLE

#pragma pack(1)
typedef struct
{
    u16 pid;                 //product id   //
    u16 vid;                 //version id   //
} st_tpd_info;
#pragma pack()

st_tpd_info tpd_info;

#define CST3XX_BIN_SIZE    (24*1024 + 24)
//#define CST3XX_MAX_X  340
#define CST3XX_MAX_X  480
#define CST3XX_REL_X  268
#define CST3XX_MAX_Y  800
#define ERROR_TOUCH   1

static void cst3xx_ts_power(struct cst3xx_ts_data *ts, bool poweron)
{
	if (gpio_is_valid(ts->pdata->reset_gpio))
		gpio_direction_output(ts->pdata->reset_gpio, poweron);
}

static int cst3xx_into_program_mode(struct i2c_client * client)
{
	int ret;
	unsigned char buf[4];

	buf[0] = 0xA0;
	buf[1] = 0x01;
	buf[2] = 0xAA;	//set cmd to enter program mode
	ret = cst3xx_i2c_write(client, buf, 3);
	if (ret < 0)  return -1;

	mdelay(2);

	buf[0] = 0xA0;
	buf[1] = 0x02;	//check whether into program mode
	ret = cst3xx_i2c_read_register(client, buf, 1);
	if (ret < 0)  return -1;

	if (buf[0] != 0x55) return -1;

	return 0;
}
static int cst3xx_erase_program_area(struct i2c_client * client)
{
	int ret;
	unsigned char buf[3];

	buf[0] = 0xA0;
	buf[1] = 0x02;
	buf[2] = 0x00;		//set cmd to erase main area
	ret = cst3xx_i2c_write(client, buf, 3);
	if (ret < 0) return -1;

	mdelay(5);

	buf[0] = 0xA0;
	buf[1] = 0x03;
	ret = cst3xx_i2c_read_register(client, buf, 1);
	if (ret < 0)  return -1;

	if (buf[0] != 0x55) return -1;

	return 0;
}


static void cst3xx_reset_ic(struct cst3xx_ts_data *ts,unsigned int ms)
{

	unsigned char buf[4];
	buf[0] = 0xD1;
	buf[1] = 0x0E;
	cst3xx_i2c_write(ts->client, buf, 2);
/*
	gpio_direction_output(ts->pdata->reset_gpio,0);
	mdelay(2);
	gpio_direction_output(ts->pdata->reset_gpio,1);
*/
	mdelay(ms);
}


static int cst3xx_write_program_data(struct i2c_client * client,
		const unsigned char *pdata)
{
	int i, ret;
	unsigned char *i2c_buf;
	unsigned short eep_addr;
	int total_kbyte;

	unsigned char temp_buf[8];
	unsigned short iic_addr;
	int  j;


	i2c_buf = kmalloc(sizeof(unsigned char)*(1024 + 2), GFP_KERNEL);
	if (i2c_buf == NULL)
		return -1;

	//make sure fwbin len is N*1K
	//total_kbyte = len / 1024;
	total_kbyte = 24;
	for (i=0; i<total_kbyte; i++) {
		i2c_buf[0] = 0xA0;
		i2c_buf[1] = 0x14;
		eep_addr = i << 10;		//i * 1024
		i2c_buf[2] = eep_addr;
		i2c_buf[3] = eep_addr>>8;
		ret = cst3xx_i2c_write(client, i2c_buf, 4);
		if (ret < 0)
			goto error_out;

		memcpy(i2c_buf, pdata + eep_addr, 1024);
		for(j=0; j<256; j++) {
			iic_addr = (j<<2);
    	temp_buf[0] = (iic_addr+0xA018)>>8;
    	temp_buf[1] = (iic_addr+0xA018)&0xFF;
		temp_buf[2] = i2c_buf[iic_addr+0];
		temp_buf[3] = i2c_buf[iic_addr+1];
		temp_buf[4] = i2c_buf[iic_addr+2];
		temp_buf[5] = i2c_buf[iic_addr+3];
    	      ret = cst3xx_i2c_write(client, temp_buf, 6);
    		if (ret < 0)
    			goto error_out;
		}

		i2c_buf[0] = 0xA0;
		i2c_buf[1] = 0x04;
		i2c_buf[2] = 0xEE;
		ret = cst3xx_i2c_write(client, i2c_buf, 3);
		if (ret < 0)
			goto error_out;

		mdelay(60);

		i2c_buf[0] = 0xA0;
		i2c_buf[1] = 0x05;
		ret = cst3xx_i2c_read_register(client, i2c_buf, 1);
		if (ret < 0)
			goto error_out;

		if (i2c_buf[0] != 0x55)
			goto error_out;

	}

	i2c_buf[0] = 0xA0;
	i2c_buf[1] = 0x03;
	i2c_buf[2] = 0x00;
	ret = cst3xx_i2c_write(client, i2c_buf, 3);
	if (ret < 0)
		goto error_out;

	mdelay(8);

	if (i2c_buf != NULL) {
		kfree(i2c_buf);
		i2c_buf = NULL;
	}

	return 0;

error_out:
	if (i2c_buf != NULL) {
		kfree(i2c_buf);
		i2c_buf = NULL;
	}
	return -1;
}


static int cst3xx_check_checksum(struct i2c_client * client)
{
	int ret;
	int i;
	unsigned int  checksum;
	unsigned int  bin_checksum;
	unsigned char buf[4];
	const unsigned char *pData;

	for(i=0; i<5; i++)
	{
		buf[0] = 0xA0;
		buf[1] = 0x00;
		ret = cst3xx_i2c_read_register(client, buf, 1);
		if(ret < 0)
		{
			mdelay(2);
			continue;
		}

		if(buf[0]!=0)
			break;
		else
		mdelay(2);
	}
    mdelay(2);


    if(buf[0]==0x01)
	{
		buf[0] = 0xA0;
		buf[1] = 0x08;
		ret = cst3xx_i2c_read_register(client, buf, 4);

		if(ret < 0)	return -1;

		// read chip checksum
		checksum = buf[0] + (buf[1]<<8) + (buf[2]<<16) + (buf[3]<<24);

        pData=(unsigned char  *)pcst3xx_update_firmware +24*1024+16;   //7*1024 +512
		bin_checksum = pData[0] + (pData[1]<<8) + (pData[2]<<16) + (pData[3]<<24);

        printk("  hyn the updated ic checksum is :0x%x. the updating firmware checksum is:0x%x------\n", checksum, bin_checksum);

        if(checksum!=bin_checksum)
		{
			printk(" cst3xx hyn check sum error.\n");
			return -1;

		}

	}
	else
	{
		printk(" cst3xx hyn No checksum.\n");
		return -1;
	}
	return 0;
}

static int cst3xx_exit_program_mode(struct i2c_client * client)
{
	int ret;
	unsigned char buf[3];

	buf[0] = 0xA0;
	buf[1] = 0x06;
	buf[2] = 0xEE;
	ret = cst3xx_i2c_write(client, buf, 3);
	if (ret < 0)
		return -1;

	mdelay(10);	//wait for restart


	return 0;
}


static int cst3xx_update_firmware(struct cst3xx_ts_data *ts, const unsigned char *pdata)
{
	int ret;
	int retry = 0;

	printk(" cst3xx----------upgrade cst3xx begain------------\n");
	disable_irq(ts->irq);
	mdelay(20);
START_FLOW:

#ifdef HYN_UPDATE_FIRMWARE_POWERON_ENABLE
	close_vcc3v3Power();
	mdelay(20);
	open_vcc3v3Power();
	mdelay(5+retry);
#else
	cst3xx_reset_ic(ts,5+retry);
#endif
	ret = cst3xx_into_program_mode(ts->client);
	if (ret < 0) {
		printk(" cst3xx[cst3xx]into program mode failed.\n");
		goto err_out;
	}

	ret = cst3xx_erase_program_area(ts->client);
	if (ret) {
		printk(" cst3xx[cst3xx]erase main area failed.\n");
		goto err_out;
	}

	ret = cst3xx_write_program_data(ts->client, pdata);
	if (ret < 0) {
		printk(" cst3xx[cst3xx]write program data into cstxxx failed.\n");
		goto err_out;
	}

    ret =cst3xx_check_checksum(ts->client);
	if (ret < 0) {
		printk(" cst3xx[cst3xx] after write program cst3xx_check_checksum failed.\n");
		goto err_out;
	}

	ret = cst3xx_exit_program_mode(ts->client);
	if (ret < 0) {
		printk(" cst3xx[cst3xx]exit program mode failed.\n");
		goto err_out;
	}

	cst3xx_reset_ic(ts,20);

	printk(" cst3xx hyn----------cst3xx_update_firmware  end------------\n");

	enable_irq(ts->irq);

	return 0;

err_out:
	if (retry < 30) {
		retry++;
		mdelay(20);
		goto START_FLOW;
	}
	else {
		enable_irq(ts->irq);
		return -1;
	}
}




/*******************************************************
Function:
    get firmware version, ic type...
Input:
    client: i2c client
Output:
    success: 0
    fail:	-1
*******************************************************/
static int cst3xx_firmware_info(struct i2c_client * client)
{
	int ret;
	unsigned char buf[28];
//	unsigned short ic_type, project_id;


	buf[0] = 0xD1;
	buf[1] = 0x01;
	ret = cst3xx_i2c_write(client, buf, 2);
	if (ret < 0) return -1;

	mdelay(40);

	buf[0] = 0xD1;
	buf[1] = 0xFC;
	ret = cst3xx_i2c_read_register(client, buf, 4);
	if (ret < 0) return -1;

	//0xCACA0000
	g_cst3xx_ic_checkcode = buf[3];
	g_cst3xx_ic_checkcode <<= 8;
	g_cst3xx_ic_checkcode |= buf[2];
	g_cst3xx_ic_checkcode <<= 8;
	g_cst3xx_ic_checkcode |= buf[1];
	g_cst3xx_ic_checkcode <<= 8;
	g_cst3xx_ic_checkcode |= buf[0];

	printk("linc cst3xx [cst3xx] the chip g_cst3xx_ic_checkcode:0x%x.\r\n",g_cst3xx_ic_checkcode);

	mdelay(2);

	buf[0] = 0xD2;
	buf[1] = 0x04;
	ret = cst3xx_i2c_read_register(client, buf, 4);
	if (ret < 0) return -1;
	g_cst3xx_ic_type = buf[3];
	g_cst3xx_ic_type <<= 8;
	g_cst3xx_ic_type |= buf[2];


	g_cst3xx_ic_project_id = buf[1];
	g_cst3xx_ic_project_id <<= 8;
	g_cst3xx_ic_project_id |= buf[0];

	printk("linc cst3xx [cst3xx] the chip ic g_cst3xx_ic_type :0x%x, g_cst3xx_ic_project_id:0x%x\r\n",
		g_cst3xx_ic_type, g_cst3xx_ic_project_id);

	mdelay(2);

	buf[0] = 0xD2;
	buf[1] = 0x08;
	ret = cst3xx_i2c_read_register(client, buf, 8);
	if (ret < 0) return -1;

	g_cst3xx_ic_version = buf[3];
	g_cst3xx_ic_version <<= 8;
	g_cst3xx_ic_version |= buf[2];
	g_cst3xx_ic_version <<= 8;
	g_cst3xx_ic_version |= buf[1];
	g_cst3xx_ic_version <<= 8;
	g_cst3xx_ic_version |= buf[0];

	g_cst3xx_ic_checksum = buf[7];
	g_cst3xx_ic_checksum <<= 8;
	g_cst3xx_ic_checksum |= buf[6];
	g_cst3xx_ic_checksum <<= 8;
	g_cst3xx_ic_checksum |= buf[5];
	g_cst3xx_ic_checksum <<= 8;
	g_cst3xx_ic_checksum |= buf[4];

	tpd_info.vid = g_cst3xx_ic_version;
    tpd_info.pid = 0x00;


	printk(" cst3xx [cst3xx] the chip ic version:0x%x, checksum:0x%x\r\n",
		g_cst3xx_ic_version, g_cst3xx_ic_checksum);

	if(g_cst3xx_ic_version==0xA5A5A5A5)
	{
		printk(" cst3xx [cst3xx] the chip ic don't have firmware. \n");
		return -1;
	}
	if((g_cst3xx_ic_checkcode&0xffff0000)!=0xCACA0000){
		printk("linc cst3xx [cst3xx] cst3xx_firmware_info read error .\r\n");
		return -1;
	}

    buf[0] = 0xD1;
	buf[1] = 0x09;
	ret = cst3xx_i2c_write(client, buf, 2);
	if (ret < 0) return -1;
    mdelay(5);


	return 0;
}

static int cst3xx_update_judge( unsigned char *pdata, int strict)
{
	unsigned short ic_type, project_id;
	unsigned int fw_checksum, fw_version;
	const unsigned int *p;
	int i;
	unsigned char *pBuf;

	fw_checksum = 0x55;
	p = (const unsigned int *)pdata;
	for (i=0; i<(CST3XX_BIN_SIZE-4); i+=4) {
		fw_checksum += (*p);
		p++;
	}

	if (fw_checksum != (*p)) {
		printk(" cst3xx[cst3xx]calculated checksum error:0x%x not equal 0x%x.\n", fw_checksum, *p);
		return -1;	//bad fw, so do not update
	}

	pBuf = &pdata[CST3XX_BIN_SIZE-16];

	project_id = pBuf[1];
	project_id <<= 8;
	project_id |= pBuf[0];

	ic_type = pBuf[3];
	ic_type <<= 8;
	ic_type |= pBuf[2];

	fw_version = pBuf[7];
	fw_version <<= 8;
	fw_version |= pBuf[6];
	fw_version <<= 8;
	fw_version |= pBuf[5];
	fw_version <<= 8;
	fw_version |= pBuf[4];

	fw_checksum = pBuf[11];
	fw_checksum <<= 8;
	fw_checksum |= pBuf[10];
	fw_checksum <<= 8;
	fw_checksum |= pBuf[9];
	fw_checksum <<= 8;
	fw_checksum |= pBuf[8];

	printk(" cst3xx[cst3xx]the updating firmware:project_id:0x%04x,ic type:0x%04x,version:0x%x,checksum:0x%x\n",
			project_id, ic_type, fw_version, fw_checksum);

	if (strict > 0) {

		if (g_cst3xx_ic_checksum != fw_checksum){
			if (g_cst3xx_ic_version >fw_version){
				printk("[cst3xx]fw version(0x%x), ic version(0x%x).\n",fw_version, g_cst3xx_ic_version);
			//	return -1;
			}
		}else{
			printk("[cst3xx]fw checksum(0x%x), ic checksum(0x%x).\n",fw_checksum, g_cst3xx_ic_checksum);
			return -1;
		}
	}

	return 0;
}
static int cst3xx_boot_update_fw(struct cst3xx_ts_data *ts,  unsigned char *pdata)
{
	int ret;
	int retry = 0;
	int flag = 0;

	while (retry++ < 3) {
		ret = cst3xx_firmware_info(ts->client);
		if (ret == 0) {
			flag = 1;
			break;
		}
	}

	if (flag == 1) {
		ret = cst3xx_update_judge(pdata, 1);
		if (ret < 0) {
			printk(" cst3xx[cst3xx] no need to update firmware.\n");
			return 0;
		}
	}

	ret = cst3xx_update_firmware(ts, pdata);
	if (ret < 0){
		printk(" cst3xx [cst3xx] update firmware failed.\n");
		return -1;
	}

    mdelay(50);

	ret = cst3xx_firmware_info(ts->client);
	if (ret < 0) {
		printk(" cst3xx [cst3xx] after update read version and checksum fail.\n");
		return -1;
	}

	return 0;
}
static int hyn_boot_update_fw(struct cst3xx_ts_data * ts)
{
	unsigned char *ptr_fw;
	int ret;
	ptr_fw = pcst3xx_update_firmware;
	ret = cst3xx_boot_update_fw(ts, ptr_fw);
	return ret;

}


/*******************************Chose which I2C interface ********************************************/

#if USE_ANALOG_I2C

#define I2CSDA  GPIO_PB(23)
#define I2CSCL   GPIO_PB(22)

#define I2CSDA_HIGH gpio_direction_output(I2CSDA,1)
#define I2CSDA_LOW  gpio_direction_output(I2CSDA,0)

#define I2CSCL_HIGH  gpio_direction_output(I2CSCL,1)
#define I2CSCL_LOW  gpio_direction_output(I2CSCL,0)

#define delay_data 2
static void iicstart(void)
{
     I2CSDA_HIGH;
      I2CSCL_HIGH;
      __udelay(2);
      I2CSDA_LOW;
      __udelay(2);
       I2CSCL_LOW;
       __udelay(2);
}
static void iicstop(void)
{
     I2CSDA_LOW;
     I2CSCL_HIGH;
     __udelay(2);
     I2CSDA_HIGH;
     __udelay(2);

}
static unsigned char waitack(void)
{
   u8 ucErrTime=0;

    gpio_direction_input(GPIO_PB(23));
   //  __udelay(2);

  //  I2CSDA_HIGH;
  //  __udelay(2);
     I2CSCL_HIGH;
     __udelay(2);

    while(__gpio_get_value(GPIO_PB(23)))
    {
        ucErrTime++;
     //   printk("time out ++ \n");
        if(ucErrTime>250)
         {
                iicstop();
                return 1;
        }
    }
    I2CSCL_LOW;
     __udelay(2);
    return 0;
}
static void sendack(void)
{
     I2CSCL_LOW;
     __udelay(delay_data);
     I2CSDA_LOW;
     __udelay(delay_data);

     I2CSCL_HIGH;
     __udelay(delay_data);
     I2CSCL_LOW;
     __udelay(delay_data);

}
static void sendnotack(void)
{
     I2CSCL_LOW;
     __udelay(delay_data);
    I2CSDA_HIGH;
    __udelay(delay_data);

    I2CSCL_HIGH;
    __udelay(delay_data);
    I2CSCL_LOW;
    __udelay(delay_data);
}
static void iicsendbyte(unsigned char ch)
{
     unsigned char i;
     I2CSCL_LOW;
     for(i = 0;i<8;i++)
     {
          if(ch&0x80)
          {
              I2CSDA_HIGH;
          }
          else
          {
              I2CSDA_LOW;
          }
           __udelay(delay_data);
          ch<<=1;
          I2CSCL_HIGH;
          __udelay(delay_data);
               I2CSCL_LOW;
          __udelay(delay_data);
     }
}
static unsigned char iicreceivebyte(unsigned char ack)
{
    unsigned char i=8;
    unsigned char ddata=0;

    gpio_direction_input(GPIO_PC(27));
    __udelay(delay_data);

     for(i = 0;i<8;i++)
     {
         I2CSCL_LOW;
         __udelay(delay_data);
         I2CSCL_HIGH;
         __udelay(delay_data);

         ddata<<=1;
         if(gpio_get_value(GPIO_PC(27))) ddata++;
    }

    if(!ack)
        sendnotack();
     else
        sendack();

    return ddata;
}
static int cst3xx_i2c_read(struct i2c_client *client, unsigned char *buf, int len)
{
	printk(">>>>>>>>>>>>>>>>>func=%s,line=%d\n\n",__func__,__LINE__);
    int count = len;
     iicstart();
     __udelay(delay_data);

/*
	 iicsendbyte( (0x1a<<1) | 0);
     __udelay(delay_data);

     if(waitack())
     {
        iicstop();
                printk(" cst3xx_i2c_read wait ack timeoue \n");
        return 1;
     }

     iicsendbyte(buf[0]);
     __udelay(delay_data);
     waitack();
     __udelay(delay_data);
*/
     iicstart();
     __udelay(delay_data);

     iicsendbyte((0x1a << 1 ) | 1);
     __udelay(delay_data);
     if(waitack())
     {
        iicstop();
                printk(" cst3xx_i2c_read wait ack timeoue \n");
        return 1;
     }
     __udelay(delay_data);

    while(count)
    {
         if(count == 1) *buf = iicreceivebyte(0);
         else *buf = iicreceivebyte(1);
         count --;
         buf++;
    }
     iicstop();
     __udelay(delay_data);
     return 0;
}

static int cst3xx_i2c_write(struct i2c_client *client, unsigned char *buf, int len)
{
	HYN_FUNC_ENTER();
	printk(">>>>>>>>>>>>>>>>>func=%s,line=%d\n\n",__func__,__LINE__);
     int count = len;
      iicstart();
   //   __udelay(delay_data);

      iicsendbyte((0x1A << 1) | 0);
//      __udelay(delay_data);

     if(waitack())
     {
        iicstop();
        printk(" cst3xx_i2c_write send address wait ack timeoue \n");
        return 1;
     }

      while(count--)
      {
            iicsendbyte(*(buf++));
            __udelay(delay_data);
             if(waitack())
             {
                iicstop();
               printk(" cst3xx_i2c_write wait ack timeoue \n");
                return 1;
             }
            __udelay(delay_data);
      }

      iicstop();
      __udelay(delay_data);

	return 0;
}

static int cst3xx_i2c_read_register(struct i2c_client *client, unsigned char *buf, int len)
{
	printk(">>>>>>>>>>>>>>>>>func=%s,line=%d\n\n",__func__,__LINE__);
	int ret = -1;

    ret = cst3xx_i2c_write(client, buf, 2);
    if(ret) printk("failed to write cst3xx\n");

    ret = cst3xx_i2c_read(client, buf, len);
    if(ret) printk("failed to write cst3xx\n");
    return ret;
}

#elif  USE_LOWEST_I2CA
static int cst3xx_i2c_read(struct i2c_client *client, unsigned char *buf, int len)
{
	HYN_FUNC_ENTER();
//	printk(">>>>>>>low speed>>>>>>>>>>func=%s,line=%d\n\n",__func__,__LINE__);
	int ret = -1;
	int retries = 0;

	while (retries < 2) {
		ret = i2c_master_recv(client, buf, len);
		if(ret<=0)
		    retries++;
        else
            break;
	}

	return ret;
}

static int cst3xx_i2c_write(struct i2c_client *client, unsigned char *buf, int len)
{
	HYN_FUNC_ENTER();
//	printk(">>>>>>low speed>>>>>>>>>>>func=%s,line=%d\n\n",__func__,__LINE__);
	int ret = -1;
	int retries = 0;

	while (retries < 2) {
		ret = i2c_master_send(client, buf, len);
		if(ret<=0)
		    retries++;
        else
            break;
	}

	return ret;
}

static int cst3xx_i2c_read_register(struct i2c_client *client, unsigned char *buf, int len)
{
	HYN_FUNC_ENTER();
//	printk(">>>>>low speed>>>>>>>>>>>>func=%s,line=%d\n\n",__func__,__LINE__);
	int ret = -1;

    ret = cst3xx_i2c_write(client, buf, 2);

    ret = cst3xx_i2c_read(client, buf, len);

    return ret;
}

#elif USE_HIGHEST_I2CA
static int cst3xx_i2c_read(struct i2c_client *client, unsigned char *buf, int len)
{
	HYN_FUNC_ENTER();
	printk(">>>>>>high speed>>>>>>>>>>>func=%s,line=%d\n\n",__func__,__LINE__);
	struct i2c_msg msg;
	int ret = -1;
	int retries = 0;

	msg.flags |= I2C_M_RD;
	msg.addr   = client->addr;
	msg.len    = len;
	msg.buf    = buf;

	while (retries < 2) {
		ret = i2c_transfer(client->adapter, &msg, 1);
		if(ret == 1)
			break;
		retries++;
	}

	return ret;
}
/*******************************************************
Function:
    read data from register.
Input:
    buf: first two byte is register addr, then read data store into buf
    len: length of data that to read
Output:
    success: number of messages
    fail:	negative errno
*******************************************************/
static int cst3xx_i2c_read_register(struct i2c_client *client, unsigned char *buf, int len)
{
	HYN_FUNC_ENTER();
	printk(">>>>>>high speed>>>>>>>>>>>func=%s,line=%d\n\n",__func__,__LINE__);
	struct i2c_msg msgs[2];
	int ret = -1;
	int retries = 0;

	msgs[0].flags = 0;
	msgs[0].addr  = client->addr;
	msgs[0].len   = 2;
	msgs[0].buf   = buf;

	msgs[1].flags |= I2C_M_RD;
	msgs[1].addr   = client->addr;
	msgs[1].len    = len;
	msgs[1].buf    = buf;

	while (retries < 2) {
		ret = i2c_transfer(client->adapter, msgs, 2);
		if(ret == 2)
			break;
		retries++;
	}

	return ret;
}

static int cst3xx_i2c_write(struct i2c_client *client, unsigned char *buf, int len)
{
	HYN_FUNC_ENTER();
	printk(">>>>>>high speed>>>>>>>>>>>func=%s,line=%d\n\n",__func__,__LINE__);
	struct i2c_msg msg;
	int ret = -1;
	int retries = 0;

	msg.flags = 0;
	msg.addr  = client->addr;
	msg.len   = len;
	msg.buf   = buf;

	while (retries < 2) {
		ret = i2c_transfer(client->adapter, &msg, 1);
		if(ret == 1)
			break;
		retries++;
	}

	return ret;
}
#else
#endif
/*********************************i2c end********************************/


static int cst3xx_i2c_test(struct i2c_client *client)
{
	int retry = 0;
	int ret;
	unsigned char buf[4];

	buf[0] = 0xD0;
	buf[1] = 0x00;
	while (retry++ < 5) {
		ret = cst3xx_i2c_write(client, buf, 2);
		if (ret == 0)
			return ret;

		mdelay(2);
	}

    if(retry==5) printk(" cst3xx hyn I2C TEST error.ret:%d;\n", ret);

	return ret;
}


static void cst3xx_reset(struct cst3xx_ts_data *ts) {
	if(gpio_is_valid(ts->pdata->reset_gpio)) {
		gpio_direction_output(ts->pdata->reset_gpio,0);
		mdelay(100);
		gpio_direction_output(ts->pdata->reset_gpio,1);
		mdelay(100);
	}
}
/****************************  basic communication end*******************************/


static void cst3xx_touch_report(struct cst3xx_ts_data *ts)
{
	unsigned char buf[30];
	unsigned char i2c_buf[8];
	unsigned char key_status, key_id = 0, finger_id, sw;
	unsigned int  input_x = 0,bx;
	unsigned int  input_y = 0,by;
	unsigned int  input_w = 0;
	unsigned int input_x_delta = 0;
	unsigned int input_y_delta = 0;
       unsigned char cnt_up, cnt_down;
	int   i, ret, idx;
	int cnt, i2c_len;

	int  len_1, len_2;

      key_status = 0;

	buf[0] = 0xD0;
	buf[1] = 0x00;
	ret = cst3xx_i2c_read_register(ts->client, buf, 7);
	if(ret < 0) {
		printk("  iic read touch point data failed.\n");
		goto OUT_PROCESS;
	}
  //   printk(" read buf : buf[0]: %x ,buf[1]: %x ,buf[2]: %x, buf[3]: %x,buf[4]: %x ,buf[5]: %x ,buf[6]: %x, buf[7]: %x,\n",buf[0],buf[1],buf[2],buf[3],buf[4],buf[5],buf[6],buf[7]);

	if(buf[6] != 0xAB) {
		printk(KERN_INFO " data is not valid..\r\n");
		goto OUT_PROCESS;
	}

    	cnt = buf[5] & 0x7F;
	if(cnt > 1) goto OUT_PROCESS;
	else if(cnt==0)     goto CLR_POINT;

	if(cnt == 0x01) {
		goto FINGER_PROCESS;
	}

FINGER_PROCESS:

    /*
     *  sw: buf[0] low 4 bit
     *  x pos:  buf[1] low 4 bit | buf[3] high 4 bit
     *  y pos:  buf[2] low 4 bit | buf[3] low  4 bit
     *  z press : buf[4]
     *  finger_id:buf[0] high 4 bit
     */
	i2c_buf[0] = 0xD0;
	i2c_buf[1] = 0x00;
	i2c_buf[2] = 0xAB;

	ret = cst3xx_i2c_write(ts->client, i2c_buf, 3);
	if(ret < 0) {
		printk(" cst3xx hyn send read touch info ending failed.\r\n");
		cst3xx_reset_ic(ts,20);
              goto END;
	}

	idx = 0;
       cnt_up = 0;
       cnt_down = 0;
	input_x = (unsigned int)((buf[idx + 1] << 4) | ((buf[idx + 3] >> 4) & 0x0F));
	input_y = (unsigned int)((buf[idx + 2] << 4) | (buf[idx + 3] & 0x0F));
	input_w = (unsigned int)(buf[idx + 4]);
	sw = (buf[idx] & 0x0F) >> 1;
	finger_id = (buf[idx] >> 4) & 0x0F;

    //printk(KERN_DEBUG " no handle : cst3xxPoint x:%d, y:%3d, w:%3d, id:%d, sw:%d. \n", input_x, input_y, input_w,finger_id, sw);
	bx = input_x;
	by = input_y;

//	input_x=input_x*CST3XX_MAX_X/268;  //update 0729: no handle == handle
//	input_x=input_x*CST3XX_MAX_X/360;  //
//	input_y=input_y*CST3XX_MAX_Y/800;

//	if(input_x >= CST3XX_MAX_X) input_x = CST3XX_MAX_X;
//	if(input_y >= CST3XX_MAX_Y) input_y = CST3XX_MAX_Y;

//	input_x = CST3XX_MAX_X-input_x;
//	input_y = CST3XX_MAX_Y-input_y;
#if 1 // clivia
	#define TP_START_X	106
	#define TP_WIDTH	268
	#define TP_MAX_X	(TP_START_X+TP_WIDTH-1)
	#define TP_START_Y	0
	#define TP_HEIGHT	800
	#define TP_MAX_Y	(TP_START_Y+TP_HEIGHT-1)
	if(input_x < 106)
		input_x = 106;
	else if(input_x > TP_MAX_X)
		input_x = TP_MAX_X;
	else
		input_x = input_x - 106;
	if(input_y > TP_MAX_Y)
		input_y = TP_MAX_Y;

#else
	if(input_x > CST3XX_MAX_X-1)
		input_x = CST3XX_MAX_X-1;
	if(input_y > CST3XX_MAX_Y-1)
		input_y = CST3XX_MAX_Y-1;
#ifdef HYN_DRAW_POINT
	if(input_x > 300){
		input_x_delta = input_x - (CST3XX_MAX_X/2);
		input_x = input_x + input_x_delta/((CST3XX_MAX_X/2)/20);
		if(input_x > CST3XX_MAX_X) input_x = CST3XX_MAX_X;
		}
	if(input_x < 40){
		input_x_delta = (CST3XX_MAX_X/2) - input_x;
		input_x = input_x - input_x_delta/((CST3XX_MAX_X/2)/12);
		if(input_x < 0) input_x = 0;
		}
	if(input_y > 760){
		input_y_delta = input_y - (CST3XX_MAX_Y/2);
		input_y = input_y + input_y_delta/((CST3XX_MAX_Y/2)/12);
		if(input_y > CST3XX_MAX_Y) input_y = CST3XX_MAX_Y;
		}
	if(input_y < 40){
		input_y_delta = (CST3XX_MAX_Y/2) - input_y;
		input_y = input_y - input_y_delta/((CST3XX_MAX_Y/2)/25);
		if(input_y < 0) input_y = 0;
		}
	input_x -= (CST3XX_MAX_X - CST3XX_REL_X)/2;
#endif
#endif

//    printk(KERN_DEBUG " handle : cst3xxPoint x:%d, y:%d, w:%d, id:%d, sw:%d. \n\n", input_x, input_y, input_w,finger_id, sw);
      //clivia printk(KERN_DEBUG "cst3xxPoint bx:%3d, by=%3d, x:%3d, y:%3d, w:%3d, id:%d, sw:%d. \n", bx, by, input_x, input_y, input_w,finger_id, sw);
	if (sw == 0x03 ) {
         	   if((input_x > x_temp_pos + ERROR_TOUCH) || (input_x < x_temp_pos -ERROR_TOUCH)  || (input_y > y_temp_pos + ERROR_TOUCH) || (input_y < y_temp_pos -ERROR_TOUCH))
               {
          //           printk(" REPORT TOUCH DOWM Handle \n");
					x_temp_pos = input_x;
					y_temp_pos = input_y;
					input_report_key(ts->input_dev, BTN_TOUCH, 1);
    		        input_report_abs(ts->input_dev, ABS_X, input_x);
					input_report_abs(ts->input_dev, ABS_Y, input_y);
    		        input_report_abs(ts->input_dev, ABS_PRESSURE, 1);
    		        input_sync(ts->input_dev);
                 }
                cnt_down++;
       }
      else if(sw == 0) {
                cnt_up++;
        }

        if((cnt_up>0) && (cnt_down==0) ){
      //      printk(" REPORT TOUCH UP Handle \n");
            x_temp_pos = 0;
            y_temp_pos = 0;
            input_report_key(ts->input_dev, BTN_TOUCH, 0);
            input_report_abs(ts->input_dev, ABS_PRESSURE, 0);
            input_mt_sync(ts->input_dev);
        }
       input_sync(ts->input_dev);
	goto END;
CLR_POINT:

	    input_report_key(ts->input_dev, BTN_TOUCH, 0);
           input_mt_sync(ts->input_dev);
           input_sync(ts->input_dev);

OUT_PROCESS:
	buf[0] = 0xD0;
	buf[1] = 0x00;
	buf[2] = 0xAB;
	ret = cst3xx_i2c_write(ts->client, buf, 3);
	if (ret < 0) {
		printk("  send read touch info ending failed.\n");
		cst3xx_reset_ic(ts,20);
	}
END:
	return;
}


/******************************* irq handle  begin************************************/
static void cst3xx_work_handler(struct work_struct *work)
{
	struct cst3xx_ts_data *cst3xx_ts = container_of(work, struct cst3xx_ts_data, work);
	int ret = 0;
    /******************here handle touch *******************/
    //    printk(KERN_INFO "-----------enter cst3xx_work_handler------------\n");
         cst3xx_touch_report(cst3xx_ts);
	  enable_irq(cst3xx_ts->irq);
}

static irqreturn_t cst3xx_ts_irq_handler(int irq, void *dev_id)
{
	struct cst3xx_ts_data *ts = dev_id;

   //    printk( KERN_INFO "-----------enter cst3xx_ts_irq_handler------------\n");

	disable_irq_nosync(ts->irq);//use in interrupt,disable_irq will make dead lock.

	if (!work_pending(&ts->work)) {
		queue_work(ts->workqueue, &ts->work);
	//    queue_delayed_work(ts->workqueue,&ts->dwork,1);//10ms * x
	} else {
		enable_irq(ts->irq);
	}

	return IRQ_HANDLED;
}
/******************************* irq handle  end************************************/
static int cst3xx_parse_dt(struct device *dev, struct cst3xx_platform_data *pdata)
{
    int ret = 0;
    struct device_node *np = dev->of_node;
    u32 temp_val = 0;

    HYN_FUNC_ENTER();

    /* reset, irq gpio info */
    pdata->reset_gpio = of_get_named_gpio_flags(np, "hyn,reset-gpio",
                        0, &pdata->reset_gpio_flags);
    if (pdata->reset_gpio < 0)
        HYN_ERROR("Unable to get reset_gpio");

    pdata->int_gpio = of_get_named_gpio_flags(np, "hyn,irq-gpio",
                      0, pdata->irqflags);
    if (pdata->int_gpio < 0)
        HYN_ERROR("Unable to get irq_gpio");

    HYN_INFO(" irq gpio:%d, reset gpio:%d",
              pdata->int_gpio, pdata->reset_gpio);

    HYN_FUNC_EXIT();
    return 0;
}
static int cst3xx_gpio_configure(struct cst3xx_ts_data *data)
{
    int ret = 0;

    HYN_FUNC_ENTER();
    /* request irq gpio */
    if (gpio_is_valid(data->pdata->int_gpio)) {
        ret = gpio_request(data->pdata->int_gpio, "cst3xx_irq_gpio");
        if (ret) {
            HYN_ERROR("[GPIO]irq gpio request failed");
            goto err_irq_gpio_req;
        }

        ret = gpio_direction_input(data->pdata->int_gpio);
        if (ret) {
            HYN_ERROR("[GPIO]set_direction for irq gpio failed");
            goto err_irq_gpio_dir;
        }
    }

    /* request reset gpio */
    if (gpio_is_valid(data->pdata->reset_gpio)) {
        ret = gpio_request(data->pdata->reset_gpio, "cst3xx_reset_gpio");
        if (ret) {
            HYN_ERROR("[GPIO]reset gpio request failed");
            goto err_irq_gpio_dir;
        }

//        ret = gpio_direction_output(data->pdata->reset_gpio, 1);
//        if (ret) {
//            HYN_ERROR("[GPIO]set_direction for reset gpio failed");
//            goto err_reset_gpio_dir;
//        }
    }

    HYN_FUNC_EXIT();
    return 0;

err_reset_gpio_dir:
    if (gpio_is_valid(data->pdata->reset_gpio))
        gpio_free(data->pdata->reset_gpio);
err_irq_gpio_dir:
    if (gpio_is_valid(data->pdata->int_gpio))
        gpio_free(data->pdata->int_gpio);
err_irq_gpio_req:
    HYN_FUNC_EXIT();
    return ret;
}

static int cst3xx_input_init(struct cst3xx_ts_data *ts_data)
{
    int ret = 0;
    int key_num = 0;
    struct cst3xx_platform_data *pdata = ts_data->pdata;
    struct input_dev *input_dev;

    HYN_FUNC_ENTER();
    input_dev = input_allocate_device();
    if (!input_dev) {
        HYN_ERROR("Failed to allocate memory for input device");
        return -ENOMEM;
    }

    /* Init and register Input device */
    input_dev->name = CST3XX_NAME;

    input_dev->id.bustype = BUS_I2C;
    input_dev->dev.parent = ts_data->dev;

    input_set_drvdata(input_dev, ts_data);

    __set_bit(EV_SYN, input_dev->evbit);
    __set_bit(EV_ABS, input_dev->evbit);
    __set_bit(EV_KEY, input_dev->evbit);
    __set_bit(BTN_TOUCH, input_dev->keybit);
//    __set_bit(INPUT_PROP_DIRECT, input_dev->propbit);
    set_bit(ABS_X, input_dev->absbit); //add from hyn
	set_bit(ABS_Y, input_dev->absbit);
    set_bit(ABS_PRESSURE, input_dev->absbit);

/*---add from hyn*/
	input_set_abs_params(input_dev, ABS_X, ts_data->pdata->x_min, ts_data->pdata->x_max, 0, 0);
	input_set_abs_params(input_dev, ABS_Y, ts_data->pdata->y_min, ts_data->pdata->y_max, 0, 0);
    input_set_abs_params(input_dev, ABS_PRESSURE, 0, 1, 0 , 0);
/*add from hyn---*/

    ret = input_register_device(input_dev);
    if (ret) {
        HYN_ERROR("Input device registration failed");
        input_set_drvdata(input_dev, NULL);
        input_free_device(input_dev);
        input_dev = NULL;
        return ret;
    }

    ts_data->input_dev = input_dev;

    HYN_FUNC_EXIT();
    return 0;
}

static int cst3xx_irq_registration(struct cst3xx_ts_data *ts_data)
{
    int ret = 0;
    struct cst3xx_platform_data *pdata = ts_data->pdata;

    ts_data->irq = gpio_to_irq(pdata->int_gpio);
    pdata->irqflags = IRQF_TRIGGER_FALLING | IRQF_ONESHOT;
    HYN_INFO("irq:%d, flag:%x", ts_data->irq, pdata->irqflags);
    ret = request_threaded_irq(ts_data->irq, NULL, cst3xx_ts_irq_handler,
                               pdata->irqflags,
                               CST3XX_NAME, ts_data);

    return ret;
}

static int cst3xx_ts_probe_entry(struct cst3xx_ts_data *ts_data)
{
    int ret = 0;
    int rc=-1;
    int pdata_size = sizeof(struct cst3xx_platform_data);

    HYN_FUNC_ENTER();
    ts_data->pdata = kzalloc(pdata_size, GFP_KERNEL);
    if (!ts_data->pdata) {
        HYN_ERROR("allocate memory for platform_data fail");
        return -ENOMEM;
    }

    if (ts_data->dev->of_node) {
        ret = cst3xx_parse_dt(ts_data->dev, ts_data->pdata);
        if (ret)
            HYN_ERROR("device-tree parse fail");
    } else {
        if (ts_data->dev->platform_data) {
            memcpy(ts_data->pdata, ts_data->dev->platform_data, pdata_size);
        } else {
            HYN_ERROR("platform_data is null");
            return -ENODEV;
        }
    }
    //init work for cst3xx..

	INIT_WORK(&ts_data->work, cst3xx_work_handler);

    ts_data->workqueue = create_singlethread_workqueue("cst3xx_wq");
    if (!ts_data->workqueue) {
        HYN_ERROR("create cst3xx workqueue fail");
    }

//    spin_lock_init(&ts_data->irq_lock);
//    mutex_init(&ts_data->report_mutex);
//    mutex_init(&ts_data->bus_lock);

    /* Init communication interface */
    ret = cst3xx_bus_init(ts_data);
    if (ret) {
        HYN_ERROR("bus initialize fail");
        goto err_bus_init;
    }
#if USE_ANALOG_I2C
        if (gpio_is_valid(I2CSDA)
        {
            gpio_free(I2CSDA);
            ret = gpio_request(GPIO_PB(23),"I2C2-SDA");
            if(ret < 0){
                printk("I2C2-SDA gpio request failed\n");
                return ret;
            }
        }
         if (gpio_is_valid(I2CSCL))
        {
            gpio_free(I2CSCL);
            ret = gpio_request(GPIO_PB(22),"I2C2-SCL");
            if(ret < 0){
                printk("I2C2-SCL gpio request failed\n");
                return ret;
            }
         }
#endif
       ret =  cst3xx_i2c_test(ts_data->client);
      // if(ret>=0){
	   if(ret<0){


          mdelay(100);

          printk( KERN_INFO "cst3xx test OK -------------------\n");
#ifdef HYN_UPDATE_FIRMWARE_ENABLE
	        ret = cst3xx_boot_update_fw(ts_data,pcst3xx_update_firmware);//
			if(ret < 0){
			printk(" cst3xx hyn_boot_update_fw failed.\n");
			return -1;
			}
#endif

       }else{
         // printk( KERN_INFO "cst3xx hyn i2c communication failed.---------------\n");
          mdelay(20);
		  rc = hyn_boot_update_fw(ts_data);
		  if(rc < 0){
			printk(" cst3xx hyn_boot_update_fw failed.\n");
			//return -1;
		}
       }

    ret = cst3xx_input_init(ts_data);
    if (ret) {
        HYN_ERROR("input initialize fail");
        goto err_input_init;
    }

    ret = cst3xx_gpio_configure(ts_data);
    if (ret) {
        HYN_ERROR("configure the gpios fail");
        goto err_gpio_config;
    }
       printk( KERN_INFO "cst3xx reset ----before---------------\n");
       cst3xx_reset(ts_data);
       printk( KERN_INFO "cst3xx reset ----after--------------\n");

    ret = cst3xx_irq_registration(ts_data);
    if (ret) {
        HYN_ERROR("request irq failed");
        goto err_irq_req;
    }

    HYN_FUNC_EXIT();
    return 0;

err_irq_req:
    if (gpio_is_valid(ts_data->pdata->reset_gpio))
        gpio_free(ts_data->pdata->reset_gpio);
    if (gpio_is_valid(ts_data->pdata->int_gpio))
        gpio_free(ts_data->pdata->int_gpio);
err_gpio_config:
    kfree_safe(ts_data->point_buf);
    kfree_safe(ts_data->events);
err_input_init:
    if (ts_data->workqueue)
        destroy_workqueue(ts_data->workqueue);
err_bus_init:
    kfree_safe(ts_data->bus_tx_buf);
    kfree_safe(ts_data->bus_rx_buf);
    kfree_safe(ts_data->pdata);

    HYN_FUNC_EXIT();
    return ret;
}
static int cst3xx_ts_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    int ret = 0;
    struct cst3xx_ts_data *ts_data = NULL;
	struct input_dev *input_dev;

	struct cst3xx_platform_data *pdata = (struct cst3xx_platform_data*)client->dev.platform_data;

    HYN_INFO("Touch Screen(I2C BUS) driver prboe...");
    if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
        HYN_ERROR("I2C not supported");
        return -ENODEV;
    }

    /* malloc memory for global struct variable */
    ts_data = (struct cst3xx_ts_data *)kzalloc(sizeof(*ts_data), GFP_KERNEL);
    if (!ts_data) {
        HYN_ERROR("allocate memory for cst3xx_data fail");
        return -ENOMEM;
    }
    ts_data->client = client;
    ts_data->dev = &client->dev;
    ts_data->pdata = pdata;
    i2c_set_clientdata(client, ts_data);

    ret = cst3xx_ts_probe_entry(ts_data);
    if (ret) {
        HYN_ERROR("Touch Screen(I2C BUS) driver probe fail");
        kfree_safe(ts_data);
        return ret;
    }
	if(client->addr != 0x1A) //check address
	{
		client->addr = 0x1A;
		printk(">>>>>>>>>>>>>1>>>xtang:i2c_client_HYN->addr=%d.\n",client->addr);

	}else
		printk(">>>>>>>>>>>>2>>>>xtang:i2c_client_HYN->addr=%d.\n",client->addr);


    HYN_INFO("Touch Screen(I2C BUS) driver prboe successfully");
    return 0;
}


static int cst3xx_ts_remove(struct i2c_client *client)
{
	struct cst3xx_ts_data *ts = i2c_get_clientdata(client);
        free_irq(ts->irq,ts);
        devm_free_irq(&ts->client->dev, ts->client->irq,ts);
        destroy_workqueue(ts->workqueue);
        input_unregister_device(ts->input_dev);
        devm_gpio_free(&ts->client->dev, ts->pdata->reset_gpio);
        gpio_free(ts->pdata->int_gpio);

#if USE_ANALOG_I2C
         gpio_free(I2CSDA);
        gpio_free(I2CSCL);
#endif
//i2c bus init.
	cst3xx_bus_exit(ts);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int cst3xx_ts_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct cst3xx_ts_data *ts = i2c_get_clientdata(client);

	if (device_may_wakeup(&client->dev)) {
		enable_irq_wake(client->irq);
	} else {
		disable_irq(client->irq);
		cst3xx_ts_power(ts, false);//reset
	}

	return 0;
}

static int cst3xx_ts_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct cst3xx_ts_data *ts = i2c_get_clientdata(client);

	if (device_may_wakeup(&client->dev)) {
		disable_irq_wake(client->irq);
	} else {
		cst3xx_ts_power(ts, true);
		enable_irq(client->irq);
	}
	return 0;
}

#endif

static SIMPLE_DEV_PM_OPS(cst3xx_ts_pm_ops,
			 cst3xx_ts_suspend, cst3xx_ts_resume);

static const struct i2c_device_id cst3xx_ts_id[] = {
	{ CST3XX_NAME, 0 },
	{ }
};
static const struct of_device_id cst3xx_dt_match[] = {
    {.compatible = "hyn,cst3xx", },
    {},
};
MODULE_DEVICE_TABLE(of, cst3xx_dt_match);

static struct i2c_driver cst3xx_ts_driver = {
    .probe = cst3xx_ts_probe,
    .remove = cst3xx_ts_remove,
    .driver = {
        .name = CST3XX_NAME,
        .owner = THIS_MODULE,
        .of_match_table = of_match_ptr(cst3xx_dt_match),
	.pm	= &cst3xx_ts_pm_ops,
    },
    .id_table = cst3xx_ts_id,
};

static int __init cst3xx_ts_init(void)
{
    int ret = 0;

    HYN_FUNC_ENTER();
    ret = i2c_add_driver(&cst3xx_ts_driver);
    if ( ret != 0 ) {
        HYN_ERROR("hyn touch screen driver init failed!");
    }
    HYN_FUNC_EXIT();
    return ret;
}

static void __exit cst3xx_ts_exit(void)
{
    i2c_del_driver(&cst3xx_ts_driver);
}
//module_init(cst3xx_ts_init);
late_initcall(cst3xx_ts_init);
module_exit(cst3xx_ts_exit);

MODULE_AUTHOR("HYN Driver Team");
MODULE_DESCRIPTION("HYN Touchscreen Driver");
MODULE_LICENSE("GPL v2");
