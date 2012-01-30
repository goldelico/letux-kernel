/*  Copyright (c) 2010  Nikolaus Schaller <hns@goldelico.com>
 
 This driver supports the TI TRF7960 RFID reader. The datasheet
 is avaliable from their website:
 
 http://focus.ti.com/docs/prod/folders/print/trf7960.html#technicaldocuments
 
 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.
 
 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.
 
 You should have received a copy of the GNU General Public License
 along with this program; if not, write to the Free Software
 Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

/*
 this driver presents in /sys
 
 a) power control (off, on, on+scanning)
 b) an automatic inventory scan creating a subdirectory for each tag that is currently found
 c) access to found tags through the subdirectory
 
 -> see samples/kobject/kset-example.c
 -> but 		r = kobject_init_and_add(&mgr->kobj, &manager_ktype, &pdev->dev.kobj, "manager%d", i);
 
 and see http://www.makelinux.net/ldd3/chp-14-sect-2

 */



#include <linux/module.h>
#include <linux/init.h>
#include <linux/spi/spi.h>
#include <linux/slab.h>
#include <linux/delay.h>

/* gpio pins to fix HW problem with TRF7960 being in "SPI without SS" mode */

#define spi_xfer bitbang_spi_xfer
#define spi_claim_bus(X)
#define spi_release_bus(X)

#define McSPI3_CLK	130
#define McSPI3_SIMO	131
#define McSPI3_SOMI	132

#define IRQ	156
#define EN	161
#define EN2	159

#define VIO	18	/* 1.8 V */

#if 0	/* code to be ported to kernel */

typedef u8 uchar;

// uchar -> ??
// replace struct trf with some kalloc'ed storage
// handle interrupts


struct trf7960 {
	/*
	 * OMAP3 McSPI (MultiChannel SPI) has 4 busses (modules)
	 * with different number of chip selects (CS, channels):
	 * McSPI1 has 4 CS (bus 0, cs 0 - 3)
	 * McSPI2 has 2 CS (bus 1, cs 0 - 1)
	 * McSPI3 has 2 CS (bus 2, cs 0 - 1)
	 * McSPI4 has 1 CS (bus 3, cs 0)
	 */
	int bus;
	int cs;
	int clock;
	int irq;	/* GPIO that receives IRQ; use -1 if we have no IRQ */
	int en;		/* GPIO that controls EN; use -1 if hardwired */
	int en2;	/* GPIO that controls EN2; use -1 if hardwired (use same if both are parallel) */
	int vio;	/* specify 10*VIO i.e. 18 for 1.8V, 33 for 3.3V */
	struct spi_slave *slave;
	/* internal */
	int done;	/* done interrupt flag */
	uchar *datapointer;	/* RX/TX data pointer */
	int bytes;		/* number of remaining bytes to transmit (may be negative) */
};

/* low level functions */

#define TRF7960_REG_CSC		0x00	/* Chip Status control (R/W) */
#define TRF7960_REG_ISOC	0x01	/* ISO control (R/W) */
#define TRF7960_REG_ISO14443BTXOPT	0x02	/* TX Options (R/W) */
#define TRF7960_REG_ISO14443AHBROPT	0x03	/* High Bitrate Options (R/W) */
#define TRF7960_REG_TXTIMERMSB	0x04	/* TX Timer (R/W) */
#define TRF7960_REG_TXTIMERLSB	0x05	/* TX Timer (R/W) */
#define TRF7960_REG_TXPULSEC	0x06	/* TX Pulse Lenght control (R/W) */
#define TRF7960_REG_RXNRWAIT	0x07	/* RX no response wait (R/W) */
#define TRF7960_REG_RXWAIT	0x08	/* RX wait time (after TX) (R/W) */
#define TRF7960_REG_MODCLK	0x09	/* Modulator and SYS_CLK (R/W) */
#define TRF7960_REG_RXSPECIAL	0x0a	/* RX special setting (R/W) */
#define TRF7960_REG_REGIO	0x0b	/* Regulator and IO control (R/W) */
#define TRF7960_REG_IRQ	0x0c	/* IRQ status (R) */
#define TRF7960_REG_IRQMASK	0x0d	/* Collision position (MSB) and Interrupt Mask (R/W) */
#define TRF7960_REG_COLLISION	0x0e	/* Collision position (LSB) (R) */
#define TRF7960_REG_RSSI	0x0f	/* RSSI levels and oscillator status (R) */
#define TRF7960_REG_FIFO_STATUS	0x1c	/* FIFO status (R) */
#define TRF7960_REG_FIFO_TXLEN1	0x1d	/* TX length byte 1 (R/W) */
#define TRF7960_REG_FIFO_TXLEN2	0x1e	/* TX length byte 2 (R/W) */
#define TRF7960_REG_FIFO_DATA	0x1f	/* FIFO I/O register (R/W) */

/* FIXME: add bit masks for these registers */

#define TRF7960_CMD_IDLE	0x00
#define TRF7960_CMD_INIT	0x03
#define TRF7960_CMD_RESET	0x0f		/* reset FIFO */
#define TRF7960_CMD_TX_NOCRC	0x10
#define TRF7960_CMD_TX_CRC	0x11	/* TX with CRC */
#define TRF7960_CMD_TX_DELAYED_NOCRC	0x12
#define TRF7960_CMD_TX_DELAYED_CRC	0x13
#define TRF7960_CMD_TX_NEXT_SLOT	0x14	/* send EOF / next slot */
#define TRF7960_CMD_RX_BLOCK	0x16	/* block receiver */
#define TRF7960_CMD_RX_ENABLE	0x17	/* enable receiver */
#define TRF7960_CMD_TEST_RF_INT	0x18
#define TRF7960_CMD_TEST_RF_EXT	0x19
#define TRF7960_CMD_RX_GAIN_ADJUST	0x1a

/* first byte sent through SPI */
#define TRF7960_COMMAND		0x80
#define TRF7960_ADDRESS		0x00	/* combine with READ/WRITE and optionally CONTINUE */
#define TRF7960_READ		0x40
#define TRF7960_WRITE		0x00
#define TRF7960_CONTINUE	0x20

/* power modes (increasing power demand) */
#define TRF7960_POWER_DOWN	0
#define TRF7960_POWER_60kHz	1		/* VDD_X available, 60 kHz */
#define TRF7960_POWER_STANDBY	2	/* 13.56 MHz osc. on, SYS_CLK available; regulators in low power */
#define TRF7960_POWER_ACTIVE	3	/* 13.56 MHz osc. on, SYS_CLK available; regulators active */
#define TRF7960_POWER_RX	4	/* RX active */
#define TRF7960_POWER_RXTX_HALF	5	/* RX+TX active; half power mode */
#define TRF7960_POWER_RXTX_FULL	6	/* RX+TX active; full power mode */

/* protocols */
#define TRF7960_PROTOCOL_ISO15693_LBR_1SC_4		0x00
#define TRF7960_PROTOCOL_ISO15693_LBR_1SC_256	0x01
#define TRF7960_PROTOCOL_ISO15693_HBR_1SC_4		0x02
#define TRF7960_PROTOCOL_ISO15693_HBR_1SC_256	0x03
#define TRF7960_PROTOCOL_ISO15693_LBR_2SC_4		0x04
#define TRF7960_PROTOCOL_ISO15693_LBR_2SC_256	0x05
#define TRF7960_PROTOCOL_ISO15693_HBR_2SC_4		0x06
#define TRF7960_PROTOCOL_ISO15693_HBR_2SC_256	0x07
#define TRF7960_PROTOCOL_ISO14443A_BR_106	0x08
#define TRF7960_PROTOCOL_ISO14443A_BR_212	0x09
#define TRF7960_PROTOCOL_ISO14443A_BR_424	0x0a
#define TRF7960_PROTOCOL_ISO14443A_BR_848	0x0b
#define TRF7960_PROTOCOL_ISO14443B_BR_106	0x0c
#define TRF7960_PROTOCOL_ISO14443B_BR_212	0x0d
#define TRF7960_PROTOCOL_ISO14443B_BR_424	0x0e
#define TRF7960_PROTOCOL_ISO14443B_BR_848	0x0f
#define TRF7960_PROTOCOL_TAGIT	0x13

/* */

#define HALFBIT 1	/* 1us gives approx. 500kHz clock */

static int bitbang_spi_xfer(struct spi_slave *slave, int bitlen, uchar writeBuffer[], uchar readBuffer[], int flags)
{ /* generates bitlen+2 clock pulses */
	static int first=1;
	int bit;
	uchar wb=0;
	uchar rb;
	if(first) { /* if not correctly done by pinmux */
		omap_set_gpio_direction(McSPI3_CLK, 0);
		omap_set_gpio_direction(McSPI3_SIMO, 0);
		omap_set_gpio_direction(McSPI3_SOMI, 1);
		omap_set_gpio_dataout(McSPI3_CLK, 0);
		omap_set_gpio_dataout(McSPI3_SIMO, 0);	/* send out constant 0 bits (idle command) */
		first=0;
		udelay(100);
	}
#if 0
	printf("bitbang_spi_xfer %d bits\n", bitlen);
#endif
	if(flags & SPI_XFER_BEGIN) {
		omap_set_gpio_dataout(McSPI3_CLK, 1);
		udelay(HALFBIT);	/* may be optional (>50ns) */
		omap_set_gpio_dataout(McSPI3_SIMO, 1);	/* start condition (data transision while clock=1) */
		udelay(HALFBIT);		
	}
	omap_set_gpio_dataout(McSPI3_CLK, 0);
	for(bit=0; bit < bitlen; bit++)
		{ /* write data */
			if(bit%8 == 0)
				wb=writeBuffer[bit/8];
			omap_set_gpio_dataout(McSPI3_SIMO, (wb&0x80)?1:0);	/* send MSB first */
			wb <<= 1;
			udelay(HALFBIT);
			omap_set_gpio_dataout(McSPI3_CLK, 1);
			udelay(HALFBIT);
			omap_set_gpio_dataout(McSPI3_CLK, 0);
			rb = (rb<<1) | omap_get_gpio_datain(McSPI3_SOMI);	/* sample on falling edge and receive MSB first */
			if(bit%8 == 7)
				readBuffer[bit/8]=rb;
		}
	if(flags & SPI_XFER_END) {
		omap_set_gpio_dataout(McSPI3_SIMO, 1);	/* set data to 1 */
		udelay(HALFBIT);
		omap_set_gpio_dataout(McSPI3_CLK, 1);
		udelay(HALFBIT);
		omap_set_gpio_dataout(McSPI3_SIMO, 0);	/* stop condition (data transision while clock=1) */
		udelay(HALFBIT);	/* may be optional (>50ns) */
		omap_set_gpio_dataout(McSPI3_CLK, 0);
		udelay(HALFBIT);
	}
	return 0;
}

static inline int readRegister(struct trf7960 *device, uchar addr)
{
	uchar writeBuffer[2];
	uchar readBuffer[2];
	writeBuffer[0] = TRF7960_ADDRESS | TRF7960_READ | addr;
	spi_claim_bus(device->slave);
	if(spi_xfer(device->slave, 16, writeBuffer, readBuffer, SPI_XFER_BEGIN | SPI_XFER_END) != 0)
		return -1;
	spi_release_bus(device->slave);
	return readBuffer[1];
}

static inline int writeRegister(struct trf7960 *device, uchar addr, uchar byte)
{
	uchar writeBuffer[2];
	uchar readBuffer[2];
	writeBuffer[0] = TRF7960_ADDRESS | TRF7960_WRITE | addr;
	writeBuffer[1] = byte;
	spi_claim_bus(device->slave);
	if(spi_xfer(device->slave, 2*8, writeBuffer, readBuffer, SPI_XFER_BEGIN | SPI_XFER_END) != 0)
		return -1;
	spi_release_bus(device->slave);
	return 0;
}

static inline int sendCommand(struct trf7960 *device, uchar cmd)
{
	uchar writeBuffer[1];
	uchar readBuffer[1];
	writeBuffer[0] = TRF7960_COMMAND | cmd;
	spi_claim_bus(device->slave);
	if(spi_xfer(device->slave, 8, writeBuffer, readBuffer, SPI_XFER_BEGIN | SPI_XFER_END) != 0)
		return -1;
	spi_release_bus(device->slave);
	return 0;
}

/* mid level  (should be partially mapped to sysfs) */

static int resetIRQ(struct trf7960 *device)
{
	uchar writeBuffer[3];
	uchar readBuffer[3];
	writeBuffer[0] = TRF7960_ADDRESS | TRF7960_READ | TRF7960_CONTINUE | TRF7960_REG_IRQ;
	writeBuffer[1] = 0;	// dummy read
	writeBuffer[2] = 0;	// dummy read
	spi_claim_bus(device->slave);
	if(spi_xfer(device->slave, 3*8, writeBuffer, readBuffer, SPI_XFER_BEGIN | SPI_XFER_END) != 0)
		return -1;
	spi_release_bus(device->slave);
#if 0
	{
	int i;
	for(i=0; i<3; i++)
		printf("rb[i]=%02x\n", readBuffer[i]);
	}
#endif
	return readBuffer[1];
}

static int getCollisionPosition(struct trf7960 *device)
{
	uchar writeBuffer[3];
	uchar readBuffer[3];
	writeBuffer[0] = TRF7960_ADDRESS | TRF7960_READ | TRF7960_CONTINUE | TRF7960_REG_IRQMASK;
	writeBuffer[1] = 0;	// dummy read
	writeBuffer[2] = 0;	// dummy read
	spi_claim_bus(device->slave);
	if(spi_xfer(device->slave, 3*8, writeBuffer, readBuffer, SPI_XFER_BEGIN | SPI_XFER_END) != 0)
		return -1;
	spi_release_bus(device->slave);
	return ((readBuffer[1]&0xc0)<<2) | readBuffer[2];	// combine into 10 bits
}


#endif


#if 0
/* Kernel driver definitions and functions */

// FIXME: demangle interrupt (data received) and work thread (regularly scan for inventory) */

static void trf7960_work(struct work_struct *work)
{
	struct trf7960 *ts =
	container_of(to_delayed_work(work), struct trf7960, work);
	/* run scan inventory command */
#if 0
	struct ts_event tc;
	u32 rt;
	
	/*
	 * NOTE: We can't rely on the pressure to determine the pen down
	 * state, even though this controller has a pressure sensor.
	 * The pressure value can fluctuate for quite a while after
	 * lifting the pen and in some cases may not even settle at the
	 * expected value.
	 *
	 * The only safe way to check for the pen up condition is in the
	 * work function by reading the pen signal state (it's a GPIO
	 * and IRQ). Unfortunately such callback is not always available,
	 * in that case we have rely on the pressure anyway.
	 */
	if (ts->get_pendown_state) {
		if (unlikely(!ts->get_pendown_state())) {
			tsc2007_send_up_event(ts);
			ts->pendown = false;
			goto out;
		}
		
		dev_dbg(&ts->client->dev, "pen is still down\n");
	}
	
	tsc2007_read_values(ts, &tc);
	
	rt = tsc2007_calculate_pressure(ts, &tc);
	if (rt > MAX_12BIT) {
		/*
		 * Sample found inconsistent by debouncing or pressure is
		 * beyond the maximum. Don't report it to user space,
		 * repeat at least once more the measurement.
		 */
		dev_dbg(&ts->client->dev, "ignored pressure %d\n", rt);
		goto out;
		
	}
	
	if (rt) {
		struct input_dev *input = ts->input;
		
		if (!ts->pendown) {
			dev_dbg(&ts->client->dev, "DOWN\n");
			
			input_report_key(input, BTN_TOUCH, 1);
			ts->pendown = true;
		}
		
		input_report_abs(input, ABS_X, tc.x);
		input_report_abs(input, ABS_Y, tc.y);
		input_report_abs(input, ABS_PRESSURE, rt);
		
		input_sync(input);
		
		dev_dbg(&ts->client->dev, "point(%4d,%4d), pressure (%4u)\n",
				tc.x, tc.y, rt);
		
	} else if (!ts->get_pendown_state && ts->pendown) {
		/*
		 * We don't have callback to check pendown state, so we
		 * have to assume that since pressure reported is 0 the
		 * pen was lifted up.
		 */
		trf7960_send_up_event(ts);
		ts->pendown = false;
	}
#endif
out:
	if (ts->pendown)
		schedule_delayed_work(&ts->work,
							  msecs_to_jiffies(TS_POLL_PERIOD));
	else
		enable_irq(ts->irq);
}

static irqreturn_t trf7960_irq(int irq, void *handle)
{
	struct trf7960 *ts = handle;
	
	if (!ts->get_pendown_state || likely(ts->get_pendown_state())) {
		disable_irq_nosync(ts->irq);
		schedule_delayed_work(&ts->work,
							  msecs_to_jiffies(TS_POLL_DELAY));
	}
	
	if (ts->clear_penirq)
		ts->clear_penirq();
	
	return IRQ_HANDLED;
}

static void trf7960_free_irq(struct trf7960 *ts)
{
	free_irq(ts->irq, ts);
	if (cancel_delayed_work_sync(&ts->work)) {
		/*
		 * Work was pending, therefore we need to enable
		 * IRQ here to balance the disable_irq() done in the
		 * interrupt handler.
		 */
		enable_irq(ts->irq);
	}
}

static int __devinit trf7960_probe(struct i2c_client *client,
								   const struct i2c_device_id *id)
{
#if 0
	struct trf7960 *ts;
	struct trf7960_platform_data *pdata = pdata = client->dev.platform_data;
	struct input_dev *input_dev;
	int err;
	
	if (!pdata) {
		dev_err(&client->dev, "platform data is required!\n");
		return -EINVAL;
	}
	
	if (!i2c_check_functionality(client->adapter,
								 I2C_FUNC_SMBUS_READ_WORD_DATA))
		return -EIO;
	
	ts = kzalloc(sizeof(struct trf7960), GFP_KERNEL);
	input_dev = input_allocate_device();
	if (!ts || !input_dev) {
		err = -ENOMEM;
		goto err_free_mem;
	}
#if 0
	
	ts->client = client;
	ts->irq = client->irq;
	ts->input = input_dev;
	INIT_DELAYED_WORK(&ts->work, trf7960_work);

	ts->model             = pdata->model;
	ts->x_plate_ohms      = pdata->x_plate_ohms;
	ts->get_pendown_state = pdata->get_pendown_state;
	ts->clear_penirq      = pdata->clear_penirq;
	
	snprintf(ts->phys, sizeof(ts->phys),
			 "%s/input0", dev_name(&client->dev));
	
	input_dev->name = "TRF7960 RFID Reader";
	input_dev->phys = ts->phys;
	input_dev->id.bustype = BUS_I2C;
	
	input_dev->evbit[0] = BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS);
	input_dev->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH);
	
	input_set_abs_params(input_dev, ABS_X, 0, MAX_12BIT, 0, 0);
	input_set_abs_params(input_dev, ABS_Y, 0, MAX_12BIT, 0, 0);
	input_set_abs_params(input_dev, ABS_PRESSURE, 0, MAX_12BIT, 0, 0);
#endif	
	if (pdata->init_platform_hw)
		pdata->init_platform_hw();
	
	err = request_irq(ts->irq, trf7960_irq, 0,
					  client->dev.driver->name, ts);
	if (err < 0) {
		dev_err(&client->dev, "irq %d busy?\n", ts->irq);
		goto err_free_mem;
	}
#if 0
	/* Prepare for touch readings - power down ADC and enable PENIRQ */
	err = tsc2007_xfer(ts, PWRDOWN);
	if (err < 0)
		goto err_free_irq;
	
	err = input_register_device(input_dev);
	if (err)
		goto err_free_irq;
	
	i2c_set_clientdata(client, ts);
#endif
#endif
	return 0;
	
err_free_irq:
	trf7960_free_irq(ts);
	if (pdata->exit_platform_hw)
		pdata->exit_platform_hw();
err_free_mem:
	input_free_device(input_dev);
	kfree(ts);
	return err;
}

static int __devexit trf7960_remove(struct i2c_client *client)
{
	struct trf7960	*ts = i2c_get_clientdata(client);
	struct trf7960_platform_data *pdata = client->dev.platform_data;
	
	trf7960_free_irq(ts);
	
	if (pdata->exit_platform_hw)
		pdata->exit_platform_hw();
	
//	input_unregister_device(ts->input);
	kfree(ts);

	return 0;
}


static const struct spi_device_id trf7960_id[] = {
	{ "trf7960", 0 },
	{ }
};

static struct spi_driver trf7960_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name	= "trf7960"
	},
	.id_table	= trf7960_id,
	.probe		= trf7960_probe,
	.remove		= trf7960_remove,
	// suspend, resume
};

static int __init trf7960_init(void)
{
	return spi_register_driver(&trf7960_driver);
}

static void __exit trf7960_exit(void)
{
	spi_unregister_driver(&trf7960_driver);
}


MODULE_AUTHOR("Nikolaus Schaller <hns@goldelico.com>");
MODULE_DESCRIPTION("TRF7960 driver");
MODULE_LICENSE("GPL");

module_init(trf7960_init);
module_exit(trf7960_exit);

#endif
