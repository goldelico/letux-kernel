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
 b) an automatic inventory scan creating a subdirectory for each found tag
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


#if 0

/* how many of these functions do we really need? */

uchar readFifo(struct trf7960 *device)
{
	return readRegister(device, 0x1f);
}

int writeFifo(struct trf7960 *device, uchar byte)
{
	return writeRegister(device, 0x1f, byte);
}

int prepareSend(struct trf7960 *device, int framelength, int options)
{
	return sendCommand(device, 0x11); // optionally without CRC? with Timer?
}

int getRSSI(struct trf7960 *device)
{
	sendCommand(device, 0x18);
	return readRegister(device, 0x0f);
}

int receiverGainAdjust(struct trf7960 *device)
{
	return sendCommand(device, 0x1a);
}

int blockReceiver(struct trf7960 *device, int flag)
{
	return sendCommand(device, flag?0x16:0x17);
}

int setRfidMode(struct trf7960 *device, int mode)
{
	// 6 bit register 0x01
	return -1;
}

int setNoResponseWait(struct trf7960 *device, int useconds)
{ /* in 37.76 us steps */
	return writeRegister(device, 0x07, (100*useconds+3776/2)/3776);
}

int setTxRxWait(struct trf7960 *device, int useconds)
{ /* in 9.44 us steps */
	return writeRegister(device, 0x08, (100*useconds+944/2)/944);
}

#endif

/* how to handle spi_claim_bus during irq handler? */

static int prepareIrq(struct trf7960 *device, uchar *data, unsigned int bytes)
{
#if 1
	printf("prepareIrq data=%p bytes=%u\n", data, bytes);
#endif
#if 0
	{
	//	uchar irq=readRegister(device, TRF7960_REG_IRQ);
	uchar irq=resetIRQ(device);
	printf("    irq-status=%02x\n", irq);		
	}
#endif
#if 0
	while(device->irq >= 0 && omap_get_gpio_datain(device->irq)) {
		printf("prepareIrq: IRQ pin already active!\n");
		resetIRQ(device);
	}
#endif
	device->done=0;	/* not yet done */
	device->datapointer=data;
	device->bytes=bytes;
	/* enable IRQ */
	return 0;
}

static void handleInterrupt(struct trf7960 *device)
{ /* process interrupt */
	unsigned char buffer[12];
	//	uchar irq=readRegister(device, TRF7960_REG_IRQ);
	uchar irq=resetIRQ(device);
	if(!irq)
		return;	/* false alarm or waitIrq */
	if(device->done)
		return;	/* unprocessed previous interrupt */
#if 0
	printf("handleirq %02x\n", irq);
#endif
	device->done=irq;	/* set done flag with interrupt flags */
#if 0	// read again test (did a read reset the IRQ flags?)
	irq=readRegister(device, TRF7960_REG_IRQ);
	printf("handleirq %02x %02x\n", device->done, irq);
	irq=device->done;	/* restore */
#endif
#if 0
	udelay(5);
	if(device->irq >= 0 && omap_get_gpio_datain(device->irq))
		printf("  IRQ pin still/again active!\n");
#endif
	if(irq & 0x80) { /* end of TX */
#if 1
		printf("handleirq end of TX %02x\n", irq);
#endif
		sendCommand(device, TRF7960_CMD_RESET);	/* reset FIFO */
		return;
	}
	if(irq & 0x02) { /* collision occurred */
		int position;
#if 1
		printf("handleirq collision %02x\n", irq);
#endif
		// FIXME: combine into single message
		sendCommand(device, TRF7960_CMD_RX_BLOCK);	/* block RX */
		position=getCollisionPosition(device);
#if 1
		printf("position=%d\n", position);
#endif
		// number of valid bytes is collpos - 32
		// read bytes
		// handle broken byte
		sendCommand(device, TRF7960_CMD_RESET);	/* reset FIFO */
	}
	else if(irq & 0x40) { /* end of RX */
		uchar fifosr=readRegister(device, TRF7960_REG_FIFO_STATUS);
		uchar unread=(fifosr&0xf) + 1;
#if 1
		printf("handleirq end of RX %02x fifosr=%02x unread=%d\n", irq, fifosr, unread);
#endif
		int n = device->bytes <= unread ? device->bytes : unread;	/* limit to remaining bytes in FIFO or buffer */
		buffer[0] = TRF7960_READ | TRF7960_CONTINUE | TRF7960_REG_FIFO_DATA;	/* continuous read from FIFO */
		memset(&buffer[1], 0, sizeof(buffer)/sizeof(buffer[0])-1);	/* clear buffer so that we don't write garbage */
		if(spi_xfer(device->slave, 8*sizeof(buffer[0])*(2 + n), buffer, buffer, SPI_XFER_BEGIN | SPI_XFER_END) != 0)	/* we need 8 more clocks to receive the last byte */
			return;	/* spi error */
		memcpy(device->datapointer, &buffer[1], n);	/* buffer[0] are the first 8 bits shifted out while we did send the command */
		device->datapointer += n;
		device->bytes -= n;
		// handle broken byte
		sendCommand(device, TRF7960_CMD_RESET);	/* reset FIFO */		
	}
	else if(irq & 0x20) { /* FIFO interrupt */
		int n = device->bytes <= 9 ? device->bytes : 9;	/* limit to 9 bytes */
#if 1
		printf("handleirq fifo request %02x (%d bytes remaining)\n", irq, device->bytes);
#endif
		if(n > 0) {
			if(irq & 0x80) { /* write next n bytes to FIFO (up to 9 or as defined by bytes) */
#if 1
				printf("write more (%d)\n", n);
#endif
				buffer[0] = TRF7960_WRITE | TRF7960_CONTINUE | TRF7960_REG_FIFO_DATA;	/* continuous write to FIFO */
				memcpy(&buffer[1], device->datapointer, n);
				if(spi_xfer(device->slave, 8*sizeof(buffer[0])*(1 + n), buffer, buffer, SPI_XFER_BEGIN | SPI_XFER_END) != 0)
					return;	/* spi error */
				device->datapointer += n;
				device->bytes -= n;				
			} else { /* read next 9 bytes from FIFO in one sequence */
#if 1
				printf("read more (%d)\n", n);
#endif
				buffer[0] = TRF7960_READ | TRF7960_CONTINUE | TRF7960_REG_FIFO_DATA;	/* continuous read from FIFO */
				memset(&buffer[1], 0, sizeof(buffer)/sizeof(buffer[0])-1);	/* clear buffer so that we don't write garbage */
				if(spi_xfer(device->slave, 8*sizeof(buffer[0])*(2 + n), buffer, buffer, SPI_XFER_BEGIN | SPI_XFER_END) != 0)	/* we need 8 more clocks to receive the last byte */
					return;	/* spi error */
				memcpy(device->datapointer, &buffer[1], n);	/* buffer[0] are the first 8 bits shifted out while we did send the command */
				device->datapointer += n;
				device->bytes -= n;
			}
			device->done=0;	/* don't notify successful data trandfer as 'done' status */
		}
		// else some error (FIFO interrupt but no data available or buffer is full
	}
}

static void waitIrq(struct trf7960 *device)
{ /* wait for IRQ */
#if 0
	printf("waitIrq %d\n", device->irq);
#endif
	if(device->irq >= 0)	{ /* check IRQ pin */
		int cnt=2000;	// software timeout
		while(!omap_get_gpio_datain(device->irq) && cnt-- > 0)
			udelay(500); /* wait for IRQ pin */
		handleInterrupt(device);
	}
	else { /* poll interrupt register to check for interrupts */
		while(!device->done)
			handleInterrupt(device);
	}
#if 0
	printf("waitIrq -> %02x\n", device->done);
#endif
}

static int setPowerMode(struct trf7960 *device, int mode)
{ /* control power modes */
	int status = 0;
#if 1
	printf("setPowerMode %d\n", mode);
#endif
	if(mode < TRF7960_POWER_DOWN || mode > TRF7960_POWER_RXTX_FULL)
		return -1;
	if(device->en >= 0)
		omap_set_gpio_direction(device->en, 0);		/* make output */
	if(device->en2 >= 0)
		omap_set_gpio_direction(device->en2, 0);	/* make output */
	if(device->irq >= 0)
		omap_set_gpio_direction(device->irq, 1);	/* make input */
	if(mode == TRF7960_POWER_DOWN) {
		if(device->slave) {
			spi_free_slave(device->slave);
			device->slave = NULL;
		}
		if(device->en >= 0)
			omap_set_gpio_dataout(device->en, 0);
		if(device->en2 >= 0 && device->en != device->en2)
			omap_set_gpio_dataout(device->en2, 0);	/* not tied togehter */
	}
	else if(mode == TRF7960_POWER_60kHz) {
		if(device->en >= 0 && device->en2 >= 0 && device->en == device->en2)
			return -1;	/* can't control them separately */
		if(device->en >= 0)
			omap_set_gpio_dataout(device->en, 0);
		if(device->en2 >= 0)
			omap_set_gpio_dataout(device->en2, 1);
	}
	else {
		if(device->en >= 0)
			omap_set_gpio_dataout(device->en, 1);
		if(!device->slave) {
			device->slave = spi_setup_slave(device->bus, device->cs, device->clock, SPI_MODE_0);
			if(!device->slave)
				return -1;	// failed			
		}
		udelay(1000);	/* wait until we can read/write */
		status = readRegister(device, TRF7960_REG_CSC);
#if 1
		printf("CSC = %02x\n", status);
#endif
		if(status < 0)
			return status;	/* some error */
		switch(mode) {
			case TRF7960_POWER_STANDBY:
				status |= 0x80;
				break;
			case TRF7960_POWER_ACTIVE:
				status &= 0x5d;
				break;
			case TRF7960_POWER_RX:
				status &= 0x5d;
				status |= 0x02;
				break;
			case TRF7960_POWER_RXTX_HALF:
				status &= 0x4f;
				status |= 0x20;
				break;
			case TRF7960_POWER_RXTX_FULL:
				status &= 0x4f;
				status |= 0x30;
				break;
			default:
				return -1;
		}
#if 1
		printf("   => %02x\n", status);
#endif
		status = writeRegister(device, TRF7960_REG_CSC, status);
		// init other registers (only if previous mode was 0 or 1)
		if(device->vio < 27)
			; /* set bit 5 in Reg #0x0b to decrease output resistance for low voltage I/O */
		udelay(5000);	/* wait until reader has recovered (should depend on previous and current mode) */
	}
	return status;
}

static int chooseProtocol(struct trf7960 *device, int protocol)
{
#if 1
	printf("chooseProtocol %d\n", protocol);
#endif
	if((protocol < 0 || protocol >15) && protocol != 0x13)
		return -1;
	protocol |= readRegister(device, TRF7960_REG_CSC) & 0x80;	/* keep no RX CRC mode */
	return writeRegister(device, TRF7960_REG_ISOC, protocol);
}

/* high level functions (protocol handlers) */

int scanInventory(struct trf7960 *device, uchar flags, uchar length, void (*found)(struct trf7960 *device, uchar uid[8], int rssi))
{ /* poll for tag uids and resolve collisions */
	static uchar buffer[32];	/* shared rx/tx buffer */
	uchar collisionslots[16];	/* up to 16 collision slots */
	int collisions = 0;
	uchar mask[8];	/* up to 8 mask bytes */
	int slot;
	int slots = (flags & (1<<5)) ? 1 : 16;	/* multislot flag */
	
	int masksize = (length + 7) / 8;	/* add one mask byte for each started 8 bits */
	int pdusize = 3 + masksize;			/* flags byte + command byte + mask length byte + mask bytes */
	
	int protocol = readRegister(device, TRF7960_REG_ISOC) & 0x1f;
	
	if((protocol & 0x18) == TRF7960_PROTOCOL_ISO15693_LBR_1SC_4)
		;	/* ISO15693 */
	// FIXME: implement different algorithms for other protocols
	if((protocol & 0x1c) == TRF7960_PROTOCOL_ISO14443A_BR_106)
		return -1;	/* ISO14443A */
	if((protocol & 0x1c) == TRF7960_PROTOCOL_ISO14443B_BR_106)
		return -1;	/* ISO14443B */
	if(protocol == TRF7960_PROTOCOL_TAGIT)
		return -1;	/* Tag-It */
	if(protocol >= 0x10)
		return -1;	/* undefined */
#if 1
	printf("inventoryRequest\n");
#endif
	spi_claim_bus(device->slave);
	
	if(writeRegister(device, TRF7960_REG_IRQMASK, 0x3f))	/* enable no-response interrupt */
		return -1;
	
	// write modulator control
	/* writeRegister(device, TRF... 0x09, something); */
	// set rxnoresponse timeout to 0x2f if bit1 is 0 (low data rate), 0x13 else (high data rate)
	
	buffer[0] = TRF7960_COMMAND | TRF7960_CMD_RESET;	/* reset FIFO */
	buffer[1] = TRF7960_COMMAND | TRF7960_CMD_TX_CRC;	/* start TX with CRC */
	buffer[2] = 0x3d;	/* continuous write to register 0x1d */
	
	buffer[5] = flags;	/* ISO15693 flags */
	buffer[6] = 0x01;	/* ISO15693 inventory command */
	if(flags & (1<<4)) { /* AFI */
		buffer[7] = 0;	/* insert AFI value */
		buffer[8] = length;	/* mask length in bits */
		memcpy(&buffer[9], mask, masksize);	/* append mask */
		pdusize++;
	}
	else {
		buffer[7] = length;	/* mask length in bits */
		memcpy(&buffer[8], mask, masksize);	/* append mask */		
	}
	buffer[3] = pdusize >> 8;
	buffer[4] = pdusize << 4;
	prepareIrq(device, &buffer[5+12], pdusize-12);
	if(pdusize > 12)
		pdusize=12; /* limit initial transmission - remainder is sent by interrupt handler */
#if 0
	printf("length = %d\n", length);
	printf("masksize = %d\n", masksize);
	printf("pdusize = %d\n", pdusize);
	printf("bitsize = %d\n", 8*sizeof(buffer[0])*(5 + pdusize));
#endif
	if(spi_xfer(device->slave, 8*sizeof(buffer[0])*(5 + pdusize), buffer, buffer, SPI_XFER_BEGIN | SPI_XFER_END) != 0)
		return -1;
#if 0
	printf("cmd sent\n");
#endif
	waitIrq(device);	/* wait for TX interrupt */
#if 0
	printf("tx done %02x\n", device->done);
#endif
	if(!(device->done & 0x80)) {
#if 1
		printf(" unknown TX interrupt %02x\n", device->done);
#endif
		return -1;
	}
	collisions=0;
	for(slot=0; slot < slots; slot++) { /* repeat for all time slots */
		int rssi;
#if 0
		printf("slot %d\n", slot);
#endif
		prepareIrq(device, buffer, 8);	/* prepare for RX of tag id */
		waitIrq(device);	/* wait for RX interrupt */
#if 1
		printf("rx[%d] done %02x", slot, device->done);
#endif
		rssi = readRegister(device, TRF7960_REG_RSSI) & 0x3f;
#if 0
		printf(" rssi=%02o", rssi);	/* print 2 octal digits for AUX and Active channel */
#endif
		if(device->done & 0x02) { /* collision */
#if 1
			printf(" collision\n");
#endif
			collisionslots[collisions++]=slot;	/* remember slot number */
		}
		else if(device->done & 0x01) {
			/* ignore no response timeout */
#if 1
			printf(" no response\n");
#endif
		}
		/* check for other errors */
		else if(device->done & 0x40) { /* RX done - FIFO already reset */
#if 0
			printf(" valid id received\n");
#endif
			// buffer[0] and buffer[1] appear to be statsu flags and 00 if ok
			(*found)(device, &buffer[2], rssi);	/* notify caller */
		}
		else {
#if 1
			printf(" unknown condition %02x\n", device->done);
#endif
			break;	/* unknown interrupt reason */
		}
		if(!(device->done & 0x40))
			sendCommand(device, TRF7960_CMD_RESET);	/* reset FIFO */
		if(slots == 16) { /* send EOF only in ISO15693 multislot Inventory command */
			sendCommand(device, TRF7960_CMD_RX_BLOCK);
			sendCommand(device, TRF7960_CMD_RX_ENABLE);
			sendCommand(device, TRF7960_CMD_TX_NEXT_SLOT);
		}
	}
	if(slots == 16 ) {
		int i;
#if 1
		printf("did have %d collisions\n", collisions);
#endif
		for(i=0; i<collisions; i++) { /* loop over all slots with collision */
			// generate new mask (increased length by 4) from collision slot numbers
			// inventoryRequest(new mask, length+4)
		}
	}
	// FIXME: how to handle collision in single slot mode?
	// disable irq
	spi_release_bus(device->slave);
	return 0;
}

static int readBlocks(struct trf7960 *device, uchar uid[8], uchar firstBlock, uchar blocks, uchar data[32])
{ /* read single/multiple blocks */
	int flags=0;
	static uchar buffer[32];	/* shared rx/tx buffer */
	int pdusize = 4 + (uid?8:0);			/* flags byte + command byte + optional uid + firstblock + #blocks */
#if 1
	printf("readBlocks\n");
#endif
	if(blocks == 0)
		return 0;	// no blocks
	spi_claim_bus(device->slave);
	
	if(writeRegister(device, TRF7960_REG_IRQMASK, 0x3f))	/* enable no-response interrupt */
		return -1;
	
	buffer[0] = TRF7960_COMMAND | TRF7960_CMD_RESET;	/* reset FIFO */
	buffer[1] = TRF7960_COMMAND | TRF7960_CMD_TX_CRC;	/* start TX with CRC */
	buffer[2] = 0x3d;	/* continuous write to register 0x1d */
	
	buffer[5] = flags;	/* ISO15693 flags */
	buffer[6] = 0x23;	/* ISO15693 read multiple blocks command */
	if(uid) { /* include uid */
		memcpy(&buffer[7], uid, 8);
		buffer[15]=firstBlock;
		buffer[16]=blocks;
		pdusize=4+8;
	} else { /* no UID */
		buffer[7]=firstBlock;
		buffer[8]=blocks;
		pdusize=4;		
	}
	buffer[3] = pdusize >> 8;
	buffer[4] = pdusize << 4;
	prepareIrq(device, &buffer[5+12], pdusize-12);
	if(pdusize > 12)
		pdusize=12; /* limit initial transmission - remainder is sent by interrupt handler */
#if 1
	printf("firstBlock = %d\n", firstBlock);
	printf("blocks = %d\n", blocks);
	printf("pdusize = %d\n", pdusize);
	printf("bitsize = %d\n", 8*sizeof(buffer[0])*(5 + pdusize));
#endif
	if(spi_xfer(device->slave, 8*sizeof(buffer[0])*(5 + pdusize), buffer, buffer, SPI_XFER_BEGIN | SPI_XFER_END) != 0)
		return -1;
#if 1
	printf("cmd sent\n");
#endif
	waitIrq(device);	/* wait for TX interrupt */
#if 1
	printf("tx done %02x\n", device->done);
#endif
	if(!device->done & 0x80) {
#if 1
		printf(" unknown TX interrupt %02x\n", device->done);
#endif
		return -1;
	}
	prepareIrq(device, data, 32*blocks);	/* prepare for receiving n*32 bytes RX */
	waitIrq(device);	/* wait for RX interrupt */
#if 1
	printf("rx done %02x\n", device->done);
#endif
	// check for standard RX done or Collision or timeout or other errors
	// if ok, read the flags byte from the received PDU to determine potential errors
	// FIXME: the first bytes received are not data bytes but the received PDU!
	return 0;
}

static int writeBlocks(struct trf7960 *device, uchar uid[8], uchar firstBlock, uchar blocks, uchar data[32])
{ /* write single/multiple blocks */
	return -1;
}

#endif	/* code to be ported to kernel */


/* Kernel driver definitions and functions */

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
//	.probe		= trf7960_probe,
//	.remove		= trf7960_remove,
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
