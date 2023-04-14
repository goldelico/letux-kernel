/*
 * Copyright (c) 2012 Ingenic Semiconductor Co., Ltd.
 *              http://www.ingenic.com/
 *
 * Core file for Ingenic Display Controller driver
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/errno.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <asm/irq.h>
#include <asm/io.h>
#include <linux/delay.h>
#include "mipi_csi.h"

void dump_csi_reg(void)
{
	printk("****>>>>> dump csi reg <<<<<******\n");
	printk("**********VERSION =%08x\n", csi_core_read(VERSION));
	printk("**********N_LANES =%08x\n", csi_core_read(N_LANES));
	printk("**********PHY_SHUTDOWNZ = %08x\n", csi_core_read(PHY_SHUTDOWNZ));
	printk("**********DPHY_RSTZ = %08x\n", csi_core_read(DPHY_RSTZ));
	printk("**********CSI2_RESETN =%08x\n", csi_core_read(CSI2_RESETN));
	printk("**********PHY_STATE = %08x\n", csi_core_read(PHY_STATE));
	printk("**********DATA_IDS_1 = %08x\n", csi_core_read(DATA_IDS_1));
	printk("**********DATA_IDS_2 = %08x\n", csi_core_read(DATA_IDS_2));
	printk("**********ERR1 = %08x\n", csi_core_read(ERR1));
	printk("**********ERR2 = %08x\n", csi_core_read(ERR2));
	printk("**********MASK1 =%08x\n", csi_core_read(MASK1));
	printk("**********MASK2 =%08x\n", csi_core_read(MASK2));
	printk("**********PHY_TST_CTRL0 = %08x\n", csi_core_read(PHY_TST_CTRL0));
	printk("**********PHY_TST_CTRL1 = %08x\n", csi_core_read(PHY_TST_CTRL1));
}

void check_csi_error(void) {

	unsigned int temp1, temp2;
	while(1){
		dump_csi_reg();
		temp1 = csi_core_read(ERR1);
		temp2 = csi_core_read(ERR2);
		if(temp1 != 0)
			printk("error-------- 1:0x%08x\n", temp1);
		if(temp2 != 0)
			printk("error-------- 2:0x%08x\n", temp2);
	}
}

static unsigned char csi_core_write_part(unsigned int address, unsigned int data, unsigned char shift, unsigned char width)
{
        unsigned int mask = (1 << width) - 1;
        unsigned int temp = csi_core_read(address);
        temp &= ~(mask << shift);
        temp |= (data & mask) << shift;
        csi_core_write(address, temp);

	return 0;
}

static unsigned char  csi_event_disable(unsigned int  mask, unsigned char err_reg_no)
{
	switch (err_reg_no)
	{
		case 1:
			csi_core_write(MASK1, mask | csi_core_read(MASK1));
			break;
		case 2:
			csi_core_write(MASK2, mask | csi_core_read(MASK2));
			break;
		default:
			return ERR_OUT_OF_BOUND;
	}

	return 0;
}

unsigned char csi_set_on_lanes(unsigned char lanes)
{
	csi_core_write_part(N_LANES, (lanes - 1), 0, 2);
	return 0;
}

static int csi_phy_ready(void)
{
	int ready;

	// TODO: phy0: lane0 is ready. need to be update for other lane
	ready = csi_core_read(PHY_STATE);

	if ((ready & (1 << 10 )) && (ready & (1<<4)))
		return 1;

	return 0;
}


/* Reduce power consumption */
int csi_phy_set_bandgap(void)
{
	unsigned int reg;

	/*reset phy*/
	csi_core_write_part(PHY_SHUTDOWNZ, 0, 0, 1);
	csi_core_write_part(CSI2_RESETN, 0, 0, 1);
	csi_core_write_part(DPHY_RSTZ, 0, 0, 1);

	csi_core_write_part(CSI2_RESETN, 1, 0, 1);
	csi_core_write_part(DPHY_RSTZ, 1, 0, 1);
	csi_core_write_part(PHY_SHUTDOWNZ, 1, 0, 1);

	/* set bandgap (reg0b[7])*/
	reg = csi_phy_read(RXPHY_REG_0_0b);
	reg |= (0x1 << 7);
	csi_phy_write(RXPHY_REG_0_0b, reg);
	/* printk("debug ---- > reg0b[7]:0x%08x, %d\n", csi_phy_read(RXPHY_REG_0_0b), __LINE__); */
	return 0;
}

int csi_phy_start(int version, unsigned int lans)
{
	int retries = 30;
	int i;
	unsigned int reg;

	if (version == 1) {
		printk("*******%s***** lans = %d\n", __func__, lans);
		csi_set_on_lanes(lans);

		/* both csi0 csi1 */
		*(volatile unsigned int *) 0xb0074008 = 1;
		*(volatile unsigned int *) 0xb007400c = 1;
		*(volatile unsigned int *) 0xb0073008 = 1;
		*(volatile unsigned int *) 0xb007300c = 1;

		csi_core_write_part(CSI2_RESETN, 0, 0, 1);
		csi_core_write_part(CSI2_RESETN, 1, 0, 1);
	} else {
		/*reset phy*/
		csi_core_write_part(PHY_SHUTDOWNZ, 0, 0, 1);
		csi_core_write_part(CSI2_RESETN, 0, 0, 1);
		csi_core_write_part(DPHY_RSTZ, 0, 0, 1);

		printk("lans: %d\n", lans);
		csi_set_on_lanes(lans);

		csi_core_write_part(CSI2_RESETN, 1, 0, 1);
		csi_core_write_part(DPHY_RSTZ, 1, 0, 1);
		csi_core_write_part(PHY_SHUTDOWNZ, 1, 0, 1);

		udelay(1000);

		/*r/w phy register*/
		switch(lans) {
			case 1:
				csi_phy_write(RXPHY_REG_0_00, 0x75);
				break;
			case 2:
				csi_phy_write(RXPHY_REG_0_00, 0x7d);
				break;
			default:
				printk("Config lans error\n");
				break;
		}
		csi_phy_write(RXPHY_REG_2_0a, 0x3f);

		/* clear bandgap (reg0b[7])*/
		reg = csi_phy_read(RXPHY_REG_0_0b);
		reg &= ~(0x1 << 7);
		csi_phy_write(RXPHY_REG_0_0b, reg);
		/* printk("debug ---- > reg0b[7]:0x%08x\n", csi_phy_read(RXPHY_REG_0_0b)); */

	}

	/* MASK all interrupts */
	csi_event_disable(0xffffffff, 1);
	csi_event_disable(0xffffffff, 2);

	if (version == 2) {
		/* wait for phy ready */
		for (i = 0; i < retries; i++) {
			if (csi_phy_ready())
				break;
			udelay(300);
		}

		if (i >= retries) {
			printk("CSI PHY is not ready\n");
			return -1;


		}
	}

	return 0;
}

int csi_phy_stop(int version)
{

	printk("csi_phy_stop being called \n");
	/*reset phy*/
	/* both csi0 csi1 */
	if (version == 1) {
		*(volatile unsigned int *) 0xb0074008 = 0;
		*(volatile unsigned int *) 0xb007400c = 0;
	} else {
		csi_phy_set_bandgap();
	}
	*(volatile unsigned int *) 0xb0073008 = 0;
	*(volatile unsigned int *) 0xb007300c = 0;
//	csi_core_write_part(PHY_SHUTDOWNZ, 0, 0, 1);
//	csi_core_write_part(DPHY_RSTZ, 0, 0, 1);
	csi_core_write_part(CSI2_RESETN, 0, 0, 1);

	return 0;
}
