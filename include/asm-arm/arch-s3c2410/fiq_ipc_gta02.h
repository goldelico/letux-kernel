#ifndef _LINUX_FIQ_IPC_H
#define _LINUX_FIQ_IPC_H

/*
 * this defines the struct which is used to communicate between the FIQ
 * world and the normal linux kernel world.  One of these structs is
 * statically defined for you in the monolithic kernel so the FIQ ISR code
 * can safely touch it any any time.
 *
 * You also want to include this file in your kernel module that wants to
 * communicate with your FIQ code.  Add any kinds of vars that are used by
 * the FIQ ISR and the module in here.
 *
 * To get you started there is just an int that is incremented every FIQ
 * you can remove this when you are ready to customize, but it is useful
 * for testing
 */

#include <asm/arch/pwm.h>
#include <asm/plat-s3c/regs-timer.h>

enum hdq_bitbang_states {
	HDQB_IDLE = 0,
	HDQB_TX_BREAK,
	HDQB_TX_BREAK_RECOVERY,
	HDQB_ADS_CALC,
	HDQB_ADS_LOW,
	HDQB_ADS_HIGH,
	HDQB_WAIT_RX,
	HDQB_DATA_RX_LOW,
	HDQB_DATA_RX_HIGH,
	HDQB_WAIT_TX,
};

struct fiq_ipc {
	/* vibrator */
	unsigned long vib_gpio_pin; /* which pin to meddle with */
	u8 vib_pwm; /* 0 = OFF -- will ensure GPIO deasserted and stop FIQ */
	u8 vib_pwm_latched;

	/* hdq */
	u8 hdq_probed; /* nonzero after HDQ driver probed */
	struct mutex hdq_lock; /* if you want to use hdq, you have to take lock */
	unsigned long hdq_gpio_pin; /* GTA02 = GPD14 which pin to meddle with */
	u8 hdq_ads; /* b7..b6 = register address, b0 = r/w */
	u8 hdq_tx_data; /* data to tx for write action */
	u8 hdq_rx_data; /* data received in read action */
	u8 hdq_request_ctr; /* incremented by "user" to request a transfer */
	u8 hdq_transaction_ctr; /* incremented after each transfer */
	u8 hdq_error; /* 0 = no error */
};

/* actual definition lives in arch/arm/mach-s3c2440/fiq_c_isr.c */
extern struct fiq_ipc fiq_ipc;
extern unsigned long _fiq_count_fiqs;
extern void fiq_kick(void);  /* provoke a FIQ "immediately" */

#endif /* _LINUX_FIQ_IPC_H */
