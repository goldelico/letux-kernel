#ifndef __LINUX_AXP_MFD_216_H_
#define __LINUX_AXP_MFD_216_H_
/* Unified sub device IDs for AXP */
/* LDO0 For RTCLDO ,LDO1-3 for ALDO,LDO*/
enum {
	AXP_ID_RTC,   //RTCLDO
	AXP_ID_ALDO1,   //ALDO1
	AXP_ID_ALDO2,   //ALDO2
	AXP_ID_ALDO3,   //ALDO3

	AXP_ID_ELDO1,   //ELDO1
	AXP_ID_ELDO2,  //ELDO2

	AXP_ID_DCDC1,
	AXP_ID_DCDC2,
	AXP_ID_DCDC3,
	AXP_ID_DCDC4,
	AXP_ID_DCDC5,

	AXP_ID_LDOIO1,



	AXP_ID_SUPPLY,
	AXP_ID_GPIO,
};

/*For AXP216*/
#define AXP216                   (216)
#define AXP_STATUS              (0x00)
#define AXP_MODE_CHGSTATUS      (0x01)
#define AXP_IC_TYPE			    (0x03)
#define AXP_BUFFER1             (0x04)
#define AXP_BUFFER2             (0x05)
#define AXP_BUFFER3             (0x06)
#define AXP_BUFFER4             (0x07)
#define AXP_BUFFER5             (0x08)
#define AXP_BUFFER6             (0x09)
#define AXP_BUFFER7             (0x0A)
#define AXP_BUFFER8             (0x0B)
#define AXP_BUFFER9             (0x0C)
#define AXP_BUFFERA             (0x0D)
#define AXP_BUFFERB             (0x0E)
#define AXP_BUFFERC             (0x0F)
#define AXP_IPS_SET             (0x30)
#define AXP_VOFF_SET            (0x31)
#define AXP_OFF_CTL             (0x32)
#define AXP_CHARGE1             (0x33)
#define AXP_CHARGE2             (0x34)
#define AXP_CHARGE3             (0x35)
#define AXP_POK_SET             (0x36)
#define AXP_INTEN1              (0x40)
#define AXP_INTEN2              (0x41)
#define AXP_INTEN3              (0x42)
#define AXP_INTEN4              (0x43)
#define AXP_INTEN5              (0x44)
#define AXP_INTSTS1             (0x48)
#define AXP_INTSTS2             (0x49)
#define AXP_INTSTS3             (0x4A)
#define AXP_INTSTS4             (0x4B)
#define AXP_INTSTS5             (0x4C)

#define AXP_LDO_DC_EN1          (0X10)
#define AXP_LDO_DC_EN2          (0X12)
#define AXP_LDO_DC_EN3          (0X13)
#define AXP_DLDO1OUT_VOL        (0x15)
#define AXP_DLDO2OUT_VOL        (0x16)
#define AXP_DLDO3OUT_VOL        (0x17)
#define AXP_DLDO4OUT_VOL        (0x18)
#define AXP_ELDO1OUT_VOL        (0x19)
#define AXP_ELDO2OUT_VOL        (0x1A)
#define AXP_ELDO3OUT_VOL        (0x1B)
#define AXP_DC5LDOOUT_VOL       (0x1C)
#define AXP_DC1OUT_VOL          (0x21)
#define AXP_DC2OUT_VOL          (0x22)
#define AXP_DC3OUT_VOL          (0x23)
#define AXP_DC4OUT_VOL          (0x24)
#define AXP_DC5OUT_VOL          (0x25)
#define AXP_GPIO0LDOOUT_VOL     (0x91)
#define AXP_GPIO1LDOOUT_VOL     (0x93)
#define AXP_ALDO1OUT_VOL        (0x28)
#define AXP_ALDO2OUT_VOL        (0x29)
#define AXP_ALDO3OUT_VOL        (0x2A)
#define AXP_OFFLEVEL_DELAY      (0x37)
#define AXP_DCDC_FREQSET        (0x3B)

#define AXP_DCDC_MODESET        (0x80)
#define AXP_ADC_EN              (0x82)
#define AXP_HOTOVER_CTL         (0x8F)

#define AXP_GPIO0_CTL           (0x90)
#define AXP_GPIO1_CTL           (0x92)
#define AXP_GPIO01_SIGNAL       (0x94)
#define AXP_BAT_CHGCOULOMB3     (0xB0)
#define AXP_BAT_CHGCOULOMB2     (0xB1)
#define AXP_BAT_CHGCOULOMB1     (0xB2)
#define AXP_BAT_CHGCOULOMB0     (0xB3)
#define AXP_BAT_DISCHGCOULOMB3  (0xB4)
#define AXP_BAT_DISCHGCOULOMB2  (0xB5)
#define AXP_BAT_DISCHGCOULOMB1  (0xB6)
#define AXP_BAT_DISCHGCOULOMB0  (0xB7)
#define AXP_COULOMB_CTL         (0xB8)



/* bit definitions for AXP events ,irq event */
/*  AXP216*/
#define	AXP_IRQ_USBLO			( 1 <<  1)
#define	AXP_IRQ_USBRE			( 1 <<  2)
#define	AXP_IRQ_USBIN			( 1 <<  3)
#define	AXP_IRQ_USBOV     	( 1 <<  4)
#define	AXP_IRQ_ACRE     		( 1 <<  5)
#define	AXP_IRQ_ACIN     		( 1 <<  6)
#define	AXP_IRQ_ACOV     		( 1 <<  7)
#define	AXP_IRQ_TEMLO      	( 1 <<  8)
#define	AXP_IRQ_TEMOV      	( 1 <<  9)
#define	AXP_IRQ_CHAOV			( 1 << 10)
#define	AXP_IRQ_CHAST 	    ( 1 << 11)
#define	AXP_IRQ_BATATOU    	( 1 << 12)
#define	AXP_IRQ_BATATIN  		( 1 << 13)
#define AXP_IRQ_BATRE			( 1 << 14)
#define AXP_IRQ_BATIN		( 1 << 15)
#define AXP_IRQ_QBATINWORK	( 1 << 16)
#define AXP_IRQ_BATINWORK	( 1 << 17)
#define AXP_IRQ_QBATOVWORK	( 1 << 18)
#define AXP_IRQ_BATOVWORK	( 1 << 19)
#define AXP_IRQ_QBATINCHG	( 1 << 20)
#define AXP_IRQ_BATINCHG	( 1 << 21)
#define AXP_IRQ_QBATOVCHG	( 1 << 22)
#define AXP_IRQ_BATOVCHG	( 1 << 23)
#define AXP_IRQ_EXTLOWARN2  	( 1 << 24)
#define AXP_IRQ_EXTLOWARN1  	( 1 << 25)
#define AXP_IRQ_ICTEMOV    	( 1 << 31)
#define AXP_IRQ_GPIO0TG     	((uint64_t)1 << 32)
#define AXP_IRQ_GPIO1TG     	((uint64_t)1 << 33)
#define AXP_IRQ_POKLO     	((uint64_t)1 << 35)
#define AXP_IRQ_POKSH     	((uint64_t)1 << 36)

#define AXP_IRQ_PEKFE     	((uint64_t)1 << 37)
#define AXP_IRQ_PEKRE     	((uint64_t)1 << 38)
#define AXP_IRQ_TIMER     	((uint64_t)1 << 39)


/* Status Query Interface */
/*  AXP216 */
#define AXP_STATUS_SOURCE    	( 1 <<  0)
#define AXP_STATUS_ACUSBSH 	( 1 <<  1)
#define AXP_STATUS_BATCURDIR 	( 1 <<  2)
#define AXP_STATUS_USBLAVHO 	( 1 <<  3)
#define AXP_STATUS_USBVA    	( 1 <<  4)
#define AXP_STATUS_USBEN    	( 1 <<  5)
#define AXP_STATUS_ACVA	    ( 1 <<  6)
#define AXP_STATUS_ACEN	    ( 1 <<  7)

#define AXP_STATUS_BATINACT  	( 1 << 11)

#define AXP_STATUS_BATEN     	( 1 << 13)
#define AXP_STATUS_INCHAR    	( 1 << 14)
#define AXP_STATUS_ICTEMOV   	( 1 << 15)

#define AXP_LDO1_MIN		3000000
#define AXP_LDO1_MAX		3000000
#define AXP_ALDO1_MIN		700000
#define AXP_ALDO1_MAX		3300000
#define AXP_ALDO2_MIN		700000
#define AXP_ALDO2_MAX		3300000
#define AXP_ALDO3_MIN		700000
#define AXP_ALDO3_MAX		3300000

#define AXP_ELDO1_MIN		700000
#define AXP_ELDO1_MAX		3300000
#define AXP_ELDO2_MIN         700000
#define AXP_ELDO2_MAX         3300000


#define AXP_DCDC1_MIN		1600000
#define AXP_DCDC1_MAX		3400000
#define AXP_DCDC2_MIN		600000
#define AXP_DCDC2_MAX		1540000
#define AXP_DCDC3_MIN		600000
#define AXP_DCDC3_MAX		1860000
#define AXP_DCDC4_MIN		600000
#define AXP_DCDC4_MAX		1540000
#define AXP_DCDC5_MIN		1000000
#define AXP_DCDC5_MAX		2550000


#define GPIO_LDO1_MIN		700000
#define GPIO_LDO1_MAX		3300000

#endif /* __LINUX_AXP_MFD_216_H_ */
