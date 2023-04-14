#ifndef __LINUX_AXP_CFG_H_
#define __LINUX_AXP_CFG_H_
#include "axp-mfd.h"

/*Éè±¸µØÖ·*/
/*
	Ò»°ã²»¸Ä±ä£º
	AXP216:0x34
*/
#define	AXP_DEVICES_ADDR	(0x68 >> 1)
/*i2c¿ØÖÆÆ÷µÄÉè±¸ºÅ:¾ßÌå¿´ËùÊ¹ÓÃÆ½Ì¨Ó²¼þµÄÁ¬½Ó*/
//#define	AXP_I2CBUS			3 //for aa.
#define	AXP_I2CBUS			4      // for ab.
/*µçÔ´Ð¾Æ¬¶ÔÓ¦µÄÖÐ¶ÏºÅ£º¾ßÌå¿´ËùÊ¹ÓÃµÄÆ½Ì¨Ó²¼þµÄÁ¬½Ó£¬
ÖÐ¶ÏÏßnmiÁ¬½ÓcpuµÄÄÄÂ·irq»òÕßgpio*/
//#define AXP_IRQNO			IRQ_EINT(20)
#define AXP_IRQNO			(32*3+18) //PD(18) //6//192//192//64

/*³õÊ¼»¯¸÷Â·Êä³ö£¬µ¥Î»mV£¬0¶¼Îª¹Ø±Õ*/
/*
	ldo1£º
		ÓÉÓ²¼þ¾ö¶¨Êä³öµçÑ¹£¬Èí¼þ¸Ä²»ÁË£¬Ö»ÊÇÈí¼þµÄÏÔÊ¾µçÑ¹£º
*/
#define AXP_LDO1_VALUE			1800 // 3000
/*
	aldo1£º
		AXP:700~3300,100/step
*/
#define AXP_ALDO1_VALUE		1800
/*
	aldo2£º
		AXP:700~3300,100/step
*/
#define AXP_ALDO2_VALUE		1000
/*
	aldo3£º
		AXP:700~3300,100/step
*/
#define AXP_ALDO3_VALUE		3300

/*
	eldo1£º
		AXP:700~3300,100/step
*/
#define AXP_ELDO1_VALUE		3300
/*
	eldo2£º
		AXP:700~3300,100/step
*/
#define AXP_ELDO2_VALUE		3300


/*
	DCDC1:
		AXP:1600~3400,100/setp
*/
#define AXP_DCDC1_VALUE		3300
/*
	DCDC2£º
		AXP:600~1540£¬20/step
*/
#define AXP_DCDC2_VALUE		1200 //1100
/*
	DCDC3£º
		AXP:600~1860£¬20/step
*/
#define AXP_DCDC3_VALUE		900 //1100
/*
	DCDC4£º
		AXP:600~1540£¬20/step
*/
#define AXP_DCDC4_VALUE		1000
/*
	DCDC5£º
		AXP:1000~2550£¬50/step
*/
#define AXP_DCDC5_VALUE		1800 //1200

/*µç³ØÈÝÁ¿£¬mAh£º¸ù¾ÝÊµ¼Êµç³ØÈÝÁ¿À´¶¨Òå£¬¶Ô¿âÂØ¼Æ·½·¨À´Ëµ
Õâ¸ö²ÎÊýºÜÖØÒª£¬±ØÐëÅäÖÃ*/
#define BATCAP				1000/*752*/ /*680*/

/*³õÊ¼»¯µç³ØÄÚ×è£¬m¦¸£ºÒ»°ãÔÚ100~200Ö®¼ä£¬²»¹ý×îºÃ¸ù¾ÝÊµ¼Ê
²âÊÔ³öÀ´µÄÈ·¶¨£¬·½·¨ÊÇ´ò¿ª´òÓ¡ÐÅÏ¢£¬²»½Óµç³ØÉÕºÃ¹Ì¼þºó£¬
ÉÏµç³Ø£¬²»½Ó³äµçÆ÷£¬¿ª»ú£¬¿ª»ú1·ÖÖÓºó£¬½ÓÉÏ³äµçÆ÷£¬³ä
1~2·ÖÖÓ£¬¿´´òÓ¡ÐÅÏ¢ÖÐµÄrdcÖµ£¬ÌîÈëÕâÀï*/
#define BATRDC				203 //clivia 100
/*¿ªÂ·µçÑ¹·½·¨ÖÐµÄµç³ØµçÑ¹µÄ»º´æ*/
#define AXP_VOL_MAX			1
/*
	³äµç¹¦ÄÜÊ¹ÄÜ£º
        AXP:0-¹Ø±Õ£¬1-´ò¿ª
*/
#define CHGEN       1

/*
	³äµçµçÁ÷ÉèÖÃ£¬uA£¬0Îª¹Ø±Õ£º
		AXP:300~2550,100/step
*/
/*¿ª»ú³äµçµçÁ÷£¬uA*/
#define STACHGCUR			750*1000
/*¹ØÆÁ³äµçµçÁ÷£¬uA*/
#define EARCHGCUR			750*1000
/*ÐÝÃß³äµçµçÁ÷£¬uA*/
#define SUSCHGCUR			750*1000
/*¹Ø»ú³äµçµçÁ÷£¬uA*/
#define CLSCHGCUR			750*1000

/*Ä¿±ê³äµçµçÑ¹£¬mV*/
/*
	AXP:4100000/4200000/4240000/4350000
*/
#define CHGVOL				4350000
/*³äµçµçÁ÷Ð¡ÓÚÉèÖÃµçÁ÷µÄENDCHGRATE%Ê±£¬Í£Ö¹³äµç£¬%*/
/*
	AXP:10\15
*/
#define ENDCHGRATE			10
/*¹Ø»úµçÑ¹£¬mV*/
/*
	ÏµÍ³Éè¼ÆµÄ¹Ø»ú¹ýºóµÄµç³Ø¶ËµçÑ¹£¬ÐèÒªÓë¹Ø»ú°Ù·Ö±È¡¢
	¿ªÂ·µçÑ¹¶ÔÓ¦°Ù·Ö±È±í¼°µÍµç¾¯¸æµçÑ¹Ïà»¥ÅäºÏ²Å»áÓÐ×÷ÓÃ
*/
#define SHUTDOWNVOL			3300

/*adc²ÉÑùÂÊÉèÖÃ£¬Hz*/
/*
	AXP:100\200\400\800
*/
#define ADCFREQ				100
/*Ô¤³äµç³¬Ê±Ê±¼ä£¬min*/
/*
	AXP:40\50\60\70
*/
#define CHGPRETIME			50
/*ºãÁ÷³äµç³¬Ê±Ê±¼ä£¬min*/
/*
	AXP:360\480\600\720
*/
#define CHGCSTTIME			480


/*pek¿ª»úÊ±¼ä£¬ms*/
/*
	°´power¼üÓ²¼þ¿ª»úÊ±¼ä£º
		AXP:128/1000/2000/3000
*/
#define PEKOPEN				1000
/*pek³¤°´Ê±¼ä£¬ms*/
/*
	°´power¼ü·¢³¤°´ÖÐ¶ÏµÄÊ±¼ä£¬¶ÌÓÚ´ËÊ±¼äÊÇ¶Ì°´£¬·¢¶Ì°´¼üirq£¬
	³¤ÓÚ´ËÊ±¼äÊÇ³¤°´£¬·¢³¤°´¼üirq£º
		AXP:1000/1500/2000/2500
*/
#define PEKLONG				1500
/*pek³¤°´¹Ø»úÊ¹ÄÜ*/
/*
	°´power¼ü³¬¹ý¹Ø»úÊ±³¤Ó²¼þ¹Ø»ú¹¦ÄÜÊ¹ÄÜ£º
		AXP:0-²»¹Ø£¬1-¹Ø»ú
*/
#define PEKOFFEN			1
/*pek³¤°´¹Ø»úÊ¹ÄÜºó¿ª»úÑ¡Ôñ*/
/*
	°´power¼ü³¬¹ý¹Ø»úÊ±³¤Ó²¼þ¹Ø»ú»¹ÊÇÖØÆôÑ¡Ôñ:
		AXP:0-Ö»¹Ø»ú²»ÖØÆô£¬1-¹Ø»úºóÖØÆô
*/
#define PEKOFFRESTART			0
/*pekpwrÑÓ³ÙÊ±¼ä£¬ms*/
/*
	¿ª»úºópowerokÐÅºÅµÄÑÓ³ÙÊ±¼ä£º
		AXP20:8/16/32/64
*/
#define PEKDELAY			32
/*pek³¤°´¹Ø»úÊ±¼ä£¬ms*/
/*
	°´power¼üµÄ¹Ø»úÊ±³¤£º
		AXP:4000/6000/8000/10000
*/
#define PEKOFF				6000
/*¹ýÎÂ¹Ø»úÊ¹ÄÜ*/
/*
	AXPÄÚ²¿ÎÂ¶È¹ý¸ßÓ²¼þ¹Ø»ú¹¦ÄÜÊ¹ÄÜ£º
		AXP:0-²»¹Ø£¬1-¹Ø»ú
*/
#define OTPOFFEN			0
/* ³äµçµçÑ¹ÏÞÖÆÊ¹ÄÜ*/
/*
	AXP:0-¹Ø±Õ£¬1-´ò¿ª
*/
#define USBVOLLIMEN		1
/*  ³äµçÏÞÑ¹£¬mV£¬0Îª²»ÏÞÖÆ*/
/*
	AXP:4000~4700£¬100/step
*/
#define USBVOLLIM			4500
/*  USB³äµçÏÞÑ¹£¬mV£¬0Îª²»ÏÞÖÆ*/
/*
	AXP:4000~4700£¬100/step
*/
#define USBVOLLIMPC			4700

/* ³äµçµçÁ÷ÏÞÖÆÊ¹ÄÜ*/
/*
	AXP:0-¹Ø±Õ£¬1-´ò¿ª
*/
#define USBCURLIMEN		1
/* ³äµçÏÞÁ÷£¬mA£¬0Îª²»ÏÞÖÆ*/
/*
	AXP:500/900
*/
#define USBCURLIM			0
/* usb ³äµçÏÞÁ÷£¬mA£¬0Îª²»ÏÞÖÆ*/
/*
	AXP:500/900
*/
#define USBCURLIMPC			500
/* PMU ÖÐ¶Ï´¥·¢»½ÐÑÊ¹ÄÜ*/
/*
	AXP:0-²»»½ÐÑ£¬1-»½ÐÑ
*/
#define IRQWAKEUP			0
/* N_VBUSEN PIN ¹¦ÄÜ¿ØÖÆ*/
/*
	AXP:0-Êä³ö£¬Çý¶¯OTGÉýÑ¹Ä£¿é£¬1-ÊäÈë£¬¿ØÖÆVBUSÍ¨Â·
*/
#define VBUSEN			1
/* ACIN/VBUS In-short ¹¦ÄÜÉèÖÃ*/
/*
	AXP:0-AC VBUS·Ö¿ª£¬1-Ê¹ÓÃVBUSµ±AC,ÎÞµ¥¶ÀAC
*/
#define VBUSACINSHORT			0
/* CHGLED ¹Ü½Å¿ØÖÆÉèÖÃ*/
/*
	AXP:0-Çý¶¯Âí´ï£¬1-ÓÉ³äµç¹¦ÄÜ¿ØÖÆ
*/
#define CHGLEDFUN			0
/* CHGLED LED ÀàÐÍÉèÖÃ*/
/*
	AXP:0-³äµçÊ±led³¤ÁÁ£¬1-³äµçÊ±ledÉÁË¸
*/
#define CHGLEDTYPE			0
/* µç³Ø×ÜÈÝÁ¿Ð£ÕýÊ¹ÄÜ*/
/*
	AXP:0-¹Ø±Õ£¬1-´ò¿ª
*/
#define BATCAPCORRENT			1
/* ³äµçÍê³Éºó£¬³äµçÊä³öÊ¹ÄÜ*/
/*
	AXP:0-¹Ø±Õ£¬1-´ò¿ª
*/
#define BATREGUEN			0
/* µç³Ø¼ì²â¹¦ÄÜÊ¹ÄÜ*/
/*
	AXP:0-¹Ø±Õ£¬1-´ò¿ª
*/
#define BATDET		1
/* PMUÖØÖÃÊ¹ÄÜ*/
/*
	AXP:0-¹Ø±Õ£¬1-´ò¿ª°´µçÔ´¼ü16ÃëÖØÖÃPMU¹¦ÄÜ
*/
#define PMURESET		0
/*µÍµç¾¯¸æµçÑ¹1£¬%*/
/*
	¸ù¾ÝÏµÍ³Éè¼ÆÀ´¶¨£º
	AXP:5%~20%
*/
#define BATLOWLV1    15
/*µÍµç¾¯¸æµçÑ¹2£¬%*/
/*
	¸ù¾ÝÏµÍ³Éè¼ÆÀ´¶¨£º
	AXP:0%~15%
*/
#define BATLOWLV2    0

#define ABS(x)				((x) >0 ? (x) : -(x) )

#ifdef	CONFIG_KP_AXP
/*AXP GPIO start NUM,¸ù¾ÝÆ½Ì¨Êµ¼ÊÇé¿ö¶¨Òå*/
#define AXP_NR_BASE 100

/*AXP GPIO NUM,°üÀ¨Âí´ïÇý¶¯¡¢LCD powerÒÔ¼°VBUS driver pin*/
#define AXP_NR 5

/*³õÊ¼»¯¿ªÂ·µçÑ¹¶ÔÓ¦°Ù·Ö±È±í*/
/*
	¿ÉÒÔÊ¹ÓÃÄ¬ÈÏÖµ£¬µ«ÊÇ×îºÃ¸ù¾ÝÊµ¼Ê²âÊÔµÄµç³ØÀ´È·¶¨Ã¿¼¶
	¶ÔÓ¦µÄÊ£Óà°Ù·Ö±È£¬ÌØ±ðÐèÒª×¢Òâ£¬¹Ø»úµçÑ¹SHUTDOWNVOLºÍµç³Ø
	ÈÝÁ¿¿ªÊ¼Ð£×¼Ê£ÓàÈÝÁ¿°Ù·Ö±ÈBATCAPCORRATEÕâÁ½¼¶µÄ×¼È·ÐÔ
	AXPÊÊÓÃ
*/
#define OCVREG0				0		 //3.13V
#define OCVREG1				0		 //3.27V
#define OCVREG2				0		 //3.34V
#define OCVREG3				0		 //3.41V
#define OCVREG4				5		 //3.48V
#define OCVREG5				12		 //3.52V
#define OCVREG6				23		 //3.55V
#define OCVREG7				32		 //3.57V
#define OCVREG8				39		 //3.59V
#define OCVREG9				44		 //3.61V
#define OCVREGA				52		 //3.63V
#define OCVREGB				54		 //3.64V
#define OCVREGC				58		 //3.66V
#define OCVREGD				63		 //3.7V
#define OCVREGE				67		 //3.73V
#define OCVREGF				70		 //3.77V
#define OCVREG10		 	72                //3.78V
#define OCVREG11		 	74                //3.8V
#define OCVREG12		 	76                //3.82V
#define OCVREG13		 	79                //3.84V
#define OCVREG14		 	82                //3.85V
#define OCVREG15		 	84                //3.87V
#define OCVREG16		 	87                //3.91V
#define OCVREG17		 	90                //3.94V
#define OCVREG18		 	92                //3.98V
#define OCVREG19		 	93                //4.01V
#define OCVREG1A		 	95                //4.05V
#define OCVREG1B		 	96                //4.08V
#define OCVREG1C		 	98                //4.1V
#define OCVREG1D		 	100                //4.12V
#define OCVREG1E		 	100                //4.14V
#define OCVREG1F		 	100                //4.15V

#endif

#ifdef AXP_INPUT_PWR_KEY
static const uint64_t AXP_NOTIFIER_ON = (AXP_IRQ_USBIN |AXP_IRQ_USBRE |
				       		                            AXP_IRQ_ACIN |AXP_IRQ_ACRE |
				       		                            AXP_IRQ_BATIN |AXP_IRQ_BATRE |
				       		                            AXP_IRQ_CHAST |AXP_IRQ_CHAOV |
						                            (uint64_t)AXP_IRQ_PEKFE |(uint64_t)AXP_IRQ_PEKRE);
#else
/*Ñ¡ÔñÐèÒª´ò¿ªµÄÖÐ¶ÏÊ¹ÄÜ*/
static const uint64_t AXP_NOTIFIER_ON = (AXP_IRQ_USBIN |AXP_IRQ_USBRE |
				       		                            AXP_IRQ_ACIN |AXP_IRQ_ACRE |
				       		                            AXP_IRQ_BATIN |AXP_IRQ_BATRE |
				       		                            AXP_IRQ_CHAST |AXP_IRQ_CHAOV /*|
						                            (uint64_t)AXP_IRQ_PEKFE |(uint64_t)AXP_IRQ_PEKRE*/);
#endif
/* ÐèÒª×ö²åÈë»ðÅ£¡¢usb¹Ø»úÖØÆô½øbootÊ±power_startÉèÖÃÎª1£¬·ñÔòÎª0*/
#define POWER_START 1
#define RESET_WHEN_ACIN 1	//clivia add ÔÚ¹Ø»úÊ±ac in »òÕßusb in£¬reset

#endif
