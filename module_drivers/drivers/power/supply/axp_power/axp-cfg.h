#ifndef __LINUX_AXP_CFG_H_
#define __LINUX_AXP_CFG_H_
#include "axp-mfd.h"

/*设备地址*/
/*
	一般不改变：
	AXP216:0x34
*/
#define	AXP_DEVICES_ADDR	(0x68 >> 1)
/*i2c控制器的设备号:具体看所使用平台硬件的连接*/
//#define	AXP_I2CBUS			3 //for aa.
#define	AXP_I2CBUS			4      // for ab.
/*电源芯片对应的中断号：具体看所使用的平台硬件的连接，
中断线nmi连接cpu的哪路irq或者gpio*/
//#define AXP_IRQNO			IRQ_EINT(20)
#define AXP_IRQNO			(32*3+18) //PD(18) //6//192//192//64

/*初始化各路输出，单位mV，0都为关闭*/
/*
	ldo1：
		由硬件决定输出电压，软件改不了，只是软件的显示电压：
*/
#define AXP_LDO1_VALUE			1800 // 3000
/*
	aldo1：
		AXP:700~3300,100/step
*/
#define AXP_ALDO1_VALUE		1800
/*
	aldo2：
		AXP:700~3300,100/step
*/
#define AXP_ALDO2_VALUE		1000
/*
	aldo3：
		AXP:700~3300,100/step
*/
#define AXP_ALDO3_VALUE		3300

/*
	eldo1：
		AXP:700~3300,100/step
*/
#define AXP_ELDO1_VALUE		3300
/*
	eldo2：
		AXP:700~3300,100/step
*/
#define AXP_ELDO2_VALUE		3300


/*
	DCDC1:
		AXP:1600~3400,100/setp
*/
#define AXP_DCDC1_VALUE		3300
/*
	DCDC2：
		AXP:600~1540，20/step
*/
#define AXP_DCDC2_VALUE		1200 //1100
/*
	DCDC3：
		AXP:600~1860，20/step
*/
#define AXP_DCDC3_VALUE		900 //1100
/*
	DCDC4：
		AXP:600~1540，20/step
*/
#define AXP_DCDC4_VALUE		1000
/*
	DCDC5：
		AXP:1000~2550，50/step
*/
#define AXP_DCDC5_VALUE		1800 //1200

/*电池容量，mAh：根据实际电池容量来定义，对库仑计方法来说
这个参数很重要，必须配置*/
#define BATCAP				1000/*752*/ /*680*/

/*初始化电池内阻，mΩ：一般在100~200之间，不过最好根据实际
测试出来的确定，方法是打开打印信息，不接电池烧好固件后，
上电池，不接充电器，开机，开机1分钟后，接上充电器，充
1~2分钟，看打印信息中的rdc值，填入这里*/
#define BATRDC				203 //clivia 100
/*开路电压方法中的电池电压的缓存*/
#define AXP_VOL_MAX			1
/*
	充电功能使能：
        AXP:0-关闭，1-打开
*/
#define CHGEN       1

/*
	充电电流设置，uA，0为关闭：
		AXP:300~2550,100/step
*/
/*开机充电电流，uA*/
#define STACHGCUR			750*1000
/*关屏充电电流，uA*/
#define EARCHGCUR			750*1000
/*休眠充电电流，uA*/
#define SUSCHGCUR			750*1000
/*关机充电电流，uA*/
#define CLSCHGCUR			750*1000

/*目标充电电压，mV*/
/*
	AXP:4100000/4200000/4240000/4350000
*/
#define CHGVOL				4350000
/*充电电流小于设置电流的ENDCHGRATE%时，停止充电，%*/
/*
	AXP:10\15
*/
#define ENDCHGRATE			10
/*关机电压，mV*/
/*
	系统设计的关机过后的电池端电压，需要与关机百分比、
	开路电压对应百分比表及低电警告电压相互配合才会有作用
*/
#define SHUTDOWNVOL			3300

/*adc采样率设置，Hz*/
/*
	AXP:100\200\400\800
*/
#define ADCFREQ				100
/*预充电超时时间，min*/
/*
	AXP:40\50\60\70
*/
#define CHGPRETIME			50
/*恒流充电超时时间，min*/
/*
	AXP:360\480\600\720
*/
#define CHGCSTTIME			480


/*pek开机时间，ms*/
/*
	按power键硬件开机时间：
		AXP:128/1000/2000/3000
*/
#define PEKOPEN				1000
/*pek长按时间，ms*/
/*
	按power键发长按中断的时间，短于此时间是短按，发短按键irq，
	长于此时间是长按，发长按键irq：
		AXP:1000/1500/2000/2500
*/
#define PEKLONG				1500
/*pek长按关机使能*/
/*
	按power键超过关机时长硬件关机功能使能：
		AXP:0-不关，1-关机
*/
#define PEKOFFEN			1
/*pek长按关机使能后开机选择*/
/*
	按power键超过关机时长硬件关机还是重启选择:
		AXP:0-只关机不重启，1-关机后重启
*/
#define PEKOFFRESTART			0
/*pekpwr延迟时间，ms*/
/*
	开机后powerok信号的延迟时间：
		AXP20:8/16/32/64
*/
#define PEKDELAY			32
/*pek长按关机时间，ms*/
/*
	按power键的关机时长：
		AXP:4000/6000/8000/10000
*/
#define PEKOFF				6000
/*过温关机使能*/
/*
	AXP内部温度过高硬件关机功能使能：
		AXP:0-不关，1-关机
*/
#define OTPOFFEN			0
/* 充电电压限制使能*/
/*
	AXP:0-关闭，1-打开
*/
#define USBVOLLIMEN		1
/*  充电限压，mV，0为不限制*/
/*
	AXP:4000~4700，100/step
*/
#define USBVOLLIM			4500
/*  USB充电限压，mV，0为不限制*/
/*
	AXP:4000~4700，100/step
*/
#define USBVOLLIMPC			4700

/* 充电电流限制使能*/
/*
	AXP:0-关闭，1-打开
*/
#define USBCURLIMEN		1
/* 充电限流，mA，0为不限制*/
/*
	AXP:500/900
*/
#define USBCURLIM			0
/* usb 充电限流，mA，0为不限制*/
/*
	AXP:500/900
*/
#define USBCURLIMPC			500
/* PMU 中断触发唤醒使能*/
/*
	AXP:0-不唤醒，1-唤醒
*/
#define IRQWAKEUP			0
/* N_VBUSEN PIN 功能控制*/
/*
	AXP:0-输出，驱动OTG升压模块，1-输入，控制VBUS通路
*/
#define VBUSEN			1
/* ACIN/VBUS In-short 功能设置*/
/*
	AXP:0-AC VBUS分开，1-使用VBUS当AC,无单独AC
*/
#define VBUSACINSHORT			0
/* CHGLED 管脚控制设置*/
/*
	AXP:0-驱动马达，1-由充电功能控制
*/
#define CHGLEDFUN			0
/* CHGLED LED 类型设置*/
/*
	AXP:0-充电时led长亮，1-充电时led闪烁
*/
#define CHGLEDTYPE			0
/* 电池总容量校正使能*/
/*
	AXP:0-关闭，1-打开
*/
#define BATCAPCORRENT			1
/* 充电完成后，充电输出使能*/
/*
	AXP:0-关闭，1-打开
*/
#define BATREGUEN			0
/* 电池检测功能使能*/
/*
	AXP:0-关闭，1-打开
*/
#define BATDET		1
/* PMU重置使能*/
/*
	AXP:0-关闭，1-打开按电源键16秒重置PMU功能
*/
#define PMURESET		0
/*低电警告电压1，%*/
/*
	根据系统设计来定：
	AXP:5%~20%
*/
#define BATLOWLV1    15
/*低电警告电压2，%*/
/*
	根据系统设计来定：
	AXP:0%~15%
*/
#define BATLOWLV2    0

#define ABS(x)				((x) >0 ? (x) : -(x) )

#ifdef	CONFIG_KP_AXP
/*AXP GPIO start NUM,根据平台实际情况定义*/
#define AXP_NR_BASE 100

/*AXP GPIO NUM,包括马达驱动、LCD power以及VBUS driver pin*/
#define AXP_NR 5

/*初始化开路电压对应百分比表*/
/*
	可以使用默认值，但是最好根据实际测试的电池来确定每级
	对应的剩余百分比，特别需要注意，关机电压SHUTDOWNVOL和电池
	容量开始校准剩余容量百分比BATCAPCORRATE这两级的准确性
	AXP适用
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
/*选择需要打开的中断使能*/
static const uint64_t AXP_NOTIFIER_ON = (AXP_IRQ_USBIN |AXP_IRQ_USBRE |
				       		                            AXP_IRQ_ACIN |AXP_IRQ_ACRE |
				       		                            AXP_IRQ_BATIN |AXP_IRQ_BATRE |
				       		                            AXP_IRQ_CHAST |AXP_IRQ_CHAOV /*|
						                            (uint64_t)AXP_IRQ_PEKFE |(uint64_t)AXP_IRQ_PEKRE*/);
#endif
/* 需要做插入火牛、usb关机重启进boot时power_start设置为1，否则为0*/
#define POWER_START 1
#define RESET_WHEN_ACIN 1	//clivia add 在关机时ac in 或者usb in，reset

#endif
