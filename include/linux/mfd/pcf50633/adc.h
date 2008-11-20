#ifndef __LINUX_MFD_PCF50633_ADC_H
#define __LINUX_MFD_PCF50633_ADC_H

#include <linux/platform_device.h>

/* ADC Registers */
#define	PCF50633_REG_ADCC3 0x52
#define	PCF50633_REG_ADCC2 0x53
#define	PCF50633_REG_ADCC1 0x54
#define	PCF50633_REG_ADCS1 0x55
#define	PCF50633_REG_ADCS2 0x56
#define	PCF50633_REG_ADCS3 0x57

#define	PCF50633_ADCC1_ADCSTART		0x01
#define	PCF50633_ADCC1_RES_10BIT	0x02
#define	PCF50633_ADCC1_AVERAGE_NO	0x00
#define	PCF50633_ADCC1_AVERAGE_4	0x04
#define	PCF50633_ADCC1_AVERAGE_8	0x08
#define	PCF50633_ADCC1_AVERAGE_16	0x0c
#define	PCF50633_ADCC1_MUX_BATSNS_RES	0x00
#define	PCF50633_ADCC1_MUX_BATSNS_SUBTR	0x10
#define	PCF50633_ADCC1_MUX_ADCIN2_RES	0x20
#define	PCF50633_ADCC1_MUX_ADCIN2_SUBTR	0x30
#define	PCF50633_ADCC1_MUX_BATTEMP	0x60
#define	PCF50633_ADCC1_MUX_ADCIN1	0x70
#define PCF50633_ADCC1_AVERAGE_MASK	0x0c
#define	PCF50633_ADCC1_ADCMUX_MASK	0xf0

#define	PCF50633_ADCC2_RATIO_NONE	0x00
#define	PCF50633_ADCC2_RATIO_BATTEMP	0x01
#define	PCF50633_ADCC2_RATIO_ADCIN1	0x02
#define	PCF50633_ADCC2_RATIO_BOTH	0x03
#define	PCF50633_ADCC2_RATIOSETTL_100US	0x04

#define	PCF50633_ADCC3_ACCSW_EN		0x01
#define	PCF50633_ADCC3_NTCSW_EN		0x04
#define	PCF50633_ADCC3_RES_DIV_TWO	0x10
#define	PCF50633_ADCC3_RES_DIV_THREE	0x00

#define	PCF50633_ADCS3_REF_NTCSW	0x00
#define	PCF50633_ADCS3_REF_ACCSW	0x10
#define	PCF50633_ADCS3_REF_2V0		0x20
#define	PCF50633_ADCS3_REF_VISA		0x30
#define	PCF50633_ADCS3_REF_2V0_2	0x70
#define	PCF50633_ADCS3_ADCRDY		0x80

#define PCF50633_ADCS3_ADCDAT1L_MASK	0x03
#define PCF50633_ADCS3_ADCDAT2L_MASK	0x0c
#define PCF50633_ADCS3_ADCDAT2L_SHIFT	2
#define PCF50633_ASCS3_REF_MASK		0x70


struct pcf50633;

#define PCF50633_MAX_ADC_FIFO_DEPTH 8

struct pcf50633_adc_request;

struct pcf50633_adc {
	struct platform_device *pdev;

	/* Private stuff */
	struct pcf50633_adc_request *queue[PCF50633_MAX_ADC_FIFO_DEPTH];
	int queue_head;
	int queue_tail;
	struct mutex queue_mutex;
};

#ifdef CONFIG_MFD_PCF50633_ADC
extern int
pcf50633_adc_async_read(struct pcf50633 *pcf, int mux, int avg,
		void (*callback)(struct pcf50633 *, void *, int),
		void *callback_param);
extern int
pcf50633_adc_sync_read(struct pcf50633 *pcf, int mux, int avg);
#else
int
pcf50633_adc_async_read(struct pcf50633 *pcf, int mux, int avg,
		void (*callback)(struct pcf50633 *, void *, int),
		void *callback_param)
{
	return -ENODEV;
}

int
pcf50633_adc_sync_read(struct pcf50633 *pcf, int mux, int avg)
{
	return -ENODEV;
}
#endif /* CONFIG_PCF50633_ADC */

#endif /* __LINUX_PCF50633_ADC_H */
