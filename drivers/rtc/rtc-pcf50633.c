#include <linux/rtc.h>
#include <linux/platform_device.h>
#include <linux/bcd.h>
#include <linux/pcf50633.h>
#include <linux/rtc/pcf50633.h>
#include <linux/i2c.h>

enum pcf50633_time_indexes {
	PCF50633_TI_SEC = 0,
	PCF50633_TI_MIN,
	PCF50633_TI_HOUR,
	PCF50633_TI_WKDAY,
	PCF50633_TI_DAY,
	PCF50633_TI_MONTH,
	PCF50633_TI_YEAR,
	PCF50633_TI_EXTENT /* always last */
};


struct pcf50633_time {
	u_int8_t time[PCF50633_TI_EXTENT];
};

static void pcf2rtc_time(struct rtc_time *rtc, struct pcf50633_time *pcf)
{
	rtc->tm_sec = bcd2bin(pcf->time[PCF50633_TI_SEC]);
	rtc->tm_min = bcd2bin(pcf->time[PCF50633_TI_MIN]);
	rtc->tm_hour = bcd2bin(pcf->time[PCF50633_TI_HOUR]);
	rtc->tm_wday = bcd2bin(pcf->time[PCF50633_TI_WKDAY]);
	rtc->tm_mday = bcd2bin(pcf->time[PCF50633_TI_DAY]);
	rtc->tm_mon = bcd2bin(pcf->time[PCF50633_TI_MONTH]);
	rtc->tm_year = bcd2bin(pcf->time[PCF50633_TI_YEAR]) + 100;
}

static void rtc2pcf_time(struct pcf50633_time *pcf, struct rtc_time *rtc)
{
	pcf->time[PCF50633_TI_SEC] = bin2bcd(rtc->tm_sec);
	pcf->time[PCF50633_TI_MIN] = bin2bcd(rtc->tm_min);
	pcf->time[PCF50633_TI_HOUR] = bin2bcd(rtc->tm_hour);
	pcf->time[PCF50633_TI_WKDAY] = bin2bcd(rtc->tm_wday);
	pcf->time[PCF50633_TI_DAY] = bin2bcd(rtc->tm_mday);
	pcf->time[PCF50633_TI_MONTH] = bin2bcd(rtc->tm_mon);
	pcf->time[PCF50633_TI_YEAR] = bin2bcd(rtc->tm_year - 100);
}

static int pcf50633_rtc_ioctl(struct device *dev, unsigned int cmd,
			      unsigned long arg)
{
	struct pcf50633_data *pcf = dev->platform_data;

	switch (cmd) {
	case RTC_AIE_OFF:
		/* disable the alarm interrupt */
		pcf50633_reg_set_bit_mask(pcf, PCF50633_REG_INT1M,
				 PCF50633_INT1_ALARM, PCF50633_INT1_ALARM);
		return 0;
	case RTC_AIE_ON:
		/* enable the alarm interrupt */
		pcf50633_reg_clear_bits(pcf, PCF50633_REG_INT1M, PCF50633_INT1_ALARM);
		return 0;
	case RTC_PIE_OFF:
		/* disable periodic interrupt (hz tick) */
		pcf->flags &= ~PCF50633_F_RTC_SECOND;
		pcf50633_reg_set_bit_mask(pcf, PCF50633_REG_INT1M,
				 PCF50633_INT1_SECOND, PCF50633_INT1_SECOND);
		return 0;
	case RTC_PIE_ON:
		/* ensable periodic interrupt (hz tick) */
		pcf->flags |= PCF50633_F_RTC_SECOND;
		pcf50633_reg_clear_bits(pcf, PCF50633_REG_INT1M, PCF50633_INT1_SECOND);
		return 0;
	}
	return -ENOIOCTLCMD;
}

#ifdef PCF50633_RTC
void pcf50633_rtc_handle_event(struct pcf50633_data *pcf,
					enum pcf50633_rtc_event evt)
{
	switch(evt) {
		case PCF50633_RTC_EVENT_ALARM:
			rtc_update_irq(pcf->rtc, 1, RTC_AF | RTC_IRQF);
			break;
		case PCF50633_RTC_EVENT_SECOND:
			rtc_update_irq(pcf->rtc, 1, RTC_PF | RTC_IRQF);
	}
}
#else
void pcf50633_rtc_handle_event(struct pcf50633_data *pcf,
					enum pcf50633_rtc_event evt)
{

}
#endif

static int pcf50633_rtc_read_time(struct device *dev, struct rtc_time *tm)
{
	struct pcf50633_data *pcf = dev->platform_data;
	struct pcf50633_time pcf_tm;
	int ret;

	mutex_lock(&pcf->lock);

	ret = i2c_smbus_read_i2c_block_data(pcf->client,
					    PCF50633_REG_RTCSC,
					    PCF50633_TI_EXTENT,
					    &pcf_tm.time[0]);
	if (ret != PCF50633_TI_EXTENT)
		dev_err(dev, "Failed to read time :-(\n");

	mutex_unlock(&pcf->lock);

	dev_dbg(dev, "PCF_TIME: %02x.%02x.%02x %02x:%02x:%02x\n",
		pcf_tm.time[PCF50633_TI_DAY],
		pcf_tm.time[PCF50633_TI_MONTH],
		pcf_tm.time[PCF50633_TI_YEAR],
		pcf_tm.time[PCF50633_TI_HOUR],
		pcf_tm.time[PCF50633_TI_MIN],
		pcf_tm.time[PCF50633_TI_SEC]);

	pcf2rtc_time(tm, &pcf_tm);

	dev_dbg(dev, "RTC_TIME: %u.%u.%u %u:%u:%u\n",
		tm->tm_mday, tm->tm_mon, tm->tm_year,
		tm->tm_hour, tm->tm_min, tm->tm_sec);

	return 0;
}

static int pcf50633_rtc_set_time(struct device *dev, struct rtc_time *tm)
{
	struct pcf50633_data *pcf = dev->platform_data;
	struct pcf50633_time pcf_tm;
	int ret;

	dev_dbg(dev, "RTC_TIME: %u.%u.%u %u:%u:%u\n",
		tm->tm_mday, tm->tm_mon, tm->tm_year,
		tm->tm_hour, tm->tm_min, tm->tm_sec);
	rtc2pcf_time(&pcf_tm, tm);
	dev_dbg(dev, "PCF_TIME: %02x.%02x.%02x %02x:%02x:%02x\n",
		pcf_tm.time[PCF50633_TI_DAY],
		pcf_tm.time[PCF50633_TI_MONTH],
		pcf_tm.time[PCF50633_TI_YEAR],
		pcf_tm.time[PCF50633_TI_HOUR],
		pcf_tm.time[PCF50633_TI_MIN],
		pcf_tm.time[PCF50633_TI_SEC]);

	mutex_lock(&pcf->lock);
	/* FIXME: disable second interrupt */

	ret = i2c_smbus_write_i2c_block_data(pcf->client,
					     PCF50633_REG_RTCSC,
					     PCF50633_TI_EXTENT,
					     &pcf_tm.time[0]);
	if (ret)
		dev_err(dev, "Failed to set time %d\n", ret);

	/* FIXME: re-enable second interrupt */
	mutex_unlock(&pcf->lock);

	return 0;
}

static int pcf50633_rtc_read_alarm(struct device *dev, struct rtc_wkalrm *alrm)
{
	struct pcf50633_data *pcf = dev->platform_data;
	struct pcf50633_time pcf_tm;
	int ret;
	u_int8_t reg;

	mutex_lock(&pcf->lock);
	
	pcf50633_read(pcf, PCF50633_REG_INT1M, 1, &reg);     
	alrm->enabled = reg & PCF50633_INT1_ALARM ? 0 : 1;

	ret = pcf50633_read(pcf, PCF50633_REG_RTCSCA,
				PCF50633_TI_EXTENT, &pcf_tm.time[0]);

	if (ret != PCF50633_TI_EXTENT)
		dev_err(dev, "Failed to read Alarm time :-(\n");

	mutex_unlock(&pcf->lock);

	pcf2rtc_time(&alrm->time, &pcf_tm);

	return 0;
}

static int pcf50633_rtc_set_alarm(struct device *dev, struct rtc_wkalrm *alrm)
{
	struct pcf50633_data *pcf = dev->platform_data;
	struct pcf50633_time pcf_tm;
	u_int8_t irqmask;
	int ret;

	rtc2pcf_time(&pcf_tm, &alrm->time);

	mutex_lock(&pcf->lock);

	/* disable alarm interrupt */
	pcf50633_read(pcf, PCF50633_REG_INT1M, 1, &irqmask);
	irqmask |= PCF50633_INT1_ALARM;
	pcf50633_write(pcf, PCF50633_REG_INT1M, 1, &irqmask);

	ret = pcf50633_write(pcf, PCF50633_REG_RTCSCA,
					PCF50633_TI_EXTENT, &pcf_tm.time[0]);
	if (ret)
		dev_err(dev, "Failed to write alarm time :-( %d\n", ret);

	if (alrm->enabled) {
		/* (re-)enaable alarm interrupt */
		pcf50633_read(pcf, PCF50633_REG_INT1M, 1, &irqmask);
		irqmask &= ~PCF50633_INT1_ALARM;
		pcf50633_write(pcf, PCF50633_REG_INT1M, 1, &irqmask);
	}

	mutex_unlock(&pcf->lock);

	/* FIXME */
	return 0;
}
static struct rtc_class_ops pcf50633_rtc_ops = {
	.ioctl		= pcf50633_rtc_ioctl,
	.read_time	= pcf50633_rtc_read_time,
	.set_time	= pcf50633_rtc_set_time,
	.read_alarm	= pcf50633_rtc_read_alarm,
	.set_alarm	= pcf50633_rtc_set_alarm,
};

static int pcf50633_rtc_probe(struct platform_device *pdev)
{
	struct rtc_device *rtc;

	rtc = rtc_device_register("pcf50633", &pdev->dev,
					&pcf50633_rtc_ops, THIS_MODULE);
	if (IS_ERR(rtc))
		return -ENODEV;
	
	return 0;
}

static int pcf50633_rtc_remove(struct platform_device *pdev)
{
	return 0;
}


static struct platform_driver pcf50633_rtc_driver = {
	.driver = {
		.name = "pcf50633-rtc",
	},
	.probe = pcf50633_rtc_probe,
	.remove = __devexit_p(pcf50633_rtc_remove),
};

static int __init pcf50633_rtc_init(void)
{
	return platform_driver_register(&pcf50633_rtc_driver);
}
module_init(pcf50633_rtc_init);

static void __exit pcf50633_rtc_exit(void)
{
	platform_driver_unregister(&pcf50633_rtc_driver);
}
module_exit(pcf50633_rtc_exit);


MODULE_DESCRIPTION("RTC driver for NXP PCF50633 power management unit");
MODULE_AUTHOR("Balaji Rao <balajirrao@openmoko.org>");
MODULE_LICENSE("GPL");

