#include <linux/mfd/pcf50633/core.h>
#include <linux/mfd/pcf50633/adc.h>

struct pcf50633_adc_request {
	int mux;
	int avg;
	int result;
	void (*callback)(struct pcf50633 *, void *, int);
	void *callback_param;

	/* Used in case of sync requests */
	struct completion completion;

};

static void adc_read_setup(struct pcf50633 *pcf,
				 int channel, int avg)
{
	channel &= PCF50633_ADCC1_ADCMUX_MASK;

	/* kill ratiometric, but enable ACCSW biasing */
	pcf50633_reg_write(pcf, PCF50633_REG_ADCC2, 0x00);
	pcf50633_reg_write(pcf, PCF50633_REG_ADCC3, 0x01);

	/* start ADC conversion of selected channel */
	pcf50633_reg_write(pcf, PCF50633_REG_ADCC1, channel | avg |
		    PCF50633_ADCC1_ADCSTART | PCF50633_ADCC1_RES_10BIT);

}

static void trigger_next_adc_job_if_any(struct pcf50633 *pcf)
{
	int head, tail;

	mutex_lock(&pcf->adc.queue_mutex);

	head = pcf->adc.queue_head;
	tail = pcf->adc.queue_tail;

	if (!pcf->adc.queue[head])
		goto out;

	adc_read_setup(pcf, pcf->adc.queue[head]->mux,
				pcf->adc.queue[head]->avg);
out:
	mutex_unlock(&pcf->adc.queue_mutex);
}

static int
adc_enqueue_request(struct pcf50633 *pcf, struct pcf50633_adc_request *req)
{
	int head, tail;

	mutex_lock(&pcf->adc.queue_mutex);
	head = pcf->adc.queue_head;
	tail = pcf->adc.queue_tail;

	if (pcf->adc.queue[tail]) {
		mutex_unlock(&pcf->adc.queue_mutex);
		return -EBUSY;
	}

	pcf->adc.queue[tail] = req;

	pcf->adc.queue_tail =
		(tail + 1) & (PCF50633_MAX_ADC_FIFO_DEPTH - 1);

	mutex_unlock(&pcf->adc.queue_mutex);

	trigger_next_adc_job_if_any(pcf);

	return 0;
}

static void
pcf50633_adc_sync_read_callback(struct pcf50633 *pcf, void *param, int result)
{
	struct pcf50633_adc_request *req;

	/*We know here that the passed param is an adc_request object */
	req = (struct pcf50633_adc_request *)param;

	req->result = result;
	complete(&req->completion);
}

int pcf50633_adc_sync_read(struct pcf50633 *pcf, int mux, int avg)
{

	struct pcf50633_adc_request *req;
	int result;

	/* req is freed when the result is ready, in pcf50633_work*/
	req = kzalloc(sizeof(*req), GFP_KERNEL);
	if (!req)
		return -ENOMEM;

	req->mux = mux;
	req->avg = avg;
	req->callback =  pcf50633_adc_sync_read_callback;
	req->callback_param = req;
	init_completion(&req->completion);

	adc_enqueue_request(pcf, req);

	wait_for_completion(&req->completion);
	result = req->result;

	return result;
}
EXPORT_SYMBOL(pcf50633_adc_sync_read);

int pcf50633_adc_async_read(struct pcf50633 *pcf, int mux, int avg,
			     void (*callback)(struct pcf50633 *, void *, int),
			     void *callback_param)
{
	struct pcf50633_adc_request *req;

	/* req is freed when the result is ready, in pcf50633_work*/
	req = kmalloc(sizeof(*req), GFP_KERNEL);
	if (!req)
		return -ENOMEM;

	req->mux = mux;
	req->avg = avg;
	req->callback = callback;
	req->callback_param = callback_param;

	adc_enqueue_request(pcf, req);

	return 0;
}
EXPORT_SYMBOL(pcf50633_adc_async_read);

static int adc_result(struct pcf50633 *pcf)
{
	u16 ret = (pcf50633_reg_read(pcf, PCF50633_REG_ADCS1) << 2) |
			(pcf50633_reg_read(pcf, PCF50633_REG_ADCS3) &
						  PCF50633_ADCS3_ADCDAT1L_MASK);
	dev_info(pcf->dev, "adc result = %d\n", ret);

	return ret;
}

static void pcf50633_adc_irq(struct pcf50633 *pcf, int irq, void *unused)
{
	struct pcf50633_adc_request *req;
	int head;

	mutex_lock(&pcf->adc.queue_mutex);
	head = pcf->adc.queue_head;

	req = pcf->adc.queue[head];
	if (!req) {
		dev_err(pcf->dev, "ADC queue empty\n");
		mutex_unlock(&pcf->adc.queue_mutex);
		return;
	}
	pcf->adc.queue[head] = NULL;
	pcf->adc.queue_head = (head + 1) &
				      (PCF50633_MAX_ADC_FIFO_DEPTH - 1);

	mutex_unlock(&pcf->adc.queue_mutex);
	req->callback(pcf, req->callback_param, adc_result(pcf));

	kfree(req);

	trigger_next_adc_job_if_any(pcf);
}

int __init pcf50633_adc_probe(struct platform_device *pdev)
{
	struct pcf50633 *pcf;

	pcf = platform_get_drvdata(pdev);

	/* Set up IRQ handlers */
	pcf->irq_handler[PCF50633_IRQ_ADCRDY].handler = pcf50633_adc_irq;

	mutex_init(&pcf->adc.queue_mutex);
	return 0;
}

static int __devexit pcf50633_adc_remove(struct platform_device *pdev)
{
	struct pcf50633 *pcf;

	pcf = platform_get_drvdata(pdev);
	pcf->irq_handler[PCF50633_IRQ_ADCRDY].handler = NULL;

	return 0;
}

struct platform_driver pcf50633_adc_driver = {
	.driver = {
		.name = "pcf50633-adc",
	},
	.probe = pcf50633_adc_probe,
	.remove = __devexit_p(pcf50633_adc_remove),
};

static int __init pcf50633_adc_init(void)
{
		return platform_driver_register(&pcf50633_adc_driver);
}
module_init(pcf50633_adc_init);

static void __exit pcf50633_adc_exit(void)
{
		platform_driver_unregister(&pcf50633_adc_driver);
}
module_exit(pcf50633_adc_exit);

MODULE_AUTHOR("Balaji Rao <balajirrao@openmoko.org>");
MODULE_DESCRIPTION("PCF50633 adc driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:pcf50633-adc");

