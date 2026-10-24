// SPDX-License-Identifier: GPL-2.0
/*
 * Ingenic X1600 SPI flash controller (SFC), as a spi-mem controller.
 *
 * Programmed I/O: each spi-mem operation is one SFC transfer, its data
 * moved through the FIFO by the CPU. The registers and the transfer
 * sequence follow Rockbox's X1600 port, which drives the HiBy R1's
 * SPI-NAND with them, and Zhou Yanjie's 2022 Ingenic SFC driver.
 */

#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/spi/spi.h>
#include <linux/spi/spi-mem.h>

#define SFC_GLB			0x00
#define SFC_DEV_CONF		0x04
#define SFC_TRAN_CONF0		0x14
#define SFC_TRAN_LEN		0x2c
#define SFC_DEV_ADDR0		0x30
#define SFC_DEV_ADDR_PLUS0	0x48
#define SFC_MEM_ADDR		0x60
#define SFC_TRIG		0x64
#define SFC_SR			0x68
#define SFC_SCR			0x6c
#define SFC_INTC		0x70
#define SFC_CGE			0x78
#define SFC_GLB1		0x94
#define SFC_TRAN_CFG1_0		0x9c
#define SFC_DATA		0x1000

#define GLB_DES_EN		BIT(15)
#define GLB_CDT_EN		BIT(14)
#define GLB_TRAN_DIR_WRITE	BIT(13)
#define GLB_THRESHOLD		GENMASK(12, 7)
#define GLB_OP_MODE_DMA		BIT(6)
#define GLB_PHASE_NUM		GENMASK(5, 3)
#define GLB_WP_EN		BIT(2)
#define GLB_BURST_MD		GENMASK(1, 0)
#define GLB_BURST_MD_INCR32	3

#define DEV_CONF_SMP_DELAY	GENMASK(17, 16)
#define DEV_CONF_TSH		GENMASK(8, 5)
#define DEV_CONF_CE_DL		BIT(2)
#define DEV_CONF_HOLD_DL	BIT(1)
#define DEV_CONF_WP_DL		BIT(0)

#define TRAN_CONF_ADDR_WIDTH	GENMASK(28, 26)
#define TRAN_CONF_CMD_EN	BIT(24)
#define TRAN_CONF_DUMMY_BITS	GENMASK(22, 17)
#define TRAN_CONF_DATA_EN	BIT(16)
#define TRAN_CONF_CMD		GENMASK(15, 0)

/* The X1600 keeps the transfer mode here, not in TRAN_CONF as the X1000. */
#define TRAN_CFG1_TRAN_MD	GENMASK(7, 4)

#define TRIG_FLUSH		BIT(2)
#define TRIG_STOP		BIT(1)
#define TRIG_START		BIT(0)

#define SR_END			BIT(4)
#define SR_TREQ			BIT(3)
#define SR_RREQ			BIT(2)
#define SR_OVER			BIT(1)
#define SR_UNDER		BIT(0)
#define SR_ALL			GENMASK(4, 0)

#define GLB1_CHIP1_SEL		BIT(31)
#define GLB1_CHIP_SEL		GENMASK(1, 0)

/* A transfer mode: the widths of the command, address and data phases. */
#define TRAN_MD_1_1_1		0
#define TRAN_MD_1_1_2		1
#define TRAN_MD_1_2_2		2
#define TRAN_MD_1_1_4		5
#define TRAN_MD_1_4_4		6

/*
 * FIFO words moved per request: a quarter of the 128-word FIFO, as the
 * vendor SPL sets it. The FIFO is read and written in bursts of this many
 * words (fewer at the end of a transfer).
 */
#define SFC_FIFO_THRESHOLD	32

#define SFC_TIMEOUT_US		100000

/* The SPI clock is a quarter of the SFC clock (the vendor's sfc_clk / 4). */
#define SFC_CLK_PER_SPI_CLK	4

/*
 * Above this SFC clock, the timing Rockbox drives the HiBy R1's W25N01GV
 * with at 150 MHz: a shorter chip-select high time and a sample delay.
 */
#define SFC_FAST_HZ		100000000

struct ingenic_sfc {
	void __iomem *base;
	struct device *dev;
	struct clk *clk;
	u32 speed_hz;
};

static int ingenic_sfc_tran_mode(const struct spi_mem_op *op)
{
	unsigned int addr = op->addr.nbytes ? op->addr.buswidth : 1;
	unsigned int data = op->data.nbytes ? op->data.buswidth : 1;

	if (op->cmd.buswidth != 1)
		return -EINVAL;
	if (addr == 1 && data == 1)
		return TRAN_MD_1_1_1;
	if (addr == 1 && data == 2)
		return TRAN_MD_1_1_2;
	if (addr == 2 && data == 2)
		return TRAN_MD_1_2_2;
	if (addr == 1 && data == 4)
		return TRAN_MD_1_1_4;
	if (addr == 4 && data == 4)
		return TRAN_MD_1_4_4;
	return -EINVAL;
}

static bool ingenic_sfc_supports_op(struct spi_mem *mem, const struct spi_mem_op *op)
{
	if (!spi_mem_default_supports_op(mem, op))
		return false;
	if (op->cmd.nbytes != 1 || op->addr.nbytes > 4)
		return false;
	if (op->dummy.nbytes &&
	    op->dummy.nbytes * 8 / op->dummy.buswidth > FIELD_MAX(TRAN_CONF_DUMMY_BITS))
		return false;
	return ingenic_sfc_tran_mode(op) >= 0;
}

/* Waits for the bits of @mask in SR; fails on a FIFO overrun or underrun. */
static int ingenic_sfc_wait(struct ingenic_sfc *sfc, u32 mask)
{
	u32 sr;
	int ret;

	ret = readl_poll_timeout(sfc->base + SFC_SR, sr,
				 sr & (mask | SR_OVER | SR_UNDER), 0, SFC_TIMEOUT_US);
	if (ret)
		return ret;
	return sr & (SR_OVER | SR_UNDER) ? -EIO : 0;
}

static int ingenic_sfc_pio(struct ingenic_sfc *sfc, const struct spi_mem_op *op)
{
	bool write = op->data.dir == SPI_MEM_DATA_OUT;
	u32 req = write ? SR_TREQ : SR_RREQ;
	unsigned int pos = 0, len = op->data.nbytes;
	int ret;

	while (pos < len) {
		unsigned int burst = min(len - pos, SFC_FIFO_THRESHOLD * 4);
		unsigned int end = pos + burst;

		ret = ingenic_sfc_wait(sfc, req);
		if (ret)
			return ret;
		writel(req, sfc->base + SFC_SCR);

		for (; pos < end; pos += 4) {
			unsigned int n = min(end - pos, 4U);

			if (write) {
				u32 word = 0;

				memcpy(&word, op->data.buf.out + pos, n);
				writel(word, sfc->base + SFC_DATA);
			} else {
				u32 word = readl(sfc->base + SFC_DATA);

				memcpy(op->data.buf.in + pos, &word, n);
			}
		}
		pos = end;
	}
	return 0;
}

static int ingenic_sfc_set_speed(struct ingenic_sfc *sfc, u32 speed_hz)
{
	unsigned long rate;
	u32 conf;
	int ret;

	if (!speed_hz || speed_hz == sfc->speed_hz)
		return 0;

	ret = clk_set_rate(sfc->clk, (unsigned long)speed_hz * SFC_CLK_PER_SPI_CLK);
	if (ret)
		return ret;
	rate = clk_get_rate(sfc->clk);

	conf = DEV_CONF_CE_DL | DEV_CONF_HOLD_DL | DEV_CONF_WP_DL;
	if (rate > SFC_FAST_HZ)
		conf |= FIELD_PREP(DEV_CONF_TSH, 11) | FIELD_PREP(DEV_CONF_SMP_DELAY, 1);
	else
		conf |= FIELD_PREP(DEV_CONF_TSH, 15);
	writel(conf, sfc->base + SFC_DEV_CONF);

	sfc->speed_hz = speed_hz;
	dev_dbg(sfc->dev, "SFC clock %lu Hz\n", rate);
	return 0;
}

static int ingenic_sfc_exec_op(struct spi_mem *mem, const struct spi_mem_op *op)
{
	struct ingenic_sfc *sfc = spi_controller_get_devdata(mem->spi->controller);
	unsigned int dummy_bits = 0;
	u32 val;
	int ret, mode;

	mode = ingenic_sfc_tran_mode(op);
	if (mode < 0)
		return mode;
	ret = ingenic_sfc_set_speed(sfc, op->max_freq ?: mem->spi->max_speed_hz);
	if (ret)
		return ret;
	if (op->dummy.nbytes)
		dummy_bits = op->dummy.nbytes * 8 / op->dummy.buswidth;

	/* Whatever ran before, stopped or not. */
	writel(TRIG_STOP, sfc->base + SFC_TRIG);

	val = readl(sfc->base + SFC_GLB) & ~GLB_TRAN_DIR_WRITE;
	if (op->data.nbytes && op->data.dir == SPI_MEM_DATA_OUT)
		val |= GLB_TRAN_DIR_WRITE;
	writel(val, sfc->base + SFC_GLB);

	val = readl(sfc->base + SFC_TRAN_CFG1_0) & ~TRAN_CFG1_TRAN_MD;
	writel(val | FIELD_PREP(TRAN_CFG1_TRAN_MD, mode), sfc->base + SFC_TRAN_CFG1_0);

	val = TRAN_CONF_CMD_EN | FIELD_PREP(TRAN_CONF_CMD, op->cmd.opcode) |
	      FIELD_PREP(TRAN_CONF_ADDR_WIDTH, op->addr.nbytes) |
	      FIELD_PREP(TRAN_CONF_DUMMY_BITS, dummy_bits);
	if (op->data.nbytes)
		val |= TRAN_CONF_DATA_EN;
	writel(val, sfc->base + SFC_TRAN_CONF0);
	writel(op->data.nbytes, sfc->base + SFC_TRAN_LEN);
	writel(op->addr.val, sfc->base + SFC_DEV_ADDR0);
	writel(0, sfc->base + SFC_DEV_ADDR_PLUS0);

	writel(SR_ALL, sfc->base + SFC_SCR);
	/* Every data phase must start with an empty FIFO. */
	writel(TRIG_FLUSH, sfc->base + SFC_TRIG);
	writel(TRIG_START, sfc->base + SFC_TRIG);

	ret = op->data.nbytes ? ingenic_sfc_pio(sfc, op) : 0;
	if (!ret)
		ret = ingenic_sfc_wait(sfc, SR_END);
	if (ret) {
		dev_err(sfc->dev, "command %02x failed: %d (sr %08x)\n",
			op->cmd.opcode, ret, readl(sfc->base + SFC_SR));
		writel(TRIG_STOP, sfc->base + SFC_TRIG);
		writel(TRIG_FLUSH, sfc->base + SFC_TRIG);
		return ret;
	}
	writel(SR_END, sfc->base + SFC_SCR);
	return 0;
}

static const struct spi_controller_mem_ops ingenic_sfc_mem_ops = {
	.supports_op = ingenic_sfc_supports_op,
	.exec_op = ingenic_sfc_exec_op,
};

static void ingenic_sfc_hw_init(struct ingenic_sfc *sfc)
{
	u32 val;

	/*
	 * Programmed I/O, with the command-table and descriptor modes that a
	 * boot loader may have left (the vendor kernel uses them) turned off.
	 */
	writel(FIELD_PREP(GLB_THRESHOLD, SFC_FIFO_THRESHOLD) |
	       FIELD_PREP(GLB_PHASE_NUM, 1) | GLB_WP_EN |
	       FIELD_PREP(GLB_BURST_MD, GLB_BURST_MD_INCR32),
	       sfc->base + SFC_GLB);
	val = readl(sfc->base + SFC_GLB1) & ~(GLB1_CHIP1_SEL | GLB1_CHIP_SEL);
	writel(val, sfc->base + SFC_GLB1);
	/* The slowest timing and no sample delay, as Rockbox starts the chip. */
	writel(DEV_CONF_CE_DL | DEV_CONF_HOLD_DL | DEV_CONF_WP_DL |
	       FIELD_PREP(DEV_CONF_TSH, 15), sfc->base + SFC_DEV_CONF);
	writel(0, sfc->base + SFC_CGE);
	writel(SR_ALL, sfc->base + SFC_INTC);
	writel(0, sfc->base + SFC_MEM_ADDR);
	writel(TRIG_STOP, sfc->base + SFC_TRIG);
	writel(TRIG_FLUSH, sfc->base + SFC_TRIG);
}

static int ingenic_sfc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct spi_controller *ctlr;
	struct ingenic_sfc *sfc;

	ctlr = devm_spi_alloc_host(dev, sizeof(*sfc));
	if (!ctlr)
		return -ENOMEM;
	sfc = spi_controller_get_devdata(ctlr);
	sfc->dev = dev;

	sfc->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(sfc->base))
		return PTR_ERR(sfc->base);

	/* Before any register access: a read with the clock gated hangs. */
	sfc->clk = devm_clk_get_enabled(dev, NULL);
	if (IS_ERR(sfc->clk))
		return dev_err_probe(dev, PTR_ERR(sfc->clk), "Unable to get clock\n");

	ingenic_sfc_hw_init(sfc);

	ctlr->mode_bits = SPI_RX_DUAL | SPI_RX_QUAD | SPI_TX_DUAL | SPI_TX_QUAD;
	ctlr->mem_ops = &ingenic_sfc_mem_ops;
	ctlr->num_chipselect = 1;
	ctlr->bus_num = -1;
	ctlr->dev.of_node = dev->of_node;

	return devm_spi_register_controller(dev, ctlr);
}

static const struct of_device_id ingenic_sfc_of_match[] = {
	{ .compatible = "ingenic,x1600-sfc" },
	{ }
};
MODULE_DEVICE_TABLE(of, ingenic_sfc_of_match);

static struct platform_driver ingenic_sfc_driver = {
	.probe = ingenic_sfc_probe,
	.driver = {
		.name = "ingenic-sfc",
		.of_match_table = ingenic_sfc_of_match,
	},
};
module_platform_driver(ingenic_sfc_driver);

MODULE_DESCRIPTION("Ingenic X1600 SPI flash controller driver");
MODULE_LICENSE("GPL");
