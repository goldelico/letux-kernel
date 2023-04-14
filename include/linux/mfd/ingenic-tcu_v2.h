#ifndef __INGENIC_TCU_H__
#define __INGENIC_TCU_H__

#define CHN_TDFR	(0x0)    /*channel N data full register*/
#define CHN_TDHR	(0x4)    /*channel N data half register*/
#define CHN_TCNT	(0x8)    /*channel N counter register*/
#define CHN_TCSR	(0xc)    /*channel N control register*/

#define TCU_TSR		(0x1C)   /* Timer Stop Register */
#define TCU_TSSR	(0x2C)   /* Timer Stop Set Register */
#define TCU_TSCR	(0x3C)   /* Timer Stop Clear Register */
#define TCU_TER		(0x10)   /* Timer Counter Enable Register */
#define TCU_TESR	(0x14)   /* Timer Counter Enable Set Register */
#define TCU_TECR	(0x18)   /* Timer Counter Enable Clear Register */
#define TCU_TFR		(0x20)   /* Timer Flag Register */
#define TCU_TFSR	(0x24)   /* Timer Flag Set Register */
#define TCU_TFCR	(0x28)   /* Timer Flag Clear Register */
#define TCU_TMR		(0x30)   /* Timer Mask Register */
#define TCU_TMSR	(0x34)   /* Timer Mask Set Register */
#define TCU_TMCR	(0x38)   /* Timer Mask Clear Register */
#define TCU_STORE_FLAG          (0x200)
#define TCU_STORE_FLAG_SET      (0x204)
#define TCU_STORE_FLAG_CLR      (0x208)
#define TCU_STORE_MASK_RD       (0x210)
#define TCU_STORE_MASK_SET      (0x214)
#define TCU_STORE_MASK_CLR      (0x218)

#define CH_TDFR(n)	(0x40 + (n)*0x10) /* Timer Data Full Reg */
#define CH_TDHR(n)	(0x44 + (n)*0x10) /* Timer Data Half Reg */
#define CH_TCNT(n)	(0x48 + (n)*0x10) /* Timer Counter Reg */
#define CH_TCSR(n)	(0x4C + (n)*0x10) /* Timer Control Reg */

#define CHN_CAP(n)	(0xc0 + (n)*0x04) /*Capture register*/
#define CHN_CAP_VAL(n)	(0xe0 + (n)*0x04) /*Capture Value register*/
#define CHN_FIL_VAL(n)	(0x1a0 + (n)*0x04) /*filter value*/

#define TCU_STORE_VAL(n)	(0x220 + (n)*0x04)
#define TCU_STORE_FIL_VAL(n)	(0x240 + (n)*0x04)

//OST control
#define TER_OSTEN	(1 << 15)   /* enable the counter in ost */
#define TMR_OSTM	(1 << 15)   /* ost comparison match interrupt mask */
#define TFR_OSTF	(1 << 15)   /* ost interrupt flag */
#define	TSR_OSTS	(1 << 15)   /*the clock supplies to osts is stopped */

//watchdog bit
#define TSR_WDTS	(1 << 16)   /*the clock supplies to wdt is stopped */
#define TCU_FLAG_RD	(1 << 24)   /*HALF comparison match flag of WDT. (TCNT = TDHR) */

//timer control Register bit operation
#define CSR_EXT_EN	(1 << 2)  /* select extal as the timer clock input */
#define CSR_DIV1	(0x0 << 3) /*select the TCNT count clock frequency*/
#define CSR_DIV4	(0x1 << 3)
#define CSR_DIV16	(0x2 << 3)
#define CSR_DIV64	(0x3 << 3)
#define CSR_DIV256	(0x4 << 3)
#define CSR_DIV1024	(0x5 << 3)
#define CSR_DIV_MSK	(0x7 << 3)

#define CSR_DIR_HIGH	(0x0 << 8)  /*0:direction signal hold on 1*/
#define CSR_DIR_CLK	(0x1 << 8)  /*1:use clock with direction signal*/
#define CSR_DIR_GPIO0	(0x2 << 8)  /*2:use gpio0 with direction signal*/
#define CSR_DIR_GPIO1	(0x3 << 8)  /*3:use gpio1 with direction signal*/
#define CSR_DIR_QUADRATURE	(0x4 << 8)  /*4:use gpio0 and gpio1 quadrature result with direction signal*/
#define CSR_DIR_MSK	(0x7 << 8)

#define CSR_GATE_LOW	(0x0 << 11)  /*0:gate signal hold on 0*/
#define CSR_GATE_CLK	(0x1 << 11)  /*1:use clock with gate signal*/
#define CSR_GATE_GPIO0	(0x2 << 11)  /*2:use gpio0 with gate signal*/
#define CSR_GATE_GPIO1	(0x3 << 11)  /*3:use gpio1 with gate signal*/
#define CSR_GATE_MSK	(0x3 << 11)

#define CLK_POS		(16)
#define CLK_NEG		(17)
#define GPIO0_POS	(18)
#define GPIO0_NEG	(19)
#define GPIO1_POS	(20)
#define GPIO1_NEG	(21)
#define STORE_EN	(24)
#define STORE_POS_EN	(25)
#define STORE_NEG_EN	(26)


#define CSR_CM_FCZ	(0x0 << 22)  /*count range 0 <-> full with clear to 0*/
#define CSR_CM_MCZ	(0x1 << 22)  /*count range 0 <-> 0xffff with clear to 0*/
#define CSR_CM_MH	(0x2 << 22)  /*count range 0 <-> 0xffff with hold on*/
#define CSR_CM_MSK	(0x3 << 22)

#define TCU_CONTROL_BIT(n)	(1 << n)

//Capture register bit operation
#define CAP_SEL_CLK	(0x0 << 16)  /*TCU will capture clock*/
#define CAP_SEL_GPIO0	(0x1 << 16)  /*TCU will capture gpio0*/
#define CAP_SEL_GPIO1	(0x2 << 16)  /*TCU will capture gpio1*/
/*if capture mode is disable then these bits will:*/
#define CAP_SEL_CCG0P	(0x1 << 16)  /*counter will clear when gpio0 posedge*/
#define CAP_SEL_CCG1P	(0x2 << 16)  /*counter will clear when gpio1 posedge*/
#define CAP_SEL_MSK	(0x7 << 16)
#define CAP_NUM_MSK	(0xff)

//filter value register bit operation
#define  FIL_VAL_GPIO1_MSK	(0x3ff << 16)  /*Gpio1 filter counter value*/
#define  FIL_VAL_GPIO0_MSK	(0x3ff)	       /*Gpio0 filter counter value*/

//Common 1 bit offset
#define ONE_BIT_OFFSET(n)	TCU_CONTROL_BIT(n)


#define timeout	 (500)
#define CAPTURE_LOOP_FLAGS (0x1 << 7)

#define TCU_CHN_TDFR	(0x0)
#define TCU_CHN_TDHR	(0x4)
#define TCU_CHN_TCNT	(0x8)
#define TCU_CHN_TCSR	(0xc)
#define TCU_FULL0	(0x40)
#define TCU_CHN_OFFSET	(0x10)


#define tcu_readl(tcu,reg)			\
	__raw_readl((tcu)->iomem + reg)
#define tcu_writel(tcu,reg,value)			\
	__raw_writel((value), (tcu)->iomem + reg)

#define tcu_chn_readl(tcu_chn,reg)					\
	__raw_readl((tcu_chn)->tcu->iomem + (tcu_chn)->reg_base + reg)
#define tcu_chn_writel(tcu_chn,reg,value)				\
	__raw_writel((value), (tcu_chn)->tcu->iomem + (tcu_chn)->reg_base + reg)

enum tcu_prescale {
	TCU_PRESCALE_1,
	TCU_PRESCALE_4,
	TCU_PRESCALE_16,
	TCU_PRESCALE_64,
	TCU_PRESCALE_256,
	TCU_PRESCALE_1024
};

enum tcu_irq_type {
	NULL_IRQ_MODE,
	FULL_IRQ_MODE,
	HALF_IRQ_MODE,
	FULL_HALF_IRQ_MODE,
};

enum tcu_clksrc {
	TCU_CLKSRC_NULL,
	TCU_CLKSRC_EXT = ONE_BIT_OFFSET(2),
	TCU_CLKSRC_GPIO0 = ONE_BIT_OFFSET(6),
	TCU_CLKSRC_GPIO1 = ONE_BIT_OFFSET(7)
};

enum tcu_count_mode{
	COUNT_MODE_FCZ,
	COUNT_MODE_MCZ,
	COUNT_MODE_MH
};

enum tcu_gate_sel{
	GATE_SEL_HZ,
	GATE_SEL_CLK,
	GATE_SEL_GPIO0,
	GATE_SEL_GPIO1
};

enum tcu_gate_pola{
	GATE_POLA_LOW,
	GATE_POLA_HIGH
};

enum tcu_dir_sel{
	DIR_SEL_HH,
	DIR_SEL_CLK,
	DIR_SEL_GPIO0,
	DIR_SEL_GPIO1,
	DIR_SEL_GPIO_QUA
};

enum tcu_dir_pola{
	DIR_POLA_LOW,
	DIR_POLA_HIGH
};

enum tcu_mode_sel{
	GENERAL_MODE,
	GATE_MODE,
	DIRECTION_MODE,
	QUADRATURE_MODE,
	POS_MODE,
	CAPTURE_MODE,
	FILTER_MODE
};

enum tcu_pos_sel{
	GPIO0_POS_CLR = 1,
	GPIO1_POS_CLR,
	GPIO0_NEG_CLR
};

enum tcu_capture_sel{
	CAPTURE_CLK,
	CAPTURE_GPIO0,
	CAPTURE_GPIO1
};

enum signal_pos_neg{
	SIG_INIT,
	SIG_NEG_EN,
	SIG_POS_EN,
	SIG_POS_NEG_EN
};

struct ingenic_tcu {
	void __iomem *iomem;
	struct resource *res;
	struct clk *clk;
	int irq;
	int irq_trigger;
	int irq_base;
	spinlock_t lock;
};


struct ingenic_tcu_chn {
	int index;			/* Channel number */
	int id;				/* Channel number */
	int en_flag;
	u32 reg_base;
	void __iomem *iomem;
	int using;

	enum tcu_irq_type irq_type;
	enum tcu_clksrc clk_ext,clk_gpio0,clk_gpio1;
	enum tcu_prescale prescale;
	enum tcu_count_mode count_mode;
	enum tcu_gate_sel gate_sel;
	enum tcu_gate_pola gate_pola;
	enum tcu_dir_sel dir_sel;
	enum tcu_dir_pola dir_pola;
	enum tcu_mode_sel mode_sel;
	enum signal_pos_neg sig_ext,sig_gpio0,sig_gpio1;
	enum tcu_pos_sel pos_sel;
	enum tcu_capture_sel capture_sel;

	int capture_num;
	int half_num;
	int full_num;
	int fil_a_num;
	int fil_b_num;
	int count_value;
	unsigned int divi_ratio;  /*  0/1/2/3/4/5/something else------>1/4/16/64/256/1024/mask  */
	int shutdown_mode;
	struct ingenic_tcu *tcu;
	bool gpio_trigger;
};
static inline void ingenic_tcu_enable_counter(struct ingenic_tcu_chn *tcu_chn)
{
	tcu_writel(tcu_chn->tcu, TCU_TESR, 1 << tcu_chn->index);
}

static inline int ingenic_tcu_disable_counter(struct ingenic_tcu_chn *tcu_chn)
{
	tcu_writel(tcu_chn->tcu, TCU_TECR, 1 << tcu_chn->index);
	return 1;
}

static inline void ingenic_tcu_start_counter(struct ingenic_tcu_chn *tcu_chn)
{
	tcu_writel(tcu_chn->tcu, TCU_TSCR, 1 << tcu_chn->index);
}

static inline void ingenic_tcu_stop_counter(struct ingenic_tcu_chn *tcu_chn)
{
	tcu_writel(tcu_chn->tcu, TCU_TSSR, 1 << tcu_chn->index);
}

static inline void ingenic_tcu_set_count(struct ingenic_tcu_chn *tcu_chn, u16 cnt)
{
	tcu_chn_writel(tcu_chn, CHN_TCNT, cnt);
}

static inline unsigned long ingenic_tcu_flag_st(struct ingenic_tcu_chn *tcu_chn)
{
	return tcu_readl(tcu_chn->tcu, TCU_TFR);
}

static inline int ingenic_tcu_get_count(struct ingenic_tcu_chn *tcu_chn)
{
	return tcu_chn_readl(tcu_chn, CHN_TCNT);
}

static inline int ingenic_tcu_get_cap_val(struct ingenic_tcu_chn *tcu_chn)
{
	return tcu_readl(tcu_chn->tcu,CHN_CAP_VAL(tcu_chn->index));
}

static inline unsigned int ingenic_tcu_store_flag_st(struct ingenic_tcu *tcu)
{
	return tcu_readl(tcu, TCU_STORE_FLAG);
}

static inline void ingenic_tcu_store_flag_clr(struct ingenic_tcu_chn *tcu_chn)
{
	tcu_writel(tcu_chn->tcu, TCU_STORE_FLAG_CLR, 1 << tcu_chn->index);
}

static inline void ingenic_tcu_store_flag_set(struct ingenic_tcu_chn *tcu_chn)
{
	tcu_writel(tcu_chn->tcu, TCU_STORE_FLAG_SET, 1 << tcu_chn->index);
}

static inline unsigned int ingenic_tcu_store_mask_val(struct ingenic_tcu *tcu)
{
	return tcu_readl(tcu, TCU_STORE_MASK_RD);
}

static inline void ingenic_tcu_store_mask_clr(struct ingenic_tcu_chn *tcu_chn)
{
	tcu_writel(tcu_chn->tcu, TCU_STORE_MASK_CLR, 1 << tcu_chn->index);
}

static inline void ingenic_tcu_store_mask_set(struct ingenic_tcu_chn *tcu_chn)
{
	tcu_writel(tcu_chn->tcu, TCU_STORE_MASK_SET, 1 << tcu_chn->index);
}

static inline unsigned int ingenic_tcu_store_val(struct ingenic_tcu_chn *tcu_chn)
{
	return tcu_readl(tcu_chn->tcu, TCU_STORE_VAL(tcu_chn->index));
}

static inline void ingenic_tcu_store_set_filter(struct ingenic_tcu_chn *tcu_chn,
		unsigned int val)
{
	tcu_writel(tcu_chn->tcu, TCU_STORE_FIL_VAL(tcu_chn->index), val & 0x3ff);
}

static inline void ingenic_tcu_store_enable(struct ingenic_tcu_chn *tcu_chn)
{
	unsigned int tmp;
	tmp = tcu_chn_readl(tcu_chn, CHN_TCSR);
	tcu_chn_writel(tcu_chn, CHN_TCSR, (1 << STORE_EN) | tmp);
}

static inline void ingenic_tcu_store_disable(struct ingenic_tcu_chn *tcu_chn)
{
	unsigned int tmp;
	tmp = tcu_chn_readl(tcu_chn, CHN_TCSR);
	tcu_chn_writel(tcu_chn, CHN_TCSR, ~(1 << STORE_EN) & tmp);
}

static inline void ingenic_tcu_store_neg_enable(struct ingenic_tcu_chn *tcu_chn)
{
	unsigned int tmp;
	tmp = tcu_readl(tcu_chn->tcu, CH_TCSR(tcu_chn->index));
	tcu_writel(tcu_chn->tcu, CH_TCSR(tcu_chn->index), (1 << STORE_NEG_EN) | tmp);
}

static inline void ingenic_tcu_store_neg_disable(struct ingenic_tcu_chn *tcu_chn)
{
	unsigned int tmp;
	tmp = tcu_readl(tcu_chn->tcu, CH_TCSR(tcu_chn->index));
	tcu_writel(tcu_chn->tcu, CH_TCSR(tcu_chn->index), ~(1 << STORE_NEG_EN) | tmp);
}

static inline void ingenic_tcu_store_pos_enable(struct ingenic_tcu_chn *tcu_chn)
{
	unsigned int tmp;
	tmp = tcu_readl(tcu_chn->tcu, CH_TCSR(tcu_chn->index));
	tcu_writel(tcu_chn->tcu, CH_TCSR(tcu_chn->index), (1 << STORE_POS_EN) | tmp);
}

static inline void ingenic_tcu_store_pos_disable(struct ingenic_tcu_chn *tcu_chn)
{
	unsigned int tmp;
	tmp = tcu_readl(tcu_chn->tcu, CH_TCSR(tcu_chn->index));
	tcu_writel(tcu_chn->tcu, CH_TCSR(tcu_chn->index), ~(1 << STORE_POS_EN) | tmp);
}

void sws_pr_debug(struct ingenic_tcu_chn *tcu_chn);
void ingenic_tcu_config_chn(struct ingenic_tcu_chn *tcu_chn);
void ingenic_tcu_clear_irq_flag(struct ingenic_tcu_chn *tcu_chn);
#endif /* __ingenic_TCU_H__ */
