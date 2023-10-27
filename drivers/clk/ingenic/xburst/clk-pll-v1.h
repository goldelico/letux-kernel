#ifndef __INGENIC_CLK_PLLV_H__
#define __INGENIC_CLK_PLLV_H__


extern const struct clk_ops ingenic_pll_ro_ops;

static const s8 pll_od_encode[7] = {-1, 2, 4, 8, 16, 32, 64};

#define PLL_RATE(_rate, _fd, _rd, _od)	\
{					\
	.rate = _rate,			\
	.fd = _fd,			\
	.rd = _rd,			\
	.od = _od,			\
}

struct ingenic_pll_rate_table {
	unsigned int rate;
	unsigned int fd;
	unsigned int rd;
	unsigned int od;
};



#define PLL_DESC(_regoff, _fd, _fd_w, _rd, _rd_w, _od, _od_w, _on, _en, _od_code)	\
{				\
	.regoff = _regoff,	\
	.fd_sft = _fd,		\
	.fd_width = _fd_w,	\
	.rd_sft = _rd,		\
	.rd_width = _rd_w,	\
	.od_sft = _od,		\
	.od_width = _od_w,	\
	.od_encode = _od_code,	\
	.on_bit = _on,		\
	.en_bit =  _en,		\
}

struct ingenic_pll_hwdesc {
	u32 regoff;
	u8 fd_sft;
	u8 fd_width;
	u8 rd_sft;
	u8 rd_width;
	u8 od_sft;
	u8 od_width;
	u8 on_bit;
	u8 en_bit;
	const s8 *od_encode;		/*od rules, -1 not support*/
};

#endif /*__INGENIC_CLK_PLLV1_H__*/
