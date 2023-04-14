/*
 * A V4L2 driver for OmniVision SC230AI cameras.
 *
 * Copyright 2006 One Laptop Per Child Association, Inc.  Written
 * by Jonathan Corbet with substantial inspiration from Mark
 * McClelland's ovcamchip code.
 *
 * Copyright 2006-7 Jonathan Corbet <corbet@lwn.net>
 *
 * This file may be distributed under the terms of the GNU General
 * Public License, version 2.
 */
#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/videodev2.h>
#include <linux/regulator/consumer.h>
#include <media/v4l2-device.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-clk.h>
#include <media/v4l2-mediabus.h>
#include <media/v4l2-image-sizes.h>
#include <linux/of_gpio.h>
#include <isp-sensor.h>


#define SC230AI_CHIP_ID_H	(0xcb)
#define SC230AI_CHIP_ID_L	(0x34)
#define SC230AI_REG_END		0xff
#define SC230AI_REG_DELAY	0x00
#define SC230AI_PAGE_REG	    0xfd

static bool debug;
module_param(debug, bool, 0644);
MODULE_PARM_DESC(debug, "Debug level (0-1)");

struct sc230ai_win_size {
	unsigned int width;
	unsigned int height;
	unsigned int mbus_code;
	struct sensor_info sensor_info;
	enum v4l2_colorspace colorspace;
	void *regs;

};

struct sc230ai_gpio {
	int pin;
	int active_level;
};

struct sc230ai_supply{
	struct regulator *avdd;
	struct regulator *dvdd;
	struct regulator *dovdd;
};

struct sc230ai_info {
	struct sc230ai_supply supply;
	struct v4l2_subdev sd;
	struct v4l2_ctrl_handler hdl;
	struct v4l2_ctrl *gain;
	struct v4l2_ctrl *again;

	struct v4l2_clk *clk;
	struct clk *sclka;

	struct v4l2_ctrl *exposure;

	struct media_pad pad;

	struct v4l2_subdev_format *format;	/*current fmt.*/
	struct sc230ai_win_size *win;

	struct sc230ai_gpio xshutdn;
	struct sc230ai_gpio pwdn;
	struct sc230ai_gpio efsync;
	struct sc230ai_gpio led;
};


/*
 * the part of driver maybe modify about different sensor and different board.
 */
struct again_lut {
	unsigned int value;	/*sensor regs value*/
	unsigned int fine_value;	/*sensor regs value*/
	unsigned int gain;	/*isp gain*/
};

struct again_lut sc230ai_again_lut[] = {
	/*
	{0x00,	0},
	{0x01,	65536},
	{0x40,	115455},
	{0x48,	180991},
	{0x49,	246527},
	{0x4b,	312063},
	{0x4f,	377599},
	{0x5f,	443135},
	*/

	{0x00,0x80,0},
	{0x00,0x82,1501},
	{0x00,0x84,2886},
	{0x00,0x86,4343},
	{0x00,0x88,5776},
	{0x00,0x8a,7101},
	{0x00,0x8c,8494},
	{0x00,0x8e,9782},
	{0x00,0x90,11136},
	{0x00,0x92,12471},
	{0x00,0x94,13706},
	{0x00,0x96,15006},
	{0x00,0x98,16288},
	{0x00,0x9a,17474},
	{0x00,0x9c,18724},
	{0x00,0x9e,19880},
	{0x00,0xa0,21098},
	{0x00,0xa2,22300},
	{0x00,0xa4,23414},
	{0x00,0xa6,24588},
	{0x00,0xa8,25747},
	{0x00,0xaa,26821},
	{0x00,0xac,27953},
	{0x00,0xae,29003},
	{0x00,0xb0,30109},
	{0x00,0xb2,31203},
	{0x00,0xb4,32217},
	{0x00,0xb6,33287},
	{0x00,0xb8,34345},
	{0x00,0xba,35326},
	{0x00,0xbc,36362},
	{0x00,0xbe,37322},
	{0x00,0xc0,38336},
	{0x00,0xc2,39339},
	{0x00,0xc4,40270},
	{0x00,0xc6,41253},
	{0x00,0xc8,42226},
	{0x00,0xca,43129},
	{0x00,0xcc,44083},
	{0x00,0xce,44968},
	{0x00,0xd0,45904},
	{0x00,0xd2,46830},
	{0x00,0xd4,47691},
	{0x00,0xd6,48600},
	{0x00,0xd8,49500},
	{0x00,0xda,50337},
	{0x00,0xdc,51221},
	{0x00,0xdf,52042},
	{0x00,0xe0,52911},
	{0x00,0xe2,53771},
	{0x00,0xe4,54571},
	{0x00,0xe6,55417},
	{0x00,0xe8,56255},
	{0x00,0xea,57034},
	{0x00,0xec,57858},
	{0x00,0xee,58624},
	{0x00,0xf0,59434},
	{0x00,0xf2,60237},
	{0x00,0xf4,60984},
	{0x00,0xf6,61775},
	{0x00,0xf8,62559},
	{0x00,0xfa,63288},
	{0x00,0xfc,64059},
	{0x00,0xfe,64777},
	{0x01,0x80,65536},
	{0x01,0x82,66990},
	{0x01,0x84,68468},
	{0x01,0x86,69879},
	{0x01,0x88,71268},
	{0x01,0x8a,72637},
	{0x01,0x8c,74030},
	{0x01,0x8e,75360},
	{0x01,0x90,76672},
	{0x01,0x92,77966},
	{0x01,0x94,79283},
	{0x01,0x96,80542},
	{0x01,0x98,81784},
	{0x01,0x9a,83010},
	{0x01,0x9c,84260},
	{0x01,0x9e,85454},
	{0x01,0xa0,86634},
	{0x01,0xa2,87799},
	{0x01,0xa4,88987},
	{0x01,0xa6,90124},
	{0x01,0xa8,91247},
	{0x01,0xaa,92357},
	{0x01,0xac,93489},
	{0x01,0xae,94573},
	{0x01,0xb0,95645},
	{0x01,0xb2,96705},
	{0x01,0xb4,97787},
	{0x01,0xb6,98823},
	{0x01,0xb8,99848},
	{0x01,0xba,100862},
	{0x01,0xbc,101898},
	{0x01,0xbe,102890},
	{0x01,0xc0,103872},
	{0x01,0xc2,104844},
	{0x01,0xc4,105837},
	{0x01,0xc6,106789},
	{0x01,0xc8,107732},
	{0x01,0xca,108665},
	{0x01,0xcc,109619},
	{0x01,0xce,110534},
	{0x01,0xd0,111440},
	{0x01,0xd2,112338},
	{0x01,0xd4,113255},
	{0x01,0xd6,114136},
	{0x01,0xd8,115008},
	{0x40,0x80,115455},
	{0x40,0x82,116323},
	{0x40,0x84,117182},
	{0x40,0x86,118034},
	{0x40,0x88,118878},
	{0x40,0x8a,119715},
	{0x40,0x8c,120544},
	{0x40,0x8e,121366},
	{0x40,0x90,122181},
	{0x40,0x92,122989},
	{0x40,0x94,123790},
	{0x40,0x96,124585},
	{0x40,0x98,125373},
	{0x40,0x9a,126154},
	{0x40,0x9c,126929},
	{0x40,0x9e,127697},
	{0x40,0xa0,128460},
	{0x40,0xa2,129216},
	{0x40,0xa4,129966},
	{0x40,0xa6,130711},
	{0x40,0xa8,131449},
	{0x40,0xaa,132910},
	{0x40,0xac,134347},
	{0x40,0xae,135764},
	{0x40,0xb0,137159},
	{0x40,0xb2,138534},
	{0x40,0xb4,139890},
	{0x40,0xb6,141226},
	{0x40,0xb8,142544},
	{0x40,0xba,143843},
	{0x40,0xbc,145125},
	{0x40,0xbe,146390},
	{0x40,0xc0,147638},
	{0x40,0xc2,148870},
	{0x40,0xc4,150086},
	{0x40,0xc6,151286},
	{0x40,0xc8,152472},
	{0x40,0xca,153643},
	{0x40,0xcc,154799},
	{0x40,0xce,155942},
	{0x40,0xd0,157071},
	{0x40,0xd2,158186},
	{0x40,0xd4,159289},
	{0x40,0xd6,160379},
	{0x40,0xd8,161456},
	{0x40,0xda,162521},
	{0x40,0xdc,163575},
	{0x40,0xdf,164616},
	{0x40,0xe0,165647},
	{0x40,0xe2,166666},
	{0x40,0xe4,167675},
	{0x40,0xe6,168672},
	{0x40,0xe8,169660},
	{0x40,0xea,170637},
	{0x40,0xec,171604},
	{0x40,0xee,172562},
	{0x40,0xf0,173509},
	{0x40,0xf2,174448},
	{0x40,0xf4,175377},
	{0x40,0xf6,176297},
	{0x40,0xf8,177208},
	{0x40,0xfa,178111},
	{0x40,0xfc,179005},
	{0x40,0xfe,179891},
	{0x48,0x80,180991},
	{0x48,0x82,181859},
	{0x48,0x84,182718},
	{0x48,0x86,183570},
	{0x48,0x88,184414},
	{0x48,0x8a,185251},
	{0x48,0x8c,186080},
	{0x48,0x8e,186902},
	{0x48,0x90,187717},
	{0x48,0x92,188525},
	{0x48,0x94,189326},
	{0x48,0x96,190121},
	{0x48,0x98,190909},
	{0x48,0x9a,191690},
	{0x48,0x9c,192465},
	{0x48,0x9e,193233},
	{0x48,0xa0,193996},
	{0x48,0xa2,194752},
	{0x48,0xa4,195502},
	{0x48,0xa6,196247},
	{0x48,0xa8,197718},
	{0x48,0xaa,199167},
	{0x48,0xac,200594},
	{0x48,0xae,202000},
	{0x48,0xb0,203385},
	{0x48,0xb2,204751},
	{0x48,0xb4,206096},
	{0x48,0xb6,207423},
	{0x48,0xb8,208732},
	{0x48,0xba,210022},
	{0x48,0xbc,211296},
	{0x48,0xbe,212552},
	{0x48,0xc0,213792},
	{0x48,0xc2,215016},
	{0x48,0xc4,216224},
	{0x48,0xc6,217417},
	{0x48,0xc8,218595},
	{0x48,0xca,219759},
	{0x48,0xcc,220908},
	{0x48,0xce,222044},
	{0x48,0xd0,223166},
	{0x48,0xd2,224275},
	{0x48,0xd4,225371},
	{0x48,0xd6,226455},
	{0x48,0xd8,227526},
	{0x48,0xda,228585},
	{0x48,0xdc,229633},
	{0x48,0xdf,230669},
	{0x48,0xe0,231694},
	{0x48,0xe2,232708},
	{0x48,0xe4,233711},
	{0x48,0xe6,234703},
	{0x48,0xe8,235686},
	{0x48,0xea,236658},
	{0x48,0xec,237620},
	{0x48,0xee,238573},
	{0x48,0xf0,239516},
	{0x48,0xf2,240450},
	{0x48,0xf4,241374},
	{0x48,0xf6,242290},
	{0x48,0xf8,243197},
	{0x48,0xfa,244095},
	{0x48,0xfc,244985},
	{0x48,0xfe,245866},
	{0x49,0x80,246527},
	{0x49,0x82,247395},
	{0x49,0x84,248254},
	{0x49,0x86,249106},
	{0x49,0x88,249950},
	{0x49,0x8a,250787},
	{0x49,0x8c,251616},
	{0x49,0x8e,252438},
	{0x49,0x90,253253},
	{0x49,0x92,254061},
	{0x49,0x94,254862},
	{0x49,0x96,255657},
	{0x49,0x98,256445},
	{0x49,0x9a,257226},
	{0x49,0x9c,258001},
	{0x49,0x9e,258769},
	{0x49,0xa0,259532},
	{0x49,0xa2,260288},
	{0x49,0xa4,261038},
	{0x49,0xa6,261783},
	{0x49,0xa8,263254},
	{0x49,0xaa,264703},
	{0x49,0xac,266130},
	{0x49,0xae,267536},
	{0x49,0xb0,268921},
	{0x49,0xb2,270287},
	{0x49,0xb4,271632},
	{0x49,0xb6,272959},
	{0x49,0xb8,274268},
	{0x49,0xba,275558},
	{0x49,0xbc,276832},
	{0x49,0xbe,278088},
	{0x49,0xc0,279328},
	{0x49,0xc2,280552},
	{0x49,0xc4,281760},
	{0x49,0xc6,282953},
	{0x49,0xc8,284131},
	{0x49,0xca,285295},
	{0x49,0xcc,286444},
	{0x49,0xce,287580},
	{0x49,0xd0,288702},
	{0x49,0xd2,289811},
	{0x49,0xd4,290907},
	{0x49,0xd6,291991},
	{0x49,0xd8,293062},
	{0x49,0xda,294121},
	{0x49,0xdc,295169},
	{0x49,0xdf,296205},
	{0x49,0xe0,297230},
	{0x49,0xe2,298244},
	{0x49,0xe4,299247},
	{0x49,0xe6,300239},
	{0x49,0xe8,301222},
	{0x49,0xea,302194},
	{0x49,0xec,303156},
	{0x49,0xee,304109},
	{0x49,0xf0,305052},
	{0x49,0xf2,305986},
	{0x49,0xf4,306910},
	{0x49,0xf6,307826},
	{0x49,0xf8,308733},
	{0x49,0xfa,309631},
	{0x49,0xfc,310521},
	{0x49,0xfe,311402},
	{0x4b,0x80,312063},
	{0x4b,0x82,312931},
	{0x4b,0x84,313790},
	{0x4b,0x86,314642},
	{0x4b,0x88,315486},
	{0x4b,0x8a,316323},
	{0x4b,0x8c,317152},
	{0x4b,0x8e,317974},
	{0x4b,0x90,318789},
	{0x4b,0x92,319597},
	{0x4b,0x94,320398},
	{0x4b,0x96,321193},
	{0x4b,0x98,321981},
	{0x4b,0x9a,322762},
	{0x4b,0x9c,323537},
	{0x4b,0x9e,324305},
	{0x4b,0xa0,325068},
	{0x4b,0xa2,325824},
	{0x4b,0xa4,326574},
	{0x4b,0xa6,327319},
	{0x4b,0xa8,328057},
	{0x4b,0xaa,329518},
	{0x4b,0xac,330955},
	{0x4b,0xae,332372},
	{0x4b,0xb0,333767},
	{0x4b,0xb2,335142},
	{0x4b,0xb4,336498},
	{0x4b,0xb6,337834},
	{0x4b,0xb8,339152},
	{0x4b,0xba,340451},
	{0x4b,0xbc,341733},
	{0x4b,0xbe,342998},
	{0x4b,0xc0,344246},
	{0x4b,0xc2,345478},
	{0x4b,0xc4,346694},
	{0x4b,0xc6,347894},
	{0x4b,0xc8,349080},
	{0x4b,0xca,350251},
	{0x4b,0xcc,351407},
	{0x4b,0xce,352550},
	{0x4b,0xd0,353679},
	{0x4b,0xd2,354794},
	{0x4b,0xd4,355897},
	{0x4b,0xd6,356987},
	{0x4b,0xd8,358064},
	{0x4b,0xda,359129},
	{0x4b,0xdc,360183},
	{0x4b,0xdf,361224},
	{0x4b,0xe0,362255},
	{0x4b,0xe2,363274},
	{0x4b,0xe4,364283},
	{0x4b,0xe6,365280},
	{0x4b,0xe8,366268},
	{0x4b,0xea,367245},
	{0x4b,0xec,368212},
	{0x4b,0xee,369170},
	{0x4b,0xf0,370117},
	{0x4b,0xf2,371056},
	{0x4b,0xf4,371985},
	{0x4b,0xf6,372905},
	{0x4b,0xf8,373816},
	{0x4b,0xfa,374719},
	{0x4b,0xfc,375613},
	{0x4b,0xfe,376499},
	{0x4f,0x80,377599},
	{0x4f,0x82,378467},
	{0x4f,0x84,379326},
	{0x4f,0x86,380178},
	{0x4f,0x88,381022},
	{0x4f,0x8a,381859},
	{0x4f,0x8c,382688},
	{0x4f,0x8e,383510},
	{0x4f,0x90,384325},
	{0x4f,0x92,385133},
	{0x4f,0x94,385934},
	{0x4f,0x96,386729},
	{0x4f,0x98,387517},
	{0x4f,0x9a,388298},
	{0x4f,0x9c,389073},
	{0x4f,0x9e,389841},
	{0x4f,0xa0,390604},
	{0x4f,0xa2,391360},
	{0x4f,0xa4,392110},
	{0x4f,0xa6,392855},
	{0x4f,0xa8,393593},
	{0x4f,0xaa,395054},
	{0x4f,0xac,396491},
	{0x4f,0xae,397908},
	{0x4f,0xb0,399303},
	{0x4f,0xb2,400678},
	{0x4f,0xb4,402034},
	{0x4f,0xb6,403370},
	{0x4f,0xb8,404688},
	{0x4f,0xba,405987},
	{0x4f,0xbc,407269},
	{0x4f,0xbe,408534},
	{0x4f,0xc0,409782},
	{0x4f,0xc2,411014},
	{0x4f,0xc4,412230},
	{0x4f,0xc6,413430},
	{0x4f,0xc8,414616},
	{0x4f,0xca,415787},
	{0x4f,0xcc,416943},
	{0x4f,0xce,418086},
	{0x4f,0xd0,419215},
	{0x4f,0xd2,420330},
	{0x4f,0xd4,421433},
	{0x4f,0xd6,422523},
	{0x4f,0xd8,423600},
	{0x4f,0xda,424665},
	{0x4f,0xdc,425719},
	{0x4f,0xdf,426760},
	{0x4f,0xe0,427791},
	{0x4f,0xe2,428810},
	{0x4f,0xe4,429819},
	{0x4f,0xe6,430816},
	{0x4f,0xe8,431804},
	{0x4f,0xea,432781},
	{0x4f,0xec,433748},
	{0x4f,0xee,434706},
	{0x4f,0xf0,435653},
	{0x4f,0xf2,436592},
	{0x4f,0xf4,437521},
	{0x4f,0xf6,438441},
	{0x4f,0xf8,439352},
	{0x4f,0xfa,440255},
	{0x4f,0xfc,441149},
	{0x4f,0xfe,442035},
	{0x5f,0x80,443135},
};

static inline struct sc230ai_info *to_state(struct v4l2_subdev *sd)
{
	return container_of(sd, struct sc230ai_info, sd);
}

static inline struct v4l2_subdev *to_sd(struct v4l2_ctrl *ctrl)
{
	return &container_of(ctrl->handler, struct sc230ai_info, hdl)->sd;
}
struct regval_list {
	uint16_t reg_num;
	unsigned char value;
};

static struct regval_list sc230ai_init_regs_1920_1080_30fps_MIPI[] = {
	{0x0103,0x01},
	{0x0100,0x00},
	{0x36e9,0x80},
	{0x37f9,0x80},
	{0x301f,0x01},
	{0x3221,0x06},
	{0x3301,0x07},
	{0x3304,0x50},
	{0x3306,0x70},
	{0x3308,0x18},
	{0x3309,0x68},
	{0x330a,0x01},
	{0x330b,0x20},
	{0x331e,0x41},
	{0x331f,0x59},
	{0x3333,0x10},
	{0x3334,0x40},
	{0x335d,0x60},
	{0x335e,0x06},
	{0x335f,0x08},
	{0x3364,0x5e},
	{0x337c,0x02},
	{0x337d,0x0a},
	{0x3390,0x01},
	{0x3391,0x0b},
	{0x3392,0x0f},
	{0x3393,0x09},
	{0x3394,0x0d},
	{0x3395,0x60},
	{0x3396,0x48},
	{0x3397,0x49},
	{0x3398,0x4b},
	{0x3399,0x06},
	{0x339a,0x0a},
	{0x339b,0x0d},
	{0x339c,0x60},
	{0x33a2,0x04},
	{0x33af,0x40},
	{0x33b1,0x80},
	{0x33b3,0x40},
	{0x33b9,0x0a},
	{0x33f9,0xa0},
	{0x33fb,0xbf},
	{0x33fc,0x5f},
	{0x33fd,0x7f},
	{0x349f,0x03},
	{0x34a6,0x4b},
	{0x34a7,0x5f},
	{0x34a8,0x30},
	{0x34a9,0x20},
	{0x34aa,0x01},
	{0x34ab,0x28},
	{0x34ac,0x01},
	{0x34ad,0x58},
	{0x34f8,0x7f},
	{0x34f9,0x10},
	{0x3630,0xc0},
	{0x3633,0x44},
	{0x363b,0x20},
	{0x3670,0x09},
	{0x3674,0xb0},
	{0x3675,0x80},
	{0x3676,0x88},
	{0x367c,0x40},
	{0x367d,0x49},
	{0x3690,0x44},
	{0x3691,0x33},
	{0x3692,0x43},
	{0x369c,0x49},
	{0x369d,0x4f},
	{0x36ae,0x4b},
	{0x36af,0x4f},
	{0x36b0,0x87},
	{0x36b1,0x94},
	{0x36b2,0xbc},
	{0x36d0,0x01},
	{0x3722,0x97},
	{0x3728,0x90},
	{0x3901,0x02},
	{0x3902,0xc5},
	{0x3904,0x04},
	{0x3907,0x00},
	{0x3908,0x41},
	{0x3909,0x00},
	{0x390a,0x00},
	{0x3933,0x84},
	{0x3934,0x10},
	{0x3940,0x78},
	{0x3942,0x04},
	{0x3943,0x11},
	{0x3e00,0x00},
	{0x3e01,0x8c},
	{0x3e02,0x20},
	{0x440e,0x02},
	{0x5010,0x01},
	{0x5787,0x08},
	{0x5788,0x03},
	{0x5789,0x00},
	{0x578a,0x10},
	{0x578b,0x08},
	{0x578c,0x00},
	{0x5790,0x08},
	{0x5791,0x04},
	{0x5792,0x00},
	{0x5793,0x10},
	{0x5794,0x08},
	{0x5795,0x00},
	{0x5799,0x06},
	{0x57ad,0x00},
	{0x5ae0,0xfe},
	{0x5ae1,0x40},
	{0x5ae2,0x3f},
	{0x5ae3,0x38},
	{0x5ae4,0x28},
	{0x5ae5,0x3f},
	{0x5ae6,0x38},
	{0x5ae7,0x28},
	{0x5ae8,0x3f},
	{0x5ae9,0x3c},
	{0x5aea,0x2c},
	{0x5aeb,0x3f},
	{0x5aec,0x3c},
	{0x5aed,0x2c},
	{0x5af4,0x3f},
	{0x5af5,0x38},
	{0x5af6,0x28},
	{0x5af7,0x3f},
	{0x5af8,0x38},
	{0x5af9,0x28},
	{0x5afa,0x3f},
	{0x5afb,0x3c},
	{0x5afc,0x2c},
	{0x5afd,0x3f},
	{0x5afe,0x3c},
	{0x5aff,0x2c},
	{0x36e9,0x20},
	{0x37f9,0x27},
	{SC230AI_REG_END, 0x00},	/* END MARKER */
};

static struct regval_list sc230ai_init_regs_1920_1080_60fps_MIPI[] = {
	{0x0103,0x01},
	{0x0100,0x00},
	{0x36e9,0x80},
	{0x37f9,0x80},
	{0x301f,0x02},
	{0x3221,0x06},
	{0x3301,0x09},
	{0x3304,0x50},
	{0x3306,0x48},
	{0x3308,0x18},
	{0x3309,0x68},
	{0x330a,0x00},
	{0x330b,0xc0},
	{0x331e,0x41},
	{0x331f,0x59},
	{0x3333,0x10},
	{0x3334,0x40},
	{0x335d,0x60},
	{0x335e,0x06},
	{0x335f,0x08},
	{0x3364,0x5e},
	{0x337c,0x02},
	{0x337d,0x0a},
	{0x3390,0x01},
	{0x3391,0x0b},
	{0x3392,0x0f},
	{0x3393,0x0c},
	{0x3394,0x0d},
	{0x3395,0x60},
	{0x3396,0x48},
	{0x3397,0x49},
	{0x3398,0x4f},
	{0x3399,0x0a},
	{0x339a,0x0f},
	{0x339b,0x14},
	{0x339c,0x60},
	{0x33a2,0x04},
	{0x33af,0x40},
	{0x33b1,0x80},
	{0x33b3,0x40},
	{0x33b9,0x0a},
	{0x33f9,0x70},
	{0x33fb,0x90},
	{0x33fc,0x4b},
	{0x33fd,0x5f},
	{0x349f,0x03},
	{0x34a6,0x4b},
	{0x34a7,0x4f},
	{0x34a8,0x30},
	{0x34a9,0x20},
	{0x34aa,0x00},
	{0x34ab,0xe0},
	{0x34ac,0x01},
	{0x34ad,0x00},
	{0x34f8,0x5f},
	{0x34f9,0x10},
	{0x3630,0xc0},
	{0x3633,0x44},
	{0x3637,0x29},
	{0x363b,0x20},
	{0x3670,0x09},
	{0x3674,0xb0},
	{0x3675,0x80},
	{0x3676,0x88},
	{0x367c,0x40},
	{0x367d,0x49},
	{0x3690,0x44},
	{0x3691,0x44},
	{0x3692,0x54},
	{0x369c,0x49},
	{0x369d,0x4f},
	{0x36ae,0x4b},
	{0x36af,0x4f},
	{0x36b0,0x87},
	{0x36b1,0x94},
	{0x36b2,0xbc},
	{0x36d0,0x01},
	{0x36ea,0x0b},
	{0x36eb,0x04},
	{0x36ec,0x0c},
	{0x36ed,0x24},
	{0x370f,0x01},
	{0x3722,0x17},
	{0x3728,0x90},
	{0x37b0,0x17},
	{0x37b1,0x17},
	{0x37b2,0x97},
	{0x37b3,0x4b},
	{0x37b4,0x4f},
	{0x37fa,0x0b},
	{0x37fb,0x24},
	{0x37fc,0x10},
	{0x37fd,0x22},
	{0x3901,0x02},
	{0x3902,0xc5},
	{0x3904,0x04},
	{0x3907,0x00},
	{0x3908,0x41},
	{0x3909,0x00},
	{0x390a,0x00},
	{0x391f,0x04},
	{0x3933,0x84},
	{0x3934,0x10},
	{0x3940,0x78},
	{0x3942,0x04},
	{0x3943,0x11},
	{0x3e00,0x00},
	{0x3e01,0x8c},
	{0x3e02,0x20},
	{0x440e,0x02},
	{0x5010,0x01},
	{0x5787,0x08},
	{0x5788,0x03},
	{0x5789,0x00},
	{0x578a,0x10},
	{0x578b,0x08},
	{0x578c,0x00},
	{0x5790,0x08},
	{0x5791,0x04},
	{0x5792,0x00},
	{0x5793,0x10},
	{0x5794,0x08},
	{0x5795,0x00},
	{0x5799,0x06},
	{0x57ad,0x00},
	{0x5ae0,0xfe},
	{0x5ae1,0x40},
	{0x5ae2,0x3f},
	{0x5ae3,0x38},
	{0x5ae4,0x28},
	{0x5ae5,0x3f},
	{0x5ae6,0x38},
	{0x5ae7,0x28},
	{0x5ae8,0x3f},
	{0x5ae9,0x3c},
	{0x5aea,0x2c},
	{0x5aeb,0x3f},
	{0x5aec,0x3c},
	{0x5aed,0x2c},
	{0x5af4,0x3f},
	{0x5af5,0x38},
	{0x5af6,0x28},
	{0x5af7,0x3f},
	{0x5af8,0x38},
	{0x5af9,0x28},
	{0x5afa,0x3f},
	{0x5afb,0x3c},
	{0x5afc,0x2c},
	{0x5afd,0x3f},
	{0x5afe,0x3c},
	{0x5aff,0x2c},
	{0x36e9,0x20},
	{0x37f9,0x24},
	{SC230AI_REG_END, 0x00},
};


static struct regval_list sc230ai_stream_on[] = {
	{0x0100,0x01},
	{SC230AI_REG_END, 0x00},
};

static struct regval_list sc230ai_stream_off[] = {
	{0x0100,0x00},
	{SC230AI_REG_END, 0x00},
};


int sc230ai_read(struct v4l2_subdev *sd, unsigned short reg,
		unsigned char *value)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	uint8_t buf[2] = {(reg>>8)&0xff, reg&0xff};
	struct i2c_msg msg[2] = {
		[0] = {
			.addr	= client->addr,
			.flags	= 0,
			.len	= 2,
			.buf	= buf,
		},
		[1] = {
			.addr	= client->addr,
			.flags	= I2C_M_RD,
			.len	= 1,
			.buf	= value,
		}
	};
	int ret;
	ret = i2c_transfer(client->adapter, msg, 2);
	if (ret > 0)
		ret = 0;

	return ret;
}

static int sc230ai_write(struct v4l2_subdev *sd, unsigned short reg,
			unsigned char value)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	uint8_t buf[3] = {(reg>>8)&0xff, reg&0xff, value};
	struct i2c_msg msg = {
		.addr	= client->addr,
		.flags	= 0,
		.len	= 3,
		.buf	= buf,
	};
	int ret;
	unsigned int timeout  = 100;
	while(timeout--){
		ret = i2c_transfer(client->adapter, &msg, 1);
		if(ret == -EAGAIN){
			msleep(100);
			continue;
		}
		else
			break;
	}
	if (ret > 0)
		ret = 0;
	return ret;
}

static int sc230ai_read_array(struct v4l2_subdev *sd, struct regval_list *vals)
{
	int ret;
	unsigned char val;
	while (vals->reg_num != SC230AI_REG_END) {
		if (vals->reg_num == SC230AI_REG_DELAY) {
				msleep(vals->value);
		} else {
			ret = sc230ai_read(sd, vals->reg_num, &val);
			if (ret < 0)
				return ret;
			if (vals->reg_num == SC230AI_PAGE_REG){
				val &= 0xf8;
				val |= (vals->value & 0x07);
				ret = sc230ai_write(sd, vals->reg_num, val);
				ret = sc230ai_read(sd, vals->reg_num, &val);
			}
		}
		vals++;
	}
	return 0;
}
static int sc230ai_write_array(struct v4l2_subdev *sd, struct regval_list *vals)
{
	int ret;
	while (vals->reg_num != SC230AI_REG_END) {
		if (vals->reg_num == SC230AI_REG_DELAY) {
				msleep(vals->value);
		} else {
			ret = sc230ai_write(sd, vals->reg_num, vals->value);
			if (ret < 0)
				return ret;
		}
		vals++;
	}
	return 0;
}


/*
 * Stuff that knows about the sensor.
 */
static int sc230ai_lightup(struct v4l2_subdev *sd, u32 val)
{
	struct sc230ai_info *info = to_state(sd);

	if(val) {
		gpio_direction_output(info->led.pin, info->led.active_level);
	} else {
		gpio_direction_output(info->led.pin, !info->led.active_level);
	}
	return 0;
}

static int sc230ai_xshutdn(struct v4l2_subdev *sd, u32 val)
{
	struct sc230ai_info *info = to_state(sd);

	if(val) {
		gpio_direction_output(info->xshutdn.pin, info->xshutdn.active_level);
	} else {
		gpio_direction_output(info->xshutdn.pin, !info->xshutdn.active_level);
	}
	return 0;
}

static int sc230ai_pwdn(struct v4l2_subdev *sd, u32 val)
{
	struct sc230ai_info *info = to_state(sd);

	if(val) {
		gpio_direction_output(info->pwdn.pin, info->pwdn.active_level);
		msleep(10);
	} else {
		gpio_direction_output(info->pwdn.pin, !info->pwdn.active_level);
	}
	return 0;
}

static int sc230ai_reset(struct v4l2_subdev *sd, u32 val)
{
	struct sc230ai_info *info = to_state(sd);
	unsigned char v;
	int ret;

	/*software reset*/
	ret = sc230ai_read(sd, 0x0103, &v);

	if(val) {
		v |= 1;
		ret += sc230ai_write(sd, 0x0103, v);
	}
	return 0;
}


static int sc230ai_init(struct v4l2_subdev *sd, u32 val)
{
	struct sc230ai_info *info = to_state(sd);
	int ret = 0;

	ret = sc230ai_write_array(sd, info->win->regs);

	return ret;
}


static int sc230ai_detect(struct v4l2_subdev *sd, unsigned int *ident)
{
	unsigned char v;
	int ret;

	ret = sc230ai_read(sd, 0x3107, &v);
	pr_debug("-----%s: %d ret = %d, v = 0x%02x\n", __func__, __LINE__, ret,v);
	if (ret < 0)
		return ret;
	if (v != SC230AI_CHIP_ID_H)
		return -ENODEV;
	*ident = v;

	ret = sc230ai_read(sd, 0x3108, &v);
	pr_debug("-----%s: %d ret = %d, v = 0x%02x\n", __func__, __LINE__, ret,v);
	if (ret < 0)
		return ret;
	if (v != SC230AI_CHIP_ID_L)
		return -ENODEV;
	*ident = (*ident << 8) | v;
	return 0;

}

static struct sc230ai_win_size sc230ai_win_sizes[] = {
	{
		.width				= 1920,
		.height				= 1080,
		.sensor_info.fps				= 30 << 16 | 1,
		.sensor_info.mipi_cfg.twidth		= 1920,
		.sensor_info.mipi_cfg.theight		= 1080,
		.sensor_info.mipi_cfg.mipi_crop_start0x	= 0,
		.sensor_info.mipi_cfg.mipi_crop_start0y	= 0,
		.mbus_code	= MEDIA_BUS_FMT_SBGGR10_1X10,
		.colorspace	= V4L2_COLORSPACE_SRGB,
		.regs 		= sc230ai_init_regs_1920_1080_30fps_MIPI,
	},
	{
		.width				= 1920,
		.height				= 1080,
		.sensor_info.fps				= 60 << 16 | 1,
		.sensor_info.mipi_cfg.twidth		= 1920,
		.sensor_info.mipi_cfg.theight		= 1080,
		.sensor_info.mipi_cfg.mipi_crop_start0x	= 0,
		.sensor_info.mipi_cfg.mipi_crop_start0y	= 0,
		.mbus_code	= MEDIA_BUS_FMT_SBGGR10_1X10,
		.colorspace	= V4L2_COLORSPACE_SRGB,
		.regs 		= sc230ai_init_regs_1920_1080_60fps_MIPI,
	},
};

static int sc230ai_enum_mbus_code(struct v4l2_subdev *sd,
		struct v4l2_subdev_pad_config *cfg,
		struct v4l2_subdev_mbus_code_enum *code)
{
#if 0
	if (code->pad || code->index >= N_SC230AI_FMTS)
		return -EINVAL;

	code->code = sc230ai_formats[code->index].mbus_code;
#endif
	return 0;
}

/*
 * Set a format.
 */
static int sc230ai_set_fmt(struct v4l2_subdev *sd,
		struct v4l2_subdev_pad_config *cfg,
		struct v4l2_subdev_format *format)
{
	struct sc230ai_format_struct *ovfmt;
	struct sc230ai_win_size *wsize;
	struct sc230ai_info *info = to_state(sd);
	int ret;

	if (format->pad)
		return -EINVAL;

	return 0;
}

static int sc230ai_get_fmt(struct v4l2_subdev *sd,
		       struct v4l2_subdev_pad_config *cfg,
		       struct v4l2_subdev_format *format)
{
	struct sc230ai_info *info = to_state(sd);
	struct v4l2_mbus_framefmt *fmt = &format->format;
	int ret = 0;

	if(!info->win) {
		dev_err(sd->dev, "sensor win_size not set!\n");
		return -EINVAL;
	}

	fmt->width = info->win->width;
	fmt->height = info->win->height;
	fmt->code = info->win->mbus_code;
	fmt->colorspace = info->win->colorspace;
	*(unsigned int *)fmt->reserved = &info->win->sensor_info;


//	printk("----%s, %d, width: %d, height: %d, code: %x\n",
//			__func__, __LINE__, fmt->width, fmt->height, fmt->code);

	return ret;
}

static int sc230ai_s_brightness(struct v4l2_subdev *sd, int value)
{
	int ret = 0;

	return ret;
}

static int sc230ai_s_contrast(struct v4l2_subdev *sd, int value)
{
	int ret = 0;

	return ret;
}

static int sc230ai_s_hflip(struct v4l2_subdev *sd, int value)
{
	int ret = 0;

	return ret;
}

static int sc230ai_s_vflip(struct v4l2_subdev *sd, int value)
{
	int ret = 0;

	return ret;
}

/*
 * GAIN is split between REG_GAIN and REG_VREF[7:6].  If one believes
 * the data sheet, the VREF parts should be the most significant, but
 * experience shows otherwise.  There seems to be little value in
 * messing with the VREF bits, so we leave them alone.
 */
static int sc230ai_g_gain(struct v4l2_subdev *sd, __s32 *value)
{
	int ret = 0;
	unsigned char gain = 0;

	*value = gain;
	return ret;
}

static int sc230ai_s_gain(struct v4l2_subdev *sd, int value)
{
	int ret = 0;

//	printk("---%s, %d, s_gain: value: %d\n", __func__, __LINE__, value);

	return ret;
}

static unsigned int again_to_regval(int gain, unsigned int *value, unsigned int *fine_value)
{
	struct again_lut *lut = NULL;
	int i = 0;
	for(i = 0; i < ARRAY_SIZE(sc230ai_again_lut); i++) {
		lut = &sc230ai_again_lut[i];

		if(gain <= lut->gain) {
			*value = lut->value;
			*fine_value = lut->fine_value;
			return lut->value;
		}
	}
	/*last value.*/
	*value = lut->value;
	*fine_value = lut->fine_value;
	return lut->value;
}

static int regval_to_again(unsigned int regval, unsigned int fine_reg_val)
{
	struct again_lut *lut = NULL;
	int i = 0;
	for(i = 0; i < ARRAY_SIZE(sc230ai_again_lut); i++) {
		lut = &sc230ai_again_lut[i];

		if(regval == lut->value && fine_reg_val == lut->fine_value) {
			return lut->gain;
		}
	}
	printk("%s, %d regval not mapped to isp gain\n", __func__, __LINE__);
	return -EINVAL;
}

static int sc230ai_g_again(struct v4l2_subdev *sd, __s32 *value)
{
	char v = 0;
	unsigned int reg_val = 0;
	unsigned int fine_reg_val = 0;
	int ret = 0;


	ret = sc230ai_read(sd, 0x3e09, &v);
	reg_val = v ;
	ret = sc230ai_read(sd, 0x3e07, &v);
	fine_reg_val = v ;

	*value = regval_to_again(reg_val, fine_reg_val);

	return ret;

}
/*set analog gain db value, map value to sensor register.*/
static int sc230ai_s_again(struct v4l2_subdev *sd, int value)
{
	struct sc230ai_info *info = to_state(sd);
	unsigned int reg_value;
	unsigned int reg_fine_value;
	int ret = 0;

	if(value < info->again->minimum || value > info->again->maximum) {
		/* use default value. */
		again_to_regval(info->again->default_value, &reg_value, &reg_fine_value);
	} else {
		again_to_regval(value, &reg_value, &reg_fine_value);
	}

	ret += sc230ai_write(sd, 0x3e09, (unsigned char)(reg_value));
	ret += sc230ai_write(sd, 0x3e07, (unsigned char)(reg_fine_value));
	if (ret < 0){
		printk("sc230ai_write error  %d\n" ,__LINE__ );
		return ret;
	}
	return 0;
}

/*
 * Tweak autogain.
 */
static int sc230ai_s_autogain(struct v4l2_subdev *sd, int value)
{
	int ret = 0;

	return ret;
}

static int sc230ai_s_exp(struct v4l2_subdev *sd, int value)
{
	struct sc230ai_info *info = to_state(sd);
	int ret = 0;

	value *= 2; /*unit in half line*/
	ret += sc230ai_write(sd, 0x3e00, (unsigned char)((value >> 12) & 0xf));
	ret += sc230ai_write(sd, 0x3e01, (unsigned char)((value >> 4) & 0xff));
	ret += sc230ai_write(sd, 0x3e02, (unsigned char)(value & 0xf) << 4);

	if (ret < 0) {
		printk("sc230ai_write error  %d\n" ,__LINE__);
		return ret;
	}

	return 0;
}

static int sc230ai_g_volatile_ctrl(struct v4l2_ctrl *ctrl)
{
	struct v4l2_subdev *sd = to_sd(ctrl);
	struct sc230ai_info *info = to_state(sd);

	switch (ctrl->id) {
	case V4L2_CID_AUTOGAIN:
		return sc230ai_g_gain(sd, &info->gain->val);
	case V4L2_CID_ANALOGUE_GAIN:
		return sc230ai_g_again(sd, &info->again->val);
	}
	return -EINVAL;
}

static int sc230ai_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct v4l2_subdev *sd = to_sd(ctrl);
	struct sc230ai_info *info = to_state(sd);

	switch (ctrl->id) {
	case V4L2_CID_BRIGHTNESS:
		return sc230ai_s_brightness(sd, ctrl->val);
	case V4L2_CID_CONTRAST:
		return sc230ai_s_contrast(sd, ctrl->val);
	case V4L2_CID_VFLIP:
		return sc230ai_s_vflip(sd, ctrl->val);
	case V4L2_CID_HFLIP:
		return sc230ai_s_hflip(sd, ctrl->val);
	case V4L2_CID_AUTOGAIN:
		/* Only set manual gain if auto gain is not explicitly
		   turned on. */
		if (!ctrl->val) {
			/* sc230ai_s_gain turns off auto gain */
			return sc230ai_s_gain(sd, info->gain->val);
		}
		return sc230ai_s_autogain(sd, ctrl->val);
	case V4L2_CID_GAIN:
		return sc230ai_s_gain(sd, ctrl->val);
	case V4L2_CID_ANALOGUE_GAIN:
		return sc230ai_s_again(sd, ctrl->val);
	case V4L2_CID_EXPOSURE:
		return sc230ai_s_exp(sd, ctrl->val);
	}
	return -EINVAL;
}

static const struct v4l2_ctrl_ops sc230ai_ctrl_ops = {
	.s_ctrl = sc230ai_s_ctrl,
	.g_volatile_ctrl = sc230ai_g_volatile_ctrl,
};

#ifdef CONFIG_VIDEO_ADV_DEBUG
static int sc230ai_g_register(struct v4l2_subdev *sd, struct v4l2_dbg_register *reg)
{
	unsigned char val = 0;
	int ret;

	ret = sc230ai_read(sd, reg->reg & 0xffff, &val);
	reg->val = val;
	reg->size = 1;
	return ret;
}

static int sc230ai_s_register(struct v4l2_subdev *sd, const struct v4l2_dbg_register *reg)
{
	sc230ai_write(sd, reg->reg & 0xffff, reg->val & 0xff);
	return 0;
}
#endif

static int sc230ai_power(struct v4l2_subdev *sd, int on)
{
	struct sc230ai_info *info = to_state(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret;

	info->supply.avdd  = devm_regulator_get_optional(&client->dev, "avdd");
	info->supply.dvdd  = devm_regulator_get_optional(&client->dev, "dvdd");
	info->supply.dovdd = devm_regulator_get_optional(&client->dev, "dovdd");

	if (IS_ERR(info->supply.avdd)||IS_ERR(info->supply.dvdd)
		||IS_ERR(info->supply.dovdd)) {
			if ((PTR_ERR(info->supply.avdd) == -EPROBE_DEFER)
				||(PTR_ERR(info->supply.avdd) == -EPROBE_DEFER)
					||(PTR_ERR(info->supply.avdd) == -EPROBE_DEFER))
				return -EPROBE_DEFER;
			printk("No sc230ai vdd regulator found\n");
	}

	if ((!IS_ERR(info->supply.avdd))&&(!IS_ERR(info->supply.avdd))
			&&(!IS_ERR(info->supply.avdd)))  {
		if(on){
			ret = regulator_enable(info->supply.avdd);
			ret = regulator_enable(info->supply.dvdd);
			ret = regulator_enable(info->supply.dovdd);
			if (ret)
			   dev_err(&client->dev, "sc230ai vdd supply disable failed\n");
		}
		else{
			ret = regulator_disable(info->supply.avdd);
			ret = regulator_disable(info->supply.dvdd);
			ret = regulator_disable(info->supply.dovdd);
			if (ret)
			    dev_err(&client->dev, "sc230ai vdd supply disable failed\n");
		}
	}else{
		dev_err(&client->dev, "sc230ai vdd supply IS_ERR failed\n");
	}
	return ret;
}



int sc230ai_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct sc230ai_info *info = to_state(sd);
	int ret = 0;

	if (enable) {
		ret = sc230ai_write_array(sd, sc230ai_stream_on);
		printk("sc230ai stream on\n");

	}
	else {
		ret = sc230ai_write_array(sd, sc230ai_stream_off);
		printk("sc230ai stream off\n");
	}
	return ret;
}


int sc230ai_g_frame_interval(struct v4l2_subdev *sd, struct v4l2_subdev_frame_interval *interval)
{
	struct sc230ai_info *info = to_state(sd);
	if(info->win->sensor_info.fps){
		interval->interval.numerator = info->win->sensor_info.fps & 0xffff;
		interval->interval.denominator = info->win->sensor_info.fps >> 16;
		return 0;
	}
	return -EINVAL;
}
/* ----------------------------------------------------------------------- */

static const struct v4l2_subdev_core_ops sc230ai_core_ops = {
#ifdef CONFIG_VIDEO_ADV_DEBUG
	.g_register = sc230ai_g_register,
	.s_register = sc230ai_s_register,
#endif

};

static const struct v4l2_subdev_video_ops sc230ai_video_ops = {
	.s_stream = sc230ai_s_stream,
	.g_frame_interval = sc230ai_g_frame_interval,
};

static const struct v4l2_subdev_pad_ops sc230ai_pad_ops = {
	//.enum_frame_interval = sc230ai_enum_frame_interval,
	//.num_frame_size = sc230ai_enum_frame_size,
	//.enum_mbus_code = sc230ai_enum_mbus_code,
	.set_fmt = sc230ai_set_fmt,
	.get_fmt = sc230ai_get_fmt,
};

static const struct v4l2_subdev_ops sc230ai_ops = {
	.core = &sc230ai_core_ops,
	.video = &sc230ai_video_ops,
	.pad = &sc230ai_pad_ops,
};

/* ----------------------------------------------------------------------- */

static int sc230ai_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct v4l2_subdev *sd;
	struct sc230ai_info *info;
	int ret;
	unsigned int ident = 0;
	int gpio = -EINVAL;
	unsigned int flags;

	info = devm_kzalloc(&client->dev, sizeof(*info), GFP_KERNEL);
	if (info == NULL)
		return -ENOMEM;
	sd = &info->sd;

	gpio = of_get_named_gpio_flags(client->dev.of_node, "ingenic,xshutdn-gpio", 0, &flags);
	if(gpio_is_valid(gpio)) {
		info->xshutdn.pin = gpio;
		info->xshutdn.active_level = (flags == OF_GPIO_ACTIVE_LOW) ? 0 : 1;
	}
	gpio = of_get_named_gpio_flags(client->dev.of_node, "ingenic,pwdn-gpio", 0, &flags);
	if(gpio_is_valid(gpio)) {
		info->pwdn.pin = gpio;
		info->pwdn.active_level = (flags == OF_GPIO_ACTIVE_LOW) ? 0 : 1;
	}
	gpio = of_get_named_gpio_flags(client->dev.of_node, "ingenic,efsync-gpio", 0, &flags);
	if(gpio_is_valid(gpio)) {
		info->efsync.pin = gpio;
		info->efsync.active_level = (flags == OF_GPIO_ACTIVE_LOW) ? 0 : 1;
	}
	gpio = of_get_named_gpio_flags(client->dev.of_node, "ingenic,led-gpio", 0, &flags);
	if(gpio_is_valid(gpio)) {
		info->led.pin = gpio;
		info->led.active_level = (flags == OF_GPIO_ACTIVE_LOW) ? 0 : 1;
	}


	v4l2_i2c_subdev_init(sd, client, &sc230ai_ops);
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;

	sc230ai_power(sd,1);

	/*clk*/
	info->clk = v4l2_clk_get(&client->dev, "div_cim");
	if (IS_ERR(info->clk)) {
		ret = PTR_ERR(info->clk);
		goto err_clkget;
	}

	ret = v4l2_clk_set_rate(info->clk, 24000000);
	if(ret)
		dev_err(sd->dev, "clk_set_rate err!\n");

	ret = v4l2_clk_enable(info->clk);
	if(ret)
		dev_err(sd->dev, "clk_enable err!\n");

	sc230ai_xshutdn(sd, 1);
	sc230ai_pwdn(sd, 1);
	sc230ai_lightup(sd, 0);
	sc230ai_reset(sd, 1);
#if 1
	/* Make sure it's an sc230ai */
	ret = sc230ai_detect(sd, &ident);
	if (ret) {
		v4l_err(client,
			"chip found @ 0x%x (%s) is not an sc230ai chip.\n",
			client->addr << 1, client->adapter->name);
		return ret;
	}
#endif

	v4l_info(client, "chip found @ 0x%02x (%s)\n",
			client->addr << 1, client->adapter->name);

	v4l2_ctrl_handler_init(&info->hdl, 8);
	v4l2_ctrl_new_std(&info->hdl, &sc230ai_ctrl_ops,
			V4L2_CID_BRIGHTNESS, 0, 255, 1, 128);
	v4l2_ctrl_new_std(&info->hdl, &sc230ai_ctrl_ops,
			V4L2_CID_CONTRAST, 0, 127, 1, 64);
	v4l2_ctrl_new_std(&info->hdl, &sc230ai_ctrl_ops,
			V4L2_CID_VFLIP, 0, 1, 1, 0);
	v4l2_ctrl_new_std(&info->hdl, &sc230ai_ctrl_ops,
			V4L2_CID_HFLIP, 0, 1, 1, 0);
	info->gain = v4l2_ctrl_new_std(&info->hdl, &sc230ai_ctrl_ops,
			V4L2_CID_GAIN, 0, 255, 1, 128);
	info->again = v4l2_ctrl_new_std(&info->hdl, &sc230ai_ctrl_ops,
			V4L2_CID_ANALOGUE_GAIN, 0, 443135, 1, 10000);
	/*unit exposure lines: */
	info->exposure = v4l2_ctrl_new_std(&info->hdl, &sc230ai_ctrl_ops,
			V4L2_CID_EXPOSURE, 1,  1152 - 4 , 4, 600);

	sd->ctrl_handler = &info->hdl;
	if (info->hdl.error) {
		int err = info->hdl.error;

		v4l2_ctrl_handler_free(&info->hdl);
		return err;
	}
	v4l2_ctrl_handler_setup(&info->hdl);

	info->win = &sc230ai_win_sizes[0];
	sc230ai_init(sd, 1);

	info->pad.flags = MEDIA_PAD_FL_SOURCE;
	ret = media_entity_pads_init(&info->sd.entity, 1, &info->pad);
	if(ret < 0) {
		goto err_entity_init;
	}
	info->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;
	ret = v4l2_async_register_subdev(&info->sd);
	if (ret < 0)
		goto err_videoprobe;

	dev_info(&client->dev, "sc230ai Probed\n");
	return 0;
err_videoprobe:
err_entity_init:
	v4l2_clk_put(info->clk);
err_clkget:
	return ret;
}


static int sc230ai_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct sc230ai_info *info = to_state(sd);

	v4l2_device_unregister_subdev(sd);
	v4l2_ctrl_handler_free(&info->hdl);
	v4l2_clk_put(info->clk);
	return 0;
}

static const struct i2c_device_id sc230ai_id[] = {
	{ "sc230ai", 0},
	{ }
};
MODULE_DEVICE_TABLE(i2c, sc230ai_id);

static const struct of_device_id sc230ai_of_match[] = {
	{.compatible = "smartsens,sc230ai", },
	{},
};
MODULE_DEVICE_TABLE(of, sc230ai_of_match);


static int sc230ai_suspend(struct device *dev)
{
        struct i2c_client *client = container_of(dev, struct i2c_client, dev);
        struct v4l2_subdev *sd = i2c_get_clientdata(client);
        struct sc230ai_info *info = to_state(sd);
	int ret=0;

	sc230ai_power(sd,0);
	v4l2_clk_disable(info->clk);
        return 0;
}

static int sc230ai_resume(struct device *dev)
{
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct sc230ai_info *info = to_state(sd);
	int ret=0;

	sc230ai_power(sd,1);
	v4l2_clk_enable(info->clk);
	sc230ai_reset(sd, 1);
	sc230ai_init(sd, 1);
	return 0;
}

const struct dev_pm_ops sc230ai_pm =
{
        .suspend = sc230ai_suspend,
        .resume  = sc230ai_resume,
};


static struct i2c_driver sc230ai_driver = {
	.driver = {
		.name	= "sc230ai",
		.of_match_table = of_match_ptr(sc230ai_of_match),
		.pm	=  &sc230ai_pm,
	},
	.probe		= sc230ai_probe,
	.remove		= sc230ai_remove,
	.id_table	= sc230ai_id,
};

module_i2c_driver(sc230ai_driver);
MODULE_AUTHOR("qpz <aric.pzqi@ingenic.com>");
MODULE_DESCRIPTION("A low-level driver for SmartSens sc230ai sensors");
MODULE_LICENSE("GPL");
