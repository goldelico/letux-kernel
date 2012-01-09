/*
 * drivers/media/video/ov9655.c
 *
 * Based on mt9v113 decoder driver
 *
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 */

#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/videodev2.h>
#include <linux/io.h>

#include <media/v4l2-int-device.h>
#include <media/ov9655.h>

#include "ov9655_regs.h"

/* Module Name */
#define OV9655_MODULE_NAME		"ov9655"

/* Private macros for TVP */
#define I2C_RETRY_COUNT                 (5)

/* Debug functions */
static int debug = 1;
module_param(debug, bool, 0644);
MODULE_PARM_DESC(debug, "Debug level (0-1)");

static int ov9655_read(struct i2c_client *client, const u8 reg)
{
	return i2c_smbus_read_byte_data(client, reg);
}

static int ov9655_write(struct i2c_client *client, const u8 reg,
						const u8 value)
{
	return i2c_smbus_write_byte_data(client, reg, value);
}

static int ov9655_set(struct i2c_client *client, const u8 reg,
					  const u8 value, const u8 mask)
{
	int ret;
	
	ret = ov9655_read(client, reg);
	if (ret < 0)
		return ret;
	return ov9655_write(client, reg, (ret & ~mask) | (value & mask));
}

static int ov9655_write_regs(struct i2c_client *client,
							 const struct ov9655_reg *regs, const int n)
{
	int i, ret;
	
	for (i = 0; i < n; i++) {
		ret = ov9655_write(client, regs->addr, regs->value);
		if (ret < 0)
			return ret;
		regs++;
	}
	
	return 0;
}

/*
 * enum ov9655_std - enum for supported standards
 */
enum ov9655_std {
	OV9655_STD_VGA = 0,
	OV9655_STD_QVGA,
	OV9655_STD_SXGA,
	OV9655_STD_CIF,
	OV9655_STD_INVALID
};

/*
 * enum ov9655_state - enum for different decoder states
 */
enum ov9655_state {
	STATE_NOT_DETECTED,
	STATE_DETECTED
};

/*
 * struct ov9655_std_info - Structure to store standard informations
 * @width: Line width in pixels
 * @height:Number of active lines
 * @video_std: Value to write in REG_VIDEO_STD register
 * @standard: v4l2 standard structure information
 */
struct ov9655_std_info {
	unsigned long width;
	unsigned long height;
	u8 video_std;
	struct v4l2_standard standard;
};

/*
 * struct ov9655_decoded - decoder object
 * @v4l2_int_device: Slave handle
 * @pdata: Board specific
 * @client: I2C client data
 * @id: Entry from I2C table
 * @ver: Chip version
 * @state: decoder state - detected or not-detected
 * @pix: Current pixel format
 * @num_fmts: Number of formats
 * @fmt_list: Format list
 * @current_std: Current standard
 * @num_stds: Number of standards
 * @std_list: Standards list
 */
struct ov9655_decoder {
	struct v4l2_int_device *v4l2_int_device;
	const struct ov9655_platform_data *pdata;
	struct i2c_client *client;

	struct i2c_device_id *id;

	int ver;
	enum ov9655_state state;

	struct v4l2_pix_format pix;
	int num_fmts;
	const struct v4l2_fmtdesc *fmt_list;

	enum ov9655_std current_std;
	int num_stds;
	struct ov9655_std_info *std_list;
};


/* Number of pixels and number of lines per frame for different standards */
#define VGA_NUM_ACTIVE_PIXELS		(640)	/* 4:3 */
#define VGA_NUM_ACTIVE_LINES		(480)
#define QVGA_NUM_ACTIVE_PIXELS		(320)	/* 4:3 */
#define QVGA_NUM_ACTIVE_LINES		(240)
#define SXGA_NUM_ACTIVE_PIXELS		(1280)	/* 5:4 */
#define SXGA_NUM_ACTIVE_LINES		(1024)
#define CIF_NUM_ACTIVE_PIXELS		(352)	/* 5:4 */
#define CIF_NUM_ACTIVE_LINES		(288)

#ifdef OLD_MATERIAL

/* Tokens for register write */
#define TOK_WRITE                       (0)     /* token for write operation */
#define TOK_TERM                        (1)     /* terminating token */
#define TOK_DELAY                       (2)     /* delay token for reg list */
#define TOK_SKIP                        (3)     /* token to skip a register */

/* OV9655 register set for VGA mode */
static struct ov9655_reg ov9655_vga_reg[] = {
	{TOK_WRITE, 0x098C, 0x2739},
	{TOK_WRITE, 0x0990, 0x0000},
	{TOK_WRITE, 0x098C, 0x273B},
	{TOK_WRITE, 0x0990, 0x027F},
	{TOK_WRITE, 0x098C, 0x273D},
	{TOK_WRITE, 0x0990, 0x0000},
	{TOK_WRITE, 0x098C, 0x273F},
	{TOK_WRITE, 0x0990, 0x01DF},
	{TOK_WRITE, 0x098C, 0x2703},
	{TOK_WRITE, 0x0990, 0x0280},
	{TOK_WRITE, 0x098C, 0x2705},
	{TOK_WRITE, 0x0990, 0x01E0},
	{TOK_WRITE, 0x098C, 0x2715},
	{TOK_WRITE, 0x0990, 0x0001},
	{TOK_WRITE, 0x098C, 0x2717},
	{TOK_WRITE, 0x0990, 0x0026},
	{TOK_WRITE, 0x098C, 0x2719},
	{TOK_WRITE, 0x0990, 0x001A},
	{TOK_WRITE, 0x098C, 0x271B},
	{TOK_WRITE, 0x0990, 0x006B},
	{TOK_WRITE, 0x098C, 0x271D},
	{TOK_WRITE, 0x0990, 0x006B},
	{TOK_WRITE, 0x098C, 0x271F},
	{TOK_WRITE, 0x0990, 0x0202},
	{TOK_WRITE, 0x098C, 0x2721},
	{TOK_WRITE, 0x0990, 0x034A},

	{TOK_WRITE, 0x098C, 0xA103},
	{TOK_WRITE, 0x0990, 0x0005},
	{TOK_DELAY, 0, 100},
	{TOK_TERM, 0, 0},
};

/* OV9655 default register values */
static struct ov9655_reg ov9655_reg_list[] = {
	{TOK_WRITE, 0x0018, 0x4028},
	{TOK_DELAY, 0, 100},
	{TOK_WRITE, 0x001A, 0x0011},
	{TOK_WRITE, 0x001A, 0x0010},
	{TOK_WRITE, 0x0018, 0x4028},
	{TOK_DELAY, 0, 100},
	{TOK_WRITE, 0x098C, 0x02F0},
	{TOK_WRITE, 0x0990, 0x0000},
	{TOK_WRITE, 0x098C, 0x02F2},
	{TOK_WRITE, 0x0990, 0x0210},
	{TOK_WRITE, 0x098C, 0x02F4},
	{TOK_WRITE, 0x0990, 0x001A},
	{TOK_WRITE, 0x098C, 0x2145},
	{TOK_WRITE, 0x0990, 0x02F4},
	{TOK_WRITE, 0x098C, 0xA134},
	{TOK_WRITE, 0x0990, 0x0001},
	{TOK_WRITE, 0x31E0, 0x0001},
	{TOK_WRITE, 0x001A, 0x0210},
	{TOK_WRITE, 0x001E, 0x0777},
	{TOK_WRITE, 0x0016, 0x42DF},
	{TOK_WRITE, 0x0014, 0x2145},
	{TOK_WRITE, 0x0010, 0x0234},
	{TOK_WRITE, 0x0012, 0x0000},
	{TOK_WRITE, 0x0014, 0x244B},
	{TOK_WRITE, 0x0014, 0x304B},
	{TOK_DELAY, 0, 100},
	{TOK_WRITE, 0x0014, 0xB04A},
	{TOK_WRITE, 0x098C, 0xAB1F},
	{TOK_WRITE, 0x0990, 0x00C7},
	{TOK_WRITE, 0x098C, 0xAB31},
	{TOK_WRITE, 0x0990, 0x001E},
	{TOK_WRITE, 0x098C, 0x274F},
	{TOK_WRITE, 0x0990, 0x0004},
	{TOK_WRITE, 0x098C, 0x2741},
	{TOK_WRITE, 0x0990, 0x0004},
	{TOK_WRITE, 0x098C, 0xAB20},
	{TOK_WRITE, 0x0990, 0x0054},
	{TOK_WRITE, 0x098C, 0xAB21},
	{TOK_WRITE, 0x0990, 0x0046},
	{TOK_WRITE, 0x098C, 0xAB22},
	{TOK_WRITE, 0x0990, 0x0002},
	{TOK_WRITE, 0x098C, 0xAB24},
	{TOK_WRITE, 0x0990, 0x0005},
	{TOK_WRITE, 0x098C, 0x2B28},
	{TOK_WRITE, 0x0990, 0x170C},
	{TOK_WRITE, 0x098C, 0x2B2A},
	{TOK_WRITE, 0x0990, 0x3E80},
	{TOK_WRITE, 0x3210, 0x09A8},
	{TOK_WRITE, 0x098C, 0x2306},
	{TOK_WRITE, 0x0990, 0x0315},
	{TOK_WRITE, 0x098C, 0x2308},
	{TOK_WRITE, 0x0990, 0xFDDC},
	{TOK_WRITE, 0x098C, 0x230A},
	{TOK_WRITE, 0x0990, 0x003A},
	{TOK_WRITE, 0x098C, 0x230C},
	{TOK_WRITE, 0x0990, 0xFF58},
	{TOK_WRITE, 0x098C, 0x230E},
	{TOK_WRITE, 0x0990, 0x02B7},
	{TOK_WRITE, 0x098C, 0x2310},
	{TOK_WRITE, 0x0990, 0xFF31},
	{TOK_WRITE, 0x098C, 0x2312},
	{TOK_WRITE, 0x0990, 0xFF4C},
	{TOK_WRITE, 0x098C, 0x2314},
	{TOK_WRITE, 0x0990, 0xFE4C},
	{TOK_WRITE, 0x098C, 0x2316},
	{TOK_WRITE, 0x0990, 0x039E},
	{TOK_WRITE, 0x098C, 0x2318},
	{TOK_WRITE, 0x0990, 0x001C},
	{TOK_WRITE, 0x098C, 0x231A},
	{TOK_WRITE, 0x0990, 0x0039},
	{TOK_WRITE, 0x098C, 0x231C},
	{TOK_WRITE, 0x0990, 0x007F},
	{TOK_WRITE, 0x098C, 0x231E},
	{TOK_WRITE, 0x0990, 0xFF77},
	{TOK_WRITE, 0x098C, 0x2320},
	{TOK_WRITE, 0x0990, 0x000A},
	{TOK_WRITE, 0x098C, 0x2322},
	{TOK_WRITE, 0x0990, 0x0020},
	{TOK_WRITE, 0x098C, 0x2324},
	{TOK_WRITE, 0x0990, 0x001B},
	{TOK_WRITE, 0x098C, 0x2326},
	{TOK_WRITE, 0x0990, 0xFFC6},
	{TOK_WRITE, 0x098C, 0x2328},
	{TOK_WRITE, 0x0990, 0x0086},
	{TOK_WRITE, 0x098C, 0x232A},
	{TOK_WRITE, 0x0990, 0x00B5},
	{TOK_WRITE, 0x098C, 0x232C},
	{TOK_WRITE, 0x0990, 0xFEC3},
	{TOK_WRITE, 0x098C, 0x232E},
	{TOK_WRITE, 0x0990, 0x0001},
	{TOK_WRITE, 0x098C, 0x2330},
	{TOK_WRITE, 0x0990, 0xFFEF},
	{TOK_WRITE, 0x098C, 0xA348},
	{TOK_WRITE, 0x0990, 0x0008},
	{TOK_WRITE, 0x098C, 0xA349},
	{TOK_WRITE, 0x0990, 0x0002},
	{TOK_WRITE, 0x098C, 0xA34A},
	{TOK_WRITE, 0x0990, 0x0090},
	{TOK_WRITE, 0x098C, 0xA34B},
	{TOK_WRITE, 0x0990, 0x00FF},
	{TOK_WRITE, 0x098C, 0xA34C},
	{TOK_WRITE, 0x0990, 0x0075},
	{TOK_WRITE, 0x098C, 0xA34D},
	{TOK_WRITE, 0x0990, 0x00EF},
	{TOK_WRITE, 0x098C, 0xA351},
	{TOK_WRITE, 0x0990, 0x0000},
	{TOK_WRITE, 0x098C, 0xA352},
	{TOK_WRITE, 0x0990, 0x007F},
	{TOK_WRITE, 0x098C, 0xA354},
	{TOK_WRITE, 0x0990, 0x0043},
	{TOK_WRITE, 0x098C, 0xA355},
	{TOK_WRITE, 0x0990, 0x0001},
	{TOK_WRITE, 0x098C, 0xA35D},
	{TOK_WRITE, 0x0990, 0x0078},
	{TOK_WRITE, 0x098C, 0xA35E},
	{TOK_WRITE, 0x0990, 0x0086},
	{TOK_WRITE, 0x098C, 0xA35F},
	{TOK_WRITE, 0x0990, 0x007E},
	{TOK_WRITE, 0x098C, 0xA360},
	{TOK_WRITE, 0x0990, 0x0082},
	{TOK_WRITE, 0x098C, 0x2361},
	{TOK_WRITE, 0x0990, 0x0040},
	{TOK_WRITE, 0x098C, 0xA363},
	{TOK_WRITE, 0x0990, 0x00D2},
	{TOK_WRITE, 0x098C, 0xA364},
	{TOK_WRITE, 0x0990, 0x00F6},
	{TOK_WRITE, 0x098C, 0xA302},
	{TOK_WRITE, 0x0990, 0x0000},
	{TOK_WRITE, 0x098C, 0xA303},
	{TOK_WRITE, 0x0990, 0x00EF},
	{TOK_WRITE, 0x098C, 0xAB20},
	{TOK_WRITE, 0x0990, 0x0024},
	{TOK_WRITE, 0x098C, 0xA103},
	{TOK_WRITE, 0x0990, 0x0006},
	{TOK_DELAY, 0, 100},
	{TOK_WRITE, 0x098C, 0xA103},
	{TOK_WRITE, 0x0990, 0x0005},
	{TOK_DELAY, 0, 100},
	{TOK_WRITE, 0x098C, 0x222D},
	{TOK_WRITE, 0x0990, 0x0081},
	{TOK_WRITE, 0x098C, 0xA408},
	{TOK_WRITE, 0x0990, 0x001F},
	{TOK_WRITE, 0x098C, 0xA409},
	{TOK_WRITE, 0x0990, 0x0021},
	{TOK_WRITE, 0x098C, 0xA40A},
	{TOK_WRITE, 0x0990, 0x0025},
	{TOK_WRITE, 0x098C, 0xA40B},
	{TOK_WRITE, 0x0990, 0x0027},
	{TOK_WRITE, 0x098C, 0x2411},
	{TOK_WRITE, 0x0990, 0x0081},
	{TOK_WRITE, 0x098C, 0x2413},
	{TOK_WRITE, 0x0990, 0x009A},
	{TOK_WRITE, 0x098C, 0x2415},
	{TOK_WRITE, 0x0990, 0x0081},
	{TOK_WRITE, 0x098C, 0x2417},
	{TOK_WRITE, 0x0990, 0x009A},
	{TOK_WRITE, 0x098C, 0xA404},
	{TOK_WRITE, 0x0990, 0x0010},
	{TOK_WRITE, 0x098C, 0xA40D},
	{TOK_WRITE, 0x0990, 0x0002},
	{TOK_WRITE, 0x098C, 0xA40E},
	{TOK_WRITE, 0x0990, 0x0003},
	{TOK_WRITE, 0x098C, 0xA410},
	{TOK_WRITE, 0x0990, 0x000A},

	{TOK_WRITE, 0x098C, 0xA20C},
	{TOK_WRITE, 0x0990, 0x0003},
	{TOK_WRITE, 0x098C, 0xA20B},
	{TOK_WRITE, 0x0990, 0x0000},
	{TOK_WRITE, 0x098C, 0xA215},
	{TOK_WRITE, 0x0990, 0x0004},

	{TOK_WRITE, 0x098C, 0xA103},
	{TOK_WRITE, 0x0990, 0x0006},
	{TOK_DELAY, 0, 100},
	/* test pattern all white*/
	/* {TOK_WRITE, 0x098C, 0xA766},
	{TOK_WRITE, 0x0990, 0x0001},
	*/
	{TOK_WRITE, 0x098C, 0xA103},
	{TOK_WRITE, 0x0990, 0x0005},
	{TOK_DELAY, 0, 100},
	{TOK_TERM, 0, 0},
};

#endif // OLD_MATERIAL

// FIXME: this list writes several registers reserved according to the (preliminary) data sheet!!!
// and it writes several strange values...
// therefore we have to suspect that this is not a very good setup

/* another source of information could be the OV9640 (not exactly compatible!!!)
 http://bsp.gateworks.com/linux/kernel/v2.6/linux-2.6.26-ti/drivers/media/video/ov9640.c
 */

static const struct ov9655_reg ov9655_init_regs[] = {
	{ OV9655_GAIN, 0x00 },
	{ OV9655_BLUE, 0x80 },
	{ OV9655_RED, 0x80 },
	{ OV9655_VREF, 0x1b },
	{ OV9655_COM1, 0x03 },
	{ OV9655_COM5, 0x61 },
	{ OV9655_COM6, 0x40 },	/* manually update window size and timing */
	{ OV9655_CLKRC, 0x01 }, //was 0x03
	{ OV9655_COM7, 0x02 },
	{ OV9655_COM8, 0xe7 },
	{ OV9655_COM9, 0x2a },
	{ OV9655_COM10, 0x08 },
	{ OV9655_REG16, 0x24 },
	{ OV9655_HSTART, 0x1d },
	{ OV9655_HSTOP, 0xbd },
	{ OV9655_VSTART, 0x01 },
	{ OV9655_VSTOP, 0x81 },
	{ OV9655_MVFP, 0x00 },
	{ OV9655_AEW, 0x3c },
	{ OV9655_AEB, 0x36 },
	{ OV9655_VPT, 0x72 },
	{ OV9655_BBIAS, 0x08 },
	{ OV9655_GBBIAS, 0x08 },
	{ OV9655_PREGAIN, 0x15 },
	{ OV9655_EXHCH, 0x00 },
	{ OV9655_EXHCL, 0x00 },
	{ OV9655_RBIAS, 0x08 },
	{ OV9655_HREF, 0x3f },
	{ OV9655_CHLF, 0x00 },
	{ OV9655_AREF2, 0x00 },
	{ OV9655_ADC2, 0x72 },
	{ OV9655_AREF4, 0x57 },
	{ OV9655_TSLB, 0x80 },
	{ OV9655_COM11, 0x05 }, //was 0xcc // was 0xa4; 0x05 disable night mode
	{ OV9655_COM13, 0x99 },
	{ OV9655_COM14, 0x0c },
	{ OV9655_EDGE, 0x82 }, // was 0xc1
	{ OV9655_COM15, 0xc0 },
	{ OV9655_COM16, 0x00 },
	{ OV9655_COM17, 0xc1 },
	/*
	{ 0x43, 0x0a },
	{ 0x44, 0xf0 },
	{ 0x45, 0x46 },
	{ 0x46, 0x62 },
	{ 0x47, 0x2a },
	{ 0x48, 0x3c },
	{ 0x4a, 0xfc },
	{ 0x4b, 0xfc },
	{ 0x4c, 0x7f },
	{ 0x4d, 0x7f },
	{ 0x4e, 0x7f },
	 */
	{ OV9655_AWBOP1, 0x85 },
	{ OV9655_AWBOP2, 0xa9 },
	{ OV9655_AWBOP3, 0x64 },
	{ OV9655_AWBOP4, 0x84 },
	{ OV9655_AWBOP5, 0x53 },
	{ OV9655_AWBOP6, 0x0e },
	{ OV9655_BLMT, 0xf0 },
	{ OV9655_RLMT, 0xf0 },
	{ OV9655_GLMT, 0xf0 },
	{ OV9655_LCC1, 0x00 },
	{ OV9655_LCC2, 0x00 },
	{ OV9655_LCC3, 0x02 },
	{ OV9655_DBLV, 0xca }, //was 0xca // was 0xda
	/*
	{ 0x6c, 0x04 },
	{ 0x6d, 0x55 },
	{ 0x6e, 0x00 },
	{ 0x6f, 0x9d },
	 */
	{ OV9655_DNSTH, 0x21 },
	/*
	{ 0x71, 0x78 },
	{ 0x77, 0x02 },
	 */
	{ OV9655_SLOP, 0x12 },
	{ OV9655_GAM1, 0x08 },
	/*	{ OV9655_GAM2, 0x15 },
	 { OV9655_GAM3, 0x24 },
	 { OV9655_GAM4, 0x45 },
	 { OV9655_GAM5, 0x55 },
	 { OV9655_GAM6, 0x6a },
	 { OV9655_GAM7, 0x78 },
	 { OV9655_GAM8, 0x87 },
	 { OV9655_GAM9, 0x96 },
	 { OV9655_GAM10, 0xa3 },
	 { OV9655_GAM11, 0xb4 },
	 */	{ OV9655_GAM2, 0x16 },
	{ OV9655_GAM3, 0x30 },
	{ OV9655_GAM4, 0x5e },
	{ OV9655_GAM5, 0x72 },
	{ OV9655_GAM6, 0x82 },
	{ OV9655_GAM7, 0x8e },
	{ OV9655_GAM8, 0x9a },
	{ OV9655_GAM9, 0xa4 },
	{ OV9655_GAM10, 0xac },
	{ OV9655_GAM11, 0xb8 },
	{ OV9655_GAM12, 0xc3 },
	{ OV9655_GAM13, 0xd6 },
	{ OV9655_GAM14, 0xe6 },
	{ OV9655_GAM15, 0xf2 },
	/*{ 0x8a, 0x03 },
	{ 0x90, 0x7d },
	{ 0x91, 0x7b },
	 */
	{ OV9655_LCC6, 0x03 },
	/*{ 0x9f, 0x7a },
	{ 0xa0, 0x79 },*/
	{ OV9655_AECH, 0x40 },
	{ OV9655_COM21, 0x50 },
	/*{ 0xa5,0x68 },*/
	{ OV9655_GREEN, 0x4a },
	{ OV9655_REFA8, 0xc1 },
	{ OV9655_REFA9, 0xef },
	/*{ 0xaa, 0x92 },
	{ 0xab, 0x04 },*/
	{ OV9655_BLC1, 0x80 },
	{ OV9655_BLC2, 0x80 },
	{ OV9655_BLC3, 0x80 },
	{ OV9655_BLC4, 0x80 },
	{ OV9655_BLC7, 0xf2 },
	{ OV9655_BLC8, 0x20 },
	{ OV9655_CTRLB4, 0x20 },
	/*{ 0xb5, 0x00 },
	{ 0xb6, 0xaf },
	{ 0xbb, 0xae },*/
	{ OV9655_ADBOFF, 0x7f },
	{ OV9655_ADROFF, 0x7f },
	{ OV9655_ADGBOFF, 0x7f },
	{ OV9655_ADGROFF, 0x7f },
	/*{ 0xc1, 0xc0 },
	{ 0xc2, 0x01 },
	{ 0xc3, 0x4e },
	{ 0xc6, 0x85 },*/
	{ OV9655_COM24, 0x80 },
	/*{ 0xc9, 0xe0 },
	{ 0xca, 0xe8 },
	{ 0xcb, 0xf0 },
	{ 0xcc, 0xd8 },
	{ 0xcd, 0x93 },*/
	/* without VarioPixel */
	{ OV9655_AREF1, 0x3d },
	{ OV9655_AREF3, 0x34 },
	{ OV9655_LCC4, 0x16 },
	{ OV9655_LCC5, 0x01 },
	/*{ 0x69, 0x02 },*/
	{ OV9655_COM19, 0x0d },
	{ OV9655_COM20, 0x03 },
	{ OV9655_LCC7, 0x04 },
	/*{ 0xc0, 0xe2 },*/
	{ OV9655_BD50MAX, 0x05 },
	{ OV9655_BD50, 0x9d },
	{ OV9655_BD60, 0x83 },
	{ OV9655_BD60MAX, 0x07 },
	/*{ 0x76, 0x01 },*/
};

//// CHECKME: how can this switch to VGA?

/* Register values for VGA format */
static const struct ov9655_reg ov9655_vga[] = {
	{ OV9655_AREF3, 0xfa },
/*	{ 0x69, 0x0a }, */
	{ OV9655_COM19, 0x89 },
/*	{ 0xc0, 0xaa }, */
};

/* Register values for YUV format */
static const struct ov9655_reg ov9655_yuv_regs[] = {
	{ OV9655_MTX1, 0x80 },
	{ OV9655_MTX2, 0x80 },
	{ OV9655_MTX3, 0x00 },
	{ OV9655_MTX4, 0x22 },
	{ OV9655_MTX5, 0x5e },
	{ OV9655_MTX6, 0x80 },
	{ OV9655_MTXS, 0x1e },
};

/* Register values for RGB format */
static const struct ov9655_reg ov9655_rgb_regs[] = {
	{ OV9655_MTX1, 0x98 },
	{ OV9655_MTX2, 0x98 },
	{ OV9655_MTX3, 0x00 },
	{ OV9655_MTX4, 0x28,},
	{ OV9655_MTX5, 0x70 },
	{ OV9655_MTX6, 0x98 },
	{ OV9655_MTXS, 0x1a },
};


/* List of image formats supported by ov9655
 * Currently we are using 8 bit mode only, but can be
 * extended to 10/20 bit mode.
 */
static const struct v4l2_fmtdesc ov9655_fmt_list[] = {
	{
	 .index = 0,
	 .type = V4L2_BUF_TYPE_VIDEO_CAPTURE,
	 .flags = 0,
	 .description = "8-bit UYVY 4:2:2 Format",
	 .pixelformat = V4L2_PIX_FMT_UYVY,
	},
};

/*
 * Supported standards -
 *
 * Currently supports two standards only, need to add support for rest of the
 * modes, like SECAM, etc...
 */
#define OV9655_IMAGE_STD_VGA			(0x01)
#define OV9655_IMAGE_STD_QVGA			(0x02)
#define OV9655_IMAGE_STD_INVALID		(0xFF)

static struct ov9655_std_info ov9655_std_list[] = {
	/* Standard: STD_NTSC_MJ */
	[OV9655_STD_VGA] = {
	 .width = VGA_NUM_ACTIVE_PIXELS,
	 .height = VGA_NUM_ACTIVE_LINES,
	 .video_std = OV9655_IMAGE_STD_VGA,
	 .standard = {
		      .index = 0,
		      .id = OV9655_IMAGE_STD_VGA,
		      .name = "VGA",
		      .frameperiod = {1001, 30000},
		      .framelines = 480
		     },
	/* Standard: STD_PAL_BDGHIN */
	},
	[OV9655_STD_QVGA] = {
	 .width = QVGA_NUM_ACTIVE_PIXELS,
	 .height = QVGA_NUM_ACTIVE_LINES,
	 .video_std = OV9655_IMAGE_STD_QVGA,
	 .standard = {
		      .index = 1,
		      .id = OV9655_IMAGE_STD_QVGA,
		      .name = "QVGA",
		      .frameperiod = {1001, 30000},
		      .framelines = 320
		     },
	},
	/* Standard: need to add for additional standard */
};


/*
 * Control structure for Auto Gain
 *     This is temporary data, will get replaced once
 *     v4l2_ctrl_query_fill supports it.
 */
static const struct v4l2_queryctrl ov9655_autogain_ctrl = {
	.id = V4L2_CID_AUTOGAIN,
	.name = "Gain, Automatic",
	.type = V4L2_CTRL_TYPE_BOOLEAN,
	.minimum = 0,
	.maximum = 1,
	.step = 1,
	.default_value = 1,
};

const struct v4l2_fract ov9655_frameintervals[] = {
	{  .numerator = 1, .denominator = 10 }
};

#ifdef OLD_MATERIAL

static int ov9655_read_reg(struct i2c_client *client, unsigned short reg)
{
	int err = 0;
	struct i2c_msg msg[1];
	unsigned char data[2];
	unsigned short val = 0;

	if (!client->adapter) {
		err = -ENODEV;
		return err;
	}else {
		/* TODO: addr should be set up where else client->addr */
		msg->addr = OV9655_I2C_ADDR;
		msg->flags = 0;
		msg->len = I2C_TWO_BYTE_TRANSFER;
		msg->buf = data;
		data[0] = (reg & I2C_TXRX_DATA_MASK_UPPER) >>
			    I2C_TXRX_DATA_SHIFT;
		data[1] = (reg & I2C_TXRX_DATA_MASK);
		err = i2c_transfer(client->adapter, msg, 1);
		if (err >= 0) {
			msg->flags = I2C_M_RD;
			msg->len = I2C_TWO_BYTE_TRANSFER;	/* 2 byte read */
			err = i2c_transfer(client->adapter, msg, 1);
			if (err >= 0) {
				val = ((data[0] & I2C_TXRX_DATA_MASK)
					<< I2C_TXRX_DATA_SHIFT)
				    | (data[1] & I2C_TXRX_DATA_MASK);
			}
		}
	}
	return (int)(0x0000ffff & val);
}


static int ov9655_write_reg(struct i2c_client *client, unsigned short reg, unsigned short val)
{
	int err = 0;
	int trycnt = 0;

	struct i2c_msg msg[1];
	unsigned char data[4];
	err = -1;

	v4l_dbg(1, debug, client,
		 "ov9655_write_reg reg=0x%x, val=0x%x\n",
		 reg,val);

	while ((err < 0) && (trycnt < I2C_RETRY_COUNT)) {
		trycnt++;
		if (!client->adapter) {
			err = -ENODEV;
		} else {
			/* TODO:addr should be set up where else client->addr */
			msg->addr = OV9655_I2C_ADDR;
			msg->flags = 0;
			msg->len = I2C_FOUR_BYTE_TRANSFER;
			msg->buf = data;
			data[0] = (reg & I2C_TXRX_DATA_MASK_UPPER) >>
			    I2C_TXRX_DATA_SHIFT;
			data[1] = (reg & I2C_TXRX_DATA_MASK);
			data[2] = (val & I2C_TXRX_DATA_MASK_UPPER) >>
			    I2C_TXRX_DATA_SHIFT;
			data[3] = (val & I2C_TXRX_DATA_MASK);
			err = i2c_transfer(client->adapter, msg, 1);
		}
	}
	if (err < 0)
		printk(KERN_INFO "\n I2C write failed");

	return err;
}

/*
 * ov9655_write_regs : Initializes a list of registers
 *		if token is TOK_TERM, then entire write operation terminates
 *		if token is TOK_DELAY, then a delay of 'val' msec is introduced
 *		if token is TOK_SKIP, then the register write is skipped
 *		if token is TOK_WRITE, then the register write is performed
 *
 * reglist - list of registers to be written
 * Returns zero if successful, or non-zero otherwise.
 */
static int ov9655_write_regs(struct i2c_client *client,
			      const struct ov9655_reg reglist[])
{
	int err;
	const struct ov9655_reg *next = reglist;

	for (; next->token != TOK_TERM; next++) {
		if (next->token == TOK_DELAY) {
			msleep(next->val);
			continue;
		}

		if (next->token == TOK_SKIP)
			continue;

		err = ov9655_write_reg(client, next->reg, next->val);
		if (err < 0) {
			v4l_err(client, "Write failed. Err[%d]\n", err);
			return err;
		}
	}
	return 0;
}

#endif

/*
 * ov9655_get_current_std:
 * Returns the current standard
 */
static enum ov9655_std ov9655_get_current_std(struct ov9655_decoder
						*decoder)
{
	// should read from chip!!!
	return OV9655_STD_VGA;
}

/*
 * Configure the ov9655 with the current register settings
 * Returns zero if successful, or non-zero otherwise.
 */
static int ov9655_configure(struct ov9655_decoder *decoder)
{
	int err;
	/* common register initialization */
	err =
	    ov9655_write_regs(decoder->client, ov9655_init_regs, ARRAY_SIZE(ov9655_init_regs));
	if (err)
		return err;
	/* and configure as default to VGA mode */
	err =
	ov9655_write_regs(decoder->client, ov9655_vga, ARRAY_SIZE(ov9655_vga));
	if (err)
		return err;
	
	return 0;
}

#if not
/*
 * Configure the OV9655 as default to VGA mode
 * Returns zero if successful, or non-zero otherwise.
 */
static int ov9655_vga_mode(struct ov9655_decoder *decoder)
{
	int err;

#if 0
	err =
	    ov9655_write_regs(decoder->client, ov9655_vga_reg);
	if (err)
		return err;
#endif
	return 0;
}
#endif

/*
 * ioctl_enum_framesizes - V4L2 sensor if handler for vidioc_int_enum_framesizes
 * @s: pointer to standard V4L2 device structure
 * @frms: pointer to standard V4L2 framesizes enumeration structure
 *
 * Returns possible framesizes depending on choosen pixel format
 */
static int ioctl_enum_framesizes(struct v4l2_int_device *s,
					struct v4l2_frmsizeenum *frms)
{
	struct ov9655_decoder *decoder = s->priv;
	int ifmt;

	for (ifmt = 0; ifmt < decoder->num_fmts; ifmt++) {
		if (frms->pixel_format == decoder->fmt_list[ifmt].pixelformat)
			break;
	}
	/* Is requested pixelformat not found on sensor? */
	if (ifmt == decoder->num_fmts)
		return -EINVAL;

	/* Do we already reached all discrete framesizes? */
	if (frms->index >= decoder->num_stds)
		return -EINVAL;

	frms->type = V4L2_FRMSIZE_TYPE_DISCRETE;
	frms->discrete.width = decoder->std_list[frms->index].width;
	frms->discrete.height = decoder->std_list[frms->index].height;

	return 0;

}

static int ioctl_enum_frameintervals(struct v4l2_int_device *s,
					struct v4l2_frmivalenum *frmi)
{
	struct ov9655_decoder *decoder = s->priv;
	int ifmt;

	if (frmi->index >= 1)
		return -EINVAL;

	for (ifmt = 0; ifmt < decoder->num_fmts; ifmt++) {
		if (frmi->pixel_format == decoder->fmt_list[ifmt].pixelformat)
			break;
	}
	/* Is requested pixelformat not found on sensor? */
	if (ifmt == decoder->num_fmts)
		return -EINVAL;

	if (frmi->index >= ARRAY_SIZE(ov9655_frameintervals))
		return -EINVAL;

	frmi->type = V4L2_FRMSIZE_TYPE_DISCRETE;
	frmi->discrete.numerator =
		ov9655_frameintervals[frmi->index].numerator;
	frmi->discrete.denominator =
		ov9655_frameintervals[frmi->index].denominator;
	return 0;
}


/*
 * Detect if an ov9655 is present, and if so which revision.
 * A device is considered to be detected if the chip ID (LSB and MSB)
 * registers match the expected values.
 * Any value of the rom version register is accepted.
 * Returns ENODEV error number if no device is detected, or zero
 * if a device is detected.
 */
static int ov9655_detect(struct ov9655_decoder *decoder)
{
	unsigned short val=0;
	printk("ov9655_detect\n");

	
	/* Read chip manufacturer register */
	val = (ov9655_read(decoder->client, OV9655_MIDH) << 8) + ov9655_read(decoder->client, OV9655_MIDL);

	if (val < 0) {
		v4l_dbg(1, debug, decoder->client, "Strange error reading sensor"
				" manufacturer\n");
		return -ENODEV;
	}
	v4l_dbg(1, debug, decoder->client, "chip manufacturer detected 0x%x\n", val);
	
	if (0x7fa2 != val) {
		v4l_dbg(1, debug, decoder->client, "No OmniVision sensor detected, "
				" manufacturer register read 0x%x\n", val);
		return -ENODEV;
	}
	
	val = ov9655_read(decoder->client, OV9655_PID);
	v4l_dbg(1, debug, decoder->client, "chip pid detected 0x%x\n", val);

	if (OV9655_CHIP_PID != val) {
		/* We didn't read the values we expected, so this must not be
		 * OV9655.
		 */
		v4l_err(decoder->client,
				"chip pid mismatch read 0x%x, expecting 0x%x\n", val,
				OV9655_CHIP_PID);
		return -ENODEV;
	}

	val = ov9655_read(decoder->client, OV9655_VER);
	v4l_dbg(1, debug, decoder->client, "chip ver detected 0x%x\n", val);

	if (OV9655_CHIP_VER4 == val)
		decoder->ver = 4;
	else if (OV9655_CHIP_VER5 == val)
		decoder->ver = 5;
	else 		{
		/* We didn't read the values we expected, so this must not be
		 * OV9655.
		 */
		v4l_err(decoder->client,
			"chip id mismatch read 0x%x, expecting 0x%x or 0x%x\n", val,
				OV9655_CHIP_VER4, OV9655_CHIP_VER5);
		return -ENODEV;
	}

	decoder->state = STATE_DETECTED;

	v4l_info(decoder->client,
			"%s found at 0x%x (%s)\n", decoder->client->name,
			decoder->client->addr << 1,
			decoder->client->adapter->name);

	return 0;
}

/*
 * Following are decoder interface functions implemented by
 * ov9655 decoder driver.
 */

/*
 * ioctl_querystd - V4L2 decoder interface handler for VIDIOC_QUERYSTD ioctl
 * @s: pointer to standard V4L2 device structure
 * @std_id: standard V4L2 std_id ioctl enum
 *
 * Returns the current standard detected by ov9655. If no active input is
 * detected, returns -EINVAL
 */
static int ioctl_querystd(struct v4l2_int_device *s, v4l2_std_id *std_id)
{
	struct ov9655_decoder *decoder = s->priv;
	enum ov9655_std current_std;

	if (std_id == NULL)
		return -EINVAL;

	/* get the current standard */
	current_std = ov9655_get_current_std(decoder);
	if (current_std == OV9655_IMAGE_STD_INVALID)
		return -EINVAL;

	decoder->current_std = current_std;
	*std_id = decoder->std_list[current_std].standard.id;

	v4l_dbg(1, debug, decoder->client, "Current STD: %s\n",
			decoder->std_list[current_std].standard.name);
	return 0;
}

/*
 * ioctl_s_std - V4L2 decoder interface handler for VIDIOC_S_STD ioctl
 * @s: pointer to standard V4L2 device structure
 * @std_id: standard V4L2 v4l2_std_id ioctl enum
 *
 * If std_id is supported, sets the requested standard. Otherwise, returns
 * -EINVAL
 */
static int ioctl_s_std(struct v4l2_int_device *s, v4l2_std_id *std_id)
{
	struct ov9655_decoder *decoder = s->priv;
	int err, i;

	if (std_id == NULL)
		return -EINVAL;

	for (i = 0; i < decoder->num_stds; i++)
		if (*std_id & decoder->std_list[i].standard.id)
			break;

	if ((i == decoder->num_stds) || (i == OV9655_STD_INVALID))
		return -EINVAL;
#if not
	/* fixme - this tries to set the standard in the mt9 camera */
	err = ov9655_write_reg(decoder->client, REG_VIDEO_STD,
				decoder->std_list[i].video_std);
	if (err)
		return err;

	decoder->current_std = i;
	/* and here we update the register value written if we do a reset... */
	ov9655_reg_list[REG_VIDEO_STD].val = decoder->std_list[i].video_std;
#endif
	decoder->current_std = i;

	v4l_dbg(1, debug, decoder->client, "Standard set to: %s\n",
			decoder->std_list[i].standard.name);
	return 0;
}

/*
 * ioctl_s_routing - V4L2 decoder interface handler for VIDIOC_S_INPUT ioctl
 * @s: pointer to standard V4L2 device structure
 * @index: number of the input
 *
 * If index is valid, selects the requested input. Otherwise, returns -EINVAL if
 * the input is not supported or there is no active signal present in the
 * selected input.
 */
static int ioctl_s_routing(struct v4l2_int_device *s,
				struct v4l2_routing *route)
{
	return 0;
}

/*
 * ioctl_queryctrl - V4L2 decoder interface handler for VIDIOC_QUERYCTRL ioctl
 * @s: pointer to standard V4L2 device structure
 * @qctrl: standard V4L2 v4l2_queryctrl structure
 *
 * If the requested control is supported, returns the control information.
 * Otherwise, returns -EINVAL if the control is not supported.
 */
static int
ioctl_queryctrl(struct v4l2_int_device *s, struct v4l2_queryctrl *qctrl)
{
	struct ov9655_decoder *decoder = s->priv;
	int err = -EINVAL;

	if (qctrl == NULL)
		return err;
	switch (qctrl->id) {
#if not
			// FIXME: adapt to OV9655 controls
	case V4L2_CID_BRIGHTNESS:
		/* Brightness supported is same as standard one (0-255),
		 * so make use of standard API provided.
		 */
		err = v4l2_ctrl_query_fill(qctrl, 0, 255, 1, 128);
		break;
	case V4L2_CID_CONTRAST:
	case V4L2_CID_SATURATION:
		/* Saturation and Contrast supported is -
		 *	Contrast: 0 - 255 (Default - 128)
		 *	Saturation: 0 - 255 (Default - 128)
		 */
		err = v4l2_ctrl_query_fill(qctrl, 0, 255, 1, 128);
		break;
	case V4L2_CID_HUE:
		/* Hue Supported is -
		 *	Hue - -180 - +180 (Default - 0, Step - +180)
		 */
		err = v4l2_ctrl_query_fill(qctrl, -180, 180, 180, 0);
		break;
	case V4L2_CID_AUTOGAIN:
		/* Autogain is either 0 or 1*/
		memcpy(qctrl, &ov9655_autogain_ctrl,
				sizeof(struct v4l2_queryctrl));
		err = 0;
		break;
#endif
	default:
		v4l_err(decoder->client,
			"invalid control id %d\n", qctrl->id);
		return err;
	}
	v4l_dbg(1, debug, decoder->client,
			"Query Control: %s : Min - %d, Max - %d, Def - %d\n",
			qctrl->name,
			qctrl->minimum,
			qctrl->maximum,
			qctrl->default_value);

	return err;
}

/*
 * ioctl_g_ctrl - V4L2 decoder interface handler for VIDIOC_G_CTRL ioctl
 * @s: pointer to standard V4L2 device structure
 * @ctrl: pointer to v4l2_control structure
 *
 * If the requested control is supported, returns the control's current
 * value from the decoder. Otherwise, returns -EINVAL if the control is not
 * supported.
 */
static int
ioctl_g_ctrl(struct v4l2_int_device *s, struct v4l2_control *ctrl)
{
	struct ov9655_decoder *decoder = s->priv;

	if (ctrl == NULL)
		return -EINVAL;
	switch (ctrl->id) {
#if not
			// FIXME: adapt to OV9655 controls
	case V4L2_CID_BRIGHTNESS:
		ctrl->value = ov9655_reg_list[REG_BRIGHTNESS].val;
		break;
	case V4L2_CID_CONTRAST:
		ctrl->value = ov9655_reg_list[REG_CONTRAST].val;
		break;
	case V4L2_CID_SATURATION:
		ctrl->value = ov9655_reg_list[REG_SATURATION].val;
		break;
	case V4L2_CID_HUE:
		ctrl->value = ov9655_reg_list[REG_HUE].val;
		if (ctrl->value == 0x7F)
			ctrl->value = 180;
		else if (ctrl->value == 0x80)
			ctrl->value = -180;
		else
			ctrl->value = 0;

		break;
	case V4L2_CID_AUTOGAIN:
		ctrl->value = ov9655_reg_list[REG_AFE_GAIN_CTRL].val;
		if ((ctrl->value & 0x3) == 3)
			ctrl->value = 1;
		else
			ctrl->value = 0;

		break;
#endif
	default:
		v4l_err(decoder->client,
			"invalid control id %d\n", ctrl->id);
		return -EINVAL;
	}
	v4l_dbg(1, debug, decoder->client,
			"Get Control: ID - %d - %d\n",
			ctrl->id, ctrl->value);
	return 0;
}

/*
 * ioctl_s_ctrl - V4L2 decoder interface handler for VIDIOC_S_CTRL ioctl
 * @s: pointer to standard V4L2 device structure
 * @ctrl: pointer to v4l2_control structure
 *
 * If the requested control is supported, sets the control's current
 * value in HW. Otherwise, returns -EINVAL if the control is not supported.
 */
static int
ioctl_s_ctrl(struct v4l2_int_device *s, struct v4l2_control *ctrl)
{
	struct ov9655_decoder *decoder = s->priv;
	int err = -EINVAL, value;

	if (ctrl == NULL)
		return err;

	value = (__s32) ctrl->value;
#if not
	// FIXME: adapt to OV9655 controls
	switch (ctrl->id) {
	case V4L2_CID_BRIGHTNESS:
		if (ctrl->value < 0 || ctrl->value > 255) {
			v4l_err(decoder->client,
					"invalid brightness setting %d\n",
					ctrl->value);
			return -ERANGE;
		}
		err = ov9655_write_reg(decoder->client, REG_BRIGHTNESS,
				value);
		if (err)
			return err;
		ov9655_reg_list[REG_BRIGHTNESS].val = value;
		break;
	case V4L2_CID_CONTRAST:
		if (ctrl->value < 0 || ctrl->value > 255) {
			v4l_err(decoder->client,
					"invalid contrast setting %d\n",
					ctrl->value);
			return -ERANGE;
		}
		err = ov9655_write_reg(decoder->client, REG_CONTRAST,
				value);
		if (err)
			return err;
		ov9655_reg_list[REG_CONTRAST].val = value;
		break;
	case V4L2_CID_SATURATION:
		if (ctrl->value < 0 || ctrl->value > 255) {
			v4l_err(decoder->client,
					"invalid saturation setting %d\n",
					ctrl->value);
			return -ERANGE;
		}
		err = ov9655_write_reg(decoder->client, REG_SATURATION,
				value);
		if (err)
			return err;
		ov9655_reg_list[REG_SATURATION].val = value;
		break;
	case V4L2_CID_HUE:
		if (value == 180)
			value = 0x7F;
		else if (value == -180)
			value = 0x80;
		else if (value == 0)
			value = 0;
		else {
			v4l_err(decoder->client,
					"invalid hue setting %d\n",
					ctrl->value);
			return -ERANGE;
		}
		err = ov9655_write_reg(decoder->client, REG_HUE,
				value);
		if (err)
			return err;
		ov9655_reg_list[REG_HUE].val = value;
		break;
	case V4L2_CID_AUTOGAIN:
		if (value == 1)
			value = 0x0F;
		else if (value == 0)
			value = 0x0C;
		else {
			v4l_err(decoder->client,
					"invalid auto gain setting %d\n",
					ctrl->value);
			return -ERANGE;
		}
		err = ov9655_write_reg(decoder->client, REG_AFE_GAIN_CTRL,
				value);
		if (err)
			return err;
		ov9655_reg_list[REG_AFE_GAIN_CTRL].val = value;
		break;
	default:
		v4l_err(decoder->client,
			"invalid control id %d\n", ctrl->id);
		return err;
	}
#endif
	v4l_dbg(1, debug, decoder->client,
			"Set Control: ID - %d - %d\n",
			ctrl->id, ctrl->value);

	return err;
}

/*
 * ioctl_enum_fmt_cap - Implement the CAPTURE buffer VIDIOC_ENUM_FMT ioctl
 * @s: pointer to standard V4L2 device structure
 * @fmt: standard V4L2 VIDIOC_ENUM_FMT ioctl structure
 *
 * Implement the VIDIOC_ENUM_FMT ioctl to enumerate supported formats
 */
static int
ioctl_enum_fmt_cap(struct v4l2_int_device *s, struct v4l2_fmtdesc *fmt)
{
	struct ov9655_decoder *decoder = s->priv;
	int index;

	if (fmt == NULL)
		return -EINVAL;

	index = fmt->index;
	if ((index >= decoder->num_fmts) || (index < 0))
		return -EINVAL;	/* Index out of bound */

	if (fmt->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;	/* only capture is supported */

	memcpy(fmt, &decoder->fmt_list[index],
		sizeof(struct v4l2_fmtdesc));

	v4l_dbg(1, debug, decoder->client,
			"Current FMT: index - %d (%s)\n",
			decoder->fmt_list[index].index,
			decoder->fmt_list[index].description);
	return 0;
}

/*
 * ioctl_try_fmt_cap - Implement the CAPTURE buffer VIDIOC_TRY_FMT ioctl
 * @s: pointer to standard V4L2 device structure
 * @f: pointer to standard V4L2 VIDIOC_TRY_FMT ioctl structure
 *
 * Implement the VIDIOC_TRY_FMT ioctl for the CAPTURE buffer type. This
 * ioctl is used to negotiate the image capture size and pixel format
 * without actually making it take effect.
 */
static int
ioctl_try_fmt_cap(struct v4l2_int_device *s, struct v4l2_format *f)
{
	struct ov9655_decoder *decoder = s->priv;
	int ifmt;
	struct v4l2_pix_format *pix;
	enum ov9655_std current_std;

	if (f == NULL)
		return -EINVAL;

	if (f->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		f->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

	pix = &f->fmt.pix;

	/* Calculate height and width based on current standard */
	current_std = ov9655_get_current_std(decoder);
	if (current_std == OV9655_STD_INVALID)
		return -EINVAL;

	decoder->current_std = current_std;
	pix->width = decoder->std_list[current_std].width;
	pix->height = decoder->std_list[current_std].height;

	for (ifmt = 0; ifmt < decoder->num_fmts; ifmt++) {
		if (pix->pixelformat ==
			decoder->fmt_list[ifmt].pixelformat)
			break;
	}
	if (ifmt == decoder->num_fmts)
		ifmt = 0;	/* None of the format matched, select default */
	pix->pixelformat = decoder->fmt_list[ifmt].pixelformat;

	pix->field = V4L2_FIELD_NONE;
	pix->bytesperline = pix->width * 2;
	pix->sizeimage = pix->bytesperline * pix->height;
	pix->colorspace = V4L2_COLORSPACE_SMPTE170M;
	pix->priv = 0;

	v4l_dbg(1, debug, decoder->client,
			"Try FMT: pixelformat - %s, bytesperline - %d, "
			"Width - %d, Height - %d\n",
			decoder->fmt_list[ifmt].description, pix->bytesperline,
			pix->width, pix->height);
	return 0;
}

/*
 * ioctl_s_fmt_cap - V4L2 decoder interface handler for VIDIOC_S_FMT ioctl
 * @s: pointer to standard V4L2 device structure
 * @f: pointer to standard V4L2 VIDIOC_S_FMT ioctl structure
 *
 * If the requested format is supported, configures the HW to use that
 * format, returns error code if format not supported or HW can't be
 * correctly configured.
 */
static int
ioctl_s_fmt_cap(struct v4l2_int_device *s, struct v4l2_format *f)
{
	struct ov9655_decoder *decoder = s->priv;
	struct v4l2_pix_format *pix;
	int rval;

	if (f == NULL)
		return -EINVAL;

	if (f->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;	/* only capture is supported */

	pix = &f->fmt.pix;
	rval = ioctl_try_fmt_cap(s, f);
	if (rval)
		return rval;

	decoder->pix = *pix;

	return rval;
}

/*
 * ioctl_g_fmt_cap - V4L2 decoder interface handler for ioctl_g_fmt_cap
 * @s: pointer to standard V4L2 device structure
 * @f: pointer to standard V4L2 v4l2_format structure
 *
 * Returns the decoder's current pixel format in the v4l2_format
 * parameter.
 */
static int
ioctl_g_fmt_cap(struct v4l2_int_device *s, struct v4l2_format *f)
{
	struct ov9655_decoder *decoder = s->priv;

	if (f == NULL)
		return -EINVAL;

	if (f->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;	/* only capture is supported */

	f->fmt.pix = decoder->pix;

	v4l_dbg(1, debug, decoder->client,
			"Current FMT: bytesperline - %d, "
			"Width - %d, Height - %d\n",
			decoder->pix.bytesperline,
			decoder->pix.width, decoder->pix.height);
	return 0;
}

/*
 * ioctl_g_parm - V4L2 decoder interface handler for VIDIOC_G_PARM ioctl
 * @s: pointer to standard V4L2 device structure
 * @a: pointer to standard V4L2 VIDIOC_G_PARM ioctl structure
 *
 * Returns the decoder's video CAPTURE parameters.
 */
static int
ioctl_g_parm(struct v4l2_int_device *s, struct v4l2_streamparm *a)
{
	struct ov9655_decoder *decoder = s->priv;
	struct v4l2_captureparm *cparm;
	enum ov9655_std current_std;

	if (a == NULL)
		return -EINVAL;

	if (a->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;	/* only capture is supported */

	memset(a, 0, sizeof(*a));
	a->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

	/* get the current standard */
	current_std = ov9655_get_current_std(decoder);
	if (current_std == OV9655_STD_INVALID)
		return -EINVAL;

	decoder->current_std = current_std;

	cparm = &a->parm.capture;
	cparm->capability = V4L2_CAP_TIMEPERFRAME;
	cparm->timeperframe =
		decoder->std_list[current_std].standard.frameperiod;

	return 0;
}

/*
 * ioctl_s_parm - V4L2 decoder interface handler for VIDIOC_S_PARM ioctl
 * @s: pointer to standard V4L2 device structure
 * @a: pointer to standard V4L2 VIDIOC_S_PARM ioctl structure
 *
 * Configures the decoder to use the input parameters, if possible. If
 * not possible, returns the appropriate error code.
 */
static int
ioctl_s_parm(struct v4l2_int_device *s, struct v4l2_streamparm *a)
{
	struct ov9655_decoder *decoder = s->priv;
	struct v4l2_fract *timeperframe;
	enum ov9655_std current_std;

	if (a == NULL)
		return -EINVAL;

	if (a->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;	/* only capture is supported */

	timeperframe = &a->parm.capture.timeperframe;

	/* get the current standard */
	current_std = ov9655_get_current_std(decoder);
	if (current_std == OV9655_STD_INVALID)
		return -EINVAL;

	decoder->current_std = current_std;

	*timeperframe =
	    decoder->std_list[current_std].standard.frameperiod;

	return 0;
}

/*
 * ioctl_g_ifparm - V4L2 decoder interface handler for vidioc_int_g_ifparm_num
 * @s: pointer to standard V4L2 device structure
 * @p: pointer to standard V4L2 vidioc_int_g_ifparm_num ioctl structure
 *
 * Gets slave interface parameters.
 * Calculates the required xclk value to support the requested
 * clock parameters in p. This value is returned in the p
 * parameter.
 */
static int ioctl_g_ifparm(struct v4l2_int_device *s, struct v4l2_ifparm *p)
{
	struct ov9655_decoder *decoder = s->priv;
	int rval;

	if (p == NULL)
		return -EINVAL;

	if (NULL == decoder->pdata->ifparm)
		return -EINVAL;

	rval = decoder->pdata->ifparm(p);
	if (rval) {
		v4l_err(decoder->client, "g_ifparm.Err[%d]\n", rval);
		return rval;
	}

	p->u.bt656.clock_curr = 24000000; /* TODO:read clock rate from sensor */

	return 0;
}

/*
 * ioctl_g_priv - V4L2 decoder interface handler for vidioc_int_g_priv_num
 * @s: pointer to standard V4L2 device structure
 * @p: void pointer to hold decoder's private data address
 *
 * Returns device's (decoder's) private data area address in p parameter
 */
static int ioctl_g_priv(struct v4l2_int_device *s, void *p)
{
	struct ov9655_decoder *decoder = s->priv;

	if (NULL == decoder->pdata->priv_data_set)
		return -EINVAL;

	return decoder->pdata->priv_data_set(p);
}

/*
 * ioctl_s_power - V4L2 decoder interface handler for vidioc_int_s_power_num
 * @s: pointer to standard V4L2 device structure
 * @on: power state to which device is to be set
 *
 * Sets devices power state to requrested state, if possible.
 */
static int ioctl_s_power(struct v4l2_int_device *s, enum v4l2_power on)
{
	struct ov9655_decoder *decoder = s->priv;
	int err = 0;

	switch (on) {
	case V4L2_POWER_OFF:
		/* Power Down Sequence */
		/* TODO: FIXME: implement proper OFF and Standby code here */
#if 0
		err = ov9655_write_reg(decoder->client, REG_OPERATION_MODE,
				0x01);
#endif
		/* Disable mux for ov9655 data path */
		if (decoder->pdata->power_set)
			err |= decoder->pdata->power_set(s, on);
		decoder->state = STATE_NOT_DETECTED;
		break;

	case V4L2_POWER_STANDBY:
		if (decoder->pdata->power_set)
			err = decoder->pdata->power_set(s, on);
		break;

	case V4L2_POWER_ON:
		/* Enable mux for ov9655 data path */
		if (decoder->state == STATE_NOT_DETECTED) {

			if (decoder->pdata->power_set)
				err = decoder->pdata->power_set(s, on);

			/* Detect the sensor is not already detected */
			err |= ov9655_detect(decoder);
			if (err) {
				v4l_err(decoder->client,
						"Unable to detect decoder\n");
				return err;
			}
		}
		/* Only VGA mode for now */
		err = ov9655_configure(decoder);
//		err |= ov9655_vga_mode(decoder);
		break;

	default:
		err = -ENODEV;
		break;
	}

	return err;
}

/*
 * ioctl_init - V4L2 decoder interface handler for VIDIOC_INT_INIT
 * @s: pointer to standard V4L2 device structure
 *
 * Initialize the decoder device (calls ov9655_configure())
 */
static int ioctl_init(struct v4l2_int_device *s)
{
	struct ov9655_decoder *decoder = s->priv;
	int err = 0;

	err = ov9655_configure(decoder);
//	err |= ov9655_vga_mode(decoder);

	return err;
}

/*
 * ioctl_dev_exit - V4L2 decoder interface handler for vidioc_int_dev_exit_num
 * @s: pointer to standard V4L2 device structure
 *
 * Delinitialise the dev. at slave detach. The complement of ioctl_dev_init.
 */
static int ioctl_dev_exit(struct v4l2_int_device *s)
{
	return 0;
}

/*
 * ioctl_dev_init - V4L2 decoder interface handler for vidioc_int_dev_init_num
 * @s: pointer to standard V4L2 device structure
 *
 * Initialise the device when slave attaches to the master. Returns 0 if
 * ov9655 device could be found, otherwise returns appropriate error.
 */
static int ioctl_dev_init(struct v4l2_int_device *s)
{
	struct ov9655_decoder *decoder = s->priv;
	int err;

	err = ov9655_detect(decoder);
	if (err < 0) {
		v4l_err(decoder->client,
			"Unable to detect decoder\n");
		return err;
	}

	v4l_info(decoder->client,
		 "chip version 0x%.2x detected\n", decoder->ver);

	err = ov9655_configure(decoder);
//	err |= ov9655_vga_mode(decoder);

	return 0;
}

static struct v4l2_int_ioctl_desc ov9655_ioctl_desc[] = {
 {vidioc_int_dev_init_num, (v4l2_int_ioctl_func*) ioctl_dev_init},
	{vidioc_int_dev_exit_num, (v4l2_int_ioctl_func*) ioctl_dev_exit},
	{vidioc_int_s_power_num, (v4l2_int_ioctl_func*) ioctl_s_power},
	{vidioc_int_g_priv_num, (v4l2_int_ioctl_func*) ioctl_g_priv},
	{vidioc_int_g_ifparm_num, (v4l2_int_ioctl_func*) ioctl_g_ifparm},
	{vidioc_int_init_num, (v4l2_int_ioctl_func*) ioctl_init},
	{vidioc_int_enum_fmt_cap_num,
	 (v4l2_int_ioctl_func *) ioctl_enum_fmt_cap},
	{vidioc_int_try_fmt_cap_num,
	 (v4l2_int_ioctl_func *) ioctl_try_fmt_cap},
	{vidioc_int_g_fmt_cap_num,
	 (v4l2_int_ioctl_func *) ioctl_g_fmt_cap},
	{vidioc_int_s_fmt_cap_num,
	 (v4l2_int_ioctl_func *) ioctl_s_fmt_cap},
	{vidioc_int_g_parm_num, (v4l2_int_ioctl_func *) ioctl_g_parm},
	{vidioc_int_s_parm_num, (v4l2_int_ioctl_func *) ioctl_s_parm},
	{vidioc_int_queryctrl_num,
	 (v4l2_int_ioctl_func *) ioctl_queryctrl},
	{vidioc_int_g_ctrl_num, (v4l2_int_ioctl_func *) ioctl_g_ctrl},
	{vidioc_int_s_ctrl_num, (v4l2_int_ioctl_func *) ioctl_s_ctrl},
	{vidioc_int_querystd_num, (v4l2_int_ioctl_func *) ioctl_querystd},
	{vidioc_int_s_std_num, (v4l2_int_ioctl_func *) ioctl_s_std},
	{vidioc_int_s_video_routing_num,
		(v4l2_int_ioctl_func *) ioctl_s_routing},
	{vidioc_int_enum_framesizes_num,
		(v4l2_int_ioctl_func *)ioctl_enum_framesizes},
	{vidioc_int_enum_frameintervals_num,
		(v4l2_int_ioctl_func *)ioctl_enum_frameintervals},
};

static struct v4l2_int_slave ov9655_slave = {
	.ioctls = ov9655_ioctl_desc,
	.num_ioctls = ARRAY_SIZE(ov9655_ioctl_desc),
};

static struct ov9655_decoder ov9655_dev = {
	.state = STATE_NOT_DETECTED,

	.fmt_list = ov9655_fmt_list,
	.num_fmts = ARRAY_SIZE(ov9655_fmt_list),

	.pix = {		/* Default to 8-bit YUV 422 */
		.width = VGA_NUM_ACTIVE_PIXELS,
		.height = VGA_NUM_ACTIVE_LINES,
		.pixelformat = V4L2_PIX_FMT_UYVY,
		.field = V4L2_FIELD_NONE,
		.bytesperline = VGA_NUM_ACTIVE_PIXELS * 2,
		.sizeimage =
		VGA_NUM_ACTIVE_PIXELS * 2 * VGA_NUM_ACTIVE_LINES,
		.colorspace = V4L2_COLORSPACE_SMPTE170M,
		},

	.current_std = OV9655_STD_VGA,
	.std_list = ov9655_std_list,
	.num_stds = ARRAY_SIZE(ov9655_std_list),

};

static struct v4l2_int_device ov9655_int_device = {
	.module = THIS_MODULE,
	.name = OV9655_MODULE_NAME,
	.priv = &ov9655_dev,
	.type = v4l2_int_type_slave,
	.u = {
	      .slave = &ov9655_slave,
	      },
};

/*
 * ov9655_probe - decoder driver i2c probe handler
 * @client: i2c driver client device structure
 *
 * Register decoder as an i2c client device and V4L2
 * device.
 */
static int
ov9655_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct ov9655_decoder *decoder = &ov9655_dev;
	int err;

	/* Check if the adapter supports the needed features */
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_BYTE_DATA))
		return -EIO;

	decoder->pdata = client->dev.platform_data;
	if (!decoder->pdata) {
		v4l_err(client, "No platform data!!\n");
		return -ENODEV;
	}
	/*
	 * Save the id data, required for power up sequence
	 */
	decoder->id = (struct i2c_device_id *)id;
	/* Attach to Master */
	strcpy(ov9655_int_device.u.slave->attach_to, decoder->pdata->master);
	decoder->v4l2_int_device = &ov9655_int_device;
	decoder->client = client;
	i2c_set_clientdata(client, decoder);

	/* Register with V4L2 layer as slave device */
	err = v4l2_int_device_register(decoder->v4l2_int_device);
	if (err) {
		i2c_set_clientdata(client, NULL);
		v4l_err(client,
			"Unable to register to v4l2. Err[%d]\n", err);

	} else
		v4l_info(client, "Registered to v4l2 master %s!!\n",
				decoder->pdata->master);

	return 0;
}

/*
 * ov9655_remove - decoder driver i2c remove handler
 * @client: i2c driver client device structure
 *
 * Unregister decoder as an i2c client device and V4L2
 * device. Complement of ov9655_probe().
 */
static int __exit ov9655_remove(struct i2c_client *client)
{
	struct ov9655_decoder *decoder = i2c_get_clientdata(client);

	if (!client->adapter)
		return -ENODEV;	/* our client isn't attached */

	v4l2_int_device_unregister(decoder->v4l2_int_device);
	i2c_set_clientdata(client, NULL);

	return 0;
}
/*
 * ov9655 Init/Power on Sequence
 */
static const struct ov9655_reg ov9655m_init_reg_seq[] = {
	/*
	{TOK_WRITE, REG_OPERATION_MODE, 0x01},
	{TOK_WRITE, REG_OPERATION_MODE, 0x00},
	 */
};
static const struct ov9655_init_seq ov9655m_init = {
	.no_regs = ARRAY_SIZE(ov9655m_init_reg_seq),
	.init_reg_seq = ov9655m_init_reg_seq,
};

/*
 * I2C Device Table -
 *
 * name - Name of the actual device/chip.
 * driver_data - Driver data
 */
static const struct i2c_device_id ov9655_id[] = {
	{"ov9655", (unsigned long)&ov9655m_init},
	{},
};

MODULE_DEVICE_TABLE(i2c, ov9655_id);

static struct i2c_driver ov9655_i2c_driver = {
	.driver = {
		   .name = OV9655_MODULE_NAME,
		   .owner = THIS_MODULE,
		   },
	.probe = ov9655_probe,
	.remove = __exit_p(ov9655_remove),
	.id_table = ov9655_id,
};

/*
 * ov9655_init
 *
 * Module init function
 */
static int __init ov9655_init(void)
{
	return i2c_add_driver(&ov9655_i2c_driver);
}

/*
 * ov9655_cleanup
 *
 * Module exit function
 */
static void __exit ov9655_cleanup(void)
{
	i2c_del_driver(&ov9655_i2c_driver);
}

module_init(ov9655_init);
module_exit(ov9655_cleanup);

MODULE_DESCRIPTION("OmniVision OV9655 Camera driver");
MODULE_AUTHOR("H. Nikolaus Schaller <hns@goldelico.com>");
MODULE_LICENSE("GPL");


/*********************************************
* COPY of a SOC_CAMERA driver for the OV9655 *
*********************************************/

#ifdef SOC_CAMERA_DRIVER_SAMPLE_CODE_TO_BE_USED_AS_RAW_MATERIAL_AND_DELETED_IF_DRIVER_WORKS

/*
 * Driver for OV9655 CMOS Image Sensor from OmniVision
 *
 * Copyright (C) 2008 - 2009
 * Heinz Nixdorf Institute - University of Paderborn
 * Department of System and Circuit Technology
 * Stefan Herbrechtsmeier <hbmeier@hni.uni-paderborn.de>
 *
 * Based on rj54n1cbc, mt9t031 and soc_camera_platform driver
 *
 * Copyright (C) 2008, Guennadi Liakhovetski, DENX Software Engineering <lg@denx.de>
 * Copyright (C) 2009, Guennadi Liakhovetski <g.liakhovetski@gmx.de>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/init.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/videodev2.h>

#include <media/soc_camera.h>
#include <media/soc_mediabus.h>
#include <media/v4l2-common.h>
#include <media/v4l2-subdev.h>
#include <media/v4l2-chip-ident.h>

/* ov9655 i2c address 0x30
 * The platform has to define i2c_board_info
 * and call i2c_register_board_info() */

/* ov9655 register addresses */
#define OV9655_GAIN			0x00
#define OV9655_BLUE			0x01
#define OV9655_RED			0x02
#define OV9655_VREF			0x03
#define OV9655_COM1			0x04
#define OV9655_BAVE			0x05
#define OV9655_GBAVE			0x06
#define OV9655_GRAVE			0x07
#define OV9655_RAVE			0x08
#define OV9655_COM2			0x09
#define OV9655_COM2_SLEEP	0x10
#define OV9655_PID			0x0A
#define OV9655_VER			0x0B
#define OV9655_COM3			0x0C
#define OV9655_COM3_SWAP	0x40
#define OV9655_COM4			0x0D
#define OV9655_COM5			0x0E
#define OV9655_COM6			0x0F
#define OV9655_COM6_TIMING	0x02	
#define OV9655_COM6_WINDOW	0x04
#define OV9655_AEC			0x10
#define OV9655_CLKRC			0x11
#define OV9655_CLKRC_EXT	0x40
#define OV9655_CLKRC_SCALAR	0x3f
#define OV9655_COM7			0x12
#define OV9655_COM7_FMT_MASK	0x07
#define OV9655_COM7_RAW		0x00
#define OV9655_COM7_RAW_INT	0x01
#define OV9655_COM7_YUV		0x02
#define OV9655_COM7_RGB		0x03
#define OV9655_COM7_RGB5X5	0x07
#define OV9655_COM7_SXGA	0x00
#define OV9655_COM7_VGA		0x60
#define OV9655_COM8			0x13
#define OV9655_COM8_AGC		0x04
#define OV9655_COM8_AWB		0x02
#define OV9655_COM8_AEC		0x01
#define OV9655_COM9			0x14
#define OV9655_COM10			0x15
#define OV9655_REG16			0x16
#define OV9655_HSTART			0x17
#define OV9655_HSTOP			0x18
#define OV9655_VSTART			0x19
#define OV9655_VSTOP			0x1A
#define OV9655_PSHFT			0x1B
#define OV9655_MIDH			0x1C
#define OV9655_MIDL			0x1D
#define OV9655_MVFP			0x1E
#define OV9655_MVFP_VFLIP	0x10
#define OV9655_MVFP_MIRROR	0x20
#define OV9655_LAEC			0x1F
#define OV9655_BOS			0x20
#define OV9655_GBOS			0x21
#define OV9655_GROS			0x22
#define OV9655_ROS			0x23
#define OV9655_AEW			0x24
#define OV9655_AEB			0x25
#define OV9655_VPT			0x26
#define OV9655_BBIAS			0x27
#define OV9655_GBBIAS			0x28
#define OV9655_PREGAIN			0x29
#define OV9655_EXHCH			0x2A
#define OV9655_EXHCL			0x2B
#define OV9655_RBIAS			0x2C
#define OV9655_ADVFL			0x2D
#define OV9655_ADVFH			0x2E
#define OV9655_YAVE			0x2F
#define OV9655_HSYST			0x30
#define OV9655_HSYEN			0x31
#define OV9655_HREF			0x32
#define OV9655_CHLF			0x33
#define OV9655_AREF1			0x34
#define OV9655_AREF2			0x35
#define OV9655_AREF3			0x36
#define OV9655_ADC1			0x37
#define OV9655_ADC2			0x38
#define OV9655_AREF4			0x39
#define OV9655_TSLB			0x3A
#define OV9655_TSLB_YUV_MASK	0x0C
#define OV9655_TSLB_YUYV	0x00
#define OV9655_TSLB_YVYU	0x04
#define OV9655_TSLB_VYUY	0x08
#define OV9655_TSLB_UYVY	0x0C
#define OV9655_COM11			0x3B
#define OV9655_COM12			0x3C
#define OV9655_COM13			0x3D
#define OV9655_COM14			0x3E
#define OV9655_COM14_ZOOM	0x02
#define OV9655_EDGE			0x3F
#define OV9655_COM15			0x40
#define OV9655_COM15_RGB_MASK	0x30
#define OV9655_COM15_RGB	0x00
#define OV9655_COM15_RGB565	0x10
#define OV9655_COM15_RGB555	0x30
#define OV9655_COM16			0x41
#define OV9655_COM16_SCALING	0x01
#define OV9655_COM17			0x42
#define OV9655_MTX1			0x4F
#define OV9655_MTX2			0x50
#define OV9655_MTX3			0x51
#define OV9655_MTX4			0x52
#define OV9655_MTX5			0x53
#define OV9655_MTX6			0x54
#define OV9655_BRTN			0x55
#define OV9655_CNST1			0x56
#define OV9655_CNST2			0x57
#define OV9655_MTXS			0x58
#define OV9655_AWBOP1			0x59
#define OV9655_AWBOP2			0x5A
#define OV9655_AWBOP3			0x5B
#define OV9655_AWBOP4			0x5C
#define OV9655_AWBOP5			0x5D
#define OV9655_AWBOP6			0x5E
#define OV9655_BLMT			0x5F
#define OV9655_RLMT			0x60
#define OV9655_GLMT			0x61
#define OV9655_LCC1			0x62
#define OV9655_LCC2			0x63
#define OV9655_LCC3			0x64
#define OV9655_LCC4			0x65
#define OV9655_LCC5			0x66
#define OV9655_MANU			0x67
#define OV9655_MANV			0x68
#define OV9655_BD50MAX			0x6A
#define OV9655_DBLV			0x6B
#define OV9655_DNSTH			0x70
#define OV9655_POIDX			0x72
#define OV9655_POIDX_VDROP	0x40
#define OV9655_PCKDV			0x73
#define OV9655_XINDX			0x74
#define OV9655_YINDX			0x75
#define OV9655_SLOP			0x7A
#define OV9655_GAM1			0x7B
#define OV9655_GAM2			0x7C
#define OV9655_GAM3			0x7D
#define OV9655_GAM4			0x7E
#define OV9655_GAM5			0x7F
#define OV9655_GAM6			0x80
#define OV9655_GAM7			0x81
#define OV9655_GAM8			0x82
#define OV9655_GAM9			0x83
#define OV9655_GAM10			0x84
#define OV9655_GAM11			0x85
#define OV9655_GAM12			0x86
#define OV9655_GAM13			0x87
#define OV9655_GAM14			0x88
#define OV9655_GAM15			0x89
#define OV9655_COM18			0x8B
#define OV9655_COM19			0x8C
#define OV9655_COM20			0x8D
#define OV9655_DMLNL			0x92
#define OV9655_DMNLH			0x93
#define OV9655_LCC6			0x9D
#define OV9655_LCC7			0x9E
#define OV9655_AECH			0xA1
#define OV9655_BD50			0xA2
#define OV9655_BD60			0xA3
#define OV9655_COM21			0xA4
#define OV9655_GREEN			0xA6
#define OV9655_VZST			0xA7
#define OV9655_REFA8			0xA8
#define OV9655_REFA9			0xA9
#define OV9655_BLC1			0xAC
#define OV9655_BLC2			0xAD
#define OV9655_BLC3			0xAE
#define OV9655_BLC4			0xAF
#define OV9655_BLC5			0xB0
#define OV9655_BLC6			0xB1
#define OV9655_BLC7			0xB2
#define OV9655_BLC8			0xB3
#define OV9655_CTRLB4			0xB4
#define OV9655_FRSTL			0xB7
#define OV9655_FRSTH			0xB8
#define OV9655_ADBOFF			0xBC
#define OV9655_ADROFF			0xBD
#define OV9655_ADGBOFF			0xBE
#define OV9655_ADGROFF			0xBF
#define OV9655_COM23			0xC4
#define OV9655_BD60MAX			0xC5
#define OV9655_COM24			0xC7

#define OV9655_MAX_WIDTH		1280
#define OV9655_MIN_WIDTH		2
#define OV9655_MAX_HEIGHT		1024
#define OV9655_MIN_HEIGHT		2
#define OV9655_COLUMN_SKIP		237
#define OV9655_ROW_SKIP			11
#define OV9655_LEFT_SKIP		3
#define OV9655_TOP_SKIP			1

#define OV9655_COLUMS			1520
#define OV9655_ROWS			1050

struct ov9655_reg {
	u8 addr;
	u8 value;
};

static const struct ov9655_reg ov9655_init_regs[] = {
	{ OV9655_GAIN, 0x00 },
	{ OV9655_BLUE, 0x80 },
	{ OV9655_RED, 0x80 },
	{ OV9655_VREF, 0x1b },
	{ OV9655_COM1, 0x03 },
	{ OV9655_COM5, 0x61 },
	/* manually update window size and timing */
	{ OV9655_COM6, 0x40 },
	{ OV9655_CLKRC, 0x01 }, //was 0x03
	{ OV9655_COM7, 0x02 },
	{ OV9655_COM8, 0xe7 },
	{ OV9655_COM9, 0x2a },
	{ OV9655_REG16, 0x24 },
	{ OV9655_HSTART, 0x1d },
	{ OV9655_HSTOP, 0xbd },
	{ OV9655_VSTART, 0x01 },
	{ OV9655_VSTOP, 0x81 },
	{ OV9655_MVFP, 0x00 },
	{ OV9655_AEW, 0x3c },
	{ OV9655_AEB, 0x36 },
	{ OV9655_VPT, 0x72 },
	{ OV9655_BBIAS, 0x08 },
	{ OV9655_GBBIAS, 0x08 },
	{ OV9655_PREGAIN, 0x15 },
	{ OV9655_EXHCH, 0x00 },
	{ OV9655_EXHCL, 0x00 },
	{ OV9655_RBIAS, 0x08 },
	{ OV9655_HREF, 0x3f },
	{ OV9655_CHLF, 0x00 },
	{ OV9655_AREF2, 0x00 },
	{ OV9655_ADC2, 0x72 },
	{ OV9655_AREF4, 0x57 },
	{ OV9655_TSLB, 0x80 },
	{ OV9655_COM11, 0x05 }, //was 0xcc // was 0xa4; 0x05 disable night mode
	{ OV9655_COM13, 0x99 },
	{ OV9655_COM14, 0x0c },
	{ OV9655_EDGE, 0x82 }, // was 0xc1
	{ OV9655_COM15, 0xc0 },
	{ OV9655_COM16, 0x00 },
	{ OV9655_COM17, 0xc1 },
	{ 0x43, 0x0a },
	{ 0x44, 0xf0 },
	{ 0x45, 0x46 },
	{ 0x46, 0x62 },
	{ 0x47, 0x2a },
	{ 0x48, 0x3c },
	{ 0x4a, 0xfc },
	{ 0x4b, 0xfc },
	{ 0x4c, 0x7f },
	{ 0x4d, 0x7f },
	{ 0x4e, 0x7f },
	{ OV9655_AWBOP1, 0x85 },
	{ OV9655_AWBOP2, 0xa9 },
	{ OV9655_AWBOP3, 0x64 },
	{ OV9655_AWBOP4, 0x84 },
	{ OV9655_AWBOP5, 0x53 },
	{ OV9655_AWBOP6, 0x0e },
	{ OV9655_BLMT, 0xf0 },
	{ OV9655_RLMT, 0xf0 },
	{ OV9655_GLMT, 0xf0 },
	{ OV9655_LCC1, 0x00 },
	{ OV9655_LCC2, 0x00 },
	{ OV9655_LCC3, 0x02 },
	{ OV9655_DBLV, 0x4a }, //was 0xca // was 0xda
	{ 0x6c, 0x04 },
	{ 0x6d, 0x55 },
	{ 0x6e, 0x00 },
	{ 0x6f, 0x9d },
	{ OV9655_DNSTH, 0x21 },
	{ 0x71, 0x78 },
	{ 0x77, 0x02 },
	{ OV9655_SLOP, 0x12 },
	{ OV9655_GAM1, 0x08 },
	/*	{ OV9655_GAM2, 0x15 },
	 { OV9655_GAM3, 0x24 },
	 { OV9655_GAM4, 0x45 },
	 { OV9655_GAM5, 0x55 },
	 { OV9655_GAM6, 0x6a },
	 { OV9655_GAM7, 0x78 },
	 { OV9655_GAM8, 0x87 },
	 { OV9655_GAM9, 0x96 },
	 { OV9655_GAM10, 0xa3 },
	 { OV9655_GAM11, 0xb4 },
	 */	{ OV9655_GAM2, 0x16 },
	{ OV9655_GAM3, 0x30 },
	{ OV9655_GAM4, 0x5e },
	{ OV9655_GAM5, 0x72 },
	{ OV9655_GAM6, 0x82 },
	{ OV9655_GAM7, 0x8e },
	{ OV9655_GAM8, 0x9a },
	{ OV9655_GAM9, 0xa4 },
	{ OV9655_GAM10, 0xac },
	{ OV9655_GAM11, 0xb8 },
	{ OV9655_GAM12, 0xc3 },
	{ OV9655_GAM13, 0xd6 },
	{ OV9655_GAM14, 0xe6 },
	{ OV9655_GAM15, 0xf2 },
	{ 0x8a, 0x03 },
	{ 0x90, 0x7d },
	{ 0x91, 0x7b },
	{ OV9655_LCC6, 0x03 },
	{ 0x9f, 0x7a },
	{ 0xa0, 0x79 },
	{ OV9655_AECH, 0x40 },
	{ OV9655_COM21, 0x50 },
	{ 0xa5,0x68 },
	{ OV9655_GREEN, 0x4a },
	{ OV9655_REFA8, 0xc1 },
	{ OV9655_REFA9, 0xef },
	{ 0xaa, 0x92 },
	{ 0xab, 0x04 },
	{ OV9655_BLC1, 0x80 },
	{ OV9655_BLC2, 0x80 },
	{ OV9655_BLC3, 0x80 },
	{ OV9655_BLC4, 0x80 },
	{ OV9655_BLC7, 0xf2 },
	{ OV9655_BLC8, 0x20 },
	{ OV9655_CTRLB4, 0x20 },
	{ 0xb5, 0x00 },
	{ 0xb6, 0xaf },
	{ 0xbb, 0xae },
	{ OV9655_ADBOFF, 0x7f },
	{ OV9655_ADROFF, 0x7f },
	{ OV9655_ADGBOFF, 0x7f },
	{ OV9655_ADGROFF, 0x7f },
	{ 0xc1, 0xc0 },
	{ 0xc2, 0x01 },
	{ 0xc3, 0x4e },
	{ 0xc6, 0x85 },
	{ OV9655_COM24, 0x80 },
	{ 0xc9, 0xe0 },
	{ 0xca, 0xe8 },
	{ 0xcb, 0xf0 },
	{ 0xcc, 0xd8 },
	{ 0xcd, 0x93 },
	/* without VarioPixel */
	{ OV9655_AREF1, 0x3d },
	{ OV9655_AREF3, 0x34 },
	{ OV9655_LCC4, 0x16 },
	{ OV9655_LCC5, 0x01 },
	{ 0x69, 0x02 },
	{ OV9655_COM19, 0x0d },
	{ OV9655_COM20, 0x03 },
	{ OV9655_LCC7, 0x04 },
	{ 0xc0, 0xe2 },
	{ OV9655_BD50MAX, 0x05 },
	{ OV9655_BD50, 0x9d },
	{ OV9655_BD60, 0x83 },
	{ OV9655_BD60MAX, 0x07 },
	{ 0x76, 0x01 },
};

/* Register values for VGA format */
static const struct ov9655_reg ov9655_vga[] = {
	{ OV9655_AREF3, 0xfa },
	{ 0x69, 0x0a },
	{ OV9655_COM19, 0x89 },
	{ 0xc0, 0xaa },
};

/* Register values for YUV format */
static const struct ov9655_reg ov9655_yuv_regs[] = {
	{ OV9655_MTX1, 0x80 },
	{ OV9655_MTX2, 0x80 },
	{ OV9655_MTX3, 0x00 },
	{ OV9655_MTX4, 0x22 },
	{ OV9655_MTX5, 0x5e },
	{ OV9655_MTX6, 0x80 },
	{ OV9655_MTXS, 0x1e },
};

/* Register values for RGB format */
static const struct ov9655_reg ov9655_rgb_regs[] = {
	{ OV9655_MTX1, 0x98 },
	{ OV9655_MTX2, 0x98 },
	{ OV9655_MTX3, 0x00 },
	{ OV9655_MTX4, 0x28,},
	{ OV9655_MTX5, 0x70 },
	{ OV9655_MTX6, 0x98 },
	{ OV9655_MTXS, 0x1a },
};

struct ov9655_format {
	enum v4l2_mbus_pixelcode code;
	enum v4l2_colorspace colorspace;
	u8 format;
	u8 subformat;
	u8 order;
};

struct ov9655 {
	struct v4l2_subdev subdev;
	const struct ov9655_format *fmt;
	struct v4l2_rect rect;
	struct i2c_client *client;
	unsigned short width;
	unsigned short height;
	u8 com10;
};

/*
 * supported format list
 */
static const struct ov9655_format ov9655_formats[] = {
	{
	.code		= V4L2_MBUS_FMT_YUYV8_2X8_LE,
	.colorspace	= V4L2_COLORSPACE_JPEG,
	.format		= OV9655_COM7_YUV,
	.order		= OV9655_TSLB_YUYV,
	}, {
		.code		= V4L2_MBUS_FMT_YVYU8_2X8_LE,
		.colorspace	= V4L2_COLORSPACE_JPEG,
		.format		= OV9655_COM7_YUV,
		.order		= OV9655_TSLB_YVYU,
	}, {
		.code		= V4L2_MBUS_FMT_YUYV8_2X8_BE,
		.colorspace	= V4L2_COLORSPACE_JPEG,
		.format		= OV9655_COM7_YUV,
		.order		= OV9655_TSLB_VYUY,
	}, {
		.code		= V4L2_MBUS_FMT_YVYU8_2X8_BE,
		.colorspace	= V4L2_COLORSPACE_JPEG,
		.format		= OV9655_COM7_YUV,
		.order		= OV9655_TSLB_UYVY,
	}, {
		.code		= V4L2_MBUS_FMT_RGB555_2X8_PADHI_LE,
		.colorspace	= V4L2_COLORSPACE_SRGB,
		.format		= OV9655_COM7_RGB5X5,
		.subformat	= OV9655_COM15_RGB555,
		.order		= OV9655_COM3_SWAP,
	}, {
		.code		= V4L2_MBUS_FMT_RGB555_2X8_PADHI_BE,
		.colorspace	= V4L2_COLORSPACE_SRGB,
		.format		= OV9655_COM7_RGB5X5,
		.subformat	= OV9655_COM15_RGB555,
		.order		= 0,
	}, {
		.code		= V4L2_MBUS_FMT_RGB565_2X8_LE,
		.colorspace	= V4L2_COLORSPACE_SRGB,
		.format		= OV9655_COM7_RGB5X5,
		.subformat	= OV9655_COM15_RGB565,
		.order		= OV9655_COM3_SWAP,
	}, {
		.code		= V4L2_MBUS_FMT_RGB565_2X8_BE,
		.colorspace	= V4L2_COLORSPACE_SRGB,
		.format		= OV9655_COM7_RGB5X5,
		.subformat	= OV9655_COM15_RGB565,
		.order		= 0,
	}, {
		.code		= V4L2_MBUS_FMT_SBGGR8_1X8,
		.colorspace	= V4L2_COLORSPACE_SRGB,
		.format		= OV9655_COM7_RAW,
	},
};

static struct ov9655 *to_ov9655(const struct i2c_client *client)
{
	return container_of(i2c_get_clientdata(client), struct ov9655, subdev);
}

static int ov9655_read(struct i2c_client *client, const u8 reg)
{
	return i2c_smbus_read_byte_data(client, reg);
}

static int ov9655_write(struct i2c_client *client, const u8 reg,
						const u8 value)
{
	return i2c_smbus_write_byte_data(client, reg, value);
}

static int ov9655_set(struct i2c_client *client, const u8 reg,
					  const u8 value, const u8 mask)
{
	int ret;
	
	ret = ov9655_read(client, reg);
	if (ret < 0)
		return ret;
	return ov9655_write(client, reg, (ret & ~mask) | (value & mask));
}

static int ov9655_write_regs(struct i2c_client *client,
							 const struct ov9655_reg *regs, const int n)
{
	int i, ret;
	
	for (i = 0; i < n; i++) {
		ret = ov9655_write(client, regs->addr, regs->value);
		if (ret < 0)
			return ret;
		regs++;
	}
	
	return 0;
}

static int ov9655_reset(struct i2c_client *client)
{
	struct soc_camera_device *icd = client->dev.platform_data;
	struct soc_camera_link *icl = to_soc_camera_link(icd);
	struct ov9655 *ov9655 = to_ov9655(client);
	int ret;
	
	/*  Reset all registers to default values */
	if (!(icl->reset && icl->reset(&client->dev))) {
		/* Either no platform reset, or platform reset failed */
		ret = ov9655_write(client, OV9655_COM7, 0x80);
		mdelay(10);
		if (ret < 0)
			return ret;
	}
	
	/*  Set registers to init values */
	ret = ov9655_write_regs(client, ov9655_init_regs,
							ARRAY_SIZE(ov9655_init_regs));
	if (ret < 0)
		return ret;
	
	/* Set bus param */
	ret = ov9655_write(client, OV9655_COM10, ov9655->com10);
	if (ret < 0)
		return ret;
	
	dev_err(&client->dev, "Call reset (%u)", ov9655->com10);
	
	/* Disable the chip output */
	ret = ov9655_set(client, OV9655_COM2, OV9655_COM2_SLEEP,
					 OV9655_COM2_SLEEP);
	
	return ret;
}

/*
 * Interface active, can use i2c. If it fails, it can indeed mean, that
 * this wasn't our capture interface, so, we wait for the right one
 */
static int ov9655_video_probe(struct soc_camera_device *icd,
							  struct i2c_client *client)
{
	struct ov9655 *ov9655 = to_ov9655(client);
	struct soc_camera_link *icl = to_soc_camera_link(icd);
	s32 midh, midl, pid, ver;
	int ret;
	
	if (!icd->dev.parent ||
	    to_soc_camera_host(icd->dev.parent)->nr != icd->iface)
		return -ENODEV;
	
	/* Enable the camera chip */
	if (icl->power) {
		ret = icl->power(&client->dev, 1);
		if (ret < 0) {
			dev_err(icd->vdev->parent,
					"Platform failed to power-on the camera.\n");
			return ret;
		}
	}
	
	/* Read chip manufacturer register */
	if ((midh = ov9655_read(client, OV9655_MIDH)) < 0
	    || (midl = ov9655_read(client, OV9655_MIDL)) < 0) {
		dev_err(&client->dev, "Strange error reading sensor"
				" manufacturer\n");
		return -ENODEV;
	}
	
	if (midh != 0x7f || midl != 0xa2) {
		dev_err(&client->dev, "No OmniVision sensor detected, "
				" manufacturer register read 0x%i%i\n", midh, midl);
		return -ENODEV;
	}
	
	/* Read chip product register */
	pid = ov9655_read(client, OV9655_PID);
	
	if (pid != 0x96) {
		dev_info(&client->dev, "No OmniVision OV9655 sensor detected,"
				 " product register read 0x%x\n", pid);
		return -ENODEV;
	}
	
	/* Read out the chip version register */
	ver = ov9655_read(client, OV9655_VER);
	
	switch (ver) {
		case 0x56:
		case 0x57:
			break;
		default:
			dev_err(&client->dev, "No OmniVision OV9655 sensor detected,"
					" version register read 0x%x\n", ver);
			return -ENODEV;
	}
	
	ov9655->fmt = &ov9655_formats[0];
	
	dev_info(&client->dev, "Detected a OmniVision (0x%02x%02x) OV9655 sensor,"
			 " id 0x%02x%02x at adress %x without VarioPixel\n",
			 midh, midl, pid, ver, client->addr);
	
	return ov9655_reset(client);
}

static int ov9655_release(struct soc_camera_device *icd,
						  struct i2c_client *client)
{
	struct soc_camera_link *icl = to_soc_camera_link(icd);
	
	if (icl->power)
		icl->power(&client->dev, 0);
	
	return 0;
}

static int ov9655_g_skip_top_lines(struct v4l2_subdev *sd, u32 *lines)
{
	*lines = OV9655_TOP_SKIP;
	
	return 0;
}

/* Adjust crop rect to achieve fmt width and height */
static int ov9655_set_params(struct i2c_client *client, struct v4l2_rect *rect,
							 u32 width, u32 height)
{
	u8 hpoidx, vpoidx, zoom, scaling, poidx;
	u16 hstart, hstop, hskip, vstart, vstop, vskip;
	int ret;
	
	/* Zoom and scaling */
	for(hpoidx = 0;
	    ((rect->width >> (hpoidx + 1)) >= width) && (hpoidx < 3);
	    hpoidx++);
	
	for(vpoidx = 0;
	    ((rect->height >> (vpoidx + 1)) >= height) && (vpoidx < 3);
	    vpoidx++);
	
	if (hpoidx || vpoidx) {
		zoom = OV9655_COM14_ZOOM;
		scaling = OV9655_COM16_SCALING;
		poidx = (vpoidx << 4) | hpoidx;
		
		/* Drop unused vertical pixel data
		 to avoid green image on left side */
		if (vpoidx == 1)
			poidx |= OV9655_POIDX_VDROP;
		
		ret = ov9655_write(client, OV9655_POIDX, poidx);
		if (ret < 0)
			return ret;
		
		ret = ov9655_write(client, OV9655_XINDX, 0x10);
		if (ret < 0)
			return ret;
		
		ret = ov9655_write(client, OV9655_YINDX, 0x10);
		if (ret < 0)
			return ret;
	} else {
		zoom = 0;
		scaling = 0;
	}
	
	ret = ov9655_write(client, OV9655_PCKDV, hpoidx);
	if (ret < 0)
		return ret;
	
	ret = ov9655_write(client, OV9655_COM24, 0x80 | (hpoidx & 0x3));
	if (ret < 0)
		return ret;
	
	
	ret = ov9655_set(client, OV9655_COM14, zoom, OV9655_COM14_ZOOM);
	if (ret < 0)
		return ret;
	
	ret = ov9655_set(client, OV9655_COM16, scaling, OV9655_COM16_SCALING);
	if (ret < 0)
		return ret;
	
	rect->width = width << hpoidx;
	rect->height = height << vpoidx;
	hskip = OV9655_LEFT_SKIP << hpoidx;
	vskip = OV9655_TOP_SKIP << vpoidx;
	
	dev_err(&client->dev, "Scaled for %ux%u (%ux%u) : %ux%u\n",
			width, height, rect->width, rect->height, hpoidx, vpoidx);
	
	/* Sensor window */
	hstart	= rect->left + hskip;
	hstop	= rect->left + hskip + rect->width;
	vstart	= rect->top;
	vstop	= rect->top + vskip + rect->height;
	
	/*
	 * Horizontal: 11 bits, top 8 live in hstart and hstop.  Bottom 3 of
	 * hstart are in href[2:0], bottom 3 of hstop in href[5:3]. There is
	 * a mystery "edge offset" value in the top two bits of href.
	 *
	 * HSTOP values above 1520 don't work. Truncate with 1520 works!
	 */
	ret = ov9655_write(client, OV9655_HSTART, (hstart >> 3) & 0xff);
	if (ret < 0)
		return ret;
	ret = ov9655_write(client, OV9655_HSTOP, ((hstop % 1520) >> 3) & 0xff);
	if (ret < 0)
		return ret;
	ret = ov9655_set(client, OV9655_HREF,
					 ((hstop & 0x7) << 3) | (hstart & 0x7), 0x3f);
	if (ret < 0)
		return ret;
	
	/*
	 * Vertical: similar arrangement
	 */
	ret =  ov9655_write(client, OV9655_VSTART, (vstart >> 3) & 0xff);
	if (ret < 0)
		return ret;
	ret = ov9655_write(client, OV9655_VSTOP, (vstop >> 3) & 0xff);
	if (ret < 0)
		return ret;
	ret = ov9655_set(client, OV9655_VREF,
					 ((vstop & 0x7) << 3) | (vstart & 0x7), 0x3f);
	if (ret < 0)
		return ret;
	
	dev_err(&client->dev, "Adjusted for %ux%u : hstart %u, hstop %u = %u, "
			"vstart %u, vstop %u = %u\n",
			rect->width, rect->height, hstart, hstop, hstop - hstart,
			vstart, vstop, vstop - vstart);
	
	return 0;
}

static const struct ov9655_format *ov9655_find_format(
													  enum v4l2_mbus_pixelcode code, const struct ov9655_format *fmt,
													  int n)
{
	int i;
	for (i = 0; i < n; i++)
		if (fmt[i].code == code)
			return fmt + i;
	
	return NULL;
}

static int ov9655_try_fmt(struct v4l2_subdev *sd,
						  struct v4l2_mbus_framefmt *mf)
{
	struct i2c_client *client = sd->priv;
	struct ov9655 *ov9655 = to_ov9655(client);
	const struct ov9655_format *fmt;
	int align = mf->code == V4L2_MBUS_FMT_SBGGR8_1X8;
	
	v4l_bound_align_image(&mf->width, OV9655_MIN_WIDTH,
						  OV9655_MAX_WIDTH, align,
						  &mf->height, OV9655_MIN_HEIGHT,
						  OV9655_MAX_HEIGHT, align, 0);
	
	fmt = ov9655_find_format(mf->code, ov9655_formats,
							 ARRAY_SIZE(ov9655_formats));
	if (!fmt) {
		fmt = ov9655->fmt;
		mf->code = fmt->code;
	}
	
	mf->colorspace = fmt->colorspace;
	
	return 0;
}

/* ToDo: Depends on mclk (26 MHz), PLL (4) and format (2 bytes per pixel) */
static int ov9655_g_parm(struct v4l2_subdev *sd, struct v4l2_streamparm *parms)
{
	struct i2c_client *client = sd->priv;
	struct v4l2_captureparm *cp = &parms->parm.capture;
	int div, fps, ret;
	
	if (parms->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;
	
	ret = ov9655_read(client, OV9655_CLKRC);
	if (ret < 0)
		return ret;
	div = (ret & OV9655_CLKRC_EXT) ? 1 : (ret & OV9655_CLKRC_SCALAR) + 1;
	
	fps = 26000000 / (OV9655_COLUMS * OV9655_ROWS) * 4 / 2 * 1000 / div;
	
	memset(cp, 0, sizeof(struct v4l2_captureparm));
	cp->capability = V4L2_CAP_TIMEPERFRAME;
	cp->timeperframe.numerator = 1000;
	cp->timeperframe.denominator = fps;
	
	return 0;
}

/* ToDo: Depends on mclk (26 MHz), PLL (4) and format (2 bytes per pixel) */
static int ov9655_s_parm(struct v4l2_subdev *sd, struct v4l2_streamparm *parms)
{
	struct i2c_client *client = sd->priv;
	struct v4l2_captureparm *cp = &parms->parm.capture;
	struct v4l2_fract *tpf = &cp->timeperframe;
	u8 clkrc;
	int div, fps;
	
	dev_err(&client->dev, "Call s_parm\n");
	
	if (parms->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;
	
	if (cp->extendedmode != 0)
		return -EINVAL;
	
	if (tpf->numerator == 0 || tpf->denominator == 0)
		div = 1;  /* Reset to full rate */
	else
		div = 26000000 / (OV9655_COLUMS * OV9655_ROWS) * 4 / 2 *
		tpf->numerator / tpf->denominator;
	
	if (div > OV9655_CLKRC_SCALAR)
		clkrc = OV9655_CLKRC_SCALAR;
	else if (div < 1)
		clkrc = 0;
	else
		clkrc = div - 1;
	
	fps = 26000000 / (OV9655_COLUMS * OV9655_ROWS) * 4 / 2 * 1000 / (clkrc + 1);
	
	tpf->numerator = 1000;
	tpf->denominator = fps;
	return ov9655_set(client, OV9655_CLKRC, clkrc, OV9655_CLKRC_SCALAR);
}

static int ov9655_s_crop(struct v4l2_subdev *sd, struct v4l2_crop *a)
{
	struct i2c_client *client = sd->priv;
	struct ov9655 *ov9655 = to_ov9655(client);
	struct v4l2_rect *rect = &a->c;
	int ret;
	
	//ToDo: Check if this is needed
	//rect->width = ALIGN(rect->width, 2);
	//rect->height = ALIGN(rect->height, 2);
	
	soc_camera_limit_side(&rect->left, &rect->width,
						  OV9655_COLUMN_SKIP, OV9655_MIN_WIDTH, OV9655_MAX_WIDTH);
	
	soc_camera_limit_side(&rect->top, &rect->height,
						  OV9655_ROW_SKIP, OV9655_MIN_HEIGHT, OV9655_MAX_HEIGHT);
	
	ret = ov9655_set_params(client, rect, ov9655->width, ov9655->height);
	if (ret < 0)
		return ret;
	
	ov9655->rect = *rect;
	
	return 0;
}

static int ov9655_g_crop(struct v4l2_subdev *sd, struct v4l2_crop *a)
{
	struct i2c_client *client = sd->priv;
	struct ov9655 *ov9655 = to_ov9655(client);
	
	a->c	= ov9655->rect;
	a->type	= V4L2_BUF_TYPE_VIDEO_CAPTURE;
	
	return 0;
}

static int ov9655_cropcap(struct v4l2_subdev *sd, struct v4l2_cropcap *a)
{
	a->bounds.left			= OV9655_COLUMN_SKIP;
	a->bounds.top			= OV9655_ROW_SKIP;
	a->bounds.width			= OV9655_MAX_WIDTH;
	a->bounds.height		= OV9655_MAX_HEIGHT;
	a->defrect			= a->bounds;
	a->type				= V4L2_BUF_TYPE_VIDEO_CAPTURE;
	a->pixelaspect.numerator	= 1;
	a->pixelaspect.denominator	= 1;
	
	return 0;
}

static int ov9655_g_fmt(struct v4l2_subdev *sd,
						struct v4l2_mbus_framefmt *mf)
{
	struct i2c_client *client = sd->priv;
	struct ov9655 *ov9655 = to_ov9655(client);
	
	mf->width	= ov9655->width;
	mf->height	= ov9655->height;
	mf->code	= ov9655->fmt->code;
	mf->colorspace	= ov9655->fmt->colorspace;
	mf->field	= V4L2_FIELD_NONE;
	
	return 0;
}

static int ov9655_s_fmt(struct v4l2_subdev *sd,
						struct v4l2_mbus_framefmt *mf)
{
	struct i2c_client *client = sd->priv;
	struct ov9655 *ov9655 = to_ov9655(client);
	const struct ov9655_format *fmt;
	int ret;
	
	fmt = ov9655_find_format(mf->code, ov9655_formats,
							 ARRAY_SIZE(ov9655_formats));
	if (!fmt)
		return -EINVAL;
	
	ov9655_reset(client);
	
	ret = ov9655_set(client, OV9655_COM7, fmt->format,
					 OV9655_COM7_FMT_MASK);
	msleep(50);
	
	switch (fmt->code) {
		case V4L2_MBUS_FMT_YUYV8_2X8_LE:
		case V4L2_MBUS_FMT_YVYU8_2X8_LE:
		case V4L2_MBUS_FMT_YUYV8_2X8_BE:
		case V4L2_MBUS_FMT_YVYU8_2X8_BE:
			ret = ov9655_write_regs(client, ov9655_yuv_regs,
									ARRAY_SIZE(ov9655_yuv_regs));
			if (ret < 0)
				return ret;
			
			ret = ov9655_set(client, OV9655_TSLB, fmt->order,
							 OV9655_TSLB_YUV_MASK);
			if (ret < 0)
				return ret;
			break;
		case V4L2_MBUS_FMT_RGB555_2X8_PADHI_LE:
		case V4L2_MBUS_FMT_RGB555_2X8_PADHI_BE:
		case V4L2_MBUS_FMT_RGB565_2X8_LE:
		case V4L2_MBUS_FMT_RGB565_2X8_BE:
			ret = ov9655_write_regs(client, ov9655_rgb_regs,
									ARRAY_SIZE(ov9655_rgb_regs));
			if (ret < 0)
				return ret;
			
			ret = ov9655_set(client, OV9655_COM15, fmt->subformat,
							 OV9655_COM15_RGB_MASK);
			if (ret < 0)
				return ret;
			
			ret = ov9655_set(client, OV9655_COM3, fmt->order,
							 OV9655_COM3_SWAP);
			if (ret < 0)
				return ret;
			break;
		case V4L2_MBUS_FMT_SBGGR8_1X8:
			break;
		default:
			return -EINVAL;
	}
	
	ret = ov9655_set_params(client, &ov9655->rect, mf->width, mf->height);
	if (ret < 0)
		return ret;
	
	ov9655->fmt = fmt;
	ov9655->width = mf->width;
	ov9655->height = mf->height;
	
	return 0;
}

static unsigned long ov9655_query_bus_param(struct soc_camera_device *icd)
{
	struct soc_camera_link *icl = to_soc_camera_link(icd);
	unsigned long flags;
	
	flags = SOCAM_PCLK_SAMPLE_RISING | SOCAM_PCLK_SAMPLE_FALLING |
	SOCAM_HSYNC_ACTIVE_HIGH | SOCAM_HSYNC_ACTIVE_LOW |
	SOCAM_VSYNC_ACTIVE_HIGH | SOCAM_VSYNC_ACTIVE_LOW |
	SOCAM_DATA_ACTIVE_HIGH | SOCAM_MASTER |
	SOCAM_DATAWIDTH_8;
	
	return soc_camera_apply_sensor_flags(icl, flags);
}

static int ov9655_set_bus_param(struct soc_camera_device *icd,
								unsigned long flags)
{
	struct i2c_client *client = to_i2c_client(to_soc_camera_control(icd));
	struct ov9655 *ov9655 = to_ov9655(client);
	u8 com10 = 0x0;
	int ret;
	
	/* PCLK reverse */
	if (flags & SOCAM_PCLK_SAMPLE_RISING)
		com10 |= 0x10;
	
	/* HREF negative */
	if (flags & SOCAM_HSYNC_ACTIVE_LOW)
		com10 |= 0x08;
	
	/* VSYNC negative */
	if (flags & SOCAM_VSYNC_ACTIVE_LOW)
		com10 |= 0x02;
	
	ret = ov9655_write(client, OV9655_COM10, com10);
	if (ret < 0)
		return ret;
	
	dev_err(&client->dev, "Call set_bus_param (%u)", com10);
	
	ov9655->com10 = com10;
	
	return 0;
}

static int ov9655_g_chip_ident(struct v4l2_subdev *sd,
							   struct v4l2_dbg_chip_ident *id)
{
	struct i2c_client *client = sd->priv;
	
	if (id->match.type != V4L2_CHIP_MATCH_I2C_ADDR)
		return -EINVAL;
	
	if (id->match.addr != client->addr)
		return -ENODEV;
	
	id->ident	= V4L2_IDENT_UNKNOWN;
	id->revision	= 0;
	
	return 0;
}

const struct v4l2_queryctrl ov9655_controls[] = {
	{
	.id		= V4L2_CID_BRIGHTNESS,
	.type		= V4L2_CTRL_TYPE_INTEGER,
	.name		= "Brightness",
	.minimum	= -127,
	.maximum	= 127,
	.step		= 1,
	.default_value	= 0,
	.flags		= V4L2_CTRL_FLAG_SLIDER
	}, {
		.id		= V4L2_CID_CONTRAST,
		.type		= V4L2_CTRL_TYPE_INTEGER,
		.name		= "Contrast",
		.minimum	= 0,
		.maximum	= 255,
		.step		= 1,
		.default_value	= 0x40,
		.flags		= V4L2_CTRL_FLAG_SLIDER,
	}, {
		.id		= V4L2_CID_AUTO_WHITE_BALANCE,
		.type		= V4L2_CTRL_TYPE_BOOLEAN,
		.name		= "Automatic White Balance",
		.minimum	= 0,
		.maximum	= 1,
		.step		= 1,
		.default_value	= 1,
	}, {
		.id		= V4L2_CID_RED_BALANCE,
		.type		= V4L2_CTRL_TYPE_INTEGER,
		.name		= "Red Channel Gain",
		.minimum	= 0,
		.maximum	= 255,
		.step		= 1,
		.default_value	= 0x80,
		.flags		= V4L2_CTRL_FLAG_SLIDER,
	}, {
		.id		= V4L2_CID_BLUE_BALANCE,
		.type		= V4L2_CTRL_TYPE_INTEGER,
		.name		= "Blue Channel Gain",
		.minimum	= 0,
		.maximum	= 255,
		.step		= 1,
		.default_value	= 0x80,
		.flags		= V4L2_CTRL_FLAG_SLIDER,
	}, {
		.id		= V4L2_CID_EXPOSURE,
		.type		= V4L2_CTRL_TYPE_INTEGER,
		.name		= "Exposure",
		.minimum	= 0,
		.maximum	= 255,
		.step		= 1,
		.default_value	= 0x40,
		.flags		= V4L2_CTRL_FLAG_SLIDER,
	}, {
		.id		= V4L2_CID_AUTOGAIN,
		.type		= V4L2_CTRL_TYPE_BOOLEAN,
		.name		= "Automatic Gain",
		.minimum	= 0,
		.maximum	= 1,
		.step		= 1,
		.default_value	= 1,
	}, {
		.id		= V4L2_CID_GAIN,
		.type		= V4L2_CTRL_TYPE_INTEGER,
		.name		= "Gain",
		.minimum	= 0,
		.maximum	= 255,
		.step		= 1,
		.default_value	= 0,
		.flags		= V4L2_CTRL_FLAG_SLIDER,
	}, {
		.id		= V4L2_CID_HFLIP,
		.type		= V4L2_CTRL_TYPE_BOOLEAN,
		.name		= "Horizontal flip",
		.minimum	= 0,
		.maximum	= 1,
		.step		= 1,
		.default_value	= 0,
	}, {
		.id		= V4L2_CID_VFLIP,
		.type		= V4L2_CTRL_TYPE_BOOLEAN,
		.name		= "Vertically flip",
		.minimum	= 0,
		.maximum	= 1,
		.step		= 1,
		.default_value	= 0,
	}, {
		.id		= V4L2_CID_EXPOSURE_AUTO,
		.type		= V4L2_CTRL_TYPE_BOOLEAN,
		.name		= "Automatic Exposure",
		.minimum	= 0,
		.maximum	= 1,
		.step		= 1,
		.default_value	= 1,
	}
};

static struct soc_camera_ops ov9655_ops = {
	.set_bus_param		= ov9655_set_bus_param,
	.query_bus_param	= ov9655_query_bus_param,
	.controls		= ov9655_controls,
	.num_controls		= ARRAY_SIZE(ov9655_controls),
};

static int ov9655_g_ctrl(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	struct i2c_client *client = sd->priv;
	int value;
	
	switch (ctrl->id) {
		case V4L2_CID_BRIGHTNESS:
			value = ov9655_read(client, OV9655_AEW);
			if (value < 0)
				return -EIO;
			ctrl->value = value + 3;
			break;
		case V4L2_CID_CONTRAST:
			value = ov9655_read(client, OV9655_CNST1);
			if (value < 0)
				return -EIO;
			ctrl->value = value + 3;
			break;
		case V4L2_CID_AUTO_WHITE_BALANCE:
			value = ov9655_read(client, OV9655_COM8);
			if (value < 0)
				return -EIO;
			ctrl->value = !!(value & OV9655_COM8_AWB);
			break;
		case V4L2_CID_RED_BALANCE:
			value = ov9655_read(client, OV9655_RED);
			if (value < 0)
				return -EIO;
			ctrl->value = value;
			break;
		case V4L2_CID_BLUE_BALANCE:
			value = ov9655_read(client, OV9655_BLUE);
			if (value < 0)
				return -EIO;
			ctrl->value = value;
			break;
		case V4L2_CID_EXPOSURE:
			value = ov9655_read(client, OV9655_AEC);
			if (value < 0)
				return -EIO;
			ctrl->value = value;
			break;
		case V4L2_CID_AUTOGAIN:
			value = ov9655_read(client, OV9655_COM8);
			if (value < 0)
				return -EIO;
			ctrl->value = !!(value & OV9655_COM8_AGC);
			break;
		case V4L2_CID_GAIN:
			value = ov9655_read(client, OV9655_GAIN);
			if (value < 0)
				return -EIO;
			ctrl->value = value;
			break;
		case V4L2_CID_HFLIP:
			value = ov9655_read(client, OV9655_MVFP);
			if (value < 0)
				return -EIO;
			ctrl->value = !!(value & OV9655_MVFP_MIRROR);
			break;
		case V4L2_CID_VFLIP:
			value = ov9655_read(client, OV9655_MVFP);
			if (value < 0)
				return -EIO;
			ctrl->value = !!(value & OV9655_MVFP_VFLIP);
			break;
		case V4L2_CID_EXPOSURE_AUTO:
			value = ov9655_read(client, OV9655_COM8);
			if (value < 0)
				return -EIO;
			ctrl->value = !!(value & OV9655_COM8_AEC);
			break;
	}
	return 0;
}

static int ov9655_s_ctrl(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	struct i2c_client *client = sd->priv;
	const struct v4l2_queryctrl *qctrl;
	int value, ret = 0;
	
	qctrl = soc_camera_find_qctrl(&ov9655_ops, ctrl->id);
	
	if (!qctrl)
		return -EINVAL;
	
	switch (ctrl->id) {
		case V4L2_CID_BRIGHTNESS:
			if (ctrl->value > qctrl->maximum ||
				ctrl->value < qctrl->minimum)
				return -EINVAL;
			value = (ctrl->value < 0) ?
			0x80 | abs(ctrl->value) : ctrl->value;
			ret = ov9655_write(client, OV9655_BRTN, ctrl->value);
			if (ret < 0)
				return -EIO;
			break;
		case V4L2_CID_CONTRAST:
			if (ctrl->value > qctrl->maximum ||
				ctrl->value < qctrl->minimum)
				return -EINVAL;
			ret = ov9655_write(client, OV9655_CNST1, ctrl->value);
			if (ret >= 0)
				ret = ov9655_write(client, OV9655_AEW, ctrl->value);
			break;
		case V4L2_CID_AUTO_WHITE_BALANCE:
			value = (ctrl->value) ? OV9655_COM8_AGC : 0;
			ret = ov9655_set(client, OV9655_COM8, value, OV9655_COM8_AWB);
			break;
		case V4L2_CID_RED_BALANCE:
			if (ctrl->value > qctrl->maximum ||
				ctrl->value < qctrl->minimum)
				return -EINVAL;
			/* The user wantsvpoidx to set red gain manually, hope, she
			 * knows, what she's doing... Switch AWB off. */
			ret = ov9655_set(client, OV9655_COM8, 0, OV9655_COM8_AWB);
			if (ret >= 0)
				ret = ov9655_write(client, OV9655_RED, ctrl->value);
			break;
		case V4L2_CID_BLUE_BALANCE:
			if (ctrl->value > qctrl->maximum ||
				ctrl->value < qctrl->minimum)
				return -EINVAL;
			/* The user wants to set blue gain manually, hope, she
			 * knows, what she's doing... Switch AWB off. */
			ret = ov9655_set(client, OV9655_COM8, 0, OV9655_COM8_AWB);
			if (ret >= 0)
				ret = ov9655_write(client, OV9655_BLUE, ctrl->value);
			break;
		case V4L2_CID_EXPOSURE:
			if (ctrl->value > qctrl->maximum ||
				ctrl->value < qctrl->minimum)
				return -EINVAL;
			/* The user wants to set exposure manually, hope, she
			 * knows, what she's doing... Switch AEC off. */
			ret = ov9655_set(client, OV9655_COM8, 0, OV9655_COM8_AEC);
			if (ret >= 0)
				ret = ov9655_write(client, OV9655_AEC, ctrl->value);
			break;
		case V4L2_CID_AUTOGAIN:
			value = (ctrl->value) ? OV9655_COM8_AGC : 0;
			ret = ov9655_set(client, OV9655_COM8, value, OV9655_COM8_AGC);
			break;
		case V4L2_CID_GAIN:
			if (ctrl->value > qctrl->maximum ||
				ctrl->value < qctrl->minimum)
				return -EINVAL;
			/* The user wants to set gain manually, hope, she
			 * knows, what she's doing... Switch AGC off. */
			ret = ov9655_set(client, OV9655_COM8, 0, OV9655_COM8_AGC);
			if (ret >= 0)
				ret = ov9655_write(client, OV9655_GAIN, ctrl->value);
			break;
		case V4L2_CID_HFLIP:
			value = (ctrl->value) ? OV9655_MVFP_MIRROR : 0;
			ret = ov9655_set(client, OV9655_MVFP, value, OV9655_MVFP_MIRROR);
			break;
		case V4L2_CID_VFLIP:
			value = (ctrl->value) ? OV9655_MVFP_VFLIP : 0;
			ret = ov9655_set(client, OV9655_MVFP, value, OV9655_MVFP_VFLIP);
			break;
		case V4L2_CID_EXPOSURE_AUTO:
			value = (ctrl->value) ? OV9655_COM8_AGC : 0;
			ret = ov9655_set(client, OV9655_COM8, value, OV9655_COM8_AEC);
			break;
	}
	return (ret < 0) ? -EIO : 0;
}

#ifdef CONFIG_VIDEO_ADV_DEBUG
static int ov9655_g_register(struct v4l2_subdev *sd,
							 struct v4l2_dbg_register *reg)
{
	struct i2c_client *client = sd->priv;
	
	if (reg->match.type != V4L2_CHIP_MATCH_I2C_ADDR || reg->reg > 0xff)
		return -EINVAL;
	
	if (reg->match.addr != client->addr)
		return -ENODEV;
	
	reg->size = 1;
	reg->val = ov9655_read(client, reg->reg);
	
	if (reg->val > 0xff)
		return -EIO;
	
	return 0;
}

static int ov9655_set_register(struct soc_camera_device *icd,
							   struct v4l2_dbg_register *reg)
{
	struct i2c_client *client = sd->priv;
	
	if (reg->match.type != V4L2_CHIP_MATCH_I2C_ADDR || reg->reg > 0xff)
		return -EINVAL;
	
	if (reg->match.addr != client->addr)
		return -ENODEV;
	
	if (ov9655_write(client, reg->reg, reg->val) < 0)
		return -EIO;
	
	return 0;
}
#endif

static struct v4l2_subdev_core_ops ov9655_subdev_core_ops = {
	.g_ctrl		= ov9655_g_ctrl,
	.s_ctrl		= ov9655_s_ctrl,
	.g_chip_ident	= ov9655_g_chip_ident,
#ifdef CONFIG_VIDEO_ADV_DEBUG
	.g_register	= ov9655_g_register,
	.s_register	= ov9655_s_register,
#endif
};

static int ov9655_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct i2c_client *client = sd->priv;
	int ret;
	
	if (enable)
		ret = ov9655_set(client, OV9655_COM2, 0, OV9655_COM2_SLEEP);
	else
		ret = ov9655_set(client, OV9655_COM2, OV9655_COM2_SLEEP,
						 OV9655_COM2_SLEEP);
	
	if (ret < 0)
		return -EIO;
	
	return 0;
}

static int ov9655_enum_fmt(struct v4l2_subdev *sd, int index,
						   enum v4l2_mbus_pixelcode *code)
{
	if ((unsigned int)index >= ARRAY_SIZE(ov9655_formats))
		return -EINVAL;
	
	*code = ov9655_formats[index].code;
	return 0;
}

static struct v4l2_subdev_video_ops ov9655_subdev_video_ops = {
	.s_stream	= ov9655_s_stream,
	.s_mbus_fmt	= ov9655_s_fmt,
	.g_mbus_fmt	= ov9655_g_fmt,
	.try_mbus_fmt	= ov9655_try_fmt,
	.s_crop		= ov9655_s_crop,
	.g_crop		= ov9655_g_crop,
	.cropcap	= ov9655_cropcap,
	.enum_mbus_fmt	= ov9655_enum_fmt,
	.s_parm		= ov9655_s_parm,
	.g_parm		= ov9655_g_parm,
};

static struct v4l2_subdev_sensor_ops ov9655_subdev_sensor_ops = {
	.g_skip_top_lines	= ov9655_g_skip_top_lines,
};

static struct v4l2_subdev_ops ov9655_subdev_ops = {
	.core	= &ov9655_subdev_core_ops,
	.video	= &ov9655_subdev_video_ops,
	.sensor	= &ov9655_subdev_sensor_ops,
};

static int ov9655_probe(struct i2c_client *client,
						const struct i2c_device_id *did)
{
	struct ov9655 *ov9655;
	struct soc_camera_device *icd = client->dev.platform_data;
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	struct soc_camera_link *icl;
	int ret;
	
	if (!icd) {
		dev_err(&client->dev, "Missing soc-camera data!\n");
		return -EINVAL;
	}
	
	icl = to_soc_camera_link(icd);
	if (!icl) {
		dev_err(&client->dev, "Missing platform_data for driver\n");
		return -EINVAL;
	}
	
	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE_DATA)) {
		dev_warn(&adapter->dev,
				 "I2C-Adapter doesn't support I2C_FUNC_SMBUS_BYTE_DATA\n");
		return -EIO;
	}
	
	ov9655 = kzalloc(sizeof(struct ov9655), GFP_KERNEL);
	if (!ov9655) {
		dev_err(&client->dev,
				"Failed to allocate memory for private data!\n");
		return -ENOMEM;
	}
	
	v4l2_i2c_subdev_init(&ov9655->subdev, client, &ov9655_subdev_ops);
	
	icd->ops = &ov9655_ops;
	
	ov9655->rect.left = OV9655_COLUMN_SKIP;
	ov9655->rect.top = OV9655_ROW_SKIP;
	ov9655->rect.width = OV9655_MAX_WIDTH;
	ov9655->rect.height = OV9655_MAX_HEIGHT;
	ov9655->width = OV9655_MAX_WIDTH;
	ov9655->height = OV9655_MAX_HEIGHT;
	ov9655->fmt = &ov9655_formats[0];
	
	ret = ov9655_video_probe(icd, client);
	if (ret) {
		icd->ops = NULL;
		i2c_set_clientdata(client, NULL);
		kfree(ov9655);
	}
	
	return ret;
}

static int ov9655_remove(struct i2c_client *client)
{
	struct ov9655 *ov9655 = to_ov9655(client);
	struct soc_camera_device *icd = client->dev.platform_data;
	
	ov9655_release(icd, client);
	
	icd->ops = NULL;
	i2c_set_clientdata(client, NULL);
	client->driver = NULL;
	kfree(ov9655);
	
	return 0;
}

static const struct i2c_device_id ov9655_id[] = {
	{ "ov9655", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, ov9655_id);

static struct i2c_driver ov9655_i2c_driver = {
	.driver = {
		.name = "ov9655",
	},
	.probe		= ov9655_probe,
	.remove		= ov9655_remove,
	.id_table	= ov9655_id,
};

static int __init ov9655_mod_init(void)
{
	return i2c_add_driver(&ov9655_i2c_driver);
}

static void __exit ov9655_mod_exit(void)
{
	i2c_del_driver(&ov9655_i2c_driver);
}

module_init(ov9655_mod_init);
module_exit(ov9655_mod_exit);

MODULE_DESCRIPTION("OmniVision OV9655 Camera driver");
MODULE_AUTHOR("Stefan Herbrechtsmeier <hbmeier@hni.upb.de>");
MODULE_LICENSE("GPL");

#endif
