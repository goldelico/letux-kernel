#ifndef __LCD_PANEL_H__
#define __LCD_PANEL_H__

enum ingenic_lcd_type {
        LCD_TYPE_TFT = 0,
        LCD_TYPE_SLCD =1,
        LCD_TYPE_MIPI_SLCD = 2,
	LCD_TYPE_MIPI_TFT,
};

/* smart lcd interface_type */
enum smart_lcd_type {
	SMART_LCD_TYPE_6800,
	SMART_LCD_TYPE_8080,
	SMART_LCD_TYPE_SPI_3,
	SMART_LCD_TYPE_SPI_4,
};

/* smart lcd format */
enum smart_lcd_format {
	SMART_LCD_FORMAT_565,
	SMART_LCD_FORMAT_666,
	SMART_LCD_FORMAT_888,
};

/* smart lcd command width */
enum smart_lcd_cwidth {
	SMART_LCD_CWIDTH_8_BIT,
	SMART_LCD_CWIDTH_9_BIT,
	SMART_LCD_CWIDTH_16_BIT,
	SMART_LCD_CWIDTH_18_BIT,
	SMART_LCD_CWIDTH_24_BIT,
};

/* smart lcd data width */
enum smart_lcd_dwidth {
        SMART_LCD_DWIDTH_8_BIT,
        SMART_LCD_DWIDTH_9_BIT,
        SMART_LCD_DWIDTH_16_BIT,
        SMART_LCD_DWIDTH_18_BIT,
        SMART_LCD_DWIDTH_24_BIT,
};

/* smart lcd data width */
enum smart_config_type {
	SMART_CONFIG_DATA,
	SMART_CONFIG_PRM,
	SMART_CONFIG_CMD,
	SMART_CONFIG_UDELAY,
};
struct smart_lcd_data_table {
	enum smart_config_type type;
	unsigned int value;
};
enum tft_lcd_color_even {
	TFT_LCD_COLOR_EVEN_RGB,
	TFT_LCD_COLOR_EVEN_RBG,
	TFT_LCD_COLOR_EVEN_BGR,
	TFT_LCD_COLOR_EVEN_BRG,
	TFT_LCD_COLOR_EVEN_GBR,
	TFT_LCD_COLOR_EVEN_GRB,
};

enum tft_lcd_color_odd {
	TFT_LCD_COLOR_ODD_RGB,
	TFT_LCD_COLOR_ODD_RBG,
	TFT_LCD_COLOR_ODD_BGR,
	TFT_LCD_COLOR_ODD_BRG,
	TFT_LCD_COLOR_ODD_GBR,
	TFT_LCD_COLOR_ODD_GRB,
};

enum tft_lcd_mode {
	TFT_LCD_MODE_PARALLEL_888,
	TFT_LCD_MODE_PARALLEL_666,
	TFT_LCD_MODE_PARALLEL_565,
	TFT_LCD_MODE_SERIAL_RGB,
	TFT_LCD_MODE_SERIAL_RGBD,
};

struct tft_config {
	unsigned int pix_clk_inv:1;
	unsigned int de_dl:1;
	unsigned int sync_dl:1;
	unsigned int vsync_dl:1;
	enum tft_lcd_color_even color_even;
	enum tft_lcd_color_odd color_odd;
	enum tft_lcd_mode mode;
};

struct smart_config {
	unsigned int te_switch:1;
	unsigned int te_mipi_switch:1;
	unsigned int te_md:1;
	unsigned int te_dp:1;
	unsigned int te_anti_jit:1;
	unsigned int dc_md:1;
	unsigned int wr_md:1;
	enum smart_lcd_type smart_type;
	enum smart_lcd_format pix_fmt;
	enum smart_lcd_dwidth dwidth;
	enum smart_lcd_cwidth cwidth;
	unsigned int bus_width;

	unsigned long write_gram_cmd;
	unsigned int length_cmd;
	struct smart_lcd_data_table *data_table;
	unsigned int length_data_table;
	int (*init) (void);
	int (*gpio_for_slcd) (void);
};


struct lcd_panel_ops {
	void (*init)(void *panel);
	void (*enable)(void *panel);
	void (*disable)(void *panel);
};

struct jzdsi_data {
	struct fb_videomode *modes;
	struct video_config video_config;
	struct dsi_config dsi_config;
	unsigned int bpp_info;
	unsigned int max_bps;
	unsigned int dsi_iobase;
	unsigned int dsi_phy_iobase;
};

struct lcd_panel {
	const char *name;
	unsigned int num_modes;
	struct fb_videomode *modes;
	struct jzdsi_data *dsi_pdata;

	enum ingenic_lcd_type lcd_type;
	unsigned int bpp;
	unsigned int width;
	unsigned int height;

	struct smart_config *smart_config;
	struct tft_config *tft_config;

	unsigned dither_enable:1;
	struct {
		unsigned dither_red;
		unsigned dither_green;
		unsigned dither_blue;
	} dither;

	struct lcd_panel_ops *ops;
};



#endif
