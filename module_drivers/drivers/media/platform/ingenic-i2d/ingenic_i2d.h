#ifndef __I2D_H__
#define __I2D_H__

#define I2D_BASE	0xB30B0000

#define I2D_RTL_VERSION	        0x00
#define I2D_SHD_CTRL        0x08
#define I2D_CTRL	        0x10
#define I2D_IMG_SIZE	    0x14
#define I2D_IMG_MODE	    0x18
#define I2D_SRC_ADDR_Y	    0x20
#define I2D_SRC_ADDR_UV	    0x24
#define I2D_SRC_Y_STRID	    0x28
#define I2D_SRC_UV_STRID	0x2C
#define I2D_DST_ADDR_Y	    0x30
#define I2D_DST_ADDR_UV	    0x34
#define I2D_DST_Y_STRID	    0x38
#define I2D_DST_UV_STRID	0x3C
#define I2D_IRQ_STATE	    0x80
#define I2D_IRQ_CLEAR	    0x84
#define I2D_IRQ_MASK	    0x88
#define I2D_TIMEOUT_VALUE	0x90
#define I2D_TIMEOUT_MODE	0x94
#define I2D_CLK_GATE	    0x98
#define I2D_DBG_0	        0xA0

#define u32 unsigned int

/* VERSION*/
#define I2D_VERSION (1 << 0)

/* SHD_CTRL */
#define I2D_MODE_SHADOW (1 << 1)

/* SHD_CTRL */
#define I2D_MODE_SHADOW (1 << 1)
#define I2D_ADDR_SHADOW (1 << 0)

/* CTRL */
#define I2D_RESET       (1 << 16)
#define I2D_SAFE_RESET  (1 << 4)
#define I2D_START       (1 << 0)

/* IMG_SIZE */
#define I2D_WIDTH       (16)
#define I2D_HEIGHT      (0)

/* I2D_MODE */
#define I2D_DATA_TYPE            (4)
#define I2D_DATA_TYPE_NV12       (0 << I2D_DATA_TYPE)
#define I2D_DATA_TYPE_RAW8       (1 << I2D_DATA_TYPE)
#define I2D_DATA_TYPE_RGB565     (2 << I2D_DATA_TYPE)
#define I2D_DATA_TYPE_ARGB8888   (3 << I2D_DATA_TYPE)

#define I2D_FLIP_MODE            (2)
#define I2D_FLIP_MODE_0          (0 << I2D_FLIP_MODE)
#define I2D_FLIP_MODE_90         (1 << I2D_FLIP_MODE)
#define I2D_FLIP_MODE_180        (2 << I2D_FLIP_MODE)
#define I2D_FLIP_MODE_270        (3 << I2D_FLIP_MODE)

#define I2D_MIRR        (1 << 1)
#define I2D_FLIP        (1 << 0)

/* SRC_ADDR_Y */
#define I2D_SRC_Y       (0)

/* SRC_ADDR_UV */
#define I2D_SRC_UV      (0)

/* SRC_Y_STRID */
#define I2D_SRC_STRID_Y       (0)

/* SRC_UV_STRID */
#define I2D_SRC_STRID_UV      (0)

/* DST_ADDR_Y */
#define I2D_DST_Y       (0)

/* DST_ADDR_UV */
#define I2D_DST_UV      (0)

/* DST_Y_STRID */
#define I2D_DST_STRID_Y       (0)

/* DST_UV_STRID */
#define I2D_DST_STRID_UV      (0)

/* IRQ_STATE */
#define I2D_IRQ_TIMEOUT       (1)
#define I2D_IRQ_FRAME_DONE    (0)

/* IRQ_CLEAR */
#define I2D_IRQ_TIMEOUT_CLR       (1 << 1)
#define I2D_IRQ_FRAME_DONE_CLR    (1 << 0)

/* IRQ_MASK */
#define I2D_IRQ_TIMEOUT_MASK       (0 << 1)
#define I2D_IRQ_FRAME_DONE_MASK    (0 << 0)

/* TIMEOUT_MODE */
#define I2D_TIMEOUT_RST       (0)

enum                          
{
    FLIP_MODE_0_DEGREE   = 1,
    FLIP_MODE_90_DEGREE  = 2,
    FLIP_MODE_180_DEGREE = 3,
    FLIP_MODE_270_DEGREE = 4,
    FLIP_MODE_MIRR       = 5,
    FLIP_MODE_FLIP       = 6,
};


struct i2d_param
{
    unsigned int        src_w;
    unsigned int        src_h;
    unsigned int        data_type;
    unsigned int        rotate_enable;
    unsigned int        rotate_angle;
    unsigned int        flip_enable;
    unsigned int        mirr_enable;

    unsigned int        src_addr_y;
    unsigned int        src_addr_uv;
    unsigned int        dst_addr_y;
    unsigned int        dst_addr_uv;

    unsigned int        src_y_strid;
    unsigned int        src_uv_strid;
    unsigned int        dst_y_strid;
    unsigned int        dst_uv_strid;
};

static inline unsigned int reg_read(struct ingenic_i2d_dev *dev,int offset)
{
	return readl(dev->base + offset);
}

static inline void reg_write(struct ingenic_i2d_dev *dev,int offset, unsigned int val)
{
	writel(val, dev->base + offset);
}

void i2d_safe_reset(struct ingenic_i2d_dev *dev);

void i2d_reset(struct ingenic_i2d_dev *dev);

int i2d_set_init_info(struct ingenic_i2d_dev *dev,struct i2d_frame_info *info,dma_addr_t src_addr,dma_addr_t dst_addr);

int i2d_start(struct ingenic_i2d_dev *dev);

int i2d_irq_clear(struct ingenic_i2d_dev *dev);
#endif		// __I2D_H__



