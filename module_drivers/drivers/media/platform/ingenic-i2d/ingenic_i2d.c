#include <linux/io.h>
#include <linux/delay.h>

#include "i2d_video.h"
#include "ingenic_i2d.h"



static void reg_bit_set(struct ingenic_i2d_dev* dev,int offset,int bit)
{
	unsigned int val = 0;
	val = reg_read(dev,offset);
	val |= bit;
	reg_write(dev,offset,val);
}

static void reg_bit_clr(struct ingenic_i2d_dev *dev,int offset, int bit)
{
	unsigned int val = 0;
	val = reg_read(dev,offset);
	val &= ~ (bit);
	reg_write(dev,offset,val);
}

void i2d_safe_reset(struct ingenic_i2d_dev *dev)
{
	reg_bit_set(dev,I2D_CTRL,I2D_SAFE_RESET);
}

void i2d_reset(struct ingenic_i2d_dev *dev)
{
	reg_bit_set(dev,I2D_CTRL,I2D_RESET);
}

static int i2d_reg_set(struct ingenic_i2d_dev *i2d, struct i2d_param *i2d_param)
{
    unsigned int fmt = 0;
    unsigned int srcw = 0;
    unsigned int srch = 0;
    unsigned int flip_enable = 0, mirr_enable = 0, rotate_angle = 0, rotate_enable = 0;
    unsigned int flip_mode = 0;
    unsigned int value = 0;

	unsigned int src_y_pbuf = 0;
	unsigned int src_uv_pbuf = 0;
    unsigned int dst_y_pbuf = 0;
    unsigned int dst_uv_pbuf = 0;

    unsigned int src_y_strid = 0;
	unsigned int src_uv_strid = 0;
    unsigned int dst_y_strid = 0;
    unsigned int dst_uv_strid = 0;

    struct i2d_param *ip = i2d_param;
    if (i2d == NULL) {
		dev_err(i2d->dev, "i2d: i2d is NULL or i2d_param is NULL\n");
		return -1;
	}

    fmt = ip->data_type;
    srcw = ip->src_w;
    srch = ip->src_h;
    flip_enable = ip->flip_enable;
    mirr_enable = ip->mirr_enable;
    rotate_enable = ip->rotate_enable;
    rotate_angle = ip->rotate_angle;

    switch(fmt) {
        case V4L2_PIX_FMT_NV12:
            src_y_strid  = srcw;
            src_uv_strid = srcw;
            if ((rotate_angle == 0) || (rotate_angle == 180) || (flip_enable == 1) || (mirr_enable == 1)) {
                dst_y_strid = srcw;
                dst_uv_strid = srcw;
            } else if ((rotate_angle == 90) || (rotate_angle == 270)) {
                dst_y_strid = srch;
                dst_uv_strid = srch;
            }
            reg_write(i2d, I2D_IMG_MODE, I2D_DATA_TYPE_NV12);
            break;
        case V4L2_PIX_FMT_SBGGR8:
            src_y_strid = srcw;
            src_uv_strid = srcw;
            if ((rotate_angle == 0) || (rotate_angle == 180) || (flip_enable == 1) || (mirr_enable == 1)) {
                dst_y_strid = srcw;
                dst_uv_strid = srcw;
            } else if ((rotate_angle == 90) || (rotate_angle == 270)) {
                dst_y_strid = srch;
                dst_uv_strid = srch;
            }
            reg_write(i2d, I2D_IMG_MODE, I2D_DATA_TYPE_RAW8);
            break;
        case V4L2_PIX_FMT_RGB565:
            src_y_strid = srcw << 1;
            src_uv_strid = srcw;
            if ((rotate_angle == 0) || (rotate_angle == 180) || (flip_enable == 1) || (mirr_enable == 1)) {
                dst_y_strid = srcw << 1;
                dst_uv_strid = srcw;
            } else if ((rotate_angle == 90) || (rotate_angle == 270)) {
                dst_y_strid = srch << 1;
                dst_uv_strid = srch;
            }
            reg_write(i2d, I2D_IMG_MODE, I2D_DATA_TYPE_RGB565);
            break;
        case V4L2_PIX_FMT_ARGB32:
            src_y_strid = srcw << 2;
            src_uv_strid = srcw;
            if ((rotate_angle == 0) || (rotate_angle == 180) || (flip_enable == 1) || (mirr_enable == 1)) {
                dst_y_strid = srcw << 2;
                dst_uv_strid = srcw;
            } else if ((rotate_angle == 90) || (rotate_angle == 270)) {
                dst_y_strid = srch << 2;
                dst_uv_strid = srch;
            }
            reg_write(i2d, I2D_IMG_MODE, I2D_DATA_TYPE_ARGB8888);
            break;
        default:
            src_y_strid  = srcw;
            src_uv_strid = srcw;
            if ((rotate_angle == 0) || (rotate_angle == 180) || (flip_enable == 1) || (mirr_enable == 1)) {
                dst_y_strid = srcw;
                dst_uv_strid = srcw;
            } else if ((rotate_angle == 90) || (rotate_angle == 270)) {
                dst_y_strid = srch;
                dst_uv_strid = srch;
            }
			printk("not support fmt ,set default nv12");
            reg_write(i2d, I2D_IMG_MODE, I2D_DATA_TYPE_NV12);
            break;
    }


#ifdef TIMEOUT_TEST
    reg_write(i2d, I2D_TIMEOUT_MODE, (0 << 0)); //timeout mode 0:not restart  1:auto restart
    reg_write(i2d, I2D_TIMEOUT_VALUE, (0x64 << 0)); //timeout value
#endif
    reg_write(i2d, I2D_IMG_SIZE, (srcw << I2D_WIDTH | srch << I2D_HEIGHT));
   // reg_write(i2d, I2D_IMG_MODE, I2D_DATA_TYPE_NV12);
    reg_write(i2d, I2D_IMG_SIZE, (srcw << I2D_WIDTH | srch << I2D_HEIGHT));
    //reg_write(i2d, I2D_IMG_MODE, I2D_FLIP_MODE_90);

    if(rotate_enable == 1) {
        unsigned int value = 0, tmp1 = 0;
		if(rotate_angle == 0){
			tmp1 = 0;
			dst_uv_pbuf = ip->dst_addr_y + dst_y_strid*srch;
		} else if(rotate_angle == 90){
			tmp1 = 1;
			dst_uv_pbuf = ip->dst_addr_y + dst_y_strid*srcw;
		} else if(rotate_angle == 180){
			tmp1 = 2;
			dst_uv_pbuf = ip->dst_addr_y + dst_y_strid*srch;
		} else if(rotate_angle == 270){
			tmp1 = 3;
			dst_uv_pbuf = ip->dst_addr_y + dst_y_strid*srcw;
		} else {
			tmp1 = 0;
            dst_uv_pbuf = ip->dst_addr_y + dst_y_strid*srch;
		}
		/* set rotate angle  */
		value = reg_read(i2d,I2D_IMG_MODE);
		value &= ~(3 << 2);
		value |= (tmp1 << 2);
		reg_write(i2d,I2D_IMG_MODE,value);
	}

     if(mirr_enable == 1) {
        dst_uv_pbuf = ip->dst_addr_y + dst_y_strid*srch;
        value = reg_read(i2d, I2D_IMG_MODE);
        reg_write(i2d, I2D_IMG_MODE, (value | I2D_MIRR));
    }

    if(flip_enable == 1) {
        dst_uv_pbuf = ip->dst_addr_y + dst_y_strid*srch;
        value = reg_read(i2d, I2D_IMG_MODE);
        reg_write(i2d, I2D_IMG_MODE, (value | I2D_FLIP));
    }



    src_y_pbuf = ip->src_addr_y;
    src_uv_pbuf = ip->src_addr_y + src_y_strid*srch;
    dst_y_pbuf = ip->dst_addr_y;

    reg_write(i2d, I2D_SRC_ADDR_Y, src_y_pbuf);
    reg_write(i2d, I2D_SRC_ADDR_UV, src_uv_pbuf);
    reg_write(i2d, I2D_DST_ADDR_Y, dst_y_pbuf);
    reg_write(i2d, I2D_DST_ADDR_UV, dst_uv_pbuf);


    reg_write(i2d, I2D_SRC_Y_STRID, src_y_strid);
    reg_write(i2d, I2D_SRC_UV_STRID, src_uv_strid);
    reg_write(i2d, I2D_DST_Y_STRID, dst_y_strid);
    reg_write(i2d, I2D_DST_UV_STRID, dst_uv_strid);


    return 0;
}

int i2d_set_init_info(struct ingenic_i2d_dev *dev,struct i2d_frame_info *info,dma_addr_t src_addr,dma_addr_t dst_addr)
{
	struct ingenic_i2d_ctx *ctx;
	struct i2d_param param;
	ctx = dev->cur_ctx;
	param.src_w = info->width;
	param.src_h = info->height;
	param.data_type = info->fmt->fourcc;
	param.rotate_enable = 1;
	param.rotate_angle = ctx->angle;
	param.flip_enable = ctx->vflip;
	param.mirr_enable = ctx->hflip;
	param.src_addr_y = src_addr;
	param.dst_addr_y = dst_addr;
	return i2d_reg_set(dev,&param);
}

#if 0
int i2d_set_angle(ingenic_i2d_dev* dev, uint32_t angle)
{

}

int i2d_set_vflip(ingenic_i2d_dev* dev, uint32_t vflip)
{

}

int i2d_set_hflip(ingenic_i2d_dev* dev, uint32_t hflip)
{

}
#endif

int i2d_start(struct ingenic_i2d_dev *dev)
{
	reg_bit_set(dev, I2D_IRQ_MASK, (I2D_IRQ_TIMEOUT_MASK | I2D_IRQ_FRAME_DONE_MASK));	// mask 0
	reg_write(dev,I2D_SHD_CTRL,0xffffffff);
	reg_bit_set(dev, I2D_CTRL, I2D_START);
	return 0;
}


int i2d_irq_clear(struct ingenic_i2d_dev *dev)
{
	unsigned int status;
	status = reg_read(dev,I2D_IRQ_STATE);
	reg_bit_set(dev,I2D_IRQ_CLEAR,status);
	/* if (status & 0x1){ */
	/*     complete(&dev->done_i2d); */
	/* } */
	return status;
}
