#ifndef __VIC_REG_H__
#define __VIC_REG_H__

#define VIC_BASE(n)                                  (0x0 + 0x00000 + n * 0x10000)

//#define VIC_DB_CFG                                  0x10
#define DVP_DATA_POS                                (1<<24)
#define DVP_RGB_ORDER                               (1<<21)
#define DVP_RAW_ALIG                                (1<<20)
#define DVP_DATA_TYPE                               (17)
#define DVP_RAW8                                    (0<<DVP_DATA_TYPE)
#define DVP_RAW10                                   (1<<DVP_DATA_TYPE)
#define DVP_RAW12                                   (2<<DVP_DATA_TYPE)
#define DVP_YUV422_16BIT                            (3<<DVP_DATA_TYPE)
#define DVP_RGB565_16BIT                            (4<<DVP_DATA_TYPE)
#define DVP_BRG565_16BIT                            (5<<DVP_DATA_TYPE)
#define DVP_YUV422_8BIT                             (6<<DVP_DATA_TYPE)
#define DVP_RGB565_8BIT                             (7<<DVP_DATA_TYPE)

#define DVP_TIMEING_MODE                            (1<<15)
#define DVP_SONY_MODE                               (2 << 15)
#define BT_INTF_WIDE                                (1<<11)
#define BT_LINE_MODE                                (1<<10)
#define BT_601_TIMING_MODE                          (1<<9)
#define YUV_DATA_ORDER                              (4)
#define UYVY                                        (0<<YUV_DATA_ORDER)
#define VYUY                                        (1<<YUV_DATA_ORDER)
#define YUYV                                        (2<<YUV_DATA_ORDER)
#define YVYU                                        (3<<YUV_DATA_ORDER)
#define FIRST_FIELD_TYPE                            (1<<3)
#define INTERLACE_EN                                (1<<2)
#define HSYN_POLAR                                  (1<<1)
#define VSYN_POLAR                                  (1<<0)

//#define VIC_HCNT0                                   0x04
//#define VIC_HCNT1                                   0x08


//#define VIC_FRM_ECC                                0x08
#define FRAME_ECC_EN                               (1<<0)
#define FRAME_ECC_MODE                             (1<<1)
//#define VIC_INTF_TYPE                              0x0C
#define INTF_TYPE_BT656                            0x0
#define INTF_TYPE_BT601                            0x1
#define INTF_TYPE_MIPI                             0x2
#define INTF_TYPE_DVP                              0x3
#define INTF_TYPE_BT1120                           0x4

//#define VIC_RESOLUTION                             0x04
#define H_RESOLUTION                               (1<<16)
#define V_RESOLUTION                               (1)


//#define VIC_IDI_TYPE                               (0x14)/* VIC_INPUT_MIPI */
#define MIPI_RAW8                                  0x0
#define MIPI_RAW10                                 0x1
#define MIPI_RAW12                                 0x2
#define MIPI_RGB555                                0x3
#define MIPI_RGB565                                0x4
#define MIPI_RGB666                                0x5
#define MIPI_RGB888                                0x6
#define MIPI_YUV422                                0x7
#define MIPI_YUV422_10BIT                          0x8

//#define VIC_AB_VALUE                               (0x18)
#define A_VALUE                                    (1<<16)
#define B_VALUE                                    (1)

//#define VIC_GLOBAL_CFG                             (0x50)
#define ISP_PRESET_MODE1                           (0<<5)
#define ISP_PRESET_MODE2                           (1<<5)
#define ISP_PRESET_MODE3                           (2<<5)
#define VCKE_EN                                    (1<<4)
#define BLANK_EN                                   (2)
#define AB_MODE_SELECT                             (0)

#//define VIC_CONTROL                                (0x0)
#define VIC_RESET                                  (1<<4)
#define GLB_SAFE_RST                               (1<<3)
#define GLB_RST                                    (1<<2)
#define REG_ENABLE                                 (1<<1)
#define VIC_SRART                                  (1<<0)

#define    HCROP_DIFF_EN                                     25
#define    VCOMP_EN                                          24
#define    HCOMP_EN                                          23
#define    LINE_SYNC_MODE                                    22
#define    WORK_START_FLAG                                   20
#define    DATA_TYPE_EN                                      18
#define    DATA_TYPE_VALUE                                   12
#define    DEL_START                                         8
#define    SENSOR_FRM_NUM                                    4
#define    SENSOR_FID_MODE                                   2
#define    SENSOR_MODE                                       0

#define VIC_DMA_OUTPUT_MAX_WIDTH                   3840



#define VIC_ADDR_VIC_CTRL(n)                                  (VIC_BASE(n) + 0x0000)
#define VIC_ADDR_VIC_RES(n)                                   (VIC_BASE(n) + 0x0004)
#define VIC_ADDR_VIC_FRM_ECC(n)                               (VIC_BASE(n) + 0x0008)
#define VIC_ADDR_VIC_IN_INTF(n)                               (VIC_BASE(n) + 0x000c)
#define VIC_ADDR_VIC_IN_DVP(n)                                (VIC_BASE(n) + 0x0010)
#define VIC_ADDR_VIC_IN_CSI_FMT(n)                            (VIC_BASE(n) + 0x0014)
#define VIC_ADDR_VIC_IN_HOR_PARA0(n)                          (VIC_BASE(n) + 0x0018)
#define VIC_ADDR_VIC_IN_HOR_PARA1(n)                          (VIC_BASE(n) + 0x001c)
#define VIC_ADDR_VIC_IN_VER_PARA0(n)                          (VIC_BASE(n) + 0x0030)
#define VIC_ADDR_VIC_IN_VER_PARA1(n)                          (VIC_BASE(n) + 0x0034)
#define VIC_ADDR_VIC_IN_VER_PARA2(n)                          (VIC_BASE(n) + 0x0038)
#define VIC_ADDR_VIC_IN_VER_PARA3(n)                          (VIC_BASE(n) + 0x003c)
#define VIC_ADDR_VIC_VLD_LINE_SAV(n)                          (VIC_BASE(n) + 0x0060)
#define VIC_ADDR_VIC_VLD_LINE_EAV(n)                          (VIC_BASE(n) + 0x0064)
#define VIC_ADDR_VIC_VLD_FRM_SAV(n)                           (VIC_BASE(n) + 0x0070)
#define VIC_ADDR_VIC_VLD_FRM_EAV(n)                           (VIC_BASE(n) + 0x0074)
#define VIC_ADDR_VIC_VC_CONTROL(n)                            (VIC_BASE(n) + 0x008c)
#define VIC_ADDR_VIC_VC_CONTROL_CH0_PIX(n)                    (VIC_BASE(n) + 0x0090)
#define VIC_ADDR_VIC_VC_CONTROL_CH1_PIX(n)                    (VIC_BASE(n) + 0x0094)
#define VIC_ADDR_VIC_VC_CONTROL_CH2_PIX(n)                    (VIC_BASE(n) + 0x0098)
#define VIC_ADDR_VIC_VC_CONTROL_CH3_PIX(n)                    (VIC_BASE(n) + 0x009c)
#define VIC_ADDR_VIC_VC_CONTROL_CH0_LINE(n)                   (VIC_BASE(n) + 0x00a0)
#define VIC_ADDR_VIC_VC_CONTROL_CH1_LINE(n)                   (VIC_BASE(n) + 0x00a4)
#define VIC_ADDR_VIC_VC_CONTROL_CH2_LINE(n)                   (VIC_BASE(n) + 0x00a8)
#define VIC_ADDR_VIC_VC_CONTROL_CH3_LINE(n)                   (VIC_BASE(n) + 0x00ac)
#define VIC_ADDR_VIC_VC_CONTROL_FIFO_USE(n)                   (VIC_BASE(n) + 0x00b0)
#define VIC_ADDR_MIPI_ALL_WIDTH_4BYTE(n)                      (VIC_BASE(n) + 0x0100)
#define VIC_ADDR_MIPI_VCROP_DEL01(n)                          (VIC_BASE(n) + 0x0104)
#define VIC_ADDR_MIPI_VCROP_DEL23(n)                          (VIC_BASE(n) + 0x0108)
#define VIC_ADDR_MIPI_SENSOR_CONTROL(n)                       (VIC_BASE(n) + 0x010c)
#define VIC_ADDR_MIPI_HCROP_CH0(n)                            (VIC_BASE(n) + 0x0110)
#define VIC_ADDR_MIPI_HCROP_CH1(n)                            (VIC_BASE(n) + 0x0114)
#define VIC_ADDR_MIPI_HCROP_CH2(n)                            (VIC_BASE(n) + 0x0118)
#define VIC_ADDR_MIPI_HCROP_CH3(n)                            (VIC_BASE(n) + 0x011c)
#define VIC_ADDR_VC_CONTROL_CONTROL(n)                        (VIC_BASE(n) + 0x01a0)
#define VIC_ADDR_VC_CONTROL_DELEY(n)                          (VIC_BASE(n) + 0x01a4)
#define VIC_ADDR_VC_CONTROL_TIZIANO_ROUTE(n)                  (VIC_BASE(n) + 0x01a8)
#define VIC_ADDR_VC_CONTROL_DMAOUT_ROUTE(n)                   (VIC_BASE(n) + 0x01ac)
#define VIC_ADDR_VC_CONTROL_DELEY_BLK(n)                      (VIC_BASE(n) + 0x01b0)
#define VIC_ADDR_VC_CONTROL_LIMIT(n)                          (VIC_BASE(n) + 0x01b4)
#define VIC_ADDR_VIC_INT_STATU(n)                             (VIC_BASE(n) + 0x01e0)
#define VIC_ADDR_VIC_INT_STATU2(n)                            (VIC_BASE(n) + 0x01e4)
#define VIC_ADDR_VIC_INT_MASK(n)                              (VIC_BASE(n) + 0x01e8)
#define VIC_ADDR_VIC_INT_MASK2(n)                             (VIC_BASE(n) + 0x01ec)
#define VIC_ADDR_VIC_INT_CLR(n)                               (VIC_BASE(n) + 0x01f0)
#define VIC_ADDR_VIC_INT_CLR2(n)                              (VIC_BASE(n) + 0x01f4)
#define VIC_ADDR_DMA_CONFIGURE(n)                             (VIC_BASE(n) + 0x0300)
#define VIC_ADDR_DMA_RESOLUTION(n)                            (VIC_BASE(n) + 0x0304)
#define VIC_ADDR_DMA_RESET(n)                                 (VIC_BASE(n) + 0x0308)
#define VIC_ADDR_DMA_Y_CH_STRIDE(n)                           (VIC_BASE(n) + 0x0310)
#define VIC_ADDR_DMA_UV_CH_STRIDE(n)                          (VIC_BASE(n) + 0x0314)
#define VIC_ADDR_DMA_Y_CH0_BANK0_ADDR(n)                      (VIC_BASE(n) + 0x0318)
#define VIC_ADDR_DMA_Y_CH0_BANK1_ADDR(n)                      (VIC_BASE(n) + 0x031c)
#define VIC_ADDR_DMA_Y_CH0_BANK2_ADDR(n)                      (VIC_BASE(n) + 0x0320)
#define VIC_ADDR_DMA_Y_CH0_BANK3_ADDR(n)                      (VIC_BASE(n) + 0x0324)
#define VIC_ADDR_DMA_Y_CH0_BANK4_ADDR(n)                      (VIC_BASE(n) + 0x0328)
#define VIC_ADDR_DMA_UV_CH0_BANK0_ADDR(n)                     (VIC_BASE(n) + 0x032c)
#define VIC_ADDR_DMA_UV_CH0_BANK1_ADDR(n)                     (VIC_BASE(n) + 0x0330)
#define VIC_ADDR_DMA_UV_CH0_BANK2_ADDR(n)                     (VIC_BASE(n) + 0x0334)
#define VIC_ADDR_DMA_UV_CH0_BANK3_ADDR(n)                     (VIC_BASE(n) + 0x0338)
#define VIC_ADDR_DMA_UV_CH0_BANK4_ADDR(n)                     (VIC_BASE(n) + 0x033c)
#define VIC_ADDR_DMA_Y_CH1_BANK0_ADDR(n)                      (VIC_BASE(n) + 0x0340)
#define VIC_ADDR_DMA_Y_CH1_BANK1_ADDR(n)                      (VIC_BASE(n) + 0x0344)
#define VIC_ADDR_DMA_Y_CH1_BANK2_ADDR(n)                      (VIC_BASE(n) + 0x0348)
#define VIC_ADDR_DMA_Y_CH1_BANK3_ADDR(n)                      (VIC_BASE(n) + 0x034c)
#define VIC_ADDR_DMA_Y_CH1_BANK4_ADDR(n)                      (VIC_BASE(n) + 0x0350)
#define VIC_ADDR_DMA_UV_CH1_BANK0_ADDR(n)                     (VIC_BASE(n) + 0x0354)
#define VIC_ADDR_DMA_UV_CH1_BANK1_ADDR(n)                     (VIC_BASE(n) + 0x0358)
#define VIC_ADDR_DMA_UV_CH1_BANK2_ADDR(n)                     (VIC_BASE(n) + 0x035c)
#define VIC_ADDR_DMA_UV_CH1_BANK3_ADDR(n)                     (VIC_BASE(n) + 0x0360)
#define VIC_ADDR_DMA_UV_CH1_BANK4_ADDR(n)                     (VIC_BASE(n) + 0x0364)
#define VIC_ADDR_DMA_Y_CH0_ADDR(n)                            (VIC_BASE(n) + 0x0380)
#define VIC_ADDR_DMA_UV_CH0_ADDR(n)                           (VIC_BASE(n) + 0x0384)
#define VIC_ADDR_DMA_Y_CH1_ADDR(n)                            (VIC_BASE(n) + 0x0388)
#define VIC_ADDR_DMA_UV_CH1_ADDR(n)                           (VIC_BASE(n) + 0x038c)
#define VIC_ADDR_VC_CONTROL_INPUT_MUX_OUTROUTE(n)             (VIC_BASE(n) + 0x01c0)

#define INPUT_MXU_MIPI_EN                                     (1 << 0)
#define INPUT_MXU_OUT                                         (1 << 1)
#define INPUT_MXU_DVP_EN                                      (1 << 4)


#endif
