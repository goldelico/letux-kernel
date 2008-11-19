 /*----------------------------------------------------------
  *   (C) 2004 Samsung Electronics
  *   SW.LEE < hitchcar@samsung.com>
  * 
  ----------------------------------------------------------- */

#ifndef __FIMC20_CAMERA_H__
#define __FIMC20_CAMERA_H__

extern u32 * camregs;

#ifdef CONFIG_ARCH_S3C24A0
#define CAM_BASE_ADD             0x48000000
#else /* S3C2440A */
#define CAM_BASE_ADD             0x4F000000
#endif

#if ! defined(FExtr)
#define UData(Data)     ((unsigned long) (Data))
#define FExtr(Data, Field) \
                        ((UData (Data) >> FShft (Field)) & FAlnMsk (Field))
#define FInsrt(Value, Field) \
                        (UData (Value) << FShft (Field))
#define FSize(Field)    ((Field) >> 16)
#define FShft(Field)    ((Field) & 0x0000FFFF)
#define FMsk(Field)     (((UData (1) << FSize (Field)) - 1) << FShft (Field))
#define FAlnMsk(Field)  ((UData (1) << FSize (Field)) - 1)
#define F1stBit(Field)  (UData (1) << FShft (Field))
#define Fld(Size, Shft) (((Size) << 16) + (Shft))
#endif

/*
 * CAMERA IP
 * P-port is used as RGB Capturing device which including scale and crop
 *  those who want to see(preview ) the image on display needs RGB image.
 *
 * C-port is used as YCbCr(4:2:0, 4:2:2) Capturing device which including the scale and crop
 *   the prefix of C-port have the meaning of "Codec" ex. mpeg4, h263.. which requries the
     YCBCB format not RGB 
 */ 

#define S3C2440_CAM_REG_CISRCFMT            (0x00) // RW Input Source Format
#define S3C2440_CAM_REG_CIWDOFST            (0x04) // Window offset register
#define S3C2440_CAM_REG_CIGCTRL             (0x08) // Global control register
#define S3C2440_CAM_REG_CICOYSA0            (0x18) // Y 1 st frame start ads
#define S3C2440_CAM_REG_CICOYSA1            (0x1C) // Y 2 nd frame start ads
#define S3C2440_CAM_REG_CICOYSA2            (0x20) // Y 3 rd frame start ads
#define S3C2440_CAM_REG_CICOYSA3            (0x24) // Y 4 th frame start ads
#define S3C2440_CAM_REG_CICOCBSA0           (0x28) // Cb 1 st frame start ads
#define S3C2440_CAM_REG_CICOCBSA1           (0x2C) // Cb 2 nd frame start ads
#define S3C2440_CAM_REG_CICOCBSA2           (0x30) // Cb 3 rd frame start ads
#define S3C2440_CAM_REG_CICOCBSA3           (0x34) // Cb 4 th frame start ads
#define S3C2440_CAM_REG_CICOCRSA0           (0x38) // Cr 1 st frame start ads
#define S3C2440_CAM_REG_CICOCRSA1           (0x3C) // Cr 2 nd frame start ads
#define S3C2440_CAM_REG_CICOCRSA2           (0x40) // Cr 3 rd frame start ads
#define S3C2440_CAM_REG_CICOCRSA3           (0x44) // Cr 4 th frame start ads
#define S3C2440_CAM_REG_CICOTRGFMT          (0x48) // Target img format of codec
#define S3C2440_CAM_REG_CICOCTRL            (0x4C) // Codec DMA control related
#define S3C2440_CAM_REG_CICOSCPRERATIO      (0x50) // Codec pre-scaler ratio
#define S3C2440_CAM_REG_CICOSCPREDST        (0x54) // Codec pre-scaler dest
#define S3C2440_CAM_REG_CICOSCCTRL          (0x58) // Codec main-scaler control
#define S3C2440_CAM_REG_CICOTAREA           (0x5C) // Codec pre-scaler dest
#define S3C2440_CAM_REG_CICOSTATUS          (0x64) // Codec path status
#define S3C2440_CAM_REG_CIPRCLRSA0          (0x6C) // RGB 1 st frame start ads
#define S3C2440_CAM_REG_CIPRCLRSA1          (0x70) // RGB 2 nd frame start ads
#define S3C2440_CAM_REG_CIPRCLRSA2          (0x74) // RGB 3 rd frame start ads
#define S3C2440_CAM_REG_CIPRCLRSA3          (0x78) // RGB 4 th frame start ads
#define S3C2440_CAM_REG_CIPRTRGFMT          (0x7C) // Target img fmt of preview
#define S3C2440_CAM_REG_CIPRCTRL            (0x80) // Preview DMA ctl related
#define S3C2440_CAM_REG_CIPRSCPRERATIO      (0x84) // Preview pre-scaler ratio
#define S3C2440_CAM_REG_CIPRSCPREDST        (0x88) // Preview pre-scaler dest
#define S3C2440_CAM_REG_CIPRSCCTRL          (0x8C) // Preview main-scaler ctl
#define S3C2440_CAM_REG_CIPRTAREA           (0x90) // Preview pre-scaler dest
#define S3C2440_CAM_REG_CIPRSTATUS          (0x98) // Preview path status
#define S3C2440_CAM_REG_CIIMGCPT            (0xA0) // Image capture enable cmd

#define S3C2440_CAM_REG_CICOYSA(__x)        (0x18 + (__x)*4 )
#define S3C2440_CAM_REG_CICOCBSA(__x)       (0x28 + (__x)*4 )
#define S3C2440_CAM_REG_CICOCRSA(__x)       (0x38 + (__x)*4 )
#define S3C2440_CAM_REG_CIPRCLRSA(__x)      (0x6C + (__x)*4 )

/* CISRCFMT BitField */
#define SRCFMT_ITU601       BIT31
#define SRCFMT_ITU656       0
#define SRCFMT_UVOFFSET_128 BIT30
#define fCAM_SIZE_H         Fld(13, 16)
#define fCAM_SIZE_V         Fld(13, 0)
#define SOURCE_HSIZE(x)     FInsrt((x), fCAM_SIZE_H)
#define SOURCE_VSIZE(x)     FInsrt((x), fCAM_SIZE_V)


/* Window Option Register */
#define WINOFEN             BIT31
#define CO_FIFO_Y           BIT30
#define CO_FIFO_CB          BIT15
#define CO_FIFO_CR          BIT14
#define PR_FIFO_CB          BIT13
#define PR_FIFO_CR          BIT12
#define fWINHOR             Fld(11, 16)
#define fWINVER             Fld(11, 0)
#define WINHOROFST(x)       FInsrt((x), fWINHOR)
#define WINVEROFST(x)       FInsrt((x), fWINVER)

/* Global Control Register */
#define GC_SWRST            BIT31
#define GC_CAMRST           BIT30
#define GC_INVPOLPCLK       BIT26
#define GC_INVPOLVSYNC      BIT25
#define GC_INVPOLHREF       BIT24

/*--------------------------------------------------
                REGISTER BIT FIELD DEFINITION TO 
                          YCBCR and RGB 
----------------------------------------------------*/
/* Codec Target Format Register */
#define IN_YCBCR420         0
#define IN_YCBCR422         BIT31
#define OUT_YCBCR420        0
#define OUT_YCBCR422        BIT30

#if 0
#define FLIP_NORMAL	    0
#define FLIP_X              (BIT14)
#define FLIP_Y              (BIT15)
#define FLIP_MIRROR         (BIT14|BIT15)
#endif

/**  BEGIN    ************************************/
/* Cotents: Common in both P and C port          */
#define fTARGET_HSIZE       Fld(13,16)
#define TARGET_HSIZE(x)     FInsrt((x), fTARGET_HSIZE)
#define fTARGET_VSIZE       Fld(13,0)
#define TARGET_VSIZE(x)     FInsrt((x), fTARGET_VSIZE)
#define FLIP_X_MIRROR       BIT14           
#define FLIP_Y_MIRROR       BIT15           
#define FLIP_180_MIRROR     (BIT14 | BIT15) 
/**  END     *************************************/

/* Codec DMA Control Register */
#define fYBURST_M           Fld(5,19)
#define fYBURST_R           Fld(5,14)
#define fCBURST_M           Fld(5,9)
#define fCBURST_R           Fld(5,4)
#define YBURST_M(x)         FInsrt((x), fYBURST_M)
#define CBURST_M(x)         FInsrt((x), fCBURST_M)
#define YBURST_R(x)         FInsrt((x), fYBURST_R)
#define CBURST_R(x)         FInsrt((x), fCBURST_R)
#define LAST_IRQ_EN         BIT2              /* Common in both P and C port */
/*
 *  Check the done signal of capturing image for JPEG 
 *  !!! AutoClear Bit 
 */


/* (Codec, Preview ) Pre-Scaler Control Register 1 */
#define fSHIFT              Fld(4,28)
#define PRE_SHIFT(x)        FInsrt((x), fSHIFT)
#define fRATIO_H            Fld(7,16)
#define PRE_HRATIO(x)       FInsrt((x), fRATIO_H)
#define fRATIO_V            Fld(7,0)
#define PRE_VRATIO(x)       FInsrt((x), fRATIO_V)

/* (Codec, Preview ) Pre-Scaler Control Register 2*/
#define fDST_WIDTH          Fld(12,16)
#define fDST_HEIGHT         Fld(12,0)
#define PRE_DST_WIDTH(x)    FInsrt((x), fDST_WIDTH)
#define PRE_DST_HEIGHT(x)   FInsrt((x), fDST_HEIGHT)


/* (Codec, Preview) Main-scaler control Register */
#define S_METHOD            BIT31  /* Sampling method only for P-port */
#define SCALERSTART         BIT15 
/* Codec scaler bypass for upper 2048x2048 
   where ImgCptEn_CoSC and ImgCptEn_PrSC should be 0 
*/

#define SCALERBYPASS        BIT31  
#define RGB_FMT24           BIT30
#define RGB_FMT16           0

/*
#define SCALE_UP_H          BIT29
#define SCALE_UP_V          BIT28
*/

#define fMAIN_HRATIO        Fld(9, 16)
#define MAIN_HRATIO(x)      FInsrt((x), fMAIN_HRATIO)

#define SCALER_START        BIT15

#define fMAIN_VRATIO        Fld(9, 0)
#define MAIN_VRATIO(x)      FInsrt((x), fMAIN_VRATIO)

/* (Codec, Preview ) DMA Target AREA Register */
#define fCICOTAREA          Fld(26,0)
#define TARGET_DMA_AREA(x)  FInsrt((x), fCICOTAREA)

/* Preview DMA Control Register */
#define fRGBURST_M          Fld(5,19)
#define fRGBURST_R          Fld(5,14)
#define RGBURST_M(x)        FInsrt((x), fRGBURST_M)
#define RGBURST_R(x)        FInsrt((x), fRGBURST_R)


/* (Codec, Preview) Status Register */
#define CO_OVERFLOW_Y       BIT31
#define CO_OVERFLOW_CB      BIT30
#define CO_OVERFLOW_CR      BIT29
#define PR_OVERFLOW_CB      BIT31
#define PR_OVERFLOW_CR      BIT30

#define VSYNC               BIT28

#define fFRAME_CNT          Fld(2,26)
#define FRAME_CNT(x)        FExtr((x),fFRAME_CNT) 
           
#define WIN_OFF_EN          BIT25
#define fFLIP_MODE          Fld(2,23)
#define FLIP_MODE(x)        EExtr((x), fFLIP_MODE)
#define CAP_STATUS_CAMIF    BIT22
#define CAP_STATUS_CODEC    BIT21
#define CAP_STATUS_PREVIEW  BIT21
#define VSYNC_A             BIT20
#define VSYNC_B             BIT19

/* Image Capture Enable Regiser */
#define	CAMIF_CAP_ON         BIT31
#define CAMIF_CAP_CODEC_ON   BIT30
#define CAMIF_CAP_PREVIEW_ON BIT29




#endif /* S3C2440_CAMER_H */
