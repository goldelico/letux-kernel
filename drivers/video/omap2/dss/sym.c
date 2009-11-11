//
// Texas Instruments OMAP(tm) Platform Software.
// Copyright 2004 by Texas Instruments Incorporated. All rights
// reserved. Property of Texas Instruments Incorporated. Restricted
// rights to use, duplicate or disclose this code are granted
// through contract.

/**
* Defintion of OMAP4 DSS Registers
*
*/

/**
* @addtogroup omap_shared OMAP ASSP
* @ingroup omap_silicon
* @{
*/

#include <linux/kernel.h>
#include <linux/io.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/interrupt.h>

extern void __iomem  *dss_base;
extern void __iomem  *dispc_base;
extern void __iomem  *dsi_base;

/********************************************************************/
/* Include files                                                    */


//extern static unsigned int KSetNone;
static unsigned int KSetAll = 0xffffffff;
static unsigned int KClearNone = 0x00000000;
static unsigned int KClearAll = 0xffffffff;

static unsigned int KHwBaseDss						= 0x58000000; //L3 physical base address

/************************************* 1) DSS_REVISION REGISTER. OFFSET : 0x00 ****************************************/

static unsigned int KHoDSS_RevisionNumber   = 0x00 ;


/************************************* 2) DSS_CONTROL REGISTER. OFFSET : 0x40 ****************************************/

static unsigned int KHoDSS_Control          = 0x40 ;
	static unsigned int KHtDSS_Control_VENC_HDMI_SWITCH		= 1<<15; //0 - VENC 1- HDMI
	static unsigned int KHtDSS_Control_RFBI_SWITCH			= 1<<14; //sync generator for SD Video 0 - Internal IP 1- Component VENC IP
	static unsigned int KHtDSS_Control_SYNC_SWITCH			= 1<<13; //DISPC LCD2_CLK clock 0 - PRCM 1- PLL2
	static unsigned int KHtDSS_Control_LCD2_CLK_SWITCH		= 1<<12; //DISPC LCD2_CLK clock 0 - PRCM 1- PLL2
	static unsigned int KHtDSS_Control_TV_CLK_SWITCH		= 1<<11; //VENC/HDMI fclock 0 - HDMIPLL 1- TV_CLK from pad
	static unsigned int KHtDSS_Control_DSI2_CLK_SWITCH		= 1<<10; //DSI2_CLK 0 - PRCM 1- DSI2 PLL
	static unsigned int KHsDSS_Control_FCK_CLK_SWITCH		= 8 ;
		static unsigned int KHtDSS_Control_FCK_CLK_PRCM		= 0x0; //DSS_CLK selected (from PRCM)
		static unsigned int KHtDSS_Control_FCK_CLK_DSI1PLL	= 0x1; //PLL1_CLK1 selected (from DSI1 PLL)
		static unsigned int KHtDSS_Control_FCK_CLK_DSI2PLL	= 0x2; //PLL2_CLK1 selected (from DSI2 PLL)
		static unsigned int KHtDSS_Control_FCK_CLK_HDMIPLL	= 0x3; //PLL3_CLK1 selected (from HDMI PLL)
	static unsigned int KHtDSS_Control_VENCOutSel           = 1<<6 ; // 0 for composite;1 for S-Video
	static unsigned int KHtDSS_Control_LCDDAC_PWRDNBG       = 1<<5 ; // Power down Band GAP
	static unsigned int KHtDSS_Control_DAC_DEMEN            = 1<<4 ; // DAC Dynamic Element Matching Enable
	static unsigned int KHtDSS_Control_VENC_Clock_4x_Enable = 1<<3 ; // VENC clock 4x enable
	static unsigned int KHtDSS_Control_VENC_Clock_Mode      = 1<<2 ; // VENC clock mode (0 for normal; 1 for square pixel
	static unsigned int KHtDSS_DPLL_APLL_Clk                = 1<<0 ; // DPLL/APLL clock switch(0 for DPLL; 1 for APLL)

/************************************* 3) DSS_SYSCONFIG REGISTER. OFFSET : 0x10 ****************************************/

static unsigned int KHoDSS_SysConfig = 0x10;

	static unsigned int KHtDSSAutoIdle = 1<<0;

/************************************* 4) DSS_SYSSTATUS REGISTER. OFFSET : 0x5C ****************************************/

static unsigned int KHoDSS_Status           = 0x5C ;

	static unsigned int KHtDSS_Status_DPLL_enable = 1<<1 ; // DPLL clock active
	static unsigned int KHtDSS_Status_APLL_enable = 1<<0 ; // APLL clock active


/************************************* DISPLAY CONTROLLER SPECIFIC REGISTER MAPPINGS ****************************************/

// Display subsystem controller (Physical 0x58001000)
static unsigned int KHwBaseDisplayCtrlReg       = 0x58000000 + 0x1000;

/************************************* 1) DISPC_REVISION REGISTER. OFFSET : 0x00 ****************************************/

/// contains the IP revision code
static unsigned int KHoDISP_RevNumber   = 0x00 ;

/************************************* 2) DISPC_SYSCONFIG REGISTER. OFFSET : 0x10 ****************************************/

/// allows to control various parameters of the OCP interface
static unsigned int KHoDISP_SYSCONFIG   = 0x10 ;

	static unsigned int KHoDISP_SYSCONFIGSmartStandby = 1<<13 ;
	static unsigned int KHoDISP_SYSCONFIGNoStandby    = 1<<12 ;
	static unsigned int KHoDISP_SYSCONFIGWarmReset    = 1<<5  ;
	static unsigned int KHoDISP_SYSCONFIGSmartIdle    = 1<<4  ;
	static unsigned int KHoDISP_SYSCONFIGNoIdle       = 1<<3  ;
	static unsigned int KHoDISP_SYSCONFIGWakeUpEn     = 1<<2  ;
	static unsigned int KHoDISP_SYSCONFIGSoftReset    = 1<<1  ;
	static unsigned int KHoDISP_SYSCONFIGAutoIdle     = 1<<0  ;

/************************************* 3) DISPC_SYSSTATUS REGISTER. OFFSET : 0x14 ****************************************/

// Contains DISPC System status
static unsigned int KHoDISP_SYSSTATUS   = 0x14 ;

	static unsigned int KHoDISP_SYSSTATUSResetDone    = 1<<0  ;

/************************************* 4) DISPC_IRQSTATUS REGISTER. OFFSET : 0x18 ****************************************/

// Contains IRQ Status of DISPC
static unsigned int KHoDISP_IRQSTATUS = 0x18 ;

	/// WriteBackFifoOverFlow
	static unsigned int KHtDISP_IRQSTATUSWbFifoOFlow  			= 1<<25 ;
	///  Frame Done IRQ (TV)
	static unsigned int KHtDISP_IRQSTATUSFrameDoneTv      		= 1<<24 ;
	///  Frame Done IRQ (WB)
	static unsigned int KHtDISP_IRQSTATUSFrameDoneWb      		= 1<<23 ;
	///  Frame Done IRQ (LCD2)
	static unsigned int KHtDISP_IRQSTATUSFrameDone2      		= 1<<22 ;
	/// AcBias Count Status (Secondar LCD)
	static unsigned int KHtDISP_IRQSTATUSAcBiasCntStat2  		= 1<<21 ;
	/// Vid3FifoUnderFlow
	static unsigned int KHtDISP_IRQSTATUSVid3FifoUFlow  		= 1<<20 ;
	/// Vid3EndWindow
	static unsigned int KHtDISP_IRQSTATUSVid3EndWind    		= 1<<19 ;
	/// Vsync 2
	static unsigned int KHtDISP_IRQSTATUSVsync2          		= 1<<18 ;
	// Secondary LCD Sunc Lost
	static unsigned int KHtDISP_IRQSTATUSSyncLost2       		= 1<<17 ;
	// Wakeup IRQ
	static unsigned int KHtDISP_IRQSTATUSWakeUp        			= 1<<16 ;
	// Digital Sync Lost
	static unsigned int KHtDISP_IRQSTATUSDigitalSyncLost        = 1<<15 ;
	// Primary LCD Sync Lost
	static unsigned int KHtDISP_IRQSTATUSSyncLost       		= 1<<14 ;
	/// Vid2EndWindow
	static unsigned int KHtDISP_IRQSTATUSVid2EndWind    		= 1<<13 ;
	/// Vid2FifoUnderFlow
	static unsigned int KHtDISP_IRQSTATUSVid2FifoUFlow  		= 1<<12 ;
	/// Vid1EndWindow
	static unsigned int KHtDISP_IRQSTATUSVid1EndWind    		= 1<<11 ;
	/// Vid1FifoUnderFlow
	static unsigned int KHtDISP_IRQSTATUSVid1FifoUFlow  		= 1<<10 ;
	// OCR Error
	static unsigned int KHtDISP_IRQSTATUSOcpError       		= 1<<9 ;
	/// Palette Gamma Loading
	static unsigned int KHtDISP_IRQSTATUSPalGamLoad     		= 1<<8 ;
	/// Graphics End window
	static unsigned int KHtDISP_IRQSTATUSGfxEndWin      		= 1<<7 ;
	/// Graphics Fifo Under Flow
	static unsigned int KHtDISP_IRQSTATUSGfxFifoUFlow   		= 1<<6 ;
	/// Programmed Line Number
	static unsigned int KHtDISP_IRQSTATUSPgmdLnNo       		= 1<<5 ;
	/// AcBias Count Status
	static unsigned int KHtDISP_IRQSTATUSAcBiasCntStat1  		= 1<<4 ;
	/// EVSYSNC_ODD
	static unsigned int KHtDISP_IRQSTATUSEvSyncOdd      		= 1<<3 ;
	/// EVSYNC_EVEN
	static unsigned int KHtDISP_IRQSTATUSEvSyncEven     		= 1<<2 ;
	/// Vsync
	static unsigned int KHtDISP_IRQSTATUSVsync          		= 1<<1 ;
	///  Frame Done IRQ (LCD1)
	static unsigned int KHtDISP_IRQSTATUSFrameDone      		= 1<<0 ;

/************************************* 5) DISPC_IRQENABLE REGISTER. OFFSET : 0x1C ****************************************/

/// allows to mask/unmask the module internal sources of interrupt, on an
/// event-by-event basis
static unsigned int KHoDISP_IRQEN   = 0x1C ;

	/// WriteBackFifoOverFlow
	static unsigned int KHtDISP_IRQWbFifoOFlowEn  				= 1<<25 ;
	///  Frame Done IRQ (TV)
	static unsigned int KHtDISP_IRQFrameDoneTvEn      			= 1<<24 ;
	///  Frame Done IRQ (WB)
	static unsigned int KHtDISP_IRQFrameDoneWbEn      			= 1<<23 ;
	///  Frame Done IRQ (LCD2)
	static unsigned int KHtDISP_IRQFrameDone2En      			= 1<<22 ;
	/// AcBias Count Status (Secondar LCD)
	static unsigned int KHtDISP_IRQAcBiasCntStat2En  			= 1<<21 ;
	/// Vid3FifoUnderFlow
	static unsigned int KHtDISP_IRQVid3FifoUFlowEn  			= 1<<20 ;
	/// Vid3EndWindow
	static unsigned int KHtDISP_IRQVid3EndWindEn    			= 1<<19 ;
	/// Vsync 2
	static unsigned int KHtDISP_IRQVsync2En          			= 1<<18 ;
	// Secondary LCD Sunc Lost
	static unsigned int KHtDISP_IRQSyncLost2En       			= 1<<17 ;
	// Wakeup IRQ
	static unsigned int KHtDISP_IRQWakeUpEn        				= 1<<16 ;
	// Digital Sync Lost
	static unsigned int KHtDISP_IRQDigitalSyncLostEn        	= 1<<15 ;
	/// Primary LCD Sync Lost
	static unsigned int KHtDISP_IRQSyncLost1En         			= 1<<14 ;
	/// Vid2EndWindow
	static unsigned int KHtDISP_IRQVid2EndWindEn       			= 1<<13 ;
	/// Vid2FifoUnderFlow
	static unsigned int KHtDISP_IRQVid2FifoUFlowEn     			= 1<<12 ;
	/// Vid1EndWindow
	static unsigned int KHtDISP_IRQVid1EndWindEn       			= 1<<11 ;
	/// Vid1FifoUnderFlow
	static unsigned int KHtDISP_IRQVid1FifoUFlowEn     			= 1<<10 ;
	// OCP Error
	static unsigned int KHtDISP_IRQOcpErrorEn          			= 1<<9 ;
	/// Palette Gamma Loading
	static unsigned int KHtDISP_IRQPalGamLoadEn        			= 1<<8 ;
	/// Graphics End window
	static unsigned int KHtDISP_IRQGfxEndWinEn         			= 1<<7 ;
	/// Graphics Fifo Under Flow
	static unsigned int KHtDISP_IRQGfxFifoUFlowEn      			= 1<<6 ;
	/// Programmed Line Number
	static unsigned int KHtDISP_IRQPgmdLnNoEn          			= 1<<5 ;
	/// AcBias Count Status
	static unsigned int KHtDISP_IRQAcBiasCntStat1En    			= 1<<4 ;
	/// EVSYSNC_ODD
	static unsigned int KHtDISP_IRQEvSyncOddEn         			= 1<<3 ;
	/// EVSYNC_EVEN
	static unsigned int KHtDISP_IRQEvSyncEvenEn        			= 1<<2 ;
	/// Vsync
	static unsigned int KHtDISP_IRQVsyncEn            			= 1<<1 ;
	///  Frame Done IRQ (LCD1)
	static unsigned int KHtDISP_IRQFrameDoneEn         			= 1<<0 ;

/************************************* 6) DISPC_CONTROL1 REGISTER. OFFSET : 0x40 ****************************************/

/// allows to configure the display controller module.
static unsigned int KHoDISP_CONTROL     						= 0x40 ;

	/// Spatial/Temporal Dithering No of Frames
	static unsigned int KHsDISPC_STDitheringFrames      		= 30 ;
	/// State of unused bits (TDM mode only)
	static unsigned int KHsDISPC_TDMUNUSEDBITS      			= 25 ;
	/// Cycle format (TDM mode only)
	static unsigned int KHsDISPC_TDMCYCLEFORMAT     			= 23 ;
	/// Output Interface width (TDM mode only)
	static unsigned int KHsDISPC_TDMPARALLELMODE    			= 21 ;
	/// bit shift for hold time for digital output
	static unsigned int KHsDISPC_HOLDTIME           			= 17 ;
	/// Enable the multiple cycle format
	static unsigned int KHtDISP_CONTROL_TDMEn           		= 1<<20 ;
	/// General Purpose Output 1 signal set
	static unsigned int KHtDISP_CONTROL_TFTGPOut1       		= 1<<16 ;
	/// General Purpose Output 0 signal set
	static unsigned int KHtDISP_CONTROL_TFTGPOut0       		= 1<<15 ;
	/// General Purpose input 1 signal set
	static unsigned int KHtDISP_CONTROL_GPIn1           		= 1<<14 ;
	/// General Purpose input 0 signal set
	static unsigned int KHtDISP_CONTROL_GPIn0           		= 1<<13 ;
	/// Overlay Optimisation
	static unsigned int KHtDISP_CONTROL_OverLayOpt      		= 1<<12 ;
	/// RFBI Mode Select
	static unsigned int KHtDISP_CONTROL_RFBIMODE        		= 1<<11 ;
	/// Secure bit set/reset by secure request only
	static unsigned int KHtDISP_CONTROL_Secure          		= 1<<10 ;
	/// 16 bits output for the LCD
	static unsigned int KHtDISP_CONTROL_TFTDataLines_16b 		= 1<<8 ;
	//18 bits output for the LCD
	static unsigned int KHtDISP_CONTROL_TFTDataLines_18b 		=  1<<9 ;
	//24 bits output for the LCD
	static unsigned int KHtDISP_CONTROL_TFTDataLines_24b 		= ( 1<<9 | 1<<8 );
	//Clear mask - corresponds to 12 Bits output for LCD
	static unsigned int KHtDISP_CONTROL_TFTDataLinesClearMask 	= ( 1<<9 | 1<<8 );
	/// Spatial/Temporal dithering enable
	static unsigned int KHtDISP_CONTROL_STDitheringEn   		= 1<<7 ;
	/// Digital GO Command
	static unsigned int KHtDISP_CONTROL_GoDigitalUpdate 		= 1<<6 ;
	/// LCD Go command update
	static unsigned int KHtDISP_CONTROL_GoLCDUpdate     		= 1<<5 ;
	/// Mono 8-bit mode
	static unsigned int KHtDISP_CONTROL_M8BEn           		= 1<<4 ;
	/// TFT LCD display Type
	static unsigned int KHtDISP_CONTROL_TFTEn           		= 1<<3 ;
	/// Monochrome Operation Enable
	static unsigned int KHtDISP_CONTROL_MONOCHROMEEn    		= 1<<2 ;
	/// Digital output enable
	static unsigned int KHtDISP_CONTROL_DigitalEn       		= 1<<1 ;
	/// LCD output enable
	static unsigned int KHtDISP_CTRL_CTRL_LcdEn         		= 1<<0 ;

/************************************* 7) DISPC_CONFIG1 REGISTER. OFFSET : 0x44 ****************************************/

/// allows to configure the display controller module. Shadow
/// register, updated on VFP start period or EVSYNC.
static unsigned int KHoDISP_CONFIG      = 0x44 ;

	/// Full Range for CSC
	static unsigned int KHtDISP_CONFIG_FullRange 				= 1<<25 ;
	/// Enables Color Space Conv
	static unsigned int KHtDISP_CONFIG_ColConvEnable 			= 1<<24 ;
	/// First Field
	static unsigned int KHtDISP_CONFIG_FidFirst 				= 1<<23 ;
	/// Progressive/Interlace selection
	static unsigned int KHtDISP_CONFIG_OpModeEnable 			= 1<<22 ;
	/// Digital Alphablender
	static unsigned int KHtDISP_CONFIG_TVAlphaBlenderEnable 	= 1<<19 ;
	/// LCD Alphablender
	static unsigned int KHtDISP_CONFIG_LCDAlphaBlenderEnable	= 1<<18 ;
	/// RE-Fill COntrol
	static unsigned int KHtDISP_CONFIG_FIFOFilling          	= 1<<17 ;
	/// Handscheck b/w DMA buffer and STALL signal
	static unsigned int KHtDISP_CONFIG_FIFOHandCheck        	= 1<<16 ;
	/// CPR
	static unsigned int KHtDISP_CONFIG_CPR                  	= 1<<15 ;
	/// Merge FIF0
	static unsigned int KHtDISP_CONFIG_FIFOMerge            	= 1<<14 ;
	/// Trans color key selection (digital output)
	static unsigned int KHtDISP_CONFIG_TCKDigSelection      	= 1<<13 ;
	/// Trans color key enabled
	static unsigned int KHtDISP_CONFIG_TCKDigEnable         	= 1<<12 ;
	/// Trans color key selection (LCD output)
	static unsigned int KHtDISP_CONFIG_TCKLcdSelection      	= 1<<11 ;
	/// Enabl the trans color key for the LCD
	static unsigned int KHtDISP_CONFIG_TCKLcdEnable         	= 1<<10 ;
	/// Functional clocks gated enabled
	static unsigned int KHtDISP_CONFIG_FuncClksGated        	= 1<<9 ;
	/// ACBias Gated Enabled
	static unsigned int KHtDISP_CONFIG_ACBiasGated          	= 1<<8 ;
	/// VSYNC Gated Enabled
	static unsigned int KHtDISP_CONFIG_VsyncGated           	= 1<<7 ;
	/// HSYNC Gated Enabled
	static unsigned int KHtDISP_CONFIG_HsyncGated           	= 1<<6 ;
	/// Pixel Clock Gated Enabled
	static unsigned int KHtDISP_CONFIG_PxlClkGated          	= 1<<5 ;
	/// Pixel Data Gated Enabled
	static unsigned int KHtDISP_CONFIG_PxlDataGated         	= 1<<4 ;
	/// Palette/gamma table selection
	static unsigned int KHtDISP_CONFIG_PaletteGammaTable    	= 1<<3 ;
	/// Pixel gated enable (only for TFT)
	static unsigned int KHtDISP_CONFIG_PxlGated             	= 1<<0 ;
	/// Frame data only loaded every frame
	static unsigned int KHsDISP_CONFIG_LOADMODE             	= 1 ;
	/// Palette/gamma tableloaded every frame
	static unsigned int KHbDISP_CONFIG_PaletteLdEveryFrame  	= 0x0 ;
	/// Palette/gamma table to be loaded.
	static unsigned int KHbDISP_CONFIG_PaletteLdUser        	= 0x1 ;
	/// Frame data only loaded every frame.
	static unsigned int KHbDISP_CONFIG_Framedata            	= 0x2 ;
	/// Palette/gamma table and frame data loaded
	static unsigned int KHbDISP_CONFIG_PaletteFrameData     	= 0x3 ;

/************************************* 8) DISPC_CAPABLE REGISTER. OFFSET : 0x48 ****************************************/

	/// register allows to configure the display controller capability. (DEPRECATED)
	static unsigned int KHoDISP_CAPABLE     = 0x48 ;

		/// GfxGammaTableCapable
		static unsigned int KHtDISP_CAPABLE_GfxGammaTable       = 1<<9 ;
		/// GfxLayerCapable
		static unsigned int KHtDISP_CAPABLE_GfxLayer            = 1<<8 ;
		/// Graphics destination transparency color key
		static unsigned int KHtDISP_CAPABLE_GfxTransDst         = 1<<7 ;
		/// STNditheringCapable
		static unsigned int KHtDISP_CAPABLE_StnDithering        = 1<<6 ;
		/// TFTditheringCapable
		static unsigned int KHtDISP_CAPABLE_TftDithering        = 1<<5 ;
		/// Video src. transparency color key
		static unsigned int KHtDISP_CAPABLE_VidTransSrc         = 1<<4 ;
		/// Video layer
		static unsigned int KHtDISP_CAPABLE_VidLayer            = 1<<3 ;
		/// Video vertical up-sampling
		static unsigned int KHtDISP_CAPABLE_VidVertFir          = 1<<2 ;
		/// Video horizontal up-sampling
		static unsigned int KHtDISP_CAPABLE_VidHorFir           = 1<<1 ;
		/// Video path
		static unsigned int KHtDISP_CAPABLE_Vid                 = 1<<0 ;

/************************************* 9) DISPC_DEF_COLOR0 REGISTER. OFFSET : 0x4C ****************************************/

/// to configure the default solid background color for the Primary LCD
static unsigned int KHoDISP_DEFAULT_COLOR0 = 0x4C ;

	/// Set Default color to BLUE
	static unsigned int KHtDISP_DEFAULT_COLOR0_16B_Blue  = 0x001F ;
	/// Set Default color to GREEN
	static unsigned int KHtDISP_DEFAULT_COLOR0_16B_Green = 0x07E0 ;
	/// Set Default color to RED
	static unsigned int KHtDISP_DEFAULT_COLOR0_16B_Red   = 0xF800 ;

/************************************* 10) DISPC_DEF_COLOR1 REGISTER. OFFSET : 0x50 ****************************************/

/// to configure the default solid background color for the TV
static unsigned int KHoDISP_DEFAULT_COLOR1 = 0x50 ;


/************************************* 11) DISPC_TRANS_COLOR0 REGISTER. OFFSET : 0x54 ****************************************/

/// sets the transparency color value for the video/graphics overlays for Primary LCD
static unsigned int KHoDISP_TRANS_COLOR0    = 0x54 ;

/************************************* 12) DISPC_TRANS_COLOR1 REGISTER. OFFSET : 0x58 ****************************************/

/// sets the transparency color value for the video/graphics overlays for TV
static unsigned int KHoDISP_TRANS_COLOR1    = 0x58 ;

/************************************* 13) DISPC_LINE_STATUS REGISTER. OFFSET : 0x5C ****************************************/

/// indicates the current LCD panel display line number.
static unsigned int KHoDISP_LINE_STATUS     = 0x5C ;


#ifdef __WAKEUP_4430__
static unsigned int KHoDISPC_GLOBAL_BUFFER = 0x800;
#endif

/************************************* 14) DISPC_LINE_NUMBER REGISTER. OFFSET : 0x60 ****************************************/

/// register indicates the LCD panel display line number for the interrupt and the DMA request.
static unsigned int KHoDISP_LINE_NUMBER     = 0x60 ;

	// Configures the line number
	static unsigned int KHsDISP_LINE_NUMBER     = 0 ;

/************************************* 15) DISPC_TIMING_H1 REGISTER. OFFSET : 0x64 ****************************************/

/// configures the timing logic for the HSYNC signal (Primary LCD)
static unsigned int KHoDISP_TIMING_H        = 0x64 ;

	/// Horizontal Synchronization Pulse Width
	static unsigned int KHsDISP_TIMING_H_HSW    = 0 ;
	/// Horizontal Front Porch
	static unsigned int KHsDISP_TIMING_H_HFP    = 8 ;
	/// Horizontal Back Porch
	static unsigned int KHsDISP_TIMING_H_HBP    = 20;

/************************************* 16) DISPC_TIMING_V1 REGISTER. OFFSET : 0x68 ****************************************/

/// configures the timing logic for the VSYNC signal (Primary LCD)
static unsigned int KHoDISP_TIMING_V        = 0x68 ;

	/// Vertical Synchronization Pulse Width
	static unsigned int KHsDISP_TIMING_V_VSW    = 0 ;
	/// Vertical Front Porch
	static unsigned int KHsDISP_TIMING_V_VFP    = 8 ;
	/// Vertical Back Porch
	static unsigned int KHsDISP_TIMING_V_VBP    = 20;

/************************************* 17) DISPC_POL_FREQ1 REGISTER. OFFSET : 0x6C ****************************************/

/// configures the signal configuration
static unsigned int KHoDISP_POL_FREQ        = 0x6C ;

	/// HSYNC/VSYNC Alignment
	static unsigned int KHtDISP_POL_FREQ_Align              = 1<<18 ;
	/// HSYNC/VSYNC pixel clock control on/off
	static unsigned int KHtDISP_POL_FREQ_OnOff              = 1<<17 ;
	/// Program HSYNC/VSYNC rise or fall
	static unsigned int KHtDISP_POL_FREQ_Rf                 = 1<<16 ;
	/// Invert output enable
	static unsigned int KHtDISP_POL_FREQ_InvOutputEn        = 1<<15 ;
	/// Invert pixel clock
	static unsigned int KHtDISP_POL_FREQ_Ipc                = 1<<14 ;
	/// Invert HSYNC
	static unsigned int KHtDISP_POL_FREQ_Ihs                = 1<<13 ;
	/// Invert VSYNC
	static unsigned int KHtDISP_POL_FREQ_Ivs                = 1<<12 ;
	/// AC Bias Pin Transitions
	static unsigned int KHsDISP_POL_FREQ_ACBI   			= 8 ;
	/// AC Bias Pin Frequency Mask
	static unsigned int KHmDISP_POL_FREQ_ACB            	= 0xFF ;
	/// AC Bias Pin Frequency Shift
	static unsigned int KHsDISP_POL_FREQ_ACB    			= 0 ;
	/// AC Bias Pin Frequency Value
	static unsigned int KHtDISP_POL_FREQ_ACB                = 0 ;

/************************************* 18) DISPC_DIVISOR1 REGISTER. OFFSET : 0x70 ****************************************/

/// Configures the divisors for Primary LCD
static unsigned int KHoDISP_DIV         = 0x70 ;

	/// Display Controller Logic clock divisor mask
	static unsigned int KHmDISP_DIV_Lcd         			= 0xFF ;
	/// Display Controller Logic clock divisor
	static unsigned int KHsDISP_DIV_Lcd         			= 16 ;
	/// Pixel clock Divisor Mask
	static unsigned int KHmDISP_DIV_PCD         			= 0xFF  ;
	/// Pixel clock Divisor
	static unsigned int KHsDISP_DIV_PCD         			= 0  ;

/************************************* 19) DISPC_GLOBAL_ALPHA REGISTER. OFFSET : 0x74 ****************************************/

/// Configures the global alpha
static unsigned int KHoDISP_GLOBAL_ALPHA        = 0x74 ;

	/// Shift for Global Alhpha (VID 3)
	static unsigned int KHsVID3_GLOBAL_ALPHA    		= 24;
	/// Shift for Global Alhpha (VID 2)
	static unsigned int KHsVID2_GLOBAL_ALPHA    		= 16;
	/// Shift for Global Alhpha (VID 1)
	static unsigned int KHsVID1_GLOBAL_ALPHA    		= 8;
	/// Shift for Global Alhpha (Graphics)
	static unsigned int KHsGfx_GLOBAL_ALPHA    			= 0;

/************************************* 20) DISPC_SIZE_TV REGISTER. OFFSET : 0x78 ****************************************/

/// configures the size of the digital output field (interlace/Prog)
static unsigned int KHoDISP_SIZE_DIG    = 0x78 ;

	/// Pixel Per Line
	static unsigned int KHsSIZE_DIG_LPP         = 16  ;
	/// Delta Size Value
	static unsigned int KHsSIZE_DIG_DEL_LPP     = 14 ;
	/// Lines Per Panel
	static unsigned int KHsSIZE_DIG_PPL         = 0 ;

/************************************* 21) DISPC_SIZE_LCD1 REGISTER. OFFSET : 0x7C ****************************************/

/// configures the size of the Primary LCD
static unsigned int KHoDISP_SIZE_LCD    = 0x7C ;

	/// Pixel Per Line
	static unsigned int KHsSIZE_LCD_LPP         = 16  ;
	/// Delta Size Value
	static unsigned int KHsSIZE_LCD_DEL_LPP     = 14 ;
	/// Lines Per Panel
	static unsigned int KHsSIZE_LCD_PPL         = 0 ;

//******************************************************************************************************************************
//******************************************************************************************************************************
//*************************************** GRAPHICS PIPELINE SPECIFIC REGISTER DEFINITIONS **************************************
//******************************************************************************************************************************
//******************************************************************************************************************************

/************************************* 22) DISPC_GFX_BA0 REGISTER. OFFSET : 0x80 ****************************************/

/// configures the base address of the graphics buffer displayed in the graphics window
static unsigned int KHoDISP_GFX_BA0     = 0x80 ;

/************************************* 23) DISPC_GFX_POS REGISTER. OFFSET : 0x88 ****************************************/

/// configures the position of the graphics window
static unsigned int KHoDISP_GFX_POS     = 0x88 ;

	/// Shift of the X position of the graphics window
	static unsigned int KHsDISP_GFX_POSGfxPosX  				= 0  ;
	/// X position of the graphics window
	static unsigned int KHtDISP_GFX_POSGfxPosX          		= 0  ;
	/// Shift of the Y position of the graphics window
	static unsigned int KHsDISP_GFX_POSGfxPosY 					= 16 ;
	/// Y position of the graphics window
	static unsigned int KHtDISP_GFX_POSGfxPosY          		= 0  ;

/************************************* 24) DISPC_GFX_SIZE REGISTER. OFFSET : 0x8C ****************************************/

/// configures the size of the graphics window
static unsigned int KHoDISP_GFX_SIZE    = 0x8C ;

	/// Shift of the X position of the graphics window
	static unsigned int KHsDISP_GFX_SIZEGfxSizeX 				= 0    ;
	/// Shift of the Y position of the graphics window
	static unsigned int KHsDISP_GFX_SIZEGfxSizeY 				= 16   ;

/************************************* 25) DISPC_GFX_ATTR REGISTER. OFFSET : 0xA0 ****************************************/

/// configures the graphics attributes: format, transparency color key enable
static unsigned int KHoDISP_GFX_ATTR    = 0xA0 ;

	/// LCD channel selctor (1/2/WB)
	static unsigned int KHsDISP_GFX_ATTRChOut2 					= 30;
	/// Pri LCD Selected
	static unsigned int KHtDISP_GFX_ATTRPriLCD 					= 0x0;
	/// Sec LCD Selected
	static unsigned int KHtDISP_GFX_ATTRSecLCD 					= 0x1;
	/// Writeback O/P Selected
	static unsigned int KHtDISP_GFX_ATTRWbOut  					= 0x3;
	/// Burst type (1D/2D)
	static unsigned int KHtDISP_GFX_ATTRBurstType 				= 1<<29;
	/// Z-Order defining priority of the layer
	static unsigned int KHsDISP_GFX_ATTRZOrder 					= 26;
	// Default order - 0
	static unsigned int KHtDISP_GFX_ATTRZOrder0 				= 0x0;
	// Z-Order 1
	static unsigned int KHtDISP_GFX_ATTRZOrder1 				= 0x1;
	// Z-Order 2
	static unsigned int KHtDISP_GFX_ATTRZOrder2 				= 0x2;
	// Z-Order 3
	static unsigned int KHtDISP_GFX_ATTRZOrder3 				= 0x3;
	/// Z-Order Enable
	static unsigned int KHtDISP_GFX_ATTRZOrderEn 				= 1<<25;
	/// Anti-Flicker
	static unsigned int KHtDISP_GFX_ATTRAntiFlickr 				= 1<<24;
	/// Auto Self-Refresh
	static unsigned int KHtDISP_GFX_ATTRSelfRefreshAuto 		= 1<<17;
	/// Self-Refresh
	static unsigned int KHtDISP_GFX_ATTRSelfRefresh 			= 1<<15;
	/// Arbitration (Priority of Gfx Pipe)
	static unsigned int KHtDISP_GFX_ATTRArbitration 			= 1<<14;
	/// Rotation Angle
	static unsigned int KHsDISP_GFX_ATTRRotation 				= 12;
	/// Zero Degree rotation
	static unsigned int KHtDISP_GFX_ATTRRotation0 				= 0x0;
	/// 90 Degree rotation
	static unsigned int KHtDISP_GFX_ATTRRotation90 				= 0x1;
	/// 180 Degree rotation
	static unsigned int KHtDISP_GFX_ATTRRotation180 			= 0x2;
	/// 270 Degree rotation
	static unsigned int KHtDISP_GFX_ATTRRotation2780 			= 0x3;
	/// Enable preload
	static unsigned int KHtDISP_GFX_ATTRPreload         		= 1<<11 ;
	/// 24 bit output selected
	static unsigned int KHtDISP_GFX_ATTR24bOutputEn         	= 1<<9  ;
	/// Graphics DMA Burst Size (16*32)
	static unsigned int KHtDISP_GFX_ATTRBurst16_32          	= 1<<7  ;
	/// Graphics DMA Burst Size (8*32)
	static unsigned int KHtDISP_GFX_ATTRBurst8_32           	= 1<<6  ;
	/// Enable graphics replication logic
	static unsigned int KHtDISP_GFX_ATTRRep_En              	= 1<<5  ;
	/// Graphics format ARGB444
	static unsigned int KHtDISP_GFX_ATTRGfxFormatARGB16    		= 1<<3 | 1<<1;
	/// Graphics format ARGB
	static unsigned int KHtDISP_GFX_ATTRGfxFormatARGB32    		= 1<<4 | 1<<3;
	/// Graphics format RGBA
	static unsigned int KHtDISP_GFX_ATTRGfxFormatRGBA32    		= 1<<4 | 1<<3 | 1<<1;
	/// Graphics format ARGB444
	static unsigned int KHtDISP_GFX_ATTRGfxFormatRGB24Packed	= 1<<4 | 1<<1  ;
	/// Graphics format RGB24
	static unsigned int KHtDISP_GFX_ATTRGfxFormatRGB24      	= 1<<4  ;
	/// Graphics format RGB16
	static unsigned int KHtDISP_GFX_ATTRGfxFormatRGB16      	= 1<<3 | 1<<2  ;
	/// Graphics format RGB12
	static unsigned int KHtDISP_GFX_ATTRGfxFormatRGB12      	= 1<<3  ;
	/// Graphics format CLUT8 - 8bpp
	static unsigned int KHtDISP_GFX_ATTRGfxFormatCLUT8      	= 1<<2 | 1<<1  ;
	/// Graphics format CLUT4 - 4bpp
	static unsigned int KHtDISP_GFX_ATTRGfxFormatCLUT4      	= 1<<2  ;
	/// Graphics format CLUT2 - 2bpp
	static unsigned int KHtDISP_GFX_ATTRGfxFormatCLUT2      	= 1<<1  ;
	/// Graphics windows present on the screen
	static unsigned int KHtDISP_GFX_Enable                  	= 1<<0  ;

/************************************* 26) DISPC_GFX_BUF_THRESHOLD REGISTER. OFFSET : 0xA4 ****************************************/

/// configures the graphics FIFO
static unsigned int KHoDISP_GFX_THRESHOLD   = 0xA4 ;

	/// Shift for the high threshold value
	static unsigned int KHsDISP_GFX_THRESHOLDHigh   				= 16 ;
	/// Number of bytes defining the high threshold value
	static unsigned int KHtDISP_GFX_THRESHOLDHigh           		= 0x3FC ;
	/// Shift for the low threshold value
	static unsigned int KHsDISP_GFX_THRESHOLDLow    				= 0 ;
	/// Number of bytes defining the low threshold value
	static unsigned int KHtDISP_GFX_THRESHOLDLow            		= 0x3BC ;

/************************************* 27) DISPC_GFX_BUF_SIZE_STATUS REGISTER. OFFSET : 0xA8 ****************************************/

/// defines the Graphics FIFO size.
static unsigned int KHoDISP_GFX_FIFO_SIZE_STATUS    				= 0xA8 ;

/************************************* 28) DISPC_GFX_ROW_INC REGISTER. OFFSET : 0xAC ****************************************/

/// configures the No of bytes to incr at the end of the row
static unsigned int KHoDISP_GFX_ROW_INC     						= 0xAC ;

	/// No of bytes to skip at the end of the row
	static unsigned int KHtDISP_GFX_ROW_INCValue            		= 0x1 ;

/************************************* 29) DISPC_PIXEL_INC REGISTER. OFFSET : 0xB0 ****************************************/

/// configures the no of bytes to incr B/W 2 pixels
static unsigned int KHoDISP_GFX_PIXEL_INC   						= 0xB0 ;

	/// No of bytes to incr B/W 2 pixels.
	static unsigned int KHtDISP_GFX_PIXEL_INCValue  				= 0x1  ;

/************************************* 30) DISPC_GFX_WINDOW_SKIP REGISTER. OFFSET : 0xB4 ****************************************/

/// configures the number of bytes to skip during video window #n display.
static unsigned int KHoDISP_GFX_WINDOW_SKIP     					= 0xB4 ;

/************************************* 31) DISPC_GFX_TABLE_BA REGISTER. OFFSET : 0xB8 ****************************************/

/// configures the base address of the palette buffer or the gamma table buffer
static unsigned int KHoDISP_GFX_TABLE_BA        					= 0xB8 ;


//******************************************************************************************************************************
//******************************************************************************************************************************
//*************************************** VIDEO PIPELINE 1 SPECIFIC REGISTER DEFINITIONS ***************************************
//******************************************************************************************************************************
//******************************************************************************************************************************

/************************************* 32) DISPC_VID1_BA REGISTER. OFFSET : 0xBC ****************************************/

/// VIdeo 1 Base Address
static unsigned int KHoDISP_VID1_BA0        = 0xBC ;
static unsigned int KHoDISP_VID1_BA1        = 0xC0 ;

/************************************* 33) DISPC_VID1_POS REGISTER. OFFSET : 0xC4 ****************************************/

static unsigned int KHoDISP_VID1_POS        = 0xC4 ;

	// POS X Shift
	static unsigned int KHsDISP_VID1_POSGfxPosX = 0  ;
	// POS X
	static unsigned int KHtDISP_VID1_POSGfxPosX = 0  ;
	// POS Y Shift
	static unsigned int KHsDISP_VID1_POSGfxPosY = 16 ;
	// POS Y
	static unsigned int KHtDISP_VID1_POSGfxPosY = 0  ;

/************************************* 34) DISPC_VID1_SIZE REGISTER. OFFSET : 0xC8 ****************************************/

static unsigned int KHoDISP_VID1_SIZE   = 0xC8 ;

	// Shift of the X position of the video1 window
	static unsigned int KHsDISP_VID1_SIZEGfxSizeX = 0    ;
	// Shift of the Y position of the video1 window
	static unsigned int KHsDISP_VID1_SIZEGfxSizeY = 16   ;

/************************************* 35) DISPC_VID1_ATTRIBUTE REGISTER. OFFSET : 0xCC ****************************************/

static unsigned int KHoDISP_VID1_ATTR   = 0xCC ;

	/// LCD channel selctor (1/2/WB)
	static unsigned int KHsDISP_VID1_ATTRChOut2 				= 30;
	/// Pri LCD Selected
	static unsigned int KHtDISP_VID1_ATTRPriLCD 				= 0x0;
	/// Sec LCD Selected
	static unsigned int KHtDISP_VID1_ATTRSecLCD 				= 0x1;
	/// Writeback O/P Selected
	static unsigned int KHtDISP_VID1_ATTRWbOut  				= 0x3;
	/// Burst type (1D/2D)
	static unsigned int KHtDISP_VID1_ATTRBurstType 				= 1<<29;
	/// Z-Order defining priority of the layer
	static unsigned int KHsDISP_VID1_ATTRZOrder 				= 26;
	// Default order - 0
	static unsigned int KHtDISP_VID1_ATTRZOrder0 				= 0x0;
	// Z-Order 1
	static unsigned int KHtDISP_VID1_ATTRZOrder1 				= 0x1;
	// Z-Order 2
	static unsigned int KHtDISP_VID1_ATTRZOrder2 				= 0x2;
	// Z-Order 3
	static unsigned int KHtDISP_VID1_ATTRZOrder3 				= 0x3;
	/// Z-Order Enable
	static unsigned int KHtDISP_VID1_ATTRZOrderEn 				= 1<<25;
	/// Enables Self-Refresh
	static unsigned int KHtDISP_VID1_ATTRSelfRefresh 			= 1<<24;
	/// Arbitration (Priority of Vid1 Pipe)
	static unsigned int KHtDISP_VID1_ATTRArbitration 			= 1<<23;
	/// Double Stride (Only for YUV420)
	static unsigned int KHtDISP_VID1_ATTRDblStride 				= 1<<22;
	/// Video vertical resize tap number
	static unsigned int KHtDISP_VID1_ATTRVertTaps            	= 1<<21;
	/// Video optimization (Not in use)
	static unsigned int KHtDISP_VID1_ATTRDmaOptimize     		= 1<<20;
	/// Video preload value
	static unsigned int KHtDISP_VID1_ATTRFifoPreload        	= 1<<19;
	/// Self Refresh Auto
	static unsigned int KHtDISP_VID1_ATTRSelfRefAuto        	= 1<<17 ;
	/// 24 bit output selected
	static unsigned int KHtDISP_VID1_ATTR24bOutputEn        	= 1<<16 ;
	/// Video1 DMA Burst Size (4*128)
	static unsigned int KHtDISP_VID1_ATTRBurst4_128      		= 1<<14  ;
	/// Video1 DMA Burst Size (8*128)
	static unsigned int KHtDISP_VID1_ATTRBurst8_128     		= 1<<15  ;
	/// Video1 DMA Burst Size (2*128)
	static unsigned int KHtDISP_VID1_ATTRBurst2_128      		= 0;
	/// Rotation 0 degree
	static unsigned int KHtDISP_VID1_ATTRRotation_0     		= 0;
	/// Rotation 90 degree
	static unsigned int KHtDISP_VID1_ATTRRotation_90          	= 1<<12  ;
	/// Rotation 180 degree
	static unsigned int KHtDISP_VID1_ATTRRotation_180          	= 1<<13  ;
	/// Rotation 270 degree
	static unsigned int KHtDISP_VID1_ATTRRotation_270          	= 1<<12 | 1<<13  ;
	/// Full Range
	static unsigned int KHtDISP_VID1_ATTRFullRange          	= 1<<11  ;
	/// Enable video1 replication logic
	static unsigned int KHtDISP_VID1_ATTRRep_En          		= 1<<10  ;
	/// Enable Color Conversion
	static unsigned int KHtDISP_VID1_ATTRColorConv_En   		= 1<<9;
	/// Vertical Downsampling
	static unsigned int KHtDISP_VID1_ATTRResize_Ver_Conf    	= 1<<8;
	/// Horizontal Downsampling
	static unsigned int KHtDISP_VID1_ATTRResize_Hor_Conf    	= 1<<7;
	/// Horizontal Resize Enable
	static unsigned int KHtDISP_VID1_ATTRResize_Hor_En  		= 1<<5;
	/// Vertical Resize Enable
	static unsigned int KHtDISP_VID1_ATTRResize_Ver_En  		= 1<<6;
	/// video1 format YUV2
	static unsigned int KHtDISP_VID1_ATTRVid1FormatYUV2 		= 1<<4 | 1<<2 ;
	/// video1 format UYVY
	static unsigned int KHtDISP_VID1_ATTRVid1FormatUYVY 		= 1<<4 | 1<<2 | 1<<1;
	/// video1 format RGB16
	static unsigned int KHtDISP_VID1_ATTRVid1FormatRGB16    	= 1<<3 | 1<<2  ;
	///video1 format RGB24U
	static unsigned int KHtDISP_VID1_ATTRVid1FormatRGB24U 		= 1<<4;
	///video1 format RGB24
	static unsigned int KHtDISP_VID1_ATTRVid1FormatRGB24  		= 1<<4 | 1<<1;
	///video1 format YUV420 Semi Planar
	static unsigned int KHtDISP_VID1_ATTRVid1FormatYUV420SP  	= 0<<1;

	/// Video1 window present on the screen
	static unsigned int KHtDISP_VID1_Enable                  	= 1<<0  ;

	//Double stride bit - for supporting YUV420 NV12
	static unsigned int KHtDISP_VID1_ATTRDoubleStride        = 1<<22;

// The below 2 fields Will be deletd soon
	static unsigned int KHtDISP_VID1_ATTRLineBufferSplit    = 1<<22;
	/// Video row repeat (YUV case only when rotating 90 or 270 degrees)
	static unsigned int KHtDISP_VID1_ATTRRowRepeatEn        = 1<<18;

/************************************* 36) DISPC_VID1_THRESHOLD REGISTER. OFFSET : 0xD0 ****************************************/

static unsigned int KHoDISP_VID1_THRESHOLD  = 0xD0 ;

	/// Shift for the high threshold value
	static unsigned int KHsDISP_VID1_THRESHOLDHigh 				=        16   ;
	/// No of bytes defining the high threshold value
	static unsigned int KHtDISP_VID1_THRESHOLDHigh 				=        0x3FC ;
	/// Shift for the low threshold value
	static unsigned int KHsDISP_VID1_THRESHOLDLow  				=        0    ;
	#ifdef ENABLE_DSS_SMARTSTANDBY
	// Number of bytes defining the low threshold value
	static unsigned int KHtDISP_VID1_THRESHOLDLow  				=        0x3BC ;
	#else
	// OMAPS00138607 Number of bytes defining the low threshold value
	static unsigned int KHtDISP_VID1_THRESHOLDLow  				=        0x3FC -64 ;
	#endif
/************************************* 37) DISPC_VID1_BUF_SIZE_STATUS REGISTER. OFFSET : 0xD4 ****************************************/

/// defines the Video FIFO size for the video pipeline
static unsigned int KHoDISP_VID1_FIFO_SIZE_STATUS     			= 0xD4 ;

/************************************* 38) DISPC_VID1_ROW_INCR REGISTER. OFFSET : 0xD8 ****************************************/

static unsigned int KHoDISP_VID1_ROW_INC     					= 0xD8 ;

	/// Number of bytes to skip at the end of the row
	static unsigned int KHtDISP_VID1_ROW_INCValue  				=     0x1  ;

/************************************* 39) DISPC_VID1_PIXEL_INCR REGISTER. OFFSET : 0xDC ****************************************/

static unsigned int KHoDISP_VID1_PIXEL_INC   					= 0xDC ;

	/// Number of bytes to increment between 2 pixels.
	static unsigned int KHtDISP_VID1_PIXEL_INCValue  			=     0x1  ;

/************************************* 40) DISPC_VID1_FIR REGISTER. OFFSET : 0xE0 ****************************************/

/// configures the resize factors for horizontal and vertical
/// up/down-sampling of the video window
static unsigned int KHoDISP_VID1_FIR            				= 0xE0 ;

	/// shift for Vertical increment of the up/down-sampling filter
	static unsigned int KHsDISP_VID1_FIRVINC                	= 16 ;
	/// shift for Horizontal increment of the up/down-sampling filter
	static unsigned int KHsDISP_VID1_FIRHINC                	= 0 ;

/************************************* 41) DISPC_VID1_PICTURE_SIZE REGISTER. OFFSET : 0xE4 ****************************************/

/// configure the size of the video picture associated with the video layer
static unsigned int KHoDISP_VID1_PICTURE_SIZE   				= 0xE4 ;

	/// shift for Number of lines of the video picture Encoded value
	static unsigned int KHsDISP_VID1_ORGSIZEY                   = 16 ;
	/// shift for Number of pixels of the video picture Encoded value
	static unsigned int KHsDISP_VID1_ORGSIZEX                   = 0 ;

/************************************* 42) DISPC_VID1_ACCUJ REGISTER. OFFSET : 0xE8,0xEC ****************************************/

/// configures the resize accumulator init values for horizontal and
/// vertical up/down-sampling of the video window
static unsigned int KHoDISP_VID1_ACCU0          				= 0xE8 ;
static unsigned int KHoDISP_VID1_ACCU1          				= 0xEC ;
	/// shift for Vertical initialization accu value
	static unsigned int KHsDISP_VID1_VERTICALACCU           	= 16 ;
	/// shift for Horizontal initialization accu value
	static unsigned int KHsDISP_VID1_HORIZONTALACCU         	= 0 ;

/************************************* 43) DISPC_VID1_FIR_COEF_H_I REGISTER. OFFSET : 0xF0 - 0x128 ****************************************/

/// The bank of registers configure the up/down-scaling coefficients for
/// the vertical and horizontal resize of the video picture associated with
/// the video window
static unsigned int KHoDISP_VID1_FIR_COEF_H0    				= 0xF0 ;
static unsigned int KHoDISP_VID1_FIR_COEF_H1    				= 0xF8 ;
static unsigned int KHoDISP_VID1_FIR_COEF_H2    				= 0x100 ;
static unsigned int KHoDISP_VID1_FIR_COEF_H3    				= 0x108 ;
static unsigned int KHoDISP_VID1_FIR_COEF_H4    				= 0x110 ;
static unsigned int KHoDISP_VID1_FIR_COEF_H5    				= 0x118 ;
static unsigned int KHoDISP_VID1_FIR_COEF_H6    				= 0x120 ;
static unsigned int KHoDISP_VID1_FIR_COEF_H7    				= 0x128 ;

	/// shift for Signed coefficient C3 for the horizontal up/downscaling
	/// with the phase n
	static unsigned int KHsDISP_VID1_FIRHC3             		= 24 ;
	/// shift for Signed coefficient C2 for the horizontal up/downscaling
	/// with the phase n
	static unsigned int KHsDISP_VID1_FIRHC2             		= 16 ;
	/// shift for Signed coefficient C1 for the horizontal up/downscaling
	/// with the phase n
	static unsigned int KHsDISP_VID1_FIRHC1             		= 8 ;
	/// shift for Signed coefficient C0 for the horizontal up/downscaling
	/// with the phase n
	static unsigned int KHsDISP_VID1_FIRHC0             		= 0 ;

/************************************* 44) DISPC_VID1_FIR_COEF_HV_I REGISTER. OFFSET : 0xF4 - 0x12C ****************************************/

/// The bank of registers configure the down/up/down-scaling coefficients
/// for the vertical and horizontal resize of the video picture associated
/// with the video window
static unsigned int KHoDISP_VID1_FIR_COEF_HV0   				= 0xF4 ;
static unsigned int KHoDISP_VID1_FIR_COEF_HV1   				= 0xFC ;
static unsigned int KHoDISP_VID1_FIR_COEF_HV2   				= 0x104 ;
static unsigned int KHoDISP_VID1_FIR_COEF_HV3   				= 0x10C ;
static unsigned int KHoDISP_VID1_FIR_COEF_HV4   				= 0x114 ;
static unsigned int KHoDISP_VID1_FIR_COEF_HV5   				= 0x11C ;
static unsigned int KHoDISP_VID1_FIR_COEF_HV6   				= 0x124 ;
static unsigned int KHoDISP_VID1_FIR_COEF_HV7   				= 0x12C ;

	/// shift for Signed coefficient C2 for the vertical up/downscaling
	/// with the phase n
	static unsigned int KHsDISP_VID1_FIRVC2             		= 24 ;
	/// shift for Signed coefficient C1 for the vertical up/downscaling
	/// with the phase n
	static unsigned int KHsDISP_VID1_FIRVC1             		= 16 ;
	/// shift for Signed coefficient C0 for the vertical up/downscaling
	/// with the phase n
	static unsigned int KHsDISP_VID1_FIRVC0             		= 8 ;
	/// shift for Signed coefficient C4 for the horizontal up/downscaling
	/// with the phase n
	static unsigned int KHsDISP_VID1_FIRHC4             		= 0 ;

/************************************* 45) DISPC_VID1_CONV_COEF0 REGISTER. OFFSET : 0x130 ****************************************/

/// configures the color space conversion matrix coefficients for the video
/// pipeline
static unsigned int KHoDISP_VID1_CONV_COEF0 = 0x130 ;

	/// RCR Co Efficient
	static unsigned int KHsDISP_RCRCOEF0                = 16 ;
	/// RY Co Efficient
	static unsigned int KHsDISP_RYCOEF0                 = 0 ;

/************************************* 46) DISPC_VID1_CONV_COEF1 REGISTER. OFFSET : 0x134 ****************************************/

/// configures the color space conversion matrix coefficients for the video
/// pipeline
static unsigned int KHoDISP_VID1_CONV_COEF1     = 0x134 ;

	/// shift for GY Coefficient
	static unsigned int KHsDISP_GYCOEF1                 = 16 ;
	/// shift for RCb Coefficient
	static unsigned int KHsDISP_RCBCOEF1                = 0 ;

/************************************* 47) DISPC_VID1_CONV_COEF2 REGISTER. OFFSET : 0x138 ****************************************/

/// configures the color space conversion matrix coefficients for the video
/// pipeline
static unsigned int KHoDISP_VID1_CONV_COEF2     = 0x138 ;

	/// shift for GCB Coefficient
	static unsigned int KHsDISP_GCBCOEF2                = 16 ;
	/// shift for GCR Coefficient
	static unsigned int KHsDISP_GCRCOEF2                = 0 ;

/************************************* 48) DISPC_VID1_CONV_COEF3 REGISTER. OFFSET : 0x13C ****************************************/

/// configures the color space conversion matrix coefficients for the video
/// pipeline
static unsigned int KHoDISP_VID1_CONV_COEF3     = 0x13C ;

	/// shift for BCR Coefficient
	static unsigned int KHsDISP_BCRCOEF3                = 16 ;
	/// shift for BY Coefficient
	static unsigned int KHsDISP_BYCOEF3                 = 0 ;

/************************************* 49) DISPC_VID1_CONV_COEF4 REGISTER. OFFSET : 0x140 ****************************************/

/// configures the color space conversion matrix coefficients for the video
/// pipeline
static unsigned int KHoDISP_VID1_CONV_COEF4     = 0x140 ;

	/// shift for BCB Coefficient
	static unsigned int KHsDISP_BCBCOEF4                = 0 ;


//******************************************************************************************************************************
//******************************************************************************************************************************
//*************************************** VIDEO PIPELINE 2 SPECIFIC REGISTER DEFINITIONS ***************************************
//******************************************************************************************************************************
//******************************************************************************************************************************


/************************************* 50) DISPC_VID2_BA REGISTER. OFFSET : 0x14C ****************************************/

/// VIdeo 2 Base Address
static unsigned int KHoDISP_VID2_BA0        = 0x14C ;
static unsigned int KHoDISP_VID2_BA1        = 0x150 ;

/************************************* 51) DISPC_VID2_POS REGISTER. OFFSET : 0x154 ****************************************/

static unsigned int KHoDISP_VID2_POS        = 0x154 ;

	// POS X Shift
	static unsigned int KHsDISP_VID2_POSGfxPosX = 0  ;
	// POS X
	static unsigned int KHtDISP_VID2_POSGfxPosX = 0  ;
	// POS Y Shift
	static unsigned int KHsDISP_VID2_POSGfxPosY = 16 ;
	// POS Y
	static unsigned int KHtDISP_VID2_POSGfxPosY = 0  ;

/************************************* 52) DISPC_VID2_SIZE REGISTER. OFFSET : 0x158 ****************************************/

static unsigned int KHoDISP_VID2_SIZE   = 0x158 ;

	// Shift of the X position of the video1 window
	static unsigned int KHsDISP_VID2_SIZEGfxSizeX = 0    ;
	// Shift of the Y position of the video1 window
	static unsigned int KHsDISP_VID2_SIZEGfxSizeY = 16   ;

/************************************* 53) DISPC_VID2_ATTRIBUTE REGISTER. OFFSET : 0x15C ****************************************/

static unsigned int KHoDISP_VID2_ATTR   = 0x15C ;

	/// LCD channel selctor (1/2/WB)
	static unsigned int KHsDISP_VID2_ATTRChOut2 				= 30;
	/// Pri LCD Selected
	static unsigned int KHtDISP_VID2_ATTRPriLCD 				= 0x0;
	/// Sec LCD Selected
	static unsigned int KHtDISP_VID2_ATTRSecLCD 				= 0x1;
	/// Writeback O/P Selected
	static unsigned int KHtDISP_VID2_ATTRWbOut  				= 0x3;
	/// Burst type (1D/2D)
	static unsigned int KHtDISP_VID2_ATTRBurstType 				= 1<<29;
	/// Z-Order defining priority of the layer
	static unsigned int KHsDISP_VID2_ATTRZOrder 				= 26;
	// Default order - 0
	static unsigned int KHtDISP_VID2_ATTRZOrder0 				= 0x0;
	// Z-Order 1
	static unsigned int KHtDISP_VID2_ATTRZOrder1 				= 0x1;
	// Z-Order 2
	static unsigned int KHtDISP_VID2_ATTRZOrder2 				= 0x2;
	// Z-Order 3
	static unsigned int KHtDISP_VID2_ATTRZOrder3 				= 0x3;
	/// Z-Order Enable
	static unsigned int KHtDISP_VID2_ATTRZOrderEn 				= 1<<25;
	/// Enables Self-Refresh
	static unsigned int KHtDISP_VID2_ATTRSelfRefresh 			= 1<<24;
	/// Arbitration (Priority of Vid1 Pipe)
	static unsigned int KHtDISP_VID2_ATTRArbitration 			= 1<<23;
	/// Double Stride (Only for YUV420)
	static unsigned int KHtDISP_VID2_ATTRDblStride 				= 1<<22;
	/// Video vertical resize tap number
	static unsigned int KHtDISP_VID2_ATTRVertTaps            	= 1<<21;
	/// Video optimization (Not in use)
	static unsigned int KHtDISP_VID2_ATTRDmaOptimize     		= 1<<20;
	/// Video preload value
	static unsigned int KHtDISP_VID2_ATTRFifoPreload        	= 1<<19;
	/// Self Refresh Auto
	static unsigned int KHtDISP_VID2_ATTRSelfRefAuto        	= 1<<17 ;
	/// 24 bit output selected
	static unsigned int KHtDISP_VID2_ATTR24bOutputEn        	= 1<<16 ;
	/// Video1 DMA Burst Size (4*128)
	static unsigned int KHtDISP_VID2_ATTRBurst4_128      		= 1<<14  ;
	/// Video1 DMA Burst Size (8*128)
	static unsigned int KHtDISP_VID2_ATTRBurst8_128     		= 1<<15  ;
	/// Video1 DMA Burst Size (2*128)
	static unsigned int KHtDISP_VID2_ATTRBurst2_128      		= 0;
	/// Rotation 0 degree
	static unsigned int KHtDISP_VID2_ATTRRotation_0     		= 0;
	/// Rotation 90 degree
	static unsigned int KHtDISP_VID2_ATTRRotation_90          	= 1<<12  ;
	/// Rotation 180 degree
	static unsigned int KHtDISP_VID2_ATTRRotation_180          	= 1<<13  ;
	/// Rotation 270 degree
	static unsigned int KHtDISP_VID2_ATTRRotation_270          	= 1<<12 | 1<<13  ;
	/// Full Range
	static unsigned int KHtDISP_VID2_ATTRFullRange          	= 1<<11  ;
	/// Enable video1 replication logic
	static unsigned int KHtDISP_VID2_ATTRRep_En          		= 1<<10  ;
	/// Enable Color Conversion
	static unsigned int KHtDISP_VID2_ATTRColorConv_En   		= 1<<9;
	/// Vertical Downsampling
	static unsigned int KHtDISP_VID2_ATTRResize_Ver_Conf    	= 1<<8;
	/// Horizontal Downsampling
	static unsigned int KHtDISP_VID2_ATTRResize_Hor_Conf    	= 1<<7;
	/// Horizontal Resize Enable
	static unsigned int KHtDISP_VID2_ATTRResize_Hor_En  		= 1<<5;
	/// Vertical Resize Enable
	static unsigned int KHtDISP_VID2_ATTRResize_Ver_En  		= 1<<6;
	/// video1 format YUV2
	static unsigned int KHtDISP_VID2_ATTRVid1FormatYUV2 		= 1<<4 | 1<<2 ;
	/// video1 format UYVY
	static unsigned int KHtDISP_VID2_ATTRVid1FormatUYVY 		= 1<<4 | 1<<2 | 1<<1;
	/// video1 format RGB16
	static unsigned int KHtDISP_VID2_ATTRVid1FormatRGB16    	= 1<<3 | 1<<2  ;
	///video1 format RGB24U
	static unsigned int KHtDISP_VID2_ATTRVid1FormatRGB24U 		= 1<<4;
	///video1 format RGB24
	static unsigned int KHtDISP_VID2_ATTRVid1FormatRGB24  		= 1<<4 | 1<<1;
	/// Video2 format ARGB16
	static unsigned int KHtDISP_VID2_ATTRVid2FormatARGB16   	= 1<<3 | 1<<1;
	/// Video2 format ARGB32
	static unsigned int KHtDISP_VID2_ATTRVid2FormatARGB32 		= 1<<4 | 1<<3;
	/// Video2 format RGBA32
	static unsigned int KHtDISP_VID2_ATTRVid2FormatRGBA32 		= 1<<4 | 1<<3 | 1<<1;
	///video2 format YUV420 Semi Planar
	static unsigned int KHtDISP_VID2_ATTRVid2FormatYUV420SP  	= 0<<1;
	/// Video1 window present on the screen
	static unsigned int KHtDISP_VID2_Enable                  	= 1<<0  ;

	//Double stride bit - for supporting YUV420 NV12
	static unsigned int KHtDISP_VID2_ATTRDoubleStride        = 1<<22;

// The below 2 fields Will be deletd soon
	static unsigned int KHtDISP_VID2_ATTRLineBufferSplit    = 1<<22;
	/// Video row repeat (YUV case only when rotating 90 or 270 degrees)
	static unsigned int KHtDISP_VID2_ATTRRowRepeatEn        = 1<<18;




/************************************* 54) DISPC_VID2_THRESHOLD REGISTER. OFFSET : 0x160 ****************************************/

static unsigned int KHoDISP_VID2_THRESHOLD  = 0x160 ;

	/// Shift for the high threshold value
	static unsigned int KHsDISP_VID2_THRESHOLDHigh 				=        16   ;
	/// No of bytes defining the high threshold value
	static unsigned int KHtDISP_VID2_THRESHOLDHigh 				=        0x3FC ;
	/// Shift for the low threshold value
	static unsigned int KHsDISP_VID2_THRESHOLDLow  				=        0    ;
	#ifdef ENABLE_DSS_SMARTSTANDBY
	// Number of bytes defining the low threshold value
	static unsigned int KHtDISP_VID2_THRESHOLDLow  				=        0x3BC ;
	#else
	// OMAPS00138607 Number of bytes defining the low threshold value
	static unsigned int KHtDISP_VID2_THRESHOLDLow  				=        0x3FC -64 ;
	#endif

/************************************* 55) DISPC_VID2_BUF_SIZE_STATUS REGISTER. OFFSET : 0x164 ****************************************/

/// defines the Video FIFO size for the video pipeline
static unsigned int KHoDISP_VID2_FIFO_SIZE_STATUS     			= 0x164 ;

/************************************* 56) DISPC_VID2_ROW_INCR REGISTER. OFFSET : 0x168 ****************************************/

static unsigned int KHoDISP_VID2_ROW_INC     					= 0x168 ;

	/// Number of bytes to skip at the end of the row
	static unsigned int KHtDISP_VID2_ROW_INCValue  				=     0x1  ;

/************************************* 57) DISPC_VID2_PIXEL_INCR REGISTER. OFFSET : 0x16C ****************************************/

static unsigned int KHoDISP_VID2_PIXEL_INC   					= 0x16C ;

	/// Number of bytes to increment between 2 pixels.
	static unsigned int KHtDISP_VID2_PIXEL_INCValue  			=     0x1  ;

/************************************* 58) DISPC_VID2_FIR REGISTER. OFFSET : 0x170 ****************************************/

/// configures the resize factors for horizontal and vertical
/// up/down-sampling of the video window
static unsigned int KHoDISP_VID2_FIR            				= 0x170 ;

	/// shift for Vertical increment of the up/down-sampling filter
	static unsigned int KHsDISP_VID2_FIRVINC                	= 16 ;
	/// shift for Horizontal increment of the up/down-sampling filter
	static unsigned int KHsDISP_VID2_FIRHINC                	= 0 ;

/************************************* 59) DISPC_VID2_PICTURE_SIZE REGISTER. OFFSET : 0x174 ****************************************/

/// configure the size of the video picture associated with the video layer
static unsigned int KHoDISP_VID2_PICTURE_SIZE   				= 0x174 ;

	/// shift for Number of lines of the video picture Encoded value
	static unsigned int KHsDISP_VID2_ORGSIZEY                   = 16 ;
	/// shift for Number of pixels of the video picture Encoded value
	static unsigned int KHsDISP_VID2_ORGSIZEX                   = 0 ;

/************************************* 60) DISPC_VID2_ACCUJ REGISTER. OFFSET : 0x178-0x17C ****************************************/

/// configures the resize accumulator init values for horizontal and
/// vertical up/down-sampling of the video window
static unsigned int KHoDISP_VID2_ACCU0          				= 0x178 ;
static unsigned int KHoDISP_VID2_ACCU1          				= 0x17C ;
	/// shift for Vertical initialization accu value
	static unsigned int KHsDISP_VID2_VERTICALACCU           	= 16 ;
	/// shift for Horizontal initialization accu value
	static unsigned int KHsDISP_VID2_HORIZONTALACCU         	= 0 ;

/************************************* 61) DISPC_VID2_FIR_COEF_H_I REGISTER. OFFSET : 0x180 - 0x1B8 ****************************************/

/// The bank of registers configure the up/down-scaling coefficients for
/// the vertical and horizontal resize of the video picture associated with
/// the video window
static unsigned int KHoDISP_VID2_FIR_COEF_H0    				= 0x180 ;
static unsigned int KHoDISP_VID2_FIR_COEF_H1    				= 0x188 ;
static unsigned int KHoDISP_VID2_FIR_COEF_H2    				= 0x190 ;
static unsigned int KHoDISP_VID2_FIR_COEF_H3    				= 0x198 ;
static unsigned int KHoDISP_VID2_FIR_COEF_H4    				= 0x1A0 ;
static unsigned int KHoDISP_VID2_FIR_COEF_H5    				= 0x1A8 ;
static unsigned int KHoDISP_VID2_FIR_COEF_H6    				= 0x1B0 ;
static unsigned int KHoDISP_VID2_FIR_COEF_H7    				= 0x1B8 ;

	/// shift for Signed coefficient C3 for the horizontal up/downscaling
	/// with the phase n
	static unsigned int KHsDISP_VID2_FIRHC3             		= 24 ;
	/// shift for Signed coefficient C2 for the horizontal up/downscaling
	/// with the phase n
	static unsigned int KHsDISP_VID2_FIRHC2             		= 16 ;
	/// shift for Signed coefficient C1 for the horizontal up/downscaling
	/// with the phase n
	static unsigned int KHsDISP_VID2_FIRHC1             		= 8 ;
	/// shift for Signed coefficient C0 for the horizontal up/downscaling
	/// with the phase n
	static unsigned int KHsDISP_VID2_FIRHC0             		= 0 ;

/************************************* 62) DISPC_VID2_FIR_COEF_HV_I REGISTER. OFFSET : 0x184 - 0x1BC ****************************************/

/// The bank of registers configure the down/up/down-scaling coefficients
/// for the vertical and horizontal resize of the video picture associated
/// with the video window
static unsigned int KHoDISP_VID2_FIR_COEF_HV0   				= 0x184 ;
static unsigned int KHoDISP_VID2_FIR_COEF_HV1   				= 0x18C ;
static unsigned int KHoDISP_VID2_FIR_COEF_HV2   				= 0x194 ;
static unsigned int KHoDISP_VID2_FIR_COEF_HV3   				= 0x19C ;
static unsigned int KHoDISP_VID2_FIR_COEF_HV4   				= 0x1A4 ;
static unsigned int KHoDISP_VID2_FIR_COEF_HV5   				= 0x1AC ;
static unsigned int KHoDISP_VID2_FIR_COEF_HV6   				= 0x1B4 ;
static unsigned int KHoDISP_VID2_FIR_COEF_HV7   				= 0x1BC ;

	/// shift for Signed coefficient C2 for the vertical up/downscaling
	/// with the phase n
	static unsigned int KHsDISP_VID2_FIRVC2             		= 24 ;
	/// shift for Signed coefficient C1 for the vertical up/downscaling
	/// with the phase n
	static unsigned int KHsDISP_VID2_FIRVC1             		= 16 ;
	/// shift for Signed coefficient C0 for the vertical up/downscaling
	/// with the phase n
	static unsigned int KHsDISP_VID2_FIRVC0             		= 8 ;
	/// shift for Signed coefficient C4 for the horizontal up/downscaling
	/// with the phase n
	static unsigned int KHsDISP_VID2_FIRHC4             		= 0 ;

/************************************* 63) DISPC_VID2_CONV_COEF0 REGISTER. OFFSET : 0x1C0 ****************************************/

/// configures the color space conversion matrix coefficients for the video
/// pipeline
static unsigned int KHoDISP_VID2_CONV_COEF0 = 0x1C0 ;

/************************************* 64) DISPC_VID2_CONV_COEF1 REGISTER. OFFSET : 0x1C4 ****************************************/

/// configures the color space conversion matrix coefficients for the video
/// pipeline
static unsigned int KHoDISP_VID2_CONV_COEF1     = 0x1C4 ;

/************************************* 65) DISPC_VID2_CONV_COEF2 REGISTER. OFFSET : 0x1C8 ****************************************/

/// configures the color space conversion matrix coefficients for the video
/// pipeline
static unsigned int KHoDISP_VID2_CONV_COEF2     = 0x1C8 ;

/************************************* 66) DISPC_VID2_CONV_COEF3 REGISTER. OFFSET : 0x1CC ****************************************/

/// configures the color space conversion matrix coefficients for the video
/// pipeline
static unsigned int KHoDISP_VID2_CONV_COEF3     = 0x1CC ;

/************************************* 67) DISPC_VID2_CONV_COEF4 REGISTER. OFFSET : 0x1D0 ****************************************/

/// configures the color space conversion matrix coefficients for the video
/// pipeline
static unsigned int KHoDISP_VID2_CONV_COEF4     = 0x1D0 ;


/************************************* 68) DISPC_DATA1_CYCLEJ REGISTER. OFFSET : 0x1D4 - 0x1DC ****************************************/

/// configures the output data format for 1st cycle
static unsigned int KHoDISP_DATA1_CYCLE1            = 0x1D4 ;
/// configures the output data format for 2nd cycle.
static unsigned int KHoDISP_DATA1_CYCLE2            = 0x1D8 ;
/// configures the output data format for 3rd cycle.
static unsigned int KHoDISP_DATA1_CYCLE3            = 0x1DC ;

	/// shift for Alignment of the bits from pixel#2 on the output interface
	static unsigned int KHsDISP_BITALIGNMENTPIXEL2      = 24 ;
	/// shift for Number of bits from the pixel #2
	static unsigned int KHsDISP_NBBITSPIXEL2            = 16 ;
	/// shift for Alignment of the bits from pixel#2 on the output interface
	static unsigned int KHsDISP_BITALIGNMENTPIXEL1      = 8 ;
	/// shift for Number of bits from the pixel #1
	static unsigned int KHsDISP_NBBITSPIXEL1            = 0 ;

/************************************* 69) DISPC_VID1_FIR_COEF_VI REGISTER. OFFSET : 0x1E0 - 0x1FC ****************************************/

/// The bank of registers configure the up-/down-scaling coefficients
/// for the vertical of the video picture associated with the video window
/// These registers have the extra 2 taps of 5-tap vertical coefficients
static unsigned int KHoDISP_VID1_FIR_COEF_V0   = 0x1E0 ;
static unsigned int KHoDISP_VID1_FIR_COEF_V1   = 0x1E4 ;
static unsigned int KHoDISP_VID1_FIR_COEF_V2   = 0x1E8 ;
static unsigned int KHoDISP_VID1_FIR_COEF_V3   = 0x1EC ;
static unsigned int KHoDISP_VID1_FIR_COEF_V4   = 0x1F0 ;
static unsigned int KHoDISP_VID1_FIR_COEF_V5   = 0x1F4 ;
static unsigned int KHoDISP_VID1_FIR_COEF_V6   = 0x1F8 ;
static unsigned int KHoDISP_VID1_FIR_COEF_V7   = 0x1FC ;

	/// shift for Signed coefficient C00 for the vertical up/downscaling
	/// with the phase n
	static unsigned int KHsDISP_VID1_FIRVC00           = 0 ;

	/// shift for Signed coefficient C22 for the vertical up/downscaling
	/// with the phase n
	static unsigned int KHsDISP_VID1_FIRVC22           = 8;

/************************************* 70) DISPC_VID2_FIR_COEF_VI REGISTER. OFFSET : 0x200 - 0x21C ****************************************/

/// The bank of registers configure the up-/down-scaling coefficients
/// for the vertical resize of the video picture associated with the video window
/// These registers have the extra 2 taps of 5-tap vertical coefficients
static unsigned int KHoDISP_VID2_FIR_COEF_V0   = 0x200 ;
static unsigned int KHoDISP_VID2_FIR_COEF_V1   = 0x204 ;
static unsigned int KHoDISP_VID2_FIR_COEF_V2   = 0x208 ;
static unsigned int KHoDISP_VID2_FIR_COEF_V3   = 0x20C ;
static unsigned int KHoDISP_VID2_FIR_COEF_V4   = 0x210 ;
static unsigned int KHoDISP_VID2_FIR_COEF_V5   = 0x214 ;
static unsigned int KHoDISP_VID2_FIR_COEF_V6   = 0x218 ;
static unsigned int KHoDISP_VID2_FIR_COEF_V7   = 0x21C ;

	/// shift for Signed coefficient C00 for the vertical up/downscaling
	/// with the phase n
	static unsigned int KHsDISP_VID2_FIRVC00           = 0 ;

	/// shift for Signed coefficient C22 for the vertical up/downscaling
	/// with the phase n
	static unsigned int KHsDISP_VID2_FIRVC22           = 8;

/************************************* 71) DISPC_GFX_PRELOAD REGISTER. OFFSET : 0x22C ****************************************/

static unsigned int KHoDISP_GFX_PRELOAD         = 0x22C ;

/************************************* 72) DISPC_VID1_PRELOAD REGISTER. OFFSET : 0x230 ****************************************/

static unsigned int KHoDISP_VID1_PRELOAD        = 0x230 ;

/************************************* 73) DISPC_VID2_PRELOAD REGISTER. OFFSET : 0x234 ****************************************/

static unsigned int KHoDISP_VID2_PRELOAD        = 0x234 ;

//******************************************************************************************************************************
//******************************************************************************************************************************
//*************************************** VIDEO PIPELINE 3 SPECIFIC REGISTER DEFINITIONS ***************************************
//******************************************************************************************************************************
//******************************************************************************************************************************

/************************************* 74) DISPC_VID3_ACCUJ REGISTER. OFFSET : 0x300-0x304 ****************************************/

/// configures the resize accumulator init values for horizontal and
/// vertical up/down-sampling of the video window
static unsigned int KHoDISP_VID3_ACCU0          				= 0x300 ;
static unsigned int KHoDISP_VID3_ACCU1          				= 0x304 ;
	/// shift for Vertical initialization accu value
	static unsigned int KHsDISP_VID3_VERTICALACCU           	= 16 ;
	/// shift for Horizontal initialization accu value
	static unsigned int KHsDISP_VID3_HORIZONTALACCU         	= 0 ;

/************************************* 75) DISPC_VID3_BA REGISTER. OFFSET : 0x308 ****************************************/

/// VIdeo 3 Base Address
static unsigned int KHoDISP_VID3_BA0        = 0x308 ;
static unsigned int KHoDISP_VID3_BA1        = 0x30C ;

/************************************* 76) DISPC_VID3_FIR_COEF_H_I REGISTER. OFFSET : 0x310 - 0x348 ****************************************/

/// The bank of registers configure the up/down-scaling coefficients for
/// the vertical and horizontal resize of the video picture associated with
/// the video window
static unsigned int KHoDISP_VID3_FIR_COEF_H0    				= 0x310 ;
static unsigned int KHoDISP_VID3_FIR_COEF_H1    				= 0x318 ;
static unsigned int KHoDISP_VID3_FIR_COEF_H2    				= 0x320 ;
static unsigned int KHoDISP_VID3_FIR_COEF_H3    				= 0x328 ;
static unsigned int KHoDISP_VID3_FIR_COEF_H4    				= 0x330 ;
static unsigned int KHoDISP_VID3_FIR_COEF_H5    				= 0x338 ;
static unsigned int KHoDISP_VID3_FIR_COEF_H6    				= 0x340 ;
static unsigned int KHoDISP_VID3_FIR_COEF_H7    				= 0x348 ;

	/// shift for Signed coefficient C3 for the horizontal up/downscaling
	/// with the phase n
	static unsigned int KHsDISP_VID3_FIRHC3             		= 24 ;
	/// shift for Signed coefficient C2 for the horizontal up/downscaling
	/// with the phase n
	static unsigned int KHsDISP_VID3_FIRHC2             		= 16 ;
	/// shift for Signed coefficient C1 for the horizontal up/downscaling
	/// with the phase n
	static unsigned int KHsDISP_VID3_FIRHC1             		= 8 ;
	/// shift for Signed coefficient C0 for the horizontal up/downscaling
	/// with the phase n
	static unsigned int KHsDISP_VID3_FIRHC0             		= 0 ;

/************************************* 77) DISPC_VID3_FIR_COEF_HV_I REGISTER. OFFSET : 0x314 - 0x34C ****************************************/

/// The bank of registers configure the down/up/down-scaling coefficients
/// for the vertical and horizontal resize of the video picture associated
/// with the video window
static unsigned int KHoDISP_VID3_FIR_COEF_HV0   				= 0x314 ;
static unsigned int KHoDISP_VID3_FIR_COEF_HV1   				= 0x31C ;
static unsigned int KHoDISP_VID3_FIR_COEF_HV2   				= 0x324 ;
static unsigned int KHoDISP_VID3_FIR_COEF_HV3   				= 0x32C ;
static unsigned int KHoDISP_VID3_FIR_COEF_HV4   				= 0x334 ;
static unsigned int KHoDISP_VID3_FIR_COEF_HV5   				= 0x33C ;
static unsigned int KHoDISP_VID3_FIR_COEF_HV6   				= 0x344 ;
static unsigned int KHoDISP_VID3_FIR_COEF_HV7   				= 0x34C ;

	/// shift for Signed coefficient C2 for the vertical up/downscaling
	/// with the phase n
	static unsigned int KHsDISP_VID3_FIRVC2             		= 24 ;
	/// shift for Signed coefficient C1 for the vertical up/downscaling
	/// with the phase n
	static unsigned int KHsDISP_VID3_FIRVC1             		= 16 ;
	/// shift for Signed coefficient C0 for the vertical up/downscaling
	/// with the phase n
	static unsigned int KHsDISP_VID3_FIRVC0             		= 8 ;
	/// shift for Signed coefficient C4 for the horizontal up/downscaling
	/// with the phase n
	static unsigned int KHsDISP_VID3_FIRHC4             		= 0 ;

/************************************* 78) DISPC_VID3_FIR_COEF_VI REGISTER. OFFSET : 0x350 - 0x36C ****************************************/

/// The bank of registers configure the up-/down-scaling coefficients
/// for the vertical of the video picture associated with the video window
/// These registers have the extra 2 taps of 5-tap vertical coefficients
static unsigned int KHoDISP_VID3_FIR_COEF_V0   = 0x350 ;
static unsigned int KHoDISP_VID3_FIR_COEF_V1   = 0x354 ;
static unsigned int KHoDISP_VID3_FIR_COEF_V2   = 0x358 ;
static unsigned int KHoDISP_VID3_FIR_COEF_V3   = 0x35C ;
static unsigned int KHoDISP_VID3_FIR_COEF_V4   = 0x360 ;
static unsigned int KHoDISP_VID3_FIR_COEF_V5   = 0x364 ;
static unsigned int KHoDISP_VID3_FIR_COEF_V6   = 0x368 ;
static unsigned int KHoDISP_VID3_FIR_COEF_V7   = 0x36C ;

	/// shift for Signed coefficient C00 for the vertical up/downscaling
	/// with the phase n
	static unsigned int KHsDISP_VID3_FIRVC00           = 0 ;

	/// shift for Signed coefficient C22 for the vertical up/downscaling
	/// with the phase n
	static unsigned int KHsDISP_VID3_FIRVC22           = 8;

/************************************* 79) DISPC_VID3_ATTRIBUTE REGISTER. OFFSET : 0x370 ****************************************/

static unsigned int KHoDISP_VID3_ATTR   = 0x370 ;

	/// LCD channel selctor (1/2/WB)
	static unsigned int KHsDISP_VID3_ATTRChOut2 				= 30;
	/// Pri LCD Selected
	static unsigned int KHtDISP_VID3_ATTRPriLCD 				= 0x0;
	/// Sec LCD Selected
	static unsigned int KHtDISP_VID3_ATTRSecLCD 				= 1<<30;
	/// Writeback O/P Selected
	static unsigned int KHtDISP_VID3_ATTRWbOut  				= 1<<30| 1<<31;
	/// Burst type (1D/2D)
	static unsigned int KHtDISP_VID3_ATTRBurstType 				= 1<<29;
	/// Z-Order defining priority of the layer
	static unsigned int KHsDISP_VID3_ATTRZOrder 				= 26;
	// Default order - 0
	static unsigned int KHtDISP_VID3_ATTRZOrder0 				= 0x0;
	// Z-Order 1
	static unsigned int KHtDISP_VID3_ATTRZOrder1 				= 0x1;
	// Z-Order 2
	static unsigned int KHtDISP_VID3_ATTRZOrder2 				= 0x2;
	// Z-Order 3
	static unsigned int KHtDISP_VID3_ATTRZOrder3 				= 0x3;
	/// Z-Order Enable
	static unsigned int KHtDISP_VID3_ATTRZOrderEn 				= 1<<25;
	/// Enables Self-Refresh
	static unsigned int KHtDISP_VID3_ATTRSelfRefresh 			= 1<<24;
	/// Arbitration (Priority of Vid1 Pipe)
	static unsigned int KHtDISP_VID3_ATTRArbitration 			= 1<<23;
	/// Double Stride (Only for YUV420)
	static unsigned int KHtDISP_VID3_ATTRDblStride 				= 1<<22;
	/// Video vertical resize tap number
	static unsigned int KHtDISP_VID3_ATTRVertTaps            	= 1<<21;
	/// Video optimization (Not in use)
	static unsigned int KHtDISP_VID3_ATTRDmaOptimize     		= 1<<20;
	/// Video preload value
	static unsigned int KHtDISP_VID3_ATTRFifoPreload        	= 1<<19;
	/// Self Refresh Auto
	static unsigned int KHtDISP_VID3_ATTRSelfRefAuto        	= 1<<17 ;
	/// 24 bit output selected
	static unsigned int KHtDISP_VID3_ATTR24bOutputEn        	= 1<<16 ;
	/// Video1 DMA Burst Size (4*128)
	static unsigned int KHtDISP_VID3_ATTRBurst4_128      		= 1<<14  ;
	/// Video1 DMA Burst Size (8*128)
	static unsigned int KHtDISP_VID3_ATTRBurst8_128     		= 1<<15  ;
	/// Video1 DMA Burst Size (2*128)
	static unsigned int KHtDISP_VID3_ATTRBurst2_128      		= 0;
	/// Rotation 0 degree
	static unsigned int KHtDISP_VID3_ATTRRotation_0     		= 0;
	/// Rotation 90 degree
	static unsigned int KHtDISP_VID3_ATTRRotation_90          	= 1<<12  ;
	/// Rotation 180 degree
	static unsigned int KHtDISP_VID3_ATTRRotation_180          	= 1<<13  ;
	/// Rotation 270 degree
	static unsigned int KHtDISP_VID3_ATTRRotation_270          	= 1<<12 | 1<<13  ;
	/// Full Range
	static unsigned int KHtDISP_VID3_ATTRFullRange          	= 1<<11  ;
	/// Enable video1 replication logic
	static unsigned int KHtDISP_VID3_ATTRRep_En          		= 1<<10  ;
	/// Enable Color Conversion
	static unsigned int KHtDISP_VID3_ATTRColorConv_En   		= 1<<9;
	/// Vertical Downsampling
	static unsigned int KHtDISP_VID3_ATTRResize_Ver_Conf    	= 1<<8;
	/// Horizontal Downsampling
	static unsigned int KHtDISP_VID3_ATTRResize_Hor_Conf    	= 1<<7;
	/// Horizontal Resize Enable
	static unsigned int KHtDISP_VID3_ATTRResize_Hor_En  		= 1<<5;
	/// Vertical Resize Enable
	static unsigned int KHtDISP_VID3_ATTRResize_Ver_En  		= 1<<6;
	/// video1 format YUV2
	static unsigned int KHtDISP_VID3_ATTRVid1FormatYUV2 		= 1<<4 | 1<<2 ;
	/// video1 format UYVY
	static unsigned int KHtDISP_VID3_ATTRVid1FormatUYVY 		= 1<<4 | 1<<2 | 1<<1;
	/// video1 format RGB16
	static unsigned int KHtDISP_VID3_ATTRVid1FormatRGB16    	= 1<<3 | 1<<2  ;
	///video1 format RGB24U
	static unsigned int KHtDISP_VID3_ATTRVid1FormatRGB24U 		= 1<<4;
	///video1 format RGB24
	static unsigned int KHtDISP_VID3_ATTRVid1FormatRGB24  		= 1<<4 | 1<<1;
	/// Video2 format ARGB16
	static unsigned int KHtDISP_VID3_ATTRVid2FormatARGB16   	= 1<<3 | 1<<1;
	/// Video2 format ARGB32
	static unsigned int KHtDISP_VID3_ATTRVid2FormatARGB32 		= 1<<4 | 1<<3;
	/// Video2 format RGBA32
	static unsigned int KHtDISP_VID3_ATTRVid2FormatRGBA32 		= 1<<4 | 1<<3 | 1<<1;
	///video3 format YUV420 Semi Planar
	static unsigned int KHtDISP_VID3_ATTRVid2FormatYUV420SP  	= 0<<1;
	/// Video1 window present on the screen
	static unsigned int KHtDISP_VID3_Enable                  	= 1<<0  ;

	//Double stride bit - for supporting YUV420 NV12
	static unsigned int KHtDISP_VID3_ATTRDoubleStride        = 1<<22;

/************************************* 80) DISPC_VID3_CONV_COEF0 REGISTER. OFFSET : 0x374 ****************************************/

/// configures the color space conversion matrix coefficients for the video
/// pipeline
static unsigned int KHoDISP_VID3_CONV_COEF0 = 0x374 ;

/************************************* 81) DISPC_VID3_CONV_COEF1 REGISTER. OFFSET : 0x378 ****************************************/

/// configures the color space conversion matrix coefficients for the video
/// pipeline
static unsigned int KHoDISP_VID3_CONV_COEF1     = 0x378 ;

/************************************* 82) DISPC_VID3_CONV_COEF2 REGISTER. OFFSET : 0x37C ****************************************/

/// configures the color space conversion matrix coefficients for the video
/// pipeline
static unsigned int KHoDISP_VID3_CONV_COEF2     = 0x37C ;

/************************************* 83) DISPC_VID3_CONV_COEF3 REGISTER. OFFSET : 0x380 ****************************************/

/// configures the color space conversion matrix coefficients for the video
/// pipeline
static unsigned int KHoDISP_VID3_CONV_COEF3     = 0x380 ;

/************************************* 84) DISPC_VID3_CONV_COEF4 REGISTER. OFFSET : 0x384 ****************************************/

/// configures the color space conversion matrix coefficients for the video
/// pipeline
static unsigned int KHoDISP_VID3_CONV_COEF4     = 0x384 ;

/************************************* 85) DISPC_VID3_BUF_SIZE_STATUS REGISTER. OFFSET : 0x388 ****************************************/

/// defines the Video FIFO size for the video pipeline
static unsigned int KHoDISP_VID3_FIFO_SIZE_STATUS     			= 0x388 ;

/************************************* 86) DISPC_VID3_THRESHOLD REGISTER. OFFSET : 0x38C ****************************************/

static unsigned int KHoDISP_VID3_THRESHOLD  = 0x38C ;

	/// Shift for the high threshold value
	static unsigned int KHsDISP_VID3_THRESHOLDHigh 				=        16   ;
	/// No of bytes defining the high threshold value
	static unsigned int KHtDISP_VID3_THRESHOLDHigh 				=        0x3FC ;
	/// Shift for the low threshold value
	static unsigned int KHsDISP_VID3_THRESHOLDLow  				=        0    ;
	#ifdef ENABLE_DSS_SMARTSTANDBY
	// Number of bytes defining the low threshold value
	static unsigned int KHtDISP_VID3_THRESHOLDLow  				=        0x3BC ;
	#else
	// OMAPS00138607 Number of bytes defining the low threshold value
	static unsigned int KHtDISP_VID3_THRESHOLDLow  				=        0x3FC -64 ;
	#endif

/************************************* 87) DISPC_VID3_FIR REGISTER. OFFSET : 0x390 ****************************************/

/// configures the resize factors for horizontal and vertical
/// up/down-sampling of the video window
static unsigned int KHoDISP_VID3_FIR            				= 0x390 ;

	/// shift for Vertical increment of the up/down-sampling filter
	static unsigned int KHsDISP_VID3_FIRVINC                	= 16 ;
	/// shift for Horizontal increment of the up/down-sampling filter
	static unsigned int KHsDISP_VID3_FIRHINC                	= 0 ;

/************************************* 88) DISPC_VID3_PICTURE_SIZE REGISTER. OFFSET : 0x394 ****************************************/

/// configure the size of the video picture associated with the video layer
static unsigned int KHoDISP_VID3_PICTURE_SIZE   				= 0x394 ;

	/// shift for Number of lines of the video picture Encoded value
	static unsigned int KHsDISP_VID3_ORGSIZEY                   = 16 ;
	/// shift for Number of pixels of the video picture Encoded value
	static unsigned int KHsDISP_VID3_ORGSIZEX                   = 0 ;

/************************************* 89) DISPC_VID3_PIXEL_INCR REGISTER. OFFSET : 0x398 ****************************************/

static unsigned int KHoDISP_VID3_PIXEL_INC   					= 0x398 ;

	/// Number of bytes to increment between 2 pixels.
	static unsigned int KHtDISP_VID3_PIXEL_INCValue  			=     0x1  ;

/************************************* 90) DISPC_VID3_POS REGISTER. OFFSET : 0x39C ****************************************/

static unsigned int KHoDISP_VID3_POS        = 0x39C ;

	// POS X Shift
	static unsigned int KHsDISP_VID3_POSGfxPosX = 0  ;
	// POS X
	static unsigned int KHtDISP_VID3_POSGfxPosX = 0  ;
	// POS Y Shift
	static unsigned int KHsDISP_VID3_POSGfxPosY = 16 ;
	// POS Y
	static unsigned int KHtDISP_VID3_POSGfxPosY = 0  ;

/************************************* 91) DISPC_VID3_PRELOAD REGISTER. OFFSET : 0x3A0 ****************************************/

static unsigned int KHoDISP_VID3_PRELOAD        = 0x3A0 ;

/************************************* 92) DISPC_VID3_ROW_INCR REGISTER. OFFSET : 0x3A4 ****************************************/

static unsigned int KHoDISP_VID3_ROW_INC     					= 0x3A4 ;

	/// Number of bytes to skip at the end of the row
	static unsigned int KHtDISP_VID3_ROW_INCValue  				=     0x1  ;

/************************************* 93) DISPC_VID3_SIZE REGISTER. OFFSET : 0x3A8 ****************************************/

static unsigned int KHoDISP_VID3_SIZE   = 0x3A8 ;

	// Shift of the X position of the video1 window
	static unsigned int KHsDISP_VID3_SIZEGfxSizeX = 0    ;
	// Shift of the Y position of the video1 window
	static unsigned int KHsDISP_VID3_SIZEGfxSizeY = 16   ;

//******************************************************************************************************************************
//******************************************************************************************************************************
//************************************** WRITE-BACK PIPELINE SPECIFIC REGISTER DEFINITIONS *************************************
//******************************************************************************************************************************
//******************************************************************************************************************************

/************************************* 94) DISPC_WB_ACCUJ REGISTER. OFFSET : 0x500-0x504 ****************************************/

/// configures the resize accumulator init values for horizontal and
/// vertical up/down-sampling of the WB window
static unsigned int KHoDISP_WB_ACCU0          				= 0x500 ;
static unsigned int KHoDISP_WB_ACCU1          				= 0x504 ;

	/// shift for Vertical initialization accu value
	static unsigned int KHsDISP_WB_VERTICALACCU           	= 16 ;
	/// shift for Horizontal initialization accu value
	static unsigned int KHsDISP_WB_HORIZONTALACCU         	= 0 ;

/************************************* 95) DISPC_WB_BA REGISTER. OFFSET : 0x508 ****************************************/

/// WB Base Address
static unsigned int KHoDISP_WB_BA0        = 0x508 ;
static unsigned int KHoDISP_WB_BA1        = 0x50C ;

/************************************* 96) DISPC_WB_FIR_COEF_H_I REGISTER. OFFSET : 0x510 - 0x548 ****************************************/

/// The bank of registers configure the up/down-scaling coefficients for
/// the vertical and horizontal resize of the video picture associated with
/// the video window
static unsigned int KHoDISP_WB_FIR_COEF_H0    				= 0x510 ;
static unsigned int KHoDISP_WB_FIR_COEF_H1    				= 0x518 ;
static unsigned int KHoDISP_WB_FIR_COEF_H2    				= 0x520 ;
static unsigned int KHoDISP_WB_FIR_COEF_H3    				= 0x528 ;
static unsigned int KHoDISP_WB_FIR_COEF_H4    				= 0x530 ;
static unsigned int KHoDISP_WB_FIR_COEF_H5    				= 0x538 ;
static unsigned int KHoDISP_WB_FIR_COEF_H6    				= 0x540 ;
static unsigned int KHoDISP_WB_FIR_COEF_H7    				= 0x548 ;

	/// shift for Signed coefficient C3 for the horizontal up/downscaling
	/// with the phase n
	static unsigned int KHsDISP_WB_FIRHC3             		= 24 ;
	/// shift for Signed coefficient C2 for the horizontal up/downscaling
	/// with the phase n
	static unsigned int KHsDISP_WB_FIRHC2             		= 16 ;
	/// shift for Signed coefficient C1 for the horizontal up/downscaling
	/// with the phase n
	static unsigned int KHsDISP_WB_FIRHC1             		= 8 ;
	/// shift for Signed coefficient C0 for the horizontal up/downscaling
	/// with the phase n
	static unsigned int KHsDISP_WB2_FIRHC0             		= 0 ;

/************************************* 97) DISPC_WB_FIR_COEF_HV_I REGISTER. OFFSET : 0x514 - 0x54C ****************************************/

/// The bank of registers configure the down/up/down-scaling coefficients
/// for the vertical and horizontal resize of the video picture associated
/// with the video window
static unsigned int KHoDISP_WB_FIR_COEF_HV0   				= 0x514 ;
static unsigned int KHoDISP_WB_FIR_COEF_HV1   				= 0x51C ;
static unsigned int KHoDISP_WB_FIR_COEF_HV2   				= 0x524 ;
static unsigned int KHoDISP_WB_FIR_COEF_HV3   				= 0x52C ;
static unsigned int KHoDISP_WB_FIR_COEF_HV4   				= 0x534 ;
static unsigned int KHoDISP_WB_FIR_COEF_HV5   				= 0x53C ;
static unsigned int KHoDISP_WB_FIR_COEF_HV6   				= 0x544 ;
static unsigned int KHoDISP_WB_FIR_COEF_HV7   				= 0x54C ;

	/// shift for Signed coefficient C2 for the vertical up/downscaling
	/// with the phase n
	static unsigned int KHsDISP_WB_FIRVC2             		= 24 ;
	/// shift for Signed coefficient C1 for the vertical up/downscaling
	/// with the phase n
	static unsigned int KHsDISP_WB_FIRVC1             		= 16 ;
	/// shift for Signed coefficient C0 for the vertical up/downscaling
	/// with the phase n
	static unsigned int KHsDISP_WB_FIRVC0             		= 8 ;
	/// shift for Signed coefficient C4 for the horizontal up/downscaling
	/// with the phase n
	static unsigned int KHsDISP_WB_FIRHC4             		= 0 ;

/************************************* 98) DISPC_WB_FIR_COEF_VI REGISTER. OFFSET : 0x550 - 0x56C ****************************************/

/// The bank of registers configure the up-/down-scaling coefficients
/// for the vertical resize of the video picture associated with the video window
/// These registers have the extra 2 taps of 5-tap vertical coefficients
static unsigned int KHoDISP_WB_FIR_COEF_V0   = 0x550 ;
static unsigned int KHoDISP_WB_FIR_COEF_V1   = 0x554 ;
static unsigned int KHoDISP_WB_FIR_COEF_V2   = 0x558 ;
static unsigned int KHoDISP_WB_FIR_COEF_V3   = 0x55C ;
static unsigned int KHoDISP_WB_FIR_COEF_V4   = 0x560 ;
static unsigned int KHoDISP_WB_FIR_COEF_V5   = 0x564 ;
static unsigned int KHoDISP_WB_FIR_COEF_V6   = 0x568 ;
static unsigned int KHoDISP_WB_FIR_COEF_V7   = 0x56C ;

	/// shift for Signed coefficient C00 for the vertical up/downscaling
	/// with the phase n
	static unsigned int KHsDISP_WB_FIRVC00           = 0 ;

	/// shift for Signed coefficient C22 for the vertical up/downscaling
	/// with the phase n
	static unsigned int KHsDISP_WB_FIRVC22           = 8;

/************************************* 99) DISPC_WB_ATTRIBUTE REGISTER. OFFSET : 0x570 ****************************************/

static unsigned int KHoDISP_WB_ATTR   = 0x570 ;

	/// Shift No of Idles b/w requests on L3
	static unsigned int KHsDISP_WB_ATTRIdleNo 				= 28;
	/// No of Idles b/w requests on L3
	static unsigned int KHtDISP_WB_ATTRIdleNo 				= 0x0;
	/// Idle Size
	static unsigned int KHtDISP_WB_ATTRIdleSize 			= 1<<27;
	/// Capture Mode
	static unsigned int KHsDISP_WB_ATTRCapMode 				= 24;
	/// All Frames captured
	static unsigned int KHtDISP_WB_ATTRAllFrames 			= 0x0;
	/// One Frame captured
	static unsigned int KHtDISP_WB_ATTROneFrame 			= 0x1;
	/// Arbitration (Priority of WB Pipe)
	static unsigned int KHtDISP_WB_ATTRArbitration 			= 1<<23;
	/// Double Stride (Only for YUV420)
	static unsigned int KHtDISP_WB_ATTRDblStride 			= 1<<22;
	/// Video vertical resize tap number
	static unsigned int KHtDISP_WB_ATTRVertTaps            	= 1<<21;
	/// Write Back Mode (Capture/Mem-Mem)
	static unsigned int KHtDISP_WB_ATTRWbMemToMemMode       = 1<<19;
	/// Video Channel In Config
	static unsigned int KHsDISP_WB_ATTRChannelIn            = 16;
	/// Pri LCD Output
	static unsigned int KHtDISP_WB_ATTRPriOut            	= 0x0;
	/// Sec LCD Output
	static unsigned int KHtDISP_WB_ATTRSecOut            	= 1<<16;
	/// TV Output
	static unsigned int KHtDISP_WB_ATTRTvOut            	= 1<<17;
	/// Gfx Pipe Output
	static unsigned int KHtDISP_WB_ATTRGfxOut            	= 1<<16 | 1<<17;
	/// Vid1 Pipe Output
	static unsigned int KHtDISP_WB_ATTRVid1Out            	= 1<<18;
	/// Vid2 Pipe Output
	static unsigned int KHtDISP_WB_ATTRVid2Out            	= 1<<18 | 1<<16;
	/// Vid3 Pipe Output
	static unsigned int KHtDISP_WB_ATTRVid3Out            	= 1<<18 | 1<<17;
	/// WB DMA Burst Size (4*128)
	static unsigned int KHtDISP_WB_ATTRBurst4_128      		= 1<<14  ;
	/// WB DMA Burst Size (8*128)
	static unsigned int KHtDISP_WB_ATTRBurst8_128     		= 1<<15  ;
	/// WB DMA Burst Size (2*128)
	static unsigned int KHtDISP_WB_ATTRBurst2_128      		= 0;
	/// Rotation 0 degree
	static unsigned int KHtDISP_WB_ATTRRotation_0     		= 0;
	/// Rotation 90 degree
	static unsigned int KHtDISP_WB_ATTRRotation_90          = 1<<12  ;
	/// Rotation 180 degree
	static unsigned int KHtDISP_WB_ATTRRotation_180         = 1<<13  ;
	/// Rotation 270 degree
	static unsigned int KHtDISP_WB_ATTRRotation_270         = 1<<12 | 1<<13  ;
	/// Full Range
	static unsigned int KHtDISP_WB_ATTRFullRange          	= 1<<11  ;
	/// Enable Truncation
	static unsigned int KHtDISP_WB_ATTRTrunc_En          	= 1<<10  ;
	/// Enable Color Conversion
	static unsigned int KHtDISP_WB_ATTRColorConv_En   		= 1<<9;
	/// Burst Type (1D/2D)
	static unsigned int KHtDISP_WB_ATTRBurst_Type    		= 1<<8;
	/// PreMultiplied Alpha Enable
	static unsigned int KHtDISP_WB_ATTRPreMul_Alpha_En    	= 1<<7;
	/// Horizontal Resize Enable
	static unsigned int KHtDISP_WB_ATTRResize_Hor_En  		= 1<<5;
	/// Vertical Resize Enable
	static unsigned int KHtDISP_WB_ATTRResize_Ver_En  		= 1<<6;
	/// video1 format YUV2
	static unsigned int KHtDISP_WB_ATTRVid1FormatYUV2 		= 1<<4 | 1<<2 ;
	/// video1 format UYVY
	static unsigned int KHtDISP_WB_ATTRVid1FormatUYVY 		= 1<<4 | 1<<2 | 1<<1;
	/// video1 format RGB16
	static unsigned int KHtDISP_WB_ATTRVid1FormatRGB16    	= 1<<3 | 1<<2  ;
	///video1 format RGB24U
	static unsigned int KHtDISP_WB_ATTRVid1FormatRGB24U 	= 1<<4;
	///video1 format RGB24
	static unsigned int KHtDISP_WB_ATTRVid1FormatRGB24  	= 1<<4 | 1<<1;
	/// Video2 format ARGB16
	static unsigned int KHtDISP_WB_ATTRWBFormatARGB16   	= 1<<3 | 1<<1;
	/// Video2 format ARGB32
	static unsigned int KHtDISP_WB_ATTRWBFormatARGB32 		= 1<<4 | 1<<3;
	/// Video2 format RGBA32
	static unsigned int KHtDISP_WB_ATTRWBFormatRGBA32 		= 1<<4 | 1<<3 | 1<<1;
	/// Video1 window present on the screen
	static unsigned int KHtDISP_WB_Enable                  	= 1<<0  ;

/************************************* 100) DISPC_WB_CONV_COEF0 REGISTER. OFFSET : 0x574 ****************************************/

/// configures the color space conversion matrix coefficients for the video
/// pipeline
static unsigned int KHoDISP_WB_CONV_COEF0 = 0x574 ;

/************************************* 101) DISPC_WB_CONV_COEF1 REGISTER. OFFSET : 0x578 ****************************************/

/// configures the color space conversion matrix coefficients for the video
/// pipeline
static unsigned int KHoDISP_WB_CONV_COEF1     = 0x578 ;

/************************************* 102) DISPC_WB_CONV_COEF2 REGISTER. OFFSET : 0x57C ****************************************/

/// configures the color space conversion matrix coefficients for the video
/// pipeline
static unsigned int KHoDISP_WB_CONV_COEF2     = 0x57C ;

/************************************* 103) DISPC_WB_CONV_COEF3 REGISTER. OFFSET : 0x580 ****************************************/

/// configures the color space conversion matrix coefficients for the video
/// pipeline
static unsigned int KHoDISP_WB_CONV_COEF3     = 0x580 ;

/************************************* 104) DISPC_WB_CONV_COEF4 REGISTER. OFFSET : 0x584 ****************************************/

/// configures the color space conversion matrix coefficients for the video
/// pipeline
static unsigned int KHoDISP_WB_CONV_COEF4     = 0x584 ;

/************************************* 105) DISPC_WB_BUF_SIZE_STATUS REGISTER. OFFSET : 0x588 ****************************************/

/// defines the WB FIFO size for the video pipeline
static unsigned int KHoDISP_WB_FIFO_SIZE_STATUS     			= 0x588 ;

/************************************* 106) DISPC_WB_THRESHOLD REGISTER. OFFSET : 0x58C ****************************************/

static unsigned int KHoDISP_WB_THRESHOLD  = 0x58C ;

	/// Shift for the high threshold value
	static unsigned int KHsDISP_WB_THRESHOLDHigh 				=        16   ;
	/// No of bytes defining the high threshold value
	static unsigned int KHtDISP_WB_THRESHOLDHigh 				=        0x3FC ;
	/// Shift for the low threshold value
	static unsigned int KHsDISP_WB_THRESHOLDLow  				=        0    ;
	#ifdef ENABLE_DSS_SMARTSTANDBY
	// Number of bytes defining the low threshold value
	static unsigned int KHtDISP_WB_THRESHOLDLow  				=        0x3BC ;
	#else
	// OMAPS00138607 Number of bytes defining the low threshold value
	static unsigned int KHtDISP_VWB_THRESHOLDLow  				=        0x3FC -64 ;
	#endif

/************************************* 107) DISPC_WB_FIR REGISTER. OFFSET : 0x590 ****************************************/

/// configures the resize factors for horizontal and vertical
/// up/down-sampling of the video window
static unsigned int KHoDISP_WB_FIR            				= 0x590 ;

	/// shift for Vertical increment of the up/down-sampling filter
	static unsigned int KHsDISP_WB_FIRVINC                	= 16 ;
	/// shift for Horizontal increment of the up/down-sampling filter
	static unsigned int KHsDISP_WB_FIRHINC                	= 0 ;

/************************************* 108) DISPC_WB_PICTURE_SIZE REGISTER. OFFSET : 0x594 ****************************************/

/// configure the size of the video picture associated with the video layer
static unsigned int KHoDISP_WB_PICTURE_SIZE   				= 0x594 ;

	/// shift for Number of lines of the video picture Encoded value
	static unsigned int KHsDISP_WB_ORGSIZEY                   = 16 ;
	/// shift for Number of pixels of the video picture Encoded value
	static unsigned int KHsDISP_WB_ORGSIZEX                   = 0 ;

/************************************* 109) DISPC_WB_PIXEL_INCR REGISTER. OFFSET : 0x598 ****************************************/

static unsigned int KHoDISP_WB_PIXEL_INC   					= 0x598 ;

	/// Number of bytes to increment between 2 pixels.
	static unsigned int KHtDISP_WB_PIXEL_INCValue  			=     0x1  ;

/************************************* 110) DISPC_WB_ROW_INCR REGISTER. OFFSET : 0x5A4 ****************************************/

static unsigned int KHoDISP_WB_ROW_INC     					= 0x5A4 ;

	/// Number of bytes to skip at the end of the row
	static unsigned int KHtDISP_WB_ROW_INCValue  				=     0x1  ;

/************************************* 111) DISPC_WB_SIZE REGISTER. OFFSET : 0x5A8 ****************************************/

static unsigned int KHoDISP_WB_SIZE   = 0x5A8 ;

	// Shift of the X position of the video1 window
	static unsigned int KHsDISP_WB_SIZEGfxSizeX = 0    ;
	// Shift of the Y position of the video1 window
	static unsigned int KHsDISP_WB_SIZEGfxSizeY = 16   ;

/************************************* 112) DISPC_VID1_BA_UV REGISTER. OFFSET : 0x600 ****************************************/

/// VIdeo 1 UV Base Address
static unsigned int KHoDISP_VID1_BA0_UV        = 0x600 ;
static unsigned int KHoDISP_VID1_BA1_UV        = 0x604 ;

/************************************* 113) DISPC_VID2_BA_UV REGISTER. OFFSET : 0x608 ****************************************/

/// VIdeo 2 UV Base Address
static unsigned int KHoDISP_VID2_BA0_UV        = 0x608 ;
static unsigned int KHoDISP_VID2_BA1_UV        = 0x60C ;

/************************************* 114) DISPC_VID3_BA_UV REGISTER. OFFSET : 0x610 ****************************************/

/// VIdeo 3 UV Base Address
static unsigned int KHoDISP_VID3_BA0_UV        = 0x610 ;
static unsigned int KHoDISP_VID3_BA1_UV        = 0x614 ;

/************************************* 115) DISPC_WB_BA_UV REGISTER. OFFSET : 0x618 ****************************************/

/// WB UV Base Address
static unsigned int KHoDISP_WB_BA0_UV        = 0x618 ;
static unsigned int KHoDISP_WB_BA1_UV        = 0x61C ;

//******************************************************************************************************************************
//******************************************************************************************************************************
//************************************** DISPLAY CONTROLLER 2 SPECIFIC REGISTER DEFINITIONS ************************************
//******************************************************************************************************************************
//******************************************************************************************************************************

/************************************* 116) DISPC_CONTROL2 REGISTER. OFFSET : 0x238 ****************************************/

/// allows to configure the display controller module.
static unsigned int KHoDISP_CONTROL2     						= 0x238 ;

	/// Spatial/Temporal Dithering No of Frames
	static unsigned int KHsDISP_CONTROL2_STDitheringFrames      = 30 ;
	/// State of unused bits (TDM mode only)
	static unsigned int KHsDISP_CONTROL2_TDMUNUSEDBITS      	= 25 ;
	/// Cycle format (TDM mode only)
	static unsigned int KHsDISP_CONTROL2_TDMCYCLEFORMAT     	= 23 ;
	/// Output Interface width (TDM mode only)
	static unsigned int KHsDISP_CONTROL2_TDMPARALLELMODE    	= 21 ;
	/// Enable the multiple cycle format
	static unsigned int KHtDISP_CONTROL2_TDMEn           		= 1<<20 ;
	/// TV Overlay Optimization
	static unsigned int KHtDISP_CONTROL2_TvOverLayOpt           = 1<<13 ;
	/// Overlay Optimisation
	static unsigned int KHtDISP_CONTROL2_OverLayOpt      		= 1<<12 ;
	/// RFBI Mode Select
	static unsigned int KHtDISP_CONTROL2_RFBIMODE        		= 1<<11 ;
	/// 16 bits output for the LCD
	static unsigned int KHtDISP_CONTROL2_TFTDataLines_16b 		= 1<<8 ;
	//18 bits output for the LCD
	static unsigned int KHtDISP_CONTROL2_TFTDataLines_18b 		=  1<<9 ;
	//24 bits output for the LCD
	static unsigned int KHtDISP_CONTROL2_TFTDataLines_24b 		= ( 1<<9 | 1<<8 );
	//Clear mask - corresponds to 12 Bits output for LCD
	static unsigned int KHtDISP_CONTROL2_TFTDataLinesClearMask 	= ( 1<<9 | 1<<8 );
	/// Spatial/Temporal dithering enable
	static unsigned int KHtDISP_CONTROL2_STDitheringEn   		= 1<<7 ;
	/// WB GO Command
	static unsigned int KHtDISP_CONTROL2_GoWbUpdate 			= 1<<6 ;
	/// LCD Go command update
	static unsigned int KHtDISP_CONTROL2_GoLCDUpdate     		= 1<<5 ;
	/// Mono 8-bit mode
	static unsigned int KHtDISP_CONTROL2_M8BEn           		= 1<<4 ;
	/// TFT LCD DISP2lay Type
	static unsigned int KHtDISP_CONTROL2_TFTEn           		= 1<<3 ;
	/// Monochrome Operation Enable
	static unsigned int KHtDISP_CONTROL2_MONOCHROMEEn    		= 1<<2 ;
	/// LCD output enable
	static unsigned int KHtDISP_CONTROL2_LcdEn         			= 1<<0 ;

/************************************* 117) DISPC_DEF_COLOR2 REGISTER. OFFSET : 0x3AC ****************************************/

/// to configure the default solid background color for the Sec LCD
static unsigned int KHoDISP_DEFAULT_COLOR2 = 0x3AC ;

/************************************* 118) DISPC_TRANS_COLOR2 REGISTER. OFFSET : 0x3B0 ****************************************/

/// sets the transparency color value for the video/graphics overlays for Sec LCD
static unsigned int KHoDISP_TRANS_COLOR2    = 0x3B0 ;

/************************************* 119) DISPC_DATA2_CYCLEJ REGISTER. OFFSET : 0x3C0 - 0x3C8 ****************************************/

/// configures the output data format for 1st cycle
static unsigned int KHoDISP_DATA2_CYCLE1            = 0x3C0 ;
/// configures the output data format for 2nd cycle.
static unsigned int KHoDISP_DATA2_CYCLE2            = 0x3C4 ;
/// configures the output data format for 3rd cycle.
static unsigned int KHoDISP_DATA2_CYCLE3            = 0x3C8 ;

/************************************* 120) DISPC_SIZE_LCD2 REGISTER. OFFSET : 0x3CC ****************************************/

/// configures the size of the Secondary LCD
static unsigned int KHoDISP_SIZE_LCD2    = 0x3CC ;

/************************************* 121) DISPC_TIMING_H2 REGISTER. OFFSET : 0x400 ****************************************/

/// configures the timing logic for the HSYNC signal (Sec LCD)
static unsigned int KHoDISP_TIMING_H2        = 0x400 ;

/************************************* 122) DISPC_TIMING_V2 REGISTER. OFFSET : 0x404 ****************************************/

/// configures the timing logic for the VSYNC signal (Sec LCD)
static unsigned int KHoDISP_TIMING_V2        = 0x404 ;

/************************************* 123) DISPC_POL_FREQ2 REGISTER. OFFSET : 0x408 ****************************************/

/// configures the signal configuration
static unsigned int KHoDISP_POL_FREQ2        = 0x408 ;

/************************************* 124) DISPC_DIVISOR2 REGISTER. OFFSET : 0x40C ****************************************/

/// Configures the divisors for Sec LCD
static unsigned int KHoDISP_DIV2         = 0x40C ;

/************************************* 125) DISPC_CONFIG2 REGISTER. OFFSET : 0x620 ****************************************/

/// allows to configure the display controller module. Shadow
/// register, updated on VFP start period or EVSYNC.
static unsigned int KHoDISP_CONFIG2      = 0x620 ;

	/// Full Range for CSC
	static unsigned int KHtDISP_CONFIG2_FullRange 				= 1<<25 ;
	/// Enables Color Space Conv
	static unsigned int KHtDISP_CONFIG2_ColConvEnable 			= 1<<24 ;
	/// First Field
	static unsigned int KHtDISP_CONFIG2_FidFirst 				= 1<<23 ;
	/// Progressive/Interlace selection
	static unsigned int KHtDISP_CONFIG2_OpModeEnable 			= 1<<22 ;
	/// Handscheck b/w DMA buffer and STALL signal
	static unsigned int KHtDISP_CONFIG2_FIFOHandCheck        	= 1<<16 ;
	/// CPR
	static unsigned int KHtDISP_CONFIG2_CPR                  	= 1<<15 ;
	/// Trans color key selection (LCD output)
	static unsigned int KHtDISP_CONFIG2_TCKLcdSelection      	= 1<<11 ;
	/// Enabl the trans color key for the LCD
	static unsigned int KHtDISP_CONFIG2_TCKLcdEnable         	= 1<<10 ;
	/// ACBias Gated Enabled
	static unsigned int KHtDISP_CONFIG2_ACBiasGated          	= 1<<8 ;
	/// VSYNC Gated Enabled
	static unsigned int KHtDISP_CONFIG2_VsyncGated           	= 1<<7 ;
	/// HSYNC Gated Enabled
	static unsigned int KHtDISP_CONFIG2_HsyncGated           	= 1<<6 ;
	/// Pixel Clock Gated Enabled
	static unsigned int KHtDISP_CONFIG2_PxlClkGated          	= 1<<5 ;
	/// Pixel Data Gated Enabled
	static unsigned int KHtDISP_CONFIG2_PxlDataGated         	= 1<<4 ;
	/// Pixel gated enable (only for TFT)
	static unsigned int KHtDISP_CONFIG2_PxlGated             	= 1<<0 ;

/************************************* 125) DISPC_VID1_ATTRIBUTES2 REGISTER. OFFSET : 0x624 ****************************************/

/// Defines the attributes for VID1 Window
static unsigned int KHoDISP_VID1_ATTR2      					= 0x624 ;

	/// YUV Chroma Re-Smapling
	static unsigned int KHtDISP_VID1_ATTR2YuvChReSmpl 			= 1<<8 ;
	/// Shift for VC1 Range CBCR
	static unsigned int KHsDISP_VID1_ATTR2Vc1RangeCbCr 			= 4 ;
	/// Shift for VC1 Range Y
	static unsigned int KHsDISP_VID1_ATTR2Vc1RangeY 			= 1 ;
	/// VC1 Enable
	static unsigned int KHtDISP_VID1_ATTR2Vc1En 				= 1<<0 ;

/************************************* 126) DISPC_VID2_ATTRIBUTES2 REGISTER. OFFSET : 0x628 ****************************************/

/// Defines the attributes for VID2 Window
static unsigned int KHoDISP_VID2_ATTR2      					= 0x628 ;

	/// YUV Chroma Re-Smapling
	static unsigned int KHtDISP_VID2_ATTR2YuvChReSmpl 			= 1<<8 ;
	/// Shift for VC1 Range CBCR
	static unsigned int KHsDISP_VID2_ATTR2Vc1RangeCbCr 			= 4 ;
	/// Shift for VC1 Range Y
	static unsigned int KHsDISP_VID2_ATTR2Vc1RangeY 			= 1 ;
	/// VC1 Enable
	static unsigned int KHtDISP_VID2_ATTR2Vc1En 				= 1<<0 ;

/************************************* 127) DISPC_VID3_ATTRIBUTES2 REGISTER. OFFSET : 0x62C ****************************************/

/// Defines the attributes for VID3 Window
static unsigned int KHoDISP_VID3_ATTR2      					= 0x62C ;

	/// YUV Chroma Re-Smapling
	static unsigned int KHtDISP_VID3_ATTR2YuvChReSmpl 			= 1<<8 ;
	/// Shift for VC1 Range CBCR
	static unsigned int KHsDISP_VID3_ATTR2Vc1RangeCbCr 			= 4 ;
	/// Shift for VC1 Range Y
	static unsigned int KHsDISP_VID3_ATTR2Vc1RangeY 			= 1 ;
	/// VC1 Enable
	static unsigned int KHtDISP_VID3_ATTR2Vc1En 				= 1<<0 ;

/************************************* 128) DISPC_GAMMA_TABLE0 REGISTER. OFFSET : 0x630 ****************************************/

/// Defines the Gamma Table on Pri LCD Out
static unsigned int KHoDISP_GAMMA_TABLE0      					= 0x630 ;

	/// Location where bit-field val is stored
	static unsigned int KHsDISP_GAMMA_TABLE0_Index      		= 24 ;
	/// Value R
	static unsigned int KHsDISP_GAMMA_TABLE0_ValR      			= 16 ;
	/// Value G
	static unsigned int KHsDISP_GAMMA_TABLE0_ValG      			= 8 ;
	/// Value B
	static unsigned int KHsDISP_GAMMA_TABLE0_ValB      			= 0 ;

/************************************* 129) DISPC_GAMMA_TABLE1 REGISTER. OFFSET : 0x634 ****************************************/

/// Defines the Gamma Table on Sec LCD Out
static unsigned int KHoDISP_GAMMA_TABLE1      					= 0x634 ;

	/// Location where bit-field val is stored
	static unsigned int KHsDISP_GAMMA_TABLE1_Index      		= 24 ;
	/// Value R
	static unsigned int KHsDISP_GAMMA_TABLE1_ValR      			= 16 ;
	/// Value G
	static unsigned int KHsDISP_GAMMA_TABLE1_ValG      			= 8 ;
	/// Value B
	static unsigned int KHsDISP_GAMMA_TABLE1_ValB      			= 0 ;

/************************************* 130) DISPC_GAMMA_TABLE2 REGISTER. OFFSET : 0x638 ****************************************/

/// Defines the Gamma Table on TV Out
static unsigned int KHoDISP_GAMMA_TABLE2      					= 0x638 ;

	/// Resets Internal Index
	static unsigned int KHsDISP_GAMMA_TABLE2_ResetIndex      	= 1<<31;
	/// Value R
	static unsigned int KHsDISP_GAMMA_TABLE2_ValR      			= 20 ;
	/// Value G
	static unsigned int KHsDISP_GAMMA_TABLE2_ValG      			= 10 ;
	/// Value B
	static unsigned int KHsDISP_GAMMA_TABLE2_ValB      			= 0 ;

/************************************* 131) DISPC_VID1_FIR2 REGISTER. OFFSET : 0x63C ****************************************/

/// configures the resize factors for horizontal and vertical
/// up/down-sampling of the video window (Sec Lcd)
static unsigned int KHoDISP_VID1_FIR2            				= 0x63C ;

	/// shift for Vertical increment of the up/down-sampling filter
	static unsigned int KHsDISP_VID1_FIR2VINC                	= 16 ;
	/// shift for Horizontal increment of the up/down-sampling filter
	static unsigned int KHsDISP_VID1_FIR2HINC                	= 0 ;

/************************************* 132) DISPC_VID1_ACCU2J REGISTER. OFFSET : 0x640-0x17C ****************************************/

/// configures the resize accumulator init values for horizontal and
/// vertical up/down-sampling of the video window (Sec LCD)
static unsigned int KHoDISP_VID1_ACCU2_0          				= 0x640 ;
static unsigned int KHoDISP_VID1_ACCU2_1          				= 0x644 ;
	/// shift for Vertical initialization accu value
	static unsigned int KHsDISP_VID1_VERTICALACCU2           	= 16 ;
	/// shift for Horizontal initialization accu value
	static unsigned int KHsDISP_VID1_HORIZONTALACCU2         	= 0 ;

/************************************* 133) DISPC_VID1_FIR_COEF_H2_I REGISTER. OFFSET : 0x648 - 0x680 ****************************************/

/// The bank of registers configure the up/down-scaling coefficients for
/// the vertical and horizontal resize of the video picture associated with
/// the video window
static unsigned int KHoDISP_VID1_FIR_COEF_H2_0    				= 0x648 ;
static unsigned int KHoDISP_VID1_FIR_COEF_H2_1    				= 0x650 ;
static unsigned int KHoDISP_VID1_FIR_COEF_H2_2    				= 0x658 ;
static unsigned int KHoDISP_VID1_FIR_COEF_H2_3    				= 0x660 ;
static unsigned int KHoDISP_VID1_FIR_COEF_H2_4    				= 0x668 ;
static unsigned int KHoDISP_VID1_FIR_COEF_H2_5    				= 0x670 ;
static unsigned int KHoDISP_VID1_FIR_COEF_H2_6    				= 0x678 ;
static unsigned int KHoDISP_VID1_FIR_COEF_H2_7    				= 0x680 ;

/************************************* 134) DISPC_VID1_FIR_COEF_HV2_I REGISTER. OFFSET : 0x64C - 0x684 ****************************************/

/// The bank of registers configure the down/up/down-scaling coefficients
/// for the vertical and horizontal resize of the video picture associated
/// with the video window
static unsigned int KHoDISP_VID1_FIR_COEF_HV2_0   				= 0x64C ;
static unsigned int KHoDISP_VID1_FIR_COEF_HV2_1   				= 0x654 ;
static unsigned int KHoDISP_VID1_FIR_COEF_HV2_2   				= 0x65C ;
static unsigned int KHoDISP_VID1_FIR_COEF_HV2_3   				= 0x664 ;
static unsigned int KHoDISP_VID1_FIR_COEF_HV2_4   				= 0x66C ;
static unsigned int KHoDISP_VID1_FIR_COEF_HV2_5   				= 0x674 ;
static unsigned int KHoDISP_VID1_FIR_COEF_HV2_6   				= 0x67C ;
static unsigned int KHoDISP_VID1_FIR_COEF_HV2_7   				= 0x684 ;

/************************************* 135) DISPC_VID1_FIR_COEF_V2_I REGISTER. OFFSET : 0x688 - 0x6A4 ****************************************/

/// The bank of registers configure the up-/down-scaling coefficients
/// for the vertical of the video picture associated with the video window
/// These registers have the extra 2 taps of 5-tap vertical coefficients
static unsigned int KHoDISP_VID1_FIR_COEF_V2_0   = 0x688 ;
static unsigned int KHoDISP_VID1_FIR_COEF_V2_1   = 0x68C ;
static unsigned int KHoDISP_VID1_FIR_COEF_V2_2   = 0x690 ;
static unsigned int KHoDISP_VID1_FIR_COEF_V2_3   = 0x694 ;
static unsigned int KHoDISP_VID1_FIR_COEF_V2_4   = 0x698 ;
static unsigned int KHoDISP_VID1_FIR_COEF_V2_5   = 0x69C ;
static unsigned int KHoDISP_VID1_FIR_COEF_V2_6   = 0x6A0 ;
static unsigned int KHoDISP_VID1_FIR_COEF_V2_7   = 0x6A4 ;

/************************************* 136) DISPC_VID2_FIR2 REGISTER. OFFSET : 0x6A8 ****************************************/

/// configures the resize factors for horizontal and vertical
/// up/down-sampling of the video window (Sec Lcd)
static unsigned int KHoDISP_VID2_FIR2            				= 0x6A8 ;

	/// shift for Vertical increment of the up/down-sampling filter
	static unsigned int KHsDISP_VID2_FIR2VINC                	= 16 ;
	/// shift for Horizontal increment of the up/down-sampling filter
	static unsigned int KHsDISP_VID2_FIR2HINC                	= 0 ;

/************************************* 137) DISPC_VID2_ACCU2_J REGISTER. OFFSET : 0x6AC-0x6B0 ****************************************/

/// configures the resize accumulator init values for horizontal and
/// vertical up/down-sampling of the video window (Sec LCD)
static unsigned int KHoDISP_VID2_ACCU2_0          				= 0x6AC ;
static unsigned int KHoDISP_VID2_ACCU2_1          				= 0x6B0 ;
	/// shift for Vertical initialization accu value
	static unsigned int KHsDISP_VID2_VERTICALACCU2           	= 16 ;
	/// shift for Horizontal initialization accu value
	static unsigned int KHsDISP_VID2_HORIZONTALACCU2         	= 0 ;

/************************************* 138) DISPC_VID2_FIR_COEF_H2_I REGISTER. OFFSET : 0x6B4 - 0x6EC ****************************************/

/// The bank of registers configure the up/down-scaling coefficients for
/// the vertical and horizontal resize of the video picture associated with
/// the video window
static unsigned int KHoDISP_VID2_FIR_COEF_H2_0    				= 0x6B4 ;
static unsigned int KHoDISP_VID2_FIR_COEF_H2_1    				= 0x6BC ;
static unsigned int KHoDISP_VID2_FIR_COEF_H2_2    				= 0x6C4 ;
static unsigned int KHoDISP_VID2_FIR_COEF_H2_3    				= 0x6CC ;
static unsigned int KHoDISP_VID2_FIR_COEF_H2_4    				= 0x6D4 ;
static unsigned int KHoDISP_VID2_FIR_COEF_H2_5    				= 0x6DC ;
static unsigned int KHoDISP_VID2_FIR_COEF_H2_6    				= 0x6E4 ;
static unsigned int KHoDISP_VID2_FIR_COEF_H2_7    				= 0x6EC ;

/************************************* 139) DISPC_VID2_FIR_COEF_HV2_I REGISTER. OFFSET : 0x6B8 - 0x6F0 ****************************************/

/// The bank of registers configure the down/up/down-scaling coefficients
/// for the vertical and horizontal resize of the video picture associated
/// with the video window
static unsigned int KHoDISP_VID2_FIR_COEF_HV2_0   				= 0x6B8 ;
static unsigned int KHoDISP_VID2_FIR_COEF_HV2_1   				= 0x6C0 ;
static unsigned int KHoDISP_VID2_FIR_COEF_HV2_2   				= 0x6C8 ;
static unsigned int KHoDISP_VID2_FIR_COEF_HV2_3   				= 0x6D0 ;
static unsigned int KHoDISP_VID2_FIR_COEF_HV2_4   				= 0x6D8 ;
static unsigned int KHoDISP_VID2_FIR_COEF_HV2_5   				= 0x6E0 ;
static unsigned int KHoDISP_VID2_FIR_COEF_HV2_6   				= 0x6E8 ;
static unsigned int KHoDISP_VID2_FIR_COEF_HV2_7   				= 0x6F0 ;

/************************************* 140) DISPC_VID2_FIR_COEF_V2_I REGISTER. OFFSET : 0x6F4 - 0x710 ****************************************/

/// The bank of registers configure the up-/down-scaling coefficients
/// for the vertical of the video picture associated with the video window
/// These registers have the extra 2 taps of 5-tap vertical coefficients
static unsigned int KHoDISP_VID2_FIR_COEF_V2_0   = 0x6F4 ;
static unsigned int KHoDISP_VID2_FIR_COEF_V2_1   = 0x6F8 ;
static unsigned int KHoDISP_VID2_FIR_COEF_V2_2   = 0x6FC ;
static unsigned int KHoDISP_VID2_FIR_COEF_V2_3   = 0x700 ;
static unsigned int KHoDISP_VID2_FIR_COEF_V2_4   = 0x704 ;
static unsigned int KHoDISP_VID2_FIR_COEF_V2_5   = 0x708 ;
static unsigned int KHoDISP_VID2_FIR_COEF_V2_6   = 0x70C ;
static unsigned int KHoDISP_VID2_FIR_COEF_V2_7   = 0x710 ;

/************************************* 141) DISPC_VID3_FIR2 REGISTER. OFFSET : 0x724 ****************************************/

/// configures the resize factors for horizontal and vertical
/// up/down-sampling of the video window (Sec Lcd)
static unsigned int KHoDISP_VID3_FIR2            				= 0x724 ;

	/// shift for Vertical increment of the up/down-sampling filter
	static unsigned int KHsDISP_VID3_FIR2VINC                	= 16 ;
	/// shift for Horizontal increment of the up/down-sampling filter
	static unsigned int KHsDISP_VID3_FIR2HINC                	= 0 ;

/************************************* 142) DISPC_VID3_ACCU2_J REGISTER. OFFSET : 0x728-0x72C ****************************************/

/// configures the resize accumulator init values for horizontal and
/// vertical up/down-sampling of the video window (Sec LCD)
static unsigned int KHoDISP_VID3_ACCU2_0          				= 0x728 ;
static unsigned int KHoDISP_VID3_ACCU2_1          				= 0x72C ;
	/// shift for Vertical initialization accu value
	static unsigned int KHsDISP_VID3_VERTICALACCU2           	= 16 ;
	/// shift for Horizontal initialization accu value
	static unsigned int KHsDISP_VID3_HORIZONTALACCU2         	= 0 ;

/************************************* 143) DISPC_VID3_FIR_COEF_H2_I REGISTER. OFFSET : 0x730 - 0x768 ****************************************/

/// The bank of registers configure the up/down-scaling coefficients for
/// the vertical and horizontal resize of the video picture associated with
/// the video window
static unsigned int KHoDISP_VID3_FIR_COEF_H2_0    				= 0x730 ;
static unsigned int KHoDISP_VID3_FIR_COEF_H2_1    				= 0x738 ;
static unsigned int KHoDISP_VID3_FIR_COEF_H2_2    				= 0x740 ;
static unsigned int KHoDISP_VID3_FIR_COEF_H2_3    				= 0x748 ;
static unsigned int KHoDISP_VID3_FIR_COEF_H2_4    				= 0x750 ;
static unsigned int KHoDISP_VID3_FIR_COEF_H2_5    				= 0x758 ;
static unsigned int KHoDISP_VID3_FIR_COEF_H2_6    				= 0x760 ;
static unsigned int KHoDISP_VID3_FIR_COEF_H2_7    				= 0x768 ;

/************************************* 144) DISPC_VID3_FIR_COEF_HV2_I REGISTER. OFFSET : 0x734 - 0x76C ****************************************/

/// The bank of registers configure the down/up/down-scaling coefficients
/// for the vertical and horizontal resize of the video picture associated
/// with the video window
static unsigned int KHoDISP_VID3_FIR_COEF_HV2_0   				= 0x734 ;
static unsigned int KHoDISP_VID3_FIR_COEF_HV2_1   				= 0x73C ;
static unsigned int KHoDISP_VID3_FIR_COEF_HV2_2   				= 0x744 ;
static unsigned int KHoDISP_VID3_FIR_COEF_HV2_3   				= 0x74C ;
static unsigned int KHoDISP_VID3_FIR_COEF_HV2_4   				= 0x754 ;
static unsigned int KHoDISP_VID3_FIR_COEF_HV2_5   				= 0x75C ;
static unsigned int KHoDISP_VID3_FIR_COEF_HV2_6   				= 0x764 ;
static unsigned int KHoDISP_VID3_FIR_COEF_HV2_7   				= 0x76C ;

/************************************* 145) DISPC_VID3_FIR_COEF_V2_I REGISTER. OFFSET : 0x770 - 0x78C ****************************************/

/// The bank of registers configure the up-/down-scaling coefficients
/// for the vertical of the video picture associated with the video window
/// These registers have the extra 2 taps of 5-tap vertical coefficients
static unsigned int KHoDISP_VID3_FIR_COEF_V2_0   = 0x770 ;
static unsigned int KHoDISP_VID3_FIR_COEF_V2_1   = 0x774 ;
static unsigned int KHoDISP_VID3_FIR_COEF_V2_2   = 0x778 ;
static unsigned int KHoDISP_VID3_FIR_COEF_V2_3   = 0x77C ;
static unsigned int KHoDISP_VID3_FIR_COEF_V2_4   = 0x780 ;
static unsigned int KHoDISP_VID3_FIR_COEF_V2_5   = 0x784 ;
static unsigned int KHoDISP_VID3_FIR_COEF_V2_6   = 0x788 ;
static unsigned int KHoDISP_VID3_FIR_COEF_V2_7   = 0x78C ;

/************************************* 146) DISPC_WB_FIR2 REGISTER. OFFSET : 0x790 ****************************************/

/// configures the resize factors for horizontal and vertical
/// up/down-sampling of the video window (WB)
static unsigned int KHoDISP_WB_FIR2            				= 0x790 ;

	/// shift for Vertical increment of the up/down-sampling filter
	static unsigned int KHsDISP_WB_FIR2VINC                	= 16 ;
	/// shift for Horizontal increment of the up/down-sampling filter
	static unsigned int KHsDISP_WB_FIR2HINC                	= 0 ;

/************************************* 147) DISPC_WB_ACCU2_J REGISTER. OFFSET : 0x794-0x798 ****************************************/

/// configures the resize accumulator init values for horizontal and
/// vertical up/down-sampling of the video window (WB)
static unsigned int KHoDISP_WB_ACCU2_0          				= 0x794 ;
static unsigned int KHoDISP_WB_ACCU2_1          				= 0x798 ;
	/// shift for Vertical initialization accu value
	static unsigned int KHsDISP_WB_VERTICALACCU2           	= 16 ;
	/// shift for Horizontal initialization accu value
	static unsigned int KHsDISP_WB_HORIZONTALACCU2         	= 0 ;

/************************************* 148) DISPC_WB_FIR_COEF_H2_I REGISTER. OFFSET : 0x7A0 - 0x7D8 ****************************************/

/// The bank of registers configure the up/down-scaling coefficients for
/// the vertical and horizontal resize of the video picture associated with
/// the video window
static unsigned int KHoDISP_WB_FIR_COEF_H2_0    				= 0x7A0 ;
static unsigned int KHoDISP_WB_FIR_COEF_H2_1    				= 0x7A8 ;
static unsigned int KHoDISP_WB_FIR_COEF_H2_2    				= 0x7B0 ;
static unsigned int KHoDISP_WB_FIR_COEF_H2_3    				= 0x7B8 ;
static unsigned int KHoDISP_WB_FIR_COEF_H2_4    				= 0x7C0 ;
static unsigned int KHoDISP_WB_FIR_COEF_H2_5    				= 0x7C8 ;
static unsigned int KHoDISP_WB_FIR_COEF_H2_6    				= 0x7D0 ;
static unsigned int KHoDISP_WB_FIR_COEF_H2_7    				= 0x7D8 ;

/************************************* 149) DISPC_WB_FIR_COEF_HV2_I REGISTER. OFFSET : 0x734 - 0x7DC ****************************************/

/// The bank of registers configure the down/up/down-scaling coefficients
/// for the vertical and horizontal resize of the video picture associated
/// with the video window
static unsigned int KHoDISP_WB_FIR_COEF_HV2_0   				= 0x7A4 ;
static unsigned int KHoDISP_WB_FIR_COEF_HV2_1   				= 0x7AC ;
static unsigned int KHoDISP_WB_FIR_COEF_HV2_2   				= 0x7B4 ;
static unsigned int KHoDISP_WB_FIR_COEF_HV2_3   				= 0x7BC ;
static unsigned int KHoDISP_WB_FIR_COEF_HV2_4   				= 0x7C4 ;
static unsigned int KHoDISP_WB_FIR_COEF_HV2_5   				= 0x7CC ;
static unsigned int KHoDISP_WB_FIR_COEF_HV2_6   				= 0x7D4 ;
static unsigned int KHoDISP_WB_FIR_COEF_HV2_7   				= 0x7DC ;

/************************************* 150) DISPC_WB_FIR_COEF_V2_I REGISTER. OFFSET : 0x7E0 - 0x7DC ****************************************/

/// The bank of registers configure the up-/down-scaling coefficients
/// for the vertical of the video picture associated with the video window
/// These registers have the extra 2 taps of 5-tap vertical coefficients
static unsigned int KHoDISP_WB_FIR_COEF_V2_0   = 0x7E0 ;
static unsigned int KHoDISP_WB_FIR_COEF_V2_1   = 0x7E4 ;
static unsigned int KHoDISP_WB_FIR_COEF_V2_2   = 0x7E8 ;
static unsigned int KHoDISP_WB_FIR_COEF_V2_3   = 0x7EC ;
static unsigned int KHoDISP_WB_FIR_COEF_V2_4   = 0x7D0 ;
static unsigned int KHoDISP_WB_FIR_COEF_V2_5   = 0x7D4 ;
static unsigned int KHoDISP_WB_FIR_COEF_V2_6   = 0x7D8 ;
static unsigned int KHoDISP_WB_FIR_COEF_V2_7   = 0x7DC ;

/************************************* 151) DISPC_GLOBAL_BUFFER REGISTER. OFFSET : 0x800 ****************************************/

/// Configures DMA FIFO alloc for pipelines
static unsigned int KHoDISPC_GLOBAL_BUFFER 				= 0x800;

	/// Write-back DMA BOTTOM buffer
	static unsigned int KHsDISPC_GLOBAL_BUFFER_Wb_Bottom 				= 27;
	/// Write-back DMA TOP buffer
	static unsigned int KHsDISPC_GLOBAL_BUFFER_Wb_Top	 				= 24;
	/// VID3 DMA BOTTOM buffer
	static unsigned int KHsDISPC_GLOBAL_BUFFER_Vid3_Bottom 				= 21;
	/// VID3 DMA TOP buffer
	static unsigned int KHsDISPC_GLOBAL_BUFFER_Vid3_Top	 				= 18;
	/// VID2 DMA BOTTOM buffer
	static unsigned int KHsDISPC_GLOBAL_BUFFER_Vid2_Bottom 				= 15;
	/// VID2 DMA TOP buffer
	static unsigned int KHsDISPC_GLOBAL_BUFFER_Vid2_Top	 				= 12;
	/// VID1 DMA BOTTOM buffer
	static unsigned int KHsDISPC_GLOBAL_BUFFER_Vid1_Bottom 				= 9;
	/// VID1 DMA TOP buffer
	static unsigned int KHsDISPC_GLOBAL_BUFFER_Vid1_Top	 				= 6;
	/// Gfx DMA BOTTOM buffer
	static unsigned int KHsDISPC_GLOBAL_BUFFER_Gfx_Bottom 				= 3;
	/// Gfx DMA TOP buffer
	static unsigned int KHsDISPC_GLOBAL_BUFFER_Gfx_Top	 				= 0;
	/// Buffer Allocated to Gfx Pipe
	static unsigned int KHtDISPC_GLOBAL_BUFFER_Gfx_Pipe	 				= 0x0;
	/// Buffer Allocated to Vid1 Pipe
	static unsigned int KHtDISPC_GLOBAL_BUFFER_Vid1_Pipe	 			= 0x1;
	/// Buffer Allocated to Vid2 Pipe
	static unsigned int KHtDISPC_GLOBAL_BUFFER_Vid2_Pipe	 			= 0x2;
	/// Buffer Allocated to Vid3 Pipe
	static unsigned int KHtDISPC_GLOBAL_BUFFER_Vid3_Pipe	 			= 0x3;
	/// Buffer Allocated to Wb Pipe
	static unsigned int KHtDISPC_GLOBAL_BUFFER_Wb_Pipe	 				= 0x4;

/************************************* 152) DISPC_DIVISOR REGISTER. OFFSET : 0x804 ****************************************/

/// Configures the divisors for generating core func clock
static unsigned int KHoDISP_DIVISOR         							= 0x804 ;

	/// Display Controller Logic clock divisor
	static unsigned int KHsDISP_DIVISOR_Lcd         					= 16 ;
	/// DISPC_DIVISOR Enable Bit
	static unsigned int KHtDISP_DIVISOR_En         						= 1<<0 ;

/************************************* 153) DISPC_WB_ATTRIBUTES2 REGISTER. OFFSET : 0x810 ****************************************/

/// Sets the counter to control the delay to flush the WB pipe
static unsigned int KHoDISP_WB_ATTR2         							= 0x810 ;

	/// Shift for the delay count
	static unsigned int KHsDISP_Wb_DelayCount         					= 0 ;


//******************************************************************************************************************************
//******************************************************************************************************************************
//************************************************ DSI SPECIFIC REGISTER DEFINITIONS *******************************************
//******************************************************************************************************************************
//******************************************************************************************************************************

// DSI module base addresses //
static unsigned int KHwBaseDsiProtoEng		= 0x4000;
static unsigned int KHwBaseComplexIO		= 0x4200;
static unsigned int KHwBaseDsiPllController	= 0x4300;

// Secondary DSI Offset
static unsigned int KHoSecDsiOffset			= 0x1000;

// DSI Module registers //
static unsigned int KHoDsiRevision			= (0x00);
static unsigned int KHoDsiSysConfig			= (0x10);
static unsigned int KHoDsiSysStatus			= (0x14);
static unsigned int KHoDsiIrqStatus			= (0x18);
static unsigned int KHoDsiIrqEnable			= (0x1C);
static unsigned int KHoDsiCtrl				= (0x40);

// New register for OMAP-4 DSI
static unsigned int KHoDsiGnq				= (0x44);

static unsigned int	KHoComplexIOCfg1		= (0x48);
static unsigned int KHoComplexIOIrqStatus	= (0x4C);
static unsigned int KHoComplexIOIrqEnable	= (0x50);
static unsigned int KHoDsiClkCtrl			= (0x54);
static unsigned int KHoDsiTiming1			= (0x58);
static unsigned int KHoDsiTiming2			= (0x5C);
static unsigned int KHoDsiVmTiming1			= (0x60);
static unsigned int KHoDsiVmTiming2			= (0x64);
static unsigned int KHoDsiVmTiming3			= (0x68);
static unsigned int KHoDsiClkTiming			= (0x6C);
static unsigned int KHoDsiTxFifoVcSize		= (0x70);
static unsigned int KHoDsiRxFifoVcSize		= (0x74);
static unsigned int	KHoComplexIOCfg2		= (0x78);
static unsigned int KHoDsiFifoVcFull		= (0x7C);
static unsigned int KHoDsiVmTiming4			= (0x80);
static unsigned int KHoDsiFifoVcEmpty		= (0x84);
static unsigned int KHoDsiVmTiming5			= (0x88);
static unsigned int KHoDsiVmTiming6			= (0x8C);
static unsigned int KHoDsiVmTiming7			= (0x90);

// New register Set for OMAP-4 DSI
static unsigned int KHoDsiStpClkTiming		= (0x94);
static unsigned int KHoDsiCtrl2				= (0x98);
static unsigned int KHoDsiVmTiming8			= (0x9C);
static unsigned int KHoTeHsyncWidth0		= (0xA0);
static unsigned int KHoTeHsyncWidth1		= (0xAC);
static unsigned int KHoTeVsyncWidth0		= (0xA4);
static unsigned int KHoTeVsyncWidth1		= (0xB0);
static unsigned int KHoTeHsyncNumber0		= (0xA8);
static unsigned int KHoTeHsyncNumber1		= (0xB4);


static unsigned int KHoDsiVc0Ctrl			= (0x100);
static unsigned int KHoDsiVc0Te				= (0x104);
static unsigned int KHoDsiVcLongPktHeader	= (0x108);
static unsigned int KHoDsiVcLongPktPayLoad	= (0x10C);
static unsigned int KHoDsiVcShortPktHeader	= (0x110);
static unsigned int KHoDsiVc0IrqStatus		= (0x118);
static unsigned int KHoDsiVc0IrqEnable		= (0x11C);

static unsigned int KHmDsiClkCtrlLpClkDivisor 	= 1<<0| 1<<1| 1<<2 | 1<<3 | 1<<4 | 1<<5 | 1<<6 | 1<<7 | 1<<8 | 1<<9 | 1<<10 | 1<<11 | 1<<12;
static unsigned int KHtDsiClkCtrlDdrClkAlwaysOn = 1<<13;
static unsigned int KHtDsiClkCtrlPwrStatus 		= 1<<28|1<<29;

static unsigned int KHtDsiIrqEnableTeTriggerIrqEn = 1<<16;

static unsigned int KHtDsiVc0IrqStatusBtaIrq 		= 1<<5;
static unsigned int KHtDsiVc0IrqStatusPktSentIrq 	= 1<<2;

static unsigned int KHtDsiVc0CtrlBtaEn 			= 1<<6;
static unsigned int KHtDsiVc0CtrlModeSpeed 		= 1<<9;
static unsigned int KHtDsiVc0CtrlRxFifoNotEmpty = 1<<20;

// DSI ComplexIO registers //
static unsigned int	KHoDsiPhyConfig0	= (0x00);
static unsigned int	KHoDsiPhyConfig1	= (0x04);
static unsigned int	KHoDsiPhyConfig2	= (0x08);
static unsigned int	KHoDsiPhyConfig5	= (0x14);

static unsigned int KHtComplexIOCfg1PwrStatus = 1<<26 | 1<<25;
static unsigned int KHtComplexIOCfg1ResetDone = 1<<29;

// DSI PLL registers //
static unsigned int KHoDsiPllCntrl		= (0x00);
static unsigned int KHoDsiPllStatus		= (0x04);
static unsigned int KHoDsiPllGo			= (0x08);
static unsigned int KHoDsiPllConfig1	= (0x0C);
static unsigned int KHoDsiPllConfig2	= (0x10);

static unsigned int KHtDsiPllStatusResetDone = 1<<0;


//******************************************************************************************************************************
//******************************************************************************************************************************
//************************************** VIDEO ENCODER SPECIFIC REGISTER DEFINITIONS ************************************
//******************************************************************************************************************************
//******************************************************************************************************************************
static unsigned int KHwBaseVENCReg          = 0x58000000 + 0x3000 ;
    /// register contains the revision ID for the encoder
    static unsigned int KHoVENCRevId            = 0x00 ;

    /// Status
    static unsigned int KHoVencStatus           = 0x04 ;
            /// Closed Caption Status for Even Field
            static unsigned int KHtVENCCce              = 1<<4 ;
            /// Closed Caption Status for Odd Field
            static unsigned int KHtVENCCco              = 1<<3 ;
            /// Mask for Field Sequence ID
            static unsigned int KHmVENCFsq              = 0x07 ;

    /// register specifies the input video source and format
    static unsigned int KHoVENCFCtrl            = 0x08 ;
        /// Select Video Data Source
        static unsigned int KHsVENC_SVDS            = 6 ;
        /// Background color select
        static unsigned int KHsVENC_BCOLOR          = 2 ;
        /// shift for to specify the video input data stream
        /// format and timing
        static unsigned int KHsVENC_FMT             = 0 ;
            /// RESET the encoder
            static unsigned int KHtVENCReset            = 1<<8 ;
            /// RGB /YCrCb input coding range
            static unsigned int KHtVENCRgbf             = 1<<5 ;

    /// Encoder output clock
    static unsigned int KHoVENCVidOutCtrl       = 0x10 ;
            static unsigned int KHtVENC27MhzClk         = 1<<0 ;

    static unsigned int KHoVENCSyncCtrl         = 0x14 ;
        /// shift for selecting Vertical blanking mode
        static unsigned int KHsVENC_VBLKM           = 10 ;
        /// shift for selecting Horizontal blanking mode
        static unsigned int KHsVENC_HBLKM           = 8 ;
            /// Free running
            static unsigned int KHtVENCFree             = 1<<15 ;
            /// Enable to detect F and V bits only on EAV in
            /// ITU-R 656 input mode
            static unsigned int KHtVENCEsav             = 1<<14 ;
            /// Ignore protection bits in ITU-R 656 input mode
            static unsigned int KHtVENCIgnp             = 1<<13 ;
            /// Blank shaping
            static unsigned int KHtVENCNbLnks           = 1<<12 ;
            /// Encoder on master or slave of external sync
            static unsigned int KHtVENCMasterSl         = 1<<7 ;
            /// FID input polarity
            static unsigned int KHtVENCFidPol           = 1<<6 ;
            /// VS input polarity
            static unsigned int KHtVENCVsPol            = 1<<3 ;
            /// HS Input Polarity
            static unsigned int KHtVENCHsPol            = 1<<2 ;
            /// FID extracted from external FID or HSYNC and VSYNC
            static unsigned int KHtVENCFidHsVsMod       = 1<<0 ;

    /// Line length or total number of pixels in a scan line
    static unsigned int KHoVENCLLen             = 0x1C ;
            static unsigned int KHbVENCLlen             = 0x359 ;

    /// The frame length or total number of lines in a frame
    static unsigned int KHoVENCFLens            = 0x20 ;
            static unsigned int KHbVENCFlens            = 0x03C ;

    /// Filter Control
    static unsigned int KHoVENCHFltrCtrl        = 0x24 ;
        /// shift for selecting Chrominance interpolation filter control
        static unsigned int KHsVENC_CINTP           = 1 ;
            /// Luminance interpolation filter control
            static unsigned int KHtVENCYIntp            = 1<<0 ;

    /// Frequency code control
    static unsigned int KHoVENCCcCarrWssCarr    = 0x28 ;
        /// shift for selecting Wide screen signaling run-in code frequency
        /// control
        static unsigned int KHsVENC_FWSS            = 16 ;
            static unsigned int KHbVENCFwss             = 0x043F ;
        /// shift for selecting Close caption run-in code frequency control
        static unsigned int KHsVENC_FWC             = 0 ;
            static unsigned int KHbVENCFwc              = 0x2631 ;

    /// Phase of the encoded video color subcarrier
    static unsigned int KHoVENCCPhase           = 0x2C ;

    /// Gain control for Cb signal
    static unsigned int KHoVENCGainU            = 0x30 ;

    /// Gain control of Cr signal
    static unsigned int KHoVENCGainV            = 0x34 ;

    /// Gain control of Y signal
    static unsigned int KHoVENCGainY            = 0x38 ;

    /// Black Level setting
    static unsigned int KHoVENCBlackLvl         = 0x3C ;

    ///  Blank Level setting
    static unsigned int KHoVENCBlankLvl         = 0x40 ;

    static unsigned int KHoVENCXColor           = 0x44 ;
        /// shift for setting chroma channel delay compensation
        static unsigned int KHsVENC_XCBW            = 3 ;
        /// shift for setting Cross color reduction filter selection
        static unsigned int KHsVENC_LCD             = 0 ;
            /// Cross color reduction enable
            static unsigned int KHtVENCXce              = 1<<6 ;

    static unsigned int KHoVENCMCtrl            = 0x48 ;
        /// shift for setting chroma channel delay compensation
        static unsigned int KHsVENC_CBW             = 2 ;
            /// PAL switch phase setting
            static unsigned int KHtVENCPalphs           = 1<<5 ;
            /// Phase alternation line encoding selection
            static unsigned int KHtVENCPal              = 1<<1 ;
            /// Field rate selection.
            static unsigned int KHtVENCFfrq             = 1<<0 ;

    static unsigned int KHoVENCBstAmpWssData    = 0x4C ;
        /// shift for setting Wide Screen Signaling data
        static unsigned int KHsVENC_WSS_D           = 8 ;
        /// shift for setting Setting of amplitude of color burst
        static unsigned int KHsVENC_BSTAP           = 0 ;
            /// Square-pixel sampling rate
            static unsigned int KHtVENCSqp              = 1<<7 ;

    /// Color subcarrier frequency registers
    static unsigned int KHoVENCSCarr            = 0x50 ;

    static unsigned int KHoVENCLine21           = 0x54 ;
        /// shift for setting closed-caption data in the even field
        static unsigned int KHsVENC_L21E            = 16 ;
        /// shift for setting closed-caption data in the odd field
        static unsigned int KHsVENC_L21O            = 0 ;

    static unsigned int KHoVENCLnSel            = 0x58 ;
            /// Selects the line where closed-caption or extended
            /// service data are encoded
            static unsigned int KHtVENCSline            = 0x15 ;

    static unsigned int KHoVENCL21WcCtrl        = 0x5C ;
        /// shift for setting WSS encoding
        static unsigned int KHsVENC_EVEN_ODD_EN     = 13 ;
        /// shift for setting line where WSS data are encoded
        static unsigned int KHsVENC_LINE            = 8 ;
        /// shift for setting the Line21 closed-caption encoding
        static unsigned int KHsVENC_L21EN           = 0 ;
            /// wss inverter
            static unsigned int KHtVENCWssInv           = 1<<15 ;

    static unsigned int KHoVENCHtriggerVtrigger = 0x60 ;
        /// shift for setting Vertical trigger reference for VSYNC.
        static unsigned int KHsVENC_VTRIG           = 16 ;
        /// shift for setting Horizontal trigger phase, which sets HSYNC
        static unsigned int KHsVENC_HTRIG           = 0 ;

    static unsigned int KHoVENCSAvidEAvid       = 0x64 ;
        /// shift for setting end of active video.
        static unsigned int KHsVENC_EAVID           = 16 ;
        /// shift for setting Start of active video.
        static unsigned int KHsVENC_SAVID           = 0 ;

    static unsigned int KHoVENCFlenFal          = 0x68 ;
        /// shift for setting First Active Line of Field
        static unsigned int KHsVENC_FAL             = 16 ;
        /// shift for setting Field length
        static unsigned int KHsVENC_FLEN            = 0 ;

    static unsigned int KHoVENCLalPhaseRst      = 0x6C ;
        /// shift for setting Phase reset mode
        static unsigned int KHsVENC_PRES            = 17 ;
        /// shift for setting Last Active Line of Field
        static unsigned int KHsVENC_LAL             = 0 ;
            /// Vertical blanking setting
            static unsigned int KHtVENCSblank           = 1<<16 ;

    static unsigned int KHoVENCHsIntStrtStpX    = 0x70 ;
        /// shift for setting HSYNC internal stop.
        static unsigned int KHsVENC_HS_INT_STOP_X       = 16 ;
        /// shift for setting HSYNC internal start
        static unsigned int KHsVENC_HS_INT_START_X      = 0 ;

    static unsigned int KHoVENCHsExtStrtStpX    = 0x74 ;
        /// shift for setting HSYNC External stop.
        static unsigned int KHsVENC_HS_EXT_STOP_X       = 16 ;
        /// shift for setting HSYNC external start
        static unsigned int KHsVENC_HS_EXT_START_X      = 0 ;

    static unsigned int KHoVENCVsIntStrtX       = 0x78 ;
        /// shift for setting VSYNC internal start.
        static unsigned int KHsVENC_VS_INT_START_X      = 16 ;

    static unsigned int KHoVENCVsIntStpXIntStrtY = 0x7C ;
        /// shift for setting VSYNC internal start
        static unsigned int KHsVENC_VS_INT_START_Y      = 16 ;
        /// shift for setting VSYNC internal stop
        static unsigned int KHsVENC_VS_INT_STOP_X       = 0 ;

    static unsigned int KHoVENCVsIntStpYExtStrtX    = 0x80 ;
        /// shift for setting VSYNC external start.
        static unsigned int KHsVENC_VS_EXT_START_X      = 16 ;
        /// shift for setting VSYNC external stop
        static unsigned int KHsVENC_VS_INT_STOP_Y       = 0 ;

    static unsigned int KHoVENCVsExtStpXExtStrtY    = 0x84 ;
        /// shift for setting VSYNC external start.
        static unsigned int KHsVENC_VS_EXT_START_Y      = 16 ;
        /// shift for setting VSYNC external stop
        static unsigned int KHsVENC_VS_EXT_STOP_X       = 0 ;

    static unsigned int KHoVENCVsExtStpY            = 0x88 ;
            /// shift for setting VSYNC external stop
        static unsigned int KHsVENC_VS_EXT_STOP_Y       = 0 ;

    static unsigned int KHoVENCAVidStrtStpX         = 0x90 ;
        /// shift for setting AVID stop
        static unsigned int KHsVENC_AVID_STOP_X         = 16 ;
        /// shift for setting AVID start
        static unsigned int KHsVENC_AVID_START_X        = 0 ;

    static unsigned int KHoVENCAVidStrtStpY         = 0x94 ;
        /// shift for setting AVID stop
        static unsigned int KHsVENC_AVID_STOP_Y         = 16 ;
        /// shift for setting AVID start
        static unsigned int KHsVENC_AVID_START_Y        = 0 ;

    static unsigned int KHoVENCFidIntStrtXY         = 0xA0 ;
        /// shift for setting FID internal start
        static unsigned int KHsVENC_FID_INT_START_Y     = 16 ;
        static unsigned int KHsVENC_FID_INT_START_X     = 0 ;

    static unsigned int KHoVENCFidIntOffsetY        = 0xA4 ;
        /// shift for setting FID external start
        static unsigned int KHsVENC_FID_EXT_START_X     = 16 ;
        /// shift for setting FID internal offset
        static unsigned int KHsVENC_FID_INT_OFFSET_Y    = 0 ;

    static unsigned int KHoVENCFidExtStrtY      = 0xA8 ;
        /// shift for setting FID external offset
        static unsigned int KHsVENC_FID_EXT_OFFSET_Y    = 16 ;
        /// shift for setting FID internal start
        static unsigned int KHsVENC_FID_EXT_START_Y     = 0 ;

    static unsigned int KHoVENCTVDETGPX             = 0xB0 ;
        /// shift for setting TVDETGP internal stop
        static unsigned int KHsVENC_TVDETGP_INT_STOP_X  = 16 ;
        /// shift for setting TVDETGP internal start
        static unsigned int KHsVENC_TVDETGP_INT_START_X = 0 ;

    static unsigned int KHoVENCTVDETGPY             = 0xB4 ;
        /// shift for setting TVDETGP internal stop
        static unsigned int KHsVENC_TVDETGP_INT_STOP_Y  = 16 ;
        /// shift for setting TVDETGP internal start
        static unsigned int KHsVENC_TVDETGP_INT_START_Y = 0 ;

    static unsigned int KHoVENCGenCtrl              = 0xB8 ;
            /// UVPHASE_POL MS mode UV phase
            static unsigned int KHtVENCMsMode           = 1<<26 ;
            /// UVPHASE_POL 656 input mode UV phase
            static unsigned int KHtVENCUVPhase656       = 1<<25 ;
            /// UVPHASE_POL CBAR mode UV phase
            static unsigned int KHtVENCCbarMode         = 1<<24 ;
            /// HSYNC internal polarity
            static unsigned int KHtVENCHip              = 1<<23 ;
            /// VSYNC Internal Polarity
            static unsigned int KHtVENCVip              = 1<<22 ;
            /// HSYNC external polarity
            static unsigned int KHtVENCHep              = 1<<21 ;
            /// VSYNC external polarity
            static unsigned int KHtVENCVep              = 1<<20 ;
            /// AVID polarity
            static unsigned int KHtVENCAvidp            = 1<<19 ;
            /// FID internal polarity
            static unsigned int KHtVENCFip              = 1<<18 ;
            /// FID external polarity
            static unsigned int KHtVENCFep              = 1<<17 ;
            /// TVDETGP polarity
            static unsigned int KHtVENCTvdp             = 1<<16 ;
            /// TVDETGP generation enable
            static unsigned int KHtVENCTvdpEn           = 1<<0 ;

    /// DAC test controls and DAC A test value
    static unsigned int KHoVENC_Output_Control          = 0xC4 ;
        static unsigned int KHtVENC_CHROMA_SOURCE       = 1<<7 ;
        static unsigned int KHtVENC_COMPOSITE_SOURCE    = 1<<6 ;
        static unsigned int KHtVENC_LUMA_SOURCE         = 1<<5 ;
        static unsigned int KHtVENC_TEST_MODE           = 1<<4 ;
        static unsigned int KHtVENC_VIDEO_INVERT        = 1<<3 ;
        static unsigned int KHtVENC_CHROMA_ENABLE       = 1<<2 ;
        static unsigned int KHtVENC_COMPOSITE_ENABLE    = 1<<1 ;
        static unsigned int KHtVENC_LUMA_ENABLE         = 1<<0 ;

    static unsigned int KHoVENC_Output_Test             = 0xC8 ;
        static unsigned int KHsVENC_CHROMA_SOURCE       = 15 ;
        static unsigned int KHsVENC_COMPOSITE_SOURCE    = 0 ;



unsigned int Register32(unsigned int anAddr)
/**
 *  Read a 32-bit register
 *
 *  @returns register contents
 */
{

	return __raw_readl(anAddr);

//__asm__ __volatile__("ldr r0, [r0]");
//__asm__ __volatile__("mov pc, lr");
}

void SetRegister32(unsigned int anAddr, unsigned int aValue)
/**
 *  Store aData in a 32-bit register
 */
{

	__raw_writel(aValue, anAddr);

//	__asm__ __volatile__("str r1, [r0]");
//	__asm__ __volatile__("mov pc, lr");
}

void ModifyRegister32(unsigned int anAddr, unsigned int aClearMask,
                                                unsigned int aSetMask)
/**
 *  Modify contents of a 32-bit register
 *
 *  @param anAddr      Register address
 *  @param aClearMask  Any bits set in this mask will be cleared
 *  @param aSetMask    Bits to be set after the clear
 */
{
	u32 val;

	val = __raw_readl(anAddr);
	val &= ~(aClearMask);
	val |= (aSetMask);
	__raw_writel(val, anAddr);

//	__asm__ __volatile__("ldr r3, [r0]");
//	__asm__ __volatile__("bic r3, r3, r1");
//	__asm__ __volatile__("orr r3, r3, r2");
//	__asm__ __volatile__("str r3, [r0]");
//	__asm__ __volatile__(" mov pc, lr");
}



#define gVidModeVc 0

void SendShortPacket(unsigned char aDcsCmdType,unsigned char aVirChannel,unsigned char aData0,unsigned char aData1,unsigned char aMode, unsigned char aEccEnable)
{
	int u;
	unsigned int val,header=0,count=10000;
	aMode = 2;

	// Disable Command Mode
	val = Register32(KHwBaseDsiProtoEng + KHoDsiVc0Ctrl + (gVidModeVc*0x20));
	val = val & ~(1<<0);
	SetRegister32(KHwBaseDsiProtoEng + KHoDsiVc0Ctrl + (gVidModeVc*0x20),val);

	// speed selection (HS or LPS)
	val = Register32(KHwBaseDsiProtoEng +  KHoDsiVc0Ctrl+(gVidModeVc*0x20));
	val = val & ~(1<<9);
	SetRegister32(KHwBaseDsiProtoEng + KHoDsiVc0Ctrl+(gVidModeVc*0x20),val);

	// Enable Command Mode
	val = Register32(KHwBaseDsiProtoEng + KHoDsiVc0Ctrl + (gVidModeVc*0x20));
	val = val | (1<<0);
	SetRegister32(KHwBaseDsiProtoEng + KHoDsiVc0Ctrl + (gVidModeVc*0x20),val);

	// Send Short packet
	header = (0<<24)|
	         (aData1<<16)|
	         (aData0<<8)|
	         (aDcsCmdType<<0)|
	         (aVirChannel<<6);

	//printk("HEADER = 0x%x",header);
	mdelay(2);

	SetRegister32(KHwBaseDsiProtoEng + KHoDsiVcShortPktHeader+(gVidModeVc*0x20),header);

	if ((aData0 == 0x1) || (aData0 == 0x11))
	{
		//SetRegister32(KHwBaseDsiProtoEng + KHoDsiVcLongPktHeader+(gVidModeVc*0x20),0x00000409);
		//SetRegister32(KHwBaseDsiProtoEng + KHoDsiVcLongPktPayLoad+(gVidModeVc*0x20),0xFFFFFFFF);
	}


	do
	{
		val = Register32(KHwBaseDsiProtoEng + KHoDsiVc0IrqStatus+(gVidModeVc*0x20));
		for (u=0;u<100000;u++);
	}while ( (!(val & KHtDsiVc0IrqStatusPktSentIrq)) && (--count));


	if(count)
	{
		SetRegister32(KHwBaseDsiProtoEng + KHoDsiVc0IrqStatus+(gVidModeVc*0x20),KHtDsiVc0IrqStatusPktSentIrq);
		printk("Packet sent success");
		return;
	}
	else
	{
		printk("Packet sent fail");
	}

}


int ResetDsiProtoEng()
{
	unsigned int val=0;
	unsigned int val2;
	unsigned int u;

	// Reset the DSI protocol engine
	val = Register32(KHwBaseDsiProtoEng + KHoDsiSysConfig);
	val = val | (1<<1);
	SetRegister32(KHwBaseDsiProtoEng + KHoDsiSysConfig,val);

	// Wait for reset to complete
	while(Register32(KHwBaseDsiProtoEng + KHoDsiSysStatus) != 0x1 );


	//doing a dummy read to PHY register for reset work around
	/*val = Register32(KHwBaseComplexIO + KHoDsiPhyConfig0);
	do
	{
		val = Register32(KHwBaseDsiProtoEng + KHoComplexIOCfg1);
		// Wait loop
		for (u = 0; u<100000;u++);
	}while(!(val & KHtComplexIOCfg1ResetDone)); // Check the RESET done bit*/


	return 1;
}

void LockDsiPll()
{
	unsigned int val;

		// Select the manual mode of PLL update
		val = Register32(KHwBaseDsiPllController + KHoDsiPllCntrl);
		val = val & ~(1<<0);
		SetRegister32(KHwBaseDsiPllController + KHoDsiPllCntrl,val);

		// DSIPHY clock is disabled and HSDIV in bypass mode
		val = Register32(KHwBaseDsiPllController + KHoDsiPllConfig2);
		val = val & ~(1<<14);
		val = val | (1<<20);
		SetRegister32(KHwBaseDsiPllController + KHoDsiPllConfig2,val);

		// Input reference clock to PLL is SYSCLK (DSS2_FCLK)
		val = Register32(KHwBaseDsiPllController + KHoDsiPllConfig2);
		val = val & ~(1<<11);
		SetRegister32(KHwBaseDsiPllController + KHoDsiPllConfig2,val);

		// Enable the HIGHFREQ divider if input clock is greater than 32MHz
		val = Register32(KHwBaseDsiPllController + KHoDsiPllConfig2);
		val = val | (0<<12);
		SetRegister32(KHwBaseDsiPllController + KHoDsiPllConfig2,val);

		// Configure the divisor values
		val = Register32(KHwBaseDsiPllController + KHoDsiPllConfig1);
		val = val |

		  (3<<26) |
		  (3<<21) |
		  (102<<9) |
		  (18<<1) |
		  (1<<0);

		SetRegister32(KHwBaseDsiPllController + KHoDsiPllConfig1,val);

		// Enable the DSI proto engine clock divider from HSDIV
		val = Register32(KHwBaseDsiPllController + KHoDsiPllConfig2);
		val = val | (1<<18);
		SetRegister32(KHwBaseDsiPllController + KHoDsiPllConfig2,val);

		// Enable the DSS clock divider from HSDIV
		val = Register32(KHwBaseDsiPllController + KHoDsiPllConfig2);
		val = val | (1<<16);
		SetRegister32(KHwBaseDsiPllController + KHoDsiPllConfig2,val);

		// Select the refernce as SYSCLK
		val = Register32(KHwBaseDsiPllController + KHoDsiPllConfig2);
		val = val | (0x3<<21);
		SetRegister32(KHwBaseDsiPllController + KHoDsiPllConfig2,val);

		// Select the DCO frequency range to 500-1000MHz
		val = Register32(KHwBaseDsiPllController + KHoDsiPllConfig2);
		val = val | (0x2<<1);
		//SetRegister32(KHwBaseDsiPllController + KHoDsiPllConfig2,val);
		ModifyRegister32(KHwBaseDsiPllController + KHoDsiPllConfig2,(1<<1 | 1<<2 | 1<<3),val);

		// Enable PLL reference clock
		val = Register32(KHwBaseDsiPllController + KHoDsiPllConfig2);
		val = val | (1<<13);
		SetRegister32(KHwBaseDsiPllController + KHoDsiPllConfig2,val);

		// Enable DSIPHY clock and HSDIV in normal mode
		val = Register32(KHwBaseDsiPllController + KHoDsiPllConfig2);
		val = val | (1<<14);
		val = val & (~(1<<20));
		SetRegister32(KHwBaseDsiPllController + KHoDsiPllConfig2,val);

		// Start the PLL locking by setting PLL GO
		val = 1;
		SetRegister32(KHwBaseDsiPllController + KHoDsiPllGo,val);

		// Waiting for the lock request to be issued to PLL
		while(Register32(KHwBaseDsiPllController + KHoDsiPllGo) != 0);

		// Waiting for the PLL to be locked
		while( ( (Register32(KHwBaseDsiPllController + KHoDsiPllStatus) & 0x02) != 0x02));

		//
	return;
}

void DsiCplxIoPwrOn()
{
	int u;
	int val;

	//Kern::Printf("dsiMux = %x",dsiMux);
	//turn Cio clock on
	ModifyRegister32((KHwBaseDsiProtoEng + (0 * KHoSecDsiOffset))  + KHoDsiClkCtrl,KClearNone,1<<14);

	// send power command to complexio module
	val = Register32(KHwBaseDsiProtoEng + KHoComplexIOCfg1);
	val = ((val & 0xE7FFFFFF) |
	      (1 << 27));

	SetRegister32(KHwBaseDsiProtoEng + KHoComplexIOCfg1,val);

	// Wait loop
	for (u = 0; u<100000;u++);

	val = Register32(KHwBaseDsiProtoEng + KHoComplexIOCfg1);
	val = val | (1<<30);
	SetRegister32(KHwBaseDsiProtoEng + KHoComplexIOCfg1,val);

	// Wait loop
	for (u = 0; u<100000;u++);

	// Check whether the power status is changed
	do
	{
		val = Register32(KHwBaseDsiProtoEng + KHoComplexIOCfg1);
		val = ((val & KHtComplexIOCfg1PwrStatus) >> 25);
		// Wait loop
		for (u = 0; u<100000;u++);
	}while((val != 1));
}


void ConfigCmplxIo()
{
	unsigned int val;
	int u;
	int count;
	void __iomem*phymux_base;

	// Do The Mux
	unsigned int dsiMux = 0xFFFF0000;//((1 << 14) | (1 << 19) | (0x1F << 24) | (0x0 << 29)); // Pulling Down DSI lanes, Enabling DSI Lanes


	phymux_base = ioremap(0x4A100000,0x1000);

	//turn Cio clock on
	ModifyRegister32((KHwBaseDsiProtoEng + (0 * KHoSecDsiOffset))  + KHoDsiClkCtrl,KClearNone,1<<14);
	// Turning on DSI PHY Mux
	SetRegister32(phymux_base+0x618,dsiMux);
	dsiMux = Register32(phymux_base+0x618);

// COnfigure the lanes
	val = Register32(KHwBaseDsiProtoEng + KHoComplexIOCfg1);
	val	= val |
	      (
		   (0<<11)|
	       (3<<8)| // LAne 2
	       (0<<7)|
	       (2<<4)| // Lane 1
	       (0<<3)|
	       (1<<0)); // Clk Pos
	SetRegister32(KHwBaseDsiProtoEng + KHoComplexIOCfg1,val);

	//DsiCplxIoPwrOn();

	// Timing Configurations
	// Configure the DSI PHY timing parameters
	val = ((9<<24)|
		   (20<<16)|
		   (6<<8)|
		   (15<<0));
	SetRegister32(KHwBaseComplexIO + KHoDsiPhyConfig0,val);

	val = Register32(KHwBaseComplexIO + KHoDsiPhyConfig1);
	val = val & ~(0x007FFFFF);
	val = ((2<<29)|
	       (0<<27)|
	       (2<<24)|
	       (3<<16)|
	       (6<<8)|
	       (26<<0) );
	SetRegister32(KHwBaseComplexIO + KHoDsiPhyConfig1,val);

	val = Register32(KHwBaseComplexIO + KHoDsiPhyConfig2); /* this is required to preserve the reset data */
	val = val & ~(0x000000FF);
	val = val | (7 << 0);
	SetRegister32(KHwBaseComplexIO + KHoDsiPhyConfig2,val);

	// Set the GO bit
	val = Register32(KHwBaseDsiProtoEng + KHoComplexIOCfg1);
	val = val | (1<<30);
	SetRegister32(KHwBaseDsiProtoEng + KHoComplexIOCfg1,val);

	count = 100;

	// Waiting for the Go bit to be reset by HW
	while( (Register32(KHwBaseDsiProtoEng + KHoComplexIOCfg1) & (1<<30) ) && (--count) )
	{
		for (u = 0; u<100000;u++);
	}

	if(count == 0)
	{
		//__KTRACE_OPT(KDLL,Kern::Printf("config_dphy: dsi_complexio_HW_reset_status incomplete !!!! \n"));
	}

	/*do
	{
		val = Register32(KHwBaseDsiProtoEng + KHoComplexIOCfg1);
		// Wait loop
		for (u = 0; u<100000;u++);
	}while(!(val & KHtComplexIOCfg1ResetDone)); // Check the RESET done bit*/

	// enable the COMPLEXIO interrupts
	SetRegister32(KHwBaseDsiProtoEng + KHoComplexIOIrqStatus,0xFFFFFFFF); // clear the events
	SetRegister32(KHwBaseDsiProtoEng + KHoComplexIOIrqEnable,0x0); // enable the interrupt events

	DsiCplxIoPwrOn();

	// Timer configuration
	val = 0;
	val = Register32(KHwBaseDsiProtoEng  + KHoDsiTiming1);
	val = val |
	      (0 << 31)	| // TA_TO : 0x0 turn around is off
	      (1 << 15)	| // Control of ForceTxStopMode signal
	      (1 << 14)	| // STOP_STATE_X16_IO
	      (0x999);	  // STOP_STATE_COUNTER_IO

	SetRegister32(KHwBaseDsiProtoEng + KHoDsiTiming1,val);

	// Disable Command Mode
	val = Register32(KHwBaseDsiProtoEng + KHoDsiVc0Ctrl + (gVidModeVc*0x20));
	val = val & ~(1<<0);
	SetRegister32(KHwBaseDsiProtoEng + KHoDsiVc0Ctrl + (gVidModeVc*0x20),val);

	// Enable Command Mode
	val = Register32(KHwBaseDsiProtoEng + KHoDsiVc0Ctrl + (gVidModeVc*0x20));
	val = val | (1<<0);
	SetRegister32(KHwBaseDsiProtoEng + KHoDsiVc0Ctrl + (gVidModeVc*0x20),val);

	val = Register32(KHwBaseDsiProtoEng + KHoDsiTiming2);
	val = val |
	      (0<<31)|		// HS_TX_TO
	      (0<<15);		// LP_RX_TO

	SetRegister32(KHwBaseDsiProtoEng + KHoDsiTiming2,val);

	// config DDR CLK timer
	val = Register32(KHwBaseDsiProtoEng + KHoDsiClkTiming);
	val = (10 << 8) |
	      (9 << 0);
	SetRegister32(KHwBaseDsiProtoEng + KHoDsiClkTiming, val);

	return ;
}


void ConfigDsiProtoEngine()
{
	int u;
	unsigned int val=0;
	int ret;

	/* GPIO reset */
#define	GPIO_OE		0x48059134
#define	GPIO_DATAOUT	0x4805913C
#define	OMAP24XX_GPIO_CLEARDATAOUT	0x48059190
#define	OMAP24XX_GPIO_SETDATAOUT	0x48059194


	printk("ResetDSIprotoEngine");

	// 1) Reset the DSI protocol engine
	ret = ResetDsiProtoEng();
	if(ret != 1)
	{
		return;
	}

	printk("clear the reset bit");

	// 2) Clear the reset Bit
	val = Register32(KHwBaseDsiProtoEng + KHoDsiSysConfig);
	val = val & ~(1<<0);
	SetRegister32(KHwBaseDsiProtoEng + KHoDsiSysConfig,val);

	// 3) Make sure the Virtual channels are disabled before configuration
	val = Register32(KHwBaseDsiProtoEng + KHoDsiVc0Ctrl + (gVidModeVc*0x20));
	val = val & ~(1<<0);
	SetRegister32(KHwBaseDsiProtoEng + KHoDsiVc0Ctrl + (gVidModeVc*0x20),val);

#if 1
	val = Register32(KHwBaseDsiProtoEng + KHoDsiVc0Ctrl + ((gVidModeVc+1)*0x20));
	val = val & ~(1<<0);
	SetRegister32(KHwBaseDsiProtoEng + KHoDsiVc0Ctrl + ((gVidModeVc+1)*0x20),val);
#endif

	// 4) Setup the clock control
	val = Register32(KHwBaseDsiProtoEng + KHoDsiClkCtrl);
	val = val & ~(KHmDsiClkCtrlLpClkDivisor);
	val = val |
	      (1<<21) |
	      (1<<20) |
	      (1<<18) |
	      (0<<16) |
	      (0<<15) |  // 0 -> NULL packet generation disabled
	      (1<<14) |
	      (1<<13) |
	      (6<<0); // 50/7 => 7MHz
	SetRegister32(KHwBaseDsiProtoEng + KHoDsiClkCtrl,val);

#if 1 //HS mode
	/*Config VideoMode Timing */ //sv
	SetRegister32(KHwBaseDsiProtoEng + KHoDsiVmTiming1,0x02004006);
	SetRegister32(KHwBaseDsiProtoEng + KHoDsiVmTiming2,0x04010001);
	SetRegister32(KHwBaseDsiProtoEng + KHoDsiVmTiming3,0x036F01E0);
	SetRegister32(KHwBaseDsiProtoEng + KHoDsiVmTiming4,0x00487296);
	SetRegister32(KHwBaseDsiProtoEng + KHoDsiVmTiming5,0x0082DF3B);
	SetRegister32(KHwBaseDsiProtoEng + KHoDsiVmTiming6,0x7A6731D1);
	SetRegister32(KHwBaseDsiProtoEng + KHoDsiVmTiming7,0x00090007);

	/*Config VC channel */
	SetRegister32(KHwBaseDsiProtoEng + KHoDsiVc0Ctrl + ((gVidModeVc+1)*0x20),0x60809382);  //video channel

	//Clear all IRQ
	SetRegister32(KHwBaseDsiProtoEng + KHoDsiVc0IrqStatus + ((gVidModeVc+1)*0x20), 0xFF);
	SetRegister32(KHwBaseDsiProtoEng + KHoDsiVc0IrqEnable + ((gVidModeVc+1)*0x20), 0x0);
#endif //HS mode	

	// 5) Configure Command Mode Channel
	// configure the VC channel to command mode (command mode, L4 as input, HS, no DMA )
	val = Register32(KHwBaseDsiProtoEng + KHoDsiVc0Ctrl +(gVidModeVc*0x20));
	val = val & ~((1<<4)| (1<<1));
	val = val |
	      (4<<27) |
	      (4<<21) |
	      (3<<10) |
	      (3<<17) |
	      (1<<8) |
	      (1<<7) |
	      (1<<8) |
	      (0<<9) ;

	SetRegister32(KHwBaseDsiProtoEng + KHoDsiVc0Ctrl+(0*0x20),val);
	// Enable interrupt events for Command mode channel
	SetRegister32(KHwBaseDsiProtoEng + KHoDsiVc0IrqStatus+(0*0x20),0x000000FF); /* clear the events */
	SetRegister32(KHwBaseDsiProtoEng + KHoDsiVc0IrqEnable+(0*0x20),0x000000); /* enable the events */

	// 6) Configure the RX and TX fifo
	val = (0<<28) | 			// VC3 Not used
	      (0<<24) | 			// VC3 Not used
	      (0<<20) | 			// VC2 Not used
	      (0<<16) | 			// VC2 Not used
	      (4<<12) | 			// VC1 FIFO length
	      (0<<8) | 				// VC1 FIFO start address
	      (4<<4) | 				// VC0 FIFO length
	      (0<<0); 				// VC0 FIFO start address
	SetRegister32(KHwBaseDsiProtoEng + KHoDsiTxFifoVcSize,val);
	val = (0<<28) | 			// VC3 Not used
	      (0<<24) | 			// VC3 Not used
	      (0<<20) | 			// VC2 Not used
	      (0<<16) | 			// VC2 Not used
	      (1<<12) | 			// VC1 FIFO length
	      (0<<8) | 				// VC1 FIFO start address
	      (1<<4) | 				// VC0 Not used for BTA
	      (0<<0);				// VC0 Not used for BTA
	SetRegister32(KHwBaseDsiProtoEng + KHoDsiRxFifoVcSize,val);

	// 7) enable DSI IRQ
	SetRegister32(KHwBaseDsiProtoEng + KHoDsiIrqStatus,0x001FFFFF);

	printk("configure the pll");

	// 8) Configure DSI PLL
		// 1) Send PWR_ON command
		// send the power command
		val = Register32(KHwBaseDsiProtoEng + KHoDsiClkCtrl);
		val = ((val & ~(3 << 30)) | (2 << 30));
		SetRegister32(KHwBaseDsiProtoEng + KHoDsiClkCtrl,val);
		// Check whether the power status is changed
		do
		{
			val = Register32(KHwBaseDsiProtoEng + KHoDsiClkCtrl);
			val = ((val & KHtDsiClkCtrlPwrStatus) >> 28);
			// Wait loop
			for (u = 0; u<100000;u++);
		}while((val != 2));

		// 2) Wait for the PLL to get reset
		do
		{
			val = Register32(KHwBaseDsiPllController + KHoDsiPllStatus);
		}while(!(val & KHtDsiPllStatusResetDone)); // Check the RESET done bit

		// 3) Send PWR_ON command after reset
		val = Register32(KHwBaseDsiProtoEng + KHoDsiClkCtrl);
		val = ((val & ~(3 << 30)) | (2 << 30));
		SetRegister32(KHwBaseDsiProtoEng + KHoDsiClkCtrl,val);
		// Check whether the power status is changed
		do
		{
			val = Register32(KHwBaseDsiProtoEng + KHoDsiClkCtrl);
			val = ((val & KHtDsiClkCtrlPwrStatus) >> 28);
			// Wait loop
			for (u = 0; u<100000;u++);
		}while( (val != 2));

		// 4) Lock DSI PLL
		LockDsiPll();

	//Switch the dispc to DSI PLL
	SetRegister32(KHwBaseDss + KHoDSS_Control,0x103);
	for (u = 0; u<100000;u++);

	// Configure ComplexIO
	ConfigCmplxIo();

	//Configure DSI registers //sv
	val = 0x6A98;
	SetRegister32(KHwBaseDsiProtoEng + KHoDsiCtrl,val);


	// Enable OMAP-DSI Interface
	val = Register32(KHwBaseDsiProtoEng + KHoDsiCtrl);
	val = (val | (1<<0));
	SetRegister32(KHwBaseDsiProtoEng + KHoDsiCtrl,val);

	// Enable Command Mode
	val = Register32(KHwBaseDsiProtoEng + KHoDsiVc0Ctrl + (gVidModeVc*0x20));
	val = val | (1<<0);
	SetRegister32(KHwBaseDsiProtoEng + KHoDsiVc0Ctrl + (gVidModeVc*0x20),val);

#if 1 //HS mode
	// Enable Video Mode
	val = Register32(KHwBaseDsiProtoEng + KHoDsiVc0Ctrl + ((gVidModeVc+1)*0x20));
	val = val | (1<<0);
	SetRegister32(KHwBaseDsiProtoEng + KHoDsiVc0Ctrl + ((gVidModeVc+1)*0x20),val);
#endif

	// Around 100ms delay for TAAL to stabilize
	for (val=0; val <10; val++)
	for (u = 0; u < 100000; u++);

	// On 3430 SDP, Power to taal is controlled by GPIO Pin
	// This is Implemented in VarintLcdPowerUp();

	printk("configure complexio");

	val = 0;
	// Register 12
	val = val | (0x58 << 0);
	SetRegister32((KHwBaseComplexIO + (0 * KHoSecDsiOffset))  + 0x30,val);

	// Register 14
	val = 0;
	val = val | (1 << 31) | 1<<11 | (0x54 << 23) | 1<<19 | 1<<18 | (0x7 << 14);
	SetRegister32((KHwBaseComplexIO + (0 * KHoSecDsiOffset))  + 0x38,val);

	// Register 8
	val = 0;
	val = val | (1 << 11) | (16 << 6) | 1<<5 | (0xE << 0);
	SetRegister32((KHwBaseComplexIO + (0 * KHoSecDsiOffset))  + 0x20,val);

	// Around 100ms delay for TAAL to stabilize
	for (val=0; val <10; val++)
	for (u = 0; u < 100000; u++);

	mdelay(200);

	printk("send short packet");
#if 0
	// Send Command packet to TAAL to reset
	SendShortPacket(0x05, // DCS_WRITE_SHORT_NO_PARAM_CMD
			  0, // Virt Channel
			  1, // Actual Short Data (DCS_SOFT_RESET)
			  0, // No of butes, sent as zero for short packet
			  2, //mode : DSI_LPS_MODE_TX
			  1); // ECC ENable
	// Around 100ms delay for TAAL to stabiliz

	mdelay(200);

	// Send Command packet to TAAL to exit sleep mode
	SendShortPacket(0x05, // DCS_WRITE_SHORT_NO_PARAM_CMD
			  0, // Virt Channel
			  0x11, // Actual Short Data (DCS_EXIT_SLEEP_MODE)
			  0, // No of butes, sent as zero for short packet
			  2, //mode : DSI_LPS_MODE_TX
			  1); // ECC ENable

	for (val=0; val <10; val++)
	for (u = 0; u < 100000; u++);
		mdelay(100);

	// Send Command packet to set all pixels ON
	SendShortPacket(0x05, // DCS_WRITE_SHORT_NO_PARAM_CMD
			  0, // Virt Channel
			  0x23, // Actual Short Data (DCS_SET_ALL_PIXELS_ON)
			  0, // No of butes, sent as zero for short packet
			  2, //mode : DSI_LPS_MODE_TX
			  1); // ECC ENable

	// Send Command packet to set show the pixels on TAAL
	SendShortPacket(0x05, // DCS_WRITE_SHORT_NO_PARAM_CMD
			  0, // Virt Channel
			  0x29, // Actual Short Data (DCS_SET_DISPLAY_ON)
			  0, // No of butes, sent as zero for short packet
			  2, //mode : DSI_LPS_MODE_TX
			  1); // ECC ENable

	// Send Command packet to set show the pixels on TAAL (One more time)
	SendShortPacket(0x05, // DCS_WRITE_SHORT_NO_PARAM_CMD
			  0, // Virt Channel
			  0x29, // Actual Short Data (DCS_SET_DISPLAY_ON)
			  0, // No of butes, sent as zero for short packet
			  2, //mode : DSI_LPS_MODE_TX
			  1); // ECC ENable

#endif

}


void Setup_SDP()
{

	KHwBaseDss = dss_base;
	KHwBaseDisplayCtrlReg	= dispc_base;
	KHwBaseDsiProtoEng		= dsi_base ;
	KHwBaseComplexIO		= dsi_base + 0x200;
	KHwBaseDsiPllController	= dsi_base + 0x300;

	printk("dss base = 0x%x, dispc_base = 0x%x, dsiproto = 0x%x, complexio = 0x%x, dsipll = 0x%x",dss_base, dispc_base,KHwBaseDsiProtoEng,KHwBaseComplexIO,KHwBaseDsiPllController);

	ConfigDsiProtoEngine();
}
