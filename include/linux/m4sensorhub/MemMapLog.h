/**********************************************************************
*
*   Copyright (C) 2012 Motorola, Inc.
*
**********************************************************************
File    : MemMapLog.h
Purpose :
**********************************************************************/
#ifndef __MEMMAP_LOG_H__
#define __MEMMAP_LOG_H__
/****************************** Defines *******************************/
#define LOG_EN_SIZE 2
/* Number of log tags per element of logenable array */
#define LOG_TAGS_PER_ENABLE 16
#define LOG_MAX 23
#define LOG_NO_OF_BITS_PER_TAG 2
/*This is set to 0x03 since each logtag uses 2 bits in logenable */
#define LOG_TAG_MASK 0x03
#define LOG_LEVELS_MAX 4

struct memMapLog {
	u32 logEnable[LOG_EN_SIZE];
	u8 isLogImmediate;
};

static char acLogTags[LOG_MAX][40] = {
	"LOG_GENERAL",
	"LOG_TIMER",
	"LOG_ACCEL",
	"LOG_TEMPERATURE",
	"LOG_PRESSURE",
	"LOG_PEDOMETER",
	"LOG_TCMD",
	"LOG_GYRO",
	"LOG_COMPASS",
	"LOG_FUSION",
	"LOG_METS",
	"LOG_GESTURE",
	"LOG_POWER",
	"LOG_CORRELATION",
	"LOG_GPS",
	"LOG_DL",
	"LOG_AUDIO",
	"LOG_DISP",
	"LOG_WRIST",
	"LOG_PASSIVE",
	"LOG_EMG",
	"LOG_HR",
	"LOG_ALS",
};

static char acLogLevels[LOG_LEVELS_MAX][15] = {
	"LOG_DISABLE",
	"LOG_ERROR",
	"LOG_VERBOSE",
	"LOG_DEBUG"
};
#endif /* __MEMMAP_LOG_H__ */
