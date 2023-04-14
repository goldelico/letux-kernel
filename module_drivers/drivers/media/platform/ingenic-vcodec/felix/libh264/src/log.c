//#include "config.h"

#ifdef __KERNEL__
#else
#include <stdarg.h>
#include <stdlib.h>
#endif
#include "common.h"
//#include "internal.h"
#include "log.h"


#define LINE_SZ 1024


static int av_log_level = AV_LOG_INFO;
static int flags;

#define MAX_LOG_BUFFER_SIZE	1024
unsigned char print_buffer[MAX_LOG_BUFFER_SIZE];
void av_log_default_callback(void* ptr, int level, const char* fmt, va_list vl)
{
	int n = 0;
	n = vsprintf(print_buffer, fmt, vl);
	if(n > MAX_LOG_BUFFER_SIZE) {
		printf("%s,%d, log too large.!\n", __func__, __LINE__);
		return;
	}

	if(level < av_log_level)
		printf(print_buffer);
}

static void (*av_log_callback)(void*, int, const char*, va_list) =
    av_log_default_callback;

void av_log(void* avcl, int level, const char *fmt, ...)
{
    va_list vl;
    va_start(vl, fmt);
    av_vlog(avcl, level, fmt, vl);
    va_end(vl);
}

void av_vlog(void* avcl, int level, const char *fmt, va_list vl)
{
    void (*log_callback)(void*, int, const char*, va_list) = av_log_callback;
    if (log_callback)
        log_callback(avcl, level, fmt, vl);
}

int av_log_get_level(void)
{
    return av_log_level;
}

void av_log_set_level(int level)
{
    av_log_level = level;
}

void av_log_set_flags(int arg)
{
    flags = arg;
}

int av_log_get_flags(void)
{
    return flags;
}

void av_log_set_callback(void (*callback)(void*, int, const char*, va_list))
{
    av_log_callback = callback;

}

static void missing_feature_sample(int sample, void *avc, const char *msg,
                                   va_list argument_list)
{
    av_vlog(avc, AV_LOG_WARNING, msg, argument_list);
    av_log(avc, AV_LOG_WARNING, " is not implemented. Update your FFmpeg "
           "version to the newest one from Git. If the problem still "
           "occurs, it means that your file has a feature which has not "
           "been implemented.\n");
    if (sample)
        av_log(avc, AV_LOG_WARNING, "If you want to help, upload a sample "
               "of this file to ftp://upload.ffmpeg.org/incoming/ "
               "and contact the ffmpeg-devel mailing list. (ffmpeg-devel@ffmpeg.org)\n");
}

void avpriv_request_sample(void *avc, const char *msg, ...)
{
    va_list argument_list;

    va_start(argument_list, msg);
    missing_feature_sample(1, avc, msg, argument_list);
    va_end(argument_list);
}

void avpriv_report_missing_feature(void *avc, const char *msg, ...)
{
    va_list argument_list;

    va_start(argument_list, msg);
    missing_feature_sample(0, avc, msg, argument_list);
    va_end(argument_list);
}
