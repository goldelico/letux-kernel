#ifndef __MPEGUTILS_H__
#define __MPEGUTILS_H__


/**
 * Return value for header parsers if frame is not coded.
 * */
#define FRAME_SKIPPED 100

/* picture type */
#define PICT_TOP_FIELD     1
#define PICT_BOTTOM_FIELD  2
#define PICT_FRAME         3

/**
 * Value of Picture.reference when Picture is not a reference picture, but
 * is held for delayed output.
 */
#define DELAYED_PIC_REF 4

#define MAX_MB_BYTES    (30 * 16 * 16 * 3 / 8 + 120)
#define MAX_FCODE        7




#endif
