#ifndef __MATHOPS_H__
#define __MATHOPS_H__

#ifdef __KERNEL__
#include <linux/types.h>
#endif


#define MAX_NEG_CROP 1024
extern const uint32_t ff_inverse[257];
extern const uint8_t ff_sqrt_tab[256];
extern const uint8_t ff_crop_tab[256 + 2 * MAX_NEG_CROP];
extern const uint8_t ff_zigzag_direct[64];
extern const uint8_t ff_zigzag_scan[16+1];

#ifndef NEG_SSR32
#   define NEG_SSR32(a,s) ((( int32_t)(a))>>(32-(s)))
#endif

#ifndef NEG_USR32
#   define NEG_USR32(a,s) (((uint32_t)(a))>>(32-(s)))
#endif


#ifndef ff_ctz
#define ff_ctz(v) __builtin_ctz(v)
#endif
#ifndef ff_ctzll
#define ff_ctzll(v) __builtin_ctzll(v)
#endif
#ifndef ff_clz
#define ff_clz(v) __builtin_clz(v)
#endif

#ifndef ff_log2
#define ff_log2(x) (31 - __builtin_clz((x)|1))
#endif

#ifndef ff_log2_16bit
#define ff_log2_16bit	ff_log2
#endif

#define av_log2	ff_log2


#ifndef sign_extend
static inline int sign_extend(int val, unsigned bits)
{
    unsigned shift = 8 * sizeof(int) - bits;
    union { unsigned u; int s; } v = { (unsigned) val << shift };
    return v.s >> shift;
}
#endif

#ifndef zero_extend
static inline unsigned zero_extend(unsigned val, unsigned bits)
{
    return (val << ((8 * sizeof(int)) - bits)) >> ((8 * sizeof(int)) - bits);
}
#endif


#endif
