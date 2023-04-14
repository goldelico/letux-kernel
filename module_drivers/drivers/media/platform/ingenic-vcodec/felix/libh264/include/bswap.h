#ifndef AVUTIL_BSWAP_H
#define AVUTIL_BSWAP_H

//#include "libavutil/avconfig.h"
//#include "attributes.h"

#define AV_BSWAP16C(x) (((x) << 8 & 0xff00)  | ((x) >> 8 & 0x00ff))
#define AV_BSWAP32C(x) (AV_BSWAP16C(x) << 16 | AV_BSWAP16C((x) >> 16))
#define AV_BSWAP64C(x) (AV_BSWAP32C(x) << 32 | AV_BSWAP32C((x) >> 32))

#define AV_BSWAPC(s, x) AV_BSWAP##s##C(x)

#ifndef av_bswap16
static inline const uint16_t av_bswap16(uint16_t x)
{
    x= (x>>8) | (x<<8);
    return x;
}
#endif

#ifndef av_bswap32
static inline const uint32_t av_bswap32(uint32_t x)
{
    return AV_BSWAP32C(x);
}
#endif

#ifndef av_bswap64
static inline uint64_t const av_bswap64(uint64_t x)
{
    return (uint64_t)av_bswap32(x) << 32 | av_bswap32(x >> 32);
}
#endif

// be2ne ... big-endian to native-endian
// le2ne ... little-endian to native-endian

#define av_be2ne16(x) av_bswap16(x)
#define av_be2ne32(x) av_bswap32(x)
#define av_be2ne64(x) av_bswap64(x)
#define av_le2ne16(x) (x)
#define av_le2ne32(x) (x)
#define av_le2ne64(x) (x)
#define AV_BE2NEC(s, x) AV_BSWAPC(s, x)
#define AV_LE2NEC(s, x) (x)

#define AV_BE2NE16C(x) AV_BE2NEC(16, x)
#define AV_BE2NE32C(x) AV_BE2NEC(32, x)
#define AV_BE2NE64C(x) AV_BE2NEC(64, x)
#define AV_LE2NE16C(x) AV_LE2NEC(16, x)
#define AV_LE2NE32C(x) AV_LE2NEC(32, x)
#define AV_LE2NE64C(x) AV_LE2NEC(64, x)

#endif /* AVUTIL_BSWAP_H */
