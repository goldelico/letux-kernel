#ifndef AVUTIL_MEM_H
#define AVUTIL_MEM_H

#ifdef __KERNEL__
#else
#include <limits.h>
#include <stdint.h>
#endif

//#include "attributes.h"
#include "error.h"

void *av_malloc(size_t size) ;//av_malloc_attrib av_alloc_size(1);

void *av_mallocz(size_t size);

void *av_malloc_array(size_t nmemb, size_t size);

void *av_mallocz_array(size_t nmemb, size_t size);

void *av_calloc(size_t nmemb, size_t size);

void *av_realloc(void *ptr, size_t size);

int av_reallocp(void *ptr, size_t size);

void *av_realloc_f(void *ptr, size_t nelem, size_t elsize);

void *av_realloc_array(void *ptr, size_t nmemb, size_t size);

int av_reallocp_array(void *ptr, size_t nmemb, size_t size);

void *av_fast_realloc(void *ptr, unsigned int *size, size_t min_size);

void av_fast_malloc(void *ptr, unsigned int *size, size_t min_size);

void av_fast_mallocz(void *ptr, unsigned int *size, size_t min_size);

void av_free(void *ptr);

void av_freep(void *ptr);

char *av_strdup(const char *s);

char *av_strndup(const char *s, size_t len);

void *av_memdup(const void *p, size_t size);

void av_memcpy_backptr(uint8_t *dst, int back, int cnt);

/**
 * Multiply two `size_t` values checking for overflow.
 *
 * @param[in]  a,b Operands of multiplication
 * @param[out] r   Pointer to the result of the operation
 * @return 0 on success, AVERROR(EINVAL) on overflow
 */
static inline int av_size_mult(size_t a, size_t b, size_t *r)
{
    size_t t = a * b;
    /* Hack inspired from glibc: don't try the division if nelem and elsize
     * are both less than sqrt(SIZE_MAX). */
    if ((a | b) >= ((size_t)1 << (sizeof(size_t) * 4)) && a && t / a != b)
        return AVERROR(EINVAL);
    *r = t;
    return 0;
}

void av_max_alloc(size_t max);

#endif /* AVUTIL_MEM_H */
