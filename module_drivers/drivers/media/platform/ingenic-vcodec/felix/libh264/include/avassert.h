#ifndef AVUTIL_AVASSERT_H
#define AVUTIL_AVASSERT_H

#ifdef __KERNEL__
#else
#include <stdlib.h>
#endif
//#include "avutil.h"
#include "log.h"

/**
 * assert() equivalent, that is always enabled.
 */
#define av_assert0(cond) do {                                           \
    if (!(cond)) {                                                      \
        av_log(NULL, AV_LOG_PANIC, "Assertion %s failed at %s:%d\n",    \
               cond, __FILE__, __LINE__);                 \
    }                                                                   \
} while (0)

#define assert(cond)	av_assert0(cond)


/**
 * assert() equivalent, that does not lie in speed critical code.
 * These asserts() thus can be enabled without fearing speed loss.
 */
#if defined(ASSERT_LEVEL) && ASSERT_LEVEL > 0
#define av_assert1(cond) av_assert0(cond)
#else
#define av_assert1(cond) ((void)0)
#endif


/**
 * assert() equivalent, that does lie in speed critical code.
 */
#if defined(ASSERT_LEVEL) && ASSERT_LEVEL > 1
#define av_assert2(cond) av_assert0(cond)
#define av_assert2_fpu() av_assert0_fpu()
#else
#define av_assert2(cond) ((void)0)
#define av_assert2_fpu() ((void)0)
#endif

/**
 * Assert that floating point operations can be executed.
 *
 * This will av_assert0() that the cpu is not in MMX state on X86
 */
void av_assert0_fpu(void);

#endif /* AVUTIL_AVASSERT_H */
