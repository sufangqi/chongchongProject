/******************************************************************************
 *     Copyright (c) 2015 PuTao Co., Ltd.
 *     All Rights Reserved.
 ******************************************************************************/

#ifndef __TIME_H__
#define __TIME_H__

#ifdef _PERF_

#ifdef _ON_LINUX_
#include <sys/time.h>
#endif

static unsigned long long getTickCountMicroSec()
{
#ifdef _ON_LINUX_
    struct timeval g_tv;
    struct timezone g_tz;
    gettimeofday(&g_tv, &g_tz);
    return g_tv.tv_sec * 1000000 + g_tv.tv_usec;
#else
    return (unsigned long long)(((double)cv::getTickCount()/(double)cv::getTickFrequency())*1000000.0f);
#endif
}

#define PERF_START() \
    _ptstart = getTickCountMicroSec();

#define PERF_END(header) \
    _ptend      = getTickCountMicroSec(); \
    _ptelapse   = _ptend - _ptstart; \
    _pttelapse += _ptelapse; \
    PTDEBUG("%10s: %lld us\n", header, _ptelapse);

#else /* !_PERF_ */

#define PERF_START()
#define PERF_END(header)

#endif /* _PERF_ */

#endif /* __TIME_H__ */

/* EOF */


