/****************************************************************************
 *
 * Copyright (c) 2025 Renesas Electronics Corporation and/or its affiliates
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 ****************************************************************************/

/**
 * @file time_port_shim.cpp
 * @brief Time port shim implementation for Renesas RZ/V2H
 */

#if defined(__PX4_FREERTOS)

#include <cstdio>
#include <errno.h>
#include <stdint.h>
#include <string.h>
#include <time.h>
#include <atomic>

// Forward declaration — implemented in remote_storage_rpc.c
#ifdef __cplusplus
extern "C" {
#endif
int rzv_remote_clock_settime(int64_t unix_sec);
#ifdef __cplusplus
}
#endif

#include "time_port_shim.h"

#include <FreeRTOS.h>
#include <task.h>

extern "C" struct tm *gmtime_r(const time_t *timer, struct tm *result);

#ifndef NANOSECONDS_PER_SECOND
#define NANOSECONDS_PER_SECOND 1000000000ULL
#endif

namespace {

int64_t g_realtime_offset_ns{0};

constexpr uint64_t kNsPerTick = NANOSECONDS_PER_SECOND / configTICK_RATE_HZ;

uint64_t fallback_monotonic_ns()
{
    static std::atomic<uint64_t> s_last_time_ns{0};

    TimeOut_t timeout_state{};
    vTaskSetTimeOutState(&timeout_state);

    uint64_t ticks = (static_cast<uint64_t>(timeout_state.xOverflowCount) << (sizeof(TickType_t) * 8));
    ticks += timeout_state.xTimeOnEntering;

    uint64_t time_ns = ticks * kNsPerTick;

    uint64_t prev = s_last_time_ns.load(std::memory_order_relaxed);

    while (time_ns <= prev) {
        if (s_last_time_ns.compare_exchange_weak(prev, prev + 1, std::memory_order_relaxed)) {
            return prev + 1;
        }
    }

    s_last_time_ns.store(time_ns, std::memory_order_relaxed);
    return time_ns;
}

uint64_t monotonic_time_ns()
{
    const uint64_t board_time_ns = px4_board_monotonic_time_ns();

    if (board_time_ns != 0) {
        return board_time_ns;
    }

    return fallback_monotonic_ns();
}

uint64_t freertos_ticks_to_ns()
{
    return monotonic_time_ns();
}

int do_clock_gettime(clockid_t clk_id, struct timespec *tp)
{
    if (tp == nullptr) {
        errno = EFAULT;
        return -1;
    }

    const uint64_t ns = freertos_ticks_to_ns();

    switch (clk_id) {
    case CLOCK_MONOTONIC:
#if defined(CLOCK_MONOTONIC_RAW)
    case CLOCK_MONOTONIC_RAW:
#endif
#if defined(CLOCK_BOOTTIME)
    case CLOCK_BOOTTIME:
#endif
        tp->tv_sec = static_cast<time_t>(ns / NANOSECONDS_PER_SECOND);
        tp->tv_nsec = static_cast<long>(ns % NANOSECONDS_PER_SECOND);
        return 0;

    case CLOCK_REALTIME:
    {
        const int64_t realtime_ns = static_cast<int64_t>(ns) + g_realtime_offset_ns;

        if (realtime_ns < 0) {
            tp->tv_sec = 0;
            tp->tv_nsec = 0;
            return 0;
        }

        tp->tv_sec = static_cast<time_t>(realtime_ns / static_cast<int64_t>(NANOSECONDS_PER_SECOND));
        tp->tv_nsec = static_cast<long>(realtime_ns % static_cast<int64_t>(NANOSECONDS_PER_SECOND));
        return 0;
    }

    default:
        errno = EINVAL;
        return -1;
    }
}

int do_clock_settime(clockid_t clk_id, const struct timespec *tp)
{
    if ((clk_id != CLOCK_REALTIME) || (tp == nullptr)) {
        errno = EINVAL;
        return -1;
    }

    int64_t requested = static_cast<int64_t>(tp->tv_sec) * static_cast<int64_t>(NANOSECONDS_PER_SECOND) + tp->tv_nsec;
    int64_t monotonic = static_cast<int64_t>(freertos_ticks_to_ns());
    g_realtime_offset_ns = requested - monotonic;
    return 0;
}

unsigned int do_sleep(unsigned int seconds)
{
    const TickType_t ticks = pdMS_TO_TICKS(static_cast<uint64_t>(seconds) * 1000ULL);
    vTaskDelay(ticks > 0 ? ticks : 1);
    return 0;
}

struct tm *do_localtime_r(const time_t *timer, struct tm *result)
{
    if ((timer == nullptr) || (result == nullptr)) {
        errno = EINVAL;
        return nullptr;
    }

#if defined(_POSIX_THREAD_SAFE_FUNCTIONS) || defined(__NEWLIB__)
    return gmtime_r(timer, result);
#else
    struct tm *tmp = gmtime(timer);

    if (tmp == nullptr) {
        return nullptr;
    }

    memcpy(result, tmp, sizeof(struct tm));
    return result;
#endif
}

} // namespace

extern "C" uint64_t px4_port_fallback_monotonic_time_ns()
{
    return fallback_monotonic_ns();
}

extern "C" __attribute__((weak)) uint64_t px4_board_monotonic_time_ns()
{
    return 0;
}

extern "C" uint64_t px4_port_monotonic_time_ns()
{
    return monotonic_time_ns();
}

extern "C" int px4_port_clock_gettime(clockid_t clk_id, struct timespec *tp)
{
    return do_clock_gettime(clk_id, tp);
}

extern "C" int px4_port_clock_settime(clockid_t clk_id, const struct timespec *tp)
{
    int ret = do_clock_settime(clk_id, tp);

    // Forward wall-clock updates to CA55 so file mtimes (used in QGC log list) are correct.
    // QGC sends SYSTEM_TIME once per second; we sync CA55 at most once per 30 seconds.
    // Using statics is safe here: clock_settime is only called from the MAVLink receiver task.
    if (ret == 0 && clk_id == CLOCK_REALTIME && tp != nullptr) {
        static int64_t s_last_synced_sec = 0;

        if (tp->tv_sec != 0 && (tp->tv_sec - s_last_synced_sec) >= 30) {
            s_last_synced_sec = tp->tv_sec;
            rzv_remote_clock_settime(tp->tv_sec);
        }
    }

    return ret;
}

extern "C" unsigned int px4_port_sleep(unsigned int seconds)
{
    return do_sleep(seconds);
}

__attribute__((weak)) int clock_gettime(clockid_t clk_id, struct timespec *tp)
{
    return px4_port_clock_gettime(clk_id, tp);
}

__attribute__((weak)) int clock_settime(clockid_t clk_id, const struct timespec *tp)
{
    return px4_port_clock_settime(clk_id, tp);
}

__attribute__((weak)) unsigned int sleep(unsigned int seconds)
{
    return px4_port_sleep(seconds);
}

__attribute__((weak)) struct tm *localtime_r(const time_t *timer, struct tm *result)
{
    return do_localtime_r(timer, result);
}

/* localtime() and gmtime() share a single static buffer — not thread-safe.
 * Concurrent callers from different FreeRTOS tasks will corrupt each other's result.
 * Acceptable for PX4 log formatting (single logging task) but callers that need
 * safety must use localtime_r() / gmtime_r() with their own struct tm storage.
 */
__attribute__((weak)) struct tm *localtime(const time_t *timer)
{
    static struct tm tm_buf{};
    return do_localtime_r(timer, &tm_buf);
}

__attribute__((weak)) struct tm *gmtime(const time_t *timer)
{
    static struct tm tm_buf{};
    return do_localtime_r(timer, &tm_buf);
}

__attribute__((weak)) size_t strftime(char *str, size_t maxsize, const char *format, const struct tm *timeptr)
{
    if ((str == nullptr) || (maxsize == 0) || (format == nullptr) || (timeptr == nullptr)) {
        return 0;
    }

    size_t out = 0;

    for (const char *p = format; *p != '\0'; ++p) {
        if (*p != '%') {
            if (out + 1U >= maxsize) { str[out] = '\0'; return 0; }
            str[out++] = *p;
            continue;
        }

        ++p;
        if (*p == '\0') { break; }

        char tmp[32];
        int n = 0;

        switch (*p) {
        case 'Y': n = snprintf(tmp, sizeof(tmp), "%04d", timeptr->tm_year + 1900);   break;
        case 'y': n = snprintf(tmp, sizeof(tmp), "%02d", (timeptr->tm_year + 1900) % 100); break;
        case 'm': n = snprintf(tmp, sizeof(tmp), "%02d", timeptr->tm_mon + 1);        break;
        case 'd': n = snprintf(tmp, sizeof(tmp), "%02d", timeptr->tm_mday);           break;
        case 'e': n = snprintf(tmp, sizeof(tmp), "%2d",  timeptr->tm_mday);           break;
        case 'H': n = snprintf(tmp, sizeof(tmp), "%02d", timeptr->tm_hour);           break;
        case 'I': n = snprintf(tmp, sizeof(tmp), "%02d", (timeptr->tm_hour % 12) == 0 ? 12 : timeptr->tm_hour % 12); break;
        case 'M': n = snprintf(tmp, sizeof(tmp), "%02d", timeptr->tm_min);            break;
        case 'S': n = snprintf(tmp, sizeof(tmp), "%02d", timeptr->tm_sec);            break;
        case 'j': n = snprintf(tmp, sizeof(tmp), "%03d", timeptr->tm_yday + 1);       break;
        case 'w': n = snprintf(tmp, sizeof(tmp), "%d",   timeptr->tm_wday);           break;
        case 'u': n = snprintf(tmp, sizeof(tmp), "%d",   timeptr->tm_wday == 0 ? 7 : timeptr->tm_wday); break;
        case 'p': n = snprintf(tmp, sizeof(tmp), "%s",   timeptr->tm_hour < 12 ? "AM" : "PM"); break;
        case 'Z': n = snprintf(tmp, sizeof(tmp), "UTC");                              break;
        case 'z': n = snprintf(tmp, sizeof(tmp), "+0000");                            break;
        case 'n': tmp[0] = '\n'; tmp[1] = '\0'; n = 1;                               break;
        case 't': tmp[0] = '\t'; tmp[1] = '\0'; n = 1;                               break;
        case '%': tmp[0] = '%'; tmp[1] = '\0'; n = 1;                                break;
        case 'c': /* full datetime */
            n = snprintf(tmp, sizeof(tmp), "%04d-%02d-%02d %02d:%02d:%02d",
                         timeptr->tm_year + 1900, timeptr->tm_mon + 1, timeptr->tm_mday,
                         timeptr->tm_hour, timeptr->tm_min, timeptr->tm_sec);
            break;
        case 'x': /* date only */
            n = snprintf(tmp, sizeof(tmp), "%04d-%02d-%02d",
                         timeptr->tm_year + 1900, timeptr->tm_mon + 1, timeptr->tm_mday);
            break;
        case 'X': /* time only */
            n = snprintf(tmp, sizeof(tmp), "%02d:%02d:%02d",
                         timeptr->tm_hour, timeptr->tm_min, timeptr->tm_sec);
            break;
        default:
            /* Unknown specifier: emit literally (e.g. "%Q" -> "%Q") */
            tmp[0] = '%'; tmp[1] = *p; tmp[2] = '\0'; n = 2;
            break;
        }

        if ((n <= 0) || (out + static_cast<size_t>(n) >= maxsize)) {
            str[out] = '\0';
            return 0;
        }

        memcpy(str + out, tmp, static_cast<size_t>(n));
        out += static_cast<size_t>(n);
    }

    str[out] = '\0';
    return out;
}

__asm__(".weak clock_gettime\n"
        "clock_gettime = _Z13clock_gettimeiP8timespec\n"
        ".weak clock_settime\n"
        "clock_settime = _Z13clock_settimeiPK8timespec\n"
        ".weak sleep\n"
        "sleep = _Z5sleepj\n"
        ".weak gmtime\n"
        "gmtime = _Z5gmtimePKl\n"
        ".weak localtime_r\n"
        "localtime_r = _Z11localtime_rPKlP2tm\n"
        ".weak localtime\n"
        "localtime = _Z8localtimePKl\n"
        ".weak strftime\n"
        "strftime = _Z8strftimePcjPKcPK2tm\n");

#endif /* __PX4_FREERTOS */
