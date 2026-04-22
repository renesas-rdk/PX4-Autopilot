/****************************************************************************
 *
 * Copyright (c) 2025 Renesas Electronics Corporation and/or its affiliates
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 ****************************************************************************/

/**
 * @file time_port_shim.h
 * @brief Time port shim header for Renesas RZ/V2H
 */
#pragma once

#if defined(__PX4_FREERTOS)


#include <stdint.h>
#include <time.h>

#ifdef __cplusplus
extern "C" {
#endif

uint64_t px4_port_monotonic_time_ns(void);
uint64_t px4_port_fallback_monotonic_time_ns(void);
uint64_t px4_board_monotonic_time_ns(void);
int px4_port_clock_gettime(clockid_t clk_id, struct timespec *tp);
int px4_port_clock_settime(clockid_t clk_id, const struct timespec *tp);
unsigned int px4_port_sleep(unsigned int seconds);

#ifdef __cplusplus
}

#endif /* __PX4_FREERTOS */

#endif
