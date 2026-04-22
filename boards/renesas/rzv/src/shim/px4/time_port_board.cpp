/****************************************************************************
 *
 * Copyright (c) 2025 Renesas Electronics Corporation and/or its affiliates
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 ****************************************************************************/

/**
 * @file time_port_board.cpp
 * @brief Time port board implementation for Renesas RZ/V2H
 */

#if defined(__PX4_FREERTOS)

#include "time_port_shim.h"

#include <cstdint>

extern "C" {
uint64_t hrt_absolute_time(void);
}

extern "C" uint64_t px4_board_monotonic_time_ns()
{
    const uint64_t time_us = hrt_absolute_time();

    if (time_us == 0) {
        return px4_port_fallback_monotonic_time_ns();
    }

    return time_us * 1000ULL;
}

#endif /* __PX4_FREERTOS */
