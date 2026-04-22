/****************************************************************************
 *
 * Copyright (c) 2025 Renesas Electronics Corporation and/or its affiliates
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 ****************************************************************************/

/**
 * @file freertos_time_utils.hpp
 * @brief Time utility functions for FreeRTOS on Renesas RZ/V2H
 */
#pragma once

#if defined(__PX4_FREERTOS)


#include <FreeRTOS.h>
#include <drivers/drv_hrt.h>

#include <stdint.h>
#include <time.h>

namespace rzv::freertos
{

static constexpr int64_t kNsPerSec{1000000000LL};
static constexpr int64_t kNsPerTick{1000000000LL / configTICK_RATE_HZ};

inline int64_t timespec_to_ns(const struct timespec &ts)
{
	return static_cast<int64_t>(ts.tv_sec) * kNsPerSec + static_cast<int64_t>(ts.tv_nsec);
}

inline TickType_t ns_to_ticks(int64_t ns)
{
	if ((ns <= 0) || (kNsPerTick <= 0)) {
		return 0;
	}

	uint64_t ticks = (static_cast<uint64_t>(ns) + (static_cast<uint64_t>(kNsPerTick) - 1ULL)) /
			 static_cast<uint64_t>(kNsPerTick);

	if (ticks == 0) {
		ticks = 1;
	}

	TickType_t finite_max = portMAX_DELAY;

#if (INCLUDE_vTaskSuspend == 1)
	if (finite_max > 0) {
		--finite_max;
	}
#endif

	if ((finite_max > 0) && (ticks > static_cast<uint64_t>(finite_max))) {
		return finite_max;
	}

	if (ticks > static_cast<uint64_t>(portMAX_DELAY)) {
		return portMAX_DELAY;
	}

	return static_cast<TickType_t>(ticks);
}

inline TickType_t abstime_to_ticks(const struct timespec *abstime)
{
	if (abstime == nullptr) {
		return 0;
	}

	struct timespec now {};

	if (clock_gettime(CLOCK_MONOTONIC, &now) == 0) {
		return ns_to_ticks(timespec_to_ns(*abstime) - timespec_to_ns(now));
	}

	const uint64_t now_us = hrt_absolute_time();

	if (now_us > 0ULL) {
		const int64_t now_ns = static_cast<int64_t>(now_us) * 1000LL;
		return ns_to_ticks(timespec_to_ns(*abstime) - now_ns);
	}

	// fallback to a single tick if we couldn't compute the delta
	return 1;
}

} // namespace rzv::freertos

#endif /* __PX4_FREERTOS */
