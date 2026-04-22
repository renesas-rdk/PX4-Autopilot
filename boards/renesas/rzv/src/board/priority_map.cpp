/****************************************************************************
 *
 * Copyright (c) 2025 Renesas Electronics Corporation and/or its affiliates
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 ****************************************************************************/

/**
 * @file priority_map.cpp
 * @brief Board-specific priority mapping for Renesas RZ/V2H
 */

#if defined(__PX4_FREERTOS)

#include <px4_platform_common/tasks.h>

#include <FreeRTOS.h>
#include <FreeRTOSConfig.h>

#include <cmath>

namespace
{
struct PriorityMapEntry {
	int key;
	UBaseType_t mapped;
};

// Mapping applied to priorities computed with configMAX_PRIORITIES=32.
constexpr PriorityMapEntry kCompressedPriorityMap[] = {
	{31, 30},   // wq:rate_ctrl
	{30, 29},   // wq:SPI0
	{29, 28},   // SPI work queues, keeps cascade
	{28, 27},
	{23, 25},   // I2C/I2C-like workers
	{22, 25},
	{21, 25},
	{20, 25},
	{19, 25},
	{18, 26},   // wq:nav_and_controllers
	{17, 28},   // wq:INS0
	{13, 23},   // wq:hp_default
	{12, 22},   // wq:uavcan / custom mid-priority drivers
	{10, 18},   // wq:ttyS0
	{9, 18},    // wq:ttyS1
	{8, 18},    // wq:ttyS2
	{7, 18},    // wq:ttyS3
	{6, 18},    // wq:ttyS4
	{5, 18},    // wq:ttyS5
	{4, 18},    // wq:ttyS6
	{3, 18},    // wq:ttyS7
	{2, 18},    // wq:ttyS8
	{1, 9},     // Main task, ttyS9 fallback
	{0, 8},     // Commander, navigator, etc.
	{-1, 18},   // wq:ttyUnknown
	{-4, 14},   // wq:slow_driver equivalents
	{-19, 12},  // wq:lp_default (relative -50)
};

// Legacy PX4 priorities captured from running.log before compressing FreeRTOS priorities.
// These values ensure that any hardcoded priority still lands in the expected slot.
constexpr PriorityMapEntry kLegacyPriorityMap[] = {
	{95, 30}, // wq:rate_ctrl
	{94, 29}, // wq:SPI0
	{93, 28}, // other SPI workers
	{92, 27}, // estimator watchdogs
	{82, 26}, // wq:nav_and_controllers
	{81, 28}, // wq:INS0
	{55, 27}, // commander (SCHED_PRIORITY_DEFAULT + 40)
	{77, 23}, // wq:hp_default
	{76, 24}, // drdy_task deferred worker
	{37, 12}, // dataman (SCHED_PRIORITY_DEFAULT - 10 with legacy max)
	{68, 18}, // serial work queues (ttyS*)
	{45, 12}, // wq:lp_default
	{21, 9},  // commander and similar medium tasks
	{15, 10}, // mavrx task
	{1, 8},   // Main/daemon threads
	{0, 8},   // anything requesting base priority 0
};

constexpr int kLegacyMaxPriority = 95;

template <size_t N>
int lookup_priority(int priority, const PriorityMapEntry (&table)[N])
{
	for (const auto &entry : table) {
		if (priority == entry.key) {
			return entry.mapped;
		}
	}

	return -1;
}
}

extern "C" int px4_board_map_priority(int priority)
{
	if (priority < 0) {
		const int mapped = lookup_priority(priority, kCompressedPriorityMap);

		if (mapped >= 0) {
			return mapped;
		}

		return 0;
	}

	if (priority < configMAX_PRIORITIES) {
		const int mapped = lookup_priority(priority, kCompressedPriorityMap);

		if (mapped >= 0) {
			return mapped;
		}

		return priority;
	}

	for (const auto &entry : kLegacyPriorityMap) {
		if (priority == entry.key) {
			return entry.mapped;
		}
	}

	constexpr float legacy_max = static_cast<float>(kLegacyMaxPriority);
	constexpr float freertos_max = static_cast<float>(configMAX_PRIORITIES - 1);
	const float scaled = (static_cast<float>(priority) / legacy_max) * freertos_max;

	int mapped = lrintf(scaled);

	if (mapped >= static_cast<int>(configMAX_PRIORITIES)) {
		mapped = configMAX_PRIORITIES - 1;
	}

	if (mapped < 0) {
		mapped = 0;
	}

	return mapped;
}

#endif /* __PX4_FREERTOS */
