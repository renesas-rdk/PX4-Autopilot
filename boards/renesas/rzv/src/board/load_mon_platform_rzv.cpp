/****************************************************************************
 *
 * Copyright (c) 2025 Renesas Electronics Corporation and/or its affiliates
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 ****************************************************************************/


#if defined(__PX4_FREERTOS)

#include <climits>
#include <vector>

#include <px4_platform_common/load_mon_platform.h>

extern "C" {
#include <FreeRTOS.h>
#include <task.h>
}

/**
 * @brief FreeRTOS CPU/RAM load monitor for Renesas RZ/V2H (Cortex-R8).
 *
 * Design note — why NOT use the 3rd parameter (pulTotalRunTime) of
 * uxTaskGetSystemState():
 *   On this AWS FreeRTOS port the 3rd parameter is always 0, even though
 *   per-task ulRunTimeCounter values are correctly accumulated by the
 *   scheduler (verified: `top once` shows valid CPU% by using them).
 *   Therefore we compute the "total" ourselves as the sum of every task's
 *   ulRunTimeCounter — identical to the approach used by print_load.cpp.
 *
 * State reuse:
 *   last_total_runtime_counter  → sum of ALL task counters from previous call
 *   last_idle_runtime_counter   → idle-task counter from previous call
 */
extern "C" __EXPORT bool px4_load_mon_platform_cpuload(px4_load_mon_platform_state_t *state, cpuload_s *cpuload)
{
	const UBaseType_t reported_tasks = uxTaskGetNumberOfTasks();

	if (reported_tasks == 0) {
		return false;
	}

	// Extra slots guard against tasks created between uxTaskGetNumberOfTasks()
	// and uxTaskGetSystemState() causing the function to return 0 (buffer too small).
	const UBaseType_t buffer_size = reported_tasks + 16;
	std::vector<TaskStatus_t> task_status(buffer_size);
	// Pass a dummy variable instead of nullptr: some older FreeRTOS versions do not
	// NULL-check pulTotalRunTime before writing to it.  The value is intentionally
	// discarded — we compute the total from per-task counters below.
	configRUN_TIME_COUNTER_TYPE unused_total = 0;
	UBaseType_t populated = uxTaskGetSystemState(task_status.data(), buffer_size, &unused_total);

	if (populated == 0) {
		return false;
	}

	task_status.resize(populated);

	// Sum per-task ulRunTimeCounter values.  These are reliably non-zero on this
	// platform (confirmed by `top`).  The sum is used as the "total elapsed runtime"
	// reference instead of the broken 3rd-param total_runtime_counter.
	uint32_t total_counter = 0;
	uint32_t idle_counter  = 0;
	const TaskHandle_t idle_handle = xTaskGetIdleTaskHandle();

	for (const TaskStatus_t &ts : task_status) {
		total_counter += static_cast<uint32_t>(ts.ulRunTimeCounter);

		if (ts.xHandle == idle_handle) {
			idle_counter = static_cast<uint32_t>(ts.ulRunTimeCounter);
		}
	}

	// First-call guard: capture baseline and skip publishing this cycle.
	// Sentinel: last_total_runtime_counter == 0  (zero-initialised by LoadMon.hpp).
	// Only advance past the guard when we have valid counter data (total > 0).
	if (state->last_total_runtime_counter == 0) {
		if (total_counter > 0) {
			state->last_total_runtime_counter = total_counter;
			state->last_idle_runtime_counter  = idle_counter;
		}

		return false;
	}

	// Unsigned-wrapping-safe delta (both total and idle wrap at UINT32_MAX).
	const uint32_t total_delta = (total_counter >= state->last_total_runtime_counter)
				     ? (total_counter - state->last_total_runtime_counter)
				     : (UINT32_MAX - state->last_total_runtime_counter + total_counter + 1U);

	const uint32_t idle_delta = (idle_counter >= state->last_idle_runtime_counter)
				    ? (idle_counter - state->last_idle_runtime_counter)
				    : (UINT32_MAX - state->last_idle_runtime_counter + idle_counter + 1U);

	state->last_total_runtime_counter = total_counter;
	state->last_idle_runtime_counter  = idle_counter;

	if (total_delta == 0) {
		return false;
	}

#if defined(configTOTAL_HEAP_SIZE)
	const size_t free_heap  = xPortGetFreeHeapSize();
	const float  total_heap = static_cast<float>(configTOTAL_HEAP_SIZE);
	cpuload->ram_usage = (total_heap > 0.f)
			     ? 1.f - (static_cast<float>(free_heap) / total_heap)
			     : -1.f;
#else
	cpuload->ram_usage = -1.f;
#endif

	cpuload->load = 1.f - (static_cast<float>(idle_delta) / static_cast<float>(total_delta));

	return true;
}

#endif /* __PX4_FREERTOS */
