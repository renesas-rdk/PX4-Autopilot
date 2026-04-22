/****************************************************************************
 *
 * Copyright (c) 2025 Renesas Electronics Corporation and/or its affiliates
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 ****************************************************************************/

/**
 * @file posix_shim.cpp
 * @brief POSIX shim implementation for Renesas RZ/V2H
 */

#if defined(__PX4_FREERTOS)

#include "posix_shim.h"

#include <errno.h>
#include <pthread.h>
#include <string.h>
#include <stdint.h>

#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"

#include "bsp_api.h"

extern "C" uint64_t hrt_absolute_time(void);

namespace
{

constexpr uint64_t kMicrosecondsPerSecond = 1000000ULL;

#if (INCLUDE_xTaskGetSchedulerState == 1)
bool scheduler_running()
{
	return xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED;
}
#else
bool scheduler_running()
{
	return true;
}
#endif

#if (INCLUDE_vTaskSuspend == 1)
TickType_t max_delay_chunk()
{
	return (portMAX_DELAY > 0U) ? (portMAX_DELAY - 1U) : portMAX_DELAY;
}
#else
TickType_t max_delay_chunk()
{
	return portMAX_DELAY;
}
#endif

void busy_wait_us(uint32_t usec)
{
	if (usec == 0U) {
		return;
	}

	const uint64_t deadline = hrt_absolute_time() + static_cast<uint64_t>(usec);

	while (hrt_absolute_time() < deadline) {
		// prevent compiler from optimizing away the loop entirely
		__asm volatile("" ::: "memory");
	}
}

uint64_t usec_to_ticks(uint64_t usec)
{
	if (usec == 0U) {
		return 0U;
	}

	const uint64_t numerator = usec * static_cast<uint64_t>(configTICK_RATE_HZ) + (kMicrosecondsPerSecond - 1ULL);
	return numerator / kMicrosecondsPerSecond;
}

void delay_ticks(uint64_t ticks)
{
	if (ticks == 0U) {
		vTaskDelay(1);
		return;
	}

	const TickType_t max_chunk = max_delay_chunk();

	while (ticks > 0U) {
		TickType_t chunk = static_cast<TickType_t>((ticks > static_cast<uint64_t>(max_chunk)) ? max_chunk : ticks);

		if (chunk == 0U) {
			chunk = 1U;
		}

		vTaskDelay(chunk);
		ticks -= static_cast<uint64_t>(chunk);
	}
}

struct px4_pthread_attr_internal_layout {
	uint32_t ulStackSize;
	uint16_t usSchedPriorityDetachState;
};

struct px4_pthread_internal_layout {
	px4_pthread_attr_internal_layout xAttr;
	void *(*pvStartRoutine)(void *);
	void *xTaskArg;
	TaskHandle_t xTaskHandle;
	StaticSemaphore_t xJoinBarrier;
	StaticSemaphore_t xJoinMutex;
	void *xReturn;
	char pcName[configMAX_TASK_NAME_LEN];
};

} // namespace

namespace
{

int px4_usleep_impl(useconds_t usec)
{
	if (usec == 0U) {
		if (scheduler_running()) {
			taskYIELD();
		}

		return 0;
	}

#if (INCLUDE_xTaskGetSchedulerState == 1)
	if (!scheduler_running()) {
		busy_wait_us(static_cast<uint32_t>(usec));
		return 0;
	}
#endif

	const uint64_t ticks = usec_to_ticks(static_cast<uint64_t>(usec));
	delay_ticks(ticks);
	return 0;
}
} // namespace

extern "C" {

// Internal C implementation with non-conflicting name
int __posix_usleep_c_impl(unsigned long usec)
{
	return px4_usleep_impl(static_cast<useconds_t>(usec));
}

// C linkage version for C code (main_task_entry.c, i2c_fsp_backend.c, etc.)
int usleep(unsigned long usec)
{
	return __posix_usleep_c_impl(usec);
}

unsigned sleep(unsigned seconds)
{
	if (seconds == 0U) {
		if (scheduler_running()) {
			taskYIELD();
		}

		return 0;
	}

#if (INCLUDE_xTaskGetSchedulerState == 1)
	if (!scheduler_running()) {
		unsigned remaining = seconds;

		while (remaining > 0U) {
			const unsigned chunk = (remaining > 3600U) ? 3600U : remaining;
			busy_wait_us(static_cast<uint32_t>(chunk) * static_cast<uint32_t>(kMicrosecondsPerSecond));
			remaining -= chunk;
		}

		return 0;
	}
#endif

	const uint64_t ticks = static_cast<uint64_t>(seconds) * static_cast<uint64_t>(configTICK_RATE_HZ);
	delay_ticks(ticks);
	return 0;
}

} // extern "C"

// Note: pthread_setname_np and px4_pthread_setname_np are already defined in FreeRTOS_POSIX_pthread.c
// Removed duplicate definitions to avoid multiple definition errors

// Note: C++ version of usleep is defined in posix_shim_cpp.cpp to avoid
// redefinition conflicts with the C version above

#endif /* __PX4_FREERTOS */
