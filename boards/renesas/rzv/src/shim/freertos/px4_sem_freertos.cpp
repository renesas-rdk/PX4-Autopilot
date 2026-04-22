/****************************************************************************
 *
 * Copyright (c) 2025 Renesas Electronics Corporation and/or its affiliates
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 ****************************************************************************/

/**
 * @file px4_sem_freertos.cpp
 * @brief PX4 Semaphore implementation for FreeRTOS on Renesas RZ/V2H
 */

#if defined(__PX4_FREERTOS)

#include <px4_platform_common/defines.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/time.h>
#include <px4_platform_common/workqueue.h>
#include <px4_platform_common/sem.h>

#include <errno.h>
#include <time.h>

#include <FreeRTOS.h>
#include <drivers/drv_hrt.h>
#include <semphr.h>

#include "freertos_time_utils.hpp"
static constexpr UBaseType_t kDefaultMaxCount{0x00FFFFFFu};

bool px4_in_isr()
{
#if defined(__arm__) && !defined(__aarch64__)
#if defined(__ARM_ARCH_6M__) || defined(__ARM_ARCH_7M__) || defined(__ARM_ARCH_7EM__) || defined(__ARM_ARCH_8M_BASE__) || defined(__ARM_ARCH_8M_MAIN__)
	/* Cortex-M: Use IPSR to detect exception context */
	uint32_t ipsr = 0;
	__asm volatile("MRS %0, IPSR" : "=r"(ipsr)::"memory");
	return (ipsr != 0U);
#else
	/* Cortex-R8 (CR8) and other ARM32: Check CPSR mode bits
	 * ARM processor modes:
	 *   0x10 - User mode       (task context)
	 *   0x11 - FIQ mode        (fast interrupt - ISR)
	 *   0x12 - IRQ mode        (normal interrupt - ISR)
	 *   0x13 - SVC mode        (supervisor call - exception)
	 *   0x17 - Abort mode      (data/prefetch abort - exception)
	 *   0x1B - Undefined mode  (undefined instruction - exception)
	 *   0x1F - System mode     (privileged task context)
	 *
	 * For FreeRTOS ISR-safe API: Only User (0x10) and System (0x1F)
	 * are task context. All other modes are exception/interrupt context.
	 */
	uint32_t cpsr = 0;
	__asm volatile("MRS %0, CPSR" : "=r"(cpsr)::"memory");
	const uint32_t mode = cpsr & 0x1FU;

	/* Return true if NOT in User or System mode (i.e., in exception context) */
	return (mode != 0x10U) && (mode != 0x1FU);
#endif
#else
	return false;
#endif
}

int px4_sem_init(px4_sem_t *s, int pshared, unsigned value)
{
	(void)pshared;

	if (s == nullptr) {
		errno = EINVAL;
		return -1;
	}

	UBaseType_t max_count = kDefaultMaxCount;

	if (value > max_count) {
		max_count = value;
	}

	s->handle = xSemaphoreCreateCountingStatic(max_count, value, &s->storage);
	s->max_count = max_count;

	if (s->handle == nullptr) {
		errno = ENOMEM;
		return -1;
	}

	return 0;
}

int px4_sem_setprotocol(px4_sem_t *, int)
{
	return 0;
}

int px4_sem_wait(px4_sem_t *s)
{
	if ((s == nullptr) || (s->handle == nullptr)) {
		errno = EINVAL;
		return -1;
	}

	while (xSemaphoreTake(s->handle, portMAX_DELAY) != pdPASS) {
		/* Retry until successful. */
	}

	return 0;
}

int px4_sem_trywait(px4_sem_t *s)
{
	if ((s == nullptr) || (s->handle == nullptr)) {
		errno = EINVAL;
		return -1;
	}

	if (xSemaphoreTake(s->handle, 0) == pdPASS) {
		return 0;
	}

	errno = EAGAIN;
	return -1;
}

int px4_sem_timedwait(px4_sem_t *s, const struct timespec *abstime)
{
	if ((s == nullptr) || (s->handle == nullptr)) {
		errno = EINVAL;
		return -1;
	}

	if (abstime == nullptr) {
		return px4_sem_wait(s);
	}

	TickType_t wait_ticks = rzv::freertos::abstime_to_ticks(abstime);

	if (wait_ticks == 0) {
		wait_ticks = 1;
	}

	if (xSemaphoreTake(s->handle, wait_ticks) == pdPASS) {
		return 0;
	}

	errno = ETIMEDOUT;
	return -1;
}

int px4_sem_post(px4_sem_t *s)
{
	if ((s == nullptr) || (s->handle == nullptr)) {
		errno = EINVAL;
		return -1;
	}

	BaseType_t ret;

	if (px4_in_isr()) {
		BaseType_t higher_priority_woken = pdFALSE;
		ret = xSemaphoreGiveFromISR(s->handle, &higher_priority_woken);

		if ((ret == pdPASS) && (higher_priority_woken == pdTRUE)) {
			portYIELD_FROM_ISR(higher_priority_woken);
		}

	} else {
		if ((s->max_count > 0U) && (uxSemaphoreGetCount(s->handle) >= s->max_count)) {
			errno = EOVERFLOW;
			return -1;
		}

		ret = xSemaphoreGive(s->handle);
	}

	if (ret != pdPASS) {
		errno = EAGAIN;
		return -1;
	}

	return 0;
}

int px4_sem_getvalue(px4_sem_t *s, int *sval)
{
	if ((s == nullptr) || (s->handle == nullptr) || (sval == nullptr)) {
		errno = EINVAL;
		return -1;
	}

	*sval = static_cast<int>(uxSemaphoreGetCount(s->handle));
	return 0;
}

int px4_sem_destroy(px4_sem_t *s)
{
	if ((s == nullptr) || (s->handle == nullptr)) {
		return 0;
	}

	vSemaphoreDelete(s->handle);
	s->handle = nullptr;
	s->max_count = 0;
	return 0;
}

#endif /* __PX4_FREERTOS */
