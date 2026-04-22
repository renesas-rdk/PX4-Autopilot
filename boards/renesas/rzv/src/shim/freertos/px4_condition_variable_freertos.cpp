/****************************************************************************
 *
 * Copyright (c) 2025 Renesas Electronics Corporation and/or its affiliates
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 ****************************************************************************/

/**
 * @file px4_condition_variable_freertos.cpp
 * @brief PX4 Condition Variable implementation for FreeRTOS on Renesas RZ/V2H
 */
#include "condition_variable.h"

#if !defined(__PX4_FREERTOS)
#error "px4_condition_variable_freertos.cpp must only be built for FreeRTOS targets"
#endif

#include <errno.h>

#include "freertos_time_utils.hpp"

static constexpr UBaseType_t kDefaultCondMaxCount{0x00FFFFFFu};

static int px4_cond_lazy_init(px4_cond_t *c)
{
	if (c == nullptr) {
		errno = EINVAL;
		return -1;
	}

	if (c->initialized) {
		return 0;
	}

	taskENTER_CRITICAL();

	if (!c->initialized) {
		c->wait_sem = xSemaphoreCreateCountingStatic(kDefaultCondMaxCount, 0, &c->wait_sem_storage);
		c->lock = xSemaphoreCreateMutexStatic(&c->lock_storage);
		c->waiters = 0;

		if ((c->wait_sem == nullptr) || (c->lock == nullptr)) {
			if (c->wait_sem != nullptr) {
				vSemaphoreDelete(c->wait_sem);
				c->wait_sem = nullptr;
			}

			if (c->lock != nullptr) {
				vSemaphoreDelete(c->lock);
				c->lock = nullptr;
			}

			taskEXIT_CRITICAL();
			errno = ENOMEM;
			return -1;
		}

		c->initialized = true;
	}

	taskEXIT_CRITICAL();

	return 0;
}

static inline void cond_inc_waiters(px4_cond_t *c)
{
	xSemaphoreTake(c->lock, portMAX_DELAY);
	++c->waiters;
	xSemaphoreGive(c->lock);
}

static inline void cond_dec_waiters(px4_cond_t *c)
{
	xSemaphoreTake(c->lock, portMAX_DELAY);

	if (c->waiters > 0) {
		--c->waiters;
	}

	xSemaphoreGive(c->lock);
}

int px4_cond_init(px4_cond_t *c)
{
	if (c == nullptr) {
		errno = EINVAL;
		return -1;
	}

	c->wait_sem = nullptr;
	c->lock = nullptr;
	c->waiters = 0;
	c->initialized = false;
	return px4_cond_lazy_init(c);
}

int px4_cond_destroy(px4_cond_t *c)
{
	if (c == nullptr) {
		errno = EINVAL;
		return -1;
	}

	if (c->wait_sem != nullptr) {
		vSemaphoreDelete(c->wait_sem);
		c->wait_sem = nullptr;
	}

	if (c->lock != nullptr) {
		vSemaphoreDelete(c->lock);
		c->lock = nullptr;
	}

	c->waiters = 0;
	c->initialized = false;
	return 0;
}

int px4_cond_signal(px4_cond_t *c)
{
	if (px4_cond_lazy_init(c) != 0) {
		return -1;
	}

	xSemaphoreTake(c->lock, portMAX_DELAY);

	if (c->waiters > 0) {
		/* Only give semaphore, don't decrement waiters here.
		 * The waiter will decrement when it wakes up.
		 * This avoids race condition where signal is "stolen" by wrong waiter.
		 */
		xSemaphoreGive(c->wait_sem);
	}

	xSemaphoreGive(c->lock);
	return 0;
}

int px4_cond_broadcast(px4_cond_t *c)
{
	if (px4_cond_lazy_init(c) != 0) {
		return -1;
	}

	xSemaphoreTake(c->lock, portMAX_DELAY);
	uint32_t to_release = c->waiters;
	/* Don't clear waiters here - each waiter will decrement when it wakes */
	xSemaphoreGive(c->lock);

	while (to_release--) {
		xSemaphoreGive(c->wait_sem);
	}

	return 0;
}

int px4_cond_wait(px4_cond_t *c, px4_mutex_t *m)
{
	return px4_cond_timedwait(c, m, nullptr);
}

int px4_cond_timedwait(px4_cond_t *c, px4_mutex_t *m, const struct timespec *abstime)
{
	if (px4_cond_lazy_init(c) != 0) {
		return -1;
	}

	/* Hold internal lock while incrementing waiters AND releasing user mutex.
	 * This minimizes the race window where a signal could be "stolen" by
	 * another waiter that incremented after us but took the semaphore first.
	 */
	xSemaphoreTake(c->lock, portMAX_DELAY);
	++c->waiters;

	/* Release user mutex while still holding internal lock */
	if (px4_mutex_unlock(m) != 0) {
		--c->waiters;
		xSemaphoreGive(c->lock);
		return -1;
	}

	/* Now release internal lock and prepare to wait */
	xSemaphoreGive(c->lock);

	TickType_t ticks = rzv::freertos::abstime_to_ticks(abstime);

	if (ticks == 0) {
		ticks = 1;
	}

	BaseType_t take_result = xSemaphoreTake(c->wait_sem, ticks);

	/* Absorb a signal that arrived in the race window between timeout and
	 * cond_dec_waiters().  Without this, the semaphore count would be left at
	 * 1 while waiters drops to 0, causing the NEXT caller to get a spurious
	 * wakeup.  Taking it here converts a race-window timeout into a success,
	 * which is explicitly permitted by POSIX (spurious wakeup rule).
	 * We do NOT hold c->lock while taking wait_sem — that is intentional and
	 * safe because:
	 *   - signal() only gives wait_sem when waiters > 0 (checked under lock)
	 *   - we are still counted in waiters until cond_dec_waiters() below
	 *   - a zero-timeout take cannot block, so no priority inversion risk
	 */
	if ((take_result != pdPASS) && (ticks != portMAX_DELAY)) {
		take_result = xSemaphoreTake(c->wait_sem, 0);
	}

	/* Always decrement waiters when waking up, regardless of success/timeout.
	 * This matches the protocol where signal() does not decrement.
	 */
	cond_dec_waiters(c);

	if (px4_mutex_lock(m) != 0) {
		return -1;
	}

	if (take_result != pdPASS) {
		errno = ETIMEDOUT;
		return -1;
	}

	return 0;
}
