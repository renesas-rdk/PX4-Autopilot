/****************************************************************************
 *
 * Copyright (c) 2025 Renesas Electronics Corporation and/or its affiliates
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 ****************************************************************************/

/**
 * @file px4_mutex_freertos.cpp
 * @brief PX4 Mutex implementation for FreeRTOS on Renesas RZ/V2H
 */
#include "mutex.h"

#if !defined(__PX4_FREERTOS)
#error "px4_mutex_freertos.cpp must only be built for FreeRTOS targets"
#endif

#include <errno.h>

static int px4_mutex_lazy_init(px4_mutex_t *m, bool recursive)
{
	if (m == nullptr) {
		errno = EINVAL;
		return -1;
	}

	if (m->handle != nullptr) {
		return 0;
	}

	taskENTER_CRITICAL();

	if (m->handle == nullptr) {
		SemaphoreHandle_t handle = recursive ?
					   xSemaphoreCreateRecursiveMutexStatic(&m->storage) :
					   xSemaphoreCreateMutexStatic(&m->storage);

		if (handle == nullptr) {
			taskEXIT_CRITICAL();
			errno = ENOMEM;
			return -1;
		}

		m->handle = handle;
		m->recursive = recursive;
	}

	taskEXIT_CRITICAL();

	return 0;
}

static inline BaseType_t px4_mutex_take(px4_mutex_t *m, TickType_t ticks_to_wait)
{
	if (px4_mutex_lazy_init(m, m->recursive ? true : false) != 0) {
		return pdFAIL;
	}

	if (m->recursive) {
		return xSemaphoreTakeRecursive(m->handle, ticks_to_wait);
	}

	return xSemaphoreTake(m->handle, ticks_to_wait);
}

static inline BaseType_t px4_mutex_give(px4_mutex_t *m)
{
	if ((m == nullptr) || (m->handle == nullptr)) {
		errno = EINVAL;
		return pdFAIL;
	}

	if (m->recursive) {
		return xSemaphoreGiveRecursive(m->handle);
	}

	return xSemaphoreGive(m->handle);
}

int px4_mutex_init(px4_mutex_t *m)
{
	if (m == nullptr) {
		errno = EINVAL;
		return -1;
	}

	m->handle = nullptr;
	m->recursive = false;
	return px4_mutex_lazy_init(m, false);
}

int px4_mutex_init_recursive(px4_mutex_t *m)
{
	if (m == nullptr) {
		errno = EINVAL;
		return -1;
	}

	m->handle = nullptr;
	m->recursive = true;
	return px4_mutex_lazy_init(m, true);
}

int px4_mutex_lock(px4_mutex_t *m)
{
	return (px4_mutex_take(m, portMAX_DELAY) == pdPASS) ? 0 : -1;
}

int px4_mutex_trylock(px4_mutex_t *m)
{
	if (px4_mutex_lazy_init(m, m->recursive ? true : false) != 0) {
		return -1;
	}

	if (px4_mutex_take(m, 0) == pdPASS) {
		return 0;
	}

	errno = EBUSY;
	return -1;
}

int px4_mutex_unlock(px4_mutex_t *m)
{
	return (px4_mutex_give(m) == pdPASS) ? 0 : -1;
}

int px4_mutex_destroy(px4_mutex_t *m)
{
	if (m == nullptr) {
		errno = EINVAL;
		return -1;
	}

	if (m->handle != nullptr) {
		vSemaphoreDelete(m->handle);
		m->handle = nullptr;
	}

	m->recursive = false;
	return 0;
}
