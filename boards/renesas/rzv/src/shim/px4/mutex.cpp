/****************************************************************************
 *
 * Copyright (c) 2025 Renesas Electronics Corporation and/or its affiliates
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 ****************************************************************************/

/**
 * @file mutex.cpp
 * @brief PX4 Mutex implementation for FreeRTOS on Renesas RZ/V2H
 */
#include "mutex.h"

#include <errno.h>

#if !defined(__PX4_FREERTOS)

int px4_mutex_init(px4_mutex_t *m)
{
	return pthread_mutex_init(m, nullptr);
}

int px4_mutex_init_recursive(px4_mutex_t *m)
{
	pthread_mutexattr_t attr;
	int ret = pthread_mutexattr_init(&attr);

	if (ret != 0) {
		return ret;
	}

	ret = pthread_mutexattr_settype(&attr, PTHREAD_MUTEX_RECURSIVE);

	if (ret == 0) {
		ret = pthread_mutex_init(m, &attr);
	}

	pthread_mutexattr_destroy(&attr);
	return ret;
}

int px4_mutex_lock(px4_mutex_t *m)
{
	return pthread_mutex_lock(m);
}

int px4_mutex_trylock(px4_mutex_t *m)
{
	return pthread_mutex_trylock(m);
}

int px4_mutex_unlock(px4_mutex_t *m)
{
	return pthread_mutex_unlock(m);
}

int px4_mutex_destroy(px4_mutex_t *m)
{
	return pthread_mutex_destroy(m);
}

#endif // !__PX4_FREERTOS
