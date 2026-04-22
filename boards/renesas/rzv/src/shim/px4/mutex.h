/****************************************************************************
 *
 * Copyright (c) 2025 Renesas Electronics Corporation and/or its affiliates
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 ****************************************************************************/

/**
 * @file mutex.h
 *
 * PX4 mutex abstraction that provides a FreeRTOS-backed implementation when
 * running on __PX4_FREERTOS, while defaulting to pthread mutexes on POSIX.
 */

#pragma once

#include <px4_platform_common/defines.h>

#include <stdbool.h>

#if defined(__PX4_FREERTOS)
#include <FreeRTOS.h>
#include <semphr.h>
#else
#include <pthread.h>
#endif

__BEGIN_DECLS

#if defined(__PX4_FREERTOS)

typedef struct {
	StaticSemaphore_t storage;
	SemaphoreHandle_t handle;
	bool recursive;
} px4_mutex_t;

#define PX4_MUTEX_INITIALIZER { {}, nullptr, false }

#else

typedef pthread_mutex_t px4_mutex_t;

#define PX4_MUTEX_INITIALIZER PTHREAD_MUTEX_INITIALIZER

#endif

int px4_mutex_init(px4_mutex_t *m);
int px4_mutex_init_recursive(px4_mutex_t *m);
int px4_mutex_lock(px4_mutex_t *m);
int px4_mutex_trylock(px4_mutex_t *m);
int px4_mutex_unlock(px4_mutex_t *m);
int px4_mutex_destroy(px4_mutex_t *m);

__END_DECLS
