/****************************************************************************
 *
 * Copyright (c) 2025 Renesas Electronics Corporation and/or its affiliates
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 ****************************************************************************/

/**
 * @file condition_variable.h
 *
 * PX4 condition variable abstraction layered on top of FreeRTOS when
 * __PX4_FREERTOS is set, or pthread condition variables elsewhere.
 */

#pragma once

#include <px4_platform_common/defines.h>
#include "mutex.h"

#include <stdbool.h>
#include <time.h>

#if defined(__PX4_FREERTOS)
#include <FreeRTOS.h>
#include <semphr.h>
#else
#include <pthread.h>
#endif

__BEGIN_DECLS

#if defined(__PX4_FREERTOS)

typedef struct {
	StaticSemaphore_t wait_sem_storage;
	StaticSemaphore_t lock_storage;
	SemaphoreHandle_t wait_sem;
	SemaphoreHandle_t lock;
	uint32_t waiters;
	bool initialized;
} px4_cond_t;

#define PX4_COND_INITIALIZER { {}, {}, nullptr, nullptr, 0u, false }

#else

typedef pthread_cond_t px4_cond_t;

#define PX4_COND_INITIALIZER PTHREAD_COND_INITIALIZER

#endif

int px4_cond_init(px4_cond_t *c);
int px4_cond_destroy(px4_cond_t *c);
int px4_cond_signal(px4_cond_t *c);
int px4_cond_broadcast(px4_cond_t *c);
int px4_cond_wait(px4_cond_t *c, px4_mutex_t *m);
int px4_cond_timedwait(px4_cond_t *c, px4_mutex_t *m, const struct timespec *abstime);

__END_DECLS
