/****************************************************************************
 *
 * Copyright (c) 2025 Renesas Electronics Corporation and/or its affiliates
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 ****************************************************************************/

/**
 * @file condition_variable.cpp
 * @brief PX4 Condition Variable implementation for FreeRTOS on Renesas RZ/V2H
 */
#include "condition_variable.h"

#if !defined(__PX4_FREERTOS)

int px4_cond_init(px4_cond_t *c)
{
	return pthread_cond_init(c, nullptr);
}

int px4_cond_destroy(px4_cond_t *c)
{
	return pthread_cond_destroy(c);
}

int px4_cond_signal(px4_cond_t *c)
{
	return pthread_cond_signal(c);
}

int px4_cond_broadcast(px4_cond_t *c)
{
	return pthread_cond_broadcast(c);
}

int px4_cond_wait(px4_cond_t *c, px4_mutex_t *m)
{
	return pthread_cond_wait(c, m);
}

int px4_cond_timedwait(px4_cond_t *c, px4_mutex_t *m, const struct timespec *abstime)
{
	return pthread_cond_timedwait(c, m, abstime);
}

#endif // !__PX4_FREERTOS
