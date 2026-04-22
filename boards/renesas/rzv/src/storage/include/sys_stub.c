/****************************************************************************
 *
 * Copyright (c) 2025 Renesas Electronics Corporation and/or its affiliates
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 ****************************************************************************/


#if defined(__PX4_FREERTOS)

#include "sys.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stdio.h>
#include <string.h>

void *sys_mem_alloc(sys_mem_group_t group, size_t size)
{
    (void) group;
    return pvPortMalloc(size);
}

void sys_mem_free(sys_mem_group_t group, void *ptr)
{
    (void) group;
    if (ptr) vPortFree(ptr);
}

void *sys_mem_free_null(sys_mem_group_t group, void *ptr)
{
    sys_mem_free(group, ptr);
    return NULL;
}

uint32_t sys_time_get(void)
{
    return (uint32_t) (xTaskGetTickCount() * portTICK_PERIOD_MS);
}

char *sys_time_iso8601_get(void)
{
    static char buf[32];
    /* No RTC available: synthesize from uptime milliseconds */
    uint32_t ms = sys_time_get();
    uint32_t s = ms / 1000U;
    uint32_t msec = ms % 1000U;
    uint32_t min = s / 60U;
    uint32_t sec = s % 60U;
    uint32_t hr  = min / 60U;
    min %= 60U;
    /* Day/hour rollover ignored; format as 0000-00-00THH:MM:SS.mmmZ */
    snprintf(buf, sizeof(buf), "0000-00-00T%02lu:%02lu:%02lu.%03luZ",
             (unsigned long)hr, (unsigned long)min, (unsigned long)sec, (unsigned long)msec);
    return buf;
}

void sys_timer_start(sys_timer_t *t, uint32_t timeout_ms)
{
    if (!t) return;
    t->deadline_ms = sys_time_get() + timeout_ms;
}

int sys_timer_reached(const sys_timer_t *t)
{
    if (!t) return 1;
    uint32_t now = sys_time_get();
    /* Handle wrap-around: (now - deadline) >= 0 is reached */
    return (int)((int32_t)(now - t->deadline_ms) >= 0);
}

#endif /* __PX4_FREERTOS */
