/****************************************************************************
 *
 * Copyright (c) 2025 Renesas Electronics Corporation and/or its affiliates
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 ****************************************************************************/

#ifndef SYS_STUB_H
#define SYS_STUB_H

#if defined(__PX4_FREERTOS)


#include <stdint.h>
#include <stddef.h>

typedef enum {
    SYS_MEM_GROUP_FATFS = 0,
    SYS_MEM_GROUP_FILEBUF = 1,
} sys_mem_group_t;

/* Minimal memory helpers used by storage */
void *sys_mem_alloc(sys_mem_group_t group, size_t size);
void  sys_mem_free(sys_mem_group_t group, void *ptr);
void *sys_mem_free_null(sys_mem_group_t group, void *ptr);

/* Minimal time helpers */
uint32_t sys_time_get(void);
char *sys_time_iso8601_get(void);

/* Simple timer helper (millisecond deadline) */
typedef struct {
    uint32_t deadline_ms;
} sys_timer_t;

void sys_timer_start(sys_timer_t *t, uint32_t timeout_ms);
int  sys_timer_reached(const sys_timer_t *t);


#endif /* __PX4_FREERTOS */

#endif /* SYS_STUB_H */
