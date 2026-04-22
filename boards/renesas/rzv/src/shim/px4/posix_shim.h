/****************************************************************************
 *
 * Copyright (c) 2025 Renesas Electronics Corporation and/or its affiliates
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 ****************************************************************************/

/**
 * @file posix_shim.h
 * @brief POSIX shim header for Renesas RZ/V2H
 */
#pragma once

#if defined(__PX4_FREERTOS)


#include <sys/types.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Note: usleep() is defined in posix_shim.cpp with both C and C++ linkage
// to support both C code (main_task_entry.c) and C++ code (PX4 library)
// int usleep(unsigned long usec); // Declared but causes redefinition issues

unsigned sleep(unsigned seconds);

#ifdef __cplusplus
}

#endif /* __PX4_FREERTOS */

#endif
