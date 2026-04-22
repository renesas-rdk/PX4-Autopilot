/****************************************************************************
 *
 * Copyright (c) 2025 Renesas Electronics Corporation and/or its affiliates
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 ****************************************************************************/

#ifndef DEBUG_STUB_H
#define DEBUG_STUB_H

#if defined(__PX4_FREERTOS)


#include <stdio.h>

#ifndef APP_PRINT
#define APP_PRINT(...)   do { printf(__VA_ARGS__); } while (0)
#endif

#ifndef LPERROR
#define LPERROR(...)     do { printf(__VA_ARGS__); } while (0)
#endif


#endif /* __PX4_FREERTOS */

#endif /* DEBUG_STUB_H */
