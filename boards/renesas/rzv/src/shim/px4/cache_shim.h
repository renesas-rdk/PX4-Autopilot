/****************************************************************************
 *
 * Copyright (c) 2025 Renesas Electronics Corporation and/or its affiliates
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 ****************************************************************************/

/**
 * @file cache_shim.h
 * @brief Cache control shim for Renesas RZ/V2H
 */
#pragma once

#if defined(__PX4_FREERTOS)


#include <stdint.h>

#include "bsp_api.h"
#include "cr/bsp_cache.h"

#ifdef __cplusplus
extern "C" {
#endif

void R_BSP_CacheEnable(void);
void R_BSP_CacheDisable(void);
void R_BSP_CacheInvalidate(void);

#ifdef __cplusplus
}

#endif /* __PX4_FREERTOS */

#endif
