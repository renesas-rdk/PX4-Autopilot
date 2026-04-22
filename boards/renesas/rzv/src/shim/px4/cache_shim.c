/****************************************************************************
 *
 * Copyright (c) 2025 Renesas Electronics Corporation and/or its affiliates
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 ****************************************************************************/

/**
 * @file cache_shim.c
 * @brief Cache control shim for Renesas RZ/V2H
 */

#if defined(__PX4_FREERTOS)

#include "cache_shim.h"

void R_BSP_CacheEnable(void)
{
    R_BSP_CacheEnablePrediction();
    R_BSP_CacheEnableInst();
    R_BSP_CacheEnableData();
}

void R_BSP_CacheDisable(void)
{
    R_BSP_CacheDisableInst();
    R_BSP_CacheDisableData();
}

void R_BSP_CacheInvalidate(void)
{
    R_BSP_CacheInvalidateAllInst();
    R_BSP_CacheInvalidateAllData();
}

#endif /* __PX4_FREERTOS */
