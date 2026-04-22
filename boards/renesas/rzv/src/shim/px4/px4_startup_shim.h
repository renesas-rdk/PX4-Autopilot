/****************************************************************************
 *
 * Copyright (c) 2025 Renesas Electronics Corporation and/or its affiliates
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 ****************************************************************************/

/**
 * @file px4_startup_shim.h
 * @brief PX4 Startup shim header for Renesas RZ/V2H
 */
#pragma once

#if defined(__PX4_FREERTOS)


#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

int rzv_px4_bootstrap(void);
int rzv_px4_run_command(const char *command, int silently_fail);
int rzv_px4_run_script(const char *script_path, int silently_fail);
void rzv_px4_enable_param_autosave(int enable);
/**
 * Check SYS_AUTOCONFIG after param load and re-apply factory defaults if > 0.
 * Call once, after the "param load" startup step completes.
 */
int rzv_px4_apply_autoconfig(void);
/**
 * Load saved params from SDRAM region written by U-Boot before CR8 release.
 * Layout: uint32_t size @ 0x41710000, BSON data @ 0x41710008 (8-byte aligned).
 * Returns PX4_OK on success or when no SDRAM params are present (first boot).
 * Sets an internal flag so deferred_param_sync_task skips a redundant reload.
 */
int rzv_px4_load_sdram_params(void);
/**
 * Start the deferred param-sync background task.
 * The task polls for CA55 RPC readiness (every 500 ms, zero CPU cost while
 * waiting) and calls param_load_default() + rzv_px4_apply_autoconfig() once
 * the endpoint becomes available.  No-op when RZV_ENABLE_CA55=OFF.
 */
void rzv_px4_start_deferred_param_sync(void);

#ifdef __cplusplus
}

#endif /* __PX4_FREERTOS */

#endif
