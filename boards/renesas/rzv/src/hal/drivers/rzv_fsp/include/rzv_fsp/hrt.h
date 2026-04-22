/****************************************************************************
 *
 * Copyright (c) 2025 Renesas Electronics Corporation and/or its affiliates
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 ****************************************************************************/

#pragma once

#if defined(__PX4_FREERTOS)


#include <visibility.h>
#include <drivers/drv_hrt.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Force GTM0 timer initialization from task context.
 * Must be called before enabling any ISR that calls rzv_hrt_absolute_time()
 * (e.g., the MPU9250 DRDY interrupt). Without this, the first ISR fires
 * before GTM0 is open and hw_time_us() falls back to 1 ms tick resolution,
 * corrupting EKF2 timestamps during early boot.
 */
__EXPORT void rzv_hrt_init(void);

__EXPORT hrt_abstime rzv_hrt_absolute_time(void);

typedef struct {
	uint32_t isr_overflows;
	uint32_t sw_overflows;
	uint32_t skip_isr_hits;
	uint32_t status_failures;
	uint32_t fallback_calls;
	uint32_t monotonic_adjustments;
} rzv_hrt_diagnostics_t;

__EXPORT void rzv_hrt_get_diagnostics(rzv_hrt_diagnostics_t *diag);

/**
 * Print HRT diagnostics to console for debugging.
 * Call periodically (e.g., every 30 seconds) to monitor HRT health.
 * Warning will be logged if fallback_calls > 0 (indicates timing jitter).
 */
__EXPORT void rzv_hrt_print_diagnostics(void);

#ifdef __cplusplus
}

#endif /* __PX4_FREERTOS */

#endif
