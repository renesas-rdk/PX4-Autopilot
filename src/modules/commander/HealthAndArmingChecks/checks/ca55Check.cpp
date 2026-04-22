/****************************************************************************
 *
 * Copyright (c) 2025 Renesas Electronics Corporation and/or its affiliates
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 ****************************************************************************/

#ifdef __PX4_FREERTOS
/****************************************************************************
 *
 * Renesas RZ/V2H CA55 readiness arming check.
 *
 * Blocks arming until the CA55 Linux side has booted and the OpenAMP RPC
 * endpoint is bound. This ensures every arming cycle produces a .ulg file
 * in /drone-data/cr8_data/log/.
 *
 * If SDLOG_MODE == -1 (logging disabled), this check is skipped entirely so
 * the drone can arm immediately without waiting for CA55 to boot.
 *
 * Typical CA55 boot time: ~25-30 s after CR8 firmware starts.
 * QGC shows "Preflight Fail: CA55 not ready" until then.
 *
 ****************************************************************************/

#include "ca55Check.hpp"

/* Declared in px4_startup_shim.cpp — true only after deferred_param_sync completes
 * (session_reset + param_save done).  Gating on this rather than the raw
 * px4_openamp_rpc_endpoint_ready() prevents the arming-during-deferred-sync
 * race condition that caused CR8 crashes.                                    */
extern "C" bool rzv_ca55_storage_ready(void);

void CA55ReadinessCheck::checkAndReport(const Context &context, Report &reporter)
{
	/* SDLOG_MODE == -1 means logging is disabled; CA55 storage is not needed */
	if (_param_sdlog_mode.get() == -1) {
		return;
	}

	if (!rzv_ca55_storage_ready()) {
		/* EVENT
		 * @description
		 * CA55 Linux has not finished booting, arming is blocked so the flight logger can write
		 * .ulg files to /drone-data/cr8_data/log/.
		 *
		 * Wait ~30s after power-on for CA55 to complete its boot sequence.
		 */
		reporter.armingCheckFailure(NavModes::All, health_component_t::system,
					    events::ID("check_ca55_not_ready"),
					    events::Log::Error, "External memory not ready, logger unavailable");

		if (context.isArmingRequest() && reporter.mavlink_log_pub()) {
			mavlink_log_critical(reporter.mavlink_log_pub(),
					     "Preflight Fail: External memory not ready");
		}
	}
}
#endif // __PX4_FREERTOS
