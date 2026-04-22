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
 * On __PX4_FREERTOS builds: blocks arming until CA55 Linux is booted and
 * the OpenAMP RPC endpoint is ready (logger can write .ulg files).
 * On all other platforms: no-op (check always passes).
 *
 ****************************************************************************/

#pragma once

#include "../Common.hpp"

class CA55ReadinessCheck : public HealthAndArmingCheckBase
{
public:
	CA55ReadinessCheck() = default;
	~CA55ReadinessCheck() override = default;

	void checkAndReport(const Context &context, Report &reporter) override;

private:
	DEFINE_PARAMETERS_CUSTOM_PARENT(HealthAndArmingCheckBase,
					(ParamInt<px4::params::SDLOG_MODE>) _param_sdlog_mode
				       )
};
#endif // __PX4_FREERTOS
