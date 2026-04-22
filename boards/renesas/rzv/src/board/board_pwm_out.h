/****************************************************************************
 *
 * Copyright (c) 2025 Renesas Electronics Corporation and/or its affiliates
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 ****************************************************************************/

/**
 * @file board_pwm_out.h
 * @brief PWM output driver header for Renesas RZ/V2H
 */
#pragma once

#if defined(__PX4_FREERTOS)


#include <px4_platform/pwm_out_base.h>
#include "hal_data.h"
#include <FreeRTOS.h>
#include <semphr.h>

#define BOARD_PWM_OUT_IMPL RZVPWMOut

namespace pwm_out
{

class RZVPWMOut : public PWMOutBase
{
public:
	explicit RZVPWMOut(int max_outputs);
	~RZVPWMOut() override = default;

	int init() override;
	int send_output_pwm(const uint16_t *pwm, int num_outputs) override;
	void set_armed(bool armed) override { _armed = armed; }

private:
	struct Channel {
		const timer_instance_t *instance{nullptr};
		uint32_t period_counts{0};
		uint32_t period_us{20000};
		uint32_t min_counts{0};
		uint32_t max_counts{0};
		gpt_io_pin_t pin{GPT_IO_PIN_GTIOCA};
		bool active{false};
	};

	static constexpr int kMaxChannels = 4;
	Channel _channels[kMaxChannels]{};
	int _num_outputs{0};
	bool _armed{false};

	SemaphoreHandle_t _mutex{nullptr};
	StaticSemaphore_t _mutex_storage{};

	static uint32_t microseconds_to_counts(uint32_t micros, const Channel &channel);
};

} // namespace pwm_out

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Emergency PWM disarm — callable from fault/crash handlers (no RTOS, no mutex).
 * Sets all PWM channels to 1000 µs (ESC disarmed value) via direct FSP API.
 * Safe to call from vApplicationStackOverflowHook, vApplicationMallocFailedHook,
 * or any HardFault handler.
 */
void board_emergency_disarm_all_motors(void);

#ifdef __cplusplus
}

#endif /* __PX4_FREERTOS */

#endif
