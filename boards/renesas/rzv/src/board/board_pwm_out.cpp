/****************************************************************************
 *
 * Copyright (c) 2025 Renesas Electronics Corporation and/or its affiliates
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 ****************************************************************************/

/**
 * @file board_pwm_out.cpp
 * @brief PWM output driver for Renesas RZ/V2H
 */

#if defined(__PX4_FREERTOS)

#include "board_pwm_out.h"

#include <px4_platform_common/log.h>
#include <drivers/drv_hrt.h>

#ifndef MODULE_NAME
#define MODULE_NAME "board_pwm"
#endif

using namespace pwm_out;

/* -------------------------------------------------------------------------
 * Emergency disarm table — populated during init(), used by fault handlers.
 * Stored outside the class so crash handlers can access it without a valid
 * C++ object pointer (which may be corrupted in a stack-overflow scenario).
 * -------------------------------------------------------------------------*/
static struct {
	const timer_instance_t *instance;
	gpt_io_pin_t            pin;
	uint32_t                disarm_counts; /* pre-computed 1000 µs count */
} g_emrg_channels[4] = {};
static int g_emrg_num_channels = 0;

extern "C" void board_emergency_disarm_all_motors(void)
{
	/* Direct FSP duty-cycle writes — no RTOS calls, no mutex.
	 * dutyCycleSet() is a single GTCCR register write and is ISR/fault safe. */
	for (int i = 0; i < g_emrg_num_channels; ++i) {
		if (g_emrg_channels[i].instance && g_emrg_channels[i].disarm_counts > 0U) {
			g_emrg_channels[i].instance->p_api->dutyCycleSet(
				g_emrg_channels[i].instance->p_ctrl,
				g_emrg_channels[i].disarm_counts,
				g_emrg_channels[i].pin);
		}
	}
}

namespace
{

struct TimerBinding {
	const timer_instance_t *instance;
	gpt_io_pin_t pin;
};

constexpr TimerBinding kTimerBindings[] = {
	{&g_timer1, GPT_IO_PIN_GTIOCA},
	{&g_timer2, GPT_IO_PIN_GTIOCB},
	{&g_timer3, GPT_IO_PIN_GTIOCA},
	{&g_timer4, GPT_IO_PIN_GTIOCB},
};

constexpr uint32_t kDefaultPeriodUs = 20000u;
constexpr uint32_t kDefaultMinUs = 900u;
constexpr uint32_t kDefaultMaxUs = 2100u;

} // namespace

RZVPWMOut::RZVPWMOut(int max_outputs)
{
	_num_outputs = (max_outputs <= kMaxChannels) ? max_outputs : kMaxChannels;

	_mutex = xSemaphoreCreateMutexStatic(&_mutex_storage);
}

int RZVPWMOut::init()
{
	for (int i = 0; i < _num_outputs; ++i) {
		const TimerBinding &binding = kTimerBindings[i];
		Channel &ch = _channels[i];

		if (binding.instance == nullptr || binding.instance->p_api == nullptr) {
			PX4_ERR("PWM channel %d timer binding missing", i);
			return PX4_ERROR;
		}

		ch.instance = binding.instance;
		ch.pin = binding.pin;

		fsp_err_t err = ch.instance->p_api->open(ch.instance->p_ctrl, ch.instance->p_cfg);

		if (err != FSP_SUCCESS && err != FSP_ERR_ALREADY_OPEN) {
			PX4_ERR("PWM timer %d open failed (%d)", i, err);
			return PX4_ERROR;
		}

		timer_info_t info{};

		if (ch.instance->p_api->infoGet(ch.instance->p_ctrl, &info) == FSP_SUCCESS && info.clock_frequency > 0U) {
			ch.period_counts = info.period_counts;
			ch.period_us = static_cast<uint32_t>((static_cast<uint64_t>(info.period_counts) * 1000000ULL) / info.clock_frequency);

		} else {
			// Fallback to configuration constants if query fails
			const timer_cfg_t *cfg = static_cast<const timer_cfg_t *>(ch.instance->p_cfg);
			ch.period_counts = cfg ? cfg->period_counts : 0U;
			ch.period_us = kDefaultPeriodUs;
		}

		if (ch.period_counts == 0U || ch.period_us == 0U) {
			PX4_ERR("PWM timer %d invalid period", i);
			return PX4_ERROR;
		}

		ch.min_counts = microseconds_to_counts(kDefaultMinUs, ch);
		ch.max_counts = microseconds_to_counts(kDefaultMaxUs, ch);

		// Initialize with disarmed output (1000 us).  Do NOT start yet — all
		// channels are started simultaneously below via GPTCOM GTSTR writes.
		uint32_t init_counts = microseconds_to_counts(1000U, ch);
		ch.instance->p_api->dutyCycleSet(ch.instance->p_ctrl, init_counts, ch.pin);
		ch.active = true;

		// Populate emergency disarm table (used by fault/crash handlers).
		if (i < 4) {
			g_emrg_channels[i].instance      = ch.instance;
			g_emrg_channels[i].pin           = ch.pin;
			g_emrg_channels[i].disarm_counts = init_counts;
		}
	}
	g_emrg_num_channels = _num_outputs;

	/* Simultaneous start of all PWM channels via GPTCOM GTSTR registers.
	 *
	 * Writing a bitmask to the GPTCOM GTSTR register starts multiple GPT
	 * channels in a single bus cycle, eliminating the phase skew that would
	 * result from calling p_api->start() one channel at a time.
	 *
	 * Hardware GPT groups on RZ/V2H:
	 *   Group A  R_GPT0  (0x13010000): channels 0–7  (FSP ch 0–7)
	 *   Group B  R_GPT10 (0x13020000): channels 10–17 (FSP ch 8–15)
	 *
	 * FSP channel_mask = 1 << ch  (ch < 8)  or  1 << (ch % 8)  (ch >= 8).
	 * Our timers: g_timer1=FSPch6 g_timer2=FSPch7 → Group A mask 0xC0
	 *             g_timer3=FSPch9 g_timer4=FSPch10 → Group B mask 0x06
	 */
	{
		uint32_t mask_a = 0U;
		uint32_t mask_b = 0U;

		for (int i = 0; i < _num_outputs; ++i) {
			if (!_channels[i].active || _channels[i].instance == nullptr) {
				continue;
			}

			const auto *ctrl = static_cast<const gpt_instance_ctrl_t *>(_channels[i].instance->p_ctrl);
			const uint32_t fsp_ch = _channels[i].instance->p_cfg->channel;

			if (fsp_ch < 8U) {
				mask_a |= ctrl->channel_mask;
			} else {
				mask_b |= ctrl->channel_mask;
			}
		}

		if (mask_a != 0U) {
			R_GPT0->GTSTR = mask_a;   /* Start Group A channels simultaneously */
		}

		if (mask_b != 0U) {
			R_GPT10->GTSTR = mask_b;  /* Start Group B channels simultaneously */
		}
	}

	PX4_INFO("RZ/V PWM initialized (%d channels)", _num_outputs);
	return PX4_OK;
}

int RZVPWMOut::send_output_pwm(const uint16_t *pwm, int num_outputs)
{
	if (pwm == nullptr || num_outputs <= 0) {
		return PX4_ERROR;
	}

	/* Serialize access to timer registers.
	 * Block instead of dropping the frame on timeout to avoid output jitter.
	 */
	bool lock_held = false;

	if (_mutex != nullptr) {
		lock_held = (xSemaphoreTake(_mutex, portMAX_DELAY) == pdTRUE);

		if (!lock_held) {
			PX4_ERR("PWM mutex acquisition failed");
			return PX4_ERROR;
		}
	}

	const int limit = (num_outputs < _num_outputs) ? num_outputs : _num_outputs;

	for (int i = 0; i < limit; ++i) {
		Channel &ch = _channels[i];

		if (!ch.active || ch.instance == nullptr) {
			continue;
		}

		uint32_t command = pwm[i];

		if (command < kDefaultMinUs) {
			command = kDefaultMinUs;

		} else if (command > kDefaultMaxUs) {
			command = kDefaultMaxUs;
		}

		uint32_t duty_counts = microseconds_to_counts(command, ch);
		duty_counts = duty_counts > ch.max_counts ? ch.max_counts : duty_counts;
		duty_counts = duty_counts < ch.min_counts ? ch.min_counts : duty_counts;

		ch.instance->p_api->dutyCycleSet(ch.instance->p_ctrl, duty_counts, ch.pin);
	}

	if (lock_held) {
		xSemaphoreGive(_mutex);
	}

	return PX4_OK;
}

uint32_t RZVPWMOut::microseconds_to_counts(uint32_t micros, const Channel &channel)
{
	return static_cast<uint32_t>((static_cast<uint64_t>(micros) * channel.period_counts) / channel.period_us);
}

#endif /* __PX4_FREERTOS */
