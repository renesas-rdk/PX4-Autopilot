/****************************************************************************
 *
 * Copyright (c) 2025 Renesas Electronics Corporation and/or its affiliates
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 ****************************************************************************/


#if defined(__PX4_FREERTOS)

#define MODULE_NAME "rzv_hrt"

#include "rzv_fsp/hrt.h"

#include <px4_platform_common/log.h>
#include <px4_platform_common/time.h>

#include <FreeRTOS.h>
#include <task.h>

#include <rzv_gen/hal_data.h>
#include <inttypes.h>
#include <type_traits>

#define USEC_PER_SEC    (1000000ULL)

static volatile uint32_t s_status_failures{0};
static volatile uint32_t s_fallback_calls{0};
static volatile uint32_t s_monotonic_adjustments{0};
static volatile uint32_t s_wrap_events{0};
static volatile bool     s_logged_init{false};
static volatile bool     s_logged_fail_open{false};
static volatile bool     s_logged_status_error{false};
static volatile bool     s_logged_fallback{false};
static volatile bool     s_logged_timer_stall{false};

namespace
{

static constexpr uint32_t RZV_HRT_STALL_THRESHOLD = 1024U;

static inline hrt_abstime fallback_time_us(bool in_isr)
{
	const TickType_t ticks = in_isr ? xTaskGetTickCountFromISR() : xTaskGetTickCount();
	return static_cast<hrt_abstime>(ticks) * (USEC_PER_SEC / configTICK_RATE_HZ);
}

static inline uint32_t read_cpsr()
{
	uint32_t cpsr;
	__asm volatile("mrs %0, CPSR" : "=r"(cpsr));
	return cpsr;
}

static inline bool is_exception_mode(uint32_t cpsr)
{
	const uint32_t mode = cpsr & 0x1FU;
	return (mode != 0x1FU) && (mode != 0x10U);
}

static inline uint32_t enter_critical_raw()
{
	uint32_t cpsr;
	__asm volatile("mrs %0, CPSR" : "=r"(cpsr));
	__asm volatile("cpsid if");
	return cpsr;
}

static inline void exit_critical_raw(uint32_t cpsr)
{
	if ((cpsr & (1U << 7)) == 0U) {
		__asm volatile("cpsie i");
	}

	if ((cpsr & (1U << 6)) == 0U) {
		__asm volatile("cpsie f");
	}
}

static hrt_abstime ensure_monotonic(hrt_abstime candidate)
{
	static hrt_abstime last_time = 0;
	const uint32_t critical = enter_critical_raw();

	if (candidate <= last_time) {
		candidate = last_time + 1;
		__atomic_fetch_add(&s_monotonic_adjustments, 1U, __ATOMIC_RELAXED);
	}

	last_time = candidate;
	exit_critical_raw(critical);
	return candidate;
}

#if defined(__SIZEOF_INT128__)
static inline hrt_abstime counts_to_time_us(uint64_t counts, uint32_t frequency_hz)
{
	if (frequency_hz == 0U) {
		return 0;
	}

	const __uint128_t scaled = static_cast<__uint128_t>(counts) * USEC_PER_SEC
				   + static_cast<__uint128_t>(frequency_hz) / 2U;
	return static_cast<hrt_abstime>(scaled / frequency_hz);
}
#else
static inline hrt_abstime counts_to_time_us(uint64_t counts, uint32_t frequency_hz)
{
	if (frequency_hz == 0U) {
		return 0;
	}

	const uint64_t seconds = counts / frequency_hz;
	const uint64_t remainder = counts % frequency_hz;
	const uint64_t rounded = (remainder * USEC_PER_SEC + (frequency_hz / 2U)) / frequency_hz;
	return seconds * USEC_PER_SEC + rounded;
}
#endif

/* ordered list of HRT timer candidates — GTM0 (P1CLK) preferred,
 * GTM3 (P5CLK, channel 3) used as backup when GTM0 open fails.
 * Both run at 100 MHz with period_counts=0x1869F (1 ms interval). */
static const timer_instance_t *const kHrtTimerCandidates[] = {&gtm0, &gtm3};

static hrt_abstime hw_time_us()
{
	static bool initialized = false;
	static bool init_failed = false;
	static bool counting_up = true;
	static uint32_t period_counts = 0;
	static uint32_t last_position = 0;
	static uint64_t accumulated_counts = 0;
	static uint32_t clock_frequency_hz = 0;
	static uint32_t counter_span = 0;
	static uint32_t zero_delta_count = 0;
	static bool timer_stalled = false;
	static const timer_instance_t *s_hrt_timer = nullptr;

	const uint32_t cpsr_snapshot = read_cpsr();
	const bool in_isr = is_exception_mode(cpsr_snapshot);

	if (!initialized && !init_failed) {
		if (in_isr) {
			return ensure_monotonic(fallback_time_us(true));
		}

		/* Try each candidate timer in priority order (GTM0 → GTM3 backup) */
		for (const timer_instance_t *candidate : kHrtTimerCandidates) {
			timer_info_t info{};
			const fsp_err_t open_ret = candidate->p_api->open(candidate->p_ctrl, candidate->p_cfg);

			if ((open_ret != FSP_SUCCESS) && (open_ret != FSP_ERR_ALREADY_OPEN)) {
				continue;  /* Try next candidate */
			}

			(void)candidate->p_api->start(candidate->p_ctrl);

			if (FSP_SUCCESS == candidate->p_api->infoGet(candidate->p_ctrl, &info)) {
				period_counts = info.period_counts;
				counting_up = (info.count_direction == TIMER_DIRECTION_UP);

				if (info.clock_frequency != 0U) {
					clock_frequency_hz = info.clock_frequency;
				}
			}

			if (period_counts == 0U) {
				period_counts = 100000U;
			}

			const uint64_t raw_span = static_cast<uint64_t>(period_counts) + 1ULL;

			if (raw_span <= UINT32_MAX) {
				counter_span = static_cast<uint32_t>(raw_span);

			} else {
				counter_span = UINT32_MAX;
			}

			if (clock_frequency_hz == 0U) {
				const uint64_t estimated = raw_span * 1000ULL;

				if (estimated <= UINT32_MAX) {
					clock_frequency_hz = static_cast<uint32_t>(estimated);
				}
			}

			if (clock_frequency_hz == 0U) {
				clock_frequency_hz = 100000000U;
			}

			const uint32_t critical = enter_critical_raw();
			initialized = (clock_frequency_hz != 0U) && (period_counts != 0U) && (counter_span != 0U);

			if (initialized) {
				s_hrt_timer = candidate;
			}

			last_position = 0;
			accumulated_counts = 0;
			exit_critical_raw(critical);

			const bool is_backup = (candidate != &gtm0);

			if (!__atomic_exchange_n(&s_logged_init, true, __ATOMIC_ACQ_REL)) {
				if (is_backup) {
					PX4_WARN("RZV HRT init: GTM0 unavailable, using GTM3 backup");
				}

				PX4_DEBUG("RZV HRT init: timer=%s period=%" PRIu32 " span=%" PRIu32 " freq=%" PRIu32 " Hz dir=%s",
					 is_backup ? "GTM3" : "GTM0",
					 period_counts, counter_span, clock_frequency_hz, counting_up ? "up" : "down");
			}

			break;  /* Successfully initialized — stop trying candidates */
		}

		if (!initialized) {
			const uint32_t critical = enter_critical_raw();
			init_failed = true;
			exit_critical_raw(critical);

			if (!__atomic_exchange_n(&s_logged_fail_open, true, __ATOMIC_ACQ_REL)) {
				PX4_ERR("RZV HRT init failed: all GTM candidates (GTM0, GTM3) failed to open");
			}
		}
	}

	if (!initialized || init_failed || (clock_frequency_hz == 0U) || (s_hrt_timer == nullptr)) {
		__atomic_fetch_add(&s_fallback_calls, 1U, __ATOMIC_RELAXED);

		if (!in_isr && !__atomic_exchange_n(&s_logged_fallback, true, __ATOMIC_ACQ_REL)) {
			PX4_WARN("RZV HRT fallback to FreeRTOS tick (init=%d, fail=%d, freq=%" PRIu32 ")",
				 static_cast<int>(initialized), static_cast<int>(init_failed), clock_frequency_hz);
		}

		return ensure_monotonic(fallback_time_us(in_isr));
	}

	const uint32_t critical = enter_critical_raw();

	timer_status_t timer_status{};
	const fsp_err_t status = s_hrt_timer->p_api->statusGet(s_hrt_timer->p_ctrl, &timer_status);

	if (status != FSP_SUCCESS) {
		exit_critical_raw(critical);
		__atomic_fetch_add(&s_status_failures, 1U, __ATOMIC_RELAXED);
		__atomic_fetch_add(&s_fallback_calls, 1U, __ATOMIC_RELAXED);

		if (!in_isr && !__atomic_exchange_n(&s_logged_status_error, true, __ATOMIC_ACQ_REL)) {
			PX4_ERR("RZV HRT statusGet failed: %d", status);
		}

		return ensure_monotonic(fallback_time_us(in_isr));
	}

	const uint32_t raw_counter = static_cast<uint32_t>(timer_status.counter);

	if (!initialized || (counter_span == 0U)) {
		last_position = counting_up ? raw_counter : (counter_span - 1U - raw_counter);
		exit_critical_raw(critical);
		return ensure_monotonic(fallback_time_us(in_isr));
	}

	const uint32_t position = counting_up ? raw_counter : (counter_span - 1U - raw_counter);
	uint32_t delta_counts;

	if (position >= last_position) {
		delta_counts = position - last_position;

	} else {
		delta_counts = (counter_span - last_position) + position;
		__atomic_fetch_add(&s_wrap_events, 1U, __ATOMIC_RELAXED);
	}

	last_position = position;
	accumulated_counts += delta_counts;

	if (delta_counts == 0U) {
		if (zero_delta_count < UINT32_MAX) {
			++zero_delta_count;
		}

		if (zero_delta_count >= RZV_HRT_STALL_THRESHOLD) {
			timer_stalled = true;
		}

	} else {
		zero_delta_count = 0;
		timer_stalled = false;
		__atomic_store_n(&s_logged_timer_stall, false, __ATOMIC_RELEASE);
	}

	const bool stall_active = timer_stalled;

	exit_critical_raw(critical);

	if (stall_active) {
		__atomic_fetch_add(&s_fallback_calls, 1U, __ATOMIC_RELAXED);

		if (!in_isr && !__atomic_exchange_n(&s_logged_timer_stall, true, __ATOMIC_ACQ_REL)) {
			PX4_WARN("RZV HRT detected stalled timer, falling back to FreeRTOS tick");
		}

		return ensure_monotonic(fallback_time_us(in_isr));
	}

	const hrt_abstime candidate = counts_to_time_us(accumulated_counts, clock_frequency_hz);
	return ensure_monotonic(candidate);
}

} // namespace

void rzv_hrt_init()
{
	/* Trigger HRT timer open/start from task context so that ISRs (e.g.,
	 * MPU9250 DRDY) see initialized=true and use full-resolution timestamps
	 * instead of the 1 ms FreeRTOS-tick fallback.
	 *
	 * hw_time_us() tries GTM0 first, then GTM3 as backup.  It skips
	 * init when called from ISR context (in_isr=true), so this function must
	 * be invoked from a task before the first DRDY fires.
	 * Subsequent calls are no-ops once initialized=true.
	 */
	(void)hw_time_us();
}

hrt_abstime rzv_hrt_absolute_time()
{
	return hw_time_us();
}

void rzv_hrt_get_diagnostics(rzv_hrt_diagnostics_t *diag)
{
	if (diag == nullptr) {
		return;
	}

	const uint32_t critical = enter_critical_raw();
	diag->isr_overflows = 0;
	diag->sw_overflows = s_wrap_events;
	diag->skip_isr_hits = 0;
	diag->status_failures = s_status_failures;
	diag->fallback_calls = s_fallback_calls;
	diag->monotonic_adjustments = s_monotonic_adjustments;
	exit_critical_raw(critical);
}

void rzv_hrt_print_diagnostics()
{
	rzv_hrt_diagnostics_t diag{};
	rzv_hrt_get_diagnostics(&diag);

	/* Log warning if fallback mode was used (indicates timing jitter) */
	if (diag.fallback_calls > 0) {
		PX4_WARN("HRT HEALTH: fallback=%" PRIu32 " (timing jitter detected!)",
			 diag.fallback_calls);
	}

	/* Log detailed diagnostics at info level */
	PX4_INFO("HRT: fallback=%" PRIu32 " adj=%" PRIu32 " wrap=%" PRIu32 " status_err=%" PRIu32,
		 diag.fallback_calls,
		 diag.monotonic_adjustments,
		 diag.sw_overflows,
		 diag.status_failures);
}

#endif /* __PX4_FREERTOS */
