/****************************************************************************
 *
 *   Copyright (c) 2023 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include "ICM45686.hpp"

using namespace time_literals;

#if defined(__PX4_FREERTOS)
// HAL ring (micro_hal.cpp): pop the timestamp captured in the HW DRDY ISR for this pin
// (returns 0 if none). Lets DataReady() use the true ISR time, not deferred-task time.
extern "C" uint64_t rzv_sensor_hal_pop_drdy_timestamp(uint32_t pinset);
#endif

static constexpr int16_t combine(uint8_t msb, uint8_t lsb)
{
	return (msb << 8u) | lsb;
}

static constexpr uint16_t combine_uint(uint8_t msb, uint8_t lsb)
{
	return (msb << 8u) | lsb;
}

ICM45686::ICM45686(const I2CSPIDriverConfig &config) :
	SPI(config),
	I2CSPIDriver(config),
	// custom2 == 1: -P flag — force polled (timer) mode by pretending there is no DRDY pin
	_drdy_gpio(config.custom2 == 1 ? 0 : config.drdy_gpio),
	_px4_accel(get_device_id(), config.rotation),
	_px4_gyro(get_device_id(), config.rotation)
{
	if (_drdy_gpio != 0) {
		_drdy_missed_perf = perf_alloc(PC_COUNT, MODULE_NAME": DRDY missed");
	}

	if (config.custom1 != 0) {
		_enable_clock_input = true;
		_input_clock_freq = config.custom1;
		// TODO: this is not tested
		ConfigureCLKIN();

	} else {
		_enable_clock_input = false;
	}

#if defined(__PX4_FREERTOS)
	// RZ/V2H: cap the FIFO read rate to a deterministic value regardless of when params
	// load (PX4Gyroscope reads IMU_GYRO_RATEMAX in its ctor, which on RZV can run before
	// the deferred param sync, returning a default rather than the configured rate).
	static constexpr int32_t RZV_IMU_FIFO_RATE_MAX_HZ = 200;
	ConfigureSampleRate(math::min(_px4_gyro.get_max_rate_hz(), RZV_IMU_FIFO_RATE_MAX_HZ));
#else
	ConfigureSampleRate(_px4_gyro.get_max_rate_hz());
#endif
}

ICM45686::~ICM45686()
{
	perf_free(_bad_register_perf);
	perf_free(_bad_transfer_perf);
	perf_free(_fifo_empty_perf);
	perf_free(_fifo_overflow_perf);
	perf_free(_fifo_reset_perf);
	perf_free(_drdy_missed_perf);
	perf_free(_gyro_spike_perf);
}

int ICM45686::init()
{
	int ret = SPI::init();

	if (ret != PX4_OK) {
		DEVICE_DEBUG("SPI::init failed (%i)", ret);
		return ret;
	}

	return Reset() ? 0 : -1;
}

bool ICM45686::Reset()
{
	DataReadyInterruptDisable();
	_state = STATE::RESET;
	ScheduleClear();
	ScheduleNow();
	return true;
}

void ICM45686::exit_and_cleanup()
{
	DataReadyInterruptDisable();
	I2CSPIDriverBase::exit_and_cleanup();
}

void ICM45686::print_status()
{
	I2CSPIDriverBase::print_status();

	PX4_INFO("FIFO empty interval: %d us (%.1f Hz)", _fifo_empty_interval_us, 1e6 / _fifo_empty_interval_us);
	PX4_INFO("Clock input: %s", _enable_clock_input ? "enabled" : "disabled");
	PX4_INFO("FIFO TMST dt: %.1f us (mode: %s, res: %.2f us, raw: %u)", (double)_fifo_measured_dt_us,
		 (_tmst_mode == 1) ? "direct" : (_tmst_mode == 2) ? "diff" : "nominal",
		 (double)_tmst_res_us, (unsigned)_tmst_last_raw);

	perf_print_counter(_bad_register_perf);
	perf_print_counter(_bad_transfer_perf);
	perf_print_counter(_fifo_empty_perf);
	perf_print_counter(_fifo_overflow_perf);
	perf_print_counter(_fifo_reset_perf);
	perf_print_counter(_drdy_missed_perf);
	perf_print_counter(_gyro_spike_perf);
}

int ICM45686::probe()
{
	for (int i = 0; i < 3; i++) {
		const uint8_t whoami = RegisterRead(Register::BANK_0::WHO_AM_I);

		if (whoami != WHOAMI) {
			DEVICE_DEBUG("unexpected WHO_AM_I 0x%02x", whoami);
			return PX4_ERROR;
		}
	}

	return PX4_OK;
}

void ICM45686::RunImpl()
{
	const hrt_abstime now = hrt_absolute_time();

	switch (_state) {
	case STATE::RESET:
		// DEVICE_CONFIG: Software reset configuration
		RegisterWrite(Register::BANK_0::REG_MISC2, REG_MISC2_BIT::SOFT_RST);
		_reset_timestamp = now;
		_failure_count = 0;
		_state = STATE::WAIT_FOR_RESET;
		ScheduleDelayed(1_ms); // wait 1 ms for soft reset to be effective
		break;

	case STATE::WAIT_FOR_RESET:
		if ((RegisterRead(Register::BANK_0::WHO_AM_I) == WHOAMI)
		    && ((RegisterRead(Register::BANK_0::REG_MISC2) & Bit1) == 0x0)) {

			// Wakeup accel and gyro and schedule remaining configuration
			RegisterWrite(Register::BANK_0::PWR_MGMT0, PWR_MGMT0_BIT::GYRO_MODE_LOW_NOISE | PWR_MGMT0_BIT::ACCEL_MODE_LOW_NOISE);
			_state = STATE::CONFIGURE;
			ScheduleDelayed(30_ms); // 30 ms gyro startup time, 10 ms accel from sleep to valid data

		} else {
			// RESET not complete
			if (hrt_elapsed_time(&_reset_timestamp) > 1000_ms) {
				PX4_DEBUG("Reset failed, retrying");
				_state = STATE::RESET;
				ScheduleDelayed(100_ms);

			} else {
				PX4_DEBUG("Reset not complete, check again in 10 ms");
				ScheduleDelayed(10_ms);
			}
		}

		break;

	case STATE::CONFIGURE:
		if (Configure()) {
			// if configure succeeded then reset the FIFO
			_state = STATE::FIFO_RESET;
			ScheduleDelayed(1_ms);

		} else {
			// CONFIGURE not complete
			if (hrt_elapsed_time(&_reset_timestamp) > 1000_ms) {
				PX4_DEBUG("Configure failed, resetting");
				_state = STATE::RESET;

			} else {
				PX4_DEBUG("Configure failed, retrying");
			}

			ScheduleDelayed(100_ms);
		}

		break;

	case STATE::FIFO_RESET:

		_state = STATE::FIFO_READ;
		FIFOReset();

		if (DataReadyInterruptConfigure()) {
			_data_ready_interrupt_enabled = true;

			// backup schedule as a watchdog timeout
			ScheduleDelayed(100_ms);

		} else {
			_data_ready_interrupt_enabled = false;
			ScheduleOnInterval(_fifo_empty_interval_us, _fifo_empty_interval_us);
		}

		break;

	case STATE::FIFO_READ: {
			hrt_abstime timestamp_sample = now;

			if (_data_ready_interrupt_enabled) {
				// scheduled from interrupt if _drdy_timestamp_sample was set as expected
				const hrt_abstime drdy_timestamp_sample = _drdy_timestamp_sample.fetch_and(0);

				if ((now - drdy_timestamp_sample) < _fifo_empty_interval_us) {
					timestamp_sample = drdy_timestamp_sample;

				} else {
					perf_count(_drdy_missed_perf);
				}

				// push backup schedule back
				ScheduleDelayed(_fifo_empty_interval_us * 2);
			}

			bool success = false;

			if (FIFORead(timestamp_sample)) {
				success = true;

				if (_failure_count > 0) {
					_failure_count--;
				}
			}

			if (!success) {
				_failure_count++;

				// full reset if things are failing consistently
				if (_failure_count > 10) {
					Reset();
					return;
				}
			}

			if (_data_ready_interrupt_enabled) {
				// INT1 is latched (RZV TINT misses the short pulse). Read INT1_STATUS0
				// after draining the FIFO to clear the latch so it re-arms for the next
				// watermark crossing; otherwise it asserts once and never edges again.
				RegisterRead(Register::BANK_0::INT1_STATUS0);

				// If the FIFO refilled to/above the watermark during the SPI read, the latched
				// INT1 won't give the TINT a fresh edge, so drain again now instead of waiting
				// for the watchdog (avoids a DRDY-missed). Converges: each pass drains.
				if (FIFOReadCount() >= _fifo_watermark) {
					ScheduleNow();
				}
			}

			if (!success || hrt_elapsed_time(&_last_config_check_timestamp) > 100_ms) {
				// check configuration registers periodically or immediately following any failure
				const register_bank0_config_t &checked = _register_bank0_cfg[_checked_register_bank0];

				// The FIFO watermark registers (FIFO_CONFIG1_0/1) are effectively write-only
				// here: their readback is unreliable (shadow/latch quirk -> occasional 0x00).
				// They are still force-written in Configure(); they MUST NOT be part of the
				// periodic RegisterCheck, otherwise a benign 0x00 readback would trigger a full
				// IMU Reset() mid-flight (~50-100 ms gyro/accel blackout -> EKF starvation).
				const bool skip_check = (checked.reg == Register::BANK_0::FIFO_CONFIG1_0)
							|| (checked.reg == Register::BANK_0::FIFO_CONFIG1_1);

				if (skip_check || RegisterCheck(checked)) {
					_last_config_check_timestamp = now;
					_checked_register_bank0 = (_checked_register_bank0 + 1) % size_register_bank0_cfg;
					_register_check_fail_count = 0;

				} else {
					perf_count(_bad_register_perf);
#if defined(__PX4_FREERTOS)
					// RZ/V2H: a failed readback here is almost always a transient corrupted READ,
					// not real register drift. IMU#1's INT1 pin shares physical net P50 with the
					// debug-console UART TXD; at boot the sensor's push-pull INT1 (POR default)
					// contends with the UART until Configure() sets INT1 open-drain, and the reads
					// come back bit-shifted (FIFO_CONFIG0 = 0x9F<<1). A full Reset() re-runs SOFT_RST
					// -> INT1 returns to its push-pull POR default -> RE-OPENS the contention every
					// cycle: a self-sustaining reconfigure storm (+ EKF gyro/filter faults) that only
					// clears when the boot-log UART traffic on P50 subsides. Re-write the register in
					// place instead of resetting; the open-drain INT1 config then STAYS applied (never
					// re-defaulted) so the contention ends immediately. Escalate to a real Reset() only
					// if the SAME check keeps failing (genuine drift, not the boot read-artifact).
					RegisterSetAndClearBits(checked.reg, checked.set_bits, checked.clear_bits);
					_last_config_check_timestamp = now;

					if (++_register_check_fail_count >= 20) {
						_register_check_fail_count = 0;
						Reset();
					}
#else
					// register check failed, force reset
					Reset();
#endif
				}
			}
		}

		break;
	}
}

void ICM45686::ConfigureSampleRate(int sample_rate)
{
	// round down to the nearest FIFO sample dt
	const float min_interval = FIFO_SAMPLE_DT;
	_fifo_empty_interval_us = math::max(roundf((1e6f / (float)sample_rate) / min_interval) * min_interval, min_interval);

	_fifo_gyro_samples = roundf(math::min((float)_fifo_empty_interval_us / (1e6f / GYRO_RATE), (float)FIFO_MAX_SAMPLES));

	// recompute FIFO empty interval (us) with actual gyro sample limit
	_fifo_empty_interval_us = _fifo_gyro_samples * (1e6f / GYRO_RATE);

	ConfigureFIFOWatermark(_fifo_gyro_samples);
}

void ICM45686::ConfigureFIFOWatermark(uint8_t samples)
{
	// ICM-45686 FIFO_WM (FIFO_CONFIG1_0/1) is the watermark in FIFO *records*, compared
	// against FIFO_COUNT which the datasheet register description reports in packets (the
	// overflow check also treats the count as records: FIFO::SIZE / sizeof(FIFO::DATA)).
	// (The upstream value samples * sizeof(FIFO::DATA) is in bytes and exceeds the ~409
	// record FIFO depth for any real batch, so FIFO_THS would never assert.)
	const uint16_t fifo_watermark_threshold = samples;
	_fifo_watermark = fifo_watermark_threshold;

	for (auto &r : _register_bank0_cfg) {
		if (r.reg == Register::BANK_0::FIFO_CONFIG1_0) {
			r.set_bits = fifo_watermark_threshold & 0xFF;        // FIFO_WM[7:0]

		} else if (r.reg == Register::BANK_0::FIFO_CONFIG1_1) {
			r.set_bits = (fifo_watermark_threshold >> 8) & 0xFF; // FIFO_WM[15:8]
		}
	}
}

void ICM45686::ConfigureCLKIN()
{
	for (auto &r0 : _register_bank0_cfg) {
		if (r0.reg == Register::BANK_0::RTC_CONFIG) {
			r0.set_bits = RTC_CONFIG_BIT::RTC_MODE;
		}
	}

	for (auto &r0 : _register_bank0_cfg) {
		if (r0.reg == Register::BANK_0::IOC_PAD_SCENARIO_OVRD) {
			r0.set_bits = PADS_INT2_CFG_OVRD | PADS_INT2_CFG_OVRD_CLKIN;
		}
	}
}

bool ICM45686::Configure()
{
#if defined(__PX4_FREERTOS)
	// ── RZ/V2H UI anti-alias low-pass filter (mandatory at the lowered 800 Hz ODR) ───
	// At ODR 800 Hz the Nyquist is 400 Hz; frame/motor resonance >400 Hz would alias
	// straight into the control band. The ICM-45686 UI LPF bandwidth is NOT in BANK_0
	// (no GYRO/ACCEL_CONFIG1 like the 42688P) — it lives in the indirect IPREG space,
	// reached via the IREG interface (BANK_0 0x7C/0x7D/0x7E = ADDR_HI/ADDR_LO/DATA).
	// Per the TDK ICM-45686 driver the value is a fraction of ODR: 0x03 = ODR/16, which
	// at 800 Hz = 50 Hz (mirrors the MPU9250 RZ/V port's 41 Hz DLPF; fixed 3rd order).
	// Must run after wake (PWR_MGMT0 was set + 30 ms gyro startup in WAIT_FOR_RESET, so
	// the IPREG clocks are up) and before the FIFO is enabled below. This pairs with the
	// 800 Hz ODR above; on the upstream 6400 Hz ODR a different BW index would be needed.
	//   GYRO_UI_LPFBW  = IPREG_SYS1_REG_172 @ 0xA4AC
	//   ACCEL_UI_LPFBW = IPREG_SYS2_REG_131 @ 0xA583
	{
		auto delay_us = [](uint32_t us) {
			const hrt_abstime t0 = hrt_absolute_time();

			while (hrt_elapsed_time(&t0) < us) { /* short busy-wait, startup only */ }
		};

		auto write_ipreg = [&](uint16_t addr, uint8_t data) {
			// IREG burst write: SPI auto-increments 0x7C -> 0x7D -> 0x7E.
			uint8_t cmd[4] { 0x7C, (uint8_t)((addr >> 8) & 0xFF), (uint8_t)(addr & 0xFF), data };
			delay_us(4);   // TDK: ~4 us before IREG access (no IREG_DONE poll on this part)
			transfer(cmd, cmd, sizeof(cmd));
			delay_us(4);   // TDK: ~4 us after IREG access
		};

		write_ipreg(0xA4AC, 0x03); // GYRO_UI_LPFBW  = ODR/16 = 50 Hz @ 800 Hz ODR
		write_ipreg(0xA583, 0x03); // ACCEL_UI_LPFBW = ODR/16 = 50 Hz @ 800 Hz ODR
		// (readback-verified on RZ/V2H .141 2026-06-26: both read back 0x03)

		auto read_ipreg = [&](uint16_t addr) -> uint8_t {
			uint8_t cmd_addr[3] { 0x7C, (uint8_t)((addr >> 8) & 0xFF), (uint8_t)(addr & 0xFF) };
			delay_us(4);
			transfer(cmd_addr, cmd_addr, sizeof(cmd_addr));
			delay_us(4);
			uint8_t cmd_data[2] { static_cast<uint8_t>(0x7E | DIR_READ), 0 };
			transfer(cmd_data, cmd_data, sizeof(cmd_data));
			delay_us(4);
			return cmd_data[1];
		};

		// Start the internal timestamp counter (SMC_CONTROL_0.TMST_EN, IPREG_TOP1
		// 0xA258 Bit0 — default 0, so the FIFO TMST field reads all-zero without
		// this). UpdateMeasuredDt() consumes the field for the true per-sample dt
		// in polled mode. RMW to preserve TEMP_DIS / clock-select bits.
		const uint8_t smc0 = read_ipreg(0xA258);

		if ((smc0 & 0x01) == 0) {
			write_ipreg(0xA258, smc0 | 0x01);
		}
	}

	// FIFO TMST field = DELTA ticks between frames at 1 us/LSB
	// (direct interpretation in UpdateMeasuredDt locks onto res = 1 us).
	RegisterSetAndClearBits(Register::BANK_0::TMST_WOM_CONFIG,
				TMST_WOM_CONFIG_BIT::TMST_DELTA_EN,
				TMST_WOM_CONFIG_BIT::TMST_RESOL);
#endif /* __PX4_FREERTOS */

	// first set and clear all configured register bits
	for (const auto &reg_cfg : _register_bank0_cfg) {
		RegisterSetAndClearBits(reg_cfg.reg, reg_cfg.set_bits, reg_cfg.clear_bits);
	}

	// now check that all are configured
	bool success = true;

	for (const auto &reg_cfg : _register_bank0_cfg) {
		// FIFO_CONFIG1_0/1 (watermark) are effectively write-only here: their readback is
		// unreliable (shadow/latch quirk -> occasional 0x00), so checking them can fail the
		// whole configure spuriously. They are force-written below (and in FIFOReset). Skip
		// them on the initial check too, mirroring the periodic RegisterCheck skip.
		if ((reg_cfg.reg == Register::BANK_0::FIFO_CONFIG1_0)
		    || (reg_cfg.reg == Register::BANK_0::FIFO_CONFIG1_1)) {
			continue;
		}

		if (!RegisterCheck(reg_cfg)) {
			success = false;
		}
	}

	// 20-bits data format used the only FSR settings that are operational
	// are ±4000dps for gyroscope and ±32 for accelerometer
	_px4_accel.set_range(32.f * CONSTANTS_ONE_G);
	_px4_gyro.set_range(math::radians(4000.f));

	// data is published from the 16-bit FIFO registers (data[19:4]) which always cover the
	// full range: 1024 LSB/g and 131/16 LSB/dps
	_px4_accel.set_scale(CONSTANTS_ONE_G / 8192.f * 8.f);
	_px4_gyro.set_scale(math::radians(1.f / 131.f * 16.f));

	// Force-write the FIFO watermark (records) so the FIFO_THS threshold actually latches.
	// Per datasheet the threshold takes effect ONLY when the MSByte (FIFO_CONFIG1_1) is
	// written, and the LSByte must be written first. The table-driven RegisterSetAndClearBits
	// skips a write when the value is unchanged, so for small watermarks (MSByte == reset 0x00)
	// the threshold would never latch and INT1 would never assert. Write both explicitly.
	RegisterWrite(Register::BANK_0::FIFO_CONFIG1_0, _fifo_watermark & 0xFF);
	RegisterWrite(Register::BANK_0::FIFO_CONFIG1_1, (_fifo_watermark >> 8) & 0xFF);

	return success;
}

template <typename T>
bool ICM45686::RegisterCheck(const T &reg_cfg)
{
	bool success = true;

	const uint8_t reg_value = RegisterRead(reg_cfg.reg);

	const bool set_mismatch   = reg_cfg.set_bits   && ((reg_value & reg_cfg.set_bits) != reg_cfg.set_bits);
	const bool clear_mismatch = reg_cfg.clear_bits && ((reg_value & reg_cfg.clear_bits) != 0);

	if (set_mismatch || clear_mismatch) {
		success = false;

		// Rate-limit the diagnostic to 1/s per instance. A transient corrupted
		// SPI read (3 IMUs sharing one bus under OpenAMP; historically also the
		// P50/INT1-vs-console contention) fails these checks in bursts, and each
		// PX4_INFO is a polled-UART console write inside the sensor work queue —
		// unthrottled it burns CR8 CPU exactly during the burst. RunImpl re-writes
		// the register in place and escalates to Reset() on its own fail counter,
		// so the per-check line is purely diagnostic; one line/s keeps the signal.
		const hrt_abstime now = hrt_absolute_time();

		if (now - _last_regcheck_log_timestamp >= 1_s) {
			_last_regcheck_log_timestamp = now;

			if (set_mismatch) {
				PX4_INFO("0x%02hhX: 0x%02hhX (0x%02hhX not set)", (uint8_t)reg_cfg.reg, reg_value, reg_cfg.set_bits);
			}

			if (clear_mismatch) {
				PX4_INFO("0x%02hhX: 0x%02hhX (0x%02hhX not cleared)", (uint8_t)reg_cfg.reg, reg_value, reg_cfg.clear_bits);
			}
		}
	}

	return success;
}

template <typename T>
uint8_t ICM45686::RegisterRead(T reg)
{
	uint8_t cmd[2] {};
	cmd[0] = static_cast<uint8_t>(reg) | DIR_READ;
	transfer(cmd, cmd, sizeof(cmd));
	return cmd[1];
}

template <typename T>
void ICM45686::RegisterWrite(T reg, uint8_t value)
{
	uint8_t cmd[2] { (uint8_t)reg, value };
	transfer(cmd, cmd, sizeof(cmd));
}

template <typename T>
void ICM45686::RegisterSetAndClearBits(T reg, uint8_t setbits, uint8_t clearbits)
{
	const uint8_t orig_val = RegisterRead(reg);

	uint8_t val = (orig_val & ~clearbits) | setbits;

	if (orig_val != val) {
		RegisterWrite(reg, val);
	}
}

int ICM45686::DataReadyInterruptCallback(int irq, void *context, void *arg)
{
	static_cast<ICM45686 *>(arg)->DataReady();
	return 0;
}

void ICM45686::DataReady()
{
#if defined(__PX4_FREERTOS)
	// Use the timestamp latched in the HW DRDY ISR (HAL ring), consuming one entry per call so
	// batched IRQs aren't collapsed to deferred-task time; fall back to now if the ring is empty.
	const uint64_t isr_ts = rzv_sensor_hal_pop_drdy_timestamp((uint32_t)_drdy_gpio);
	_drdy_timestamp_sample.store(isr_ts != 0 ? isr_ts : hrt_absolute_time());
#else
	_drdy_timestamp_sample.store(hrt_absolute_time());
#endif
	ScheduleNow();
}

bool ICM45686::DataReadyInterruptConfigure()
{
	if (_drdy_gpio == 0) {
		return false;
	}

	// Setup data ready on falling edge (INT1 is configured pulsed, active low)
	return px4_arch_gpiosetevent(_drdy_gpio, false, true, true, &DataReadyInterruptCallback, this) == 0;
}

bool ICM45686::DataReadyInterruptDisable()
{
	if (_drdy_gpio == 0) {
		return false;
	}

	return px4_arch_gpiosetevent(_drdy_gpio, false, false, false, nullptr, nullptr) == 0;
}

uint16_t ICM45686::FIFOReadCount()
{
	// read FIFO count
	uint8_t fifo_count_buf[3] {};
	fifo_count_buf[0] = static_cast<uint8_t>(Register::BANK_0::FIFO_COUNT_0) | DIR_READ;

	if (transfer(fifo_count_buf, fifo_count_buf, sizeof(fifo_count_buf)) != PX4_OK) {
		perf_count(_bad_transfer_perf);
		return 0;
	}

	// FIFO_COUNT_0 is supposed to contain the high bits and FIFO_COUNT_1 the low bits,
	// according to the manual, however, the device is configured to little endianness
	// which means FIFO and FIFO count are pre-swapped..
	return combine(fifo_count_buf[2], fifo_count_buf[1]);
}

bool ICM45686::FIFORead(const hrt_abstime &timestamp_sample)
{
	const uint16_t fifo_packets = FIFOReadCount();

	if (fifo_packets == 0) {
		perf_count(_fifo_empty_perf);
		return false;
	}

	if (fifo_packets >= FIFO::SIZE / sizeof(FIFO::DATA)) {
		// FIFO saturated: in stop-on-full mode newer samples have been dropped, reset for a
		// clean restart rather than draining a stale backlog
		perf_count(_fifo_overflow_perf);
		FIFOReset();
		return false;
	}

	FIFOTransferBuffer buffer{};
	const size_t transfer_size = math::min(sizeof(FIFOTransferBuffer), fifo_packets * sizeof(FIFO::DATA) + 1);

	if (transfer((uint8_t *)&buffer, (uint8_t *)&buffer, transfer_size) != PX4_OK) {
		perf_count(_bad_transfer_perf);
		return false;
	}

	unsigned valid_samples = 0;

	for (unsigned i = 0; i < transfer_size / sizeof(FIFO::DATA); i++) {
		bool valid = true;

		// With FIFO_ACCEL_EN and FIFO_GYRO_EN header should be 8’b_0110_10xx
		const uint8_t FIFO_HEADER = buffer.f[i].FIFO_Header;

		if (FIFO_HEADER & FIFO::FIFO_HEADER_BIT::HEADER_MSG) {
			// FIFO sample empty if HEADER_MSG set
			valid = false;

		} else if (!(FIFO_HEADER & FIFO::FIFO_HEADER_BIT::HEADER_ACCEL)) {
			// accel bit not set
			valid = false;

		} else if (!(FIFO_HEADER & FIFO::FIFO_HEADER_BIT::HEADER_GYRO)) {
			// gyro bit not set
			valid = false;

		} else if (!(FIFO_HEADER & FIFO::FIFO_HEADER_BIT::HEADER_20)) {
			// Packet does not contain a new and valid extended 20-bit data
			valid = false;

		} else if ((FIFO_HEADER & FIFO::FIFO_HEADER_BIT::HEADER_TIMESTAMP_FSYNC) != Bit3) {
			// Packet does not contain ODR timestamp
			valid = false;

		} else if (FIFO_HEADER & FIFO::FIFO_HEADER_BIT::HEADER_ODR_ACCEL) {
			// accel ODR changed
			valid = false;

		} else if (FIFO_HEADER & FIFO::FIFO_HEADER_BIT::HEADER_ODR_GYRO) {
			// gyro ODR changed
			valid = false;
		}

		if (valid) {
			valid_samples++;

		} else {
			perf_count(_bad_transfer_perf);
			break;
		}
	}

	if (valid_samples > 0) {
		if (ProcessTemperature(buffer.f, valid_samples)) {
			UpdateMeasuredDt(buffer.f, valid_samples);
			ProcessGyro(timestamp_sample, buffer.f, valid_samples);
			ProcessAccel(timestamp_sample, buffer.f, valid_samples);
			return true;
		}
	}

	return false;
}

void ICM45686::UpdateMeasuredDt(const FIFO::DATA fifo[], const uint8_t samples)
{
	if (_enable_clock_input) {
		return; // CLKIN path computes dt per sample in ProcessAccel/ProcessGyro
	}

	// Per-sample interval from the FIFO timestamp field on the internal clock.
	// Neither the field semantics (delta ticks between frames vs free-running
	// counter) nor the tick resolution is fully documented for this part, so
	// evaluate direct/diff interpretations across the candidate resolutions
	// and keep whichever yields plausible ODR periods.
	static constexpr float RES_CANDIDATES_US[] = {1.f, 16.f, FIFO_TIMESTAMP_SCALING};
	static constexpr unsigned N_RES = sizeof(RES_CANDIDATES_US) / sizeof(RES_CANDIDATES_US[0]);

	float sum_direct[N_RES] {};
	unsigned n_direct[N_RES] {};
	float sum_diff[N_RES] {};
	unsigned n_diff[N_RES] {};
	uint16_t prev = 0;

	for (unsigned i = 0; i < samples; i++) {
		// Swapped as device is in little endian by default.
		const uint16_t t = combine_uint(fifo[i].Timestamp_L, fifo[i].Timestamp_H);
		_tmst_last_raw = t;

		for (unsigned r = 0; r < N_RES; r++) {
			const float d_direct = (float)t * RES_CANDIDATES_US[r];

			if ((d_direct > 0.5f * FIFO_SAMPLE_DT) && (d_direct < 1.5f * FIFO_SAMPLE_DT)) {
				sum_direct[r] += d_direct;
				n_direct[r]++;
			}

			if (i > 0) {
				const uint16_t dt_ticks = (uint16_t)(t - prev); // uint16 wrap-safe
				const float d_diff = (float)dt_ticks * RES_CANDIDATES_US[r];

				if ((d_diff > 0.5f * FIFO_SAMPLE_DT) && (d_diff < 1.5f * FIFO_SAMPLE_DT)) {
					sum_diff[r] += d_diff;
					n_diff[r]++;
				}
			}
		}

		prev = t;
	}

	// Pick the interpretation with the most in-range samples (direct preferred on ties).
	float measured = 0.f;
	unsigned best_n = 0;

	for (unsigned r = 0; r < N_RES; r++) {
		if (n_direct[r] > best_n) {
			best_n = n_direct[r];
			measured = sum_direct[r] / n_direct[r];
			_tmst_mode = 1;
			_tmst_res_us = RES_CANDIDATES_US[r];
		}

		if (n_diff[r] > best_n) {
			best_n = n_diff[r];
			measured = sum_diff[r] / n_diff[r];
			_tmst_mode = 2;
			_tmst_res_us = RES_CANDIDATES_US[r];
		}
	}

	if (best_n == 0) {
		return; // keep previous estimate (or nominal), don't poison dt
	}

	// Low-pass so single-batch noise doesn't jitter the published dt.
	_fifo_measured_dt_us = 0.9f * _fifo_measured_dt_us + 0.1f * measured;
}

void ICM45686::FIFOReset()
{
	perf_count(_fifo_reset_perf);
	_drdy_timestamp_sample.store(0);

	// Disable FIFO
	RegisterClearBits(Register::BANK_0::FIFO_CONFIG3,
			  FIFO_CONFIG3_BIT::FIFO_ES1_EN |
			  FIFO_CONFIG3_BIT::FIFO_ES0_EN |
			  FIFO_CONFIG3_BIT::FIFO_HIRES_EN |
			  FIFO_CONFIG3_BIT::FIFO_GYRO_EN |
			  FIFO_CONFIG3_BIT::FIFO_ACCEL_EN |
			  FIFO_CONFIG3_BIT::FIFO_IF_EN);

	// Disable FIFO by switching to bypass mode
	RegisterSetAndClearBits(Register::BANK_0::FIFO_CONFIG0,
				FIFO_CONFIG0_BIT::FIFO_MODE_BYPASS_SET,
				FIFO_CONFIG0_BIT::FIFO_MODE_BYPASS_CLEAR);

	// When the FIFO is disabled we can actually set the FIFO depth
	RegisterSetBits(Register::BANK_0::FIFO_CONFIG0, FIFO_CONFIG0_BIT::FIFO_DEPTH_8K_SET);

	// And then enable FIFO again
	RegisterSetAndClearBits(Register::BANK_0::FIFO_CONFIG0, FIFO_CONFIG0_BIT::FIFO_MODE_STOP_ON_FULL_SET,
				FIFO_CONFIG0_BIT::FIFO_MODE_STOP_ON_FULL_CLEAR);

	// And enable again
	RegisterSetBits(Register::BANK_0::FIFO_CONFIG3,
			FIFO_CONFIG3_BIT::FIFO_HIRES_EN |
			FIFO_CONFIG3_BIT::FIFO_GYRO_EN |
			FIFO_CONFIG3_BIT::FIFO_ACCEL_EN |
			FIFO_CONFIG3_BIT::FIFO_IF_EN);

	// Insert the 16-bit ODR timestamp into each FIFO frame (consumed by
	// UpdateMeasuredDt for the true per-sample dt).
	RegisterSetBits(Register::BANK_0::FIFO_CONFIG4,
			FIFO_CONFIG4_BIT::FIFO_TMST_FSYNC_EN);

	// Defensively re-apply the FIFO watermark: the bypass/re-enable cycle above may
	// clear the watermark comparison, and the threshold only takes effect when the
	// MSByte (FIFO_CONFIG1_1) is (re)written, LSByte first.
	RegisterWrite(Register::BANK_0::FIFO_CONFIG1_0, _fifo_watermark & 0xFF);
	RegisterWrite(Register::BANK_0::FIFO_CONFIG1_1, (_fifo_watermark >> 8) & 0xFF);
}

void ICM45686::ProcessAccel(const hrt_abstime &timestamp_sample, const FIFO::DATA fifo[], const uint8_t samples)
{
	sensor_accel_fifo_s accel{};
	accel.timestamp_sample = timestamp_sample;
	accel.samples = 0;
	accel.dt = _enable_clock_input ? FIFO_SAMPLE_DT : _fifo_measured_dt_us;

	for (int i = 0; i < samples; i++) {
		if (_enable_clock_input) {
			// Swapped as device is in little endian by default.
			const uint16_t timestamp_fifo = combine_uint(fifo[i].Timestamp_L, fifo[i].Timestamp_H);
			accel.dt = (float)timestamp_fifo * ((1.f / _input_clock_freq) * 1e6f);
		}

		// The 16-bit FIFO registers hold data[19:4] of the 20-bit hires packet, covering the
		// full +/-32 g range at 1024 LSB/g (scale set in Configure()). The 20-bit extension
		// nibble is intentionally unused so the published scale stays constant instead of
		// toggling with batch content.
		const int16_t accel_x = combine(fifo[i].ACCEL_DATA_XL, fifo[i].ACCEL_DATA_XH);
		const int16_t accel_y = combine(fifo[i].ACCEL_DATA_YL, fifo[i].ACCEL_DATA_YH);
		const int16_t accel_z = combine(fifo[i].ACCEL_DATA_ZL, fifo[i].ACCEL_DATA_ZH);

		// sample invalid if -32768 (16-bit truncation of the hires invalid marker -524288)
		if (accel_x != INT16_MIN && accel_y != INT16_MIN && accel_z != INT16_MIN) {
			accel.x[accel.samples] = accel_x;
			accel.y[accel.samples] = accel_y;
			accel.z[accel.samples] = accel_z;
			accel.samples++;
		}
	}

	// correct frame for publication
	for (int i = 0; i < accel.samples; i++) {
		// sensor's frame is +x forward, +y left, +z up
		//  flip y & z to publish right handed with z down (x forward, y right, z down)
		accel.x[i] = accel.x[i];
		accel.y[i] = (accel.y[i] == INT16_MIN) ? INT16_MAX : -accel.y[i];
		accel.z[i] = (accel.z[i] == INT16_MIN) ? INT16_MAX : -accel.z[i];
	}

	_px4_accel.set_error_count(perf_event_count(_bad_register_perf) + perf_event_count(_bad_transfer_perf) +
				   perf_event_count(_fifo_empty_perf) + perf_event_count(_fifo_overflow_perf));

	if (accel.samples > 0) {
		_px4_accel.updateFIFO(accel);
	}
}

void ICM45686::ProcessGyro(const hrt_abstime &timestamp_sample, const FIFO::DATA fifo[], const uint8_t samples)
{
	sensor_gyro_fifo_s gyro{};
	gyro.timestamp_sample = timestamp_sample;
	gyro.samples = 0;
	gyro.dt = _enable_clock_input ? FIFO_SAMPLE_DT : _fifo_measured_dt_us;

	for (int i = 0; i < samples; i++) {
		if (_enable_clock_input) {
			// Swapped as device is in little endian by default.
			const uint16_t timestamp_fifo = combine_uint(fifo[i].Timestamp_L, fifo[i].Timestamp_H);
			gyro.dt = (float)timestamp_fifo * ((1.f / _input_clock_freq) * 1e6f);
		}

		// The 16-bit FIFO registers hold data[19:4] of the 20-bit hires packet, covering the
		// full +/-4000 dps range (scale set in Configure()). The 20-bit extension nibble is
		// intentionally unused so the published scale stays constant instead of toggling with
		// batch content.
		const int16_t gx = combine(fifo[i].GYRO_DATA_XL, fifo[i].GYRO_DATA_XH);
		const int16_t gy = combine(fifo[i].GYRO_DATA_YL, fifo[i].GYRO_DATA_YH);
		const int16_t gz = combine(fifo[i].GYRO_DATA_ZL, fifo[i].GYRO_DATA_ZH);

		// RZ/V2H shared 3-IMU SPI bus corruption filter.
		// Under OpenAMP/AXI bus contention a corrupted read can still carry a VALID FIFO header
		// while its payload is garbage (measured on a stationary board: implausible full-/large-
		// scale gyro, up to ±32767). Upstream ProcessGyro published every sample (no per-sample
		// validity filter, unlike ProcessAccel), so the garbage reached the EKF and spun the
		// attitude estimate. Reject corrupt samples two ways:
		//   1. == INT16_MIN: the invalid marker (mirrors ProcessAccel); left in, the frame-flip
		//      below would remap it to +INT16_MAX (a full-scale +4000 dps spike).
		//   2. Rate-step: samples arrive at the fixed ODR, so the change from the previous good
		//      sample is an angular-acceleration proxy. GYRO_MAX_STEP is set ~1000x above any
		//      real airframe's angular acceleration, so a sustained high rate (aggressive flight)
		//      passes untouched — only an abrupt jump (corruption) trips it. This is why a
		//      rate-STEP filter is used rather than an absolute rate clip.
		// The reference (_last_gyro_*) is updated from accepted samples only, so a burst of
		// corrupt samples is compared against the last good value, not against garbage; an escape
		// hatch re-seeds after GYRO_REJECT_RUN_MAX consecutive rejects so a stale/bad reference
		// (e.g. a corrupt first sample) cannot lock the gyro stream out indefinitely.
		static constexpr int GYRO_MAX_STEP = 4096;       // 4096 cnt * 16/131 ~= 500 dps per ODR sample
		static constexpr int GYRO_REJECT_RUN_MAX = 4;

		if (gx == INT16_MIN || gy == INT16_MIN || gz == INT16_MIN) {
			perf_count(_gyro_spike_perf);
			continue;
		}

		if (_last_gyro_valid
		    && (abs((int)gx - _last_gyro_x) > GYRO_MAX_STEP
			|| abs((int)gy - _last_gyro_y) > GYRO_MAX_STEP
			|| abs((int)gz - _last_gyro_z) > GYRO_MAX_STEP)) {
			perf_count(_gyro_spike_perf);

			if (++_gyro_reject_run < GYRO_REJECT_RUN_MAX) {
				continue;
			}
			// else: reference is likely stale/corrupt — fall through and re-seed with this sample
		}

		_gyro_reject_run = 0;
		_last_gyro_x = gx; _last_gyro_y = gy; _last_gyro_z = gz;
		_last_gyro_valid = true;

		gyro.x[gyro.samples] = gx;
		gyro.y[gyro.samples] = gy;
		gyro.z[gyro.samples] = gz;
		gyro.samples++;
	}

	// correct frame for publication
	for (int i = 0; i < gyro.samples; i++) {
		// sensor's frame is +x forward, +y left, +z up
		//  flip y & z to publish right handed with z down (x forward, y right, z down)
		gyro.x[i] = gyro.x[i];
		gyro.y[i] = (gyro.y[i] == INT16_MIN) ? INT16_MAX : -gyro.y[i];
		gyro.z[i] = (gyro.z[i] == INT16_MIN) ? INT16_MAX : -gyro.z[i];
	}

	_px4_gyro.set_error_count(perf_event_count(_bad_register_perf) + perf_event_count(_bad_transfer_perf) +
				  perf_event_count(_fifo_empty_perf) + perf_event_count(_fifo_overflow_perf));

	if (gyro.samples > 0) {
		_px4_gyro.updateFIFO(gyro);
	}
}

bool ICM45686::ProcessTemperature(const FIFO::DATA fifo[], const uint8_t samples)
{
	int16_t temperature[FIFO_MAX_SAMPLES];
	float temperature_sum{0};

	int valid_samples = 0;

	for (int i = 0; i < samples; i++) {
		// Swapped as device is in little endian by default.
		const int16_t t = combine(fifo[i].TEMP_DATA_L, fifo[i].TEMP_DATA_H);

		// sample invalid if -32768
		if (t != -32768) {
			temperature_sum += t;
			temperature[valid_samples] = t;
			valid_samples++;
		}
	}

	if (valid_samples > 0) {
		const float temperature_avg = temperature_sum / valid_samples;

		for (int i = 0; i < valid_samples; i++) {
			// temperature changing wildly is an indication of a transfer error
			if (fabsf(temperature[i] - temperature_avg) > 1000) {
				perf_count(_bad_transfer_perf);
				return false;
			}
		}

		// use average temperature reading
		const float temp_c = (temperature_avg / TEMPERATURE_SENSITIVITY) + TEMPERATURE_OFFSET;

		if (PX4_ISFINITE(temp_c)) {
			_px4_accel.set_temperature(temp_c);
			_px4_gyro.set_temperature(temp_c);
			return true;

		} else {
			perf_count(_bad_transfer_perf);
		}
	}

	return false;
}
