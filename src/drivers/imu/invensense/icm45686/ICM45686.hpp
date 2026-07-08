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

/**
 * @file ICM45686.hpp
 *
 * Driver for the Invensense ICM45686 connected via SPI.
 *
 */

#pragma once

#include "InvenSense_ICM45686_registers.hpp"

#include <drivers/drv_hrt.h>
#include <lib/drivers/accelerometer/PX4Accelerometer.hpp>
#include <lib/drivers/device/spi.h>
#include <lib/drivers/gyroscope/PX4Gyroscope.hpp>
#include <lib/geo/geo.h>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/atomic.h>
#include <px4_platform_common/i2c_spi_buses.h>

using namespace InvenSense_ICM45686;

class ICM45686 : public device::SPI, public I2CSPIDriver<ICM45686>
{
public:
	ICM45686(const I2CSPIDriverConfig &config);
	~ICM45686() override;

	static void print_usage();

	void RunImpl();

	int init() override;
	void print_status() override;

private:
	void exit_and_cleanup() override;

	// Sensor Configuration
#if defined(__PX4_FREERTOS)
	// RZ/V2H: 800 Hz ODR shrinks the per-read SPI/DMAC transfer 641B->81B so the byte-wise
	// DMAC read finishes before AXI-fabric contention stalls it (mirrors the MPU9250 RZ/V
	// low-ODR port). Paired with the 50 Hz UI anti-alias LPF set in Configure().
	static constexpr float FIFO_SAMPLE_DT{1e6f / 800.f};
#else
	static constexpr float FIFO_SAMPLE_DT{1e6f / 6400.f};     // 6400 Hz accel & gyro ODR configured
#endif
	static constexpr float GYRO_RATE{1e6f / FIFO_SAMPLE_DT};
	static constexpr float ACCEL_RATE{1e6f / FIFO_SAMPLE_DT};

	static constexpr float FIFO_TIMESTAMP_SCALING{16.f *(32.f / 30.f)}; // Used when not using clock input

	// maximum FIFO samples per transfer is limited to the size of sensor_accel_fifo/sensor_gyro_fifo
	static constexpr int32_t FIFO_MAX_SAMPLES{math::min(FIFO::SIZE / sizeof(FIFO::DATA), sizeof(sensor_gyro_fifo_s::x) / sizeof(sensor_gyro_fifo_s::x[0]), sizeof(sensor_accel_fifo_s::x) / sizeof(sensor_accel_fifo_s::x[0]) * (int)(GYRO_RATE / ACCEL_RATE))};

	// Transfer data
	struct FIFOTransferBuffer {
		uint8_t cmd{static_cast<uint8_t>(Register::BANK_0::FIFO_DATA) | DIR_READ};
		FIFO::DATA f[FIFO_MAX_SAMPLES] {};
	} __attribute__((packed));
	// ensure padding is right
	static_assert(sizeof(FIFOTransferBuffer) == (1 + FIFO_MAX_SAMPLES *sizeof(FIFO::DATA)));

	struct register_bank0_config_t {
		Register::BANK_0 reg;
		uint8_t set_bits{0};
		uint8_t clear_bits{0};
	};

	int probe() override;

	bool Reset();

	bool Configure();
	void ConfigureSampleRate(int sample_rate);
	void ConfigureFIFOWatermark(uint8_t samples);
	void ConfigureCLKIN();

	template <typename T> bool RegisterCheck(const T &reg_cfg);
	template <typename T> uint8_t RegisterRead(T reg);
	template <typename T> void RegisterWrite(T reg, uint8_t value);
	template <typename T> void RegisterSetAndClearBits(T reg, uint8_t setbits, uint8_t clearbits);
	template <typename T> void RegisterSetBits(T reg, uint8_t setbits) { RegisterSetAndClearBits(reg, setbits, 0); }
	template <typename T> void RegisterClearBits(T reg, uint8_t clearbits) { RegisterSetAndClearBits(reg, 0, clearbits); }

	static int DataReadyInterruptCallback(int irq, void *context, void *arg);
	void DataReady();
	bool DataReadyInterruptConfigure();
	bool DataReadyInterruptDisable();

	uint16_t FIFOReadCount();
	bool FIFORead(const hrt_abstime &timestamp_sample);
	void FIFOReset();

	void ProcessAccel(const hrt_abstime &timestamp_sample, const FIFO::DATA fifo[], const uint8_t samples);
	void ProcessGyro(const hrt_abstime &timestamp_sample, const FIFO::DATA fifo[], const uint8_t samples);
	bool ProcessTemperature(const FIFO::DATA fifo[], const uint8_t samples);
	void UpdateMeasuredDt(const FIFO::DATA fifo[], const uint8_t samples);

	const spi_drdy_gpio_t _drdy_gpio;

	PX4Accelerometer _px4_accel;
	PX4Gyroscope _px4_gyro;

	perf_counter_t _bad_register_perf{perf_alloc(PC_COUNT, MODULE_NAME": bad register")};
	perf_counter_t _bad_transfer_perf{perf_alloc(PC_COUNT, MODULE_NAME": bad transfer")};
	perf_counter_t _fifo_empty_perf{perf_alloc(PC_COUNT, MODULE_NAME": FIFO empty")};
	perf_counter_t _fifo_overflow_perf{perf_alloc(PC_COUNT, MODULE_NAME": FIFO overflow")};
	perf_counter_t _fifo_reset_perf{perf_alloc(PC_COUNT, MODULE_NAME": FIFO reset")};
	perf_counter_t _drdy_missed_perf{nullptr};
	perf_counter_t _gyro_spike_perf{perf_alloc(PC_COUNT, MODULE_NAME": gyro spike rejected")};

	// RZ/V2H shared 3-IMU SPI bus: previous good gyro sample (counts), kept across FIFO reads
	// so the spike filter in ProcessGyro can reject bus-corruption by inter-sample jump.
	int16_t _last_gyro_x{0}, _last_gyro_y{0}, _last_gyro_z{0};
	bool _last_gyro_valid{false};
	uint8_t _gyro_reject_run{0}; // consecutive rate-step rejects (escape hatch re-seeds the reference)

	px4::atomic<hrt_abstime> _drdy_timestamp_sample{0};

	hrt_abstime _reset_timestamp{0};
	hrt_abstime _last_config_check_timestamp{0};
	hrt_abstime _last_regcheck_log_timestamp{0};  // rate-limit RegisterCheck mismatch prints (see RegisterCheck)
	hrt_abstime _temperature_update_timestamp{0};
	int _failure_count{0};

	bool _enable_clock_input{false};
	float _input_clock_freq{0.f};

	// Per-sample interval measured from the FIFO timestamp field (internal
	// clock, FIFO_TIMESTAMP_SCALING) — low-pass filtered; replaces the nominal
	// FIFO_SAMPLE_DT so polled-mode batches carry the true ODR period.
	float _fifo_measured_dt_us{FIFO_SAMPLE_DT};
	float _tmst_res_us{0.f};        // detected tick resolution (us/LSB), 0 until locked
	uint16_t _tmst_last_raw{0};     // last raw TMST field value (debug/status)
	uint8_t _tmst_mode{0}; // 0 = nominal (TMST unusable), 1 = direct (delta ticks), 2 = counter diff

	bool _data_ready_interrupt_enabled{false};

	enum class STATE : uint8_t {
		RESET,
		WAIT_FOR_RESET,
		CONFIGURE,
		FIFO_RESET,
		FIFO_READ,
	} _state{STATE::RESET};

	uint16_t _fifo_empty_interval_us{1250}; // default 1250 us / 800 Hz transfer interval
	int32_t _fifo_gyro_samples{static_cast<int32_t>(_fifo_empty_interval_us / (1000000 / GYRO_RATE))};
	uint16_t _fifo_watermark{0}; // FIFO_THS watermark in records, force-written in Configure()

	uint8_t _checked_register_bank0{0};
	uint8_t _register_check_fail_count{0};  // consecutive RegisterCheck fails → re-write in place; Reset() only after many (RZ/V2H boot P50/INT1 read-artifact, see RunImpl)
	static constexpr uint8_t size_register_bank0_cfg{13};
	register_bank0_config_t _register_bank0_cfg[size_register_bank0_cfg] {
#if defined(__PX4_FREERTOS)
		// RZ/V2H runs all three IMUs POLLED (-P): the sensor INT1 pin is never used
		// (DataReadyInterruptConfigure() returns false when _drdy_gpio == 0, so the
		// polled FIFO path never reads INT1_STATUS). Leave INT1 disabled and high-Z.
		//
		// This is critical, not just tidy: IMU#1's INT1 net is the same physical pad
		// as the SCI0 debug-console TXD0 (P50, freed from DRDY once all IMUs went
		// polled). If the sensor drives INT1 push-pull it fights the UART output on
		// that net -- the contention corrupts SPI reads (the FIFO_CONFIG0 verify reads
		// back 0x3E = 0x9F<<1 instead of 0x9F), RegisterCheck fails, and RunImpl's
		// check-fail branch calls Reset() every cycle -> endless reconfigure storm with
		// ~50-100 ms gyro/accel blackouts (IMU produces no usable data; sometimes the
		// initial Configure() fails outright and the driver never enters the work queue).
		// Route no source to INT1 (INT1_CONFIG0 = 0) and set open-drain (INT1_DRIVE = 1)
		// so the pin stays high-Z and P50 belongs solely to the UART.
		{ Register::BANK_0::INT1_CONFIG0, 0, 0xFF },
		{ Register::BANK_0::INT1_CONFIG2, INT1_CONFIG2_BIT::INT1_DRIVE, INT1_CONFIG2_BIT::INT1_MODE | INT1_CONFIG2_BIT::INT1_POLARITY },
#else
		// route only the FIFO threshold (watermark) interrupt to INT1
		{ Register::BANK_0::INT1_CONFIG0, INT1_CONFIG0_BIT::INT1_STATUS_EN_FIFO_THS, (uint8_t)~INT1_CONFIG0_BIT::INT1_STATUS_EN_FIFO_THS },
		// INT1: push-pull, LATCHED, active low. RZV TINT edge-detector misses the
		// default short INT1 *pulse* (~0.01% caught), so use a latched level that holds
		// asserted until cleared; the driver reads INT1_STATUS0 after each FIFO read to
		// re-arm. (Set INT1_MODE = latched; clear INT1_DRIVE = push-pull, INT1_POLARITY = active low.)
		{ Register::BANK_0::INT1_CONFIG2, INT1_CONFIG2_BIT::INT1_MODE, INT1_CONFIG2_BIT::INT1_DRIVE | INT1_CONFIG2_BIT::INT1_POLARITY },
#endif
		{ Register::BANK_0::PWR_MGMT0, PWR_MGMT0_BIT::GYRO_MODE_LOW_NOISE | PWR_MGMT0_BIT::ACCEL_MODE_LOW_NOISE, 0 },

#if defined(__PX4_FREERTOS)
		// RZ/V2H: 800 Hz ODR (see FIFO_SAMPLE_DT) to keep the SPI/DMAC read small.
		{ Register::BANK_0::GYRO_CONFIG0, GYRO_CONFIG0_BIT::GYRO_UI_FS_SEL_4000_DPS_SET | GYRO_CONFIG0_BIT::GYRO_ODR_800_HZ_SET, GYRO_CONFIG0_BIT::GYRO_UI_FS_SEL_4000_DPS_CLEAR | GYRO_CONFIG0_BIT::GYRO_ODR_800_HZ_CLEAR },
		{ Register::BANK_0::ACCEL_CONFIG0, ACCEL_CONFIG0_BIT::ACCEL_UI_FS_SEL_32_G_SET | ACCEL_CONFIG0_BIT::ACCEL_ODR_800_HZ_SET, ACCEL_CONFIG0_BIT::ACCEL_UI_FS_SEL_32_G_CLEAR | ACCEL_CONFIG0_BIT::ACCEL_ODR_800_HZ_CLEAR },
#else
		{ Register::BANK_0::GYRO_CONFIG0, GYRO_CONFIG0_BIT::GYRO_UI_FS_SEL_4000_DPS_SET | GYRO_CONFIG0_BIT::GYRO_ODR_6400_HZ_SET, GYRO_CONFIG0_BIT::GYRO_UI_FS_SEL_4000_DPS_CLEAR | GYRO_CONFIG0_BIT::GYRO_ODR_6400_HZ_CLEAR },
		{ Register::BANK_0::ACCEL_CONFIG0, ACCEL_CONFIG0_BIT::ACCEL_UI_FS_SEL_32_G_SET | ACCEL_CONFIG0_BIT::ACCEL_ODR_6400_HZ_SET, ACCEL_CONFIG0_BIT::ACCEL_UI_FS_SEL_32_G_CLEAR | ACCEL_CONFIG0_BIT::ACCEL_ODR_6400_HZ_CLEAR },
#endif
		{ Register::BANK_0::FIFO_CONFIG4, 0, FIFO_CONFIG4_BIT::FIFO_COMP_EN },
		{ Register::BANK_0::FIFO_CONFIG0, FIFO_CONFIG0_BIT::FIFO_MODE_STOP_ON_FULL_SET | FIFO_CONFIG0_BIT::FIFO_DEPTH_8K_SET, FIFO_CONFIG0_BIT::FIFO_MODE_STOP_ON_FULL_CLEAR | FIFO_CONFIG0_BIT::FIFO_DEPTH_8K_CLEAR },
		// FIFO watermark threshold (FIFO_WM[15:0], in records), populated at runtime by
		// ConfigureFIFOWatermark(). These two entries MUST exist in the table: without
		// them the watermark is never written to the chip, the FIFO_THS interrupt routed
		// to INT1 never asserts, and every FIFO read falls back to the watchdog timer
		// (DRDY missed ~100%). (The matching omission is present in upstream PX4 PR#27663.)
		{ Register::BANK_0::FIFO_CONFIG1_0, 0, 0 }, // FIFO_WM[7:0]  set at runtime
		{ Register::BANK_0::FIFO_CONFIG1_1, 0, 0 }, // FIFO_WM[15:8] set at runtime
		// Compare FIFO count "greater than" the watermark (sustained level while the FIFO
		// holds >= WM records) rather than the default "equal" (true only the instant
		// count == WM). Without this the FIFO_THS status never latches and INT1 never
		// asserts. (icm42688p sets the equivalent FIFO_WM_GT_TH bit for the same reason.)
		{ Register::BANK_0::FIFO_CONFIG2, FIFO_CONFIG2_BIT::FIFO_WR_WM_GT_TH_GREATER_THAN, 0 },
		{ Register::BANK_0::FIFO_CONFIG3, FIFO_CONFIG3_BIT::FIFO_HIRES_EN | FIFO_CONFIG3_BIT::FIFO_GYRO_EN | FIFO_CONFIG3_BIT::FIFO_ACCEL_EN | FIFO_CONFIG3_BIT::FIFO_IF_EN, 0 },

		{ Register::BANK_0::RTC_CONFIG, 0, 0}, // RTC_MODE[5] set at runtime
		{ Register::BANK_0::IOC_PAD_SCENARIO_OVRD, 0, 0}, // PADS_INT2_CFG_OVRD and PADS_INT2_CFG_OVRD_VAL set at runtime
	};
};
