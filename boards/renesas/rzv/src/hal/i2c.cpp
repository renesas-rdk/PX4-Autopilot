/****************************************************************************
 *
 * Copyright (c) 2025 Renesas Electronics Corporation and/or its affiliates
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 ****************************************************************************/

/**
 * @file i2c.cpp
 * @brief I2C bus description for Renesas RZ/V2H
 *
 * Hardware Configuration:
 * - I2C Bus 7 (SCI7 in I2C mode) - Internal sensors
 *   - BMP280 barometer at address 0x76 (defined in board_config.h)
 *   - Future: Additional sensors can be added via FSP configuration
 *
 * The actual I2C backend is provided by the Renesas FSP shim layer:
 * - rzv_i2c_cdev.cpp: Character device interface
 * - i2c_fsp_backend.c: FSP hardware abstraction with:
 *   * Automatic retry with speed fallback (1MHz → 400kHz → 100kHz)
 *   * Per-device speed tracking (future: auto-restore feature)
 *   * Bus recovery via SCL toggle
 *   * DMA-safe bounce buffers (256 bytes)
 *
 * Device Detection:
 * PX4 drivers probe devices at runtime via character device (/dev/i2c-7).
 * No compile-time device list needed for this architecture.
 */

#if defined(__PX4_FREERTOS)

#include <px4_arch/i2c_hw_description.h>
#include "board_config.h"

/**
 * I2C Bus Configuration
 *
 * RZV board uses FSP-managed I2C, so we only define the bus number.
 * Actual device addresses are probed at runtime by PX4 drivers.
 *
 * Future Enhancement: When PX4 adds support for per-device config on
 * FreeRTOS platforms, add device list here for compile-time validation.
 */
constexpr px4_i2c_bus_t px4_i2c_buses[I2C_BUS_MAX_BUS_ITEMS] = {
	// I2C Bus 7: Internal sensor bus (SCI7 in I2C master mode)
	// - Supports standard (100kHz), fast (400kHz), fast+ (1MHz)
	// - BMP280 barometer at 0x76
	initI2CBusInternal(7),
};

#endif /* __PX4_FREERTOS */
