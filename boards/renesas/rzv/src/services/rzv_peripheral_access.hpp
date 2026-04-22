/****************************************************************************
 *
 * Copyright (c) 2025 Renesas Electronics Corporation and/or its affiliates
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 ****************************************************************************/

/**
 * @file rzv_peripheral_access.hpp
 * @brief Peripheral access definitions for Renesas RZ/V2H
 */
#pragma once

#if defined(__PX4_FREERTOS)


#include <cstddef>
#include <cstdint>

#include "hal_data.h"

namespace px4::board::rzv
{

// -----------------------------------------------------------------------------
// I2C bindings
// -----------------------------------------------------------------------------

struct I2cBusBinding {
	uint8_t bus;
	const i2c_master_instance_t *instance;
};

inline constexpr I2cBusBinding i2c_bus_bindings[] = {
	{7, &g_i2c_baro},
};

inline constexpr size_t i2c_bus_count()
{
	return sizeof(i2c_bus_bindings) / sizeof(i2c_bus_bindings[0]);
}

inline const I2cBusBinding *find_i2c_binding(uint8_t bus)
{
	for (size_t i = 0; i < i2c_bus_count(); i++) {
		if (i2c_bus_bindings[i].bus == bus) {
			return &i2c_bus_bindings[i];
		}
	}

	return nullptr;
}

inline i2c_master_ctrl_t *i2c_ctrl(uint8_t bus)
{
	const I2cBusBinding *binding = find_i2c_binding(bus);

	if ((binding != nullptr) && (binding->instance != nullptr)) {
		return reinterpret_cast<i2c_master_ctrl_t *>(binding->instance->p_ctrl);
	}

	return nullptr;
}

// -----------------------------------------------------------------------------
// SPI accessors
// -----------------------------------------------------------------------------

inline spi_b_instance_ctrl_t *spi_ctrl()
{
	return &g_spi_imu_ctrl;
}

inline const spi_cfg_t *spi_config()
{
	return &g_spi_imu_cfg;
}

inline const spi_instance_t *spi_instance()
{
	return &g_spi_imu;
}

// -----------------------------------------------------------------------------
// UART accessors (optimized with constexpr lookup)
// -----------------------------------------------------------------------------

/**
 * UART Channel Bindings
 *
 * Using constexpr lookup table instead of runtime switch for zero overhead.
 * Compiler can optimize the lookup into direct access.
 */
struct UartBinding {
	uint8_t channel;
	const uart_instance_t *instance;
	sci_b_uart_instance_ctrl_t *ctrl;
};

inline constexpr UartBinding uart_bindings[] = {
	{0, &g_uart_rc,    &g_uart_rc_ctrl},     // RC input (SBUS)
	{1, &g_uart_mav,   &g_uart_mav_ctrl},    // MAVLink
	{2, &g_uart_gps,   &g_uart_gps_ctrl},    // GPS
	{3, &g_uart_lidar, &g_uart_lidar_ctrl},  // LIDAR
};

inline constexpr size_t uart_channel_count()
{
	return sizeof(uart_bindings) / sizeof(uart_bindings[0]);
}

inline const UartBinding *find_uart_binding(uint8_t channel)
{
	for (size_t i = 0; i < uart_channel_count(); i++) {
		if (uart_bindings[i].channel == channel) {
			return &uart_bindings[i];
		}
	}

	return nullptr;
}

/**
 * Get UART control structure for a given channel
 *
 * @param channel UART channel number (0-3)
 * @return Pointer to control structure, or nullptr if invalid
 */
inline sci_b_uart_instance_ctrl_t *uart_ctrl(uint8_t channel)
{
	const UartBinding *binding = find_uart_binding(channel);
	return (binding != nullptr) ? binding->ctrl : nullptr;
}

/**
 * Get UART instance for a given channel
 *
 * @param channel UART channel number (0-3)
 * @return Pointer to UART instance, or nullptr if invalid
 */
inline const uart_instance_t *uart_instance(uint8_t channel)
{
	const UartBinding *binding = find_uart_binding(channel);
	return (binding != nullptr) ? binding->instance : nullptr;
}

} // namespace px4::board::rzv

#endif /* __PX4_FREERTOS */
