/****************************************************************************
 *
 * Copyright (c) 2025 Renesas Electronics Corporation and/or its affiliates
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 ****************************************************************************/

/**
 * @file spi.cpp
 * @brief SPI bus description for Renesas RZ/V2H
 *
 * Hardware Configuration:
 * - SPI Bus 0 (SPI_B0) - Internal IMU
 *   - MPU9250: 9-axis IMU (Gyro + Accel + Magnetometer)
 *     - DRDY GPIO: Port 5, Pin 0 (BSP_IO_PORT_05_PIN_00)
 *     - CS: Controlled by FSP (SPI_B SSL0)
 *     - Max frequency: 20 MHz (IMU datasheet limit)
 *     - SPI Mode: Mode 3 (CPOL=1, CPHA=1)
 *
 * The actual SPI backend is provided by the Renesas FSP shim layer:
 * - rzv_spi_cdev.cpp: Character device interface (/dev/spidev0.0)
 * - spi_fsp_backend.c: FSP hardware abstraction with:
 *   * Adaptive DMA (used for transfers ≥64 bytes, 4-byte aligned)
 *   * DMA-safe bounce buffers (1024 bytes TX/RX)
 *   * Configurable CPOL/CPHA via ioctl
 *   * Dynamic bitrate calculation
 *
 * Hardware Version Support:
 * - RZV_V1: Current board with MPU9250
 * - Future: Can add different IMU variants (e.g., ICM42688P, BMI088)
 */

#if defined(__PX4_FREERTOS)

#include <px4_arch/spi_hw_description.h>
#include <px4_platform_common/spi.h>
#include <drivers/drv_sensor.h>
#include "board_config.h"

#include <hal_data.h>

/**
 * SPI Bus Configuration
 *
 * Maps PX4 logical SPI bus 0 to the MPU9250 IMU device.
 * The FSP configuration handles actual pin mapping and SPI mode settings.
 */
constexpr px4_spi_bus_t px4_spi_buses[SPI_BUS_MAX_BUS_ITEMS] = {
	{
		.devices = {
			{
				// MPU9250 9-axis IMU (Gyro + Accel + Mag)
				.cs_gpio = 0,  // CS managed by FSP (SPI_B SSL0)
				.drdy_gpio = BSP_IO_PORT_05_PIN_00,  // Data Ready: Port 5, Pin 0
				.devid = PX4_SPIDEV_ID(PX4_SPI_DEVICE_ID, DRV_IMU_DEVTYPE_MPU9250),
				.devtype_driver = DRV_IMU_DEVTYPE_MPU9250,
			},
			// Future: Additional SPI devices can be added here
			// Example: Flash memory, additional IMU, etc.
		},
		.power_enable_gpio = 0,       // No software power control (always on)
		.bus = 0,                     // SPI_B0 (FSP instance)
		.is_external = false,         // Internal sensor
		.requires_locking = false,    // Single device, no multi-device arbitration
	},
};

#endif /* __PX4_FREERTOS */
