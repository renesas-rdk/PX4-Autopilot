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
 * - SPI Bus 0 (SPI_B0) - Internal IMUs
 *   - 3x ICM-45688 (driven by the upstream icm45686 driver) sharing the single
 *     SPI_B0 channel, each on its own SSL line:
 *       IMU#1 -> SSL0, IMU#2 -> SSL1, IMU#3 -> SSL2
 *   - SPI Mode: Mode 3 (CPOL=1, CPHA=1), max 24 MHz (driver default)
 *
 * Device-id / chip-select convention (matches upstream posix
 * spi_hw_description.h: devid low 16 bits = chip-select index, NOT devtype):
 * - devid = PX4_SPIDEV_ID(PX4_SPI_DEVICE_ID, n) with n = 1..3
 *   -> cdev path /dev/spidev0.n (rzv_spi_cdev.cpp), unique per IMU
 *   -> DeviceId.address = n, so the three identical devtypes get distinct
 *      uORB device ids (calibration slots stay stable per position)
 * - cs_gpio = n is a LOGICAL chip-select index (see board_config.h
 *   GPIO_PIN_MASK comment), used by the SPIBusIterator '-c' CLI filter;
 *   the FSP backend drives SSL line (n - 1) per transfer. It is NOT a GPIO.
 *
 * The actual SPI backend is provided by the Renesas FSP shim layer:
 * - rzv_spi_cdev.cpp: Character device interface (/dev/spidev0.{1,2,3})
 * - spi_fsp_backend.c: FSP hardware abstraction with:
 *   * Per-device SSL routing (SPCMD0.SSLA, idle-guarded)
 *   * Adaptive DMA (used for transfers >=64 bytes, 4-byte aligned)
 *   * DMA-safe bounce buffers (1024 bytes TX/RX)
 *   * Configurable CPOL/CPHA, dynamic bitrate
 *
 * Retired (driver still built, no board table entry / no rcS autostart):
 * - MPU9250 on SSL0 (previous board revision)
 */

#if defined(__PX4_FREERTOS)

#include <px4_arch/spi_hw_description.h>
#include <px4_platform_common/spi.h>
#include <drivers/drv_sensor.h>
#include "board_config.h"

#include <hal_data.h>

constexpr px4_spi_bus_t px4_spi_buses[SPI_BUS_MAX_BUS_ITEMS] = {
	{
		.devices = {
			// drdy_gpio = 0 for all three: the DRDY/INT1 approach is fully retired on
			// this board — all IMUs run POLLED (-P in config.txt). The former DRDY pads
			// have been repurposed (P50 -> SCI0 debug-console TXD0; PA0 -> SDHI0 VccQ on
			// the CA55/DTB side; P74 freed) and the FSP no longer configures any TINT
			// channel. Leaving the old pins here was a latent foot-gun: if -P were ever
			// dropped, the driver would try to drive/read a pin that is now a UART output
			// (P50) or owned by CA55 (PA0). Keep these 0 so the driver stays polled-only
			// regardless of the -P flag. The sensor's own INT1 output is also disabled in
			// the icm45686 driver register table (see ICM45686.hpp, __PX4_FREERTOS block).
			{
				// ICM-45688 IMU #1 (SSL0)
				.cs_gpio = 1,        // logical CS index -> SSL0
				.drdy_gpio = 0,      // polled (-P); P50 pad is now the SCI0 debug-console TXD0
				.devid = PX4_SPIDEV_ID(PX4_SPI_DEVICE_ID, 1),
				.devtype_driver = DRV_IMU_DEVTYPE_ICM45686,
			},
			{
				// ICM-45688 IMU #2 (SSL1)
				.cs_gpio = 2,        // logical CS index -> SSL1
				.drdy_gpio = 0,      // polled (-P); PA0 pad is now SDHI0 VccQ (CA55/DTB side)
				.devid = PX4_SPIDEV_ID(PX4_SPI_DEVICE_ID, 2),
				.devtype_driver = DRV_IMU_DEVTYPE_ICM45686,
			},
			{
				// ICM-45688 IMU #3 (SSL2)
				.cs_gpio = 3,        // logical CS index -> SSL2
				.drdy_gpio = 0,      // polled (-P); P74 pad freed (no TINT channel in FSP)
				.devid = PX4_SPIDEV_ID(PX4_SPI_DEVICE_ID, 3),
				.devtype_driver = DRV_IMU_DEVTYPE_ICM45686,
			},
		},
		.power_enable_gpio = 0,       // No software power control (always on)
		.bus = 0,                     // SPI_B0 (FSP instance)
		.is_external = false,         // Internal sensors
		.requires_locking = true,     // 3 devices share the channel (backend mutex serializes)
	},
};

#endif /* __PX4_FREERTOS */
