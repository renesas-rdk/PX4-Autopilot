/****************************************************************************
 *
 * Copyright (c) 2025 Renesas Electronics Corporation and/or its affiliates
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 ****************************************************************************/

/**
 * @file rzv_device_registry.cpp
 * @brief Device registry for Renesas RZ/V2H FreeRTOS+FSP board shim
 */

#if defined(__PX4_FREERTOS)

#include "rzv_fsp/device_registry.h"
#include "../../../board/rzv_serial_config.h"
#define MODULE_NAME "rzv_fsp_registry"

#include <px4_platform_common/log.h>
#include <px4_platform_common/defines.h>
#include <errno.h>

#include "rzv_fsp/spi_fsp_backend.h"
#include "rzv_fsp/i2c_fsp_backend.h"
#include "rzv_fsp/uart_fsp_backend.h"
#include "rzv_fsp/uart_rc_ringbuffer.h"

int rzv_init_spi_cdevs(rzv_spi_backend_t *backend, uint8_t bus);
int rzv_init_i2c_cdev(rzv_i2c_backend_t *backend, uint8_t bus);
int rzv_init_uart_cdev(const char *devname, uint8_t logical_channel, rzv_uart_backend_t *backend);

namespace
{

rzv_spi_backend_t g_spi_backend{};
rzv_i2c_backend_t g_i2c_backend{};
rzv_uart_backend_t g_uart_rc_backend{};
rzv_uart_backend_t g_uart_mav_backend{};
rzv_uart_backend_t g_uart_gps_backend{};
rzv_uart_backend_t g_uart_lidar_backend{};

} // namespace

extern "C" int rzv_register_fsp_devices(void)
{
	int ret = PX4_OK;

	if (rzv_init_spi_cdevs(&g_spi_backend, g_spi_imu_cfg.channel) != 0) {
		ret = -EIO;
	}

	if (rzv_init_i2c_cdev(&g_i2c_backend, g_i2c_baro_cfg.channel) != 0) {
		ret = -EIO;
	}

    if (rzv_init_uart_cdev(RZV_RC_SERIAL_DEVICE, 0U, &g_uart_rc_backend) != 0) {
		PX4_WARN("Failed to register %s", RZV_RC_SERIAL_DEVICE);
        ret = -EIO;
    }

    if (rzv_init_uart_cdev(RZV_MAVLINK_SERIAL_DEVICE, 1U, &g_uart_mav_backend) != 0) {
        PX4_WARN("Failed to register %s", RZV_MAVLINK_SERIAL_DEVICE);
        ret = -EIO;
    }

    if (rzv_init_uart_cdev(RZV_GPS_SERIAL_DEVICE, 2U, &g_uart_gps_backend) != 0) {
        PX4_WARN("Failed to register %s", RZV_GPS_SERIAL_DEVICE);
        ret = -EIO;
    }

    if (rzv_init_uart_cdev(RZV_LIDAR_SERIAL_DEVICE, 3U, &g_uart_lidar_backend) != 0) {
        PX4_WARN("Failed to register %s", RZV_LIDAR_SERIAL_DEVICE);
        ret = -EIO;
    }

	return ret;
}

rzv_uart_backend_t *rzv_get_uart_backend(uint8_t logical_channel)
{
	switch (logical_channel) {
	case 0:
		return &g_uart_rc_backend;
	case 1:
		return &g_uart_mav_backend;
	case 2:
		return &g_uart_gps_backend;
	case 3:
		return &g_uart_lidar_backend;
	default:
		return nullptr;
	}
}

extern "C" int uart_open_channel(uint8_t channel, uint32_t baudrate, bool mode_8N1)
{
	(void)mode_8N1;
	rzv_uart_backend_t *backend = rzv_get_uart_backend(channel);

	if (backend == nullptr) {
		return -ENODEV;
	}

	if (!backend->opened) {
		if (rzv_uart_backend_init(backend, channel) != 0) {
			return -EIO;
		}
	}

	if ((baudrate != 0U) && (rzv_uart_backend_configure(backend, baudrate) != 0)) {
		return -EIO;
	}

	return 0;
}

extern "C" int uart_close_channel(uint8_t channel)
{
	rzv_uart_backend_t *backend = rzv_get_uart_backend(channel);

	if ((backend != nullptr) && backend->opened) {
		rzv_uart_backend_shutdown(backend);
	}

	return 0;
}

extern "C" int uart_read_channel(uint8_t channel, uint8_t *buffer, int max_len)
{
	if ((buffer == nullptr) || (max_len <= 0)) {
		return -EINVAL;
	}

	rzv_uart_backend_t *backend = rzv_get_uart_backend(channel);

	if ((backend == nullptr) || !backend->opened) {
		return -ENODEV;
	}

	int ret = rzv_uart_backend_read(backend, buffer, static_cast<size_t>(max_len));

	return (ret < 0) ? -EIO : ret;
}

extern "C" int uart_write_channel(uint8_t channel, const uint8_t *buffer, int len)
{
	if ((buffer == nullptr) || (len <= 0)) {
		return -EINVAL;
	}

	rzv_uart_backend_t *backend = rzv_get_uart_backend(channel);

	if ((backend == nullptr) || !backend->opened) {
		return -ENODEV;
	}

	int ret = rzv_uart_backend_write(backend, buffer, static_cast<size_t>(len));

	return (ret < 0) ? -EIO : ret;
}

extern "C" int uart_set_baudrate_channel(uint8_t channel, uint32_t baudrate)
{
	rzv_uart_backend_t *backend = rzv_get_uart_backend(channel);

	if ((backend == nullptr) || (baudrate == 0U)) {
		return -ENODEV;
	}

	if (!backend->opened) {
		if (rzv_uart_backend_init(backend, channel) != 0) {
			return -EIO;
		}
	}

	return (rzv_uart_backend_configure(backend, baudrate) == 0) ? 0 : -EIO;
}

extern "C" int uart_set_serial_mode_channel(uint8_t channel, bool mode_8n1)
{
	rzv_uart_backend_t *backend = rzv_get_uart_backend(channel);

	if (backend == nullptr) {
		return -ENODEV;
	}

	if (!backend->opened) {
		if (rzv_uart_backend_init(backend, channel) != 0) {
			return -EIO;
		}
	}

	backend->config.parity = mode_8n1 ? UART_PARITY_OFF : UART_PARITY_EVEN;
	backend->config.stop_bits = mode_8n1 ? UART_STOP_BITS_1 : UART_STOP_BITS_2;

	return (rzv_uart_backend_reopen(backend) == 0) ? 0 : -EIO;
}

extern "C" int uart_rc_buffer_available(uint8_t channel)
{
	return uart_rc_buffer_available_internal(channel);
}

extern "C" int uart_rc_buffer_read(uint8_t channel, uint8_t *buffer, int max_len)
{
	return uart_rc_buffer_read_internal(channel, buffer, max_len);
}

extern "C" void uart_rc_buffer_flush(uint8_t channel)
{
	uart_rc_buffer_flush_internal(channel);
}

#endif /* __PX4_FREERTOS */
