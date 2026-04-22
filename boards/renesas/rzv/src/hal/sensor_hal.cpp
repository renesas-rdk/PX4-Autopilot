/****************************************************************************
 *
 * Copyright (c) 2025 Renesas Electronics Corporation and/or its affiliates
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 ****************************************************************************/

/*
 * Board-specific sensor HAL for Renesas RZ/V platforms.
 *
 * Provides DMA-safe buffer helpers and DRDY wiring helpers so that drivers
 * can stay close to the upstream PX4 sources.
 */


#if defined(__PX4_FREERTOS)

#include "sensor_hal.h"

#include <px4_platform_common/log.h>

#include <rzv_fsp/dma_buffer.h>

extern "C" int rzv_sensor_hal_configure_drdy(uint32_t pinset, bool risingedge, bool fallingedge,
		bool event, xcpt_t callback, void *arg);

sensor_hal_dma_buffer sensor_hal_dma_alloc(size_t size, size_t alignment)
{
	sensor_hal_dma_buffer buffer{};
	buffer.ptr = rzv_dma_alloc(size, alignment);

	if (buffer.ptr != nullptr) {
		buffer.size = size;
	}

	return buffer;
}

void sensor_hal_dma_free(sensor_hal_dma_buffer *buffer)
{
	if ((buffer != nullptr) && (buffer->ptr != nullptr)) {
		rzv_dma_free(buffer->ptr);
		buffer->ptr = nullptr;
		buffer->size = 0;
	}
}

bool sensor_hal_dma_is_safe(const void *ptr, size_t length)
{
	return rzv_dma_buffer_is_dma_safe(ptr, length);
}

int sensor_hal_configure_drdy(uint32_t pinset, bool risingedge, bool fallingedge,
			      bool event, xcpt_t callback, void *arg)
{
	return rzv_sensor_hal_configure_drdy(pinset, risingedge, fallingedge, event, callback, arg);
}

#endif /* __PX4_FREERTOS */
