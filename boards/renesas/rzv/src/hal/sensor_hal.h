/****************************************************************************
 *
 * Copyright (c) 2025 Renesas Electronics Corporation and/or its affiliates
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 ****************************************************************************/

/*
 * Board-specific sensor HAL for Renesas RZ/V
 *
 * Centralizes helpers (DMA buffers, DRDY wiring, watchdog utilities)
 * so IMU/other sensor drivers can stay close to upstream PX4 sources.
 */

#pragma once

#if defined(__PX4_FREERTOS)


#include <cstddef>
#include <cstdint>

#include <px4_platform_common/px4_config.h>

#ifdef __cplusplus
extern "C" {
#endif

struct sensor_hal_dma_buffer {
	void *ptr;
	size_t size;
};

/**
 * Allocate a DMA-safe buffer.
 *
 * @param size          Bytes requested.
 * @param alignment     Required alignment (power of two).
 * @return              Handle describing the buffer. ptr==nullptr on failure.
 */
sensor_hal_dma_buffer sensor_hal_dma_alloc(size_t size, size_t alignment);

/**
 * Release a buffer previously obtained via sensor_hal_dma_alloc.
 */
void sensor_hal_dma_free(sensor_hal_dma_buffer *buffer);

/**
 * Check if a buffer resides inside a DMA-safe region.
 */
bool sensor_hal_dma_is_safe(const void *buffer, size_t length);

/**
 * Configure the DRDY interrupt for a sensor pin.
 *
 * Internally delegates to px4_arch_gpiosetevent so existing drivers retain
 * their behaviour. Returns 0 on success, negative errno on failure.
 */
int sensor_hal_configure_drdy(uint32_t pinset, bool risingedge, bool fallingedge,
			      bool event, xcpt_t callback, void *arg);

#ifdef __cplusplus
}

#endif /* __PX4_FREERTOS */

#endif
