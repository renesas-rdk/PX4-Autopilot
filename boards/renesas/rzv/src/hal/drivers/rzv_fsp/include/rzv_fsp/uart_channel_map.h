/****************************************************************************
 *
 * Copyright (c) 2025 Renesas Electronics Corporation and/or its affiliates
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 ****************************************************************************/

/**
 * @file uart_channel_map.h
 * @brief Shared mapping between UART device paths and logical channels.
 */

#pragma once

#if defined(__PX4_FREERTOS)


#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
	const char *device;       /**< Device node name (e.g. /dev/ttyS1) */
	uint8_t channel;          /**< Logical UART channel index */
	uint32_t default_baud;    /**< Default baudrate for this device */
	bool mode_8n1;            /**< true if default mode is 8N1, false for 8E2 */
} uart_channel_info_t;

extern const uart_channel_info_t g_uart_channel_table[];
extern const size_t g_uart_channel_table_size;

/**
 * @brief Lookup channel information by device path.
 * @param device Device node string (e.g. "/dev/ttyS1")
 * @return Pointer to channel info, or NULL if not found
 */
const uart_channel_info_t *uart_channel_get_info_by_device(const char *device);

/**
 * @brief Lookup channel information by logical channel index.
 * @param channel Logical channel number
 * @return Pointer to channel info, or NULL if not found
 */
const uart_channel_info_t *uart_channel_get_info_by_channel(uint8_t channel);

/**
 * @brief Helper to map device path to logical channel while retrieving defaults.
 * @param device Device node string
 * @param mode_8n1 Output pointer for default word format (optional)
 * @param default_baud Output pointer for default baud (optional)
 * @return Channel index on success, -1 on failure
 */
int uart_map_device(const char *device, bool *mode_8n1, uint32_t *default_baud);

#ifdef __cplusplus
}

#endif /* __PX4_FREERTOS */

#endif
