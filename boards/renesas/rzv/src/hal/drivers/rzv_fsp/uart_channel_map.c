/****************************************************************************
 *
 * Copyright (c) 2025 Renesas Electronics Corporation and/or its affiliates
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 ****************************************************************************/

/**
 * @file uart_channel_map.c
 * @brief UART channel mapping for Renesas RZ/V2H
 */


#if defined(__PX4_FREERTOS)

#include "rzv_fsp/uart_channel_map.h"
#include "../../../board/rzv_serial_config.h"

#include <string.h>

const uart_channel_info_t g_uart_channel_table[] = {
	{RZV_RC_SERIAL_DEVICE, 0U, 100000U, false},  /* RC input (SBUS default 8E2) */
	{RZV_MAVLINK_SERIAL_DEVICE, 1U, 57600U,  true},   /* MAVLink telemetry */
	{RZV_GPS_SERIAL_DEVICE, 2U, 115200U,   true},   /* GPS (Ublox M10) */
	{RZV_LIDAR_SERIAL_DEVICE, 3U, 115200U,  true},   /* TFmini Plus */
};

const size_t g_uart_channel_table_size = sizeof(g_uart_channel_table) / sizeof(g_uart_channel_table[0]);

const uart_channel_info_t *uart_channel_get_info_by_device(const char *device)
{
	if (device == NULL) {
		return NULL;
	}

	for (size_t i = 0; i < g_uart_channel_table_size; i++) {
		if (strcmp(device, g_uart_channel_table[i].device) == 0) {
			return &g_uart_channel_table[i];
		}
	}

	return NULL;
}

const uart_channel_info_t *uart_channel_get_info_by_channel(uint8_t channel)
{
	for (size_t i = 0; i < g_uart_channel_table_size; i++) {
		if (g_uart_channel_table[i].channel == channel) {
			return &g_uart_channel_table[i];
		}
	}

	return NULL;
}

int uart_map_device(const char *device, bool *mode_8n1, uint32_t *default_baud)
{
	const uart_channel_info_t *info = uart_channel_get_info_by_device(device);

	if (info == NULL) {
		return -1;
	}

	if (mode_8n1 != NULL) {
		*mode_8n1 = info->mode_8n1;
	}

	if (default_baud != NULL) {
		*default_baud = info->default_baud;
	}

	return (int)info->channel;
}

#endif /* __PX4_FREERTOS */
