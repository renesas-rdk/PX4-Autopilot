/****************************************************************************
 *
 * Copyright (c) 2025 Renesas Electronics Corporation and/or its affiliates
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 ****************************************************************************/

#pragma once

#if defined(__PX4_FREERTOS)


#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

int rzv_uart_transport_map_device(const char *device, bool *mode_8n1, uint32_t *default_baud);
int rzv_uart_transport_read_channel(uint8_t channel, uint8_t *buffer, int max_len);
int rzv_uart_transport_write_channel(uint8_t channel, const uint8_t *buffer, int len);
int rzv_uart_transport_set_baudrate_channel(uint8_t channel, uint32_t baudrate);
int rzv_uart_transport_wait_for_data(uint8_t channel, int timeout_ms);
int rzv_uart_transport_read_fast(uint8_t channel, uint8_t *buffer, int max_len);
int rzv_uart_transport_read_with_policy(uint8_t channel, uint8_t *buffer, int max_len);
int rzv_uart_transport_available(uint8_t channel);
void rzv_uart_transport_flush(uint8_t channel);
int rzv_uart_transport_get_extended_stats(uint8_t channel, uint32_t *rx_count,
					      uint32_t *overflow_count,
					      uint32_t *isr_writes,
					      uint32_t *app_reads);
int rzv_uart_transport_get_indices(uint8_t channel, uint32_t *widx, uint32_t *ridx);
void rzv_uart_transport_debug_isr_stats(void);

#ifdef __cplusplus
}

#endif /* __PX4_FREERTOS */

#endif
