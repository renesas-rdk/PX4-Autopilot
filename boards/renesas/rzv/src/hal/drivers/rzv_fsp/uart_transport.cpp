/****************************************************************************
 *
 * Copyright (c) 2025 Renesas Electronics Corporation and/or its affiliates
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 ****************************************************************************/


#if defined(__PX4_FREERTOS)

#include "rzv_fsp/uart_transport.h"

#include "rzv_fsp/uart_channel_map.h"
#include "rzv_fsp/uart_fsp_backend.h"
#include "rzv_fsp/uart_rc_ringbuffer.h"

#define RZV_UART_TRANSPORT_RC_CHANNEL 0U

extern "C" {
int uart_read_channel(uint8_t channel, uint8_t *buffer, int max_len);
int uart_write_channel(uint8_t channel, const uint8_t *buffer, int len);
int uart_set_baudrate_channel(uint8_t channel, uint32_t baudrate);
int uart_rc_buffer_read(uint8_t channel, uint8_t *buffer, int max_len);
int uart_rc_buffer_available(uint8_t channel);
void uart_rc_buffer_flush(uint8_t channel);
}

extern "C" int rzv_uart_transport_map_device(const char *device, bool *mode_8n1, uint32_t *default_baud)
{
	return uart_map_device(device, mode_8n1, default_baud);
}

extern "C" int rzv_uart_transport_read_channel(uint8_t channel, uint8_t *buffer, int max_len)
{
	return uart_read_channel(channel, buffer, max_len);
}

extern "C" int rzv_uart_transport_write_channel(uint8_t channel, const uint8_t *buffer, int len)
{
	return uart_write_channel(channel, buffer, len);
}

extern "C" int rzv_uart_transport_set_baudrate_channel(uint8_t channel, uint32_t baudrate)
{
	return uart_set_baudrate_channel(channel, baudrate);
}

extern "C" int rzv_uart_transport_wait_for_data(uint8_t channel, int timeout_ms)
{
	return uart_rc_buffer_wait_for_data(channel, timeout_ms);
}

extern "C" int rzv_uart_transport_read_fast(uint8_t channel, uint8_t *buffer, int max_len)
{
	return uart_rc_buffer_read(channel, buffer, max_len);
}

extern "C" int rzv_uart_transport_read_with_policy(uint8_t channel, uint8_t *buffer, int max_len)
{
	if (channel == RZV_UART_TRANSPORT_RC_CHANNEL) {
		return uart_rc_buffer_read(channel, buffer, max_len);
	}

	return uart_read_channel(channel, buffer, max_len);
}

extern "C" int rzv_uart_transport_available(uint8_t channel)
{
	return uart_rc_buffer_available(channel);
}

extern "C" void rzv_uart_transport_flush(uint8_t channel)
{
	uart_rc_buffer_flush(channel);
}

extern "C" int rzv_uart_transport_get_extended_stats(uint8_t channel, uint32_t *rx_count,
					       uint32_t *overflow_count,
					       uint32_t *isr_writes,
					       uint32_t *app_reads)
{
	return uart_rc_buffer_get_extended_stats(channel, rx_count, overflow_count, isr_writes, app_reads);
}

extern "C" int rzv_uart_transport_get_indices(uint8_t channel, uint32_t *widx, uint32_t *ridx)
{
	return uart_rc_buffer_get_indices(channel, widx, ridx);
}

extern "C" void rzv_uart_transport_debug_isr_stats(void)
{
	rzv_uart_debug_isr_stats();
}

#endif /* __PX4_FREERTOS */
