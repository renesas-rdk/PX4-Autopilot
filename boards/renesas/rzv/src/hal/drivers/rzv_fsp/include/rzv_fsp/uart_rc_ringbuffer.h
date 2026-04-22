/****************************************************************************
 *
 * Copyright (c) 2025 Renesas Electronics Corporation and/or its affiliates
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 ****************************************************************************/

/**
 * @file uart_rc_ringbuffer.h
 * @brief Non-blocking ring buffer for RC UART input
 */

#ifndef UART_RC_RINGBUFFER_H
#define UART_RC_RINGBUFFER_H

#if defined(__PX4_FREERTOS)


#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize ring buffer for specific UART channel
 * @param channel UART channel number (0-3)
 */
void uart_rc_buffer_init_channel(uint8_t channel);

/**
 * @brief Get number of bytes available in ring buffer (non-blocking)
 * @param channel UART channel number
 * @return Number of bytes available to read
 */
int uart_rc_buffer_available_internal(uint8_t channel);

/**
 * @brief Read bytes from ring buffer (non-blocking)
 * @param channel UART channel number
 * @param buffer Destination buffer
 * @param max_len Maximum bytes to read
 * @return Number of bytes actually read (0 if empty)
 */
int uart_rc_buffer_read_internal(uint8_t channel, uint8_t *buffer, int max_len);

/**
 * @brief Flush (clear) ring buffer
 * @param channel UART channel number
 */
void uart_rc_buffer_flush_internal(uint8_t channel);

/**
 * @brief Write bytes to ring buffer from UART RX ISR (ISR-safe)
 * @param channel UART channel number
 * @param data Pointer to received data
 * @param length Number of bytes received
 */
void uart_rc_buffer_write_from_isr(uint8_t channel, const uint8_t *data, uint32_t length);

/**
 * @brief Get buffer statistics
 * @param channel UART channel number
 * @param rx_count Total bytes received (output, can be NULL)
 * @param overflow_count Overflow events (output, can be NULL)
 */
void uart_rc_buffer_get_stats(uint8_t channel, uint32_t *rx_count, uint32_t *overflow_count);

int uart_rc_buffer_get_extended_stats(uint8_t channel, uint32_t *rx_count,
                                      uint32_t *overflow_count,
                                      uint32_t *isr_writes,
                                      uint32_t *app_reads);

int uart_rc_buffer_get_indices(uint8_t channel, uint32_t *widx, uint32_t *ridx);

/**
 * @brief Wait for data to be available in ring buffer with timeout
 * @param channel UART channel number
 * @param timeout_ms Timeout in milliseconds (0 = no wait, -1 = wait indefinitely)
 * @return Number of bytes available, or 0 if timeout
 *
 * This function allows the task to block instead of polling, significantly reducing CPU usage.
 * Use this function instead of polling loop uart_rc_buffer_available_internal().
 */
int uart_rc_buffer_wait_for_data(uint8_t channel, int timeout_ms);

/**
 * @brief Debug: Print full ring buffer state (for troubleshooting)
 * @param channel UART channel number
 */
void uart_rc_buffer_debug_state(uint8_t channel);

#ifdef __cplusplus
}
#endif


#endif /* __PX4_FREERTOS */

#endif /* UART_RC_RINGBUFFER_H */
