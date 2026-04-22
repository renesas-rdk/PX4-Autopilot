/****************************************************************************
 *
 * Copyright (c) 2025 Renesas Electronics Corporation and/or its affiliates
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 ****************************************************************************/

/**
 * @file uart_rc_ringbuffer.c
 * @brief Non-blocking ring buffer for RC UART input (interrupt-driven)
 *
 * This implements a circular buffer that is filled by UART RX interrupts
 * and read by the rc_input task without blocking.
 */


#if defined(__PX4_FREERTOS)

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include "FreeRTOS.h"
#include "semphr.h"

#include "rzv_fsp/dma_buffer.h"
#if defined(__GNUC__)
#define UART_RINGBUF_DMA_STORAGE __attribute__((aligned(RZV_DMA_DEFAULT_ALIGNMENT), section(".noncache_buffer")))
#else
#define UART_RINGBUF_DMA_STORAGE
#endif

#define UART_RC_RINGBUF_SIZE  8192U  /* Must be power of 2 for fast modulo; large enough for MAVLink console bursts */

typedef struct {
	uint8_t *buffer;
	volatile uint32_t write_idx;
	volatile uint32_t read_idx;
	SemaphoreHandle_t mutex;
	SemaphoreHandle_t data_ready_sem;  /* Semaphore to signal when new data is available */
	volatile bool initialized;
	volatile uint32_t overflow_count;
	volatile uint32_t total_rx_count;
    volatile uint32_t app_read_calls;
} uart_rc_ringbuffer_t;

static uart_rc_ringbuffer_t g_rc_ringbuf[4] = {0};  /* Support up to 4 UART channels */
static volatile uint32_t g_total_isr_writes[4] = {0};

static uint8_t g_uart_ringbuf_storage[4][UART_RC_RINGBUF_SIZE] UART_RINGBUF_DMA_STORAGE;

/* Static semaphore storage to avoid heap allocation */
static StaticSemaphore_t g_rc_mutex_buffer[4];
static StaticSemaphore_t g_rc_data_sem_buffer[4];

/**
 * @brief Initialize ring buffer for a specific UART channel
 */
void uart_rc_buffer_init_channel(uint8_t channel)
{
	if (channel >= 4U) {
		return;
	}

	uart_rc_ringbuffer_t *rb = &g_rc_ringbuf[channel];

	if (rb->buffer == NULL) {
		rb->buffer = g_uart_ringbuf_storage[channel];
	}

	/* Use static storage for semaphores to avoid heap allocation */
	if (rb->mutex == NULL) {
		rb->mutex = xSemaphoreCreateMutexStatic(&g_rc_mutex_buffer[channel]);
	}

	/* Create data ready semaphore with static storage */
	if (rb->data_ready_sem == NULL) {
		rb->data_ready_sem = xSemaphoreCreateBinaryStatic(&g_rc_data_sem_buffer[channel]);
	}

	if (rb->mutex != NULL) {
		xSemaphoreTake(rb->mutex, portMAX_DELAY);
		rb->write_idx = 0U;
		rb->read_idx = 0U;
		rb->overflow_count = 0U;
		rb->total_rx_count = 0U;
		rb->app_read_calls = 0U;

		if (rb->buffer != NULL) {
			memset((void *)rb->buffer, 0, UART_RC_RINGBUF_SIZE);
		}

		rb->initialized = true;
		xSemaphoreGive(rb->mutex);
	}
}

/**
 * @brief Get available bytes in ring buffer (non-blocking)
 */
int uart_rc_buffer_available_internal(uint8_t channel)
{
	if ((channel >= 4U) || !g_rc_ringbuf[channel].initialized) {
		return 0;
	}

	uart_rc_ringbuffer_t *rb = &g_rc_ringbuf[channel];

	uint32_t widx = rb->write_idx;
	uint32_t ridx = rb->read_idx;

	/* Calculate available bytes in ring buffer */
	int avail;
	if (widx >= ridx) {
		avail = (int)(widx - ridx);
	} else {
		avail = (int)(UART_RC_RINGBUF_SIZE - ridx + widx);
	}

	return avail;
}

/**
 * @brief Read bytes from ring buffer (non-blocking)
 * @return Number of bytes actually read (0 if buffer empty)
 */
int uart_rc_buffer_read_internal(uint8_t channel, uint8_t *buffer, int max_len)
{
	if ((channel >= 4U) || (buffer == NULL) || (max_len <= 0)) {
		return 0;
	}

	uart_rc_ringbuffer_t *rb = &g_rc_ringbuf[channel];

	if (!rb->initialized || (rb->buffer == NULL)) {
		return 0;
	}

	/* On overflow: stale frames (possibly with arm/switch states from before disarm)
	 * may be queued. Skip directly to the newest data so we never replay old states.
	 * ISR-safe: only this task modifies read_idx; ISR only modifies write_idx. */
	if (rb->overflow_count > 0U) {
		rb->read_idx = rb->write_idx;
		rb->overflow_count = 0U;
		return 0;  /* Let caller retry on next scheduled wake */
	}

	/* Lock-free read: we only need to ensure we don't read past the write pointer.
	 * Since this is the only consumer, we own read_idx.
	 * write_idx is updated by ISR, so we capture it once.
	 */
	uint32_t widx = rb->write_idx;
	uint32_t ridx = rb->read_idx;

	/* Calculate available bytes in ring buffer */
	int available;
	if (widx >= ridx) {
		available = (int)(widx - ridx);
	} else {
		available = (int)(UART_RC_RINGBUF_SIZE - ridx + widx);
	}

	int to_read = (available < max_len) ? available : max_len;
	int bytes_read = 0;

	if (to_read > 0) {
		/* Optimize with memcpy (handle wrap-around) */
		uint32_t chunk1 = UART_RC_RINGBUF_SIZE - ridx;
		if (chunk1 > (uint32_t)to_read) {
			chunk1 = (uint32_t)to_read;
		}
		
		memcpy(buffer, &rb->buffer[ridx], chunk1);
		
		if (chunk1 < (uint32_t)to_read) {
			uint32_t chunk2 = (uint32_t)to_read - chunk1;
			memcpy(&buffer[chunk1], &rb->buffer[0], chunk2);
		}

		rb->read_idx = (ridx + (uint32_t)to_read) & (UART_RC_RINGBUF_SIZE - 1U);
		bytes_read = to_read;
		rb->app_read_calls++;
	}

	return bytes_read;
}

/**
 * @brief Flush (clear) ring buffer
 */
void uart_rc_buffer_flush_internal(uint8_t channel)
{
	if ((channel >= 4U) || !g_rc_ringbuf[channel].initialized) {
		return;
	}

	uart_rc_ringbuffer_t *rb = &g_rc_ringbuf[channel];

	if (rb->mutex != NULL) {
		xSemaphoreTake(rb->mutex, portMAX_DELAY);
		rb->read_idx = rb->write_idx;  /* Fast flush: just move read pointer */
		xSemaphoreGive(rb->mutex);
	}
}

/**
 * @brief Write bytes to ring buffer (called from UART RX ISR)
 * @note This is ISR-safe, uses atomic operations
 */
void uart_rc_buffer_write_from_isr(uint8_t channel, const uint8_t *data, uint32_t length)
{
	if ((channel >= 4U) || (data == NULL) || (length == 0U)) {
		return;
	}

	uart_rc_ringbuffer_t *rb = &g_rc_ringbuf[channel];

	if (!rb->initialized || (rb->buffer == NULL)) {
		/* DEBUG: Track calls to uninitialized buffer */
		static volatile uint32_t uninit_call_count = 0;
		uninit_call_count++;
		return;
	}

	uint32_t widx = rb->write_idx;
	uint32_t ridx = rb->read_idx;
	const bool buffer_was_empty = (widx == ridx);

	/* Calculate free space (leave 1 byte gap to distinguish full from empty) */
	uint32_t free_space;
	if (ridx > widx) {
		free_space = ridx - widx - 1U;
	} else {
		free_space = (UART_RC_RINGBUF_SIZE - widx) + ridx - 1U;
	}

	uint32_t to_write = length;
	if (to_write > free_space) {
		rb->overflow_count += (to_write - free_space);
		to_write = free_space;
	}

	if (to_write > 0U) {
		uint32_t chunk1 = UART_RC_RINGBUF_SIZE - widx;
		if (chunk1 > to_write) {
			chunk1 = to_write;
		}

		memcpy(&rb->buffer[widx], data, chunk1);

		if (chunk1 < to_write) {
			uint32_t chunk2 = to_write - chunk1;
			memcpy(&rb->buffer[0], &data[chunk1], chunk2);
		}

		/* CRITICAL: Update write_idx AFTER all data written */
		rb->write_idx = (widx + to_write) & (UART_RC_RINGBUF_SIZE - 1U);
		rb->total_rx_count += to_write;
	}
	
	/* DEBUG: Track total ISR write calls */
	if (channel < 4U) {
		g_total_isr_writes[channel]++;
	}

	/* Signal semaphore to wake up any waiting read task */
	if ((to_write > 0U) && buffer_was_empty && (rb->data_ready_sem != NULL)) {
		BaseType_t higher_priority_woken = pdFALSE;
		xSemaphoreGiveFromISR(rb->data_ready_sem, &higher_priority_woken);
		portYIELD_FROM_ISR(higher_priority_woken);
	}
}

/**
 * @brief Get buffer statistics (for debugging)
 */
void uart_rc_buffer_get_stats(uint8_t channel, uint32_t *rx_count, uint32_t *overflow_count)
{
	if (channel >= 4U) {
		return;
	}

	uart_rc_ringbuffer_t *rb = &g_rc_ringbuf[channel];

	if (rx_count != NULL) {
		*rx_count = rb->total_rx_count;
	}

	if (overflow_count != NULL) {
		*overflow_count = rb->overflow_count;
	}
}

/**
 * @brief Debug: Get full ring buffer state (for troubleshooting)
 */
void uart_rc_buffer_debug_state(uint8_t channel)
{
#ifdef CONFIG_RZV_UART_DEBUG
	if (channel >= 4U) {
		return;
	}

	uart_rc_ringbuffer_t *rb = &g_rc_ringbuf[channel];
	
	// Use printf for simple debug output (guaranteed to work in C files)
	printf("[RINGBUF_DEBUG] Channel %u:\n", (unsigned)channel);
	printf("  initialized: %d\n", rb->initialized ? 1 : 0);
	printf("  write_idx: %u\n", (unsigned)rb->write_idx);
	printf("  read_idx: %u\n", (unsigned)rb->read_idx);
	printf("  total_rx: %u\n", (unsigned)rb->total_rx_count);
	printf("  overflow: %u\n", (unsigned)rb->overflow_count);
	printf("  mutex: %p\n", (void*)rb->mutex);
#else
	(void)channel;
#endif
}

int uart_rc_buffer_get_extended_stats(uint8_t channel, uint32_t *rx_count,
                                      uint32_t *overflow_count,
                                      uint32_t *isr_writes,
                                      uint32_t *app_reads)
{
	if (channel >= 4U) {
		return -1;
	}

	uart_rc_ringbuffer_t *rb = &g_rc_ringbuf[channel];

	if (rx_count != NULL) {
		*rx_count = rb->total_rx_count;
	}

	if (overflow_count != NULL) {
		*overflow_count = rb->overflow_count;
	}

	if (isr_writes != NULL) {
		*isr_writes = g_total_isr_writes[channel];
	}

	if (app_reads != NULL) {
		*app_reads = rb->app_read_calls;
	}

	return 0;
}

int uart_rc_buffer_get_indices(uint8_t channel, uint32_t *widx, uint32_t *ridx)
{
	if (channel >= 4U) {
		return -1;
	}

	uart_rc_ringbuffer_t *rb = &g_rc_ringbuf[channel];

	if (widx != NULL) {
		*widx = rb->write_idx;
	}

	if (ridx != NULL) {
		*ridx = rb->read_idx;
	}

	return 0;
}

/**
 * @brief Wait for data to be available in ring buffer with timeout
 * @param channel UART channel number
 * @param timeout_ms Timeout in milliseconds (0 = no wait, -1 = wait indefinitely)
 * @return Number of bytes available, or 0 if timeout
 *
 * This function allows the task to block instead of polling, significantly reducing CPU usage.
 */
int uart_rc_buffer_wait_for_data(uint8_t channel, int timeout_ms)
{
	if (channel >= 4U) {
		return 0;
	}

	uart_rc_ringbuffer_t *rb = &g_rc_ringbuf[channel];

	if (!rb->initialized || rb->data_ready_sem == NULL) {
		return 0;
	}

	/* Check if data is already available */
	int avail = uart_rc_buffer_available_internal(channel);
	if (avail > 0) {
		return avail;
	}

	/* Wait on semaphore */
	TickType_t wait_ticks;
	if (timeout_ms < 0) {
		wait_ticks = portMAX_DELAY;
	} else if (timeout_ms == 0) {
		wait_ticks = 0;
	} else {
		wait_ticks = pdMS_TO_TICKS(timeout_ms);
		if (wait_ticks == 0 && timeout_ms > 0) {
			wait_ticks = 1;  /* Ít nhất 1 tick */
		}
	}

	/* Block on semaphore */
	BaseType_t result = xSemaphoreTake(rb->data_ready_sem, wait_ticks);

	if (result == pdTRUE) {
		/* New data available */
		return uart_rc_buffer_available_internal(channel);
	}

	/* Timeout - check again once more (possible race condition) */
	return uart_rc_buffer_available_internal(channel);
}

#endif /* __PX4_FREERTOS */
