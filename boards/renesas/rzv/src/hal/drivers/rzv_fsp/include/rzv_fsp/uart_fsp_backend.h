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
#include <stddef.h>
#include <stdint.h>

#include <FreeRTOS.h>
#include <semphr.h>

#include "hal_data.h"
#include "r_sci_b_uart.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
	uint8_t channel;
    sci_b_uart_instance_ctrl_t *ctrl;
    uart_cfg_t config;
    sci_b_uart_extended_cfg_t extend;
    uint8_t *tx_bounce;
    uint8_t *rx_bounce;
    size_t bounce_len;
    bool dma_tx_enabled;
    bool dma_rx_enabled;
    size_t dma_rx_block_len;
    size_t dma_rx_min_chunk;
    volatile bool dma_rx_active;
    volatile uint32_t dma_rx_restart_failures;
    volatile bool dma_rx_needs_restart;  /* Flag set by ISR, cleared by task context */
    volatile bool isr_rx_started;        /* ISR mode: true if R_SCI_B_UART_Read() started */
    bool use_ring_buffer;
    bool opened;
    uint32_t baudrate;
    sci_b_baud_setting_t baud_setting;
    StaticSemaphore_t rx_sem_buffer;
    StaticSemaphore_t tx_sem_buffer;
    SemaphoreHandle_t rx_sem;
    SemaphoreHandle_t tx_sem;
    StaticSemaphore_t mutex_buffer;
    SemaphoreHandle_t mutex;
    
    /* Interrupt statistics (updated in ISR, read from user space) */
    volatile uint32_t event_count;
    volatile uint32_t rx_char_count;
    volatile uint32_t overflow_count;
    volatile uint32_t error_count;

    /* Overflow detection and DMA watchdog */
    volatile bool hw_overflow_detected;      /* Set by ISR, cleared by read */
    volatile uint64_t last_read_time_us;     /* For DMA watchdog */
    volatile uint64_t last_dma_rx_time_us;   /* Last successful DMA RX */
    volatile uint64_t last_overflow_warn_us; /* Throttle overflow warnings */
} rzv_uart_backend_t;

int rzv_uart_backend_init(rzv_uart_backend_t *handle, uint8_t channel);
int rzv_uart_backend_configure(rzv_uart_backend_t *handle, uint32_t baudrate);
int rzv_uart_backend_read(rzv_uart_backend_t *handle, uint8_t *buffer, size_t length);
int rzv_uart_backend_write(rzv_uart_backend_t *handle, const uint8_t *buffer, size_t length);
void rzv_uart_backend_shutdown(rzv_uart_backend_t *handle);
int rzv_uart_backend_reopen(rzv_uart_backend_t *handle);

/**
 * @brief Print debug statistics from UART ISR counters
 * This function prints ISR activity counters and first received bytes for all channels.
 * Useful for debugging UART RX issues.
 */
void rzv_uart_debug_isr_stats(void);

#ifdef __cplusplus
}

#endif /* __PX4_FREERTOS */

#endif
