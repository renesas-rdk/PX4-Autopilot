/****************************************************************************
 *
 * Copyright (c) 2025 Renesas Electronics Corporation and/or its affiliates
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 ****************************************************************************/

/**
 * @file uart_rc_switch.h
 * @brief UART RC channel switching API - Stub implementation
 * 
 * This file provides API compatibility for PX4 RC protocol drivers.
 * Since hardware is fixed to FS-A8S (SBUS only), these are now stubs.
 * 
 * UART config fixed in FSP: UART1 (SCI7), 100kHz, 8E2 (SBUS)
 * Previous implementation removed October 19, 2025 (code cleanup).
 */

#ifndef UART_RC_SWITCH_H
#define UART_RC_SWITCH_H

#if defined(__PX4_FREERTOS)


#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

void uart_set_rc_serial_mode(bool use_8n1);
int uart_reopen_rc_channel(uint32_t baudrate);
void uart_rc_buffer_flush(uint8_t channel);
int uart_rc_buffer_read(uint8_t channel, uint8_t *buffer, int max_len);
int uart_rc_buffer_available(uint8_t channel);

#ifdef __cplusplus
}
#endif


#endif /* __PX4_FREERTOS */

#endif /* UART_RC_SWITCH_H */
