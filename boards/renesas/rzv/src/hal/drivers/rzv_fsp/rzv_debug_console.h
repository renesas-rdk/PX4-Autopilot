/*
 * Copyright (c) 2026 Renesas Electronics Corporation and/or its affiliates
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/**
 * @file rzv_debug_console.h
 * @brief Polled debug-console sink over the SCI_B UART0 instance
 *        (g_uart_debug_tx, TXD0 on pin P50, 115200 8N1).
 *
 * Mirrors the CR8 stdout/stderr stream (PX4_INFO / printf, routed through
 * newlib _write fd 1/2 in syscalls.c) out a physical UART so it can be read
 * on a host USB-to-TTY at /dev/ttyUSB0 without J-Link / SEGGER RTT.
 *
 * Output-only (TX). Polled byte writes: works from any context (task, ISR,
 * early boot before the scheduler, fault handlers) and never enables the
 * TXI interrupt, so it cannot race the FSP async UART path.
 *
 * P50 was previously the ICM-45688 IMU#1 data-ready (DRDY/TINT) pin; freed
 * once all IMUs moved to polled mode (see spi_fsp_backend.c / config.txt
 * -P). Debug TX moved here from its former pad P52, which is wired to the
 * GPS M10 "Safety Switch" line and is now free for that function again
 * (see GPS_SAFETY_SWITCH in rzv_cfg/fsp_cfg/bsp/bsp_pin_cfg.h) — no more
 * pin-borrow conflict between the two.
 */

#ifndef RZV_DEBUG_CONSOLE_H_
#define RZV_DEBUG_CONSOLE_H_

#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Open g_uart_debug_tx once. Safe to call multiple times (idempotent). */
void rzv_debug_console_init(void);

/** Blocking polled write of @p len bytes; '\n' is expanded to "\r\n".
 *  No-op until rzv_debug_console_init() has succeeded. */
void rzv_debug_console_write(const char *buf, size_t len);

#ifdef __cplusplus
}
#endif

#endif /* RZV_DEBUG_CONSOLE_H_ */
