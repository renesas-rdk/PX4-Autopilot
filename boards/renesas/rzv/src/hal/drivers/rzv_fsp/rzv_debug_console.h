/*
 * Copyright (c) 2026 Renesas Electronics Corporation and/or its affiliates
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/**
 * @file rzv_debug_console.h
 * @brief Polled debug-console sink over the SCI_B UART1 instance
 *        (g_uart_debug_tx, TXD1 on pin P52, 115200 8N1).
 *
 * Mirrors the CR8 stdout/stderr stream (PX4_INFO / printf, routed through
 * newlib _write fd 1/2 in syscalls.c) out a physical UART so it can be read
 * on a host USB-to-TTY without J-Link / SEGGER RTT.
 *
 * Output-only (TX). Polled byte writes: works from any context (task, ISR,
 * early boot before the scheduler, fault handlers) and never enables the
 * TXI interrupt, so it cannot race the FSP async UART path.
 *
 * PIN HISTORY — debug TX lives on P52 (SCI1 TXD1). It was briefly moved to
 * P50 (SCI0 TXD0) while P50 was thought free after all IMUs went polled, but
 * P50 is the same physical net as the ICM-45688 IMU#1 INT1 pad: at boot the
 * sensor's push-pull INT1 fought the UART output, corrupting SPI reads and
 * driving a RegisterCheck reconfigure storm (see ICM45686.cpp RunImpl). The
 * console mirror also replayed the last log line at above line rate. Moving
 * debug TX back to P52 removes the borrow entirely; P50 belongs solely to the
 * IMU INT1 net again. (P52 was the GPS M10 "Safety Switch" input; that
 * function is unused on this board.) See bsp_pin_cfg.h / configuration.xml.
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
