/*
 * Copyright (c) 2026 Renesas Electronics Corporation and/or its affiliates
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/**
 * @file rzv_debug_console.h
 * @brief Polled debug-console sink over the SCI_B UART1 instance
 *        (g_uart1_debug_tx, TXD1 on pin P52, 115200 8N1).
 *
 * Mirrors the CR8 stdout/stderr stream (PX4_INFO / printf, routed through
 * newlib _write fd 1/2 in syscalls.c) out a physical UART so it can be read
 * on a host USB-to-TTY at /dev/ttyUSB0 without J-Link / SEGGER RTT.
 *
 * Output-only (TX). Polled byte writes: works from any context (task, ISR,
 * early boot before the scheduler, fault handlers) and never enables the
 * TXI interrupt, so it cannot race the FSP async UART path.
 *
 * ┌──────────────────────────────────────────────────────────────────────────┐
 * │ TEMPORARY PIN BORROW (hardware note):                                      │
 * │ The debug-UART TX (TXD1) is on pin P52, which on this board is wired to    │
 * │ the GPS M10 "Safety Switch" line. We are BORROWING that pad for debug TX    │
 * │ only (board→host). While this debug console is in use, the GPS M10 safety  │
 * │ switch on P52 is unavailable. This is a bring-up/debug arrangement — revert │
 * │ the pad to its safety-switch function (or move debug TX to a free UART)     │
 * │ before relying on the safety switch.                                       │
 * └──────────────────────────────────────────────────────────────────────────┘
 */

#ifndef RZV_DEBUG_CONSOLE_H_
#define RZV_DEBUG_CONSOLE_H_

#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Open g_uart1_debug_tx once. Safe to call multiple times (idempotent). */
void rzv_debug_console_init(void);

/** Blocking polled write of @p len bytes; '\n' is expanded to "\r\n".
 *  No-op until rzv_debug_console_init() has succeeded. */
void rzv_debug_console_write(const char *buf, size_t len);

#ifdef __cplusplus
}
#endif

#endif /* RZV_DEBUG_CONSOLE_H_ */
