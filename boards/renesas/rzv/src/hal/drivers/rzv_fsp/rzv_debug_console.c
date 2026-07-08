/*
 * Copyright (c) 2026 Renesas Electronics Corporation and/or its affiliates
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/**
 * @file rzv_debug_console.c
 * @brief Polled debug-console TX over SCI_B UART1 (g_uart_debug_tx / P52 / 115200 8N1).
 *
 * See rzv_debug_console.h for rationale. This object also provides:
 *   - rzv_uart_debug_callback(): the (empty) callback referenced by the
 *     generated g_uart_debug_tx_cfg in rzv_gen/hal_data.c. hal_data.o's
 *     reference to this symbol forces this translation unit to be linked,
 *     which in turn defines rzv_console_aux_write() (below) so the weak hook
 *     in src/renesas-robotics-platform/cr8/runtime/syscalls.c resolves to a strong def.
 *   - rzv_console_aux_write(): the strong override of the platform's weak
 *     auxiliary console sink, called for fd 1/2 inside _write().
 */

#include "hal_data.h"        /* g_uart_debug_tx*, R_SCI_B_UART_Open, R_SCI_B0_Type */
#include "rzv_debug_console.h"

#include <stdbool.h>
#include <stdint.h>

/* CSR.TDREC: write-1-to-clear of the TDRE (transmit-data-empty) flag.
 * In FIFO mode TDRE is not auto-cleared by a TDR write, so software must
 * re-arm it after each byte (mirrors the FSP TXI ISR). */
#define RZV_SCI_B_CFCLR_TDREC   (0x20000000U)

/* CFCLR write-1-to-clear mask for every RX-side flag: RDRFC (b31), FERC (b28),
 * PERC (b27), ORERC (b24), ERSC (b4). Used when quiescing the receiver. */
#define RZV_SCI_B_CFCLR_RX_ALL  (0x99000010U)

static volatile bool s_console_inited = false;

/* Referenced by g_uart_debug_tx_cfg.p_callback in rzv_gen/hal_data.c.
 * The debug console is polled-only, so nothing to do here. */
void rzv_uart_debug_callback(uart_callback_args_t *p_args)
{
    (void)p_args;
}

void rzv_debug_console_init(void)
{
    if (s_console_inited) {
        return;
    }

    /* Force a clean SCI channel before open so a WARM CR8 restart re-inits the
     * debug UART cleanly (the fresh FSP ctrl makes Close no-op). Per-channel
     * MODULE_STOP; Open below re-STARTs it; harmless on cold boot. */
    R_BSP_MODULE_STOP(FSP_IP_SCI, g_uart_debug_tx_cfg.channel);
    R_BSP_SoftwareDelay(500, BSP_DELAY_UNITS_MICROSECONDS);

    fsp_err_t err = R_SCI_B_UART_Open(&g_uart_debug_tx_ctrl, &g_uart_debug_tx_cfg);
    if ((FSP_SUCCESS == err) || (FSP_ERR_ALREADY_OPEN == err)) {
        /* R_SCI_B_UART_Open() enables the transmitter (CCR0.TE) but leaves the
         * TXI interrupt disabled (TIE). Polled TX is therefore safe. */

        /* TX-only hardening: this console is a log sink — no shell, and the
         * pin config muxes only TXD1 (P52); no RX pin is assigned. Open()
         * nevertheless enables the receiver and its RXI/ERI vectors (the
         * generated cfg carries rxi_ipl/eri_ipl=14), so a floating/unmuxed
         * RX input could still latch noise into an ISR. Stop the receiver
         * and keep the RX-side vectors off (defense-in-depth). */
        R_SCI_B0_Type *reg = g_uart_debug_tx_ctrl.p_reg;
        if (reg != NULL) {
            reg->CCR0_b.RE  = 0;
            reg->CCR0_b.RIE = 0;
            reg->CFCLR      = RZV_SCI_B_CFCLR_RX_ALL;
            reg->FFCLR      = 1U; /* DRC: drop any byte already in the RX FIFO */
        }
        if (g_uart_debug_tx_cfg.rxi_irq >= 0) {
            R_BSP_IrqDisable(g_uart_debug_tx_cfg.rxi_irq);
        }
        if (g_uart_debug_tx_cfg.eri_irq >= 0) {
            R_BSP_IrqDisable(g_uart_debug_tx_cfg.eri_irq);
        }

        s_console_inited = true;
    }
}

static inline void rzv_debug_console_putc(R_SCI_B0_Type *reg, uint8_t c)
{
    /* Wait until the transmit data register / FIFO can accept a byte. */
    while (0U == reg->CSR_b.TDRE) {
        /* busy-wait */
    }
    reg->TDR_BY = c;
    /* Re-arm TDRE for the next slot (W1C). */
    reg->CFCLR = RZV_SCI_B_CFCLR_TDREC;
}

void rzv_debug_console_write(const char *buf, size_t len)
{
    if (!s_console_inited || (buf == NULL) || (len == 0U)) {
        return;
    }

    R_SCI_B0_Type *reg = g_uart_debug_tx_ctrl.p_reg;
    if (reg == NULL) {
        return;
    }

    for (size_t i = 0U; i < len; i++) {
        const char ch = buf[i];
        if (ch == '\n') {
            rzv_debug_console_putc(reg, (uint8_t)'\r');
        }
        rzv_debug_console_putc(reg, (uint8_t)ch);
    }

    /* Drain the shifter so the last bytes are flushed before we return
     * (important when a crash/halt follows the final log line). */
    while (0U == reg->CSR_b.TEND) {
        /* busy-wait */
    }
}

/* Strong override of the platform's weak auxiliary console sink (declared
 * weak in src/renesas-robotics-platform/cr8/runtime/syscalls.c). Called for fd 1/2. */
void rzv_console_aux_write(const char *buf, size_t len)
{
    rzv_debug_console_write(buf, len);
}
