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

#include "hal_data.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
	uint8_t bus;
	spi_b_instance_ctrl_t *ctrl;
	spi_cfg_t config;
	spi_b_extended_cfg_t extend;
	uint32_t current_speed_hz;
	uint8_t current_mode;
	uint8_t bits_per_word;
	bool opened;
	bool needs_reopen;	/* set after an unrecovered transfer failure; next transfer reopens first */
} rzv_spi_backend_t;

/* Per-transfer device configuration. The backend applies every field inside
 * the same critical section as the transfer itself, so multiple devices
 * sharing the channel (different SSL lines, potentially different mode/speed)
 * can never interleave a settings change with another device's transfer. */
typedef struct {
	uint8_t  ssl_index;	/* 0..3 -> SPCMD0.SSLA (hardware SSL line) */
	uint8_t  mode;		/* SPI_CPOL | SPI_CPHA */
	uint8_t  bits_per_word;	/* 8 or 16 */
	uint32_t speed_hz;
} rzv_spi_xfer_cfg_t;

int rzv_spi_backend_init(rzv_spi_backend_t *handle, uint8_t bus);
int rzv_spi_backend_transfer(rzv_spi_backend_t *handle, const rzv_spi_xfer_cfg_t *cfg,
			     const void *tx, void *rx, size_t length);

#ifdef __cplusplus
}

#endif /* __PX4_FREERTOS */

#endif
