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
} rzv_spi_backend_t;

int rzv_spi_backend_init(rzv_spi_backend_t *handle, uint8_t bus);
int rzv_spi_backend_set_mode(rzv_spi_backend_t *handle, uint8_t mode);
int rzv_spi_backend_set_bits_per_word(rzv_spi_backend_t *handle, uint8_t bits);
int rzv_spi_backend_set_speed(rzv_spi_backend_t *handle, uint32_t frequency_hz);
int rzv_spi_backend_transfer(rzv_spi_backend_t *handle, const void *tx, void *rx, size_t length);

#ifdef __cplusplus
}

#endif /* __PX4_FREERTOS */

#endif
