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
#include "r_sci_b_i2c.h"
#include "posix_compat/i2c-dev.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
	uint8_t bus;
    i2c_master_ctrl_t *ctrl;
    i2c_master_cfg_t config;
    sci_b_i2c_extended_cfg_t extend;
	bool opened;
	uint32_t bus_hz;
	uint16_t current_addr;
	uint16_t current_flags;
	bool addr_valid;
	uint32_t error_streak;
} rzv_i2c_backend_t;

int rzv_i2c_backend_init(rzv_i2c_backend_t *handle, uint8_t bus, uint32_t frequency_hz);
int rzv_i2c_backend_transfer(rzv_i2c_backend_t *handle, struct i2c_msg *msgs, size_t msg_count);
void rzv_i2c_backend_shutdown(rzv_i2c_backend_t *handle);

#ifdef __cplusplus
}

#endif /* __PX4_FREERTOS */

#endif
