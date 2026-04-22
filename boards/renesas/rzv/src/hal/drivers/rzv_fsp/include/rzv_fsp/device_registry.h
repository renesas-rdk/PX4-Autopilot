/****************************************************************************
 *
 * Copyright (c) 2025 Renesas Electronics Corporation and/or its affiliates
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 ****************************************************************************/

#pragma once

#if defined(__PX4_FREERTOS)


#include "rzv_fsp/uart_fsp_backend.h"

#ifdef __cplusplus
extern "C" {
#endif

int rzv_register_fsp_devices(void);
rzv_uart_backend_t *rzv_get_uart_backend(uint8_t logical_channel);

#ifdef __cplusplus
}

#endif /* __PX4_FREERTOS */

#endif
