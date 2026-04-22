/****************************************************************************
 *
 * Copyright (c) 2025 Renesas Electronics Corporation and/or its affiliates
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 ****************************************************************************/

/**
 * @file rzv_serial_config.h
 * @brief Helper macros so UART device assignments can be changed from default.px4board.
 */

#pragma once

#if defined(__PX4_FREERTOS)


#include <px4_boardconfig.h>

#ifndef CONFIG_BOARD_SERIAL_RC
#error "CONFIG_BOARD_SERIAL_RC must be defined in the board config"
#endif

#ifndef CONFIG_BOARD_SERIAL_TEL1
#error "CONFIG_BOARD_SERIAL_TEL1 must be defined in the board config"
#endif

#ifndef CONFIG_BOARD_SERIAL_GPS1
#define CONFIG_BOARD_SERIAL_GPS1 "/dev/ttyS9"
#endif

#ifndef CONFIG_BOARD_SERIAL_LIDAR
#define CONFIG_BOARD_SERIAL_LIDAR "/dev/ttyS4"
#endif

#define RZV_RC_SERIAL_DEVICE      CONFIG_BOARD_SERIAL_RC
#define RZV_MAVLINK_SERIAL_DEVICE CONFIG_BOARD_SERIAL_TEL1
#define RZV_GPS_SERIAL_DEVICE     CONFIG_BOARD_SERIAL_GPS1
#define RZV_LIDAR_SERIAL_DEVICE   CONFIG_BOARD_SERIAL_LIDAR

#endif /* __PX4_FREERTOS */
