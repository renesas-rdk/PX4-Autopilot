/****************************************************************************
 *
 * Copyright (c) 2025 Renesas Electronics Corporation and/or its affiliates
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 ****************************************************************************/

/**
 * @file uart_openamp_backend.h
 * @brief OpenAMP transport backend for UART (MAVLink over RPMsg)
 *
 * This backend routes UART traffic through OpenAMP/RPMsg instead of
 * FSP hardware UART. Used when RZV_ENABLE_CA55=ON to enable MAVLink
 * telemetry over WiFi via Linux/CA55.
 *
 * Architecture:
 *   PX4 (CR8) → OpenAMP ch3 → CA55/Linux → UDP:14550 → QGC
 */

#pragma once

#if defined(__PX4_FREERTOS)


#ifdef __cplusplus
extern "C" {
#endif

#include "rzv_fsp/uart_fsp_backend.h"
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/**
 * @brief OpenAMP channel assignments
 */
#define OPENAMP_CHANNEL_UXRCE_DDS  2   ///< uXRCE-DDS channel (existing)
#define OPENAMP_CHANNEL_MAVLINK    3   ///< MAVLink channel (new)

/**
 * @brief RPMsg service names
 */
#define RPMSG_SERVICE_UXRCE    "rpmsg-openamp-demo-channel"  ///< Existing uXRCE-DDS service
#define RPMSG_SERVICE_MAVLINK  "rpmsg-mavlink-channel"       ///< New MAVLink service

/**
 * @brief OpenAMP-specific backend data
 */
typedef struct {
    uint8_t channel;                ///< OpenAMP channel number (2 or 3)
    const char *service_name;       ///< RPMsg service name
    bool initialized;               ///< Initialization flag
    uint32_t tx_count;              ///< Transmitted packet count
    uint32_t rx_count;              ///< Received packet count
    uint32_t error_count;           ///< Error count
} rzv_uart_openamp_data_t;

/**
 * @brief Initialize OpenAMP transport backend
 *
 * @param backend UART backend structure (will be configured for OpenAMP)
 * @param logical_channel Logical UART channel (1 = MAVLink)
 * @return 0 on success, negative on error
 *
 * @note This function:
 *       - Sets up RPMsg endpoint for MAVLink channel
 *       - Registers callbacks for OpenAMP communication
 *       - Initializes buffers and state tracking
 */
int rzv_uart_openamp_init(rzv_uart_backend_t *backend, uint8_t logical_channel);

/**
 * @brief Read data from OpenAMP channel
 *
 * @param backend UART backend structure
 * @param buf Buffer to store received data
 * @param len Maximum bytes to read
 * @return Number of bytes read, or negative on error
 *
 * @note This function blocks until data is available or timeout occurs.
 *       It reads from the OpenAMP receive buffer populated by RPMsg callbacks.
 */
int rzv_uart_openamp_read(rzv_uart_backend_t *backend, uint8_t *buf, size_t len);

/**
 * @brief Write data to OpenAMP channel
 *
 * @param backend UART backend structure
 * @param buf Data to transmit
 * @param len Number of bytes to write
 * @return Number of bytes written, or negative on error
 *
 * @note This function sends data via RPMsg to the CA55 Linux side.
 *       Data is forwarded to UDP:14550 by CustomXRCEAgent.
 */
int rzv_uart_openamp_write(rzv_uart_backend_t *backend, const uint8_t *buf, size_t len);

/**
 * @brief Shutdown OpenAMP transport
 *
 * @param backend UART backend structure
 *
 * @note This function:
 *       - Destroys RPMsg endpoint
 *       - Frees allocated resources
 *       - Resets state flags
 */
void rzv_uart_openamp_shutdown(rzv_uart_backend_t *backend);

/**
 * @brief Get OpenAMP channel for logical UART channel
 *
 * @param logical_channel Logical UART channel number
 * @return OpenAMP channel number, or -1 if not supported
 */
static inline int rzv_uart_get_openamp_channel(uint8_t logical_channel)
{
    switch (logical_channel) {
    case 1:  // MAVLink channel
        return OPENAMP_CHANNEL_MAVLINK;
    default:
        return -1;  // Not supported via OpenAMP
    }
}

/**
 * @brief Get RPMsg service name for logical UART channel
 *
 * @param logical_channel Logical UART channel number
 * @return Service name string, or NULL if not supported
 */
static inline const char *rzv_uart_get_rpmsg_service_name(uint8_t logical_channel)
{
    switch (logical_channel) {
    case 1:  // MAVLink channel
        return RPMSG_SERVICE_MAVLINK;
    default:
        return NULL;
    }
}

#ifdef __cplusplus
}

#endif /* __PX4_FREERTOS */

#endif
