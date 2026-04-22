/****************************************************************************
 *
 * Copyright (c) 2025 Renesas Electronics Corporation and/or its affiliates
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 ****************************************************************************/

/**
 * @file uart_openamp_backend.c
 * @brief OpenAMP transport backend for UART (MAVLink over RPMsg)
 *
 * This backend routes UART traffic through OpenAMP/RPMsg instead of
 * FSP hardware UART. Used when RZV_ENABLE_CA55=ON to enable MAVLink
 * telemetry over WiFi via Linux/CA55.
 */

#define MODULE_NAME "uart_openamp"

#include "uart_openamp_backend.h"
#include "rzv_fsp/uart_fsp_backend.h"

#include <px4_platform_common/log.h>
#include <px4_platform_common/time.h>
#include <inttypes.h>
#include <FreeRTOS_POSIX/time.h>
#include <FreeRTOS_POSIX/unistd.h>
#include <drivers/drv_hrt.h>
#include <string.h>
#include <errno.h>

#ifdef __PX4_FREERTOS
#include "openamp/open_amp.h"
#include "openamp/rpmsg.h"

/* Import existing OpenAMP globals from uxrce_dds_client */
extern struct rpmsg_device* rpdev;
extern struct rpmsg_endpoint rpmsg_ept;  // uXRCE-DDS endpoint

/* MAVLink-specific RPMsg endpoint */
static struct rpmsg_endpoint rpmsg_ept_mavlink = {0};
static bool mavlink_endpoint_ready = false;
static bool mavlink_endpoint_bound = false;

/* RX buffer for MAVLink channel */
#define MAVLINK_RX_BUFFER_SIZE 1024
static uint8_t mavlink_rx_buffer[MAVLINK_RX_BUFFER_SIZE];
static size_t mavlink_rx_length = 0;
static volatile bool mavlink_message_received = false;

/* Statistics */
static uint32_t mavlink_tx_count = 0;
static uint32_t mavlink_rx_count = 0;
static uint32_t mavlink_error_count = 0;

/* Semaphore for blocking read instead of polling */
#include <FreeRTOS.h>
#include <semphr.h>
static StaticSemaphore_t mavlink_rx_sem_buffer;
static SemaphoreHandle_t mavlink_rx_sem = NULL;

/**
 * @brief MAVLink RPMsg receive callback
 *
 * Called by OpenAMP when data arrives from CA55 on MAVLink channel.
 * Copies data to RX buffer and signals availability.
 */
static int mavlink_rpmsg_callback(struct rpmsg_endpoint *ept, void *data,
                                   size_t len, uint32_t src, void *priv)
{
    (void)priv;
    (void)src;

    /* Sanity check */
    if (!ept) {
        PX4_ERR("MAVLink RX: NULL endpoint!");
        return RPMSG_SUCCESS;
    }

    /* Send response handshake to CA55 on first message */
    static bool handshake_sent = false;
    if (!handshake_sent && ept->dest_addr != RPMSG_ADDR_ANY) {
        const char *handshake = "HANDSHAKE_MAVLink_CR8";
        int ret = rpmsg_send(ept, (void*)handshake, strlen(handshake) + 1);
        if (ret < 0) {
            PX4_WARN("MAVLink: Failed to send response handshake: %d", ret);
        } else {
            PX4_INFO("MAVLink: Response handshake sent to 0x%" PRIx32, ept->dest_addr);
            handshake_sent = true;
        }
    }

    /* Validate length */
    if (len == 0) {
        return RPMSG_SUCCESS;
    }

    if (len > MAVLINK_RX_BUFFER_SIZE) {
        PX4_WARN("MAVLink: RX message too large (%zu bytes), truncating", len);
        len = MAVLINK_RX_BUFFER_SIZE;
        mavlink_error_count++;
    }

    /* Copy to buffer (simple implementation - can be improved with ring buffer) */
    if (mavlink_message_received) {
        /* Previous message not consumed yet - overwrite (data loss) */
        PX4_WARN("MAVLink: RX buffer overrun, dropping previous message");
        mavlink_error_count++;
    }

    memcpy(mavlink_rx_buffer, data, len);
    mavlink_rx_length = len;
    mavlink_message_received = true;
    mavlink_rx_count++;

    /* Signal semaphore to wake up blocking read */
    if (mavlink_rx_sem != NULL) {
        xSemaphoreGive(mavlink_rx_sem);
    }

    return RPMSG_SUCCESS;
}

/**
 * @brief MAVLink RPMsg unbind callback
 *
 * Called when CA55 disconnects the MAVLink service.
 */
static void mavlink_rpmsg_unbind(struct rpmsg_endpoint *ept)
{
    (void)ept;
    PX4_WARN("MAVLink: RPMsg service unbound");
    mavlink_endpoint_bound = false;
}

/**
 * @brief Initialize OpenAMP transport backend for MAVLink
 */
int rzv_uart_openamp_init(rzv_uart_backend_t *backend, uint8_t logical_channel)
{
    if (backend == NULL) {
        PX4_ERR("MAVLink OpenAMP: NULL backend");
        return -EINVAL;
    }

    /* Only support MAVLink channel (logical channel 1) */
    if (logical_channel != 1) {
        PX4_ERR("MAVLink OpenAMP: Unsupported channel %u (expected 1)", logical_channel);
        return -EINVAL;
    }

    PX4_INFO("MAVLink: Initializing OpenAMP transport (channel 3)");

    /* Check if RPMsg device is initialized (from uXRCE-DDS) */
    if (rpdev == NULL) {
        PX4_ERR("MAVLink OpenAMP: RPMsg device not initialized");
        PX4_ERR("  Hint: Ensure uxrce_dds_client is started before MAVLink");
        return -ENODEV;
    }

    /* Check if already initialized */
    if (mavlink_endpoint_ready) {
        PX4_WARN("MAVLink OpenAMP: Already initialized");
        return 0;
    }

    /* Reset state */
    mavlink_rx_length = 0;
    mavlink_message_received = false;
    mavlink_tx_count = 0;
    mavlink_rx_count = 0;
    mavlink_error_count = 0;

    /* Initialize semaphore for blocking read (static storage to avoid heap) */
    if (mavlink_rx_sem == NULL) {
        mavlink_rx_sem = xSemaphoreCreateBinaryStatic(&mavlink_rx_sem_buffer);
    }

    /* Create RPMsg endpoint for MAVLink channel
     * NOTE: We reuse the existing RPMsg device from uXRCE-DDS,
     * but create a separate endpoint with a different service name.
     */
    int ret = rpmsg_create_ept(&rpmsg_ept_mavlink, rpdev,
                                RPMSG_SERVICE_MAVLINK,
                                RPMSG_ADDR_ANY,       // Let system assign local address
                                RPMSG_ADDR_ANY,       // Destination TBD (learned from first RX)
                                mavlink_rpmsg_callback,
                                mavlink_rpmsg_unbind);

    if (ret != 0) {
        PX4_ERR("MAVLink OpenAMP: Failed to create RPMsg endpoint: %d", ret);
        return -EIO;
    }

    mavlink_endpoint_ready = true;
    mavlink_endpoint_bound = true;
    backend->opened = true;

    PX4_INFO("MAVLink OpenAMP: Transport ready");
    PX4_INFO("  Service: %s", RPMSG_SERVICE_MAVLINK);
    PX4_INFO("  Local addr: 0x%" PRIx32, rpmsg_ept_mavlink.addr);
    PX4_INFO("  Dest addr: 0x%" PRIx32 " (will be learned from CA55)", rpmsg_ept_mavlink.dest_addr);
    PX4_INFO("MAVLink OpenAMP: Waiting for CA55 handshake...");

    return 0;
}

/**
 * @brief Read data from OpenAMP MAVLink channel
 *
 * Blocks until data is available or timeout occurs.
 */
int rzv_uart_openamp_read(rzv_uart_backend_t *backend, uint8_t *buf, size_t len)
{
    (void)backend;

    if (buf == NULL || len == 0) {
        return -EINVAL;
    }

    if (!mavlink_endpoint_ready) {
        return -ENOTCONN;
    }

    if (!mavlink_endpoint_bound) {
        PX4_WARN("MAVLink OpenAMP: Endpoint not bound");
        return -ENOTCONN;
    }

    /* Wait for data with timeout using semaphore (efficient blocking) */
    if (!mavlink_message_received && mavlink_rx_sem != NULL) {
        /* Block on semaphore instead of polling - reduces CPU usage significantly */
        (void)xSemaphoreTake(mavlink_rx_sem, pdMS_TO_TICKS(100));
    }

    if (!mavlink_message_received) {
        /* Timeout - no data available (not an error) */
        return 0;
    }

    /* Copy data to caller's buffer */
    size_t copy_len = (mavlink_rx_length > len) ? len : mavlink_rx_length;
    memcpy(buf, mavlink_rx_buffer, copy_len);

    /* Clear flag to indicate buffer consumed */
    mavlink_message_received = false;

    if (copy_len < mavlink_rx_length) {
        PX4_WARN("MAVLink OpenAMP: Read buffer too small (%zu < %zu), data truncated",
                 len, mavlink_rx_length);
        mavlink_error_count++;
    }

    return (int)copy_len;
}

/**
 * @brief Write data to OpenAMP MAVLink channel
 *
 * Sends data via RPMsg to CA55, which forwards to UDP:14550.
 */
int rzv_uart_openamp_write(rzv_uart_backend_t *backend, const uint8_t *buf, size_t len)
{
    (void)backend;

    if (buf == NULL || len == 0) {
        return -EINVAL;
    }

    if (!mavlink_endpoint_ready) {
        return -ENOTCONN;
    }

    if (!mavlink_endpoint_bound) {
        PX4_WARN("MAVLink OpenAMP: Endpoint not bound");
        return -ENOTCONN;
    }

    /* Check if destination is known */
    if (rpmsg_ept_mavlink.dest_addr == RPMSG_ADDR_ANY) {
        PX4_WARN("MAVLink OpenAMP: Destination address unknown, waiting for CA55");
        return -EAGAIN;
    }

    /* Send via RPMsg */
    int ret = rpmsg_send_offchannel_raw(&rpmsg_ept_mavlink,
                                        rpmsg_ept_mavlink.addr,
                                        rpmsg_ept_mavlink.dest_addr,
                                        buf,
                                        (int)len,
                                        false);

    if (ret == RPMSG_ERR_NO_BUFF) {
        mavlink_error_count++;
        if (mavlink_error_count % 100 == 1) {
            PX4_WARN("MAVLink OpenAMP: RPMsg buffers full, dropping packet");
        }
        return -EAGAIN;
    }

    if (ret < 0) {
        PX4_ERR("MAVLink OpenAMP: RPMsg send failed: %d", ret);
        mavlink_error_count++;
        return -EIO;
    }

    mavlink_tx_count++;

    return (int)len;
}

/**
 * @brief Shutdown OpenAMP MAVLink transport
 */
void rzv_uart_openamp_shutdown(rzv_uart_backend_t *backend)
{
    if (!mavlink_endpoint_ready) {
        return;
    }

    PX4_INFO("MAVLink OpenAMP: Shutting down (TX=%" PRIu32 ", RX=%" PRIu32 ", errors=%" PRIu32 ")",
             mavlink_tx_count, mavlink_rx_count, mavlink_error_count);

    /* Destroy RPMsg endpoint */
    rpmsg_destroy_ept(&rpmsg_ept_mavlink);

    mavlink_endpoint_ready = false;
    mavlink_endpoint_bound = false;

    if (backend) {
        backend->opened = false;
    }

    PX4_INFO("MAVLink OpenAMP: Shutdown complete");
}

/**
 * @brief Print MAVLink OpenAMP status (for debugging)
 */
void rzv_uart_openamp_print_status(void)
{
    PX4_INFO("MAVLink OpenAMP Status:");
    PX4_INFO("  Endpoint ready: %s", mavlink_endpoint_ready ? "YES" : "NO");
    PX4_INFO("  Endpoint bound: %s", mavlink_endpoint_bound ? "YES" : "NO");
    PX4_INFO("  Service name: %s", RPMSG_SERVICE_MAVLINK);
    PX4_INFO("  Local addr: 0x%" PRIx32, rpmsg_ept_mavlink.addr);
    PX4_INFO("  Dest addr: 0x%" PRIx32, rpmsg_ept_mavlink.dest_addr);
    PX4_INFO("  TX count: %" PRIu32, mavlink_tx_count);
    PX4_INFO("  RX count: %" PRIu32, mavlink_rx_count);
    PX4_INFO("  Error count: %" PRIu32, mavlink_error_count);
    PX4_INFO("  RX pending: %s (%zu bytes)",
             mavlink_message_received ? "YES" : "NO", mavlink_rx_length);
}

#else // !__PX4_FREERTOS

/* Stub implementations for non-FreeRTOS builds */

int rzv_uart_openamp_init(rzv_uart_backend_t *backend, uint8_t logical_channel)
{
    (void)backend;
    (void)logical_channel;
    return -ENOTSUP;
}

int rzv_uart_openamp_read(rzv_uart_backend_t *backend, uint8_t *buf, size_t len)
{
    (void)backend;
    (void)buf;
    (void)len;
    return -ENOTSUP;
}

int rzv_uart_openamp_write(rzv_uart_backend_t *backend, const uint8_t *buf, size_t len)
{
    (void)backend;
    (void)buf;
    (void)len;
    return -ENOTSUP;
}

void rzv_uart_openamp_shutdown(rzv_uart_backend_t *backend)
{
    (void)backend;
}

void rzv_uart_openamp_print_status(void)
{
}

#endif // __PX4_FREERTOS
