/****************************************************************************
 *
 * Copyright (c) 2025 Renesas Electronics Corporation and/or its affiliates
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 ****************************************************************************/


#if defined(__PX4_FREERTOS)

#include "openamp_transport.h"
#include "openamp_rpc_client.h"
#include "openamp_rpc.h"
#include <px4_platform_common/log.h>
#include <FreeRTOS_POSIX/time.h>
#include <FreeRTOS_POSIX/unistd.h>
#include <string.h>
#include <stdio.h>
#include <inttypes.h>
#include <uxr/client/util/time.h>

#include "openamp/open_amp.h"
#include "openamp/rpmsg.h"

// IRQ-safe critical section for ring buffer protection
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

// Semaphore to notify when new data is available in the ring buffer
// Instead of polling every 10ms, the task will block on this semaphore
static SemaphoreHandle_t rx_data_sem = NULL;
static StaticSemaphore_t rx_data_sem_buffer;

// Define buffer sizes
#define RX_BUFFER_SIZE 2048
#define RX_BUFFER_COUNT 4  // 4 buffers = 8KB total

// Ring buffer slot structure
struct RxBufferSlot {
	uint8_t data[RX_BUFFER_SIZE];
	size_t length;
	volatile bool occupied;
};

// Ring buffer for receiving data
static RxBufferSlot rx_ring_buffer[RX_BUFFER_COUNT];
static volatile uint32_t rx_write_idx = 0;  // Written by ISR/callback
static volatile uint32_t rx_read_idx = 0;   // Read by transport_read
static volatile uint32_t rx_buffer_drops = 0;  // Track overflow
static uint32_t rx_buffer_peak_usage = 0;

static bool is_initialized = false;

// Statistics
static uint32_t bytes_sent = 0;
static uint32_t bytes_received = 0;
static uint32_t send_errors = 0;
static uint32_t receive_errors = 0;
static uint32_t connection_attempts = 0;

// Connection state
static bool endpoint_bound = true;  // Assume initially bound until unbind occurs

static void update_endpoint_bound()
{
	if (evt_svc_unbind_uxrce) {
		endpoint_bound = false;
	}
}

// Global rpmsg endpoints - initialized by the main system
extern struct rpmsg_endpoint rpmsg_ept;        // RPC endpoint (service-0) - NOT used here
extern struct rpmsg_endpoint rpmsg_ept_uxrce;  // UXRCE-DDS endpoint (service-1) - used by this transport

// Global rpmsg device - initialized by the main system
extern struct rpmsg_device* rpdev_uxrce;

// Import the is_rpmsg_ept_ready function from the microros_transport.c file
// This ensures we use the same function that MainTask_entry uses
extern "C" {
    unsigned int is_rpmsg_ept_ready(struct rpmsg_endpoint *ept);
}

// Function to check transport status
void px4_custom_transport_print_status(void) {
    uint32_t current_usage = (rx_write_idx - rx_read_idx);
    PX4_INFO("Transport status: initialized=%s, bound=%s, sent=%" PRIu32 " bytes, received=%" PRIu32
             " bytes, errors=%" PRIu32 "/%" PRIu32 ", conn_attempts=%" PRIu32,
             is_initialized ? "yes" : "no", endpoint_bound ? "yes" : "no",
             bytes_sent, bytes_received, send_errors, receive_errors, connection_attempts);
    PX4_INFO("RX ring: drops=%" PRIu32 ", peak=%" PRIu32 "/%d, current=%" PRIu32,
             rx_buffer_drops, rx_buffer_peak_usage, RX_BUFFER_COUNT, current_usage);
}

// Special message types for handshaking
#define MSG_TYPE_HANDSHAKE_REQUEST 0xA5A5A501
#define MSG_TYPE_HANDSHAKE_RESPONSE 0xA5A5A502
#define MSG_TYPE_SHUTDOWN 0xEF56A55A

// Send a handshake request to verify the connection
static bool send_handshake_request(void) {
    uint32_t handshake_msg = MSG_TYPE_HANDSHAKE_REQUEST;

    // Use rpmsg_trysend() to avoid holding transport lock across a blocking wait
    px4_openamp_transport_lock();
    int result = rpmsg_trysend(&rpmsg_ept_uxrce, &handshake_msg, sizeof(handshake_msg));
    px4_openamp_transport_unlock();

    if (result < 0) {
        PX4_WARN("Handshake request failed: %d", result);
        return false;
    }

    PX4_DEBUG("Sent handshake request");
    return true;
}

// Implementation of our transport functions
bool px4_custom_transport_open(struct uxrCustomTransport* transport) {
    (void)transport;
    PX4_DEBUG("Opening custom OpenAMP transport");
    connection_attempts++;

    if (rx_data_sem == NULL) {
        rx_data_sem = xSemaphoreCreateBinaryStatic(&rx_data_sem_buffer);
        if (rx_data_sem == NULL) {
            PX4_ERR("Failed to create RX data semaphore");
            return false;
        }
    }

    // Reset ring buffer and statistics for this connection attempt
    for (int i = 0; i < RX_BUFFER_COUNT; i++) {
        rx_ring_buffer[i].occupied = false;
        rx_ring_buffer[i].length = 0;
    }
    rx_write_idx = 0;
    rx_read_idx = 0;
    endpoint_bound = true;
    evt_svc_unbind_uxrce = 0;

    // Clear semaphore state (in case it was signaled previously)
    xSemaphoreTake(rx_data_sem, 0);

    // Check if the endpoint exists and is properly initialized
    if (!rpdev_uxrce) {
        PX4_ERR("UXRCE RPMsg device is NULL");
        return false;
    }

    // For slave endpoints (CR8), is_rpmsg_ept_ready() will return false until
    // the master (CA55) connects. We should only require that the endpoint is bound.
    bool is_ready = is_rpmsg_ept_ready(&rpmsg_ept_uxrce);

    // Debug the RPMsg endpoint state
    PX4_DEBUG("UXRCE-DDS RPMsg endpoint state: rdev=%p, addr=0x%x, dest_addr=0x%x, ready=%d, bound=%d",
             rpmsg_ept_uxrce.rdev,
             static_cast<unsigned int>(rpmsg_ept_uxrce.addr),
             static_cast<unsigned int>(rpmsg_ept_uxrce.dest_addr),
             is_ready,
             endpoint_bound);

    if (!endpoint_bound) {
        PX4_ERR("RPMsg endpoint is not bound - OpenAMP initialization failed");
        return false;
    }

    if (!is_ready) {
        PX4_DEBUG("RPMsg endpoint is bound but not ready (waiting for master connection)");
        PX4_DEBUG("This is normal for slave endpoints - transport will work once master connects");
    } else {
        PX4_DEBUG("RPMsg endpoint is ready and bound");
    }

    // Send a handshake to verify bidirectional communication works
    if (!send_handshake_request()) {
        PX4_WARN("Failed to send initial handshake - connection may be unstable");
        // Continue anyway, as this might still work
    }

    is_initialized = true;
    PX4_DEBUG("OpenAMP transport opened successfully");
    return true;
}

bool px4_custom_transport_close(struct uxrCustomTransport* transport) {
    (void)transport;
    PX4_DEBUG("Closing custom OpenAMP transport (sent: %" PRIu32 " bytes, received: %" PRIu32
             " bytes, errors: %" PRIu32 "/%" PRIu32 ")",
             bytes_sent, bytes_received, send_errors, receive_errors);

    is_initialized = false;
    return true;
}

size_t px4_custom_transport_write(struct uxrCustomTransport* transport, const uint8_t* buf, size_t len, uint8_t* error) {
    (void)transport;

    update_endpoint_bound();
    if (!is_initialized || !endpoint_bound) {
        PX4_ERR("Transport not initialized or endpoint not bound");
        *error = 1;
        return 0;
    }

    // Acquire transport lock to prevent RPC/UXRCE-DDS contention for shared virtio device.
    // Use rpmsg_trysend() (non-blocking) instead of rpmsg_send() (can block up to 3s holding lock).
    // Holding the lock for 3s while vring TX is full causes priority inversion: the high-priority
    // logger writer task (SCHED_PRIORITY_ATTITUDE_CONTROL) spins on the lock with 1ms yields,
    // consuming CPU and starving the MAVLink task → QGC heartbeat timeout → connection lost.
    px4_openamp_transport_lock();
    int result = rpmsg_trysend(&rpmsg_ept_uxrce, (void*)buf, len);
    px4_openamp_transport_unlock();

    if (result == RPMSG_ERR_NO_BUFF) {
        /* TX vring temporarily full (CA55 slow to consume). Release lock, wait 2ms for
         * virtio to reclaim used buffers, then retry once. UXRCE-DDS will retry at its
         * own rate if this also fails. */
        usleep(2000);
        px4_openamp_transport_lock();
        result = rpmsg_trysend(&rpmsg_ept_uxrce, (void*)buf, len);
        px4_openamp_transport_unlock();
    }

    if (result < 0) {
        PX4_ERR("Failed to send message: %d (no_buf=%s)", result,
                result == RPMSG_ERR_NO_BUFF ? "yes" : "no");
        *error = 1;
        send_errors++;
        return 0;
    }

    bytes_sent += len;
    *error = 0;
    return len;
}

// Custom transport read function - uses buffer filled by the external rpmsg_recv_callback
size_t px4_custom_transport_read(struct uxrCustomTransport* transport, uint8_t* buf, size_t len, int timeout, uint8_t* error) {
    (void)transport;

    update_endpoint_bound();
    if (!is_initialized || !endpoint_bound) {
        PX4_ERR("Transport not initialized or endpoint not bound");
        *error = 1;
        return 0;
    }

    // Determine wait ticks based on timeout parameter
    // timeout = 0: non-blocking, timeout < 0: block indefinitely, timeout > 0: block with timeout
    TickType_t wait_ticks;
    if (timeout < 0) {
        wait_ticks = portMAX_DELAY;
    } else if (timeout == 0) {
        wait_ticks = 0;
    } else {
        wait_ticks = pdMS_TO_TICKS(timeout);
        if (wait_ticks == 0 && timeout > 0) {
            wait_ticks = 1;  // Đảm bảo ít nhất 1 tick nếu timeout > 0
        }
    }

    // Wait for data using semaphore instead of polling
    // Semaphore is signaled by px4_uxrce_forward_message() when new data arrives
    while (true) {
        // Check if there is data available in the ring buffer
        uint32_t read_slot = rx_read_idx % RX_BUFFER_COUNT;
        if (rx_ring_buffer[read_slot].occupied) {
            break;  // Data is available
        }

        // Wait on semaphore for new data
        // Task will BLOCK here instead of polling, freeing CPU for other tasks
        BaseType_t sem_result = xSemaphoreTake(rx_data_sem, wait_ticks);

        // Check if endpoint is still valid
        update_endpoint_bound();
        if (!endpoint_bound) {
            PX4_WARN("UXRCE-DDS RPMsg endpoint no longer bound during read operation");
            *error = 1;
            return 0;
        }

        // If timeout occurred
        if (sem_result != pdTRUE) {
            // Check the buffer again (possible race condition)
            read_slot = rx_read_idx % RX_BUFFER_COUNT;
            if (rx_ring_buffer[read_slot].occupied) {
                break;  // Data became available just before timeout
            }
            *error = 0;  // Timeout is not an error
            return 0;
        }

        // Semaphore is signaled, check buffer again (loop to verify)
    }

    // We have a message in the ring buffer
    uint32_t read_slot = rx_read_idx % RX_BUFFER_COUNT;
    RxBufferSlot* slot = &rx_ring_buffer[read_slot];

    size_t copy_len = (slot->length > len) ? len : slot->length;
    memcpy(buf, slot->data, copy_len);

    // Mark slot as free and increment read index
    taskENTER_CRITICAL();
    slot->occupied = false;
    rx_read_idx++;
    taskEXIT_CRITICAL();

    *error = 0;
    return copy_len;
}

// Forward UXRCE-DDS messages from RPC callback
// This function is called from C code (main_task_entry.c) when a non-RPC message arrives
extern "C" bool px4_uxrce_forward_message(const void *data, size_t len)
{
    // Validate inputs
    if (!data || len == 0 || len > RX_BUFFER_SIZE) {
        return false;
    }

    // Calculate write slot
    taskENTER_CRITICAL();
    uint32_t write_slot = rx_write_idx % RX_BUFFER_COUNT;
    RxBufferSlot* slot = &rx_ring_buffer[write_slot];

    // Check if ring buffer is full (slot occupied)
    if (slot->occupied) {
        rx_buffer_drops++;
        receive_errors++;
        taskEXIT_CRITICAL();
        return false;  // Buffer full, drop message (UXRCE-DDS will retry)
    }
    taskEXIT_CRITICAL();

    // Copy message to ring buffer slot outside critical section
    memcpy(slot->data, data, len);
    slot->length = len;
    bytes_received += len;

    // Mark slot as occupied and increment write index
    taskENTER_CRITICAL();
    slot->occupied = true;
    rx_write_idx++;

    // Update peak usage statistics
    uint32_t current_usage = (rx_write_idx - rx_read_idx);
    if (current_usage > rx_buffer_peak_usage) {
        rx_buffer_peak_usage = current_usage;
    }
    taskEXIT_CRITICAL();

    // Signal semaphore to wake up any waiting read task
    // This function may be called from ISR (OpenAMP callback) or task context
    if (rx_data_sem != NULL) {
        // Check if currently in ISR context (Cortex-R8 mode check)
        uint32_t cpsr;
        __asm volatile("MRS %0, CPSR" : "=r"(cpsr));
        const uint32_t mode = cpsr & 0x1FU;
        const bool in_isr = (mode != 0x1FU) && (mode != 0x10U);  // Not System/User mode
        if (in_isr) {
            BaseType_t higher_priority_woken = pdFALSE;
            xSemaphoreGiveFromISR(rx_data_sem, &higher_priority_woken);
            portYIELD_FROM_ISR(higher_priority_woken);
        } else {
            xSemaphoreGive(rx_data_sem);
        }
    }

    return true;
}

#endif /* __PX4_FREERTOS */
