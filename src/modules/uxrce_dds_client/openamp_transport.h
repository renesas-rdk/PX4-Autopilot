/****************************************************************************
 *
 * Copyright (c) 2025 Renesas Electronics Corporation and/or its affiliates
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 ****************************************************************************/

#ifndef OPENAMP_TRANSPORT_H
#define OPENAMP_TRANSPORT_H

#if defined(__PX4_FREERTOS)


#include <stdint.h>
#include <stdbool.h>
#include "uxr/client/profile/transport/custom/custom_transport.h"

// External declarations for RPMsg/OpenAMP functions used by PX4
extern void *platform;
extern struct rpmsg_endpoint rpmsg_ept;        // RPC endpoint (service-0)
extern struct rpmsg_endpoint rpmsg_ept_uxrce;  // UXRCE-DDS endpoint (service-1)
extern struct rpmsg_device *rpdev_uxrce;
extern volatile int evt_svc_unbind;        // For RPC endpoint
extern volatile int evt_svc_unbind_uxrce;  // For UXRCE-DDS endpoint

#ifdef __cplusplus
extern "C" {
#endif

// PX4 custom transport functions - prefixed to avoid naming conflicts
bool px4_custom_transport_open(struct uxrCustomTransport* transport);
bool px4_custom_transport_close(struct uxrCustomTransport* transport);
size_t px4_custom_transport_write(struct uxrCustomTransport* transport, const uint8_t* buf, size_t len, uint8_t* error);
size_t px4_custom_transport_read(struct uxrCustomTransport* transport, uint8_t* buf, size_t len, int timeout, uint8_t* error);

// Service callbacks for RPMsg
int px4_rpmsg_recv_callback(struct rpmsg_endpoint *ept, void *data, size_t len, uint32_t src, void *priv);
void px4_rpmsg_service_unbind(struct rpmsg_endpoint *ept);

// Additional utility function
void px4_custom_transport_print_status(void);

// Forward UXRCE-DDS messages from RPC callback
// Returns true if message was accepted, false if buffer full or invalid
bool px4_uxrce_forward_message(const void *data, size_t len);

// Global OpenAMP transport lock to serialize RPC and UXRCE-DDS operations
// Prevents deadlock from shared virtio device mutex contention
void px4_openamp_transport_lock(void);
void px4_openamp_transport_unlock(void);

#ifdef __cplusplus
}
#endif


#endif /* __PX4_FREERTOS */

#endif // OPENAMP_TRANSPORT_H
