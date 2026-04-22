/****************************************************************************
 *
 * Copyright (c) 2025 Renesas Electronics Corporation and/or its affiliates
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 ****************************************************************************/

/**
 * @file posix_socket_stub.c
 * @brief POSIX socket stub implementation for Renesas RZ/V2H
 */

#if defined(__PX4_FREERTOS)

#include "FreeRTOS.h"
#include "task.h"
#include "FreeRTOS_POSIX.h"
#include <sys/socket.h>

#include <errno.h>
#include <pthread.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "openamp_rpc_client.h"
#include "openamp_rpc.h"

#define REMOTE_SOCKET_BASE 5000
#define REMOTE_SOCKET_SLOTS 32

struct remote_socket_entry {
    int remote_fd;
};

static pthread_mutex_t g_socket_lock = PTHREAD_MUTEX_INITIALIZER;
static struct remote_socket_entry g_remote_sockets[REMOTE_SOCKET_SLOTS];
static bool g_socket_initialized = false;

#define SOCKET_RPC_TIMEOUT_MS   100U
#define SOCKET_RPC_MAX_RETRIES    3U

static void init_socket_table_locked(void)
{
    if (g_socket_initialized) {
        return;
    }

    for (size_t i = 0; i < REMOTE_SOCKET_SLOTS; ++i) {
        g_remote_sockets[i].remote_fd = -1;
    }

    g_socket_initialized = true;
}

static int perform_socket_rpc(uint16_t opcode,
                              const void *payload,
                              size_t payload_len,
                              void *response_buffer,
                              size_t response_capacity,
                              size_t *response_len,
                              int32_t *ret_val_out)
{
    for (uint32_t attempt = 0U; attempt < SOCKET_RPC_MAX_RETRIES; ++attempt) {
        int32_t ret_val = -1;
        int32_t err_val = 0;

        if (px4_openamp_rpc_call_timeout(PX4_RPC_CATEGORY_SOCKET,
                                         opcode,
                                         payload,
                                         payload_len,
                                         response_buffer,
                                         response_capacity,
                                         response_len,
                                         &ret_val,
                                         &err_val,
                                         SOCKET_RPC_TIMEOUT_MS) < 0) {
            /* Transport timeout: retry up to SOCKET_RPC_MAX_RETRIES times */
            continue;
        }

        /* Application-level error from CA55: do not retry */
        errno = err_val;

        if (err_val != 0) {
            return -1;
        }

        if (ret_val_out) {
            *ret_val_out = ret_val;
        }

        return 0;
    }

    /* All retries exhausted (max wait: SOCKET_RPC_MAX_RETRIES * SOCKET_RPC_TIMEOUT_MS ms) */
    return -1;
}

static int allocate_socket_fd(int remote_fd)
{
    pthread_mutex_lock(&g_socket_lock);
    init_socket_table_locked();

    for (size_t i = 0; i < REMOTE_SOCKET_SLOTS; ++i) {
        if (g_remote_sockets[i].remote_fd < 0) {
            g_remote_sockets[i].remote_fd = remote_fd;
            int local_fd = REMOTE_SOCKET_BASE + (int)i;
            pthread_mutex_unlock(&g_socket_lock);
            return local_fd;
        }
    }

    pthread_mutex_unlock(&g_socket_lock);
    errno = EMFILE;
    return -1;
}

static int get_remote_fd(int local_fd, int *remote_fd_out, size_t *slot_index_out)
{
    pthread_mutex_lock(&g_socket_lock);
    init_socket_table_locked();

    if (local_fd < REMOTE_SOCKET_BASE) {
        pthread_mutex_unlock(&g_socket_lock);
        errno = EBADF;
        return -1;
    }

    size_t slot = (size_t)(local_fd - REMOTE_SOCKET_BASE);

    if (slot >= REMOTE_SOCKET_SLOTS) {
        pthread_mutex_unlock(&g_socket_lock);
        errno = EBADF;
        return -1;
    }

    int remote_fd = g_remote_sockets[slot].remote_fd;

    if (remote_fd < 0) {
        pthread_mutex_unlock(&g_socket_lock);
        errno = EBADF;
        return -1;
    }

    if (remote_fd_out) {
        *remote_fd_out = remote_fd;
    }

    if (slot_index_out) {
        *slot_index_out = slot;
    }

    pthread_mutex_unlock(&g_socket_lock);
    return 0;
}

static int close_remote_fd_direct(int remote_fd)
{
    struct {
        int32_t fd;
    } req = {
        .fd = remote_fd,
    };

    return perform_socket_rpc(PX4_RPC_CLOSE_SOCKET, &req, sizeof(req), NULL, 0, NULL, NULL);
}

bool rzv_socket_is_remote_fd(int fd)
{
    bool remote = false;

    pthread_mutex_lock(&g_socket_lock);
    init_socket_table_locked();

    if (fd >= REMOTE_SOCKET_BASE) {
        size_t slot = (size_t)(fd - REMOTE_SOCKET_BASE);

        if (slot < REMOTE_SOCKET_SLOTS && g_remote_sockets[slot].remote_fd >= 0) {
            remote = true;
        }
    }

    pthread_mutex_unlock(&g_socket_lock);
    return remote;
}

int rzv_socket_close_remote(int fd)
{
    size_t slot = 0;
    int remote_fd = 0;

    if (get_remote_fd(fd, &remote_fd, &slot) < 0) {
        return -1;
    }

    pthread_mutex_lock(&g_socket_lock);

    if (g_remote_sockets[slot].remote_fd != remote_fd) {
        pthread_mutex_unlock(&g_socket_lock);
        errno = EBADF;
        return -1;
    }

    g_remote_sockets[slot].remote_fd = -2;
    pthread_mutex_unlock(&g_socket_lock);

    if (close_remote_fd_direct(remote_fd) < 0) {
        pthread_mutex_lock(&g_socket_lock);

        if (g_remote_sockets[slot].remote_fd == -2) {
            g_remote_sockets[slot].remote_fd = remote_fd;
        }

        pthread_mutex_unlock(&g_socket_lock);
        return -1;
    }

    pthread_mutex_lock(&g_socket_lock);

    if (g_remote_sockets[slot].remote_fd == -2) {
        g_remote_sockets[slot].remote_fd = -1;
    }

    pthread_mutex_unlock(&g_socket_lock);
    errno = 0;
    return 0;
}

int socket(int domain, int type, int protocol)
{
    struct {
        int32_t domain;
        int32_t type;
        int32_t protocol;
    } req = {
        .domain = domain,
        .type = type,
        .protocol = protocol,
    };

    int32_t remote_fd = -1;

    if (perform_socket_rpc(PX4_RPC_SOCKET, &req, sizeof(req), NULL, 0, NULL, &remote_fd) < 0) {
        return -1;
    }

    int local_fd = allocate_socket_fd(remote_fd);

    if (local_fd < 0) {
        close_remote_fd_direct(remote_fd);
        return -1;
    }

    errno = 0;
    return local_fd;
}

int bind(int sockfd, const struct sockaddr *addr, socklen_t addrlen)
{
    int remote_fd = 0;

    if (get_remote_fd(sockfd, &remote_fd, NULL) < 0) {
        return -1;
    }

    if ((addr == NULL && addrlen > 0) || (addr != NULL && addrlen == 0)) {
        errno = EINVAL;
        return -1;
    }

    struct {
        int32_t fd;
        int32_t addrlen;
    } header = {
        .fd = remote_fd,
        .addrlen = (int32_t)addrlen,
    };

    size_t payload_len = sizeof(header) + (size_t)addrlen;
    uint8_t stack_buf[128];
    uint8_t *payload = stack_buf;

    if (payload_len > sizeof(stack_buf)) {
        payload = (uint8_t *)malloc(payload_len);

        if (!payload) {
            errno = ENOMEM;
            return -1;
        }
    }

    memcpy(payload, &header, sizeof(header));

    if (addrlen > 0 && addr) {
        memcpy(payload + sizeof(header), addr, addrlen);
    }

    int result = perform_socket_rpc(PX4_RPC_BIND, payload, payload_len, NULL, 0, NULL, NULL);

    if (payload != stack_buf) {
        free(payload);
    }

    return (result == 0) ? 0 : -1;
}

int connect(int sockfd, const struct sockaddr *addr, socklen_t addrlen)
{
    int remote_fd = 0;

    if (get_remote_fd(sockfd, &remote_fd, NULL) < 0) {
        return -1;
    }

    if ((addr == NULL && addrlen > 0) || (addr != NULL && addrlen == 0)) {
        errno = EINVAL;
        return -1;
    }

    struct {
        int32_t fd;
        int32_t addrlen;
    } header = {
        .fd = remote_fd,
        .addrlen = (int32_t)addrlen,
    };

    size_t payload_len = sizeof(header) + (size_t)addrlen;
    uint8_t stack_buf[128];
    uint8_t *payload = stack_buf;

    if (payload_len > sizeof(stack_buf)) {
        payload = (uint8_t *)malloc(payload_len);

        if (!payload) {
            errno = ENOMEM;
            return -1;
        }
    }

    memcpy(payload, &header, sizeof(header));

    if (addrlen > 0 && addr) {
        memcpy(payload + sizeof(header), addr, addrlen);
    }

    int result = perform_socket_rpc(PX4_RPC_CONNECT, payload, payload_len, NULL, 0, NULL, NULL);

    if (payload != stack_buf) {
        free(payload);
    }

    return (result == 0) ? 0 : -1;
}

int listen(int sockfd, int backlog)
{
    int remote_fd = 0;

    if (get_remote_fd(sockfd, &remote_fd, NULL) < 0) {
        return -1;
    }

    struct {
        int32_t fd;
        int32_t backlog;
    } req = {
        .fd = remote_fd,
        .backlog = backlog,
    };

    return (perform_socket_rpc(PX4_RPC_LISTEN, &req, sizeof(req), NULL, 0, NULL, NULL) == 0) ? 0 : -1;
}

int accept(int sockfd, struct sockaddr *addr, socklen_t *addrlen)
{
    int remote_fd = 0;

    if (get_remote_fd(sockfd, &remote_fd, NULL) < 0) {
        return -1;
    }

    if ((addr == NULL) != (addrlen == NULL)) {
        errno = EINVAL;
        return -1;
    }

    int32_t capacity = 0;

    if (addr && addrlen) {
        capacity = (int32_t)*addrlen;
    }

    struct {
        int32_t fd;
        int32_t addrlen;
    } req = {
        .fd = remote_fd,
        .addrlen = capacity,
    };

    size_t response_capacity = sizeof(int32_t) + ((capacity > 0) ? (size_t)capacity : 0);
    uint8_t stack_buf[128];
    uint8_t *response = NULL;
    bool heap_buffer = false;

    if (response_capacity <= sizeof(stack_buf)) {
        response = stack_buf;
    } else {
        response = (uint8_t *)malloc(response_capacity);

        if (!response) {
            errno = ENOMEM;
            return -1;
        }

        heap_buffer = true;
    }

    size_t response_len = 0;
    int32_t new_remote_fd = -1;

    int rpc_result = perform_socket_rpc(PX4_RPC_ACCEPT,
                                        &req,
                                        sizeof(req),
                                        response,
                                        response_capacity,
                                        &response_len,
                                        &new_remote_fd);

    if (rpc_result < 0) {
        if (heap_buffer) {
            free(response);
        }

        return -1;
    }

    if (response_len > response_capacity) {
        if (heap_buffer) {
            free(response);
        }

        errno = EMSGSIZE;
        close_remote_fd_direct(new_remote_fd);
        return -1;
    }

    if (addr && addrlen) {
        socklen_t actual_len = 0;

        if (response_len >= sizeof(int32_t)) {
            int32_t remote_addrlen = 0;
            memcpy(&remote_addrlen, response, sizeof(remote_addrlen));

            if (remote_addrlen < 0) {
                remote_addrlen = 0;
            }

            actual_len = (socklen_t)remote_addrlen;
            size_t available = response_len - sizeof(int32_t);
            size_t copy_len = (available < (size_t)actual_len) ? available : (size_t)actual_len;

            if (copy_len > (size_t)*addrlen) {
                if (heap_buffer) {
                    free(response);
                }

                errno = EMSGSIZE;
                close_remote_fd_direct(new_remote_fd);
                return -1;
            }

            if (copy_len > 0) {
                memcpy(addr, response + sizeof(int32_t), copy_len);
            }

            *addrlen = actual_len;

        } else {
            *addrlen = 0;
        }
    }

    if (heap_buffer) {
        free(response);
    }

    int local_fd = allocate_socket_fd(new_remote_fd);

    if (local_fd < 0) {
        close_remote_fd_direct(new_remote_fd);
        return -1;
    }

    errno = 0;
    return local_fd;
}

int setsockopt(int sockfd, int level, int optname, const void *optval, socklen_t optlen)
{
    int remote_fd = 0;

    if (get_remote_fd(sockfd, &remote_fd, NULL) < 0) {
        return -1;
    }

    if ((optval == NULL && optlen > 0) || (optval != NULL && optlen == 0)) {
        errno = EINVAL;
        return -1;
    }

    struct {
        int32_t fd;
        int32_t level;
        int32_t optname;
        int32_t optlen;
    } header = {
        .fd = remote_fd,
        .level = level,
        .optname = optname,
        .optlen = (int32_t)optlen,
    };

    size_t payload_len = sizeof(header) + (size_t)optlen;
    uint8_t stack_buf[128];
    uint8_t *payload = stack_buf;

    if (payload_len > sizeof(stack_buf)) {
        payload = (uint8_t *)malloc(payload_len);

        if (!payload) {
            errno = ENOMEM;
            return -1;
        }
    }

    memcpy(payload, &header, sizeof(header));

    if (optlen > 0 && optval) {
        memcpy(payload + sizeof(header), optval, optlen);
    }

    int result = perform_socket_rpc(PX4_RPC_SETSOCKOPT, payload, payload_len, NULL, 0, NULL, NULL);

    if (payload != stack_buf) {
        free(payload);
    }

    return (result == 0) ? 0 : -1;
}

int shutdown(int sockfd, int how)
{
    int remote_fd = 0;

    if (get_remote_fd(sockfd, &remote_fd, NULL) < 0) {
        return -1;
    }

    struct {
        int32_t fd;
        int32_t how;
    } req = {
        .fd = remote_fd,
        .how = how,
    };

    return (perform_socket_rpc(PX4_RPC_SHUTDOWN, &req, sizeof(req), NULL, 0, NULL, NULL) == 0) ? 0 : -1;
}

ssize_t send(int sockfd, const void *buf, size_t len, int flags)
{
    int remote_fd = 0;

    if (get_remote_fd(sockfd, &remote_fd, NULL) < 0) {
        return -1;
    }

    if (len > 0 && buf == NULL) {
        errno = EINVAL;
        return -1;
    }

    if (len > UINT32_MAX) {
        errno = EINVAL;
        return -1;
    }

    struct {
        int32_t fd;
        int32_t flags;
        uint32_t len;
    } header = {
        .fd = remote_fd,
        .flags = flags,
        .len = (uint32_t)len,
    };

    size_t payload_len = sizeof(header) + len;
    /* 512 bytes covers MAVLink v2 max frame (~280 bytes) + header (12 bytes)
     * without heap allocation in the common case (M1 hot-path fix). */
    uint8_t stack_buf[512];
    uint8_t *payload = stack_buf;

    if (payload_len > sizeof(stack_buf)) {
        payload = (uint8_t *)malloc(payload_len);

        if (!payload) {
            errno = ENOMEM;
            return -1;
        }
    }

    memcpy(payload, &header, sizeof(header));

    if (len > 0 && buf) {
        memcpy(payload + sizeof(header), buf, len);
    }

    int32_t ret_val = 0;
    int result = perform_socket_rpc(PX4_RPC_SEND, payload, payload_len, NULL, 0, NULL, &ret_val);

    if (payload != stack_buf) {
        free(payload);
    }

    if (result < 0) {
        return -1;
    }

    errno = 0;
    return (ssize_t)ret_val;
}

ssize_t recv(int sockfd, void *buf, size_t len, int flags)
{
    int remote_fd = 0;

    if (get_remote_fd(sockfd, &remote_fd, NULL) < 0) {
        return -1;
    }

    if (len > 0 && buf == NULL) {
        errno = EINVAL;
        return -1;
    }

    if (len > UINT32_MAX) {
        errno = EINVAL;
        return -1;
    }

    struct {
        int32_t fd;
        int32_t flags;
        uint32_t len;
    } req = {
        .fd = remote_fd,
        .flags = flags,
        .len = (uint32_t)len,
    };

    size_t response_len = 0;
    int32_t ret_val = 0;

    if (perform_socket_rpc(PX4_RPC_RECV,
                           &req,
                           sizeof(req),
                           buf,
                           len,
                           &response_len,
                           &ret_val) < 0) {
        return -1;
    }

    if ((size_t)ret_val > len || response_len > len) {
        errno = EMSGSIZE;
        return -1;
    }

    errno = 0;
    return (ssize_t)ret_val;
}

int getsockname(int sockfd, struct sockaddr *addr, socklen_t *addrlen)
{
    int remote_fd = 0;

    if (!addr || !addrlen) {
        errno = EINVAL;
        return -1;
    }

    if (get_remote_fd(sockfd, &remote_fd, NULL) < 0) {
        return -1;
    }

    int32_t capacity = (int32_t)*addrlen;

    struct {
        int32_t fd;
        int32_t addrlen;
    } req = {
        .fd = remote_fd,
        .addrlen = capacity,
    };

    size_t response_capacity = sizeof(int32_t) + (capacity > 0 ? (size_t)capacity : 0);
    uint8_t stack_buf[128];
    uint8_t *response = NULL;
    bool heap_buffer = false;

    if (response_capacity <= sizeof(stack_buf)) {
        response = stack_buf;
    } else {
        response = (uint8_t *)malloc(response_capacity);

        if (!response) {
            errno = ENOMEM;
            return -1;
        }

        heap_buffer = true;
    }

    size_t response_len = 0;

    if (perform_socket_rpc(PX4_RPC_GETSOCKNAME,
                           &req,
                           sizeof(req),
                           response,
                           response_capacity,
                           &response_len,
                           NULL) < 0) {
        if (heap_buffer) {
            free(response);
        }

        return -1;
    }

    if (response_len > response_capacity) {
        if (heap_buffer) {
            free(response);
        }

        errno = EMSGSIZE;
        return -1;
    }

    socklen_t actual_len = 0;

    if (response_len >= sizeof(int32_t)) {
        int32_t remote_size = 0;
        memcpy(&remote_size, response, sizeof(remote_size));

        if (remote_size < 0) {
            remote_size = 0;
        }

        actual_len = (socklen_t)remote_size;
        size_t available = response_len - sizeof(int32_t);
        size_t copy_len = (available < (size_t)actual_len) ? available : (size_t)actual_len;

        if (copy_len > (size_t)*addrlen) {
            if (heap_buffer) {
                free(response);
            }

            errno = EMSGSIZE;
            return -1;
        }

        if (copy_len > 0) {
            memcpy(addr, response + sizeof(int32_t), copy_len);
        }

    }

    *addrlen = actual_len;

    if (heap_buffer) {
        free(response);
    }

    errno = 0;
    return 0;
}

int getpeername(int sockfd, struct sockaddr *addr, socklen_t *addrlen)
{
    int remote_fd = 0;

    if (!addr || !addrlen) {
        errno = EINVAL;
        return -1;
    }

    if (get_remote_fd(sockfd, &remote_fd, NULL) < 0) {
        return -1;
    }

    int32_t capacity = (int32_t)*addrlen;

    struct {
        int32_t fd;
        int32_t addrlen;
    } req = {
        .fd = remote_fd,
        .addrlen = capacity,
    };

    size_t response_capacity = sizeof(int32_t) + (capacity > 0 ? (size_t)capacity : 0);
    uint8_t stack_buf[128];
    uint8_t *response = NULL;
    bool heap_buffer = false;

    if (response_capacity <= sizeof(stack_buf)) {
        response = stack_buf;
    } else {
        response = (uint8_t *)malloc(response_capacity);

        if (!response) {
            errno = ENOMEM;
            return -1;
        }

        heap_buffer = true;
    }

    size_t response_len = 0;

    if (perform_socket_rpc(PX4_RPC_GETPEERNAME,
                           &req,
                           sizeof(req),
                           response,
                           response_capacity,
                           &response_len,
                           NULL) < 0) {
        if (heap_buffer) {
            free(response);
        }

        return -1;
    }

    if (response_len > response_capacity) {
        if (heap_buffer) {
            free(response);
        }

        errno = EMSGSIZE;
        return -1;
    }

    socklen_t actual_len = 0;

    if (response_len >= sizeof(int32_t)) {
        int32_t remote_size = 0;
        memcpy(&remote_size, response, sizeof(remote_size));

        if (remote_size < 0) {
            remote_size = 0;
        }

        actual_len = (socklen_t)remote_size;
        size_t available = response_len - sizeof(int32_t);
        size_t copy_len = (available < (size_t)actual_len) ? available : (size_t)actual_len;

        if (copy_len > (size_t)*addrlen) {
            if (heap_buffer) {
                free(response);
            }

            errno = EMSGSIZE;
            return -1;
        }

        if (copy_len > 0) {
            memcpy(addr, response + sizeof(int32_t), copy_len);
        }

    }

    *addrlen = actual_len;

    if (heap_buffer) {
        free(response);
    }

    errno = 0;
    return 0;
}


int rzv_socket_ioctl_remote(int fd,
                            unsigned long request,
                            const void *write_data,
                            size_t write_len,
                            uint64_t scalar_value,
                            void *read_buffer,
                            size_t read_capacity,
                            size_t *out_read_len)
{
    int remote_fd = 0;

    if (get_remote_fd(fd, &remote_fd, NULL) < 0) {
        return -1;
    }

    if (write_len > 0 && write_data == NULL) {
        errno = EINVAL;
        return -1;
    }

    if (write_len > UINT32_MAX || read_capacity > UINT32_MAX) {
        errno = EINVAL;
        return -1;
    }

    struct socket_ioctl_request {
        int32_t fd;
        uint32_t reserved0;
        uint64_t request;
        uint32_t write_len;
        uint32_t read_len;
        uint64_t scalar_value;
    } __attribute__((packed));

    struct socket_ioctl_request header = {
        .fd = remote_fd,
        .reserved0 = 0,
        .request = request,
        .write_len = (uint32_t)write_len,
        .read_len = (uint32_t)read_capacity,
        .scalar_value = scalar_value,
    };

    size_t payload_len = sizeof(header) + write_len;
    /* 512 bytes covers typical ioctl write payloads without heap allocation (M1). */
    uint8_t stack_buf[512];
    uint8_t *payload = stack_buf;

    if (payload_len > sizeof(stack_buf)) {
        payload = (uint8_t *)malloc(payload_len);

        if (!payload) {
            errno = ENOMEM;
            return -1;
        }
    }

    memcpy(payload, &header, sizeof(header));

    if (write_len > 0 && write_data) {
        memcpy(payload + sizeof(header), write_data, write_len);
    }

    size_t response_len = 0;
    int32_t ret_val = 0;

    int result = perform_socket_rpc(PX4_RPC_SOCKET_IOCTL,
                                    payload,
                                    payload_len,
                                    read_buffer,
                                    read_capacity,
                                    &response_len,
                                    &ret_val);

    if (payload != stack_buf) {
        free(payload);
    }

    if (result < 0) {
        return -1;
    }

    if (read_buffer == NULL && response_len > 0) {
        errno = EMSGSIZE;
        return -1;
    }

    if (response_len > read_capacity) {
        errno = EMSGSIZE;
        return -1;
    }

    if (out_read_len) {
        *out_read_len = response_len;
    }

    errno = 0;
    return ret_val;
}

#endif /* __PX4_FREERTOS */
