/****************************************************************************
 *
 * Copyright (c) 2025 Renesas Electronics Corporation and/or its affiliates
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 ****************************************************************************/

/**
 * @file posix_ioctl_stub.c
 * @brief POSIX ioctl stub implementation for Renesas RZ/V2H
 */

#if defined(__PX4_FREERTOS)

#include <sys/ioctl.h>

#include <errno.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

int rzv_socket_is_remote_fd(int fd);
int rzv_socket_ioctl_remote(int fd,
                            unsigned long request,
                            const void *write_data,
                            size_t write_len,
                            uint64_t scalar_value,
                            void *read_buffer,
                            size_t read_capacity,
                            size_t *out_read_len);
int px4_rzv_remote_handles_fd(int fd);
int px4_rzv_remote_get_fd_value(int fd, int *remote_fd_out);
int rzv_remote_fs_ioctl_rpc(int fd,
                            unsigned long request,
                            const void *write_data,
                            size_t write_len,
                            uint64_t scalar_value,
                            void *read_buffer,
                            size_t read_capacity,
                            size_t *out_read_len);
int px4_ioctl(int fd, int cmd, unsigned long arg);

static uintptr_t extract_argument(unsigned long request, va_list args)
{
    unsigned dir = _IOC_DIR(request);
    size_t size = _IOC_SIZE(request);

    if (dir == 0 && size == 0) {
        return va_arg(args, uintptr_t);
    }

    return va_arg(args, uintptr_t);
}

static int handle_socket_ioctl(int fd,
                               unsigned long request,
                               const void *write_data,
                               size_t write_len,
                               uint64_t scalar_value,
                               void *read_buffer,
                               size_t read_capacity)
{
    size_t read_len = 0;
    int ret = rzv_socket_ioctl_remote(fd,
                                      request,
                                      write_data,
                                      write_len,
                                      scalar_value,
                                      read_buffer,
                                      read_capacity,
                                      &read_len);

    if (ret < 0) {
        return -1;
    }

    if (read_buffer == NULL && read_len > 0) {
        errno = EMSGSIZE;
        return -1;
    }

    if (read_len > read_capacity) {
        errno = EMSGSIZE;
        return -1;
    }

    errno = 0;
    return ret;
}

static int handle_remote_fs_ioctl(int fd,
                                  unsigned long request,
                                  const void *write_data,
                                  size_t write_len,
                                  uint64_t scalar_value,
                                  void *read_buffer,
                                  size_t read_capacity)
{
    int remote_fd = 0;

    if (px4_rzv_remote_get_fd_value(fd, &remote_fd) < 0) {
        return -1;
    }

    size_t read_len = 0;
    int ret = rzv_remote_fs_ioctl_rpc(remote_fd,
                                      request,
                                      write_data,
                                      write_len,
                                      scalar_value,
                                      read_buffer,
                                      read_capacity,
                                      &read_len);

    if (ret < 0) {
        return -1;
    }

    if (read_buffer == NULL && read_len > 0) {
        errno = EMSGSIZE;
        return -1;
    }

    if (read_len > read_capacity) {
        errno = EMSGSIZE;
        return -1;
    }

    errno = 0;
    return ret;
}

int ioctl(int fd, unsigned long request, ...)
{
    va_list ap;
    va_start(ap, request);
    uintptr_t arg_raw = extract_argument(request, ap);
    va_end(ap);

    size_t arg_size = _IOC_SIZE(request);
    unsigned dir = _IOC_DIR(request);

    const void *write_data = NULL;
    void *read_data = NULL;
    size_t write_len = 0;
    size_t read_len = 0;

    if (dir & _IOC_WRITE) {
        write_data = (const void *)arg_raw;
        write_len = arg_size;
    }

    if (dir & _IOC_READ) {
        read_data = (void *)arg_raw;
        read_len = arg_size;
    }

    if ((write_len > 0 && write_data == NULL) || (read_len > 0 && read_data == NULL)) {
        errno = EINVAL;
        return -1;
    }

    if (rzv_socket_is_remote_fd(fd)) {
        return handle_socket_ioctl(fd,
                                   request,
                                   write_data,
                                   write_len,
                                   (uint64_t)arg_raw,
                                   read_data,
                                   read_len);
    }

    if (px4_rzv_remote_handles_fd(fd)) {
        return handle_remote_fs_ioctl(fd,
                                      request,
                                      write_data,
                                      write_len,
                                      (uint64_t)arg_raw,
                                      read_data,
                                      read_len);
    }

    return px4_ioctl(fd, (int)request, arg_raw);
}

#endif /* __PX4_FREERTOS */
