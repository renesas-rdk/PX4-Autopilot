/****************************************************************************
 *
 * Copyright (c) 2025 Renesas Electronics Corporation and/or its affiliates
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 ****************************************************************************/

/**
 * @file embedded_metadata.h
 * @brief QGC metadata files embedded in CR8 firmware ROM.
 *
 * Serves component_general.json.xz, actuators.json.xz, parameters.json.xz,
 * all_events.json.xz, and rc.board_defaults.cmds directly from flash/SDRAM
 * before CA55/Linux is available, ensuring QGC can show the Actuators tab
 * even if it connects during early boot.
 */
#pragma once

#if defined(__PX4_FREERTOS)


#include <sys/types.h>
#include <unistd.h>
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Check if a path belongs to the embedded metadata virtual filesystem.
 * Returns true for the embedded virtual paths (json.xz metadata + rc.board_defaults.cmds).
 */
bool rzv_embedded_metadata_is_handled(const char *path);

/**
 * Open an embedded metadata file.
 * Returns a handle fd >= RZV_EMBEDDED_FD_BASE, or -1 with errno set.
 */
int rzv_embedded_metadata_open(const char *path, int flags);

/**
 * Read from an embedded metadata handle.
 */
ssize_t rzv_embedded_metadata_read(int fd, void *buf, size_t count);

/**
 * Seek within an embedded metadata handle.
 */
off_t rzv_embedded_metadata_lseek(int fd, off_t offset, int whence);

/**
 * Get total size of the embedded file for a given fd (for fstat-like use).
 * Returns -1 if fd is not an embedded handle.
 */
off_t rzv_embedded_metadata_size(int fd);

/**
 * Get total size of an embedded file by path without allocating an fd slot.
 * Returns -1 if path is not found.  Use this in stat() to avoid wasting a slot.
 */
off_t rzv_embedded_metadata_size_by_path(const char *path);

/**
 * Close an embedded metadata handle.
 * Returns 0 on success, -1 with errno on error.
 */
int rzv_embedded_metadata_close(int fd);

/**
 * Check if fd belongs to the embedded metadata layer.
 */
bool rzv_embedded_metadata_is_fd(int fd);

#ifdef __cplusplus
}

#endif /* __PX4_FREERTOS */

#endif
