/****************************************************************************
 *
 * Copyright (c) 2025 Renesas Electronics Corporation and/or its affiliates
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 ****************************************************************************/

/**
 * @file posix_fs_stubs.c
 * @brief POSIX filesystem stubs for Renesas RZ/V2H
 */
#include <errno.h>
#include <stdbool.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>
#include <stdint.h>
#include <px4_platform_common/posix.h>
#include "services/embedded_metadata.h"

extern int rzv_remote_fs_stat_impl(const char *path, int32_t *out_mode, int64_t *out_size, int64_t *out_mtime);

int px4_rzv_remote_mkdir(const char *path, mode_t mode);
int px4_rzv_remote_rmdir(const char *path);
int px4_rzv_remote_truncate(const char *path, off_t length);

static bool is_remote_fs_path(const char *path)
{
	if (path == NULL) {
		return false;
	}

	return (strncmp(path, "/fs/", 4) == 0);
}

int truncate(const char *path, off_t length)
{
	if (is_remote_fs_path(path)) {
		if (px4_rzv_remote_truncate(path, length) == 0) {
			return 0;
		}

		return -1;
	}

	errno = ENOSYS;
	return -1;
}

#if defined(__PX4_FREERTOS)
int px4_fsync(int fd);
#endif

__attribute__((weak)) int fsync(int fd)
{
#if defined(__PX4_FREERTOS)
	return px4_fsync(fd);
#else
	(void)fd;
	return 0;
#endif
}

int rmdir(const char *path)
{
	if (is_remote_fs_path(path)) {
		if (px4_rzv_remote_rmdir(path) == 0) {
			return 0;
		}

		return -1;
	}

	errno = ENOSYS;
	return -1;
}

int mkdir(const char *path, mode_t mode)
{
	if (is_remote_fs_path(path)) {
		if (px4_rzv_remote_mkdir(path, mode) == 0) {
			return 0;
		}

		return -1;
	}

	errno = ENOSYS;
	return -1;
}

int stat(const char *path, struct stat *buf)
{
	if (!path || !buf) {
		errno = EINVAL;
		return -1;
	}

	// Handle local pseudo-files
	if (strncmp(path, "/dev/", 5) == 0 || strncmp(path, "/obj/", 5) == 0) {
		memset(buf, 0, sizeof(struct stat));
		buf->st_mode = S_IFCHR | 0666;
		return 0;
	}

	// Fast-path: embedded metadata files don't need an RPC round-trip.
	// Query size directly without allocating an fd slot.
	if (rzv_embedded_metadata_is_handled(path)) {
		off_t sz = rzv_embedded_metadata_size_by_path(path);
		if (sz >= 0) {
			memset(buf, 0, sizeof(struct stat));
			buf->st_size = sz;
			buf->st_mode = S_IFREG | 0444;
			return 0;
		}
	}

	// Query CA55 for full stat (mode, size, mtime) via RPC.
	// mtime is critical for QGC Log Download to display correct date and track
	// which files have been downloaded (time_utc=0 causes "Date Unknown" + loses
	// "Downloaded" state on refresh).
	int32_t rpc_mode  = 0;
	int64_t rpc_size  = 0;
	int64_t rpc_mtime = 0;

	if (rzv_remote_fs_stat_impl(path, &rpc_mode, &rpc_size, &rpc_mtime) == 0) {
		memset(buf, 0, sizeof(struct stat));
		buf->st_mode  = (mode_t)rpc_mode;
		buf->st_size  = (off_t)rpc_size;
		buf->st_mtime = (time_t)rpc_mtime;
		return 0;
	}

	// Fallback: if STAT RPC unavailable, try open+lseek for size (no mtime)
	int fd = px4_open(path, O_RDONLY, 0666);
	if (fd < 0) {
		// If open fails, use px4_access to see if it exists (might be a directory)
		if (px4_access(path, F_OK) == 0) {
			memset(buf, 0, sizeof(struct stat));
			buf->st_mode = S_IFDIR | 0777;
			buf->st_size = 0;
			return 0;
		}
		errno = ENOENT;
		return -1;
	}

	off_t size = px4_lseek(fd, 0, SEEK_END);
	px4_close(fd);

	memset(buf, 0, sizeof(struct stat));
	buf->st_size = (size < 0) ? 0 : size;
	buf->st_mode = S_IFREG | 0666;

	return 0;
}

