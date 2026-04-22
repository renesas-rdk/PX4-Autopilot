/****************************************************************************
 *
 * Copyright (c) 2025 Renesas Electronics Corporation and/or its affiliates
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 ****************************************************************************/

/**
 * @file remote_storage.cpp
 * @brief Remote filesystem access for Renesas RZ/V2H
 */

#if defined(__PX4_FREERTOS)

#include <cstring>
#include <errno.h>
#include <fcntl.h>
#include <pthread.h>
#include <stdarg.h>
#include <stdint.h>
#include <sys/types.h>
#include <unistd.h>

#include <px4_platform_common/posix.h>
#include <px4_platform_common/log.h>
#include "embedded_metadata.h"

#include "openamp_rpc_client.h"

static constexpr int kRemoteFdBase = 4000;
static constexpr int kRemoteFdSlots = 64;

struct RemoteEntry {
	int remote_fd{-1};
	off_t stub_offset{0};
};

static constexpr int kStubRemoteFd = -3;

static RemoteEntry g_remote_fds[kRemoteFdSlots];
static pthread_mutex_t g_remote_mutex = PTHREAD_MUTEX_INITIALIZER;
static bool g_remote_unavailable_warned{false};

static inline bool remote_transport_available()
{
	return px4_openamp_rpc_endpoint_ready();
}

static void log_remote_unavailable_once(const char *op)
{
	if (!g_remote_unavailable_warned) {
		PX4_WARN("Remote FS unavailable (%s), skipping operations", op);
		g_remote_unavailable_warned = true;
	}
}

static bool entry_is_stub(const RemoteEntry &entry)
{
	return entry.remote_fd == kStubRemoteFd;
}

static int allocate_slot()
{
	for (int i = 0; i < kRemoteFdSlots; ++i) {
		if (g_remote_fds[i].remote_fd < 0) {
			g_remote_fds[i].remote_fd = -2;
			return i;
		}
	}

	return -1;
}

static int slot_from_fd(int fd)
{
	if (fd < kRemoteFdBase) {
		return -1;
	}

	const int slot = fd - kRemoteFdBase;

	if (slot < 0 || slot >= kRemoteFdSlots) {
		return -1;
	}

	return slot;
}

static void release_slot(int slot)
{
	if (slot >= 0 && slot < kRemoteFdSlots) {
		g_remote_fds[slot].remote_fd = -1;
		g_remote_fds[slot].stub_offset = 0;
	}
}

extern "C" {

int rzv_socket_is_remote_fd(int fd);
int rzv_socket_close_remote(int fd);

__attribute__((weak)) int rzv_remote_fs_open_impl(const char *path, int flags, mode_t mode)
{
	(void)path;
	(void)flags;
	(void)mode;
	return -ENOSYS;
}

__attribute__((weak)) ssize_t rzv_remote_fs_read_impl(int fd, void *buffer, size_t buflen)
{
	(void)fd;
	(void)buffer;
	(void)buflen;
	return -ENOSYS;
}

__attribute__((weak)) ssize_t rzv_remote_fs_write_impl(int fd, const void *buffer, size_t buflen)
{
	(void)fd;
	(void)buffer;
	(void)buflen;
	return -ENOSYS;
}

__attribute__((weak)) int rzv_remote_fs_close_impl(int fd)
{
	(void)fd;
	return -ENOSYS;
}

__attribute__((weak)) off_t rzv_remote_fs_lseek_impl(int fd, off_t offset, int whence)
{
	(void)fd;
	(void)offset;
	(void)whence;
	return -ENOSYS;
}

__attribute__((weak)) int rzv_remote_fs_fsync_impl(int fd)
{
	(void)fd;
	return 0;
}

__attribute__((weak)) int rzv_remote_fs_unlink_impl(const char *path)
{
	(void)path;
	return -ENOSYS;
}

__attribute__((weak)) int rzv_remote_fs_access_impl(const char *path, int mode)
{
	(void)path;
	(void)mode;
	return -ENOSYS;
}

__attribute__((weak)) int rzv_remote_fs_mkdir_impl(const char *path, mode_t mode)
{
	(void)path;
	(void)mode;
	return -ENOSYS;
}

__attribute__((weak)) int rzv_remote_fs_rmdir_impl(const char *path)
{
	(void)path;
	return -ENOSYS;
}

__attribute__((weak)) int rzv_remote_fs_truncate_impl(const char *path, off_t length)
{
	(void)path;
	(void)length;
	return -ENOSYS;
}

__attribute__((weak)) int rzv_remote_system_reboot(void)
{
	return -ENOSYS;
}

__attribute__((weak)) int rzv_remote_fs_session_open(uint64_t timestamp_us, char *path_out, size_t path_size)
{
	(void)timestamp_us;
	if (path_out && path_size > 0) { path_out[0] = '\0'; }
	return -ENOSYS;
}

__attribute__((weak)) int rzv_remote_fs_session_close(int32_t fd)
{
	(void)fd;
	return -ENOSYS;
}

// Helper: Check if path is a local UART device
static inline bool is_local_uart_device(const char *path)
{
	if (!path) {
		return false;
	}

	// Match /dev/ttyS* patterns
	if (strncmp(path, "/dev/ttyS", 9) == 0) {
		return true;
	}

	return false;
}

bool px4_rzv_remote_handles_fd(int fd)
{
	// Embedded metadata fds (5000–5015) must be routed through this layer.
	// rzv_embedded_metadata_read/close/lseek already handle them; we just need
	// this function to return true so cdev_platform.cpp doesn't fall through
	// to CDev and return -EINVAL for read/lseek/close on embedded fds.
	if (rzv_embedded_metadata_is_fd(fd)) {
		return true;
	}

	pthread_mutex_lock(&g_remote_mutex);
	const int slot = slot_from_fd(fd);
	bool handled = (slot >= 0) && (g_remote_fds[slot].remote_fd >= 0 || entry_is_stub(g_remote_fds[slot]));
	pthread_mutex_unlock(&g_remote_mutex);
	return handled;
}

int px4_rzv_remote_get_fd_value(int fd, int *remote_fd_out)
{
	if (!remote_fd_out) {
		errno = EINVAL;
		return -1;
	}

	pthread_mutex_lock(&g_remote_mutex);
	const int slot = slot_from_fd(fd);

	if (slot < 0 || g_remote_fds[slot].remote_fd < 0) {
		pthread_mutex_unlock(&g_remote_mutex);
		errno = EBADF;
		return -1;
	}

	if (entry_is_stub(g_remote_fds[slot])) {
		pthread_mutex_unlock(&g_remote_mutex);
		errno = ENOSYS;
		return -1;
	}

	*remote_fd_out = g_remote_fds[slot].remote_fd;
	pthread_mutex_unlock(&g_remote_mutex);
	return 0;
}

int px4_rzv_remote_open(const char *path, int flags, mode_t mode)
{
	if (!path) {
		errno = EINVAL;
		return -1;
	}

	// --- Tier 0: Embedded metadata served from ROM (no CA55 needed) ---
	// Intercept requests for QGC metadata files so they work before CA55 boots.
	// CA55-based FTP (normal path) will take over once RPC is ready, but the
	// embedded files ensure QGC can show the Actuators tab even during early boot.
	if (rzv_embedded_metadata_is_handled(path)) {
		int emb_fd = rzv_embedded_metadata_open(path, flags);
		if (emb_fd >= 0) {
			return emb_fd;
		}
		// Fall through on error (e.g. O_WRONLY attempt) to normal path
	}

	// Don't route UART devices through remote FS
	if (is_local_uart_device(path)) {
		errno = ENOENT;  // Let caller fall through to normal px4_open
		return -1;
	}

	// Don't route in-process uORB CDev paths (/obj/, /dev/) to CA55.
	// These paths must create local in-memory DeviceNodes via _device_master->advertise()
	// for uORB subscription callbacks (registerCallback) to work.
	// Routing /obj/ to CA55 causes px4_open to return a valid fd, which makes
	// uORBManager::node_open() skip advertise(), leaving no in-memory DeviceNode,
	// so Subscription::subscribe() (getDeviceNode) always returns null → registerCallback fails.
	if (strncmp(path, "/obj/", 5) == 0 || strncmp(path, "/dev/", 5) == 0) {
		errno = ENOENT;  // Force local CDev path
		return -1;
	}

	pthread_mutex_lock(&g_remote_mutex);
	int slot = allocate_slot();

	if (slot < 0) {
		pthread_mutex_unlock(&g_remote_mutex);
		errno = EMFILE;
		return -1;
	}

	if (!remote_transport_available()) {
		// Return ENOENT (not a stub fd) so callers handle failure correctly:
		//   - rzv_px4_run_script("/etc/init.d/rc.board_defaults"): fd<0 → embedded fallback
		//   - param_load_default: fails silently, factory defaults remain
		// Stub fds cause EBADF in px4_read/lseek because cdev_platform.cpp
		// does not route fd>=kRemoteFdBase through px4_rzv_remote_read.
		release_slot(slot);
		pthread_mutex_unlock(&g_remote_mutex);
		log_remote_unavailable_once("open");
		errno = ENOENT;
		return -1;
	}

	int remote_fd = rzv_remote_fs_open_impl(path, flags, mode);

	if (remote_fd < 0) {
		release_slot(slot);
		pthread_mutex_unlock(&g_remote_mutex);
		errno = -remote_fd;
		return -1;
	}

	g_remote_fds[slot].remote_fd = remote_fd;
	pthread_mutex_unlock(&g_remote_mutex);
	return kRemoteFdBase + slot;
}

int px4_rzv_remote_close(int fd)
{
	// Route embedded metadata fd
	if (rzv_embedded_metadata_is_fd(fd)) {
		return rzv_embedded_metadata_close(fd);
	}

	pthread_mutex_lock(&g_remote_mutex);
	int slot = slot_from_fd(fd);

	if (slot < 0 || g_remote_fds[slot].remote_fd < 0) {
		pthread_mutex_unlock(&g_remote_mutex);
		errno = EBADF;
		return -1;
	}

	int remote_fd = g_remote_fds[slot].remote_fd;
	const bool is_stub = entry_is_stub(g_remote_fds[slot]);
	release_slot(slot);
	pthread_mutex_unlock(&g_remote_mutex);

	if (is_stub) {
		return 0;
	}

	int ret = rzv_remote_fs_close_impl(remote_fd);

	if (ret < 0) {
		errno = -ret;
		return -1;
	}

	return 0;
}

ssize_t px4_rzv_remote_read(int fd, void *buffer, size_t buflen)
{
	// Route embedded metadata fd
	if (rzv_embedded_metadata_is_fd(fd)) {
		return rzv_embedded_metadata_read(fd, buffer, buflen);
	}

	pthread_mutex_lock(&g_remote_mutex);
	const int slot = slot_from_fd(fd);
	const bool is_stub = (slot >= 0) && entry_is_stub(g_remote_fds[slot]);
	const int remote_fd = (slot >= 0) ? g_remote_fds[slot].remote_fd : -1;
	pthread_mutex_unlock(&g_remote_mutex);

	if (slot < 0 || remote_fd < 0) {
		errno = EBADF;
		return -1;
	}

	if (is_stub) {
		return 0;
	}

	ssize_t ret = rzv_remote_fs_read_impl(remote_fd, buffer, buflen);

	if (ret < 0) {
		errno = -ret;
		return -1;
	}

	return ret;
}

ssize_t px4_rzv_remote_write(int fd, const void *buffer, size_t buflen)
{
	pthread_mutex_lock(&g_remote_mutex);
	const int slot = slot_from_fd(fd);
	const bool is_stub = (slot >= 0) && entry_is_stub(g_remote_fds[slot]);
	const int remote_fd = (slot >= 0) ? g_remote_fds[slot].remote_fd : -1;

	if (slot < 0 || remote_fd < 0) {
		pthread_mutex_unlock(&g_remote_mutex);
		errno = EBADF;
		return -1;
	}

	if (is_stub) {
		g_remote_fds[slot].stub_offset += (off_t)buflen;
		pthread_mutex_unlock(&g_remote_mutex);
		return (ssize_t)buflen;
	}

	pthread_mutex_unlock(&g_remote_mutex);

	ssize_t ret = rzv_remote_fs_write_impl(remote_fd, buffer, buflen);

	if (ret < 0) {
		errno = -ret;
		return -1;
	}

	return ret;
}

off_t px4_rzv_remote_lseek(int fd, off_t offset, int whence)
{
	// Route embedded metadata fd
	if (rzv_embedded_metadata_is_fd(fd)) {
		return rzv_embedded_metadata_lseek(fd, offset, whence);
	}

	pthread_mutex_lock(&g_remote_mutex);
	const int slot = slot_from_fd(fd);
	const int remote_fd = (slot >= 0) ? g_remote_fds[slot].remote_fd : -1;
	const bool is_stub = (slot >= 0) && entry_is_stub(g_remote_fds[slot]);

	if (remote_fd < 0) {
		pthread_mutex_unlock(&g_remote_mutex);
		errno = EBADF;
		return -1;
	}

	if (is_stub) {
		off_t new_pos = 0;

		switch (whence) {
		case SEEK_SET:
			new_pos = offset;
			break;

		case SEEK_CUR:
			new_pos = g_remote_fds[slot].stub_offset + offset;
			break;

		case SEEK_END:
			new_pos = offset;
			break;

		default:
			pthread_mutex_unlock(&g_remote_mutex);
			errno = EINVAL;
			return -1;
		}

		g_remote_fds[slot].stub_offset = new_pos;
		pthread_mutex_unlock(&g_remote_mutex);
		return new_pos;
	}

	pthread_mutex_unlock(&g_remote_mutex);
	off_t ret = rzv_remote_fs_lseek_impl(remote_fd, offset, whence);

	if (ret < 0) {
		errno = -ret;
		return -1;
	}

	return ret;
}

int px4_rzv_remote_fsync(int fd)
{
	pthread_mutex_lock(&g_remote_mutex);
	const int slot = slot_from_fd(fd);
	const bool is_stub = (slot >= 0) && entry_is_stub(g_remote_fds[slot]);
	const int remote_fd = (slot >= 0) ? g_remote_fds[slot].remote_fd : -1;
	pthread_mutex_unlock(&g_remote_mutex);

	if (slot < 0 || remote_fd < 0) {
		errno = EBADF;
		return -1;
	}

	if (is_stub) {
		return 0;
	}

	int ret = rzv_remote_fs_fsync_impl(remote_fd);

	if (ret < 0) {
		errno = -ret;
		return -1;
	}

	return 0;
}

int px4_rzv_remote_unlink(const char *path)
{
	if (!remote_transport_available()) {
		log_remote_unavailable_once("unlink");
		return 0;
	}

	int ret = rzv_remote_fs_unlink_impl(path);

	if (ret < 0) {
		errno = -ret;
		return -1;
	}

	return 0;
}

int px4_rzv_remote_access(const char *path, int mode)
{
	// [THRONE] Bypass remote FS for local UART devices
	if (is_local_uart_device(path)) {
		// For /dev/ttyS* devices, assume they exist locally
		// The actual UART driver (UART.cpp) will handle open/close
		return 0;  // F_OK: file exists
	}

	if (!remote_transport_available()) {
		log_remote_unavailable_once("access");
		return 0;
	}

	int ret = rzv_remote_fs_access_impl(path, mode);

	if (ret < 0) {
		errno = -ret;
		return -1;
	}

	return ret;
}

int px4_rzv_remote_mkdir(const char *path, mode_t mode)
{
	if (!path) {
		errno = EINVAL;
		return -1;
	}

	if (is_local_uart_device(path)) {
		errno = ENOTSUP;
		return -1;
	}

	/* Do NOT short-circuit on !remote_transport_available() here.
	 * rzv_remote_fs_mkdir_impl() calls fs_rpc() which already waits up to 30s
	 * for the RPC endpoint to become ready. An early bail-out causes errno=ENOTCONN
	 * in the logger when it tries to create its session directory shortly after boot
	 * before the CA55 OpenAMP link is fully established. */
	int ret = rzv_remote_fs_mkdir_impl(path, mode);

	if (ret < 0) {
		errno = -ret;
		return -1;
	}

	return ret;
}

int px4_rzv_remote_rmdir(const char *path)
{
	if (!path) {
		errno = EINVAL;
		return -1;
	}

	if (is_local_uart_device(path)) {
		errno = ENOTSUP;
		return -1;
	}

	if (!remote_transport_available()) {
		log_remote_unavailable_once("rmdir");
		return 0;
	}

	int ret = rzv_remote_fs_rmdir_impl(path);

	if (ret < 0) {
		errno = -ret;
		return -1;
	}

	return ret;
}

int px4_rzv_remote_truncate(const char *path, off_t length)
{
	if (!path) {
		errno = EINVAL;
		return -1;
	}

	if (is_local_uart_device(path)) {
		errno = ENOTSUP;
		return -1;
	}

	if (!remote_transport_available()) {
		log_remote_unavailable_once("truncate");
		return 0;
	}

	int ret = rzv_remote_fs_truncate_impl(path, length);

	if (ret < 0) {
		errno = -ret;
		return -1;
	}

	return ret;
}

} // extern "C"

extern "C" int open(const char *path, int flags, ...)
{
	mode_t mode = 0;

	if (flags & O_CREAT) {
		va_list p;
		va_start(p, flags);
		mode = va_arg(p, int);
		va_end(p);
	}

	return px4_open(path, flags, mode);
}

extern "C" int close(int fd)
{
	if (rzv_socket_is_remote_fd(fd)) {
		return rzv_socket_close_remote(fd);
	}

	return px4_close(fd);
}

extern "C" ssize_t read(int fd, void *buffer, size_t buflen)
{
	return px4_read(fd, buffer, buflen);
}

extern "C" ssize_t write(int fd, const void *buffer, size_t buflen)
{
	return px4_write(fd, buffer, buflen);
}

extern "C" off_t lseek(int fd, off_t offset, int whence)
{
	return px4_lseek(fd, offset, whence);
}

extern "C" int fsync(int fd)
{
	return px4_fsync(fd);
}

extern "C" int unlink(const char *pathname)
{
	return px4_unlink(pathname);
}

extern "C" int access(const char *pathname, int mode)
{
	return px4_access(pathname, mode);
}

#endif /* __PX4_FREERTOS */
