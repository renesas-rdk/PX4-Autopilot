/****************************************************************************
 *
 * Copyright (c) 2015 Mark Charlebois. All rights reserved.
 * Copyright (c) 2018 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include "cdev_platform.hpp"

#include "../CDev.hpp"

#include <px4_platform_common/log.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/time.h>
#if defined(__PX4_FREERTOS)
#include <cstring>
#include <stdint.h>

static constexpr const char k_px4_freertos_thread_name[] = "px4-task";

#include <errno.h>
#include <fcntl.h>
#include <stdarg.h>
#endif
#include <stdlib.h>
#if defined(__PX4_FREERTOS)
#include <stdio.h>

#include <sys/types.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <termios.h>
#include <unistd.h>

#ifndef F_OK
#define F_OK 0
#endif
#ifndef R_OK
#define R_OK 4
#endif
#ifndef W_OK
#define W_OK 2
#endif
#ifndef X_OK
#define X_OK 1
#endif

extern "C" {
int px4_rzv_remote_open(const char *path, int flags, mode_t mode);
int px4_rzv_remote_close(int fd);
ssize_t px4_rzv_remote_read(int fd, void *buffer, size_t buflen);
ssize_t px4_rzv_remote_write(int fd, const void *buffer, size_t buflen);
off_t px4_rzv_remote_lseek(int fd, off_t offset, int whence);
int px4_rzv_remote_fsync(int fd);
int px4_rzv_remote_unlink(const char *path);
int px4_rzv_remote_access(const char *path, int mode);
bool px4_rzv_remote_handles_fd(int fd);
}
#endif /* __PX4_FREERTOS */

const cdev::px4_file_operations_t cdev::CDev::fops = {};

pthread_mutex_t devmutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t filemutex = PTHREAD_MUTEX_INITIALIZER;

struct px4_dev_t {
	char *name{nullptr};
	cdev::CDev *cdev{nullptr};

	px4_dev_t(const char *n, cdev::CDev *c) : cdev(c)
	{
		name = strdup(n);
	}

	~px4_dev_t()
	{
		free(name);
	}
private:
	px4_dev_t() = default;
};

#define PX4_MAX_FD 512
static px4_dev_t *devmap[PX4_MAX_FD] {};
static cdev::file_t filemap[PX4_MAX_FD] {};
#if defined(__PX4_FREERTOS)

#if defined(CONFIG_UART)
#include "rzv_fsp/device_registry.h"
#include "rzv_fsp/uart_fsp_backend.h"
#include "rzv_fsp/uart_channel_map.h"
extern "C" {
int uart_open_channel(uint8_t channel, uint32_t baudrate, bool mode_8N1);
int uart_write_channel(uint8_t channel, const uint8_t *buffer, unsigned length);
int uart_read_channel(uint8_t channel, uint8_t *buffer, unsigned length);
int uart_close_channel(uint8_t channel);
int uart_rc_buffer_read(uint8_t channel, uint8_t *buffer, int max_len);
int uart_rc_buffer_available(uint8_t channel);
void uart_rc_buffer_flush(uint8_t channel);
void uart_set_rc_serial_mode(bool use_8n1);
int uart_reopen_rc_channel(uint32_t baudrate);
}

using RzvUartConfig = uart_channel_info_t;

static cdev::CDev *g_rzv_uart_instances[4] = {nullptr, nullptr, nullptr, nullptr};

class RzvUartDevice : public cdev::CDev
{
public:
	RzvUartDevice(const RzvUartConfig &cfg)
		: cdev::CDev(cfg.device), _cfg(cfg)
	{
		reset_termios_defaults();
	}

	~RzvUartDevice() override = default;

	int init() override
	{
		return cdev::CDev::init();
	}

	int open(cdev::file_t *filep) override
	{
		if (_open_handles == 0) {
			if (uart_open_channel(_cfg.channel, _cfg.default_baud, _cfg.mode_8n1) != 0) {
				return -EIO;
			}
		}

		int ret = cdev::CDev::open(filep);

		if (ret < 0) {
			if (_open_handles == 0) {
				uart_close_channel(_cfg.channel);
			}
			return ret;
		}

		++_open_handles;
		_backend = rzv_get_uart_backend(_cfg.channel);
		refresh_termios_from_backend();
		return PX4_OK;
	}

	int close(cdev::file_t *filep) override
	{
		int ret = cdev::CDev::close(filep);

		if (ret == 0) {
			if (_open_handles > 0) {
				--_open_handles;
			}

			if (_open_handles == 0) {
				uart_close_channel(_cfg.channel);
				_backend = nullptr;
				reset_termios_defaults();
			}
		}

		return ret;
	}

	ssize_t read(cdev::file_t *, char *buffer, size_t buflen) override
	{
		if (!buffer || buflen == 0) {
			return -EINVAL;
		}

		int ret = uart_read_channel(_cfg.channel, reinterpret_cast<uint8_t *>(buffer), (unsigned)buflen);
		return (ret >= 0) ? ret : -EIO;
	}

	ssize_t write(cdev::file_t *, const char *buffer, size_t buflen) override
	{
		if (!buffer || buflen == 0) {
			return -EINVAL;
		}

		int ret = uart_write_channel(_cfg.channel, reinterpret_cast<const uint8_t *>(buffer), (unsigned)buflen);
		return (ret >= 0) ? ret : -EIO;
	}

	int ioctl(cdev::file_t *filep, int cmd, unsigned long arg) override
	{
		switch (cmd) {
		case FIONREAD:
#if FIONREAD != 0x541B
		case 0x541B: /* Linux/TIOCINQ */
#endif
#if FIONREAD != 0x6678
		case 0x6678: /* FreeRTOS POSIX stub */
#endif
			if (arg == 0UL) {
				return -EINVAL;
			}

			*reinterpret_cast<int *>(arg) = uart_rc_buffer_available(_cfg.channel);
			return PX4_OK;

		case FIONSPACE:
			if (arg == 0UL) {
				return -EINVAL;
			}

			*reinterpret_cast<int *>(arg) = 512;
			return PX4_OK;

#ifdef TIOCINQ
		case TIOCINQ:
			if (arg == 0UL) {
				return -EINVAL;
			}

			*reinterpret_cast<int *>(arg) = uart_rc_buffer_available(_cfg.channel);
			return PX4_OK;
#endif

#ifdef TIOCOUTQ
		case TIOCOUTQ:
			if (arg == 0UL) {
				return -EINVAL;
			}

			*reinterpret_cast<int *>(arg) = 512;
			return PX4_OK;
#endif

#ifdef TCGETS
		case TCGETS:
#endif
#ifdef TCGETS2
		case TCGETS2:
#endif
#ifdef TCGETA
		case TCGETA:
#endif
		{
			if (arg == 0UL) {
				return -EINVAL;
			}

			auto *t = reinterpret_cast<termios *>(arg);
			termios cached = _state.cfg;
#ifdef CBAUD
			cached.c_cflag &= ~CBAUD;
			cached.c_cflag |= static_cast<tcflag_t>(encode_speed(_state.baud));
#endif
			std::memcpy(t, &cached, sizeof(termios));
			return PX4_OK;
		}

#ifdef TCSETS
		case TCSETS:
#endif
#ifdef TCSETSW
		case TCSETSW:
#endif
#ifdef TCSETSF
		case TCSETSF:
#endif
#ifdef TCSETS2
		case TCSETS2:
#endif
#ifdef TCSETSW2
		case TCSETSW2:
#endif
#ifdef TCSETSF2
		case TCSETSF2:
#endif
		{
			if (arg == 0UL) {
				return -EINVAL;
			}

			auto *cfg = reinterpret_cast<termios *>(arg);
			return apply_termios(*cfg);
		}

#ifdef TIOCSINVERT
		case TIOCSINVERT:
			_state.inverted = (arg != 0UL);
			return PX4_OK;
#endif

#ifdef TIOCSSINGLEWIRE
		case TIOCSSINGLEWIRE:
			_state.single_wire = (arg != 0UL);
			return PX4_OK;
#endif

#ifdef TIOCSSWAP
		case TIOCSSWAP:
			_state.swap_lines = (arg != 0UL);
			return PX4_OK;
#endif

#ifdef TCFLSH
		case TCFLSH:
			if (arg == TCIFLUSH || arg == TCIOFLUSH) {
				if (_cfg.channel == 0) {
					uart_rc_buffer_flush(_cfg.channel);
				}
				return PX4_OK;
			}

			return PX4_OK;
#endif

		default:
			return cdev::CDev::ioctl(filep, cmd, arg);
		}
	}

	void flush_rx() { uart_rc_buffer_flush(_cfg.channel); }

private:
	static uint32_t decode_speed(speed_t speed)
	{
#ifdef B0
		if (speed == B0) { return 0; }
#endif
#ifdef B50
		if (speed == B50) { return 50; }
#endif
#ifdef B75
		if (speed == B75) { return 75; }
#endif
#ifdef B110
		if (speed == B110) { return 110; }
#endif
#ifdef B134
		if (speed == B134) { return 134; }
#endif
#ifdef B150
		if (speed == B150) { return 150; }
#endif
#ifdef B200
		if (speed == B200) { return 200; }
#endif
#ifdef B300
		if (speed == B300) { return 300; }
#endif
#ifdef B600
		if (speed == B600) { return 600; }
#endif
#ifdef B1200
		if (speed == B1200) { return 1200; }
#endif
#ifdef B1800
		if (speed == B1800) { return 1800; }
#endif
#ifdef B2400
		if (speed == B2400) { return 2400; }
#endif
#ifdef B4800
		if (speed == B4800) { return 4800; }
#endif
#ifdef B9600
		if (speed == B9600) { return 9600; }
#endif
#ifdef B19200
		if (speed == B19200) { return 19200; }
#endif
#ifdef B38400
		if (speed == B38400) { return 38400; }
#endif
#ifdef B57600
		if (speed == B57600) { return 57600; }
#endif
#ifdef B115200
		if (speed == B115200) { return 115200; }
#endif
#ifdef B230400
		if (speed == B230400) { return 230400; }
#endif
#ifdef B460800
		if (speed == B460800) { return 460800; }
#endif
#ifdef B500000
		if (speed == B500000) { return 500000; }
#endif
#ifdef B576000
		if (speed == B576000) { return 576000; }
#endif
#ifdef B921600
		if (speed == B921600) { return 921600; }
#endif
#ifdef B1000000
		if (speed == B1000000) { return 1000000; }
#endif
#ifdef B1500000
		if (speed == B1500000) { return 1500000; }
#endif
		return static_cast<uint32_t>(speed);
	}

	static speed_t encode_speed(uint32_t baud)
	{
		switch (baud) {
#ifdef B0
		case 0: return B0;
#endif
#ifdef B50
		case 50: return B50;
#endif
#ifdef B75
		case 75: return B75;
#endif
#ifdef B110
		case 110: return B110;
#endif
#ifdef B134
		case 134: return B134;
#endif
#ifdef B150
		case 150: return B150;
#endif
#ifdef B200
		case 200: return B200;
#endif
#ifdef B300
		case 300: return B300;
#endif
#ifdef B600
		case 600: return B600;
#endif
#ifdef B1200
		case 1200: return B1200;
#endif
#ifdef B1800
		case 1800: return B1800;
#endif
#ifdef B2400
		case 2400: return B2400;
#endif
#ifdef B4800
		case 4800: return B4800;
#endif
#ifdef B9600
		case 9600: return B9600;
#endif
#ifdef B19200
		case 19200: return B19200;
#endif
#ifdef B38400
		case 38400: return B38400;
#endif
#ifdef B57600
		case 57600: return B57600;
#endif
#ifdef B115200
		case 115200: return B115200;
#endif
#ifdef B230400
		case 230400: return B230400;
#endif
#ifdef B460800
		case 460800: return B460800;
#endif
#ifdef B500000
		case 500000: return B500000;
#endif
#ifdef B576000
		case 576000: return B576000;
#endif
#ifdef B921600
		case 921600: return B921600;
#endif
#ifdef B1000000
		case 1000000: return B1000000;
#endif
#ifdef B1500000
		case 1500000: return B1500000;
#endif
		default:
			return static_cast<speed_t>(baud);
		}
	}

	static bool is_8n1(const termios &cfg)
	{
		const tcflag_t bits = cfg.c_cflag & CSIZE;
		const bool eight_bits = (bits == CS8) || (bits == 0);
		const bool no_parity = (cfg.c_cflag & PARENB) == 0;
		const bool one_stop = (cfg.c_cflag & CSTOPB) == 0;
		return eight_bits && no_parity && one_stop;
	}

	void reset_termios_defaults()
	{
		_state.baud = (_cfg.default_baud != 0U) ? _cfg.default_baud : 115200U;
		termios default_cfg{};
		default_cfg.c_cflag = CLOCAL | CREAD | CS8;
		if (!_cfg.mode_8n1) {
			default_cfg.c_cflag |= PARENB | CSTOPB;
		}
#ifdef CBAUD
		default_cfg.c_cflag &= ~CBAUD;
		default_cfg.c_cflag |= static_cast<tcflag_t>(encode_speed(_state.baud));
#endif
		_state.cfg = default_cfg;
		_state.single_wire = false;
		_state.inverted = false;
		_state.swap_lines = false;
	}

	void refresh_termios_from_backend()
	{
		if (_backend && _backend->opened && _backend->baudrate != 0U) {
			_state.baud = _backend->baudrate;
		}

	#ifdef CBAUD
		_state.cfg.c_cflag &= ~CBAUD;
		_state.cfg.c_cflag |= static_cast<tcflag_t>(encode_speed(_state.baud));
	#endif
	}

	int apply_termios(const termios &cfg)
	{
		uint32_t desired_baud = _state.baud;

#ifdef CBAUD
		{
			const tcflag_t baud_bits = cfg.c_cflag & CBAUD;
			uint32_t decoded = decode_speed(static_cast<speed_t>(baud_bits));

			if ((decoded != 0U) || (baud_bits == 0)) {
				desired_baud = decoded;
			}
		}
#endif

		if (desired_baud == 0U) {
			desired_baud = _state.baud;
		}

		if (_backend && desired_baud != 0U && desired_baud != _state.baud) {
			if (rzv_uart_backend_configure(_backend, desired_baud) != 0) {
				return -EIO;
			}
		}

		if (_cfg.channel == 0) {
			uart_set_rc_serial_mode(is_8n1(cfg));

			if (desired_baud != 0U && desired_baud != _state.baud) {
				(void)uart_reopen_rc_channel(desired_baud);
			}
		}

		_state.baud = desired_baud;
		_state.cfg = cfg;
#ifdef CBAUD
		_state.cfg.c_cflag &= ~CBAUD;
		_state.cfg.c_cflag |= static_cast<tcflag_t>(encode_speed(_state.baud));
#endif
		return PX4_OK;
	}

	const RzvUartConfig _cfg;
	rzv_uart_backend_t *_backend{nullptr};

	struct {
		termios cfg{};
		uint32_t baud{0};
		bool single_wire{false};
		bool inverted{false};
		bool swap_lines{false};
	} _state{};

	int _open_handles{0};
};

static const RzvUartConfig *rzv_find_uart_config(const char *path)
{
	return uart_channel_get_info_by_device(path);
}

static const RzvUartConfig *rzv_find_uart_config_by_suffix(const char *path)
{
	if (!path) {
		return nullptr;
	}

	if (strncmp(path, "/dev/ttyS", 9) != 0) {
		return nullptr;
	}

	const char suffix = path[9];

	if (suffix < '0' || suffix > '9') {
		return nullptr;
	}

	if (path[10] != '\0') {
		return nullptr;
	}

	for (size_t i = 0; i < g_uart_channel_table_size; ++i) {
		const auto &cfg = g_uart_channel_table[i];

		if (cfg.device[9] == suffix && cfg.device[10] == '\0') {
			return &cfg;
		}
	}

	return nullptr;
}

static cdev::CDev *getDev(const char *path);

static RzvUartDevice *rzv_get_or_create_uart(const RzvUartConfig &cfg)
{
	if (cfg.channel >= (sizeof(g_rzv_uart_instances) / sizeof(g_rzv_uart_instances[0]))) {
		return nullptr;
	}

	auto *instance = static_cast<RzvUartDevice *>(g_rzv_uart_instances[cfg.channel]);

	if (instance == nullptr) {
#if defined(__PX4_FREERTOS) && defined(CONFIG_UART)
		if (getDev(cfg.device) != nullptr) {
			/* Device already provided by another backend (typically the FSP layer). */
			return nullptr;
		}
#endif

		RzvUartDevice *new_uart = new RzvUartDevice(cfg);

		if (!new_uart) {
			PX4_ERR("Failed to allocate UART device for %s", cfg.device);
			return nullptr;
		}

		const int init_ret = new_uart->init();

		if (init_ret != PX4_OK) {
			PX4_ERR("UART %s init failed (%d)", cfg.device, init_ret);
			delete new_uart;
			return nullptr;
		}

		instance = new_uart;
		instance->flush_rx();
		g_rzv_uart_instances[cfg.channel] = instance;
		PX4_INFO("Registered UART device %s (channel %u, baud %u)", cfg.device,
			 (unsigned)cfg.channel, (unsigned)cfg.default_baud);
	}

	return instance;
}

static pthread_mutex_t g_rzv_uart_mutex = PTHREAD_MUTEX_INITIALIZER;
static bool g_rzv_uart_registered = false;

static void rzv_register_default_uart_devices(void)
{
	pthread_mutex_lock(&g_rzv_uart_mutex);

	if (!g_rzv_uart_registered) {
		for (size_t i = 0; i < g_uart_channel_table_size; ++i) {
			const auto &cfg = g_uart_channel_table[i];

			(void)rzv_get_or_create_uart(cfg);
		}

		g_rzv_uart_registered = true;
	}

	pthread_mutex_unlock(&g_rzv_uart_mutex);
}

extern "C" __EXPORT void px4_rzv_register_default_uart_devices_once(void)
{
	rzv_register_default_uart_devices();
}
#else
extern "C" __EXPORT void px4_rzv_register_default_uart_devices_once(void)
{
}
#endif
#endif /* __PX4_FREERTOS */

class VFile : public cdev::CDev
{
public:
	VFile(const char *fname, mode_t mode) : cdev::CDev(fname) {}
	~VFile() override = default;

	ssize_t write(cdev::file_t *handlep, const char *buffer, size_t buflen) override
	{
		// ignore what was written, but let pollers know something was written
		poll_notify(POLLIN);
		return buflen;
	}
};

static cdev::CDev *getDev(const char *path)
{
	pthread_mutex_lock(&devmutex);

	for (const auto &dev : devmap) {
		if (dev && (strcmp(dev->name, path) == 0)) {
			pthread_mutex_unlock(&devmutex);
			return dev->cdev;
		}
	}

	pthread_mutex_unlock(&devmutex);

	return nullptr;
}

static cdev::CDev *getFile(int fd)
{
	pthread_mutex_lock(&filemutex);
	cdev::CDev *dev = nullptr;

	if (fd < PX4_MAX_FD && fd >= 0) {
		dev = filemap[fd].cdev;
	}

	pthread_mutex_unlock(&filemutex);
	return dev;
}

extern "C" {

	int register_driver(const char *name, const cdev::px4_file_operations_t *fops, cdev::mode_t mode, void *data)
	{
		PX4_DEBUG("CDev::register_driver %s", name);
		int ret = -ENOSPC;

		if (name == nullptr || data == nullptr) {
			return -EINVAL;
		}

		pthread_mutex_lock(&devmutex);

		// Make sure the device does not already exist
		for (const auto &dev : devmap) {
			if (dev && (strcmp(dev->name, name) == 0)) {
				pthread_mutex_unlock(&devmutex);
				return -EEXIST;
			}
		}

		for (auto &dev : devmap) {
			if (dev == nullptr) {
				dev = new px4_dev_t(name, (cdev::CDev *)data);
				PX4_DEBUG("Registered DEV %s", name);
				ret = PX4_OK;
				break;
			}
		}

		pthread_mutex_unlock(&devmutex);

		if (ret != PX4_OK) {
			PX4_ERR("No free devmap entries - increase devmap size");
		}

		return ret;
	}

	int unregister_driver(const char *name)
	{
		PX4_DEBUG("CDev::unregister_driver %s", name);
		int ret = -EINVAL;

		if (name == nullptr) {
			return -EINVAL;
		}

		pthread_mutex_lock(&devmutex);

		for (auto &dev : devmap) {
			if (dev && (strcmp(name, dev->name) == 0)) {
				delete dev;
				dev = nullptr;
				PX4_DEBUG("Unregistered DEV %s", name);
				ret = PX4_OK;
				break;
			}
		}

		pthread_mutex_unlock(&devmutex);

		return ret;
	}

	int px4_open(const char *path, int flags, ...)
	{
		PX4_DEBUG("px4_open");
		cdev::CDev *dev = getDev(path);
		int ret = 0;
#if defined(__PX4_FREERTOS)
		int i = -1;
		mode_t mode = 0;
#else
		int i;
		mode_t mode;
#endif /* __PX4_FREERTOS */

#if defined(__PX4_FREERTOS)
		if (flags & O_CREAT) {
#else
		if (!dev && (flags & PX4_F_WRONLY) != 0 &&
		    strncmp(path, "/obj/", 5) != 0 &&
		    strncmp(path, "/dev/", 5) != 0) {
#endif /* __PX4_FREERTOS */
			va_list p;
			va_start(p, flags);
			mode = va_arg(p, int);
			va_end(p);
#if defined(__PX4_FREERTOS)
		}

		if (!dev) {
#if defined(CONFIG_UART)
			if (!dev) {
				const RzvUartConfig *uart_cfg = rzv_find_uart_config(path);

				if (uart_cfg == nullptr) {
					uart_cfg = rzv_find_uart_config_by_suffix(path);
				}

				if (uart_cfg != nullptr) {
					if (rzv_get_or_create_uart(*uart_cfg) != nullptr) {
						dev = getDev(path);
					} else {
						errno = ENODEV;
						return -1;
					}
				}
			}

		if (!dev) {
			int remote_fd = px4_rzv_remote_open(path, flags, mode);

			if (remote_fd >= 0) {
				return remote_fd;
			}

			if (errno != 0) {
				switch (errno) {
				case ENOSYS:
				case ENOENT:
				case ENODEV:
				case EIO:
				case ETIMEDOUT:
					// Allow fallback to local resources (e.g. ROMFS) for these cases.
					errno = 0;
					break;

				default:
					return -1;
				}
			} else {
				// Unexpected remote failure without errno, fall back to local handling.
				errno = 0;
			}
		}
#endif /* CONFIG_UART */

			if (!dev &&
				(flags & PX4_F_WRONLY) != 0 &&
				strncmp(path, "/obj/", 5) != 0 &&
				strncmp(path, "/dev/", 5) != 0) {
				PX4_DEBUG("Creating virtual file %s", path);
				dev = new VFile(path, mode);
				register_driver(path, nullptr, 0666, (void *)dev);
			}
#else
			// Create the file
			PX4_DEBUG("Creating virtual file %s", path);
			dev = new VFile(path, mode);
			register_driver(path, nullptr, 0666, (void *)dev);
#endif /* __PX4_FREERTOS */
		}

		if (dev) {
			pthread_mutex_lock(&filemutex);

			for (i = 0; i < PX4_MAX_FD; ++i) {
				if (filemap[i].cdev == nullptr) {
					filemap[i] = cdev::file_t(flags, dev);
					break;
				}
			}

			pthread_mutex_unlock(&filemutex);

			if (i < PX4_MAX_FD) {
				ret = dev->open(&filemap[i]);

			} else {
				const unsigned NAMELEN = 32;
				char thread_name[NAMELEN] {};

#if defined(__PX4_FREERTOS)
				strncpy(thread_name, k_px4_freertos_thread_name, sizeof(thread_name) - 1);
				thread_name[sizeof(thread_name) - 1] = '\0';

				if (thread_name[0] == 0) {
					(void)snprintf(thread_name, NAMELEN, "cdev");
				}
#else
#ifndef __PX4_QURT
				int nret = pthread_getname_np(pthread_self(), thread_name, NAMELEN);

				if (nret || thread_name[0] == 0) {
					PX4_WARN("failed getting thread name");
				}
#endif
#endif /* __PX4_FREERTOS */

				PX4_WARN("%s: exceeded maximum number of file descriptors, accessing %s", thread_name, path);

				ret = -ENOENT;
			}

		} else {
			ret = -EINVAL;
		}

		if (ret < 0) {
			errno = -ret;
			return -1;
		}

		PX4_DEBUG("px4_open fd = %d", i);
		return i;
	}

	int px4_close(int fd)
	{
#if defined(__PX4_FREERTOS)
		if (px4_rzv_remote_handles_fd(fd)) {
			return px4_rzv_remote_close(fd);
		}
#endif /* __PX4_FREERTOS */
		int ret;

		cdev::CDev *dev = getFile(fd);

		if (dev) {
			pthread_mutex_lock(&filemutex);
			ret = dev->close(&filemap[fd]);

			filemap[fd].cdev = nullptr;

			pthread_mutex_unlock(&filemutex);
			PX4_DEBUG("px4_close fd = %d", fd);

		} else {
			ret = -EINVAL;
		}

		if (ret < 0) {
			ret = PX4_ERROR;
		}

		return ret;
	}

	ssize_t px4_read(int fd, void *buffer, size_t buflen)
	{
#if defined(__PX4_FREERTOS)
		if (px4_rzv_remote_handles_fd(fd)) {
			return px4_rzv_remote_read(fd, buffer, buflen);
		}
#endif /* __PX4_FREERTOS */
		int ret;

		cdev::CDev *dev = getFile(fd);

		if (dev) {
			PX4_DEBUG("px4_read fd = %d", fd);
			ret = dev->read(&filemap[fd], (char *)buffer, buflen);

		} else {
			ret = -EINVAL;
		}

		if (ret < 0) {
			ret = PX4_ERROR;
		}

		return ret;
	}

	ssize_t px4_write(int fd, const void *buffer, size_t buflen)
	{
#if defined(__PX4_FREERTOS)
		if (px4_rzv_remote_handles_fd(fd)) {
			return px4_rzv_remote_write(fd, buffer, buflen);
		}
#endif /* __PX4_FREERTOS */
		int ret;

		cdev::CDev *dev = getFile(fd);

		if (dev) {
			PX4_DEBUG("px4_write fd = %d", fd);
			ret = dev->write(&filemap[fd], (const char *)buffer, buflen);

		} else {
			ret = -EINVAL;
		}

		if (ret < 0) {
			ret = PX4_ERROR;
		}

		return ret;
	}

#if defined(__PX4_FREERTOS)
	off_t px4_lseek(int fd, off_t offset, int whence)
	{
	#if defined(__PX4_FREERTOS)
		if (px4_rzv_remote_handles_fd(fd)) {
			return px4_rzv_remote_lseek(fd, offset, whence);
		}
	#endif
		errno = ENOSYS;
		return -1;
	}

	int px4_fsync(int fd)
	{
	#if defined(__PX4_FREERTOS)
		if (px4_rzv_remote_handles_fd(fd)) {
			return px4_rzv_remote_fsync(fd);
		}
	#endif
		return 0;
	}

	int px4_unlink(const char *pathname)
	{
	#if defined(__PX4_FREERTOS)
		int ret = px4_rzv_remote_unlink(pathname);

		if (ret == 0 || errno != ENOSYS) {
			return ret;
		}
		errno = 0;
	#endif
		errno = ENOSYS;
		return -1;
	}

#endif /* __PX4_FREERTOS */
	int px4_ioctl(int fd, int cmd, unsigned long arg)
	{
#if defined(__PX4_FREERTOS)
		if (px4_rzv_remote_handles_fd(fd)) {
			errno = ENOTTY;
			return -1;
		}
#endif /* __PX4_FREERTOS */
		PX4_DEBUG("px4_ioctl fd = %d", fd);
		int ret = 0;

		cdev::CDev *dev = getFile(fd);

		if (dev) {
			ret = dev->ioctl(&filemap[fd], cmd, arg);

		} else {
			ret = -EINVAL;
		}

		return ret;
	}

	int px4_poll(px4_pollfd_struct_t *fds, unsigned int nfds, int timeout)
	{
#if defined(__PX4_FREERTOS)
		for (unsigned int i = 0; i < nfds; ++i) {
			if (px4_rzv_remote_handles_fd(fds[i].fd)) {
				errno = ENOTSUP;
				return -1;
			}
		}
#endif /* __PX4_FREERTOS */
		if (nfds == 0) {
			PX4_WARN("px4_poll with no fds");
			return -1;
		}

		px4_sem_t sem;
		int count = 0;
		int ret = -1;

		const unsigned NAMELEN = 32;
		char thread_name[NAMELEN] {};

#if defined(__PX4_FREERTOS)
		strncpy(thread_name, k_px4_freertos_thread_name, sizeof(thread_name) - 1);
		thread_name[sizeof(thread_name) - 1] = '\0';

		if (thread_name[0] == 0) {
			(void)snprintf(thread_name, NAMELEN, "cdev");
		}
#else
#ifndef __PX4_QURT
		int nret = pthread_getname_np(pthread_self(), thread_name, NAMELEN);

		if (nret || thread_name[0] == 0) {
			PX4_WARN("failed getting thread name");
		}
#endif
#endif /* __PX4_FREERTOS */

		PX4_DEBUG("Called px4_poll timeout = %d", timeout);

		px4_sem_init(&sem, 0, 0);

		// sem use case is a signal
		px4_sem_setprotocol(&sem, SEM_PRIO_NONE);

		// Go through all fds and check them for a pollable state
		bool fd_pollable = false;

		for (unsigned int i = 0; i < nfds; ++i) {
			fds[i].sem     = &sem;
			fds[i].revents = 0;
			fds[i].priv    = nullptr;

			cdev::CDev *dev = getFile(fds[i].fd);

			// If fd is valid
			if (dev) {
				PX4_DEBUG("%s: px4_poll: CDev->poll(setup) %d", thread_name, fds[i].fd);
				ret = dev->poll(&filemap[fds[i].fd], &fds[i], true);

				if (ret < 0) {
					PX4_WARN("%s: px4_poll() error: %s", thread_name, strerror(errno));
					break;
				}

				if (ret >= 0) {
					fd_pollable = true;
				}
			}
		}

		// If any FD can be polled, lock the semaphore and
		// check for new data
		if (fd_pollable) {
			if (timeout > 0) {
				// Get the current time
				struct timespec ts;
				// Note, we can't actually use CLOCK_MONOTONIC on macOS
				// but that's hidden and implemented in px4_clock_gettime.
				px4_clock_gettime(CLOCK_MONOTONIC, &ts);

				// Calculate an absolute time in the future
				const unsigned billion = (1000 * 1000 * 1000);
				uint64_t nsecs = ts.tv_nsec + ((uint64_t)timeout * 1000 * 1000);
				ts.tv_sec += nsecs / billion;
				nsecs -= (nsecs / billion) * billion;
				ts.tv_nsec = nsecs;

				ret = px4_sem_timedwait(&sem, &ts);

				if (ret && errno != ETIMEDOUT) {
					PX4_WARN("%s: px4_poll() sem error: %s", thread_name, strerror(errno));
				}

			} else if (timeout < 0) {
				px4_sem_wait(&sem);
			}

			// We have waited now (or not, depending on timeout),
			// go through all fds and count how many have data
			for (unsigned int i = 0; i < nfds; ++i) {

				cdev::CDev *dev = getFile(fds[i].fd);

				// If fd is valid
				if (dev) {
					PX4_DEBUG("%s: px4_poll: CDev->poll(teardown) %d", thread_name, fds[i].fd);
					ret = dev->poll(&filemap[fds[i].fd], &fds[i], false);

					if (ret < 0) {
						PX4_WARN("%s: px4_poll() 2nd poll fail", thread_name);
						break;
					}

					if (fds[i].revents) {
						count += 1;
					}
				}
			}
		}

		px4_sem_destroy(&sem);

		// Return the positive count if present,
		// return the negative error number if failed
		return (count) ? count : ret;
	}

	int px4_access(const char *pathname, int mode)
	{
#if defined(__PX4_FREERTOS)

		const int supported_modes = F_OK | R_OK | W_OK | X_OK;

		if ((mode & ~supported_modes) != 0) {
#else
		if (mode != F_OK) {
#endif /* __PX4_FREERTOS */
			errno = EINVAL;
			return -1;
		}

#if defined(__PX4_FREERTOS)
		int last_errno = ENOENT;

		int remote = px4_rzv_remote_access(pathname, mode);

		if (remote == 0) {
			return 0;
		}

		if (remote < 0) {
			last_errno = (errno != 0) ? errno : ENOSYS;

			if (last_errno != ENOSYS && last_errno != ENOENT) {
				errno = last_errno;
				return -1;
			}
		} else {
			last_errno = 0;
		}

	cdev::CDev *dev = getDev(pathname);

	if (dev != nullptr) {
		return 0;
#else
		cdev::CDev *dev = getDev(pathname);
		return (dev != nullptr) ? 0 : -1;
#endif /* __PX4_FREERTOS */
	}
#if defined(__PX4_FREERTOS)
#if defined(CONFIG_UART)
	const RzvUartConfig *uart_cfg = rzv_find_uart_config(pathname);

	if (uart_cfg == nullptr) {
		uart_cfg = rzv_find_uart_config_by_suffix(pathname);
	}

	if (uart_cfg != nullptr) {
		return 0;
	}

	if (strncmp(pathname, "/dev/ttyS", 9) == 0) {
		return 0;
	}
#endif /* CONFIG_UART */

	errno = last_errno;
	return -1;
}
#endif /* __PX4_FREERTOS */

	void px4_show_files()
	{
		PX4_INFO("Files:");

		pthread_mutex_lock(&devmutex);

		for (const auto &dev : devmap) {
			if (dev) {
				PX4_INFO_RAW("   %s\n", dev->name);
			}
		}

		pthread_mutex_unlock(&devmutex);
	}

} // extern "C"
