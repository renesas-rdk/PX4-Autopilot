/****************************************************************************
 *
 * Copyright (c) 2025 Renesas Electronics Corporation and/or its affiliates
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 ****************************************************************************/


#if defined(__PX4_FREERTOS)

#define MODULE_NAME "rzv_uart_cdev"

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/defines.h>

#include <cerrno>
#include <new>
#include <optional>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <cstring>
#include <sys/ioctl.h>

#ifndef FIONREAD_LINUX
#define FIONREAD_LINUX 0x541B /* Linux/TIOCINQ value */
#endif
#ifndef FIONREAD_PX4
#define FIONREAD_PX4 0x6678 /* FreeRTOS POSIX stub value used by PX4 */
#endif

#include "rzv_fsp/uart_fsp_backend.h"
#include "rzv_fsp/device_registry.h"

#include <cdev/CDev.hpp>

#ifdef __cplusplus
extern "C" {
#endif
int uart_rc_buffer_available(uint8_t channel);
#if defined(__PX4_FREERTOS)
int uart_rc_buffer_read(uint8_t channel, uint8_t *data, int length);
#else
int uart_rc_buffer_read(uint8_t channel, uint8_t *data, uint32_t length);
#endif /* __PX4_FREERTOS */
void uart_rc_buffer_flush(uint8_t channel);
#ifdef __cplusplus
}
#endif

namespace

{

using cdev::file_t;

class RZVUartCDev : public cdev::CDev
{
public:
	RZVUartCDev(const char *devname, uint8_t logical, rzv_uart_backend_t &backend) :
		cdev::CDev(devname),
		_logical(logical),
		_backend(backend)
	{
		reset_termios_defaults();
	}

	int init()
	{
		return cdev::CDev::init();
	}

	int open(file_t *filep) override
	{
		int ret = cdev::CDev::open(filep);

		if (ret != PX4_OK) {
			return ret;
		}

		if ((_open_count == 0U) && !_backend.opened) {
			if (rzv_uart_backend_init(&_backend, _logical) != 0) {
				cdev::CDev::close(filep);
				return -EIO;
			}
		}

		++_open_count;
		refresh_termios_from_backend();
		return PX4_OK;
	}

	ssize_t read(file_t *, char *buffer, size_t buflen) override
	{
		if ((buffer == nullptr) || (buflen == 0)) {
			return -EINVAL;
		}

		/* RC channel (0): Read from ring buffer directly - NO blocking call */
		if (_logical == 0) {
			int available = uart_rc_buffer_available(_logical);
			if (available <= 0) {
				return 0;  /* No data available, non-blocking */
			}
			
			size_t to_read = (buflen < (size_t)available) ? buflen : (size_t)available;
			int ret = uart_rc_buffer_read(_logical, reinterpret_cast<uint8_t *>(buffer), (uint32_t)to_read);
			return (ret < 0) ? -EIO : static_cast<ssize_t>(ret);
		}

		/* Other channels: Use blocking read */
		int ret = rzv_uart_backend_read(&_backend, reinterpret_cast<uint8_t *>(buffer), buflen);

		return (ret < 0) ? ret : static_cast<ssize_t>(ret);
	}

	ssize_t write(file_t *, const char *buffer, size_t buflen) override
	{
		if ((buffer == nullptr) || (buflen == 0)) {
			return -EINVAL;
		}

		int ret = rzv_uart_backend_write(&_backend, reinterpret_cast<const uint8_t *>(buffer), buflen);

		return (ret < 0) ? ret : static_cast<ssize_t>(ret);
	}

	int close(file_t *filep) override
	{
		int ret = cdev::CDev::close(filep);

		if (ret == PX4_OK) {
			if (_open_count > 0U) {
				--_open_count;
			}

			if ((_open_count == 0U) && _backend.opened) {
				rzv_uart_backend_shutdown(&_backend);
			}
		}

		return ret;
	}

	int ioctl(file_t *filep, int cmd, unsigned long arg) override
	{
		(void)filep; // Unused parameter

		switch (cmd) {
		case FIONREAD:
#if FIONREAD != FIONREAD_LINUX
		case FIONREAD_LINUX:
#endif
#if FIONREAD != FIONREAD_PX4
		case FIONREAD_PX4:
#endif
			if (arg == 0UL) {
				return -EINVAL;
			}

			*reinterpret_cast<int *>(arg) = uart_rc_buffer_available(_logical);
			return PX4_OK;

#ifdef FIONSPACE
		case FIONSPACE:
			if (arg == 0UL) {
				return -EINVAL;
			}

			*reinterpret_cast<int *>(arg) = 512;
			return PX4_OK;
#endif

#ifdef TIOCINQ
		case TIOCINQ:
			if (arg == 0UL) {
				return -EINVAL;
			}

			*reinterpret_cast<int *>(arg) = uart_rc_buffer_available(_logical);
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
				if (_logical == 0) {
					uart_rc_buffer_flush(_logical);
				}
				return PX4_OK;
			}

			return PX4_OK;
#endif

		default:
			return cdev::CDev::ioctl(filep, cmd, arg);
		}
	}

private:
	uint8_t _logical{0U};
	rzv_uart_backend_t &_backend;
	unsigned _open_count{0U};

	struct {
		uint32_t baud{115200};
		termios cfg{};
		bool single_wire{false};
		bool inverted{false};
		bool swap_lines{false};
	} _state;

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
		_state.baud = 115200U;
		termios default_cfg{};
		default_cfg.c_cflag = CLOCAL | CREAD | CS8;
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
		if (_backend.opened && _backend.baudrate != 0U) {
			_state.baud = _backend.baudrate;
		}

#ifdef CBAUD
		_state.cfg.c_cflag &= ~CBAUD;
		_state.cfg.c_cflag |= static_cast<tcflag_t>(encode_speed(_state.baud));
#endif
	}

	int apply_termios(const termios &cfg)
	{
		// Extract baudrate from termios
		uint32_t new_baud = _state.baud;

#ifdef CBAUD
		const tcflag_t baud_bits = cfg.c_cflag & CBAUD;
		uint32_t decoded = decode_speed(static_cast<speed_t>(baud_bits));

		if ((decoded != 0U) || (baud_bits == 0)) {
			new_baud = decoded;
		}
#else
		// If no CBAUD, try cfgetispeed if available
#ifdef cfgetispeed
		new_baud = decode_speed(cfgetispeed(&cfg));
#endif
#endif

		if (new_baud == 0U) {
			new_baud = _state.baud; // Keep current if invalid
		}

		if (new_baud != _state.baud) {
			if (_backend.opened) {
				if (rzv_uart_backend_configure(&_backend, new_baud) != 0) {
					PX4_ERR("Failed to set baudrate %u on logical %u", (unsigned)new_baud, (unsigned)_logical);
					return -EIO;
				}
			}

			_state.baud = new_baud;
		}

		_state.cfg = cfg;
#ifdef CBAUD
		_state.cfg.c_cflag &= ~CBAUD;
		_state.cfg.c_cflag |= static_cast<tcflag_t>(encode_speed(_state.baud));
#endif

		return PX4_OK;
	}
};

} // namespace

int rzv_init_uart_cdev(const char *devname, uint8_t logical_channel, rzv_uart_backend_t *backend)
{
	if ((devname == nullptr) || (backend == nullptr)) {
		return -EINVAL;
	}

	static std::optional<RZVUartCDev> uart_nodes[4];
	static bool initialized[4] = {};

	if (logical_channel >= 4U) {
		return -EINVAL;
	}

	if (!uart_nodes[logical_channel].has_value()) {
		uart_nodes[logical_channel].emplace(devname, logical_channel, *backend);
	}

	if (!initialized[logical_channel]) {
		int ret = uart_nodes[logical_channel]->init();

		if (ret == PX4_OK) {
			initialized[logical_channel] = true;
			return 0;
		}

		PX4_ERR("Failed to init %s (%d)", devname, ret);
		return ret;
	}

	return 0;
}

#endif /* __PX4_FREERTOS */
