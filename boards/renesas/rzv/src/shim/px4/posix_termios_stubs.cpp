/****************************************************************************
 *
 * Copyright (c) 2025 Renesas Electronics Corporation and/or its affiliates
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 ****************************************************************************/

/**
 * @file posix_termios_stubs.cpp
 * @brief POSIX termios stubs for Renesas RZ/V2H
 */

#if defined(__PX4_FREERTOS)

#include <termios.h>
#include <errno.h>

#include "board/uart_rc_switch.h"

namespace {

constexpr int kUartFdBase = 200;
constexpr int kUartSupportedChannels = 4;

} // namespace

extern "C" {

int cfsetispeed(struct termios *termios_p, speed_t speed)
{
	if (!termios_p) {
		errno = EINVAL;
		return -1;
	}

	// Store speed (c_cflag encoding is simple - baud rate value directly)
	// For RZ/V2H UART.cpp, it extracts baud from ioctl, not from termios
	(void)speed;
	return 0;
}

int cfsetospeed(struct termios *termios_p, speed_t speed)
{
	if (!termios_p) {
		errno = EINVAL;
		return -1;
	}

	(void)speed;
	return 0;
}

int tcgetattr(int fd, struct termios *termios_p)
{
	if (fd < 0 || !termios_p) {
		errno = EINVAL;
		return -1;
	}

	// Initialize with safe defaults for serial port
	termios_p->c_iflag = 0;                     // No input processing
	termios_p->c_oflag = 0;                     // No output processing
	termios_p->c_cflag = CS8 | CREAD | CLOCAL;  // 8N1, enable receiver, ignore modem control
	termios_p->c_lflag = 0;                     // No line processing (raw mode)
	
	// VMIN=1, VTIME=0 for blocking read of at least 1 char
	termios_p->c_cc[VMIN] = 1;
	termios_p->c_cc[VTIME] = 0;
	
	return 0;
}

int tcsetattr(int fd, int optional_actions, const struct termios *termios_p)
{
	if (fd < 0 || !termios_p) {
		errno = EINVAL;
		return -1;
	}

	// optional_actions: TCSANOW, TCSADRAIN, TCSAFLUSH
	// For now, we just accept the request without actually applying settings
	// The UART driver (UART.cpp) already handles baud rate via ioctl()
	(void)optional_actions;
	
	return 0;
}

int tcflush(int fd, int queue_selector)
{
	if (fd < 0) {
		errno = EINVAL;
		return -1;
	}

	if ((queue_selector != TCIFLUSH) && (queue_selector != TCOFLUSH) && (queue_selector != TCIOFLUSH)) {
		errno = EINVAL;
		return -1;
	}

	if ((fd >= kUartFdBase) && (fd < (kUartFdBase + kUartSupportedChannels))) {
		if ((queue_selector == TCIFLUSH) || (queue_selector == TCIOFLUSH)) {
			const int channel = fd - kUartFdBase;

			if (channel >= 0 && channel < kUartSupportedChannels) {
				uart_rc_buffer_flush(static_cast<uint8_t>(channel));
			}
		}

		// No dedicated TX queue to flush; treat as success.
		return 0;
	}

	// For non-UART descriptors, nothing to flush but treat as success.
	return 0;
}

}

#endif /* __PX4_FREERTOS */
