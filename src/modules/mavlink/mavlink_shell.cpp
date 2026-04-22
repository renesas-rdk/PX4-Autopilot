/****************************************************************************
 *
 *   Copyright (c) 2016-2022 PX4 Development Team. All rights reserved.
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

/**
 * @file mavlink_shell.cpp
 * A shell to be used via MAVLink
 *
 * @author Beat Küng <beat-kueng@gmx.net>
 */

#include "mavlink_shell.h"
#include <px4_platform_common/defines.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/posix.h>

#include <unistd.h>
#include <errno.h>
#include <sys/ioctl.h>

#ifdef __PX4_NUTTX
#include <nshlib/nshlib.h>
#endif /* __PX4_NUTTX */

#ifdef __PX4_POSIX
#include "../../../platforms/posix/src/px4/common/px4_daemon/pxh.h"
#endif /* __PX4_POSIX */

#ifdef __PX4_CYGWIN
#include <asm/socket.h>
#endif

#if defined(__PX4_FREERTOS)
#include <atomic>
#include <cctype>
#include <string>

namespace
{
static constexpr size_t SHELL_RX_BUFFER_SIZE = 256;
static constexpr size_t SHELL_TX_BUFFER_SIZE = 1024;
static constexpr TickType_t SHELL_STREAM_WAIT = pdMS_TO_TICKS(20);

// Accessed from multiple tasks: MAVLink thread (set/clear) and any PX4 logging
// thread (read via mavlink_shell_stdout_hook). Must be atomic.
std::atomic<MavlinkShell *> g_active_shell{nullptr};
}

MavlinkShell::~MavlinkShell()
{
	if (g_active_shell == this) {
		g_active_shell = nullptr;
	}

	_should_exit.store(true);

	if (_rx_stream != nullptr) {
		const uint8_t wake = '\n';
		(void)xStreamBufferSend(_rx_stream, &wake, sizeof(wake), SHELL_STREAM_WAIT);
	}

	if (_task >= 0) {
		px4_task_delete(_task);
		_task = -1;
	}

	if (_tx_mutex != nullptr) {
		vSemaphoreDelete(_tx_mutex);
		_tx_mutex = nullptr;
	}

	if (_rx_stream != nullptr) {
		vStreamBufferDelete(_rx_stream);
		_rx_stream = nullptr;
	}

	if (_tx_stream != nullptr) {
		vStreamBufferDelete(_tx_stream);
		_tx_stream = nullptr;
	}
}

int MavlinkShell::start()
{
	if (_task >= 0) {
		return 0;
	}

	_rx_stream = xStreamBufferCreate(SHELL_RX_BUFFER_SIZE, 1);
	_tx_stream = xStreamBufferCreate(SHELL_TX_BUFFER_SIZE, 1);
	_tx_mutex = xSemaphoreCreateMutex();

	if ((_rx_stream == nullptr) || (_tx_stream == nullptr) || (_tx_mutex == nullptr)) {
		if (_tx_mutex != nullptr) {
			vSemaphoreDelete(_tx_mutex);
			_tx_mutex = nullptr;
		}

		if (_rx_stream != nullptr) {
			vStreamBufferDelete(_rx_stream);
			_rx_stream = nullptr;
		}

		if (_tx_stream != nullptr) {
			vStreamBufferDelete(_tx_stream);
			_tx_stream = nullptr;
		}

		errno = ENOMEM;
		return -ENOMEM;
	}

	_should_exit.store(false);
	g_active_shell = this;

	// 8192 bytes: Pxh::process_line() can call PX4 module commands with deep stacks
	// (e.g. param set, logger, ekf2 status). 3072 was too small for some commands.
	_task = px4_task_spawn_cmd("mavlink_shell",
			   SCHED_DEFAULT,
			   SCHED_PRIORITY_DEFAULT,
			   8192,
			   &MavlinkShell::shell_start_thread,
			   nullptr);

	if (_task < 0) {
		g_active_shell = nullptr;
		vSemaphoreDelete(_tx_mutex);
		_tx_mutex = nullptr;
		vStreamBufferDelete(_rx_stream);
		_rx_stream = nullptr;
		vStreamBufferDelete(_tx_stream);
		_tx_stream = nullptr;
		return -errno;
	}

	append_prompt();

	return 0;
}

size_t MavlinkShell::write(uint8_t *buffer, size_t len)
{
	if ((buffer == nullptr) || (len == 0) || (_rx_stream == nullptr)) {
		return 0;
	}

	size_t total_sent = 0;

	while (total_sent < len) {
		const size_t sent = xStreamBufferSend(_rx_stream,
						      buffer + total_sent,
						      len - total_sent,
						      SHELL_STREAM_WAIT);

		if (sent == 0) {
			break;
		}

		total_sent += sent;
	}

	return total_sent;
}

size_t MavlinkShell::read(uint8_t *buffer, size_t len)
{
	if ((buffer == nullptr) || (len == 0) || (_tx_stream == nullptr)) {
		return 0;
	}

	return xStreamBufferReceive(_tx_stream, buffer, len, 0);
}

size_t MavlinkShell::available()
{
	if (_tx_stream == nullptr) {
		return 0;
	}

	return xStreamBufferBytesAvailable(_tx_stream);
}

void MavlinkShell::append_output(const char *buffer, size_t len)
{
	if ((buffer == nullptr) || (len == 0) || (_tx_stream == nullptr) || (_tx_mutex == nullptr)) {
		return;
	}

	if (xSemaphoreTake(_tx_mutex, SHELL_STREAM_WAIT) == pdTRUE) {
		xStreamBufferSend(_tx_stream, buffer, len, SHELL_STREAM_WAIT);
		xSemaphoreGive(_tx_mutex);
	}
}

void MavlinkShell::append_prompt()
{
	static constexpr const char prompt[] = "pxh> ";
	append_output(prompt, sizeof(prompt) - 1);
}

void MavlinkShell::shell_thread_main()
{
	std::string line;
	line.reserve(128);
	bool last_was_cr = false;

	while (!_should_exit.load()) {
		uint8_t ch = 0;
		const size_t received = xStreamBufferReceive(_rx_stream, &ch, sizeof(ch), portMAX_DELAY);

		if (received == 0) {
			continue;
		}

		if (ch == '\r' || ch == '\n') {
			if (!(ch == '\n' && last_was_cr)) {
				append_output("\r\n", 2);
			}

			last_was_cr = (ch == '\r');

			if (!line.empty()) {
				px4_daemon::Pxh::process_line(line, false);
				line.clear();
			}

			append_prompt();
			continue;
		}

		last_was_cr = false;

		if (ch == 0x03) { // CTRL+C
			line.clear();
			append_output("^C\r\n", 4);
			append_prompt();
			continue;
		}

		if (ch == '\b' || ch == 0x7f) {
			if (!line.empty()) {
				line.pop_back();
				append_output("\b \b", 3);
			}

			continue;
		}

		if (std::isprint(ch) != 0) {
			line.push_back(static_cast<char>(ch));
			const char echo = static_cast<char>(ch);
			append_output(&echo, 1);
		}
	}

	if (g_active_shell == this) {
		g_active_shell = nullptr;
	}
}

int MavlinkShell::shell_start_thread(int argc, char *argv[])
{
	(void)argc;
	(void)argv;

	MavlinkShell *shell = g_active_shell.load();

	if (shell != nullptr) {
		shell->shell_thread_main();
	}

	return 0;
}

extern "C" void mavlink_shell_stdout_hook(const char *buffer, size_t len)
{
	// Load atomically once; safe to call ->append_output() if pointer is non-null
	// since the shell is destroyed only after _should_exit is set and the task joined.
	MavlinkShell *shell = g_active_shell.load();

	if ((shell != nullptr) && (buffer != nullptr) && (len > 0)) {
		shell->append_output(buffer, len);
	}
}

extern "C" bool mavlink_shell_stdout_active(void)
{
	return g_active_shell.load() != nullptr;
}

#else /* !__PX4_FREERTOS */

MavlinkShell::~MavlinkShell()
{
	//closing the pipes will stop the thread as well
	if (_to_shell_fd >= 0) {
		PX4_INFO("Stopping mavlink shell");
		close(_to_shell_fd);
	}

	if (_from_shell_fd >= 0) {
		close(_from_shell_fd);
	}
}

int MavlinkShell::start()
{
	//this currently only works for NuttX & POSIX
#if !defined(__PX4_NUTTX) && !defined(__PX4_POSIX)
	return -1;
#endif

	PX4_INFO("Starting mavlink shell");

	int p1[2], p2[2];

	/* Create the shell task and redirect its stdin & stdout. If we used pthread, we would redirect
	 * stdin/out of the calling process as well, so we need px4_task_spawn_cmd. However NuttX only
	 * keeps (duplicates) the first 3 fd's when creating a new task, all others are not inherited.
	 * This means we need to temporarily change the first 3 fd's of the current task (or at least
	 * the first 2 if stdout=stderr).
	 */

	if (pipe(p1) != 0) {
		return -errno;
	}

	if (pipe(p2) != 0) {
		close(p1[0]);
		close(p1[1]);
		return -errno;
	}

	int ret = 0;

	_from_shell_fd  = p1[0];
	_to_shell_fd = p2[1];
	_shell_fds[0]  = p2[0];
	_shell_fds[1] = p1[1];

	/*
	 * Ensure that during the temporary phase no other thread from the same task writes to
	 * stdout (as it would end up in the pipe).
	 */
#ifdef __PX4_NUTTX
	sched_lock();
#endif /* __PX4_NUTTX */

#ifdef __PX4_POSIX
	int remote_in_fd = dup(_shell_fds[0]);	// Input file descriptor for the remote shell
	int remote_out_fd = dup(_shell_fds[1]); // Output file descriptor for the remote shell

	char r_in[32];
	char r_out[32];
	snprintf(r_in, sizeof(r_in), "%d", remote_in_fd);
	snprintf(r_out, sizeof(r_out), "%d", remote_out_fd);
	char *const argv[3] = {r_in, r_out, nullptr};

#else
	int fd_backups[2]; //we don't touch stderr, we will redirect it to stdout in the startup of the shell task

	for (int i = 0; i < 2; ++i) {
		fd_backups[i] = dup(i);

		if (fd_backups[i] == -1) {
			ret = -errno;
		}
	}

	dup2(_shell_fds[0], 0);
	dup2(_shell_fds[1], 1);
#endif

	if (ret == 0) {
		_task = px4_task_spawn_cmd("mavlink_shell",
					   SCHED_DEFAULT,
					   SCHED_PRIORITY_DEFAULT,
					   2048,
					   &MavlinkShell::shell_start_thread,
#ifdef __PX4_POSIX
					   argv);
#else
					   nullptr);
#endif

		if (_task < 0) {
			ret = -1;
		}
	}

#if !defined(__PX4_POSIX)

	//restore fd's
	for (int i = 0; i < 2; ++i) {
		if (dup2(fd_backups[i], i) == -1) {
			ret = -errno;
		}

		close(fd_backups[i]);
	}

#endif

	//close unused pipe fd's
	close(_shell_fds[0]);
	close(_shell_fds[1]);

#ifdef __PX4_NUTTX
	sched_unlock();
#endif /* __PX4_NUTTX */

	return ret;
}

int MavlinkShell::shell_start_thread(int argc, char *argv[])
{
#ifdef __PX4_NUTTX
	dup2(1, 2); //redirect stderror to stdout

	const int ret = nsh_consolemain(0, NULL);

	if (ret) {
		PX4_ERR("Mavlink shell failed: %d%s", ret, (ret == -ENOMEM) ? " (out of memory)" : "");
		return ret;
	}

#endif /* __PX4_NUTTX */

#ifdef __PX4_POSIX

	if (argc != 3) {
		PX4_ERR("Mavlink shell bug");
		return -1;
	}

	int remote_in_fd = atoi(argv[1]);
	int remote_out_fd = atoi(argv[2]);

	px4_daemon::Pxh pxh;
	pxh.run_remote_pxh(remote_in_fd, remote_out_fd);
#endif

	return 0;
}

size_t MavlinkShell::write(uint8_t *buffer, size_t len)
{
	return ::write(_to_shell_fd, buffer, len);
}

size_t MavlinkShell::read(uint8_t *buffer, size_t len)
{
	return ::read(_from_shell_fd, buffer, len);
}

size_t MavlinkShell::available()
{
	int ret = 0;

	if (ioctl(_from_shell_fd, FIONREAD, (unsigned long)&ret) == OK) {
		return ret;
	}

	return 0;
}
#endif /* __PX4_FREERTOS */
