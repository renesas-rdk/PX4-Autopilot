/****************************************************************************
 *
 * Copyright (c) 2025 Renesas Electronics Corporation and/or its affiliates
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 ****************************************************************************/

/**
 * @file serial_impl_freertos_stub.cpp
 * @brief Minimal stub for device::SerialImpl on the RZ/V FreeRTOS platform.
 *
 * The real UART implementation lives in rzv_uart_cdev.cpp. This stub only
 * satisfies the link requirements from Serial.cpp when the POSIX serial
 * backend is disabled.
 */


#if defined(__PX4_FREERTOS)

#include <SerialImpl.hpp>
#include <cstring>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>

#include <FreeRTOS.h>
#include <task.h>

#include "rzv_fsp/uart_transport.h"

extern "C" {
int open(const char *name, int flags, ...);
int close(int file);
ssize_t read(int file, void *ptr, size_t len);
ssize_t write(int file, const void *ptr, size_t len);
}

using namespace device;

SerialImpl::SerialImpl(const char *port, uint32_t baudrate,
		       ByteSize bytesize, Parity parity, StopBits stopbits, FlowControl flowcontrol)
{
	if (port != nullptr) {
		strncpy(_port, port, sizeof(_port) - 1);
		_port[sizeof(_port) - 1] = '\0';
	} else {
		_port[0] = '\0';
	}

	_baudrate = baudrate;
	_bytesize = bytesize;
	_parity = parity;
	_stopbits = stopbits;
	_flowcontrol = flowcontrol;
}

SerialImpl::~SerialImpl() = default;

bool SerialImpl::open()
{
	if (_open) {
		return true;
	}

	int fd = ::open(_port, O_RDWR | O_NOCTTY | O_NONBLOCK, 0);

	if (fd < 0) {
		return false;
	}

	_serial_fd = fd;

	int channel = rzv_uart_transport_map_device(_port, nullptr, nullptr);
	if ((channel >= 0) && (_baudrate != 0U)) {
		(void)rzv_uart_transport_set_baudrate_channel((uint8_t)channel, _baudrate);
	}

	_open = true;
	return true;
}

bool SerialImpl::isOpen() const
{
	return _open;
}

bool SerialImpl::close()
{
	if (!_open) {
		return true;
	}

	::close(_serial_fd);
	_serial_fd = -1;
	_open = false;
	return true;
}

ssize_t SerialImpl::read(uint8_t *buffer, size_t buffer_size)
{
	if (!_open || buffer == nullptr || buffer_size == 0) {
		return -1;
	}

	return ::read(_serial_fd, buffer, buffer_size);
}

ssize_t SerialImpl::readAtLeast(uint8_t *buffer, size_t buffer_size, size_t character_count, uint32_t timeout_us)
{
	(void)character_count;

	/* Try non-blocking read first — if ring buffer already has data, return immediately */
	ssize_t ret = read(buffer, buffer_size);

	if (ret > 0) {
		return ret;
	}

	/* No data yet. We MUST yield CPU here, otherwise the GPS driver's tight retry loop
	 * in UBX::receive() will spin at 100% CPU and starve all other FreeRTOS tasks
	 * (SPI/I2C sensors, work queues, etc.) causing system-wide TIMEOUT errors.
	 *
	 * Use the ring buffer semaphore to block efficiently until data arrives from the
	 * UART ISR, or until timeout. The parameter name says timeout_us but callers
	 * (Serial::readAtLeast) actually pass milliseconds. */
	const int channel = rzv_uart_transport_map_device(_port, nullptr, nullptr);

	if (channel >= 0) {
		int wait_ms = (timeout_us > 0U) ? (int)timeout_us : 1;

		if (wait_ms > 50) {
			wait_ms = 50;
		}

		int avail = rzv_uart_transport_wait_for_data((uint8_t)channel, wait_ms);

		if (avail > 0) {
			return read(buffer, buffer_size);
		}
	} else {
		/* Unknown channel: yield 1 tick to prevent spin */
		vTaskDelay(1);
	}

	return 0;
}

ssize_t SerialImpl::write(const void *buffer, size_t buffer_size)
{
	if (!_open || buffer == nullptr || buffer_size == 0) {
		return -1;
	}

	return ::write(_serial_fd, buffer, buffer_size);
}

void SerialImpl::flush()
{
}

const char *SerialImpl::getPort() const
{
	return (_port[0] != '\0') ? _port : nullptr;
}

bool SerialImpl::validatePort(const char *port)
{
	return (port != nullptr) && (port[0] != '\0');
}

bool SerialImpl::setPort(const char *port)
{
	if (!validatePort(port)) {
		return false;
	}

	strncpy(_port, port, sizeof(_port) - 1);
	_port[sizeof(_port) - 1] = '\0';
	return true;
}

bool SerialImpl::setBaudrate(uint32_t baudrate)
{
	_baudrate = baudrate;

	if (_open) {
		int channel = rzv_uart_transport_map_device(_port, nullptr, nullptr);

		if ((channel >= 0) && (rzv_uart_transport_set_baudrate_channel((uint8_t)channel, _baudrate) != 0)) {
			return false;
		}
	}

	return true;
}

uint32_t SerialImpl::getBaudrate() const
{
	return _baudrate;
}

ByteSize SerialImpl::getBytesize() const
{
	return ByteSize::EightBits;
}

bool SerialImpl::setBytesize(ByteSize bytesize)
{
	(void)bytesize;
	return false;
}

Parity SerialImpl::getParity() const
{
	return Parity::None;
}

bool SerialImpl::setParity(Parity parity)
{
	(void)parity;
	return false;
}

StopBits SerialImpl::getStopbits() const
{
	return StopBits::One;
}

bool SerialImpl::setStopbits(StopBits stopbits)
{
	(void)stopbits;
	return false;
}

FlowControl SerialImpl::getFlowcontrol() const
{
	return FlowControl::Disabled;
}

bool SerialImpl::setFlowcontrol(FlowControl flowcontrol)
{
	(void)flowcontrol;
	return false;
}

bool SerialImpl::getSingleWireMode() const
{
	return false;
}

bool SerialImpl::setSingleWireMode()
{
	return false;
}

bool SerialImpl::getSwapRxTxMode() const
{
	return false;
}

bool SerialImpl::setSwapRxTxMode()
{
	return false;
}

bool SerialImpl::setInvertedMode(bool enable)
{
	(void)enable;
	return false;
}

bool SerialImpl::getInvertedMode() const
{
	return false;
}

#endif /* __PX4_FREERTOS */
