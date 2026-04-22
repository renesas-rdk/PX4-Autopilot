
/**
 * @file SPI.cpp
 *
 * POSIX-style SPI device wrapper for Renesas RZ/V platforms. The actual
 * hardware interaction is provided by the RZV FSP backend exposed through
 * character devices registered under /dev/spidevX.Y.
 */


#if defined(__PX4_FREERTOS)

#include "SPI.hpp"

#include <cerrno>
#include <cstring>

#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <posix_compat/spi/spidev.h>

#include <px4_platform_common/i2c_spi_buses.h>
#include <px4_platform_common/log.h>

namespace device
{

SPI::SPI(uint8_t device_type, const char *name, int bus, uint32_t device, enum spi_mode_e mode, uint32_t frequency) :
	CDev(name, nullptr),
	_device(device),
	_mode(mode),
	_frequency(frequency)
{
	_device_id.devid_s.devtype = device_type;
	_device_id.devid_s.bus_type = DeviceBusType_SPI;
	_device_id.devid_s.bus = bus;
	_device_id.devid_s.address = (uint8_t)device;
}

SPI::SPI(const I2CSPIDriverConfig &config)
	: SPI(config.devid_driver_index, config.module_name, config.bus, config.spi_devid, config.spi_mode,
	      config.bus_frequency)
{
}

SPI::~SPI()
{
	if (_fd >= 0) {
		::close(_fd);
		_fd = -1;
	}
}

int SPI::init()
{
	char dev_path[24];
	snprintf(dev_path, sizeof(dev_path), "/dev/spidev%i.%i", get_device_bus(), static_cast<int>(PX4_SPI_DEV_ID(_device)));
	_fd = ::open(dev_path, O_RDWR);

	if (_fd < 0) {
		PX4_ERR("could not open %s (%d)", dev_path, errno);
		return PX4_ERROR;
	}

	int ret = probe();

	if (ret != OK) {
		PX4_ERR("probe failed");
		return ret;
	}

	ret = CDev::init();

	if (ret != OK) {
		PX4_ERR("cdev init failed");
		return ret;
	}

	return PX4_OK;
}

int SPI::_transfer(uint8_t *send, uint8_t *recv, unsigned len)
{
	spi_ioc_transfer spi_transfer{};

	spi_transfer.tx_buf = reinterpret_cast<uint64_t>(send);
	spi_transfer.rx_buf = reinterpret_cast<uint64_t>(recv);
	spi_transfer.len = len;
	spi_transfer.speed_hz = _frequency;
	spi_transfer.bits_per_word = 8;

	int result = ::ioctl(_fd, SPI_IOC_MESSAGE(1), &spi_transfer);

	if (result != (int)len) {
		PX4_ERR("SPI transfer failed (%d, errno=%d)", result, errno);
		return PX4_ERROR;
	}

	return PX4_OK;
}

int SPI::transfer(uint8_t *send, uint8_t *recv, unsigned len)
{
	if ((send == nullptr) && (recv == nullptr)) {
		return -EINVAL;
	}

	uint32_t speed = _frequency;

	if (::ioctl(_fd, SPI_IOC_WR_MODE, &_mode) < 0) {
		PX4_ERR("Failed to set SPI mode (%d)", errno);
		return PX4_ERROR;
	}

	if (::ioctl(_fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed) < 0) {
		PX4_ERR("Failed to set SPI speed (%d)", errno);
		return PX4_ERROR;
	}

	return _transfer(send, recv, len);
}

int SPI::_transferhword(uint16_t *send, uint16_t *recv, unsigned len)
{
	spi_ioc_transfer spi_transfer{};

	spi_transfer.tx_buf = reinterpret_cast<uint64_t>(send);
	spi_transfer.rx_buf = reinterpret_cast<uint64_t>(recv);
	spi_transfer.len = len * 2;
	spi_transfer.speed_hz = _frequency;
	spi_transfer.bits_per_word = 16;

	int result = ::ioctl(_fd, SPI_IOC_MESSAGE(1), &spi_transfer);

	if (result != (int)(len * 2)) {
		PX4_ERR("SPI 16-bit transfer failed (%d, errno=%d)", result, errno);
		return PX4_ERROR;
	}

	return PX4_OK;
}

int SPI::transferhword(uint16_t *send, uint16_t *recv, unsigned len)
{
	if ((send == nullptr) && (recv == nullptr)) {
		return -EINVAL;
	}

	uint32_t speed = _frequency;

	if (::ioctl(_fd, SPI_IOC_WR_MODE, &_mode) < 0) {
		PX4_ERR("Failed to set SPI mode (%d)", errno);
		return PX4_ERROR;
	}

	uint8_t bits_per_word = 16;

	if (::ioctl(_fd, SPI_IOC_WR_BITS_PER_WORD, &bits_per_word) < 0) {
		PX4_ERR("Failed to set bits per word (%d)", errno);
		return PX4_ERROR;
	}

	if (::ioctl(_fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed) < 0) {
		PX4_ERR("Failed to set SPI speed (%d)", errno);
		return PX4_ERROR;
	}

	return _transferhword(send, recv, len);
}

} // namespace device

#endif /* __PX4_FREERTOS */
