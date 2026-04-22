
/**
 * @file I2C.cpp
 *
 * POSIX-style I2C wrapper for RZ/V platforms using the FSP-backed /dev/i2c-X
 * character devices.
 */


#if defined(__PX4_FREERTOS)

#include "I2C.hpp"

#include <cerrno>

#include <fcntl.h>
#include <posix_compat/i2c-dev.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <px4_platform_common/i2c_spi_buses.h>
#include <px4_platform_common/log.h>

namespace device
{

I2C::I2C(uint8_t device_type, const char *name, const int bus, const uint16_t address, const uint32_t frequency) :
	CDev(name, nullptr)
{
	_device_id.devid_s.devtype = device_type;
	_device_id.devid_s.bus_type = DeviceBusType_I2C;
	_device_id.devid_s.bus = bus;
	_device_id.devid_s.address = address;
}

I2C::I2C(const I2CSPIDriverConfig &config)
	: I2C(config.devid_driver_index, config.module_name, config.bus, config.i2c_address, config.bus_frequency)
{
}

I2C::~I2C()
{
	if (_fd >= 0) {
		::close(_fd);
		_fd = -1;
	}
}

int I2C::init()
{
	char dev_path[24] {};
	snprintf(dev_path, sizeof(dev_path), "/dev/i2c-%i", get_device_bus());
	_fd = ::open(dev_path, O_RDWR);

	if (_fd < 0) {
		PX4_ERR("failed to init I2C bus %d (%d)", get_device_bus(), errno);
		return -ENOENT;
	}

	int ret = probe();

	if (ret != OK) {
		PX4_ERR("I2C probe failed");
		return ret;
	}

	ret = CDev::init();

	if (ret != OK) {
		PX4_ERR("cdev init failed");
		return ret;
	}

	return PX4_OK;
}

int I2C::transfer(const uint8_t *send, const unsigned send_len, uint8_t *recv, const unsigned recv_len)
{
	if (_fd < 0) {
		PX4_ERR("I2C device not opened");
		return PX4_ERROR;
	}

	struct i2c_msg msgv[2] {};
	unsigned msgs = 0;

	if (send_len > 0) {
		msgv[msgs].addr = get_device_address();
		msgv[msgs].flags = 0;
		msgv[msgs].buf = const_cast<uint8_t *>(send);
		msgv[msgs].len = send_len;
		msgs++;
	}

	if (recv_len > 0) {
		msgv[msgs].addr = get_device_address();
		msgv[msgs].flags = I2C_M_RD;
		msgv[msgs].buf = recv;
		msgv[msgs].len = recv_len;
		msgs++;
	}

	if (msgs == 0) {
		return -EINVAL;
	}

	i2c_rdwr_ioctl_data packets{};
	packets.msgs  = msgv;
	packets.nmsgs = msgs;

	int ret = ::ioctl(_fd, I2C_RDWR, &packets);

	if (ret < 0) {
		PX4_DEBUG("I2C transfer failed addr=0x%02x errno=%d", get_device_address(), errno);
		return PX4_ERROR;
	}

	return PX4_OK;
}

} // namespace device

#endif /* __PX4_FREERTOS */
