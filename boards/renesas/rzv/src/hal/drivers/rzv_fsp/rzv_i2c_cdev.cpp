/****************************************************************************
 *
 * Copyright (c) 2025 Renesas Electronics Corporation and/or its affiliates
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 ****************************************************************************/


#if defined(__PX4_FREERTOS)

#define MODULE_NAME "rzv_i2c_cdev"

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/defines.h>

#include <cerrno>
#include <cstdlib>
#include <fcntl.h>
#include <unistd.h>

#include "rzv_fsp/i2c_fsp_backend.h"
#include "rzv_fsp/dma_buffer.h"
#include "posix_compat/i2c-dev.h"

#include <cdev/CDev.hpp>

namespace
{

using cdev::file_t;

/* DMA-safe bounce buffer for I2C transfers.
 *
 * The FSP I2C DMA engine requires buffers in the noncached memory region.
 * A single 256-byte bounce buffer is sufficient for all I2C sensor transfers
 * (BMP280 calibration=26B, BMP388=24B, BMM150=8B, ICP20100=16B).
 * Transfers are serialized by the FSP I2C backend, so one static buffer suffices.
 *
 * The buffer is shared across all messages in a single I2C_RDWR call using
 * a sequential offset scheme: each message gets a contiguous slice.
 */
static constexpr size_t I2C_BOUNCE_BUF_SIZE   = 256U;
static constexpr int    I2C_BOUNCE_MAX_MSGS    = 8;
static uint8_t s_i2c_bounce[I2C_BOUNCE_BUF_SIZE]
__attribute__((aligned(RZV_DMA_DEFAULT_ALIGNMENT), section(".noncache_buffer")));

class RZVI2CCDev : public cdev::CDev
{
public:
	RZVI2CCDev(const char *devname, uint8_t bus, rzv_i2c_backend_t &backend) :
		cdev::CDev(devname),
		_bus(bus),
		_backend(backend)
	{
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

		if (!_backend.opened) {
			if (rzv_i2c_backend_init(&_backend, _bus, 100000U) != 0) {
				cdev::CDev::close(filep);
				return -EIO;
			}
		}

		return PX4_OK;
	}

	int ioctl(file_t *, int cmd, unsigned long arg) override
	{
		if (static_cast<unsigned long>(cmd) == I2C_RDWR) {
			if (arg == 0UL) {
				return -EINVAL;
			}

			i2c_rdwr_ioctl_data *data = reinterpret_cast<i2c_rdwr_ioctl_data *>(arg);

			if ((data->msgs == nullptr) || (data->nmsgs == 0U)) {
				return -EINVAL;
			}

			const int nmsgs = static_cast<int>(data->nmsgs);

			if (nmsgs > I2C_BOUNCE_MAX_MSGS) {
				return -EINVAL;
			}

			/* Build a local copy of the msgs array.  For each message whose buffer
			 * is not DMA-safe we redirect it to a slice of s_i2c_bounce.  Write
			 * messages are copied in; read results are copied back after the transfer.
			 *
			 * bounce_offset tracks the next free byte in s_i2c_bounce.
			 * bounce_used[i] records whether message i used the bounce path.
			 */
			struct i2c_msg local_msgs[I2C_BOUNCE_MAX_MSGS];
			bool bounce_used[I2C_BOUNCE_MAX_MSGS] = {};
			size_t bounce_offset = 0U;

			memcpy(local_msgs, data->msgs, static_cast<size_t>(nmsgs) * sizeof(struct i2c_msg));

			for (int i = 0; i < nmsgs; ++i) {
				struct i2c_msg *msg = &local_msgs[i];

				if (msg->buf == nullptr || rzv_dma_buffer_is_dma_safe(msg->buf, msg->len)) {
					continue;
				}

				if (bounce_offset + msg->len > I2C_BOUNCE_BUF_SIZE) {
					PX4_ERR("I2C bounce buffer overflow (need %zu, have %zu)",
						bounce_offset + msg->len, I2C_BOUNCE_BUF_SIZE);
					return -ENOMEM;
				}

				if (!(msg->flags & I2C_M_RD)) {
					/* Write message: copy data into bounce buffer. */
					memcpy(s_i2c_bounce + bounce_offset, msg->buf, msg->len);
				}

				msg->buf = s_i2c_bounce + bounce_offset;
				bounce_used[i] = true;
				bounce_offset += msg->len;
			}

			if (rzv_i2c_backend_transfer(&_backend, local_msgs, nmsgs) != 0) {
				return -EIO;
			}

			/* Copy read data from bounce buffer back to caller's buffers. */
			bounce_offset = 0U;

			for (int i = 0; i < nmsgs; ++i) {
				if (bounce_used[i]) {
					if (data->msgs[i].flags & I2C_M_RD) {
						memcpy(data->msgs[i].buf, s_i2c_bounce + bounce_offset,
						       data->msgs[i].len);
					}

					bounce_offset += data->msgs[i].len;
				}
			}

			return nmsgs;
		}

		return -ENOTTY;
	}

private:
	uint8_t _bus{0U};
	rzv_i2c_backend_t &_backend;
};

static bool g_i2c_registered = false;

} // namespace

int rzv_init_i2c_cdev(rzv_i2c_backend_t *backend, uint8_t bus)
{
	if (backend == nullptr) {
		return -EINVAL;
	}

	if (!g_i2c_registered) {
		static RZVI2CCDev i2c_dev("/dev/i2c-7", bus, *backend);
		int init_ret = i2c_dev.init();

		if (init_ret == PX4_OK) {
			g_i2c_registered = true;
			return 0;
		}

		PX4_ERR("Failed to init /dev/i2c-7 (%d)", init_ret);
		return init_ret;
	}

	return 0;
}

#endif /* __PX4_FREERTOS */
