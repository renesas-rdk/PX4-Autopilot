/****************************************************************************
 *
 * Copyright (c) 2025 Renesas Electronics Corporation and/or its affiliates
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 ****************************************************************************/


#if defined(__PX4_FREERTOS)

#define MODULE_NAME "rzv_spi_cdev"

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/spi.h>

#include <array>
#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <memory>
#include <new>
#include <vector>
#include <unistd.h>

#include "rzv_fsp/spi_fsp_backend.h"
#include "posix_compat/spi/spidev.h"

#include <cdev/CDev.hpp>

namespace
{

using cdev::file_t;

/* This layer is intentionally thin: it only translates the spidev ioctl ABI
 * into per-file state and forwards transfers to the FSP backend. All
 * DMA-safety handling (bounce buffers, cache maintenance), bus mutual
 * exclusion, retries and per-device SSL routing live in spi_fsp_backend.c —
 * settings and transfer are applied there inside one critical section. */

struct SpiFileState {
	uint8_t mode{SPI_MODE_0};
	uint8_t bits_per_word{8};
	uint32_t speed_hz{1000000U};
};

class RZVSpiCDev : public cdev::CDev
{
public:
	RZVSpiCDev(const char *devname, uint8_t bus, uint8_t ssl_index, rzv_spi_backend_t &backend) :
		cdev::CDev(devname),
		_bus(bus),
		_ssl_index(ssl_index),
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
			if (rzv_spi_backend_init(&_backend, _bus) != 0) {
				PX4_ERR("SPI backend init failed");
				cdev::CDev::close(filep);
				return -EIO;
			}
		}

		/* new (not calloc): default member initializers (mode/bits/speed) must run */
		SpiFileState *state = new (std::nothrow) SpiFileState();

		if (state == nullptr) {
			cdev::CDev::close(filep);
			return -ENOMEM;
		}

		state->speed_hz = _backend.current_speed_hz;

		filep->f_priv = state;
		return PX4_OK;
	}

	int close(file_t *filep) override
	{
		if (filep != nullptr && filep->f_priv != nullptr) {
			delete static_cast<SpiFileState *>(filep->f_priv);
			filep->f_priv = nullptr;
		}

		return cdev::CDev::close(filep);
	}

	int ioctl(file_t *filep, int cmd, unsigned long arg) override
	{
		SpiFileState *state = (filep != nullptr) ? static_cast<SpiFileState *>(filep->f_priv) : nullptr;
		const unsigned int command = static_cast<unsigned int>(cmd);

		if (state == nullptr) {
			return -EINVAL;
		}

		const unsigned int request_type = _IOC_TYPE(command);
		const unsigned int request_nr = _IOC_NR(command);
		const unsigned int request_dir = _IOC_DIR(command);

		/* SPI_IOC_WR_* only update per-file state. The hardware is configured
		 * atomically with the next transfer inside the backend critical
		 * section — applying settings here would race transfers of other
		 * devices sharing the channel. SPI_IOC_RD_* return per-file state. */

		if ((request_type == SPI_IOC_MAGIC) && (request_nr == 1U) && (request_dir == _IOC_WRITE)) {
			if (arg == 0UL) {
				return -EINVAL;
			}

			state->mode = *reinterpret_cast<uint8_t *>(arg);
			return PX4_OK;
		}

		if ((request_type == SPI_IOC_MAGIC) && (request_nr == 1U) && (request_dir == _IOC_READ)) {
			if (arg == 0UL) {
				return -EINVAL;
			}

			*reinterpret_cast<uint8_t *>(arg) = state->mode;
			return PX4_OK;
		}

		if ((request_type == SPI_IOC_MAGIC) && (request_nr == 3U) && (request_dir == _IOC_WRITE)) {
			if (arg == 0UL) {
				return -EINVAL;
			}

			uint8_t bits = *reinterpret_cast<uint8_t *>(arg);

			if ((bits != 8U) && (bits != 16U)) {
				return -EINVAL;
			}

			state->bits_per_word = bits;
			return PX4_OK;
		}

		if ((request_type == SPI_IOC_MAGIC) && (request_nr == 3U) && (request_dir == _IOC_READ)) {
			if (arg == 0UL) {
				return -EINVAL;
			}

			*reinterpret_cast<uint8_t *>(arg) = state->bits_per_word;
			return PX4_OK;
		}

		if ((request_type == SPI_IOC_MAGIC) && (request_nr == 4U) && (request_dir == _IOC_WRITE)) {
			if (arg == 0UL) {
				return -EINVAL;
			}

			uint32_t speed = *reinterpret_cast<uint32_t *>(arg);

			if (speed == 0U) {
				return -EINVAL;
			}

			state->speed_hz = speed;
			return PX4_OK;
		}

		if ((request_type == SPI_IOC_MAGIC) && (request_nr == 4U) && (request_dir == _IOC_READ)) {
			if (arg == 0UL) {
				return -EINVAL;
			}

			*reinterpret_cast<uint32_t *>(arg) = state->speed_hz;
			return PX4_OK;
		}

		if ((request_type == SPI_IOC_MAGIC) && (request_nr == 0U)) {
			return handle_transfer(state, command, arg);
		}

		return cdev::CDev::ioctl(filep, cmd, arg);
	}

private:
	int handle_transfer(SpiFileState *state, unsigned int cmd, unsigned long arg)
	{
		if ((arg == 0UL) || (state == nullptr)) {
			return -EINVAL;
		}

		const size_t msg_size = _IOC_SIZE(cmd);
		const size_t transfers = (msg_size / sizeof(struct spi_ioc_transfer));

		if (transfers == 0U) {
			return 0;
		}

		struct spi_ioc_transfer *user_transfers = reinterpret_cast<struct spi_ioc_transfer *>(arg);
		size_t bytes_transferred = 0;

		for (size_t i = 0; i < transfers; i++) {
			struct spi_ioc_transfer xfer = user_transfers[i];

			rzv_spi_xfer_cfg_t cfg{};
			cfg.ssl_index = _ssl_index;
			cfg.mode = state->mode;
			cfg.bits_per_word = (xfer.bits_per_word != 0U) ? xfer.bits_per_word : state->bits_per_word;
			cfg.speed_hz = (xfer.speed_hz != 0U) ? xfer.speed_hz : state->speed_hz;

			const void *tx_ptr = reinterpret_cast<const void *>(static_cast<uintptr_t>(xfer.tx_buf));
			void *rx_ptr = reinterpret_cast<void *>(static_cast<uintptr_t>(xfer.rx_buf));

			/* Raw pointers go straight to the backend: it owns the DMA-safety
			 * decision (bounce/cache) and rejects len > bounce size. */
			if (rzv_spi_backend_transfer(&_backend, &cfg, tx_ptr, rx_ptr, xfer.len) != 0) {
				return -EIO;
			}

			/* Per-transfer overrides persist, matching prior behavior. */
			state->bits_per_word = cfg.bits_per_word;
			state->speed_hz = cfg.speed_hz;

			bytes_transferred += xfer.len;
		}

		return static_cast<int>(bytes_transferred);
	}

	uint8_t _bus{0U};
	uint8_t _ssl_index{0U};
	rzv_spi_backend_t &_backend;
};

static std::array<bool, SPI_BUS_MAX_BUS_ITEMS> g_spi_bus_registered{};
static std::vector<std::unique_ptr<RZVSpiCDev>> g_spi_devices;
} // namespace

int rzv_init_spi_cdevs(rzv_spi_backend_t *backend, uint8_t bus)
{
	if (backend == nullptr) {
		return -EINVAL;
	}

	if (bus >= SPI_BUS_MAX_BUS_ITEMS) {
		return -EINVAL;
	}

	if (g_spi_bus_registered[bus]) {
		return PX4_OK;
	}

	int ret = PX4_OK;
	bool found = false;
	const int target_bus = static_cast<int>(bus);

	for (int idx = 0; idx < SPI_BUS_MAX_BUS_ITEMS; ++idx) {
		const px4_spi_bus_t &cfg = px4_spi_buses[idx];

		if (cfg.bus != target_bus) {
			continue;
		}

		found = true;

		for (int dev_idx = 0; dev_idx < SPI_BUS_MAX_DEVICES; ++dev_idx) {
			const px4_spi_bus_device_t &device = cfg.devices[dev_idx];

			if (device.devid == 0) {
				continue;
			}

			const uint32_t chip_id = PX4_SPI_DEV_ID(device.devid);

			if (chip_id == 0) {
				continue;
			}

			/* cs_gpio is a logical chip-select index (1..4, see board_config.h);
			 * the physical SSL line is (cs_gpio - 1). 0 keeps the legacy
			 * "FSP default line" behavior (SSL0). */
			const uint8_t ssl_index = (device.cs_gpio != 0) ? static_cast<uint8_t>(device.cs_gpio - 1) : 0U;

			char dev_path[32];
			snprintf(dev_path, sizeof(dev_path), "/dev/spidev%u.%u", bus, static_cast<unsigned>(chip_id));

			auto cdev = std::make_unique<RZVSpiCDev>(dev_path, bus, ssl_index, *backend);
			int init_ret = cdev->init();

			if (init_ret != PX4_OK) {
				PX4_ERR("Failed to init %s (%d)", dev_path, init_ret);
				ret = (ret == PX4_OK) ? init_ret : ret;
				continue;
			}

			g_spi_devices.push_back(std::move(cdev));
		}
	}

	if (!found) {
		PX4_WARN("no SPI configuration found for bus %u", bus);
		return -ENOENT;
	}

	g_spi_bus_registered[bus] = true;
	return ret;
}

#endif /* __PX4_FREERTOS */
