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
#include <vector>
#include <unistd.h>

#include "rzv_fsp/spi_fsp_backend.h"
#include "rzv_fsp/dma_buffer.h"
#include "posix_compat/spi/spidev.h"

#include <cdev/CDev.hpp>

namespace
{

using cdev::file_t;

/* DMA-safe bounce buffers for SPI transfers.
 *
 * The FSP SPI DMA engine requires buffers in the noncached memory region.
 * Drivers (MPU9250, ICM20948, etc.) may pass stack or heap buffers that are
 * not DMA-safe. These static bounce buffers are placed in .noncache_buffer
 * and used transparently when the caller's buffer fails rzv_dma_buffer_is_dma_safe().
 *
 * Thread safety: SPI transfers on a single bus are already serialized by the
 * FSP backend mutex, so a single pair of static buffers per file is sufficient.
 *
 * Size: 1024 bytes covers the largest known sensor FIFO transfer
 * (MPU9250/ICM20948 FIFO: max ~512 bytes per batch read).
 */
static constexpr size_t SPI_BOUNCE_BUF_SIZE = 1024U;
static uint8_t s_spi_tx_bounce[SPI_BOUNCE_BUF_SIZE]
__attribute__((aligned(RZV_DMA_DEFAULT_ALIGNMENT), section(".noncache_buffer")));
static uint8_t s_spi_rx_bounce[SPI_BOUNCE_BUF_SIZE]
__attribute__((aligned(RZV_DMA_DEFAULT_ALIGNMENT), section(".noncache_buffer")));

struct SpiFileState {
	uint8_t mode{SPI_MODE_0};
	uint8_t bits_per_word{8};
	uint32_t speed_hz{1000000U};
};

class RZVSpiCDev : public cdev::CDev
{
public:
	RZVSpiCDev(const char *devname, uint8_t bus, rzv_spi_backend_t &backend) :
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
			if (rzv_spi_backend_init(&_backend, _bus) != 0) {
				PX4_ERR("SPI backend init failed");
				cdev::CDev::close(filep);
				return -EIO;
			}
		}

		SpiFileState *state = static_cast<SpiFileState *>(std::calloc(1, sizeof(SpiFileState)));

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
			std::free(filep->f_priv);
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

		if ((request_type == SPI_IOC_MAGIC) && (request_nr == 1U) && (request_dir == _IOC_WRITE)) {
			if (arg == 0UL) {
				return -EINVAL;
			}

			uint8_t mode = *reinterpret_cast<uint8_t *>(arg);

			if (rzv_spi_backend_set_mode(&_backend, mode) != 0) {
				return -EIO;
			}

			state->mode = mode;
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

			if (rzv_spi_backend_set_bits_per_word(&_backend, bits) != 0) {
				return -EIO;
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

			if (rzv_spi_backend_set_speed(&_backend, speed) != 0) {
				return -EIO;
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
	static constexpr int SPI_IOC_MAGIC_MASK = 0xffff0000;

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

			if (xfer.bits_per_word != 0U && xfer.bits_per_word != state->bits_per_word) {
				if (rzv_spi_backend_set_bits_per_word(&_backend, xfer.bits_per_word) != 0) {
					return -EIO;
				}

				state->bits_per_word = xfer.bits_per_word;
			}

			if (xfer.speed_hz != 0U && xfer.speed_hz != state->speed_hz) {
				if (rzv_spi_backend_set_speed(&_backend, xfer.speed_hz) != 0) {
					return -EIO;
				}

				state->speed_hz = xfer.speed_hz;
			}

			const void *tx_ptr = reinterpret_cast<const void *>(static_cast<uintptr_t>(xfer.tx_buf));
			void *rx_ptr = reinterpret_cast<void *>(static_cast<uintptr_t>(xfer.rx_buf));

			if (xfer.len > SPI_BOUNCE_BUF_SIZE) {
				PX4_ERR("SPI transfer len %u exceeds bounce buffer %zu", (unsigned)xfer.len, SPI_BOUNCE_BUF_SIZE);
				return -EINVAL;
			}

			/* Bounce tx/rx buffers into DMA-safe memory if needed. */
			const void *dma_tx = tx_ptr;
			void       *dma_rx = rx_ptr;

			const bool tx_bounce = tx_ptr && !rzv_dma_buffer_is_dma_safe(tx_ptr, xfer.len);
			const bool rx_bounce = rx_ptr && !rzv_dma_buffer_is_dma_safe(rx_ptr, xfer.len);

			if (tx_bounce) {
				memcpy(s_spi_tx_bounce, tx_ptr, xfer.len);
				dma_tx = s_spi_tx_bounce;
			}

			if (rx_bounce) {
				dma_rx = s_spi_rx_bounce;
			}

			if (rzv_spi_backend_transfer(&_backend, dma_tx, dma_rx, xfer.len) != 0) {
				return -EIO;
			}

			if (rx_bounce) {
				memcpy(rx_ptr, s_spi_rx_bounce, xfer.len);
			}

			bytes_transferred += xfer.len;
		}

		return static_cast<int>(bytes_transferred);
	}

	uint8_t _bus{0U};
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

			char dev_path[32];
			snprintf(dev_path, sizeof(dev_path), "/dev/spidev%u.%u", bus, static_cast<unsigned>(chip_id));

			auto cdev = std::make_unique<RZVSpiCDev>(dev_path, bus, *backend);
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
