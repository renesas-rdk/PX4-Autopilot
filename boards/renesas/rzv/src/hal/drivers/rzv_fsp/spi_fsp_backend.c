/****************************************************************************
 *
 * Copyright (c) 2025 Renesas Electronics Corporation and/or its affiliates
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 ****************************************************************************/


#if defined(__PX4_FREERTOS)

#include "rzv_fsp/spi_fsp_backend.h"
#define MODULE_NAME "rzv_spi_backend"

#include <FreeRTOS.h>
#include <semphr.h>
#include <task.h>

#include <px4_platform_common/log.h>
#include <px4_platform_common/time.h>
#include <errno.h>

#include <string.h>
#include <stdint.h>

#include "bsp_api.h"
#include "r_spi_b.h"
#include "rzv_fsp/dma_buffer.h"
#include "posix_compat/spi/spidev.h"

#define RZV_SPI_BOUNCE_LEN 1024U
/* DMA heuristics: prefer DMA once transfers are cache-line safe, word aligned,
 * and large enough that the setup overhead is amortized. Smaller or misaligned
 * operations stay on the bounce buffers so the CPU handles the copies. */
#define RZV_SPI_DMA_MIN_BYTES 64U
#define RZV_SPI_DMA_WORD_ALIGN 4U
#define RZV_SPI_TRANSFER_TIMEOUT_US 20000U
/* Number of retries after a transfer error (timeout or abort).
 * After each failure the SPI peripheral is reset via rzv_spi_reopen() to
 * clear any stuck state before the next attempt.  Two retries bring the
 * double-failure probability from 0.17 err/s to ~0.03 err/s at idle and
 * correspondingly reduce motor-EMI-induced errors during flight. */
#define RZV_SPI_TRANSFER_MAX_RETRIES 2
#define RZV_SPI_EVENT_RESET ((spi_event_t)0)

#if defined(__GNUC__)
#define RZV_SPI_DMA_ALIGN __attribute__((aligned(32)))
#define RZV_SPI_DMA_NONCACHE __attribute__((section(".noncache_buffer")))
#else
#define RZV_SPI_DMA_ALIGN
#define RZV_SPI_DMA_NONCACHE
#endif

static StaticSemaphore_t g_spi_mutex_buffer;
static SemaphoreHandle_t g_spi_mutex;
static StaticSemaphore_t g_spi_event_sem_buffer;
static SemaphoreHandle_t g_spi_event_sem;

static volatile spi_event_t g_spi_event = RZV_SPI_EVENT_RESET;

static uint8_t g_spi_tx_bounce[RZV_SPI_BOUNCE_LEN] RZV_SPI_DMA_ALIGN RZV_SPI_DMA_NONCACHE;
static uint8_t g_spi_rx_bounce[RZV_SPI_BOUNCE_LEN] RZV_SPI_DMA_ALIGN RZV_SPI_DMA_NONCACHE;

#if defined(TRANSFER_EVENT_COMPLETE)
#define RZV_SPI_DMA_DONE_EVENT TRANSFER_EVENT_COMPLETE
#elif defined(TRANSFER_EVENT_TRANSFER_END)
#define RZV_SPI_DMA_DONE_EVENT TRANSFER_EVENT_TRANSFER_END
#else
#define RZV_SPI_DMA_DONE_EVENT 0U
#endif

static void rzv_spi_reset_event_state(void)
{
	g_spi_event = RZV_SPI_EVENT_RESET;

	if (g_spi_event_sem != NULL) {
		while (xSemaphoreTake(g_spi_event_sem, 0) == pdTRUE) {
			/* drain semaphore */
		}
	}
}

void rzv_spi_imu_callback(spi_callback_args_t *p_args)
{
	BaseType_t higher_priority_task_woken = pdFALSE;

	if (p_args != NULL) {
		g_spi_event = p_args->event;

	} else {
		g_spi_event = SPI_EVENT_TRANSFER_ABORTED;
	}

	if (g_spi_event_sem != NULL) {
		(void)xSemaphoreGiveFromISR(g_spi_event_sem, &higher_priority_task_woken);
		portYIELD_FROM_ISR(higher_priority_task_woken);
	}
}

static fsp_err_t rzv_spi_wait_for_event(uint32_t timeout_us, spi_event_t *event_out)
{
	if (g_spi_event_sem == NULL) {
		return FSP_ERR_ASSERTION;
	}

	const uint64_t start_us = hrt_absolute_time();
	const uint64_t spin_until = start_us + 100U;
	const uint64_t deadline_us = start_us + (uint64_t)timeout_us;

	while (g_spi_event == RZV_SPI_EVENT_RESET) {
		if (hrt_absolute_time() > spin_until) {
			break;
		}
	}

	while (g_spi_event == RZV_SPI_EVENT_RESET) {
		const uint64_t now_us = hrt_absolute_time();

		if (now_us >= deadline_us) {
			return FSP_ERR_TIMEOUT;
		}

		uint64_t remaining_us = deadline_us - now_us;

		if (remaining_us == 0U) {
			remaining_us = 1U;
		}

		uint32_t wait_ms = (remaining_us > 1000U) ? (uint32_t)((remaining_us + 999U) / 1000U) : 1U;
		TickType_t wait_ticks = pdMS_TO_TICKS(wait_ms);

		if (wait_ticks == 0U) {
			wait_ticks = 1U;
		}

		(void)xSemaphoreTake(g_spi_event_sem, wait_ticks);
	}

	spi_event_t event = g_spi_event;
	rzv_spi_reset_event_state();

	if (event_out != NULL) {
		*event_out = event;
	}

	if (event == SPI_EVENT_TRANSFER_COMPLETE) {
		return FSP_SUCCESS;
	}

	if (event == SPI_EVENT_TRANSFER_ABORTED) {
		return FSP_ERR_TRANSFER_ABORTED;
	}

	return FSP_ERR_ASSERTION;
}

static void rzv_spi_copy_extend(rzv_spi_backend_t *handle)
{
	memcpy(&handle->extend,
	       (const spi_b_extended_cfg_t *)g_spi_imu_cfg.p_extend,
	       sizeof(spi_b_extended_cfg_t));
}

static int rzv_spi_reopen(rzv_spi_backend_t *handle)
{
	fsp_err_t err = FSP_SUCCESS;

	if (handle->opened) {
		err = R_SPI_B_Close(handle->ctrl);

		if ((err != FSP_SUCCESS) && (err != FSP_ERR_NOT_OPEN)) {
			PX4_ERR("R_SPI_B_Close failed (%d)", err);
			return -1;
		}

		handle->opened = false;
	}

	err = R_SPI_B_Open(handle->ctrl, &handle->config);

	if (err != FSP_SUCCESS) {
		PX4_ERR("R_SPI_B_Open failed (%d)", err);
		return -1;
	}

	err = R_SPI_B_CallbackSet(handle->ctrl, rzv_spi_imu_callback, NULL, NULL);

	if (err != FSP_SUCCESS) {
		PX4_ERR("R_SPI_B_CallbackSet failed (%d)", err);
		return -1;
	}

	handle->opened = true;
	rzv_spi_reset_event_state();
	return 0;
}

int rzv_spi_backend_init(rzv_spi_backend_t *handle, uint8_t bus)
{
	if ((handle == NULL) || (bus != g_spi_imu_cfg.channel)) {
		return -1;
	}

	/* Force a clean RSPI peripheral block before (re-)opening.
	 *
	 * On a WARM CR8 restart (Linux remoteproc dynamic reload, or a CA55 reboot that
	 * re-loads CR8 — the sensors stay powered) the FSP control block is fresh (CR8
	 * RAM is zeroed at reset), so R_SPI_B_Close() sees it as "not open" and no-ops —
	 * leaving the RSPI hardware block in whatever stale FIFO/DMA state the previous
	 * firmware instance left it in. The ICM-45688 then probes OK but its FIFO reads
	 * time out ("no valid data"), which looked like the sensor needing a physical
	 * power-cycle but was really the master-side peripheral. An explicit MODULE_STOP
	 * here (R_SPI_B_Open below re-STARTs the module) gives a clean block on every
	 * start. Harmless on cold boot (block is already off). Validated: 3/3 warm
	 * reloads read gravity on all three IMUs (vs accel TIMEOUT without this). */
	R_BSP_MODULE_STOP(FSP_IP_RSPI, g_spi_imu_cfg.channel);
	R_BSP_SoftwareDelay(500, BSP_DELAY_UNITS_MICROSECONDS);

	memset(handle, 0, sizeof(*handle));
	rzv_spi_reset_event_state();

	if (g_spi_mutex == NULL) {
		g_spi_mutex = xSemaphoreCreateMutexStatic(&g_spi_mutex_buffer);
	}

	if (g_spi_mutex == NULL) {
		PX4_ERR("SPI semaphore init failed");
		return -1;
	}

	if (g_spi_event_sem == NULL) {
		g_spi_event_sem = xSemaphoreCreateBinaryStatic(&g_spi_event_sem_buffer);

		if (g_spi_event_sem == NULL) {
			PX4_ERR("SPI event semaphore init failed");
			return -1;
		}

		/* ensure empty state */
		(void)xSemaphoreTake(g_spi_event_sem, 0);
	}

	handle->bus = bus;
	handle->ctrl = &g_spi_imu_ctrl;
	memcpy(&handle->config, &g_spi_imu_cfg, sizeof(spi_cfg_t));
	rzv_spi_copy_extend(handle);
	handle->config.p_extend = &handle->extend;
	handle->config.p_callback = rzv_spi_imu_callback;
	handle->current_speed_hz = 1000000U;

	/* Per-device SSL routing assumes every SSL line is active-low: R_SPI_B_Open
	 * only programs the SPCR3 polarity bit for the currently selected SSL, and
	 * with SPI_B_SSLP_LOW (= 0) the cleared SPCR3 nibble leaves all four lines
	 * active-low, so switching SPCMD0.SSLA never needs a polarity update. An
	 * active-high configuration would silently break the other lines, so fail
	 * closed here instead. */
	const spi_b_extended_cfg_t *gen_extend = (const spi_b_extended_cfg_t *)g_spi_imu_cfg.p_extend;

	if (gen_extend->ssl_polarity != SPI_B_SSLP_LOW) {
		PX4_ERR("SPI SSL routing requires active-low SSL polarity (got %d)",
			(int)gen_extend->ssl_polarity);
		return -1;
	}

	uint8_t initial_mode = 0U;
	if (handle->config.clk_polarity == SPI_CLK_POLARITY_HIGH) {
		initial_mode |= SPI_CPOL;
	}

	if (handle->config.clk_phase == SPI_CLK_PHASE_EDGE_EVEN) {
		initial_mode |= SPI_CPHA;
	}

	handle->current_mode = initial_mode;
	handle->bits_per_word = 8U;

	if (rzv_spi_reopen(handle) != 0) {
		return -1;
	}

	return 0;
}

/* Apply the per-transfer device configuration (mode/speed/bits/SSL).
 *
 * MUST be called with g_spi_mutex held: settings and the subsequent transfer
 * form one critical section, so devices sharing the channel can never
 * interleave a settings change with another device's transfer.
 *
 * SSL routing: the FSP r_spi_b driver only programs SPCMD0.SSLA at open time
 * (no runtime select API), so switching lines is either a direct SSLA RMW —
 * only when the peripheral is provably idle (SPCR.SPE == 0, the transfer-active
 * enable bit, and SPPSR == 0, the in-use guard FSP itself checks before
 * starting a transfer) — or a full reopen (fail-closed path). The invariant
 * `handle->extend.ssl_select == currently selected SSL` is maintained before
 * any reopen, so every reopen (mode/speed change, retry reset, failure
 * recovery) restores the correct line via config.p_extend.
 */
static int rzv_spi_apply_cfg_locked(rzv_spi_backend_t *handle, const rzv_spi_xfer_cfg_t *cfg)
{
	bool reopen_required = handle->needs_reopen;

	if ((cfg->bits_per_word != 8U) && (cfg->bits_per_word != 16U)) {
		PX4_ERR("SPI unsupported bits per word (%u)", (unsigned)cfg->bits_per_word);
		return -1;
	}

	handle->bits_per_word = cfg->bits_per_word;

	const uint8_t requested_mode = cfg->mode & (SPI_CPOL | SPI_CPHA);

	if (requested_mode != handle->current_mode) {
		uint32_t polarity = (requested_mode & SPI_CPOL) ? SPI_CLK_POLARITY_HIGH : SPI_CLK_POLARITY_LOW;
		uint32_t phase = (requested_mode & SPI_CPHA) ? SPI_CLK_PHASE_EDGE_EVEN : SPI_CLK_PHASE_EDGE_ODD;

		if ((handle->config.clk_polarity != polarity) || (handle->config.clk_phase != phase)) {
			handle->config.clk_polarity = polarity;
			handle->config.clk_phase = phase;
			reopen_required = true;
		}

		handle->current_mode = requested_mode;
	}

	if ((cfg->speed_hz != 0U) && (cfg->speed_hz != handle->current_speed_hz)) {
		rspck_div_setting_t bitrate;
		fsp_err_t err = R_SPI_B_CalculateBitrate(cfg->speed_hz, handle->extend.clock_source, &bitrate);

		if (err != FSP_SUCCESS) {
			PX4_ERR("R_SPI_B_CalculateBitrate(%lu) failed (%d)", (unsigned long)cfg->speed_hz, err);
			return -1;
		}

		handle->extend.spck_div = bitrate;
		handle->current_speed_hz = cfg->speed_hz;
		reopen_required = true;
	}

	if (cfg->ssl_index > (uint8_t)SPI_B_SSL_SELECT_SSL3) {
		PX4_ERR("SPI invalid SSL index (%u)", (unsigned)cfg->ssl_index);
		return -1;
	}

	const spi_b_ssl_select_t ssl = (spi_b_ssl_select_t)cfg->ssl_index;

	if (ssl != handle->extend.ssl_select) {
		handle->extend.ssl_select = ssl;

		if (!reopen_required) {
			R_SPI_B0_Type *p_regs = handle->ctrl->p_regs;
			const bool idle = ((p_regs->SPCR & R_SPI_B0_SPCR_SPE_Msk) == 0U)
					  && (p_regs->SPPSR == 0U);

			if (idle) {
				uint32_t spcmd0 = p_regs->SPCMD0;
				spcmd0 &= ~R_SPI_B0_SPCMD0_SSLA_Msk;
				spcmd0 |= ((uint32_t)ssl << R_SPI_B0_SPCMD0_SSLA_Pos) & R_SPI_B0_SPCMD0_SSLA_Msk;
				p_regs->SPCMD0 = spcmd0;

			} else {
				/* Peripheral busy or stuck (e.g. after an aborted transfer):
				 * never RMW command registers in that state. */
				reopen_required = true;
			}
		}
	}

	if (reopen_required) {
		if (rzv_spi_reopen(handle) != 0) {
			handle->needs_reopen = true;
			return -1;
		}

		handle->needs_reopen = false;
	}

	return 0;
}

static spi_bit_width_t rzv_spi_get_bit_width(const rzv_spi_backend_t *handle)
{
	return (handle->bits_per_word == 16U) ? SPI_BIT_WIDTH_16_BITS : SPI_BIT_WIDTH_8_BITS;
}

static void rzv_spi_copy_from_bounce(void *dest, const void *src, size_t byte_count)
{
	if ((dest == NULL) || (src == NULL) || (byte_count == 0U)) {
		return;
	}

	memcpy(dest, src, byte_count);
	R_BSP_CacheCleanRangeData(dest, (uint32_t)byte_count);
}

int rzv_spi_backend_transfer(rzv_spi_backend_t *handle, const rzv_spi_xfer_cfg_t *cfg,
			     const void *tx, void *rx, size_t length_bytes)
{
	if ((handle == NULL) || (cfg == NULL) || !handle->opened || (length_bytes == 0U)) {
		return -1;
	}

	/* Unconditional cap: even DMA-safe caller buffers are rejected above the
	 * bounce size, preserving the historical cdev-level guard semantics. */
	if (length_bytes > RZV_SPI_BOUNCE_LEN) {
		PX4_ERR("SPI transfer len %u exceeds max %u", (unsigned)length_bytes,
			(unsigned)RZV_SPI_BOUNCE_LEN);
		return -1;
	}

	if (g_spi_mutex == NULL) {
		return -1;
	}

	/* Increased timeout from 20ms to 100ms for high-load scenarios.
	 * Add retry mechanism: try twice before failing.
	 */
	int retry_count = 0;
	const int max_retries = 2;

	while (xSemaphoreTake(g_spi_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
		retry_count++;

		if (retry_count >= max_retries) {
			PX4_ERR("SPI mutex timeout after %d retries", retry_count);
			return -1;
		}

		PX4_DEBUG("SPI mutex retry %d/%d", retry_count, max_retries);
		vTaskDelay(pdMS_TO_TICKS(1));  /* Brief yield before retry */
	}

	/* Device settings (SSL/mode/speed/bits) are applied inside the same
	 * critical section as the transfer itself. */
	if (rzv_spi_apply_cfg_locked(handle, cfg) != 0) {
		xSemaphoreGive(g_spi_mutex);
		return -1;
	}

	const size_t byte_count = length_bytes;

	if ((handle->bits_per_word == 16U) && ((byte_count & 0x1U) != 0U)) {
		PX4_ERR("SPI 16-bit transfer requires even byte count (%u)", (unsigned)byte_count);
		xSemaphoreGive(g_spi_mutex);
		return -1;
	}

	const bool same_buffer = (tx != NULL) && (rx != NULL) && (tx == rx);
	const bool tx_dma_safe = (tx != NULL) ? rzv_dma_buffer_is_dma_safe(tx, byte_count) : false;
	const bool rx_dma_safe = (rx != NULL) ? rzv_dma_buffer_is_dma_safe(rx, byte_count) : false;
	const bool dma_len_aligned = ((byte_count & (RZV_SPI_DMA_WORD_ALIGN - 1U)) == 0U);
	const bool dma_size_ok = byte_count >= RZV_SPI_DMA_MIN_BYTES;

	bool use_tx_bounce = (tx == NULL) || !tx_dma_safe;
	bool use_rx_bounce = (rx == NULL) || !rx_dma_safe;

	if ((tx != NULL) && (((uintptr_t)tx & (RZV_SPI_DMA_WORD_ALIGN - 1U)) != 0U)) {
		use_tx_bounce = true;
	}

	if ((rx != NULL) && (((uintptr_t)rx & (RZV_SPI_DMA_WORD_ALIGN - 1U)) != 0U)) {
		use_rx_bounce = true;
	}

	if (!dma_size_ok || !dma_len_aligned) {
		if (tx != NULL) {
			use_tx_bounce = true;
		}

		if (rx != NULL) {
			use_rx_bounce = true;
		}
	}

	/* When the caller uses the same buffer for TX and RX we must go through the
	 * dedicated bounce buffers. The SPI peripheral performs TX and RX via
	 * independent DMA channels, and overlapping addresses would race and
	 * corrupt the command phase before it is clocked out. */
	if (same_buffer) {
		use_tx_bounce = true;
		use_rx_bounce = true;
	}

	/* byte_count <= RZV_SPI_BOUNCE_LEN is guaranteed by the unconditional
	 * cap at function entry, so the bounce buffers always fit. */

	const void *tx_ptr = tx;
	void *rx_ptr = rx;

	if (use_tx_bounce) {
		if (tx != NULL) {
			memcpy(g_spi_tx_bounce, tx, byte_count);

		} else {
			memset(g_spi_tx_bounce, 0, byte_count);
		}

		tx_ptr = g_spi_tx_bounce;
	}

	if (use_rx_bounce) {
		rx_ptr = g_spi_rx_bounce;
	}

	const spi_bit_width_t bit_width = rzv_spi_get_bit_width(handle);
	const size_t word_count = (bit_width == SPI_BIT_WIDTH_16_BITS) ? (byte_count / 2U) : byte_count;
	fsp_err_t err = FSP_ERR_TIMEOUT;
	spi_event_t event = RZV_SPI_EVENT_RESET;

	for (int attempt = 0; attempt <= RZV_SPI_TRANSFER_MAX_RETRIES; attempt++) {
		if (attempt > 0) {
			/* Reset SPI peripheral to clear any stuck FIFO / state machine after
			 * a timeout or abort before retrying the transfer. */
			if (rzv_spi_reopen(handle) != 0) {
				PX4_ERR("SPI reopen failed on retry %d", attempt);
				break;
			}

			PX4_DEBUG("SPI retry %d/%d len=%u", attempt, RZV_SPI_TRANSFER_MAX_RETRIES,
				  (unsigned)byte_count);
		}

		if (tx_ptr != NULL) {
			R_BSP_CacheCleanRangeData((void *)tx_ptr, (uint32_t)byte_count);
		}

		if (rx_ptr != NULL) {
			/* Ensure no dirty cache lines in destination before DMA starts. */
			R_BSP_CacheInvalidateRangeData(rx_ptr, (uint32_t)byte_count);
		}

		rzv_spi_reset_event_state();
		event = RZV_SPI_EVENT_RESET;

		err = R_SPI_B_WriteRead(handle->ctrl, tx_ptr, rx_ptr, word_count, bit_width);

		if (err == FSP_SUCCESS) {
			err = rzv_spi_wait_for_event(RZV_SPI_TRANSFER_TIMEOUT_US, &event);
		}

		if (err == FSP_SUCCESS) {
			/* Invalidate cache AFTER DMA completes so CPU observes freshly written data. */
			if (rx_ptr != NULL) {
				R_BSP_CacheInvalidateRangeData(rx_ptr, (uint32_t)byte_count);
			}

			if ((rx != NULL) && (rx_ptr == g_spi_rx_bounce)) {
				rzv_spi_copy_from_bounce(rx, g_spi_rx_bounce, byte_count);
			}

			xSemaphoreGive(g_spi_mutex);
			return 0;
		}

		PX4_ERR("SPI transfer error (%d) len=%u event=%d attempt=%d/%d",
			err, (unsigned)byte_count, (int)event,
			attempt + 1, RZV_SPI_TRANSFER_MAX_RETRIES + 1);
	}

	PX4_ERR("  tx=%p rx=%p tx_bounce=%p rx_bounce=%p", tx_ptr, rx_ptr, g_spi_tx_bounce, g_spi_rx_bounce);

	/* Never hand a possibly-stuck peripheral to the next caller: reset it now,
	 * or mark it so the next transfer reopens before touching registers. */
	if (rzv_spi_reopen(handle) != 0) {
		handle->needs_reopen = true;
	}

	xSemaphoreGive(g_spi_mutex);
	return -1;
}

void spi_master_callback(spi_callback_args_t *p_args)
{
	rzv_spi_imu_callback(p_args);
}

#endif /* __PX4_FREERTOS */
