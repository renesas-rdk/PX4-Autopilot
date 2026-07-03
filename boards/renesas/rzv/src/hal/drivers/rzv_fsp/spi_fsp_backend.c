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
#include "posix_compat/spi/spidev.h"

/* PX4 drives the IMU SPI channel in pure interrupt/FIFO mode: the DMAC
 * transfer instances that hal_data.c may attach to g_spi_imu_cfg are stripped
 * in rzv_spi_backend_init().  Rationale for this choice:
 *  - the SPI0_RXI->DMAC->DMAINT->CE completion linkage occasionally drops
 *    under AXI/DDR contention -> "SPI transfer error (20) ... event=0";
 *  - per-transfer DMAC reconfiguration costs more CPU than the FIFO-drain
 *    ISRs at these transfer sizes (wq:SPI0 10.83% DMA vs 9.40% interrupt);
 *  - interrupt mode needs no bounce buffers or cache maintenance: the CPU
 *    reads/writes the caller's buffers directly, and SCKASE (master SCK
 *    auto-stop) guarantees no RX overrun.
 * The DMAC capability itself stays at the platform/e2studio level for other
 * products; PX4 simply does not use it. */
/* Upper bound on a single transfer, matching the historical cdev guard. */
#define RZV_SPI_MAX_TRANSFER_LEN 1024U
#define RZV_SPI_TRANSFER_TIMEOUT_US 20000U
/* Number of retries after a transfer error (timeout or abort).
 * After each failure the SPI peripheral is reset via rzv_spi_reopen() to
 * clear any stuck state before the next attempt.  Two retries bring the
 * double-failure probability from 0.17 err/s to ~0.03 err/s at idle and
 * correspondingly reduce motor-EMI-induced errors during flight. */
#define RZV_SPI_TRANSFER_MAX_RETRIES 2
#define RZV_SPI_EVENT_RESET ((spi_event_t)0)

static StaticSemaphore_t g_spi_mutex_buffer;
static SemaphoreHandle_t g_spi_mutex;
static StaticSemaphore_t g_spi_event_sem_buffer;
static SemaphoreHandle_t g_spi_event_sem;

static volatile spi_event_t g_spi_event = RZV_SPI_EVENT_RESET;

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

	/* Interrupt/FIFO mode: R_SPI_B_Open() skips the DMAC transfer instances
	 * entirely when both pointers are NULL (see the rationale at the top of
	 * this file).  The platform-level DMAC config stays untouched. */
	handle->config.p_transfer_tx = NULL;
	handle->config.p_transfer_rx = NULL;

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

int rzv_spi_backend_transfer(rzv_spi_backend_t *handle, const rzv_spi_xfer_cfg_t *cfg,
			     const void *tx, void *rx, size_t length_bytes)
{
	if ((handle == NULL) || (cfg == NULL) || !handle->opened || (length_bytes == 0U)) {
		return -1;
	}

	if (length_bytes > RZV_SPI_MAX_TRANSFER_LEN) {
		PX4_ERR("SPI transfer len %u exceeds max %u", (unsigned)length_bytes,
			(unsigned)RZV_SPI_MAX_TRANSFER_LEN);
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

	/* Interrupt/FIFO mode moves every byte with the CPU (r_spi_b_transmit /
	 * r_spi_b_receive), so the caller's buffers are used directly: no DMA
	 * coherency concerns, no bounce copies, no cache maintenance.  A shared
	 * tx==rx buffer is safe too — byte i is always consumed from the TX
	 * buffer before the received byte i is stored (TX leads RX through the
	 * shift register), unlike the racing dual-DMA-channel case.  NULL tx
	 * (send zeros) and NULL rx (discard) are handled natively by r_spi_b. */
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

		rzv_spi_reset_event_state();
		event = RZV_SPI_EVENT_RESET;

		err = R_SPI_B_WriteRead(handle->ctrl, tx, rx, word_count, bit_width);

		if (err == FSP_SUCCESS) {
			err = rzv_spi_wait_for_event(RZV_SPI_TRANSFER_TIMEOUT_US, &event);
		}

		if (err == FSP_SUCCESS) {
			xSemaphoreGive(g_spi_mutex);
			return 0;
		}

		PX4_ERR("SPI transfer error (%d) len=%u event=%d attempt=%d/%d",
			err, (unsigned)byte_count, (int)event,
			attempt + 1, RZV_SPI_TRANSFER_MAX_RETRIES + 1);
	}

	PX4_ERR("  tx=%p rx=%p", tx, rx);

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
