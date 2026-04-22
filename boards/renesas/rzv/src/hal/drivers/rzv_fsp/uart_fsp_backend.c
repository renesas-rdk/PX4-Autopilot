/****************************************************************************
 *
 * Copyright (c) 2025 Renesas Electronics Corporation and/or its affiliates
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 ****************************************************************************/


#if defined(__PX4_FREERTOS)

#include "rzv_fsp/uart_fsp_backend.h"
#define MODULE_NAME "rzv_uart_backend"

#include <px4_platform_common/log.h>
#include <px4_platform_common/time.h>
#include <errno.h>

#include "bsp_api.h"
#include "rzv_fsp/dma_buffer.h"
#include "rzv_fsp/uart_channel_map.h"
#include "rzv_fsp/uart_rc_ringbuffer.h"

#if defined(RZV_MAVLINK_USE_OPENAMP) && (RZV_MAVLINK_USE_OPENAMP == 1)
#define RZV_MAVLINK_OPENAMP_ENABLED 1
#else
#define RZV_MAVLINK_OPENAMP_ENABLED 0
#endif

#if RZV_MAVLINK_OPENAMP_ENABLED
#include "uart_openamp_backend.h"
#endif

#include <FreeRTOS.h>
#include <semphr.h>
#include <task.h>

#include <string.h>
#include <stdint.h>

/* TX DMA timeout: must cover worst-case = 512 B @ 57600 baud ≈ 89 ms.
 * 200 ms gives 2× margin without affecting IMU SPI (DMA completes in < 1 ms). */
#define RZV_UART_TRANSFER_TIMEOUT_MS 200U
#define RZV_UART_MAX_CHANNELS        4U
#define RZV_UART_BOUNCE_LEN          512U
#define RZV_UART_DMA_MIN_BYTES       16U
#define RZV_UART_DMA_WORD_ALIGN      4U
#define RZV_UART_DMA_RX_BLOCK_LEN_DEFAULT 32U
#ifndef RZV_UART_STATS_ENABLE
#define RZV_UART_STATS_ENABLE 0
#endif

/* Adaptive DMA block sizes per channel (optimized for traffic patterns) */
#define RZV_UART_DMA_BLOCK_RC     16U   /* RC: Low-rate, fixed 25-byte SBUS frames */
#define RZV_UART_DMA_BLOCK_MAVLINK 128U  /* MAVLink: High-rate, variable messages */
#define RZV_UART_DMA_BLOCK_GPS     32U   /* GPS: Medium-rate NMEA/UBX */
#define RZV_UART_DMA_BLOCK_LIDAR   128U  /* LIDAR: Increase to reduce DMA completion ISR overhead */

/* DMA watchdog configuration */
#define RZV_UART_DMA_WATCHDOG_INTERVAL_MS 100U  /* Check every 100ms */
#define RZV_UART_DMA_IDLE_TIMEOUT_MS      500U  /* Restart if idle >500ms */

/* Logical Channel Definitions */
#define RZV_UART_CH_RC      0U
#define RZV_UART_CH_MAVLINK 1U
#define RZV_UART_CH_GPS     2U
#define RZV_UART_CH_LIDAR   3U

#if defined(__GNUC__)
#define RZV_UART_DMA_ALIGN __attribute__((aligned(32)))
#define RZV_UART_DMA_NONCACHE __attribute__((section(".noncache_buffer")))
#else
#define RZV_UART_DMA_ALIGN
#define RZV_UART_DMA_NONCACHE
#endif

static uint8_t g_uart_tx_bounce[RZV_UART_MAX_CHANNELS][RZV_UART_BOUNCE_LEN] RZV_UART_DMA_ALIGN RZV_UART_DMA_NONCACHE;
static uint8_t g_uart_rx_bounce[RZV_UART_MAX_CHANNELS][RZV_UART_BOUNCE_LEN] RZV_UART_DMA_ALIGN RZV_UART_DMA_NONCACHE;

/* Static instance tracking for callback lookup */
static rzv_uart_backend_t *g_uart_backend_instances[RZV_UART_MAX_CHANNELS] = {NULL, NULL, NULL, NULL};

extern sci_b_baud_setting_t g_uart_rc_baud_setting;
extern sci_b_baud_setting_t g_uart_mav_baud_setting;
extern sci_b_baud_setting_t g_uart_gps_baud_setting;
extern sci_b_baud_setting_t g_uart_lidar_baud_setting;

static rzv_uart_backend_t *get_handle_from_channel(uint32_t fsp_channel)
{
	/* FSP channel mapping:
	 * - UART1 (RC): FSP channel 7 → logical 0
	 * - UART3 (MAVLink): FSP channel 0 → logical 1
	 */
	for (uint8_t i = 0; i < RZV_UART_MAX_CHANNELS; i++) {
		if (g_uart_backend_instances[i] != NULL && 
		    g_uart_backend_instances[i]->config.channel == fsp_channel) {
			return g_uart_backend_instances[i];
		}
	}
	return NULL;
}

static inline size_t rzv_uart_effective_dma_block_len(rzv_uart_backend_t *handle)
{
	size_t block = handle->dma_rx_block_len;
	size_t min_chunk = handle->dma_rx_min_chunk;

	if ((block == 0U) || (block > handle->bounce_len)) {
		block = handle->bounce_len;
	}

	if (min_chunk == 0U) {
		min_chunk = RZV_UART_DMA_MIN_BYTES;
	}

	if (min_chunk < 1U) {
		min_chunk = 1U;
	}

	if (block < min_chunk) {
		block = min_chunk;
	}

	return block;
}

static void rzv_uart_dma_start_rx(rzv_uart_backend_t *handle)
{
	if ((handle == NULL) || !handle->dma_rx_enabled || (handle->rx_bounce == NULL)) {
		if (handle != NULL) {
			handle->dma_rx_active = false;
		}

		return;
	}

	const size_t block = rzv_uart_effective_dma_block_len(handle);
	handle->dma_rx_block_len = block;

	R_BSP_CacheInvalidateRangeData(handle->rx_bounce, (uint32_t)block);
	fsp_err_t err = R_SCI_B_UART_Read(handle->ctrl, handle->rx_bounce, (uint32_t)block);

	if (err == FSP_SUCCESS) {
		handle->dma_rx_active = true;
	} else {
		handle->dma_rx_active = false;
		handle->dma_rx_restart_failures++;
	}
}

/**
 * @brief Process deferred DMA restart request from ISR
 *
 * This function MUST be called from task context, NOT from ISR.
 * It handles DMA restart requests that were deferred from the ISR callback
 * to avoid calling blocking HAL APIs in interrupt context.
 *
 * @param handle UART backend handle
 *
 * IMPORTANT: This function is called automatically by rzv_uart_backend_read()
 * when data is available. For channels that don't use the ring buffer read API,
 * a dedicated task may be needed to periodically check and process restarts.
 */
static inline void rzv_uart_process_deferred_restart(rzv_uart_backend_t *handle)
{
	if ((handle != NULL) && handle->dma_rx_needs_restart && handle->dma_rx_enabled) {
		/* Clear flag BEFORE restarting to avoid race condition */
		handle->dma_rx_needs_restart = false;

		/* This call is safe here because in task context, not ISR */
		rzv_uart_dma_start_rx(handle);
	}
}


static void rzv_uart_callback(uart_callback_args_t *p_args)
{
	if (p_args == NULL) {
		return;
	}

	/* Get handle from context if available, otherwise lookup by FSP channel */
	rzv_uart_backend_t *handle = (rzv_uart_backend_t *)p_args->p_context;
	if (handle == NULL) {
		/* Context is NULL (FSP config issue in hal_data.c), try to lookup by channel */
		handle = get_handle_from_channel(p_args->channel);
		if (handle == NULL) {
			return;  /* Cannot identify channel */
		}
	}
	BaseType_t higher_priority_task_woken = pdFALSE;

	/* CRITICAL: NO PX4_ERR/PX4_INFO in ISR context!
	 * Those functions call orb_publish() which tries to take semaphores
	 * → FreeRTOS assertion failure → system hang
	 * 
	 * Only track counters, no logging in ISR
	 */
	
	uint8_t ch = handle->channel;
	if (ch < RZV_UART_MAX_CHANNELS) {
		handle->event_count++;
		
		if ((p_args->event & UART_EVENT_RX_CHAR) != 0U) {
			if ((handle->channel == RZV_UART_CH_RC) || !handle->dma_rx_enabled) {
				handle->rx_char_count++;
			}
		}
		
		if ((p_args->event & (UART_EVENT_ERR_OVERFLOW | UART_EVENT_ERR_FRAMING | 
		                      UART_EVENT_ERR_PARITY | UART_EVENT_BREAK_DETECT)) != 0U) {
			handle->error_count++;
			if ((p_args->event & UART_EVENT_ERR_OVERFLOW) != 0U) {
				handle->overflow_count++;
			}
		}
	}

	/* Handle byte-by-byte RX for any channel without DMA support */
	if ((p_args->event & UART_EVENT_RX_CHAR) != 0U) {
		if (!handle->dma_rx_enabled) {
			uint8_t byte = (uint8_t)(p_args->data & 0xFFU);

			/* Ring buffer channels (GPS, MAVLink, LIDAR, RC) never call R_SCI_B_UART_Read(),
			 * so rx_dest_bytes is always 0 — that is NORMAL, not an error.
			 * Only flag a restart for non-ring-buffer ISR channels where an active
			 * R_SCI_B_UART_Read() bounce-buffer transfer was expected to be running.
			 */
			if (!handle->use_ring_buffer && (handle->ctrl->rx_dest_bytes == 0U)) {
				handle->dma_rx_active = false;
				handle->dma_rx_needs_restart = true;

				if (handle->rx_sem != NULL) {
					xSemaphoreGiveFromISR(handle->rx_sem, &higher_priority_task_woken);
				}
			}

			uart_rc_buffer_write_from_isr(handle->channel, &byte, 1U);
			/* DO NOT call portYIELD here - let it complete quickly */
			return;
		}
	}

	/* Handle overflow error - clear by reading status */
	if ((p_args->event & UART_EVENT_ERR_OVERFLOW) != 0U) {
		/* Overflow errors are automatically cleared by FSP driver.
		 * If we are using DMA, the transfer might have been aborted or stalled.
		 * We need to ensure it restarts.
		 */

		/* Track overflow for diagnostics (checked outside ISR) */
		handle->hw_overflow_detected = true;

		if (handle->dma_rx_enabled) {
			/* Check if driver thinks transfer is in progress */
			bool driver_active = (handle->ctrl->rx_transfer_in_progress != 0U);

			/* Only restart if driver is NOT active.
			 * If driver is active, restarting will cause data loss (resetting buffer).
			 * If driver is inactive (aborted by overflow), we MUST restart.
			 */
			if (!driver_active) {
				handle->dma_rx_active = false;
				handle->dma_rx_needs_restart = true;

				/* Signal task to handle restart in task context */
				if (handle->rx_sem != NULL) {
					xSemaphoreGiveFromISR(handle->rx_sem, &higher_priority_task_woken);
				}
			}
		}
		/* Do NOT return here, check for other events */
	}

	/* DMA completion events for channels using DMA (MAVLink channel 1) */
	if ((p_args->event & UART_EVENT_RX_COMPLETE) != 0U) {
		if (handle->dma_rx_enabled) {
			const size_t block = rzv_uart_effective_dma_block_len(handle);

			if ((block > 0U) && (handle->rx_bounce != NULL)) {
				handle->dma_rx_active = false;
				R_BSP_CacheInvalidateRangeData(handle->rx_bounce, (uint32_t)block);

				if (handle->use_ring_buffer) {
					uart_rc_buffer_write_from_isr(handle->channel, handle->rx_bounce, (uint32_t)block);
					handle->rx_char_count += (uint32_t)block;
				}

				handle->dma_rx_needs_restart = true;
			}
		}

		if (handle->rx_sem != NULL) {
			xSemaphoreGiveFromISR(handle->rx_sem, &higher_priority_task_woken);
		}
	}

	if ((p_args->event & UART_EVENT_TX_COMPLETE) != 0U) {
		if (handle->tx_sem != NULL) {
			xSemaphoreGiveFromISR(handle->tx_sem, &higher_priority_task_woken);
		}
	}

	/* Yield to higher priority task if woken.
	 * Note: Previously RC channel was excluded to "avoid nested interrupt issues".
	 * This was incorrect - portYIELD_FROM_ISR is safe in nested interrupts as long
	 * as interrupt priorities are configured below configMAX_SYSCALL_INTERRUPT_PRIORITY.
	 * Skipping yield for RC caused SBUS frame processing delays.
	 * The real fix for nested interrupt safety is proper px4_in_isr() detection
	 * (which now covers all ARM exception modes) and correct interrupt priorities.
	 */
	if (higher_priority_task_woken != pdFALSE) {
		portYIELD_FROM_ISR(higher_priority_task_woken);
	}
}

typedef struct {
	uint8_t logical_id;
	uart_instance_t const *instance;
	sci_b_uart_instance_ctrl_t *ctrl;
	uart_cfg_t const *cfg;
	sci_b_uart_extended_cfg_t const *extend;
	sci_b_baud_setting_t *baud_setting;
} rzv_uart_hw_map_t;

static const rzv_uart_hw_map_t g_uart_hw_map[] = {
	{RZV_UART_CH_RC, &g_uart_rc, &g_uart_rc_ctrl, &g_uart_rc_cfg, &g_uart_rc_cfg_extend, &g_uart_rc_baud_setting},
	{RZV_UART_CH_MAVLINK, &g_uart_mav, &g_uart_mav_ctrl, &g_uart_mav_cfg, &g_uart_mav_cfg_extend, &g_uart_mav_baud_setting},
	{RZV_UART_CH_GPS, &g_uart_gps, &g_uart_gps_ctrl, &g_uart_gps_cfg, &g_uart_gps_cfg_extend, &g_uart_gps_baud_setting},
	{RZV_UART_CH_LIDAR, &g_uart_lidar, &g_uart_lidar_ctrl, &g_uart_lidar_cfg, &g_uart_lidar_cfg_extend, &g_uart_lidar_baud_setting},
};

static const rzv_uart_hw_map_t *rzv_uart_find(uint8_t logical_id)
{
	for (size_t i = 0; i < (sizeof(g_uart_hw_map) / sizeof(g_uart_hw_map[0])); i++) {
		if (g_uart_hw_map[i].logical_id == logical_id) {
			return &g_uart_hw_map[i];
		}
	}

	return NULL;
}

int rzv_uart_backend_init(rzv_uart_backend_t *handle, uint8_t logical_channel)
{
	/* MAVLink OpenAMP routing (when CA55 enabled) */
#if RZV_MAVLINK_OPENAMP_ENABLED
	if (logical_channel == RZV_UART_CH_MAVLINK) {
		PX4_INFO("MAVLink: Routing via OpenAMP (CA55 enabled)");
		return rzv_uart_openamp_init(handle, logical_channel);
	}
#endif

	if (handle == NULL) {
		return -1;
	}

	const rzv_uart_hw_map_t *map = rzv_uart_find(logical_channel);

	if (map == NULL) {
		PX4_ERR("UART logical channel %u not available", (unsigned)logical_channel);
		return -1;
	}

	if (logical_channel >= RZV_UART_MAX_CHANNELS) {
		PX4_ERR("UART logical channel %u out of bounce range", (unsigned)logical_channel);
		return -1;
	}

	memset(handle, 0, sizeof(*handle));

	handle->ctrl = map->ctrl;
	memcpy(&handle->config, map->cfg, sizeof(uart_cfg_t));
	memcpy(&handle->extend, map->extend, sizeof(sci_b_uart_extended_cfg_t));
	handle->tx_bounce = g_uart_tx_bounce[logical_channel];
	handle->rx_bounce = g_uart_rx_bounce[logical_channel];
	handle->bounce_len = RZV_UART_BOUNCE_LEN;
	handle->channel = logical_channel;
	handle->dma_tx_enabled = (map->cfg->p_transfer_tx != NULL);
	handle->dma_rx_enabled = (map->cfg->p_transfer_rx != NULL);
	handle->dma_rx_block_len = 0U;
	handle->dma_rx_min_chunk = RZV_UART_DMA_MIN_BYTES;

	/* Disable DMA for MAVLink and GPS to ensure stability (use ISR) */
	if ((logical_channel == RZV_UART_CH_MAVLINK) ||
	    (logical_channel == RZV_UART_CH_GPS)) {
		handle->dma_rx_enabled = false;
	}

	handle->dma_rx_active = false;
	handle->dma_rx_restart_failures = 0U;
	handle->dma_rx_needs_restart = false;
	handle->isr_rx_started = false;  /* ISR RX will be started on first read */
	handle->use_ring_buffer = true; /* Always use ring buffer for async RX */

	/* Initialize interrupt statistics counters */
	handle->event_count = 0;
	handle->rx_char_count = 0;
	handle->overflow_count = 0;
	handle->error_count = 0;

	/* Initialize overflow detection and DMA watchdog */
	handle->hw_overflow_detected = false;
	handle->last_read_time_us = hrt_absolute_time();
	handle->last_dma_rx_time_us = hrt_absolute_time();
	handle->last_overflow_warn_us = 0;  /* Allow first warning immediately */

	if (map->baud_setting != NULL) {
		handle->baud_setting = *map->baud_setting;
		/* Calculate actual baud rate from BRR register value
		 * Formula: baud_rate ≈ PCLK / ((BRR + 1) × 64)
		 * For 400MHz PCLK: 
		 *   BRR=61  → 100,806 baud (RC/SBUS)
		 *   BRR=107 → 57,870 baud (MAVLink)
		 */
		uint32_t brr = handle->baud_setting.baudrate_bits_b.brr;
		uint32_t pclk = 400000000U; // 400MHz for RZ/V2H
		handle->baudrate = pclk / ((brr + 1U) * 64U);
	} else {
		memset(&handle->baud_setting, 0, sizeof(handle->baud_setting));
		handle->baudrate = 115200U; // fallback default
	}

	handle->extend.p_baud_setting = &handle->baud_setting;
	handle->config.p_extend = &handle->extend;
	handle->config.p_callback = rzv_uart_callback;
	handle->config.p_context = handle;

	PX4_DEBUG("UART ch%u: baud=%u, rx_fifo_trigger=%u, start_edge=%u",
		  (unsigned)logical_channel,
		  (unsigned)handle->baudrate,
		  (unsigned)handle->extend.rx_fifo_trigger,
		  (unsigned)handle->extend.rx_edge_start);

	handle->rx_sem = xSemaphoreCreateBinaryStatic(&handle->rx_sem_buffer);
	handle->tx_sem = xSemaphoreCreateBinaryStatic(&handle->tx_sem_buffer);
	handle->mutex = xSemaphoreCreateMutexStatic(&handle->mutex_buffer);

	if ((handle->rx_sem == NULL) || (handle->tx_sem == NULL) || (handle->mutex == NULL)) {
		PX4_ERR("UART semaphore initialization failed");
		return -1;
	}

	fsp_err_t err = R_SCI_B_UART_Open(handle->ctrl, &handle->config);

	if (err != FSP_SUCCESS) {
		PX4_ERR("R_SCI_B_UART_Open failed (%d)", err);
		return -1;
	}

	err = R_SCI_B_UART_CallbackSet(handle->ctrl, rzv_uart_callback, handle, NULL);

	if (err != FSP_SUCCESS) {
		PX4_ERR("R_SCI_B_UART_CallbackSet failed (%d)", err);
		R_SCI_B_UART_Close(handle->ctrl);
		return -1;
	}

	/* Register instance for callback lookup */
	if (logical_channel < RZV_UART_MAX_CHANNELS) {
		g_uart_backend_instances[logical_channel] = handle;
	}

	/* Initialize ring buffer for all UART consumers */
	if (handle->use_ring_buffer) {
		PX4_DEBUG("UART ch%u: Initializing ring buffer...", (unsigned)logical_channel);
		uart_rc_buffer_init_channel(logical_channel);
		/* Verify ring buffer was initialized */
		int avail = uart_rc_buffer_available_internal(logical_channel);
		PX4_DEBUG("UART ch%u: Ring buffer available=%d (should be 0)", (unsigned)logical_channel, avail);

		/* Adaptive DMA block sizes optimized per channel traffic pattern */
		switch (logical_channel) {
		case RZV_UART_CH_RC:
			handle->dma_rx_block_len = RZV_UART_DMA_BLOCK_RC;  /* 16 bytes: low-rate SBUS */
			break;

		case RZV_UART_CH_MAVLINK:
			handle->dma_rx_block_len = RZV_UART_DMA_BLOCK_MAVLINK;  /* 128 bytes: high-rate */
			break;

		case RZV_UART_CH_GPS:
			handle->dma_rx_block_len = RZV_UART_DMA_BLOCK_GPS;  /* 32 bytes: medium-rate */
			break;

		case RZV_UART_CH_LIDAR:
			handle->dma_rx_block_len = RZV_UART_DMA_BLOCK_LIDAR;  /* 32 bytes: medium-rate */
			break;

		default:
			handle->dma_rx_block_len = RZV_UART_DMA_RX_BLOCK_LEN_DEFAULT;  /* 32 bytes fallback */
			break;
		}

		handle->dma_rx_restart_failures = 0U;
		handle->dma_rx_active = false;

		/* Only attempt to start DMAC if it was enabled in configuration */
		if (handle->dma_rx_enabled) {
			rzv_uart_dma_start_rx(handle);

			if (handle->dma_rx_active) {
				/* Only log at DEBUG level to avoid flooding console on resets */
				PX4_DEBUG("UART (ch%u): DMAC RX enabled (%u-byte blocks)",
							(unsigned)logical_channel,
							(unsigned)handle->dma_rx_block_len);
			} else {
				PX4_WARN("UART (ch%u): DMAC RX start failed, using interrupt fallback",
							(unsigned)logical_channel);
				handle->dma_rx_enabled = false;
				handle->dma_rx_block_len = 0U;
			}
		} else {
			/* DMAC intentionally disabled - use ISR mode */
			PX4_DEBUG("UART (ch%u): Using interrupt-based RX (DMAC disabled)",
						(unsigned)logical_channel);
			handle->dma_rx_block_len = 0U;

			/* ISR RX will be started on first read from task context */
			PX4_DEBUG("UART (ch%u): ISR RX will start on first read",
				  (unsigned)logical_channel);
		}

		/* DEBUG: Print ring buffer state after initialization */
#ifdef CONFIG_RZV_UART_DEBUG
		uart_rc_buffer_debug_state(logical_channel);
#endif
	}

	handle->opened = true;

	/* DEBUG: Log LIDAR channel initialization */
	if (logical_channel == RZV_UART_CH_LIDAR) {
		PX4_DEBUG("UART LIDAR (ch%u): Initialized - DMA=%s, baudrate=%u, ring_buffer=enabled",
			 (unsigned)logical_channel,
			 handle->dma_rx_enabled ? "enabled" : "ISR-only",
			 (unsigned)handle->baudrate);
	}

	return 0;
}

int rzv_uart_backend_configure(rzv_uart_backend_t *handle, uint32_t baudrate)
{
	if ((handle == NULL) || !handle->opened || (baudrate == 0U)) {
		return -1;
	}

	sci_b_baud_setting_t settings;
	fsp_err_t err = R_SCI_B_UART_BaudCalculate(baudrate, true, 5000U, &settings);

	if (err != FSP_SUCCESS) {
		PX4_ERR("UART baud calculate failed (%d) for %lu", err, (unsigned long)baudrate);
		return -1;
	}

	err = R_SCI_B_UART_BaudSet(handle->ctrl, &settings);

	if (err != FSP_SUCCESS) {
		PX4_ERR("UART baud set failed (%d)", err);
		return -1;
	}

	handle->baud_setting = settings;
	handle->baudrate = baudrate;

	return 0;
}

int rzv_uart_backend_read(rzv_uart_backend_t *handle, uint8_t *buffer, size_t length)
{
	/* OpenAMP backend dispatch */
#if RZV_MAVLINK_OPENAMP_ENABLED
	if (handle->channel == RZV_UART_CH_MAVLINK && handle->opened) {
		return rzv_uart_openamp_read(handle, buffer, length);
	}
#endif

	if ((handle == NULL) || (buffer == NULL) || (length == 0U) || !handle->opened) {
		return -1;
	}

	/* Check for hardware overflow and alert user (throttled to 1/10sec) */
	if (handle->hw_overflow_detected) {
		handle->hw_overflow_detected = false;  /* Clear flag */

		const uint64_t now_us = hrt_absolute_time();
		const uint64_t throttle_interval_us = 10000000ULL;  /* 10 seconds (was 5) */

		/* Throttle warnings to avoid log spam and system load */
		if ((now_us - handle->last_overflow_warn_us) >= throttle_interval_us) {
			handle->last_overflow_warn_us = now_us;

			/* Get ring buffer overflow count for detailed diagnostics */
			uint32_t ring_overflow = 0;
			uart_rc_buffer_get_stats(handle->channel, NULL, &ring_overflow);

			/* Use DEBUG instead of WARN to reduce orb_publish load */
			PX4_DEBUG("UART[%d] overflow! HW FIFO full, ring buffer overflow=%u",
				  (int)handle->channel, (unsigned)ring_overflow);
		}
	}

	/* Process any deferred DMA restart requests from ISR
	 * This is safe here because in task context, not ISR.
	 * The ISR only sets the flag; we do the actual restart here.
	 */
	rzv_uart_process_deferred_restart(handle);

	/* Update last read timestamp for DMA watchdog */
	handle->last_read_time_us = hrt_absolute_time();

	/* Channels using ring buffer (MAV, GPS, LIDAR) deliver data through ISR-fed ring buffer.
	 * UART_EVENT_RX_CHAR fires automatically for every received byte when no R_SCI_B_UART_Read()
	 * transfer is active. Do NOT call R_SCI_B_UART_Read() here — doing so redirects bytes into
	 * the bounce buffer instead of the ring buffer, causing UART_EVENT_RX_CHAR to stop firing
	 * until the bounce buffer fills (~44ms at 115200), starving all ring-buffer readers (GPS, MAVLink).
	 */
	if (handle->use_ring_buffer) {
		int available = uart_rc_buffer_available_internal(handle->channel);

		size_t to_read = 0;
		int actual = 0;

		if (available > 0) {
			to_read = (size_t)available;

			if (to_read > length) {
				to_read = length;
			}

			actual = uart_rc_buffer_read_internal(handle->channel, buffer, (int)to_read);
		}

		return (actual > 0) ? actual : 0;
	}

	if ((handle->rx_bounce == NULL) || (handle->bounce_len == 0U)) {
		return -1;
	}

	if (xSemaphoreTake(handle->mutex, pdMS_TO_TICKS(RZV_UART_TRANSFER_TIMEOUT_MS)) != pdTRUE) {
		return -1;
	}

	size_t remaining = length;
	size_t offset = 0U;
	bool timeout_detected = false;

	while (remaining > 0U) {
		size_t chunk = remaining;

		while (xSemaphoreTake(handle->rx_sem, 0) == pdTRUE) {
			/* drain */
		}

		uint8_t *target = &buffer[offset];
		uint8_t *rx_ptr = target;
		bool use_bounce = false;

		if (handle->dma_rx_enabled) {
			const bool dma_safe = rzv_dma_buffer_is_dma_safe(target, chunk);
			const bool aligned = (((uintptr_t)target & (RZV_UART_DMA_WORD_ALIGN - 1U)) == 0U);
			const bool size_ok = chunk >= RZV_UART_DMA_MIN_BYTES;

			use_bounce = !dma_safe || !aligned || !size_ok;

			if (use_bounce && (chunk > handle->bounce_len)) {
				chunk = handle->bounce_len;
			}

			rx_ptr = use_bounce ? handle->rx_bounce : target;

			R_BSP_CacheInvalidateRangeData(rx_ptr, (uint32_t)chunk);

		} else {
			rx_ptr = target;
		}

		fsp_err_t err = R_SCI_B_UART_Read(handle->ctrl, rx_ptr, (uint32_t)chunk);

		if (err != FSP_SUCCESS) {
			xSemaphoreGive(handle->mutex);
			return -1;
		}

		if (xSemaphoreTake(handle->rx_sem, pdMS_TO_TICKS(RZV_UART_TRANSFER_TIMEOUT_MS)) != pdTRUE) {
			timeout_detected = true;
			break;
		}

		if (use_bounce) {
			R_BSP_CacheInvalidateRangeData(handle->rx_bounce, (uint32_t)chunk);
			memcpy(target, handle->rx_bounce, chunk);

		} else if (handle->dma_rx_enabled) {
			R_BSP_CacheInvalidateRangeData(rx_ptr, (uint32_t)chunk);
		}

		offset += chunk;
		remaining -= chunk;
	}

	xSemaphoreGive(handle->mutex);

	if (timeout_detected && (offset == 0U)) {
		/* Non-blocking behaviour: no new bytes available within timeout */
		return 0;
	}

	return (int)offset;
}

int rzv_uart_backend_write(rzv_uart_backend_t *handle, const uint8_t *buffer, size_t length)
{
	/* OpenAMP backend dispatch */
#if RZV_MAVLINK_OPENAMP_ENABLED
	if (handle->channel == RZV_UART_CH_MAVLINK && handle->opened) {
		return rzv_uart_openamp_write(handle, buffer, length);
	}
#endif

	if ((handle == NULL) || (buffer == NULL) || (length == 0U) || !handle->opened) {
		return -1;
	}

	if ((handle->tx_bounce == NULL) || (handle->bounce_len == 0U)) {
		return -1;
	}

	if (xSemaphoreTake(handle->mutex, pdMS_TO_TICKS(RZV_UART_TRANSFER_TIMEOUT_MS)) != pdTRUE) {
		return -1;
	}

	size_t remaining = length;
	size_t offset = 0U;

	while (remaining > 0U) {
		size_t chunk = remaining;

		while (xSemaphoreTake(handle->tx_sem, 0) == pdTRUE) {
			/* drain */
		}

		const uint8_t *source = &buffer[offset];
		const uint8_t *tx_ptr = source;
		bool use_bounce = false;

		if (handle->dma_tx_enabled) {
			const bool dma_safe = rzv_dma_buffer_is_dma_safe(source, chunk);
			const bool aligned = (((uintptr_t)source & (RZV_UART_DMA_WORD_ALIGN - 1U)) == 0U);
			const bool size_ok = chunk >= RZV_UART_DMA_MIN_BYTES;

			use_bounce = !dma_safe || !aligned || !size_ok;

			if (use_bounce && (chunk > handle->bounce_len)) {
				chunk = handle->bounce_len;
			}

			tx_ptr = use_bounce ? handle->tx_bounce : source;

			if (use_bounce) {
				memcpy(handle->tx_bounce, source, chunk);
				R_BSP_CacheCleanRangeData(handle->tx_bounce, (uint32_t)chunk);

			} else {
				R_BSP_CacheCleanRangeData((void *)tx_ptr, (uint32_t)chunk);
			}

		} else {
			tx_ptr = source;
		}

		fsp_err_t err = R_SCI_B_UART_Write(handle->ctrl, tx_ptr, (uint32_t)chunk);

		if (err != FSP_SUCCESS) {
			xSemaphoreGive(handle->mutex);
			return -1;
		}

		if (xSemaphoreTake(handle->tx_sem, pdMS_TO_TICKS(RZV_UART_TRANSFER_TIMEOUT_MS)) != pdTRUE) {
			xSemaphoreGive(handle->mutex);
			PX4_ERR("UART write timeout logical %u (hw %u)",
				(unsigned)handle->channel,
				(unsigned)handle->config.channel);
			return -1;
		}

		offset += chunk;
		remaining -= chunk;
	}

	xSemaphoreGive(handle->mutex);

	return (int)length;
}

int rzv_uart_backend_reopen(rzv_uart_backend_t *handle)
{
	if (handle == NULL) {
		return -1;
	}

	if (handle->opened) {
		R_SCI_B_UART_Close(handle->ctrl);
		handle->opened = false;
	}

	fsp_err_t err = R_SCI_B_UART_Open(handle->ctrl, &handle->config);

	if (err != FSP_SUCCESS) {
		PX4_ERR("R_SCI_B_UART_Open reopen failed (%d)", err);
		return -1;
	}

	err = R_SCI_B_UART_CallbackSet(handle->ctrl, rzv_uart_callback, handle, NULL);

	if (err != FSP_SUCCESS) {
		PX4_ERR("R_SCI_B_UART_CallbackSet reopen failed (%d)", err);
		R_SCI_B_UART_Close(handle->ctrl);
		return -1;
	}

	handle->opened = true;

	/* Restart RX for all channels using ring buffer */
	if (handle->use_ring_buffer) {
		uart_rc_buffer_flush_internal(handle->channel);

		if (handle->dma_rx_enabled) {
			/* DMA-enabled channels: restart DMA reception */
			handle->dma_rx_restart_failures = 0U;
			handle->dma_rx_active = false;
			handle->dma_rx_needs_restart = false;
			rzv_uart_dma_start_rx(handle);

			if (!handle->dma_rx_active) {
				PX4_WARN("UART (ch%u): DMAC RX restart failed, using interrupt fallback",
					 (unsigned)handle->channel);
				handle->dma_rx_enabled = false;
				handle->dma_rx_block_len = 0U;
			}
		} else {
			/* ISR-only channels: reset flag, let first read start ISR RX */
			handle->isr_rx_started = false;
			PX4_DEBUG("UART (ch%u): ISR RX will restart on first read",
				  (unsigned)handle->channel);
		}
	}

	return 0;
}

void rzv_uart_backend_shutdown(rzv_uart_backend_t *handle)
{
	/* OpenAMP backend dispatch */
#if RZV_MAVLINK_OPENAMP_ENABLED
	if (handle->channel == RZV_UART_CH_MAVLINK && handle->opened) {
		rzv_uart_openamp_shutdown(handle);
		return;
	}
#endif

	if ((handle == NULL) || !handle->opened) {
		return;
	}

	/* Unregister instance from callback lookup */
	if (handle->channel < RZV_UART_MAX_CHANNELS) {
		g_uart_backend_instances[handle->channel] = NULL;
	}

	handle->dma_rx_active = false;

	R_SCI_B_UART_Close(handle->ctrl);
	handle->opened = false;

	if (handle->use_ring_buffer) {
		uart_rc_buffer_flush_internal(handle->channel);
	}
}


void rzv_uart_rc_callback(uart_callback_args_t *p_args)
{
	rzv_uart_callback(p_args);
}

void rzv_uart_mav_callback(uart_callback_args_t *p_args)
{
	rzv_uart_callback(p_args);
}

void rzv_uart_gps_callback(uart_callback_args_t *p_args)
{
	rzv_uart_callback(p_args);
}

void rzv_uart_lidar_callback(uart_callback_args_t *p_args)
{
	rzv_uart_callback(p_args);
}

/**
 * @brief Get UART health statistics
 *
 * @param handle UART backend handle
 * @param ring_rx Total bytes received by ring buffer (output)
 * @param ring_overflow Ring buffer overflow count (output)
 * @param hw_overflow Hardware FIFO overflow count (output)
 * @param dma_restarts DMA restart failure count (output)
 * @return 0 on success, -1 on error
 */
int rzv_uart_get_health_stats(rzv_uart_backend_t *handle,
			       uint32_t *ring_rx,
			       uint32_t *ring_overflow,
			       uint32_t *hw_overflow,
			       uint32_t *dma_restarts)
{
	if (handle == NULL) {
		return -1;
	}

	/* Get ring buffer stats if using ring buffer */
	if (handle->use_ring_buffer) {
		uint32_t rx_count = 0;
		uint32_t overflow_count = 0;

		uart_rc_buffer_get_extended_stats(handle->channel,
						  &rx_count,
						  &overflow_count,
						  NULL, NULL);

		if (ring_rx != NULL) {
			*ring_rx = rx_count;
		}

		if (ring_overflow != NULL) {
			*ring_overflow = overflow_count;
		}
	} else {
		if (ring_rx != NULL) {
			*ring_rx = 0;
		}

		if (ring_overflow != NULL) {
			*ring_overflow = 0;
		}
	}

	/* Get hardware overflow count */
	if (hw_overflow != NULL) {
		*hw_overflow = handle->overflow_count;
	}

	/* Get DMA restart failures */
	if (dma_restarts != NULL) {
		*dma_restarts = handle->dma_rx_restart_failures;
	}

	return 0;
}

void rzv_uart_debug_isr_stats(void)
{
	for (uint8_t ch = 0; ch < RZV_UART_MAX_CHANNELS; ch++) {
		rzv_uart_backend_t *handle = g_uart_backend_instances[ch];

		if (handle == NULL || !handle->opened) {
			continue;
		}

		uint32_t ring_rx = 0;
		uint32_t ring_overflow = 0;
		uint32_t hw_overflow = 0;
		uint32_t dma_restarts = 0;

		rzv_uart_get_health_stats(handle, &ring_rx, &ring_overflow, &hw_overflow, &dma_restarts);

		PX4_INFO("UART[%u]: ev=%u rx_char=%u err=%u of_hw=%u of_ring=%u dma_restart=%u ring_rx=%u",
			 (unsigned)ch,
			 (unsigned)handle->event_count,
			 (unsigned)handle->rx_char_count,
			 (unsigned)handle->error_count,
			 (unsigned)hw_overflow,
			 (unsigned)ring_overflow,
			 (unsigned)dma_restarts,
			 (unsigned)ring_rx);
	}
}

#endif /* __PX4_FREERTOS */
