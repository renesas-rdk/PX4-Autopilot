/****************************************************************************
 *
 * Copyright (c) 2025 Renesas Electronics Corporation and/or its affiliates
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 ****************************************************************************/


#if defined(__PX4_FREERTOS)

#include "rzv_fsp/i2c_fsp_backend.h"
#define MODULE_NAME "rzv_i2c_backend"

#include <FreeRTOS.h>
#include <semphr.h>
#include <task.h>

#include <visibility.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/time.h>
#include <errno.h>

#include <string.h>

#include <stdarg.h>
#include <stdio.h>

#include "bsp_api.h"
#include "r_sci_b_i2c.h"

extern uint8_t _ncbuffer_start;
extern uint8_t _ncbuffer_end;

static inline bool rzv_i2c_dma_safe(const void *ptr, size_t len)
{
	uintptr_t start = (uintptr_t)&_ncbuffer_start;
	uintptr_t end = (uintptr_t)&_ncbuffer_end;
	uintptr_t p = (uintptr_t)ptr;
	return (p >= start) && ((p + len) <= end) && ((p % 4U) == 0U);
}

#define RZV_I2C_DEFAULT_RATE_HZ 100000U
#define RZV_I2C_TRANSFER_TIMEOUT_MS 100U
#define RZV_I2C_MUTEX_TIMEOUT_MS   (RZV_I2C_TRANSFER_TIMEOUT_MS + 50U)
#define RZV_I2C_RETRY_COUNT        3U
#define RZV_I2C_EVENT_RESET ((i2c_master_event_t)0)
#define RZV_I2C_BOUNCE_LEN 256U
#define RZV_I2C_RECOVERY_BACKOFF_US 200U
#define RZV_I2C_ABORT_RECOVERY_THRESHOLD 3U
#define RZV_I2C_ERROR_LOG_THROTTLE_US (5U * 1000U * 1000U)

#if defined(__GNUC__)
#define RZV_I2C_DMA_ALIGN __attribute__((aligned(32)))
#define RZV_I2C_DMA_NONCACHE __attribute__((section(".noncache_buffer")))
#else
#define RZV_I2C_DMA_ALIGN
#define RZV_I2C_DMA_NONCACHE
#endif

static StaticSemaphore_t g_i2c_mutex_buffer;
static SemaphoreHandle_t g_i2c_mutex;
static StaticSemaphore_t g_i2c_event_sem_buffer;
static SemaphoreHandle_t g_i2c_event_sem;

static uint8_t g_i2c_tx_bounce[RZV_I2C_BOUNCE_LEN] RZV_I2C_DMA_ALIGN RZV_I2C_DMA_NONCACHE;
static uint8_t g_i2c_rx_bounce[RZV_I2C_BOUNCE_LEN] RZV_I2C_DMA_ALIGN RZV_I2C_DMA_NONCACHE;

static volatile i2c_master_event_t g_i2c_event = RZV_I2C_EVENT_RESET;
static hrt_abstime g_i2c_last_error_log_us = 0;
static uint32_t g_i2c_suppressed_errors = 0;
static uint16_t g_i2c_last_error_addr = 0xFFFFU; /* I2C address whose errors filled the current throttle window */
static uint16_t g_i2c_current_addr = 0xFFFFU;    /* address being transferred right now (set in process_message) */
static TickType_t g_i2c_last_recovery_tick = 0;
static uint32_t g_i2c_recovery_count = 0;

static void i2c_scl_toggle_recovery(void);
static void rzv_i2c_apply_bus_rate(rzv_i2c_backend_t *handle);
static bool rzv_i2c_reduce_bus_speed(rzv_i2c_backend_t *handle);

static void rzv_i2c_post_success_maintenance(void)
{
	/* SCI-B driver does not expose SDA status; no post-success maintenance required */
}

static void rzv_i2c_log_error(const rzv_i2c_backend_t *handle, const char *fmt, ...)
{
	const hrt_abstime now = hrt_absolute_time();

	/* Per-address throttle: if the current transfer address differs from the address that last
	 * filled the throttle window, reset the throttle immediately.
	 * This ensures that BMP280 (0x76) / INA228 (0x45) errors are NEVER suppressed because
	 * IST8310 probe (0x0E, not connected) flooded the log window at boot.
	 * Without this: IST8310's 30 boot-time NACKs suppress any BMP280 error for the next 5s.
	 */
	if (g_i2c_current_addr != g_i2c_last_error_addr) {
		if (g_i2c_suppressed_errors > 0) {
			const uint32_t log_bus_hz = (handle != NULL) ? handle->bus_hz : RZV_I2C_DEFAULT_RATE_HZ;
			PX4_INFO("I2C[%lu Hz] addr 0x%02x: suppressed %lu similar errors",
				 (unsigned long)log_bus_hz,
				 (unsigned)g_i2c_last_error_addr,
				 (unsigned long)g_i2c_suppressed_errors);
			g_i2c_suppressed_errors = 0;
		}

		g_i2c_last_error_addr = g_i2c_current_addr;
		g_i2c_last_error_log_us = 0;  /* Reset throttle for new address */
	}

	if ((g_i2c_last_error_log_us != 0) && ((now - g_i2c_last_error_log_us) < RZV_I2C_ERROR_LOG_THROTTLE_US)) {
		g_i2c_suppressed_errors++;
		return;
	}

	if (g_i2c_suppressed_errors > 0) {
		const uint32_t log_bus_hz = (handle != NULL) ? handle->bus_hz : RZV_I2C_DEFAULT_RATE_HZ;
		PX4_INFO("I2C[%lu Hz] addr 0x%02x: suppressed %lu similar errors",
			 (unsigned long)log_bus_hz,
			 (unsigned)g_i2c_last_error_addr,
			 (unsigned long)g_i2c_suppressed_errors);
		g_i2c_suppressed_errors = 0;
	}

	char buffer[160] = {};
	va_list args;
	va_start(args, fmt);
	(void)vsnprintf(buffer, sizeof(buffer), fmt, args);
	va_end(args);

	PX4_WARN("I2C[%lu Hz] %s",
		 (unsigned long)((handle != NULL) ? handle->bus_hz : RZV_I2C_DEFAULT_RATE_HZ),
		 buffer);
	g_i2c_last_error_log_us = now;
}

static void rzv_i2c_apply_bus_rate(rzv_i2c_backend_t *handle)
{
	if (handle == NULL) {
		return;
	}

	if (handle->bus_hz >= 1000000U) {
		handle->bus_hz = 1000000U;
		handle->config.rate = I2C_MASTER_RATE_FASTPLUS;

	} else if (handle->bus_hz >= 400000U) {
		handle->bus_hz = 400000U;
		handle->config.rate = I2C_MASTER_RATE_FAST;

	} else {
		handle->bus_hz = 100000U;
		handle->config.rate = I2C_MASTER_RATE_STANDARD;
	}
}

static bool rzv_i2c_reduce_bus_speed(rzv_i2c_backend_t *handle)
{
	if (handle == NULL) {
		return false;
	}

	uint32_t new_speed = handle->bus_hz;

	if (handle->bus_hz > 400000U) {
		new_speed = 400000U;

	} else if (handle->bus_hz > 100000U) {
		new_speed = 100000U;

	} else {
		return false;
	}

	if (new_speed != handle->bus_hz) {
		handle->bus_hz = new_speed;
		rzv_i2c_apply_bus_rate(handle);
		return true;
	}

	return false;
}

static void rzv_i2c_reset_event_state(void)
{
	g_i2c_event = RZV_I2C_EVENT_RESET;

	if (g_i2c_event_sem != NULL) {
		while (xSemaphoreTake(g_i2c_event_sem, 0) == pdTRUE) {
			/* drain semaphore */
		}
	}
}

static void rzv_i2c_callback(i2c_master_callback_args_t *p_args)
{
	BaseType_t higher_priority_task_woken = pdFALSE;

	if (p_args != NULL) {
		g_i2c_event = p_args->event;

	} else {
		g_i2c_event = RZV_I2C_EVENT_RESET;
	}

	if (g_i2c_event_sem != NULL) {
		(void)xSemaphoreGiveFromISR(g_i2c_event_sem, &higher_priority_task_woken);
		portYIELD_FROM_ISR(higher_priority_task_woken);
	}
}

static fsp_err_t rzv_i2c_wait_for_event(rzv_i2c_backend_t *handle, i2c_master_event_t *event_out)
{
	if (g_i2c_event_sem == NULL) {
		return FSP_ERR_ASSERTION;
	}

	const uint64_t start_us = hrt_absolute_time();
	const uint64_t spin_until = start_us + 100U;
	const uint64_t deadline_us = start_us + (uint64_t)RZV_I2C_TRANSFER_TIMEOUT_MS * 1000U;

	while (g_i2c_event == RZV_I2C_EVENT_RESET) {
		if (hrt_absolute_time() > spin_until) {
			break;
		}
	}

	while (g_i2c_event == RZV_I2C_EVENT_RESET) {
		const uint64_t now_us = hrt_absolute_time();

		if (now_us >= deadline_us) {
			return FSP_ERR_TIMEOUT;
		}

		uint64_t remaining_us = deadline_us - now_us;
		uint32_t wait_ms = 1U;

		if (remaining_us > 1000U) {
			wait_ms = (uint32_t)((remaining_us + 999U) / 1000U);
		}

		if (wait_ms > RZV_I2C_TRANSFER_TIMEOUT_MS) {
			wait_ms = RZV_I2C_TRANSFER_TIMEOUT_MS;
		}

		TickType_t wait_ticks = pdMS_TO_TICKS(wait_ms);

		if (wait_ticks == 0U) {
			wait_ticks = 1U;
		}

		(void)xSemaphoreTake(g_i2c_event_sem, wait_ticks);
	}

	i2c_master_event_t event = g_i2c_event;
	rzv_i2c_reset_event_state();

	if (event_out != NULL) {
		*event_out = event;
	}

	if ((event == I2C_MASTER_EVENT_TX_COMPLETE) || (event == I2C_MASTER_EVENT_RX_COMPLETE)) {
		return FSP_SUCCESS;
	}

	if (event == I2C_MASTER_EVENT_ABORTED) {
		/* NACK from slave = device absent or rejected the transaction.
		 * This is normal behaviour during driver probe (device not present) and
		 * does not indicate a bus error — do NOT log a warning here.
		 * The upper layer (drivers__device "I2C probe failed") provides user-visible
		 * feedback.  Recovery (close/reopen) is reserved for FSP_ERR_TIMEOUT only.
		 */
		PX4_DEBUG("I2C addr 0x%02x NACK (event=%d)",
			  (unsigned)g_i2c_current_addr, (int)event);
		return FSP_ERR_TRANSFER_ABORTED;
	}

	rzv_i2c_log_error(handle, "I2C unexpected event=%d", (int)event);
	return FSP_ERR_ASSERTION;
}

static void i2c_scl_toggle_recovery(void)
{
	/* SCI-B does not expose manual clock toggling via GPIO.
	 * Implement a more robust recovery sequence:
	 * 1. Multiple abort attempts to generate STOP conditions
	 * 2. Close and reopen to reset peripheral state
	 * 3. Dummy read attempt to generate clock pulses (up to 9 clocks to release stuck slave)
	 *
	 * I2C standard: A slave holding SDA low can be released by clocking SCL
	 * up to 9 times until it releases, then sending a STOP condition.
	 */

	/* Step 1: Multiple abort attempts to try generating STOP conditions */
	for (int i = 0; i < 3; i++) {
		(void)R_SCI_B_I2C_Abort(&g_i2c_baro_ctrl);
		vTaskDelay(pdMS_TO_TICKS(1));
	}

	/* Step 2: Close the peripheral */
	(void)R_SCI_B_I2C_Close(&g_i2c_baro_ctrl);
	vTaskDelay(pdMS_TO_TICKS(2));

	/* Step 3: Reopen the peripheral */
	fsp_err_t err = R_SCI_B_I2C_Open(&g_i2c_baro_ctrl, &g_i2c_baro_cfg);

	if (err != FSP_SUCCESS) {
		PX4_DEBUG("I2C recovery: reopen failed (%d), retrying", err);
		vTaskDelay(pdMS_TO_TICKS(5));
		(void)R_SCI_B_I2C_Open(&g_i2c_baro_ctrl, &g_i2c_baro_cfg);
	}

	/* Step 4: Attempt a dummy read to 0x00 (general call) to generate clocks
	 * This may fail but will generate clock pulses on the bus.
	 * Use short timeout to avoid blocking too long.
	 */
	uint8_t dummy = 0;
	(void)R_SCI_B_I2C_SlaveAddressSet(&g_i2c_baro_ctrl, 0x00, I2C_MASTER_ADDR_MODE_7BIT);
	(void)R_SCI_B_I2C_Read(&g_i2c_baro_ctrl, &dummy, 1, false);
	vTaskDelay(pdMS_TO_TICKS(2));
	(void)R_SCI_B_I2C_Abort(&g_i2c_baro_ctrl);

	/* Final delay to let bus settle */
	vTaskDelay(pdMS_TO_TICKS(1));
}

static void rzv_i2c_copy_extend(rzv_i2c_backend_t *handle)
{
	memcpy(&handle->extend,
	       (const sci_b_i2c_extended_cfg_t *)g_i2c_baro_cfg.p_extend,
	       sizeof(sci_b_i2c_extended_cfg_t));
}

static int rzv_i2c_reopen(rzv_i2c_backend_t *handle)
{
	fsp_err_t err = FSP_SUCCESS;

	if (handle->opened) {
		err = R_SCI_B_I2C_Close(handle->ctrl);

		if ((err != FSP_SUCCESS) && (err != FSP_ERR_NOT_OPEN)) {
			PX4_ERR("R_SCI_B_I2C_Close failed (%d)", err);
			return -1;
		}

		handle->opened = false;
	}

	err = R_SCI_B_I2C_Open(handle->ctrl, &handle->config);

	if (err != FSP_SUCCESS) {
		PX4_ERR("R_SCI_B_I2C_Open failed (%d)", err);
		return -1;
	}

	err = R_SCI_B_I2C_CallbackSet(handle->ctrl, rzv_i2c_callback, NULL, NULL);

	if (err != FSP_SUCCESS) {
		PX4_ERR("R_SCI_B_I2C_CallbackSet failed (%d)", err);
		return -1;
	}

	handle->opened = true;
	rzv_i2c_reset_event_state();
	handle->addr_valid = false;
	handle->current_addr = 0U;
	handle->current_flags = 0U;
	return 0;
}

static int rzv_i2c_recover(rzv_i2c_backend_t *handle)
{
	if ((handle == NULL) || (handle->ctrl == NULL)) {
		return -1;
	}
	const TickType_t now_ticks = xTaskGetTickCount();

	if (g_i2c_last_recovery_tick != 0) {
		const TickType_t ticks_since_last = now_ticks - g_i2c_last_recovery_tick;
		const TickType_t min_interval_ticks = pdMS_TO_TICKS(2);

		if (ticks_since_last < min_interval_ticks) {
			vTaskDelay(min_interval_ticks - ticks_since_last);
		}
	}

	handle->addr_valid = false;
	handle->current_addr = 0U;
	handle->current_flags = 0U;

	fsp_err_t err = R_SCI_B_I2C_Abort(handle->ctrl);

	if ((err != FSP_SUCCESS) && (err != FSP_ERR_NOT_OPEN)) {
		PX4_ERR("R_SCI_B_I2C_Abort failed (%d)", err);
	}

	bool toggled = false;

	if (handle->opened) {
		i2c_scl_toggle_recovery();
		toggled = true;

		err = R_SCI_B_I2C_Close(handle->ctrl);

		if ((err != FSP_SUCCESS) && (err != FSP_ERR_NOT_OPEN)) {
			PX4_ERR("R_SCI_B_I2C_Close during recovery failed (%d)", err);
			return -1;
		}

		handle->opened = false;

		vTaskDelay(pdMS_TO_TICKS(2));
	}

	if (!toggled) {
		/* If the driver was already closed, we cannot safely poke registers.
		 * Attempt a best-effort toggle only when the peripheral is open. */
		PX4_WARN("I2C recovery requested while bus already closed; skipping manual SCL toggle");
	}

	px4_usleep(RZV_I2C_RECOVERY_BACKOFF_US);
	rzv_i2c_apply_bus_rate(handle);

	const int reopen_result = rzv_i2c_reopen(handle);

	if (reopen_result == 0) {
		g_i2c_recovery_count++;
		g_i2c_last_recovery_tick = xTaskGetTickCount();
		vTaskDelay(pdMS_TO_TICKS(1));
		PX4_DEBUG("I2C bus recovery OK (count=%lu)", (unsigned long)g_i2c_recovery_count);
	}

	return reopen_result;
}

int rzv_i2c_backend_init(rzv_i2c_backend_t *handle, uint8_t bus, uint32_t frequency_hz)
{
	if (handle == NULL) {
		return -1;
	}

	if (bus != g_i2c_baro_cfg.channel) {
		PX4_ERR("Unsupported I2C bus %u", (unsigned)bus);
		return -1;
	}

	/* Force a clean SCI (I2C) peripheral block before (re-)opening — see the same
	 * note in spi_fsp_backend.c. On a WARM CR8 restart the FSP ctrl is fresh so
	 * R_SCI_B_I2C_Close no-ops and the SCI channel keeps stale state from the
	 * previous instance; an explicit per-channel MODULE_STOP (Open re-STARTs it)
	 * gives a clean block. Per-channel MSTP, harmless on cold boot. */
	R_BSP_MODULE_STOP(FSP_IP_SCI, g_i2c_baro_cfg.channel);
	R_BSP_SoftwareDelay(500, BSP_DELAY_UNITS_MICROSECONDS);

	memset(handle, 0, sizeof(*handle));
	rzv_i2c_reset_event_state();

	if (g_i2c_mutex == NULL) {
		g_i2c_mutex = xSemaphoreCreateMutexStatic(&g_i2c_mutex_buffer);
	}

	if (g_i2c_mutex == NULL) {
		PX4_ERR("I2C semaphore init failed");
		return -1;
	}

	if (g_i2c_event_sem == NULL) {
		g_i2c_event_sem = xSemaphoreCreateBinaryStatic(&g_i2c_event_sem_buffer);

		if (g_i2c_event_sem == NULL) {
			PX4_ERR("I2C event semaphore init failed");
			return -1;
		}

		/* ensure empty state */
		(void)xSemaphoreTake(g_i2c_event_sem, 0);
	}

	handle->bus = bus;
	handle->ctrl = &g_i2c_baro_ctrl;
	memcpy(&handle->config, &g_i2c_baro_cfg, sizeof(i2c_master_cfg_t));
	rzv_i2c_copy_extend(handle);
	handle->config.p_extend = &handle->extend;
	handle->config.p_callback = rzv_i2c_callback;
	handle->bus_hz = (frequency_hz != 0U) ? frequency_hz : RZV_I2C_DEFAULT_RATE_HZ;
	handle->error_streak = 0;
	rzv_i2c_apply_bus_rate(handle);

	if (rzv_i2c_reopen(handle) != 0) {
		return -1;
	}

	return 0;
}

static int rzv_i2c_select_address(rzv_i2c_backend_t *handle, uint16_t address, uint16_t flags)
{
	const uint16_t relevant_flags = flags & I2C_M_TEN;

	if (handle->addr_valid && (handle->current_addr == address) && (handle->current_flags == relevant_flags)) {
		return 0;
	}

	const bool ten_bit = (relevant_flags & I2C_M_TEN) != 0;
	fsp_err_t err = R_SCI_B_I2C_SlaveAddressSet(handle->ctrl,
				address,
				ten_bit ? I2C_MASTER_ADDR_MODE_10BIT : I2C_MASTER_ADDR_MODE_7BIT);

	if (err != FSP_SUCCESS) {
		handle->addr_valid = false;
		PX4_ERR("I2C address set failed (%d)", err);
		return -1;
	}

	handle->current_addr = address;
	handle->current_flags = relevant_flags;
	handle->addr_valid = true;

	return 0;
}

static void rzv_i2c_handle_transfer_failure(rzv_i2c_backend_t *handle,
		const struct i2c_msg *msg,
		fsp_err_t err,
		i2c_master_event_t event,
		uint32_t attempt)
{
	const unsigned long attempt_idx = (unsigned long)(attempt + 1U);

	if ((err == FSP_ERR_TIMEOUT) && (event == RZV_I2C_EVENT_RESET)) {
		PX4_DEBUG("I2C[%lu Hz] addr 0x%02x transfer timeout (attempt %lu)",
			  (unsigned long)handle->bus_hz,
			  (unsigned)msg->addr,
			  attempt_idx);

	} else if (err == FSP_ERR_TIMEOUT) {
		rzv_i2c_log_error(handle,
				  "I2C addr 0x%02x transfer timeout (attempt %lu)",
				  (unsigned)msg->addr,
				  attempt_idx);

	} else if (err == FSP_ERR_TRANSFER_ABORTED) {
		/* NACK = device absent or rejected this transaction.  Use DEBUG so that
		 * i2cdetect scanning ~100 absent addresses does not flood the log.
		 * The driver-level probe (e.g. IST8310, BMP280) will get a clean error
		 * message via PX4_ERR from the upper layer if probe fails.
		 */
		PX4_DEBUG("I2C[%lu Hz] addr 0x%02x NACK (attempt %lu)",
			  (unsigned long)handle->bus_hz,
			  (unsigned)msg->addr,
			  attempt_idx);

	} else {
		rzv_i2c_log_error(handle,
				  "I2C addr 0x%02x error (%d) (attempt %lu) event=%d",
				  (unsigned)msg->addr,
				  err,
				  attempt_idx,
				  (int)event);
	}

	/* NACK = device absent: not a bus error, do not count against streak or
	 * trigger speed reduction / recovery.  Only actual bus-level errors (timeout,
	 * unexpected events) should degrade the bus state.
	 */
	if (err != FSP_ERR_TRANSFER_ABORTED) {
		handle->error_streak++;

		if (handle->error_streak >= RZV_I2C_ABORT_RECOVERY_THRESHOLD) {
			if (rzv_i2c_reduce_bus_speed(handle)) {
				rzv_i2c_log_error(handle,
						  "I2C bus slowed to %lu Hz after repeated errors",
						  (unsigned long)handle->bus_hz);
			}

			handle->error_streak = 0;
		}

		/* Only trigger full bus recovery for confirmed bus hangs (timeout = no event
		 * fired within 100 ms = SDA stuck low).
		 */
		if (err == FSP_ERR_TIMEOUT) {
			(void)rzv_i2c_recover(handle);
		}

		px4_usleep(RZV_I2C_RECOVERY_BACKOFF_US);
	}
}

static bool rzv_i2c_process_message(rzv_i2c_backend_t *handle, struct i2c_msg *msg, bool restart)
{
	if ((msg->len > RZV_I2C_BOUNCE_LEN) && (msg->len != 0U)) {
		PX4_ERR("I2C message too large for bounce buffer (%u)", (unsigned)msg->len);
		return false;
	}

	const bool read_direction = (msg->flags & I2C_M_RD) != 0U;

	/* Set current address context for per-address error throttling in rzv_i2c_log_error() */
	g_i2c_current_addr = msg->addr;

	for (uint32_t attempt = 0; attempt < RZV_I2C_RETRY_COUNT; attempt++) {
		PX4_DEBUG("I2C[%lu Hz] probe addr=0x%02x attempt=%lu",
			  (unsigned long)(handle ? handle->bus_hz : 0U),
			  (unsigned)msg->addr,
			  (unsigned long)(attempt + 1U));

		if (rzv_i2c_select_address(handle, msg->addr, msg->flags) != 0) {
			return false;
		}

		uint8_t *transfer_ptr = (msg->len == 0U) ? msg->buf :
					(read_direction ? g_i2c_rx_bounce : g_i2c_tx_bounce);
		bool direct_buffer = false;

		/* If caller provided a DMA-safe, non-cache buffer, use it directly to avoid memcpy */
		if ((msg->len != 0U) && rzv_i2c_dma_safe(msg->buf, msg->len)) {
			transfer_ptr = msg->buf;
			direct_buffer = true;
		}

		if (!read_direction && (msg->len != 0U)) {
			if (!direct_buffer) {
				memcpy(g_i2c_tx_bounce, msg->buf, msg->len);
				R_BSP_CacheCleanRangeData(g_i2c_tx_bounce, msg->len);
			} else {
				R_BSP_CacheCleanRangeData(msg->buf, msg->len);
			}
		}

		/* Reset event state and start transfer atomically.
		 * Without the critical section, a stale I2C ISR from a previous aborted transfer
		 * could fire between reset_event_state() and the FSP Read/Write call, leaving
		 * g_i2c_event = ABORTED before the new transfer even starts. The next
		 * rzv_i2c_wait_for_event() would then immediately return with the stale event,
		 * causing a false FSP_ERR_TRANSFER_ABORTED and an unnecessary retry+recovery.
		 * The critical section is held for <1µs (just two FSP register writes).
		 */
		taskENTER_CRITICAL();
		rzv_i2c_reset_event_state();
		fsp_err_t err;

		if (!read_direction && (msg->len == 0U)) {
			/* 0-byte WRITE: address-only probe (used by i2cdetect to avoid data-phase
			 * clock-stretching issues on devices like IST8310).
			 * Some FSP versions reject R_SCI_B_I2C_Write with len=0; fall back to
			 * a 1-byte read if the write fails at the API level.
			 */
			err = R_SCI_B_I2C_Write(handle->ctrl, transfer_ptr, 0U, restart);

			if (err == FSP_ERR_INVALID_ARGUMENT) {
				/* FSP does not support 0-byte write; use a 1-byte read instead */
				err = R_SCI_B_I2C_Read(handle->ctrl, g_i2c_rx_bounce, 1U, restart);
			}

		} else {
			err = read_direction ?
					R_SCI_B_I2C_Read(handle->ctrl, transfer_ptr, msg->len, restart) :
					R_SCI_B_I2C_Write(handle->ctrl, transfer_ptr, msg->len, restart);
		}

		taskEXIT_CRITICAL();

		if (err != FSP_SUCCESS) {
			rzv_i2c_log_error(handle,
					  "I2C addr 0x%02x transfer start failed (%d) (attempt %lu)",
					  (unsigned)msg->addr,
					  err,
					  (unsigned long)(attempt + 1U));
			(void)rzv_i2c_recover(handle);
			px4_usleep(RZV_I2C_RECOVERY_BACKOFF_US);
			continue;
		}

		i2c_master_event_t event = RZV_I2C_EVENT_RESET;
		err = rzv_i2c_wait_for_event(handle, &event);

		if (err == FSP_SUCCESS) {
			if (read_direction && (msg->len != 0U)) {
				if (!direct_buffer) {
					R_BSP_CacheInvalidateRangeData(g_i2c_rx_bounce, msg->len);
					memcpy(msg->buf, g_i2c_rx_bounce, msg->len);
				} else {
					R_BSP_CacheInvalidateRangeData(msg->buf, msg->len);
				}
			}

			handle->error_streak = 0;
			rzv_i2c_post_success_maintenance();
			return true;
		}

		rzv_i2c_handle_transfer_failure(handle, msg, err, event, attempt);

		/* FSP_ERR_TRANSFER_ABORTED = NACK from slave = device absent or busy.
		 * Retrying a NACK is pointless: the device either is not present or has
		 * explicitly rejected this transaction.  Break immediately to avoid
		 * unnecessary log spam and scan latency (e.g. during i2cdetect where
		 * ~100 addresses are absent and each retry adds ~5 ms + 2 log lines).
		 * Only retry transient errors (FSP_ERR_TIMEOUT = bus stuck, start failure).
		 */
		if (err == FSP_ERR_TRANSFER_ABORTED) {
			break;
		}
	}

	PX4_DEBUG("I2C addr 0x%02x all attempts failed (last event=%d)",
		  (unsigned)msg->addr,
		  (int)g_i2c_event);
	return false;
}

int rzv_i2c_backend_transfer(rzv_i2c_backend_t *handle, struct i2c_msg *msgs, size_t msg_count)
{
	if ((handle == NULL) || (msgs == NULL) || (msg_count == 0U) || !handle->opened) {
		return -1;
	}

	if (g_i2c_mutex == NULL) {
		return -1;
	}

	if (xSemaphoreTake(g_i2c_mutex, pdMS_TO_TICKS(RZV_I2C_MUTEX_TIMEOUT_MS)) != pdTRUE) {
		PX4_ERR("I2C mutex timeout");
		return -1;
	}

	int ret = -1;

	for (size_t i = 0; i < msg_count; i++) {
		struct i2c_msg *msg = &msgs[i];
		const bool restart = (i < (msg_count - 1U));

		if (!rzv_i2c_process_message(handle, msg, restart)) {
			goto out_unlock;
		}
	}

	ret = 0;

out_unlock:
	xSemaphoreGive(g_i2c_mutex);
	return ret;
}

void rzv_i2c_backend_shutdown(rzv_i2c_backend_t *handle)
{
	if ((handle == NULL) || !handle->opened) {
		return;
	}

	R_SCI_B_I2C_Close(handle->ctrl);
	handle->opened = false;
	handle->addr_valid = false;
	handle->current_addr = 0U;
	handle->current_flags = 0U;
}

void rzv_sci_b_i2c_baro_callback(i2c_master_callback_args_t *p_args)
{
	rzv_i2c_callback(p_args);
}

#endif /* __PX4_FREERTOS */
