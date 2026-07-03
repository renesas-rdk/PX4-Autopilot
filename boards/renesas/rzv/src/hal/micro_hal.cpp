/****************************************************************************
 *
 * Copyright (c) 2025 Renesas Electronics Corporation and/or its affiliates
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 ****************************************************************************/

// FreeRTOS-based GPIO event shim with IRQ + polling fallback

#if defined(__PX4_FREERTOS)

#include <px4_platform/micro_hal.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/spi.h>

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include <px4_platform_common/tasks.h>

#include "hal_data.h"
/* hal_data.h emits BSP_TRIGGER_INTC_TINT0_* only when e2studio has r_intc_tint
 * external-IRQ (DRDY) channels configured — the FSP tree then also ships
 * r_external_irq_api.h.  The drone FSP config has the DRDY TINT channels and
 * the header removed, so the hardware-IRQ path below compiles out and every
 * DRDY registration falls back to the GPIO poll task.  Platform products that
 * keep the channels get the full hardware path from the same source. */
#if defined(BSP_TRIGGER_INTC_TINT0_INTC_TINT_TRIGGER_FALLING)
#define RZV_DRDY_HW_IRQ 1
#include "r_external_irq_api.h"
#else
#define RZV_DRDY_HW_IRQ 0
#endif
#include "r_ioport.h"
#include "sensor_hal.h"
#include <rzv_fsp/hrt.h>

#include <errno.h>
#include <atomic>

extern const px4_spi_bus_t px4_spi_buses[SPI_BUS_MAX_BUS_ITEMS];

static constexpr TickType_t DRDY_POLL_INTERVAL_TICKS = 1;

#if defined(configSTACK_DEPTH_TYPE)
static constexpr configSTACK_DEPTH_TYPE DRDY_IRQ_STACK_DEPTH_WORDS = 2048;
static constexpr configSTACK_DEPTH_TYPE DRDY_POLL_STACK_DEPTH_WORDS = 2048;
#else
static constexpr uint32_t DRDY_IRQ_STACK_DEPTH_WORDS = 2048;
static constexpr uint32_t DRDY_POLL_STACK_DEPTH_WORDS = 2048;
#endif

static constexpr UBaseType_t drdy_irq_priority()
{
	const UBaseType_t max = configMAX_PRIORITIES ? configMAX_PRIORITIES : 1;
	/* WARNING: margin must be >= 5 to keep priority BELOW wq:INS0 (27).
	 * When drdy_def priority > wq:INS0, preemption causes stack overflow.
	 * This is a known issue - do not increase priority above INS0!
	 * margin=8 gives priority 24 (safe).
	 */
	const UBaseType_t margin = 8;
	return (max > margin) ? (max - margin) : (max > 1 ? max - 1 : 0);
}

static constexpr UBaseType_t drdy_poll_priority()
{
	const UBaseType_t max = configMAX_PRIORITIES ? configMAX_PRIORITIES : 1;
	/* Polling fallback priority - keep below INS0 to avoid stack overflow.
	 * margin=12 gives priority 20 (safe).
	 */
	const UBaseType_t margin = 12;
	return (max > margin) ? (max - margin) : (max > 1 ? max - 1 : 0);
}

namespace
{

/*
 * DRDY hardware-IRQ channel map. Each IMU data-ready line is wired to a distinct
 * r_intc_tint external-IRQ channel configured in e2studio (g_external_irqN, with
 * TINT Source = the pin below, Falling edge, callback mpu_drdy_callback). This must
 * stay in sync with both the FSP pin configuration AND px4_spi_buses[].drdy_gpio.
 *
 *   ch0  g_external_irq0  P50 (BSP_IO_PORT_05_PIN_00, 0x0500)  SSL0 / IMU#1
 *   ch1  g_external_irq1  PA0 (BSP_IO_PORT_10_PIN_00, 0x0A00)  SSL1 / IMU#2
 *   ch2  g_external_irq2  P74 (BSP_IO_PORT_07_PIN_04, 0x0704)  SSL2 / IMU#3
 *
 * Pins not listed here fall back to the GPIO polling task (still functional, just
 * higher CPU and coarser timing).
 */
#if RZV_DRDY_HW_IRQ
struct DrdyIrqChannel {
	uint32_t                       portpin;   // bsp_io_port_pin_t value (lower 16 bits)
	const external_irq_instance_t *instance;
	uint32_t                       channel;   // external_irq_callback_args_t.channel
};

static const DrdyIrqChannel g_drdy_irq_channels[] = {
	{ 0x0500u, &g_external_irq0, 0u },
	{ 0x0A00u, &g_external_irq1, 1u },
	{ 0x0704u, &g_external_irq2, 2u },
};

static constexpr size_t DRDY_IRQ_CHANNEL_COUNT =
	sizeof(g_drdy_irq_channels) / sizeof(g_drdy_irq_channels[0]);

static const DrdyIrqChannel *find_irq_channel_by_pin(uint32_t pinset)
{
	const uint32_t portpin = pinset & 0xFFFFu;

	for (const auto &ch : g_drdy_irq_channels) {
		if (ch.portpin == portpin) {
			return &ch;
		}
	}

	return nullptr;
}
#endif /* RZV_DRDY_HW_IRQ */

struct DrdyClient {
	bool        in_use{false};
	uint32_t    pinset{0};
	xcpt_t      callback{nullptr};
	void       *callback_arg{nullptr};

	TaskHandle_t deferred_task{nullptr};
	StaticTask_t deferred_task_tcb{};
	StackType_t  deferred_stack[DRDY_IRQ_STACK_DEPTH_WORDS];

	TaskHandle_t poll_task{nullptr};
	volatile bool poll_run{false};
	SemaphoreHandle_t poll_stop_sem{nullptr};
	StaticSemaphore_t poll_stop_sem_buffer{};

	// Ring buffer of ISR timestamps (one entry per DRDY pulse).
	// Each DataReady() callback pops one entry so the batch-completing call
	// always receives the timestamp of its own ISR, not a later one.
	static constexpr size_t DRDY_TS_BUF_SIZE = 16;
	volatile uint64_t drdy_timestamps[DRDY_TS_BUF_SIZE]{};
	std::atomic<uint32_t> drdy_ts_write_idx{0};
	std::atomic<uint32_t> drdy_ts_read_idx{0};
};

static constexpr size_t MAX_DRDY_CLIENTS = 6;
static DrdyClient g_drdy_clients[MAX_DRDY_CLIENTS];
#if RZV_DRDY_HW_IRQ
// One client + open-flag per hardware IRQ channel (indexed by DrdyIrqChannel.channel).
static std::atomic<DrdyClient *> g_irq_client_by_channel[DRDY_IRQ_CHANNEL_COUNT] {};
static bool g_irq_open_by_channel[DRDY_IRQ_CHANNEL_COUNT] {};
#endif

static void drdy_deferred_task(void *param)
{
	auto *client = static_cast<DrdyClient *>(param);

	for (;;) {
		uint32_t pending = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

		while (pending-- > 0) {
			if (client && client->callback) {
				client->callback(0, nullptr, client->callback_arg);
			}
		}
	}
}

static inline uint32_t pinset_to_portpin(uint32_t pinset)
{
	// bsp_io_port_pin_t encodes port/pin in the lower 16 bits.
	constexpr uint32_t kPortPinMask = 0xFFFFu;
	return pinset & kPortPinMask;
}

static inline bsp_io_port_pin_t resolve_pin(uint32_t pinset)
{
	return static_cast<bsp_io_port_pin_t>(pinset_to_portpin(pinset));
}

static inline bsp_io_port_pin_t irq_supported_pin()
{
#if RZV_DRDY_HW_IRQ
	// First mapped channel — used only for the diagnostic WARN below.
	return static_cast<bsp_io_port_pin_t>(g_drdy_irq_channels[0].portpin);
#else
	return static_cast<bsp_io_port_pin_t>(0);
#endif
}

static bool pin_supports_hw_irq(uint32_t pinset)
{
#if RZV_DRDY_HW_IRQ
	return find_irq_channel_by_pin(pinset) != nullptr;
#else
	(void)pinset;
	return false;
#endif
}

static inline void signal_poll_stop(DrdyClient *client)
{
	if (client && client->poll_stop_sem) {
		xSemaphoreGive(client->poll_stop_sem);
	}
}

static void notify_client(DrdyClient *client)
{
	if (client && client->deferred_task) {
		xTaskNotifyGive(client->deferred_task);
	}
}

static inline void notify_client_from_isr(DrdyClient *client, BaseType_t *hpw)
{
	if (client && client->deferred_task) {
		vTaskNotifyGiveFromISR(client->deferred_task, hpw);
	}
}

static DrdyClient *find_client(uint32_t pinset)
{
	for (auto &client : g_drdy_clients) {
		if (client.in_use && client.pinset == pinset) {
			return &client;
		}
	}

	return nullptr;
}

static DrdyClient *allocate_client(uint32_t pinset)
{
	if (DrdyClient *existing = find_client(pinset)) {
		return existing;
	}

	for (auto &client : g_drdy_clients) {
		if (!client.in_use) {
			client.in_use = true;
			client.pinset = pinset;
			client.callback = nullptr;
			client.callback_arg = nullptr;
			client.poll_task = nullptr;
			client.poll_run = false;
			client.poll_stop_sem = xSemaphoreCreateBinaryStatic(&client.poll_stop_sem_buffer);
			if (client.poll_stop_sem) {
				(void) xSemaphoreTake(client.poll_stop_sem, 0);
			}
			return &client;
		}
	}

	return nullptr;
}

static bool ensure_deferred_task(DrdyClient *client)
{
	if (!client) {
		return false;
	}

	if (!client->deferred_task) {
		client->deferred_task = xTaskCreateStatic(drdy_deferred_task,
					   "drdy_def",
					   sizeof(client->deferred_stack) / sizeof(StackType_t),
					   client,
					   drdy_irq_priority(),
					   client->deferred_stack,
					   &client->deferred_task_tcb);
	}

	return client->deferred_task != nullptr;
}

static void drdy_poll_task(void *param)
{
	auto *client = static_cast<DrdyClient *>(param);
	const bsp_io_port_pin_t pin = resolve_pin(client->pinset);
	bsp_io_level_t last_level = BSP_IO_LEVEL_HIGH;
	(void)R_IOPORT_PinRead(&g_ioport_ctrl, pin, &last_level);

	while (client->poll_run) {
		bsp_io_level_t level = BSP_IO_LEVEL_HIGH;

		if (R_IOPORT_PinRead(&g_ioport_ctrl, pin, &level) == FSP_SUCCESS) {
			if ((last_level == BSP_IO_LEVEL_HIGH) && (level == BSP_IO_LEVEL_LOW)) {
				uint32_t widx = client->drdy_ts_write_idx.fetch_add(1, std::memory_order_relaxed);
				client->drdy_timestamps[widx % DrdyClient::DRDY_TS_BUF_SIZE] = hrt_absolute_time();
				notify_client(client);
			}

			last_level = level;
		}

		vTaskDelay(DRDY_POLL_INTERVAL_TICKS);
	}

	client->poll_task = nullptr;
	signal_poll_stop(client);
	vTaskDelete(nullptr);
}

static void stop_poll_task(DrdyClient *client)
{
	if (client && client->poll_task) {
		client->poll_run = false;

		if (client->poll_stop_sem) {
			(void)xSemaphoreTake(client->poll_stop_sem, pdMS_TO_TICKS(100));
		}
	}
}

static void destroy_deferred_task(DrdyClient *client)
{
	if (!client || !client->deferred_task) {
		return;
	}

	TaskHandle_t handle = client->deferred_task;
	client->deferred_task = nullptr;

	if (handle == xTaskGetCurrentTaskHandle()) {
		vTaskDelete(nullptr);

	} else {
		vTaskDelete(handle);
	}
}

static int start_poll_task(DrdyClient *client)
{
	if (!client) {
		return -EINVAL;
	}

	if (client->poll_task) {
		client->poll_run = true;
		return 0;
	}

	client->poll_run = true;
	BaseType_t ok = xTaskCreate(drdy_poll_task, "drdy_poll", DRDY_POLL_STACK_DEPTH_WORDS, client,
				    drdy_poll_priority(), &client->poll_task);

	if (ok != pdPASS) {
		client->poll_run = false;
		client->poll_task = nullptr;
		return -1;
	}

	return 0;
}

static int enable_hardware_irq(DrdyClient *client)
{
	if (!client) {
		return -EINVAL;
	}

#if !RZV_DRDY_HW_IRQ
	return -ENOTSUP;
#else
	const DrdyIrqChannel *ch = find_irq_channel_by_pin(client->pinset);

	if (!ch) {
		return -ENOTSUP;
	}

	std::atomic<DrdyClient *> &slot = g_irq_client_by_channel[ch->channel];
	DrdyClient *current = slot.load(std::memory_order_acquire);

	if (current && current != client) {
		return -EBUSY;
	}

	if (!g_irq_open_by_channel[ch->channel]) {
		/* Ensure GTM0 is initialized before the first DRDY fires so that
		 * mpu_drdy_callback() sees full-resolution timestamps instead of
		 * the 1 ms FreeRTOS-tick fallback (C2 race fix). Idempotent.
		 */
		rzv_hrt_init();

		fsp_err_t err = ch->instance->p_api->open(ch->instance->p_ctrl, ch->instance->p_cfg);

		if (FSP_SUCCESS != err) {
			return -EIO;
		}

		err = ch->instance->p_api->enable(ch->instance->p_ctrl);

		if (FSP_SUCCESS != err) {
			ch->instance->p_api->close(ch->instance->p_ctrl);
			return -EIO;
		}

		g_irq_open_by_channel[ch->channel] = true;
	}

	slot.store(client, std::memory_order_release);
	stop_poll_task(client);
	PX4_DEBUG("DRDY hardware IRQ ch%lu enabled for pinset=0x%08lx",
		  (unsigned long)ch->channel, (unsigned long)client->pinset);
	return 0;
#endif /* RZV_DRDY_HW_IRQ */
}

static void disable_hardware_irq(DrdyClient *client)
{
	if (!client) {
		return;
	}

#if RZV_DRDY_HW_IRQ
	const DrdyIrqChannel *ch = find_irq_channel_by_pin(client->pinset);

	if (!ch) {
		return;
	}

	g_irq_client_by_channel[ch->channel].store(nullptr, std::memory_order_release);

	if (g_irq_open_by_channel[ch->channel]) {
		(void)ch->instance->p_api->disable(ch->instance->p_ctrl);
		(void)ch->instance->p_api->close(ch->instance->p_ctrl);
		g_irq_open_by_channel[ch->channel] = false;
	}
#endif /* RZV_DRDY_HW_IRQ */
}

} // namespace

#if RZV_DRDY_HW_IRQ
/*
 * mpu_drdy_callback(): Callback for the sensor data ready interrupt
 */
extern "C" void mpu_drdy_callback(external_irq_callback_args_t *p_args)
{
	uint64_t now = hrt_absolute_time();
	BaseType_t higher_priority_woken = pdFALSE;

	// Route to the client registered on the firing channel (3 IMUs = 3 channels).
	if (p_args && (p_args->channel < DRDY_IRQ_CHANNEL_COUNT)) {
		DrdyClient *client = g_irq_client_by_channel[p_args->channel].load(std::memory_order_acquire);

		if (client) {
			// Push timestamp into ring buffer (one slot per ISR pulse).
			uint32_t widx = client->drdy_ts_write_idx.fetch_add(1, std::memory_order_relaxed);
			client->drdy_timestamps[widx % DrdyClient::DRDY_TS_BUF_SIZE] = now;
			notify_client_from_isr(client, &higher_priority_woken);
		}
	}

	portYIELD_FROM_ISR(higher_priority_woken);
}
#endif /* RZV_DRDY_HW_IRQ */

// Pop one ISR timestamp from the ring buffer (FIFO order).
// Each DataReady() callback should call this once so the batch-completing call
// receives the timestamp of its own ISR rather than a later, overwritten value.
extern "C" uint64_t rzv_sensor_hal_pop_drdy_timestamp(uint32_t pinset)
{
	if (pinset == 0) {
		return 0;
	}

	DrdyClient *client = find_client(pinset);

	if (!client) {
		return 0;
	}

	uint32_t ridx = client->drdy_ts_read_idx.load(std::memory_order_relaxed);
	uint32_t widx = client->drdy_ts_write_idx.load(std::memory_order_acquire);

	if (ridx == widx) {
		return 0; // buffer empty (no new ISR since last pop)
	}

	uint64_t ts = client->drdy_timestamps[ridx % DrdyClient::DRDY_TS_BUF_SIZE];
	client->drdy_ts_read_idx.store(ridx + 1, std::memory_order_release);
	return ts;
}

// Legacy non-destructive peek kept for any callers outside the driver.
extern "C" uint64_t rzv_sensor_hal_get_drdy_timestamp(uint32_t pinset)
{
	if (pinset == 0) {
		return 0;
	}

	DrdyClient *client = find_client(pinset);

	if (!client) {
		return 0;
	}

	uint32_t widx = client->drdy_ts_write_idx.load(std::memory_order_acquire);
	uint32_t ridx = client->drdy_ts_read_idx.load(std::memory_order_relaxed);

	if (widx == ridx) {
		return 0;
	}

	// Peek at the oldest unconsumed entry without advancing the read index.
	return client->drdy_timestamps[ridx % DrdyClient::DRDY_TS_BUF_SIZE];
}

int px4_arch_configgpio(uint32_t pinset)
{
	(void)pinset;
	return 0;
}

int px4_arch_unconfiggpio(uint32_t pinset)
{
	(void)pinset;
	return 0;
}

bool px4_arch_gpioread(uint32_t pinset)
{
	if (pinset == 0) {
		return false;
	}

	const bsp_io_port_pin_t pin = resolve_pin(pinset);
	bsp_io_level_t level = BSP_IO_LEVEL_LOW;

	if (R_IOPORT_PinRead(&g_ioport_ctrl, pin, &level) != FSP_SUCCESS) {
		return false;
	}

	return (level == BSP_IO_LEVEL_HIGH);
}

void px4_arch_gpiowrite(uint32_t pinset, bool value)
{
	if (pinset == 0) {
		return;
	}

	const bsp_io_port_pin_t pin = resolve_pin(pinset);
	(void)R_IOPORT_PinWrite(&g_ioport_ctrl, pin, value ? BSP_IO_LEVEL_HIGH : BSP_IO_LEVEL_LOW);
}

extern "C" int rzv_sensor_hal_configure_drdy(uint32_t pinset, bool risingedge, bool fallingedge,
		bool event, xcpt_t func, void *arg)
{
	(void)risingedge;
	(void)fallingedge;

	if (pinset == 0) {
		return event ? -EINVAL : 0;
	}

	if (event) {
		DrdyClient *client = allocate_client(pinset);

		if (!client) {
			PX4_ERR("No free DRDY client slots");
			return -ENOMEM;
		}

		if (!ensure_deferred_task(client)) {
			client->in_use = false;
			return -ENOMEM;
		}

		client->callback = func;
		client->callback_arg = arg;

		if (pin_supports_hw_irq(pinset)) {
			if (enable_hardware_irq(client) == 0) {
				return 0;
			}

			PX4_WARN("DRDY hardware IRQ not available, falling back to polling");
		} else {
			const bsp_io_port_pin_t expected_pin = irq_supported_pin();

			if (expected_pin != static_cast<bsp_io_port_pin_t>(0)) {
				PX4_WARN("DRDY pinset=0x%08lx not mapped to hardware IRQ (expected 0x%08lx)",
					 (unsigned long)pinset, (unsigned long)expected_pin);
			} else {
				PX4_WARN("DRDY pinset=0x%08lx requested but no hardware IRQ pin configured via px4_spi_buses",
					 (unsigned long)pinset);
			}
		}

		if (start_poll_task(client) == 0) {
			PX4_INFO("DRDY polling enabled on pinset=0x%08lx", (unsigned long)pinset);
			return 0;
		}

		client->callback = nullptr;
		client->callback_arg = nullptr;
		destroy_deferred_task(client);
		client->in_use = false;
		return -1;
	}

	DrdyClient *client = find_client(pinset);

	if (!client) {
		return -EINVAL;
	}

	disable_hardware_irq(client);

	stop_poll_task(client);
	destroy_deferred_task(client);
	client->callback = nullptr;
	client->callback_arg = nullptr;
	client->pinset = 0;
	client->in_use = false;
	return 0;
}

int px4_arch_gpiosetevent(uint32_t pinset, bool risingedge, bool fallingedge,
			  bool event, xcpt_t func, void *arg)
{
	return sensor_hal_configure_drdy(pinset, risingedge, fallingedge, event, func, arg);
}

#endif /* __PX4_FREERTOS */
