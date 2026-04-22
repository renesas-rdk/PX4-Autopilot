/****************************************************************************
 *
 * Copyright (c) 2025 Renesas Electronics Corporation and/or its affiliates
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 ****************************************************************************/


#if defined(__PX4_FREERTOS)

#include "io_worker.h"
#include "parameters/param.h"

#include <px4_platform_common/log.h>
#include <px4_platform_common/sem.h>
#include <string.h>
#include <errno.h>
#include <inttypes.h>

#include "rpc/openamp_rpc_client.h"

using namespace time_literals;

static IOWorker *g_io_worker = nullptr;
static px4_sem_t g_queue_lock;
static bool g_lock_initialized = false;

IOWorker::IOWorker()
	: ScheduledWorkItem(MODULE_NAME "_io", px4::wq_configurations::lp_default)
{
}

IOWorker::~IOWorker()
{
	shutdown();
}

int IOWorker::init()
{
	if (!g_lock_initialized) {
		if (px4_sem_init(&g_queue_lock, 0, 1) != 0) {
			PX4_ERR("IOWorker: failed to init queue lock");
			return -1;
		}

		g_lock_initialized = true;
	}

	// Initialize statistics
	memset(&_stats, 0, sizeof(_stats));

	// Schedule first run
	ScheduleNow();

	PX4_INFO("IOWorker: initialized");
	return 0;
}

void IOWorker::shutdown()
{
	_shutting_down.store(true);

	// Wait for pending operations to complete (with timeout)
	const hrt_abstime timeout = hrt_absolute_time() + 5_s;

	while (has_pending_ops() && hrt_absolute_time() < timeout) {
		px4_usleep(100000); // 100ms
	}

	if (has_pending_ops()) {
		PX4_WARN("IOWorker: shutdown with %zu pending operations", get_pending_count());
	}

	ScheduleClear();

	PX4_INFO("IOWorker: shutdown complete");
}

int IOWorker::queue_param_save(io_completion_callback_t callback, void *user_data)
{
	return enqueue_request(IOOpType::PARAM_SAVE, nullptr, 0, callback, user_data);
}

int IOWorker::queue_log_write(const void *data, size_t data_len,
			       io_completion_callback_t callback, void *user_data)
{
	if (!data || data_len == 0) {
		return -EINVAL;
	}

	return enqueue_request(IOOpType::LOG_WRITE, data, data_len, callback, user_data);
}

int IOWorker::queue_calib_save(const void *data, size_t data_len,
				io_completion_callback_t callback, void *user_data)
{
	if (!data || data_len == 0) {
		return -EINVAL;
	}

	return enqueue_request(IOOpType::CALIB_SAVE, data, data_len, callback, user_data);
}

int IOWorker::enqueue_request(IOOpType type, const void *data, size_t data_len,
			       io_completion_callback_t callback, void *user_data)
{
	if (_shutting_down.load()) {
		return -ECANCELED;
	}

	if (type == IOOpType::PARAM_SAVE) {
		// Coalesce multiple param save requests while one is pending/inflight.
		if (_inflight_active.load()) {
			_pending_param_save.store(true);
			_pending_param_cb = callback;
			_pending_param_user = user_data;
			return 0;
		}

		// If a param save is already queued, mark a pending save after it completes.
		px4_sem_wait(&g_queue_lock);
		for (RequestNode *node = _queue_head; node; node = node->next) {
			if (node->request.type == IOOpType::PARAM_SAVE) {
				px4_sem_post(&g_queue_lock);
				_pending_param_save.store(true);
				_pending_param_cb = callback;
				_pending_param_user = user_data;
				return 0;
			}
		}
		px4_sem_post(&g_queue_lock);
	}

	// Check queue size limit
	if (_queue_count.load() >= MAX_QUEUE_SIZE) {
		PX4_WARN("IOWorker: queue full (%zu/%zu), dropping request", _queue_count.load(), MAX_QUEUE_SIZE);
		_stats.dropped_ops++;
		return -ENOSPC;
	}

	// Allocate node from pool
	RequestNode *node = alloc_request_node();

	if (!node) {
		PX4_ERR("IOWorker: failed to allocate request node");
		_stats.dropped_ops++;
		return -ENOMEM;
	}

	// Initialize request
	node->request.type = type;
	node->request.callback = callback;
	node->request.user_data = user_data;
	node->request.timestamp = hrt_absolute_time();
	node->request.retry_count = 0;
	node->next = nullptr;

	// Copy data if needed
	if (data && data_len > 0) {
		node->request.data = malloc(data_len);

		if (!node->request.data) {
			free_request_node(node);
			_stats.dropped_ops++;
			return -ENOMEM;
		}

		memcpy(node->request.data, data, data_len);
		node->request.data_len = data_len;

	} else {
		node->request.data = nullptr;
		node->request.data_len = 0;
	}

	// Add to queue (thread-safe)
	px4_sem_wait(&g_queue_lock);

	if (!_queue_head) {
		_queue_head = node;
		_queue_tail = node;

	} else {
		_queue_tail->next = node;
		_queue_tail = node;
	}

	_queue_count.fetch_add(1);
	_queue_empty.store(false);
	_stats.total_ops++;

	px4_sem_post(&g_queue_lock);

	// Trigger immediate execution
	ScheduleNow();

	return 0;
}

bool IOWorker::dequeue_request(IORequest &request)
{
	px4_sem_wait(&g_queue_lock);

	RequestNode *node = _queue_head;

	if (node) {
		_queue_head = node->next;

		if (!_queue_head) {
			_queue_tail = nullptr;
			_queue_empty.store(true);
		}

		_queue_count.fetch_sub(1);

		// Copy request data
		request = node->request;

		// Free the node (but keep request.data - caller must free)
		free_request_node(node);

		px4_sem_post(&g_queue_lock);
		return true;
	}

	px4_sem_post(&g_queue_lock);
	return false;
}

IOWorker::RequestNode *IOWorker::alloc_request_node()
{
	uint32_t free_mask = _pool_free_mask.load();

	for (size_t i = 0; i < MAX_QUEUE_SIZE; i++) {
		uint32_t bit = (1u << i);

		if (free_mask & bit) {
			// Try to atomically claim this slot
			uint32_t expected = free_mask;
			uint32_t desired = free_mask & ~bit;

			if (_pool_free_mask.compare_exchange(&expected, desired)) {
				return &_request_pool[i];
			}

			// Retry with updated free_mask
			free_mask = _pool_free_mask.load();
			i = 0; // Start over
		}
	}

	return nullptr;
}

void IOWorker::free_request_node(RequestNode *node)
{
	if (!node) {
		return;
	}

	// Find index in pool
	size_t index = node - _request_pool;

	if (index < MAX_QUEUE_SIZE) {
		// Mark slot as free
		uint32_t bit = (1u << index);
		_pool_free_mask.fetch_or(bit);
	}
}

void IOWorker::Run()
{
	if (_shutting_down.load()) {
		return;
	}

	handle_inflight_completion();

	// Process one request per run to avoid blocking for too long
	IORequest request;

	if (!dequeue_request(request)) {
		// Queue empty, sleep until next request
		return;
	}

	hrt_abstime start_time = hrt_absolute_time();
	int result = -1;

	// Execute the request
	switch (request.type) {
	case IOOpType::PARAM_SAVE:
		result = execute_param_save(request);
		break;

	case IOOpType::LOG_WRITE:
		result = execute_log_write(request.data, request.data_len);
		break;

	case IOOpType::CALIB_SAVE:
		result = execute_calib_save(request.data, request.data_len);
		break;

	default:
		PX4_ERR("IOWorker: unknown operation type %d", (int)request.type);
		result = -EINVAL;
		break;
	}

	if (result == -EINPROGRESS) {
		return;
	}

	handle_request_result(request, result, start_time);

	// Schedule next run if more work available
	if (has_pending_ops()) {
		ScheduleNow();
	}
}

void IOWorker::handle_request_result(IORequest &request, int result, hrt_abstime start_time)
{
	hrt_abstime duration = hrt_elapsed_time(&start_time);

	_stats.last_op_time = hrt_absolute_time();

	if (duration > _stats.max_op_duration) {
		_stats.max_op_duration = duration;
	}

	if (result == 0) {
		_stats.completed_ops++;
		complete_request(request, result);

		if (request.data) {
			free(request.data);
		}

		return;
	}

	if (request.retry_count < MAX_RETRY_COUNT) {
		request.retry_count++;
		_stats.retried_ops++;

		PX4_WARN("IOWorker: operation failed (%d), retry %d/%d",
			 result, request.retry_count, MAX_RETRY_COUNT);

		RequestNode *node = alloc_request_node();

		if (node) {
			node->request = request;
			node->next = nullptr;

			px4_sem_wait(&g_queue_lock);

			if (!_queue_head) {
				_queue_head = node;
				_queue_tail = node;

			} else {
				_queue_tail->next = node;
				_queue_tail = node;
			}

			_queue_count.fetch_add(1);
			_queue_empty.store(false);

			px4_sem_post(&g_queue_lock);

			ScheduleDelayed(RETRY_DELAY);
			return;
		}
	}

	_stats.failed_ops++;

	if (request.retry_count >= MAX_RETRY_COUNT) {
		PX4_ERR("IOWorker: operation failed after %d retries", MAX_RETRY_COUNT);
	}

	complete_request(request, result);

	if (request.data) {
		free(request.data);
	}

}

void IOWorker::handle_inflight_completion()
{
	/* Stale-inflight watchdog: if CA55 RPC endpoint unbound while an async
	 * param-save call was in-flight, the done callback will never fire.
	 * Detect this and force-release the lock so future saves can proceed. */
	if (_inflight_active.load() && !_inflight_done.load()) {
		extern volatile int evt_svc_unbind;
		if (evt_svc_unbind && hrt_elapsed_time(&_inflight_start) > 5_s) {
			PX4_WARN("IOWorker: inflight RPC orphaned (CA55 unbound >5s), forcing reset");
			_inflight_active.store(false);
			_pending_param_save.store(true);
			/* Leave _pending_param_cb/_pending_param_user as-is so the
			 * save is retried when CA55 comes back. */
		}
	}

	if (!_inflight_done.load()) {
		return;
	}

	IORequest request = _inflight_request;
	const int result = _inflight_result.load();
	const hrt_abstime start_time = _inflight_start;

	_inflight_done.store(false);
	_inflight_active.store(false);

	PX4_DEBUG("IOWorker: handling inflight completion (result=%d)", result);

	handle_request_result(request, result, start_time);

	// Check if there's a pending param save request
	if (request.type == IOOpType::PARAM_SAVE && _pending_param_save.load()) {
		_pending_param_save.store(false);
		io_completion_callback_t cb = _pending_param_cb;
		void *user = _pending_param_user;
		_pending_param_cb = nullptr;
		_pending_param_user = nullptr;
		PX4_DEBUG("IOWorker: triggering pending param save after completion");
		const int pending_ret = enqueue_request(IOOpType::PARAM_SAVE, nullptr, 0, cb, user);

		if (pending_ret != 0) {
			PX4_WARN("IOWorker: pending param save enqueue failed (%d)", pending_ret);
		}
	}
}

int IOWorker::execute_param_save(const IORequest &request)
{
	if (_inflight_active.load()) {
		const uint32_t pending = px4_openamp_rpc_pending_count();
		const uint32_t no_buf = px4_openamp_rpc_no_buf_count();
		PX4_WARN("IOWorker: param save skipped, already inflight (pending_rpc=%" PRIu32 " no_buf=%" PRIu32 ")",
			 pending, no_buf);
		return -EBUSY;
	}

	_inflight_request = request;
	_inflight_start = hrt_absolute_time();
	_inflight_result.store(0);
	_inflight_done.store(false);
	_inflight_active.store(true);

	/* Skip RPC immediately if CA55 endpoint is known-dead.
	 * Avoids a 2-second stall inside fs_rpc(); schedules retry for when CA55 comes back. */
	extern volatile int evt_svc_unbind;
	if (evt_svc_unbind) {
		_inflight_active.store(false);
		_pending_param_save.store(true);
		_pending_param_cb   = request.callback;
		_pending_param_user = request.user_data;
		PX4_DEBUG("IOWorker: CA55 RPC unbound, param save deferred");
		return -ENOTCONN;
	}

	const uint32_t pending_before = px4_openamp_rpc_pending_count();
	const uint32_t no_buf_before = px4_openamp_rpc_no_buf_count();
	PX4_DEBUG("IOWorker: starting async param save (pending_rpc=%" PRIu32 " no_buf=%" PRIu32 ")",
		 pending_before, no_buf_before);

	const int ret = param_save_default_async(&IOWorker::param_save_async_done, this);

	if (ret == -ENOTSUP) {
		_inflight_active.store(false);
		PX4_WARN("IOWorker: async save not supported, falling back to blocking save");
		return param_save_default(false);
	}

	if (ret == -EBUSY || ret == -EAGAIN || ret == -ENOTCONN || ret == -ETIMEDOUT) {
		_inflight_active.store(false);

		/* -ENOTCONN: CA55 RPC endpoint not ready yet (CR8 booted before CA55).
		 * -ETIMEDOUT: CA55 too busy to respond (e.g. power-supply sag under motor
		 *   load causing eMMC slowness).  Both are transient — schedule a retry
		 *   instead of silently dropping the save. */
		if (ret == -EBUSY || ret == -ENOTCONN || ret == -ETIMEDOUT) {
			_pending_param_save.store(true);
			_pending_param_cb = request.callback;
			_pending_param_user = request.user_data;
		}

		const hrt_abstime now = hrt_absolute_time();
		if (now - _last_async_busy_log > 5_s) {
			const uint32_t pending = px4_openamp_rpc_pending_count();
			const uint32_t no_buf = px4_openamp_rpc_no_buf_count();
			const uint32_t no_buf_delta = (no_buf >= no_buf_before) ? (no_buf - no_buf_before) : no_buf;
			PX4_WARN("IOWorker: async save deferred (CA55 not ready) (%d) pending_rpc=%" PRIu32 " no_buf=%" PRIu32 " no_buf_delta=%" PRIu32,
				 ret, pending, no_buf, no_buf_delta);
			_last_async_busy_log = now;
		}

		return 0;
	}

	if (ret != 0) {
		_inflight_active.store(false);
		const uint32_t pending_after = px4_openamp_rpc_pending_count();
		const uint32_t no_buf_after = px4_openamp_rpc_no_buf_count();
		PX4_ERR("IOWorker: async save start failed (%d) pending_rpc=%" PRIu32 " no_buf=%" PRIu32,
			ret, pending_after, no_buf_after);
		return ret;
	}

	return -EINPROGRESS;
}

void IOWorker::param_save_async_done(int result, void *user_data)
{
	IOWorker *self = static_cast<IOWorker *>(user_data);

	if (!self) {
		return;
	}

	const hrt_abstime duration = hrt_elapsed_time(&self->_inflight_start);

	const uint32_t pending = px4_openamp_rpc_pending_count();
	const uint32_t no_buf = px4_openamp_rpc_no_buf_count();

	if (result == 0) {
		PX4_DEBUG("IOWorker: async param save completed successfully (duration=%llu ms, pending_rpc=%" PRIu32 " no_buf=%" PRIu32 ")",
			 (unsigned long long)(duration / 1000),
			 pending, no_buf);
	} else {
		PX4_ERR("IOWorker: async param save failed (%d, duration=%llu ms, pending_rpc=%" PRIu32 " no_buf=%" PRIu32 ")",
			result,
			(unsigned long long)(duration / 1000),
			pending, no_buf);
	}

	self->_inflight_result.store(result);
	self->_inflight_done.store(true);
	self->ScheduleNow();
}

int IOWorker::execute_log_write(const void *data, size_t data_len)
{
	// TODO: Implement log write
	// For now, just return success
	(void)data;
	(void)data_len;
	return 0;
}

int IOWorker::execute_calib_save(const void *data, size_t data_len)
{
	// TODO: Implement calibration save
	// For now, just return success
	(void)data;
	(void)data_len;
	return 0;
}

void IOWorker::complete_request(const IORequest &request, int result)
{
	if (request.callback) {
		request.callback(result, request.user_data);
	}
}

IOWorker *io_worker_get_instance()
{
	return g_io_worker;
}

int io_worker_init()
{
	if (g_io_worker) {
		PX4_WARN("IOWorker: already initialized");
		return 0;
	}

	g_io_worker = new IOWorker();

	if (!g_io_worker) {
		PX4_ERR("IOWorker: failed to allocate");
		return -ENOMEM;
	}

	int ret = g_io_worker->init();

	if (ret != 0) {
		delete g_io_worker;
		g_io_worker = nullptr;
		return ret;
	}

	return 0;
}

void io_worker_shutdown()
{
	if (g_io_worker) {
		g_io_worker->shutdown();
		delete g_io_worker;
		g_io_worker = nullptr;
	}

	if (g_lock_initialized) {
		px4_sem_destroy(&g_queue_lock);
		g_lock_initialized = false;
	}
}

#endif /* __PX4_FREERTOS */
