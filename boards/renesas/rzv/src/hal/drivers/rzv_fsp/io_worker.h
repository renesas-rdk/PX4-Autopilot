/****************************************************************************
 *
 * Copyright (c) 2025 Renesas Electronics Corporation and/or its affiliates
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 ****************************************************************************/

/**
 * @file io_worker.h
 *
 * Asynchronous I/O worker for non-blocking file operations.
 *
 * This module provides a low-priority work queue for file I/O operations
 * to prevent blocking critical tasks (MAVLink, flight control, etc.) when
 * file operations are slow (e.g., SD card writes, remote file systems).
 *
 * Key features:
 * - Non-blocking API - callers don't wait for I/O completion
 * - Low priority execution - doesn't interfere with critical tasks
 * - Callback support - notification when operation completes
 * - Automatic retry on transient failures
 */

#pragma once

#if defined(__PX4_FREERTOS)


#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <px4_platform_common/atomic.h>
#include <drivers/drv_hrt.h>

/**
 * I/O operation types
 */
enum class IOOpType : uint8_t {
	PARAM_SAVE,    // Save parameters to file
	LOG_WRITE,     // Write log entry
	CALIB_SAVE,    // Save calibration data
};

/**
 * I/O operation completion callback
 *
 * @param result 0 on success, negative error code on failure
 * @param user_data User-provided context pointer
 */
typedef void (*io_completion_callback_t)(int result, void *user_data);

/**
 * I/O operation request
 */
struct IORequest {
	IOOpType type;
	void *data;
	size_t data_len;
	io_completion_callback_t callback;
	void *user_data;
	hrt_abstime timestamp;  // When request was queued
	uint8_t retry_count;    // Number of retry attempts
};

/**
 * Asynchronous I/O worker
 *
 * Handles file I/O operations in a low-priority work queue to prevent
 * blocking critical tasks. Operations are queued and executed asynchronously.
 */
class IOWorker : public px4::ScheduledWorkItem
{
public:
	IOWorker();
	~IOWorker() override;

	/**
	 * Initialize the I/O worker
	 * Must be called before using any other functions.
	 *
	 * @return 0 on success, negative error code on failure
	 */
	int init();

	/**
	 * Shutdown the I/O worker
	 * Waits for pending operations to complete.
	 */
	void shutdown();

	/**
	 * Queue a parameter save operation (non-blocking)
	 *
	 * @param callback Optional callback to notify completion
	 * @param user_data User context passed to callback
	 * @return 0 on success (queued), negative error code on failure
	 */
	int queue_param_save(io_completion_callback_t callback = nullptr, void *user_data = nullptr);

	/**
	 * Queue a log write operation (non-blocking)
	 *
	 * @param data Log data to write
	 * @param data_len Length of log data
	 * @param callback Optional callback to notify completion
	 * @param user_data User context passed to callback
	 * @return 0 on success (queued), negative error code on failure
	 */
	int queue_log_write(const void *data, size_t data_len,
			    io_completion_callback_t callback = nullptr, void *user_data = nullptr);

	/**
	 * Queue a calibration save operation (non-blocking)
	 *
	 * @param data Calibration data to save
	 * @param data_len Length of calibration data
	 * @param callback Optional callback to notify completion
	 * @param user_data User context passed to callback
	 * @return 0 on success (queued), negative error code on failure
	 */
	int queue_calib_save(const void *data, size_t data_len,
			     io_completion_callback_t callback = nullptr, void *user_data = nullptr);

	/**
	 * Check if I/O worker has pending operations
	 *
	 * @return true if operations are pending
	 */
	bool has_pending_ops() const { return !_queue_empty.load() || _inflight_active.load(); }

	/**
	 * Get number of pending operations
	 *
	 * @return Number of operations in queue
	 */
	size_t get_pending_count() const { return _queue_count.load() + (_inflight_active.load() ? 1 : 0); }

	/**
	 * Get statistics
	 */
	struct Stats {
		uint32_t total_ops;
		uint32_t completed_ops;
		uint32_t failed_ops;
		uint32_t retried_ops;
		uint32_t dropped_ops;  // Queue full
		hrt_abstime last_op_time;
		hrt_abstime max_op_duration;
	};

	const Stats &get_stats() const { return _stats; }

protected:
	void Run() override;

private:
	static constexpr size_t MAX_QUEUE_SIZE = 2;   // Maximum pending operations (reduced for memory - 384 bytes saved)
	static constexpr uint8_t MAX_RETRY_COUNT = 3; // Maximum retry attempts
	static constexpr hrt_abstime RETRY_DELAY = 500000; // 500ms retry delay

	struct RequestNode {
		IORequest request;
		RequestNode *next;
	};

	// Queue management
	RequestNode *_queue_head{nullptr};
	RequestNode *_queue_tail{nullptr};
	px4::atomic<size_t> _queue_count{0};
	px4::atomic_bool _queue_empty{true};
	px4::atomic_bool _shutting_down{false};
	px4::atomic_bool _inflight_active{false};
	px4::atomic_bool _inflight_done{false};
	px4::atomic<int> _inflight_result{0};
	IORequest _inflight_request{};
	hrt_abstime _inflight_start{0};
	px4::atomic_bool _pending_param_save{false};
	io_completion_callback_t _pending_param_cb{nullptr};
	void *_pending_param_user{nullptr};
	hrt_abstime _last_async_busy_log{0};

	// Memory pool for requests (pre-allocated to avoid malloc in queue)
	RequestNode _request_pool[MAX_QUEUE_SIZE];
	px4::atomic<uint32_t> _pool_free_mask{(1u << MAX_QUEUE_SIZE) - 1}; // Bitmask of free slots

	// Statistics
	Stats _stats{};

	// Private methods
	int enqueue_request(IOOpType type, const void *data, size_t data_len,
			    io_completion_callback_t callback, void *user_data);

	bool dequeue_request(IORequest &request);

	RequestNode *alloc_request_node();
	void free_request_node(RequestNode *node);

	int execute_param_save(const IORequest &request);
	int execute_log_write(const void *data, size_t data_len);
	int execute_calib_save(const void *data, size_t data_len);

	void complete_request(const IORequest &request, int result);
	void handle_inflight_completion();
	void handle_request_result(IORequest &request, int result, hrt_abstime start_time);
	static void param_save_async_done(int result, void *user_data);
};

/**
 * Get global I/O worker instance
 *
 * @return Pointer to global I/O worker, or nullptr if not initialized
 */
IOWorker *io_worker_get_instance();

/**
 * Initialize global I/O worker
 * Called by param_init()
 *
 * @return 0 on success, negative error code on failure
 */
int io_worker_init();

/**
 * Shutdown global I/O worker
 * Called on system shutdown
 */
void io_worker_shutdown();

#endif /* __PX4_FREERTOS */
