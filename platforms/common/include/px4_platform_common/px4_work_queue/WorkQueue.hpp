/****************************************************************************
 *
 *   Copyright (c) 2019 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#pragma once

#include "WorkQueueManager.hpp"

#include <containers/BlockingList.hpp>
#include <containers/List.hpp>
#include <containers/IntrusiveQueue.hpp>
#include <px4_platform_common/atomic.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/sem.h>
#include <px4_platform_common/tasks.h>

#if defined(__PX4_FREERTOS)
#include <stdint.h>
#include <drivers/drv_hrt.h>
#include <task.h>
#endif /* __PX4_FREERTOS */
namespace px4
{

class WorkItem;

class WorkQueue : public IntrusiveSortedListNode<WorkQueue *>
{
public:
	explicit WorkQueue(const wq_config_t &wq_config);
	WorkQueue() = delete;

	~WorkQueue();

	const wq_config_t &get_config() const { return _config; }
	const char *get_name() const { return _config.name; }

	bool Attach(WorkItem *item);
	void Detach(WorkItem *item);

	void Add(WorkItem *item);
	void Remove(WorkItem *item);

	void Clear();

	void Run();

	void request_stop() { _should_exit.store(true); }

	void print_status(bool last = false);

	// WorkQueues sorted numerically by relative priority (-1 to -255)
	bool operator<=(const WorkQueue &rhs) const { return _config.relative_priority >= rhs.get_config().relative_priority; }

#if defined(__PX4_FREERTOS)
	void set_task_handle(TaskHandle_t handle) { _task_handle = handle; }
	TaskHandle_t task_handle() const { return _task_handle; }
	const char *current_work_item_name() const { return _current_work_item_name; }
#endif /* __PX4_FREERTOS */
private:

	bool should_exit() const { return _should_exit.load(); }

	inline void SignalWorkerThread();

#ifdef __PX4_NUTTX
	// In NuttX work can be enqueued from an ISR
	void work_lock() { _flags = enter_critical_section(); }
	void work_unlock() { leave_critical_section(_flags); }
	irqstate_t _flags;
#else
	// loop as the wait may be interrupted by a signal
	void work_lock() { do {} while (px4_sem_wait(&_qlock) != 0); }
	void work_unlock() { px4_sem_post(&_qlock); }
	px4_sem_t _qlock;
#endif

	IntrusiveQueue<WorkItem *>	_q;
	px4_sem_t			_process_lock;
	px4_sem_t			_exit_lock;
	const wq_config_t		&_config;
	BlockingList<WorkItem *>	_work_items;
	px4::atomic_bool		_should_exit{false};

#if defined(__PX4_FREERTOS)
	bool _stack_warned{false};
	const char *_current_work_item_name{nullptr};
	uint32_t _min_stack_bytes_seen{UINT32_MAX};
	TaskHandle_t _task_handle{nullptr};
	hrt_abstime _last_stack_sample_time{0};
#endif /* __PX4_FREERTOS */
#if defined(ENABLE_LOCKSTEP_SCHEDULER)
	int _lockstep_component {-1};
#endif // ENABLE_LOCKSTEP_SCHEDULER

};

} // namespace px4
