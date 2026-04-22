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

#include <px4_platform_common/px4_work_queue/WorkItemSingleShot.hpp>

#if defined(__PX4_FREERTOS)
#include <errno.h>
#include <time.h>
#endif /* __PX4_FREERTOS */

namespace px4
{

WorkItemSingleShot::WorkItemSingleShot(const px4::wq_config_t &config, worker_method method, void *argument)
	: px4::WorkItem("<single_shot>", config), _argument(argument), _method(method)
{
	px4_sem_init(&_sem, 0, 0);
	px4_sem_setprotocol(&_sem, SEM_PRIO_NONE);
}

WorkItemSingleShot::WorkItemSingleShot(const px4::WorkItem &work_item, worker_method method, void *argument)
	: px4::WorkItem("<single_shot>", work_item), _argument(argument), _method(method)
{
	px4_sem_init(&_sem, 0, 0);
}

WorkItemSingleShot::~WorkItemSingleShot()
{
	px4_sem_destroy(&_sem);
}

void WorkItemSingleShot::wait()
{
	while (px4_sem_wait(&_sem) != 0) {}
}

#if defined(__PX4_FREERTOS)
bool WorkItemSingleShot::wait_for(uint32_t timeout_ms)
{
	if (timeout_ms == 0) {
		while (px4_sem_wait(&_sem) != 0) {}
		return true;
	}

	struct timespec ts {};

	if (clock_gettime(CLOCK_MONOTONIC, &ts) != 0) {
		while (px4_sem_wait(&_sem) != 0) {}
		return true;
	}

	ts.tv_sec += timeout_ms / 1000u;
	ts.tv_nsec += (timeout_ms % 1000u) * 1000000u;

	if (ts.tv_nsec >= 1000000000l) {
		ts.tv_sec += 1;
		ts.tv_nsec -= 1000000000l;
	}

	while (px4_sem_timedwait(&_sem, &ts) != 0) {
		if (errno == ETIMEDOUT) {
			return false;
		}
	}

	return true;
}
#endif /* __PX4_FREERTOS */

void WorkItemSingleShot::Run()
{
	_method(_argument);
	px4_sem_post(&_sem);
}


} // namespace px4
