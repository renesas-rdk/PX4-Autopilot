/****************************************************************************
 *
 *   Copyright (c) 2015-2020 PX4 Development Team. All rights reserved.
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

/**
 * @file print_load.cpp
 *
 * Print the current system load.
 *
 * @author Lorenz Meier <lorenz@px4.io>
 */

#include <px4_platform_common/posix.h>

#include <unistd.h>
#include <string.h>
#include <stdio.h>
#include <stdbool.h>

#if defined(__PX4_FREERTOS)
#include <algorithm>
#include <array>
#include <cmath>
#include <vector>
#include <px4_platform/cpuload.h>
#endif /* __PX4_FREERTOS */
#include <px4_platform_common/log.h>
#include <px4_platform_common/printload.h>
#include <drivers/drv_hrt.h>

#ifdef __PX4_DARWIN
#include <mach/mach.h>
#endif

#ifdef __PX4_QURT
// dprintf is not available on QURT. Use the usual output to mini-dm.
#define dprintf(_fd, _text, ...) ((_fd) == 1 ? PX4_INFO((_text), ##__VA_ARGS__) : (void)(_fd))
#endif

#if defined(__PX4_FREERTOS)
extern "C" {
#include <FreeRTOS.h>
#include <task.h>
}
#else
extern struct system_load_s system_load;
#endif /* __PX4_FREERTOS */

#define CL "\033[K" // clear line

void init_print_load(struct print_load_s *s)
{
	s->total_user_time = 0;

	s->running_count = 0;
	s->blocked_count = 0;

	s->new_time = hrt_absolute_time();
	s->interval_start_time = s->new_time;

#if defined(__PX4_FREERTOS)
	for (size_t i = 0; i < CONFIG_FS_PROCFS_MAX_TASKS; i++) {
		s->last_times[i] = 0;
		s->task_numbers[i] = 0;
	}

	s->idle_time_us_total = 0;
	cpuload_monitor_start();
#else
	for (size_t i = 0; i < sizeof(s->last_times) / sizeof(s->last_times[0]); i++) {
		s->last_times[i] = 0;
	}
#endif /* __PX4_FREERTOS */

	s->interval_time_us = 0.f;
}
#if defined(__PX4_FREERTOS)

# if defined(__GNUC__)
#  pragma GCC diagnostic push
#  pragma GCC diagnostic ignored "-Wpsabi"
# endif
namespace
{

static constexpr int kStdoutFd = 1;

const char *task_state_name(eTaskState state)
{
	switch (state) {
	case eRunning:    return "RUN";
	case eReady:      return "READY";
	case eBlocked:    return "BLOCK";
	case eSuspended:  return "SUSP";
	case eDeleted:    return "DEL";
	case eInvalid:    return "INVAL";

	default:          return "UNK";
	}
}

size_t find_or_allocate_slot(print_load_s *state, uint32_t task_number)
{
	for (size_t i = 0; i < CONFIG_FS_PROCFS_MAX_TASKS; ++i) {
		if (state->task_numbers[i] == task_number) {
			return i;
		}
	}

	for (size_t i = 0; i < CONFIG_FS_PROCFS_MAX_TASKS; ++i) {
		if (state->task_numbers[i] == 0) {
			state->task_numbers[i] = task_number;
			state->last_times[i] = 0;
			return i;
		}
	}

	const size_t slot = task_number % CONFIG_FS_PROCFS_MAX_TASKS;
	state->task_numbers[slot] = task_number;
	state->last_times[slot] = 0;
	return slot;
}

struct TaskMetrics {
	const TaskStatus_t *status{nullptr};
	uint32_t task_number{0};
	double cpu_ms{0.0};
	double cpu_percent{0.0};
	double cpu_time_us{0.0};
	uint32_t stack_free_bytes{0};
	bool is_idle{false};
};

struct print_load_callback_data_s {
	int fd{0};
	char buffer[160] {};
};

void print_load_callback(void *user)
{
	auto *data = static_cast<print_load_callback_data_s *>(user);
	char clear_line[] {CL};

	if (data->fd != kStdoutFd) {
		clear_line[0] = '\0';
	}

	dprintf(data->fd, "%s%s\n", clear_line, data->buffer);
}

} // namespace

void print_load_buffer(char *buffer, int buffer_length, print_load_callback_f cb, void *user,
		       struct print_load_s *print_state)
{
	if ((buffer == nullptr) || (buffer_length <= 0) || (cb == nullptr) || (print_state == nullptr)) {
		return;
	}

	print_state->new_time = hrt_absolute_time();

	if (print_state->interval_start_time == 0) {
		print_state->interval_start_time = print_state->new_time;
	}

	const UBaseType_t reported_tasks = uxTaskGetNumberOfTasks();

	if (reported_tasks == 0) {
		snprintf(buffer, buffer_length, "No tasks available");
		cb(user);
		print_state->interval_start_time = print_state->new_time;
		return;
	}

	std::vector<TaskStatus_t> task_status(reported_tasks);
	configRUN_TIME_COUNTER_TYPE total_runtime_counter = 0;
	UBaseType_t populated = uxTaskGetSystemState(task_status.data(), reported_tasks, &total_runtime_counter);

	if (populated == 0) {
		snprintf(buffer, buffer_length, "Unable to read task state");
		cb(user);
		print_state->interval_start_time = print_state->new_time;
		return;
	}

	if (populated > task_status.size()) {
		populated = task_status.size();
	}

	task_status.resize(populated);

	if (task_status.size() > CONFIG_FS_PROCFS_MAX_TASKS) {
		task_status.resize(CONFIG_FS_PROCFS_MAX_TASKS);
	}

	std::array<bool, CONFIG_FS_PROCFS_MAX_TASKS> slot_seen{};
	std::vector<uint64_t> task_deltas(task_status.size(), 0);

	uint64_t total_delta_counter = 0;

	const TaskHandle_t idle_handle = xTaskGetIdleTaskHandle();

	for (size_t i = 0; i < task_status.size(); ++i) {
		const TaskStatus_t &ts = task_status[i];
		const uint32_t task_number = static_cast<uint32_t>(ts.xTaskNumber);
		const size_t slot = find_or_allocate_slot(print_state, task_number);
		slot_seen[slot] = true;

		const uint64_t previous = print_state->last_times[slot];
		const uint64_t current = static_cast<uint64_t>(ts.ulRunTimeCounter);

		const uint64_t delta = (current >= previous) ? (current - previous) : current;

		task_deltas[i] = delta;
		total_delta_counter += delta;

		print_state->last_times[slot] = current;
		print_state->task_numbers[slot] = task_number;
	}

	for (size_t i = 0; i < CONFIG_FS_PROCFS_MAX_TASKS; ++i) {
		if (!slot_seen[i]) {
			print_state->task_numbers[i] = 0;
			print_state->last_times[i] = 0;
		}
	}

	double interval_time_us = 0.0;

	if (print_state->new_time > print_state->interval_start_time) {
		interval_time_us = static_cast<double>(print_state->new_time - print_state->interval_start_time);
	}

	print_state->interval_time_us = static_cast<float>(interval_time_us);

	double microseconds_per_counter = 0.0;

	if (total_delta_counter > 0 && interval_time_us > 0.0) {
		microseconds_per_counter = interval_time_us / static_cast<double>(total_delta_counter);
	}

	print_state->running_count = 0;
	print_state->blocked_count = 0;

	double total_task_time_us = 0.0;
	double idle_time_us = 0.0;

	std::vector<TaskMetrics> metrics;
	metrics.reserve(task_status.size());

	for (size_t i = 0; i < task_status.size(); ++i) {
		const TaskStatus_t &ts = task_status[i];

		const double cpu_time_us = task_deltas[i] * microseconds_per_counter;
		const double cpu_ms = cpu_time_us / 1000.0;
		const double cpu_percent = (interval_time_us > 0.0) ? (cpu_time_us * 100.0 / interval_time_us) : 0.0;

		const uint32_t stack_free_bytes = static_cast<uint32_t>(ts.usStackHighWaterMark) * sizeof(StackType_t);

		switch (ts.eCurrentState) {
		case eRunning:
		case eReady:
			++print_state->running_count;
			break;

		case eBlocked:
		case eSuspended:
			++print_state->blocked_count;
			break;

		default:
			break;
		}

		const bool is_idle = (ts.xHandle == idle_handle);

		if (is_idle) {
			idle_time_us += cpu_time_us;

		} else {
			total_task_time_us += cpu_time_us;
		}

		TaskMetrics metric{};
		metric.status = &ts;
		metric.task_number = static_cast<uint32_t>(ts.xTaskNumber);
		metric.cpu_ms = cpu_ms;
		metric.cpu_percent = cpu_percent;
		metric.cpu_time_us = cpu_time_us;
		metric.stack_free_bytes = stack_free_bytes;
		metric.is_idle = is_idle;
		metrics.push_back(metric);
	}

	float idle_load = 0.f;
	float task_load = 0.f;
	float sched_load = 0.f;

	if (interval_time_us > 0.0) {
		idle_load = static_cast<float>(idle_time_us / interval_time_us);
		task_load = static_cast<float>(total_task_time_us / interval_time_us);

		const float remainder = 1.f - idle_load - task_load;
		sched_load = remainder > 0.f ? remainder : 0.f;
	}

	print_state->total_user_time = static_cast<uint64_t>(total_task_time_us + 0.5);

	if (idle_time_us > 0.0) {
		print_state->idle_time_us_total += static_cast<uint64_t>(idle_time_us + 0.5);
	}

	std::stable_sort(metrics.begin(), metrics.end(), [](const TaskMetrics &a, const TaskMetrics &b) {
		if (a.is_idle != b.is_idle) {
			return !a.is_idle;
		}

		if (std::fabs(a.cpu_percent - b.cpu_percent) > 1e-3) {
			return a.cpu_percent > b.cpu_percent;
		}

		return a.task_number < b.task_number;
	});

	snprintf(buffer, buffer_length, "%4s %-16s %8s %7s %11s %10s %-8s",
		 "ID", "TASK", "CPU(ms)", "CPU(%)", "STACK_MIN", "PRIO(BASE)", "STATE");
	cb(user);

	for (const TaskMetrics &metric : metrics) {
		const TaskStatus_t &ts = *metric.status;

		const char *name = (ts.pcTaskName && ts.pcTaskName[0] != '\0') ? ts.pcTaskName : "<unnamed>";
#if defined(configUSE_MUTEXES) && (configUSE_MUTEXES == 1)
		const unsigned long base_priority = static_cast<unsigned long>(ts.uxBasePriority);
#else
		const unsigned long base_priority = static_cast<unsigned long>(ts.uxCurrentPriority);
#endif

		snprintf(buffer, buffer_length,
			 "%4lu %-16.16s %8.2f %7.2f %11lu %5lu (%5lu) %-8s",
			 static_cast<unsigned long>(metric.task_number),
			 name,
			 metric.cpu_ms,
			 metric.cpu_percent,
			 static_cast<unsigned long>(metric.stack_free_bytes),
			 static_cast<unsigned long>(ts.uxCurrentPriority),
			 base_priority,
			 task_state_name(ts.eCurrentState));
		cb(user);
	}

	buffer[0] = '\0';
	cb(user);

	snprintf(buffer, buffer_length, "Processes: %u total, %d running, %d sleeping",
		 static_cast<unsigned>(metrics.size()),
		 print_state->running_count,
		 print_state->blocked_count);
	cb(user);

	snprintf(buffer, buffer_length, "CPU usage: %.2f%% tasks, %.2f%% sched, %.2f%% idle",
		 static_cast<double>(task_load * 100.f),
		 static_cast<double>(sched_load * 100.f),
		 static_cast<double>(idle_load * 100.f));
	cb(user);

	snprintf(buffer, buffer_length, "Uptime: %.3fs total, %.3fs idle",
		 static_cast<double>(print_state->new_time) / 1e6,
		 static_cast<double>(print_state->idle_time_us_total) / 1e6);
	cb(user);

	print_state->interval_start_time = print_state->new_time;
}

void print_load(int fd, struct print_load_s *print_state)
{
	if (fd == kStdoutFd) {
		dprintf(fd, "\033[H"); // move cursor home and clear screen
	}

	print_load_callback_data_s data{};
	data.fd = fd;

	print_load_buffer(data.buffer, sizeof(data.buffer), print_load_callback, &data, print_state);
}
# if defined(__GNUC__)
#  pragma GCC diagnostic pop
# endif

#else // !__PX4_FREERTOS

void print_load(int fd, struct print_load_s *print_state)
{
	char clear_line[] = CL;

	/* print system information */
	if (fd == 1) {
		dprintf(fd, "\033[H"); /* move cursor home and clear screen */

	} else {
		memset(clear_line, 0, sizeof(clear_line));
	}

#if defined(__PX4_LINUX) || defined(__PX4_CYGWIN) || defined(__PX4_QURT)
	dprintf(fd, "%sTOP NOT IMPLEMENTED ON LINUX, QURT, WINDOWS (ONLY ON NUTTX, APPLE)\n", clear_line);

#elif defined(__PX4_DARWIN)
	pid_t pid = getpid();   //-- this is the process id you need info for
	task_t task_handle;
	task_for_pid(mach_task_self(), pid, &task_handle);

	task_info_data_t tinfo;
	mach_msg_type_number_t th_info_cnt;

	th_info_cnt = TASK_INFO_MAX;
	kern_return_t kr = task_info(task_handle, TASK_BASIC_INFO, (task_info_t)tinfo, &th_info_cnt);

	if (kr != KERN_SUCCESS) {
		return;
	}

	thread_array_t thread_list;
	mach_msg_type_number_t th_cnt;

	thread_info_data_t th_info_data;
	mach_msg_type_number_t thread_info_count;

	thread_basic_info_t basic_info_th;

	// get all threads of the PX4 main task
	kr = task_threads(task_handle, &thread_list, &th_cnt);

	if (kr != KERN_SUCCESS) {
		PX4_WARN("ERROR getting thread list");
		return;
	}

	long tot_sec = 0;
	long tot_usec = 0;
	long tot_cpu = 0;

	dprintf(fd, "%sThreads: %d total\n",
		clear_line,
		th_cnt);

	for (unsigned j = 0; j < th_cnt; j++) {
		thread_info_count = THREAD_INFO_MAX;
		kr = thread_info(thread_list[j], THREAD_BASIC_INFO,
				 (thread_info_t)th_info_data, &thread_info_count);

		if (kr != KERN_SUCCESS) {
			PX4_WARN("ERROR getting thread info");
			continue;
		}

		basic_info_th = (thread_basic_info_t)th_info_data;

		if (!(basic_info_th->flags & TH_FLAGS_IDLE)) {
			tot_sec = tot_sec + basic_info_th->user_time.seconds + basic_info_th->system_time.seconds;
			tot_usec = tot_usec + basic_info_th->system_time.microseconds + basic_info_th->system_time.microseconds;
			tot_cpu = tot_cpu + basic_info_th->cpu_usage;
		}

		// char tname[128];

		// int ret = pthread_getname_np(pthread_t *thread,
		//                      const char *name, size_t len);

		dprintf(fd, "thread %d\t\t %d\n", j, basic_info_th->cpu_usage);
	}

	kr = vm_deallocate(mach_task_self(), (vm_offset_t)thread_list,
			   th_cnt * sizeof(thread_t));

	if (kr != KERN_SUCCESS) {
		PX4_WARN("ERROR cleaning up thread info");
		return;
	}

#endif
}

void print_load_buffer(char *buffer, int buffer_length, print_load_callback_f cb, void *user,
		       struct print_load_s *print_state)
{

}
#endif // __PX4_FREERTOS
