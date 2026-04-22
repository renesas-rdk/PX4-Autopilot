
#if defined(__PX4_FREERTOS)

#include <px4_platform_common/tasks.h>

extern "C" __EXPORT __attribute__((weak)) int px4_board_map_priority(int priority)
{
	return priority;
}

extern "C" __EXPORT int px4_task_adjust_priority(int priority)
{
	int adjusted = px4_board_map_priority(priority);

	if (adjusted < SCHED_PRIORITY_MIN) {
		adjusted = SCHED_PRIORITY_MIN;
	} else if (adjusted > SCHED_PRIORITY_MAX) {
		adjusted = SCHED_PRIORITY_MAX;
	}

	return adjusted;
}

#endif /* __PX4_FREERTOS */
