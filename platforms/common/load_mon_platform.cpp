
#if defined(__PX4_FREERTOS)

#include <px4_platform_common/load_mon_platform.h>

extern "C" __EXPORT __attribute__((weak)) bool px4_load_mon_platform_cpuload(px4_load_mon_platform_state_t *state,
		cpuload_s *cpuload)
{
	(void)state;
	(void)cpuload;
	return false;
}

#endif /* __PX4_FREERTOS */
