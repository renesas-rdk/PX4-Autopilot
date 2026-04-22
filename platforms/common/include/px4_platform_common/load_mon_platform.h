#pragma once

#if defined(__PX4_FREERTOS)


#include <stdint.h>
#include <stdbool.h>

#include <px4_platform_common/defines.h>
#include <uORB/topics/cpuload.h>

typedef struct px4_load_mon_platform_state_s {
	uint32_t last_total_runtime_counter;
	uint32_t last_idle_runtime_counter;
} px4_load_mon_platform_state_t;

__BEGIN_DECLS
__EXPORT bool px4_load_mon_platform_cpuload(px4_load_mon_platform_state_t *state, cpuload_s *cpuload);
__END_DECLS

#endif /* __PX4_FREERTOS */
