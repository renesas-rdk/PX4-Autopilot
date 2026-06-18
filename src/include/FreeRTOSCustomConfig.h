/* Shim: forward to the project-level FreeRTOS configuration overrides */
#pragma once
#ifdef __PX4_FREERTOS
#include "../../../src/px4_freertos/FreeRTOSCustomConfig.h"
#endif /* __PX4_FREERTOS */
