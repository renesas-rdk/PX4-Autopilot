/* Shim: forward to the project-level FreeRTOS configuration overrides */
#pragma once
#ifdef __PX4_FREERTOS
#include "../../../src/freertos_posix/FreeRTOSCustomConfig.h"
#endif /* __PX4_FREERTOS */
