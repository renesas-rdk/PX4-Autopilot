/****************************************************************************
 *
 * Copyright (c) 2025 Renesas Electronics Corporation and/or its affiliates
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 ****************************************************************************/


#if defined(__PX4_FREERTOS)

#include <stdint.h>
#include <stdio.h>

#include <px4_platform_common/log.h>

extern "C" {
#include <FreeRTOS.h>
#include <task.h>
}

#include <rzv_fsp/hrt.h>

extern "C" int px4_systemcmd_free_handler()
{
	HeapStats_t stats{};
	vPortGetHeapStats(&stats);
	const size_t free_heap = xPortGetFreeHeapSize();
	const size_t min_free_heap = xPortGetMinimumEverFreeHeapSize();
	const size_t total_heap = configTOTAL_HEAP_SIZE;
	const size_t used_heap = total_heap - free_heap;
	const float usage_percent = (total_heap > 0) ? (static_cast<float>(used_heap) / static_cast<float>(total_heap) * 100.f) : 0.f;

	printf("\n");
	printf("FreeRTOS Heap Memory Usage:\n");
	printf("---------------------------\n");
	printf("Total heap:      %8zu bytes (%zu KB)\n", total_heap, total_heap / 1024);
	printf("Used:            %8zu bytes (%zu KB)\n", used_heap, used_heap / 1024);
	printf("Free:            %8zu bytes (%zu KB)\n", free_heap, free_heap / 1024);
	printf("Min ever free:   %8zu bytes (%zu KB)\n", min_free_heap, min_free_heap / 1024);
	printf("Largest free:    %8zu bytes (%zu KB)\n", stats.xSizeOfLargestFreeBlockInBytes, stats.xSizeOfLargestFreeBlockInBytes / 1024);
	printf("Smallest free:   %8zu bytes\n", stats.xSizeOfSmallestFreeBlockInBytes);
	printf("Free blocks:     %8zu\n", stats.xNumberOfFreeBlocks);
	printf("Allocs/Frees:    %8zu / %zu\n", stats.xNumberOfSuccessfulAllocations, stats.xNumberOfSuccessfulFrees);
	printf("Usage:           %8.1f%%\n", (double)usage_percent);
	printf("\n");

	return 0;
}

extern "C" void px4_hrt_test_dump_platform_diagnostics()
{
	rzv_hrt_diagnostics_t diag{};
	rzv_hrt_get_diagnostics(&diag);
	PX4_INFO("RZV HRT diag: isr=%u sw=%u skip=%u status_fail=%u fallback=%u monotonic=%u",
		 (unsigned)diag.isr_overflows,
		 (unsigned)diag.sw_overflows,
		 (unsigned)diag.skip_isr_hits,
		 (unsigned)diag.status_failures,
		 (unsigned)diag.fallback_calls,
		 (unsigned)diag.monotonic_adjustments);
}

extern "C" bool px4_logger_track_io_latency()
{
	return true;
}

extern "C" uint64_t px4_logger_slow_write_threshold_us()
{
	return 50000;
}

extern "C" uint64_t px4_logger_slow_fsync_threshold_us()
{
	return 100000;
}

#endif /* __PX4_FREERTOS */
