/****************************************************************************
 *
 * Copyright (c) 2025 Renesas Electronics Corporation and/or its affiliates
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 ****************************************************************************/

#pragma once

#if defined(__PX4_FREERTOS)


#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Default alignment used for DMA safe buffers on RZ/V platforms.
 * This matches the cache line size required by the FSP DMA engine.
 */
#define RZV_DMA_DEFAULT_ALIGNMENT (32U)

/**
 * Allocate a DMA-safe buffer located in the non-cached memory region.
 *
 * @param size      Number of bytes to allocate (must be > 0).
 * @param alignment Alignment in bytes. If zero, RZV_DMA_DEFAULT_ALIGNMENT is used.
 *                  Alignment is rounded up to the next power-of-two.
 * @return Pointer to DMA-safe memory on success, or NULL if the allocation fails.
 */
void *rzv_dma_alloc(size_t size, size_t alignment);

/**
 * Release a DMA buffer previously obtained via rzv_dma_alloc().
 *
 * @param ptr Pointer returned by rzv_dma_alloc().
 *
 * The memory is returned to the allocator and can be reused for future
 * allocations. Passing NULL is safe and is treated as a no-op.
 */
void rzv_dma_free(void *ptr);

/**
 * Check if the provided buffer lies entirely within the DMA-safe non-cached regions
 * and satisfies the alignment requirements for the DMA engine.
 *
 * @param ptr    Pointer to the buffer.
 * @param length Buffer length in bytes.
 * @return true if the buffer can be used directly for DMA transfers, false otherwise.
 */
bool rzv_dma_buffer_is_dma_safe(const void *ptr, size_t length);

#ifdef __cplusplus
}

#endif /* __PX4_FREERTOS */

#endif
