/****************************************************************************
 *
 * Copyright (c) 2025 Renesas Electronics Corporation and/or its affiliates
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 ****************************************************************************/


#if defined(__PX4_FREERTOS)

#include "rzv_fsp/dma_buffer.h"
#define MODULE_NAME "rzv_dma_buffer"

#include <FreeRTOS.h>
#include <task.h>

#include <px4_platform_common/log.h>

typedef struct dma_block_s {
	uintptr_t addr;
	size_t size;
	bool free;
	struct dma_block_s *next;
} dma_block_t;

typedef struct {
	uintptr_t start;
	uintptr_t end;
	dma_block_t *blocks;
} rzv_dma_region_t;

extern uint8_t _ncbuffer_start;
extern uint8_t _ncbuffer_end;
extern uint8_t _ncbuffer_sdram_start;
extern uint8_t _ncbuffer_sdram_end;

static rzv_dma_region_t g_regions[] = {
	{ (uintptr_t)&_ncbuffer_start,       (uintptr_t)&_ncbuffer_end,       NULL },
	{ (uintptr_t)&_ncbuffer_sdram_start, (uintptr_t)&_ncbuffer_sdram_end, NULL },
};

#define MAX_DMA_BLOCKS (256U)

static dma_block_t g_block_storage[MAX_DMA_BLOCKS];
static dma_block_t *g_free_blocks = NULL;
static bool g_block_pool_initialized = false;
static bool g_allocator_initialized = false;

static inline bool region_valid(const rzv_dma_region_t *region)
{
	return (region != NULL) && (region->start < region->end);
}

static inline uintptr_t align_up_uintptr(uintptr_t value, size_t alignment)
{
	return (value + (alignment - 1U)) & ~((uintptr_t)alignment - 1U);
}

static inline size_t sanitize_alignment(size_t alignment)
{
	if (alignment == 0U) {
		return RZV_DMA_DEFAULT_ALIGNMENT;
	}

	if ((alignment & (alignment - 1U)) != 0U) {
		size_t adjusted = RZV_DMA_DEFAULT_ALIGNMENT;

		while (adjusted < alignment) {
			adjusted <<= 1U;
		}

		return adjusted;
	}

	return alignment;
}

static void init_block_pool(void)
{
	if (g_block_pool_initialized) {
		return;
	}

	for (size_t i = 0; i < MAX_DMA_BLOCKS; i++) {
		g_block_storage[i].addr = 0;
		g_block_storage[i].size = 0;
		g_block_storage[i].free = false;
		g_block_storage[i].next = g_free_blocks;
		g_free_blocks = &g_block_storage[i];
	}

	g_block_pool_initialized = true;
}

static dma_block_t *allocate_block(void)
{
	if (g_free_blocks == NULL) {
		PX4_ERR("rzv_dma_alloc: metadata pool exhausted");
		return NULL;
	}

	dma_block_t *block = g_free_blocks;
	g_free_blocks = block->next;

	block->addr = 0;
	block->size = 0;
	block->free = false;
	block->next = NULL;
	return block;
}

static void release_block(dma_block_t *block)
{
	if (block == NULL) {
		return;
	}

	block->addr = 0;
	block->size = 0;
	block->free = false;
	block->next = g_free_blocks;
	g_free_blocks = block;
}

static dma_block_t *split_block(dma_block_t *block, size_t offset)
{
	if ((block == NULL) || (offset == 0U) || (offset >= block->size)) {
		return NULL;
	}

	dma_block_t *new_block = allocate_block();

	if (new_block == NULL) {
		return NULL;
	}

	new_block->addr = block->addr + offset;
	new_block->size = block->size - offset;
	new_block->free = block->free;
	new_block->next = block->next;

	block->size = offset;
	block->next = new_block;
	return new_block;
}

static void coalesce_forward(dma_block_t *block)
{
	while ((block != NULL) && (block->next != NULL) && block->next->free
	       && ((block->addr + block->size) == block->next->addr)) {
		dma_block_t *next = block->next;
		block->size += next->size;
		block->next = next->next;
		release_block(next);
	}
}

static void allocator_init(void)
{
	if (g_allocator_initialized) {
		return;
	}

	init_block_pool();

	for (size_t i = 0; i < (sizeof(g_regions) / sizeof(g_regions[0])); i++) {
		rzv_dma_region_t *region = &g_regions[i];

		if (!region_valid(region)) {
			continue;
		}

		dma_block_t *block = allocate_block();

		if (block == NULL) {
			PX4_ERR("rzv_dma_alloc: failed to initialise region %u", (unsigned)i);
			continue;
		}

		block->addr = region->start;
		block->size = region->end - region->start;
		block->free = true;
		block->next = NULL;

		region->blocks = block;
	}

	g_allocator_initialized = true;
}

void *rzv_dma_alloc(size_t size, size_t alignment)
{
	if (size == 0U) {
		return NULL;
	}

	alignment = sanitize_alignment(alignment);

	taskENTER_CRITICAL();
	allocator_init();

	for (size_t i = 0; i < (sizeof(g_regions) / sizeof(g_regions[0])); i++) {
		rzv_dma_region_t *region = &g_regions[i];

		if (!region_valid(region) || (region->blocks == NULL)) {
			continue;
		}

		for (dma_block_t *block = region->blocks; block != NULL; block = block->next) {
			if (!block->free) {
				continue;
			}

			const uintptr_t aligned_addr = align_up_uintptr(block->addr, alignment);
			const size_t padding = aligned_addr - block->addr;

			if (padding > block->size) {
				continue;
			}

			const size_t usable = block->size - padding;

			if (usable < size) {
				continue;
			}

			dma_block_t *candidate = block;

			if (padding > 0U) {
				dma_block_t *aligned_block = split_block(block, padding);

				if (aligned_block == NULL) {
					continue;
				}

				candidate = aligned_block;
			}

			if (candidate->size > size) {
				if (split_block(candidate, size) == NULL) {
					continue;
				}
			}

			candidate->free = false;
			taskEXIT_CRITICAL();
			return (void *)candidate->addr;
		}
	}

	taskEXIT_CRITICAL();
	PX4_ERR("rzv_dma_alloc failed: size=%zu align=%zu", size, alignment);
	return NULL;
}

void rzv_dma_free(void *ptr)
{
	if (ptr == NULL) {
		return;
	}

	uintptr_t addr = (uintptr_t)ptr;

	taskENTER_CRITICAL();
	allocator_init();

	for (size_t i = 0; i < (sizeof(g_regions) / sizeof(g_regions[0])); i++) {
		rzv_dma_region_t *region = &g_regions[i];

		if (!region_valid(region) || (region->blocks == NULL)) {
			continue;
		}

		if ((addr < region->start) || (addr >= region->end)) {
			continue;
		}

		dma_block_t *prev = NULL;

		for (dma_block_t *block = region->blocks; block != NULL; block = block->next) {
			if (block->addr == addr) {
				if (block->free) {
					PX4_ERR("rzv_dma_free: double free detected at %p", ptr);
					taskEXIT_CRITICAL();
					return;
				}

				block->free = true;
				coalesce_forward(block);

				if ((prev != NULL) && prev->free && (prev->addr + prev->size == block->addr)) {
					prev->size += block->size;
					prev->next = block->next;
					release_block(block);
					block = prev;
					coalesce_forward(block);
				}

				taskEXIT_CRITICAL();
				return;
			}

			prev = block;
		}
	}

	taskEXIT_CRITICAL();
	PX4_ERR("rzv_dma_free: invalid pointer %p", ptr);
}

bool rzv_dma_buffer_is_dma_safe(const void *ptr, size_t length)
{
	if ((ptr == NULL) || (length == 0U)) {
		return false;
	}

	uintptr_t start = (uintptr_t)ptr;
	uintptr_t end = start + length;

	if (end < start) {
		return false;
	}

	const size_t alignment = RZV_DMA_DEFAULT_ALIGNMENT;

	if ((start & (alignment - 1U)) != 0U) {
		return false;
	}

	for (size_t i = 0; i < (sizeof(g_regions) / sizeof(g_regions[0])); i++) {
		const rzv_dma_region_t *region = &g_regions[i];

		if (!region_valid(region)) {
			continue;
		}

		if ((start >= region->start) && (end <= region->end)) {
			return true;
		}
	}

	return false;
}

#endif /* __PX4_FREERTOS */
