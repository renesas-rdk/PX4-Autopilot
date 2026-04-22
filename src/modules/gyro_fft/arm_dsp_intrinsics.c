/****************************************************************************
 *
 * Copyright (c) 2025 Renesas Electronics Corporation and/or its affiliates
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 ****************************************************************************/

#if defined(__PX4_FREERTOS)

/**
 * Fallback implementations for ARM DSP intrinsics that are not provided by
 * the Renesas FSP CMSIS port. These helpers are sufficient for the CMSIS
 * DSP routines used by gyro_fft.
 */

#include <stdint.h>

int32_t __SSAT(int32_t val, uint32_t sat)
{
	if (sat >= 32) {
		return val;
	}

	const int32_t max = (1 << (sat - 1)) - 1;
	const int32_t min = -(1 << (sat - 1));

	if (val > max) {
		return max;
	}

	if (val < min) {
		return min;
	}

	return val;
}

static inline int16_t lo(uint32_t v) { return (int16_t)(v & 0xFFFF); }
static inline int16_t hi(uint32_t v) { return (int16_t)(v >> 16); }

uint32_t __SMUAD(uint32_t a, uint32_t b)
{
	return (uint32_t)((int32_t)lo(a) * lo(b) + (int32_t)hi(a) * hi(b));
}

uint32_t __SMUSD(uint32_t a, uint32_t b)
{
	return (uint32_t)((int32_t)lo(a) * lo(b) - (int32_t)hi(a) * hi(b));
}

uint32_t __SMUADX(uint32_t a, uint32_t b)
{
	return (uint32_t)((int32_t)lo(a) * hi(b) + (int32_t)hi(a) * lo(b));
}

uint32_t __SMUSDX(uint32_t a, uint32_t b)
{
	return (uint32_t)((int32_t)lo(a) * hi(b) - (int32_t)hi(a) * lo(b));
}

uint32_t __SMLAD(uint32_t a, uint32_t b, uint32_t c)
{
	return __SMUAD(a, b) + c;
}

uint32_t __SMLADX(uint32_t a, uint32_t b, uint32_t c)
{
	return __SMUADX(a, b) + c;
}

uint32_t __SMLSDX(uint32_t a, uint32_t b, uint32_t c)
{
	return c - __SMUSDX(a, b);
}

static inline uint32_t pack16(int16_t hi_val, int16_t lo_val)
{
	return ((uint32_t)(uint16_t)hi_val << 16) | ((uint32_t)(uint16_t)lo_val);
}

uint32_t __SHADD16(uint32_t a, uint32_t b)
{
	return pack16((int16_t)((hi(a) + hi(b)) >> 1), (int16_t)((lo(a) + lo(b)) >> 1));
}

uint32_t __SHSUB16(uint32_t a, uint32_t b)
{
	return pack16((int16_t)((hi(a) - hi(b)) >> 1), (int16_t)((lo(a) - lo(b)) >> 1));
}

static inline int16_t sat16(int32_t v)
{
	if (v > 0x7FFF) { return 0x7FFF; }
	if (v < -0x8000) { return -0x8000; }
	return (int16_t)v;
}

uint32_t __QADD16(uint32_t a, uint32_t b)
{
	return pack16(sat16((int32_t)hi(a) + hi(b)), sat16((int32_t)lo(a) + lo(b)));
}

uint32_t __QSUB16(uint32_t a, uint32_t b)
{
	return pack16(sat16((int32_t)hi(a) - hi(b)), sat16((int32_t)lo(a) - lo(b)));
}

uint32_t __QASX(uint32_t a, uint32_t b)
{
	return pack16(sat16((int32_t)hi(a) + lo(b)), sat16((int32_t)lo(a) - hi(b)));
}

uint32_t __QSAX(uint32_t a, uint32_t b)
{
	return pack16(sat16((int32_t)hi(a) - lo(b)), sat16((int32_t)lo(a) + hi(b)));
}

uint32_t __SHASX(uint32_t a, uint32_t b)
{
	return pack16((int16_t)((hi(a) + lo(b)) >> 1), (int16_t)((lo(a) - hi(b)) >> 1));
}

uint32_t __SHSAX(uint32_t a, uint32_t b)
{
	return pack16((int16_t)((hi(a) - lo(b)) >> 1), (int16_t)((lo(a) + hi(b)) >> 1));
}

int32_t __QADD(int32_t a, int32_t b)
{
	int64_t tmp = (int64_t)a + b;
	if (tmp > INT32_MAX) { return INT32_MAX; }
	if (tmp < INT32_MIN) { return INT32_MIN; }
	return (int32_t)tmp;
}

int32_t __QSUB(int32_t a, int32_t b)
{
	int64_t tmp = (int64_t)a - b;
	if (tmp > INT32_MAX) { return INT32_MAX; }
	if (tmp < INT32_MIN) { return INT32_MIN; }
	return (int32_t)tmp;
}

uint32_t __PKHBT(uint32_t a, uint32_t b, uint32_t shift)
{
	return (a & 0xFFFFU) | ((b << shift) & 0xFFFF0000U);
}

uint64_t __SMLALD(uint32_t a, uint32_t b, uint64_t sum)
{
	return (uint64_t)((int32_t)lo(a) * lo(b)) +
	       (uint64_t)((int32_t)hi(a) * hi(b)) +
	       sum;
}

#endif /* __PX4_FREERTOS */
