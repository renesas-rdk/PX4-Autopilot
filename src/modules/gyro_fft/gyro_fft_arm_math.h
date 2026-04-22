/****************************************************************************
 *
 * Copyright (c) 2025 Renesas Electronics Corporation and/or its affiliates
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 ****************************************************************************/


#if defined(__PX4_FREERTOS)
#pragma once

/*
 * Lightweight wrapper around CMSIS DSP headers that provides ISR-safe
 * helpers on FreeRTOS/FSP builds and falls back to direct CMSIS calls
 * elsewhere.
 */

#include "cmsis_fsp_compat.h"

#include "arm_math.h"
#include "arm_const_structs.h"

#define GYRO_FFT_SAFE_ARM_RFFT_Q15(S, pSrc, pDst) \
	do { \
		GYRO_FFT_ISR_SAFE_ENTER(); \
		arm_rfft_q15((S), (pSrc), (pDst)); \
		GYRO_FFT_ISR_SAFE_EXIT(); \
	} while (0)

#define GYRO_FFT_SAFE_ARM_MULT_Q15(pSrcA, pSrcB, pDst, blockSize) \
	do { \
		GYRO_FFT_ISR_SAFE_ENTER(); \
		arm_mult_q15((pSrcA), (pSrcB), (pDst), (blockSize)); \
		GYRO_FFT_ISR_SAFE_EXIT(); \
	} while (0)

#define GYRO_FFT_SAFE_ARM_FLOAT_TO_Q15(pSrc, pDst, blockSize) \
	do { \
		GYRO_FFT_ISR_SAFE_ENTER(); \
		arm_float_to_q15((pSrc), (pDst), (blockSize)); \
		GYRO_FFT_ISR_SAFE_EXIT(); \
	} while (0)

#endif /* __PX4_FREERTOS */
