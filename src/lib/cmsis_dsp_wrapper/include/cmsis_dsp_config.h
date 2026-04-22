/**
 * @file cmsis_dsp_config.h
 * @brief Configuration header for CMSIS-DSP wrapper library
 *
 * Platform-specific defines and feature detection for ARM CMSIS-DSP integration
 */

#pragma once

#if defined(__PX4_FREERTOS)


#include <stdint.h>
#include <stdbool.h>

// CMSIS-DSP library headers
#include "arm_math.h"
#include "arm_math_types.h"

/**
 * Platform detection
 */
#if defined(__ARM_ARCH)
    #define CMSIS_DSP_AVAILABLE 1
#else
    #define CMSIS_DSP_AVAILABLE 0
    #warning "CMSIS-DSP wrapper built on non-ARM platform - DSP functions disabled"
#endif

/**
 * Architecture-specific optimizations
 */
#if defined(__ARM_ARCH_7EM__) || defined(__ARM_ARCH_8M_MAIN__)
    // Cortex-M4/M7/M33 - DSP extension available
    #define CMSIS_DSP_HAS_DSP_EXT 1
#elif defined(__ARM_ARCH_7R__) || defined(__ARM_ARCH_8R__)
    // Cortex-R series - compatible with CM7 optimizations
    #define CMSIS_DSP_HAS_DSP_EXT 1
#else
    #define CMSIS_DSP_HAS_DSP_EXT 0
#endif

/**
 * FPU support detection
 */
#if defined(__ARM_FP) && (__ARM_FP >= 4)
    #define CMSIS_DSP_HAS_FPU 1
#else
    #define CMSIS_DSP_HAS_FPU 0
#endif

/**
 * NEON support (Cortex-A)
 * Note: Not available on Cortex-R8
 */
#if defined(__ARM_NEON) || defined(__ARM_NEON__)
    #define CMSIS_DSP_HAS_NEON 1
#else
    #define CMSIS_DSP_HAS_NEON 0
#endif

/**
 * Helium/MVE support (Cortex-M55/M85)
 * Note: Not available on Cortex-R8
 */
#if defined(__ARM_FEATURE_MVE)
    #define CMSIS_DSP_HAS_HELIUM 1
#else
    #define CMSIS_DSP_HAS_HELIUM 0
#endif

/**
 * Validation and debugging
 */
#ifdef NDEBUG
    #define CMSIS_DSP_VALIDATION_ENABLED 0
#else
    #define CMSIS_DSP_VALIDATION_ENABLED 1
#endif

/**
 * Performance monitoring
 */
#define CMSIS_DSP_PERF_MONITORING 1

/**
 * Error tolerance for numerical validation
 */
#define CMSIS_DSP_FILTER_ERROR_TOLERANCE    (1e-5f)
#define CMSIS_DSP_QUATERNION_ERROR_TOLERANCE (1e-6f)
#define CMSIS_DSP_MATRIX_ERROR_TOLERANCE    (1e-4f)

/**
 * Platform capabilities summary
 */
static inline void cmsis_dsp_print_capabilities(void)
{
#if CMSIS_DSP_AVAILABLE
    // Platform info can be logged here
#endif
}

#endif /* __PX4_FREERTOS */
