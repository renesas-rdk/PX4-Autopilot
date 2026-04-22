/****************************************************************************
 *
 * Copyright (c) 2025 Renesas Electronics Corporation and/or its affiliates
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 ****************************************************************************/

#if defined(__PX4_FREERTOS)
/*
 * CMSIS FSP Compatibility Header for RZV2H CR8 with FreeRTOS+POSIX
 *
 * This header provides compatibility when using gyro_fft with FSP CMSIS
 * to avoid duplicate definitions between CMSIS_5 and FSP CMSIS headers.
 */

#ifndef CMSIS_FSP_COMPAT_H
#define CMSIS_FSP_COMPAT_H

/* Prevent inclusion of conflicting CMSIS Core headers */
#ifndef __CMSIS_GENERIC
#define __CMSIS_GENERIC
#endif

#include "FreeRTOS.h"
#include "task.h"
#include "portmacro.h"

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <string.h>
#include <math.h>
#include <float.h>
#include <limits.h>

/* Define missing macros that may be needed by CMSIS DSP */
#ifndef __STATIC_FORCEINLINE
#define __STATIC_FORCEINLINE static __attribute__((always_inline)) inline
#endif

#ifndef __STATIC_INLINE
#define __STATIC_INLINE static inline
#endif

#ifndef __ALIGNED
#define __ALIGNED(x) __attribute__((aligned(x)))
#endif

#ifndef __ASM
#define __ASM __asm
#endif

/* CMSIS DSP compatibility macros */
/* Map Cortex-R8 capabilities to what CMSIS DSP expects */
#ifndef ARM_MATH_CM4
#define ARM_MATH_CM4 1
#endif

#ifndef ARM_MATH_CR8
#define ARM_MATH_CR8 1
#endif

/* CMSIS DSP library expects Cortex-M definitions, map from Cortex-R8 */
#ifndef __CORTEX_M
#define __CORTEX_M 4U  /* Map to CM4 for DSP library compatibility */
#endif

#ifndef __CORTEX_R
#define __CORTEX_R 8U
#endif

#ifndef __FPU_PRESENT
#define __FPU_PRESENT 1U
#endif

#ifndef __FPU_USED
#define __FPU_USED 1U
#endif

/* ARM DSP feature detection */
#if (defined (__ARM_FEATURE_DSP) && (__ARM_FEATURE_DSP == 1))
#define ARM_MATH_DSP 1
#endif

#if defined(__PX4_FREERTOS)
#ifdef __cplusplus
}
#endif

/* __SSAT / __USAT — saturating shift intrinsics used by arm_math.h.
 * cmsis_gcc.h only emits the asm form for ARMv7-M / ARMv8-M; for all other
 * architectures (including ARMv7-R / Cortex-R8) it falls back to a pure-C
 * implementation.  We can't include cmsis_gcc.h here (FSP BSP conflicts), so
 * provide the asm macros directly — Cortex-R8 does have ssat/usat. */
#ifndef __SSAT
#define __SSAT(ARG1, ARG2) \
__extension__ \
({                          \
  int32_t __RES, __ARG1 = (ARG1); \
  __ASM volatile ("ssat %0, %1, %2" : "=r" (__RES) : "I" (ARG2), "r" (__ARG1) : "cc"); \
  __RES; \
})
#endif

#ifndef __USAT
#define __USAT(ARG1, ARG2) \
__extension__ \
({                          \
  uint32_t __RES, __ARG1 = (ARG1); \
  __ASM volatile ("usat %0, %1, %2" : "=r" (__RES) : "I" (ARG2), "r" (__ARG1) : "cc"); \
  __RES; \
})
#endif

/* ARM DSP intrinsics — inline asm implementations for Cortex-R8 / ARMv7-R.
 *
 * Problem: arm_math.h replaces "#include <cmsis_compiler.h>" with
 * "#include <cmsis_fsp_compat.h>" when __PX4_FREERTOS is defined.
 * That means cmsis_compiler.h -> cmsis_gcc.h is never processed, so the
 * __STATIC_FORCEINLINE asm wrappers for the SIMD/DSP intrinsics are absent.
 * arm_math.h detects __ARM_FEATURE_DSP==1 (Cortex-R8 has it) and sets
 * ARM_MATH_DSP=1, which suppresses the pure-C fallbacks in arm_math.h.
 * Result: "undefined reference" linker errors for every DSP intrinsic.
 *
 * We cannot simply #include "cmsis_gcc.h" here because the FSP BSP headers
 * (pulled in via visibility.h -> FreeRTOS.h -> FreeRTOSConfig.h -> bsp_api.h)
 * already define __enable_irq, __disable_irq, etc., causing redefinition
 * errors when cmsis_gcc.h tries to redefine them.
 *
 * Solution: copy only the DSP/SIMD section of cmsis_gcc.h (the block under
 * "#if __ARM_FEATURE_DSP == 1", lines 1619-2171 in CMSIS 5.7.0) directly here.
 * Each intrinsic is guarded by #ifndef so existing definitions from any FSP
 * CMSIS variant take precedence and there are no redefinition conflicts.
 */
#if (defined (__ARM_FEATURE_DSP) && (__ARM_FEATURE_DSP == 1))

#ifndef __SADD8
__STATIC_FORCEINLINE uint32_t __SADD8(uint32_t op1, uint32_t op2)
{ uint32_t r; __ASM volatile ("sadd8 %0, %1, %2" : "=r"(r) : "r"(op1), "r"(op2)); return r; }
#endif
#ifndef __QADD8
__STATIC_FORCEINLINE uint32_t __QADD8(uint32_t op1, uint32_t op2)
{ uint32_t r; __ASM ("qadd8 %0, %1, %2" : "=r"(r) : "r"(op1), "r"(op2)); return r; }
#endif
#ifndef __SHADD8
__STATIC_FORCEINLINE uint32_t __SHADD8(uint32_t op1, uint32_t op2)
{ uint32_t r; __ASM ("shadd8 %0, %1, %2" : "=r"(r) : "r"(op1), "r"(op2)); return r; }
#endif
#ifndef __UADD8
__STATIC_FORCEINLINE uint32_t __UADD8(uint32_t op1, uint32_t op2)
{ uint32_t r; __ASM volatile ("uadd8 %0, %1, %2" : "=r"(r) : "r"(op1), "r"(op2)); return r; }
#endif
#ifndef __UQADD8
__STATIC_FORCEINLINE uint32_t __UQADD8(uint32_t op1, uint32_t op2)
{ uint32_t r; __ASM ("uqadd8 %0, %1, %2" : "=r"(r) : "r"(op1), "r"(op2)); return r; }
#endif
#ifndef __UHADD8
__STATIC_FORCEINLINE uint32_t __UHADD8(uint32_t op1, uint32_t op2)
{ uint32_t r; __ASM ("uhadd8 %0, %1, %2" : "=r"(r) : "r"(op1), "r"(op2)); return r; }
#endif
#ifndef __SSUB8
__STATIC_FORCEINLINE uint32_t __SSUB8(uint32_t op1, uint32_t op2)
{ uint32_t r; __ASM volatile ("ssub8 %0, %1, %2" : "=r"(r) : "r"(op1), "r"(op2)); return r; }
#endif
#ifndef __QSUB8
__STATIC_FORCEINLINE uint32_t __QSUB8(uint32_t op1, uint32_t op2)
{ uint32_t r; __ASM ("qsub8 %0, %1, %2" : "=r"(r) : "r"(op1), "r"(op2)); return r; }
#endif
#ifndef __SHSUB8
__STATIC_FORCEINLINE uint32_t __SHSUB8(uint32_t op1, uint32_t op2)
{ uint32_t r; __ASM ("shsub8 %0, %1, %2" : "=r"(r) : "r"(op1), "r"(op2)); return r; }
#endif
#ifndef __USUB8
__STATIC_FORCEINLINE uint32_t __USUB8(uint32_t op1, uint32_t op2)
{ uint32_t r; __ASM volatile ("usub8 %0, %1, %2" : "=r"(r) : "r"(op1), "r"(op2)); return r; }
#endif
#ifndef __UQSUB8
__STATIC_FORCEINLINE uint32_t __UQSUB8(uint32_t op1, uint32_t op2)
{ uint32_t r; __ASM ("uqsub8 %0, %1, %2" : "=r"(r) : "r"(op1), "r"(op2)); return r; }
#endif
#ifndef __UHSUB8
__STATIC_FORCEINLINE uint32_t __UHSUB8(uint32_t op1, uint32_t op2)
{ uint32_t r; __ASM ("uhsub8 %0, %1, %2" : "=r"(r) : "r"(op1), "r"(op2)); return r; }
#endif

#ifndef __SADD16
__STATIC_FORCEINLINE uint32_t __SADD16(uint32_t op1, uint32_t op2)
{ uint32_t r; __ASM volatile ("sadd16 %0, %1, %2" : "=r"(r) : "r"(op1), "r"(op2)); return r; }
#endif
#ifndef __QADD16
__STATIC_FORCEINLINE uint32_t __QADD16(uint32_t op1, uint32_t op2)
{ uint32_t r; __ASM ("qadd16 %0, %1, %2" : "=r"(r) : "r"(op1), "r"(op2)); return r; }
#endif
#ifndef __SHADD16
__STATIC_FORCEINLINE uint32_t __SHADD16(uint32_t op1, uint32_t op2)
{ uint32_t r; __ASM ("shadd16 %0, %1, %2" : "=r"(r) : "r"(op1), "r"(op2)); return r; }
#endif
#ifndef __UADD16
__STATIC_FORCEINLINE uint32_t __UADD16(uint32_t op1, uint32_t op2)
{ uint32_t r; __ASM volatile ("uadd16 %0, %1, %2" : "=r"(r) : "r"(op1), "r"(op2)); return r; }
#endif
#ifndef __UQADD16
__STATIC_FORCEINLINE uint32_t __UQADD16(uint32_t op1, uint32_t op2)
{ uint32_t r; __ASM ("uqadd16 %0, %1, %2" : "=r"(r) : "r"(op1), "r"(op2)); return r; }
#endif
#ifndef __UHADD16
__STATIC_FORCEINLINE uint32_t __UHADD16(uint32_t op1, uint32_t op2)
{ uint32_t r; __ASM ("uhadd16 %0, %1, %2" : "=r"(r) : "r"(op1), "r"(op2)); return r; }
#endif
#ifndef __SSUB16
__STATIC_FORCEINLINE uint32_t __SSUB16(uint32_t op1, uint32_t op2)
{ uint32_t r; __ASM volatile ("ssub16 %0, %1, %2" : "=r"(r) : "r"(op1), "r"(op2)); return r; }
#endif
#ifndef __QSUB16
__STATIC_FORCEINLINE uint32_t __QSUB16(uint32_t op1, uint32_t op2)
{ uint32_t r; __ASM ("qsub16 %0, %1, %2" : "=r"(r) : "r"(op1), "r"(op2)); return r; }
#endif
#ifndef __SHSUB16
__STATIC_FORCEINLINE uint32_t __SHSUB16(uint32_t op1, uint32_t op2)
{ uint32_t r; __ASM ("shsub16 %0, %1, %2" : "=r"(r) : "r"(op1), "r"(op2)); return r; }
#endif
#ifndef __USUB16
__STATIC_FORCEINLINE uint32_t __USUB16(uint32_t op1, uint32_t op2)
{ uint32_t r; __ASM volatile ("usub16 %0, %1, %2" : "=r"(r) : "r"(op1), "r"(op2)); return r; }
#endif
#ifndef __UQSUB16
__STATIC_FORCEINLINE uint32_t __UQSUB16(uint32_t op1, uint32_t op2)
{ uint32_t r; __ASM ("uqsub16 %0, %1, %2" : "=r"(r) : "r"(op1), "r"(op2)); return r; }
#endif
#ifndef __UHSUB16
__STATIC_FORCEINLINE uint32_t __UHSUB16(uint32_t op1, uint32_t op2)
{ uint32_t r; __ASM ("uhsub16 %0, %1, %2" : "=r"(r) : "r"(op1), "r"(op2)); return r; }
#endif

#ifndef __SASX
__STATIC_FORCEINLINE uint32_t __SASX(uint32_t op1, uint32_t op2)
{ uint32_t r; __ASM volatile ("sasx %0, %1, %2" : "=r"(r) : "r"(op1), "r"(op2)); return r; }
#endif
#ifndef __QASX
__STATIC_FORCEINLINE uint32_t __QASX(uint32_t op1, uint32_t op2)
{ uint32_t r; __ASM ("qasx %0, %1, %2" : "=r"(r) : "r"(op1), "r"(op2)); return r; }
#endif
#ifndef __SHASX
__STATIC_FORCEINLINE uint32_t __SHASX(uint32_t op1, uint32_t op2)
{ uint32_t r; __ASM ("shasx %0, %1, %2" : "=r"(r) : "r"(op1), "r"(op2)); return r; }
#endif
#ifndef __UASX
__STATIC_FORCEINLINE uint32_t __UASX(uint32_t op1, uint32_t op2)
{ uint32_t r; __ASM volatile ("uasx %0, %1, %2" : "=r"(r) : "r"(op1), "r"(op2)); return r; }
#endif
#ifndef __UQASX
__STATIC_FORCEINLINE uint32_t __UQASX(uint32_t op1, uint32_t op2)
{ uint32_t r; __ASM ("uqasx %0, %1, %2" : "=r"(r) : "r"(op1), "r"(op2)); return r; }
#endif
#ifndef __UHASX
__STATIC_FORCEINLINE uint32_t __UHASX(uint32_t op1, uint32_t op2)
{ uint32_t r; __ASM ("uhasx %0, %1, %2" : "=r"(r) : "r"(op1), "r"(op2)); return r; }
#endif
#ifndef __SSAX
__STATIC_FORCEINLINE uint32_t __SSAX(uint32_t op1, uint32_t op2)
{ uint32_t r; __ASM volatile ("ssax %0, %1, %2" : "=r"(r) : "r"(op1), "r"(op2)); return r; }
#endif
#ifndef __QSAX
__STATIC_FORCEINLINE uint32_t __QSAX(uint32_t op1, uint32_t op2)
{ uint32_t r; __ASM ("qsax %0, %1, %2" : "=r"(r) : "r"(op1), "r"(op2)); return r; }
#endif
#ifndef __SHSAX
__STATIC_FORCEINLINE uint32_t __SHSAX(uint32_t op1, uint32_t op2)
{ uint32_t r; __ASM ("shsax %0, %1, %2" : "=r"(r) : "r"(op1), "r"(op2)); return r; }
#endif
#ifndef __USAX
__STATIC_FORCEINLINE uint32_t __USAX(uint32_t op1, uint32_t op2)
{ uint32_t r; __ASM volatile ("usax %0, %1, %2" : "=r"(r) : "r"(op1), "r"(op2)); return r; }
#endif
#ifndef __UQSAX
__STATIC_FORCEINLINE uint32_t __UQSAX(uint32_t op1, uint32_t op2)
{ uint32_t r; __ASM ("uqsax %0, %1, %2" : "=r"(r) : "r"(op1), "r"(op2)); return r; }
#endif
#ifndef __UHSAX
__STATIC_FORCEINLINE uint32_t __UHSAX(uint32_t op1, uint32_t op2)
{ uint32_t r; __ASM ("uhsax %0, %1, %2" : "=r"(r) : "r"(op1), "r"(op2)); return r; }
#endif

#ifndef __USAD8
__STATIC_FORCEINLINE uint32_t __USAD8(uint32_t op1, uint32_t op2)
{ uint32_t r; __ASM ("usad8 %0, %1, %2" : "=r"(r) : "r"(op1), "r"(op2)); return r; }
#endif
#ifndef __USADA8
__STATIC_FORCEINLINE uint32_t __USADA8(uint32_t op1, uint32_t op2, uint32_t op3)
{ uint32_t r; __ASM ("usada8 %0, %1, %2, %3" : "=r"(r) : "r"(op1), "r"(op2), "r"(op3)); return r; }
#endif

#ifndef __UXTB16
__STATIC_FORCEINLINE uint32_t __UXTB16(uint32_t op1)
{ uint32_t r; __ASM ("uxtb16 %0, %1" : "=r"(r) : "r"(op1)); return r; }
#endif
#ifndef __UXTAB16
__STATIC_FORCEINLINE uint32_t __UXTAB16(uint32_t op1, uint32_t op2)
{ uint32_t r; __ASM ("uxtab16 %0, %1, %2" : "=r"(r) : "r"(op1), "r"(op2)); return r; }
#endif
#ifndef __SXTB16
__STATIC_FORCEINLINE uint32_t __SXTB16(uint32_t op1)
{ uint32_t r; __ASM ("sxtb16 %0, %1" : "=r"(r) : "r"(op1)); return r; }
#endif
#ifndef __SXTB16_RORn
__STATIC_FORCEINLINE uint32_t __SXTB16_RORn(uint32_t op1, uint32_t rotate)
{ uint32_t r; __ASM ("sxtb16 %0, %1, ROR %2" : "=r"(r) : "r"(op1), "i"(rotate)); return r; }
#endif
#ifndef __SXTAB16
__STATIC_FORCEINLINE uint32_t __SXTAB16(uint32_t op1, uint32_t op2)
{ uint32_t r; __ASM ("sxtab16 %0, %1, %2" : "=r"(r) : "r"(op1), "r"(op2)); return r; }
#endif

#ifndef __SMUAD
__STATIC_FORCEINLINE uint32_t __SMUAD(uint32_t op1, uint32_t op2)
{ uint32_t r; __ASM volatile ("smuad %0, %1, %2" : "=r"(r) : "r"(op1), "r"(op2)); return r; }
#endif
#ifndef __SMUADX
__STATIC_FORCEINLINE uint32_t __SMUADX(uint32_t op1, uint32_t op2)
{ uint32_t r; __ASM volatile ("smuadx %0, %1, %2" : "=r"(r) : "r"(op1), "r"(op2)); return r; }
#endif
#ifndef __SMLAD
__STATIC_FORCEINLINE uint32_t __SMLAD(uint32_t op1, uint32_t op2, uint32_t op3)
{ uint32_t r; __ASM volatile ("smlad %0, %1, %2, %3" : "=r"(r) : "r"(op1), "r"(op2), "r"(op3)); return r; }
#endif
#ifndef __SMLADX
__STATIC_FORCEINLINE uint32_t __SMLADX(uint32_t op1, uint32_t op2, uint32_t op3)
{ uint32_t r; __ASM volatile ("smladx %0, %1, %2, %3" : "=r"(r) : "r"(op1), "r"(op2), "r"(op3)); return r; }
#endif
#ifndef __SMLALD
__STATIC_FORCEINLINE uint64_t __SMLALD(uint32_t op1, uint32_t op2, uint64_t acc)
{
  union { uint32_t w32[2]; uint64_t w64; } llr;
  llr.w64 = acc;
  __ASM volatile ("smlald %0, %1, %2, %3" : "=r"(llr.w32[0]), "=r"(llr.w32[1])
                  : "r"(op1), "r"(op2), "0"(llr.w32[0]), "1"(llr.w32[1]));
  return llr.w64;
}
#endif
#ifndef __SMLALDX
__STATIC_FORCEINLINE uint64_t __SMLALDX(uint32_t op1, uint32_t op2, uint64_t acc)
{
  union { uint32_t w32[2]; uint64_t w64; } llr;
  llr.w64 = acc;
  __ASM volatile ("smlaldx %0, %1, %2, %3" : "=r"(llr.w32[0]), "=r"(llr.w32[1])
                  : "r"(op1), "r"(op2), "0"(llr.w32[0]), "1"(llr.w32[1]));
  return llr.w64;
}
#endif
#ifndef __SMUSD
__STATIC_FORCEINLINE uint32_t __SMUSD(uint32_t op1, uint32_t op2)
{ uint32_t r; __ASM volatile ("smusd %0, %1, %2" : "=r"(r) : "r"(op1), "r"(op2)); return r; }
#endif
#ifndef __SMUSDX
__STATIC_FORCEINLINE uint32_t __SMUSDX(uint32_t op1, uint32_t op2)
{ uint32_t r; __ASM volatile ("smusdx %0, %1, %2" : "=r"(r) : "r"(op1), "r"(op2)); return r; }
#endif
#ifndef __SMLSD
__STATIC_FORCEINLINE uint32_t __SMLSD(uint32_t op1, uint32_t op2, uint32_t op3)
{ uint32_t r; __ASM volatile ("smlsd %0, %1, %2, %3" : "=r"(r) : "r"(op1), "r"(op2), "r"(op3)); return r; }
#endif
#ifndef __SMLSDX
__STATIC_FORCEINLINE uint32_t __SMLSDX(uint32_t op1, uint32_t op2, uint32_t op3)
{ uint32_t r; __ASM volatile ("smlsdx %0, %1, %2, %3" : "=r"(r) : "r"(op1), "r"(op2), "r"(op3)); return r; }
#endif
#ifndef __SMLSLD
__STATIC_FORCEINLINE uint64_t __SMLSLD(uint32_t op1, uint32_t op2, uint64_t acc)
{
  union { uint32_t w32[2]; uint64_t w64; } llr;
  llr.w64 = acc;
  __ASM volatile ("smlsld %0, %1, %2, %3" : "=r"(llr.w32[0]), "=r"(llr.w32[1])
                  : "r"(op1), "r"(op2), "0"(llr.w32[0]), "1"(llr.w32[1]));
  return llr.w64;
}
#endif
#ifndef __SMLSLDX
__STATIC_FORCEINLINE uint64_t __SMLSLDX(uint32_t op1, uint32_t op2, uint64_t acc)
{
  union { uint32_t w32[2]; uint64_t w64; } llr;
  llr.w64 = acc;
  __ASM volatile ("smlsldx %0, %1, %2, %3" : "=r"(llr.w32[0]), "=r"(llr.w32[1])
                  : "r"(op1), "r"(op2), "0"(llr.w32[0]), "1"(llr.w32[1]));
  return llr.w64;
}
#endif
#ifndef __SEL
__STATIC_FORCEINLINE uint32_t __SEL(uint32_t op1, uint32_t op2)
{ uint32_t r; __ASM volatile ("sel %0, %1, %2" : "=r"(r) : "r"(op1), "r"(op2)); return r; }
#endif
#ifndef __QADD
__STATIC_FORCEINLINE int32_t __QADD(int32_t op1, int32_t op2)
{ int32_t r; __ASM volatile ("qadd %0, %1, %2" : "=r"(r) : "r"(op1), "r"(op2)); return r; }
#endif
#ifndef __QSUB
__STATIC_FORCEINLINE int32_t __QSUB(int32_t op1, int32_t op2)
{ int32_t r; __ASM volatile ("qsub %0, %1, %2" : "=r"(r) : "r"(op1), "r"(op2)); return r; }
#endif

#ifndef __PKHBT
#define __PKHBT(ARG1,ARG2,ARG3) \
  ( ((((uint32_t)(ARG1))          ) & 0x0000FFFFUL) | \
    ((((uint32_t)(ARG2)) << (ARG3)) & 0xFFFF0000UL) )
#endif
#ifndef __PKHTB
#define __PKHTB(ARG1,ARG2,ARG3) \
  ( ((((uint32_t)(ARG1))          ) & 0xFFFF0000UL) | \
    ((((uint32_t)(ARG2)) >> (ARG3)) & 0x0000FFFFUL) )
#endif

#ifndef __SMMLA
__STATIC_FORCEINLINE int32_t __SMMLA(int32_t op1, int32_t op2, int32_t op3)
{ int32_t r; __ASM ("smmla %0, %1, %2, %3" : "=r"(r) : "r"(op1), "r"(op2), "r"(op3)); return r; }
#endif
#endif /* __ARM_FEATURE_DSP == 1 */
#else
/* Forward declarations for ARM DSP intrinsics */
extern int32_t __SSAT(int32_t val, uint32_t sat);
extern uint32_t __SMUAD(uint32_t a, uint32_t b);
extern uint32_t __SMUSD(uint32_t a, uint32_t b);
extern uint32_t __SMUADX(uint32_t a, uint32_t b);
extern uint32_t __SMUSDX(uint32_t a, uint32_t b);
extern uint32_t __SMLAD(uint32_t a, uint32_t b, uint32_t c);
extern uint32_t __SMLADX(uint32_t a, uint32_t b, uint32_t c);
extern uint32_t __SMLSDX(uint32_t a, uint32_t b, uint32_t c);
extern uint32_t __SHADD16(uint32_t a, uint32_t b);
#endif /* __PX4_FREERTOS */

/* ISR Safety Macros for FreeRTOS.
 *
 * CMSIS-DSP functions (arm_rfft_q15, arm_mult_q15, arm_float_to_q15) are pure
 * computational routines that operate only on caller-supplied buffers and do not
 * modify any shared global state.  Under FreeRTOS, every context switch saves and
 * restores all CPU and FPU/DSP registers, so these functions are safe to preempt.
 *
 * Previously these macros used taskENTER_CRITICAL() / taskEXIT_CRITICAL(), which
 * disabled ALL hardware interrupts for the entire FFT computation window (~several
 * hundred microseconds).  That prevented the SPI DMA completion ISR
 * (rzv_spi_imu_callback) from firing within its 20 ms deadline, causing repeated
 * "SPI transfer error" / FIFO overflow reports (Bug #2).
 *
 * No critical section is needed here.  The no-ops below preserve the call sites
 * in gyro_fft_arm_math.h without any runtime overhead.
 */
#define GYRO_FFT_ISR_SAFE_ENTER() do {} while (0)
#define GYRO_FFT_ISR_SAFE_EXIT()  do {} while (0)
#if !defined(__PX4_FREERTOS)
#ifdef __cplusplus
}
#endif
#endif /* __PX4_FREERTOS */

#endif /* CMSIS_FSP_COMPAT_H */
#endif /* __PX4_FREERTOS */
