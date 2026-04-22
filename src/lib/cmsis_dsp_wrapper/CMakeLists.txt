# CMSIS-DSP Wrapper Library for PX4 on RZ/V2H
# Provides hardware-accelerated filter, quaternion, and matrix operations

# TEMPORARILY DISABLED: CMSIS-Core conflicts with FSP headers (__enable_irq redefinition)
# TODO: Resolve conflict by using FSP's CMSIS-Core instead of gyro_fft's
# Uncomment below to re-enable after fixing the conflict

message(STATUS "cmsis_dsp_wrapper: DISABLED due to CMSIS-Core/FSP header conflict")
message(STATUS "  - gyro_fft/CMSIS_5 defines __enable_irq() which conflicts with FSP")
message(STATUS "  - Re-enable after resolving the conflict")

# Create an empty library so CMake doesn't complain about missing target
# This allows gyro_fft to still work without the wrapper
add_library(cmsis_dsp_wrapper INTERFACE)

#[[
# Original implementation (disabled):

set(CMSIS_CORE_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../../modules/gyro_fft/CMSIS_5/CMSIS/Core)
set(CMSIS_DSP_ROOT ${PX4_SOURCE_DIR}/../rzv/arm/CMSIS-DSP)

if(NOT EXISTS ${CMSIS_CORE_ROOT}/Include/cmsis_compiler.h)
    message(FATAL_ERROR "CMSIS-Core not found at ${CMSIS_CORE_ROOT}")
endif()
if(NOT EXISTS ${CMSIS_DSP_ROOT}/Include/arm_math.h)
    message(FATAL_ERROR "CMSIS-DSP not found at ${CMSIS_DSP_ROOT}")
endif()

add_compile_options(-O2)
add_compile_options(-DARM_MATH_LOOPUNROLL)
add_compile_options(-DARM_MATH_CM7)
add_compile_options($<$<COMPILE_LANGUAGE:C>:-Wno-nested-externs>)

include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMSIS_CORE_ROOT}/Include
    ${CMSIS_DSP_ROOT}/Include
    ${CMSIS_DSP_ROOT}/Include/dsp
)

px4_add_library(cmsis_dsp_wrapper
    ${CMSIS_CORE_ROOT}/Include/cmsis_compiler.h
    ${CMSIS_CORE_ROOT}/Include/cmsis_gcc.h
    ${CMSIS_DSP_ROOT}/Include/arm_math.h
    ${CMSIS_DSP_ROOT}/Include/arm_math_types.h
    ${CMSIS_DSP_ROOT}/Include/dsp/filtering_functions.h
    ${CMSIS_DSP_ROOT}/Include/dsp/quaternion_math_functions.h
    ${CMSIS_DSP_ROOT}/Include/dsp/matrix_functions.h
    include/cmsis_dsp_config.h
    include/cmsis_dsp_filters.hpp
    src/biquad_filter.cpp
    src/lowpass_filter.cpp
    src/notch_filter.cpp
)
]]
