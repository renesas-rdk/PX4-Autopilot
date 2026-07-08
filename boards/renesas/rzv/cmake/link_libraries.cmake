############################################################################
# Copyright (c) 2025 Renesas Electronics Corporation and/or its affiliates
#
# SPDX-License-Identifier: BSD-3-Clause
############################################################################

# Link FreeRTOS kernel and minimal POSIX wrapper

set(FREERTOS_ROOT ${RZV_WORKSPACE_ROOT}/rzv/aws/FreeRTOS/FreeRTOS/Source)
# B1: consume the platform FreeRTOS+POSIX shim (freertos-port) instead of the in-repo copy.
# Sources live under posix-shim/ (was source/); include tree layout is otherwise identical.
set(FREERTOS_PORT_ROOT ${RZV_WORKSPACE_ROOT}/src/renesas-robotics-platform/cr8/freertos-port)

# Build FreeRTOS kernel library
add_library(freertos_kernel STATIC
    ${FREERTOS_ROOT}/queue.c
    ${FREERTOS_ROOT}/list.c
    ${FREERTOS_ROOT}/tasks.c
    ${FREERTOS_ROOT}/timers.c
    ${FREERTOS_ROOT}/event_groups.c
    ${FREERTOS_ROOT}/stream_buffer.c
    ${FREERTOS_ROOT}/portable/MemMang/heap_4.c
    ${RZV_WORKSPACE_ROOT}/rzv/fsp/src/rm_freertos_port/cr/port.c
)

target_include_directories(freertos_kernel PUBLIC
    ${FREERTOS_ROOT}/include
    ${RZV_WORKSPACE_ROOT}/rzv/fsp/src/rm_freertos_port
    ${RZV_WORKSPACE_ROOT}/rzv/fsp/src/rm_freertos_port/cr
    ${RZV_WORKSPACE_ROOT}/rzv_cfg/aws
)

# Disable warnings-as-errors for FreeRTOS kernel (third-party code)
target_compile_options(freertos_kernel PRIVATE
    -Wno-error=cast-align
    -Wno-error=strict-prototypes
)

# Build minimal FreeRTOS POSIX wrapper (only required functions). B1: the 3 sources come
# from the platform freertos-port/posix-shim; pthread_mutex.c is the SAME file the firmware ELF
# compiles (both define __PX4_FREERTOS) — no split-brain. unistd.c is not needed here.
add_library(freertos_posix_minimal STATIC
    ${FREERTOS_PORT_ROOT}/posix-shim/FreeRTOS_POSIX_pthread_mutex.c
    ${FREERTOS_PORT_ROOT}/posix-shim/FreeRTOS_POSIX_sched.c
    ${FREERTOS_PORT_ROOT}/posix-shim/FreeRTOS_POSIX_utils.c
)

target_include_directories(freertos_posix_minimal PUBLIC
    ${FREERTOS_PORT_ROOT}/include
    ${FREERTOS_PORT_ROOT}/include/FreeRTOS_POSIX
    ${FREERTOS_PORT_ROOT}/include/private
    ${FREERTOS_ROOT}/include
)

target_link_libraries(freertos_posix_minimal PUBLIC freertos_kernel)

# Disable warnings-as-errors for POSIX wrapper (third-party code)
target_compile_options(freertos_posix_minimal PRIVATE
    -Wno-error=cast-function-type
    -Wno-error=strict-prototypes
    -Wno-error=cast-align
    -Wno-error=address
)

# Link FreeRTOS libraries to px4
target_link_libraries(px4 PRIVATE freertos_kernel freertos_posix_minimal)
