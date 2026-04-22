############################################################################
# Copyright (c) 2025 Renesas Electronics Corporation and/or its affiliates
#
# SPDX-License-Identifier: BSD-3-Clause
############################################################################

############################################################################
#
#   Renesas RZ/V2H board-specific drivers
#
############################################################################

set(RZV_BOARD_SRCS
    board/board_pwm_out.cpp
    board/board_mcu_version.c
    board/systemcmd_hooks_rzv.cpp
    hal/i2c.cpp
    services/remote_storage.cpp
    services/embedded_metadata.cpp
    hal/spi.cpp
    hal/micro_hal.cpp
    hal/sensor_hal.cpp
    hal/drivers/rzv_fsp/io_worker.cpp
    shim/px4/condition_variable.cpp
    shim/px4/mutex.cpp
    shim/px4/time_shim.cpp
    shim/px4/time_port_shim.cpp
    shim/px4/time_port_board.cpp
    shim/px4/posix_dir_stubs.c
    shim/freertos/px4_sem_freertos.cpp
    shim/freertos/px4_mutex_freertos.cpp
    shim/freertos/px4_condition_variable_freertos.cpp
)

px4_add_library(drivers_board
    ${RZV_BOARD_SRCS}
)

# Pass the PX4 build directory so embedded_metadata.cpp can locate the
# generated .json.xz files using .incbin at compile time.
# rc.board_defaults.cmds is embedded by px4_startup_shim.cpp (no duplicate here).
target_compile_definitions(drivers_board
    PRIVATE
        EMBEDDED_METADATA_BUILD_DIR=${PX4_BINARY_DIR}
)

if(TARGET component_general_json)
    add_dependencies(drivers_board component_general_json)
endif()

target_include_directories(drivers_board
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/board
        ${CMAKE_CURRENT_SOURCE_DIR}/hal
        ${CMAKE_CURRENT_SOURCE_DIR}/services
        ${CMAKE_CURRENT_SOURCE_DIR}/shim
        ${CMAKE_CURRENT_SOURCE_DIR}/shim/px4
        ${CMAKE_CURRENT_SOURCE_DIR}/shim/freertos
        ${RZV_WORKSPACE_ROOT}
)

set_property(GLOBAL APPEND PROPERTY PX4_MODULE_PATHS ${CMAKE_CURRENT_SOURCE_DIR})
set_property(GLOBAL APPEND PROPERTY PX4_MODULE_PATHS ${CMAKE_CURRENT_SOURCE_DIR}/board)
set_property(GLOBAL APPEND PROPERTY PX4_MODULE_PATHS ${PX4_SOURCE_DIR}/src/modules/mavlink)

install(DIRECTORY DESTINATION ${PX4_SOURCE_DIR}/boards/renesas/rzv)

if(TARGET px4_platform)
    target_sources(px4_platform PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/board/priority_map.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/board/load_mon_platform_rzv.cpp)
endif()
