# Wrapper to expose the main PX4 arm-none-eabi toolchain file from the posix
# platform directory (needed by ExternalProject builds such as Micro XRCE-DDS).
get_filename_component(_px4_arm_toolchain
    "${CMAKE_CURRENT_LIST_DIR}/../../../cmake/Toolchain-arm-none-eabi.cmake"
    ABSOLUTE)
if(NOT EXISTS "${_px4_arm_toolchain}")
    message(FATAL_ERROR "PX4 arm-none-eabi toolchain file missing: ${_px4_arm_toolchain}")
endif()
include("${_px4_arm_toolchain}")
