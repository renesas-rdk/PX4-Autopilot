/****************************************************************************
 *
 * Copyright (c) 2025 Renesas Electronics Corporation and/or its affiliates
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 ****************************************************************************/

/**
 * @file stub_params.c
 * @brief Stub parameter definitions for modules not included in RZ/V2H build.
 *
 * These prevent "parameter not found" warnings when QGC or other modules
 * query parameters for features not present in this configuration.
 */


#if defined(__PX4_FREERTOS)

#include <parameters/param.h>

// ============================================================================
// Fixed-Wing Parameters (FW modules not built for quadcopter)
// ============================================================================

/**
 * Fixed-wing maximum airspeed
 * @unit m/s
 * @min 0.0
 * @max 50.0
 * @group FW Path Control (stub)
 */
PARAM_DEFINE_FLOAT(FW_AIRSPD_MAX, 0.0f);

/**
 * Fixed-wing trim airspeed
 * @unit m/s
 * @min 0.0
 * @max 50.0
 * @group FW Path Control (stub)
 */
PARAM_DEFINE_FLOAT(FW_AIRSPD_TRIM, 0.0f);

/**
 * FW Pitch setpoint offset (trim)
 * @unit deg
 * @min -90.0
 * @max 90.0
 * @decimal 1
 * @group FW Attitude Control (stub)
 */
PARAM_DEFINE_FLOAT(FW_PSP_OFF, 0.0f);

/**
 * FW TECS climb rate setpoint
 * @unit m/s
 * @min 0.0
 * @max 15.0
 * @group FW TECS (stub)
 */
PARAM_DEFINE_FLOAT(FW_T_CLMB_R_SP, 0.0f);

/**
 * FW TECS sink rate setpoint
 * @unit m/s
 * @min 0.0
 * @max 15.0
 * @group FW TECS (stub)
 */
PARAM_DEFINE_FLOAT(FW_T_SINK_R_SP, 0.0f);

/**
 * Ground speed threshold for FW
 * @unit m/s
 * @min 0.0
 * @max 50.0
 * @group FW Path Control (stub)
 */
PARAM_DEFINE_FLOAT(GND_SPEED_THR_SC, 0.0f);

// ============================================================================
// VTOL Parameters (VTOL modules not built)
// ============================================================================

/**
 * VTOL back transition deceleration I gain
 * @min 0.0
 * @max 10.0
 * @group VTOL Attitude (stub)
 */
PARAM_DEFINE_FLOAT(VT_B_DEC_I, 0.10f);

/**
 * VTOL back transition deceleration setpoint
 * @unit m/s^2
 * @min 0.0
 * @max 20.0
 * @group VTOL Attitude (stub)
 */
PARAM_DEFINE_FLOAT(VT_B_DEC_MSS, 2.00f);

// ============================================================================
// Airspeed Sensor Parameters (no airspeed sensor on quadcopter)
// ============================================================================

/**
 * Airspeed scale 1
 * @min 0.5
 * @max 1.5
 * @decimal 2
 * @group Sensors (stub)
 */
PARAM_DEFINE_FLOAT(ASPD_SCALE_1, 1.0f);

// ============================================================================
// Camera Trigger Parameters (no camera trigger hardware)
// ============================================================================

/**
 * Camera trigger mode
 * @value -1 Disabled
 * @value 0 Time-based, on command
 * @value 1 Time-based, always on
 * @value 2 Distance-based, always on
 * @value 3 Distance-based, on command
 * @value 4 Mission-controlled
 * @min -1
 * @max 4
 * @group Camera Trigger (stub)
 */
PARAM_DEFINE_INT32(TRIG_MODE, 0);

// ============================================================================
// UAVCAN/DroneCAN Parameters (no CAN bus)
// ============================================================================

/**
 * UAVCAN enable
 * @value 0 Disabled
 * @value 1 Sensors Manual
 * @value 2 Sensors Automatic
 * @value 3 Sensors and ESCs
 * @min 0
 * @max 3
 * @reboot_required true
 * @group UAVCAN (stub)
 */
PARAM_DEFINE_INT32(UAVCAN_ENABLE, 0);

// ============================================================================
// Obstacle Avoidance Parameters (no obstacle avoidance hardware)
// ============================================================================
/**
 * Enable obstacle avoidance (stub)
 * @value 0 Disabled
 * @value 1 Enabled
 * @min 0
 * @max 1
 * @group Obstacle Avoidance (stub)
 */
PARAM_DEFINE_INT32(COM_OBS_AVOID, 0);

// ============================================================================
// End of stub parameters
// ============================================================================

#endif /* __PX4_FREERTOS */
