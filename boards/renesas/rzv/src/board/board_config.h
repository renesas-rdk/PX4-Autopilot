/****************************************************************************
 *
 * Copyright (c) 2025 Renesas Electronics Corporation and/or its affiliates
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 ****************************************************************************/

/**
 * @file board_config.h
 * @brief Board configuration for Renesas RZ/V2H
 */
#pragma once

#if defined(__PX4_FREERTOS)


#include "rzv_serial_config.h"
#include <bsp_api.h>
#include <bsp_pin_cfg.h>

#define BOARD_OVERRIDE_UUID "RZV2H0000000000" // length 16 as expected
#define PX4_SOC_ARCH_ID     PX4_SOC_ARCH_ID_UNUSED

#define CONFIG_I2C 1
#define CONFIG_SPI 1
#define CONFIG_UART 1

#define RC_SERIAL_PORT          RZV_RC_SERIAL_DEVICE

#define PX4_NUMBER_I2C_BUSES    1
#define PX4_NUMBER_SPI_BUSES    2

// GPIO placeholders alias the generated FSP pin symbols to avoid duplicating pin numbers here.
// Keep the software names stable while the final electrical polarity is verified on hardware.
#define RZV_GPIO_SAFETY_BTN_PLACEHOLDER   GPS_SAFETY_SWITCH
#define RZV_GPIO_SAFETY_LED_PLACEHOLDER   GPS_SAFETY_SWITCH_LED
#define RZV_GPIO_BUZZER_PLACEHOLDER       GPS_BUZZER

// PX4-facing aliases reserved for future safety/button enablement.
#define GPIO_BTN_SAFETY                   RZV_GPIO_SAFETY_BTN_PLACEHOLDER
#define GPIO_LED_SAFETY                   RZV_GPIO_SAFETY_LED_PLACEHOLDER

// Tone alarm is not enabled yet on RZ/V2H, but reserve stable software names now.
#define GPIO_TONE_ALARM_GPIO              RZV_GPIO_BUZZER_PLACEHOLDER
#define GPIO_TONE_ALARM_IDLE              RZV_GPIO_BUZZER_PLACEHOLDER

// Sensor device addresses (handled by the FreeRTOS+FSP shim)
#define PX4_I2C_OBDEV_BMP280    0x76

#define ADC_BATTERY_VOLTAGE_CHANNEL    -1
#define ADC_BATTERY_CURRENT_CHANNEL    -1
#define BOARD_NUMBER_BRICKS            1
#define BOARD_ADC_BRICK_VALID          0
#define BOARD_NUMBER_USB_BRICKS        0
#define BOARD_USB_VBUS_VALID           0

#include <system_config.h>

// Undefine POSIX defaults to allow custom MCU version implementation
#undef BOARD_OVERRIDE_CPU_VERSION
#undef board_mcu_version

#include <px4_platform_common/board_common.h>
#include "FreeRTOS.h"
#include "task.h"
#include "uart_rc_switch.h"

#endif /* __PX4_FREERTOS */
