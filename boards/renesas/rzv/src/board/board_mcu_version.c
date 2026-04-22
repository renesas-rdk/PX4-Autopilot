/****************************************************************************
 *
 * Copyright (c) 2025 Renesas Electronics Corporation and/or its affiliates
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 ****************************************************************************/

/**
 * @file board_mcu_version.c
 * Implementation of Renesas RZ/V2H CR8 SoC version API
 */


#if defined(__PX4_FREERTOS)

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>
#include <stdint.h>

#define RZV2H_MCU_REV_1 0x01

int board_mcu_version(char *rev, const char **revstr, const char **errata)
{
	*revstr = "RZ/V2H Robot RDK by SST";
	*rev = '1';

	if (errata) {
		*errata = NULL;
	}

	return RZV2H_MCU_REV_1;
}

#endif /* __PX4_FREERTOS */
