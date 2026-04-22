/****************************************************************************
 *
 * Copyright (c) 2025 Renesas Electronics Corporation and/or its affiliates
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 ****************************************************************************/

/**
 * @file posix_shim_cpp.cpp
 * @brief POSIX shim C++ overrides for Renesas RZ/V2H
 */

#if defined(__PX4_FREERTOS)

#include "posix_shim.h"

// Declare an internal C function with a different name to avoid conflict
extern "C" {
	int __posix_usleep_c_impl(unsigned long usec);
}

// C++ version - NOT in extern "C", so it gets C++ name mangling (_Z6usleepm)
int usleep(unsigned long usec)
{
	// Call the C implementation
	// We can't call "usleep" directly as it would recurse to ourselves
	// So we call the internal C implementation
	return __posix_usleep_c_impl(usec);
}

#endif /* __PX4_FREERTOS */
