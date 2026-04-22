/****************************************************************************
 *
 * Copyright (c) 2025 Renesas Electronics Corporation and/or its affiliates
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 ****************************************************************************/

/**
 * @file ioctl.h
 * @brief IOCTL definitions for Renesas RZ/V2H
 */
#pragma once

#if defined(__PX4_FREERTOS)


#include <stdarg.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Align with FreeRTOS POSIX stub (0x6678) to keep ioctl users/drivers in sync.
 * The Linux value (0x541B) caused ENOTTY when mixed headers were used.
 */
#ifdef FIONREAD
#undef FIONREAD
#endif
#define FIONREAD 0x6678


#ifndef _IOC_NRBITS
#define _IOC_NRBITS 8
#define _IOC_TYPEBITS 8
#define _IOC_SIZEBITS 14
#define _IOC_DIRBITS 2

#define _IOC_NRMASK ((1U << _IOC_NRBITS) - 1)
#define _IOC_TYPEMASK ((1U << _IOC_TYPEBITS) - 1)
#define _IOC_SIZEMASK ((1U << _IOC_SIZEBITS) - 1)
#define _IOC_DIRMASK ((1U << _IOC_DIRBITS) - 1)

#define _IOC_NRSHIFT 0
#define _IOC_TYPESHIFT (_IOC_NRSHIFT + _IOC_NRBITS)
#define _IOC_SIZESHIFT (_IOC_TYPESHIFT + _IOC_TYPEBITS)
#define _IOC_DIRSHIFT (_IOC_SIZESHIFT + _IOC_SIZEBITS)

#define _IOC(dir,type,nr,size)     (((dir) << _IOC_DIRSHIFT) |      ((type) << _IOC_TYPESHIFT) |      ((nr) << _IOC_NRSHIFT) |      ((size) << _IOC_SIZESHIFT))

#define _IOC_DIR(nr) (((nr) >> _IOC_DIRSHIFT) & _IOC_DIRMASK)
#define _IOC_TYPE(nr) (((nr) >> _IOC_TYPESHIFT) & _IOC_TYPEMASK)
#define _IOC_NR(nr) (((nr) >> _IOC_NRSHIFT) & _IOC_NRMASK)
#define _IOC_SIZE(nr) (((nr) >> _IOC_SIZESHIFT) & _IOC_SIZEMASK)

#define _IOC_NONE 0U
#define _IOC_WRITE 1U
#define _IOC_READ 2U

#define _IO(type,nr) _IOC(_IOC_NONE,(type),(nr),0)
#define _IOR(type,nr,size) _IOC(_IOC_READ,(type),(nr),sizeof(size))
#define _IOW(type,nr,size) _IOC(_IOC_WRITE,(type),(nr),sizeof(size))
#define _IOWR(type,nr,size) _IOC(_IOC_READ|_IOC_WRITE,(type),(nr),sizeof(size))
#endif

int ioctl(int fd, unsigned long request, ...);

#ifdef __cplusplus
}

#endif /* __PX4_FREERTOS */

#endif
